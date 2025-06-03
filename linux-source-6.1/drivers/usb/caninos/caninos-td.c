#include <linux/scatterlist.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include "aotg_hcd.h"

static struct kmem_cache *slab_cache;

int aotg_kmem_cache_create(void)
{
	slab_cache = KMEM_CACHE(aotg_td, SLAB_HWCACHE_ALIGN);
	
	if (!slab_cache) {
		return -ENOMEM;
	}
	return 0;
}

void aotg_kmem_cache_destroy(void)
{
	kmem_cache_destroy(slab_cache);
}

struct aotg_td *aotg_alloc_td(gfp_t mem_flags)
{
	struct aotg_td *td;
	
	td = kmem_cache_alloc(slab_cache, mem_flags);
	
	if (!td) {
		return NULL;
	}
	
	memset(td, 0, sizeof(struct aotg_td));
	
	td->cross_ring = 0;
	td->err_count = 0;
	td->urb = NULL;
	
	INIT_LIST_HEAD(&td->queue_list);
	INIT_LIST_HEAD(&td->enring_list);
	INIT_LIST_HEAD(&td->dering_list);
	
	return td;
}

void aotg_release_td(struct aotg_td *td)
{
	if (td) {
		kmem_cache_free(slab_cache, td);
	}
}

static inline int is_room_on_ring(struct aotg_ring *ring, unsigned num_trbs)
{
	return (num_trbs > atomic_read(&ring->num_trbs_free)) ? 0 : 1;
}

/**
 * scatter-gather transfer trbs count.
 * according to num_sgs and urb->transfer_buffer_length.
 */
static unsigned int count_sg_urb_need_trbs(struct urb *urb)
{
	int num_sgs, num_trbs, temp, i;
	struct scatterlist *sg = NULL;
	
	num_sgs = urb->num_mapped_sgs;
	temp = urb->transfer_buffer_length;
	num_trbs = 0;
	
	for_each_sg(urb->sg, sg, num_sgs, i)
	{
		unsigned int len = sg_dma_len(sg);
		
		if (len != 0)
			num_trbs++;
		
		len = min_t(int, len, temp);
		
		temp -= len;
		
		if (temp == 0)
			break;
	}
	
	if (usb_pipeout(urb->pipe) && (urb->transfer_flags & URB_ZERO_PACKET))
		num_trbs++;
	
	return num_trbs;
}

int ring_enqueue_sg_td(
	struct aotg_hcd *acthcd, struct aotg_ring *ring, struct aotg_td *td)
{
	u8 is_out;
	int num_sgs, num_trbs;
	int len, this_trb_len;
	u32 addr, token;
	
	struct urb *urb = td->urb;
	struct scatterlist *sg;
	
	if (unlikely(urb->sg == NULL))
	{
		dev_err(acthcd->dev, "scatterlist is NULL\n");
		return -EINVAL;
	}
	
	is_out = usb_pipeout(urb->pipe);
	len = urb->transfer_buffer_length;
	
	num_sgs = urb->num_mapped_sgs;
	num_trbs = count_sg_urb_need_trbs(urb);
	
	if (unlikely(num_trbs == 0))
	{
		dev_err(acthcd->dev, "num_trbs is zero\n");
		return -EINVAL;
	}
	
	if (unlikely(!is_room_on_ring(ring, num_trbs))) {
		return -ENOSPC;
	}
	
	td->num_trbs = num_trbs;
	
	td->trb_vaddr = ring->enqueue_trb;
	
	td->trb_dma = ring_trb_virt_to_dma(ring, ring->enqueue_trb);
	
	
	if ((td->trb_vaddr + (num_trbs - 1)) > ring->last_trb) {
		td->cross_ring = 1;
	}
	
	sg = urb->sg;
	addr = (u32)sg_dma_address(sg);
	this_trb_len = (u32)min_t(int, sg_dma_len(sg), len);
	
	if (is_out) {
		token = TRB_OF;
	}
	else {
		token = TRB_CSP | TRB_OF;
	}
	
	do
	{
		if (num_trbs == 1)
		{
			token &= ~TRB_CHN;
			
			if (is_out)
				token |= TRB_ITE;
			else
				token |= TRB_ICE;
			
			if (is_out && (urb->transfer_flags & URB_ZERO_PACKET))
				enqueue_trb(ring, 0, 0, token);
			else
				enqueue_trb(ring, addr, this_trb_len, token);
			
			return 0;
		}
		
		token |= TRB_CHN;
		
		enqueue_trb(ring, addr, this_trb_len, token);
		len -= this_trb_len;
		num_trbs--;
		num_sgs--;
		
		if (num_sgs)
		{
			sg = sg_next(sg);
			addr = (u32)sg_dma_address(sg);
			this_trb_len = (u32)min_t(int, sg_dma_len(sg), len);
		}
		
	} while (num_trbs);
	
	return 0;
}

int aotg_ring_enqueue_td(
	struct aotg_hcd *acthcd, struct aotg_ring *ring, struct aotg_td *td)
{
	struct urb *urb = td->urb;
	u8 is_out = usb_pipeout(urb->pipe);
	u32 addr, len, token;
	int num_trbs;
	
	len = urb->transfer_buffer_length;
	
	if (len > 0 && urb->num_mapped_sgs) {
		return ring_enqueue_sg_td(acthcd, ring, td);
	}
	
	if (len) {
		addr = (u32)urb->transfer_dma;
	}
	else {
		addr = 0;
	}
	
	num_trbs = 1;
	
	if (len != 0 && usb_pipeout(urb->pipe))
	{
		if (urb->transfer_flags & URB_ZERO_PACKET) {
			num_trbs++;
		}
	}
	
	if (unlikely(!is_room_on_ring(ring, num_trbs))) {
		return -ENOSPC;
	}
	
	td->num_trbs = num_trbs;
	td->trb_vaddr = ring->enqueue_trb;
	td->trb_dma = ring_trb_virt_to_dma(ring, ring->enqueue_trb);
	
	if (is_out) {
		token = TRB_OF;
	}
	else {
		token = TRB_CSP | TRB_OF;
	}
	
	if (num_trbs > 1)
	{
		token |= TRB_CHN;
		enqueue_trb(ring, addr, len, token);
		addr = 0;
		len = 0;
	}
	
	token |= TRB_LT; /*8723bu, release cpu for interrupt transfer*/
	
	if (is_out) {
		token |= TRB_ITE;
	}
	else {
		token |= TRB_ICE;
	}
	
	enqueue_trb(ring, addr, len, token);
	return 0;
}

int aotg_ring_enqueue_isoc_td(
	struct aotg_hcd *acthcd, struct aotg_ring *ring, struct aotg_td *td)
{
	u8 is_out;
	u32 start_addr;
	u32 addr, token, this_trb_len;
	int i = 0;
	int start_frame;
	int num_trbs;
	struct urb *urb = td->urb;
	
	num_trbs = urb->number_of_packets;
	
	if (unlikely(num_trbs == 0))
	{
		dev_err(acthcd->dev, "num_trbs is zero\n");
		return -EINVAL;
	}
	
	if (unlikely(!is_room_on_ring(ring, num_trbs))) {
		return -ENOSPC;
	}
	
	is_out = usb_pipeout(urb->pipe);
	
	td->num_trbs = num_trbs;
	td->trb_vaddr = ring->enqueue_trb;
	td->trb_dma = ring_trb_virt_to_dma(ring, ring->enqueue_trb);

	start_frame = readw(acthcd->base + HCFRMNRL);
	start_frame &= 0x3fff;
	urb->start_frame = start_frame;

	start_addr = (u32)urb->transfer_dma;

	if (is_out)
		token = TRB_OF;
	else
		token = TRB_CSP | TRB_OF;

	do {
		addr = start_addr + urb->iso_frame_desc[i].offset;
		this_trb_len = urb->iso_frame_desc[i].length;
		if (num_trbs == 1) {
			token &= ~TRB_CHN;
			if (is_out)
				token |= TRB_ITE;
			else
				token |= TRB_ICE;

			enqueue_trb(ring, addr, this_trb_len, token);
			break;
		}
		enqueue_trb(ring, addr, this_trb_len, token);
		i++;
		num_trbs--;
	} while (num_trbs);

	return 0;
}

