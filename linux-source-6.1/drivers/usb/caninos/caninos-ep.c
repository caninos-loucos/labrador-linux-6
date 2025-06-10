#include "aotg_hcd.h"

static int hcep_set_split_micro_frame(struct aotg_hcd *, struct aotg_hcep *);

static ulong get_fifo_slot(struct aotg_hcd *, int size);

/* must call with spinlock */
void aotg_hcep_free(struct usb_hcd *hcd, struct aotg_hcep *ep)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	
	if (!ep) {
		return;
	}
	
	if (ep->type == PIPE_CONTROL)
	{
		if (ep->ep0_index >= 0) {
			acthcd->hcep_pool.ep0[ep->ep0_index] = NULL;
		}
	}
	else
	{
		if (ep->ring) {
			writel(DMACTRL_DMACC, ep->ring->reg_dmactrl);
		}
		ep_disable(ep);
		
		if (ep->hep) {
			ep->hep->hcpriv = NULL;
		}
		
		release_fifo_slot(acthcd, ep);
		
		if (ep->is_out) {
			acthcd->hcep_pool.outep[ep->index] = NULL;
		}
		else {
			acthcd->hcep_pool.inep[ep->index] = NULL;
		}
		
		aotg_free_ring(acthcd, ep->ring);
	}
	
	kfree(ep);
}

/* must call with spinlock */
int aotg_hcep_alloc(struct usb_hcd *hcd, struct urb *urb)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	int i;
	
	BUG_ON(urb->ep == NULL || urb->ep->hcpriv != NULL);
	
	ep = kzalloc(sizeof(*ep), GFP_ATOMIC);
	
	if (!ep)
	{
		dev_err(acthcd->dev, "%s: memory allocation failed\n", __func__);
		return -ENOMEM;
	}
	
	ep->hep = urb->ep;
	ep->is_out = usb_pipeout(urb->pipe) ? 1 : 0;
	ep->udev = usb_get_dev(urb->dev);
	ep->epnum = usb_pipeendpoint(urb->pipe);
	ep->maxpacket = usb_maxpacket(ep->udev, urb->pipe);
	ep->type = usb_pipetype(urb->pipe);
	
	if (ep->type == PIPE_CONTROL)
	{
		if (acthcd->active_ep0)
		{
			kfree(ep);
			dev_err(acthcd->dev, "%s: endpoint is already active\n", __func__);
			return -EBUSY;
		}
		
		for (ep->ep0_index = -1, i = 0; i < MAX_EP_NUM; i++)
		{
			if (acthcd->hcep_pool.ep0[i] == NULL)
			{
				ep->ep0_index = i;
				break;
			}
		}
		
		if (ep->ep0_index < 0)
		{
			kfree(ep);
			dev_err(acthcd->dev, "%s: no free endpoint slot\n", __func__);
			return -ENOSPC;
		}
		
		acthcd->hcep_pool.ep0[ep->ep0_index] = ep;
		
		ep->reg_hcep_dev_addr = acthcd->base + HCEP0ADDR;
		ep->reg_hcep_port = acthcd->base + HCEP0PORT;
		ep->reg_hcep_splitcs = acthcd->base + HCEP0SPILITCS;
	}
	else
	{
		ep->buftype = EPCON_BUF_SINGLE;
		
		if (ep->type == PIPE_INTERRUPT || ep->type == PIPE_ISOCHRONOUS) {
			ep->interval = urb->ep->desc.bInterval;
		}
		if (ep->type == PIPE_ISOCHRONOUS) {
			ep->iso_packets = (urb->ep->desc.wMaxPacketSize >> 11) & 3;
		}
		
		for (ep->index = -1, i = 1; i < MAX_EP_NUM; i++) /* 0 is reserved */
		{
			if (ep->is_out)
			{
				if (acthcd->hcep_pool.outep[i] == NULL)
				{
					ep->index = i;
					ep->mask = (u8)(USB_HCD_OUT_MASK | i);
					acthcd->hcep_pool.outep[i] = ep;
					break;
				}
			}
			else
			{
				if (acthcd->hcep_pool.inep[i] == NULL)
				{
					ep->index = i;
					ep->mask = (u8)(i);
					acthcd->hcep_pool.inep[i] = ep;
					break;
				}
			}
		}
		
		if (ep->index < 0)
		{
			kfree(ep);
			dev_err(acthcd->dev, "%s: no free endpoint slot\n", __func__);
			return -ENOSPC;
		}
		
		// EPCON_BUF_SINGLE thus size = 1 * ep->maxpacket
		ep->fifo_addr = get_fifo_slot(acthcd, ep->maxpacket);
		
		if (!ep->fifo_addr)
		{
			if (ep->is_out) {
				acthcd->hcep_pool.outep[ep->index] = NULL;
			}
			else {
				acthcd->hcep_pool.inep[ep->index] = NULL;
			}
			kfree(ep);
			dev_err(acthcd->dev, "%s: no free fifo slot\n", __func__);
			return -ENOSPC;
		}
		
		ep->reg_hcepcon = get_hcepcon_reg(ep->is_out, 
			acthcd->base + HCOUT1CON,
			acthcd->base + HCIN1CON,
			ep->index);
		
		ep->reg_hcepcs = get_hcepcs_reg(ep->is_out, 
			acthcd->base + HCOUT1CS,
			acthcd->base + HCIN1CS,
			ep->index);
		
		ep->reg_hcepbc = get_hcepbc_reg(ep->is_out, 
			acthcd->base + HCOUT1BCL, 
			acthcd->base + HCIN1BCL, 
			ep->index);
		
		ep->reg_hcepctrl = get_hcepctrl_reg(ep->is_out, 
			acthcd->base + HCOUT1CTRL, 
			acthcd->base + HCIN1CTRL, 
			ep->index);
		
		ep->reg_hcmaxpck = get_hcepmaxpck_reg(ep->is_out, 
			acthcd->base + HCOUT1MAXPCKL, 
			acthcd->base + HCIN1MAXPCKL, 
			ep->index);
		
		ep->reg_hcepaddr = get_hcepaddr_reg(ep->is_out, 
			acthcd->base + HCOUT1STADDR, 
			acthcd->base + HCIN1STADDR, 
			ep->index);
		
		ep->reg_hcep_dev_addr = get_hcep_dev_addr_reg(ep->is_out,
			acthcd->base + HCOUT1ADDR, 
			acthcd->base + HCIN1ADDR, 
			ep->index);
		
		ep->reg_hcep_port = get_hcep_port_reg(ep->is_out,
			acthcd->base + HCOUT1PORT, 
			acthcd->base + HCIN1PORT, 
			ep->index);
		
		ep->reg_hcep_splitcs = get_hcep_splitcs_reg(ep->is_out,
			acthcd->base + HCOUT1SPILITCS, 
			acthcd->base + HCIN1SPILITCS, 
			ep->index);
		
		ep->reg_hcerr = get_hcerr_reg(ep->is_out,
			acthcd->base + HCOUT0ERR,
			acthcd->base + HCIN0ERR,
			ep->index);
		
		ep->reg_hcep_interval = get_hcep_interval_reg(ep->is_out,
			acthcd->base + HCOUT1BINTERVAL,
			acthcd->base + HCEP0BINTERVAL,
			ep->index);
	}
	
	if (urb->dev->parent)
	{
		if (urb->dev->tt)
		{
			u32 think_time = (urb->dev->tt->think_time / 666);
			
			if (think_time <= 0) {
				think_time = 1;
			}
			else if (think_time > 4) {
				think_time = 4;
			}
			
			think_time = think_time * 20;
			
			writeb(think_time, acthcd->base + HCTRAINTERVAL);
		}
		
		if ((urb->dev->parent->parent) &&
		    (urb->dev->parent != hcd->self.root_hub))
		{
			ep->has_hub = 1;
			ep->hub_addr = 0x7f & readb(acthcd->base + FNADDR);
		}
	}
	
	if (ep->type == PIPE_CONTROL)
	{
		/* can be used as input and output */
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1, 0); /* out */
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0, 0); /* in  */
		
		usb_setbitsw(1, acthcd->base + HCOUTxIEN0);
		usb_setbitsw(1, acthcd->base + HCINxIEN0);
		
		writew(1, acthcd->base + HCOUTxIRQ0);
		writew(1, acthcd->base + HCINxIRQ0);
		
		if (ep->has_hub) {
			usb_setbitsb(0x80, acthcd->base + FNADDR);
		}
		else {
			writeb(usb_pipedevice(urb->pipe), acthcd->base + FNADDR);
		}
	}
	else
	{
		usb_settoggle(urb->dev, ep->epnum, ep->is_out, 0);
		
		pio_irq_disable(acthcd, ep->mask);
		pio_irq_clear(acthcd, ep->mask);
		
		ep_disable(ep);
		
		writel(ep->fifo_addr, ep->reg_hcepaddr);
		writew(ep->maxpacket, ep->reg_hcmaxpck);
		
		if (ep->type == PIPE_ISOCHRONOUS) {
			writeb(EPCON_TYPE_ISO | EPCON_BUF_SINGLE, ep->reg_hcepcon);
		}
		if (ep->type == PIPE_BULK) {
			writeb(EPCON_TYPE_BULK | EPCON_BUF_SINGLE, ep->reg_hcepcon);
		}
		if (ep->type == PIPE_INTERRUPT) {
			writeb(EPCON_TYPE_INT | EPCON_BUF_SINGLE, ep->reg_hcepcon);
		}
		
		ep_reset(acthcd, ep->mask, ENDPRST_FIFORST | ENDPRST_TOGRST);
		
		writeb(ep->epnum, ep->reg_hcepctrl);
		
		if (ep->type == PIPE_INTERRUPT || ep->type == PIPE_ISOCHRONOUS) {
			writeb(ep->interval, ep->reg_hcep_interval);
		}
		
		if (ep->type == PIPE_ISOCHRONOUS) {
			usb_setb(ep->iso_packets << 4, ep->reg_hcepcon);
		}
		
		if (ep->type == PIPE_INTERRUPT)
		{
			if ((ep->udev->speed != USB_SPEED_HIGH) && ep->has_hub) {
				hcep_set_split_micro_frame(acthcd, ep);
			}
			
			ep->ring = aotg_alloc_ring(acthcd, ep, INTR_TRBS, GFP_ATOMIC);
		}
		else {
			ep->ring = aotg_alloc_ring(acthcd, ep, NUM_TRBS, GFP_ATOMIC);
		}
		
		if (!ep->ring)
		{
			ep_disable(ep);
			release_fifo_slot(acthcd, ep);
			
			if (ep->is_out) {
				acthcd->hcep_pool.outep[ep->index] = NULL;
			}
			else {
				acthcd->hcep_pool.inep[ep->index] = NULL;
			}
			kfree(ep);
			
			dev_err(acthcd->dev, "%s: ring allocation failed\n", __func__);
			return -ENOMEM;
		}
		
		INIT_LIST_HEAD(&ep->queue_td_list);
		INIT_LIST_HEAD(&ep->enring_td_list);
		INIT_LIST_HEAD(&ep->dering_td_list);
	}
	
	urb->ep->hcpriv = ep;
	
	dev_info(acthcd->dev, "%s: ep%d %s %s, ptr 0x%llx, maxpacket %d\n", 
	         __func__, ep->epnum, aotg_hcep_get_type_string(ep),
	         aotg_hcep_get_direction_string(ep), (u64)(dma_addr_t) ep->hep,
	         ep->maxpacket);
	
	return 0;
}

static ulong get_fifo_slot(struct aotg_hcd *acthcd, int size)
{
	int i, j;
	ulong addr = 0;
	int mul = DIV_ROUND_UP(size, ALLOC_FIFO_UNIT);
	int max_unit = AOTG_MAX_FIFO_SIZE/ALLOC_FIFO_UNIT;
	int find_next = 0;
	
	for (i = 2; i < max_unit;)
	{
		if (acthcd->fifo_map[i] != 0)
		{
			i++;
			continue; /*find first unused addr*/
		}
		
		for (j = i; j < max_unit; j++)
		{
			if ((j - i + 1) == mul) {
				break;
			}
			if (acthcd->fifo_map[j])
			{
				i = j;
				find_next = 1;
				break;
			}
		}
		
		if (j == max_unit) {
			break;
		}
		else if (find_next)
		{
			find_next = 0;
			continue;
		}
		else
		{
			int k;
			
			for (k = i; k <= j; k++) {
				acthcd->fifo_map[k] = BIT(31) | (i * ALLOC_FIFO_UNIT);
			}
			addr = i * ALLOC_FIFO_UNIT;
			break;
		}
	}
	return addr;
}

void release_fifo_slot(struct aotg_hcd *acthcd, struct aotg_hcep *ep)
{
	int max_unit = AOTG_MAX_FIFO_SIZE / ALLOC_FIFO_UNIT;
	int i;
	
	if (!ep || !ep->fifo_addr) {
		return;
	}
	for (i = ep->fifo_addr / ALLOC_FIFO_UNIT; i < max_unit; i++)
	{
		if ((acthcd->fifo_map[i] & ~BIT(31)) == ep->fifo_addr) {
			acthcd->fifo_map[i] = 0;
		}
		else {
			break;
		}
	}
}

const char* aotg_hcep_get_type_string(struct aotg_hcep *ep)
{
	switch (ep->type)
	{
	case PIPE_CONTROL:
		return "ctrl";
	case PIPE_BULK:
		return "bulk";
	case PIPE_INTERRUPT:
		return "intr";
	/* case PIPE_ISOCHRONOUS */
	default:
		return "iso";
	}
}

const char* aotg_hcep_get_direction_string(struct aotg_hcep *ep)
{
	if (ep->type == PIPE_CONTROL) {
		return "in/out";
	}
	return ep->is_out ? "out" : "in";
}

static int hcep_set_split_micro_frame(
	struct aotg_hcd *acthcd, struct aotg_hcep *ep)
{
	static const u8 split_val[] = {0x31, 0x42, 0x53, 0x64, 0x75, 0x17, 0x20};
	u8 set_val, rd_val;
	int i, index;
	
	for (i = 0; i < sizeof(split_val); i++)
	{
		set_val = split_val[i];
		
		for (index = 0; index < MAX_EP_NUM; index++)
		{
			if (acthcd->hcep_pool.inep[index] != NULL)
			{
				rd_val = acthcd->hcep_pool.inep[index]->reg_hcep_splitcs_val;
				
				if ((0 == rd_val) || (set_val != rd_val)) {
					continue;
				}
				if (set_val == rd_val) {
					set_val = 0;
				}
				break;
			}
		}
		if (set_val == 0) {
			continue;
		}
		
		for (index = 0; index < MAX_EP_NUM; index++)
		{
			if (acthcd->hcep_pool.outep[index] != NULL)
			{
				rd_val = acthcd->hcep_pool.outep[index]->reg_hcep_splitcs_val;
				
				if ((0 == rd_val) || (set_val != rd_val)) {
					continue;
				}
				if (set_val == rd_val) {
					set_val = 0;
				}
				break;
			}
		}
		if (set_val != 0) {
			break;
		}
	}
	
	if (set_val != 0)
	{
		ep->reg_hcep_splitcs_val = set_val;
		writeb(set_val, ep->reg_hcep_splitcs);
	}
	return 0;
}

