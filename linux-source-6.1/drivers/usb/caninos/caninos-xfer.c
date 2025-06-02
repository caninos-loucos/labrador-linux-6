#include "aotg_hcd.h"

int aotg_hcep_xfer_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	unsigned long flags;
	struct aotg_td *td;
	bool first_time;
	int retval;
	
	if (unlikely(acthcd == NULL)) {
		return -EIO;
	}
	
	spin_lock_irqsave(&acthcd->lock, flags);
	
	if (!(acthcd->port & USB_PORT_STAT_ENABLE) ||
	    !(HC_IS_RUNNING(hcd->state)) ||
	    (acthcd->port & (USB_PORT_STAT_C_CONNECTION << 16)) ||
	    (acthcd->hcd_exiting != 0) || (acthcd->inserted == 0))
	{
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: device removed\n", __func__);
		return -ENODEV;
	}
	
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	
	if (retval)
	{
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: unable to link urb to ep\n", __func__);
		return retval;
	}
	
	first_time = !urb->ep->hcpriv;
	
	if (first_time)
	{
		ep = kzalloc(sizeof(*ep), GFP_ATOMIC);
		
		if (!ep)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: unable to alloc hcep\n", __func__);
			return -ENOMEM;
		}
		
		retval = hcep_config(hcd, urb, ep);
		
		if (retval)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			hcep_free(hcd, ep);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: config failed\n", __func__);
			return retval;
		}
		
		ep->ring = aotg_alloc_ring(acthcd, ep, NUM_TRBS, GFP_ATOMIC);
		
		if (!ep->ring)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			hcep_free(hcd, ep);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: alloc ring failed\n", __func__);
			return -ENOMEM;
		}
		
		INIT_LIST_HEAD(&ep->queue_td_list);
		INIT_LIST_HEAD(&ep->enring_td_list);
		INIT_LIST_HEAD(&ep->dering_td_list);
		
		urb->ep->hcpriv = ep;
		
		dev_info(acthcd->dev, "%s: %s ep %d %s, ptr 0x%llx, maxpacket %d\n", 
		         __func__, ep->type == PIPE_BULK ? "bulk" : "iso",
		         ep->epnum, ep->is_out ? "out" : "in", 
		         (u64)(dma_addr_t) ep->hep, ep->maxpacket);
	}
	else
	{
		ep = urb->ep->hcpriv;
		
		if (ep->hep != urb->ep)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			
			dev_err(acthcd->dev, "%s: mismatch, %s ep %d %s\n", 
			        __func__, ep->type == PIPE_BULK ? "bulk" : "iso",
			        ep->epnum, ep->is_out ? "out" : "in");
			
			return -EINVAL;
		}
	}
	
	urb->hcpriv = hcd;
	
	td = aotg_alloc_td(GFP_ATOMIC);
	
	if (!td)
	{
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		
		if (first_time) {
			hcep_free(hcd, ep);
		}
		
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: td alloc failed\n", __func__);
		return -ENOMEM;
	}
	
	td->urb = urb;
	ep->urb_enque_cnt++;
	
	if (list_empty(&ep->queue_td_list))
	{
		if (usb_pipetype(urb->pipe) == PIPE_BULK) {
			retval = aotg_ring_enqueue_td(acthcd, ep->ring, td);
		}
		else {
			retval = aotg_ring_enqueue_isoc_td(acthcd, ep->ring, td);
		}
		
		if (retval == 0)
		{
			list_add_tail(&td->enring_list, &ep->enring_td_list);
			ep->ring->enring_cnt++;
		}
		else if (retval == -ENOSPC)
		{
			list_add_tail(&td->queue_list, &ep->queue_td_list);
		}
		else
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			
			//ep->urb_enque_cnt--;
			// TODO: dequeue and free td
			
			if (first_time) {
				hcep_free(hcd, ep);
			}
			
			spin_unlock_irqrestore(&acthcd->lock, flags);
			
			dev_err(acthcd->dev, "%s: enqueue td failed, %s ep %d %s\n", 
			        __func__, ep->type == PIPE_BULK ? "bulk" : "iso",
			        ep->epnum, ep->is_out ? "out" : "in");
			
			return -ENOMEM;
		}
	}
	else
	{
		list_add_tail(&td->queue_list, &ep->queue_td_list);
	}
	
	if (!list_empty(&ep->enring_td_list) && 
	    !is_ring_running(ep->ring))
	{
		
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS)
		{
			if (ep->ring->dequeue_trb != ep->ring->first_trb) {
				aotg_reorder_iso_td(acthcd, ep->ring);
			}
		}
		
		aotg_start_ring_transfer(acthcd, ep, urb);
	}
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return 0;
}

static int aotg_hcep_set_split_micro_frame(
	struct aotg_hcd *acthcd, struct aotg_hcep *ep)
{
	static const u8 split_val[] = {0x31, 0x42, 0x53, 0x64, 0x75, 0x17, 0x20};
	int i, index;
	u8 set_val, rd_val;
	
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
		
		if (set_val == 0)
			continue;
		
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

/* must call with spinlock */
int hcep_config(struct usb_hcd *hcd, struct urb *urb, struct aotg_hcep *ep)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	int i, retval;
	
	ep->is_out = usb_pipeout(urb->pipe) ? 1 : 0;
	ep->udev = usb_get_dev(urb->dev);
	ep->epnum = usb_pipeendpoint(urb->pipe);
	ep->maxpacket = usb_maxpacket(ep->udev, urb->pipe);
	ep->type = usb_pipetype(urb->pipe);
	ep->urb_enque_cnt = 0;
	ep->urb_endque_cnt = 0;
	ep->urb_stop_stran_cnt = 0;
	ep->urb_unlinked_cnt = 0;
	ep->length = 0;
	ep->has_hub = 0;
	ep->fifo_addr = 0;
	ep->buftype = EPCON_BUF_SINGLE;
	ep->index = -1;
	
	if (ep->type == PIPE_INTERRUPT || ep->type == PIPE_ISOCHRONOUS) {
		ep->interval = urb->ep->desc.bInterval;
	}
	
	if (ep->type == PIPE_ISOCHRONOUS) {
		ep->iso_packets = (urb->ep->desc.wMaxPacketSize >> 11) & 3;
		
	}
	
	for (i = 1; i < MAX_EP_NUM; i++) /* 0 is reserved */
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
	
	if (ep->index < 0) {
		dev_err(acthcd->dev, "%s: no slot for ep\n", __func__);
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
		dev_err(acthcd->dev, "%s: no fifo slot for ep\n", __func__);
		return retval;
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
	
	if (ep->type == PIPE_INTERRUPT) {
		if ((ep->udev->speed != USB_SPEED_HIGH) && ep->has_hub) {
			aotg_hcep_set_split_micro_frame(acthcd, ep);
		}
	}
	
	ep->hep = urb->ep;
	return 0;
}

void release_fifo_slot(struct aotg_hcd *acthcd, struct aotg_hcep *ep)
{
	int max_unit = AOTG_MAX_FIFO_SIZE / ALLOC_FIFO_UNIT;
	int i = ep->fifo_addr / ALLOC_FIFO_UNIT;
	
	for (; i < max_unit; i++)
	{
		if ((acthcd->fifo_map[i] & ~BIT(31)) == ep->fifo_addr) {
			acthcd->fifo_map[i] = 0;
		}
		else {
			break;
		}
	}
}

/* must call with spinlock */
ulong get_fifo_slot(struct aotg_hcd *acthcd, int size)
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

/* must call with spinlock */
void hcep_free(struct usb_hcd *hcd, struct aotg_hcep *ep)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	
	if (ep != NULL)
	{
		writel(DMACTRL_DMACC, ep->ring->reg_dmactrl);
		ep_disable(ep);
		
		if (ep->hep)
		{
			ep->hep->hcpriv = NULL;
			ep->hep = NULL;
		}
		
		release_fifo_slot(acthcd, ep);
		
		if (ep->is_out) {
			acthcd->hcep_pool.outep[ep->index] = NULL;
		}
		else {
			acthcd->hcep_pool.inep[ep->index] = NULL;
		}
		
		aotg_free_ring(acthcd, ep->ring);
		
		kfree(ep);
	}
}

