#include "aotg_hcd.h"

int aotg_hcep_intr_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_td *td, *next;
	struct aotg_hcep *ep;
	unsigned long flags;
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
		
		ep->ring = aotg_alloc_ring(acthcd, ep, INTR_TRBS, GFP_ATOMIC);
		
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
		
		dev_info(acthcd->dev, "%s: ep %d %s, ptr 0x%llx, maxpacket %d\n", 
		         __func__, ep->epnum, ep->is_out ? "out" : "in",
		         (u64)(dma_addr_t) ep->hep, ep->maxpacket);
	}
	else
	{
		ep = urb->ep->hcpriv;
		
		if (ep->hep != urb->ep)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: mismatch, ep %d %s\n",
			        __func__, ep->epnum, ep->is_out ? "out" : "in");
			return -EINVAL;
		}
	}
	
	urb->hcpriv = hcd;
	
	if (unlikely(ep->ring->intr_inited == 0))
	{
		retval = aotg_ring_enqueue_intr_td(acthcd, ep, urb, GFP_ATOMIC);
		
		if (retval)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			
			if (first_time) {
				hcep_free(hcd, ep);
			}
			
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: ring enqueue intr failed\n", __func__);
			return retval;
		}
		
		ep->ring->intr_started = 0;
	}
	
	ep->urb_enque_cnt++;
	
	list_for_each_entry_safe(td, next, &ep->enring_td_list, enring_list)
	{
		if (td->urb) {
			continue;
		}
		else {
			td->urb = urb;
			break;
		}
		// TODO: what if td->urb is not available?
	}
	
	if (ep->ring->enqueue_trb->hw_buf_len != urb->transfer_buffer_length)
	{
		aotg_intr_chg_buf_len(acthcd, ep->ring, urb->transfer_buffer_length);
		// TODO: what if it fails?
		
		dev_info(acthcd->dev, "%s: interrupt urb length changed\n", __func__);
	}
	
	if (ep->ring->intr_started == 0)
	{
		ep->ring->intr_started = 1;
		aotg_start_ring_transfer(acthcd, ep, urb);
	}
	
	if (!is_ring_running(ep->ring))
	{
		if (ep->is_out) {
			aotg_start_ring_transfer(acthcd, ep, urb);
		}
		else
		{
			if (aotg_intr_get_finish_trb(ep->ring) == 0)
			{
				ep->ring->ring_stopped = 0;
				aotg_reorder_intr_td(ep);
				ep_enable(ep);
				mb();
				writel(DMACTRL_DMACS,ep->ring->reg_dmactrl);
			}
			else {
				ep->ring->ring_stopped = 1;
			}
		}
	}
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return 0;
}

