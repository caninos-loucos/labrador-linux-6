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
	
	
	
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	
	if (retval)
	{
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: usb_hcd_link_urb_to_ep() returned %d\n",
		        __func__, retval);
		return retval;
	}
	
	first_time = !urb->ep->hcpriv;
	
	if (first_time)
	{
		retval = aotg_hcep_alloc(hcd, urb);
		
		if (retval)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: aotg_hcep_alloc() returned %d\n",
			        __func__, retval);
			return retval;
		}
	}
	
	ep = urb->ep->hcpriv;
	urb->hcpriv = hcd;
	
	if (unlikely(ep->ring->intr_inited == 0))
	{
		retval = aotg_ring_enqueue_intr_td(acthcd, ep, urb, GFP_ATOMIC);
		
		if (retval)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			
			if (first_time) {
				aotg_hcep_free(hcd, ep);
			}
			
			urb->hcpriv = NULL;
			
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: ring enqueue intr failed\n", __func__);
			return retval;
		}
		
		ep->ring->intr_started = 0;
	}
	
	//ep->urb_enque_cnt++;
	
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

