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
	
	
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	
	if (retval)
	{
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: aotg_hcep_alloc() returned %d\n",
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
	
	td = aotg_alloc_td(GFP_ATOMIC);
	
	if (!td)
	{
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		
		if (first_time) {
			aotg_hcep_free(hcd, ep);
		}
		
		urb->hcpriv = NULL;
		
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: td alloc failed\n", __func__);
		return -ENOMEM;
	}
	
	td->urb = urb;
	//ep->urb_enque_cnt++;
	
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
				aotg_hcep_free(hcd, ep);
			}
			
			urb->hcpriv = NULL;
			
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
	
	if (!list_empty(&ep->enring_td_list) && !is_ring_running(ep->ring))
	{
		
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS)
		{
			//if (ep->ring->dequeue_trb != ep->ring->first_trb) {
			//	aotg_reorder_iso_td(acthcd, ep->ring);
			//}
		}
		
		aotg_start_ring_transfer(acthcd, ep, urb);
	}
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return 0;
}

