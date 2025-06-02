#include "aotg_hcd.h"

static int config(struct usb_hcd *hcd, struct urb *urb, struct aotg_hcep *ep);

int aotg_hcep_ctrl_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	struct aotg_queue *q;
	unsigned long flags;
	bool first_time;
	int retval, i;
	
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
		
		retval = config(hcd, urb, ep);
		
		if (retval)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			kfree(ep);
			dev_err(acthcd->dev, "%s: config failed\n", __func__);
			return retval;
		}
		
		urb->ep->hcpriv = ep;
		
		dev_info(acthcd->dev, "%s: ep %d in/out, ptr 0x%llx, maxpacket %d\n", 
		         __func__, ep->epnum, (u64)(dma_addr_t) ep->hep,
		         ep->maxpacket);
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
	
	q = NULL;
	
	for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++)
	{
		if (acthcd->queue_pool[i] != NULL)
		{
			if (acthcd->queue_pool[i]->in_using == 0)
			{
				q = acthcd->queue_pool[i];
				memset(q, 0, sizeof(*q));
				q->in_using = 1;
				break;
			}
		}
	}
	
	if (!q)
	{
		q = kzalloc(sizeof(*q), GFP_ATOMIC);
		
		if (!q)
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			dev_err(acthcd->dev, "%s: unable to alloc q\n", __func__);
			return -ENOMEM;
		}
		
		for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++)
		{
			if (acthcd->queue_pool[i] == NULL)
			{
				q->in_using = 1;
				acthcd->queue_pool[i] = q;
				break;
			}
		}
		
		if (!q->in_using) /* slot for new pointer not available */
		{
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			kfree(q);
			dev_err(acthcd->dev, "%s: no slot for new pointer\n", __func__);
			return -ENOSPC;
		}
	}
	
	q->length = 0;
	q->td.trb_vaddr = NULL;
	q->ep = ep;
	q->urb = urb;
	
	INIT_LIST_HEAD(&q->enqueue_list);
	INIT_LIST_HEAD(&q->dequeue_list);
	INIT_LIST_HEAD(&q->finished_list);
	
	urb->hcpriv = hcd;
	
	list_add_tail(&q->enqueue_list, &acthcd->hcd_enqueue_list);
	spin_unlock_irqrestore(&acthcd->lock, flags);
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	
	return 0;
}

static int config(struct usb_hcd *hcd, struct urb *urb, struct aotg_hcep *ep)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	int i;
	
	ep->is_out = usb_pipeout(urb->pipe) ? 1 : 0;
	ep->udev = usb_get_dev(urb->dev);
	ep->epnum = usb_pipeendpoint(urb->pipe);
	ep->maxpacket = usb_maxpacket(ep->udev, urb->pipe);
	ep->type = usb_pipetype(urb->pipe);
	ep->urb_enque_cnt = 0;
	ep->urb_endque_cnt = 0;
	ep->urb_stop_stran_cnt = 0;
	ep->urb_unlinked_cnt = 0;
	ep->ep0_index = -1;
	ep->length = 0;
	ep->index = 0;
	ep->mask = 0;
	ep->has_hub = 0;
	
	if (acthcd->active_ep0 != NULL) {
		dev_err(acthcd->dev, "%s: ep0 already active\n", __func__);
		return -EBUSY;
	}
	
	for (i = 0; i < MAX_EP_NUM; i++)
	{
		if (acthcd->hcep_pool.ep0[i] == NULL)
		{
			ep->ep0_index = i;
			break;
		}
	}
	
	if (ep->ep0_index < 0) {
		dev_err(acthcd->dev, "%s: no slot for ep0\n", __func__);
		return -ENOSPC;
	}
	
	acthcd->hcep_pool.ep0[ep->ep0_index] = ep;
	
	ep->reg_hcep_dev_addr = acthcd->base + HCEP0ADDR;
	ep->reg_hcep_port = acthcd->base + HCEP0PORT;
	ep->reg_hcep_splitcs = acthcd->base + HCEP0SPILITCS;
	
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
	
	ep->hep = urb->ep;
	
	return 0;
}

