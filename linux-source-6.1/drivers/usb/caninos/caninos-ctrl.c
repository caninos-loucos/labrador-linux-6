#include "aotg_hcd.h"

static int handle_setup_packet(struct aotg_hcd *acthcd, struct aotg_queue *q);


int aotg_hcep_ctrl_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	struct aotg_queue *q;
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
	q = aotg_hcd_queue_alloc(acthcd);
	
	if (!q)
	{
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock_irqrestore(&acthcd->lock, flags);
		dev_err(acthcd->dev, "%s: no slot for new queue\n", __func__);
		return -ENOSPC;
	}
	
	q->ep = ep;
	q->urb = urb;
	
	list_add_tail(&q->enqueue_list, &acthcd->hcd_enqueue_list);
	urb->hcpriv = hcd;
	smp_mb();
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return 0;
}




static int handle_setup_packet(struct aotg_hcd *acthcd, struct aotg_queue *q)
{
	struct urb *urb = q->urb;
	struct aotg_hcep *ep = q->ep;
	u32 *buf;
	void __iomem *addr = acthcd->base + EP0INDATA_W0;
	int i = 0;
	
	if ((q->is_xfer_start) || (ep->q)) {
		dev_info(acthcd->dev,"q->is_xfer_start:%d\n", q->is_xfer_start);
		return 0;
	}
	if (unlikely(!HC_IS_RUNNING(aotg_to_hcd(acthcd)->state)))
	{
		dev_info(acthcd->dev, "%s: hc is not running.\n", __func__);
		return -ESHUTDOWN;
	}
	if (acthcd->active_ep0 != NULL)
	{
		dev_info(acthcd->dev, "%s: ep0 is already active.\n", __func__);
		return -EBUSY;
	}

	writeb(ep->epnum, acthcd->base + HCEP0CTRL);
	writeb((u8)ep->maxpacket, acthcd->base + HCIN0MAXPCK);

	acthcd->active_ep0 = ep;
	ep->q = q;
	q->is_xfer_start = 1;
	usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1, 1);
	ep->nextpid = USB_PID_SETUP;
	buf = (u32 *)urb->setup_packet;

	/*initialize the setup stage */
	writeb(EP0CS_HCSET, acthcd->base + EP0CS);
	while (readb(acthcd->base + EP0CS) & EP0CS_HCOUTBSY) {
		writeb(EP0CS_HCSET, acthcd->base + EP0CS);
		i++;
		if (i > 2000000) {
			dev_err(acthcd->dev,"handle_setup timeout!\n");
			break;
		}
	}

	if (!(readb(acthcd->base + EP0CS) & EP0CS_HCOUTBSY)) {
		/*fill the setup data in fifo */
		writel(*buf, addr);
		addr += 4;
		buf++;
		writel(*buf, addr);
		writeb(8, acthcd->base + HCOUT0BC);
	} else {
		dev_warn(acthcd->dev, "setup ep busy!!!!!!!\n");
	}
	
	return 0;
}

int start_setup_transfer(struct aotg_hcd *acthcd,
	struct aotg_queue *q, struct aotg_hcep *ep)
{
	struct urb *urb = q->urb;
	int retval = 0;

	q->length = urb->transfer_buffer_length;

	/* do with hub connected. */
	if (ep->has_hub) {
		if (urb->dev->speed == USB_SPEED_HIGH) {
			writeb(usb_pipedevice(urb->pipe), ep->reg_hcep_dev_addr);
			writeb(urb->dev->portnum, ep->reg_hcep_port);
		} else {
			writeb((0x80 | usb_pipedevice(urb->pipe)), ep->reg_hcep_dev_addr);
			if (urb->dev->speed == USB_SPEED_LOW)
				writeb(0x80 | urb->dev->portnum, ep->reg_hcep_port);
			else
				writeb(urb->dev->portnum, ep->reg_hcep_port);
		}
		
	} else {
		writeb(usb_pipedevice(urb->pipe), ep->reg_hcep_dev_addr);
		writeb(urb->dev->portnum, ep->reg_hcep_port);
	}
	

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		q->timeout = jiffies + HZ/2;
		retval = handle_setup_packet(acthcd, q);
		break;

	default:
		dev_err(acthcd->dev,"%s code bug\n", __func__);
		break;
	}

	return retval;
}

static inline void handle_status(
	struct aotg_hcd *acthcd, struct aotg_hcep *ep, int is_out)
{
	/*status always DATA1,set 1 to ep0 toggle */
	writeb(EP0CS_HCSETTOOGLE, acthcd->base + EP0CS);
	
	if (is_out)
		writeb(0, acthcd->base + HCIN0BC); /*recv 0 packet*/
	else
		writeb(0, acthcd->base + HCOUT0BC); /*send 0 packet*/
}

static void write_hcep0_fifo(struct aotg_hcd *acthcd,
	struct aotg_hcep *ep, struct urb *urb)
{
	u32 *buf;
	int length, count;
	void __iomem *addr = acthcd->base + EP0INDATA_W0;

	if (!(readb(acthcd->base + EP0CS) & EP0CS_HCOUTBSY)) {
		buf = (u32 *) (urb->transfer_buffer + urb->actual_length);
		prefetch(buf);

		/* how big will this packet be? */
		length = min((int)ep->maxpacket, (int)urb->transfer_buffer_length - (int)urb->actual_length);

		count = length >> 2;	/*wirte in DWORD */
		if (length & 0x3)
			count++;

		while (likely(count--)) {
			writel(*buf, addr);
			buf++;
			addr += 4;
		}

		ep->length = length;
		writeb(length, acthcd->base + HCOUT0BC);
		usb_dotoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1);
	} else {
		dev_err(acthcd->dev, "<CTRL>OUT data is not ready\n");
	}
}

static void read_hcep0_fifo(struct aotg_hcd *acthcd,
	struct aotg_hcep *ep, struct urb *urb)
{
	u8 *buf;
	unsigned overflag, is_short, shorterr, is_last;
	unsigned length, count;
	struct usb_device *udev;
	void __iomem *addr = acthcd->base + EP0OUTDATA_W0; /*HCEP0INDAT0;*/
	unsigned bufferspace;

	overflag = 0;
	is_short = 0;
	shorterr = 0;
	is_last = 0;
	udev = ep->udev;

	if (readb(acthcd->base + EP0CS) & EP0CS_HCINBSY) {
		dev_err(acthcd->dev, "<CTRL>IN data is not ready\n");
		return;
	}
	else
	{
		usb_dotoggle(udev, ep->epnum, 0);
		buf = urb->transfer_buffer + urb->actual_length;
		bufferspace = urb->transfer_buffer_length - urb->actual_length;
		
		length = count = readb(acthcd->base + HCIN0BC);
		if (length > bufferspace) {
			count = bufferspace;
			urb->status = -EOVERFLOW;
			overflag = 1;
		}

		urb->actual_length += count;
		while (count--) {
			*buf++ = readb(addr);
			addr++;
		}

		if (urb->actual_length >= urb->transfer_buffer_length) {
			ep->nextpid = USB_PID_ACK;
			is_last = 1;
			handle_status(acthcd, ep, 0);
		} else if (length < ep->maxpacket) {
			is_short = 1;
			is_last = 1;
			if (urb->transfer_flags & URB_SHORT_NOT_OK) {
				urb->status = -EREMOTEIO;
				shorterr = 1;
			}
			ep->nextpid = USB_PID_ACK;
			handle_status(acthcd, ep, 0);
		} else {
			writeb(0, acthcd->base + HCIN0BC);
		}
	}
}

static void finish_request(
	struct aotg_hcd *acthcd, struct aotg_queue *q, int status)
{
	q->status = status;
	
	if (list_empty(&q->finished_list))
		list_add_tail(&q->finished_list, &acthcd->hcd_finished_list);
	
	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return;
}

void handle_hcep0_out(struct aotg_hcd *acthcd)
{
	struct aotg_hcep *ep;
	struct urb *urb;
	struct usb_device *udev;
	struct aotg_queue *q;
	
	ep = acthcd->active_ep0;
	
	if (unlikely(!ep)) {
		return;
	}
	q = ep->q;
	if (q == NULL) {
		return;
	}
	
	urb = q->urb;
	udev = ep->udev;
	
	switch (ep->nextpid)
	{
	case USB_PID_SETUP:
		if (urb->transfer_buffer_length == urb->actual_length)
		{
			ep->nextpid = USB_PID_ACK;
			handle_status(acthcd, ep, 1); /*no-data transfer */
		}
		else if (usb_pipeout(urb->pipe))
		{
			usb_settoggle(udev, 0, 1, 1);
			ep->nextpid = USB_PID_OUT;
			write_hcep0_fifo(acthcd, ep, urb);
		}
		else
		{
			usb_settoggle(udev, 0, 0, 1);
			ep->nextpid = USB_PID_IN;
			writeb(0, acthcd->base + HCIN0BC);
		}
		break;
		
	case USB_PID_OUT:
		urb->actual_length += ep->length;
		usb_dotoggle(udev, ep->epnum, 1);
		
		if (urb->actual_length >= urb->transfer_buffer_length)
		{
			ep->nextpid = USB_PID_ACK;
			handle_status(acthcd, ep, 1); /*control write transfer */
		}
		else {
			ep->nextpid = USB_PID_OUT;
			write_hcep0_fifo(acthcd, ep, urb);
		}
		break;
		
	case USB_PID_ACK:
		finish_request(acthcd, q, 0);
		break;
		
	default:
		dev_err(acthcd->dev, "<CTRL>ep0 out ,odd pid %d, %s, %d\n",
			ep->nextpid, __func__, __LINE__);
	}
}

void handle_hcep0_in(struct aotg_hcd *acthcd)
{
	struct aotg_hcep *ep;
	struct urb *urb;
	struct usb_device *udev;
	struct aotg_queue *q;
	
	ep = acthcd->active_ep0;
	if (unlikely(!ep)) {
		return;
	}
	
	q = ep->q;
	
	if (q == NULL) {
		return;
	}
	
	urb = q->urb;
	udev = ep->udev;

	switch (ep->nextpid)
	{
	case USB_PID_IN:
		read_hcep0_fifo(acthcd, ep, urb);
		break;
	case USB_PID_ACK:
		finish_request(acthcd, q, 0);
		break;
	default:
		dev_err(acthcd->dev, "<CTRL>ep0 out ,odd pid %d\n", ep->nextpid);
	}
}

void handle_hcep0_error(struct aotg_hcd *acthcd, int is_in)
{
	struct aotg_queue *q;
	struct aotg_hcep *ep;
	u8 err_val, err_type;
	struct urb *urb;
	int status = 0;
	
	if (is_in) {
		writew(BIT(0), acthcd->base + HCINxERRIRQ0);
	}
	else {
		writew(BIT(0), acthcd->base + HCOUTxERRIRQ0);
	}
	
	ep = acthcd->active_ep0;
	
	if (ep == NULL) {
		return;
	}
	
	q = ep->q;
	
	if (is_in) {
		ep->reg_hcerr = acthcd->base + HCIN0ERR;
	}
	else {
		ep->reg_hcerr = acthcd->base + HCOUT0ERR;
	}
	
	err_val = readb(ep->reg_hcerr);
	err_type = err_val & HCINxERR_TYPE_MASK;
	
	switch (err_type)
	{
	case HCINxERR_NO_ERR:
	case HCINxERR_OVER_RUN:
		status = -EOVERFLOW;
		break;
		
	case HCINxERR_UNDER_RUN:
		status = -EREMOTEIO;
		break;
		
	case HCINxERR_STALL:
		status = -EPIPE;
		break;
		
	case HCINxERR_TIMEOUT:
		status = -ETIMEDOUT;
		break;
		
	case HCINxERR_CRC_ERR:
	case HCINxERR_TOG_ERR:
	case HCINxERR_PID_ERR:
		status = -EPROTO;
		break;
		
	/*case HCINxERR_SPLIET:*/
	default:
		status = -EPIPE;
		break;
	}
	
	if (acthcd->hcd_exiting != 0) {
		dev_err(acthcd->dev, "hcd_exiting:%d\n", acthcd->hcd_exiting);
		status = -ENODEV;
	}
	
	urb = q->urb;
	
	if (err_type != HCINxERR_STALL)
	{
		dev_info(acthcd->dev, "%s ep0 error 0x%02X error type 0x%02X\n",
			         __func__, err_val, (err_val>>2)&0x7);
	}
	
	if ((status == -EPIPE) || (status == -ENODEV))
		writeb(HCINxERR_RESEND, ep->reg_hcerr);
	
	finish_request(acthcd, q, status);
}

