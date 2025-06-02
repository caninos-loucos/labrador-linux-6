#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>

#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <linux/suspend.h>


#include "aotg_hcd.h"

static int handle_setup_packet(struct aotg_hcd *acthcd, struct aotg_queue *q);
static void handle_hcep0_in(struct aotg_hcd *acthcd);
static void handle_hcep0_out(struct aotg_hcd *acthcd);

static void release_fifo_addr(struct aotg_hcd *acthcd, ulong addr)
{
	int i;

	for (i = addr/ALLOC_FIFO_UNIT; i < AOTG_MAX_FIFO_SIZE/ALLOC_FIFO_UNIT; i++) {
		if ((acthcd->fifo_map[i] & 0x7FFFFFFF) == addr)
			acthcd->fifo_map[i] = 0;
		else
			break;
	}
	return;
}



void aotg_hcd_release_queue(struct aotg_hcd *acthcd, struct aotg_queue *q)
{
	int i;

	/* release all */
	if (q == NULL) {
		for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++) {
			if (acthcd->queue_pool[i] != NULL) {
				kfree(acthcd->queue_pool[i]);
				acthcd->queue_pool[i] = NULL;
			}
		}
		return;
	}

	q->td.trb_vaddr = NULL;

	for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++) {
		if (acthcd->queue_pool[i] == q) {
			acthcd->queue_pool[i]->in_using = 0;
			return;
		}
	}

	kfree(q);
	return;
}

static inline int is_epfifo_busy(struct aotg_hcep *ep, int is_in)
{
	if (is_in)
		return (EPCS_BUSY & readb(ep->reg_hcepcs)) == 0;
	else
		return (EPCS_BUSY & readb(ep->reg_hcepcs)) != 0;
}

static inline void ep_setup(struct aotg_hcep *ep, u8 type, u8 buftype)
{
	ep->buftype = buftype;
	writeb(type | buftype, ep->reg_hcepcon);
}

static inline void aotg_sofirq_clr(struct aotg_hcd *acthcd)
{
	usb_setbitsb((1 << 1), acthcd->base + USBIRQ);
}

static inline void aotg_sofirq_on(struct aotg_hcd *acthcd)
{
	usb_setbitsb((1 << 1), acthcd->base + USBIEN);
}

static inline void aotg_sofirq_off(struct aotg_hcd *acthcd)
{
	usb_clearbitsb(1 << 1, acthcd->base + USBIEN);
}

static inline int get_subbuffer_count(u8 buftype)
{
	int count = 0;

	switch (buftype) {
	case EPCON_BUF_SINGLE:
		count = 1;
		break;
	case EPCON_BUF_DOUBLE:
		count = 2;
		break;
	case EPCON_BUF_TRIPLE:
		count = 3;
		break;
	case EPCON_BUF_QUAD:
		count = 4;
		break;
	}

	return count;
}

static inline void aotg_config_hub_addr(struct urb *urb, struct aotg_hcep *ep)
{
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
}

void aotg_start_ring_transfer(struct aotg_hcd *acthcd,
	struct aotg_hcep *ep, struct urb *urb)
{
	u32 addr;
	struct aotg_trb *trb;
	struct aotg_ring *ring = ep->ring;

	aotg_config_hub_addr(urb, ep);
	if (usb_pipetype(urb->pipe) == PIPE_INTERRUPT) {
		writeb(ep->interval, ep->reg_hcep_interval);
		if (ring->is_out) {
			trb = ring->dequeue_trb;
			trb->hw_buf_ptr = urb->transfer_dma;
			trb->hw_buf_len = urb->transfer_buffer_length;
		}

	}
	ep_enable(ep);
	addr = ring_trb_virt_to_dma(ring, ring->dequeue_trb);
	aotg_start_ring(ring, addr);
}

static void finish_request(struct aotg_hcd *acthcd,
	struct aotg_queue *q, int status)
{
	struct urb *urb = q->urb;

	if (unlikely((acthcd == NULL) || (q == NULL) || (urb == NULL))) {
		WARN_ON(1);
		return;
	}

	q->status = status;
	
	if (list_empty(&q->finished_list))
		list_add_tail(&q->finished_list, &acthcd->hcd_finished_list);
	

	tasklet_hi_schedule(&acthcd->urb_tasklet);
	return;
}

static void tasklet_finish_request(struct aotg_hcd *acthcd,
	struct aotg_queue *q, int status)
{
	struct urb *urb = q->urb;
	struct aotg_hcep *ep = q->ep;

	if (unlikely((acthcd == NULL) || (q == NULL) || (urb == NULL))) {
		WARN_ON(1);
		return;
	}

	if ((q != NULL) && (ep != NULL)) {
		if (ep->q == NULL) {
			
		} else {
			if (ep->q == q)
				ep->q = NULL;
		}
	} else {
		
		return;
	}

	if (status == 0)
		q->err_count = 0;

	if (usb_pipetype(urb->pipe) == PIPE_CONTROL) {
		if ((acthcd->active_ep0 != NULL) && (acthcd->active_ep0 == q->ep)) {
			if (acthcd->active_ep0->q == NULL)
				acthcd->active_ep0 = NULL;
			
		}
	}


	aotg_hcd_release_queue(acthcd, q);

	ep->urb_endque_cnt++;
	return;
}

static inline void handle_status(struct aotg_hcd *acthcd,
	struct aotg_hcep *ep, int is_out)
{
	/*status always DATA1,set 1 to ep0 toggle */
	writeb(EP0CS_HCSETTOOGLE, acthcd->base + EP0CS);

	if (is_out)
		writeb(0, acthcd->base + HCIN0BC);	/*recv 0 packet*/
	else
		writeb(0, acthcd->base + HCOUT0BC);	/*send 0 packet*/
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
	void __iomem *addr = acthcd->base + EP0OUTDATA_W0;	/*HCEP0INDAT0;*/
	unsigned bufferspace;

	overflag = 0;
	is_short = 0;
	shorterr = 0;
	is_last = 0;
	udev = ep->udev;

	if (readb(acthcd->base + EP0CS) & EP0CS_HCINBSY) {
		dev_err(acthcd->dev, "<CTRL>IN data is not ready\n");
		return;
	} else {
		usb_dotoggle(udev, ep->epnum, 0);
		buf = urb->transfer_buffer + urb->actual_length;
		bufferspace = urb->transfer_buffer_length - urb->actual_length;
		/*prefetch(buf);*/

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

static void handle_hcep0_out(struct aotg_hcd *acthcd)
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

	switch (ep->nextpid) {
	case USB_PID_SETUP:
		if (urb->transfer_buffer_length == urb->actual_length) {
			ep->nextpid = USB_PID_ACK;
			handle_status(acthcd, ep, 1);	/*no-data transfer */
		} else if (usb_pipeout(urb->pipe)) {
			usb_settoggle(udev, 0, 1, 1);
			ep->nextpid = USB_PID_OUT;
			write_hcep0_fifo(acthcd, ep, urb);
		} else {
			usb_settoggle(udev, 0, 0, 1);
			ep->nextpid = USB_PID_IN;
			writeb(0, acthcd->base + HCIN0BC);
		}
		break;
	case USB_PID_OUT:
		urb->actual_length += ep->length;
		usb_dotoggle(udev, ep->epnum, 1);
		if (urb->actual_length >= urb->transfer_buffer_length) {
			ep->nextpid = USB_PID_ACK;
			handle_status(acthcd, ep, 1);	/*control write transfer */
		} else {
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

static void handle_hcep0_in(struct aotg_hcd *acthcd)
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

	switch (ep->nextpid) {
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

static void aotg_hcd_err_handle(
	struct aotg_hcd *acthcd, u32 irqvector, int ep_num, int is_in)
{
	struct urb *urb;
	struct aotg_queue *q;
	struct aotg_hcep *ep = NULL;
	struct aotg_ring *ring = NULL;
	struct aotg_td *td = NULL;
	int status = -EOVERFLOW;
	u8 err_val = 0;
	u8 err_type = 0;
	u8 reset = 0;
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);
	
	dev_info(acthcd->dev, "hcd ep err ep_num:%d, is_in:%d\n", ep_num, is_in);
	
	if (is_in)
		writew(1 << ep_num, acthcd->base + HCINxERRIRQ0);
	else
		writew(1 << ep_num, acthcd->base + HCOUTxERRIRQ0);
	if (ep_num == 0) {
		ep = acthcd->active_ep0;
		if (ep == NULL) {
			return;
		}
		q = ep->q;
		if (is_in)
			ep->reg_hcerr = acthcd->base + HCIN0ERR;
		else
			ep->reg_hcerr = acthcd->base + HCOUT0ERR;
	} else {
		if (is_in)
			ep = acthcd->hcep_pool.inep[ep_num];
		else
			ep = acthcd->hcep_pool.outep[ep_num];
		if (ep == NULL) {
			
			dev_info(acthcd->dev,"is_in:%d, ep_num:%d\n", is_in, ep_num);
			return;
		}
		ring = ep->ring;
		if (!ring) {
			return;
		}
		td = list_first_entry_or_null(&ep->enring_td_list, struct aotg_td, enring_list);
		if (!td) {
			aotg_stop_ring(ring);
			return;
		}
	}
	
	err_val = readb(ep->reg_hcerr);
	err_type = err_val & HCINxERR_TYPE_MASK;
	
	dev_info(acthcd->dev, "err_type:%x\n", err_type>>2);
	
	switch (err_type) {
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
		dev_info(acthcd->dev, "err_val:0x%x, err_type:%d\n", err_val, err_type);
		
		if (is_in)
		{
			dev_info(acthcd->dev,"HCINEP%dSPILITCS:0x%x\n", ep_num,
			readb(acthcd->base + ep_num * 8 + HCEP0SPILITCS));
		}
		else
		{
			dev_info(acthcd->dev,"HCOUTEP%dSPILITCS:0x%x\n", ep_num,
			readb(acthcd->base + (ep_num - 1) * 8 + HCOUT1SPILITCS));
		}
		status = -EPIPE;
		break;
	}

	if (!(acthcd->port & USB_PORT_STAT_ENABLE)
		|| (acthcd->port & (USB_PORT_STAT_C_CONNECTION << 16))
		|| (acthcd->hcd_exiting != 0)
		|| (acthcd->inserted == 0)
		|| !HC_IS_RUNNING(hcd->state)) {
		dev_err(acthcd->dev, "usbport, dead, port:%x, hcd_exiting:%d\n", acthcd->port, acthcd->hcd_exiting);
		status = -ENODEV;
	}

	if (ep->index == 0) {
		q = ep->q;
		urb = q->urb;
		if ((status == -EPIPE) || (status == -ENODEV))
			writeb(HCINxERR_RESEND, ep->reg_hcerr);
		finish_request(acthcd, q, status);
		dev_info(acthcd->dev, "%s ep %d error [0x%02X] error type [0x%02X], reset it...\n",
			usb_pipeout(urb->pipe) ? "HC OUT" : "HC IN", ep->index, err_val, (err_val>>2)&0x7);
	} else {
		if ((status != -EPIPE) && (status != -ENODEV)) {
			dev_info(acthcd->dev,"td->err_count:%d,ep_errcount:%d\n", td->err_count, ep->error_count);
			td->err_count++;

			if ((td->err_count < MAX_ERROR_COUNT) && (ep->error_count < 3)) {
				writeb(HCINxERR_RESEND, ep->reg_hcerr);
				return;
			}
		}

		if (status == -ETIMEDOUT || status == -EPIPE)
			ep->error_count++;

		reset = ENDPRST_FIFORST | ENDPRST_TOGRST;
		ep_disable(ep);
		if (is_in)
			ep_reset(acthcd, ep->mask, reset);
		else
			ep_reset(acthcd, ep->mask | USB_HCD_OUT_MASK, reset);

		/*if (usb_pipeout(urb->pipe)) {
			ep_reset(acthcd, ep->mask | USB_HCD_OUT_MASK, reset);
		} else {
			ep_reset(acthcd, ep->mask, reset);
		}*/

		aotg_stop_ring(ring);
		urb = td->urb;
		if (ep->type == PIPE_INTERRUPT)
			dequeue_intr_td(ring, td);
		else
			dequeue_td(ring, td, TD_IN_FINISH);

		if (urb) {
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			usb_hcd_giveback_urb(hcd, urb, status);
		} else {
			dev_info(acthcd->dev,"urb not exist!\n");
		}

		/*
		 * after, need to rewrite port_num, dev_addr when using hub ?
		 */
		
		dev_info(acthcd->dev, "%s ep %d error [0x%02X] error type [0x%02X], reset it...\n",
			is_in ? "HC IN" : "HC OUT", ep->index, err_val, (err_val>>2)&0x7);
	}

	return;
}

static void handle_suspend(struct aotg_hcd *acthcd)
{
	usb_clearbitsb(SUSPEND_IRQIEN, acthcd->base + USBEIEN);
	usb_setbitsb(SUSPEND_IRQIEN, acthcd->base + USBEIRQ);
	
	aotg_sofirq_clr(acthcd);
	aotg_sofirq_on(acthcd);
}

static void handle_sof(struct aotg_hcd *acthcd)
{
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);
	
	aotg_sofirq_clr(acthcd);
	aotg_sofirq_off(acthcd);
	
	if (HC_IS_SUSPENDED(hcd->state)) {
		usb_hcd_resume_root_hub(hcd);
	}

	usb_hcd_poll_rh_status(hcd);
}

void aotg_hcd_abort_urb(struct aotg_hcd *acthcd)
{
	int cnt;
	struct aotg_hcep *ep;
	struct urb *urb;
	struct aotg_ring *ring;
	struct aotg_td *td;
	unsigned long flags;
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);

	spin_lock_irqsave(&acthcd->lock, flags);
	

	/* Stop DMA first */
	for (cnt = 1; cnt < MAX_EP_NUM; cnt++) {
		ep = acthcd->hcep_pool.inep[cnt];
		ring = ep ? ep->ring : NULL;
		if (ep && ring)
			aotg_stop_ring(ring);
	}

	for (cnt = 1; cnt < MAX_EP_NUM; cnt++) {
		ep = acthcd->hcep_pool.outep[cnt];
		ring = ep ? ep->ring : NULL;
		if (ep && ring)
			aotg_stop_ring(ring);
	}

	for (cnt = 1; cnt < MAX_EP_NUM; cnt++) {
		ep = acthcd->hcep_pool.inep[cnt];
		if (ep) {
			ring = ep->ring;
			td = list_first_entry_or_null(&ep->enring_td_list, struct aotg_td, enring_list);
			if (!td)
				continue;
			urb = td->urb;
			if (!urb)
				continue;
			if (ep->type == PIPE_INTERRUPT)
				dequeue_intr_td(ring, td);
			else
				dequeue_td(ring, td, TD_IN_FINISH);
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock(&acthcd->lock);
			usb_hcd_giveback_urb(hcd, urb, -ENODEV);
			spin_lock(&acthcd->lock);
		}
	}

	for (cnt = 1; cnt < MAX_EP_NUM; cnt++) {
		ep = acthcd->hcep_pool.outep[cnt];
		if (ep) {
			ring = ep->ring;
			td = list_first_entry_or_null(&ep->enring_td_list, struct aotg_td, enring_list);
			if (!td)
				continue;
			urb = td->urb;
			if (!urb)
				continue;
			if (ep->type == PIPE_INTERRUPT)
				dequeue_intr_td(ring, td);
			else
				dequeue_td(ring, td, TD_IN_FINISH);

			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock(&acthcd->lock);
			usb_hcd_giveback_urb(hcd, urb, -ENODEV);
			spin_lock(&acthcd->lock);
		}
	}
	spin_unlock_irqrestore(&acthcd->lock, flags);
}

static irqreturn_t aotg_hub_irq(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	
	u32 irqvector;
	
	u8 eirq_mask = readb(acthcd->base + USBEIEN);
	u8 eirq_pending = readb(acthcd->base + USBEIRQ);
	u8 otg_state;
	
	if ((eirq_pending & SUSPEND_IRQIEN) && (eirq_mask & SUSPEND_IRQIEN)) {
		handle_suspend(acthcd);
		return IRQ_HANDLED;
	}
	
	if (eirq_pending & USBEIRQ_USBIRQ)
	{
		irqvector = (u32)readb(acthcd->base + IVECT);
		writeb(eirq_mask & USBEIRQ_USBIRQ, acthcd->base + USBEIRQ);
		
		switch (irqvector)
		{
		case UIV_IDLE:
		case UIV_SRPDET:
		case UIV_LOCSOF:
		case UIV_VBUSERR:
		case UIV_PERIPH:
			
			//when device is inserted: OTG IRQ, OTGSTATE: 0x03, USBIRQ:0x00
			//when device is removed:  OTG IRQ, OTGSTATE: 0x02, USBIRQ:0x02
			
			if (readb(acthcd->base + OTGIRQ) & (0x1<<2))
			{
				writeb(0x1<<2, acthcd->base + OTGIRQ);
				otg_state = readb(acthcd->base + OTGSTATE);
				
				if (otg_state == AOTG_STATE_A_SUSPEND) {
					return IRQ_HANDLED;
				}
				
				acthcd->put_aout_msg = 0;
				
				if (acthcd->discon_happened)
				{
					hrtimer_start(
						&acthcd->hotplug_timer,
						ktime_set(0, 500*NSEC_PER_MSEC), HRTIMER_MODE_REL);
				}
				else
				{
					acthcd->discon_happened = 1;
					hrtimer_start(
						&acthcd->hotplug_timer,
						ktime_set(0, 10*NSEC_PER_MSEC), HRTIMER_MODE_REL);
				}
			}
			else {
				dev_info(acthcd->dev, "error OTG irq! OTGIRQ: 0x%02X\n",
					readb(acthcd->base + OTGIRQ));
			}
			break;
			
		case UIV_SOF:
			writeb(USBIRQ_SOF, acthcd->base + USBIRQ);
			
			if (acthcd->bus_remote_wakeup) 
			{
				acthcd->bus_remote_wakeup = 0;
				acthcd->port |= (USB_PORT_STAT_C_SUSPEND<<16);
				acthcd->port &= ~USB_PORT_STAT_C_SUSPEND;
			}
			
			handle_sof(acthcd);
			break;
			
		case UIV_USBRESET:
			
			if (acthcd->port & (USB_PORT_STAT_POWER | USB_PORT_STAT_CONNECTION))
			{
				acthcd->speed = USB_SPEED_FULL; /*FS is the default */
				acthcd->port |= (USB_PORT_STAT_C_RESET << 16);
				acthcd->port &= ~USB_PORT_STAT_RESET;
				
				/*clear usb reset irq */
				writeb(USBIRQ_URES, acthcd->base + USBIRQ);
				
				/*reset all ep-in */
				ep_reset(
					acthcd, USB_HCD_IN_MASK,
					ENDPRST_FIFORST | ENDPRST_TOGRST);
				
				/*reset all ep-out */
				ep_reset(
					acthcd, USB_HCD_OUT_MASK,
					ENDPRST_FIFORST | ENDPRST_TOGRST);
				
				acthcd->port |= USB_PORT_STAT_ENABLE;
				acthcd->rhstate = AOTG_RH_ENABLE;
				
				/*now root port is enabled fully */
				if (readb(acthcd->base + USBCS) & USBCS_HFMODE)
				{
					acthcd->speed = USB_SPEED_HIGH;
					acthcd->port |= USB_PORT_STAT_HIGH_SPEED;
					writeb(USBIRQ_HS, acthcd->base + USBIRQ);
				}
				else if (readb(acthcd->base + USBCS) & USBCS_LSMODE)
				{
					acthcd->speed = USB_SPEED_LOW;
					acthcd->port |= USB_PORT_STAT_LOW_SPEED;
				}
				else {
					acthcd->speed = USB_SPEED_FULL;
				}
				
				writew(0xffff, acthcd->base + HCINxERRIRQ0);
				writew(0xffff, acthcd->base + HCOUTxERRIRQ0);
				writew(0xffff, acthcd->base + HCINxIRQ0);
				writew(0xffff, acthcd->base + HCOUTxIRQ0);
				writew(0xffff, acthcd->base + HCINxERRIEN0);
				writew(0xffff, acthcd->base + HCOUTxERRIEN0);
				
			}
			break;
			
		case UIV_EP0IN:
			writew(0x1, acthcd->base + HCOUTxIRQ0); //clear hcep0out irq
			handle_hcep0_out(acthcd);
			break;
			
		case UIV_EP0OUT:
			writew(0x1, acthcd->base + HCINxIRQ0); //clear hcep0in irq
			handle_hcep0_in(acthcd);
			break;
			
		case UIV_EP1IN:
			writew(0x1 << 1, acthcd->base + HCOUTxIRQ0); //clear hcep1out irq
			break;
			
		case UIV_EP1OUT:
			writew(0x1 << 1, acthcd->base + HCINxIRQ0); //clear hcep1in irq
			break;
			
		case UIV_EP2IN:
			writew(0x1 << 2, acthcd->base + HCOUTxIRQ0); //clear hcep2out irq
			break;
			
		case UIV_EP2OUT:
			writeb(0x1 << 2, acthcd->base + HCINxIRQ0); //clear hcep2in irq
			break;
			
		default:
			if ((irqvector >= UIV_HCOUT0ERR) && (irqvector <= UIV_HCOUT15ERR))
			{
				aotg_hcd_err_handle(acthcd, irqvector, (irqvector - UIV_HCOUT0ERR), 0);
			}
			else if ((irqvector >= UIV_HCIN0ERR) && (irqvector <= UIV_HCIN15ERR))
			{
				aotg_hcd_err_handle(acthcd, irqvector, (irqvector - UIV_HCIN0ERR), 1);
			}
			else
			{
				dev_err(acthcd->dev, "invalid irqvector: 0x%02X\n", (u8)irqvector);
				return IRQ_NONE;
			}
			break;
		}
	}
	
	/*clear all surprise interrupt*/
	aotg_clear_all_overflow_irq(acthcd);
	aotg_clear_all_shortpkt_irq(acthcd);
	aotg_clear_all_zeropkt_irq(acthcd);
	aotg_clear_all_hcoutdma_irq(acthcd);
	aotg_ring_irq_handler(acthcd);
	
	return IRQ_HANDLED;
}

enum hrtimer_restart aotg_hub_hotplug_timer(struct hrtimer *hrt)
{
	struct aotg_hcd *acthcd = container_of(hrt, struct aotg_hcd, hotplug_timer);
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);
	
	struct platform_device *pdev;
	
	unsigned long flags;
	int connect_changed = 0;

	if (acthcd->hcd_exiting != 0) {
		return HRTIMER_NORESTART;
	}
	
	disable_irq(acthcd->uhc_irq);
	
	spin_lock_irqsave(&acthcd->lock, flags);

	if (acthcd->put_aout_msg != 0)
	{
		pdev = to_platform_device(hcd->self.controller);
		
		
		acthcd->put_aout_msg = 0;
		spin_unlock_irqrestore(&acthcd->lock, flags);
		enable_irq(acthcd->uhc_irq);
		return HRTIMER_NORESTART;
	}

	if ((readb(acthcd->base + OTGSTATE) == AOTG_STATE_A_HOST) && (acthcd->discon_happened == 0)) {
		if (!acthcd->inserted) {
			acthcd->port |= (USB_PORT_STAT_C_CONNECTION << 16);
			/*set port status bit,and indicate the present of  a device */
			acthcd->port |= USB_PORT_STAT_CONNECTION;
			acthcd->rhstate = AOTG_RH_ATTACHED;
			acthcd->inserted = 1;
			connect_changed = 1;
		}
	} else {
		if (acthcd->inserted) {
			acthcd->port &= ~(USB_PORT_STAT_CONNECTION |
					  USB_PORT_STAT_ENABLE |
					  USB_PORT_STAT_LOW_SPEED |
					  USB_PORT_STAT_HIGH_SPEED | USB_PORT_STAT_SUSPEND);
			acthcd->port |= (USB_PORT_STAT_C_CONNECTION << 16);
			acthcd->rhstate = AOTG_RH_NOATTACHED;
			acthcd->inserted = 0;
			connect_changed = 1;
		}
		if (acthcd->discon_happened == 1) {
			acthcd->discon_happened = 0;

			if (readb(acthcd->base + OTGSTATE) == AOTG_STATE_A_HOST)
				hrtimer_start(&acthcd->hotplug_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}

	if (connect_changed)
	{
		if (HC_IS_SUSPENDED(hcd->state)) {
			usb_hcd_resume_root_hub(hcd);
		}
		usb_hcd_poll_rh_status(hcd);
	}

	if ((acthcd->inserted == 0) && (connect_changed == 1) &&
		(readb(acthcd->base + OTGSTATE) != AOTG_STATE_A_HOST)) {
		acthcd->put_aout_msg = 1;
		hrtimer_start(&acthcd->hotplug_timer, ktime_set(2, 200*NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
	
	acthcd->suspend_request_pend = 0;
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	
	enable_irq(acthcd->uhc_irq);
	
	return HRTIMER_NORESTART;
}


static inline int aotg_print_ep_timeout(
	struct aotg_hcd *acthcd, struct aotg_hcep *ep)
{
	int ret = 0;

	if (ep == NULL)
		return ret;

	if (ep->q != NULL) {
		if (ep->q->timeout == 0)
			return ret;

		if (time_after(jiffies, ep->q->timeout)) {
			ret = 1;
			dev_err(acthcd->dev,"ep->index:%x ep->mask:%x\n", ep->index, ep->mask);
			
			ep->q->timeout = jiffies + HZ;
		}
	}
	return ret;
}


void aotg_check_trb_timer(struct timer_list *t)
{
	unsigned long flags;
	struct aotg_hcep *ep;
	int i;
	
	struct aotg_hcd *acthcd = container_of(t, struct aotg_hcd, check_trb_timer);

	
	if (acthcd->hcd_exiting != 0) {
		return;
	}

	spin_lock_irqsave(&acthcd->lock, flags);
	if (acthcd->check_trb_mutex) {
		mod_timer(&acthcd->check_trb_timer, jiffies + msecs_to_jiffies(1));
		spin_unlock_irqrestore(&acthcd->lock, flags);
		return;
	}

	acthcd->check_trb_mutex = 1;
	for (i = 1; i < MAX_EP_NUM; i++) {
		ep = acthcd->hcep_pool.inep[i];
		if (ep && (ep->ring) && (ep->ring->type == PIPE_BULK))
				handle_ring_dma_tx(acthcd, i);
	}

	for (i = 1; i < MAX_EP_NUM; i++) {
		ep = acthcd->hcep_pool.outep[i];
		if (ep && (ep->ring) && (ep->ring->type == PIPE_BULK))
			handle_ring_dma_tx(acthcd, i | AOTG_DMA_OUT_PREFIX);
	}

	mod_timer(&acthcd->check_trb_timer, jiffies + msecs_to_jiffies(3));

	acthcd->check_trb_mutex = 0;
	spin_unlock_irqrestore(&acthcd->lock, flags);
	return;
}

void aotg_hub_trans_wait_timer(struct timer_list *t)
{
	unsigned long flags;
	struct aotg_hcep *ep;
	int i, ret;
	
	
	
	struct aotg_hcd *acthcd = container_of(t, struct aotg_hcd, trans_wait_timer);

	
	if (acthcd->hcd_exiting != 0) {
		return;
	}

	disable_irq(acthcd->uhc_irq);
	spin_lock_irqsave(&acthcd->lock, flags);

	ep = acthcd->active_ep0;
	ret = aotg_print_ep_timeout(acthcd, ep);

	for (i = 1; i < MAX_EP_NUM; i++) {
		ep = acthcd->hcep_pool.inep[i];
		ret |= aotg_print_ep_timeout(acthcd, ep);
	}
	for (i = 1; i < MAX_EP_NUM; i++) {
		ep = acthcd->hcep_pool.outep[i];
		if (ep == NULL)
			continue;

		ret |= aotg_print_ep_timeout(acthcd, ep);

		if (ep->fifo_busy) {
			if ((ep->fifo_busy > 80) && (ep->fifo_busy % 80 == 0))
				dev_info(acthcd->dev,"ep->fifo_busy:%d\n", ep->fifo_busy);

			if (ret == 0) {
				tasklet_hi_schedule(&acthcd->urb_tasklet);
				break;
			}
		}
	}

	if (ret != 0)
		tasklet_hi_schedule(&acthcd->urb_tasklet);

	mod_timer(&acthcd->trans_wait_timer, jiffies + msecs_to_jiffies(500));

	spin_unlock_irqrestore(&acthcd->lock, flags);
	enable_irq(acthcd->uhc_irq);
	return;
}

static inline int start_transfer(struct aotg_hcd *acthcd,
	struct aotg_queue *q, struct aotg_hcep *ep)
{
	struct urb *urb = q->urb;
	int retval = 0;

	ep->urb_enque_cnt++;
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




static int aotg_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mflags)
{
	switch(usb_pipetype(urb->pipe))
	{
	case PIPE_CONTROL:
		if (urb->transfer_buffer_length > (16 * 1024)) {
			return -EMSGSIZE;
		}
		return aotg_hcep_ctrl_submit(hcd, urb, mflags);
		
	case PIPE_BULK:
	case PIPE_ISOCHRONOUS:
		return aotg_hcep_xfer_submit(hcd, urb, mflags);
		
	/* case PIPE_INTERRUPT: */
	default:
		return aotg_hcep_intr_submit(hcd, urb, mflags);
	}
}

static int aotg_hub_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	struct aotg_queue *q = NULL, *next, *tmp;
	struct aotg_ring *ring;
	struct aotg_td *td, *next_td;
	unsigned long flags;
	int retval = 0;

	if (acthcd == NULL) {
		return -EIO;
	}
	
	spin_lock_irqsave(&acthcd->lock, flags);

	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	
	if (retval)
	{
		if (ep->type == PIPE_CONTROL)
		{
			dev_info(acthcd->dev, 
			         "%s: unable to unlink urb, ep %d in/out, ptr 0x%llx\n", 
			         __func__, ep->epnum, (u64)(dma_addr_t) ep->hep);
		}
		else
		{
			dev_info(acthcd->dev,
			         "%s: unable to unlink urb, ep %d %s, ptr 0x%llx\n", 
			         __func__, ep->epnum, ep->is_out ? "out" : "in",
			         (u64)(dma_addr_t) ep->hep);
		}
		
		spin_unlock_irqrestore(&acthcd->lock, flags);
		return retval;
	}

	ep = (struct aotg_hcep *)urb->ep->hcpriv;
	
	if (ep == NULL) {
		retval = -EINVAL;
		goto dequeue_out;
	}

	if (!usb_pipecontrol(urb->pipe))
	{
		ep->urb_unlinked_cnt++;
		ring = ep->ring;

		if (usb_pipeint(urb->pipe)) {
			list_for_each_entry_safe(td, next_td, &ep->enring_td_list, enring_list) {
				if (urb == td->urb) {
					retval = aotg_ring_dequeue_intr_td(acthcd, ep, ring, td);
					goto de_bulk;
				}
			}
			dev_err(acthcd->dev,"%s, intr dequeue err\n", __func__);
		}

		list_for_each_entry_safe(td, next_td, &ep->queue_td_list, queue_list) {
			if (urb == td->urb) {
				retval = aotg_ring_dequeue_td(acthcd, ring, td, TD_IN_QUEUE);
				goto de_bulk;
			}
		}

		list_for_each_entry_safe(td, next_td, &ep->enring_td_list, enring_list) {
			mb();
			if (urb == td->urb) {
				retval = aotg_ring_dequeue_td(acthcd, ring, td, TD_IN_RING);
				ep->urb_stop_stran_cnt++;
				goto de_bulk;
			}
		}

		retval = -EINVAL;
		goto dequeue_out;

de_bulk:
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock(&acthcd->lock);
		usb_hcd_giveback_urb(hcd, urb, status);
		spin_lock(&acthcd->lock);

		spin_unlock_irqrestore(&acthcd->lock, flags);
		return retval;
	}

	q = ep->q;

	/* ep->mask currently equal to q->dma_no. */
	if (q && (q->urb == urb))
	{
		writeb(EP0CS_HCSET, acthcd->base + EP0CS);

		/* maybe finished in tasklet_finish_request. */
		if (!list_empty(&q->finished_list)) {
			if (q->finished_list.next != LIST_POISON1)
				list_del(&q->finished_list);
		}

		if (q->is_xfer_start) {
			ep->urb_stop_stran_cnt++;
			q->is_xfer_start = 0;
		}
	} else {
		q = NULL;
		list_for_each_entry_safe(tmp, next, &acthcd->hcd_enqueue_list, enqueue_list) {
			if (tmp->urb == urb) {
				list_del(&tmp->enqueue_list);
				q = tmp;
				ep = q->ep;
				break;
			}
		}
	}

	if (likely(q)) {
		q->status = status;
		list_add_tail(&q->dequeue_list, &acthcd->hcd_dequeue_list);
		spin_unlock_irqrestore(&acthcd->lock, flags);
		tasklet_schedule(&acthcd->urb_tasklet);
		return retval;
	}
	else {
		dev_err(acthcd->dev,"dequeue's urb not find in enqueue_list!\n");
	}

dequeue_out:
	spin_unlock_irqrestore(&acthcd->lock, flags);
	return retval;
}

void urb_tasklet_func(unsigned long data)
{
	struct aotg_hcd *acthcd = (struct aotg_hcd *)data;
	struct aotg_queue *q, *next;
	struct aotg_hcep *ep;
	struct urb *urb;
	struct aotg_ring *ring;
	struct aotg_td *td;
	unsigned long flags;
	int status;
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);
	int cnt = 0;

	do {
		status = (int)spin_is_locked(&acthcd->tasklet_lock);
		
		if (status) {
			acthcd->tasklet_retry = 1;
			dev_info(acthcd->dev,"locked, urb retry later!\n");
			return;
		}
		cnt++;
		/* sometimes tasklet_lock is unlocked, but spin_trylock still will be failed,
		 * maybe caused by the instruction of strexeq in spin_trylock,it will return failed
		 * if other cpu is accessing the nearby address of &acthcd->tasklet_lock.
		 */
		status = spin_trylock(&acthcd->tasklet_lock);
		
		if ((!status) && (cnt > 10)) {
			acthcd->tasklet_retry = 1;
			dev_info(acthcd->dev,"urb retry later!\n");
			return;
		}
	} while (status == 0);

	/*disable_irq_nosync(acthcd->uhc_irq);*/
	disable_irq(acthcd->uhc_irq);
	spin_lock_irqsave(&acthcd->lock, flags);

	for (cnt = 1; cnt < MAX_EP_NUM; cnt++) {
		ep = acthcd->hcep_pool.inep[cnt];
		if (ep && (ep->type == PIPE_INTERRUPT)) {
			ring = ep->ring;
			if (ring->ring_stopped) {
				td = list_first_entry_or_null(&ep->enring_td_list, struct aotg_td, enring_list);
				if (!td)
					continue;
				urb = td->urb;
				if (!urb)
					continue;
				intr_finish_td(acthcd, ring, td);
			}
		}
	}
	/* do dequeue task. */
DO_DEQUEUE_TASK:
	urb = NULL;
	list_for_each_entry_safe(q, next, &acthcd->hcd_dequeue_list, dequeue_list) {
		if (q->status < 0) {
			urb = q->urb;
			ep = q->ep;
			if (ep) {
				ep->urb_unlinked_cnt++;
				/*ep->q = NULL;*/
			}
			list_del(&q->dequeue_list);
			status = q->status;
			tasklet_finish_request(acthcd, q, status);
			hcd = bus_to_hcd(urb->dev->bus);
			break;
		}
	}
	if (urb != NULL) {
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock_irqrestore(&acthcd->lock, flags);
		/* in usb_hcd_giveback_urb, complete function may call new urb_enqueue. */
		usb_hcd_giveback_urb(hcd, urb, status);
		spin_lock_irqsave(&acthcd->lock, flags);
		goto DO_DEQUEUE_TASK;
	}

	/* do finished task. */
DO_FINISH_TASK:
	urb = NULL;
	list_for_each_entry_safe(q, next, &acthcd->hcd_finished_list, finished_list) {
		if (q->finished_list.next != LIST_POISON1) {
			list_del(&q->finished_list);
		} else {
			break;
		}
		status = q->status;
		tasklet_finish_request(acthcd, q, status);

		hcd = aotg_to_hcd(acthcd);
		urb = q->urb;
		ep = q->ep;
		if (urb != NULL)
			break;
	}
	if (urb != NULL) {
		usb_hcd_unlink_urb_from_ep(hcd, urb);

		spin_unlock_irqrestore(&acthcd->lock, flags);

		/* in usb_hcd_giveback_urb, complete function may call new urb_enqueue. */
		usb_hcd_giveback_urb(hcd, urb, status);

		spin_lock_irqsave(&acthcd->lock, flags);
		goto DO_FINISH_TASK;
	}

	/*DO_ENQUEUE_TASK:*/
	/* do enqueue task. */
	/* start transfer directly, don't care setup appearing in bulkout. */
	q = list_first_entry_or_null(&acthcd->hcd_enqueue_list, struct aotg_queue, enqueue_list);
	if (q && (q->urb)) {
		urb = q->urb;
		ep = q->ep;

		if ((acthcd->active_ep0 != NULL) && (acthcd->active_ep0->q != NULL)) {
			acthcd->ep0_block_cnt++;
			if ((acthcd->ep0_block_cnt % 5) == 0) {
				
				dev_info(acthcd->dev,"cnt:%d\n", acthcd->ep0_block_cnt);
				acthcd->ep0_block_cnt = 0;
				spin_unlock_irqrestore(&acthcd->lock, flags);
				enable_irq(acthcd->uhc_irq);
				spin_unlock(&acthcd->tasklet_lock);
				aotg_hub_urb_dequeue(hcd, acthcd->active_ep0->q->urb, -ETIMEDOUT);
				return;
			}
			
			goto exit;
		} else {
			acthcd->ep0_block_cnt = 0;
		}

		list_del(&q->enqueue_list);
		status = start_transfer(acthcd, q, ep);

		if (unlikely(status < 0)) {
			
			hcd = bus_to_hcd(urb->dev->bus);
			aotg_hcd_release_queue(acthcd, q);

			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&acthcd->lock, flags);
			usb_hcd_giveback_urb(hcd, urb, status);
			spin_lock_irqsave(&acthcd->lock, flags);
		}
	}

	if (acthcd->tasklet_retry != 0) {
		acthcd->tasklet_retry = 0;
		goto DO_DEQUEUE_TASK;
	}
exit:
	spin_unlock_irqrestore(&acthcd->lock, flags);
	enable_irq(acthcd->uhc_irq);
	spin_unlock(&acthcd->tasklet_lock);
	return;
}

static void aotg_hub_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	int i;
	int index;
	unsigned long flags;
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	
	
	struct aotg_hcep *ep = hep->hcpriv;

	if (!ep)
		return;

	if (in_irq())
		disable_irq_nosync(acthcd->uhc_irq);
	else
		disable_irq(acthcd->uhc_irq);

	spin_lock_irqsave(&acthcd->lock, flags);

	index = ep->index;
	if (index == 0) {
		acthcd->hcep_pool.ep0[ep->ep0_index] = NULL;
		if (acthcd->active_ep0 == ep)
			acthcd->active_ep0 = NULL;

		for (i = 0; i < MAX_EP_NUM; i++) {
			if (acthcd->hcep_pool.ep0[i] != NULL)
				break;
		}
		if (i == MAX_EP_NUM) {
			usb_clearbitsw(1, acthcd->base + HCOUTxIEN0);
			usb_clearbitsw(1, acthcd->base + HCINxIEN0);
			writew(1, acthcd->base + HCOUTxIRQ0);
			writew(1, acthcd->base + HCINxIRQ0);
		}
	} else {
		ep_disable(ep);
		if (ep->mask & USB_HCD_OUT_MASK)
			acthcd->hcep_pool.outep[index] = NULL;
		else
			acthcd->hcep_pool.inep[index] = NULL;
		release_fifo_addr(acthcd, ep->fifo_addr);
	}

	hep->hcpriv = NULL;
	
	if (ep->ring)
	{
		aotg_stop_ring(ep->ring);
	}
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	
	if (ep->type == PIPE_CONTROL)
	{
		dev_info(acthcd->dev, "%s: ep %d in/out, ptr 0x%llx\n", 
		         __func__, ep->epnum, (u64)(dma_addr_t) ep->hep);
	}
	else
	{
		dev_info(acthcd->dev, "%s: ep %d %s, ptr 0x%llx\n", 
		         __func__, ep->epnum, ep->is_out ? "out" : "in",
		         (u64)(dma_addr_t) ep->hep);
	}
	
	if (ep->ring)
	{
		
		if (ep->ring->type == PIPE_INTERRUPT) {
			aotg_intr_dma_buf_free(acthcd, ep->ring);
		}
		
		aotg_free_ring(acthcd, ep->ring);
	}
	
	enable_irq(acthcd->uhc_irq);
	
	kfree(ep);
	return;
}

static int aotg_hcd_get_frame(struct usb_hcd *hcd)
{
	struct timespec64 ts;
	
	ktime_get_real_ts64(&ts);
	
	return ts.tv_nsec / 1000000;
}

static int aotg_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct aotg_hcd *acthcd;
	unsigned long flags;
	int retval = 0;

	acthcd = hcd_to_aotg(hcd);
	local_irq_save(flags);
	if (!HC_IS_RUNNING(hcd->state))
		goto done;

	if ((acthcd->port & AOTG_PORT_C_MASK) != 0) {
		*buf = (1 << 1);
		retval = 1;
	}
done:
	local_irq_restore(flags);
	return retval;
}

static void port_power(struct aotg_hcd *acthcd, int is_on)
{
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);

	/* hub is inactive unless the port is powered */
	if (is_on) {
		hcd->self.controller->power.power_state = PMSG_ON;
		dev_dbg(acthcd->dev, "<USB> power on\n");
	} else {
		hcd->self.controller->power.power_state = PMSG_SUSPEND;
		dev_dbg(acthcd->dev, "<USB> power off\n");
	}
}

static inline void aotg_hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->wHubCharacteristics = (__force __u16)
		(__constant_cpu_to_le16(0x0001));
	desc->bNbrPorts = 1;
}

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void port_reset(struct aotg_hcd *acthcd)
{
	/*portrst needs at least 55ms */
	writeb(0x1<<6 | 0x1<<5, acthcd->base + HCPORTCTRL);
}

static int aotg_hub_control(
	struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex,
	char *buf, u16 wLength)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	unsigned long flags;
	int retval = 0;
	
	if (in_irq()) {
		disable_irq_nosync(acthcd->uhc_irq);
	}
	else {
		disable_irq(acthcd->uhc_irq);
	}
	
	spin_lock_irqsave(&acthcd->lock, flags);
	
	if (!HC_IS_RUNNING(hcd->state))
	{
		dev_err(acthcd->dev, "%s: hcd state is not HC_STATE_RUNNING\n",
		        __func__);
		spin_unlock_irqrestore(&acthcd->lock, flags);
		enable_irq(acthcd->uhc_irq);
		return -EPERM;
	}
	
	switch (typeReq)
	{
	case ClearHubFeature:
		break;
		
	case ClearPortFeature:
		
		if (wIndex != 1 || wLength != 0) {
			goto hub_error;
		}
		switch (wValue)
		{
		case USB_PORT_FEAT_ENABLE:
			acthcd->port &= ~(USB_PORT_STAT_ENABLE
					| USB_PORT_STAT_LOW_SPEED
					| USB_PORT_STAT_HIGH_SPEED);
			
			acthcd->rhstate = AOTG_RH_DISABLE;
			
			if (acthcd->port & USB_PORT_STAT_POWER) {
				port_power(acthcd, 0);
			}
			break;
			
		case USB_PORT_FEAT_SUSPEND:
			/*port_resume(acthcd);*/
			acthcd->port &= ~(1 << wValue);
			break;
			
		case USB_PORT_FEAT_POWER:
			acthcd->port = 0;
			acthcd->rhstate = AOTG_RH_POWEROFF;
			port_power(acthcd, 0);
			break;
			
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			acthcd->port &= ~(1 << wValue);
			break;
			
		default:
			goto hub_error;
		}
		
		break;
	case GetHubDescriptor:
		aotg_hub_descriptor((struct usb_hub_descriptor *)buf);
		break;
		
	case GetHubStatus:
		*(__le32 *)buf = __constant_cpu_to_le32(0);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto hub_error;
		*(__le32 *)buf = cpu_to_le32(acthcd->port);
		break;
		
	case SetHubFeature:
		goto hub_error;
		break;
		
	case SetPortFeature:
		
		switch (wValue)
		{
		case USB_PORT_FEAT_POWER:
			if (unlikely(acthcd->port & USB_PORT_STAT_POWER)) {
				break;
			}
			acthcd->port |= (1 << wValue);
			acthcd->rhstate = AOTG_RH_POWERED;
			port_power(acthcd, 1);
			break;
			
		case USB_PORT_FEAT_RESET:
			if (acthcd->hcd_exiting) {
				retval = -ENODEV;
				break;
			}
			
			dev_info(acthcd->dev, "%s: port reset\n", __func__);
			port_reset(acthcd);
			
			spin_unlock_irqrestore(&acthcd->lock, flags);
			msleep_range(55);
			spin_lock_irqsave(&acthcd->lock, flags);
			
			/* if it's already enabled, disable */
			acthcd->port &= ~(USB_PORT_STAT_ENABLE
				| USB_PORT_STAT_LOW_SPEED
				| USB_PORT_STAT_HIGH_SPEED);
			
			acthcd->port |= (1 << wValue);
			
			spin_unlock_irqrestore(&acthcd->lock, flags);
			msleep_range(2);
			spin_lock_irqsave(&acthcd->lock, flags);
			
			acthcd->rhstate = AOTG_RH_RESET;
			usb_setbitsb(USBIEN_URES, acthcd->base + USBIEN);
			usb_setbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
			break;
			
		case USB_PORT_FEAT_SUSPEND:
			/*acthcd->port |= USB_PORT_FEAT_SUSPEND;*/
			acthcd->port |= (1 << wValue);
			acthcd->rhstate = AOTG_RH_SUSPEND;
			/*port_suspend(acthcd);*/
			break;
			
		default:
			if (acthcd->port & USB_PORT_STAT_POWER)
				acthcd->port |= (1 << wValue);
		}
		break;
		
	default:
hub_error:
		retval = -EPIPE;
		dev_err(acthcd->dev, "%s: hub control error\n", __func__);
		break;
	}
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	enable_irq(acthcd->uhc_irq);
	return retval;
}



void aotg_hcd_exit(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep;
	int i;
	
	
	for (i = 0; i < MAX_EP_NUM; i++)
	{
		ep = acthcd->hcep_pool.ep0[i];
		if (ep)
		{
			kfree(ep);
		}
	}
	
		for (i = 1; i < MAX_EP_NUM; i++) {
			ep = acthcd->hcep_pool.inep[i];
			if (ep) {
				
				kfree(ep);
			}
		}
		for (i = 1; i < MAX_EP_NUM; i++) {
			ep = acthcd->hcep_pool.outep[i];
			if (ep) {
				
				kfree(ep);
		}
	}
}

static int caninos_map_urb(struct usb_hcd *hcd, struct urb *urb, gfp_t flags)
{
	if (usb_pipetype(urb->pipe) != PIPE_INTERRUPT) {
		return usb_hcd_map_urb_for_dma(hcd, urb, flags);
	}
	return 0;
}

static void caninos_unmap_urb(struct usb_hcd *hcd, struct urb *urb)
{
	if (usb_pipetype(urb->pipe) != PIPE_INTERRUPT) {
		usb_hcd_unmap_urb_for_dma(hcd, urb);
	}
}

void aotg_clear_all_overflow_irq(struct aotg_hcd *acthcd)
{
	u32 irq_pend = readl(acthcd->base + HCDMAxOVERFLOWIRQ);
	if (irq_pend) {
		writel(irq_pend, acthcd->base + HCDMAxOVERFLOWIRQ);
	}
}

void aotg_clear_all_shortpkt_irq(struct aotg_hcd *acthcd)
{
	u16 irq_pend = readw(acthcd->base + HCINxSHORTPCKIRQ0);
	if (irq_pend) {
		writew(irq_pend, acthcd->base + HCINxSHORTPCKIRQ0);
	}
}

void aotg_clear_all_zeropkt_irq(struct aotg_hcd *acthcd)
{
	u16 irq_pend = readw(acthcd->base + HCINxZEROPCKIEN0);
	if (irq_pend) {
		writew(irq_pend, acthcd->base + HCINxZEROPCKIEN0);
	}
}

void aotg_clear_all_hcoutdma_irq(struct aotg_hcd *acthcd)
{
	u16 irq_pend = readw(acthcd->base + HCOUTxDMAIRQ0);
	if (irq_pend) {
		writew(irq_pend, acthcd->base + HCOUTxDMAIRQ0);
	}
}

void aotg_enable_irq(struct aotg_hcd *acthcd)
{
	writeb(USBEIRQ_USBIEN, acthcd->base + USBEIRQ);
	usb_setbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	usb_setbitsb(0x1<<2, acthcd->base + OTGIEN);
	usb_setbitsb(OTGCTRL_BUSREQ, acthcd->base + OTGCTRL);
}

void aotg_disable_irq(struct aotg_hcd *acthcd)
{
	writeb(USBEIRQ_USBIEN, acthcd->base + USBEIRQ);
	usb_clearbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	usb_clearbitsb(0x1<<2, acthcd->base + OTGIEN);
	usb_clearbitsb(OTGCTRL_BUSREQ, acthcd->base + OTGCTRL);
}

/// -------------------------------------------------------------------------

void aotg_hcep_pool_init(struct aotg_hcd *acthcd)
{
	struct aotg_hcep_pool *pool = &acthcd->hcep_pool;
	int i;
	
	for (i = 0; i < MAX_EP_NUM; i++)
	{
		pool->ep0[i] = NULL;
		pool->inep[i] = NULL;
		pool->outep[i] = NULL;
	}
}

void aotg_powergate_off(struct aotg_hcd *acthcd)
{
	//clk_disable_unprepare(acthcd->clk_usbh_cce);
	clk_disable_unprepare(acthcd->clk_usbh_pllen);
	clk_disable_unprepare(acthcd->clk_usbh_phy);
	
	pm_runtime_put_sync(acthcd->dev);
	pm_runtime_disable(acthcd->dev);
}

void aotg_powergate_on(struct aotg_hcd *acthcd)
{
	pm_runtime_enable(acthcd->dev);
	pm_runtime_get_sync(acthcd->dev);

	clk_prepare_enable(acthcd->clk_usbh_phy);
	clk_prepare_enable(acthcd->clk_usbh_pllen);
	//must not enable!
	//clk_prepare_enable(acthcd->clk_usbh_cce);
}

static int aotg_wait_reset(struct aotg_hcd *acthcd)
{
	int i = 0;
	
	reset_control_assert(acthcd->rst);
	udelay(1);
	reset_control_deassert(acthcd->rst);
	
	while (readb(acthcd->base + USBERESET) & USBERES_USBRESET)
	{
		usleep_range(5, 10);
		i++;
		
		if (i >= 1000) {
			break;
		}
	}
	
	if (readb(acthcd->base + USBERESET) & USBERES_USBRESET) {
		return -EBUSY;
	}
	return 0;
}

static void aotg_DD_set_phy(void __iomem *base, u8 reg, u8 value)
{
	u8 addrlow = reg & 0x0f;
	u8 addrhigh = (reg >> 4) & 0x0f;
	
	/*write vstatus: */
	writeb(value, base + VDSTATUS);
	mb();
	
	/*write vcontrol: */
	writeb(addrlow | 0x10, base + VDCTRL);
	udelay(1); /*the vload period should > 33.3ns*/
	writeb(addrlow & 0x0f, base + VDCTRL);
	udelay(1);
	mb();
	writeb(addrlow | 0x10, base + VDCTRL);
	udelay(1);
	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(1);
	writeb(addrhigh & 0x0f, base + VDCTRL);
	udelay(1);
	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(1);
}

static void aotg_set_hcd_phy(struct aotg_hcd *acthcd)
{
	if (acthcd->model == CANINOS_HW_MODEL_K7)
	{
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe1, 0xcf);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe6, 0xcc);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe2, 0x02);
		aotg_DD_set_phy(acthcd->base, 0xe2, 0x16);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe7, 0xa1);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe0, 0x21);
		aotg_DD_set_phy(acthcd->base, 0xe0, 0x25);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe4, 0xa6);
		aotg_DD_set_phy(acthcd->base, 0xf0, 0xfc);
	}
	else
	{
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe0, 0x35);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe1, 0xcf);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe6, 0xcb);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe7, 0x91);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0xbb);
		aotg_DD_set_phy(acthcd->base, 0xe0, 0x31);
		aotg_DD_set_phy(acthcd->base, 0xf4, 0x9b);
		aotg_DD_set_phy(acthcd->base, 0xe4, 0xa6);
		aotg_DD_set_phy(acthcd->base, 0xf0, 0xfc);
	}
}

static void aotg_hcd_stop(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	unsigned long flags;
	
	spin_lock_irqsave(&acthcd->lock, flags);
	
	acthcd->hcd_exiting = 1;
	aotg_disable_irq(acthcd);
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	
	tasklet_kill(&acthcd->urb_tasklet);
	
	del_timer_sync(&acthcd->trans_wait_timer);
	del_timer_sync(&acthcd->check_trb_timer);
	
	hrtimer_cancel(&acthcd->hotplug_timer);
	
	acthcd->port = 0;
	acthcd->rhstate = AOTG_RH_POWEROFF;
	
	aotg_hcd_release_queue(acthcd, NULL);
	aotg_hcd_exit(hcd);
}

static int aotg_hcd_start(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	unsigned long flags;
	
	spin_lock_irqsave(&acthcd->lock, flags);
	
	hcd->uses_new_polling = 1;
	writeb(readb(acthcd->base + USBEIRQ), acthcd->base + USBEIRQ);
	aotg_enable_irq(acthcd);
	
	spin_unlock_irqrestore(&acthcd->lock, flags);
	
	return 0;
}

static void aotg_hcd_shutdown(struct usb_hcd *hcd)
{
	 caninos_usb_hcd_remove(hcd);
}

static const struct hc_driver caninos_hc_driver = {
	
	.description = DRIVER_NAME,
	.product_desc = DRIVER_DESC,
	.hcd_priv_size = sizeof(struct aotg_hcd),
	
	/* generic hardware linkage */
	.irq = aotg_hub_irq,
	.flags = HCD_USB2 | HCD_MEMORY | HCD_DMA,
	
	/* basic lifecycle operations */
	.start = aotg_hcd_start,
	.stop = aotg_hcd_stop,
	
	/* managing i/o requests and associated device resources */
	.urb_enqueue = aotg_urb_enqueue,
	.urb_dequeue = aotg_hub_urb_dequeue,
	.map_urb_for_dma = caninos_map_urb,
	.unmap_urb_for_dma = caninos_unmap_urb,
	.endpoint_disable = aotg_hub_endpoint_disable,
	
	/* periodic schedule support */
	.get_frame_number = aotg_hcd_get_frame,
	
	/* root hub support */
	.hub_status_data = aotg_hub_status_data,
	.hub_control = aotg_hub_control,
	.bus_suspend = NULL,
	.bus_resume = NULL,
	
	.shutdown = aotg_hcd_shutdown,
};

struct usb_hcd *caninos_usb_create_hcd(struct device *dev)
{
	return usb_create_hcd(&caninos_hc_driver, dev, dev_name(dev));
}

int caninos_usb_add_hcd(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	int retval;
	u8 val8;
	
	aotg_powergate_on(acthcd);
	
	retval = aotg_wait_reset(acthcd);
	
	if (retval)
	{
		aotg_powergate_off(acthcd);
		dev_err(acthcd->dev, "unable to reset the hw\n");
		return retval;
	}
	
	writel(0x1, acthcd->base + HCDMABCKDOOR);
	
	if (acthcd->model == CANINOS_HW_MODEL_K7) {
		usb_writel(0x37000000 | (0x3 << 4), acthcd->usbecs);
	}
	else {
		usb_writel(0x37000000 | (0x10 << 13) | (0xb << 4), acthcd->usbecs);
	}
	
	usleep_range(100, 120);
	aotg_set_hcd_phy(acthcd);
	
	writeb(0x0, acthcd->base + TA_BCON_COUNT);
	usb_writeb(0xff, acthcd->base + TAAIDLBDIS);
	usb_writeb(0xff, acthcd->base + TAWAITBCON);
	usb_writeb(0x28, acthcd->base + TBVBUSDISPLS);
	usb_setb(1 << 7, acthcd->base + TAWAITBCON);
	usb_writew(0x1000, acthcd->base + VBUSDBCTIMERL);
	
	val8 = readb(acthcd->base + BKDOOR);
	val8 &= ~(1 << 7);
	writeb(val8, acthcd->base + BKDOOR);
	
	hcd->rsrc_start = acthcd->rsrc_start;
	hcd->rsrc_len = acthcd->rsrc_len;
	hcd->regs = acthcd->base;
	hcd->self.sg_tablesize = 32;
	hcd->self.uses_pio_for_control = 1;
	hcd->has_tt = 1;
	
	retval = usb_add_hcd(hcd, acthcd->uhc_irq, 0);
	
	if (retval)
	{
		aotg_powergate_off(acthcd);
		dev_err(acthcd->dev, "unable to reset the hw\n");
		return retval;
	}
	
	device_wakeup_enable(hcd->self.controller);
	
	return 0;
}

void caninos_usb_hcd_remove(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	usb_remove_hcd(hcd);
	aotg_powergate_off(acthcd);
	usb_put_hcd(hcd);
}

