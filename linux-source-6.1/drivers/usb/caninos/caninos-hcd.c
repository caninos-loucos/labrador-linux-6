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


	aotg_hcd_queue_free(acthcd, q);

	ep->urb_endque_cnt++;
	return;
}

static void aotg_hcd_err_handle(
	struct aotg_hcd *acthcd, u32 irqvector, int ep_num, int is_in)
{
	struct urb *urb;
	struct aotg_hcep *ep = NULL;
	struct aotg_ring *ring = NULL;
	struct aotg_td *td = NULL;
	int status = -EOVERFLOW;
	u8 err_val = 0;
	u8 err_type = 0;
	u8 reset = 0;
	struct usb_hcd *hcd = aotg_to_hcd(acthcd);
	
	if (ep_num == 0)
	{
		handle_hcep0_error(acthcd, is_in);
		return;
	}
	
	if (is_in)
		writew(1 << ep_num, acthcd->base + HCINxERRIRQ0);
	else
		writew(1 << ep_num, acthcd->base + HCOUTxERRIRQ0);
	
	{
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
	
	if (acthcd->hcd_exiting != 0) {
		dev_err(acthcd->dev, "hcd_exiting:%d\n", acthcd->hcd_exiting);
		status = -ENODEV;
	}
	
	{
		if ((status != -EPIPE) && (status != -ENODEV))
		{
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

		aotg_stop_ring(ring);
		urb = td->urb;
		
		if (ep->type == PIPE_INTERRUPT)
			dequeue_intr_td(ring, td);
		else
			dequeue_td(ring, td, TD_IN_FINISH);

		if (urb) {
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			usb_hcd_giveback_urb(hcd, urb, status);
		}
		else {
			dev_info(acthcd->dev,"urb not exist!\n");
		}

		/*
		 * after, need to rewrite port_num, dev_addr when using hub ?
		 */
		
		dev_info(acthcd->dev, "%s ep %d error [0x%02X] error type [0x%02X], reset it...\n",
			is_in ? "HC IN" : "HC OUT", ep->index, err_val, (err_val>>2)&0x7);
	}
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

static void aotg_hub_irq(struct usb_hcd *hcd, u32 irqvector)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	
	switch (irqvector)
	{
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
		break;
	}
	
	/*clear all surprise interrupt*/
	aotg_clear_all_overflow_irq(acthcd);
	aotg_clear_all_shortpkt_irq(acthcd);
	aotg_clear_all_zeropkt_irq(acthcd);
	aotg_clear_all_hcoutdma_irq(acthcd);
	
	aotg_ring_irq_handler(acthcd);
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

		if (time_after(jiffies, ep->q->timeout))
		{
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

static int aotg_hub_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	struct aotg_hcep *ep = NULL;
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
	
	if (urb->ep != NULL) {
		ep = (struct aotg_hcep *)urb->ep->hcpriv;
	}
	
	if (ep == NULL) {
		retval = -EINVAL;
		goto dequeue_out;
	}
	
	if (!usb_pipecontrol(urb->pipe))
	{
		ep->urb_unlinked_cnt++;
		ring = ep->ring;

		if (usb_pipeint(urb->pipe))
		{
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
	}
	else
	{
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
	
	if (q && (q->urb))
	{
		urb = q->urb;
		ep = q->ep;
		
		if ((acthcd->active_ep0 != NULL) && (acthcd->active_ep0->q != NULL))
		{
			acthcd->ep0_block_cnt++;
			
			if ((acthcd->ep0_block_cnt % 5) == 0) {
				
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
		status = start_setup_transfer(acthcd, q, ep);

		if (unlikely(status < 0)) {
			
			hcd = bus_to_hcd(urb->dev->bus);
			aotg_hcd_queue_free(acthcd, q);

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

// -------------------------------------------------------------------------- //

static int aotg_hcd_urb_enqueue(struct usb_hcd *hcd,
	struct urb *urb, gfp_t mem_flags)
{
	switch(usb_pipetype(urb->pipe))
	{
	case PIPE_CONTROL:
		if (urb->transfer_buffer_length > (16 * 1024)) {
			return -EMSGSIZE;
		}
		return aotg_hcep_ctrl_submit(hcd, urb, mem_flags);
		
	case PIPE_BULK:
	case PIPE_ISOCHRONOUS:
		return aotg_hcep_xfer_submit(hcd, urb, mem_flags);
		
	/* case PIPE_INTERRUPT: */
	default:
		return aotg_hcep_intr_submit(hcd, urb, mem_flags);
	}
}

static int aotg_hcd_hub_control(struct usb_hcd *hcd,
	u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	return aotg_hotplug_control(&acthcd->hotplug,
		typeReq, wValue, wIndex, buf, wLength);
}

static int aotg_hcd_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	return aotg_hotplug_status_data(&acthcd->hotplug, buf);
}

static int aotg_hcd_map_urb(struct usb_hcd *hcd,
	struct urb *urb, gfp_t mem_flags)
{
	if (usb_pipetype(urb->pipe) != PIPE_INTERRUPT) {
		return usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	}
	return 0;
}

static void aotg_hcd_unmap_urb(struct usb_hcd *hcd, struct urb *urb)
{
	if (usb_pipetype(urb->pipe) != PIPE_INTERRUPT) {
		usb_hcd_unmap_urb_for_dma(hcd, urb);
	}
}

static int aotg_hcd_get_frame(struct usb_hcd *hcd)
{
	struct timespec64 ts;
	ktime_get_real_ts64(&ts);
	return ts.tv_nsec / 1000000;
}

static int aotg_hcd_reset(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	aotg_init_hotplug(&acthcd->hotplug, acthcd);
	return 0;
}

static int aotg_hcd_start(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	aotg_start_hotplug(&acthcd->hotplug);
	return 0;
}

static void aotg_hcd_stop(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	aotg_stop_hotplug(&acthcd->hotplug);
	
	acthcd->hcd_exiting = 1;
	
	tasklet_kill(&acthcd->urb_tasklet);
	
	del_timer_sync(&acthcd->trans_wait_timer);
	del_timer_sync(&acthcd->check_trb_timer);
	
	aotg_hcd_queue_free(acthcd, NULL);
	aotg_hcd_exit(hcd);
}

static void aotg_hcd_shutdown(struct usb_hcd *hcd)
{
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

static irqreturn_t aotg_hcd_irq(struct usb_hcd *hcd)
{
	struct aotg_hcd *acthcd = hcd_to_aotg(hcd);
	u32 ivector = aotg_hotplug_irq_handler(&acthcd->hotplug);
	aotg_hub_irq(hcd, ivector);
	return IRQ_HANDLED;
}

static const struct hc_driver caninos_hc_driver = {
	
	.description = DRIVER_NAME,
	.product_desc = DRIVER_DESC,
	.hcd_priv_size = sizeof(struct aotg_hcd),
	
	/* generic hardware linkage */
	.irq = aotg_hcd_irq,
	.flags = HCD_USB2 | HCD_MEMORY | HCD_DMA,
	
	/* basic lifecycle operations */
	.reset = aotg_hcd_reset, //ok
	.start = aotg_hcd_start, //ok
	.stop = aotg_hcd_stop, //ok
	
	/* managing i/o requests and associated device resources */
	.urb_enqueue = aotg_hcd_urb_enqueue, //ok
	.urb_dequeue = aotg_hub_urb_dequeue,
	.map_urb_for_dma = aotg_hcd_map_urb, //ok
	.unmap_urb_for_dma = aotg_hcd_unmap_urb, //ok
	.endpoint_disable = aotg_hub_endpoint_disable,
	
	/* periodic schedule support */
	.get_frame_number = aotg_hcd_get_frame, //ok
	
	/* root hub support */
	.hub_status_data = aotg_hcd_hub_status_data, //ok
	.hub_control = aotg_hcd_hub_control, //ok
	
	.shutdown = aotg_hcd_shutdown, //ok
};

struct usb_hcd *caninos_usb_create_hcd(struct device *dev) {
	return usb_create_hcd(&caninos_hc_driver, dev, dev_name(dev));
}

