#include "aotg_hcd.h"

#define MSG_CANCEL    BIT(0)
#define MSG_ATTACHED  BIT(1)
#define MSG_REMOVED   BIT(2)
#define MSG_CS_HFMODE BIT(3)
#define MSG_CS_FSMODE BIT(4)
#define MSG_CS_LSMODE BIT(5)
#define MSG_REQ_RESET BIT(6)

#define AOTG_PORT_RESET      BIT(0)
#define AOTG_PORT_HIGH_SPEED BIT(1)
#define AOTG_PORT_FULL_SPEED BIT(2)
#define AOTG_PORT_LOW_SPEED  BIT(3)

static void hotplug_worker(struct work_struct *t);

static void aotg_disable_irq(struct aotg_hotplug *hotplug);

static void aotg_enable_irq(struct aotg_hotplug *hotplug);

static int aotg_port_reset(struct aotg_hotplug *hotplug);

static int aotg_hw_powerup(struct aotg_hotplug *hotplug);

static void aotg_hw_shutdown(struct aotg_hotplug *hotplug);

void aotg_init_hotplug(struct aotg_hotplug *hotplug, struct aotg_hcd *acthcd)
{
	pm_runtime_enable(acthcd->dev);
	pm_runtime_get_sync(acthcd->dev);
	
	clk_prepare_enable(acthcd->clk_usbh_phy);
	clk_prepare_enable(acthcd->clk_usbh_pllen);
	reset_control_assert(acthcd->rst);
	
	hotplug->acthcd = acthcd;
	hotplug->prev_prstate = 0;
	hotplug->wPortChange = 0;
	
	spin_lock_init(&hotplug->irq_lock);
	mutex_init(&hotplug->lock);
	init_waitqueue_head(&hotplug->event);
	INIT_WORK(&hotplug->work, hotplug_worker);
	
	atomic_set(&hotplug->mailbox, 0);
	atomic_set(&hotplug->running, 0);
	atomic_set(&hotplug->prstate, 0);
	atomic_set(&hotplug->rhstate, AOTG_RH_POWEROFF);
	
	smp_mb();
}

void aotg_stop_hotplug(struct aotg_hotplug *hotplug)
{
	mutex_lock(&hotplug->lock);
	
	if (atomic_cmpxchg_acquire(&hotplug->running, 1, 0) == 1)
	{
		atomic_fetch_or_release(MSG_CANCEL, &hotplug->mailbox);
		wake_up(&hotplug->event);
		cancel_work_sync(&hotplug->work);
	}
	
	mutex_unlock(&hotplug->lock);
}

void aotg_start_hotplug(struct aotg_hotplug *hotplug)
{
	mutex_lock(&hotplug->lock);
	
	if (atomic_cmpxchg_release(&hotplug->running, 0, 1) == 0)
	{
		atomic_set_release(&hotplug->mailbox, 0);
		schedule_work(&hotplug->work);
	}
	
	mutex_unlock(&hotplug->lock);
}

static int hotplug_procedure(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	int retval, msg;
	
	atomic_set_release(&hotplug->rhstate, AOTG_RH_NOATTACHED);
	
	while (1)
	{
		msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL)
		{
			aotg_hw_shutdown(hotplug);
			return -ECANCELED;
		}
		
		if ((readb(acthcd->base + OTGSTATE) == A_HOST) || 
			(msg & MSG_ATTACHED))
		{
			atomic_set_release(&hotplug->rhstate, AOTG_RH_ATTACHED);
			dev_info(acthcd->dev, "%s: device connected\n", __func__);
			break;
		}
		
		wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0,
			msecs_to_jiffies(1200));
	}
	
	dev_info(acthcd->dev, "%s: port reset\n", __func__);
	retval = aotg_port_reset(hotplug);
	
	if (retval)
	{
		if (retval != -ECANCELED) {
			dev_err(acthcd->dev, "%s: port reset failed\n", __func__);
		}
		aotg_hw_shutdown(hotplug);
		return retval;
	}
	
	while (1)
	{
		msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL)
		{
			aotg_hw_shutdown(hotplug);
			return -ECANCELED;
		}
		if (msg & MSG_REMOVED)
		{
			spin_lock_irqsave(&hotplug->irq_lock, flags);
			atomic_set(&hotplug->prstate, 0);
			atomic_set_release(&hotplug->rhstate, AOTG_RH_NOATTACHED);
			spin_unlock_irqrestore(&hotplug->irq_lock, flags);
			break;
		}
		if (msg & MSG_REQ_RESET)
		{
			dev_info(acthcd->dev,
				"%s: port reset requested\n", __func__);
			retval = aotg_port_reset(hotplug);
			
			if (retval)
			{
				if (retval != -ECANCELED) {
					dev_err(acthcd->dev,
						"%s: port reset failed\n", __func__);
				}
				aotg_hw_shutdown(hotplug);
				return retval;
			}
		}
		
		wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0,
			msecs_to_jiffies(600));
	}
	
	dev_info(acthcd->dev, "%s: device disconnected\n", __func__);
	return 0;
}

static void hotplug_worker(struct work_struct *t)
{
	struct aotg_hotplug *hotplug = container_of(t, typeof(*hotplug), work);
	struct aotg_hcd *acthcd = hotplug->acthcd;
	int retval = -EINVAL;
	
	dev_info(acthcd->dev, "%s: started\n", __func__);
	
	do {
		if (retval)
		{
			dev_info(acthcd->dev, "%s: hub powerup\n", __func__);
			retval = aotg_hw_powerup(hotplug);
			
			if (retval)
			{
				if (retval != -ECANCELED) {
					dev_err(acthcd->dev,
						"%s: hub powerup failed\n", __func__);
				}
				continue;
			}
			
		}
		retval = hotplug_procedure(hotplug);
		
	} while (retval != -ECANCELED);
	
	dev_info(acthcd->dev, "%s: finished\n", __func__);
}

static int hub_control_port_reset(struct aotg_hotplug *hotplug)
{
	atomic_fetch_or_release(MSG_REQ_RESET, &hotplug->mailbox);
	wake_up(&hotplug->event);
	return 0;
}

static bool status_changed(struct aotg_hotplug *hotplug, int port)
{
	int prev_port = hotplug->prev_prstate;
	hotplug->prev_prstate = port;
	
	if ((port != 0) != (prev_port != 0)) {
		hotplug->wPortChange |= USB_PORT_STAT_C_CONNECTION;
	}
	if ((port & AOTG_PORT_RESET) != (prev_port & AOTG_PORT_RESET)) {
		hotplug->wPortChange |= USB_PORT_STAT_C_RESET;
	}
	
	return (hotplug->wPortChange != 0);
}

int aotg_hotplug_status_data(struct aotg_hotplug *hotplug, char *buf)
{
	int port = atomic_read_acquire(&hotplug->prstate);
	*buf = 0;
	
	if (status_changed(hotplug, port))
	{
		*buf = (char)BIT(1);
		return 1;
	}
	return 0;
}

int aotg_hotplug_control(struct aotg_hotplug *hotplug,
	u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct usb_hub_descriptor *hub_desc;
	u16 wPortChange, wPortStatus;
	int port;
	
	switch (typeReq)
	{
	case GetHubStatus:
		*(__le32 *)buf = cpu_to_le32(0);
		break;
		
	case GetHubDescriptor:
		hub_desc = (struct usb_hub_descriptor *)buf;
		hub_desc->bDescLength = 9;
		hub_desc->bDescriptorType = USB_DT_HUB;
		hub_desc->bNbrPorts = 1;
		hub_desc->wHubCharacteristics = 0;
		hub_desc->bPwrOn2PwrGood = 1;
		hub_desc->bHubContrCurrent = 0;
		hub_desc->u.hs.DeviceRemovable[0] = 0;
		hub_desc->u.hs.DeviceRemovable[1] = 0xff;
		break;
		
	case SetHubFeature:
	case ClearHubFeature:
		switch (wValue)
		{
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
			
		default:
			return -EPIPE;
		}
		break;
		
	case ClearPortFeature:
		if (wIndex != 1) {
			return -EPIPE;
		}
		switch (wValue)
		{
		case USB_PORT_FEAT_ENABLE:
			return hub_control_port_reset(hotplug);
			
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_POWER:
			break;
			
		default:
			return -EPIPE;
		}
		break;
		
	case SetPortFeature:
		if (wIndex != 1) {
			return -EPIPE;
		}
		switch (wValue)
		{
		case USB_PORT_FEAT_RESET:
			return hub_control_port_reset(hotplug);
			
		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_POWER:
			break;
			
		default:
			return -EPIPE;
		}
		break;
		
	case GetPortStatus:
		if (wIndex != 1) {
			return -EPIPE;
		}
		port = atomic_read_acquire(&hotplug->prstate);
		status_changed(hotplug, port);
		
		wPortChange = hotplug->wPortChange;
		hotplug->wPortChange = 0;
		
		wPortStatus = USB_PORT_STAT_POWER;
		
		if (port != 0)
		{
			wPortStatus |= USB_PORT_STAT_CONNECTION;
			wPortStatus |= USB_PORT_STAT_ENABLE;
			
			if (port & AOTG_PORT_RESET) {
				wPortStatus |= USB_PORT_STAT_RESET;
			}
			else if (port & AOTG_PORT_HIGH_SPEED) {
				wPortStatus |= USB_PORT_STAT_HIGH_SPEED;
			}
			else if (port & AOTG_PORT_LOW_SPEED) {
				wPortStatus |= USB_PORT_STAT_LOW_SPEED;
			}
		}
		
		*(__le16 *)buf = cpu_to_le16(wPortStatus);
		*(__le16 *)(buf + 2) = cpu_to_le16(wPortChange);
		break;
		
	default:
		return -EPIPE;
	}
	return 0;
}

static int aotg_port_reset(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	// port reset takes around 55ms to complete
	long res = msecs_to_jiffies(150);
	unsigned long flags;
	bool success;
	int prstate;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	atomic_set_release(&hotplug->prstate, AOTG_PORT_RESET);
	
	// clear port status irq
	writeb(USBIRQ_URES | USBIRQ_HS, acthcd->base + USBIRQ);
	
	// start port reset (needs at least 55ms)
	writeb(BIT(6) | BIT(5), acthcd->base + HCPORTCTRL);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	
	do {
		int msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL) {
			return -ECANCELED;
		}
		
		success = false;
		
		if (msg & MSG_CS_HFMODE)
		{
			success = true;
			prstate = AOTG_PORT_HIGH_SPEED;
		}
		else if (msg & MSG_CS_FSMODE)
		{
			success = true;
			prstate = AOTG_PORT_FULL_SPEED;
		}
		else if (msg & MSG_CS_LSMODE)
		{
			success = true;
			prstate = AOTG_PORT_LOW_SPEED;
		}
		
		if (success)
		{
			spin_lock_irqsave(&hotplug->irq_lock, flags);
			atomic_set(&hotplug->prstate, prstate);
			atomic_set_release(&hotplug->rhstate, AOTG_RH_ENABLE);
			spin_unlock_irqrestore(&hotplug->irq_lock, flags);
			return 0;
		}
		
		res = wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0, res);
		
	} while (res > 0);
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	atomic_set(&hotplug->prstate, 0);
	atomic_set_release(&hotplug->rhstate, AOTG_RH_DISABLE);
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	return -ETIMEDOUT;
}

static void aotg_disable_irq(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	// disable port status irq
	usb_clearbitsb(USBIEN_URES | USBIEN_HS, acthcd->base + USBIEN);
	
	// disable usb and otg irqs
	usb_clearbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	usb_clearbitsb(BIT(2), acthcd->base + OTGIEN);
	usb_clearbitsb(OTGCTRL_BUSREQ, acthcd->base + OTGCTRL);
	
	// clear port status irq
	writeb(USBIRQ_URES | USBIRQ_HS, acthcd->base + USBIRQ);
	
	// clear usb and otg irqs
	writeb(USBEIRQ_USBIRQ, acthcd->base + USBEIRQ);
	writeb(BIT(2), acthcd->base + OTGIRQ);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
}

static void aotg_enable_irq(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	// enable usb and otg irqs
	usb_setbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	usb_setbitsb(BIT(2), acthcd->base + OTGIEN);
	usb_setbitsb(OTGCTRL_BUSREQ, acthcd->base + OTGCTRL);
	
	// enable port status irq
	usb_setbitsb(USBIEN_URES | USBIEN_HS, acthcd->base + USBIEN);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
}

u32 aotg_hotplug_irq_handler(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	u8 usb_irq, otg_irq, port_irq, ivector;
	unsigned long flags;
	int msg = 0;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	usb_irq  = readb(acthcd->base + USBEIRQ);
	port_irq = readb(acthcd->base + USBIRQ);
	otg_irq  = readb(acthcd->base + OTGIRQ);
	ivector = 0;
	
	if (usb_irq & USBEIRQ_USBIRQ)
	{
		ivector = (u32)readb(acthcd->base + IVECT);
		
		switch (ivector)
		{
		case UIV_IDLE:
		case UIV_SRPDET:
		case UIV_LOCSOF:
		case UIV_VBUSERR:
		case UIV_PERIPH:
			if (otg_irq & BIT(2))
			{
				u8 otg_state = readb(acthcd->base + OTGSTATE);
				
				if (otg_state == A_HOST) {
					msg |= MSG_ATTACHED;
				}
				else if (otg_state == A_WAIT_BCON) {
					msg |= MSG_REMOVED;
				}
			}
			break;
			
		case UIV_USBRESET:
			if (port_irq & USBIRQ_URES)
			{
				u8 usbcs = readb(acthcd->base + USBCS);
				
				if ((usbcs & USBCS_HFMODE) || (port_irq & USBIRQ_HS)) {
					msg |= MSG_CS_HFMODE;
				}
				else if (usbcs & USBCS_LSMODE) {
					msg |= MSG_CS_LSMODE;
				}
				else {
					msg |= MSG_CS_FSMODE;
				}
				
				// reset all input endpoints
				ep_reset(acthcd, USB_HCD_IN_MASK,
					ENDPRST_FIFORST | ENDPRST_TOGRST);
				
				// reset all output endpoints
				ep_reset(acthcd, USB_HCD_OUT_MASK,
					ENDPRST_FIFORST | ENDPRST_TOGRST);
				
				writew(0xffff, acthcd->base + HCINxERRIRQ0);
				writew(0xffff, acthcd->base + HCOUTxERRIRQ0);
				writew(0xffff, acthcd->base + HCINxIRQ0);
				writew(0xffff, acthcd->base + HCOUTxIRQ0);
				writew(0xffff, acthcd->base + HCINxERRIEN0);
				writew(0xffff, acthcd->base + HCOUTxERRIEN0);
			}
			break;
		}
	}
	
	if (usb_irq) {
		writeb(usb_irq, acthcd->base + USBEIRQ);
	}
	if (port_irq) {
		writeb(port_irq, acthcd->base + USBIRQ);
	}
	if (otg_irq) {
		writeb(otg_irq, acthcd->base + OTGIRQ);
	}
	
	if (msg)
	{
		atomic_fetch_or_release(msg, &hotplug->mailbox);
		wake_up(&hotplug->event);
	}
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	return ivector;
}

static int aotg_wait_hw_reset(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	int retry, msg;
	
	if (!(readb(acthcd->base + USBERESET) & USBERES_USBRESET)) {
		return 0;
	}
	for (retry = 10000; retry > 0; retry--)
	{
		usleep_range(100, 120);
		
		if (!(readb(acthcd->base + USBERESET) & USBERES_USBRESET)) {
			return 0;
		}
		
		msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL) {
			return -ECANCELED;
		}
	}
	return -ETIMEDOUT;
}

static void aotg_DD_set_phy(void __iomem *base, u8 reg, u8 value)
{
	u8 addrlow = reg & 0x0f;
	u8 addrhigh = (reg >> 4) & 0x0f;
	
	/*write vstatus: */
	writeb(value, base + VDSTATUS);
	smp_mb();
	
	/*write vcontrol: */
	writeb(addrlow | 0x10, base + VDCTRL);
	udelay(1); /*the vload period should be  > 33.3ns*/
	writeb(addrlow & 0x0f, base + VDCTRL);
	udelay(1);
	smp_mb();
	writeb(addrlow | 0x10, base + VDCTRL);
	udelay(1);
	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(1);
	writeb(addrhigh & 0x0f, base + VDCTRL);
	udelay(1);
	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(1);
}

static void aotg_set_hcd_phy(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	
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

static void aotg_hw_setup(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	u8 val8;
	
	writel(0x1, acthcd->base + HCDMABCKDOOR);
	
	if (acthcd->model == CANINOS_HW_MODEL_K7) {
		usb_writel(0x37000000 | (0x3 << 4), acthcd->usbecs);
	}
	else {
		usb_writel(0x37000000 | (0x10 << 13) | (0xb << 4), acthcd->usbecs);
	}
	
	usleep_range(100, 120);
	aotg_set_hcd_phy(hotplug);
	
	writeb(0x0, acthcd->base + TA_BCON_COUNT);
	usb_writeb(0xff, acthcd->base + TAAIDLBDIS);
	usb_writeb(0xff, acthcd->base + TAWAITBCON);
	usb_writeb(0x28, acthcd->base + TBVBUSDISPLS);
	usb_setb(1 << 7, acthcd->base + TAWAITBCON);
	usb_writew(0x1000, acthcd->base + VBUSDBCTIMERL);
	
	val8 = readb(acthcd->base + BKDOOR);
	val8 &= ~(1 << 7);
	writeb(val8, acthcd->base + BKDOOR);
}

static void aotg_hw_shutdown(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	const int rhstate = AOTG_RH_POWEROFF;
	
	if (atomic_read_acquire(&hotplug->rhstate) == rhstate) {
		return;
	}
	if (atomic_xchg_release(&hotplug->rhstate, rhstate) != rhstate)
	{
		aotg_disable_irq(hotplug);
		reset_control_assert(acthcd->rst);
		atomic_set_release(&hotplug->prstate, 0);
	}
}

static int aotg_hw_powerup(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	int retval;
	
	if (atomic_cmpxchg_acquire(&hotplug->rhstate,
		AOTG_RH_POWEROFF, AOTG_RH_RESET) != AOTG_RH_POWEROFF) {
		return -EINVAL;
	}
	
	udelay(1);
	reset_control_deassert(acthcd->rst);
	retval = aotg_wait_hw_reset(hotplug);
	
	if (!retval)
	{
		aotg_hw_setup(hotplug);
		aotg_enable_irq(hotplug);
		atomic_set_release(&hotplug->rhstate, AOTG_RH_POWERED);
		return 0;
	}
	
	reset_control_assert(acthcd->rst);
	atomic_set_release(&hotplug->rhstate, AOTG_RH_POWEROFF);
	return retval;
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

