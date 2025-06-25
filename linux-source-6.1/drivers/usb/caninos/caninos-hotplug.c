#include "aotg_hcd.h"

#define MSG_CANCEL    BIT(0)
#define MSG_CS_HFMODE BIT(3)
#define MSG_CS_FSMODE BIT(4)
#define MSG_CS_LSMODE BIT(5)
#define MSG_REQ_RESET BIT(6)

// usbecs register
#define USB2_ECS_VBUS_P0       BIT(10)
#define USB2_ECS_ID_P0         BIT(12)
#define USB2_ECS_DPPUEN_P0     BIT(3)
#define USB2_ECS_DMPUEN_P0     BIT(2)
#define USB2_ECS_DMPDDIS_P0    BIT(1)
#define USB2_ECS_DPPDDIS_P0    BIT(0)
#define USB2_ECS_SOFTIDEN_P0   BIT(26)
#define USB2_ECS_SOFTID_P0     BIT(27)
#define USB2_ECS_SOFTVBUSEN_P0 BIT(24)
#define USB2_ECS_SOFTVBUS_P0   BIT(25)

// OTG Control Register (for Host operation)
#define USBH_OTGCTRL 0x1BE
// Bit used for the core testing purpose.
// It should be always written with zero.
#define OTGCTRL_FORCEBCONN BIT(7)
// Bit 6 is reserved.
#define OTGCTRL_SRPDATDETEN     (1 << 5)
#define OTGCTRL_SRPVBUSDETEN    (1 << 4)
#define OTGCTRL_BHNPEN          (1 << 3)
#define OTGCTRL_ASETBHNPEN      (1 << 2)
// Bit 1 controls OTG FSM. It forces the bus power-down.
#define OTGCTRL_ABUSDROP BIT(1)
// Bit 0 is used to start or end the session
// Meaning of this bit is, depending on id signal state, the same as a_bus_req
// or b_bus_req bit from OTG Supplement Specification.
#define OTGCTRL_BUSREQ BIT(0)

static void hotplug_worker(struct work_struct *t);

static int aotg_port_powerup(struct aotg_hotplug *hotplug);

static void aotg_port_shutdown(struct aotg_hotplug *hotplug);

void aotg_init_hotplug(struct aotg_hotplug *hotplug, struct aotg_hcd *acthcd)
{
	pm_runtime_enable(acthcd->dev);
	pm_runtime_get_sync(acthcd->dev);
	
	clk_prepare_enable(acthcd->clk_usbh_phy);
	clk_prepare_enable(acthcd->clk_usbh_pllen);
	reset_control_assert(acthcd->rst);
	
	hotplug->acthcd = acthcd;
	hotplug->prev_port_stat = 0;
	hotplug->wPortChange = 0;
	
	spin_lock_init(&hotplug->irq_lock);
	mutex_init(&hotplug->lock);
	init_waitqueue_head(&hotplug->event);
	INIT_WORK(&hotplug->work, hotplug_worker);
	
	atomic_set(&hotplug->mailbox, 0);
	atomic_set(&hotplug->running, 0);
	hotplug->port_stat = 0;
	
	smp_mb();
}

void aotg_stop_hotplug(struct aotg_hotplug *hotplug)
{
	if (atomic_cmpxchg_acquire(&hotplug->running, 1, 0) == 1)
	{
		atomic_fetch_or_release(MSG_CANCEL, &hotplug->mailbox);
		wake_up(&hotplug->event);
		cancel_work_sync(&hotplug->work);
	}
}

void aotg_start_hotplug(struct aotg_hotplug *hotplug)
{
	if (atomic_cmpxchg_release(&hotplug->running, 0, 1) == 0)
	{
		atomic_set_release(&hotplug->mailbox, 0);
		schedule_work(&hotplug->work);
	}
}

static int hotplug_procedure(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	u8 otg_state;
	long res;
	int msg;
	
wait_connection:
	
	dev_info(acthcd->dev, "waiting for device connection\n");
	
	usb_setbitsb(OTGCTRL_BUSREQ, acthcd->base + USBH_OTGCTRL);
	
	while (1)
	{
		int msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL)
		{
			aotg_port_shutdown(hotplug);
			return -ECANCELED;
		}
		
		otg_state = readb(acthcd->base + OTGSTATE);
		
		if (readb(acthcd->base + OTGSTATE) == A_HOST)
		{
			mutex_lock(&hotplug->lock);
			hotplug->port_stat |= USB_PORT_STAT_CONNECTION;
			mutex_unlock(&hotplug->lock);
			
			usb_clearbitsb(OTGCTRL_BUSREQ, acthcd->base + USBH_OTGCTRL);
			
			dev_info(acthcd->dev, "device connected\n");
			break;
		}
		
		wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0,
			msecs_to_jiffies(500));
	}
	
port_reset:
	
	mutex_lock(&hotplug->lock);
	hotplug->port_stat |= USB_PORT_STAT_RESET | USB_PORT_STAT_ENABLE;
	mutex_unlock(&hotplug->lock);
	
	dev_info(acthcd->dev, "device reset\n");
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	// clear port reset irq
	writeb(USBIRQ_URES | USBIRQ_HS, acthcd->base + USBIRQ);
	
	// enable port reset irq
	usb_setbitsb(USBIEN_URES | USBIEN_HS, acthcd->base + USBIEN);
	
	// start port reset
	writeb(BIT(6) | BIT(5), acthcd->base + HCPORTCTRL);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	
	// port reset takes around 55ms to complete
	res = msecs_to_jiffies(80);
	do
	{
		int msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL)
		{
			aotg_port_shutdown(hotplug);
			return -ECANCELED;
		}
		
		if (msg & MSG_CS_HFMODE)
		{
			mutex_lock(&hotplug->lock);
			hotplug->port_stat &= ~USB_PORT_STAT_RESET;
			hotplug->port_stat &= ~USB_PORT_STAT_LOW_SPEED;
			hotplug->port_stat |= USB_PORT_STAT_HIGH_SPEED;
			mutex_unlock(&hotplug->lock);
			
			dev_info(acthcd->dev, "device is high speed\n");
			break;
		}
		
		if (msg & MSG_CS_LSMODE)
		{
			mutex_lock(&hotplug->lock);
			hotplug->port_stat &= ~USB_PORT_STAT_RESET;
			hotplug->port_stat &= ~USB_PORT_STAT_HIGH_SPEED;
			hotplug->port_stat |= USB_PORT_STAT_LOW_SPEED;
			mutex_unlock(&hotplug->lock);
			
			dev_info(acthcd->dev, "device is low speed\n");
			break;
		}
		
		if (msg & MSG_CS_FSMODE)
		{
			mutex_lock(&hotplug->lock);
			hotplug->port_stat &= ~USB_PORT_STAT_RESET;
			hotplug->port_stat &= ~USB_PORT_STAT_HIGH_SPEED;
			hotplug->port_stat &= ~USB_PORT_STAT_LOW_SPEED;
			mutex_unlock(&hotplug->lock);
			
			dev_info(acthcd->dev, "device is full speed\n");
			break;
		}
		
		res = wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0, res);
		
		if (res <= 0)
		{
			aotg_port_shutdown(hotplug);
			dev_err(acthcd->dev, "reset timedout\n");
			return -ETIMEDOUT;
		}
		
	} while (res > 0);
	
	while (1)
	{
		msg = atomic_fetch_and_acquire(0, &hotplug->mailbox);
		
		if (msg & MSG_CANCEL)
		{
			aotg_port_shutdown(hotplug);
			return -ECANCELED;
		}
		
		if (msg & MSG_REQ_RESET)
		{
			dev_info(acthcd->dev, "port reset requested\n");
			
			usb_clearbitsb(OTGCTRL_BUSREQ, acthcd->base + USBH_OTGCTRL);
			goto port_reset;
		}
		
		otg_state = readb(acthcd->base + OTGSTATE);
		
		if (otg_state == A_WAIT_BCON)
		{
			mutex_lock(&hotplug->lock);
			hotplug->port_stat = USB_PORT_STAT_POWER;
			mutex_unlock(&hotplug->lock);
			
			goto wait_connection;
		}
		else if (otg_state == A_SUSPEND)
		{
			usb_setbitsb(OTGCTRL_BUSREQ, acthcd->base + USBH_OTGCTRL);
			
			dev_info(acthcd->dev, "port resume\n");
		}
		else if (otg_state != A_HOST)
		{
			dev_err(acthcd->dev, "invalid otg_state 0x%x\n", (u32)otg_state);
			break;
		}
		
		wait_event_timeout(hotplug->event, 
			atomic_read_acquire(&hotplug->mailbox) != 0,
			msecs_to_jiffies(600));
	}
	
	
	aotg_port_shutdown(hotplug);
	return -EAGAIN;
}

static void hotplug_worker(struct work_struct *t)
{
	struct aotg_hotplug *hotplug = container_of(t, typeof(*hotplug), work);
	struct aotg_hcd *acthcd = hotplug->acthcd;
	int retval = -EINVAL;
	
	dev_info(acthcd->dev, "%s: started\n", __func__);
	
	do
	{
		retval = aotg_port_powerup(hotplug);
		
		if (retval == 0)
		{
			retval = hotplug_procedure(hotplug);
		}
		
		
	} while (retval != -ECANCELED);
	
	dev_info(acthcd->dev, "%s: finished\n", __func__);
}

static int hub_control_port_reset(struct aotg_hotplug *hotplug)
{
	atomic_fetch_or_release(MSG_REQ_RESET, &hotplug->mailbox);
	wake_up(&hotplug->event);
	return 0;
}

static bool status_changed(struct aotg_hotplug *hotplug, int ps)
{
	int prv = hotplug->prev_port_stat;
	hotplug->prev_port_stat = ps;
	
	if ((ps & USB_PORT_STAT_CONNECTION) != (prv & USB_PORT_STAT_CONNECTION)) {
		hotplug->wPortChange |= USB_PORT_STAT_C_CONNECTION;
	}
	if ((ps & USB_PORT_STAT_RESET) != (prv & USB_PORT_STAT_RESET)) {
		hotplug->wPortChange |= USB_PORT_STAT_C_RESET;
	}
	
	return (hotplug->wPortChange != 0);
}

int aotg_hotplug_status_data(struct aotg_hotplug *hotplug, char *buf)
{
	bool changed;
	*buf = 0;
	
	mutex_lock(&hotplug->lock);
	changed = status_changed(hotplug, hotplug->port_stat);
	mutex_unlock(&hotplug->lock);
	
	if (changed) {
		*buf = (char)BIT(1);
	}
	return changed ? 1 : 0;
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
			dev_info(hotplug->acthcd->dev,
				"%s: ClearPortFeature::USB_PORT_FEAT_ENABLE\n", __func__);
			
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
			
			dev_info(hotplug->acthcd->dev,
				"%s: SetPortFeature::USB_PORT_FEAT_RESET\n", __func__);
			
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
		
		mutex_lock(&hotplug->lock);
		
		port = hotplug->port_stat;
		status_changed(hotplug, port);
		
		wPortChange = hotplug->wPortChange;
		hotplug->wPortChange = 0;
		
		mutex_unlock(&hotplug->lock);
		
		wPortStatus = port;
		
		*(__le16 *)buf = cpu_to_le16(wPortStatus);
		*(__le16 *)(buf + 2) = cpu_to_le16(wPortChange);
		break;
		
	default:
		return -EPIPE;
	}
	return 0;
}

u32 aotg_hotplug_irq_handler(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	u8 usb_irq, port_irq, ivector;
	unsigned long flags;
	int msg = 0;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	usb_irq = readb(acthcd->base + USBEIRQ);
	port_irq = readb(acthcd->base + USBIRQ);
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
			dev_info(acthcd->dev, "ivector %x\n", ivector);
			break;
			
		case UIV_USBRESET:
			if (port_irq & USBIRQ_URES)
			{
				u8 usbcs = readb(acthcd->base + USBCS);
				
				// disable port reset irq
				usb_clearbitsb(USBIEN_URES | USBIEN_HS, acthcd->base + USBIEN);
				
				// clear port reset irq
				writeb(port_irq, acthcd->base + USBIRQ);
				port_irq = 0;
				
				dev_info(acthcd->dev, "UIV_USBRESET\n");
				
				if (usbcs & USBCS_HFMODE) {
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

static void aotg_port_shutdown(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	// disable port reset irq
	usb_clearbitsb(USBIEN_URES | USBIEN_HS, acthcd->base + USBIEN);
	
	// disable usb irq
	usb_clearbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	
	mutex_lock(&hotplug->lock);
	hotplug->port_stat = 0;
	mutex_unlock(&hotplug->lock);
	
	reset_control_assert(acthcd->rst);
	
	dev_info(acthcd->dev, "host shutdown\n");
}

static int aotg_port_powerup(struct aotg_hotplug *hotplug)
{
	struct aotg_hcd *acthcd = hotplug->acthcd;
	unsigned long flags;
	int retval;
	u8 val8;
	
	udelay(1);
	reset_control_deassert(acthcd->rst);
	retval = aotg_wait_hw_reset(hotplug);
	
	if (retval)
	{
		reset_control_assert(acthcd->rst);
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
	aotg_set_hcd_phy(hotplug);
	
	// time from peripheral connection to host issue bus reset
	// time = 110ms * (ta_bcon_count + 1)
	writeb(0x0, acthcd->base + TA_BCON_COUNT); // set to 110ms
	
	// suspend state timeout
	// timeout = register value[6:0] * 8.738ms
	// if bit 7 is set, timeout is never generated, thus stays in suspend
	// state for any time
	usb_writeb(BIT(7), acthcd->base + TAAIDLBDIS);
	
	// wait peripheral connection timeout
	// timeout = register value[6:0] * 34.953ms
	// if bit 7 is set, timeout is never generated, thus always wait
	usb_writeb(BIT(7), acthcd->base + TAWAITBCON);
	
	// vbus discharge timer
	// time = register value * 1.092ms
	usb_writeb(0x28, acthcd->base + TBVBUSDISPLS); // set to 43.68ms
	
	// vbus debounce timer
	// any glitch of vbus which lasts less than time will be ignored
	// time = register value[15:0] * clock period
	usb_writew(0x1000, acthcd->base + VBUSDBCTIMERL);
	
	// enable hs operation
	val8 = readb(acthcd->base + BKDOOR);
	val8 &= ~BIT(7);
	writeb(val8, acthcd->base + BKDOOR);
	
	spin_lock_irqsave(&hotplug->irq_lock, flags);
	
	// enable usb irq
	usb_setbitsb(USBEIRQ_USBIEN, acthcd->base + USBEIEN);
	
	spin_unlock_irqrestore(&hotplug->irq_lock, flags);
	
	mutex_lock(&hotplug->lock);
	hotplug->port_stat = USB_PORT_STAT_POWER;
	mutex_unlock(&hotplug->lock);
	
	dev_info(acthcd->dev, "host powered\n");
	
	return 0;
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

