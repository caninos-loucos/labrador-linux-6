/*
 * Actions OWL SoCs usb2.0 controller driver
 *
 * Copyright (c) 2015 Actions Semiconductor Co., ltd.
 * dengtaiping <dengtaiping@actions-semi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_USB_HOST_AOTG_H
#define __LINUX_USB_HOST_AOTG_H

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>
#include <linux/usb/hcd.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>

#include "aotg_regs.h"

#define TRB_ITE       (1 << 11)
#define TRB_CHN       (1 << 10)
#define TRB_CSP       (1 << 9)
#define TRB_COF       (1 << 8)
#define TRB_ICE       (1 << 7)
#define TRB_IZE       (1 << 6)
#define TRB_ISE       (1 << 5)
#define TRB_LT        (1 << 4)
#define AOTG_TRB_IOC  (1 << 3)
#define AOTG_TRB_IOZ  (1 << 2)
#define AOTG_TRB_IOS  (1 << 1)
#define TRB_OF        (1 << 0)

#define NUM_TRBS  (256)
#define RING_SIZE (NUM_TRBS * 16)

#define DRIVER_DESC "Caninos USB Host Controller Driver"
#define DRIVER_NAME "caninos-hcd"

enum caninos_hw_model {
	CANINOS_HW_MODEL_K5 = 1,
	CANINOS_HW_MODEL_K7,
};

struct aotg_trb {
	u32 hw_buf_ptr;
	u32 hw_buf_len;
	u32 hw_buf_remain;
	u32 hw_token;
};

#define PERIODIC_SIZE 64
#define MAX_PERIODIC_LOAD 500 /*50%*/

#define AOTG_BULK_FIFO_START_ADDR  0x0080
#define AOTG_INTERRUPT_FIFO_START_ADDR  0xC00

#define USB_HCD_IN_MASK 0x00
#define USB_HCD_OUT_MASK 0x10

#define AOTG_MAX_FIFO_SIZE (1024*15 + 64*2)
#define ALLOC_FIFO_UNIT 64

#define AOTG_PORT_C_MASK  ((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

#define MAX_EP_NUM 16 /* count of each type, 1st is reserved */
#define MAX_SG_TABLE (0x1 << 9)

#define MAX_ERROR_COUNT 6
#define MAX_PACKET(x) ((x)&0x7FF)

struct hcd_stats {
	unsigned long insrmv;
	unsigned long wake;
	unsigned long sof;
};

enum aotg_rh_state {
	AOTG_RH_POWEROFF,
	AOTG_RH_POWERED,
	AOTG_RH_ATTACHED,
	AOTG_RH_NOATTACHED,
	AOTG_RH_RESET,
	AOTG_RH_ENABLE,
	AOTG_RH_DISABLE,
	AOTG_RH_SUSPEND,
	AOTG_RH_ERR
};

enum control_phase {
	PHASE_UNDEFINED,
	PHASE_SETUP,
	PHASE_DATA,
	PHASE_STATUS,
};

#define INTR_TRBS (10)

#define RING_IN_OF (0xFFFE)
#define RING_OUT_OF (0xFFFE0000)

#define TD_IN_FINISH (0)
#define TD_IN_RING  (0x1 << 0)
#define TD_IN_QUEUE (0x1 << 1)

/* usbecs register. */
#define USB2_ECS_VBUS_P0        (10)
#define USB2_ECS_ID_P0          (12)
#define USB2_ECS_LS_P0_SHIFT    (8)
#define USB2_ECS_LS_P0_MASK     (0x3<<8)
#define USB2_ECS_DPPUEN_P0      (3)
#define USB2_ECS_DMPUEN_P0      (2)
#define USB2_ECS_DMPDDIS_P0     (1)
#define USB2_ECS_DPPDDIS_P0     (0)
#define USB2_ECS_SOFTIDEN_P0    (1<<26)
#define USB2_ECS_SOFTID_P0      (27)
#define USB2_ECS_SOFTVBUSEN_P0  (1<<24)
#define USB2_ECS_SOFTVBUS_P0    (25)

struct aotg_ring {
	unsigned is_running:1;
	unsigned is_out:1;
	unsigned intr_inited:1;
	unsigned intr_started:1;
	unsigned ring_stopped:1;
	int num_trbs;
	int type;
	u8 mask;
	void *priv;
	atomic_t num_trbs_free;

	int intr_mem_size;
	struct dma_pool *intr_dma_pool;
	u8 *intr_dma_buf_vaddr;
	dma_addr_t intr_dma_buf_phyaddr;
	char pool_name[32];

	struct aotg_trb *enqueue_trb;
	struct aotg_trb *dequeue_trb;
	struct aotg_trb *first_trb;
	struct aotg_trb *last_trb;
	struct aotg_trb *ring_trb;
	u32 trb_dma;

	unsigned int enring_cnt;
	unsigned int dering_cnt;

	volatile void __iomem *reg_dmalinkaddr;
	volatile void __iomem *reg_curaddr;
	volatile void __iomem *reg_dmactrl;
	volatile void __iomem *reg_dmacomplete_cnt;
};

struct aotg_td {
	struct urb *urb;
	int num_trbs;
	u32 trb_dma;
	int err_count;
	struct aotg_trb *trb_vaddr;
	struct list_head queue_list;
	struct list_head enring_list;
	struct list_head dering_list;
	u8 *intr_mem_vaddr;
	dma_addr_t intr_men_phyaddr;
	int mem_size;
	unsigned cross_ring:1;
} __aligned(64);

struct aotg_queue {
	int in_using;
	struct aotg_hcep *ep;
	struct urb *urb;
	int dma_no;
	int is_xfer_start;
	int need_zero;
	
	struct list_head enqueue_list;
	struct list_head dequeue_list;
	struct list_head finished_list;
	int status;
	int length;
	
	struct aotg_td td;
	
	struct scatterlist *cur_sg;
	int err_count;
	unsigned long timeout; /* jiffies + n. */
	
	/* fixing dma address unaligned to 4 Bytes. */
	u8 *dma_copy_buf;
	dma_addr_t dma_addr;
	
	/* for debug. */
	unsigned int seq_info;
} __attribute__ ((aligned(4)));

struct aotg_dma_buf {
	unsigned int size;
	u8 *buf;
	dma_addr_t dma;
	int in_using;
};

extern void urb_tasklet_func(unsigned long data);

struct aotg_hcep_pool
{
	struct aotg_hcep *ep0[MAX_EP_NUM];
	struct aotg_hcep *inep[MAX_EP_NUM];
	struct aotg_hcep *outep[MAX_EP_NUM];
};

#define AOTG_MAX_FIFO_MAP_CNT (AOTG_MAX_FIFO_SIZE / ALLOC_FIFO_UNIT)
#define AOTG_QUEUE_POOL_CNT 60

struct aotg_hcd
{
	enum caninos_hw_model model;
	int uhc_irq;
	void __iomem *base;
	void __iomem *usbecs;
	struct device *dev;
	volatile int hcd_exiting;
	struct reset_control *rst;
	struct clk *clk_usbh_pllen;
	struct clk *clk_usbh_phy;
	struct clk *clk_usbh_cce;
	spinlock_t lock;
	spinlock_t tasklet_lock;
	
	struct usb_hcd *hcd;
	
	resource_size_t rsrc_start, rsrc_len;
	
	struct tasklet_struct urb_tasklet;
	volatile int tasklet_retry;
	
	int inserted; /*imply a USB deivce inserting in MiniA receptacle*/
	u32 port; /*indicate portstatus and portchange*/
	enum aotg_rh_state rhstate;
	
	struct list_head hcd_enqueue_list;
	struct list_head hcd_dequeue_list;
	struct list_head hcd_finished_list;
	
	/* when using hub, every usb device need a ep0 hcep data struct,
	 * but share the same hcd ep0.
	 */
	struct aotg_hcep *active_ep0;
	struct aotg_hcep_pool hcep_pool;
	struct aotg_queue *queue_pool[AOTG_QUEUE_POOL_CNT];
	ulong fifo_map[AOTG_MAX_FIFO_MAP_CNT];
	
	
	
	
	volatile int discon_happened;
	volatile int put_aout_msg;
	
	int check_trb_mutex;
	
	struct proc_dir_entry *pde;
	enum usb_device_speed  speed;
	
	u16 hcin_dma_ien;
	u16 hcout_dma_ien;
	
	int ep0_block_cnt;
	
	struct timer_list trans_wait_timer;
	struct timer_list check_trb_timer;
	
	struct hrtimer hotplug_timer;
	
	int suspend_request_pend;
	int bus_remote_wakeup;
	
	#define AOTG_DMA_BUF_CNT 8
	struct aotg_dma_buf dma_poll[AOTG_DMA_BUF_CNT];
};

extern void aotg_hcep_pool_init(struct aotg_hcd *acthcd);

struct aotg_hcep
{
	struct usb_host_endpoint *hep;
	struct usb_device *udev;
	int index;
	/* just for ep0, when using hub, every usb device need a 
	 * ep0 hcep data struct, but share the same hcd ep0. 
	 */
	int ep0_index;
	int iso_packets;
	u32 maxpacket;
	u16 error_count;
	u16 length;
	u8 epnum;
	u8 nextpid;
	u8 mask;
	u8 type;
	u8 is_out;
	u8 buftype;
	u8 has_hub;
	u8 hub_addr;
	u8 dev_addr;
	u8 reg_hcep_splitcs_val;
	
	void __iomem *reg_hcepcs;
	void __iomem *reg_hcepcon;
	void __iomem *reg_hcepctrl;
	void __iomem *reg_hcepbc;
	void __iomem *reg_hcmaxpck;
	void __iomem *reg_hcepaddr;
	void __iomem *reg_hcerr;
	void __iomem *reg_hcep_interval;
	void __iomem *reg_hcep_dev_addr;
	void __iomem *reg_hcep_port;
	void __iomem *reg_hcep_splitcs;

	unsigned int urb_enque_cnt;
	unsigned int urb_endque_cnt;
	unsigned int urb_stop_stran_cnt;
	unsigned int urb_unlinked_cnt;

	u32 dma_bytes;
	u16 interval;
	u16 load; /* one packet times in us. */

	u16 fifo_busy;

	ulong fifo_addr;
	
	
	struct aotg_queue *q;

	struct aotg_ring *ring;
	struct list_head queue_td_list;
	struct list_head enring_td_list;
	struct list_head dering_td_list;
};

#define get_hcepcon_reg(dir , x , y , z)   ((dir ? x : y) + (z - 1)*8)
#define get_hcepcs_reg(dir , x , y , z)   ((dir ? x : y) + (z - 1)*8)
#define get_hcepctrl_reg(dir , x , y , z)   ((dir ? x : y) + (z - 1)*4)
#define get_hcepbc_reg(dir , x , y , z)   ((dir ? x : y) + (z - 1)*8)
#define get_hcepmaxpck_reg(dir , x , y , z)   ((dir ? x : y) + (z - 1)*2)
#define get_hcepaddr_reg(dir , x , y , z)  ((dir ? x : y) + (z - 1)*4)
#define get_hcep_dev_addr_reg(dir , x , y , z)  ((dir ? x : y) + (z - 1)*8)
#define get_hcep_port_reg(dir , x , y , z)  ((dir ? x : y) + (z - 1)*8)
#define get_hcep_splitcs_reg(dir , x , y , z)  ((dir ? x : y) + (z - 1)*8)
#define get_hcerr_reg(dir, x, y, z)  ((dir ? x : y) + (z)*4)
#define get_hcep_interval_reg(dir, x, y, z) (dir ? (x + (z-1)*8) : (y + (z)*8))
#define GET_DMALINKADDR_REG(dir, x, y, z) ((dir ? x : y) + (z - 1) * 0x10)
#define GET_CURADDR_REG(dir, x, y, z) ((dir ? x : y) + (z - 1) * 0x10)
#define GET_DMACTRL_REG(dir, x, y, z) ((dir ? x : y) + (z - 1) * 0x10)
#define GET_DMACOMPLETE_CNT_REG(dir, x, y, z) ((dir ? x : y) + (z - 1) * 0x10)

#define AOTG_DMA_OUT_PREFIX 0x10
#define AOTG_DMA_NUM_MASK 0xf
#define AOTG_IS_DMA_OUT(x) ((x) & AOTG_DMA_OUT_PREFIX)
#define AOTG_GET_DMA_NUM(x) ((x) & AOTG_DMA_NUM_MASK)

// caninos-xfer.c ------------------------------------------------------------//

extern void release_fifo_slot(struct aotg_hcd *, struct aotg_hcep *);

extern ulong get_fifo_slot(struct aotg_hcd *, int size);

extern void hcep_free(struct usb_hcd *, struct aotg_hcep *);

extern int hcep_config(struct usb_hcd *, struct urb *, struct aotg_hcep *);

// -------------------------------------------------------------------------- //

static inline void pio_irq_disable(struct aotg_hcd *acthcd, u8 mask)
{
	u8 is_out = mask & USB_HCD_OUT_MASK;
	u8 ep_num = mask & 0x0f;
	
	if (is_out) {
		usb_clearbitsw(1 << ep_num, acthcd->base + HCOUTxIEN0);
	}
	else {
		usb_clearbitsw(1 << ep_num, acthcd->base + HCINxIEN0);
	}
}

static inline void pio_irq_clear(struct aotg_hcd *acthcd, u8 mask)
{
	u8 is_out = mask & USB_HCD_OUT_MASK;
	u8 ep_num = mask & 0x0f;
	
	if (is_out) {
		writew(1 << ep_num, acthcd->base + HCOUTxIRQ0);
	}
	else {
		writew(1 << ep_num, acthcd->base + HCINxIRQ0);
	}
}

static inline void ep_reset(struct aotg_hcd *acthcd, u8 ep_mask, u8 type_mask)
{
	u8 val;
	writeb(ep_mask, acthcd->base + ENDPRST);
	val = ep_mask | type_mask;
	writeb(val, acthcd->base + ENDPRST);
}

static inline void ep_enable(struct aotg_hcep *ep) {
	usb_setbitsb(0x80, ep->reg_hcepcon);
}

static inline void ep_disable(struct aotg_hcep *ep) {
	usb_clearbitsb(0x80, ep->reg_hcepcon);
}

extern int aotg_hcep_ctrl_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);

extern int aotg_hcep_intr_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);

extern int aotg_hcep_xfer_submit(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);

extern u32 ring_trb_virt_to_dma(struct aotg_ring *ring, struct aotg_trb *trb_vaddr);

extern int aotg_ring_enqueue_intr_td(
	struct aotg_hcd *acthcd, struct aotg_hcep *ep,
	struct urb *urb, gfp_t mem_flags);

void aotg_start_ring_transfer(struct aotg_hcd *acthcd, struct aotg_hcep *ep, struct urb *urb);

static inline struct aotg_hcd *hcd_to_aotg(struct usb_hcd *hcd) {
	return ((struct aotg_hcd *) hcd->hcd_priv);
}

static inline struct usb_hcd * aotg_to_hcd(struct aotg_hcd *acthcd) {
	return acthcd->hcd;
}

extern void aotg_clear_all_overflow_irq(struct aotg_hcd *acthcd);

extern void aotg_clear_all_shortpkt_irq(struct aotg_hcd *acthcd);

extern void aotg_clear_all_zeropkt_irq(struct aotg_hcd *acthcd);

extern void aotg_clear_all_hcoutdma_irq(struct aotg_hcd *acthcd);

extern void aotg_enable_irq(struct aotg_hcd *acthcd);

extern void aotg_disable_irq(struct aotg_hcd *acthcd);

extern void aotg_hcep_reset(struct aotg_hcd *acthcd, u8 ep_mask, u8 type_mask);

extern void aotg_powergate_on(struct aotg_hcd *acthcd);

extern void aotg_powergate_off(struct aotg_hcd *acthcd);

extern int aotg_kmem_cache_create(void);

extern void aotg_kmem_cache_destroy(void);

extern struct aotg_td *aotg_alloc_td(gfp_t mem_flags);

extern void aotg_release_td(struct aotg_td *td);


extern int caninos_usb_add_hcd(struct usb_hcd *hcd);

extern void caninos_usb_hcd_remove(struct usb_hcd *hcd);

extern struct usb_hcd *caninos_usb_create_hcd(struct device *dev);



extern void enqueue_trb(struct aotg_ring *ring, u32 buf_ptr, u32 buf_len, u32 token);

extern int ring_enqueue_sg_td(struct aotg_hcd *acthcd, struct aotg_ring *ring, struct aotg_td *td);


void aotg_hcd_release_queue(struct aotg_hcd *acthcd, struct aotg_queue *q);
void aotg_hub_trans_wait_timer(struct timer_list *t);
enum hrtimer_restart aotg_hub_hotplug_timer(struct hrtimer *hrtimer);
void aotg_check_trb_timer(struct timer_list *t);
void aotg_power_onoff(int id, int on_off);
void aotg_hcd_dump_td(struct aotg_ring *ring, struct aotg_td *td);
struct aotg_ring *aotg_alloc_ring(struct aotg_hcd *acthcd,
	struct aotg_hcep *ep, unsigned int num_trbs, gfp_t mem_flags);
void aotg_free_ring(struct aotg_hcd *acthcd, struct aotg_ring *ring);
void enable_overflow_irq(struct aotg_hcd *acthcd, struct aotg_hcep *ep);
void dequeue_td(struct aotg_ring *ring, struct aotg_td *td, int dequeue_flag);
int aotg_ring_enqueue_td(struct aotg_hcd *acthcd,
	struct aotg_ring *ring, struct aotg_td *td);
int aotg_ring_dequeue_td(struct aotg_hcd *acthcd, struct aotg_ring *ring,
	struct aotg_td *td, int dequeue_flag);
int aotg_ring_enqueue_isoc_td(struct aotg_hcd *acthcd,
	struct aotg_ring *ring, struct aotg_td *td);

int aotg_ring_dequeue_intr_td(struct aotg_hcd *acthcd, struct aotg_hcep *ep,
	struct aotg_ring *ring,	struct aotg_td *td);
void aotg_intr_dma_pool_destroy(struct aotg_ring *ring);
void aotg_intr_dma_buf_free(struct aotg_hcd *acthcd, struct aotg_ring *ring);


int is_ring_running(struct aotg_ring *ring);
void aotg_start_ring(struct aotg_ring *ring, u32 addr);
void aotg_stop_ring(struct aotg_ring *ring);
u32 ring_trb_virt_to_dma(struct aotg_ring *ring,
	struct aotg_trb *trb_vaddr);
void aotg_ring_irq_handler(struct aotg_hcd *acthcd);
void aotg_dump_linklist_reg_2(struct aotg_hcd *acthcd, int dmanr);
int intr_finish_td(struct aotg_hcd *acthcd, struct aotg_ring *ring, struct aotg_td *td);
void dequeue_intr_td(struct aotg_ring *ring, struct aotg_td *td);
void aotg_reorder_intr_td(struct aotg_hcep *ep);
int aotg_intr_chg_buf_len(struct aotg_hcd *acthcd, struct aotg_ring *ring, int len);
int aotg_intr_get_finish_trb(struct aotg_ring *ring);
void aotg_reorder_iso_td(struct aotg_hcd *acthcd, struct aotg_ring *ring);
void handle_ring_dma_tx(struct aotg_hcd *acthcd, unsigned int irq_mask);

extern int owl_get_usb_hsdp(unsigned int *usb_hsdp);
#endif /* __LINUX_USB_HOST_AOTG_H */
