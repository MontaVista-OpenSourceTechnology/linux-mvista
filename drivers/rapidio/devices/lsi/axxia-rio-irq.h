/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __AXXIA_RIO_IRQ_H__
#define __AXXIA_RIO_IRQ_H__

/* forward declaration */
struct rio_priv;


#define RIO_MSG_MAX_OB_MBOX_MULTI_ENTRIES  15
#define RIO_MSG_MULTI_SIZE                 0x1000 /* 4Kb */
#define RIO_MSG_SEG_SIZE                   0x0100 /* 256B */
#define RIO_MSG_MAX_MSG_SIZE               RIO_MSG_MULTI_SIZE
#define RIO_MSG_MIN_OB_MSG_SIZE            706    /* 706B - AXM device advisory, defect: AX7-67 */
#define RIO_MSG_MAX_ENTRIES                1024   /* Default Max descriptor
						     table entries for internal
						     descriptor builds */
#define	RIO_MBOX_TO_IDX(mid)		\
	((mid <= RIO_MAX_RX_MBOX_4KB) ? 0 : 1)
#define	RIO_MBOX_TO_BUF_SIZE(mid)		\
	((mid <= RIO_MAX_RX_MBOX_4KB) ? RIO_MSG_MULTI_SIZE : RIO_MSG_SEG_SIZE)
#define	RIO_OUTB_DME_TO_BUF_SIZE(p, did)	\
	((did < p->num_outb_dmes[0]) ? RIO_MSG_MULTI_SIZE : RIO_MSG_SEG_SIZE)

#define DME_MAX_IB_ENGINES          32
#define     RIO_MAX_IB_DME_MSEG		32
#define     RIO_MAX_IB_DME_SSEG	        0
#define DME_MAX_OB_ENGINES          3
#define     RIO_MAX_OB_DME_MSEG		2
#define     RIO_MAX_OB_DME_SSEG	        1

#define RIO_MAX_TX_MBOX             64
#define     RIO_MAX_TX_MBOX_4KB		3
#define     RIO_MAX_TX_MBOX_256B	63
#define RIO_MAX_RX_MBOX             64
#define     RIO_MAX_RX_MBOX_4KB		3
#define     RIO_MAX_RX_MBOX_256B	63

#define RIO_MSG_MAX_LETTER          4


#define RIO_DESC_USED 0		/* Bit index for rio_msg_desc.state */

#define LINK_DOWN_TIMEOUT 0xFFFFFFFFu
#define LINK_DOWN_IRQ_MASK RAB_SRDS_CTRL1_OES_EN

struct rio_reg_range {
	uint32_t start_addr;
	uint32_t access_count;
};

struct rio_msg_desc {
/*	unsigned long state;*/
/*	int desc_no;*/
	void __iomem *msg_virt;
	dma_addr_t msg_phys;
	void *kvirt;
	int last;
};

struct rio_msg_dme {
	spinlock_t lock;
	unsigned long state;
	struct kref kref;
	struct rio_priv *priv;
	struct resource dres;
	int sz;
	int entries;
	int write_idx;
	int read_idx;
	int tx_dme_tmo;
	void *dev_id;
	int dme_no;
	int mbox;
	int letter;
	u32 dme_ctrl;
	struct rio_msg_desc *desc;
	struct rio_desc *descriptors;

#ifdef CONFIG_AXXIA_RIO_STAT
	unsigned int desc_done_count;
	unsigned int desc_error_count;
	unsigned int desc_rio_err_count;
	unsigned int desc_axi_err_count;
	unsigned int desc_tmo_err_count;
#endif
#ifdef CONFIG_AXXIA_RIO_BURST_STAT
	struct rio_msg_dme_burst {
		u32 count;
		ktime_t time;
	} burst;
#endif
} ____cacheline_internodealigned_in_smp;

struct rio_rx_mbox {
	unsigned long state;
	int mbox_no;
	struct kref kref;
	struct rio_mport *mport;
	void **virt_buffer[RIO_MSG_MAX_LETTER];
	void **cookie[RIO_MSG_MAX_LETTER];
	int last_rx_slot[RIO_MSG_MAX_LETTER];
	int next_rx_slot[RIO_MSG_MAX_LETTER];
	int ring_size[RIO_MSG_MAX_LETTER];
	struct rio_msg_dme *me[RIO_MSG_MAX_LETTER][DME_MAX_IB_ENGINES];
	int num_dme[RIO_MSG_MAX_LETTER];
	unsigned int irq_state_mask;
	unsigned int irq_letter_state[RIO_MSG_MAX_LETTER];
	unsigned int num_letter;
	struct hrtimer tmr;
};

struct rio_tx_mbox {
	unsigned long state;
	struct rio_mport *mport;
	int mbox_no;
	int dme_no;
	int ring_size;
	struct rio_msg_dme *me;
	void *dev_id;
	int tx_slot;
#ifdef CONFIG_AXXIA_RIO_STAT
	unsigned int sent_msg_count;
	unsigned int compl_msg_count;
#endif
} ____cacheline_internodealigned_in_smp;

struct rio_tx_dme {
	int	ring_size;
	int	ring_size_free;
	struct rio_msg_dme *me;
	struct hrtimer tmr;
};

#define PW_MSG_WORDS (RIO_PW_MSG_SIZE/sizeof(u32))

struct rio_pw_irq {
	/* Port Write */
	u32 discard_count;
	u32 msg_count;
	u32 msg_wc;
	u32 msg_buffer[PW_MSG_WORDS];
};

#define RIO_IRQ_ENABLED 0
#define RIO_IRQ_ACTIVE  1

#define RIO_DME_MAPPED  1
#define RIO_DME_OPEN    0

#define RIO_MB_OPEN	0
#define RIO_MB_MAPPED	1

struct rio_irq_handler {
	unsigned long state;
/*	struct rio_mport *mport;*/
	u32 irq_enab_reg_addr;
	u32 irq_state_reg_addr;
	u32 irq_state_mask;
	void (*thrd_irq_fn)(struct rio_irq_handler *h/*, u32 state*/);
	void (*release_fn)(struct rio_irq_handler *h);
	void *data;
};

extern unsigned int axxia_hrtimer_delay;
/**********************************************/
/* *********** External Functions *********** */
/**********************************************/

void axxia_rio_port_irq_init(struct rio_mport *mport);
void *axxia_get_inb_message(struct rio_mport *mport, int mbox, int letter,
			      int *sz, int *destid, void **cookie);
int axxia_add_inb_buffer(struct rio_mport *mport, int mbox,
				int letter, void *buf, void *cookie);
int axm56xx_add_inb_buffer(struct rio_mport *mport, int mbox,
				int letter, void *buf, void *cookie);
int axxia_ml_add_inb_buffer(struct rio_mport *mport, int mbox,
				void *buf, void *cookie);
void axxia_close_inb_mbox(struct rio_mport *mport, int mbox);
void axxia_close_inb_mbox_let(struct rio_mport *mport, int mbox,
				int letter_mask,
				void (*release_cb)(void *cookie));
int axxia_open_inb_mbox(struct rio_mport *mport, void *dev_id,
			  int mbox, int entries);
int axxia_open_inb_mbox_let(struct rio_mport *mport, void *dev_id,
			  int mbox, int letter_mask, int entries);
int axxia_open_inb_mbox_let_unsafe(struct rio_mport *mport, void *dev_id,
			  int mbox, int letter_mask, int entries);
int axxia_add_outb_message(struct rio_mport *mport, u16 destid,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len, void *cookie);
int axm56xx_add_outb_message(struct rio_mport *mport, u16 destid,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len, void *cookie);
void axxia_close_outb_mbox(struct rio_mport *mport, int mbox_id);
int axxia_open_outb_mbox(struct rio_mport *mport, void *dev_id, int mbox_id,
			 int entries/*, int prio*/);
int axxia_rio_doorbell_send(struct rio_mport *mport,
			      int index, u16 destid, u16 data);
int axxia_rio_pw_enable(struct rio_mport *mport, int enable);
void axxia_rio_port_get_state(struct rio_mport *mport, int cleanup);
int axxia_rio_port_irq_enable(struct rio_mport *mport);
void axxia_rio_port_irq_disable(struct rio_mport *mport);

int axxia_ml_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			      int mbox_dest, void *buffer, size_t len,
			      void *cookie);
void *axxia_ml_get_inb_message(struct rio_mport *mport, int mbox,
			       void **cookie);
int alloc_irq_handler(
	struct rio_irq_handler *h,
	void *data,
	const char *name);

void release_mbox_resources(struct rio_priv *priv, int mbox_id);
void release_irq_handler(struct rio_irq_handler *h);
void db_irq_handler(struct rio_irq_handler *h, u32 state);
extern int axxia_rio_init_sysfs(struct platform_device *dev);
extern void axxia_rio_release_sysfs(struct platform_device *dev);
int rio_parse_inb_dme_map(struct platform_device *dev, int idx, u32 *inb_map);

#if defined(CONFIG_RAPIDIO_HOTPLUG)

int axxia_rio_port_notify_cb(struct rio_mport *mport,
			       int enable,
			       void (*cb)(struct rio_mport *mport));
int axxia_rio_port_op_state(struct rio_mport *mport);

#endif

#endif /* __AXXIA_RIO_IRQ_H__ */
