/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Author: Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/hwqueue.h>
#include <linux/dmapool.h>
#include <linux/hwqueue.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/timekeeping.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#define DMA_LOOPBACK		BIT(31)
#define DMA_ENABLE		BIT(31)
#define DMA_TEARDOWN		BIT(30)

#define DMA_TX_PRIO_SHIFT	0
#define DMA_RX_PRIO_SHIFT	16
#define DMA_PRIO_MASK		BITS(3)
#define DMA_PRIO_DEFAULT	0
#define DMA_RX_TIMEOUT_DEFAULT	17500 /* cycles */
#define DMA_RX_TIMEOUT_MASK	BITS(16)
#define DMA_RX_TIMEOUT_SHIFT	0

#define CHAN_HAS_EPIB		BIT(30)
#define CHAN_HAS_PSINFO		BIT(29)
#define CHAN_ERR_RETRY		BIT(28)
#define CHAN_TYPE_HOST          0
#define CHAN_TYPE_SHIFT         26
#define CHAN_RETQMGR_SHIFT      12

#define DMA_TIMEOUT		1000	/* msecs */

#define DESC_HAS_EPIB		BIT(31)
#define DESC_PSLEN_MASK		BITS(6)
#define DESC_PSLEN_SHIFT	24
#define DESC_PSFLAGS_MASK	BITS(4)
#define DESC_PSFLAGS_SHIFT	16
#define DESC_RETQ_MASK		BITS(14)
#define DESC_RETQ_SHIFT		0
#define DESC_RETQMGR_SHIFT	12
#define DESC_LEN_MASK		BITS(22)
#define DESC_PKTTYPE_MASK       BITS(5)
#define DESC_PKTTYPE_SHIFT      25
#define DESC_EFLAGS_SHIFT	20
#define DESC_EFLAGS_MASK	BITS(4)
#define DESC_SRC_TAG_LO_SHIFT	16
#define DESC_SRC_TAG_LO_MASK	BITS(8)


#define DMA_DEFAULT_NUM_DESCS	128
#define DMA_DEFAULT_PRIORITY	DMA_PRIO_MED_L
#define DMA_DEFAULT_TAG_INFO	0

#define HWQUEUE_HAS_PACKET_SIZE	BIT(31)

static void desc_send_stats(struct work_struct *work);
static DECLARE_WORK(pktdma_desc_logger, desc_send_stats);

static int pktdma_debug_show(struct seq_file *s, void *v);
static struct dentry *stats_dentry;
static struct dentry *stats_dentry_en;
static int pktdma_set(void *data, u64 val);

static int pktdma_log_enable = 0;

DEFINE_SPINLOCK(stats_lock);

static void stat_timer_handler(unsigned long data);
int stat_time_interval = 60000;
DEFINE_TIMER(stat_timer, stat_timer_handler, 0 ,0);

enum keystone_dma_tx_priority {
	DMA_PRIO_HIGH	= 0,
	DMA_PRIO_MED_H,
	DMA_PRIO_MED_L,
	DMA_PRIO_LOW,
};

struct reg_global {
	u32	revision;
	u32	perf_control;
	u32	emulation_control;
	u32	priority_control;
	u32	qm_base_address[4];
};

struct reg_chan {
	u32	control;
	u32	mode;
	u32	__rsvd[6];
};

struct reg_tx_sched {
	u32	prio;
};

struct reg_rx_flow {
	u32	control;
	u32	tags;
	u32	tag_sel;
	u32	fdq_sel[2];
	u32	thresh[3];
};

#define BUILD_CHECK_REGS()						\
	do {								\
		BUILD_BUG_ON(sizeof(struct reg_global)   != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_chan)     != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_rx_flow)  != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_tx_sched) !=  4);	\
	} while (0)

struct keystone_hw_priv {
	struct keystone_dma_desc	*desc;
	unsigned long			 pad[3];
};

struct keystone_hw_desc {
	u32	desc_info;
	u32	tag_info;
	u32	packet_info;
	u32	buff_len;
	u32	buff;
	u32	next_desc;
	u32	orig_len;
	u32	orig_buff;
	u32	epib[4];
	u32	psdata[16];
	struct keystone_hw_priv priv;
} ____cacheline_aligned;

#define DESC_MAX_SIZE	offsetof(struct keystone_hw_desc, priv)
#define DESC_MIN_SIZE	offsetof(struct keystone_hw_desc, epib)

enum keystone_chan_state {
	/* stable states */
	CHAN_STATE_OPENED,
	CHAN_STATE_PAUSED,
	CHAN_STATE_CLOSED,

	/* transient states */
	CHAN_STATE_OPENING,
	CHAN_STATE_CLOSING,
};
#define chan_state_is_transient(state) ((state) >= CHAN_STATE_OPENING)

struct keystone_dma_desc {
	unsigned			 free_next;
	unsigned			 size;
	enum dma_status			 status;
	unsigned long			 options;
	unsigned			 sg_len;
	struct scatterlist		*sg;
	struct keystone_hw_desc		*hwdesc;
	struct dma_async_tx_descriptor	 adesc;
	unsigned			 orig_size;
	dma_addr_t			 hwdesc_dma_addr;
	unsigned			 hwdesc_dma_size;
	u64				 desc_timestamp;
	u32				 queue_id;
} ____cacheline_aligned;

struct desc_stats {
	u32				max;
	u32				min;
	struct keystone_dma_desc	*desc;
	u32				queue_id;
	struct list_head		chan_le;
	struct keystone_dma_chan	*chan;
};

#define LOG_SIZE 100
static struct desc_stats desc_push_stats = {.min = 500000};
static struct desc_stats *stats_p = &desc_push_stats;
static struct desc_stats desc_stats_log[LOG_SIZE];
static int log_rp = 0;
static int log_wp = 0;

#define desc_dev(d)		chan_dev((d)->chan)
#define desc_free_index(f)	((f) & 0xffff)
#define desc_free_token(f, t)	((f) | ((t) << 16))

struct keystone_dma_device {
	struct dma_device		 engine;
	struct clk			*clk;
	bool				 big_endian, loopback, enable_all;
	unsigned			 tx_priority, rx_priority, rx_timeout;
	unsigned			 logical_queue_managers;
	unsigned			 queues_per_queue_manager;
	unsigned			 qm_base_address[4];
	struct reg_global __iomem	*reg_global;
	struct reg_chan __iomem		*reg_tx_chan;
	struct reg_rx_flow __iomem	*reg_rx_flow;
	struct reg_chan __iomem		*reg_rx_chan;
	struct reg_tx_sched __iomem	*reg_tx_sched;
	unsigned			 max_rx_chan, max_tx_chan;
	unsigned			 max_rx_flow;
	bool				 debug;
	atomic_t			 in_use;
};
#define from_dma(dma)	container_of(dma, struct keystone_dma_device, engine)
#define to_dma(dma)	(&(dma)->engine)
#define dma_dev(dma)	((dma)->engine.dev)

struct keystone_rxpool {
	atomic_t		deficit;
	unsigned		pool_depth;
	unsigned		buffer_size;
	unsigned		filler;
};


struct keystone_dma_chan {
	enum dma_transfer_direction	 direction;
	unsigned			 desc_shift;
	u32				 num_descs;
	atomic_t			 state;
	atomic_t			 free_token;
	atomic_t			 free_first;
	atomic_t			 n_used_descs;
	unsigned			 big_endian;
	struct keystone_dma_device	*dma;
	struct dma_chan			 achan;
	struct hwqueue			*q_submit[KEYSTONE_QUEUES_PER_CHAN];
	struct hwqueue			*q_complete;
	struct hwqueue			*q_pool;
	void				*descs;
	int				 qnum_submit[KEYSTONE_QUEUES_PER_CHAN];
	int				 qnum_complete;
	int				 dest_queue_manager;
	struct dma_notify_info		 notify_info;

	/* registers */
	struct reg_chan __iomem		*reg_chan;
	struct reg_tx_sched __iomem	*reg_tx_sched;
	struct reg_rx_flow __iomem	*reg_rx_flow;

	/* configuration stuff */
	enum keystone_dma_tx_priority	 tx_sched_priority;
	int				 qcfg_submit, qcfg_complete;
	unsigned			 channel, flow;
	const char			*qname_pool;
	u32				 tag_info;
	bool				 rx_err_retry;
	u8				 debug;

	unsigned int			 scatterlist_size;
	dma_rxpool_alloc_fn		 rxpool_allocator;
	dma_rxpool_free_fn		 rxpool_destructor;
	void				*rxpool_param;
	unsigned			 rxpool_count;
	struct keystone_rxpool		 rxpools[KEYSTONE_QUEUES_PER_CHAN];
        struct desc_stats                desc_push_stats;
};

static struct list_head chan_stats_list;

#define from_achan(ch)	container_of(ch, struct keystone_dma_chan, achan)
#define to_achan(ch)	(&(ch)->achan)
#define chan_dev(ch)	(&to_achan(ch)->dev->device)
#define chan_id(ch)	(to_achan(ch)->chan_id)
#define chan_name(ch)	((char *)(ch)->achan.private)
#define chan_dbg(ch, format, arg...)				\
	do {							\
		if ((ch)->debug)				\
			dev_dbg(chan_dev(ch), format, ##arg);	\
	} while (0)
#define chan_vdbg(ch, format, arg...)				\
	do {							\
		if ((ch)->debug > 1)				\
			dev_dbg(chan_dev(ch), format, ##arg);	\
	} while (0)

bool keystone_dma_name_flt(struct dma_chan *chan, void *name)
{
	return !strcmp(chan->private, name);
}
EXPORT_SYMBOL_GPL(keystone_dma_name_flt);

static void stat_timer_handler(unsigned long data)
{
	if (pktdma_log_enable) {
		pr_info("PKTDMA_LOG -> pushed desc %p min (ns) = %u max (ns) = %u queue_id = %u \n",
			stats_p->desc, stats_p->min, stats_p->max, stats_p->queue_id);
		mod_timer(&stat_timer, jiffies + msecs_to_jiffies(stat_time_interval));
	}
}


static void desc_send_stats(struct work_struct *work)
{
	int log_cnt = 0;
	struct desc_stats *stats_log;

	while ((log_rp != log_wp) && (log_cnt < LOG_SIZE)) {
		stats_log = &desc_stats_log[log_rp];

		pr_info("PKTDMA_LOG WK_Q -> pushed desc %p DESC min = %u DESC max = %u queue_id = %u log_cnt = %d\n",
			stats_log->desc, stats_log->min, stats_log->max, stats_log->queue_id, log_cnt);

		log_rp = (log_rp + 1) % LOG_SIZE;

		log_cnt++;
	}

	if (log_cnt == LOG_SIZE)
		pr_info("PKTDMA_LOG WK_Q -> WARNING STATS LOG OVERFLOW !!! \n");

}


/**
 * dev_to_dma_chan - convert a device pointer to the its sysfs container object
 * @dev - device node
 */
static inline struct dma_chan *dev_to_dma_chan(struct device *dev)
{
	struct dma_chan_dev *chan_dev;

	chan_dev = container_of(dev, typeof(*chan_dev), device);
	return chan_dev->chan;
}

static inline struct keystone_dma_desc *
desc_from_index(struct keystone_dma_chan *chan, unsigned idx)
{
	if (unlikely(idx > chan->num_descs))
		return NULL;
	return chan->descs + (idx << chan->desc_shift);
}

static inline unsigned int
desc_to_index(struct keystone_dma_chan *chan, struct keystone_dma_desc *desc)
{
	unsigned offset = (void *)desc - chan->descs;
	BUG_ON(offset & ((1 << chan->desc_shift) - 1));
	return offset >> chan->desc_shift;
}

static inline struct keystone_dma_desc *
desc_from_hwdesc(struct keystone_hw_desc *hwdesc)
{
	return hwdesc->priv.desc;
}

static inline struct keystone_hw_desc *
desc_to_hwdesc(struct keystone_dma_desc *desc)
{
	return desc->hwdesc;
}

static inline struct keystone_dma_desc *
desc_from_adesc(struct dma_async_tx_descriptor *adesc)
{
	return container_of(adesc, struct keystone_dma_desc, adesc);
}

static inline struct dma_async_tx_descriptor *
desc_to_adesc(struct keystone_dma_desc *desc)
{
	return &desc->adesc;
}

static inline u32 hwval_to_host(struct keystone_dma_chan *chan, u32 hwval)
{
	if (unlikely(chan->big_endian))
		return be32_to_cpu(hwval);
	else
		return le32_to_cpu(hwval);
}

static inline u32 host_to_hwval(struct keystone_dma_chan *chan, u32 hostval)
{
	if (unlikely(chan->big_endian))
		return cpu_to_be32(hostval);
	else
		return cpu_to_le32(hostval);
}

static inline u32 desc_get_len(struct keystone_dma_chan *chan,
			       struct keystone_hw_desc *hwdesc)
{
	return hwval_to_host(chan, hwdesc->desc_info) & DESC_LEN_MASK;
}

static inline struct keystone_dma_desc *desc_get(struct keystone_dma_chan *chan)
{
	struct keystone_dma_desc *desc;
	unsigned first, next, token, result;

	atomic_inc(&chan->n_used_descs);

	token = atomic_inc_return(&chan->free_token);
	first = atomic_read(&chan->free_first);

	for (;;) {
		desc = desc_from_index(chan, desc_free_index(first));
		if (!desc)
			return NULL;
		next = desc_free_token(desc->free_next, token);
		result = atomic_cmpxchg(&chan->free_first, first, next);
		if (likely(result == first))
			break;
		first = result;
	}

	return desc;
}

static inline void desc_put(struct keystone_dma_chan *chan,
			    struct keystone_dma_desc *desc)
{
	unsigned index, first, next, token, result;

	desc->status = DMA_ERROR;

	token = atomic_inc_return(&chan->free_token);
	index = desc_to_index(chan, desc);
	next = desc_free_token(index, token);

	first = atomic_read(&chan->free_first);

	for (;;) {
		desc->free_next = desc_free_index(first);
		result = atomic_cmpxchg(&chan->free_first, first, next);
		if (likely(result == first))
			break;
		first = result;
	}

	atomic_dec(&chan->n_used_descs);
}

/*
 * fill up descriptor fields without letting gcc generate horrid branch ridden
 * code
 */
static inline void desc_fill(struct keystone_dma_chan *chan,
			     struct keystone_hw_desc *hwdesc,
			     u32 desc_info,
			     u32 tag_info,
			     u32 packet_info,
			     u32 buff_len,
			     u32 buff,
			     u32 next_desc,
			     u32 orig_len,
			     u32 orig_buff)
{
	if (unlikely(chan->big_endian)) {
		hwdesc->desc_info    = cpu_to_be32(desc_info);
		hwdesc->tag_info     = cpu_to_be32(tag_info);
		hwdesc->packet_info  = cpu_to_be32(packet_info);
		hwdesc->buff_len     = cpu_to_be32(buff_len);
		hwdesc->buff         = cpu_to_be32(buff);
		hwdesc->next_desc    = cpu_to_be32(next_desc);
		hwdesc->orig_len     = cpu_to_be32(orig_len);
		hwdesc->orig_buff    = cpu_to_be32(orig_buff);
	} else {
		hwdesc->desc_info    = cpu_to_le32(desc_info);
		hwdesc->tag_info     = cpu_to_le32(tag_info);
		hwdesc->packet_info  = cpu_to_le32(packet_info);
		hwdesc->buff_len     = cpu_to_le32(buff_len);
		hwdesc->buff         = cpu_to_le32(buff);
		hwdesc->next_desc    = cpu_to_le32(next_desc);
		hwdesc->orig_len     = cpu_to_le32(orig_len);
		hwdesc->orig_buff    = cpu_to_le32(orig_buff);
	}
}

static inline void desc_copy(struct keystone_dma_chan *chan,
			     u32 *out, u32 *in, int words)
{
	int i;

	if (unlikely(chan->big_endian)) {
		for (i = 0; i < words; i++)
			*out++ = cpu_to_be32(*in++);
	} else {
		for (i = 0; i < words; i++)
			*out++ = cpu_to_le32(*in++);
	}
}

static const char *chan_state_str(enum keystone_chan_state state)
{
	static const char * const state_str[] = {
		/* stable states */
		[CHAN_STATE_OPENED]	= "opened",
		[CHAN_STATE_PAUSED]	= "paused",
		[CHAN_STATE_CLOSED]	= "closed",

		/* transient states */
		[CHAN_STATE_OPENING]	= "opening",
		[CHAN_STATE_CLOSING]	= "closing",
	};

	if (state < 0 || state >= ARRAY_SIZE(state_str))
		return state_str[CHAN_STATE_CLOSED];
	else
		return state_str[state];
}

static enum keystone_chan_state chan_get_state(struct keystone_dma_chan *chan)
{
	return atomic_read(&chan->state);
}

static int chan_set_state(struct keystone_dma_chan *chan,
			  enum keystone_chan_state old,
			  enum keystone_chan_state new)
{
	enum keystone_chan_state cur;

	cur = atomic_cmpxchg(&chan->state, old, new);
	if (likely(cur == old)) {
		chan_vdbg(chan, "channel state change from %s to %s\n",
			  chan_state_str(old), chan_state_str(new));
		return 0;
	}

	dev_warn(chan_dev(chan), "state %s on switch to %s, expected %s\n",
		 chan_state_str(cur), chan_state_str(new), chan_state_str(old));
	return -EINVAL;
}

static inline bool __chan_is_alive(enum keystone_chan_state state)
{
	return (state == CHAN_STATE_PAUSED || state == CHAN_STATE_OPENED);
}

static inline bool chan_is_alive(struct keystone_dma_chan *chan)
{
	return __chan_is_alive(chan_get_state(chan));
}

static inline void __hwdesc_dump(struct keystone_dma_chan *chan,
				 struct keystone_hw_desc *hwdesc)
{
	unsigned long *data = (unsigned long *)hwdesc;

	chan_vdbg(chan, "\tdesc_info: %x\n",	hwdesc->desc_info);
	chan_vdbg(chan, "\ttag_info: %x\n",	hwdesc->tag_info);
	chan_vdbg(chan, "\tpacket_info: %x\n",	hwdesc->packet_info);
	chan_vdbg(chan, "\tbuff_len: %x\n",	hwdesc->buff_len);
	chan_vdbg(chan, "\tbuff: %x\n",		hwdesc->buff);
	chan_vdbg(chan, "\tnext_desc: %x\n",	hwdesc->next_desc);
	chan_vdbg(chan, "\torig_len: %x\n",	hwdesc->orig_len);
	chan_vdbg(chan, "\torig_buff: %x\n",	hwdesc->orig_buff);

	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x00], data[0x01], data[0x02], data[0x03],
		  data[0x04], data[0x05], data[0x06], data[0x07]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x08], data[0x09], data[0x0a], data[0x0b],
		  data[0x0c], data[0x0d], data[0x0e], data[0x0f]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x10], data[0x11], data[0x12], data[0x13],
		  data[0x14], data[0x15], data[0x16], data[0x17]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x18], data[0x19], data[0x1a], data[0x1b],
		  data[0x1c], data[0x1d], data[0x1e], data[0x1f]);
}

static inline void hwdesc_dump(struct keystone_dma_chan *chan,
			       struct keystone_hw_desc *hwdesc, const char *why)
{
	chan_vdbg(chan, "descriptor dump (%s): desc %p\n", why, hwdesc);
	__hwdesc_dump(chan, hwdesc);
}

static void desc_dump(struct keystone_dma_chan *chan,
		      struct keystone_dma_desc *desc, const char *why)
{
	struct keystone_hw_desc *hwdesc = desc_to_hwdesc(desc);

	chan_vdbg(chan, "%s: desc %p\n", why, hwdesc);
	__hwdesc_dump(chan, hwdesc);
}

static bool chan_should_process(struct keystone_dma_chan *chan, bool in_poll)
{
	enum keystone_chan_state state = chan_get_state(chan);

	/* only process paused channels in poll */
	if (in_poll)
		return (state == CHAN_STATE_PAUSED);
	return (state == CHAN_STATE_OPENED || state == CHAN_STATE_CLOSING);
}

#define dma_to_page(dma, dev) (pfn_to_page(dma_to_pfn(dev,dma)))

static struct keystone_dma_desc *chan_complete_one(
	struct keystone_dma_chan *chan,
	struct hwqueue *queue)
{
	struct keystone_hw_desc *hwdesc;
	struct keystone_dma_desc *desc = NULL;
	struct scatterlist *sg;
	dma_addr_t dma;
	unsigned size;
	unsigned sg_retlen = 0;
	unsigned packet_size = 0;
	unsigned accum_size = 0;
	u32 *data;
	int len;

	for (;;) {
		dma = hwqueue_pop(queue, &size, NULL, 0);
		if (!dma) {
			chan_dbg(chan, "processing stopped, no desc\n");
			return NULL;
		}

		/* Unmap the Host Packet Descriptor */
		hwdesc = hwqueue_unmap(queue, dma, size);
		if (!hwdesc) {
			/*
			 * This can occur when a DSP pushes a bogus descriptor
			 * into an ARM queue. The proper response is to report
			 * the problem and skip to the next packet in the queue.
			 */
			dev_warn(chan_dev(chan),
				 "failed to unmap descriptor 0x%08x\n",
				 dma);
		} else
			break;
	}

	desc = desc_from_hwdesc(hwdesc);
	prefetchw(desc);

	/* Keep time when desc was popped for hold time stats */
	desc->desc_timestamp = ktime_get_real_ns();

	chan_vdbg(chan, "popped desc %p hw %p from queue %d cpu time (ns) = %llu ns\n",
		  desc, hwdesc, hwqueue_get_id(queue), desc->desc_timestamp);

        desc->queue_id = hwqueue_get_id(queue);

	hwdesc_dump(chan, hwdesc, "complete");

	sg_retlen = 0;
	packet_size = 0;
	accum_size = 0;
	sg = desc->sg;

	if (likely((desc->options & DMA_HAS_EPIB))) {
		len  = sg_dma_len(sg) / sizeof(u32);
		data = sg_virt(sg);
		sg++;
		sg_retlen++;
		desc_copy(chan, data, hwdesc->epib, len);
	}

	if (chan->direction == DMA_DEV_TO_MEM) {
		packet_size = hwval_to_host(chan, hwdesc->desc_info) & BITS(22);

		if (likely(desc->options & DMA_HAS_PSINFO)) {
			len  = sg_dma_len(sg) / sizeof(u32);
			data = sg_virt(sg);
			sg++;
			sg_retlen++;
			desc_copy(chan, data, hwdesc->psdata, len);
		}

		if ((desc->options & DMA_HAS_EFLAGS)) {
			data = sg_virt(sg);
			sg++;
			sg_retlen++;
			*data =	(hwval_to_host(chan, hwdesc->packet_info) >>
				 DESC_EFLAGS_SHIFT) & DESC_EFLAGS_MASK;
		}
	}

	/* Process the completed descriptor chain */
	for (;;) {
		struct keystone_dma_desc *this_desc = desc_from_hwdesc(hwdesc);

		/* Populate the the scatterlist from the descriptors */
		if (chan->direction == DMA_DEV_TO_MEM) {
			dma_addr_t bufaddr = hwval_to_host(chan, hwdesc->buff);
			unsigned buflen = hwval_to_host(chan, hwdesc->buff_len) & BITS(22);
			unsigned q_num = hwval_to_host(chan, hwdesc->orig_len) >> 28;

			WARN((q_num >= chan->rxpool_count),
			     "Invalid q_num %d\n", q_num);
			atomic_inc(&chan->rxpools[q_num].deficit);

			if (sg_retlen < chan->scatterlist_size) {
				sg_set_page(sg, dma_to_page(bufaddr,
							    dma_dev(chan->dma)),
					    buflen, offset_in_page(bufaddr));
				sg_dma_address(sg) = bufaddr;
				accum_size += buflen;
				sg++;
			} else {
				chan->rxpool_destructor(
					chan->rxpool_param,
					q_num, chan->rxpools[q_num].buffer_size,
					desc_to_adesc(this_desc));
			}

			sg_retlen++;
		}

		dma = hwval_to_host(chan, hwdesc->next_desc);

		/* If this isn't the first descriptor, free it */
		if (this_desc != desc)
			desc_put(chan, this_desc);

		/* zero next_desc indicates end of the chain */
		if (!dma)
			break;

		/* Unmap the next descriptor in the chain */
		hwdesc = hwqueue_unmap(queue, dma, DESC_MIN_SIZE);
		if (!hwdesc) {
			dev_warn(chan_dev(chan),
				 "unable to unmap descriptor 0x%08x\n",
				 dma);
			break;
		}

		chan_vdbg(chan, "chained desc %p hw %p from queue %d\n",
			  desc, hwdesc, hwqueue_get_id(queue));
		hwdesc_dump(chan, hwdesc, "complete");
	}

	if (chan->direction == DMA_DEV_TO_MEM) {
		/* Mark the last filled-in entry as the end of the scatterlist */
		sg_mark_end(sg-1);

		/* Report scatterlist overflow */
		if (sg_retlen > chan->scatterlist_size) {
			dev_warn(chan_dev(chan), "scatterlist overflow: %d > %d\n",
				 sg_retlen, chan->scatterlist_size);
		}

		if (packet_size != accum_size) {
			dev_warn(chan_dev(chan),
				 "Packet size %u not equal to sum of fragments %u\n",
				 packet_size, accum_size);
		}
	}

	return desc;
}

static int chan_complete(struct keystone_dma_chan *chan, struct hwqueue *queue,
			 enum dma_status status, int budget, bool in_poll)
{
	struct dma_async_tx_descriptor *adesc;
	struct keystone_dma_desc *desc;
	int packets = 0;

	while (budget < 0 || packets < budget) {

		if (!chan_should_process(chan, in_poll))
			break;

		desc = chan_complete_one(chan, queue);
		if (!desc)
			break;

		desc->status = status;

		adesc = desc_to_adesc(desc);
		adesc->callback(adesc->callback_param);

		/* Free the FIRST descriptor in the chain */
		desc_put(chan, desc);

		packets++;
	}

	return packets;
}

static void chan_complete_callback(void *arg)
{
	struct keystone_dma_chan *chan = arg;

	if (chan->notify_info.fn) {
		chan_dbg(chan, "notify - notifying user\n");
		chan->notify_info.fn(to_achan(chan), chan->notify_info.fn_arg);
	} else {
		chan_dbg(chan, "notify - processing packets\n");
		chan_complete(chan, chan->q_complete, DMA_COMPLETE, -1, false);
		chan_dbg(chan, "notify - processing complete\n");
	}
}

dma_cookie_t chan_submit(struct dma_async_tx_descriptor *adesc)
{
	struct keystone_dma_desc *desc = desc_from_adesc(adesc);
	struct keystone_dma_chan *chan = from_achan(adesc->chan);
	struct keystone_hw_desc *hwdesc = desc_to_hwdesc(desc);
	int ret;

	if (unlikely(!chan_is_alive(chan)))
		return -EINVAL;

	if (chan->direction != DMA_MEM_TO_DEV)
		return -EINVAL;

	desc->status = DMA_IN_PROGRESS;

	hwdesc_dump(chan, hwdesc, "submit");
	chan_vdbg(chan, "pushing desc %p to queue %d\n",
		  hwdesc, hwqueue_get_id(chan->q_submit[0]));

	ret = hwqueue_push(chan->q_submit[0], desc->hwdesc_dma_addr,
			   desc->hwdesc_dma_size,
			   ((hwdesc->desc_info & BITS(17)) |
			    HWQUEUE_HAS_PACKET_SIZE));
	if (unlikely(ret < 0))
		return ret;
	else
		return adesc->cookie;
}

static struct hwqueue *chan_open_pool(struct keystone_dma_chan *chan)
{
	struct hwqueue *q;

	q = hwqueue_open(chan->qname_pool, HWQUEUE_BYNAME, O_RDWR | O_NONBLOCK);
	if (IS_ERR(q))
		dev_err(chan_dev(chan), "error %ld opening %s pool queue\n",
			PTR_ERR(q), chan->qname_pool);
	return q;
}

static struct hwqueue *chan_open_queue(struct keystone_dma_chan *chan,
				       unsigned id, unsigned flags,
				       const char *type)
{
	struct hwqueue *q;
	char name[32];

	/* always non blocking */
	flags |= O_NONBLOCK;

	scnprintf(name, sizeof(name), "%s:%s:%s", dev_name(chan_dev(chan)),
		  chan_name(chan), type);
	q = hwqueue_open(name, id, flags);
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "error %ld opening %s queue\n",
			PTR_ERR(q), type);
		return q;
	}
	return q;
}

static void chan_destroy_queues(struct keystone_dma_chan *chan)
{
	int i;

	if (chan->q_complete) {
		hwqueue_set_notifier(chan->q_complete, NULL, NULL);
		hwqueue_set_monitor(chan->q_complete, NULL);
		hwqueue_close(chan->q_complete);
	}
	chan->q_complete = NULL;
	chan->qnum_complete = 0;
	chan->dest_queue_manager = 0;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN; ++i) {
		if (chan->q_submit[i])
			hwqueue_close(chan->q_submit[i]);
		chan->q_submit[i] = NULL;
		chan->qnum_submit[i] = 0;
	}

	if (chan->q_pool)
		hwqueue_close(chan->q_pool);
	chan->q_pool = NULL;
}

static int chan_setup_queues(struct keystone_dma_chan *chan)
{
	struct keystone_dma_device *dma = chan->dma;
	unsigned flags = O_RDWR;
	struct hwqueue *q;
	int ret = 0;
	int i = 0;
	struct hwqueue_monitor_config monitor_cfg;

	/* open pool queue */
	q = chan_open_pool(chan);
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "failed to open pool queue (%ld)\n",
			PTR_ERR(q));
		goto fail;
	}
	chan->q_pool = q;

	for (i = 0; i < chan->rxpool_count; ++i) {
		chan->qnum_submit[i] = 0;
		chan->q_submit[i] = 0;
	}

	if (chan->direction == DMA_DEV_TO_MEM) {
		/* Open receive free queues */
		for (i = 0; i < chan->rxpool_count; ++i) {
			char qname[16];
			scnprintf(qname, sizeof(qname), "rxfree-%d", i);
			q = chan_open_queue(chan, HWQUEUE_ANY, flags | O_EXCL,
					    qname);
			if (IS_ERR(q)) {
				dev_err(chan_dev(chan), "failed to open %s queue (%ld)\n",
					qname, PTR_ERR(q));
				goto fail;
			}
			chan->qnum_submit[i] = hwqueue_get_id(q);
			chan->q_submit[i] = q;
		}
	} else {
		/* Open submit queue */
		q = chan_open_queue(chan, chan->qcfg_submit, flags, "submit");
		if (IS_ERR(q)) {
			dev_err(chan_dev(chan), "failed to open submit queue (%ld)\n",
				PTR_ERR(q));
			goto fail;
		}
		chan->qnum_submit[0] = hwqueue_get_id(q);
		chan->q_submit[0] = q;
	}

	/* open completion queue */
	flags = O_RDONLY | O_HIGHTHROUGHPUT;
	q = chan_open_queue(chan, chan->qcfg_complete, flags | O_EXCL,
			    "complete");
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "failed to open complete queue (%ld)\n",
			PTR_ERR(q));
		goto fail;
	}
	chan->qnum_complete = hwqueue_get_id(q);
	chan->q_complete = q;
	chan->dest_queue_manager = chan->qnum_complete /
					dma->queues_per_queue_manager;

	/* setup queue notifier */
	ret = hwqueue_set_notifier(q, chan_complete_callback, chan);
	if (ret < 0) {
		dev_err(chan_dev(chan), "failed to setup queue notify (%d)\n",
			ret);
		goto fail;
	}

	/* setup rx queue monitoring */
	if (chan->direction == DMA_DEV_TO_MEM) {

		memset(&monitor_cfg, 0, sizeof(monitor_cfg));

		for (i = 0; i < chan->rxpool_count; ++i) {
			if (chan->qnum_submit[i] != 0)
				monitor_cfg.fdq_arg[i] = chan->q_submit[i];
			else
				monitor_cfg.fdq_arg[i] = 0;
		}

		monitor_cfg.fn = knav_qmssm_event_callback;

		ret = hwqueue_set_monitor(chan->q_complete, &monitor_cfg);

		if (ret < 0)
			dev_err(chan_dev(chan), "failed to setup qmssm (%d) queue %d\n",
					ret, chan->qnum_complete);
	}

	if (chan->direction == DMA_DEV_TO_MEM) {
		dev_info(chan_dev(chan), "opened queues: submit %d/%d/%d/%d, complete %d\n",
			 chan->qnum_submit[0], chan->qnum_submit[1],
			 chan->qnum_submit[2], chan->qnum_submit[3],
			 chan->qnum_complete);
	} else {
		dev_info(chan_dev(chan), "opened queues: submit %d, complete %d\n",
			 chan->qnum_submit[0], chan->qnum_complete);
	}

	return 0;

fail:
	chan_destroy_queues(chan);
	return IS_ERR(q) ? PTR_ERR(q) : ret;
}

static int chan_start(struct keystone_dma_chan *chan)
{
	u32 v[8] = {0};

	if (chan->reg_chan) {
		__raw_writel(0, &chan->reg_chan->mode);
		__raw_writel(DMA_ENABLE, &chan->reg_chan->control);
	}

	if (chan->reg_tx_sched) {
		__raw_writel(chan->tx_sched_priority,
				&chan->reg_tx_sched->prio);
	}

	if (chan->reg_rx_flow) {
		v[0]  = CHAN_HAS_EPIB | CHAN_HAS_PSINFO;
		if (chan->rx_err_retry)
			v[0] |= CHAN_ERR_RETRY;
		v[0] |= chan->qnum_complete |
				(chan->dest_queue_manager << CHAN_RETQMGR_SHIFT) |
				(CHAN_TYPE_HOST << CHAN_TYPE_SHIFT);
		__raw_writel(v[0], &chan->reg_rx_flow->control);

		__raw_writel(v[1], &chan->reg_rx_flow->tags);
		__raw_writel(v[2], &chan->reg_rx_flow->tag_sel);

		if (chan->direction == DMA_DEV_TO_MEM) {
			switch (chan->rxpool_count) {
				case 1:
				v[3] = chan->qnum_submit[0] << 16;
				v[3] |= chan->qnum_submit[0];
				__raw_writel(v[3], &chan->reg_rx_flow->fdq_sel[0]);
				v[4] =  chan->qnum_submit[0] << 16;
				v[4] |= chan->qnum_submit[0];
				__raw_writel(v[4], &chan->reg_rx_flow->fdq_sel[1]);
				break;
				case 2:
				v[3] =  chan->qnum_submit[0] << 16;
				v[3] |= chan->qnum_submit[1];
				__raw_writel(v[3], &chan->reg_rx_flow->fdq_sel[0]);
				v[4] =  chan->qnum_submit[1] << 16;
				v[4] |= chan->qnum_submit[1];
				__raw_writel(v[4], &chan->reg_rx_flow->fdq_sel[1]);
				break;
				case 3:
				v[3] =  chan->qnum_submit[0] << 16;
				v[3] |= chan->qnum_submit[1];
				__raw_writel(v[3], &chan->reg_rx_flow->fdq_sel[0]);
				v[4] =  chan->qnum_submit[2] << 16;
				v[4] |= chan->qnum_submit[2];
				__raw_writel(v[4], &chan->reg_rx_flow->fdq_sel[1]);
				break;
				case 4:
				v[3] =  chan->qnum_submit[0] << 16;
				v[3] |= chan->qnum_submit[1];
				__raw_writel(v[3], &chan->reg_rx_flow->fdq_sel[0]);
				v[4] =  chan->qnum_submit[2] << 16;
				v[4] |= chan->qnum_submit[3];
				__raw_writel(v[4], &chan->reg_rx_flow->fdq_sel[1]);
				break;
			}
		} else {
			v[3] = chan->qnum_submit[0] << 16;
			__raw_writel(v[3], &chan->reg_rx_flow->fdq_sel[0]);
			__raw_writel(v[4], &chan->reg_rx_flow->fdq_sel[1]);
		}

		__raw_writel(v[5], &chan->reg_rx_flow->thresh[0]);
		__raw_writel(v[6], &chan->reg_rx_flow->thresh[1]);
		__raw_writel(v[7], &chan->reg_rx_flow->thresh[2]);
	}

	if (chan->direction == DMA_DEV_TO_MEM)
		dev_info(dma_dev(chan->dma), "channel started %s flow-%d A:0x%X B:0x%X C:0x%X D:0x%X E:0x%X F:0x%X G:0x%X H:0x%X",
				chan_name(chan), chan->flow, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
	else
		dev_info(dma_dev(chan->dma), "channel started");

	return 0;
}

static int chan_teardown(struct keystone_dma_chan *chan)
{
	unsigned long end, value;

	if (!chan->reg_chan)
		return 0;

	/* indicate teardown */
	__raw_writel(DMA_TEARDOWN, &chan->reg_chan->control);

	/* wait for the dma to shut itself down */
	end = jiffies + msecs_to_jiffies(DMA_TIMEOUT);
	do {
		value = __raw_readl(&chan->reg_chan->control);
		if ((value & DMA_ENABLE) == 0)
			break;
	} while (time_after(end, jiffies));

	if (__raw_readl(&chan->reg_chan->control) & DMA_ENABLE) {
		dev_err(chan_dev(chan), "timeout waiting for tx teardown\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void chan_purge_rxfree(struct keystone_dma_chan *chan, int index,
		struct hwqueue *q)
{
	struct dma_async_tx_descriptor *adesc;
	struct keystone_hw_desc *hwdesc;
	struct keystone_dma_desc *desc;
	dma_addr_t dma;
	unsigned size;

	for (;;) {
		dma = hwqueue_pop(q, &size, NULL, 0);
		if (!dma) {
			chan_dbg(chan, "processing stopped, no desc\n");
			break;
		}

		hwdesc = hwqueue_unmap(q, dma, size);
		if (!hwdesc) {
			chan_dbg(chan, "couldn't unmap desc, continuing\n");
			continue;
		}

		desc = desc_from_hwdesc(hwdesc);
		adesc = desc_to_adesc(desc);

		chan->rxpool_destructor(chan->rxpool_param, index,
				chan->rxpools[index].buffer_size, adesc);

		atomic_inc(&chan->rxpools[index].deficit);

		desc_put(chan, desc);
	}
}

static void chan_stop(struct keystone_dma_chan *chan)
{
	unsigned long end;
	int i, deficit;

	if (chan->reg_rx_flow) {
		/* first detach fdqs, starve out the flow */
		__raw_writel(0, &chan->reg_rx_flow->fdq_sel[0]);
		__raw_writel(0, &chan->reg_rx_flow->fdq_sel[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[0]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[2]);
	}

	/* teardown the dma channel if we have such a thing */
	chan_teardown(chan);

	/* drain submitted buffers */
	if (chan->direction == DMA_DEV_TO_MEM) {
		for (i = 0; i < chan->rxpool_count; ++i)
			if (chan->q_submit[i])
				chan_purge_rxfree(chan, i, chan->q_submit[i]);
	} else
		if (chan->q_submit[0])
			chan_complete(chan, chan->q_submit[0], DMA_ERROR, -1, false);

	if (chan->q_complete) {
		/* wait for active transfers to complete */
		end = jiffies + msecs_to_jiffies(DMA_TIMEOUT);
		do {
			chan_complete(chan, chan->q_complete, DMA_COMPLETE, -1, false);
			if (!atomic_read(&chan->n_used_descs))
				break;
			schedule_timeout(DMA_TIMEOUT / 10);
		} while (time_after(end, jiffies));
	}

	/* Forget the notifier info */
	chan->notify_info.fn     = NULL;
	chan->notify_info.fn_arg = NULL;

	/* then disconnect the completion side and hope for the best */
	if (chan->reg_rx_flow) {
		__raw_writel(0, &chan->reg_rx_flow->control);
		__raw_writel(0, &chan->reg_rx_flow->tags);
		__raw_writel(0, &chan->reg_rx_flow->tag_sel);
	}

	/* Check for a possible memory leak */
	if (chan->direction == DMA_DEV_TO_MEM) {
		for (i = 0; i < chan->rxpool_count; ++i) {
			deficit = atomic_read(&chan->rxpools[i].deficit);
			WARN_ONCE((chan->rxpools[i].pool_depth != deficit),
				"chan_stop: pool %d deficit %d != depth %u\n",
				i, deficit, chan->rxpools[i].pool_depth);
		}
	}

	chan_vdbg(chan, "channel stopped\n");
}

static int chan_setup_descs(struct keystone_dma_chan *chan)
{
	struct keystone_dma_desc *desc;
	struct dma_async_tx_descriptor *adesc;
	struct keystone_hw_desc *hwdesc;
	dma_addr_t dma_addr;
	int ndesc = 0, size, i;
	struct desc_stats *new_chan;
	unsigned long flags;

	chan->desc_shift = order_base_2(sizeof(*desc));
	size = 1 << chan->desc_shift;
	chan->descs = kzalloc(size * chan->num_descs, GFP_KERNEL);
	if (!chan->descs)
		return -ENOMEM;

	atomic_set(&chan->n_used_descs, chan->num_descs);
	atomic_set(&chan->free_first, -1);

	for (i = 0; i < chan->num_descs; i++) {
		desc = desc_from_index(chan, i);

		adesc = desc_to_adesc(desc);
		adesc->cookie = DMA_MIN_COOKIE + desc_to_index(chan, desc);
		adesc->chan = to_achan(chan);
		adesc->tx_submit = chan_submit;

		hwdesc = NULL;
		dma_addr = hwqueue_pop(chan->q_pool, &desc->orig_size, NULL, 0);
		if (dma_addr)
			hwdesc = hwqueue_unmap(chan->q_pool, dma_addr, desc->orig_size);
		if (IS_ERR_OR_NULL(hwdesc))
			continue;

		if (desc->orig_size < sizeof(struct keystone_hw_desc)) {
			unsigned dma_size;
			int ret;

			ret = hwqueue_map(chan->q_pool, hwdesc, desc->orig_size,
					&dma_addr, &dma_size);
			if (likely(ret == 0))
				hwqueue_push(chan->q_pool, dma_addr,
					     dma_size, 0);
			continue;
		}

		desc->hwdesc = hwdesc;

		memset(hwdesc, 0, desc->orig_size);
		hwdesc->priv.desc = desc;

		desc_put(chan, desc);

		ndesc++;
	}

	if (!ndesc) {
		kfree(chan->descs);
		chan->descs = NULL;
		dev_err(chan_dev(chan), "no descriptors for channel\n");
	} else {
		chan_vdbg(chan, "initialized %d descriptors\n", ndesc);
	}

	if (ndesc) {
		new_chan = kzalloc(sizeof(*new_chan), GFP_KERNEL);
		if (new_chan) {
			new_chan->chan = chan;
			spin_lock_irqsave(&stats_lock, flags);
			list_add_tail(&new_chan->chan_le, &chan_stats_list);
			spin_unlock_irqrestore(&stats_lock, flags);
		} else {
			dev_err(chan_dev(chan), "no memory for channel list\n");
			return -ENOMEM;
		}
	}

	return ndesc ? ndesc : -ENOMEM;
}

static void chan_destroy_descs(struct keystone_dma_chan *chan)
{
	struct keystone_dma_desc *desc;
	bool leaked = false;
	dma_addr_t dma_addr;
	unsigned dma_size;
	int ret;
	int i;

	for (;;) {
		desc = desc_get(chan);
		if (!desc)
			break;
		if (chan->q_pool) {
			ret = hwqueue_map(chan->q_pool, desc->hwdesc, desc->orig_size,
					&dma_addr, &dma_size);
			if (likely(ret == 0))
				hwqueue_push(chan->q_pool, dma_addr, dma_size, 0);
		}
		desc->hwdesc = NULL;
	}

	for (i = 0; i < chan->num_descs; i++) {
		desc = desc_from_index(chan, i);
		if (!desc)
			break;
		if (desc->hwdesc) {
			dev_warn(chan_dev(chan), "%s leaked descriptor %d\n",
					chan_name(chan), i);
			desc_dump(chan, desc, "leaked descriptor");
			leaked = true;
		}
	}

	if (!leaked)
		kfree(chan->descs);

	chan->descs = NULL;
}

static void keystone_dma_hw_init(struct keystone_dma_device *dma)
{
	unsigned v;
	int i;

	v  = dma->loopback ? DMA_LOOPBACK : 0;
	__raw_writel(v, &dma->reg_global->emulation_control);

	v = __raw_readl(&dma->reg_global->perf_control);
	v |= ((dma->rx_timeout & DMA_RX_TIMEOUT_MASK) << DMA_RX_TIMEOUT_SHIFT);
	__raw_writel(v, &dma->reg_global->perf_control);

	v = ((dma->tx_priority << DMA_TX_PRIO_SHIFT) |
	     (dma->rx_priority << DMA_RX_PRIO_SHIFT));

	__raw_writel(v, &dma->reg_global->priority_control);

	if (dma->enable_all) {
		for (i = 0; i < dma->max_rx_chan; i++)
			__raw_writel(DMA_ENABLE, &dma->reg_rx_chan[i].control);

		for (i = 0; i < dma->max_tx_chan; i++) {
			__raw_writel(0, &dma->reg_tx_chan[i].mode);
			__raw_writel(DMA_ENABLE, &dma->reg_tx_chan[i].control);
		}
	}

	for (i = 0; i < dma->logical_queue_managers; i++)
		__raw_writel(dma->qm_base_address[i],
			     &dma->reg_global->qm_base_address[i]);

}

static void keystone_dma_hw_destroy(struct keystone_dma_device *dma)
{
	/* Do nothing for now */
}

static int chan_init(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_device *dma = chan->dma;

	chan_vdbg(chan, "initializing %s channel\n",
		  chan->direction == DMA_MEM_TO_DEV   ? "transmit" :
		  chan->direction == DMA_DEV_TO_MEM ? "receive"  :
		  "unknown");

	if (chan->direction != DMA_MEM_TO_DEV &&
	    chan->direction != DMA_DEV_TO_MEM) {
		dev_err(chan_dev(chan), "bad direction\n");
		return -EINVAL;
	}

	if (atomic_inc_return(&dma->in_use) <= 1)
		keystone_dma_hw_init(dma);

	if (chan_set_state(chan, CHAN_STATE_CLOSED, CHAN_STATE_OPENING))
		return -EBUSY;

	return 0;
}

static void chan_destroy(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_device *dma = chan->dma;
	enum keystone_chan_state state;

	do {
		state = chan_get_state(chan);
		if (state == CHAN_STATE_CLOSED)
			return;
	} while (chan_set_state(chan, state, CHAN_STATE_CLOSING));

	chan_vdbg(chan, "destroying channel\n");

	chan_stop(chan);
	chan_destroy_descs(chan);
	chan_destroy_queues(chan);

	atomic_dec_return(&dma->in_use);
	keystone_dma_hw_destroy(dma);

	WARN_ON(chan_set_state(chan, CHAN_STATE_CLOSING, CHAN_STATE_CLOSED));
	chan_vdbg(chan, "channel destroyed\n");
}

static int chan_set_notify(struct keystone_dma_chan *chan, bool enable)
{
	enum keystone_chan_state prev, next;
	int ret;

retry:
	prev = chan_get_state(chan);
	if (unlikely(!__chan_is_alive(prev))) {
		dev_err(chan_dev(chan), "cannot pause/resume in %s prev\n",
			chan_state_str(prev));
		return -ENODEV;
	}

	next = enable ? CHAN_STATE_OPENED  : CHAN_STATE_PAUSED;

	if (unlikely(prev == next)) {
		chan_vdbg(chan, "already in state (%s) for %s\n",
			  chan_state_str(prev),
			  enable ? "resume" : "pause");
		/* nothing to do */
		return 0;
	}

	if (unlikely(chan_set_state(chan, prev, next)))
		goto retry;

	if (enable)
		ret = hwqueue_enable_notifier(chan->q_complete);
	else
		ret = hwqueue_disable_notifier(chan->q_complete);

	if (unlikely(ret < 0)) {
		WARN_ON(chan_set_state(chan, next, prev));
		dev_err(chan_dev(chan), "queue notifier %s failed\n",
			enable ? "enable" : "disable");
	} else {
		chan_vdbg(chan, "channel %s\n", enable ? "resumed" : "paused");
	}

	return ret;
}

static int chan_rxfree_refill(struct keystone_dma_chan *chan)
{
	int i;

	if (chan->direction != DMA_DEV_TO_MEM)
		return -EINVAL;

	for (i = 0; i < chan->rxpool_count; ++i) {
		while (atomic_dec_return(&chan->rxpools[i].deficit) >= 0) {
			struct dma_async_tx_descriptor *adesc;
			struct keystone_dma_desc *desc;
			struct keystone_hw_desc *hwdesc;
			unsigned buffer_size = chan->rxpools[i].buffer_size;
			int ret;

			adesc = chan->rxpool_allocator(chan->rxpool_param,
					i, buffer_size);
			if (!adesc)
				break;

			desc = desc_from_adesc(adesc);
			hwdesc = desc_to_hwdesc(desc);

			desc->status = DMA_IN_PROGRESS;
			ret = hwqueue_push(chan->q_submit[i],
					desc->hwdesc_dma_addr,
					desc->hwdesc_dma_size, 0);
			if (unlikely(ret < 0)) {
				dev_warn(chan_dev(chan), "push error %d in %s\n",
						ret, __func__);
				hwqueue_unmap(chan->q_submit[i],
						desc->hwdesc_dma_addr,
						desc->hwdesc_dma_size);
				chan->rxpool_destructor(chan, i,
						buffer_size, adesc);
				desc_put(chan, desc);
				break;
			}
		}
		atomic_inc(&chan->rxpools[i].deficit);
	}

	return 0;
}

static int chan_rxfree_flush(struct keystone_dma_chan *chan)
{
	int i;

	if (chan->direction != DMA_DEV_TO_MEM)
		return -EINVAL;

	for (i = 0; i < chan->rxpool_count; ++i)
		chan_purge_rxfree(chan, i, chan->q_submit[i]);

	return 0;
}

static int chan_rxfree_refill_one(struct keystone_dma_chan *chan,
				  int pool,
				  struct dma_async_tx_descriptor *adesc)
{
	struct keystone_dma_desc *desc;
	struct keystone_hw_desc *hwdesc;
	int ret;
        u64 ns_time;
        struct desc_stats *stats_log = &desc_stats_log[log_wp];

	if (atomic_dec_return(&chan->rxpools[pool].deficit) < 0)
		return -1;

	desc = desc_from_adesc(adesc);
	hwdesc = desc_to_hwdesc(desc);

	/* desc stats logger */
	if ((pktdma_log_enable) && (desc->desc_timestamp > 0) && (((log_wp + 1) % LOG_SIZE) != log_rp)) {
		ns_time = ktime_get_real_ns();
		ns_time = ns_time - desc->desc_timestamp;

		if (ns_time < stats_p->min)
			stats_p->min = ns_time;

		if (ns_time > stats_p->max) {
			/* only log and schedule work Q when MAX changes */
			stats_log->min = stats_p->min;
			stats_log->max = stats_p->max = ns_time;
			stats_log->queue_id = stats_p->queue_id = desc->queue_id;
			stats_log->desc = stats_p->desc = desc;
			stats_log->chan = chan;

			log_wp = (log_wp + 1) % LOG_SIZE;

			schedule_work(&pktdma_desc_logger);
		}

		if (ns_time > chan->desc_push_stats.max) {
			chan->desc_push_stats.min = stats_p->min;
			chan->desc_push_stats.max = ns_time;
			chan->desc_push_stats.queue_id = desc->queue_id;
			chan->desc_push_stats.desc = desc;
		}
	}

	desc->status = DMA_IN_PROGRESS;
	ret = hwqueue_push(chan->q_submit[pool],
			   desc->hwdesc_dma_addr,
			   desc->hwdesc_dma_size, 0);

	if (unlikely(ret < 0)) {
		dev_warn(chan_dev(chan), "push error %d in %s\n", ret, __func__);
		hwqueue_unmap(chan->q_submit[pool],
			      desc->hwdesc_dma_addr,
			      desc->hwdesc_dma_size);
		desc_put(chan, desc);
		return ret;
	}

	return 0;
}

static int chan_keystone_config(struct keystone_dma_chan *chan,
		struct dma_keystone_info *config)
{
	struct keystone_dma_device *dma = chan->dma;
	unsigned num_descs = config->tx_queue_depth;
	int ret;
	int i;

	if (config->direction != chan->direction)
		return -EINVAL;

	if (chan_get_state(chan) != CHAN_STATE_OPENING)
		return -EBUSY;

	chan->scatterlist_size = config->scatterlist_size;
	chan->rxpool_allocator = config->rxpool_allocator;
	chan->rxpool_destructor = config->rxpool_destructor;
	chan->rxpool_param = config->rxpool_param;
	chan->rxpool_count = (config->rxpool_count < KEYSTONE_QUEUES_PER_CHAN) ?
			config->rxpool_count : KEYSTONE_QUEUES_PER_CHAN;

	for (i = 0; i < chan->rxpool_count; ++i) {
		chan->rxpools[i].buffer_size = config->rxpools[i].buffer_size;
		chan->rxpools[i].pool_depth = config->rxpools[i].pool_depth;
		atomic_set(&chan->rxpools[i].deficit, chan->rxpools[i].pool_depth);
		num_descs += chan->rxpools[i].pool_depth;
	}
	for (; i < KEYSTONE_QUEUES_PER_CHAN; ++i) {
		chan->rxpools[i].buffer_size = 0;
		chan->rxpools[i].pool_depth = 0;
		atomic_set(&chan->rxpools[i].deficit, 0);
	}

	chan->num_descs = num_descs;

	ret = chan_setup_queues(chan);
	if (ret < 0)
		return ret;

	chan->big_endian = dma->big_endian;

	ret = chan_setup_descs(chan);
	if (ret < 0) {
		chan_destroy_queues(chan);
		return ret;
	}

	ret = chan_start(chan);
	if (ret < 0) {
		chan_destroy_descs(chan);
		chan_destroy_queues(chan);
		return ret;
	}

	WARN_ON(chan_set_state(chan, CHAN_STATE_OPENING, CHAN_STATE_OPENED));

	return 0;
}

static void chan_issue_pending(struct dma_chan *chan)
{
}

static int chan_poll(struct keystone_dma_chan *chan, int budget)
{
	int packets;

	chan_dbg(chan, "channel poll beginning, budget %d\n", budget);
	packets = chan_complete(chan, chan->q_complete, DMA_COMPLETE, budget,
				true);
	chan_dbg(chan, "channel poll complete, processed %d\n", packets);

	return packets;
}

static void *chan_get_one(struct keystone_dma_chan *chan)
{
	struct dma_async_tx_descriptor *adesc;
	struct keystone_dma_desc *desc;
	void *callback_param;

	desc = chan_complete_one(chan, chan->q_complete);
	if (!desc) {
		return NULL;
	}

	desc->status = DMA_COMPLETE;
	adesc = desc_to_adesc(desc);
	callback_param = adesc->callback_param;

	/* Free the first descriptor in the chain */
	desc_put(chan, desc);

	return callback_param;
}

#if 0
static int chan_control(struct dma_chan *achan, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_device *dma = chan->dma;
	struct dma_slave_config *config;
	struct dma_notify_info *info;
	struct dma_refill_info *rinfo;
	struct dma_keystone_info *keystone_config;

	int ret = -EINVAL;

	switch ((int)cmd) {
	case DMA_GET_RX_FLOW:
		if (chan->direction == DMA_DEV_TO_MEM && chan->reg_rx_flow)
			ret = chan->reg_rx_flow - dma->reg_rx_flow;
		break;

	case DMA_GET_RX_QUEUE:
		if (chan->direction == DMA_DEV_TO_MEM)
			ret = chan->qnum_complete;
		break;

	case DMA_POLL:
		ret = chan_poll(chan, (int)arg);
		break;

	case DMA_SET_NOTIFY:
		info = (struct dma_notify_info *)arg;
		chan->notify_info = *info;
		ret = 0;
		break;

	case DMA_PAUSE:
		ret = chan_set_notify(chan, false);
		break;

	case DMA_RESUME:
		ret = chan_set_notify(chan, true);
		break;

	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;
		ret = (config->direction != chan->direction) ? -EINVAL : 0;
		break;

	case DMA_KEYSTONE_CONFIG:
		keystone_config = (struct dma_keystone_info *)arg;
		ret = chan_keystone_config(chan, keystone_config);
		break;

	case DMA_RXFREE_REFILL:
		ret = chan_rxfree_refill(chan);
		break;

	case DMA_RXFREE_FLUSH:
		ret = chan_rxfree_flush(chan);
		break;

	case DMA_GET_TX_QUEUE:
		if (chan->direction == DMA_MEM_TO_DEV)
			ret = chan->qnum_submit[0];
		break;

	case DMA_GET_ONE:
		ret = (int) chan_get_one(chan);
		break;

	case DMA_RXFREE_REFILL_ONE:
		rinfo = (struct dma_refill_info *)arg;
		ret = chan_rxfree_refill_one(chan, rinfo->pool, rinfo->desc);
		break;

	case DMA_TERMINATE_ALL:
	default:
		ret = -ENOTSUPP;
		break;
	}

	return ret;
}
#endif

unsigned dma_get_rx_hw_fdq(struct dma_chan *achan, unsigned q_submit)
{
	unsigned ret = 0;
	struct keystone_dma_chan *chan = from_achan(achan);
	if ((chan->direction == DMA_DEV_TO_MEM) &&
			(q_submit < KEYSTONE_QUEUES_PER_CHAN) &&
			(chan->q_submit[q_submit] != NULL))
		ret = (unsigned)((void*)chan->q_submit[q_submit]);
	return ret;
}

unsigned dma_get_rx_hw_compq(struct dma_chan *achan)
{
	unsigned ret = 0;
	struct keystone_dma_chan *chan = from_achan(achan);
	if (chan->direction == DMA_DEV_TO_MEM && chan->q_complete != NULL)
		ret = (unsigned)((void*)chan->q_complete);
	return ret;
}

unsigned dma_get_tx_hw_fdq(struct dma_chan *achan)
{
	unsigned ret = 0;
	struct keystone_dma_chan *chan = from_achan(achan);
	if ((chan->direction == DMA_MEM_TO_DEV) &&
			(chan->q_complete != NULL))
		ret = (unsigned)((void*)chan->q_complete);
	return ret;
}

unsigned dma_get_rx_fdq(struct dma_chan *achan, unsigned q_submit)
{
	unsigned ret = 0;
	struct keystone_dma_chan *chan = from_achan(achan);
	if ((chan->direction == DMA_DEV_TO_MEM) &&
			(q_submit < KEYSTONE_QUEUES_PER_CHAN) &&
			(chan->q_submit[q_submit] != NULL))
		ret = chan->qnum_submit[q_submit];
	return ret;
}

int dma_rxfree_refill_one(struct dma_chan *achan, int pool,
			  struct dma_async_tx_descriptor *desc)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_rxfree_refill_one(chan, pool, desc);
}


void *dma_get_one(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_get_one(chan);
}

int dma_get_rx_queue(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	if (chan->direction == DMA_DEV_TO_MEM)
		return chan->qnum_complete;
	return -EINVAL;
}

int dma_get_rx_flow(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_device *dma = chan->dma;

	if (chan->direction == DMA_DEV_TO_MEM && chan->reg_rx_flow)
		return chan->reg_rx_flow - dma->reg_rx_flow;
	return -EINVAL;
}

int dma_rxfree_flush(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_rxfree_flush(chan);
}
EXPORT_SYMBOL_GPL(dma_rxfree_flush);

int dma_get_tx_queue(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return (chan->direction == DMA_MEM_TO_DEV) ? chan->qnum_submit[0] : -EINVAL;
}

int dma_poll(struct dma_chan *achan, int budget)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_poll(chan, budget);
}
EXPORT_SYMBOL_GPL(dma_poll);

int dma_rxfree_refill(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_rxfree_refill(chan);
}
EXPORT_SYMBOL_GPL(dma_rxfree_refill);

int dma_set_notify(struct dma_chan *achan, dma_notify_fn fn, void *fn_arg)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct dma_notify_info info;

	info.fn = fn;
	info.fn_arg = fn_arg;
	chan->notify_info = info;

	return 0;
}
EXPORT_SYMBOL_GPL(dma_set_notify);

int dma_keystone_config(struct dma_chan *achan, struct dma_keystone_info *info)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_keystone_config(chan, info);
}
EXPORT_SYMBOL_GPL(dma_keystone_config);

static int pktdma_pause(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_set_notify(chan, false);
}

static int pktdma_resume(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return chan_set_notify(chan, true);
}

static int pktdma_slave_config(struct dma_chan *achan,
			       struct dma_slave_config *cfg)
{
	struct keystone_dma_chan *chan = from_achan(achan);

	return (cfg->direction != chan->direction) ? -EINVAL : 0;
}

static enum dma_status chan_xfer_status(struct dma_chan *achan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_desc *desc;
	enum dma_status status;

	desc = desc_from_index(chan, cookie - DMA_MIN_COOKIE);
	status = desc ? desc->status : DMA_ERROR;
	chan_vdbg(chan, "returning status %d for desc %p\n", status, desc);

	return status;
}

static struct dma_async_tx_descriptor *
chan_prep_slave_sg(struct dma_chan *achan, struct scatterlist *_sg,
		   unsigned int _num_sg, enum dma_transfer_direction direction,
		   unsigned long options, void *context)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	void *caller = __builtin_return_address(0);
	struct dma_async_tx_descriptor *adesc = NULL;
	struct keystone_hw_desc *hwdesc = NULL;
	u32 pslen = 0, *psdata = NULL;
	u32 epiblen = 0, *epib = NULL;
	struct scatterlist *sg = _sg;
	unsigned num_sg = _num_sg;
	unsigned nsg;
	unsigned packet_len;
	unsigned src_tag_lo;
	u32 tag_info = chan->tag_info;
	u32 packet_type = 0;
	u32 packet_info, psflags;
	u32 next_desc;
	unsigned q_num = (options >> DMA_QNUM_SHIFT) & DMA_QNUM_MASK;

	chan_vdbg(chan, "prep, caller %pS, channel %p, direction %s\n",
		  achan, caller,
		  chan->direction == DMA_MEM_TO_DEV ? "transmit" :
		  chan->direction == DMA_DEV_TO_MEM ? "receive" : "unknown");

	psflags = options & DESC_PSFLAGS_MASK;

	if (unlikely(direction != chan->direction)) {
		dev_err(chan_dev(chan), "mismatched dma direction\n");
		return ERR_PTR(-EINVAL);
	}

	if (likely(options & DMA_HAS_EPIB)) {
		epiblen  = sg->length;
		epib = sg_virt(sg);
		num_sg--;
		sg++;

		if (unlikely(epiblen != sizeof(hwdesc->epib))) {
			dev_err(chan_dev(chan), "invalid epib length %d\n",
				epiblen);
			return ERR_PTR(-EINVAL);
		}
		epiblen /= sizeof(u32);
	}

	if (likely(options & DMA_HAS_PSINFO)) {
		pslen  = sg->length;
		psdata = sg_virt(sg);
		num_sg--;
		sg++;

		if (unlikely(pslen & (sizeof(u32) - 1) ||
			     pslen > sizeof(hwdesc->psdata))) {
			dev_err(chan_dev(chan), "invalid psdata length %d\n",
				pslen);
			return ERR_PTR(-EINVAL);
		}
		pslen /= sizeof(u32);
	}

	 /* Error flag is valid only for Rx channel */
	if (unlikely((options & DMA_HAS_EFLAGS) &&
			(direction == DMA_MEM_TO_DEV))) {
		dev_err(chan_dev(chan), "eflag requested for Tx channel\n");
		return ERR_PTR(-EINVAL);
	}

	if (options & DMA_HAS_EFLAGS) {
		num_sg--;
		sg++;

		if (unlikely(sg->length < sizeof(u32))) {
			dev_err(chan_dev(chan), "invalid eflag length %d\n",
				sg->length);
			return ERR_PTR(-EINVAL);
		}
	}

	if (unlikely(options & DMA_HAS_PKTTYPE))
		packet_type =
			((options >> DMA_PKTTYPE_SHIFT) & DMA_PKTTYPE_MASK) <<
			DESC_PKTTYPE_SHIFT;

	/* src_tag_lo should be passed only for Tx channels */
	if (unlikely((options & DMA_HAS_SRC_TAG_LO) &&
			(direction == DMA_DEV_TO_MEM))) {
		dev_err(chan_dev(chan), "src_tag_lo requested for Rx channel\n");
		return ERR_PTR(-EINVAL);
	}

	if (options & DMA_HAS_SRC_TAG_LO) {
		src_tag_lo = (options >> DMA_SRC_TAG_LO_SHIFT) & DMA_SRC_TAG_LO_MASK;
		tag_info = tag_info & ~(DESC_SRC_TAG_LO_MASK << DESC_SRC_TAG_LO_SHIFT);
		tag_info |= src_tag_lo << DESC_SRC_TAG_LO_SHIFT;
	}

	if (unlikely(!chan_is_alive(chan))) {
		dev_err(chan_dev(chan), "cannot submit in state %s\n",
			chan_state_str(chan_get_state(chan)));
		return ERR_PTR(-ENODEV);
	}

	packet_info = ((epib ? DESC_HAS_EPIB : 0)	 |
		       pslen << DESC_PSLEN_SHIFT	 |
		       psflags << DESC_PSFLAGS_SHIFT |
		       chan->qnum_complete << DESC_RETQ_SHIFT |
		       chan->dest_queue_manager << DESC_RETQMGR_SHIFT);

	/* Walk backwards through the scatterlist */
	next_desc = 0;
	packet_len = 0;
	for (sg += num_sg - 1, nsg = num_sg; nsg > 0; --sg, --nsg) {
		struct keystone_dma_desc *desc;
		u32 buflen, orig_len;
		int ret;

		desc = desc_get(chan);
		if (unlikely(!desc)) {
			/* Free the descriptors we've allocated */
			pr_info_ratelimited("%s %s out of descriptors in direction %d used %d\n",
					chan_name(chan), chan_state_str(chan_get_state(chan)),
					direction, atomic_read(&chan->n_used_descs));
			for (; nsg < num_sg && next_desc; ++nsg) {
				hwdesc = hwqueue_unmap(chan->q_submit[q_num],
						next_desc, DESC_MIN_SIZE);
				desc = desc_from_hwdesc(hwdesc);
				next_desc = hwval_to_host(chan, hwdesc->next_desc);
				desc_put(chan, desc);
			}
			return ERR_PTR(-ENOMEM);
		}

		hwdesc = desc_to_hwdesc(desc);
		prefetchw(hwdesc);

		desc->options	= options;
		desc->sg_len	= _num_sg;
		desc->sg	= _sg;
		desc->size	= DESC_MIN_SIZE;

		adesc = desc_to_adesc(desc);
		prefetchw(&adesc->callback);

		WARN_ON(sg->length	    & ~DESC_LEN_MASK	||
			chan->qnum_complete & ~DESC_RETQ_MASK);

		/*
		 * All descriptors are Host Buffer Descriptors except for
		 * the first in the chain (last we process here), which is
		 * a Host Packet Descriptor. It has extra content.
		 */
		if (nsg == 1) {
			if (chan->direction == DMA_MEM_TO_DEV) {
				desc_copy(chan, hwdesc->epib, epib, epiblen);
				desc_copy(chan, hwdesc->psdata, psdata, pslen);
			}
			desc->size += (epiblen + pslen) * 4;
		}

		buflen = sg_dma_len(sg) & BITS(22);
		orig_len = (q_num << 28) | buflen;
		packet_len += buflen;
		desc_fill(chan, hwdesc,
			  packet_type | packet_len,/* desc_info	*/
			  tag_info,		   /* tag_info	*/
			  packet_info,		   /* packet_info */
			  buflen,		   /* buff_len	*/
			  sg_dma_address(sg),	   /* buff	*/
			  next_desc,		   /* next_desc	*/
			  orig_len,		   /* orig_len	*/
			  sg_dma_address(sg));	   /* orig_buf	*/

		/*
		 * NB: In the receive case, this should be per queue.
		 *     We don't have that info, and for mapping purposes
		 *     one hwqueue is as good as any other.
		 */
		ret = hwqueue_map(chan->q_submit[q_num], hwdesc, desc->size,
				&desc->hwdesc_dma_addr, &desc->hwdesc_dma_size);
		if (unlikely(ret < 0)) {
			WARN(1, "%s() failed!\n", __func__);
			return ERR_PTR(-ENOMEM);
		}

		next_desc = desc->hwdesc_dma_addr;
	}

	return adesc;
}

static void __iomem *
dma_get_regs(struct keystone_dma_device *dma, int index, const char *name,
	     resource_size_t *_size)
{
	struct device *dev = dma_dev(dma);
	struct device_node *node = dev->of_node;
	resource_size_t size;
	struct resource res;
	void __iomem *regs;

	if (of_address_to_resource(node, index, &res)) {
		dev_err(dev, "could not find %s resource (index %d)\n",
			name, index);
		return NULL;
	}
	size = resource_size(&res);

	if (!devm_request_mem_region(dev, res.start, size, dev_name(dev))) {
		dev_err(dev, "could not reserve %s resource (index %d)\n",
			name, index);
		return NULL;
	}

	regs = devm_ioremap_nocache(dev, res.start, size);
	if (!regs) {
		dev_err(dev, "could not map %s resource (index %d)\n",
			name, index);
		return NULL;
	}

	dev_vdbg(dev, "index: %d, res:%s, size:%x, phys:%x, virt:%p\n",
		 index, name, (unsigned int)size, (unsigned int)res.start,
		 regs);

	if (_size)
		*_size = size;

	return regs;
}

static int dma_init_rx_chan(struct keystone_dma_chan *chan,
				      struct device_node *node)
{
	struct keystone_dma_device *dma = chan->dma;
	struct device *dev = dma_dev(chan->dma);
	u32 flow = 0, channel = 0;
	int ret;

	ret = of_property_read_u32(node, "flow", &flow);
	if (ret < 0) {
		dev_dbg(dev, "no flow for %s channel\n", chan_name(chan));
	} else if (flow >= dma->max_rx_flow) {
		dev_err(dev, "invalid flow %d for %s channel\n",
			flow, chan_name(chan));
		return -EINVAL;
	} else {
		chan->flow = flow;
		chan->reg_rx_flow = dma->reg_rx_flow + flow;
	}

	ret = of_property_read_u32(node, "channel", &channel);
	if (ret < 0) {
		dev_dbg(dev, "no hw channel for %s channel\n",
			chan_name(chan));
	} else if (channel >= dma->max_rx_chan) {
		dev_err(dev, "invalid hw channel %d for %s channel\n",
			channel, chan_name(chan));
		return -EINVAL;
	} else {
		chan->channel = channel;
		chan->reg_chan = dma->reg_rx_chan + channel;
	}

	chan->rx_err_retry = (of_get_property(node,
				"rx-error-retry", NULL) != NULL);

	dev_info(dev, "%s rx channel: pool %s, channel %d (%p), flow %d (%p), "
			"submit %d, complete %d, error_retry %d\n",
		chan_name(chan), chan->qname_pool,
		chan->channel, chan->reg_chan, chan->flow, chan->reg_rx_flow,
		chan->qcfg_submit, chan->qcfg_complete, chan->rx_err_retry);

	return 0;
}

static int dma_init_tx_chan(struct keystone_dma_chan *chan,
				      struct device_node *node)
{
	struct keystone_dma_device *dma = chan->dma;
	struct device *dev = dma_dev(chan->dma);
	u32 channel, priority, tag_info;
	int ret;

	ret = of_property_read_u32(node, "channel", &channel);
	if (ret < 0) {
		dev_dbg(dev, "no hw channel for %s channel\n",
			chan_name(chan));
	} else if (channel >= dma->max_tx_chan) {
		dev_err(dev, "invalid hw channel %d for %s channel\n",
			channel, chan_name(chan));
		return -EINVAL;
	} else {
		chan->channel = channel;
		chan->reg_chan = dma->reg_tx_chan + channel;
		chan->reg_tx_sched = dma->reg_tx_sched + channel;
	}

	ret = of_property_read_u32(node, "priority", &priority);
	if (ret < 0) {
		dev_dbg(dev, "no priority for %s channel\n", chan_name(chan));
		priority = DMA_DEFAULT_PRIORITY;
	}
	chan->tx_sched_priority = priority;

	ret = of_property_read_u32(node, "tag-info", &tag_info);
	if (ret < 0) {
		dev_dbg(dev, "no tag-info for %s channel\n", chan_name(chan));
		tag_info = DMA_DEFAULT_TAG_INFO;
	}
	chan->tag_info = tag_info;

	dev_info(dev, "%s tx channel: pool %s, "
		"channel %d (%p), prio %d, tag %x, submit %d, complete %d\n",
		chan_name(chan), chan->qname_pool,
		chan->channel, chan->reg_chan,
		chan->tx_sched_priority, chan->tag_info,
		chan->qcfg_submit, chan->qcfg_complete);

	return 0;
}

static int dma_init_chan(struct keystone_dma_device *dma,
				   struct device_node *node)
{
	struct device *dev = dma_dev(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	u32 queue;
	int ret;

	chan = devm_kzalloc(dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->dma	= dma;
	chan->direction	= DMA_NONE;
	atomic_set(&chan->state, CHAN_STATE_CLOSED);

	achan = to_achan(chan);
	achan->device = to_dma(dma);

	ret = of_property_read_string(node, "label", (const char **)&achan->private);
	if (ret < 0)
		achan->private = (void *)node->name;
	if (!achan->private)
		achan->private = "unknown";

	ret = of_property_read_string(node, "pool", &chan->qname_pool);
	if (ret < 0) {
		dev_err(dev, "no descriptor pool for %s channel\n",
			chan_name(chan));
		goto fail;
	}

	chan->debug = (of_get_property(node, "debug",  NULL) != NULL);
	chan->debug = chan->debug || dma->debug;

	ret = of_property_read_u32(node, "submit-queue", &queue);
	if (ret < 0) {
		dev_dbg(dev, "unspecified submit queue for %s channel\n",
			chan_name(chan));
		queue = HWQUEUE_ANY;
	}
	chan->qcfg_submit = queue;

	ret = of_property_read_u32(node, "complete-queue", &queue);
	if (ret < 0) {
		dev_dbg(dev, "unspecified completion queue for %s channel\n",
			chan_name(chan));
		queue = HWQUEUE_ANY;
	}
	chan->qcfg_complete = queue;

	ret = -EINVAL;
	if (of_find_property(node, "transmit", NULL)) {
		chan->direction = DMA_MEM_TO_DEV;
		ret = dma_init_tx_chan(chan, node);
	} else if (of_find_property(node, "receive", NULL)) {
		chan->direction = DMA_DEV_TO_MEM;
		ret = dma_init_rx_chan(chan, node);
	} else {
		dev_err(dev, "%s channel direction unknown\n", chan_name(chan));
	}

	if (ret < 0)
		goto fail;

	list_add_tail(&to_achan(chan)->device_node, &to_dma(dma)->channels);

	return 0;

fail:
	devm_kfree(dev, chan);
	return ret;
}

static ssize_t keystone_dma_show_name(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", (char *)achan->private);
}

static ssize_t keystone_dma_show_tag_info(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", chan->tag_info);
}

static ssize_t keystone_dma_store_tag_info(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);
	unsigned long tag_info;

	if (kstrtoul(buf, 0, &tag_info))
		return -EINVAL;

	chan->tag_info = tag_info;
	return count;
}

static ssize_t keystone_dma_show_debug(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", chan->debug);
}

static ssize_t keystone_dma_store_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);
	unsigned long debug;

	if (kstrtoul(buf, 0, &debug))
		return -EINVAL;

	chan->debug = debug;
	return count;
}
static ssize_t keystone_dma_show_chan_num(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->channel);
}

static ssize_t keystone_dma_show_complete_queue(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->qcfg_complete);
}

static ssize_t keystone_dma_show_free_queue(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);
	int i, l = 0;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN; i++) {
		if (chan->qnum_submit[i])
			l += scnprintf(buf + l, PAGE_SIZE - l, "%u ",
				       chan->qnum_submit[i]);
	}
	l += scnprintf(buf + l, PAGE_SIZE - l, "\n");
	return l;

}

static ssize_t keystone_dma_show_submit_queue(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->qcfg_submit);
}

static ssize_t keystone_dma_show_flow(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->flow);
}

static DEVICE_ATTR(name, S_IRUSR, keystone_dma_show_name, NULL);
static DEVICE_ATTR(chan_num, S_IRUSR, keystone_dma_show_chan_num, NULL);
static DEVICE_ATTR(tx_tag_info, S_IRUSR | S_IWUSR,
	 keystone_dma_show_tag_info, keystone_dma_store_tag_info);
static DEVICE_ATTR(debug, S_IRUSR | S_IWUSR,
	 keystone_dma_show_debug, keystone_dma_store_debug);
static DEVICE_ATTR(rx_flow, S_IRUSR, keystone_dma_show_flow, NULL);
static DEVICE_ATTR(complete_queue, S_IRUSR,
		   keystone_dma_show_complete_queue, NULL);
static DEVICE_ATTR(submit_queue, S_IRUSR, keystone_dma_show_submit_queue, NULL);
static DEVICE_ATTR(free_queue, S_IRUSR, keystone_dma_show_free_queue, NULL);

static void keystone_dma_destroy_attr(struct keystone_dma_device *dma)
{
	struct dma_device *engine = to_dma(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	struct device *dev;

	list_for_each_entry(achan, &engine->channels, device_node) {
		chan = from_achan(achan);
		dev = chan_dev(chan);

		/* remove sysfs entries */
		device_remove_file(dev, &dev_attr_name);
		device_remove_file(dev, &dev_attr_chan_num);
		device_remove_file(dev, &dev_attr_complete_queue);
		device_remove_file(dev, &dev_attr_debug);
		if (chan->direction == DMA_MEM_TO_DEV) {
			device_remove_file(dev, &dev_attr_tx_tag_info);
			device_remove_file(dev, &dev_attr_submit_queue);
		} else {
			device_remove_file(dev, &dev_attr_rx_flow);
			device_remove_file(dev, &dev_attr_free_queue);
		}
	}
}

static int  keystone_dma_setup_attr(struct keystone_dma_device *dma)
{
	struct dma_device *engine = to_dma(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	struct device *dev;
	int status = 0;

	list_for_each_entry(achan, &engine->channels, device_node) {
		chan = from_achan(achan);
		dev = chan_dev(chan);

		/* add sysfs entries */
		status = device_create_file(dev, &dev_attr_name);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file name\n");
		status = device_create_file(dev, &dev_attr_chan_num);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file chan_num\n");
		status = device_create_file(dev, &dev_attr_complete_queue);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file complete_queue\n");
		if (chan->direction == DMA_MEM_TO_DEV) {
			status = device_create_file(dev, &dev_attr_tx_tag_info);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file tx_tag_info\n");
			status = device_create_file(dev,
						    &dev_attr_submit_queue);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file submit_queue\n");
		} else {
			status = device_create_file(dev, &dev_attr_rx_flow);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file rx_flow\n");
			status = device_create_file(dev, &dev_attr_free_queue);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file rx_free_queu\n");
		}
		status = device_create_file(dev, &dev_attr_debug);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file debug\n");
	}
	return status;
}

static int keystone_dma_probe(struct platform_device *pdev)
{
	unsigned max_tx_chan, max_rx_chan, max_rx_flow, max_tx_sched;
	struct device_node *node = pdev->dev.of_node;
	struct keystone_dma_device *dma;
	struct device_node *chans, *chan;
	struct dma_device *engine;
	resource_size_t size;
	int ret, num_chan = 0;
	u32 priority, timeout;
	u32 config[4];
	u32 i;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	dma = devm_kzalloc(&pdev->dev, sizeof(*dma), GFP_KERNEL);
	if (!dma) {
		dev_err(&pdev->dev, "could not allocate driver mem\n");
		return -ENOMEM;
	}
	engine = to_dma(dma);
	engine->dev = &pdev->dev;
	platform_set_drvdata(pdev, dma);

	dma->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dma->clk))
		dma->clk = NULL;

	if (dma->clk)
		clk_prepare_enable(dma->clk);

	dma->reg_global	 = dma_get_regs(dma, 0, "global", &size);
	if (!dma->reg_global)
		return -ENODEV;
	if (size < sizeof(struct reg_global)) {
		dev_err(dma_dev(dma), "bad size (%d) for global regs\n",
			(int) size);
		return -ENODEV;
	}

	dma->reg_tx_chan = dma_get_regs(dma, 1, "txchan", &size);
	if (!dma->reg_tx_chan)
		return -ENODEV;
	max_tx_chan = size / sizeof(struct reg_chan);

	dma->reg_rx_chan = dma_get_regs(dma, 2, "rxchan", &size);
	if (!dma->reg_rx_chan)
		return -ENODEV;
	max_rx_chan = size / sizeof(struct reg_chan);

	dma->reg_tx_sched = dma_get_regs(dma, 3, "txsched", &size);
	if (!dma->reg_tx_sched)
		return -ENODEV;
	max_tx_sched = size / sizeof(struct reg_tx_sched);

	dma->reg_rx_flow = dma_get_regs(dma, 4, "rxflow", &size);
	if (!dma->reg_rx_flow)
		return -ENODEV;
	max_rx_flow = size / sizeof(struct reg_rx_flow);

	dma->enable_all	= (of_get_property(node, "enable-all", NULL) != NULL);
	dma->big_endian	= (of_get_property(node, "big-endian", NULL) != NULL);
	dma->loopback	= (of_get_property(node, "loop-back",  NULL) != NULL);
	dma->debug	= (of_get_property(node, "debug",  NULL) != NULL);

	ret = of_property_read_u32(node, "rx-priority", &priority);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unspecified rx priority using value 0\n");
		priority = DMA_PRIO_DEFAULT;
	}
	dma->rx_priority = priority;

	ret = of_property_read_u32(node, "tx-priority", &priority);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unspecified tx priority using value 0\n");
		priority = DMA_PRIO_DEFAULT;
	}
	dma->tx_priority = priority;

	ret = of_property_read_u32(node, "rx-retry-timeout", &timeout);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unspecified rx timeout using value %d\n",
				DMA_RX_TIMEOUT_DEFAULT);
		timeout = DMA_RX_TIMEOUT_DEFAULT;
	}
	dma->rx_timeout = timeout;

	ret = of_property_read_u32(node, "logical-queue-managers",
				   &dma->logical_queue_managers);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unspecified number of logical "
				"queue managers\n");
		dma->logical_queue_managers = 2;
	}

	ret = of_property_read_u32(node, "queues-per-queue-manager",
				   &dma->queues_per_queue_manager);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unspecified number of queues per "
				"queue manager\n");
		dma->queues_per_queue_manager = 4096;
	}

	ret = of_property_read_u32_array(node, "qm-base-address",
					 config, dma->logical_queue_managers);
	if (ret)
		return ret;

	for (i = 0; i < dma->logical_queue_managers; i++)
		dma->qm_base_address[i] = config[i];

	dma->max_rx_chan = max_rx_chan;
	dma->max_rx_flow = max_rx_flow;
	dma->max_tx_chan = min(max_tx_chan, max_tx_sched);

	atomic_set(&dma->in_use, 0);

	/* NSN: initialize HW for use by DSPs and userspace */
	keystone_dma_hw_init(dma);

	INIT_LIST_HEAD(&engine->channels);

	INIT_LIST_HEAD(&chan_stats_list);

	chans = of_get_child_by_name(node, "channels");
	if (!chans) {
		dev_err(dma_dev(dma), "could not find channels\n");
		return -ENODEV;
	}

	for_each_child_of_node(chans, chan) {
		if (dma_init_chan(dma, chan) >= 0)
			num_chan++;
	}

	of_node_put(chans);

	if (list_empty(&engine->channels)) {
		dev_err(dma_dev(dma), "no valid channels\n");
		return -ENODEV;
	}

	dma_cap_set(DMA_SLAVE, engine->cap_mask);

	engine->device_alloc_chan_resources = chan_init;
	engine->device_free_chan_resources  = chan_destroy;
	engine->device_issue_pending	    = chan_issue_pending;
	engine->device_tx_status	    = chan_xfer_status;
	engine->device_prep_slave_sg	    = chan_prep_slave_sg;
	engine->device_pause		    = pktdma_pause;
	engine->device_resume		    = pktdma_resume;
	engine->device_config		    = pktdma_slave_config;

	ret = dma_async_device_register(engine);
	if (ret) {
		dev_err(&pdev->dev, "unable to register dma engine\n");
		return ret;
	}

	ret = keystone_dma_setup_attr(dma);
	if (ret) {
		dev_err(&pdev->dev, "unable to setup device attr\n");
		return ret;
	}

	dev_info(dma_dev(dma), "registered %d logical channels, flows %d, "
		 "tx chans: %d, rx chans: %d%s%s\n", num_chan,
		 dma->max_rx_flow, dma->max_tx_chan, dma->max_rx_chan,
		 dma->big_endian ? ", big-endian" : "",
		 dma->loopback ? ", loopback" : "");

	return 0;
}

static int keystone_dma_remove(struct platform_device *pdev)
{
	struct keystone_dma_device *dma = platform_get_drvdata(pdev);
	struct dma_device *engine = to_dma(dma);

	if (dma->clk) {
		clk_disable_unprepare(dma->clk);
		clk_put(dma->clk);
	}
	dma->clk = NULL;

	keystone_dma_destroy_attr(dma);
	dma_async_device_unregister(engine);

	return 0;
}

static int pktdma_debug_show(struct seq_file *s, void *v)
{
	struct desc_stats *cur_chan;
	struct list_head *chan_entry;
	unsigned long flags;

	if (!pktdma_log_enable) {
		seq_printf(s, "pktdma_stats is disabled. to enable use:\n");
		seq_printf(s, "echo 1> /sys/kernel/debug/pktdma_log_enable\n\n");
	}

	seq_printf(s, "PKTDMA STATS -> pushed desc %p min (ns) = %u max (ns) = %u queue_id = %u \n",
			stats_p->desc, stats_p->min, stats_p->max, stats_p->queue_id);

	spin_lock_irqsave(&stats_lock, flags);
	list_for_each(chan_entry, &chan_stats_list) {
		cur_chan = list_entry(chan_entry, struct desc_stats, chan_le);

		seq_printf(s, "PKTDMA CHAN STATS -> chan = %p pushed desc %p min (ns) = %u max (ns) = %u queue_id = %u \n",
			cur_chan->chan, cur_chan->chan->desc_push_stats.desc,
			cur_chan->chan->desc_push_stats.min, cur_chan->chan->desc_push_stats.max,
			cur_chan->chan->desc_push_stats.queue_id);

	}
	spin_unlock_irqrestore(&stats_lock, flags);

	return 0;
}


static int pktdma_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, pktdma_debug_show, NULL);
}

static int pktdma_set(void *data, u64 val)
{
	pktdma_log_enable = val;

	if (pktdma_log_enable)
		mod_timer(&stat_timer, jiffies + msecs_to_jiffies(stat_time_interval));
	
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(pktdma_fops, NULL, pktdma_set, "%llu\n");

static struct of_device_id of_match[] = {
	{ .compatible = "ti,keystone-pktdma", },
	{},
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver keystone_dma_driver = {
	.probe	= keystone_dma_probe,
	.remove	= keystone_dma_remove,
	.driver = {
		.name		= "keystone-pktdma",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
};


static const struct file_operations pktdma_debug_ops = {
	.owner		= THIS_MODULE,
	.open		= pktdma_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init keystone_dma_init(void)
{
	BUILD_CHECK_REGS();

	stats_dentry = debugfs_create_file("pktdma_stats", 0444, NULL, NULL,
			    &pktdma_debug_ops);

	stats_dentry_en = debugfs_create_file("pktdma_log_enable", 0644, NULL, NULL, &pktdma_fops);


	return platform_driver_register(&keystone_dma_driver);
}

device_initcall_sync(keystone_dma_init);

static void __exit keystone_dma_exit(void)
{
	debugfs_remove(stats_dentry);
	debugfs_remove(stats_dentry_en);

	del_timer(&stat_timer);

	platform_driver_unregister(&keystone_dma_driver);
}
module_exit(keystone_dma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cyril Chemparathy <cyril@ti.com>");
MODULE_DESCRIPTION("TI Keystone Packet DMA driver");
