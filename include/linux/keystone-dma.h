/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com
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

#ifndef __MACH_KEYSTONE_DMA_H__
#define __MACH_KEYSTONE_DMA_H__

#include <linux/dmaengine.h>

#define BITS(x)			(BIT(x) - 1)

#define DMA_HAS_PSINFO		BIT(31)
#define DMA_HAS_EPIB		BIT(30)
#define DMA_HAS_SRC_TAG_LO	BIT(29)
#define DMA_HAS_EFLAGS		BIT(28)
#define DMA_HAS_PKTTYPE         BIT(27)
#define DMA_QNUM_SHIFT		25
#define DMA_QNUM_MASK		BITS(2)
#define DMA_PKTTYPE_SHIFT       20
#define DMA_PKTTYPE_MASK        BITS(5)
#define DMA_SRC_TAG_LO_SHIFT	12
#define DMA_SRC_TAG_LO_MASK	BITS(8)

typedef void (*dma_notify_fn)(struct dma_chan *chan, void *arg);

#define DMA_GET_RX_FLOW		1000
#define DMA_GET_RX_QUEUE	1001
#define DMA_POLL		1002
#define DMA_SET_NOTIFY		1003
#define DMA_KEYSTONE_CONFIG	1004
#define DMA_RXFREE_REFILL	1005
#define DMA_GET_TX_QUEUE	1006
#define DMA_RXFREE_FLUSH	1007
#define DMA_GET_ONE             1008
#define DMA_RXFREE_REFILL_ONE   1009

struct dma_notify_info {
	dma_notify_fn		 fn;
	void			*fn_arg;
};

struct dma_refill_info {
	struct dma_async_tx_descriptor *desc;
	int    pool;
};

int dma_get_rx_flow(struct dma_chan *chan);

int dma_get_rx_queue(struct dma_chan *chan);

int dma_get_tx_queue(struct dma_chan *chan);

int dma_poll(struct dma_chan *chan, int budget);

int dma_set_notify(struct dma_chan *chan, dma_notify_fn fn, void *fn_arg);

void *dma_get_one(struct dma_chan *chan);

/* Number of RX queues supported per channel */
#define	KEYSTONE_QUEUES_PER_CHAN	4

enum dma_keystone_thresholds {
	DMA_THRESH_NONE		= 0,
	DMA_THRESH_0		= 1,
	DMA_THRESH_0_1		= 3,
	DMA_THRESH_0_1_2	= 7
};

typedef struct dma_async_tx_descriptor *(*dma_rxpool_alloc_fn)(
		void *arg, unsigned q_num, unsigned bufsize);
typedef void (*dma_rxpool_free_fn)(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc);

struct dma_keystone_info {
	enum dma_transfer_direction	 direction;
	unsigned int			 scatterlist_size;
	unsigned int			 tx_queue_depth;
	dma_rxpool_alloc_fn		 rxpool_allocator;
	dma_rxpool_free_fn		 rxpool_destructor;
	void				*rxpool_param;
	unsigned int			 rxpool_count;
	enum dma_keystone_thresholds	 rxpool_thresh_enable;
	struct {
		unsigned	pool_depth;
		unsigned	buffer_size;
	}				 rxpools[KEYSTONE_QUEUES_PER_CHAN];
};

int dma_rxfree_refill_one(struct dma_chan *chan, int pool,
			  struct dma_async_tx_descriptor *desc);

int dma_rxfree_flush(struct dma_chan *chan);

int dma_keystone_config(struct dma_chan *chan, struct dma_keystone_info *info);

int dma_rxfree_refill(struct dma_chan *chan);

bool keystone_dma_name_flt(struct dma_chan *chan, void *name);

unsigned dma_get_rx_hw_fdq(struct dma_chan *achan, unsigned q_submit);

unsigned dma_get_rx_hw_compq(struct dma_chan *achan);

unsigned dma_get_rx_fdq(struct dma_chan *achan, unsigned q_submit);

unsigned dma_get_tx_hw_fdq(struct dma_chan *achan);

#define dma_request_channel_by_name(mask, name) \
	__dma_request_channel(&(mask), keystone_dma_name_flt, (void *)(name))

#endif /* __MACH_KEYSTONE_DMA_H__ */

