/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <linux/keystone-dma.h>

#include "keystone_rio.h"

static void keystone_rio_dma_start(struct keystone_rio_dma_chan *chan);

static inline struct keystone_rio_dma_desc *desc_from_adesc(
	struct dma_async_tx_descriptor *adesc)
{
	return container_of(adesc, struct keystone_rio_dma_desc, adesc);
}

static inline struct dma_async_tx_descriptor *desc_to_adesc(
	struct keystone_rio_dma_desc *desc)
{
	return &desc->adesc;
}

static int keystone_rio_dma_chan_set_state(struct keystone_rio_dma_chan *chan,
					   enum keystone_rio_chan_state old,
					   enum keystone_rio_chan_state new)
{
	enum keystone_rio_chan_state cur;

	cur = atomic_cmpxchg(&chan->state, old, new);

	if (likely(cur == old))
		return 0;

	return -EINVAL;
}

static inline enum keystone_rio_chan_state
keystone_rio_dma_chan_get_state(struct keystone_rio_dma_chan *chan)
{
	return atomic_read(&chan->state);
}

static inline void
keystone_rio_dma_chan_force_state(struct keystone_rio_dma_chan *chan,
				  enum keystone_rio_chan_state state)
{
	atomic_set(&chan->state, state);
}

/*
 * Return the first descriptor of the active list (called with spinlock held)
 */
static inline struct keystone_rio_dma_desc *keystone_rio_dma_first_active(
	struct keystone_rio_dma_chan *chan)
{
	if (list_empty(&chan->active_list))
		return NULL;

	return list_first_entry(&chan->active_list,
				struct keystone_rio_dma_desc,
				node);
}

/*
 * Return the next descriptor of a transfer list (called with spinlock held)
 */
static inline struct keystone_rio_dma_desc *keystone_rio_dma_next(
	struct keystone_rio_dma_chan *chan)
{
	struct keystone_rio_dma_desc *desc = chan->current_transfer;
	struct keystone_rio_dma_desc *next;

	next = list_entry(desc->tx_list.next, struct keystone_rio_dma_desc,
			  tx_list);
	if (next == desc)
		return NULL;

	return next;
}

static inline void keystone_rio_dma_complete_notify(
	struct keystone_rio_data *krio_priv,
	u32 lsu)
{
	struct keystone_rio_dma_chan *chan, *_c;

	list_for_each_entry_safe(chan, _c, &krio_priv->dma_channels[lsu], node)
		tasklet_schedule(&chan->tasklet);
}

void keystone_rio_dma_interrupt_handler(struct keystone_rio_data *krio_priv,
					u32 lsu,
					u32 error)
{
	if (unlikely(error)) {
		/*
		 * In case of error we do not know the LSU, so complete all
		 * running channels
		 */
		u32 __lsu;
		for (__lsu = 0; __lsu < KEYSTONE_RIO_LSU_NUM; __lsu++)
			keystone_rio_dma_complete_notify(krio_priv, __lsu);
	} else {
		/* Notify all channels for the corresponding LSU */
		keystone_rio_dma_complete_notify(krio_priv, lsu);
	}
}

static inline void keystone_rio_dma_chain_complete(
	struct keystone_rio_dma_chan *chan,
	struct keystone_rio_dma_desc *desc)
{
	struct dma_async_tx_descriptor *adesc = desc_to_adesc(desc);
	dma_async_tx_callback callback = adesc->callback;
	void *param = adesc->callback_param;

	chan->completed_cookie = adesc->cookie;

	if (!IS_ERR_OR_NULL(chan->sgl) && !IS_ERR_OR_NULL(chan->bounce_buffer)) {
		if (desc->packet_type == KEYSTONE_RIO_PACKET_TYPE_NREAD) {
			(void) sg_copy_from_buffer(chan->sgl, chan->sgl_nents,
						   chan->bounce_buffer,
						   chan->tran_len);
		}
		kfree(chan->bounce_buffer);
		chan->bounce_buffer = NULL;
	}

	if (callback)
		callback(param);
}

static inline void keystone_rio_dma_complete(struct keystone_rio_dma_chan *chan,
					     struct list_head *list,
					     int call_completion)
{
	struct keystone_rio_dma_desc *desc, *_d;
	struct keystone_rio_dma_desc *frag_desc, *_fd;

	list_for_each_entry_safe(desc, _d, list, node) {
		/* Call completion handler */
		if (call_completion)
			keystone_rio_dma_chain_complete(chan, desc);

		/* Free all fragment descriptors if any */
		list_for_each_entry_safe(frag_desc, _fd, &desc->tx_list,
					 tx_list)
			kfree(frag_desc);

		kfree(desc);
	}
}

/*
 * Do the completion of the whole transfer
 */
static void keystone_rio_dma_complete_all(struct keystone_rio_dma_chan *chan)
{
	LIST_HEAD(list);

	/* Complete the current active transfer and prepare the next one */
	spin_lock(&chan->lock);
	list_splice_init(&chan->active_list, &list);
	list_splice_init(&chan->queue, &chan->active_list);
	spin_unlock(&chan->lock);

	keystone_rio_dma_complete(chan, &list, 1);
}

/*
 * Return the next transfer chunk to perform (called with spinlock held)
 */
static struct keystone_rio_dma_desc *
keystone_rio_dma_next_work(struct keystone_rio_dma_chan *chan)
{
	if (!chan->current_transfer) {
		/* If no current_transfer */
		chan->current_transfer = keystone_rio_dma_first_active(chan);
		dev_dbg(chan_dev(chan), "%s: no current transfer, moving to first_active 0x%x\n",
			__func__, (u32) chan->current_transfer);
	} else {
		/* Move to next part of the transfer */
		chan->current_transfer = keystone_rio_dma_next(chan);
		dev_dbg(chan_dev(chan), "%s: moving to next part of the transfer 0x%x\n",
			__func__, (u32) chan->current_transfer);
	}
	return chan->current_transfer;
}

/*
 * Start a transfer for a given channel
 */
static void keystone_rio_dma_start(struct keystone_rio_dma_chan *chan)
{
	struct keystone_rio_dma_desc *desc;
	unsigned long flags;
	int res;

	spin_lock_irqsave(&chan->lock, flags);

	desc = chan->current_transfer;

	if (unlikely((!desc) || (keystone_rio_dma_chan_get_state(chan)
				 != RIO_CHAN_STATE_ACTIVE))) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	keystone_rio_dma_chan_set_state(chan, RIO_CHAN_STATE_ACTIVE,
					RIO_CHAN_STATE_RUNNING);

	/* Perform the DIO transfer */
	dev_dbg(chan_dev(chan), "%s: perform current DIO transfer (desc = 0x%x | buff_addr - 0x%x)\n",
		__func__, (u32) desc, (u32) desc->buff_addr);

	res = keystone_rio_lsu_start_transfer(chan->lsu,
					      desc->port_id,
					      desc->dest_id,
					      desc->buff_addr,
					      desc->rio_addr,
					      desc->size,
					      desc->sys_size,
					      desc->packet_type,
					      &desc->lsu_context,
					      1,
					      chan->krio);
	if (res) {
		desc->status = DMA_ERROR;
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_err(chan_dev(chan), "DIO: transfer error %d\n", res);
		return;
	}

	keystone_rio_dma_chan_set_state(chan, RIO_CHAN_STATE_RUNNING,
					RIO_CHAN_STATE_WAITING);

	spin_unlock_irqrestore(&chan->lock, flags);
}

static void keystone_rio_dma_tasklet(unsigned long data)
{
	struct keystone_rio_dma_chan *chan =
		(struct keystone_rio_dma_chan *) data;
	struct keystone_rio_dma_desc *desc;
	int res = 0;

	spin_lock(&chan->lock);

	desc = chan->current_transfer;

	dev_dbg(chan_dev(chan), "tasklet called for channel%d\n", chan_id(chan));

	if (unlikely((!desc) || (keystone_rio_dma_chan_get_state(chan)
				 != RIO_CHAN_STATE_WAITING))) {
		spin_unlock(&chan->lock);
		return;
	}

	/* Check the completion code */
	res = keystone_rio_lsu_complete_transfer(chan->lsu,
						 desc->lsu_context,
						 chan->krio);

	if ((res == -EAGAIN) && (desc->retry_count-- > 0)) {
		spin_unlock(&chan->lock);

		dev_dbg(chan_dev(chan),
			"LSU%d transfer not completed (busy) context 0x%x (0x%x), restart the channel completion\n",
			chan->lsu, desc->lsu_context,
			(u32) &desc->lsu_context);

		tasklet_schedule(&chan->tasklet);
		return;
	}

	if (res) {
		desc->status = DMA_ERROR;
		spin_unlock(&chan->lock);

		dev_dbg(chan_dev(chan),
			"%s: LSU%d DMA transfer failed with %d\n",
			__func__, chan->lsu, res);

		/* Stop current transfer */
		return;
	}

	/* If last part of the transfer, do the DMA completion */
	if (desc->last) {

		spin_unlock(&chan->lock);

		keystone_rio_dma_complete_all(chan);

		spin_lock(&chan->lock);

		chan->current_transfer = NULL;
	}

	/* Move to next transfer */
	desc = keystone_rio_dma_next_work(chan);

	keystone_rio_dma_chan_set_state(chan, RIO_CHAN_STATE_WAITING,
					RIO_CHAN_STATE_ACTIVE);

	spin_unlock(&chan->lock);

	/* Start next transfer if any */
	if (desc)
		keystone_rio_dma_start(chan);
}

static int keystone_rio_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);

	dev_dbg(chan_dev(chan), "init DMA engine channel%d\n", chan_id(chan));

	spin_lock_bh(&chan->lock);
	BUG_ON(!list_empty(&chan->active_list));
	BUG_ON(!list_empty(&chan->queue));

	chan->completed_cookie = dchan->cookie = 1;

	spin_unlock_bh(&chan->lock);

	keystone_rio_dma_chan_set_state(chan, RIO_CHAN_STATE_UNUSED,
					RIO_CHAN_STATE_ACTIVE);

	tasklet_enable(&chan->tasklet);

	return 0;
}

static void keystone_rio_dma_free_chan_resources(struct dma_chan *dchan)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);
	LIST_HEAD(list);

	dev_dbg(chan_dev(chan), "freeing DMA Engine channel%d\n",
		chan_id(chan));

	if (keystone_rio_dma_chan_get_state(chan) != RIO_CHAN_STATE_ACTIVE) {
		dev_warn(chan_dev(chan),
			 "freeing still running DMA channel %d!!!\n",
			 chan_id(chan));
	}

	keystone_rio_dma_chan_force_state(chan, RIO_CHAN_STATE_UNUSED);

	tasklet_disable(&chan->tasklet);

	/* Purge the current active and queued transfers */
	if (!list_empty(&chan->active_list)) {
		dev_warn(chan_dev(chan),
			 "transfer still active on DMA channel %d!!!\n",
			 chan_id(chan));
		spin_lock_bh(&chan->lock);
		list_splice_init(&chan->active_list, &list);
		spin_unlock_bh(&chan->lock);
	}

	if (!list_empty(&chan->queue)) {
		dev_warn(chan_dev(chan),
			 "queued transfers on DMA channel %d!!!\n",
			 chan_id(chan));
		spin_lock_bh(&chan->lock);
		list_splice_init(&chan->queue, &list);
		spin_unlock_bh(&chan->lock);
	}

	if (!list_empty(&list))
		keystone_rio_dma_complete(chan, &list, 0);

	chan->current_transfer = NULL;
}

static void keystone_rio_dma_issue_pending(struct dma_chan *dchan)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);

	if (keystone_rio_dma_chan_get_state(chan) == RIO_CHAN_STATE_ACTIVE) {
		struct keystone_rio_dma_desc *desc;

		spin_lock_bh(&chan->lock);
		desc = keystone_rio_dma_next_work(chan);
		spin_unlock_bh(&chan->lock);

		if (desc)
			keystone_rio_dma_start(chan);

	} else
		dev_dbg(chan_dev(chan),	"%s: DMA channel busy, state = %d\n",
			__func__, keystone_rio_dma_chan_get_state(chan));
}

static enum dma_status keystone_rio_dma_tx_status(struct dma_chan *dchan,
						  dma_cookie_t cookie,
						  struct dma_tx_state *txstate)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);
	struct keystone_rio_dma_desc *desc = chan->current_transfer;
	dma_cookie_t last_used;
	dma_cookie_t last_completed;
	enum dma_status status;

	spin_lock_bh(&chan->lock);
	last_completed = chan->completed_cookie;
	last_used      = dchan->cookie;
	spin_unlock_bh(&chan->lock);

	/*
	 * In case of error, totally complete the current transfer
	 * and start the new then return error
	 */
	if ((desc) && (desc->status == DMA_ERROR)) {

		dev_dbg(chan_dev(chan),	"DMA error\n");

		keystone_rio_dma_complete_all(chan);

		spin_lock_bh(&chan->lock);

		/* Even if not the last, stop the current transfer */
		chan->current_transfer = NULL;

		keystone_rio_dma_chan_set_state(chan, RIO_CHAN_STATE_WAITING,
						RIO_CHAN_STATE_ACTIVE);

		spin_unlock_bh(&chan->lock);

		keystone_rio_dma_issue_pending(dchan);

		return DMA_ERROR;
	}

	status = dma_async_is_complete(cookie, last_completed, last_used);

	dma_set_tx_state(txstate, last_completed, last_used, 0);

	dev_dbg(chan_dev(chan),
		"%s: exit, ret: %d, last_completed: %d, last_used: %d\n",
		__func__, (int) status, last_completed, last_used);

	return status;
}

static dma_cookie_t keystone_rio_dma_tx_submit(
	struct dma_async_tx_descriptor *adesc)
{
	struct keystone_rio_dma_desc *desc = desc_from_adesc(adesc);
	struct keystone_rio_dma_chan *chan = from_dma_chan(adesc->chan);
	unsigned long flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&chan->lock, flags);

	/* Increment the DMA cookie */
	cookie = adesc->chan->cookie;
	if (++cookie < 0)
		cookie = 1;
	adesc->chan->cookie = cookie;
	adesc->cookie = cookie;

	/* Add the transfer to the DMA */
	if (list_empty(&chan->active_list)) {
		list_add_tail(&desc->node, &chan->active_list);
		spin_unlock_irqrestore(&chan->lock, flags);
	} else {
		list_add_tail(&desc->node, &chan->queue);
		spin_unlock_irqrestore(&chan->lock, flags);
	}

	return cookie;
}

static struct dma_async_tx_descriptor *
keystone_rio_dma_prep_slave_sg(struct dma_chan *dchan,
			       struct scatterlist *sgl,
			       unsigned int sg_len,
			       enum dma_transfer_direction dir,
			       unsigned long flags,
			       void *tinfo)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);
	struct keystone_rio_dma_desc *desc = NULL;
	struct keystone_rio_dma_desc *first = NULL;
	struct rio_dma_ext *rext = (struct rio_dma_ext *) tinfo;
	u64 rio_addr = rext->rio_addr; /* limited to 64-bit for now */
	struct scatterlist *sg;
	u32 packet_type, last_packet_type;
	unsigned int i;
	struct scatterlist dev_sg = {0};
	bool highmem_transfer = false;
	size_t transfer_len = 0;

	if (!sgl || !sg_len) {
		dev_err(chan_dev(chan), "%s: no SG list\n", __func__);
		return NULL;
	}

	if (sg_len > KEYSTONE_RIO_DMA_MAX_DESC) {
		dev_err(chan_dev(chan), "%s: SG list is too long (%d)\n",
			__func__, sg_len);
		return NULL;
	}

	if (dir == DMA_DEV_TO_MEM) {
		packet_type      = KEYSTONE_RIO_PACKET_TYPE_NREAD;
		last_packet_type = KEYSTONE_RIO_PACKET_TYPE_NREAD;
	} else if (dir == DMA_MEM_TO_DEV) {
		switch (rext->wr_type) {
		case RDW_DEFAULT:
		case RDW_ALL_NWRITE:
			packet_type      = KEYSTONE_RIO_PACKET_TYPE_NWRITE;
			last_packet_type = KEYSTONE_RIO_PACKET_TYPE_NWRITE;
			break;
		case RDW_LAST_NWRITE_R:
			packet_type      = KEYSTONE_RIO_PACKET_TYPE_NWRITE;
			last_packet_type = KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
			break;
		case RDW_ALL_NWRITE_R:
		default:
			packet_type      = KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
			last_packet_type = KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
			break;
		}
	} else {
		dev_err(chan_dev(chan),	"unsupported DMA direction option\n");
		return NULL;
	}

	for_each_sg(sgl, sg, sg_len, i) {
		unsigned long pfn = page_to_pfn(sg_page(sg)) - (chan_dev(chan))->dma_pfn_offset;

		/* page address is in high memory */
		if (pfn & 0xff00000ul)
			highmem_transfer = true;
		transfer_len += sg_dma_len(sg);
	}

	if (highmem_transfer) {
		chan->bounce_buffer = (u32 *) kmalloc(transfer_len * sizeof(u32),
						      GFP_KERNEL | GFP_DMA);
		if (IS_ERR_OR_NULL(chan->bounce_buffer))
			return ERR_PTR(ENOMEM);
		chan->sgl = sgl;
		chan->sgl_nents = sg_len;
		chan->tran_len = transfer_len;

		/* NWRITE operation */
		if (dir == DMA_MEM_TO_DEV)
			chan->tran_len = sg_copy_to_buffer(sgl, sg_len,
							   (void *) chan->bounce_buffer,
							   transfer_len);
		sg_init_table(&dev_sg, 1);
		sg_dma_address(&dev_sg) = (dma_addr_t) chan->bounce_buffer;
		sg_dma_len(&dev_sg) = transfer_len;

		sgl = &dev_sg;
		sg_len = 1;
	}

	for_each_sg(sgl, sg, sg_len, i) {
		/* Allocate a (virtual) DMA descriptor for this transfer */
		desc = kmalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			dev_err(chan_dev(chan),
				"cannot allocate DMA transfer descriptor\n");
			return NULL;
		}

		dma_async_tx_descriptor_init(&desc->adesc, dchan);
		desc->adesc.tx_submit = keystone_rio_dma_tx_submit;
		desc->adesc.flags = DMA_CTRL_ACK;

		/* Fill the descriptor with the RapidIO informations */
		desc->retry_count = KEYSTONE_RIO_RETRY_CNT;
		desc->status      = DMA_COMPLETE;
		desc->port_id     = dma_to_mport(dchan->device)->index;
		desc->dest_id     = rext->destid;
		desc->rio_addr    = rio_addr;
		desc->rio_addr_u  = 0;
		desc->buff_addr   = sg_dma_address(sg);
		desc->size        = sg_dma_len(sg);
		desc->sys_size    = dma_to_mport(dchan->device)->sys_size;

		INIT_LIST_HEAD(&desc->node);
		INIT_LIST_HEAD(&desc->tx_list);

		if (sg_is_last(sg)) {
			desc->last  = true;
			packet_type = last_packet_type;
		} else
			desc->last = false;

		/* Check if we can switch to SWRITE */
		if ((packet_type == KEYSTONE_RIO_PACKET_TYPE_NWRITE)
		    && ((desc->size & 0x7) == 0)
		    && ((desc->rio_addr & 0x7) == 0)
		    && ((desc->buff_addr & 0x7) == 0)) {
			packet_type = KEYSTONE_RIO_PACKET_TYPE_SWRITE;
		}

		desc->packet_type = packet_type;

		rio_addr += sg_dma_len(sg);

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->tx_list, &first->tx_list);
	}

	first->adesc.cookie = -EBUSY;
	desc->adesc.flags  |= flags;

	return &first->adesc;
}

static int keystone_rio_dma_prep_raw_packet(
	struct keystone_rio_dma_chan *chan,
	struct keystone_rio_dma_packet_raw *pkt)
{
	struct keystone_rio_dma_desc *desc;
	struct dma_chan *dchan = to_dma_chan(chan);

	/* Allocate a (virtual) DMA descriptor for this transfer */
	desc = kmalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		dev_err(chan_dev(chan),
			"cannot allocate DMA transfer descriptor\n");
		return -ENOMEM;
	}

	dma_async_tx_descriptor_init(&desc->adesc, dchan);
	desc->adesc.tx_submit = keystone_rio_dma_tx_submit;
	desc->adesc.flags     = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	desc->adesc.cookie    = -EBUSY;

	/* Fill the descriptor with the RapidIO raw packet informations */
	desc->retry_count = KEYSTONE_RIO_RETRY_CNT;
	desc->status      = DMA_COMPLETE;
	desc->port_id     = pkt->port_id;
	desc->dest_id     = pkt->dest_id;
	desc->rio_addr    = pkt->rio_addr;
	desc->rio_addr_u  = pkt->rio_addr_u;
	desc->buff_addr   = pkt->buff_addr;
	desc->size        = pkt->size;
	desc->sys_size    = pkt->sys_size;
	desc->packet_type = pkt->packet_type;
	desc->last        = true;
	desc->lsu_context = 0;

	INIT_LIST_HEAD(&desc->node);
	INIT_LIST_HEAD(&desc->tx_list);

	pkt->tx = &desc->adesc;

	return 0;
}

static int keystone_rio_dma_terminate_all(struct keystone_rio_dma_chan *chan)
{
	LIST_HEAD(list);

	/* Stop the current transfers */
	spin_lock_bh(&chan->lock);
	list_splice_init(&chan->active_list, &list);
	list_splice_init(&chan->queue, &list);
	chan->current_transfer = NULL;
	spin_unlock_bh(&chan->lock);

	keystone_rio_dma_chan_force_state(chan, RIO_CHAN_STATE_ACTIVE);

	/* Complete all transfers */
	keystone_rio_dma_complete(chan, &list, 1);

	return 0;
}

int keystone_rio_dma_device_control(struct dma_chan *dchan,
					   enum k2_dma_ctrl_cmd cmd,
					   unsigned long arg)
{
	struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);
	int ret = 0;

	switch ((u32)cmd) {

	case DMA_TERMINATE_ALL:
		ret = keystone_rio_dma_terminate_all(chan);
		break;

	case DMA_PAUSE:
		tasklet_disable(&chan->tasklet);
		break;

	case DMA_RESUME:
		tasklet_enable(&chan->tasklet);
		break;

	case DMA_KEYSTONE_RIO_PREP_RAW_PACKET:
		ret = keystone_rio_dma_prep_raw_packet(
			chan,
			(struct keystone_rio_dma_packet_raw *) arg);
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

int keystone_rio_dma_register(struct rio_mport *mport, int channel_num)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	u32 c;
	int ret;

	if (channel_num > KEYSTONE_RIO_DMA_MAX_CHANNEL)
		return -EINVAL;

	if (channel_num == 0)
		return 0;

	mport->dma.dev = krio_priv->dev;
	mport->dma.chancnt = channel_num < 0 ?
		KEYSTONE_RIO_DMA_MAX_CHANNEL : channel_num;

	INIT_LIST_HEAD(&mport->dma.channels);

	for (c = 0; c < mport->dma.chancnt; c++) {

		struct keystone_rio_dma_chan *chan =
			kzalloc(sizeof(struct keystone_rio_dma_chan),
				GFP_KERNEL);

		if (!chan) {
			dev_err(krio_priv->dev,
				"failed to allocate channel\n");
			return -ENOMEM;
		}

		chan->dchan.device  = &mport->dma;
		chan->dchan.cookie  = 1;
		chan->dchan.chan_id = c;

		spin_lock_init(&chan->lock);

		INIT_LIST_HEAD(&chan->active_list);
		INIT_LIST_HEAD(&chan->queue);

		tasklet_init(&chan->tasklet,
			     keystone_rio_dma_tasklet,
			     (u32) chan);
		tasklet_disable(&chan->tasklet);
		list_add_tail(&chan->dchan.device_node, &mport->dma.channels);

		chan->krio = krio_priv;

		/* Allocate one LSU per channel */
		chan->lsu = keystone_rio_lsu_alloc(chan->krio);

		keystone_rio_dma_chan_force_state(chan, RIO_CHAN_STATE_UNUSED);

		list_add_tail(&chan->node,
			      &krio_priv->dma_channels[chan->lsu]);

		dev_info(krio_priv->dev,
			 "registering DMA channel %d (0x%x) using lsu %d for port %d\n",
			 c, (u32) chan, chan->lsu, mport->index);
	}

	dma_cap_zero(mport->dma.cap_mask);
	dma_cap_set(DMA_PRIVATE, mport->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, mport->dma.cap_mask);

	mport->dma.device_alloc_chan_resources =
		keystone_rio_dma_alloc_chan_resources;
	mport->dma.device_free_chan_resources =
		keystone_rio_dma_free_chan_resources;
	mport->dma.device_tx_status =
		keystone_rio_dma_tx_status;
	mport->dma.device_issue_pending =
		keystone_rio_dma_issue_pending;
	mport->dma.device_prep_slave_sg =
		keystone_rio_dma_prep_slave_sg;

	ret = dma_async_device_register(&mport->dma);
	if (ret)
		dev_err(krio_priv->dev, "failed to register DMA device\n");

	dev_dbg(mport->dma.dev, "%s: dma device registered\n", __func__);

	return ret;
}

void keystone_rio_dma_unregister(struct rio_mport *mport)
{
	dma_async_device_unregister(&mport->dma);

	dev_dbg(mport->dma.dev, "%s: dma device unregistered\n", __func__);
}
