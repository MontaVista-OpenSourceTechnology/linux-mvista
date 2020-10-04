/*
 * Driver for the LSI DMA controller DMA-32.
 *
 * The driver is based on:
 *
 * lsi-dma32.c -
 * lsi-dma.c - Copyright 2011 Mentor Graphics
 * acp_gpdma.c - Copyright (c) 2011, Ericsson AB
 *               Niclas Bengtsson <niklas.x.bengtsson@ericsson.com>
 *               Kerstin Jonsson <kerstin.jonsson@ericsson.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/export.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <asm/page.h>
#include <linux/bitops.h>
#include <linux/atomic.h>
#include <linux/sizes.h>
#include "virt-dma.h"
#include "lsi-dma32.h"

#ifdef DEBUG
#define engine_dbg(engine, fmt, ...) \
	do { \
		struct gpdma_engine *_e = engine; \
		(void)_e; \
		pr_debug("dma0: " fmt, ##__VA_ARGS__); \
	} while (0)

#define ch_dbg(dmac, fmt, ...) \
	do { \
		struct gpdma_channel *_c = dmac; \
		(void)_c; \
		pr_debug("dma0ch%d: [%s] " fmt, \
			dmac->id, __func__, ##__VA_ARGS__); \
	} while (0)
#else
#define engine_dbg(engine, fmt, ...) do {} while (0)
#define ch_dbg(dmac, fmt, ...)       do {} while (0)
#endif

static unsigned int burst = 7;
module_param(burst, uint, 0644);
MODULE_PARM_DESC(burst,
		 "Preferred burst setting (0=SINGLE,3=INCR4,5=INCR8,7=INCR16)");

static void reset_channel(struct gpdma_channel *dmac)
{
	const int WAIT = 1024;
	int i;

	/* Pause channel */
	writel(DMA_STATUS_CH_PAUS_WR_EN | DMA_STATUS_CH_PAUSE,
	     dmac->base+DMA_STATUS);
	wmb();

	/* Disable channel */
	writel(0, dmac->base+DMA_CHANNEL_CONFIG);
	for (i = 0; readl(dmac->base+DMA_CHANNEL_CONFIG) && i < WAIT; i++)
		cpu_relax();
	if (i == WAIT)
		ch_dbg(dmac, "Failed to DISABLE channel\n");

	/* Clear FIFO */
	writel(DMA_CONFIG_CLEAR_FIFO, dmac->base+DMA_CHANNEL_CONFIG);
	for (i = 0; readl(dmac->base+DMA_CHANNEL_CONFIG) && i < WAIT; i++)
		cpu_relax();
	if (i == WAIT)
		ch_dbg(dmac, "Failed to clear FIFO\n");
}

static void soft_reset(struct gpdma_engine *engine)
{
	int i;
	u32 cfg;

	/* Reset all channels */
	for (i = 0; i < engine->chip->num_channels; i++)
		reset_channel(&engine->channel[i]);

	/* Reset GPDMA by writing Magic Number to reset reg */
	writel(GPDMA_MAGIC, engine->gbase + SOFT_RESET);
	wmb();

	cfg = (engine->pool.phys & 0xfff00000) | GEN_CONFIG_EXT_MEM;

	if (engine->chip->flags & LSIDMA_EDGE_INT) {
		for (i = 0; i < engine->chip->num_channels; i++)
			cfg |= GEN_CONFIG_INT_EDGE(i);
		engine_dbg(engine, "Using edge-triggered interrupts\n");
	}
	writel(cfg, engine->gbase + GEN_CONFIG);
	engine_dbg(engine, "engine->desc.phys & 0xfff00000 == %llx\n",
		   (engine->pool.phys & 0xfff00000));

	engine->ch_busy = 0;
}

static int alloc_desc_table(struct gpdma_engine *engine)
{
	/*
	 * For controllers that doesn't support full descriptor addresses, all
	 * descriptors must be in the same 1 MB page, i.e address bits 31..20
	 * must be the same for all descriptors.
	 */
	u32 order = 20 - PAGE_SHIFT;
	int i;

	if (engine->chip->flags & LSIDMA_NEXT_FULL) {
		/*
		 * Controller can do full descriptor addresses, then we need no
		 * special alignment on the descriptor block.
		 */
		order = get_order(GPDMA_MAX_DESCRIPTORS *
				  sizeof(struct gpdma_desc));
	}

	engine->pool.va = (struct gpdma_desc *)
			  __get_free_pages(GFP_KERNEL|GFP_DMA, order);
	if (!engine->pool.va)
		return -ENOMEM;
	memset(engine->pool.va, 0, GPDMA_MAX_DESCRIPTORS * sizeof(struct gpdma_desc));
	engine->pool.order = order;
	engine->pool.phys = virt_to_phys(engine->pool.va);
	engine_dbg(engine, "order=%d pa=%#llx va=%p\n",
		   engine->pool.order, engine->pool.phys, engine->pool.va);

	INIT_LIST_HEAD(&engine->free_list);
	for (i = 0; i < GPDMA_MAX_DESCRIPTORS; i++) {
		struct gpdma_desc *desc = &engine->pool.va[i];
		async_tx_ack(&desc->vdesc.tx);
		desc->engine = engine;
		list_add_tail(&desc->vdesc.node, &engine->free_list);
	}

	return 0;
}

static void free_desc_table(struct gpdma_engine *engine)
{
	if (engine->pool.va)
		free_pages((unsigned long)engine->pool.va, engine->pool.order);
}

static struct gpdma_desc *get_descriptor(struct gpdma_engine *engine)
{
	unsigned long flags;
	struct gpdma_desc *new = NULL, *desc, *tmp;

	spin_lock_irqsave(&engine->lock, flags);
	list_for_each_entry_safe(desc, tmp, &engine->free_list, vdesc.node) {
		if (async_tx_test_ack(&desc->vdesc.tx)) {
			list_del(&desc->vdesc.node);
			new = desc;
			new->chain = NULL;
			new->head = NULL;
			new->ned = NULL;
			break;
		}
	}
	spin_unlock_irqrestore(&engine->lock, flags);

	return new;
}

/**
 * init_descriptor - Fill out all descriptor fields
 */
static void init_descriptor(struct gpdma_desc *desc,
			    dma_addr_t src, u32 src_acc,
			    dma_addr_t dst, u32 dst_acc,
			    size_t len)
{
	u32 src_count = len >> src_acc;
	u32 dst_count = len >> dst_acc;
	u32 rot_len = (2 * (1 << src_acc)) - 1;

	BUG_ON(src_count * (1<<src_acc) != len);
	BUG_ON(dst_count * (1<<dst_acc) != len);

	desc->src = src;
	desc->dst = dst;

	desc->hw.src_x_ctr     = cpu_to_le16(src_count - 1);
	desc->hw.src_y_ctr     = 0;
	desc->hw.src_x_mod     = cpu_to_le32(1 << src_acc);
	desc->hw.src_y_mod     = 0;
	desc->hw.src_addr      = cpu_to_le32(src & 0xffffffff);
	desc->hw.src_data_mask = ~0;
	desc->hw.src_access    = cpu_to_le16((rot_len << 6) |
					    (src_acc << 3) |
					    (burst & 7));
	desc->hw.dst_access    = cpu_to_le16((dst_acc << 3) |
					    (burst & 7));
	desc->hw.ch_config     = cpu_to_le32(DMA_CONFIG_ONE_SHOT(1));
	desc->hw.next_ptr      = 0;
	desc->hw.dst_x_ctr     = cpu_to_le16(dst_count - 1);
	desc->hw.dst_y_ctr     = 0;
	desc->hw.dst_x_mod     = cpu_to_le32(1 << dst_acc);
	desc->hw.dst_y_mod     = 0;
	desc->hw.dst_addr      = cpu_to_le32(dst & 0xffffffff);
}

static phys_addr_t desc_to_paddr(const struct gpdma_channel *dmac,
				 const struct gpdma_desc *desc)
{
	phys_addr_t paddr = virt_to_phys(&desc->hw);
	WARN_ON(paddr & 0xf);
	if (dmac->engine->chip->flags & LSIDMA_NEXT_FULL)
		paddr |= 0x8;
	else
		paddr &= 0xfffff;

	return paddr;
}

static void free_descriptor(struct virt_dma_desc *vd)
{
	struct gpdma_desc *desc = to_gpdma_desc(vd);
	struct gpdma_engine *engine = desc->engine;
	unsigned long flags;

	BUG_ON(desc == NULL);

	spin_lock_irqsave(&engine->lock, flags);
	while (desc) {
		list_add_tail(&desc->vdesc.node, &engine->free_list);
		desc = desc->chain;
	}
	spin_unlock_irqrestore(&engine->lock, flags);
}

static inline int desc_segment_match(struct gpdma_channel const * const dmac,
			      struct gpdma_desc const * const desc1,
			      struct gpdma_desc const * const desc2)
{
	unsigned int seg_src = (unsigned int) (desc1->src >> 32);
	unsigned int seg_dst = (unsigned int) (desc1->dst >> 32);

	return ((seg_src == (unsigned int) (desc2->src >> 32)) &&
	        (seg_dst == (unsigned int) (desc2->dst >> 32)));
}

static int segment_match(struct gpdma_engine *engine, struct gpdma_desc *desc)
{
	unsigned int gpreg_dma = readl(engine->gpreg);
	unsigned int seg_src = (gpreg_dma >> 0) & 0x3f;
	unsigned int seg_dst = (gpreg_dma >> 8) & 0x3f;

	return (seg_src == ((desc->src >> 32) & 0x3f) &&
		seg_dst == ((desc->dst >> 32) & 0x3f));
}

static int gpdma_restart(struct gpdma_channel *dmac)
{
	struct gpdma_desc    *desc;
	phys_addr_t           paddr;

	desc = dmac->active;
	if (!(dmac->engine->chip->flags & LSIDMA_SEG_REGS)) {
		/*
		 * No segment registers -> descriptor address bits must match
		 * running descriptor on any other channel.
		 */
		if (dmac->engine->ch_busy && !segment_match(dmac->engine, desc))
			return -1;
	}
	reset_channel(dmac);

	/* Physical address of descriptor to load */
	paddr = desc_to_paddr(dmac, desc);
	writel((u32)paddr, dmac->base + DMA_NXT_DESCR);

	if (dmac->engine->chip->flags & LSIDMA_SEG_REGS) {
		/* Segment bits [39..32] of descriptor, src and dst addresses */
		writel(paddr >> 32, dmac->base + DMA_DESCR_ADDR_SEG);
		writel(desc->src >> 32, dmac->base + DMA_SRC_ADDR_SEG);
		writel(desc->dst >> 32, dmac->base + DMA_DST_ADDR_SEG);
	} else {
		unsigned int seg_src = (desc->src >> 32) & 0x3f;
		unsigned int seg_dst = (desc->dst >> 32) & 0x3f;
		writel((seg_dst << 8) | seg_src, dmac->engine->gpreg);
	}
	wmb();
	writel(DMA_CONFIG_DSC_LOAD, dmac->base + DMA_CHANNEL_CONFIG);

	return 0;
}

static void gpdma_start(struct gpdma_channel *dmac)
{
	struct virt_dma_desc *vdesc;
	struct gpdma_desc    *desc;
	phys_addr_t           paddr;

	vdesc = vchan_next_desc(&dmac->vc);
	if (!vdesc) {
		clear_bit(dmac->id, &dmac->engine->ch_busy);
		dmac->active = NULL;
		return;
	}

	/* Remove from list and mark as active */
	list_del(&vdesc->node);
	desc = to_gpdma_desc(vdesc);
	dmac->active = desc;

	if (!(dmac->engine->chip->flags & LSIDMA_SEG_REGS)) {
		/*
		 * No segment registers -> descriptor address bits must match
		 * running descriptor on any other channel.
		 */
		if (dmac->engine->ch_busy && !segment_match(dmac->engine, desc))
			return;
	}

	/* Physical address of descriptor to load */
	paddr = desc_to_paddr(dmac, desc);
	writel((u32)paddr, dmac->base + DMA_NXT_DESCR);

	if (dmac->engine->chip->flags & LSIDMA_SEG_REGS) {
		/* Segment bits [39..32] of descriptor, src and dst addresses */
		writel(paddr >> 32, dmac->base + DMA_DESCR_ADDR_SEG);
		writel(desc->src >> 32, dmac->base + DMA_SRC_ADDR_SEG);
		writel(desc->dst >> 32, dmac->base + DMA_DST_ADDR_SEG);
	} else {
		unsigned int seg_src = (desc->src >> 32) & 0x3f;
		unsigned int seg_dst = (desc->dst >> 32) & 0x3f;
		writel((seg_dst << 8) | seg_src, dmac->engine->gpreg);
	}
	wmb();
	writel(DMA_CONFIG_DSC_LOAD, dmac->base + DMA_CHANNEL_CONFIG);
	set_bit(dmac->id, &dmac->engine->ch_busy);
}

static irqreturn_t gpdma_isr_err(int irqno, void *_engine)
{
	struct gpdma_engine *engine = _engine;
	u32 status = readl(engine->gbase + GEN_STAT);
	u32 ch = (status & GEN_STAT_CH0_ERROR) ? 0 : 1;
	struct gpdma_channel *dmac = &engine->channel[ch];

	if (0 == (status & (GEN_STAT_CH0_ERROR | GEN_STAT_CH1_ERROR)))
		return IRQ_NONE;

	/* Read the channel status bits and dump the error */
	status = readl(dmac->base + DMA_STATUS);
	pr_info("dma: channel%d status %08x\n", dmac->id, status);
	/* Clear the error indication */
	writel(DMA_STATUS_ERROR, dmac->base+DMA_STATUS);

	return IRQ_HANDLED;
}

static irqreturn_t gpdma_isr(int irqno, void *_dmac)
{
	struct gpdma_channel *dmac = _dmac;
	struct gpdma_desc    *desc = dmac->active;
	u32                  status;
	u32	             error;

	status = readl(dmac->base+DMA_STATUS);
	error = status & DMA_STATUS_ERROR;
	writel(DMA_STATUS_CLEAR, dmac->base+DMA_STATUS);

	ch_dbg(dmac, "irq%u channel status %08x, error %08x\n",
		irqno, status, error);

	if (error) {
		if (error & DMA_STATUS_UNALIGNED_ERR) {
			dev_warn(dmac->engine->dev,
				 "Unaligned transaction on ch%d (status=%#x)\n",
				 dmac->id, status);
			reset_channel(dmac);
		} else {
			dev_warn(dmac->engine->dev,
				 "DMA transaction error on ch%d (status=%#x)\n",
				 dmac->id, status);
		}
	}

	if (0 != (status & DMA_STATUS_CH_ACTIVE)) {
		if (NULL != dmac->active->ned) {
			dmac->active = dmac->active->ned;
			if (0 == gpdma_restart(dmac))
				return IRQ_HANDLED;
		} else
			WARN_ON((status & DMA_STATUS_CH_ACTIVE) != 0);
	}

	BUG_ON(desc == NULL);

	spin_lock(&dmac->vc.lock);
	vchan_cookie_complete(&desc->head->vdesc);
	dmac->active = NULL;
	if (vchan_next_desc(&dmac->vc)) {
		gpdma_start(dmac);
	} else {
		/* Stop channel */
		writel(0, dmac->base + DMA_CHANNEL_CONFIG);
		writel(DMA_CONFIG_CLEAR_FIFO, dmac->base + DMA_CHANNEL_CONFIG);
		clear_bit(dmac->id, &dmac->engine->ch_busy);
	}
	spin_unlock(&dmac->vc.lock);

	return IRQ_HANDLED;
}

static void flush_channel(struct gpdma_channel *dmac)
{
	unsigned long flags;
	LIST_HEAD(head);

	/* disable task and wait for completion if it's already running */
	tasklet_disable(&dmac->vc.task);

	spin_lock_irqsave(&dmac->vc.lock, flags);
	reset_channel(dmac);
	if (dmac->active) {
		free_descriptor(&dmac->active->vdesc);
		dmac->active = NULL;
	}
	vchan_get_all_descriptors(&dmac->vc, &head);
	spin_unlock_irqrestore(&dmac->vc.lock, flags);
	vchan_dma_desc_free_list(&dmac->vc, &head);

	/* re-enable task */
	tasklet_enable(&dmac->vc.task);
}

/*
 * Perform soft reset procedure on DMA Engine.  Needed occasionally to work
 * around nasty bug ACP3400 sRIO HW.
 */
static ssize_t __ref
reset_engine(struct device *dev,
	     struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct gpdma_engine *engine = dev_get_drvdata(dev);
	int i;

	if (!engine)
		return -EINVAL;

	/* Disable interrupts and tasklet and acquire each channel lock */
	for (i = 0; i < engine->chip->num_channels; i++) {
		struct gpdma_channel *dmac = &engine->channel[i];
		disable_irq(dmac->irq);
		spin_lock(&dmac->vc.lock);
		tasklet_disable(&dmac->vc.task);
	}

	soft_reset(engine);

	for (i = 0; i < engine->chip->num_channels; i++) {
		struct gpdma_channel *dmac = &engine->channel[i];
		tasklet_enable(&dmac->vc.task);
		enable_irq(dmac->irq);
		/* Restart any active jobs */
		if (dmac->active) {
			struct gpdma_desc *active = dmac->active;
			dmac->active = NULL;
			list_add(&active->vdesc.node, &dmac->vc.desc_submitted);
			if (vchan_issue_pending(&dmac->vc))
				gpdma_start(dmac);
		}
		spin_unlock(&dmac->vc.lock);
	}

	return count;
}
static DEVICE_ATTR(soft_reset, S_IWUSR, NULL, reset_engine);

/*
 *===========================================================================
 *
 *                       DMA DEVICE INTERFACE
 *
 *===========================================================================
 *
 */

/**
 * gpdma_alloc_chan_resources - Allocate resources and return the number of
 * allocated descriptors.
 *
 */
static int gpdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);

	(void) dmac;
	return 1;
}

/**
 * gpdma_free_chan_resources - Release DMA channel's resources.
 *
 */
static void gpdma_free_chan_resources(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);

	(void) dmac;
}

static int gpdma_slave_config(struct dma_chan *chan, struct dma_slave_config *cfg)
{
	struct gpdma_channel *dchan = to_gpdma_chan(chan);

	memcpy(&dchan->cfg, cfg, sizeof(dchan->cfg));

	return 0;
}

/**
 * gpdma_prep_sg - Prepares a transfer using sg lists.
 *
 */
static struct dma_async_tx_descriptor *
gpdma_prep_sg(struct dma_chan *chan,
	      struct scatterlist *dst_sg, unsigned int dst_nents,
	      struct scatterlist *src_sg, unsigned int src_nents,
	      unsigned long flags)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);
	struct gpdma_desc *first = NULL, *prev = NULL, *new = NULL, *eob = NULL;
	size_t dst_avail, src_avail;
	dma_addr_t dst, src;
	u32 src_acc, dst_acc;
	size_t len;

	if (dst_nents == 0 || src_nents == 0)
		return NULL;

	if (dst_sg == NULL || src_sg == NULL)
		return NULL;

	dst_avail = sg_dma_len(dst_sg);
	src_avail = sg_dma_len(src_sg);

	/* Loop until we run out of entries... */
	for (;;) {
		/* Descriptor count is limited to 64K */
		len = min_t(size_t, src_avail, dst_avail);
		len = min_t(size_t, len, (size_t)SZ_64K);

		if (len > 0) {
			dst = sg_dma_address(dst_sg) +
				sg_dma_len(dst_sg) - dst_avail;
			src = sg_dma_address(src_sg) +
				sg_dma_len(src_sg) - src_avail;

			src_acc = min(ffs((u32)src | len) - 1, 4);
			dst_acc = min(ffs((u32)dst | len) - 1, 4);

			new = get_descriptor(dmac->engine);
			if (!new) {
				ch_dbg(dmac, "ERROR: No descriptor\n");
				goto fail;
			}

			init_descriptor(new, src, src_acc, dst, dst_acc, len);

			/* Link descriptors together */
			if (!first) {
				first = new;
				first->head = first;
				eob = first;
			} else {
				/* if segment addresses don't match then add an
				 * end of block descriptor */
				if (!desc_segment_match(dmac, prev, new)) {
					eob->ned = new; /* link EOB descriptors */
					eob = new;
					prev->hw.ch_config |= cpu_to_le32(DMA_CONFIG_EOB);
				}
				new->head = first;
				prev->hw.next_ptr = desc_to_paddr(dmac, new);
				prev->chain = new;
			}
			prev = new;

			/* update metadata */
			dst_avail -= len;
			src_avail -= len;
		}

		/* dst: Advance to next sg-entry */
		if (dst_avail == 0) {
			/* no more entries: we're done */
			if (--dst_nents == 0)
				break;
			/* fetch the next entry: if there are no more: done */
			dst_sg = sg_next(dst_sg);
			if (dst_sg == NULL)
				break;

			dst_avail = sg_dma_len(dst_sg);
		}

		/* src: Advance to next sg-entry */
		if (src_avail == 0) {
			/* no more entries: we're done */
			if (--src_nents == 0)
				break;
			/* fetch the next entry: if there are no more: done */
			src_sg = sg_next(src_sg);
			if (src_sg == NULL)
				break;

			src_avail = sg_dma_len(src_sg);
		}
	}

	/* Interrupt on last descriptor in chain */
	if (new) {
		new->hw.ch_config |= cpu_to_le32(DMA_CONFIG_END);
		return vchan_tx_prep(&dmac->vc, &first->vdesc, flags);
	}
fail:
	if (first)
		free_descriptor(&first->vdesc);
	return NULL;
}

/**
 * gpdma_prep_slave_sg - Prepares a transfer using an sg.
 *
 */
static struct dma_async_tx_descriptor *
gpdma_prep_slave_sg(struct dma_chan *chan,
		struct scatterlist *sgl, unsigned int sg_len,
		enum dma_transfer_direction dir, unsigned long flags,
		void *context)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);
	struct scatterlist dev_sg = {0};
	struct scatterlist *sg = NULL;
	u32 sg_size = 0;
	u32 i = 0;
	struct dma_async_tx_descriptor *tx = NULL;

	for_each_sg(sgl, sg, sg_len, i)
		sg_size += sg_dma_len(sg);

	sg_init_table(&dev_sg, 1);
	sg_dma_len(&dev_sg) = sg_size;

	if (dmac->cfg.direction == DMA_MEM_TO_DEV) {
		sg_dma_address(&dev_sg) = dmac->cfg.dst_addr;
		tx = gpdma_prep_sg(chan, &dev_sg, 1, sgl, sg_len, flags);
	} else {
		sg_dma_address(&dev_sg) = dmac->cfg.src_addr;
		tx = gpdma_prep_sg(chan, sgl, sg_len, &dev_sg, 1, flags);
	}

	return tx;
}

/**
 * gpdma_prep_memcpy - Prepares a memcpy operation.
 *
 */
static struct dma_async_tx_descriptor *
gpdma_prep_memcpy(struct dma_chan *chan,
		  dma_addr_t dst,
		  dma_addr_t src,
		  size_t size,
		  unsigned long dma_flags)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);
	struct gpdma_desc *first = NULL, *prev = NULL, *new;
	u32 src_acc, dst_acc;
	size_t len;

	if (size == 0)
		return NULL;

	do {
		new = get_descriptor(dmac->engine);
		if (new == NULL) {
			ch_dbg(dmac, "ERROR: No descriptor\n");
			goto fail;
		}

		len = min_t(size_t, size, (size_t)SZ_64K);

		/* Maximize access width based on address and length alignmet */
		src_acc = min(ffs((u32)src | len) - 1, 4);
		dst_acc = min(ffs((u32)dst | len) - 1, 4);

		init_descriptor(new, src, src_acc, dst, dst_acc, len);

		if (!first) {
			first = new;
			first->head = new;
		} else {
			new->head = first;
			prev->hw.next_ptr = desc_to_paddr(dmac, new);
			prev->chain = new;
		}
		prev = new;

		size -= len;
		src  += len;
		dst  += len;

	} while (size > 0);

	prev->hw.ch_config |= cpu_to_le32(DMA_CONFIG_END);

	return vchan_tx_prep(&dmac->vc, &first->vdesc, DMA_CTRL_ACK);

fail:
	if (first)
		free_descriptor(&first->vdesc);
	return NULL;
}

/**
 * gpdma_issue_pending - Push pending transactions to hardware.
 *
 */
static void gpdma_issue_pending(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&dmac->vc.lock, flags);
	if (vchan_issue_pending(&dmac->vc) && !dmac->active)
		gpdma_start(dmac);
	spin_unlock_irqrestore(&dmac->vc.lock, flags);
}

/**
 * gpdma_tx_status - Poll for transaction completion, the optional txstate
 * parameter can be supplied with a pointer to get a struct with auxiliary
 * transfer status information, otherwise the call will just return a simple
 * status code.
 */
static enum dma_status gpdma_tx_status(struct dma_chan *chan,
				       dma_cookie_t cookie,
				       struct dma_tx_state *txstate)
{
	return dma_cookie_status(chan, cookie, txstate);
}

static int gpdma_terminate_all(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = to_gpdma_chan(chan);

	if (!dmac)
		return -EINVAL;

	flush_channel(dmac);

	return 0;
}

static int setup_channel(struct gpdma_channel *dmac, struct device_node *child)
{
	struct gpdma_engine *engine = dmac->engine;
	int rc;

	dmac->base = engine->iobase + dmac->id * engine->chip->chregs_offset;
	dev_dbg(engine->dev, "channel%d base @ %p\n", dmac->id, dmac->base);

	/* Find the IRQ line, if it exists in the device tree */
	dmac->irq = irq_of_parse_and_map(child, 0);
	dev_dbg(engine->dev, "channel %d, irq %d\n", dmac->id, dmac->irq);
	rc = devm_request_irq(engine->dev, dmac->irq, gpdma_isr, 0,
			      "lsi-dma", dmac);
	if (rc) {
		dev_err(engine->dev, "failed to request_irq, error = %d\n", rc);
		return rc;
	}
	/* Initialize the virt-channel */
	dmac->vc.desc_free = free_descriptor;
	vchan_init(&dmac->vc, &engine->dma_device);

	return 0;
}

static struct lsidma_hw lsi_dma32 = {
	.num_channels   = 2,
	.chregs_offset  = 0x80,
	.genregs_offset = 0xF00,
	.flags          = (LSIDMA_NEXT_FULL |
			   LSIDMA_SEG_REGS)
};

static struct lsidma_hw lsi_dma31 = {
	.num_channels   = 4,
	.chregs_offset  = 0x40,
	.genregs_offset = 0x400,
	.flags          = 0
};

static const struct of_device_id gpdma_of_ids[] = {
	{
		.compatible = "lsi,dma32",
		.data       = &lsi_dma32
	},
	{
		.compatible = "lsi,dma31",
		.data       = &lsi_dma31
	},
	{
		.compatible = "gp-dma,acp-dma",
		.data       = &lsi_dma31
	},
	{
		.compatible = "gp-dma,acp-gpdma",
		.data       = &lsi_dma31
	},
	{ }
};

static int gpdma_of_probe(struct platform_device *op)
{
	struct gpdma_engine *engine;
	struct dma_device   *dma;
	struct device_node *child;
	struct resource *res;
	const struct of_device_id *match;
	int rc = -ENOMEM;
	int id = 0;

	match = of_match_device(gpdma_of_ids, &op->dev);
	if (!match)
		return -EINVAL;

	engine = devm_kzalloc(&op->dev, sizeof(*engine), GFP_KERNEL);
	if (!engine)
		return -ENOMEM;

	spin_lock_init(&engine->lock);
	engine->dev = &op->dev;
	engine->chip = (struct lsidma_hw *)match->data;

	/* Initialize dma_device struct */
	dma = &engine->dma_device;
	dma->dev = &op->dev;
	dma_cap_zero(dma->cap_mask);
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma->copy_align = 2;
	dma->chancnt = engine->chip->num_channels;
	dma->device_alloc_chan_resources = gpdma_alloc_chan_resources;
	dma->device_free_chan_resources = gpdma_free_chan_resources;
	dma->device_tx_status = gpdma_tx_status;
	dma->device_prep_dma_memcpy = gpdma_prep_memcpy;
	dma->device_prep_slave_sg = gpdma_prep_slave_sg;
	dma->device_issue_pending = gpdma_issue_pending;
	dma->device_terminate_all = gpdma_terminate_all;
	dma->device_config = gpdma_slave_config;
	INIT_LIST_HEAD(&dma->channels);

	/* Map device I/O memory
	 */
	res = platform_get_resource(op, IORESOURCE_MEM, 0);
	engine->iobase = devm_ioremap_resource(&op->dev, res);
	if (IS_ERR(engine->iobase))
		return PTR_ERR(engine->iobase);
	dev_dbg(&op->dev, "mapped base @ %p\n", engine->iobase);

	res = platform_get_resource(op, IORESOURCE_MEM, 1);
	if (res) {
		engine->gpreg = devm_ioremap_nocache(&op->dev,
						     res->start,
						     resource_size(res));
		if (IS_ERR(engine->gpreg))
			return PTR_ERR(engine->gpreg);
		dev_dbg(&op->dev, "mapped gpreg @ %p\n", engine->gpreg);
	}

	engine->err_irq = platform_get_irq(op, 1);
	if (engine->err_irq) {
		rc = devm_request_irq(&op->dev, engine->err_irq,
				      gpdma_isr_err, 0, "lsi-dma-err", engine);
		if (rc) {
			dev_err(engine->dev, "failed to request irq%d\n",
				engine->err_irq);
			engine->err_irq = 0;
		}
	}

	/* General registes at device specific offset */
	engine->gbase = engine->iobase + engine->chip->genregs_offset;

	rc = alloc_desc_table(engine);
	if (rc)
		return rc;

	/* Setup channels */
	for_each_child_of_node(op->dev.of_node, child) {
		struct gpdma_channel *dmac = &engine->channel[id];

		if (id >= engine->chip->num_channels) {
			dev_dbg(engine->dev, "Too many channels (%d)\n", id);
			return -ENODEV;
		}

		dmac->id = id;
		dmac->engine = engine;
		rc = setup_channel(dmac, child);
		if (rc)
			return rc;
		++id;
	}

	soft_reset(engine);

	rc = dma_async_device_register(&engine->dma_device);
	if (rc) {
		dev_err(engine->dev, "unable to register\n");
		return rc;
	}

	device_create_file(&op->dev, &dev_attr_soft_reset);
	dev_set_drvdata(&op->dev, engine);

	return 0;
}

static int gpdma_of_remove(struct platform_device *op)
{
	struct gpdma_engine *engine = dev_get_drvdata(&op->dev);

	dev_dbg(&op->dev, "%s\n", __func__);

	device_remove_file(&op->dev, &dev_attr_soft_reset);
	dma_async_device_unregister(&engine->dma_device);
	free_desc_table(engine);
	dev_set_drvdata(&op->dev, NULL);

	return 0;
}

static struct platform_driver gpdma_of_driver = {
	.driver = {
		.name           = "lsi-dma32",
		.owner          = THIS_MODULE,
		.of_match_table = gpdma_of_ids,
	},
	.probe  = gpdma_of_probe,
	.remove = gpdma_of_remove,
};

module_platform_driver(gpdma_of_driver);

MODULE_DESCRIPTION("LSI DMA driver");
MODULE_LICENSE("GPL");
