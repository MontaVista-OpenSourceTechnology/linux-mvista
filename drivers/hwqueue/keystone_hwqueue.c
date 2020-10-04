/*
 * Keystone hardware queue driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/hwqueue.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include "hwqueue_internal.h"
#include "keystone_hwqueue.h"

static inline int khwq_pdsp_wait(u32 * __iomem addr, unsigned timeout,
				 u32 flags)
{
	unsigned long end_time;
	u32 val = 0;
	int ret;

	end_time = jiffies + msecs_to_jiffies(timeout);
	while (jiffies < end_time) {
		val = __raw_readl(addr);
		if (flags)
			val &= flags;
		if (!val)
			break;
		cpu_relax();
	}
	ret = val ? -ETIMEDOUT : 0;

	return ret;
}

static int khwq_match(struct hwqueue_instance *inst, unsigned flags)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range;
	int score = 0;

	if (!kq)
		return -ENOENT;

	range = kq->range;
	if (!range)
		return -ENOENT;

	if (range->flags & RANGE_RESERVED)
		score += 1000;

	if ((range->flags & RANGE_HAS_ACCUMULATOR) &&
	    !(flags & O_HIGHTHROUGHPUT))
		score += 100;
	if (!(range->flags & RANGE_HAS_ACCUMULATOR) &&
	    (flags & O_HIGHTHROUGHPUT))
		score += 100;

	if ((range->flags & RANGE_HAS_IRQ) &&
	    !(flags & (O_LOWLATENCY | O_HIGHTHROUGHPUT)))
		score += 100;
	if (!(range->flags & RANGE_HAS_IRQ) &&
	    (flags & (O_LOWLATENCY | O_HIGHTHROUGHPUT)))
		score += 100;

	return score;
}

static irqreturn_t khwq_int_handler(int irq, void *_instdata)
{
	struct hwqueue_instance *inst = _instdata;

	hwqueue_notify(inst);

	return IRQ_HANDLED;
}

static void khwq_set_notify(struct hwqueue_instance *inst, bool enabled)
{
	struct khwq_range_info *range;
	struct khwq_instance *kq;
	unsigned queue;

	kq    = hwqueue_inst_to_priv(inst);
	range = kq->range;

	if (range->ops) {
		if (range->ops->set_notify)
			range->ops->set_notify(range, inst, enabled);
	} else if (range->flags & RANGE_HAS_IRQ) {
		queue = hwqueue_inst_to_id(inst) - range->queue_base;
		if (enabled)
			enable_irq(range->irqs[queue].irq);
		else
			disable_irq_nosync(range->irqs[queue].irq);
	} else
		hwqueue_set_poll(inst, enabled);
}

static int khwq_setup_irq(struct khwq_range_info *range,
			  struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	unsigned queue = hwqueue_inst_to_id(inst) - range->queue_base;
	int ret = 0, irq;
	unsigned long cpu_map;

	if (range->flags & RANGE_HAS_IRQ) {
		irq = range->irqs[queue].irq;
		cpu_map = range->irqs[queue].cpu_map;

		ret = request_irq(irq, khwq_int_handler, 0, kq->irq_name, inst);
		if (ret >= 0)
			disable_irq(irq);

		if (cpu_map)
			if (irq_set_affinity(irq, to_cpumask(&cpu_map)))
				dev_warn(range->kdev->dev,
						"Failed to set IRQ affinity\n");
	}

	return ret;
}

static void khwq_free_irq(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;
	int irq;

	if (range->flags & RANGE_HAS_IRQ) {
		irq = range->irqs[id].irq;
		free_irq(irq, inst);
	}
}

static int khwq_open(struct hwqueue_instance *inst, unsigned flags)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;

	if (!range->ops)
		return khwq_setup_irq(range, inst);
	else if (range->ops->open_queue)
		return range->ops->open_queue(range, inst, flags);
	else
		return 0;
}

static void khwq_close(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;

	if (!range->ops)
		khwq_free_irq(inst);
	else if (range->ops->close_queue)
		range->ops->close_queue(range, inst);
}

static inline struct khwq_region *
khwq_find_region_by_virt(struct khwq_device *kdev, struct khwq_instance *kq,
			 void *virt)
{
	struct khwq_region *region;

	if (likely(kq->last && kq->last->virt_start <= virt &&
		   kq->last->virt_end > virt))
		return kq->last;

	for_each_region(kdev, region) {
		if (region->virt_start <= virt && region->virt_end > virt) {
			kq->last = region;
			return kq->last;
		}
	}

	return NULL;
}

static inline struct khwq_region *
khwq_find_region_by_dma(struct khwq_device *kdev, struct khwq_instance *kq,
			dma_addr_t dma)
{
	struct khwq_region *region;

	if (likely(kq->last && kq->last->dma_start <= dma &&
		   kq->last->dma_end > dma))
		return kq->last;

	for_each_region(kdev, region) {
		if (region->dma_start <= dma && region->dma_end > dma) {
			kq->last = region;
			return kq->last;
		}
	}

	return NULL;
}

static int khwq_push(struct hwqueue_instance *inst, dma_addr_t dma,
		     unsigned size, unsigned flags)
{
	struct khwq_qmgr_info *qmgr;
	unsigned id;
	u32 val;

	qmgr = khwq_find_qmgr(inst);
	if (!qmgr)
		return -ENODEV;

	id = hwqueue_inst_to_id(inst) - qmgr->start_queue;

	val = (u32)dma | ((size / 16) - 1);

	writel(val, &qmgr->reg_push[id].ptr_size_thresh);

	return 0;
}

static dma_addr_t khwq_pop(struct hwqueue_instance *inst, unsigned *size,
			   unsigned flags)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_qmgr_info *qmgr;
	u32 val, desc_size, idx;
	dma_addr_t dma;
	unsigned id;

	qmgr = khwq_find_qmgr(inst);
	if (unlikely(!qmgr))
		return 0;

	id = hwqueue_inst_to_id(inst) - qmgr->start_queue;

	/* are we accumulated? */
	if (kq->descs) {
		if (unlikely(atomic_dec_return(&kq->desc_count) < 0)) {
			atomic_inc(&kq->desc_count);
			return 0;
		}

		idx  = atomic_inc_return(&kq->desc_head);
		idx &= ACC_DESCS_MASK;

		val = kq->descs[idx];
	} else {
		val = __raw_readl(&qmgr->reg_pop[id].ptr_size_thresh);
		if (unlikely(!val))
			return 0;
	}

	dma = val & DESC_PTR_MASK;
	desc_size = ((val & DESC_SIZE_MASK) + 1) * 16;

	if (unlikely(size))
		*size = desc_size;

	return dma;
}

static int khwq_get_count(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_qmgr_info *qmgr;
	unsigned id;

	qmgr = khwq_find_qmgr(inst);
	if (unlikely(!qmgr))
		return -EINVAL;

	id = hwqueue_inst_to_id(inst) - qmgr->start_queue;

	return (__raw_readl(&qmgr->reg_peek[id].entry_count) +
		atomic_read(&kq->desc_count));
}

static int khwq_flush(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_qmgr_info *qmgr;
	unsigned id;

	qmgr = khwq_find_qmgr(inst);
	if (!qmgr)
		return -ENODEV;

	id = hwqueue_inst_to_id(inst) - qmgr->start_queue;

	atomic_set(&kq->desc_count, 0);
	writel(0, &qmgr->reg_push[id].ptr_size_thresh);
	return 0;
}

static int khwq_map(struct hwqueue_instance *inst, void *data, unsigned size,
		    dma_addr_t *dma_ptr, unsigned *size_ptr)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_region *region;
	dma_addr_t dma;

	region = khwq_find_region_by_virt(kdev, kq, data);
	if (!region)
		return -EINVAL;

	if (unlikely(!region || size > region->desc_size))
		return -EINVAL;

	size = min(size, region->desc_size);
	size = ALIGN(size, SMP_CACHE_BYTES);
	dma = region->dma_start + (data - region->virt_start);

	if (WARN_ON(dma & DESC_SIZE_MASK))
		return -EINVAL;
	dma_sync_single_for_device(kdev->dev, dma, size, DMA_TO_DEVICE);

	*dma_ptr = dma;
	*size_ptr = size;
	return 0;
}

static void *khwq_unmap(struct hwqueue_instance *inst, dma_addr_t dma,
			unsigned desc_size)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_region *region;
	void *data;

	region = khwq_find_region_by_dma(kdev, kq, dma);
	if (WARN_ON(!region))
		return NULL;

	desc_size = min(desc_size, region->desc_size);

	data = region->virt_start + (dma - region->dma_start);

	dma_sync_single_for_cpu(kdev->dev, dma, desc_size, DMA_FROM_DEVICE);

	prefetch(data);

	return data;
}

static struct hwqueue_device_ops khdev_ops = {
	.match		= khwq_match,
	.open		= khwq_open,
	.set_notify	= khwq_set_notify,
	.close		= khwq_close,
};

static struct hwqueue_inst_ops khdev_inst_ops = {
	.push		= khwq_push,
	.pop		= khwq_pop,
	.get_count	= khwq_get_count,
	.flush		= khwq_flush,
	.map		= khwq_map,
	.unmap		= khwq_unmap,
};

static struct khwq_region *
khwq_find_match_region(struct khwq_device *kdev, struct khwq_pool_info *pool)
{
	struct khwq_region *region;

	for_each_region(kdev, region) {
		if (region->id != pool->region_id)
			continue;
		if ((region->num_desc - region->used_desc) < pool->num_desc)
			continue;
		return region;
	}
	return NULL;
}

/* Map the requested pools to regions, creating regions as we go */
static void khwq_map_pools(struct khwq_device *kdev)
{
	struct khwq_region *region;
	struct khwq_pool_info *pool;

	for_each_pool(kdev, pool) {
		/* find a matching region*/
		region = khwq_find_match_region(kdev, pool);
		if (!region) {
			dev_err(kdev->dev, "failed to set up pool %s %d\n",
				pool->name, pool->num_desc);
			continue;
		}

		/* link this pool to the region... */
		pool->region = region;
		pool->region_offset = region->used_desc;
		region->used_desc += pool->num_desc;

		dev_dbg(kdev->dev, "pool %s: num:%d, size:%d, region:%d \n",
			pool->name, pool->num_desc,
			pool->desc_size, region->id);
	}
}

static int khwq_setup_region(struct khwq_device *kdev,
				       struct khwq_region *region)
{
	unsigned hw_num_desc, hw_desc_size, size;
	int id = region->id;
	struct khwq_reg_region __iomem  *regs;
	struct khwq_qmgr_info *qmgr;
	struct page *page;

	/* unused region? */
	if (!region->num_desc) {
		dev_warn(kdev->dev, "unused region %s\n", region->name);
		return 0;
	}

	/* get hardware descriptor value */
	hw_num_desc = ilog2(region->num_desc - 1) + 1;

	/* did we force fit ourselves into nothingness? */
	if (region->num_desc < 32) {
		region->num_desc = 0;
		dev_warn(kdev->dev, "too few descriptors in region %s\n", region->name);
		return 0;
	}

	size = region->num_desc * region->desc_size;
	region->virt_start = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
	if (!region->virt_start) {
		region->num_desc = 0;
		dev_err(kdev->dev, "memory alloc failed for region %s\n", region->name);
		return 0;
	}
	region->virt_end = region->virt_start + size;
	page = virt_to_page(region->virt_start);

	region->dma_start = dma_map_page(kdev->dev, page, 0, size,
					 DMA_BIDIRECTIONAL);
	if (dma_mapping_error(kdev->dev, region->dma_start)) {
		region->num_desc = 0;
		free_pages_exact(region->virt_start, size);
		dev_err(kdev->dev, "dma map failed for region %s\n", region->name);
		return 0;
	}
	region->dma_end  = region->dma_start + size;

	dev_info(kdev->dev,
		"region %s (%d): size:%d, link:%d@%d, phys:%08llx-%08llx, virt:%p-%p\n",
		region->name, id, region->desc_size, region->num_desc,
		region->link_index, region->dma_start, region->dma_end,
		region->virt_start, region->virt_end);

	hw_desc_size = (region->desc_size / 16) - 1;
	hw_num_desc -= 5;

	for_each_qmgr(kdev, qmgr) {
		regs = qmgr->reg_region + id;
		__raw_writel(region->dma_start, &regs->base);
		__raw_writel(region->link_index, &regs->start_index);
		__raw_writel(hw_desc_size << 16 | hw_num_desc,
			     &regs->size_count);
	}

	return region->num_desc;
}

static const char *khwq_find_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = node->name;
	if (!name)
		name = "unknown";
	return name;
}

static int khwq_setup_regions(struct khwq_device *kdev,
					struct device_node *regions)
{
	struct device *dev = kdev->dev;
	struct khwq_region *region;
	struct device_node *child;
	u32 temp[2];
	int ret;

	for_each_child_of_node(regions, child) {
		region = devm_kzalloc(dev, sizeof(*region), GFP_KERNEL);
		if (!region) {
			dev_err(dev, "out of memory allocating region\n");
			return -ENOMEM;
		}

		region->name = khwq_find_name(child);
		of_property_read_u32(child, "id", &region->id);
		ret = of_property_read_u32_array(child, "values", temp, 2);
		if (!ret) {
			region->num_desc  = temp[0];
			region->desc_size = temp[1];
		} else {
			dev_err(dev, "invalid region info %s\n", region->name);
			devm_kfree(dev, region);
			continue;
		}

		if (!of_get_property(child, "link-index", NULL)) {
			dev_err(dev, "No link info for %s\n", region->name);
			devm_kfree(dev, region);
			continue;
		}
		ret = of_property_read_u32(child, "link-index",
					   &region->link_index);
		if (ret) {
			dev_err(dev, "link index not found for %s\n",
				region->name);
			devm_kfree(dev, region);
			continue;
		}

		list_add_tail(&region->list, &kdev->regions);
	}
	if (list_empty(&kdev->regions)) {
		dev_err(dev, "no valid region information found\n");
		return -ENODEV;
	}

	khwq_map_pools(kdev);

	/* Next, we run through the regions and set things up */
	for_each_region(kdev, region)
		khwq_setup_region(kdev, region);

	return 0;
}

/* carve out descriptors and push into named queues */
static void khwq_fill_pools(struct khwq_device *kdev)
{
	struct khwq_region *region;
	struct khwq_pool_info *pool;
	dma_addr_t dma_addr;
	unsigned dma_size;
	int ret, i;

	for_each_pool(kdev, pool) {
		pool->queue = hwqueue_open(pool->name, HWQUEUE_ANY,
					   O_CREAT | O_RDWR | O_NONBLOCK);
		if (IS_ERR_OR_NULL(pool->queue)) {
			dev_err(kdev->dev,
				"failed to open queue for pool %s, error %ld\n",
				pool->name, PTR_ERR(pool->queue));
			pool->queue = NULL;
			continue;
		}

		region = pool->region;

		if (!region || !region->num_desc) {
			dev_err(kdev->dev, "no region for pool %s\n",
				pool->name);
			continue;
		}

		pool->desc_size = region->desc_size;
		for (i = 0; i < pool->num_desc; i++) {
			int index = pool->region_offset + i;
			void *desc;

			desc = region->virt_start + region->desc_size * index;
			ret = hwqueue_map(pool->queue, desc, pool->desc_size,
					  &dma_addr, &dma_size);
			if (ret < 0) {
				WARN_ONCE(ret, "failed map pool queue %s\n",
					  pool->name);
				continue;
			}
			ret = hwqueue_push(pool->queue, dma_addr, dma_size, 0);
			WARN_ONCE(ret, "failed push to pool queue %s\n",
				  pool->name);
		}
	}
}

static int khwq_get_link_ram(struct khwq_device *kdev,
				       const char *name,
				       struct khwq_link_ram_block *block)
{
	struct platform_device *pdev = to_platform_device(kdev->dev);
	struct device_node *node = pdev->dev.of_node;
	u32 temp[2];

	/*
	 * Note: link ram resources are specified in "entry" sized units. In
	 * reality, although entries are ~40bits in hardware, we treat them as
	 * 64-bit entities here.
	 *
	 * For example, to specify the internal link ram for Keystone-I class
	 * devices, we would set the linkram0 resource to 0x80000-0x83fff.
	 *
	 * This gets a bit weird when other link rams are used.  For example,
	 * if the range specified is 0x0c000000-0x0c003fff (i.e., 16K entries
	 * in MSMC SRAM), the actual memory used is 0x0c000000-0x0c020000,
	 * which accounts for 64-bits per entry, for 16K entries.
	 */
	if (!of_property_read_u32_array(node, name , temp, 2)) {
		if (temp[0]) {
			/*
			 * queue_base specified => using internal or onchip
			 * link ram WARNING - we do not "reserve" this block
			 */
			block->phys = (dma_addr_t)temp[0];
			block->virt = NULL;
			block->size = temp[1];
		} else {
			block->size = temp[1];
			/* queue_base not specific => allocate requested size */
			block->virt = dmam_alloc_coherent(kdev->dev,
							  8 * block->size, &block->phys,
							  GFP_KERNEL);
			if (!block->virt) {
				dev_err(kdev->dev, "failed to alloc linkram\n");
				return -ENOMEM;
			}
		}
	} else
		return -ENODEV;
	return 0;
}

static int khwq_setup_link_ram(struct khwq_device *kdev)
{
	struct khwq_link_ram_block *block;
	struct khwq_qmgr_info *qmgr;

	for_each_qmgr(kdev, qmgr) {
		block = &kdev->link_rams[0];
		dev_dbg(kdev->dev, "linkram0: phys:%08llx, virt:%p, size:%x\n",
			block->phys, block->virt, block->size);
		__raw_writel(block->phys, &qmgr->reg_config->link_ram_base0);
		__raw_writel(block->size - 1, &qmgr->reg_config->link_ram_size0);

		block++;
		if (!block->size)
			continue;

		dev_dbg(kdev->dev, "linkram1: phys:%08llx, virt:%p, size:%x\n",
			block->phys, block->virt, block->size);
		__raw_writel(block->phys, &qmgr->reg_config->link_ram_base1);
	}

	return 0;
}

static int khwq_setup_queue_range(struct khwq_device *kdev,
				 struct device_node *node)
{
	struct device *dev = kdev->dev;
	struct khwq_range_info *range;
	struct khwq_qmgr_info *qmgr;
	u32 temp[2], start, end, id, index;
	int ret, i;

	range = devm_kzalloc(dev, sizeof(*range), GFP_KERNEL);
	if (!range) {
		dev_err(dev, "out of memory allocating range\n");
		return -ENOMEM;
	}

	range->kdev = kdev;
	range->name = khwq_find_name(node);

	ret = of_property_read_u32_array(node, "values", temp, 2);
	if (!ret) {
		range->queue_base = temp[0] - kdev->base_id;
		range->num_queues = temp[1];
	} else {
		dev_err(dev, "invalid queue range %s\n", range->name);
		devm_kfree(dev, range);
		return -EINVAL;
	}

	for (i = 0; i < RANGE_MAX_IRQS; i++) {
		struct of_phandle_args oirq;

		if (of_irq_parse_one(node, i, &oirq))
			break;

		range->irqs[i].irq = irq_create_of_mapping(&oirq);

		/* Check for a valid IRQ */
		if (range->irqs[i].irq == IRQ_NONE)
			break;

		range->num_irqs++;

		/* If it is an SPI interrupt then extract the interrupt
		 * cpu mask. This mask will be used later to set the
		 * CPU affinity for the IRQ.
		 * For details on encoding of interrupt-cells for ARM GIC,
		 * refer to Documentation/devicetree/bindings/arm/gic.txt.
		 */
		if (oirq.args[0])
			continue;

		if (oirq.args_count >= 3)
			range->irqs[i].cpu_map =
				(oirq.args[2] & 0x0000ff00) >> 8;
	}

	range->num_irqs = min(range->num_irqs, range->num_queues);
	if (range->num_irqs)
		range->flags |= RANGE_HAS_IRQ;

	if (of_get_property(node, "reserved", NULL))
		range->flags |= RANGE_RESERVED;

	if (of_get_property(node, "accumulator", NULL)) {
		ret = khwq_init_acc_range(kdev, node, range);
		if (ret < 0) {
			devm_kfree(dev, range);
			return ret;
		}
	}

	if (of_get_property(node, "qos-cfg", NULL)) {
		ret = khwq_init_qos_range(kdev, node, range);
		if (ret < 0) {
			devm_kfree(dev, range);
			return ret;
		}
	}

	if (of_get_property(node, "pdsp-acc-reclamation", NULL)) {
		ret = khwq_set_acc_reclamation(kdev, node, range);
		if (ret < 0) {
			devm_kfree(dev, range);
			return ret;
		}
	}

	/* set threshold to 1, and flush out the queues */
	for_each_qmgr(kdev, qmgr) {
		start = max(qmgr->start_queue, range->queue_base);
		end   = min(qmgr->start_queue + qmgr->num_queues,
			    range->queue_base + range->num_queues);
		for (id = start; id < end; id++) {
			index = id - qmgr->start_queue;
			writel(THRESH_GTE | 1,
			       &qmgr->reg_peek[index].ptr_size_thresh);
			writel(0, &qmgr->reg_push[index].ptr_size_thresh);
		}
	}

	list_add_tail(&range->list, &kdev->queue_ranges);

	dev_dbg(dev, "added range %s: %d-%d, %d irqs%s%s%s\n",
		range->name, range->queue_base,
		range->queue_base + range->num_queues - 1,
		range->num_irqs,
		(range->flags & RANGE_HAS_IRQ) ? ", has irq" : "",
		(range->flags & RANGE_RESERVED) ? ", reserved" : "",
		(range->flags & RANGE_HAS_ACCUMULATOR) ? ", acc" : "");

	return 0;
}

static void khwq_free_queue_range(struct khwq_device *kdev,
				  struct khwq_range_info *range)
{
	if (range->ops && range->ops->free_range)
		range->ops->free_range(range);
	list_del(&range->list);
	devm_kfree(kdev->dev, range);
}

static int khwq_setup_queue_ranges(struct khwq_device *kdev,
				  struct device_node *queues)
{
	struct device_node *child;
	int ret;

	for_each_child_of_node(queues, child) {
		ret = khwq_setup_queue_range(kdev, child);
		/* return value ignored, we init the rest... */
	}

	/* ... and barf if they all failed! */
	if (list_empty(&kdev->queue_ranges)) {
		dev_err(kdev->dev, "no valid queue range found\n");
		return -ENODEV;
	}

	return 0;
}

static void khwq_free_queue_ranges(struct khwq_device *kdev)
{
	struct khwq_range_info *range;

	for (;;) {
		range = first_queue_range(kdev);
		if (!range)
			break;
		khwq_free_queue_range(kdev, range);
	}
}

static int khwq_setup_pools(struct khwq_device *kdev, struct device_node *pools)
{
	struct device *dev = kdev->dev;
	struct khwq_pool_info *pool;
	struct device_node *child;
	u32 temp[2];
	int ret;

	for_each_child_of_node(pools, child) {

		pool = devm_kzalloc(dev, sizeof(*pool), GFP_KERNEL);
		if (!pool) {
			dev_err(dev, "out of memory allocating pool\n");
			return -ENOMEM;
		}

		pool->name = khwq_find_name(child);

		ret = of_property_read_u32_array(child, "values", temp, 2);
		if (!ret) {
			pool->num_desc  = temp[0];
			pool->desc_size = temp[1];
		} else {
			dev_err(dev, "invalid queue pool %s\n", pool->name);
			devm_kfree(dev, pool);
			continue;
		}

		ret = of_property_read_u32(child, "region-id", &pool->region_id);
		if (ret < 0) {
			dev_err(dev, "invalid region id for pool %s\n", pool->name);
			devm_kfree(dev, pool);
			continue;
		}

		list_add_tail(&pool->list, &kdev->pools);

		dev_info(dev, "added pool %s: %d descriptors of size %d\n",
			pool->name, pool->num_desc, pool->desc_size);
	}

	if (list_empty(&kdev->pools)) {
		dev_err(dev, "no valid descriptor pool found\n");
		return -ENODEV;
	}

	return 0;
}

static int khwq_init_qmgrs(struct khwq_device *kdev, struct device_node *qmgrs)
{
	struct device *dev = kdev->dev;
	struct khwq_qmgr_info *qmgr;
	struct device_node *child;
	u32 temp[2];
	int ret;

	for_each_child_of_node(qmgrs, child) {
		qmgr = devm_kzalloc(dev, sizeof(*qmgr), GFP_KERNEL);
		if (!qmgr) {
			dev_err(dev, "out of memory allocating qmgr\n");
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(child, "managed-queues",
						 temp, 2);
		if (!ret) {
			qmgr->start_queue = temp[0];
			qmgr->num_queues = temp[1];
		} else {
			dev_err(dev, "invalid qmgr queue range\n");
			devm_kfree(dev, qmgr);
			continue;
		}

		dev_info(dev, "qmgr start queue %d, number of queues %d\n",
		       qmgr->start_queue, qmgr->num_queues);

		qmgr->reg_peek		= of_iomap(child, 0);
		qmgr->reg_status	= of_iomap(child, 1);
		qmgr->reg_config	= of_iomap(child, 2);
		qmgr->reg_region	= of_iomap(child, 3);
		qmgr->reg_push		= of_iomap(child, 4);
		qmgr->reg_pop		= of_iomap(child, 5);

		if (!qmgr->reg_peek || !qmgr->reg_status || !qmgr->reg_config ||
		    !qmgr->reg_region || !qmgr->reg_push || !qmgr->reg_pop) {
			dev_err(dev, "failed to map qmgr regs\n");
			if (qmgr->reg_peek)
				iounmap(qmgr->reg_peek);
			if (qmgr->reg_status)
				iounmap(qmgr->reg_status);
			if (qmgr->reg_config)
				iounmap(qmgr->reg_config);
			if (qmgr->reg_region)
				iounmap(qmgr->reg_region);
			if (qmgr->reg_push)
				iounmap(qmgr->reg_push);
			if (qmgr->reg_pop)
				iounmap(qmgr->reg_pop);
			kfree(qmgr);
			continue;
		}

		list_add_tail(&qmgr->list, &kdev->qmgrs);

		dev_info(dev, "added qmgr start queue %d, num of queues %d, "
				"reg_peek %p, reg_status %p, reg_config %p, "
				"reg_region %p, reg_push %p, reg_pop %p\n",
				qmgr->start_queue, qmgr->num_queues,
				qmgr->reg_peek, qmgr->reg_status,
				qmgr->reg_config, qmgr->reg_region,
				qmgr->reg_push, qmgr->reg_pop);
	}

	return 0;
}

static int khwq_init_pdsps(struct khwq_device *kdev, struct device_node *pdsps)
{
	struct device *dev = kdev->dev;
	struct khwq_pdsp_info *pdsp;
	struct device_node *child;
	int ret;

	for_each_child_of_node(pdsps, child) {

		pdsp = devm_kzalloc(dev, sizeof(*pdsp), GFP_KERNEL);
		if (!pdsp) {
			dev_err(dev, "out of memory allocating pdsp\n");
			return -ENOMEM;
		}

		pdsp->name = khwq_find_name(child);

		ret = of_property_read_string(child, "firmware",
					      &pdsp->firmware);
		if (ret < 0 || !pdsp->firmware) {
			dev_err(dev, "unknown firmware for pdsp %s\n",
				pdsp->name);
			kfree(pdsp);
			continue;
		}
		dev_dbg(dev, "pdsp name %s fw name :%s\n",
			pdsp->name, pdsp->firmware);

		pdsp->iram	= of_iomap(child, 0);
		pdsp->regs	= of_iomap(child, 1);
		pdsp->intd	= of_iomap(child, 2);
		pdsp->command	= of_iomap(child, 3);
		if (!pdsp->command || !pdsp->iram || !pdsp->regs || !pdsp->intd) {
			dev_err(dev, "failed to map pdsp %s regs\n",
				pdsp->name);
			if (pdsp->command)
				devm_iounmap(dev, pdsp->command);
			if (pdsp->iram)
				devm_iounmap(dev, pdsp->iram);
			if (pdsp->regs)
				devm_iounmap(dev, pdsp->regs);
			if (pdsp->intd)
				devm_iounmap(dev, pdsp->intd);
			kfree(pdsp);
			continue;
		}
		of_property_read_u32(child, "id", &pdsp->id);

		list_add_tail(&pdsp->list, &kdev->pdsps);

		dev_dbg(dev, "added pdsp %s: command %p, iram %p, "
			"regs %p, intd %p, firmware %s\n",
			pdsp->name, pdsp->command, pdsp->iram, pdsp->regs,
			pdsp->intd, pdsp->firmware);
	}

	return 0;
}


static int khwq_stop_pdsp(struct khwq_device *kdev,
			  struct khwq_pdsp_info *pdsp)
{
	u32 val, timeout = 1000;
	int ret;

	val = __raw_readl(&pdsp->regs->control) & ~PDSP_CTRL_ENABLE;
	__raw_writel(val, &pdsp->regs->control);

	ret = khwq_pdsp_wait(&pdsp->regs->control, timeout, PDSP_CTRL_RUNNING);

	if (ret < 0) {
		dev_err(kdev->dev, "timed out on pdsp %s stop\n", pdsp->name);
		return ret;
	}
	return 0;
}

static int khwq_load_pdsp(struct khwq_device *kdev,
			  struct khwq_pdsp_info *pdsp)
{
	int i, ret, fwlen;
	const struct firmware *fw;
	u32 *fwdata;

	ret = request_firmware(&fw, pdsp->firmware, kdev->dev);
	if (ret) {
		dev_err(kdev->dev, "failed to get firmware %s for pdsp %s\n",
			pdsp->firmware, pdsp->name);
		return ret;
	}

	__raw_writel(pdsp->id + 1, pdsp->command + 0x18);

	/* download the firmware */
	fwdata = (u32 *)fw->data;
	fwlen = (fw->size + sizeof(u32) - 1) / sizeof(u32);

	for (i = 0; i < fwlen; i++)
		__raw_writel(be32_to_cpu(fwdata[i]), pdsp->iram + i);

	release_firmware(fw);

	return 0;
}

static int khwq_start_pdsp(struct khwq_device *kdev,
			   struct khwq_pdsp_info *pdsp)
{
	u32 val, timeout = 1000;
	int ret;

	/* write a command for sync */
	__raw_writel(0xffffffff, pdsp->command);
	while (__raw_readl(pdsp->command) != 0xffffffff)
		cpu_relax();

	/* soft reset the PDSP */
	val  = __raw_readl(&pdsp->regs->control);
	val &= ~(PDSP_CTRL_PC_MASK | PDSP_CTRL_SOFT_RESET);
	__raw_writel(val, &pdsp->regs->control);

	/* enable pdsp */
	val = __raw_readl(&pdsp->regs->control) | PDSP_CTRL_ENABLE;
	__raw_writel(val, &pdsp->regs->control);

	/* wait for command register to clear */
	ret = khwq_pdsp_wait(pdsp->command, timeout, 0);

	if (ret < 0) {
		dev_err(kdev->dev, "timed out on pdsp %s command register wait\n",
			pdsp->name);
		return ret;
	}
	return 0;
}

static void khwq_stop_pdsps(struct khwq_device *kdev)
{
	struct khwq_pdsp_info *pdsp;

	/* disable all pdsps */
	for_each_pdsp(kdev, pdsp)
		khwq_stop_pdsp(kdev, pdsp);
}

static int khwq_start_pdsps(struct khwq_device *kdev)
{
	struct khwq_pdsp_info *pdsp;
	int ret;

	khwq_stop_pdsps(kdev);

	/* now load them all */
	for_each_pdsp(kdev, pdsp) {
		ret = khwq_load_pdsp(kdev, pdsp);
		if (ret < 0)
			return ret;
	}

	for_each_pdsp(kdev, pdsp) {
		ret = khwq_start_pdsp(kdev, pdsp);
		WARN_ON(ret);
	}

	return 0;
}

static int khwq_init_queue(struct khwq_device *kdev,
			   struct khwq_range_info *range,
			   struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;

	kq->kdev = kdev;
	kq->range = range;
	kq->irq_num = -1;

	inst->ops = &range->inst_ops;

	scnprintf(kq->irq_name, sizeof(kq->irq_name),
		  "hwqueue-%d", range->queue_base + id);

	if (range->ops && range->ops->init_queue)
		return range->ops->init_queue(range, inst);
	else
		return 0;
}

static int khwq_init_queues(struct khwq_device *kdev)
{
	struct hwqueue_device *hdev = to_hdev(kdev);
	struct khwq_range_info *range;
	int id, ret;

	for_each_queue_range(kdev, range) {
		range->inst_ops = khdev_inst_ops;
		if (range->ops && range->ops->init_range)
			range->ops->init_range(range);
		for (id = range->queue_base;
		     id < range->queue_base + range->num_queues; id++) {
			ret = khwq_init_queue(kdev, range,
					      hwqueue_id_to_inst(hdev, id));
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

static int khwq_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *qmgrs, *descs, *queues, *regions, *pdsps;
	struct device *dev = &pdev->dev;
	struct hwqueue_device *hdev;
	struct khwq_device *kdev;
	u32 temp[2];
	int ret;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	kdev = devm_kzalloc(dev, sizeof(struct khwq_device), GFP_KERNEL);
	if (!kdev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, kdev);
	kdev->dev = dev;
	INIT_LIST_HEAD(&kdev->queue_ranges);
	INIT_LIST_HEAD(&kdev->qmgrs);
	INIT_LIST_HEAD(&kdev->pools);
	INIT_LIST_HEAD(&kdev->regions);
	INIT_LIST_HEAD(&kdev->pdsps);

	if (of_property_read_u32_array(node, "range", temp, 2)) {
		dev_err(dev, "hardware queue range not specified\n");
		return -ENODEV;
	}
	kdev->base_id    = temp[0];
	kdev->num_queues = temp[1];

	kdev->clk = clk_get(dev, "clk_hwqueue");
	if (IS_ERR_OR_NULL(kdev->clk)) {
		dev_err(dev, "unable to get hardware queue clock\n");
		return -ENODEV;
	}
	if (clk_prepare_enable(kdev->clk)) {
		dev_err(dev, "unable to enable hardware queue clock\n");
		clk_put(kdev->clk);
		return -ENODEV;
	}

	qmgrs =  of_get_child_by_name(node, "qmgrs");
	if (!qmgrs) {
		dev_err(dev, "queue manager info not specified\n");
		return -ENODEV;
	}
	/* Initialize queue managers using device tree configuration */
	ret = khwq_init_qmgrs(kdev, qmgrs);
	if (ret)
		return ret;
	of_node_put(qmgrs);

	/*
	 * TODO: failure handling in this code is somewhere between moronic
	 * and non-existant - needs to be fixed
	 */

	pdsps =  of_get_child_by_name(node, "pdsps");
	/* get pdsp configuration values from device tree */
	if (pdsps) {
		ret = khwq_init_pdsps(kdev, pdsps);
		if (ret)
			return ret;

		ret = khwq_start_pdsps(kdev);
		if (ret)
			return ret;
	}
	of_node_put(pdsps);

	queues = of_get_child_by_name(node, "queues");
	if (!queues) {
		dev_err(dev, "queues not specified\n");
		return -ENODEV;
	}
	/* get usable queue range values from device tree */
	ret = khwq_setup_queue_ranges(kdev, queues);
	if (ret)
		return ret;
	of_node_put(queues);

	descs =  of_get_child_by_name(node, "descriptors");
	if (!descs) {
		dev_err(dev, "descriptor pools not specified\n");
		return -ENODEV;
	}
	/* Get descriptor pool values from device tree */
	ret = khwq_setup_pools(kdev, descs);
	if (ret) {
		khwq_free_queue_ranges(kdev);
		khwq_stop_pdsps(kdev);
		return ret;
	}
	of_node_put(descs);

	ret = khwq_get_link_ram(kdev, "linkram0", &kdev->link_rams[0]);
	if (ret) {
		dev_err(kdev->dev, "could not setup linking ram\n");
		return ret;
	}

	ret = khwq_get_link_ram(kdev, "linkram1", &kdev->link_rams[1]);
	if (ret) {
		/*
		 * nothing really, we have one linking ram already, so we just
		 * live within our means
		 */
	}

	ret = khwq_setup_link_ram(kdev);
	if (ret)
		return ret;

	regions =  of_get_child_by_name(node, "regions");
	if (!regions) {
		dev_err(dev, "region table not specified\n");
		return -ENODEV;
	}
	ret = khwq_setup_regions(kdev, regions);
	if (ret)
		return ret;
	of_node_put(regions);

	/* initialize hwqueue device data */
	hdev = to_hdev(kdev);
	hdev->dev	 = dev;
	hdev->base_id	 = kdev->base_id;
	hdev->num_queues = kdev->num_queues;
	hdev->priv_size	 = sizeof(struct khwq_instance);
	hdev->ops	 = &khdev_ops;

	/* register the hwqueue device */
	ret = hwqueue_device_register(hdev);
	if (ret < 0) {
		dev_err(dev, "hwqueue registration failed\n");
		return ret;
	}

	ret = khwq_init_queues(kdev);
	if (ret < 0) {
		dev_err(dev, "hwqueue initialization failed\n");
		return ret;
	}

	khwq_fill_pools(kdev);

	return 0;
}

static int khwq_remove(struct platform_device *pdev)
{
	struct khwq_device *kdev = platform_get_drvdata(pdev);
	struct hwqueue_device *hdev = to_hdev(kdev);
	int ret;

	if (kdev->clk) {
		clk_disable_unprepare(kdev->clk);
		clk_put(kdev->clk);
	}
	kdev->clk = NULL;

	ret = hwqueue_device_unregister(hdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "hwqueue unregistration failed\n");
		return ret;
	}

	return 0;
}

/* Match table for of_platform binding */
static struct of_device_id keystone_hwqueue_of_match[] = {
	{ .compatible = "ti,keystone-hwqueue", },
	{},
};
MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver keystone_hwqueue_driver = {
	.probe		= khwq_probe,
	.remove		= khwq_remove,
	.driver		= {
		.name	= "keystone-hwqueue",
		.owner	= THIS_MODULE,
		.of_match_table = keystone_hwqueue_of_match,
	},
};

static int __init keystone_hwqueue_init(void)
{
	return platform_driver_register(&keystone_hwqueue_driver);
}
device_initcall(keystone_hwqueue_init);

static void __exit keystone_hwqueue_exit(void)
{
	platform_driver_unregister(&keystone_hwqueue_driver);
}
module_exit(keystone_hwqueue_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware queue driver for Keystone devices");
