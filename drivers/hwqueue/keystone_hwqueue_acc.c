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
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/hwqueue.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include "hwqueue_internal.h"
#include "keystone_hwqueue.h"

static void __khwq_acc_notify(struct khwq_range_info *range,
			      struct khwq_acc_channel *acc)
{
	struct khwq_device *kdev = range->kdev;
	struct hwqueue_device *hdev = to_hdev(kdev);
	struct hwqueue_instance *inst;
	struct khwq_instance *kq;
	int range_base, queue;

	range_base = kdev->base_id + range->queue_base;

	if (range->flags & RANGE_MULTI_QUEUE) {
		for (queue = 0; queue < range->num_queues; queue++) {
			inst = hwqueue_id_to_inst(hdev, range_base + queue);
			kq = hwqueue_inst_to_priv(inst);
			if (kq->notify_needed) {
				kq->notify_needed = 0;
				dev_dbg(kdev->dev, "acc-irq: notifying %d\n",
					range_base + queue);
				hwqueue_notify(inst);
			}
		}
	} else {
		queue = acc->channel - range->acc_info.start_channel;
		inst = hwqueue_id_to_inst(hdev, range_base + queue);
		dev_dbg(kdev->dev, "acc-irq: notifying %d\n",
			range_base + queue);
		hwqueue_notify(inst);
	}
}

static int khwq_acc_set_notify(struct khwq_range_info *range,
			       struct hwqueue_instance *inst,
			       bool enabled)
{
	struct khwq_pdsp_info *pdsp = range->acc_info.pdsp;
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_device *kdev = range->kdev;
	u32 mask, offset;

	/*
	 * when enabling, we need to re-trigger an interrupt if we
	 * have descriptors pending
	 */
	if (!enabled || atomic_read(&kq->desc_count) <= 0)
		return 0;

	kq->notify_needed = 1;
	atomic_inc(&kq->acc->retrigger_count);
	mask = BIT(kq->acc->channel % 32);
	offset = ACC_INTD_OFFSET_STATUS(kq->acc->channel);

	dev_dbg(kdev->dev, "setup-notify: re-triggering irq for %s\n",
		kq->acc->name);
	__raw_writel(mask, pdsp->intd + offset);

	return 0;
}

static irqreturn_t khwq_acc_int_handler(int irq, void *_instdata)
{
	struct hwqueue_instance *inst = NULL;
	struct khwq_acc_channel *acc;
	struct khwq_instance *kq = NULL;
	struct khwq_range_info *range;
	struct hwqueue_device *hdev;
	struct khwq_pdsp_info *pdsp;
	struct khwq_acc_info *info;
	struct khwq_device *kdev;

	u32 *list, *list_cpu, val, idx, notifies;
	int range_base, channel, queue = 0;
	dma_addr_t list_dma;

	range = _instdata;
	info  = &range->acc_info;
	kdev  = range->kdev;
	hdev  = to_hdev(kdev);
	pdsp  = range->acc_info.pdsp;
	acc   = range->acc;

	range_base = kdev->base_id + range->queue_base;

	if ((range->flags & RANGE_MULTI_QUEUE) == 0) {
		/* TODO: this needs extent checks */
		for (queue = 0; queue < range->num_irqs; queue++)
			if (range->irqs[queue].irq == irq)
				break;
		inst	 = hwqueue_id_to_inst(hdev, range_base + queue);
		kq	 = hwqueue_inst_to_priv(inst);
		acc	+= queue;
	}

	channel = acc->channel;
	list_dma = acc->list_dma[acc->list_index];
	list_cpu = acc->list_cpu[acc->list_index];

	dev_dbg(kdev->dev, "acc-irq: channel %d, list %d, virt %p, phys %08llx\n",
		channel, acc->list_index, list_cpu, list_dma);

	if (atomic_read(&acc->retrigger_count)) {
		atomic_dec(&acc->retrigger_count);
		__khwq_acc_notify(range, acc);

		__raw_writel(1, pdsp->intd + ACC_INTD_OFFSET_COUNT(channel));

		/* ack the interrupt */
		__raw_writel(ACC_CHANNEL_INT_BASE + channel,
			     pdsp->intd + ACC_INTD_OFFSET_EOI);

		return IRQ_HANDLED;
	}

	notifies = __raw_readl(pdsp->intd + ACC_INTD_OFFSET_COUNT(channel));
	WARN_ON(!notifies);

	dma_sync_single_for_cpu(kdev->dev, list_dma, info->list_size, DMA_FROM_DEVICE);

	for (list = list_cpu; list < list_cpu + (info->list_size / sizeof(u32));
	     list += ACC_LIST_ENTRY_WORDS) {

		if (ACC_LIST_ENTRY_WORDS == 1) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x\n",
				acc->list_index, list, list[0]);
		} else if (ACC_LIST_ENTRY_WORDS == 2) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x %08x\n",
				acc->list_index, list, list[0], list[1]);
		} else if (ACC_LIST_ENTRY_WORDS == 4) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x %08x %08x %08x\n",
				acc->list_index, list,
				list[0], list[1], list[2], list[3]);
		}

		val = list[ACC_LIST_ENTRY_DESC_IDX];

		if (!val)
			break;

		if (range->flags & RANGE_MULTI_QUEUE) {
			queue = list[ACC_LIST_ENTRY_QUEUE_IDX] >> 16;
			if (queue < range_base || queue >= range_base + range->num_queues) {
				dev_err(kdev->dev, "bad queue %d, expecting %d-%d\n",
					queue, range_base, range_base + range->num_queues);
				break;
			}
			queue -= range_base;
			inst = hwqueue_id_to_inst(hdev, range_base + queue);
			kq = hwqueue_inst_to_priv(inst);
		}

		if (atomic_inc_return(&kq->desc_count) >= ACC_DESCS_MAX) {
			atomic_dec(&kq->desc_count);
			dev_err(kdev->dev, "acc-irq: queue %d full"
				", entry dropped\n", queue + range_base);
			/* TODO: need a statistics counter for such drops */
			continue;
		}

		idx = atomic_inc_return(&kq->desc_tail) & ACC_DESCS_MASK;
		kq->descs[idx] = val;
		kq->notify_needed = 1;
		dev_dbg(kdev->dev, "acc-irq: enqueue %08x at %d, queue %d\n",
			val, idx, queue + range_base);
	}

	__khwq_acc_notify(range, acc);

	memset(list_cpu, 0, info->list_size);
	dma_sync_single_for_device(kdev->dev, list_dma, info->list_size,
				   DMA_TO_DEVICE);

	/* flip to the other list */
	acc->list_index ^= 1;

	/* reset the interrupt counter */
	__raw_writel(1, pdsp->intd + ACC_INTD_OFFSET_COUNT(channel));

	/* ack the interrupt */
	__raw_writel(ACC_CHANNEL_INT_BASE + channel,
		     pdsp->intd + ACC_INTD_OFFSET_EOI);

	return IRQ_HANDLED;
}

int khwq_range_setup_acc_irq(struct khwq_range_info *range, int queue,
			     bool enabled)
{
	struct khwq_device *kdev = range->kdev;
	struct khwq_acc_channel *acc;
	unsigned long cpu_map;
	int ret = 0, irq;
	u32 old, new;

	if (range->flags & RANGE_MULTI_QUEUE) {
		acc = range->acc;
		irq = range->irqs[0].irq;
		cpu_map = range->irqs[0].cpu_map;
	} else {
		acc = range->acc + queue;
		irq = range->irqs[queue].irq;
		cpu_map = range->irqs[queue].cpu_map;
	}

	old = acc->open_mask;
	if (enabled)
		new = old | BIT(queue);
	else
		new = old & ~BIT(queue);
	acc->open_mask = new;

	dev_dbg(kdev->dev, "setup-acc-irq: open mask old %08x, new %08x, channel %s\n",
		old, new, acc->name);

	if (likely(new == old))
		return 0;

	if (new && !old) {
		dev_dbg(kdev->dev, "setup-acc-irq: requesting %s for channel %s\n",
			acc->name, acc->name);
		ret = request_irq(irq, khwq_acc_int_handler, 0, acc->name,
				  range);
		if (cpu_map)
			if (irq_set_affinity(irq, to_cpumask(&cpu_map)))
				dev_warn(range->kdev->dev,
						"Failed to set IRQ affinity\n");
	}

	if (old && !new) {
		dev_dbg(kdev->dev, "setup-acc-irq: freeing %s for channel %s\n",
			acc->name, acc->name);
		free_irq(irq, range);
	}

	return ret;
}

static const char *khwq_acc_result_str(enum khwq_acc_result result)
{
	static const char *result_str[] = {
		[ACC_RET_IDLE]			= "idle",
		[ACC_RET_SUCCESS]		= "success",
		[ACC_RET_INVALID_COMMAND]	= "invalid command",
		[ACC_RET_INVALID_CHANNEL]	= "invalid channel",
		[ACC_RET_INACTIVE_CHANNEL]	= "inactive channel",
		[ACC_RET_ACTIVE_CHANNEL]	= "active channel",
		[ACC_RET_INVALID_QUEUE]		= "invalid queue",

		[ACC_RET_INVALID_RET]		= "invalid return code",
	};

	if (result >= ARRAY_SIZE(result_str))
		return result_str[ACC_RET_INVALID_RET];
	else
		return result_str[result];
}

static enum khwq_acc_result
khwq_acc_write(struct khwq_device *kdev, struct khwq_pdsp_info *pdsp,
	       struct khwq_reg_acc_command *cmd)
{
	u32 result;

	/* TODO: acquire hwspinlock here */

	dev_dbg(kdev->dev, "acc command %08x %08x %08x %08x %08x\n",
		cmd->command, cmd->queue_mask, cmd->list_phys,
		cmd->queue_num, cmd->timer_config);

	__raw_writel(cmd->timer_config,	&pdsp->acc_command->timer_config);
	__raw_writel(cmd->queue_num,	&pdsp->acc_command->queue_num);
	__raw_writel(cmd->list_phys,	&pdsp->acc_command->list_phys);
	__raw_writel(cmd->queue_mask,	&pdsp->acc_command->queue_mask);
	__raw_writel(cmd->command,	&pdsp->acc_command->command);

	/* wait for the command to clear */
	do {
		result = __raw_readl(&pdsp->acc_command->command);
	} while ((result >> 8) & 0xff);

	/* TODO: release hwspinlock here */

	return (result >> 24) & 0xff;
}

static void khwq_acc_setup_cmd(struct khwq_device *kdev,
			       struct khwq_range_info *range,
			       struct khwq_reg_acc_command *cmd,
			       int queue)
{
	struct khwq_acc_info *info = &range->acc_info;
	struct khwq_acc_channel *acc;
	int queue_base;
	u32 queue_mask;

	if (range->flags & RANGE_MULTI_QUEUE) {
		acc = range->acc;
		queue_base = range->queue_base;
		queue_mask = BIT(range->num_queues) - 1;
	} else {
		acc = range->acc + queue;
		queue_base = range->queue_base + queue;
		queue_mask = 0;
	}

	memset(cmd, 0, sizeof(*cmd));
	cmd->command    = acc->channel;
	cmd->queue_mask = queue_mask;
	cmd->list_phys  = acc->list_dma[0];
	cmd->queue_num  = info->list_entries << 16;
	cmd->queue_num |= queue_base;

	cmd->timer_config = ACC_LIST_ENTRY_TYPE << 18;
	if (range->flags & RANGE_MULTI_QUEUE)
		cmd->timer_config |= ACC_CFG_MULTI_QUEUE;
	cmd->timer_config |= info->pacing_mode << 16;
	cmd->timer_config |= info->timer_count;
}

static void khwq_acc_stop(struct khwq_device *kdev,
			  struct khwq_range_info *range,
			  int queue)
{
	struct khwq_reg_acc_command cmd;
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;

	acc = range->acc + queue;

	khwq_acc_setup_cmd(kdev, range, &cmd, queue);
	cmd.command |= ACC_CMD_DISABLE_CHANNEL << 8;
	result = khwq_acc_write(kdev, range->acc_info.pdsp, &cmd);

	dev_dbg(kdev->dev, "stopped acc channel %s, result %s\n",
		acc->name, khwq_acc_result_str(result));
}

static enum khwq_acc_result khwq_acc_start(struct khwq_device *kdev,
					   struct khwq_range_info *range,
					   int queue)
{
	struct khwq_reg_acc_command cmd;
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;

	acc = range->acc + queue;

	khwq_acc_setup_cmd(kdev, range, &cmd, queue);
	cmd.command |= ACC_CMD_ENABLE_CHANNEL << 8;
	result = khwq_acc_write(kdev, range->acc_info.pdsp, &cmd);

	dev_dbg(kdev->dev, "started acc channel %s, result %s\n",
		acc->name, khwq_acc_result_str(result));

	return result;
}

static int khwq_acc_init_range(struct khwq_range_info *range)
{
	struct khwq_device *kdev = range->kdev;
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;
	int queue;

	for (queue = 0; queue < range->num_queues; queue++) {
		acc = range->acc + queue;

		khwq_acc_stop(kdev, range, queue);
		acc->list_index = 0;
		result = khwq_acc_start(kdev, range, queue);

		if (result != ACC_RET_SUCCESS)
			return -EIO;

		if (range->flags & RANGE_MULTI_QUEUE)
			return 0;
	}
	return 0;
}

static int khwq_acc_init_queue(struct khwq_range_info *range,
			       struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;

	kq->descs = kzalloc(ACC_DESCS_MAX * sizeof(u32), GFP_KERNEL);
	if (!kq->descs)
		return -ENOMEM;

	kq->acc = range->acc;
	if ((range->flags & RANGE_MULTI_QUEUE) == 0)
		kq->acc += id;
	return 0;
}

static int khwq_acc_open_queue(struct khwq_range_info *range,
			       struct hwqueue_instance *inst, unsigned flags)
{
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;

	return khwq_range_setup_acc_irq(range, id, true);
}

static int khwq_acc_close_queue(struct khwq_range_info *range,
				struct hwqueue_instance *inst)
{
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;

	return khwq_range_setup_acc_irq(range, id, false);
}

static int khwq_acc_free_range(struct khwq_range_info *range)
{
	struct khwq_device *kdev = range->kdev;
	struct khwq_acc_channel *acc;
	struct khwq_acc_info *info;
	int channel, channels;

	info = &range->acc_info;

	if (range->flags & RANGE_MULTI_QUEUE)
		channels = 1;
	else
		channels = range->num_queues;

	for (channel = 0; channel < channels; channel++) {
		acc = range->acc + channel;
		if (!acc->list_cpu[0])
			continue;
		dma_unmap_single(kdev->dev, acc->list_dma[0],
				 info->mem_size, DMA_BIDIRECTIONAL);
		free_pages_exact(acc->list_cpu[0], info->mem_size);
	}
	kfree(range->acc);
	return 0;
}

struct khwq_range_ops khwq_acc_range_ops = {
	.set_notify	= khwq_acc_set_notify,
	.init_queue	= khwq_acc_init_queue,
	.open_queue	= khwq_acc_open_queue,
	.close_queue	= khwq_acc_close_queue,
	.init_range	= khwq_acc_init_range,
	.free_range	= khwq_acc_free_range,
};

int khwq_init_acc_range(struct khwq_device *kdev, struct device_node *node,
			struct khwq_range_info *range)
{
	struct khwq_acc_channel *acc;
	struct khwq_pdsp_info *pdsp;
	struct khwq_acc_info *info;
	int ret, channel, channels;
	int list_size, mem_size;
	dma_addr_t list_dma;
	void *list_mem;
	u32 config[5];

	range->flags |= RANGE_HAS_ACCUMULATOR;
	info = &range->acc_info;

	ret = of_property_read_u32_array(node, "accumulator", config, 5);
	if (ret)
		return ret;

	info->pdsp_id		= config[0];
	info->start_channel	= config[1];
	info->list_entries	= config[2];
	info->pacing_mode	= config[3];
	info->timer_count	= config[4] / ACC_DEFAULT_PERIOD;

	if (info->start_channel > ACC_MAX_CHANNEL) {
		dev_err(kdev->dev, "channel %d invalid for range %s\n",
			info->start_channel, range->name);
		return -EINVAL;
	}

	if (info->pacing_mode > 3) {
		dev_err(kdev->dev, "pacing mode %d invalid for range %s\n",
			info->pacing_mode, range->name);
		return -EINVAL;
	}

	pdsp = khwq_find_pdsp(kdev, info->pdsp_id);
	if (!pdsp) {
		dev_err(kdev->dev, "pdsp id %d not found for range %s\n",
			info->pdsp_id, range->name);
		return -EINVAL;
	}
	info->pdsp = pdsp;

	channels = range->num_queues;

	if (of_get_property(node, "multi-queue", NULL)) {
		range->flags |= RANGE_MULTI_QUEUE;
		channels = 1;
		if (range->queue_base & (32 - 1)) {
			dev_err(kdev->dev,
				"misaligned multi-queue accumulator range %s\n",
				range->name);
			return -EINVAL;
		}
		if (range->num_queues > 32) {
			dev_err(kdev->dev,
				"too many queues in accumulator range %s\n",
				range->name);
			return -EINVAL;
		}
	}

	/* figure out list size */
	list_size  = info->list_entries;
	list_size *= ACC_LIST_ENTRY_WORDS * sizeof(u32);
	info->list_size = list_size;

	mem_size   = PAGE_ALIGN(list_size * 2);
	info->mem_size  = mem_size;

	range->acc = kzalloc(channels * sizeof(*range->acc), GFP_KERNEL);
	if (!range->acc)
		return -ENOMEM;

	for (channel = 0; channel < channels; channel++) {
		acc = range->acc + channel;
		acc->channel = info->start_channel + channel;

		/* allocate memory for the two lists */
		list_mem = alloc_pages_exact(mem_size, GFP_KERNEL | GFP_DMA);
		if (!list_mem)
			return -ENOMEM;

		list_dma = dma_map_single(kdev->dev, list_mem, mem_size,
					  DMA_BIDIRECTIONAL);
		if (dma_mapping_error(kdev->dev, list_dma)) {
			free_pages_exact(list_mem, mem_size);
			return -ENOMEM;
		}

		memset(list_mem, 0, mem_size);
		dma_sync_single_for_device(kdev->dev, list_dma, mem_size, DMA_TO_DEVICE);

		scnprintf(acc->name, sizeof(acc->name), "hwqueue-acc-%d", acc->channel);

		acc->list_cpu[0] = list_mem;
		acc->list_cpu[1] = list_mem + list_size;
		acc->list_dma[0] = list_dma;
		acc->list_dma[1] = list_dma + list_size;

		dev_dbg(kdev->dev, "%s: channel %d, phys %08llx, virt %8p\n",
			acc->name, acc->channel, list_dma, list_mem);
	}

	range->ops = &khwq_acc_range_ops;

	return 0;
}

int khwq_set_acc_reclamation(struct khwq_device *kdev, struct device_node *node,
			     struct khwq_range_info *range)
{
	int ret;
	u32 pdsp_id;
	struct khwq_pdsp_info *pdsp;
	unsigned qnum;
	struct khwq_reg_acc_command cmd;
	enum khwq_acc_result result;

	if (range->num_queues != 1) {
		dev_err(kdev->dev,
			"too many queues in accumulator reclamation range %s\n",
			range->name);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "pdsp-acc-reclamation", &pdsp_id);
	if (ret)
		return ret;

	pdsp = khwq_find_pdsp(kdev, pdsp_id);
	if (!pdsp) {
		dev_err(kdev->dev, "pdsp id %d not found for range %s\n",
			pdsp_id, range->name);
		return -EINVAL;
	}

	qnum = range->queue_base + kdev->base_id;

	memset(&cmd, 0, sizeof(cmd));
	cmd.command = ACC_CMD_SET_RECLAMATION_QUEUE << 8;
	cmd.queue_mask = qnum;
	result = khwq_acc_write(kdev, pdsp, &cmd);
	if (result != ACC_RET_SUCCESS) {
		dev_err(kdev->dev,
			"failed to set accumulator reclamation queue"
			" from range %s, result %s\n",
			range->name, khwq_acc_result_str(result));
		return -EIO;
	}

	dev_dbg(kdev->dev, "started reclamation queue %d, result %s\n",
		qnum, khwq_acc_result_str(result));

	return 0;
}
