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

#ifndef __KEYSTONE_HWQUEUE_H__
#define __KEYSTONE_HWQUEUE_H__

#define DESC_SIZE_MASK	0xful
#define DESC_PTR_MASK	(~DESC_SIZE_MASK)

#define BITS(x)		(BIT(x) - 1)

#define THRESH_GTE	BIT(7)
#define THRESH_LT	0

#define PDSP_CTRL_PC_MASK	0xffff0000
#define PDSP_CTRL_SOFT_RESET	BIT(0)
#define PDSP_CTRL_ENABLE	BIT(1)
#define PDSP_CTRL_RUNNING	BIT(15)

#define ACC_MAX_CHANNEL		48
#define ACC_DEFAULT_PERIOD		25 /* usecs */
#define ACC_DESCS_MAX			SZ_1K
#define ACC_DESCS_MASK		(ACC_DESCS_MAX - 1)
#define ACC_CHANNEL_INT_BASE		2

#define ACC_LIST_ENTRY_TYPE		1
#define ACC_LIST_ENTRY_WORDS		(1 << ACC_LIST_ENTRY_TYPE)
#define ACC_LIST_ENTRY_QUEUE_IDX	0
#define ACC_LIST_ENTRY_DESC_IDX	(ACC_LIST_ENTRY_WORDS - 1)

#define ACC_CMD_DISABLE_CHANNEL	0x80
#define ACC_CMD_ENABLE_CHANNEL	0x81
#define ACC_CMD_SET_RECLAMATION_QUEUE	0x83
#define ACC_CFG_MULTI_QUEUE		BIT(21)

#define ACC_INTD_OFFSET_EOI		(0x0010)
#define ACC_INTD_OFFSET_COUNT(ch)	(0x0300 + 4 * (ch))
#define ACC_INTD_OFFSET_STATUS(ch)	(0x0200 + 4 * ((ch) / 32))

#define RANGE_MAX_IRQS			64

enum khwq_acc_result {
	ACC_RET_IDLE,
	ACC_RET_SUCCESS,
	ACC_RET_INVALID_COMMAND,
	ACC_RET_INVALID_CHANNEL,
	ACC_RET_INACTIVE_CHANNEL,
	ACC_RET_ACTIVE_CHANNEL,
	ACC_RET_INVALID_QUEUE,

	ACC_RET_INVALID_RET,
};

struct khwq_reg_config {
	u32		revision;
	u32		__pad1;
	u32		divert;
	u32		link_ram_base0;
	u32		link_ram_size0;
	u32		link_ram_base1;
	u32		__pad2[2];
	u32		starvation[0];
};

struct khwq_reg_region {
	u32		base;
	u32		start_index;
	u32		size_count;
	u32		__pad;
};

struct khwq_reg_queue {
	u32		entry_count;
	u32		byte_count;
	u32		packet_size;
	u32		ptr_size_thresh;
};

struct khwq_reg_pdsp_regs {
	u32		control;
	u32		status;
	u32		cycle_count;
	u32		stall_count;
};

struct khwq_reg_acc_command {
	u32		command;
	u32		queue_mask;
	u32		list_phys;
	u32		queue_num;
	u32		timer_config;
};

struct khwq_region {
	const char	*name;
	unsigned	 used_desc;
	struct list_head list;
	unsigned	 id;
	unsigned	 desc_size;
	unsigned	 num_desc;
	unsigned	 fixed_mem;
	dma_addr_t	 next_pool_addr;
	dma_addr_t	 dma_start, dma_end;
	void		*virt_start, *virt_end;
	unsigned	 link_index;
};

struct khwq_pool_info {
	const char		*name;
	struct khwq_region	*region;
	int			 region_offset;
	int			 num_desc;
	int			 desc_size;
	int			 region_id;
	struct hwqueue		*queue;
	struct list_head	 list;
};

struct khwq_link_ram_block {
	dma_addr_t	 phys;
	void		*virt;
	size_t		 size;
};

struct khwq_acc_info {
	u32			 pdsp_id;
	u32			 start_channel;
	u32			 list_entries;
	u32			 pacing_mode;
	u32			 timer_count;
	int			 mem_size;
	int			 list_size;
	struct khwq_pdsp_info	*pdsp;
};

struct khwq_acc_channel {
	u32			 channel;
	u32			 list_index;
	u32			 open_mask;
	u32			*list_cpu[2];
	dma_addr_t		 list_dma[2];
	char			 name[32];
	atomic_t		 retrigger_count;
};

struct khwq_pdsp_info {
	const char				*name;
	struct khwq_reg_pdsp_regs  __iomem	*regs;
	union {
		void __iomem				*command;
		struct khwq_reg_acc_command __iomem	*acc_command;
		u32 __iomem				*qos_command;
	};
	void __iomem				*intd;
	u32 __iomem				*iram;
	const char				*firmware;
	u32					 id;
	struct list_head			 list;
	struct khwq_qos_info			*qos_info;
};

struct khwq_range_ops;

struct khwq_irq_info {
	int	irq;
	u32	cpu_map;
};

struct khwq_range_info {
	const char		*name;
	struct khwq_device	*kdev;
	unsigned		 queue_base;
	unsigned		 num_queues;
	unsigned		 flags;
	struct list_head	 list;
	struct khwq_range_ops	*ops;
	struct hwqueue_inst_ops	 inst_ops;
	struct khwq_acc_info	 acc_info;
	struct khwq_acc_channel	*acc;
	struct khwq_qos_info	*qos_info;
	unsigned		 num_irqs;
	struct khwq_irq_info	 irqs[RANGE_MAX_IRQS];
};

struct khwq_range_ops {
	int	(*init_range)(struct khwq_range_info *range);
	int	(*free_range)(struct khwq_range_info *range);
	int	(*init_queue)(struct khwq_range_info *range,
			      struct hwqueue_instance *inst);
	int	(*open_queue)(struct khwq_range_info *range,
			      struct hwqueue_instance *inst, unsigned flags);
	int	(*close_queue)(struct khwq_range_info *range,
			       struct hwqueue_instance *inst);
	int	(*set_notify)(struct khwq_range_info *range,
			      struct hwqueue_instance *inst, bool enabled);
};

#define RANGE_RESERVED		BIT(0)
#define RANGE_HAS_IRQ		BIT(1)
#define RANGE_HAS_ACCUMULATOR	BIT(2)
#define RANGE_MULTI_QUEUE	BIT(3)

struct khwq_qmgr_info {
	unsigned			 start_queue;
	unsigned			 num_queues;
	struct khwq_reg_config __iomem	*reg_config;
	struct khwq_reg_region __iomem	*reg_region;
	struct khwq_reg_queue __iomem	*reg_push, *reg_pop, *reg_peek;
	void __iomem			*reg_status;
	struct list_head		 list;
};

struct khwq_device {
	struct device			*dev;
	struct clk			*clk;
	struct hwqueue_device		 hdev;
	struct list_head		 regions;
	struct khwq_link_ram_block	 link_rams[2];
	unsigned			 num_queues;
	unsigned			 base_id;
	struct list_head		 queue_ranges;
	struct list_head		 pools;
	struct list_head		 pdsps;
	struct list_head		 qmgrs;
};

struct khwq_desc {
	u32			 val;
	unsigned		 size;
	struct list_head	 list;
};

struct khwq_instance {
	struct khwq_device	*kdev;
	struct khwq_range_info	*range;
	u32			*descs;
	atomic_t		 desc_head, desc_tail, desc_count;
	struct khwq_acc_channel	*acc;
	struct khwq_region	*last; /* cache last region used */
	struct khwq_qmgr_info	*qmgr; /* cache qmgr for the instance */
	int			 notify_needed;
	int			 irq_num; /*irq num -ve for non-irq queues */
	char			 irq_name[32];
};

#define to_hdev(_kdev)		(&(_kdev)->hdev)
#define from_hdev(_hdev)	container_of(_hdev, struct khwq_device, hdev)

#define for_each_region(kdev, region)				\
	list_for_each_entry(region, &kdev->regions, list)

#define for_each_queue_range(kdev, range)			\
	list_for_each_entry(range, &kdev->queue_ranges, list)

#define first_queue_range(kdev)					\
	list_first_entry(&kdev->queue_ranges, struct khwq_range_info, list)

#define for_each_pool(kdev, pool)				\
	list_for_each_entry(pool, &kdev->pools, list)

#define for_each_pdsp(kdev, pdsp)				\
	list_for_each_entry(pdsp, &kdev->pdsps, list)

#define for_each_qmgr(kdev, qmgr)				\
	list_for_each_entry(qmgr, &kdev->qmgrs, list)

#define for_each_policy(info, policy)				\
	list_for_each_entry(policy, &info->drop_policies, list)

static inline struct khwq_pdsp_info *
khwq_find_pdsp(struct khwq_device *kdev, unsigned pdsp_id)
{
	struct khwq_pdsp_info *pdsp;

	for_each_pdsp(kdev, pdsp)
		if (pdsp_id == pdsp->id)
			return pdsp;
	return NULL;
}

static inline struct khwq_qmgr_info *
khwq_find_qmgr(struct hwqueue_instance *inst)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;

	for_each_qmgr(kdev, qmgr) {
		if ((id >= qmgr->start_queue) &&
			(id < qmgr->start_queue + qmgr->num_queues))
			return qmgr;
	}

	return NULL;
}

int khwq_init_acc_range(struct khwq_device *kdev, struct device_node *node,
			struct khwq_range_info *range);
int khwq_set_acc_reclamation(struct khwq_device *kdev, struct device_node *node,
			     struct khwq_range_info *range);

int khwq_init_qos_range(struct khwq_device *kdev, struct device_node *node,
			struct khwq_range_info *range);

#endif /* __KEYSTONE_HWQUEUE_H__ */
