/*
 * Hardware queues handle header
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HWQUEUE_HWQUEUE_H
#define __HWQUEUE_HWQUEUE_H

#include <linux/device.h>
#include <linux/wait.h>
#include <linux/hwqueue.h>


struct hwqueue_instance {
	struct list_head	 handles;
	struct hwqueue_device	*hdev;
	spinlock_t		 lock;
	struct timer_list	 poll_timer;
	wait_queue_head_t	 wait;
	void			*priv;
	atomic_t		 num_notifiers;
	struct hwqueue_inst_ops	*ops;
	char			 name[32];
};

struct hwqueue_device_ops {
	/*
	 * Return a match quotient (0 = best .. UINT_MAX-1) for a set of
	 * option flags.  Negative error values imply "do not allocate"
	 */
	int	 (*match)(struct hwqueue_instance *inst, unsigned flags);

	/* Initialize a queue inst when opened for the first time */
	int	 (*open)(struct hwqueue_instance *inst, unsigned flags);

	/* Close a queue inst when closed by the last user */
	void	 (*close)(struct hwqueue_instance *inst);

	/* Enable or disable notification */
	void	 (*set_notify)(struct hwqueue_instance *inst, bool enabled);

	/* Get a hardware identifier for a queue */
	int	 (*get_hw_id)(struct hwqueue_instance *inst);
};

struct hwqueue_inst_ops {
	/* Push something into the queue */
	int	 (*push)(struct hwqueue_instance *inst, dma_addr_t dma,
			 unsigned size, unsigned flags);

	/* Pop something from the queue */
	dma_addr_t (*pop)(struct hwqueue_instance *inst, unsigned *size,
			  unsigned flags);

	/* Flush a queue */
	int	 (*flush)(struct hwqueue_instance *inst);

	/* Poll number of elements on the queue */
	int	 (*get_count)(struct hwqueue_instance *inst);
	
	/* Perform DMA mapping on objects to be pushed */
	int	 (*map)(struct hwqueue_instance *inst, void *data,
			unsigned size, dma_addr_t *dma_ptr, unsigned *size_ptr);
	
	/* Perform DMA unmapping on objects that have been pulled */
	void	*(*unmap)(struct hwqueue_instance *inst, dma_addr_t dma,
			unsigned desc_size);
};

struct hwqueue_device {
	unsigned			 base_id;
	unsigned			 num_queues;
	unsigned			 inst_shift;
	void				*instances;
	struct hwqueue_device_ops	*ops;

	unsigned			 priv_size;
	struct list_head		 list;
	struct device			*dev;
	struct knav_qmssm *qmon;
};

static inline int hwqueue_inst_to_id(struct hwqueue_instance *inst)
{
	struct hwqueue_device *hdev = inst->hdev;
	int offset = (void *)inst - hdev->instances;
	int inst_size = 1 << hdev->inst_shift;

	BUG_ON(offset & (inst_size - 1));
	return offset >> hdev->inst_shift;
}

static inline struct hwqueue_instance *
hwqueue_id_to_inst(struct hwqueue_device *hdev, unsigned id)
{
	return hdev->instances + (id <<  hdev->inst_shift);
}

static inline void *hwqueue_inst_to_priv(struct hwqueue_instance *inst)
{
	return (void *)(inst + 1);
}

static inline struct hwqueue *rcu_to_handle(struct rcu_head *rcu)
{
	return container_of(rcu, struct hwqueue, rcu);
}

int hwqueue_device_register(struct hwqueue_device *dev);
int hwqueue_device_unregister(struct hwqueue_device *dev);
void hwqueue_notify(struct hwqueue_instance *inst);
void hwqueue_set_poll(struct hwqueue_instance *inst, bool enabled);

#endif /* __HWQUEUE_HWQUEUE_H */
