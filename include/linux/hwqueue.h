/*
 * Hardware queue framework
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
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

#ifndef __LINUX_HWQUEUE_H
#define __LINUX_HWQUEUE_H

#include <linux/err.h>
#include <linux/time.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/percpu.h>

#define HWQUEUE_ANY			((unsigned)-1)
#define HWQUEUE_BYNAME			((unsigned)-2)

/* file fcntl flags repurposed for hwqueues */
#define O_HIGHTHROUGHPUT	O_DIRECT
#define O_LOWLATENCY		O_LARGEFILE

/* hardware queue notifier callback prototype */
typedef void (*hwqueue_notify_fn)(void *arg);

/* hardware queue monitor logger callback prototype */
typedef void (*hwqueue_monitor_fn)(void *arg);

struct hwqueue_stats {
	int	 pushes;
	int	 pops;
	int	 push_errors;
	int	 pop_errors;
	int	 notifies;
};

struct hwqueue_instance;

struct hwqueue_monitor_config {
	hwqueue_monitor_fn fn;
	void *fdq_arg[4];
};

struct hwqueue {
	int		 (*push)(struct hwqueue_instance *inst, dma_addr_t dma,
				 unsigned size, unsigned flags);
	dma_addr_t	 (*pop)(struct hwqueue_instance *inst, unsigned *size,
				unsigned flags);
	int		 (*flush)(struct hwqueue_instance *inst);
	int		 (*get_count)(struct hwqueue_instance *inst);
	int		 (*map)(struct hwqueue_instance *inst, void *data,
				unsigned size, dma_addr_t *dma_ptr, unsigned *size_ptr);
	void		*(*unmap)(struct hwqueue_instance *inst, dma_addr_t dma,
				unsigned desc_size);

	struct hwqueue_instance	*inst;
	struct hwqueue_stats __percpu *stats;
	hwqueue_notify_fn	 notifier_fn;
	void			*notifier_fn_arg;
	atomic_t		 notifier_enabled;
	struct hwqueue_monitor_config *monitor_cfg;
	atomic_t		 monitor_enabled;
	struct rcu_head		 rcu;
	unsigned		 flags;
	struct list_head	 list;
};

struct hwqueue *hwqueue_open(const char *name, unsigned id, unsigned flags);

void hwqueue_close(struct hwqueue *queue);

struct hwqueue *devm_hwqueue_open(struct device *dev, const char *name,
				  unsigned id, unsigned flags);
void devm_hwqueue_close(struct device *dev, struct hwqueue *queue);

int hwqueue_get_id(struct hwqueue *queue);

int hwqueue_get_hw_id(struct hwqueue *queue);

int hwqueue_set_notifier(struct hwqueue *queue, hwqueue_notify_fn fn,
			 void *fn_arg);
int hwqueue_enable_notifier(struct hwqueue *queue);

int hwqueue_disable_notifier(struct hwqueue *queue);

dma_addr_t __hwqueue_pop_slow(struct hwqueue *qh, unsigned *size,
			      struct timeval *timeout, unsigned flags);

int hwqueue_set_monitor(struct hwqueue *qh, struct hwqueue_monitor_config *cfg);

int hwqueue_enable_monitoring(struct hwqueue *queue);

int hwqueue_disable_monitoring(struct hwqueue *queue);

void knav_qmssm_event_callback(void *arg);

/**
 * hwqueue_get_count() - poll a hardware queue and check if empty
 *			 and return number of elements in a queue
 * @qh	- hardware queue handle
 *
 * Returns 'true' if empty, and 'false' otherwise
 */
static inline int hwqueue_get_count(struct hwqueue *qh)
{
	if (unlikely(!qh->get_count))
		return -EINVAL;
	return qh->get_count(qh->inst);
}

/**
 * hwqueue_flush() - forcibly empty a queue if possible
 * @qh	- hardware queue handle
 *
 * Returns 0 on success, errno otherwise.  This may not be universally
 * supported on all hardware queue implementations.
 */
static inline int hwqueue_flush(struct hwqueue *qh)
{
	if (unlikely(!qh->flush))
		return -EINVAL;
	return qh->flush(qh->inst);
}

/**
 * hwqueue_push() - push data (or descriptor) to the tail of a queue
 * @qh	- hardware queue handle
 * @data	- data to push
 * @size	- size of data to push
 * @flags	- can be used to pass additional information
 *
 * Returns 0 on success, errno otherwise.
 */
static inline int hwqueue_push(struct hwqueue *qh, dma_addr_t dma,
			       unsigned size, unsigned flags)
{
	int ret = 0;

	do {
		if (unlikely(!qh->push)) {
			ret = -EINVAL;
			break;
		}

		ret = qh->push(qh->inst, dma, size, flags);
	} while (0);

	if (unlikely(ret < 0))
		this_cpu_inc(qh->stats->push_errors);
	else
		this_cpu_inc(qh->stats->pushes);
	return ret;
}

/**
 * hwqueue_pop() - pop data (or descriptor) from the head of a queue
 * @qh	- hardware queue handle
 * @size	- hwqueue_pop fills this parameter on success
 * @timeout	- timeout value to use if blocking
 *
 * Returns a DMA address on success, 0 on failure.
 */
static inline dma_addr_t hwqueue_pop(struct hwqueue *qh, unsigned *size,
				struct timeval *timeout , unsigned flags)
{
	dma_addr_t ret = 0;

	do {
		if (unlikely(!qh->pop)) {
			ret = -EINVAL;
			break;
		}

		ret = qh->pop(qh->inst, size, flags);
		if (likely(ret))
			break;

		if (likely((qh->flags & O_NONBLOCK) ||
			   (timeout && !timeout->tv_sec && !timeout->tv_usec)))
			break;

		ret = __hwqueue_pop_slow(qh, size, timeout, flags);
	} while (0);

	if (likely(ret)) {
		if (IS_ERR_VALUE(ret))
			this_cpu_inc(qh->stats->pop_errors);
		else
			this_cpu_inc(qh->stats->pops);
	}
	return ret;
}

/**
 * hwqueue_map() - Map data (or descriptor) for DMA transfer
 * @qh	- hardware queue handle
 * @data	- address of data to map
 * @size	- size of data to map
 * @dma_ptr	- DMA address return pointer
 * @size_ptr	- adjusted size return pointer
 *
 * Returns 0 on success, errno otherwise.
 */
static inline int hwqueue_map(struct hwqueue *qh, void *data, unsigned size,
				dma_addr_t *dma_ptr, unsigned *size_ptr)
{
	if (unlikely(!qh->map))
		return -EINVAL;
	return qh->map(qh->inst, data, size, dma_ptr, size_ptr);
}

/**
 * hwqueue_unmap() - Unmap data (or descriptor) after DMA transfer
 * @qh	- hardware queue handle
 * @dma	- DMA address to be unmapped
 * @size	- size of data to unmap
 *
 * Returns a data/descriptor pointer on success.  Use IS_ERR_OR_NULL() to identify
 * error values on return.
 */
static inline void *hwqueue_unmap(struct hwqueue *qh, dma_addr_t dma,
				unsigned size)
{
	if (unlikely(!qh->unmap))
		return NULL;
	return qh->unmap(qh->inst, dma, size);
}

#endif /* __LINUX_HWQUEUE_H */
