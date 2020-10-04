/*
 * Hardware queue framework
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

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/percpu.h>

#include "hwqueue_internal.h"
#include "knav_qmss_monitor.h"

static LIST_HEAD(hwqueue_devices);
static DEFINE_MUTEX(hwqueue_devices_lock);

#define for_each_handle_rcu(qh, inst)			\
	list_for_each_entry_rcu(qh, &inst->handles, list)

#define for_each_device(hdev)				\
	list_for_each_entry(hdev, &hwqueue_devices, list)

#define for_each_instance(id, inst, hdev)		\
	for (id = 0, inst = hdev->instances;		\
	     id < (hdev)->num_queues;			\
	     id++, inst = hwqueue_id_to_inst(hdev, id))

static void __hwqueue_poll(struct timer_list *t);

static int hwqueue_poll_interval = 100;

static inline bool hwqueue_is_busy(struct hwqueue_instance *inst)
{
	return !list_empty(&inst->handles);
}

static inline bool hwqueue_is_exclusive(struct hwqueue_instance *inst)
{
	struct hwqueue *tmp;

	rcu_read_lock();

	for_each_handle_rcu(tmp, inst) {
		if (tmp->flags & O_EXCL) {
			rcu_read_unlock();
			return true;
		}
	}

	rcu_read_unlock();

	return false;
}

static inline bool hwqueue_is_writable(struct hwqueue *qh)
{
	unsigned acc = qh->flags & O_ACCMODE;
	return (acc == O_RDWR || acc == O_WRONLY);
}

static inline bool hwqueue_is_readable(struct hwqueue *qh)
{
	unsigned acc = qh->flags & O_ACCMODE;
	return (acc == O_RDWR || acc == O_RDONLY);
}

static inline struct hwqueue_instance *hwqueue_find_by_id(int id)
{
	struct hwqueue_device *hdev;

	for_each_device(hdev) {
		if (hdev->base_id <= id &&
		    hdev->base_id + hdev->num_queues > id) {
			id -= hdev->base_id;
			return hwqueue_id_to_inst(hdev, id);
		}
	}
	return NULL;
}

int hwqueue_device_register(struct hwqueue_device *hdev)
{
	struct hwqueue_instance *inst;
	int id, size, ret = -EEXIST;
	struct hwqueue_device *b;

	if (!hdev->ops || !hdev->dev)
		return -EINVAL;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(b) {
		if (b->base_id + b->num_queues >= hdev->base_id &&
		    hdev->base_id + hdev->num_queues >= b->base_id) {
			dev_err(hdev->dev, "id collision with %s\n",
				dev_name(b->dev));
			goto unlock_ret;
		}
	}
	ret = -ENOMEM;

	/* how much do we need for instance data? */
	size  = sizeof(struct hwqueue_instance) + hdev->priv_size;

	/* round this up to a power of 2, keep the push/pop arithmetic fast */
	hdev->inst_shift = order_base_2(size);
	size = (1 << hdev->inst_shift) * hdev->num_queues;

	hdev->instances = kzalloc(size, GFP_KERNEL);
	if (!hdev->instances)
		goto unlock_ret;

	ret = 0;
	for_each_instance(id, inst, hdev) {
		inst->hdev = hdev;
		INIT_LIST_HEAD(&inst->handles);
		timer_setup(&inst->poll_timer, __hwqueue_poll, 0);
		init_waitqueue_head(&inst->wait);
		spin_lock_init(&inst->lock);
	}

	list_add(&hdev->list, &hwqueue_devices);

	dev_info(hdev->dev, "registered queues %d-%d\n",
		 hdev->base_id, hdev->base_id + hdev->num_queues - 1);

	ret = knav_qmssm_register(hdev);
	if (ret)
		dev_info(hdev->dev, "cannot register hwqueue monitor\n");

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);
	return ret;
}
EXPORT_SYMBOL(hwqueue_device_register);

int hwqueue_device_unregister(struct hwqueue_device *hdev)
{
	struct hwqueue_instance *inst;
	int id, ret = -EBUSY;

	mutex_lock(&hwqueue_devices_lock);

	for_each_instance(id, inst, hdev) {
		if (hwqueue_is_busy(inst)) {
			dev_err(hdev->dev, "cannot unregister busy dev\n");
			goto unlock_ret;
		}
	}

	knav_qmssm_unregister(hdev);

	list_del(&hdev->list);
	dev_info(hdev->dev, "unregistered queues %d-%d\n",
		 hdev->base_id, hdev->base_id + hdev->num_queues);
	kfree(hdev->instances);
	ret = 0;

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);

	return ret;
}
EXPORT_SYMBOL(hwqueue_device_unregister);

static struct hwqueue *__hwqueue_open(struct hwqueue_instance *inst,
				      const char *name, unsigned flags,
				      void *caller)
{
	struct hwqueue_device *hdev = inst->hdev;
	struct hwqueue *qh;
	int ret;

	if (!try_module_get(hdev->dev->driver->owner))
		return ERR_PTR(-ENODEV);

	qh = kzalloc(sizeof(struct hwqueue), GFP_KERNEL);
	if (!qh) {
		module_put(hdev->dev->driver->owner);
		return ERR_PTR(-ENOMEM);
	}

	qh->flags = flags;
	qh->inst = inst;

	/* first opener? */
	if (!hwqueue_is_busy(inst)) {
		strncpy(inst->name, name, sizeof(inst->name));
		ret = hdev->ops->open(inst, flags);
		if (ret) {
			kfree(qh);
			module_put(hdev->dev->driver->owner);
			return ERR_PTR(ret);
		}
	}

	qh->stats = alloc_percpu(struct hwqueue_stats);
	if (hwqueue_is_readable(qh)) {
		qh->get_count	= inst->ops->get_count;
		qh->pop		= inst->ops->pop;
		qh->unmap	= inst->ops->unmap;
	}

	if (hwqueue_is_writable(qh)) {
		qh->flush	= inst->ops->flush;
		qh->push	= inst->ops->push;
		qh->map		= inst->ops->map;
	}

	list_add_tail_rcu(&qh->list, &inst->handles);

	return qh;
}

static struct hwqueue *hwqueue_open_by_id(const char *name, unsigned id,
					  unsigned flags, void *caller)
{
	struct hwqueue_instance *inst;
	struct hwqueue_device *hdev;
	struct hwqueue *qh;
	int match;

	mutex_lock(&hwqueue_devices_lock);

	qh = ERR_PTR(-ENODEV);
	inst = hwqueue_find_by_id(id);
	if (!inst)
		goto unlock_ret;
	hdev = inst->hdev;


	qh = ERR_PTR(-EINVAL);
	match = hdev->ops->match(inst, flags);

	if (match < 0)
		goto unlock_ret;

	qh = ERR_PTR(-EBUSY);
	if (hwqueue_is_exclusive(inst))
		goto unlock_ret;

	qh = ERR_PTR(-EEXIST);
	if ((flags & O_CREAT) && hwqueue_is_busy(inst))
		goto unlock_ret;

	qh = __hwqueue_open(inst, name, flags, caller);

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);

	return qh;
}

static struct hwqueue *hwqueue_open_any(const char *name, unsigned flags,
					void *caller)
{
	struct hwqueue_device *hdev;
	int match = INT_MAX, _match, id;
	struct hwqueue_instance *inst = NULL, *_inst;
	struct hwqueue *qh = ERR_PTR(-ENODEV);

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		for_each_instance(id, _inst, hdev) {
			_match = hdev->ops->match(_inst, flags);
			if (_match < 0) /* match error */
				continue;
			if (_match >= match) /* match is no better */
				continue;
			if (hwqueue_is_exclusive(_inst))
				continue;
			if ((flags & O_CREAT) && hwqueue_is_busy(_inst))
				continue;

			match = _match;
			inst = _inst;

			if (!match) /* made for each other */
				break;
		}
	}

	if (inst)
		qh = __hwqueue_open(inst, name, flags, caller);

	mutex_unlock(&hwqueue_devices_lock);
	return qh;
}

static struct hwqueue *hwqueue_open_by_name(const char *name, unsigned flags,
					    void *caller)
{
	struct hwqueue_device *hdev;
	struct hwqueue_instance *inst;
	struct hwqueue *qh = ERR_PTR(-EINVAL);
	int id;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		for_each_instance(id, inst, hdev) {
			int match = hdev->ops->match(inst, flags);
			if (match < 0)
				continue;
			if (!hwqueue_is_busy(inst))
				continue;
			if (hwqueue_is_exclusive(inst))
				continue;
			if (strcmp(inst->name, name))
				continue;
			qh = __hwqueue_open(inst, name, flags, caller);
			goto unlock_ret;
		}
	}

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);
	return qh;
}

/**
 * hwqueue_open() - open a hardware queue
 * @name	- name to give the queue handle
 * @id		- desired queue number if any
 *		  HWQUEUE_ANY: allocate any free queue, implies O_CREAT
 *		  HWQUEUE_BYNAME: open existing queue by name, implies !O_CREAT
 * @flags	- the following flags are applicable to queues:
 *	O_EXCL		- insist on exclusive ownership - will fail if queue is
 *			  already open.  Subsequent attempts to open the same
 *			  queue will also fail. O_EXCL => O_CREAT here.
 *	O_CREAT		- insist that queue not be already open - will fail if
 *			  queue is already open. Subsequent attempts to open
 *			  the same queue may succeed. O_CREAT is implied if
 *			  queue id == HWQUEUE_ANY.
 *	O_RDONLY	- pop only access
 *	O_WRONLY	- push only access
 *	O_RDWR		- push and pop access
 *	O_NONBLOCK	- never block on pushes and pops
 *   In addition, the following "hints" to the driver/hardware may be passed
 *   in at open:
 *	O_HIGHTHROUGHPUT - hint high throughput usage
 *	O_LOWLATENCY	 - hint low latency usage
 *
 * Returns a handle to the open hardware queue if successful.  Use IS_ERR()
 * to check the returned value for error codes.
 */
struct hwqueue *hwqueue_open(const char *name, unsigned id, unsigned flags)
{
	struct hwqueue *qh = ERR_PTR(-EINVAL);
	void *caller = __builtin_return_address(0);

	if (flags & O_EXCL)
		flags |= O_CREAT;

	switch (id) {
	case HWQUEUE_ANY:
		qh = hwqueue_open_any(name, flags, caller);
		break;
	case HWQUEUE_BYNAME:
		if (WARN_ON(flags & (O_EXCL | O_CREAT)))
			break;
		qh = hwqueue_open_by_name(name, flags, caller);
		break;
	default:
		qh = hwqueue_open_by_id(name, id, flags, caller);
		break;
	}

	return qh;
}
EXPORT_SYMBOL(hwqueue_open);

static void devm_hwqueue_release(struct device *dev, void *res)
{
	hwqueue_close(*(struct hwqueue **)res);
}

struct hwqueue *devm_hwqueue_open(struct device *dev, const char *name,
				  unsigned id, unsigned flags)
{
	struct hwqueue **ptr, *queue;

	ptr = devres_alloc(devm_hwqueue_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return NULL;

	queue = hwqueue_open(name, id, flags);
	if (queue) {
		*ptr = queue;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return queue;
}
EXPORT_SYMBOL(devm_hwqueue_open);

/**
 * hwqueue_close() - close a hardware queue handle
 * @qh	- handle to close
 */
void hwqueue_close(struct hwqueue *qh)
{
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;
	unsigned long flags;
	bool enabled;

	while (atomic_read(&qh->notifier_enabled) > 0)
		hwqueue_disable_notifier(qh);

	spin_lock_irqsave(&inst->lock, flags);
	enabled = (atomic_read(&qh->monitor_enabled) == KNAV_QMSSM_ENABLE);
	spin_unlock_irqrestore(&inst->lock, flags);
	if (enabled)
		hwqueue_disable_monitoring(qh);

	mutex_lock(&hwqueue_devices_lock);
	list_del_rcu(&qh->list);
	mutex_unlock(&hwqueue_devices_lock);

	synchronize_rcu();

	module_put(hdev->dev->driver->owner);

	if (!hwqueue_is_busy(inst))
		hdev->ops->close(inst);
	free_percpu(qh->stats);
	kfree(qh);
}
EXPORT_SYMBOL(hwqueue_close);

static int devm_hwqueue_match(struct device *dev, void *res, void *match_data)
{
	return *(void **)res == match_data;
}

void devm_hwqueue_close(struct device *dev, struct hwqueue *qh)
{
	WARN_ON(devres_destroy(dev, devm_hwqueue_release, devm_hwqueue_match,
			       (void *)qh));
	hwqueue_close(qh);
}
EXPORT_SYMBOL(devm_hwqueue_close);

/**
 * hwqueue_get_id() - get an ID number for an open queue.  This ID may be
 *		      passed to another part of the kernel, which then opens the
 *		      queue by ID.
 * @qh	- queue handle
 *
 * Returns queue id (>= 0) on success, negative return value is an error.
 */
int hwqueue_get_id(struct hwqueue *qh)
{
	struct hwqueue_instance *inst;
	unsigned base_id;

	if (!qh)
		return 0;
	inst = qh->inst;
	base_id = inst->hdev->base_id;

	return base_id + hwqueue_inst_to_id(inst);
}
EXPORT_SYMBOL(hwqueue_get_id);

/**
 * hwqueue_get_hw_id() - get an ID number for an open queue.  This ID may be
 *			 passed to hardware modules as a part of
 *			 descriptor/buffer content.
 * @qh	- queue handle
 *
 * Returns queue id (>= 0) on success, negative return value is an error.
 */
int hwqueue_get_hw_id(struct hwqueue *qh)
{
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	if (!hdev->ops->get_hw_id)
		return -EINVAL;

	return hdev->ops->get_hw_id(inst);
}
EXPORT_SYMBOL(hwqueue_get_hw_id);

/**
 * hwqueue_enable_notifier() - Enable notifier callback for a queue handle.
 * @qh	- hardware queue handle
 *
 * Returns 0 on success, errno otherwise.
 */
int hwqueue_enable_notifier(struct hwqueue *qh)
{
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;
	bool first;
	unsigned long flags;

	if (!hwqueue_is_readable(qh))
		return -EINVAL;

	if (WARN_ON(!qh->notifier_fn))
		return -EINVAL;

	/* Protect against interrupt delivery */
	spin_lock_irqsave(&inst->lock, flags);

	/* Adjust the per handle notifier count */
	first = (atomic_inc_return(&qh->notifier_enabled) == 1);
	if (!first)
		goto unlock; /* nothing to do */

	/* Now adjust the per instance notifier count */
	first = (atomic_inc_return(&inst->num_notifiers) == 1);
	if (first)
		hdev->ops->set_notify(inst, true);

unlock:
	spin_unlock_irqrestore(&inst->lock, flags);
	return 0;
}
EXPORT_SYMBOL(hwqueue_enable_notifier);

/**
 * hwqueue_disable_notifier() - Disable notifier callback for a queue handle.
 * @qh	- hardware queue handle
 *
 * Returns 0 on success, errno otherwise.
 */
int hwqueue_disable_notifier(struct hwqueue *qh)
{
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;
	bool last;
	unsigned long flags;

	if (!hwqueue_is_readable(qh))
		return -EINVAL;

	/* Protect against interrupt delivery */
	spin_lock_irqsave(&inst->lock, flags);

	last = (atomic_dec_return(&qh->notifier_enabled) == 0);
	if (!last)
		goto unlock; /* nothing to do */

	last = (atomic_dec_return(&inst->num_notifiers) == 0);
	if (last)
		hdev->ops->set_notify(inst, false);

unlock:
	spin_unlock_irqrestore(&inst->lock, flags);
	return 0;
}
EXPORT_SYMBOL(hwqueue_disable_notifier);

/**
 * hwqueue_set_notifier() - Set a notifier callback to a queue handle.  This
 *			    notifier is called whenever the queue has
 *			    something to pop.
 * @qh	- hardware queue handle
 * @fn		- callback function
 * @fn_arg	- argument for the callback function
 *
 * Hardware queues can have multiple notifiers attached to them.
 * The underlying notification mechanism may vary from queue to queue.  For
 * example, some queues may issue notify callbacks on timer expiry, and some
 * may do so in interrupt context.  Notifier callbacks may be called from an
 * atomic context, and _must not_ block ever.
 *
 * Returns 0 on success, errno otherwise.
 */
int hwqueue_set_notifier(struct hwqueue *qh, hwqueue_notify_fn fn,
			 void *fn_arg)
{
	hwqueue_notify_fn old_fn = qh->notifier_fn;

	if (!hwqueue_is_readable(qh))
		return -EINVAL;

	if (!fn && old_fn)
		hwqueue_disable_notifier(qh);

	qh->notifier_fn = fn;
	qh->notifier_fn_arg = fn_arg;

	if (fn && !old_fn)
		hwqueue_enable_notifier(qh);

	return 0;
}
EXPORT_SYMBOL(hwqueue_set_notifier);

int hwqueue_enable_monitoring(struct hwqueue *qh)
{
	struct hwqueue_instance *inst;
	unsigned long flags;

	if (!qh)
		return -EINVAL;

	inst = qh->inst;

	if (WARN_ON(!qh->monitor_cfg))
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	atomic_set(&qh->monitor_enabled, KNAV_QMSSM_ENABLE);

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
EXPORT_SYMBOL(hwqueue_enable_monitoring);

int hwqueue_disable_monitoring(struct hwqueue *qh)
{
	struct hwqueue_instance *inst;
	unsigned long flags;

	if (!qh)
		return -EINVAL;

	inst = qh->inst;

	spin_lock_irqsave(&inst->lock, flags);

	atomic_set(&qh->monitor_enabled, KNAV_QMSSM_DISABLE);

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
EXPORT_SYMBOL(hwqueue_disable_monitoring);

/**
 * hwqueue_set_monitor() - Set monitor callback to a queue handle.
 * Monitor should record available descriptors track
 * @qh - hardware queue handle
 * @cfg fn - callback function
 * @cfg fn_arg - argument for the callback function, keep target
 * hwqueue handle which should be monitor
 * in case of self monitoring @fn_arg equal @qh
 */
int hwqueue_set_monitor(struct hwqueue *qh, struct hwqueue_monitor_config *cfg)
{
	int i = 0, j = 0, k = 0;
	bool diff;

	if (!hwqueue_is_readable(qh))
		return -EINVAL;

	hwqueue_disable_monitoring(qh);

	if (!cfg) {
		kzfree(qh->monitor_cfg);
		return 0;
	}

	qh->monitor_cfg = kzalloc(sizeof(*(qh->monitor_cfg)), GFP_KERNEL);
	if (!qh->monitor_cfg)
		return -ENOMEM;

	qh->monitor_cfg->fn = cfg->fn;
	for (i = 0; i < KNAV_QMSSM_FDQ_PER_CHAN; i++) {

		if (!cfg->fdq_arg[i])
			continue;

		diff = true;
		for (j = 0; j < i; j++)
			if (cfg->fdq_arg[i] == qh->monitor_cfg->fdq_arg[j]) {
				diff = false;
				break;
			}

		if (diff)
			qh->monitor_cfg->fdq_arg[k++] = cfg->fdq_arg[i];
	}

	dev_dbg(qh->inst->hdev->dev, "%s %d [%p %p %p %p]", __func__,
			hwqueue_get_id(qh),
			qh->monitor_cfg->fdq_arg[0], qh->monitor_cfg->fdq_arg[1],
			qh->monitor_cfg->fdq_arg[2], qh->monitor_cfg->fdq_arg[3]);

	return 0;
}
EXPORT_SYMBOL(hwqueue_set_monitor);

dma_addr_t __hwqueue_pop_slow(struct hwqueue *qh, unsigned *size,
			      struct timeval *timeout, unsigned flags)
{
	struct hwqueue_instance *inst = qh->inst;
	dma_addr_t dma_addr = 0;
	int ret;

	if (timeout) {
		unsigned long expires = timeval_to_jiffies(timeout);

		ret = wait_event_interruptible_timeout(inst->wait,
				(dma_addr = qh->pop(inst, size, flags)),
				expires);
		if (ret < 0)
			return 0;
		if (!ret && !dma_addr)
			return 0;
		jiffies_to_timeval(ret, timeout);
	} else {
		ret = wait_event_interruptible(inst->wait,
				(dma_addr = qh->pop(inst, size, flags)));
		if (ret < 0)
			return 0;
		if (WARN_ON(!ret && !dma_addr))
			return 0;
	}

	return dma_addr;
}
EXPORT_SYMBOL(__hwqueue_pop_slow);

/**
 * hwqueue_notify() - notify users on data availability
 * @inst	- hardware queue instance
 *
 * Walk through the notifier list for a hardware queue instance and issue
 * callbacks.  This function is called by drivers when data is available on a
 * hardware queue, either when notified via interrupt or on timer poll.
 */
void hwqueue_notify(struct hwqueue_instance *inst)
{
	struct hwqueue *qh;
	unsigned long flags;
	bool enabled;
	bool disable;

	rcu_read_lock();

	for_each_handle_rcu(qh, inst) {
		/* Synchronize against enable/disable notifier */
		spin_lock_irqsave(&inst->lock, flags);
		enabled = atomic_read(&qh->notifier_enabled) > 0;
		disable = atomic_read(&qh->monitor_enabled) == KNAV_QMSSM_DISABLE;
		spin_unlock_irqrestore(&inst->lock, flags);

		if (!enabled)
			continue;
		if (WARN_ON(!qh->notifier_fn))
			continue;
		this_cpu_inc(qh->stats->notifies);
		qh->notifier_fn(qh->notifier_fn_arg);

		if (disable)
			continue;

		if (WARN_ON(!qh->monitor_cfg))
			continue;

		qh->monitor_cfg->fn(qh);
	}

	rcu_read_unlock();

	wake_up_interruptible(&inst->wait);
}
EXPORT_SYMBOL(hwqueue_notify);

static void __hwqueue_poll(struct timer_list *t)
{
	struct hwqueue_instance *inst = from_timer(inst, t, poll_timer);
	struct hwqueue *qh;

	rcu_read_lock();

	for_each_handle_rcu(qh, inst) {
		if (hwqueue_get_count(qh) > 0)
			hwqueue_notify(inst);
		break;
	}

	rcu_read_unlock();

	mod_timer(&inst->poll_timer, jiffies +
		  msecs_to_jiffies(hwqueue_poll_interval));
}

void hwqueue_set_poll(struct hwqueue_instance *inst, bool enabled)
{
	unsigned long expires;

	if (!enabled) {
		del_timer(&inst->poll_timer);
		return;
	}

	expires = jiffies + msecs_to_jiffies(hwqueue_poll_interval);
	mod_timer(&inst->poll_timer, expires);
}
EXPORT_SYMBOL(hwqueue_set_poll);

static void hwqueue_debug_show_instance(struct seq_file *s,
					struct hwqueue_instance *inst)
{
	struct hwqueue_device *hdev = inst->hdev;
	struct hwqueue *qh;
	int cpu = 0;
	int pushes = 0;
	int pops = 0;
	int push_errors = 0;
	int pop_errors = 0;
	int notifies = 0;

	if (!hwqueue_is_busy(inst))
		return;

	seq_printf(s, "\tqueue id %d (%s)\n",
		   hdev->base_id + hwqueue_inst_to_id(inst), inst->name);

	for_each_handle_rcu(qh, inst) {
		for_each_possible_cpu(cpu) {
			pushes += per_cpu_ptr(qh->stats, cpu)->pushes;
			pops += per_cpu_ptr(qh->stats, cpu)->pops;
			push_errors += per_cpu_ptr(qh->stats, cpu)->push_errors;
			pop_errors += per_cpu_ptr(qh->stats, cpu)->pop_errors;
			notifies += per_cpu_ptr(qh->stats, cpu)->notifies;
		}

		seq_printf(s, "\t\thandle %p: pushes %8d, pops %8d, count %8d, notifies %8d, push errors %8d, pop errors %8d\n",
				qh,
				pushes,
				pops,
				hwqueue_get_count(qh),
				notifies,
				push_errors,
				pop_errors);
	}
}

static int hwqueue_debug_show(struct seq_file *s, void *v)
{
	struct hwqueue_device *hdev;
	struct hwqueue_instance *inst;
	int id;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		seq_printf(s, "hdev %s: %u-%u\n",
			   dev_name(hdev->dev), hdev->base_id,
			   hdev->base_id + hdev->num_queues - 1);

		for_each_instance(id, inst, hdev)
			hwqueue_debug_show_instance(s, inst);
	}

	mutex_unlock(&hwqueue_devices_lock);

	return 0;
}

static int hwqueue_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwqueue_debug_show, NULL);
}

static const struct file_operations hwqueue_debug_ops = {
	.open		= hwqueue_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init hwqueue_debug_init(void)
{
	debugfs_create_file("hwqueues", S_IFREG | S_IRUGO, NULL, NULL,
			    &hwqueue_debug_ops);
	return 0;
}
device_initcall(hwqueue_debug_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware queue interface");
