/*
 * RapidIO userspace interface for Direct I/O and doorbells
 *
 * Copyright (C) 2010, 2013 Texas Instruments Incorporated
 * Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>
#include <linux/rio_regs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/uaccess.h>

#include "rio.h"

/*
 * This supports acccess to RapidIO devices using normal userspace I/O calls.
 *
 * RapidIO has a character major number assigned. We allocate minor numbers
 * dynamically using a bitmask. You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/rio0.1 device
 * nodes, since there is no fixed association of minor numbers with any
 * particular RapidIO site or device.
 */
#define RIO_DEV_MAJOR 	154	/* assigned */
#define RIO_DEV_NAME    "rio"
#define N_RIO_MINORS	32	/* number of minors per instance (up to 256) */

static unsigned long minors[N_RIO_MINORS / BITS_PER_LONG];
static unsigned long init_done = 0;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static spinlock_t       dbell_i_lock;
static spinlock_t       dbell_list_lock;
static struct list_head dbell_list;

struct dbell_cell {
	struct list_head   node;
	u16                info;
	wait_queue_head_t  waitq;
};

#ifdef CONFIG_MMU
#define USE_COPY 1
#else
#define USE_COPY 0
#endif

/*
 * RapidIO File ops
 */
static loff_t rio_dev_llseek(struct file* filp, loff_t off, int whence)
{
	loff_t new;

	switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = filp->f_pos + off; break;
	case 2:	 new = 1 + off; break;
	default: return -EINVAL;
	}

	return (filp->f_pos = new);
}

static ssize_t rio_dev_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	struct rio_dev *rdev = filp->private_data;
	int res;
	size_t count = size;
	size_t write_sz;
	char *src_buf;
	char *p;
	int copy = 0;
	int write_pos = 0;

	if (!size)
		return 0;

	if (!rdev->net->hport->ops->transfer) {
		dev_dbg(&rdev->dev, "write: no transfer method\n");
		return -EPROTONOSUPPORT;
	}

	dev_dbg(&rdev->dev, "write buffer, p = 0x%lx, size = %zu\n",
		(unsigned long)buf, size);

	/* try to do zero-copy if buffer has the right property */
	if ((virt_to_phys(buf) != L1_CACHE_ALIGN(virt_to_phys(buf))) ||
	    (size < L1_CACHE_BYTES) || (USE_COPY)) {

		int alloc_size = (size > RIO_MAX_DIO_CHUNK_SIZE) ?
			RIO_MAX_DIO_CHUNK_SIZE : size;

		p = src_buf = kmalloc(L1_CACHE_ALIGN(alloc_size), GFP_KERNEL);

		if (!p) {
			res = -ENOMEM;
			goto out;
		}

		copy = 1;

		/* if allocated buffer is still non-aligned on cache */
		if ((unsigned long) p != L1_CACHE_ALIGN((unsigned long) p)) {
			res = -ENOMEM;
			goto out;
		}

		dev_dbg(&rdev->dev, "allocating write buffer, p = 0x%lx, size = %zu\n",
			(unsigned long)p, L1_CACHE_ALIGN(size));
	} else {
		p    = src_buf = (char *) buf; /* zero-copy case */
		copy = 0;
	}

	while(count) {
		if (copy) {
			write_sz = (count <= RIO_MAX_DIO_CHUNK_SIZE) ?
				count : RIO_MAX_DIO_CHUNK_SIZE;
			if (copy_from_user(p, buf + write_pos, write_sz)) {
				res = -EFAULT;
				goto out;
			}
		} else
			write_sz = count;

		count -= write_sz;

		dev_dbg(&rdev->dev,
			"writing, size = %zu, ppos = 0x%x, buf = 0x%lx, mode = 0x%x\n",
			write_sz, (u32) *ppos + rdev->dio.base_offset,
			(unsigned long) p,
			rdev->dio.write_mode);

		/* Start the DIO transfer */
		res = rdev->net->hport->ops->transfer(rdev->net->hport,
						      rdev->net->hport->id,
						      rdev->destid,
						      (u32) (unsigned long) p,
						      (u32) (*ppos + rdev->dio.base_offset),
						      write_sz,
						      rdev->dio.write_mode);
		if (res) {
			dev_dbg(&rdev->dev, "write transfer failed (%d)\n", res);
			goto out;
		}

		if (!copy)
			p += write_sz;

		*ppos     += (u64) write_sz;
		write_pos += write_sz;
	}

	dev_dbg(&rdev->dev, "write finished, size = %zu, ppos = 0x%llx\n",
		size, *ppos + rdev->dio.base_offset);

	res = size;
out:
	if (copy)
		kfree(src_buf);

	return res;
}

static ssize_t rio_dev_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	struct rio_dev *rdev = filp->private_data;
	int res;
	size_t count = size;
	size_t read_sz;
	char *dest_buf;
	char *p;
	int copy = 0;
	int read_pos = 0;

	if (!rdev->net->hport->ops->transfer) {
		dev_dbg(&rdev->dev, "read: no transfer method\n");
		return -EPROTONOSUPPORT;
	}

	dev_dbg(&rdev->dev, "read buffer, buf = 0x%lx, size = %zu\n",
		(unsigned long) buf, size);

	/* try to do zero-copy is buffer has the right property */
	if ((virt_to_phys(buf) != L1_CACHE_ALIGN(virt_to_phys(buf))) ||
	    (size < L1_CACHE_ALIGN(size)) || (USE_COPY)) {
		int alloc_size = (size > RIO_MAX_DIO_CHUNK_SIZE) ?
			RIO_MAX_DIO_CHUNK_SIZE : size;

		p = dest_buf = kmalloc(L1_CACHE_ALIGN(alloc_size), GFP_KERNEL);
		if (!p) {
			res = -ENOMEM;
			goto out;
		}

		copy = 1;

		/* if allocated buffer is still non-aligned on cache */
		if ((unsigned long) p != L1_CACHE_ALIGN((unsigned long) p)) {
			res = -ENOMEM;
			goto out;
		}

		dev_dbg(&rdev->dev, "allocating read buffer, p = 0x%lx, size = %zu\n",
			(unsigned long)p, L1_CACHE_ALIGN(size));
	} else {
		p = dest_buf = buf; /* zero-copy case */
		copy = 0;
	}

	while(count) {
		if (copy)
			read_sz = (count <= RIO_MAX_DIO_CHUNK_SIZE) ?
				count : RIO_MAX_DIO_CHUNK_SIZE;
		else
			read_sz = count;

		count  -= read_sz;

		dev_dbg(&rdev->dev,
			"reading, size = %zu, ppos = 0x%x, base_offset = 0x%x\n",
			read_sz, (u32) *ppos, rdev->dio.base_offset);

		/* Start the DIO transfer */
		res = rdev->net->hport->ops->transfer(rdev->net->hport,
						      rdev->net->hport->id,
						      rdev->destid,
						      (u32) (unsigned long)p,
						      (u32) (*ppos + rdev->dio.base_offset),
						      read_sz,
						      RIO_DIO_MODE_READ);
		if (res) {
			dev_dbg(&rdev->dev, "read transfer failed (%d)\n", res);
			goto out;
		}

		dev_dbg(&rdev->dev, "incoming data, size = %zu, user buf = 0x%lx, buf = 0x%lx\n",
			size, (unsigned long) buf, (unsigned long) dest_buf);

		if (copy) {
			if (copy_to_user(buf + read_pos, p, read_sz)) {
				res = -EFAULT;
				goto out;
			}
		} else
			p += read_sz;

		*ppos    += (u64) read_sz;
		read_pos += read_sz;
	}

	dev_dbg(&rdev->dev, "read finished, size = %zu, ppos = 0x%llx\n",
		size,
		*ppos + rdev->dio.base_offset);

	res = size;
out:
	if (copy)
		kfree(dest_buf);

	return res;
}

static void rio_dev_dbell_callback(struct rio_mport *mport,
				   void *dev_id,
				   u16 src,
				   u16 dst,
				   u16 info)
{
	wait_queue_head_t *dbell_waitq = (wait_queue_head_t*) dev_id;

	/* Wake up user process */
	if (waitqueue_active(dbell_waitq))
		wake_up_all(dbell_waitq);
}

static struct dbell_cell* rio_dev_dbell_lookup(u16 info)
{
	struct dbell_cell *dbell;
	int found = 0;

	spin_lock(&dbell_list_lock);

	/* Look if a waitqueue already exists for this doorbell */
	list_for_each_entry(dbell, &dbell_list, node) {
		if (dbell->info == info) {
			found = 1;
			break;
		}
	}

	if (found) {
		goto out;
	}

	/* Allocate and insert the doorbell */
	dbell = (struct dbell_cell*) kmalloc(sizeof(struct dbell_cell), GFP_KERNEL);
	if (dbell == NULL)
		goto out;

	dbell->info = info;
	init_waitqueue_head(&dbell->waitq);

	list_add_tail(&dbell->node, &dbell_list);
out:
	spin_unlock(&dbell_list_lock);

	return dbell;
}

static int rio_dev_dbell_release(u16 info)
{
	struct dbell_cell *dbell;
	int found = 0;
	int res   = 0;

	spin_lock(&dbell_list_lock);

	/* Look for the corresponding waitqueue */
	list_for_each_entry(dbell, &dbell_list, node) {
		if (dbell->info == info) {
			found = 1;
			break;
		}
	}

	if (!found) {
		res = -EINVAL;
		goto out;
	}

	/* Delete and free waitqueue from list */
	list_del(&dbell->node);
	kfree(dbell);

out:
	spin_unlock(&dbell_list_lock);

	return res;
}

static int rio_dev_dbell_wait(struct rio_dev *rdev, u16 info)
{
	struct rio_mport *mport = rdev->net->hport;
	struct dbell_cell *dbell;
	unsigned long flags;
	int res;

	DECLARE_WAITQUEUE(wait, current);

	dbell = rio_dev_dbell_lookup(info);
	if (dbell == NULL)
		return -ENOMEM;

	/* Request a doorbell with our callback handler */
	res = rio_request_inb_dbell(mport,
				    (void *) &dbell->waitq,
				    info,
				    info,
				    rio_dev_dbell_callback);

	if ((res != 0) && (res != -EBUSY)) {
		dev_dbg(&rdev->dev, "DBELL: cannot request such doorbell (info = %d)\n", info);
		return res;
	}

	/* Schedule until handler is called */
	spin_lock_irqsave(&dbell_i_lock, flags);
	add_wait_queue(&dbell->waitq, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irqrestore(&dbell_i_lock, flags);

	schedule();

	spin_lock_irqsave(&dbell_i_lock, flags);
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&dbell->waitq, &wait);
	spin_unlock_irqrestore(&dbell_i_lock, flags);

	/* Release the doorbell */
	rio_release_inb_dbell(mport, info, info);
	rio_dev_dbell_release(info);

	if (signal_pending(current))
		return -ERESTARTSYS;

	dev_dbg(&rdev->dev, "DBELL: receiving doorbell (info = %d)\n", info);

	return info;
}

static long rio_dev_ioctl(struct file *filp,
			  unsigned int cmd,
			  unsigned long arg)
{
	struct rio_dev *rdev = filp->private_data;
	u32 dbell_info;
	int mode;
	int status = 0;
	u32 base;

	switch (cmd) {

	case RIO_DIO_BASE_SET:
		if (get_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		rdev->dio.base_offset = base;
		break;

	case RIO_DIO_BASE_GET:
		base = rdev->dio.base_offset;
		if (put_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DIO_MODE_SET:
		if (get_user(mode, (int *) arg)) {
                        status = -EFAULT;
			break;
		}
		switch(mode & 0xf) {
		case RIO_DIO_MODE_WRITER:
			rdev->dio.write_mode = RIO_DIO_MODE_WRITER;
			break;
		case RIO_DIO_MODE_WRITE:
			rdev->dio.write_mode = RIO_DIO_MODE_WRITE;
			break;
		case RIO_DIO_MODE_SWRITE:
			rdev->dio.write_mode = RIO_DIO_MODE_SWRITE;
			break;
		default:
			status = -EINVAL;
			break;
		}
		break;

	case RIO_DIO_MODE_GET:
		mode = rdev->dio.write_mode;
		if (put_user(mode, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DBELL_TX:
		if (get_user(dbell_info, (int *) arg)) {
                        status = -EFAULT;
			break;
		}

		/* Send a doorbell */
		if (rdev->net->hport->ops->dsend) {
			status = rdev->net->hport->ops->dsend(rdev->net->hport,
							      rdev->net->hport->id,
							      rdev->destid,
							      (u16) dbell_info);
		} else {
			status = -EPROTONOSUPPORT;
			dev_dbg(&rdev->dev, "ioctl: no dsend method\n");
		}
		break;

	case RIO_DBELL_RX:
		if (get_user(dbell_info, (int *) arg)) {
                        status = -EFAULT;
			break;
		}

		/* Wait a doorbell */
		status = rio_dev_dbell_wait(rdev, (u16) dbell_info);
		break;

	default:
		status = -EINVAL;
        }
	return status;
}

static int rio_dev_open(struct inode *inode, struct file *filp)
{
	struct rio_dev *rdev;
	int status = -ENXIO;

	WARN_ON(in_interrupt());

	rdev = rio_get_devt(inode->i_rdev, NULL);
	if (rdev == NULL)
		goto not_found;

	rdev = rio_dev_get(rdev);
	if (rdev == NULL)
		goto not_found;

	filp->private_data = rdev;
	status = 0;

not_found:
	return status;
}

static int rio_dev_release(struct inode *inode, struct file *filp)
{
	struct rio_dev *rdev;
	int status = 0;

	mutex_lock(&device_list_lock);

	rdev = filp->private_data;
	filp->private_data = NULL;

	mutex_unlock(&device_list_lock);

	return status;
}

static struct file_operations rio_dev_fops = {
	.owner =	  THIS_MODULE,
	.llseek =         rio_dev_llseek,
	.write =	  rio_dev_write,
	.read =		  rio_dev_read,
	.unlocked_ioctl = rio_dev_ioctl,
	.open =		  rio_dev_open,
	.release =	  rio_dev_release,
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/rio0.0 character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static void rio_dev_classdev_release(struct device *dev)
{
}

static struct class rio_dev_class = {
	.name	     = "rio_dev",
	.owner	     = THIS_MODULE,
	.dev_release = rio_dev_classdev_release,
};

/*
 * Called when adding a site, this will create the corresponding char device
 * with udev/mdev
 */
int rio_dev_add(struct rio_dev *rdev)
{
	struct rio_mport *port = rdev->net->hport;
	int status;
	unsigned long minor;

	if (!init_done)
		return -ENODEV;

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_RIO_MINORS);

	if (minor < N_RIO_MINORS) {
		struct device *dev;

		rdev->dio.base_offset = 0;
		rdev->dio.write_mode  = RIO_DIO_MODE_WRITER;
		rdev->dev.devt = MKDEV(RIO_DEV_MAJOR, minor);

		dev = device_create(&rio_dev_class,
				    &rdev->dev,
				    rdev->dev.devt,
				    rdev,
				    "%s%d.%d",
				    RIO_DEV_NAME,
				    port->index,
				    rdev->destid);

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&rdev->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0)
		set_bit(minor, minors);

	mutex_unlock(&device_list_lock);

	return status;
}

int rio_dev_remove(struct rio_dev *rdev, struct rio_mport *port)
{
	mutex_lock(&device_list_lock);
	device_destroy(&rio_dev_class, rdev->dev.devt);
	clear_bit(MINOR(rdev->dev.devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

int rio_dev_init(void)
{
	int status;

	spin_lock_init(&dbell_i_lock);
	spin_lock_init(&dbell_list_lock);
	INIT_LIST_HEAD(&dbell_list);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes. Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_RIO_MINORS > 256);
	status = register_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME, &rio_dev_fops);
	if (status < 0)
		return status;

	status = class_register(&rio_dev_class);
	if (status < 0) {
		unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
		return status;
	}

	init_done = 1;

	return 0;
}

void rio_dev_exit(void)
{
	class_unregister(&rio_dev_class);
	unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
	init_done = 0;
}

EXPORT_SYMBOL_GPL(rio_dev_init);
EXPORT_SYMBOL_GPL(rio_dev_exit);
EXPORT_SYMBOL_GPL(rio_dev_add);
EXPORT_SYMBOL_GPL(rio_dev_remove);
