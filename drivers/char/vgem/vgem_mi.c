// SPDX-License-Identifier: GPL-2.0
/* VGEM driver. Implements MCU-DSP interface for data exchange
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/fs.h>
#include <asm/cacheflush.h>

#include "mi_defines.h"

// MSM base address
#define MCU_BA			0x0C501000
#define DSP_BA			0x0C508000

/* IPCGRX register start address */
#define IPC_GEN_REG_START	0x02620240
#define IPCGR(n)		(4 * (n))

#define MODULE_NAME		"vgem"

#define GET_LOCK(l, id, r) \
({ \
sync_cache_r(l); \
if (ioread32(l) != 0x0) {r = -EBUSY; goto end; } \
iowrite32(id, l); \
sync_cache_w(l); \
if (ioread32(l) != id) {r = -EBUSY; goto end; } \
})

#define REL_LOCK(l) ({iowrite32(0, l); sync_cache_w(l); ioread32(l); })

#define GET_READ_LOCK(l, id, r) GET_LOCK(l, id, r)
#define REL_READ_LOCK(l) REL_LOCK(l)
#define GET_WRITE_LOCK(l, id, r) GET_LOCK(l, id, r)
#define REL_WRITE_LOCK(l) REL_LOCK(l)


static int vgem_devs = 12;
static int vgem_major;

struct mi_dev {
	struct dsp_mcu_mi __iomem *msm;
	void __iomem    *ipcgrx;
	int		irq;
	u16		mid;
	struct  cdev cdev;

	wait_queue_head_t	read_wait;
	wait_queue_head_t	write_wait;
};

static struct class *vgem_mcu_class;
static struct class *vgem_dsp_class;

static struct mi_dev vgem_devices[12];

static irqreturn_t vgem_ipcgr_handler(int irq, void *_dev)
{
	struct mi_dev *dev = _dev;

	wake_up_interruptible(&dev->read_wait);
	return IRQ_HANDLED;
}

static int vgem_open(struct inode *inode, struct file *filp)
{
	struct mi_dev *dev; /* device information */

	/*  Find the device */
	dev = container_of(inode->i_cdev, struct mi_dev, cdev);

	/* and use filp->private_data to point to the device data */
	filp->private_data = dev;

	return 0;
}

static bool data_ready(struct mi_dev *dev, struct gem_header *gem_hdr, u32 *rbi)
{
	u32 c = 0;

	GET_READ_LOCK(&dev->msm->read_lock, dev->mid, c);

	sync_cache_r(&dev->msm->read_index);
	*rbi = ioread32(&dev->msm->read_index);

	sync_cache_r(&dev->msm->buffers[*rbi].dm_buf[0]);
	memcpy_fromio(gem_hdr, &dev->msm->buffers[*rbi].dm_buf[0], sizeof(struct gem_header));

	REL_READ_LOCK(&dev->msm->read_lock);

	return (gem_hdr->magic_word == MAGIC_WORD);
end:
	return false;
}


static ssize_t vgem_read(struct file *filp, char __user *buf, size_t count,
			loff_t *f_pos)
{
	struct mi_dev *dev = filp->private_data;
	struct gem_header gem_hdr;
	u32 rbi = 0;
	int wr;
	size_t dlf = sizeof(u16);
	size_t read_bytes;
	char data[512];

	if (count == 0)
		return 0;

	if (!data_ready(dev, &gem_hdr, &rbi)) {

		if (filp->f_flags & O_NONBLOCK) {
			count = -EAGAIN;
			goto end;
		}

		wr = wait_event_interruptible(dev->read_wait, data_ready(dev, &gem_hdr, &rbi));

		if (wr) {
			pr_err("%s wait_event_interruptible %d\n", __func__, wr);
			count = wr;
			goto end;
		}
	}

	GET_READ_LOCK(&dev->msm->read_lock, dev->mid, count);

	read_bytes = 0;

	while (count > (read_bytes + dlf + gem_hdr.size)) {

		if (gem_hdr.magic_word != MAGIC_WORD)
			goto end;

		memcpy(&data[0], &gem_hdr.size, dlf);
		memcpy_fromio(&data[0] + dlf, &dev->msm->buffers[rbi].dm_buf[0], (size_t)gem_hdr.size);

		if (copy_to_user(buf + read_bytes, &data[0],  (size_t)gem_hdr.size+dlf)) {
			pr_err("%s copy_to_user\n", __func__);
			goto end;
		}

		read_bytes += (dlf + (size_t)gem_hdr.size);

		/* Clear the magic word */
		iowrite32(0, &dev->msm->buffers[rbi].dm_buf[0]);
		sync_cache_w(&dev->msm->buffers[rbi].dm_buf[0]);

		/*Update read buffer index to next buffer */
		rbi += 1;
		rbi %= 15;
		iowrite32((rbi), &dev->msm->read_index);
		sync_cache_w(&dev->msm->read_index);

		memcpy_fromio(&gem_hdr, &dev->msm->buffers[rbi].dm_buf[0], sizeof(struct gem_header));
	}

end:
	REL_READ_LOCK(&dev->msm->read_lock);
	return read_bytes;
}

static ssize_t vgem_write(struct file *filp, const char __user *buf, size_t count,
			loff_t *f_pos)
{
	struct mi_dev *dev = filp->private_data;
	struct gem_header gem_hdr;
	u32 wbi;
	struct dsp_mcu_buffer data;

	if (count == 0)
		return count;

	GET_WRITE_LOCK(&dev->msm->write_lock, dev->mid, count);

	sync_cache_r(&dev->msm->write_index);
	wbi = ioread32(&dev->msm->write_index);

	/* Check DSP buffer available for writing */
	memcpy_fromio(&gem_hdr,  &dev->msm->buffers[wbi].dm_buf[0], sizeof(gem_hdr));

	if (gem_hdr.magic_word != 0x0) {
		pr_err("%s Buffer NULL ==> dev %d, buff %d\n", __func__, dev->mid, wbi);
		count = -EBUSY;
		/* Force DSP wake up */
		goto kick;
	}

	if (count > sizeof(u32) * 0x40) {
		pr_info("%s Requested to write %d exceedes maximum. Chop %d\n",
			__func__, count, sizeof(u32) * 0x40);
		count = sizeof(u32) * 0x40;
	}

	if (copy_from_user(&data.dm_buf[0], buf, count)) {
		count = -EFAULT;
		pr_err("%s, failed to copy user data\n", __func__);
		goto end;
	}

	memcpy_toio(&dev->msm->buffers[wbi].dm_buf[0], &data.dm_buf[0], count);
	sync_cache_w(&dev->msm->buffers[wbi].dm_buf[0]);

	wbi += 1;
	iowrite32(wbi % 15, &dev->msm->write_index);
	sync_cache_w(&dev->msm->write_index);

kick:
	REL_WRITE_LOCK(&dev->msm->write_lock);
	iowrite32(1, dev->ipcgrx);
	sync_cache_w(dev->ipcgrx);
	ioread32(dev->ipcgrx);
	return count;

end:
	REL_WRITE_LOCK(&dev->msm->write_lock);
	return count;
}

static __poll_t vgem_mcu_poll(struct file *filp, poll_table *wait)
{
	struct mi_dev *dev = filp->private_data;
	struct gem_header gem_hdr;
	u32 rbi;

	__poll_t res = 0;

	poll_wait(filp, &dev->read_wait, wait);

	if (data_ready(dev, &gem_hdr, &rbi))
		res = EPOLLIN | EPOLLRDNORM;

	return res;
}

static loff_t vgem_llseek(struct file *filp, loff_t off, int whence)
{
	struct mi_dev *dev = filp->private_data;
	struct gem_header gem_hdr;
	u32 rbi;
	loff_t dr = -1;

	switch (whence) {
	case SEEK_CUR:
		if (data_ready(dev, &gem_hdr, &rbi))
			dr = (loff_t)rbi;
		break;
	default:
		break;
	}
	return dr;
}

static int vgem_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int vgem_mcu_event(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0440);
	return 0;
}

static int vgem_dsp_event(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0660);
	return 0;
}


const struct file_operations vgem_mcu_fops = {
	.owner =     THIS_MODULE,
	.read =	vgem_read,
	.open =	vgem_open,
	.poll =	vgem_mcu_poll,
	.release = vgem_release,
	.llseek = vgem_llseek,
};

const struct file_operations vgem_dsp_fops = {
	.owner = THIS_MODULE,
	.write = vgem_write,
	.open = vgem_open,
	.release = vgem_release,
};


static void vgem_cleanup(void)
{
	int i;
	struct class *c;

	pr_info("%s start\n", __func__);

	for (i = 0; i < vgem_devs; i++) {
		if (vgem_devices[i].msm)
			iounmap(vgem_devices[i].msm);
		if (vgem_devices[i].ipcgrx)
			iounmap(vgem_devices[i].ipcgrx);

		c = (i < 4) ? vgem_mcu_class : vgem_dsp_class;
		device_destroy(c, MKDEV(vgem_major, i));
	}

	unregister_chrdev_region(MKDEV(vgem_major, 0), vgem_devs);

	pr_info("%s removing class\n", __func__);
	if (vgem_mcu_class)
		class_destroy(vgem_mcu_class);
	if (vgem_dsp_class)
		class_destroy(vgem_dsp_class);
}

static int vgem_exit(struct platform_device *pdev)
{
	vgem_cleanup();
	return 0;
}

static void init_cdev(struct mi_dev *dev, int idx, const struct file_operations *fops)
{
	int err, devno = MKDEV(vgem_major, idx);

	cdev_init(&dev->cdev, fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);

	if (err)
		pr_err("%s Error %d adding vgem interface %d\n", __func__, err, idx);
}

static int vgem_init(struct device *dev_parent)
{
	int result, i, di, devno;
	dev_t	dev;
	struct	device *d;
	struct	class *c;
	const struct file_operations *fops;

	pr_info("%s start\n", __func__);

	result = alloc_chrdev_region(&dev, 0, vgem_devs, "vgem");
	if (result < 0) {
		pr_err("%s failed with error %d\n", __func__, result);
		return result;
	}

	vgem_major = MAJOR(dev);

	vgem_mcu_class = class_create(THIS_MODULE, "mcu");
	vgem_dsp_class = class_create(THIS_MODULE, "dsp");

	vgem_mcu_class->dev_uevent = vgem_mcu_event;
	vgem_dsp_class->dev_uevent = vgem_dsp_event;

	for (i = 0; i < 12; i++) {
		memset(&vgem_devices[i], 0x0, sizeof(struct mi_dev));

		if (i < 4)
			vgem_devices[i].msm = ioremap(MCU_BA + (0x1000 * i), 0x1000);
		else
			vgem_devices[i].msm = ioremap(DSP_BA + (0x1000 * (i-4)), 0x1000);

		if (!vgem_devices[i].msm) {
			result = PTR_ERR(vgem_devices[i].msm);
			goto x_err;
		}
		memset_io(vgem_devices[i].msm, 0, 0x1000);

		if (i >= 4) {
			vgem_devices[i].ipcgrx = ioremap(IPC_GEN_REG_START + IPCGR(i-4), 4);
			if (!vgem_devices[i].ipcgrx) {
				result = PTR_ERR(vgem_devices[i].ipcgrx);
				goto x_err;
			}
		}

		vgem_devices[i].mid = 1 << i;

		init_waitqueue_head(&vgem_devices[i].read_wait);
		init_waitqueue_head(&vgem_devices[i].write_wait);

		c = (i < 4) ? vgem_mcu_class : vgem_dsp_class;
		fops = (i < 4) ? &vgem_mcu_fops : &vgem_dsp_fops;
		di = (i < 4) ? i : i-4;
		devno = MKDEV(vgem_major, i);

		init_cdev(&vgem_devices[i], i, fops);

		d = device_create(c, NULL, devno, NULL, "%s%d", c->name, di);
		pr_info("%p = device_create(c, NULL, devno, NULL, %s%d)\n", d, c->name, di);
		if (IS_ERR(d)) {
			result = PTR_ERR(d);
			goto x_err;
		}
	}

	pr_info("%s initialization completed\n", __func__);
	return 0;

x_err:
	pr_err("%s, error\n", __func__);
	vgem_cleanup();

	return result;
}


static int vgem_probe(struct platform_device *pdev)
{
	int irq;
	int ret;
	int i;

	struct device *dev = &pdev->dev;
	const struct of_device_id *id;

	pr_info("%s\n", __func__);

	id = of_match_device(dev->driver->of_match_table, dev);
	if (!id) {
		pr_err("%s failed to match data\n", __func__);
		return -ENODEV;
	}

	ret = vgem_init(&pdev->dev);

	if (ret < 0) {
		pr_err("%s failed to init mi channels %d\n", __func__, ret);
		goto x_error;
	}

	for (i = 0; i < 4; ++i) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			pr_err("%s %d No platform irq\n", __func__, i);
			ret = irq;
			goto irq_error;
		}

		ret = devm_request_threaded_irq(dev, irq, NULL,
						vgem_ipcgr_handler,
						IRQF_ONESHOT,
						dev_name(dev),
						&vgem_devices[i]);
		if (ret) {
			dev_err(dev, "request for irq %d failed\n", irq);
			goto irq_error;
		}
	}

	return 0;

irq_error:
	vgem_cleanup();

x_error:
	return ret;
}

static const struct of_device_id tetra_vgem_match[] = {
	{
		.compatible = "ab,tetra_vgem",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tetra_vgem_match);

static struct platform_driver tetra_vgem_driver = {
	.probe = vgem_probe,
	.remove = vgem_exit,
	.driver = {
		   .name = "tetra_vgem",
		   .owner = THIS_MODULE,
		   .of_match_table = tetra_vgem_match,
	},
};

module_platform_driver(tetra_vgem_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Isidro Gonzalez Cuxiart");
MODULE_DESCRIPTION("TETRA vGEM driver");
MODULE_VERSION("2.0");
