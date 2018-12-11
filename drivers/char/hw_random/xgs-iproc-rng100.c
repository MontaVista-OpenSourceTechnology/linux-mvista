/*
 * Copyright (C) 2017 Broadcom Corporation
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

#include <linux/hw_random.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

/* Registers for RNG */
#define	RNG_CTRL_OFFSET			0x00000000
#define	RNG_CTRL_RESERVED_MASK		0xF00000CC
#define	RNG_CTRL_COMBLK2_OSC_DIS_SHIFT	22
#define	RNG_CTRL_COMBLK2_OSC_DIS_MASK	0x0FC00000
#define	RNG_CTRL_COMBLK1_OSC_DIS_SHIFT	16
#define	RNG_CTRL_COMBLK1_OSC_DIS_MASK	0x003F0000
#define	RNG_CTRL_JCLK_BYP_DIV_CNT_SHIFT	8
#define	RNG_CTRL_JCLK_BYP_DIV_CNT_MASK	0x0000FF00
#define	RNG_CTRL_JCLK_BYP_SRC_SHIFT	5
#define	RNG_CTRL_JCLK_BYP_SRC_MASK	0x00000020
#define	RNG_CTRL_JCLK_BYP_SEL_SHIFT	4
#define	RNG_CTRL_JCLK_BYP_SEL_MASK	0x00000010
#define	RNG_CTRL_RBG2X_SHIFT		1
#define	RNG_CTRL_RBG2X_MASK		0x00000002
#define	RNG_CTRL_RBGEN_SHIFT		0
#define	RNG_CTRL_RBGEN_MASK		0x00000001

#define	RNG_STATUS_OFFSET		0x00000004
#define	RNG_STATUS_RESERVED_MASK	0x00F00000
#define	RNG_STATUS_RND_VAL_SHIFT	24
#define	RNG_STATUS_RND_VAL_MASK		0xFF000000
#define	RNG_STATUS_WARM_CNT_SHIFT	0
#define	RNG_STATUS_WARM_CNT_MASK	0x000FFFFF

#define	RNG_DATA_OFFSET			0x00000008
#define	RNG_DATA_RESERVED_MASK		0x00000000
#define	RNG_DATA_RNG_NUM_SHIFT		0
#define	RNG_DATA_RNG_NUM_MASK		0xFFFFFFFF

#define	RNG_FF_THRES_OFFSET		0x0000000C
#define	RNG_FF_THRES_RESERVED_MASK	0xFFFFFFE0
#define	RNG_FF_THRES_RNG_FF_THRESH_SHIFT	0
#define	RNG_FF_THRES_RNG_FF_THRESH_MASK	0x0000001F

#define	RNG_INT_MASK_OFFSET		0x00000010
#define	RNG_INT_MASK_RESERVED_MASK	0xFFFFFFFE
#define	RNG_INT_MASK_OFF_SHIFT		0
#define	RNG_INT_MASK_OFF_MASK		0x00000001

static int rng100_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	u32 num_words = 0;
	u32 num_remaining = max;

	#define MAX_IDLE_TIME	(1 * HZ)
	unsigned long idle_endtime = jiffies + MAX_IDLE_TIME;

	/* Retrieve HW RNG registers base address. */
	void __iomem *base_addr = (void __iomem *)rng->priv;

	while ((num_remaining > 0) && time_before(jiffies, idle_endtime)) {
		/* Are there any random numbers available? */
		num_words = (ioread32(base_addr + RNG_STATUS_OFFSET) &
			RNG_STATUS_RND_VAL_MASK) >> RNG_STATUS_RND_VAL_SHIFT;
		if (num_words > 0) {
			if (num_remaining >= sizeof(u32)) {
				/* Buffer has room to store entire word */
				*(u32 *)buf = ioread32(base_addr +
							RNG_DATA_OFFSET);
				buf += sizeof(u32);
				num_remaining -= sizeof(u32);
			} else {
				/* Buffer can only store partial word */
				u32 rnd_number = ioread32(base_addr +
							RNG_DATA_OFFSET);
				memcpy(buf, &rnd_number, num_remaining);
				buf += num_remaining;
				num_remaining = 0;
			}

			/* Reset the IDLE timeout */
			idle_endtime = jiffies + MAX_IDLE_TIME;
		} else if (!wait) {
			/* Cannot wait, return immediately */
			break;
		} else {
			/* Can wait, give others chance to run */
			cpu_relax();
		}
	}

	return max - num_remaining;
}

static struct hwrng rng100_ops = {
	.name = "iproc-rng100",
	.read = rng100_read,
};

static int iproc_rng100_probe(struct platform_device *pdev)
{
	int error;
	u32 val;
	struct device *dev = &pdev->dev;
	void __iomem *base_addr;
	struct device_node *node = pdev->dev.of_node;

	pr_info("Broadcom IPROC RNG100 Driver\n");
	/* We only accept one device, and it must have an id of -1 */
	if (pdev->id != -1)
		return -ENODEV;

	base_addr = of_iomap(node, 0);
	if (!base_addr) {
		dev_err(&pdev->dev, "can't iomap base_addr for rng100\n");
		return -EIO;
	}
	rng100_ops.priv = (unsigned long)base_addr;

	/* Start RNG block */
	val = ioread32(base_addr + RNG_CTRL_OFFSET);
	val |= RNG_CTRL_RBGEN_MASK;
	iowrite32(val, base_addr + RNG_CTRL_OFFSET);

	/* Enable RNG RBG2X */
	val = ioread32(base_addr + RNG_CTRL_OFFSET);
	val |= RNG_CTRL_RBG2X_MASK;
	iowrite32(val, base_addr + RNG_CTRL_OFFSET);

	/* Disable RNG INTERRUPT */
	val = ioread32(base_addr + RNG_INT_MASK_OFFSET);
	val |= RNG_INT_MASK_OFF_MASK;
	iowrite32(val, base_addr + RNG_INT_MASK_OFFSET);

	/* set warmup cycle 0xfff */
	iowrite32(RNG_STATUS_WARM_CNT_MASK -
		  (0xfff & RNG_STATUS_WARM_CNT_MASK),
		  base_addr + RNG_STATUS_OFFSET);
	while ((ioread32(base_addr + RNG_STATUS_OFFSET) &
		RNG_STATUS_WARM_CNT_MASK) != RNG_STATUS_WARM_CNT_MASK)
		cpu_relax();

	/* register to the Linux RNG framework */
	error = hwrng_register(&rng100_ops);
	if (error) {
		dev_err(dev, "hwrng registration failed\n");
		iounmap(base_addr);
		return error;
	}
	dev_dbg(dev, "hwrng registered\n");

	return 0;
}

static int iproc_rng100_remove(struct platform_device *pdev)
{
	u32 val;
	void __iomem *base_addr = (void __iomem *)rng100_ops.priv;

	hwrng_unregister(&rng100_ops);

	if (base_addr) {
		/* Disable RNG hardware */
		val = ioread32(base_addr + RNG_CTRL_OFFSET);
		val &= ~RNG_CTRL_RBGEN_MASK;
		iowrite32(val, base_addr + RNG_CTRL_OFFSET);

		val = ioread32(base_addr + RNG_CTRL_OFFSET);
		val &= ~RNG_CTRL_RBG2X_MASK;
		iowrite32(val, base_addr + RNG_CTRL_OFFSET);

		iounmap(base_addr);
	}

	return 0;
}

static const struct of_device_id bcm_iproc_dt_ids[] = {
	{ .compatible = "brcm,iproc-rng100"},
	{  }
};
MODULE_DEVICE_TABLE(of, bcm_iproc_dt_ids);

static struct platform_driver iproc_rng100_driver = {
	.driver = {
		.name = "iproc-rng100",
		.owner = THIS_MODULE,
		.of_match_table = bcm_iproc_dt_ids,
	},
	.probe = iproc_rng100_probe,
	.remove = iproc_rng100_remove,
};
module_platform_driver(iproc_rng100_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("iProc RNG100 Random Number Generator driver");
MODULE_LICENSE("GPL v2");
