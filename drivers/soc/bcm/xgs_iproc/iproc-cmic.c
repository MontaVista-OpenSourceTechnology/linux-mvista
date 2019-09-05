/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/module.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/soc/bcm/iproc-cmic.h>

extern const struct sbus_ops cmicx_sbus_ops;
extern const struct sbus_ops cmicd_sbus_ops;

static struct iproc_cmic *cmic;

int iproc_cmic_schan_reg32_write(u32 blk_type, u32 addr, u32 val)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->reg32_write) {
			return cmic->sbus_ops->reg32_write(cmic, blk_type, addr, val);
		}
	}
	return -EINVAL;
}

u32 iproc_cmic_schan_reg32_read(u32 blk_type, u32 addr)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->reg32_read) {
			return cmic->sbus_ops->reg32_read(cmic, blk_type, addr);
		}
	}
	return 0;
}

int iproc_cmic_schan_reg64_write(u32 blk_type, u32 addr, u64 val)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->reg64_write) {
			return cmic->sbus_ops->reg64_write(cmic, blk_type, addr, val);
		}
	}
	return -EINVAL;
}

u64 iproc_cmic_schan_reg64_read(u32 blk_type, u32 addr)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->reg64_read) {
			return cmic->sbus_ops->reg64_read(cmic, blk_type, addr);
		}
	}
	return 0;
}

int iproc_cmic_schan_ucmem_write(u32 blk_type, u32 *mem)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->ucmem_write) {
			return cmic->sbus_ops->ucmem_write(cmic, blk_type, mem);
		}
	}
	return -EINVAL;
}

int iproc_cmic_schan_ucmem_read(u32 blk_type, u32 *mem)
{
	if (cmic && cmic->sbus_ops) {
		if (cmic->sbus_ops->ucmem_read) {
			return cmic->sbus_ops->ucmem_read(cmic, blk_type, mem);
		}
	}
	return -EINVAL;
}

void inline __iomem *iproc_cmic_base_get(void)
{
	if (cmic && cmic->base) {
		return cmic->base;
	}
	return NULL;
}

/****************************************************************************
 ***************************************************************************/
int xgs_iproc_cmic_init(int dev_id)
{
	struct device_node *np;

	cmic = kmalloc(sizeof(*cmic), GFP_KERNEL);
	if (!cmic) {
		return -ENOMEM;
	}
	cmic->device = dev_id;

	if ((np = of_find_compatible_node(NULL, NULL, "brcm,iproc-cmicx"))) {
		cmic->sbus_ops = &cmicx_sbus_ops;
	} else if ((np = of_find_compatible_node(NULL, NULL, "brcm,iproc-cmicd"))) {
		cmic->sbus_ops = &cmicd_sbus_ops;
	} else {
		printk("Can't find cmic node in dts file\n");
		return -ENODEV;
	}

	cmic->base = (void *)of_iomap(np, 0);
	if (IS_ERR(cmic->base)) {
		printk("Unable to iomap CMIC resource\n");
		return PTR_ERR(cmic->base);
	}

	if (cmic->sbus_ops) {
		if (cmic->sbus_ops->init) {
			/* Initial cmic */
			cmic->sbus_ops->init(cmic);
		}
	}

	return 0;
}

static int cmic_remove(struct platform_device *pdev)
{
	if (cmic->base) {
		iounmap(cmic->base);
	}

	devm_kfree(&pdev->dev, cmic);
	cmic = NULL;

	return 0;
}
