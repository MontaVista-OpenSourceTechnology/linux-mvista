/*
 * Copyright 2017 Broadcom Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

extern void __iomem *get_iproc_dmu_pcu_base(void);

#define PNOR_DIRECT_CMD_OFFSET			0x10
#define PNOR_SET_OPMODE_OFFSET			0X18

#define IPROC_STRAP_BOOT_DEV_NAND		1
#define IPROC_STRAP_BOOT_DEV_PNOR		4
#define ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R	0
#define PNOR_set_opmode__set_mw_R		0
#define PNOR_direct_cmd__cmd_type_R		21
#define IPROC_DMU_STRAPS_OFFSET 		0x28
#define IPROC_BOOT_STRAP_MASK			0x7

struct iproc_nor_mtd {
	void __iomem *reg_base;
	void __iomem *reg_strap;
	struct map_info map;
	struct mtd_info *mtd;
};

static int xgs_iproc_nor_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct iproc_nor_mtd *iproc_data = NULL;
	struct resource *res;
	u32 straps, val;
	u32 strap_boot_dev_shift;
	void __iomem *reg_base;

	dev_info(dev, "XGS iProc pnor interface driver\n");

	iproc_data = devm_kzalloc(dev, sizeof(*iproc_data), GFP_KERNEL);
	if (!iproc_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, iproc_data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iproc_data->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iproc_data->reg_base)) {
		dev_err(dev, "can't ioremap pnor base addr\n");
		return PTR_ERR(iproc_data->reg_base);
	}

	/* Check boot device */
	if (of_machine_is_compatible("brcm,hurricane2") ||
		of_machine_is_compatible("brcm,greyhound"))
		strap_boot_dev_shift = 9;
	else
		strap_boot_dev_shift = 10;

	straps = readl(get_iproc_dmu_pcu_base() + IPROC_DMU_STRAPS_OFFSET);
	straps = (straps >> strap_boot_dev_shift) & IPROC_BOOT_STRAP_MASK;

	if (straps == IPROC_STRAP_BOOT_DEV_NAND) {
		/* If booting from NAND, PNOR cannot be used */
		return -ENODEV;
	} else if (straps != IPROC_STRAP_BOOT_DEV_PNOR) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		iproc_data->reg_strap = devm_ioremap_resource(dev, res);

		if (!IS_ERR(iproc_data->reg_strap)) {
			/* Configure controller memory width based on strap */
			reg_base = iproc_data->reg_base;
			straps = readl(iproc_data->reg_strap) &
				(1 << ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R);

			if (straps) {
				/* 16-bit */
				val = readl(reg_base + PNOR_SET_OPMODE_OFFSET);
				val |= (1 << PNOR_set_opmode__set_mw_R);
				writel(val, reg_base + PNOR_SET_OPMODE_OFFSET);
			} else {
				/* 8-bit */
				val = readl(reg_base + PNOR_SET_OPMODE_OFFSET);
				val &= ~(1 << PNOR_set_opmode__set_mw_R);
				writel(val, reg_base + PNOR_SET_OPMODE_OFFSET);
			}

			val = readl(reg_base + PNOR_DIRECT_CMD_OFFSET);
			val |= (2 << PNOR_direct_cmd__cmd_type_R);
			writel(val, reg_base + PNOR_DIRECT_CMD_OFFSET);
		}
	}

	mdelay(1);

	iproc_data->map.bankwidth = 2;
	iproc_data->map.name = "XGS-IPROC-PNOR-FLASH";

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	iproc_data->map.phys = res->start;
	iproc_data->map.size = resource_size(res);
	iproc_data->map.virt = devm_ioremap_resource(dev, res);

	if (IS_ERR(iproc_data->map.virt)) {
		dev_err(dev, "can't ioremap pnor flash addr space\n");
		return PTR_ERR(iproc_data->map.virt);
	}

	simple_map_init(&iproc_data->map);

	dev_info(dev, "MTD probing 16 bit PNOR FLASH\n");
	iproc_data->mtd = do_map_probe("cfi_probe", &iproc_data->map);
	if (!iproc_data->mtd) {
		/* Probe for bankwidth 1 */
		dev_info(dev, "MTD probing 8 bit PNOR FLASH\n");
		iproc_data->map.bankwidth = 1;
		iproc_data->mtd = do_map_probe("cfi_probe", &iproc_data->map);
	}

	if (iproc_data->mtd) {
		iproc_data->mtd->owner = THIS_MODULE;
		iproc_data->mtd->dev.parent = dev;
		mtd_set_of_node(iproc_data->mtd, np);
		mtd_device_parse_register(iproc_data->mtd, NULL, NULL, NULL, 0);
		dev_info(dev, "PNOR MTD partitions parsed!\n");
		return 0;
	}

	dev_warn(dev, "NO PNOR FLASH found!\n");

	return -ENXIO;
}

static int xgs_iproc_nor_remove(struct platform_device *pdev)
{
	struct iproc_nor_mtd *iproc_data = platform_get_drvdata(pdev);

	if (!iproc_data)
		return 0;

	if (iproc_data->mtd) {
		mtd_device_unregister(iproc_data->mtd);
		map_destroy(iproc_data->mtd);
	}

	return 0;
}

static const struct of_device_id xgs_iproc_nor_dt_ids[] = {
	{ .compatible = "brcm,iproc-nor", },
	{ }
};
MODULE_DEVICE_TABLE(of, xgs_iproc_nor_dt_ids);

static struct platform_driver xgs_iproc_nor_driver =
{
	.driver = {
		.name = "xgs-iproc-nor",
		.owner = THIS_MODULE,
		.of_match_table = xgs_iproc_nor_dt_ids,
	},
	.probe = xgs_iproc_nor_probe,
	.remove = xgs_iproc_nor_remove,
};

module_platform_driver(xgs_iproc_nor_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("MTD map driver for XGS iProc PNOR controller");
MODULE_LICENSE("GPL");
