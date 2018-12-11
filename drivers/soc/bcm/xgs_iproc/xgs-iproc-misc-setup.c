/*
 * Copyright (C) 2016 Broadcom Corporation
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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#define IPROC_DMU_PCU_COMPATIBLE "brcm,iproc-dmu-pcu"
#define IPROC_WRAP_CTRL_COMPATIBLE "brcm,iproc-wrap-ctrl"
#define KT2_WRAP_MISC_COMPATIBLE "brcm,kt2-wrap-misc"

static void __iomem *iproc_dmu_pcu_base = NULL;
static void __iomem *iproc_wrap_ctrl_base = NULL;

extern void request_idm_timeout_interrupts(struct platform_device *);

void inline  __iomem *get_iproc_dmu_pcu_base(void)
{
	return iproc_dmu_pcu_base;
}

void inline __iomem *get_iproc_wrap_ctrl_base(void)
{
	return iproc_wrap_ctrl_base;
}

int inline is_wh2_amac_sgmii(void)
{
	return readl(get_iproc_wrap_ctrl_base() + 0xa8) & 0x04;
}

int xgs_iproc_misc_setup(void)
{
	struct device_node *np;
	void __iomem *wrap_misc_reg = NULL;
	u32 tmp;
	u32 wrap_misc_offset, serdes_ctrl_sel, serdes_mdio_sel;

	/* Get DMU/PCU base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_DMU_PCU_COMPATIBLE);
	if (!np) {
		pr_err("%s: No dmu/pcu node found\n", __func__);
		return -ENODEV ;
	}

	iproc_dmu_pcu_base = of_iomap(np, 0);
	if (!iproc_dmu_pcu_base)
		return -ENOMEM;

	/* Get WRAP CTRL base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_WRAP_CTRL_COMPATIBLE);
	if (!np) {
		pr_err("%s: No wrap ctrl node found\n", __func__);
		return -ENODEV;
	}

	iproc_wrap_ctrl_base = of_iomap(np, 0);
	if (!iproc_wrap_ctrl_base)
		return -ENOMEM;

	/* Enable AMAC SERDES MDIO SEL/CTRL for HX4/KT2 */
	if (!of_property_read_u32_index(np, "amac-serdes-mdio-ctrl-sel", 0,
		&wrap_misc_offset)) {
		of_property_read_u32_index(np, "amac-serdes-mdio-ctrl-sel", 1,
					&serdes_ctrl_sel);
		of_property_read_u32_index(np, "amac-serdes-mdio-ctrl-sel", 2,
					&serdes_mdio_sel);

		wrap_misc_reg = (void __iomem *)(iproc_wrap_ctrl_base +
						wrap_misc_offset);
		tmp = readl(wrap_misc_reg);
		tmp |= (1 << serdes_ctrl_sel) | (1 << serdes_mdio_sel);
		writel(tmp, wrap_misc_reg);
	}

	return 1;
}
