/*
 * IFC configuration
 *
 * Configure IFC chip select provided in ifc/config DTS node.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/fsl_ifc.h>

/* values from DTS */
struct ifc_config
{
	u32 cs;
	u32 cspr;
	u32 csor;
	u32 csor_ext;
	u32 amask;
	u32 ftim[4];
};

struct ifc_range {
	u32 cs;
	u32 n;
	u32 addr_h;
	u32 addr_l;
	u32 size;
};

#define IFC_CONFIG_MAX 4

int fsl_ifc_config(struct platform_device *dev,
		   struct fsl_ifc_global __iomem *p)
{
	struct ifc_config config[IFC_CONFIG_MAX], *c = config;
	struct ifc_range range[IFC_CONFIG_MAX], *r;
	int i, j, cs, config_len, range_len;

	config_len = of_property_read_variable_u32_array(
		dev->dev.of_node, "config", (u32 *)&config,
		0, sizeof(config) / sizeof(u32));
	if (config_len == -EINVAL) {
		dev_info(&dev->dev, "no config found\n");
		return 0;
	} else if (config_len < 0 ||
	    (config_len % (sizeof(struct ifc_config) / sizeof(u32))) != 0) {
		dev_err(&dev->dev, "invalid config\n");
		return -EINVAL;
	}

	range_len = of_property_read_variable_u32_array(
		dev->dev.of_node, "ranges", (u32 *)&range,
		0, sizeof(range) / sizeof(u32));
	if (range_len < 0 ||
	    (range_len % (sizeof(struct ifc_range) / sizeof(u32))) != 0) {
		dev_err(&dev->dev, "ranges missing\n");
		return -EINVAL;
	}

	config_len /= sizeof(struct ifc_config) / sizeof(u32);
	range_len /= sizeof(struct ifc_range) / sizeof(u32);

	for (i = 0; i < config_len; i++, c++) {

		cs = c->cs;
		if (cs >= FSL_IFC_BANK_COUNT)
			return -EINVAL;

		r = range;
		for (j = 0; j < range_len; j++, r++)
			if (r->cs == cs)
				break;
		if (j == range_len) {
			dev_err(&dev->dev, "cs not found in ranges node\n");
			return -EINVAL;
		}

		ifc_out32(r->addr_h, &p->cspr_cs[cs].cspr_ext);
		ifc_out32((r->addr_l & (0xffff << 16)) | c->cspr, &p->cspr_cs[cs].cspr);
		ifc_out32(c->csor, &p->csor_cs[cs].csor);
		ifc_out32(c->csor_ext, &p->csor_cs[cs].csor_ext);
		ifc_out32(c->amask, &p->amask_cs[cs].amask);
		for (j = 0; j < 4; j++)
			ifc_out32(c->ftim[j], &p->ftim_cs[cs].ftim[j]);

		dev_info(&dev->dev, "cs %d configured\n", cs);
	}
	return 0;
}
