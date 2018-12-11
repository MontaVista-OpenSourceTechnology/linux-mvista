/*
 * Copyright (C) 2016 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>

#include "mdio-xgs-iproc.h"

/* CMICD MDIO */
#define CMIC_MIIM_PARAM_OFFSET 0x080
#define CMIC_MIIM_PARAM_MIIM_CYCLE_R 29
#define CMIC_MIIM_PARAM_MIIM_CYCLE_W 3
#define CMIC_MIIM_PARAM_INTERNAL_SEL 25
#define CMIC_MIIM_PARAM_INTERNAL_SEL_W 1
#define CMIC_MIIM_PARAM_BUS_ID_R 22
#define CMIC_MIIM_PARAM_BUS_ID_W 3
#define CMIC_MIIM_PARAM_C45_SEL 21
#define CMIC_MIIM_PARAM_C45_SEL_W 1
#define CMIC_MIIM_PARAM_PHY_ID_R 16
#define CMIC_MIIM_PARAM_PHY_ID_W 5
#define CMIC_MIIM_PARAM_PHY_DATA_R 0
#define CMIC_MIIM_PARAM_PHY_DATA_W 16

#define CMIC_MIIM_READ_DATA_OFFSET 0x084
#define CMIC_MIIM_READ_DATA_DATA_R 0
#define CMIC_MIIM_READ_DATA_DATA_W 16

#define CMIC_MIIM_ADDR_OFFSET 0x088
#define CMIC_MIIM_ADDR_C45_DTYPE_R 16
#define CMIC_MIIM_ADDR_C45_DTYPE_W 5
#define CMIC_MIIM_ADDR_C45_REGADR_R 0
#define CMIC_MIIM_ADDR_C45_REGADR_W 16
#define CMIC_MIIM_ADDR_C22_REGADR_R 0
#define CMIC_MIIM_ADDR_C22_REGADR_W 5

#define CMIC_MIIM_CTRL_OFFSET 0x08c
#define CMIC_MIIM_CTRL_RD_START 1
#define CMIC_MIIM_CTRL_RD_START_W 1
#define CMIC_MIIM_CTRL_WR_START 0
#define CMIC_MIIM_CTRL_WR_START_W 1

#define CMIC_MIIM_STAT_OFFSET 0x090
#define CMIC_MIIM_STAT_OPN_DONE 0
#define CMIC_MIIM_STAT_OPN_DONE_W 1

#define CMIC_COMMON_UC0_PIO_ENDIANESS 	0x1F0

#define MIIM_PARAM_REG			CMIC_MIIM_PARAM_OFFSET
#define MIIM_PARAM_MIIM_CYCLE_SHIFT	CMIC_MIIM_PARAM_MIIM_CYCLE_R
#define MIIM_PARAM_MIIM_CYCLE_MASK	((1 << CMIC_MIIM_PARAM_MIIM_CYCLE_W)-1)
#define MIIM_PARAM_INTERNAL_SEL_SHIFT	CMIC_MIIM_PARAM_INTERNAL_SEL
#define MIIM_PARAM_INTERNAL_SEL_MASK	((1<<CMIC_MIIM_PARAM_INTERNAL_SEL_W)-1)
#define MIIM_PARAM_BUS_ID_SHIFT 	CMIC_MIIM_PARAM_BUS_ID_R
#define MIIM_PARAM_BUS_ID_MASK		((1 << CMIC_MIIM_PARAM_BUS_ID_W) - 1)
#define MIIM_PARAM_C45_SEL_SHIFT	CMIC_MIIM_PARAM_C45_SEL
#define MIIM_PARAM_C45_SEL_MASK 	((1 << CMIC_MIIM_PARAM_C45_SEL_W) - 1)
#define MIIM_PARAM_PHY_ID_SHIFT 	CMIC_MIIM_PARAM_PHY_ID_R
#define MIIM_PARAM_PHY_ID_MASK		((1 << CMIC_MIIM_PARAM_PHY_ID_W) - 1)
#define MIIM_PARAM_PHY_DATA_SHIFT 	CMIC_MIIM_PARAM_PHY_DATA_R
#define MIIM_PARAM_PHY_DATA_MASK	((1 << CMIC_MIIM_PARAM_PHY_DATA_W) - 1)

#define MIIM_READ_DATA_REG 		CMIC_MIIM_READ_DATA_OFFSET
#define MIIM_READ_DATA_DATA_SHIFT	CMIC_MIIM_READ_DATA_DATA_R
#define MIIM_READ_DATA_DATA_MASK	((1 << CMIC_MIIM_READ_DATA_DATA_W) - 1)

#define MIIM_ADDRESS_REG 		CMIC_MIIM_ADDR_OFFSET
#define MIIM_ADDR_C45_DTYPE_SHIFT	CMIC_MIIM_ADDR_C45_DTYPE_R
#define MIIM_ADDR_C45_DTYPE_MASK	((1 << CMIC_MIIM_ADDR_C45_DTYPE_W) - 1)
#define MIIM_ADDR_C45_REGADR_SHIFT	CMIC_MIIM_ADDR_C45_REGADR_R
#define MIIM_ADDR_C45_REGADR_MASK	((1 << CMIC_MIIM_ADDR_C45_REGADR_W) - 1)
#define MIIM_ADDR_C22_REGADR_SHIFT	CMIC_MIIM_ADDR_C22_REGADR_R
#define MIIM_ADDR_C22_REGADR_MASK	((1 << CMIC_MIIM_ADDR_C22_REGADR_W) - 1)

#define MIIM_CTRL_REG 			CMIC_MIIM_CTRL_OFFSET
#define MIIM_CTRL_RD_START_SHIFT	CMIC_MIIM_CTRL_RD_START
#define MIIM_CTRL_RD_START_MASK 	((1 << CMIC_MIIM_CTRL_RD_START_W) - 1)
#define MIIM_CTRL_WR_START_SHIFT	CMIC_MIIM_CTRL_WR_START
#define MIIM_CTRL_WR_START_MASK 	((1 << CMIC_MIIM_CTRL_WR_START_W) - 1)

#define MIIM_STAT_REG 			CMIC_MIIM_STAT_OFFSET
#define MIIM_STAT_OPN_DONE_SHIFT	CMIC_MIIM_STAT_OPN_DONE
#define MIIM_STAT_OPN_DONE_MASK 	((1 << CMIC_MIIM_STAT_OPN_DONE_W) - 1)

struct cmicd_miim_cmd {
	int bus_id;
	int int_sel;
	int phy_id;
	int regnum;
	int c45_sel;
	u16 op_mode;
	u16 val;
};

static struct iproc_mdio_ctrl *cmic_common = NULL;


static inline u32 cmicd_miim_reg_read(struct iproc_mdio_ctrl *cmic_mdio, u32 reg)
{
	u32 value = readl(cmic_mdio->base + reg);
#if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
       if (readl(cmic_mdio->base + CMIC_COMMON_UC0_PIO_ENDIANESS) != 0)
		/* CMICD is in big-endian mode */
		value = swab32(value);
#endif
	return value;
}

static inline void cmicd_miim_reg_write(struct iproc_mdio_ctrl *cmic_mdio,
				u32 reg, u32 data)
{
	u32 value = data;
#if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
	if (readl(cmic_mdio->base + CMIC_COMMON_UC0_PIO_ENDIANESS) != 0)
		/* CMICD is in big-endian mode */
		value = swab32(data);
#endif
	writel(value, cmic_mdio->base + reg);
}

static inline void cmicd_miim_set_op_read(u32 *data, u32 set)
{
	SET_REG_FIELD(*data, MIIM_CTRL_RD_START_SHIFT,
			MIIM_CTRL_RD_START_MASK, set);
}

static inline void cmicd_miim_set_op_write(u32 *data, u32 set)
{
	SET_REG_FIELD(*data, MIIM_CTRL_WR_START_SHIFT,
			MIIM_CTRL_WR_START_MASK, set);
}

static int do_cmicd_miim_op(struct iproc_mdio_ctrl *cmic_mdio, u32 op,
					u32 param, u32 addr)
{
	u32 val, op_done;
	unsigned long flags;
	int ret = 0;
	int usec = MII_OP_MAX_HALT_USEC;

	if (op >= MII_OP_MODE_MAX) {
		pr_err("%s : invalid op code %d\n", __func__, op);
		return -EINVAL;
	}

	spin_lock_irqsave(&cmic_mdio->lock, flags);

	cmicd_miim_reg_write(cmic_mdio, MIIM_PARAM_REG, param);
	cmicd_miim_reg_write(cmic_mdio, MIIM_ADDRESS_REG, addr);
	val = cmicd_miim_reg_read(cmic_mdio, MIIM_CTRL_REG);
	if(op == MII_OP_MODE_READ)
		cmicd_miim_set_op_read(&val, 1);
	else
		cmicd_miim_set_op_write(&val, 1);
	cmicd_miim_reg_write(cmic_mdio, MIIM_CTRL_REG, val);

	do {
		op_done = GET_REG_FIELD(
				cmicd_miim_reg_read(cmic_mdio, MIIM_STAT_REG),
				MIIM_STAT_OPN_DONE_SHIFT,
				MIIM_STAT_OPN_DONE_MASK);
		if (op_done)
			break;

		udelay(1);
	} while (usec-- > 0);

	if (op_done) {
		if(op == MII_OP_MODE_READ)
			ret = cmicd_miim_reg_read(cmic_mdio, MIIM_READ_DATA_REG);
	} else {
		ret = -ETIME;
	}

	val = cmicd_miim_reg_read(cmic_mdio, MIIM_CTRL_REG);
	if(op == MII_OP_MODE_READ)
		cmicd_miim_set_op_read(&val, 0);
	else
		cmicd_miim_set_op_write(&val, 0);
	cmicd_miim_reg_write(cmic_mdio, MIIM_CTRL_REG, val);

	spin_unlock_irqrestore(&cmic_mdio->lock, flags);

	return ret;
}


static int cmicd_miim_op(struct iproc_mdio_ctrl *cmic_mdio,
		struct cmicd_miim_cmd *cmd)
{
	u32 miim_param =0, miim_addr = 0;

	ISET_REG_FIELD(miim_param, MIIM_PARAM_BUS_ID_SHIFT,
			MIIM_PARAM_BUS_ID_MASK, cmd->bus_id);

	if (cmd->int_sel)
		ISET_REG_FIELD(miim_param, MIIM_PARAM_INTERNAL_SEL_SHIFT,
				MIIM_PARAM_INTERNAL_SEL_MASK, 1);

	ISET_REG_FIELD(miim_param, MIIM_PARAM_PHY_ID_SHIFT,
			MIIM_PARAM_PHY_ID_MASK, cmd->phy_id);

	if (cmd->op_mode == MII_OP_MODE_WRITE)
		ISET_REG_FIELD(miim_param, MIIM_PARAM_PHY_DATA_SHIFT,
				MIIM_PARAM_PHY_DATA_MASK, cmd->val);

	if (cmd->c45_sel) {
		ISET_REG_FIELD(miim_param, MIIM_PARAM_C45_SEL_SHIFT,
				MIIM_PARAM_C45_SEL_MASK, 1);
		ISET_REG_FIELD(miim_addr, MIIM_ADDR_C45_REGADR_SHIFT,
				MIIM_ADDR_C45_REGADR_MASK, cmd->regnum);
		ISET_REG_FIELD(miim_addr, MIIM_ADDR_C45_DTYPE_SHIFT,
				MIIM_ADDR_C45_REGADR_MASK, cmd->regnum >> 16);
	}
	else {
		ISET_REG_FIELD(miim_addr, MIIM_ADDR_C22_REGADR_SHIFT,
				MIIM_ADDR_C22_REGADR_MASK, cmd->regnum);
	}

	return do_cmicd_miim_op(cmic_mdio, cmd->op_mode, miim_param, miim_addr);
}


static int cmicd_mdiobus_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct iproc_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = bus_priv->bus_data;
	struct cmicd_miim_cmd cmd = {0};

	cmd.bus_id = bus_data->phybus_num;
	if (IPROC_MDIOBUS_TYPE_INTERNAL == bus_data->phybus_type)
		cmd.int_sel = 1;
	cmd.phy_id = phy_id;
	cmd.regnum = regnum;

	if (regnum & MII_ADDR_C45)
		cmd.c45_sel = 1;

	cmd.op_mode = MII_OP_MODE_READ;

	return cmicd_miim_op(bus_priv->hw_ctrl, &cmd);
}

static int cmicd_mdiobus_write(struct mii_bus *bus, int phy_id,
				int regnum, u16 val)
{
	struct iproc_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = bus_priv->bus_data;
	struct cmicd_miim_cmd cmd = {0};

	cmd.bus_id = bus_data->phybus_num;
	if (IPROC_MDIOBUS_TYPE_INTERNAL == bus_data->phybus_type)
		cmd.int_sel = 1;
	cmd.phy_id = phy_id;
	cmd.regnum = regnum;
	cmd.val = val;

	if (regnum & MII_ADDR_C45)
		cmd.c45_sel = 1;

	cmd.op_mode = MII_OP_MODE_WRITE;

	return cmicd_miim_op(bus_priv->hw_ctrl, &cmd);
}

static struct iproc_mdio_ctrl * cmicd_mdio_res_alloc(void)
{
	if (!cmic_common) {
		cmic_common = kzalloc(sizeof(*cmic_common), GFP_KERNEL);
		if (!cmic_common)
			return NULL;
		spin_lock_init(&cmic_common->lock);
		cmic_common->ref_cnt = 1;
	}
	else
		cmic_common->ref_cnt++;

	return cmic_common;
}

static void cmicd_mdio_res_free(struct iproc_mdio_ctrl *ctrl)
{
	if (ctrl) {
		ctrl->ref_cnt--;
		if (ctrl->ref_cnt == 0) {
			iounmap(ctrl->base);
			kfree(ctrl);
			cmic_common = NULL;
		}
	}
}

static int cmicd_mdiobus_probe(struct platform_device *pdev)
{
	struct mii_bus *mii_bus;
	struct device_node *dn = pdev->dev.of_node;
	struct iproc_mdiobus_private *bus_priv;
	struct iproc_mdiobus_data *bus_data;
	struct iproc_mdio_ctrl *cmicd_ctrl;
	u32 mdio_bus_id;
	const char *mdio_bus_type;
	int ret;

	cmicd_ctrl = cmicd_mdio_res_alloc();
	if (!cmicd_ctrl) {
		dev_err(&pdev->dev, "cmicd mdio resource alloc failed\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	/* Get register base address for the first mdio node only */
	if (!cmicd_ctrl->base)
		cmicd_ctrl->base = of_iomap(dn, 0);
	if (!cmicd_ctrl->base) {
		dev_err(&pdev->dev, "cmicd mdio register base map error\n");
		ret = -ENXIO;
		goto err_ctrl_free;
	}

	/* If no property available, use default: 2 */
	if (of_property_read_u32(dn, "#bus-id", &mdio_bus_id))
		mdio_bus_id = 2;

	/* If no property available, use default: "external" */
	if (of_property_read_string(dn, "bus-type", &mdio_bus_type))
		mdio_bus_type = "external";

	bus_data = devm_kzalloc(&pdev->dev, sizeof(*bus_data), GFP_KERNEL);
	if (!bus_data) {
		dev_err(&pdev->dev, "iProc MDIO bus data alloc failed\n");
		ret = -ENOMEM;
		goto err_bus_data_free;
	}

	bus_priv = devm_kzalloc(&pdev->dev, sizeof(*bus_priv), GFP_KERNEL);
	if (!bus_priv) {
		dev_err(&pdev->dev, "iProc MDIO private data alloc failed\n");
		ret = -ENOMEM;
		goto err_bus_priv_free;
	}

	bus_data->phybus_num = mdio_bus_id;
	if (!strcmp(mdio_bus_type, "internal"))
		bus_data->phybus_type = IPROC_MDIOBUS_TYPE_INTERNAL;
	else
		bus_data->phybus_type = IPROC_MDIOBUS_TYPE_EXTERNAL;

	bus_priv->bus_data = bus_data;
	bus_priv->hw_ctrl = cmicd_ctrl;

	mii_bus = mdiobus_alloc();
	if (!mii_bus) {
		dev_err(&pdev->dev, "mdiobus_alloc failed\n");
		ret = -ENOMEM;
		goto err_ctrl_free;
	}

	mii_bus->name = "iproc_cmicd_mdiobus";
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s-%d-%d", "cmicd mdio",
		mdio_bus_id, bus_data->phybus_type? 1:0);
	mii_bus->parent = &pdev->dev;
	mii_bus->read = cmicd_mdiobus_read;
	mii_bus->write = cmicd_mdiobus_write;
	mii_bus->priv = bus_priv;

	ret = of_mdiobus_register(mii_bus, dn);
	if (ret) {
		dev_err(&pdev->dev, "mdiobus register failed\n");
		goto err_bus_free;
	}

	platform_set_drvdata(pdev, mii_bus);

	return 0;

err_bus_free:
	mdiobus_free(mii_bus);
err_bus_priv_free:
	kfree(bus_priv);
err_bus_data_free:
	kfree(bus_data);
err_ctrl_free:
	cmicd_mdio_res_free(cmicd_ctrl);
err_exit:
	return ret;
}

static int cmicd_mdiobus_remove(struct platform_device *pdev)
{
	struct mii_bus *mii_bus = platform_get_drvdata(pdev);
	struct iproc_mdiobus_private *bus_priv;

	if (mii_bus) {
		bus_priv = mii_bus->priv;

		mdiobus_unregister(mii_bus);
		if (bus_priv)
			cmicd_mdio_res_free(bus_priv->hw_ctrl);
		mdiobus_free(mii_bus);
	}

	return 0;
}

static const struct of_device_id cmicd_mdio_dt_ids[] = {
	{ .compatible = "brcm,iproc-cmicd-mdio"},
	{  }
};
MODULE_DEVICE_TABLE(of, cmicd_mdio_dt_ids);

static struct platform_driver iproc_cmicd_mdiobus_driver =
{
	.driver = {
		.name = "iproc_cmicd_mdio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cmicd_mdio_dt_ids),
	},
	.probe   = cmicd_mdiobus_probe,
	.remove  = cmicd_mdiobus_remove,
};

static int __init cmicd_mdio_init(void)
{
	return platform_driver_register(&iproc_cmicd_mdiobus_driver);
}

static void __exit cmicd_mdio_exit(void)
{
	platform_driver_unregister(&iproc_cmicd_mdiobus_driver);
}

/* export ccb_mii_read() and ccb_mii_write() for SDK BDE (for northstar chips) */
int ccb_mii_read(int dev_type, int phy_addr, int reg_off, u16 *data)
{
	return 0;
}

int ccb_mii_write(int dev_type, int phy_addr, int reg_off, u16 data)
{
	return 0;
}


//module_init(cmicd_mdio_init);
subsys_initcall(cmicd_mdio_init);
module_exit(cmicd_mdio_exit);

EXPORT_SYMBOL(ccb_mii_read);
EXPORT_SYMBOL(ccb_mii_write);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("iProc CMICd mdio driver");
MODULE_LICENSE("GPL");
