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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include "mdio-xgs-iproc.h"

#define MGMT_CTL_REG			0x000
#define MGMT_CTL__BYP_SHIFT		10
#define MGMT_CTL__BYP_WIDTH		1
#define MGMT_CTL__BYP_MASK		((1 << MGMT_CTL__BYP_WIDTH) - 1)
#define MGMT_CTL__EXT_SHIFT		9
#define MGMT_CTL__EXT_WIDTH		1
#define MGMT_CTL__EXT_MASK		((1 << MGMT_CTL__EXT_WIDTH) - 1)
#define MGMT_CTL__BSY_SHIFT		8
#define MGMT_CTL__BSY_WIDTH		1
#define MGMT_CTL__BSY_MASK		((1 << MGMT_CTL__BSY_WIDTH) - 1)
#define MGMT_CTL__PRE_SHIFT		7
#define MGMT_CTL__PRE_WIDTH		1
#define MGMT_CTL__PRE_MASK		((1 << MGMT_CTL__BSY_WIDTH) - 1)
#define MGMT_CTL__MDCDIV_SHIFT		0
#define MGMT_CTL__MDCDIV_WIDTH		7
#define MGMT_CTL__MDCDIV_MASK		((1 << MGMT_CTL__MDCDIV_WIDTH) - 1)

#define MGMT_CMD_DATA_REG		0x004
#define MGMT_CMD_DATA__SB_SHIFT 	30
#define MGMT_CMD_DATA__SB_WIDTH 	2
#define MGMT_CMD_DATA__SB_MASK		((1 << MGMT_CMD_DATA__SB_WIDTH) - 1)
#define MGMT_CMD_DATA__OP_SHIFT 	28
#define MGMT_CMD_DATA__OP_WIDTH 	2
#define MGMT_CMD_DATA__OP_MASK		((1 << MGMT_CMD_DATA__OP_WIDTH) - 1)
#define MGMT_CMD_DATA__PA_SHIFT 	23
#define MGMT_CMD_DATA__PA_WIDTH 	5
#define MGMT_CMD_DATA__PA_MASK		((1 << MGMT_CMD_DATA__PA_WIDTH) - 1)
#define MGMT_CMD_DATA__RA_SHIFT 	18
#define MGMT_CMD_DATA__RA_WIDTH 	5
#define MGMT_CMD_DATA__RA_MASK		((1 << MGMT_CMD_DATA__RA_WIDTH) - 1)
#define MGMT_CMD_DATA__TA_SHIFT 	16
#define MGMT_CMD_DATA__TA_WIDTH 	2
#define MGMT_CMD_DATA__TA_MASK		((1 << MGMT_CMD_DATA__TA_WIDTH) - 1)
#define MGMT_CMD_DATA__DATA_SHIFT	0
#define MGMT_CMD_DATA__DATA_WIDTH	16
#define MGMT_CMD_DATA__DATA_MASK	((1 << MGMT_CMD_DATA__DATA_WIDTH) - 1)

#define MII_OP_HALT_USEC	10

struct cc_mii_cmd {
	int bus_id;
	int ext_sel;
	int phy_id;
	int regnum;
	u16 op_mode;
	u16 val;
};

static struct iproc_mdio_ctrl *cc_mdio_ctrl = NULL;

/*
 * For HX4/KT2, the mdio bus is shared with iProc mdio and CMICd mdio
 * controllers. By default the mdio bus is released to CMICd for SDK to run.
 * Set kernel argument mdio_bus_release to 0 if ethtool test is required.
 */
static bool mdio_bus_release = true;
static int __init set_mdio_bus_release(char *str)
{
	return strtobool(str, &mdio_bus_release);
}
__setup("mdio_bus_release=", set_mdio_bus_release);


/* HX4/KT2 need to release mdio bus from iProc to cmicd */
bool xgs_mdio_bus_release(void)
{
	if (of_machine_is_compatible("brcm,helix4") ||
		of_machine_is_compatible("brcm,katana2"))
		return mdio_bus_release;
	else
		return 0;
}

/* HX4/KT2/SB2 needs to enable iProc mdio bus access, default is cimcd access */
static void xgs_iproc_mdio_enable(struct iproc_mdio_ctrl *ctrl, int enable)
{
	void __iomem *iproc_mdio_enable_reg = NULL;
	u32 iproc_mdio_sel, serdes_mdio_sel = 0;
	u32 tmp;

	if (!ctrl->iproc_mdio_enable_reg)
		return;

	iproc_mdio_enable_reg = ctrl->iproc_mdio_enable_reg;
	iproc_mdio_sel = ctrl->iproc_mdio_sel_bit;
	if (iproc_mdio_sel > 1)
		serdes_mdio_sel = iproc_mdio_sel - 1;

	tmp = readl(iproc_mdio_enable_reg);

	if (enable) {
		tmp |= (1 << iproc_mdio_sel);
		tmp |= serdes_mdio_sel? (1 << serdes_mdio_sel) : 0;
	} else {
		tmp &= ~(1 << iproc_mdio_sel);
		if (serdes_mdio_sel)
			tmp &= ~(1 << serdes_mdio_sel);
	}

	writel(tmp, iproc_mdio_enable_reg);
}

static inline u32 cc_mii_reg_read(struct iproc_mdio_ctrl *cc_mii, u32 reg)
{
	return readl(cc_mii->base + reg);
}

static inline void cc_mii_reg_write(struct iproc_mdio_ctrl *cc_mii,
				u32 reg, u32 data)
{
	writel(data, cc_mii->base + reg);
}

static int cc_mii_busy(struct iproc_mdio_ctrl *cc_mii, int to_usec)
{
	do {
		if(!GET_REG_FIELD(cc_mii_reg_read(cc_mii, MGMT_CTL_REG),
			MGMT_CTL__BSY_SHIFT, MGMT_CTL__BSY_MASK))
			return 0;

		udelay(MII_OP_HALT_USEC);
		to_usec -= MII_OP_HALT_USEC;
	} while (to_usec > 0);

	return 1;
}

static int do_cc_mii_op(struct iproc_mdio_ctrl *cc_mii, struct cc_mii_cmd *cmd)
{
	u32 cmd_data = 0;
	u32 mgt_ctrl;
	u32 op_mode = cmd->op_mode;
	unsigned long flags;
	int ret = 0;

	if (MII_OP_MODE_WRITE == op_mode) {
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__OP_SHIFT,
				MGMT_CMD_DATA__OP_MASK, 1);
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__DATA_SHIFT,
				MGMT_CMD_DATA__DATA_MASK, cmd->val);
	}
	else if (MII_OP_MODE_READ == op_mode) {
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__OP_SHIFT,
				MGMT_CMD_DATA__OP_MASK, 2);
	}
	else {
		pr_err("%s : invalid op code %d\n", __func__, op_mode);
		return -EINVAL;
	}

	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__PA_SHIFT,
			MGMT_CMD_DATA__PA_MASK, cmd->phy_id);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__RA_SHIFT,
			MGMT_CMD_DATA__RA_MASK, cmd->regnum);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__TA_SHIFT,
			MGMT_CMD_DATA__TA_MASK, 2);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__SB_SHIFT,
			MGMT_CMD_DATA__SB_MASK, 1);

	spin_lock_irqsave(&cc_mii->lock, flags);

	if (cc_mii_busy(cc_mii, MII_OP_MAX_HALT_USEC)) {
		ret = -EBUSY;
		pr_err("%s : bus busy (1)\n", __func__);
		goto err_exit_unlock;
	}

	mgt_ctrl = cc_mii_reg_read(cc_mii, MGMT_CTL_REG);
	if (cmd->ext_sel != GET_REG_FIELD(mgt_ctrl, MGMT_CTL__EXT_SHIFT,
						MGMT_CTL__EXT_MASK)) {
		SET_REG_FIELD(mgt_ctrl, MGMT_CTL__EXT_SHIFT,
				MGMT_CTL__EXT_MASK, cmd->ext_sel);
		cc_mii_reg_write(cc_mii, MGMT_CTL_REG, mgt_ctrl);
	}

	cc_mii_reg_write(cc_mii, MGMT_CMD_DATA_REG, cmd_data);

	if (cc_mii_busy(cc_mii, MII_OP_MAX_HALT_USEC)) {
		ret = -EBUSY;
		pr_err("%s : bus busy (2)\n", __func__);
		goto err_exit_unlock;
	}

	if (MII_OP_MODE_READ == cmd->op_mode) {
		ret = GET_REG_FIELD(cc_mii_reg_read(cc_mii, MGMT_CMD_DATA_REG),
			MGMT_CMD_DATA__DATA_SHIFT, MGMT_CMD_DATA__DATA_MASK);
	}

        spin_unlock_irqrestore(&cc_mii->lock, flags);

	return ret;

err_exit_unlock:
        spin_unlock_irqrestore(&cc_mii->lock, flags);
	return ret;
}

static int cc_mdiobus_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct iproc_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = bus_priv->bus_data;
	struct cc_mii_cmd cmd = {0};
	int ret;

	xgs_iproc_mdio_enable(bus_priv->hw_ctrl, 1);

	cmd.bus_id = bus_data->phybus_num;
	if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		cmd.ext_sel = 1;
	cmd.phy_id = phy_id;
	cmd.regnum = regnum;
	cmd.op_mode = MII_OP_MODE_READ;

	ret = do_cc_mii_op(bus_priv->hw_ctrl, &cmd);

	xgs_iproc_mdio_enable(bus_priv->hw_ctrl, 0);

	return ret;
}

static int cc_mdiobus_write(struct mii_bus *bus, int phy_id,
				int regnum, u16 val)
{
	struct iproc_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = bus_priv->bus_data;
	struct cc_mii_cmd cmd = {0};
	int ret;

	xgs_iproc_mdio_enable(bus_priv->hw_ctrl, 1);

	cmd.bus_id = bus_data->phybus_num;

	if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		cmd.ext_sel = 1;

	cmd.phy_id = phy_id;
	cmd.regnum = regnum;
	cmd.op_mode = MII_OP_MODE_WRITE;
	cmd.val = val;

	ret =  do_cc_mii_op(bus_priv->hw_ctrl, &cmd);

	xgs_iproc_mdio_enable(bus_priv->hw_ctrl, 0);

	return ret;
}

static struct iproc_mdio_ctrl * cc_mdio_res_alloc(void)
{
	if (!cc_mdio_ctrl) {
		cc_mdio_ctrl = kzalloc(sizeof(*cc_mdio_ctrl), GFP_KERNEL);
		if (!cc_mdio_ctrl)
			return NULL;

		spin_lock_init(&cc_mdio_ctrl->lock);
		cc_mdio_ctrl->ref_cnt = 1;
	}
	else
		cc_mdio_ctrl->ref_cnt++;

	return cc_mdio_ctrl;
}

static void cc_mdio_res_free(struct iproc_mdio_ctrl *ctrl)
{
	if (ctrl) {
		ctrl->ref_cnt--;
		if (ctrl->ref_cnt == 0) {
			iounmap(ctrl->base);
			kfree(ctrl);
			cc_mdio_ctrl = NULL;
		}
	}
}

static void cc_mii_init(struct iproc_mdio_ctrl *cc_mii, u32 mdio_clk_rate)
{
	u32 val = 0;

	if(cc_mii->ref_cnt == 1) {
		/* Set preamble enabled */
		ISET_REG_FIELD(val, MGMT_CTL__PRE_SHIFT, MGMT_CTL__PRE_MASK, 1);
		/* Set the MII clock to 1 MHz */
		ISET_REG_FIELD(val, MGMT_CTL__MDCDIV_SHIFT, MGMT_CTL__MDCDIV_MASK,
				mdio_clk_rate/(1000000));
		cc_mii_reg_write(cc_mii, MGMT_CTL_REG, val);
	}
}

static int cc_mdiobus_probe(struct platform_device *pdev)
{
	struct mii_bus *mii_bus;
	struct device_node *dn = pdev->dev.of_node;
	struct resource *res;
	struct iproc_mdiobus_private *bus_priv;
	struct iproc_mdiobus_data *bus_data;
	struct iproc_mdio_ctrl *cc_ctrl;
	u32 mdio_clk_rate;
	const char *mdio_bus_type;
	struct clk *clk=NULL;
	int ret;

	/* hw_ctrl is shared */
	if (cc_mdio_ctrl)
		goto hw_ctrl_allocated;

	cc_ctrl = cc_mdio_res_alloc();
	if (!cc_ctrl) {
		dev_err(&pdev->dev, "CC mdio resource alloc failed\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	cc_ctrl->base = (void *)of_iomap(dn, 0);
	if (!cc_ctrl->base) {
		dev_err(&pdev->dev, "cc mdio register base map error\n");
		ret = -ENXIO;
		goto err_ctrl_free;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"iproc-mdio-enable");
	if (res) {
		cc_ctrl->iproc_mdio_enable_reg =
				devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(cc_ctrl->iproc_mdio_enable_reg)) {
			ret = PTR_ERR(cc_ctrl->iproc_mdio_enable_reg);
			goto err_ctrl_free;
		}
	}

	if (cc_ctrl->iproc_mdio_enable_reg) {
		if (of_property_read_u32(dn, "iproc-mdio-sel-bit",
			&cc_ctrl->iproc_mdio_sel_bit)) {
			dev_err(&pdev->dev, "No mdio bus select bit!\n");
			ret = -EINVAL;
			goto err_ctrl_free;
		}
	}

hw_ctrl_allocated:
	cc_ctrl = cc_mdio_ctrl;

	clk = of_clk_get(dn, 0);
	if (clk) {
		mdio_clk_rate = clk_get_rate(clk) / 2;
	} else {
		dev_warn(&pdev->dev, "No CC MDIO clock available in DT, \
			use default clock rate: 50MHz\n");
		mdio_clk_rate = 50000000;
	}

	cc_mii_init(cc_ctrl, mdio_clk_rate);

	/* If no property available, use default: "internal" */
	if (of_property_read_string(dn, "bus-type", &mdio_bus_type))
		mdio_bus_type = "internal";

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

	if (!strcmp(mdio_bus_type, "internal"))
		bus_data->phybus_type = IPROC_MDIOBUS_TYPE_INTERNAL;
	else
		bus_data->phybus_type = IPROC_MDIOBUS_TYPE_EXTERNAL;

	bus_priv->bus_data = bus_data;
	bus_priv->hw_ctrl = cc_ctrl;

	mii_bus = mdiobus_alloc();
	if (!mii_bus) {
		dev_err(&pdev->dev, "mdiobus alloc failed\n");
		ret = -ENOMEM;
		goto err_ctrl_free;
	}

	mii_bus->name = "iproc_cc_mdiobus";
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s-%s", "cc mdio",
		bus_data->phybus_type? "external":"internal");
	mii_bus->parent = &pdev->dev;
	mii_bus->read = cc_mdiobus_read;
	mii_bus->write = cc_mdiobus_write;
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
	cc_mdio_res_free(cc_ctrl);
err_exit:
	return ret;
}

static int cc_mdiobus_remove(struct platform_device *pdev)
{
	struct mii_bus *mii_bus = platform_get_drvdata(pdev);
	struct iproc_mdiobus_private *bus_priv;

	if (mii_bus) {
		bus_priv = mii_bus->priv;

		mdiobus_unregister(mii_bus);
		if (bus_priv)
			cc_mdio_res_free(bus_priv->hw_ctrl);
		mdiobus_free(mii_bus);
	}

	return 0;
}

static const struct of_device_id cc_mdio_dt_ids[] = {
	{ .compatible = "brcm,iproc-ccb-mdio"},
	{ .compatible = "brcm,iproc-ccg-mdio"},
	{  }
};
MODULE_DEVICE_TABLE(of, cc_mdio_dt_ids);


static struct platform_driver iproc_cc_mdiobus_driver =
{
	.driver = {
		.name = "iproc_cc_mdio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cc_mdio_dt_ids),
	},
	.probe   = cc_mdiobus_probe,
	.remove  = cc_mdiobus_remove,
};

static int __init cc_mdio_init(void)
{
	return platform_driver_register(&iproc_cc_mdiobus_driver);
}

static void __exit cc_mdio_exit(void)
{
	platform_driver_unregister(&iproc_cc_mdiobus_driver);
}

//module_init(cc_mdio_init);
subsys_initcall(cc_mdio_init);
module_exit(cc_mdio_exit);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("iProc CC mdio driver");
MODULE_LICENSE("GPL");
