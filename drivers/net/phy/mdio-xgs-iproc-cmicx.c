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

#define MIIM_CHAN_NUM								(3)

#define MIIM_CH0_CONTROL_REG						0x000
#define MIIM_CH1_CONTROL_REG						0x010
#define MIIM_CH2_CONTROL_REG						0x020
#define MIIM_CH3_CONTROL_REG						0x030
#define  MIIM_CONTROL__START_SHIFT					0
#define  MIIM_CONTROL__START_WIDTH					1
#define  MIIM_CONTROL__START_MASK					((1 << MIIM_CONTROL__START_WIDTH) - 1)
#define MIIM_CH0_PARAMS_REG							0x004
#define MIIM_CH1_PARAMS_REG							0x014
#define MIIM_CH2_PARAMS_REG							0x024
#define MIIM_CH3_PARAMS_REG							0x034
#define  MIIM_PARAMS__RING_MAP_SHIFT				20
#define  MIIM_PARAMS__RING_MAP_WIDTH				12
#define  MIIM_PARAMS__RING_MAP_MASK					((1 << MIIM_PARAMS__RING_MAP_WIDTH) - 1)
#define  MIIM_PARAMS__MDIO_OP_TYPE_SHIFT			17
#define  MIIM_PARAMS__MDIO_OP_TYPE_WIDTH			3
#define  MIIM_PARAMS__MDIO_OP_TYPE_MASK				((1 << MIIM_PARAMS__MDIO_OP_TYPE_WIDTH) - 1)
#define   CLAUSE_22_WRITE_OP_MODE					0x0
#define   CLAUSE_22_READ_OP_MODE					0x1
#define   CLAUSE_45_WRITE_OP_MODE					0x5
#define   CLAUSE_45_READ_OP_MODE					0x6
#define  MIIM_PARAMS__SEL_INT_PHY_SHIFT				16
#define  MIIM_PARAMS__SEL_INT_PHY_WIDTH				1
#define  MIIM_PARAMS__SEL_INT_PHY_MASK				((1 << MIIM_PARAMS__SEL_INT_PHY_WIDTH) - 1)
#define  MIIM_PARAMS__PHY_WR_DATA_SHIFT				0
#define  MIIM_PARAMS__PHY_WR_DATA_WIDTH				16
#define  MIIM_PARAMS__PHY_WR_DATA_MASK				((1 << MIIM_PARAMS__PHY_WR_DATA_WIDTH) - 1)
#define MIIM_CH0_ADDRESS_REG						0x008
#define MIIM_CH1_ADDRESS_REG						0x018
#define MIIM_CH2_ADDRESS_REG						0x028
#define MIIM_CH3_ADDRESS_REG						0x038
#define  MIIM_ADDRESS__C45_REGADDR_SHIFT			16
#define  MIIM_ADDRESS__C45_REGADDR_WIDTH			16
#define  MIIM_ADDRESS__C45_REGADDR_MASK				((1 << MIIM_ADDRESS__C45_REGADDR_WIDTH) - 1)
#define  MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_SHIFT	11
#define  MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_WIDTH	5
#define  MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_MASK	((1 << MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_WIDTH) - 1)
#define  MIIM_ADDRESS__PHY_ID_SHIFT					6
#define  MIIM_ADDRESS__PHY_ID_WIDTH					5
#define  MIIM_ADDRESS__PHY_ID_MASK					((1 << MIIM_ADDRESS__PHY_ID_WIDTH) - 1)
#define MIIM_CH0_STATUS_REG							0x00c
#define MIIM_CH1_STATUS_REG							0x01c
#define MIIM_CH2_STATUS_REG							0x02c
#define MIIM_CH3_STATUS_REG							0x03c
#define  MIIM_STATUS__DONE_SHIFT					18
#define  MIIM_STATUS__DONE_WIDTH					1
#define  MIIM_STATUS__DONE_MASK						((1 << MIIM_STATUS__DONE_WIDTH) - 1)
#define  MIIM_STATUS__ERROR_SHIFT					17
#define  MIIM_STATUS__ERROR_WIDTH					1
#define  MIIM_STATUS__ERROR_MASK					((1 << MIIM_STATUS__ERROR_WIDTH) - 1)
#define  MIIM_STATUS__ACTIVE_SHIFT					16
#define  MIIM_STATUS__ACTIVE_WIDTH					1
#define  MIIM_STATUS__ACTIVE_MASK					((1 << MIIM_STATUS__ACTIVE_WIDTH) - 1)
#define  MIIM_STATUS__PHY_RD_DATA_SHIFT				0
#define  MIIM_STATUS__PHY_RD_DATA_WIDTH				16
#define  MIIM_STATUS__PHY_RD_DATA_MASK				((1 << MIIM_STATUS__PHY_RD_DATA_WIDTH) - 1)

#define MIIM_CHn_CONTROL_REG(n)						(MIIM_CH0_CONTROL_REG | (n) << 4)
#define MIIM_CHn_PARAMS_REG(n)						(MIIM_CH0_PARAMS_REG | (n) << 4)
#define MIIM_CHn_ADDRESS_REG(n)						(MIIM_CH0_ADDRESS_REG | (n) << 4)
#define MIIM_CHn_STATUS_REG(n)						(MIIM_CH0_STATUS_REG | (n) << 4)

#define MIIM_RING0_CONTROL_REG						0x0f0
#define MIIM_RING1_CONTROL_REG						0x0f4
#define MIIM_RING2_CONTROL_REG						0x0f8
#define MIIM_RING3_CONTROL_REG						0x0fc
#define MIIM_RING4_CONTROL_REG						0x100
#define MIIM_RING5_CONTROL_REG						0x104
#define MIIM_RING6_CONTROL_REG						0x108
#define MIIM_RING7_CONTROL_REG						0x10c
#define MIIM_RING8_CONTROL_REG						0x110
#define MIIM_RING9_CONTROL_REG						0x114
#define MIIM_RING10_CONTROL_REG						0x118
#define MIIM_RING11_CONTROL_REG						0x11c
#define  MIIM_RING_CTRL__MDC_MODE_SHIFT				26
#define  MIIM_RING_CTRL__MDC_MODE_WIDTH				1
#define  MIIM_RING_CTRL__MDC_MODE_MASK				((1 << MIIM_RING_CTRL__MDC_MODE_WIDTH) - 1)
#define  MIIM_RING_CTRL__PREAMBLE_SHIFT				24
#define  MIIM_RING_CTRL__PREAMBLE_WIDTH				2
#define  MIIM_RING_CTRL__PREAMBLE_MASK				((1 << MIIM_RING_CTRL__PREAMBLE_WIDTH) - 1)
#define  MIIM_RING_CTRL__MDIO_OUT_DELAY_SHIFT		16
#define  MIIM_RING_CTRL__MDIO_OUT_DELAY_WIDTH		8
#define  MIIM_RING_CTRL__MDIO_OUT_DELAY_MASK		((1 << MIIM_RING_CTRL__MDIO_OUT_DELAY_WIDTH) - 1)
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_SHIFT	8
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_WIDTH	8
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_MASK		((1 << MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_WIDTH) - 1)
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_INT_SHIFT	0
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_INT_WIDTH	8
#define  MIIM_RING_CTRL__CLOCK_DIVIDER_INT_MASK		((1 << MIIM_RING_CTRL__CLOCK_DIVIDER_INT_WIDTH) - 1)
#define MIIM_COMMON_CONTROL_REG						0x140
#define  MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_SHIFT	0
#define  MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_WIDTH	1
#define  MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_MASK	((1 << MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_WIDTH) - 1)

#define MIIM_MAX_RINGS				12

struct cmicx_miim_cmd {
	u32 bus_id;
	u32 int_sel;
	u32 phy_id;
	u32 reg_num;
	u32 c45_sel;
	u16 op_mode;
	u16 phy_data;
};

static inline u32 cmicx_miim_read(struct iproc_mdio_ctrl *mdio_ctrl, u32 reg)
{
	return readl(mdio_ctrl->base + reg);
}

static inline void cmicx_miim_write(struct iproc_mdio_ctrl *mdio_ctrl,
				u32 reg, u32 data)
{
	writel(data, mdio_ctrl->base + reg);
}

static int cmicx_miim_init(struct iproc_mdio_ctrl *mdio_ctrl)
{
	u32 val;
	u32 mstr_ctrl;

	/* Give MDIO control to IPROC */
	val = cmicx_miim_read(mdio_ctrl, MIIM_COMMON_CONTROL_REG);
	mstr_ctrl = GET_REG_FIELD(val, MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_SHIFT,
						MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_MASK);
	if (!mstr_ctrl) {
		ISET_REG_FIELD(val, MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_SHIFT,
				MIIM_COMM_CTRL__EXT_MDIO_MSTR_CNTRL_MASK, 1);
		cmicx_miim_write(mdio_ctrl, MIIM_COMMON_CONTROL_REG, val);
	}
	return 0;
}

static int cmicx_miim_ring_init(struct iproc_mdio_ctrl *mdio_ctrl, u32 ring_idx,
					int int_divider, int ext_divider, int out_delay)
{
    u32 ring_ctrl_reg[] = { MIIM_RING0_CONTROL_REG,
                            MIIM_RING1_CONTROL_REG,
                            MIIM_RING2_CONTROL_REG,
                            MIIM_RING3_CONTROL_REG,
                            MIIM_RING4_CONTROL_REG,
                            MIIM_RING5_CONTROL_REG,
                            MIIM_RING6_CONTROL_REG,
                            MIIM_RING7_CONTROL_REG,
							MIIM_RING8_CONTROL_REG,
							MIIM_RING9_CONTROL_REG,
							MIIM_RING10_CONTROL_REG,
							MIIM_RING11_CONTROL_REG };
	u32 val;

	if (ring_idx >= MIIM_MAX_RINGS) {
		return -EINVAL;
	}

	val = cmicx_miim_read(mdio_ctrl, ring_ctrl_reg[ring_idx]);
	if (int_divider != -1) {
		ISET_REG_FIELD(val, MIIM_RING_CTRL__CLOCK_DIVIDER_INT_SHIFT,
				MIIM_RING_CTRL__CLOCK_DIVIDER_INT_MASK, int_divider);
	}
    if (ext_divider != -1) {
		ISET_REG_FIELD(val, MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_SHIFT,
				MIIM_RING_CTRL__CLOCK_DIVIDER_EXT_MASK, ext_divider);
    }
    if (out_delay != -1) {
		ISET_REG_FIELD(val, MIIM_RING_CTRL__MDIO_OUT_DELAY_SHIFT,
				MIIM_RING_CTRL__MDIO_OUT_DELAY_MASK, out_delay);
    }
	cmicx_miim_write(mdio_ctrl, ring_ctrl_reg[ring_idx], val);

    return 0;
}

static int cmicx_miim_operation(struct iproc_mdio_ctrl *mdio_ctrl,
						struct cmicx_miim_cmd *miim_cmd)
{
	unsigned long flags;
	u32 is_done, is_error;
	u32 optype = 0;
	u32 val;
	int usec = MII_OP_MAX_HALT_USEC;
	int ret = 0;

	spin_lock_irqsave(&mdio_ctrl->lock, flags);

	/* prepare transaction data */
	val = 0;
	ISET_REG_FIELD(val, MIIM_ADDRESS__PHY_ID_SHIFT,
			MIIM_ADDRESS__PHY_ID_MASK, miim_cmd->phy_id);
	if (miim_cmd->c45_sel) {
		ISET_REG_FIELD(val, MIIM_ADDRESS__C45_REGADDR_SHIFT,
				MIIM_ADDRESS__C45_REGADDR_MASK, miim_cmd->reg_num);
		ISET_REG_FIELD(val, MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_SHIFT,
				MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_MASK, (miim_cmd->reg_num >> 16));
    } else {
		ISET_REG_FIELD(val, MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_SHIFT,
				MIIM_ADDRESS__C22_REGADDR_C45_DTYPE_MASK, miim_cmd->reg_num);
	}
	cmicx_miim_write(mdio_ctrl, MIIM_CHn_ADDRESS_REG(MIIM_CHAN_NUM), val);

	val = 0;
	ISET_REG_FIELD(val, MIIM_PARAMS__PHY_WR_DATA_SHIFT,
			MIIM_PARAMS__PHY_WR_DATA_MASK, miim_cmd->phy_data);
	ISET_REG_FIELD(val, MIIM_PARAMS__SEL_INT_PHY_SHIFT,
			MIIM_PARAMS__SEL_INT_PHY_MASK, miim_cmd->int_sel);
	ISET_REG_FIELD(val, MIIM_PARAMS__RING_MAP_SHIFT,
			MIIM_PARAMS__RING_MAP_MASK, (1 << miim_cmd->bus_id));
	if (miim_cmd->c45_sel) {
		optype = CLAUSE_45_READ_OP_MODE;
		if (miim_cmd->op_mode == MII_OP_MODE_WRITE) {
			optype = CLAUSE_45_WRITE_OP_MODE;
		}
	} else {
		optype = CLAUSE_22_READ_OP_MODE;
		if (miim_cmd->op_mode == MII_OP_MODE_WRITE) {
			optype = CLAUSE_22_WRITE_OP_MODE;
		}
	}
	ISET_REG_FIELD(val, MIIM_PARAMS__MDIO_OP_TYPE_SHIFT,
			MIIM_PARAMS__MDIO_OP_TYPE_MASK, optype);
	cmicx_miim_write(mdio_ctrl, MIIM_CHn_PARAMS_REG(MIIM_CHAN_NUM), val);

	/* start transaction */
	val = 0;
	ISET_REG_FIELD(val, MIIM_CONTROL__START_SHIFT,
			MIIM_CONTROL__START_MASK, 0x1);
	cmicx_miim_write(mdio_ctrl, MIIM_CHn_CONTROL_REG(MIIM_CHAN_NUM), val);

	/* poll for DONE bit */
	do {
		val = cmicx_miim_read(mdio_ctrl, MIIM_CHn_STATUS_REG(MIIM_CHAN_NUM));
		is_done = GET_REG_FIELD(val, MIIM_STATUS__DONE_SHIFT, MIIM_STATUS__DONE_MASK);
		if (is_done) {
			break; /* MIIM operation is done */
		}
		udelay(1);
	} while (usec-- > 0);

	/* check for transaction error */
	val = cmicx_miim_read(mdio_ctrl, MIIM_CHn_STATUS_REG(MIIM_CHAN_NUM));
	is_error = GET_REG_FIELD(val, MIIM_STATUS__ERROR_SHIFT, MIIM_STATUS__ERROR_MASK);
	if (is_error) {
		printk(KERN_ERR "%s : mdio execution error.\n", __func__);
		ret = -EIO;
		goto exit;
	}

	/* in case of read - get data */
	if (miim_cmd->op_mode == MII_OP_MODE_READ) {
		miim_cmd->phy_data = GET_REG_FIELD(val, MIIM_STATUS__PHY_RD_DATA_SHIFT,
										MIIM_STATUS__PHY_RD_DATA_MASK);
    }

exit:
	/* cleanup */
	cmicx_miim_write(mdio_ctrl, MIIM_CHn_CONTROL_REG(MIIM_CHAN_NUM), 0);
	spin_unlock_irqrestore(&mdio_ctrl->lock, flags);

	return ret;
}

static int cmicx_mdiobus_read(struct mii_bus *bus, int phy_id, int reg_num)
{
	struct iproc_mdiobus_private *mdio_bus_priv = bus->priv;
	struct iproc_mdiobus_data *mdio_bus_data = mdio_bus_priv->bus_data;
	struct cmicx_miim_cmd miim_cmd = { 0 };
	int ret = 0;

	if (IPROC_MDIOBUS_TYPE_INTERNAL == mdio_bus_data->phybus_type) {
		miim_cmd.int_sel = 1;
	}
	if (reg_num & MII_ADDR_C45) {
		miim_cmd.c45_sel = 1;
	}
	miim_cmd.bus_id = mdio_bus_data->phybus_num;
	miim_cmd.phy_id = phy_id;
	miim_cmd.reg_num = reg_num;
	miim_cmd.op_mode = MII_OP_MODE_READ;
	miim_cmd.phy_data = 0;

	ret = cmicx_miim_operation(mdio_bus_priv->hw_ctrl, &miim_cmd);
	if (ret == 0) {
		return miim_cmd.phy_data;
	}
	return ret;
}

static int cmicx_mdiobus_write(struct mii_bus *bus, int phy_id,
				int reg_num, u16 val)
{
	struct iproc_mdiobus_private *mdio_bus_priv = bus->priv;
	struct iproc_mdiobus_data *mdio_bus_data = mdio_bus_priv->bus_data;
	struct cmicx_miim_cmd miim_cmd = {0};

	if (IPROC_MDIOBUS_TYPE_INTERNAL == mdio_bus_data->phybus_type) {
		miim_cmd.int_sel = 1;
	}
	if (reg_num & MII_ADDR_C45) {
		miim_cmd.c45_sel = 1;
	}
	miim_cmd.bus_id = mdio_bus_data->phybus_num;
	miim_cmd.phy_id = phy_id;
	miim_cmd.reg_num = reg_num;
	miim_cmd.op_mode = MII_OP_MODE_WRITE;
	miim_cmd.phy_data = val;

	return cmicx_miim_operation(mdio_bus_priv->hw_ctrl, &miim_cmd);
}

/*************************************************************************************
**************************************************************************************/
static int cmicx_mdiobus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = pdev->dev.of_node;
	struct mii_bus *mii_bus = NULL;
	struct iproc_mdiobus_private *mdio_bus_priv = NULL;
	struct iproc_mdiobus_data *mdio_bus_data = NULL;
	struct iproc_mdio_ctrl *mdio_ctrl = NULL;
	u32 mdio_bus_id;
	int clock_divider, out_delay;
	int divider_int = -1, divider_ext = -1;
	const char *mdio_bus_type;
	int ret;

	mdio_ctrl = devm_kzalloc(dev, sizeof(*mdio_ctrl), GFP_KERNEL);
	if (!mdio_ctrl) {
		dev_err(dev, "cmicx mdio resource allocated failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&mdio_ctrl->lock);

	/* Get register base address */
	mdio_ctrl->base = of_iomap(dn, 0);
	if (!mdio_ctrl->base) {
		dev_err(dev, "cmicx mdio register base map error\n");
		ret = -ENXIO;
		goto err;
	}

	/* If no property available, use default: 2 */
	if (of_property_read_u32(dn, "#bus-id", &mdio_bus_id)) {
		mdio_bus_id = 2;
	}

	/* If no property available, use default: "external" */
	if (of_property_read_string(dn, "bus-type", &mdio_bus_type)) {
		mdio_bus_type = "external";
	}

	/* If no property available, use default: -1 */
	if (of_property_read_u32(dn, "#divider", &clock_divider)) {
		clock_divider = -1;
	}

	/* If no property available, use default: -1 */
	if (of_property_read_u32(dn, "#delay", &out_delay)) {
		out_delay = -1;
	}

	mdio_bus_data = devm_kzalloc(dev, sizeof(*mdio_bus_data), GFP_KERNEL);
	if (!mdio_bus_data) {
		dev_err(dev, "iProc MDIO bus data allocated failed\n");
		ret = -ENOMEM;
		goto err;
	}

	mdio_bus_data->phybus_num = mdio_bus_id;
	if (!strcmp(mdio_bus_type, "internal")) {
		mdio_bus_data->phybus_type = IPROC_MDIOBUS_TYPE_INTERNAL;
		divider_int = clock_divider;
	} else {
		mdio_bus_data->phybus_type = IPROC_MDIOBUS_TYPE_EXTERNAL;
		divider_ext = clock_divider;
	}

	mdio_bus_priv = devm_kzalloc(dev, sizeof(*mdio_bus_priv), GFP_KERNEL);
	if (!mdio_bus_priv) {
		dev_err(dev, "iProc MDIO private data allocated failed\n");
		ret = -ENOMEM;
		goto err;
	}

	mdio_bus_priv->bus_data = mdio_bus_data;
	mdio_bus_priv->hw_ctrl = mdio_ctrl;

	ret = cmicx_miim_init(mdio_ctrl);
	if (ret) {
		dev_err(dev, "cmicx init failed\n");
		goto err;
	}

	ret = cmicx_miim_ring_init(mdio_ctrl, mdio_bus_id,
							divider_int, divider_ext, out_delay);
	if (ret) {
		dev_err(dev, "cmicx init ring failed\n");
		goto err;
	}

	mii_bus = mdiobus_alloc();
	if (!mii_bus) {
		dev_err(dev, "MII bus memory allocated failed\n");
		ret = -ENOMEM;
		goto err;
	}

	mii_bus->name = "iproc_cmicx_mdiobus";
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s-%d-%d", "cmicx mdio", mdio_bus_id,
				(mdio_bus_data->phybus_type == IPROC_MDIOBUS_TYPE_EXTERNAL) ? 1 : 0);
	mii_bus->parent = dev;
	mii_bus->read = cmicx_mdiobus_read;
	mii_bus->write = cmicx_mdiobus_write;
	mii_bus->priv = mdio_bus_priv;

	ret = of_mdiobus_register(mii_bus, dn);
	if (ret) {
		dev_err(dev, "mdiobus register failed\n");
		goto err;
	}

	platform_set_drvdata(pdev, mii_bus);

	return 0;

err:
	if (mii_bus) {
		mdiobus_free(mii_bus);
	}
	if (mdio_bus_priv) {
		devm_kfree(dev, mdio_bus_priv);
	}
	if (mdio_bus_data) {
		devm_kfree(dev, mdio_bus_data);
	}
	if (mdio_ctrl->base) {
		iounmap(mdio_ctrl->base);
	}
	if (mdio_ctrl) {
		devm_kfree(dev, mdio_ctrl);
	}

	return ret;
}

static int cmicx_mdiobus_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mii_bus *mii_bus = platform_get_drvdata(pdev);
	struct iproc_mdiobus_private *mdio_bus_priv;
	struct iproc_mdiobus_data *mdio_bus_data;
	struct iproc_mdio_ctrl *mdio_ctrl;

	if (mii_bus) {
		mdio_bus_priv = mii_bus->priv;
		mdio_bus_data = mdio_bus_priv->bus_data;
		mdio_ctrl = mdio_bus_priv->hw_ctrl;

		mdiobus_unregister(mii_bus);
		mdiobus_free(mii_bus);

		if (mdio_bus_priv) {
			if (mdio_ctrl) {
				if (mdio_ctrl->base) {
					iounmap(mdio_ctrl->base);
				}
				devm_kfree(dev, mdio_ctrl);
			}
			if (mdio_bus_data) {
				devm_kfree(dev, mdio_bus_data);
			}
			devm_kfree(dev, mdio_bus_priv);
		}
	}

	return 0;
}

static const struct of_device_id cmicx_mdio_dt_ids[] = {
	{ .compatible = "brcm,iproc-cmicx-mdio"},
	{  }
};
MODULE_DEVICE_TABLE(of, cmicx_mdio_dt_ids);

static struct platform_driver iproc_cmicx_mdiobus_driver =
{
	.driver = {
		.name = "iproc_cmicx_mdio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cmicx_mdio_dt_ids),
	},
	.probe   = cmicx_mdiobus_probe,
	.remove  = cmicx_mdiobus_remove,
};

static int __init cmicx_mdio_init(void)
{
	return platform_driver_register(&iproc_cmicx_mdiobus_driver);
}

static void __exit cmicx_mdio_exit(void)
{
	platform_driver_unregister(&iproc_cmicx_mdiobus_driver);
}

subsys_initcall(cmicx_mdio_init);
module_exit(cmicx_mdio_exit);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("iProc CMICx mdio driver");
MODULE_LICENSE("GPL");
