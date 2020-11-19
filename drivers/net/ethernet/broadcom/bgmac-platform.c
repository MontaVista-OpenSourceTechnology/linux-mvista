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

#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/bcma/bcma.h>
#include <linux/brcmphy.h>
#include <linux/etherdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>

#if IS_ENABLED(CONFIG_ARCH_XGS_IPROC)
#include <linux/soc/bcm/xgs-iproc-misc-setup.h>
#include <linux/phy/xgs_iproc_serdes.h>
#endif

#include "bgmac.h"

#define NICPM_PADRING_CFG		0x00000004
#define NICPM_IOMUX_CTRL		0x00000008

#define NICPM_PADRING_CFG_INIT_VAL	0x74000000
#define NICPM_IOMUX_CTRL_INIT_VAL_AX	0x21880000

#define NICPM_IOMUX_CTRL_INIT_VAL	0x3196e000
#define NICPM_IOMUX_CTRL_SPD_SHIFT	10
#define NICPM_IOMUX_CTRL_SPD_10M	0
#define NICPM_IOMUX_CTRL_SPD_100M	1
#define NICPM_IOMUX_CTRL_SPD_1000M	2

static u32 platform_bgmac_read(struct bgmac *bgmac, u16 offset)
{
	return readl(bgmac->plat.base + offset);
}

static void platform_bgmac_write(struct bgmac *bgmac, u16 offset, u32 value)
{
	writel(value, bgmac->plat.base + offset);
}

static u32 platform_bgmac_idm_read(struct bgmac *bgmac, u16 offset)
{
	return readl(bgmac->plat.idm_base + offset);
}

static void platform_bgmac_idm_write(struct bgmac *bgmac, u16 offset, u32 value)
{
	writel(value, bgmac->plat.idm_base + offset);
}

static bool platform_bgmac_clk_enabled(struct bgmac *bgmac)
{
	if (!bgmac->plat.idm_base)
		return true;

	if ((bgmac_idm_read(bgmac, BCMA_IOCTL) & BGMAC_CLK_EN) != BGMAC_CLK_EN)
		return false;
	if (bgmac_idm_read(bgmac, BCMA_RESET_CTL) & BCMA_RESET_CTL_RESET)
		return false;
	return true;
}

static void platform_bgmac_clk_enable(struct bgmac *bgmac, u32 flags)
{
	u32 val;

	if (!bgmac->plat.idm_base)
		return;

	/* The Reset Control register only contains a single bit to show if the
	 * controller is currently in reset.  Do a sanity check here, just in
	 * case the bootloader happened to leave the device in reset.
	 */
	val = bgmac_idm_read(bgmac, BCMA_RESET_CTL);
	if (val) {
		bgmac_idm_write(bgmac, BCMA_RESET_CTL, 0);
		bgmac_idm_read(bgmac, BCMA_RESET_CTL);
		udelay(1);
	}

	val = bgmac_idm_read(bgmac, BCMA_IOCTL);
#if IS_ENABLED(CONFIG_ARCH_XGS_IPROC)
	/* To make HX4/KT2 GMAC work */
	val |= flags;
	val &= ~(BGMAC_AWCACHE | BGMAC_ARCACHE | BGMAC_AWUSER |
			 BGMAC_ARUSER);
#else
	/* Some bits of BCMA_IOCTL set by HW/ATF and should not change */
	val |= flags & ~(BGMAC_AWCACHE | BGMAC_ARCACHE | BGMAC_AWUSER |
			 BGMAC_ARUSER);
#endif
	val |= BGMAC_CLK_EN;
	bgmac_idm_write(bgmac, BCMA_IOCTL, val);
	bgmac_idm_read(bgmac, BCMA_IOCTL);
	udelay(1);
}

static void platform_bgmac_cco_ctl_maskset(struct bgmac *bgmac, u32 offset,
					   u32 mask, u32 set)
{
	/* This shouldn't be encountered */
	WARN_ON(1);
}

static u32 platform_bgmac_get_bus_clock(struct bgmac *bgmac)
{
	/* This shouldn't be encountered */
	WARN_ON(1);

	return 0;
}

static void platform_bgmac_cmn_maskset32(struct bgmac *bgmac, u16 offset,
					 u32 mask, u32 set)
{
	/* This shouldn't be encountered */
	WARN_ON(1);
}

static void bgmac_nicpm_speed_set(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	u32 val;

	if (!bgmac->plat.nicpm_base)
		return;

	/* SET RGMII IO CONFIG */
	writel(NICPM_PADRING_CFG_INIT_VAL,
	       bgmac->plat.nicpm_base + NICPM_PADRING_CFG);

	val = NICPM_IOMUX_CTRL_INIT_VAL;
	switch (bgmac->net_dev->phydev->speed) {
	default:
		netdev_err(net_dev, "Unsupported speed. Defaulting to 1000Mb\n");
	case SPEED_1000:
		val |= NICPM_IOMUX_CTRL_SPD_1000M << NICPM_IOMUX_CTRL_SPD_SHIFT;
		break;
	case SPEED_100:
		val |= NICPM_IOMUX_CTRL_SPD_100M << NICPM_IOMUX_CTRL_SPD_SHIFT;
		break;
	case SPEED_10:
		val |= NICPM_IOMUX_CTRL_SPD_10M << NICPM_IOMUX_CTRL_SPD_SHIFT;
		break;
	}

	writel(val, bgmac->plat.nicpm_base + NICPM_IOMUX_CTRL);

	bgmac_adjust_link(bgmac->net_dev);
}

#if IS_ENABLED(CONFIG_ARCH_XGS_IPROC)
#define SERDES_CONTROL_OFFSET		0x1a8
#define SC_TX1G_FIFO_RST_MASK		0x00f00000
#define SC_TX1G_FIFO_RST_VAL		0x00f00000
#define SC_FORCE_SPD_STRAP_MASK 	0x00060000
#define SC_FORCE_SPD_STRAP_VAL		0x00040000
#define SC_FORCE_SPD_100M_VAL		0x00020000
#define SC_FORCE_SPD_1G_VAL		0x00040000
#define SC_REF_TERM_SEL_MASK		0x00001000
#define SC_REFSEL_MASK			0x00000c00
#define SC_REFSEL_VAL			0x00000400
#define SC_REFDIV_MASK			0x00000300
#define SC_REFDIV_VAL			0x00000000
#define SC_LCREF_EN_MASK		0x00000040
#define SC_RSTB_PLL_MASK		0x00000010
#define SC_RSTB_MDIOREGS_MASK		0x00000008
#define SC_RSTB_HW_MASK 		0x00000004
#define SC_IDDQ_MASK			0x00000002
#define SC_PWR_DOWN_MASK		0x00000001

void amac_serdes_init(struct bgmac *info, struct phy_device *phy_dev)
{
	u32 sdctl;
	void *serdes_ctl_reg;
	struct phy_device *phydev = phy_dev;

	serdes_ctl_reg = info->plat.base + SERDES_CONTROL_OFFSET;

	sdctl = (SC_TX1G_FIFO_RST_VAL | SC_FORCE_SPD_STRAP_VAL);
	if (xgs_serdes_hx4_amac(phydev))
		sdctl |= (SC_REFSEL_VAL | SC_REF_TERM_SEL_MASK);

	else if (xgs_serdes_kt2_amac(phydev))
		sdctl |= SC_REF_TERM_SEL_MASK;

	writel(sdctl, serdes_ctl_reg);

	udelay(1000);

	sdctl = readl(serdes_ctl_reg);
	sdctl |= (SC_IDDQ_MASK | SC_PWR_DOWN_MASK);
	writel(sdctl, serdes_ctl_reg);

	sdctl = readl(serdes_ctl_reg);
	sdctl &= ~(SC_IDDQ_MASK | SC_PWR_DOWN_MASK);
	writel(sdctl, serdes_ctl_reg);

	/* Bring hardware out of reset */
	sdctl = readl(serdes_ctl_reg);
	sdctl |= SC_RSTB_HW_MASK;
	writel(sdctl, serdes_ctl_reg);

	/* Bring MDIOREGS out of reset */
	sdctl = readl(serdes_ctl_reg);
	sdctl |= SC_RSTB_MDIOREGS_MASK;
	writel(sdctl, serdes_ctl_reg);

	udelay(1000);

	/* Bring PLL out of reset */
	sdctl = readl(serdes_ctl_reg);
	sdctl |= SC_RSTB_PLL_MASK;
	writel(sdctl, serdes_ctl_reg);

	udelay(1000);

	return;
}
#endif /* IS_ENABLED(CONFIG_ARCH_XGS_IPROC) */

static int platform_phy_connect(struct bgmac *bgmac)
{
	struct phy_device *phy_dev;

	if (bgmac->plat.nicpm_base) {
		phy_dev = of_phy_get_and_connect(bgmac->net_dev,
						 bgmac->dev->of_node,
						 bgmac_nicpm_speed_set);
	}
	else {
		struct device_node *np = bgmac->dev->of_node;
		u32 lane;

		/* For WH2 SGMII case, treat SERDES as PHY */
		if (of_device_is_compatible(np, "brcm,xgs-wh2-amac") &&
			is_wh2_amac_sgmii()) {
			struct device_node *phy_np;
			phy_interface_t iface = PHY_INTERFACE_MODE_SGMII;

			phy_np = of_parse_phandle(np, "serdes-phy-handle", 0);
			if (!phy_np)
				return -ENODEV;

			phy_dev = of_phy_find_device(phy_np);
			if (!phy_dev)
				return -ENODEV;

			/* Get lane from DT, otherwise set to default 3 */
			if (of_property_read_u32(phy_np, "lane-num", &lane))
				lane = 3;
			xgs_serdes_set_lane(phy_dev, lane);

			amac_serdes_init(bgmac, phy_dev);

			phy_connect_direct(bgmac->net_dev, phy_dev,
						bgmac_adjust_link, iface);

			put_device(&phy_dev->mdio.dev);
			of_node_put(phy_np);
		}
		else {
		phy_dev = of_phy_get_and_connect(bgmac->net_dev,
						 bgmac->dev->of_node,
						 bgmac_adjust_link);
		}
	}

	if (!phy_dev) {
		dev_err(bgmac->dev, "PHY connection failed\n");
		return -ENODEV;
	}

	return 0;
}

static int bgmac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bgmac *bgmac;
	struct resource *regs;
	const u8 *mac_addr;

	bgmac = bgmac_alloc(&pdev->dev);
	if (!bgmac)
		return -ENOMEM;

	platform_set_drvdata(pdev, bgmac);

	/* Set the features of the 4707 family */
	bgmac->feature_flags |= BGMAC_FEAT_CLKCTLST;
	bgmac->feature_flags |= BGMAC_FEAT_NO_RESET;
	bgmac->feature_flags |= BGMAC_FEAT_CMDCFG_SR_REV4;
	bgmac->feature_flags |= BGMAC_FEAT_TX_MASK_SETUP;
	bgmac->feature_flags |= BGMAC_FEAT_RX_MASK_SETUP;
	bgmac->feature_flags |= BGMAC_FEAT_IDM_MASK;

	bgmac->dev = &pdev->dev;
	bgmac->dma_dev = &pdev->dev;

	mac_addr = of_get_mac_address(np);
	if (mac_addr)
		ether_addr_copy(bgmac->net_dev->dev_addr, mac_addr);
	else
		dev_warn(&pdev->dev, "MAC address not present in device tree\n");

	bgmac->irq = platform_get_irq(pdev, 0);
	if (bgmac->irq < 0) {
		dev_err(&pdev->dev, "Unable to obtain IRQ\n");
		return bgmac->irq;
	}

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "amac_base");
	if (!regs) {
		dev_err(&pdev->dev, "Unable to obtain base resource\n");
		return -EINVAL;
	}

	bgmac->plat.base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(bgmac->plat.base))
		return PTR_ERR(bgmac->plat.base);

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "idm_base");
	if (regs) {
		bgmac->plat.idm_base = devm_ioremap_resource(&pdev->dev, regs);
		if (IS_ERR(bgmac->plat.idm_base))
			return PTR_ERR(bgmac->plat.idm_base);
		bgmac->feature_flags &= ~BGMAC_FEAT_IDM_MASK;
	}

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nicpm_base");
	if (regs) {
		bgmac->plat.nicpm_base = devm_ioremap_resource(&pdev->dev,
							       regs);
		if (IS_ERR(bgmac->plat.nicpm_base))
			return PTR_ERR(bgmac->plat.nicpm_base);
	}

	bgmac->read = platform_bgmac_read;
	bgmac->write = platform_bgmac_write;
	bgmac->idm_read = platform_bgmac_idm_read;
	bgmac->idm_write = platform_bgmac_idm_write;
	bgmac->clk_enabled = platform_bgmac_clk_enabled;
	bgmac->clk_enable = platform_bgmac_clk_enable;
	bgmac->cco_ctl_maskset = platform_bgmac_cco_ctl_maskset;
	bgmac->get_bus_clock = platform_bgmac_get_bus_clock;
	bgmac->cmn_maskset32 = platform_bgmac_cmn_maskset32;
	bgmac->phy_connect = bgmac_phy_connect_direct;
	bgmac->feature_flags |= BGMAC_FEAT_FORCE_SPEED_2500;
	return bgmac_enet_probe(bgmac);
}

static int bgmac_remove(struct platform_device *pdev)
{
	struct bgmac *bgmac = platform_get_drvdata(pdev);

	bgmac_enet_remove(bgmac);

	return 0;
}

#ifdef CONFIG_PM
static int bgmac_suspend(struct device *dev)
{
	struct bgmac *bgmac = dev_get_drvdata(dev);

	return bgmac_enet_suspend(bgmac);
}

static int bgmac_resume(struct device *dev)
{
	struct bgmac *bgmac = dev_get_drvdata(dev);

	return bgmac_enet_resume(bgmac);
}

static const struct dev_pm_ops bgmac_pm_ops = {
	.suspend = bgmac_suspend,
	.resume = bgmac_resume
};

#define BGMAC_PM_OPS (&bgmac_pm_ops)
#else
#define BGMAC_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct of_device_id bgmac_of_enet_match[] = {
	{.compatible = "brcm,amac",},
	{.compatible = "brcm,nsp-amac",},
	{.compatible = "brcm,xgs-iproc-amac",},
	{.compatible = "brcm,xgs-wh2-amac",},
	{.compatible = "brcm,ns2-amac",},
	{.compatible = "brcm,serdes-ctrl-amac",},
	{},
};

MODULE_DEVICE_TABLE(of, bgmac_of_enet_match);

static struct platform_driver bgmac_enet_driver = {
	.driver = {
		.name  = "bgmac-enet",
		.of_match_table = bgmac_of_enet_match,
		.pm = BGMAC_PM_OPS
	},
	.probe = bgmac_probe,
	.remove = bgmac_remove,
};

module_platform_driver(bgmac_enet_driver);
MODULE_LICENSE("GPL");
