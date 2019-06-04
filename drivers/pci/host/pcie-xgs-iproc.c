/*
 * Copyright (C) 2014 Hauke Mehrtens <hauke@hauke-m.de>
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mbus.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/phy/phy.h>

#include "pcie-iproc.h"

#define CLK_CONTROL_OFFSET		0x000

#define CFG_IND_ADDR_OFFSET		0x120
#define CFG_IND_ADDR_MASK		0x00001ffc
#define CFG_IND_DATA_OFFSET		0x124

#define CFG_ADDR_OFFSET 		0x1f8
#define CFG_ADDR_BUS_NUM_SHIFT		20
#define CFG_ADDR_BUS_NUM_MASK		0x0ff00000
#define CFG_ADDR_DEV_NUM_SHIFT		15
#define CFG_ADDR_DEV_NUM_MASK		0x000f8000
#define CFG_ADDR_FUNC_NUM_SHIFT 	12
#define CFG_ADDR_FUNC_NUM_MASK		0x00007000
#define CFG_ADDR_REG_NUM_SHIFT		2
#define CFG_ADDR_REG_NUM_MASK		0x00000ffc
#define CFG_ADDR_CFG_TYPE_SHIFT 	0
#define CFG_ADDR_CFG_TYPE_MASK		0x00000003

#define CFG_DATA_OFFSET 		0x1fc

#define SYS_RC_INTX_EN			0x330
#define SYS_RC_INTX_MASK		0xf

#define IPROC_PCI_EXP_CAP		0xac

struct pcie_sw_wa {
	const char *wa_name;
	void (*wa_func)(struct iproc_pcie *pcie);
};

extern void xgs_phy_wr_reg(struct phy_device *phydev, u32 regnum, u16 data);

static inline struct iproc_pcie *iproc_pcie_data(struct pci_bus *bus)
{
	struct iproc_pcie *pcie;
#ifdef CONFIG_ARM
	struct pci_sys_data *sys = bus->sysdata;

	pcie = sys->private_data;
#else
	pcie = bus->sysdata;
#endif
	return pcie;
}

/**
 * Note access to the configuration registers are protected at the higher layer
 * by 'pci_lock' in drivers/pci/access.c
 */
static void __iomem *iproc_pcie_map_cfg_bus(struct iproc_pcie *pcie,
					    int busno, unsigned int devfn,
					    int where)
{
	u32 slot = PCI_SLOT(devfn);
	u32 fn = PCI_FUNC(devfn);
	u32 val;

	/* root complex access */
	if (busno == 0) {
		if (slot >= 1)
			return NULL;
		writel(where & CFG_IND_ADDR_MASK,
		       pcie->base + CFG_IND_ADDR_OFFSET);
		return (pcie->base + CFG_IND_DATA_OFFSET);
	}

	if (fn > 1)
		return NULL;

	/* EP device access */
	val = (busno << CFG_ADDR_BUS_NUM_SHIFT) |
		(slot << CFG_ADDR_DEV_NUM_SHIFT) |
		(fn << CFG_ADDR_FUNC_NUM_SHIFT) |
		(where & CFG_ADDR_REG_NUM_MASK) |
		(1 & CFG_ADDR_CFG_TYPE_MASK);
	writel(val, pcie->base + CFG_ADDR_OFFSET);

	return (pcie->base + CFG_DATA_OFFSET);
}

static void __iomem *iproc_pcie_bus_map_cfg_bus(struct pci_bus *bus,
						unsigned int devfn,
						int where)
{
	return iproc_pcie_map_cfg_bus(iproc_pcie_data(bus), bus->number, devfn,
				      where);
}

static int iproc_pci_raw_config_read32(struct iproc_pcie *pcie,
				       unsigned int devfn, int where,
				       int size, u32 *val)
{
	void __iomem *addr;

	addr = iproc_pcie_map_cfg_bus(pcie, 0, devfn, where & ~0x3);
	if (!addr) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	*val = readl(addr);

	if (size <= 2)
		*val = (*val >> (8 * (where & 3))) & ((1 << (size * 8)) - 1);

	return PCIBIOS_SUCCESSFUL;
}

static int iproc_pci_raw_config_write32(struct iproc_pcie *pcie,
					unsigned int devfn, int where,
					int size, u32 val)
{
	void __iomem *addr;
	u32 mask, tmp;

	addr = iproc_pcie_map_cfg_bus(pcie, 0, devfn, where & ~0x3);
	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size == 4) {
		writel(val, addr);
		return PCIBIOS_SUCCESSFUL;
	}

	mask = ~(((1 << (size * 8)) - 1) << ((where & 0x3) * 8));
	tmp = readl(addr) & mask;
	tmp |= val << ((where & 0x3) * 8);
	writel(tmp, addr);

	return PCIBIOS_SUCCESSFUL;
}


static struct pci_ops iproc_pcie_ops = {
	.map_bus = iproc_pcie_bus_map_cfg_bus,
	.read = pci_generic_config_read32,
	.write = pci_generic_config_write32,
};

static inline void pcie_wrong_gen2_wa(struct iproc_pcie * pcie)
{
	xgs_phy_wr_reg(pcie->mdio_phy, 0x8633, 0x190);
	xgs_phy_wr_reg(pcie->mdio_phy, 0x8639, 0x191);
}

static void iproc_pcie_reset(struct iproc_pcie *pcie)
{
	/* configured as RC and send a downstream reset */
	writel(0, pcie->base + CLK_CONTROL_OFFSET);
	mdelay(1);
	writel(1, pcie->base + CLK_CONTROL_OFFSET);
	msleep(100);
}

static void pcie_rc_wa(struct iproc_pcie * pcie)
{
	/* Setting for PCIe Serdes PLL output */
	xgs_phy_wr_reg(pcie->mdio_phy, 0x2103, 0x2b1c);
	xgs_phy_wr_reg(pcie->mdio_phy, 0x1300, 0x000b);
	msleep(100);
}

/* currently for Greyhound */
static void pcie_tx_de_emp_wa(struct iproc_pcie * pcie)
{
	u32 tmp32;

	iproc_pci_raw_config_read32(pcie, 0, 0xdc, 4, &tmp32);
	tmp32 |= (0x1 << 6);
	iproc_pci_raw_config_write32(pcie, 0, 0xdc, 4, tmp32);
	iproc_pci_raw_config_read32(pcie, 0, 0xdc, 4, &tmp32);
}

static const struct pcie_sw_wa pcie_wa_tab[] = {
	{
		.wa_name = "pcie_wrong_gen2",
		.wa_func = pcie_wrong_gen2_wa,
	},
	{
		.wa_name = "pcie_rc",
		.wa_func = pcie_rc_wa,
	},
	{
		.wa_name = "pcie_perst",
		.wa_func = iproc_pcie_reset,
	},
	{
		.wa_name = "pcie_tx_de_emp",
		.wa_func = pcie_tx_de_emp_wa,
	},
};

/*
 * Run the specific pcie workaround function specified in "pcie_wa_tab",
 * if "wa_name" is found on the "wa-list" property of pcie node.
 */
static void pcie_sw_wa_func(const char *wa_name, struct iproc_pcie *pcie)
{
	struct device_node *np = pcie->dev->of_node;
	int wa_num_max = ARRAY_SIZE(pcie_wa_tab);
	int wa_num = of_property_count_strings(np, "wa-list");
	const char *wa_name_dts;
	int i;

	/* workaround required? */
	if (wa_num <= 0)
		return;

	for (i = 0; i < wa_num; i++) {
		of_property_read_string_index(np, "wa-list", i, &wa_name_dts);
		if (!strcmp(wa_name, wa_name_dts))
			break;
	}

	/* The wa_name is found on wa-list of pcie node? */
	if (i == wa_num)
		return;

	for (i = 0; i < wa_num_max; i++)
		if (!strcmp(pcie_wa_tab[i].wa_name, wa_name)) {
			pcie_wa_tab[i].wa_func(pcie);
			break;
		}
}

static int iproc_pcie_check_link(struct iproc_pcie *pcie)
{
	struct device *dev = pcie->dev;
	u32 hdr_type, link_ctrl, link_status, class;
	bool link_is_active = false;

	/* make sure we are not in EP mode */
	iproc_pci_raw_config_read32(pcie, 0, PCI_HEADER_TYPE, 1, &hdr_type);
	if ((hdr_type & 0x7f) != PCI_HEADER_TYPE_BRIDGE) {
		dev_err(dev, "in EP mode, hdr=%#02x\n", hdr_type);
		return -EFAULT;
	}

	/* SB2/GH/HR3/GH2 */
	pcie_sw_wa_func("pcie_rc", pcie);

	/* GH/HR3/GH2 */
	pcie_sw_wa_func("pcie_perst", pcie);

	/* Not enabled in DT file currently */
	pcie_sw_wa_func("pcie_tx_de_emp", pcie);

	/* Force class to PCI_CLASS_BRIDGE_PCI (0x0604) */
#define PCI_BRIDGE_CTRL_REG_OFFSET 0x43c
#define PCI_CLASS_BRIDGE_MASK      0xffff00
#define PCI_CLASS_BRIDGE_SHIFT     8
	iproc_pci_raw_config_read32(pcie, 0, PCI_BRIDGE_CTRL_REG_OFFSET,
				    4, &class);
	class &= ~PCI_CLASS_BRIDGE_MASK;
	class |= (PCI_CLASS_BRIDGE_PCI << PCI_CLASS_BRIDGE_SHIFT);
	iproc_pci_raw_config_write32(pcie, 0, PCI_BRIDGE_CTRL_REG_OFFSET,
				     4, class);

	/* check link status to see if link is active */
	iproc_pci_raw_config_read32(pcie, 0, IPROC_PCI_EXP_CAP + PCI_EXP_LNKSTA,
				    2, &link_status);
	if (link_status & PCI_EXP_LNKSTA_NLW)
		link_is_active = true;

	if (!link_is_active) {
		/* try GEN 1 link speed */
#define PCI_TARGET_LINK_SPEED_MASK	0xf
#define PCI_TARGET_LINK_SPEED_GEN2	0x2
#define PCI_TARGET_LINK_SPEED_GEN1	0x1
		iproc_pci_raw_config_read32(pcie, 0,
					    IPROC_PCI_EXP_CAP + PCI_EXP_LNKCTL2,
					    4, &link_ctrl);
		if ((link_ctrl & PCI_TARGET_LINK_SPEED_MASK) ==
		    PCI_TARGET_LINK_SPEED_GEN2) {
			link_ctrl &= ~PCI_TARGET_LINK_SPEED_MASK;
			link_ctrl |= PCI_TARGET_LINK_SPEED_GEN1;
			iproc_pci_raw_config_write32(pcie, 0,
					IPROC_PCI_EXP_CAP + PCI_EXP_LNKCTL2,
					4, link_ctrl);
			msleep(100);

			iproc_pci_raw_config_read32(pcie, 0,
					IPROC_PCI_EXP_CAP + PCI_EXP_LNKSTA,
					2, &link_status);
			if (link_status & PCI_EXP_LNKSTA_NLW)
				link_is_active = true;
		}
	}

	dev_info(pcie->dev, "link: %s\n", link_is_active ? "UP" : "DOWN");

	return link_is_active ? 0 : -ENODEV;
}

static void iproc_pcie_enable(struct iproc_pcie *pcie)
{
	writel(SYS_RC_INTX_MASK, pcie->base + SYS_RC_INTX_EN);
}

static int iproc_pcie_msi_enable(struct iproc_pcie *pcie)
{
	struct device_node *msi_node;

	msi_node = of_parse_phandle(pcie->dev->of_node, "msi-parent", 0);
	if (!msi_node)
		return -ENODEV;

	/*
	 * If another MSI controller is being used, the call below should fail
	 * but that is okay
	 */
	return iproc_msi_init(pcie, msi_node);
}

static void iproc_pcie_msi_disable(struct iproc_pcie *pcie)
{
	iproc_msi_exit(pcie);
}

static int iproc_pcie_setup(struct iproc_pcie *pcie, struct list_head *res)
{
	struct device *dev;
	int ret;
	void *sysdata;
	struct pci_bus *child;
	struct pci_host_bridge *host = pci_host_bridge_from_priv(pcie);

	if (!pcie || !pcie->dev || !pcie->base)
		return -EINVAL;

	dev = pcie->dev;

	ret = devm_request_pci_bus_resources(dev, res);
	if (ret)
		return ret;

	ret = phy_init(pcie->phy);
	if (ret) {
		dev_err(pcie->dev, "unable to initialize PCIe PHY\n");
		return ret;
	}

	ret = phy_power_on(pcie->phy);
	if (ret) {
		dev_err(pcie->dev, "unable to power on PCIe PHY\n");
		goto err_exit_phy;
	}

	iproc_pcie_reset(pcie);

#ifdef CONFIG_ARM
	pcie->sysdata.private_data = pcie;
	sysdata = &pcie->sysdata;
#else
	sysdata = pcie;
#endif

	ret = iproc_pcie_check_link(pcie);
	if (ret) {
		dev_err(pcie->dev, "no PCIe EP device detected\n");
		goto err_power_off_phy;
	}

	iproc_pcie_enable(pcie);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		if (iproc_pcie_msi_enable(pcie))
			dev_info(pcie->dev, "not using iProc MSI\n");

	pci_add_flags(PCI_REASSIGN_ALL_RSRC | PCI_REASSIGN_ALL_BUS);

	list_splice_init(res, &host->windows);
	host->busnr = 0;
	host->dev.parent = dev;
	host->ops = &iproc_pcie_ops;
	host->sysdata = sysdata;
	host->map_irq = pcie->map_irq;
	host->swizzle_irq = pci_common_swizzle;

	ret = pci_scan_root_bus_bridge(host);
	if (ret < 0) {
		dev_err(dev, "failed to scan host: %d\n", ret);
		goto err_power_off_phy;
	}

	pci_assign_unassigned_bus_resources(host->bus);

	pcie->root_bus = host->bus;

	list_for_each_entry(child, &host->bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(host->bus);

	return 0;

err_power_off_phy:
	phy_power_off(pcie->phy);
err_exit_phy:
	phy_exit(pcie->phy);

	return ret;
}

static int iproc_pcie_probe(struct platform_device *pdev)
{
	struct iproc_pcie *pcie;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *mdio_phy_np = NULL;
	struct resource reg;
	struct pci_host_bridge *bridge;
	resource_size_t iobase = 0;
	LIST_HEAD(res);
	int ret;

	bridge = devm_pci_alloc_host_bridge(&pdev->dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = &pdev->dev;
	platform_set_drvdata(pdev, pcie);

	pcie->type = (enum iproc_pcie_type)np->data;

	ret = of_address_to_resource(np, 0, &reg);
	if (ret < 0) {
		dev_err(pcie->dev, "unable to obtain controller resources\n");
		return ret;
	}

	pcie->base = devm_ioremap_resource(pcie->dev, &reg);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	/* MSI message base addr*/
	pcie->base_addr = reg.start;

	/* PHY use is optional */
	pcie->phy = devm_phy_get(&pdev->dev, "pcie-phy");
	if (IS_ERR(pcie->phy)) {
		if (PTR_ERR(pcie->phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		pcie->phy = NULL;
	}

	/* PHY controlled through MDIO for HX4/KT2 */
	mdio_phy_np = of_parse_phandle(np, "mdio-phy-handle", 0);
	if (mdio_phy_np)
		pcie->mdio_phy = of_phy_find_device(mdio_phy_np);

	ret = of_pci_get_host_bridge_resources(np, 0, 0xff, &res, &iobase);
	if (ret) {
		dev_err(pcie->dev,
			"unable to get PCI host bridge resources\n");
		return ret;
	}

	/* HX4/KT2/HR2 */
	pcie_sw_wa_func("pcie_wrong_gen2", pcie);

	pcie->map_irq = of_irq_parse_and_map_pci;

	ret = iproc_pcie_setup(pcie, &res);
	if (ret)
		dev_err(pcie->dev, "PCIe controller setup failed\n");

	pci_free_resource_list(&res);

	return ret;
}

static int iproc_pcie_remove(struct platform_device *pdev)
{
	struct iproc_pcie *pcie = platform_get_drvdata(pdev);

	pci_stop_root_bus(pcie->root_bus);
	pci_remove_root_bus(pcie->root_bus);

	iproc_pcie_msi_disable(pcie);

	phy_power_off(pcie->phy);
	phy_exit(pcie->phy);

	return 0;
}

static const struct of_device_id iproc_pcie_of_match_table[] = {
	{
		.compatible = "brcm,iproc-pcie",
		.data = (int *)IPROC_PCIE_PAXB,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, iproc_pcie_of_match_table);

static struct platform_driver iproc_pcie_pltfm_driver = {
	.driver = {
		.name = "iproc-pcie",
		.of_match_table = of_match_ptr(iproc_pcie_of_match_table),
	},
	.probe = iproc_pcie_probe,
	.remove = iproc_pcie_remove,
};

module_platform_driver(iproc_pcie_pltfm_driver);

MODULE_DESCRIPTION("Broadcom XGS iProc PCIe driver");
MODULE_LICENSE("GPL v2");
