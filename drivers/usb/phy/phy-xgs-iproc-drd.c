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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/usb/phy.h>
#include <linux/kthread.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>

#ifdef DEBUG
#define USB_GET_RH_STATE		0
#define USB_DUMP_PHY_REGS		0 /* Only support on HX5 */
#endif /* DEBUG */

#define ICFG_USB_CTRL_ADDR(base)							(base + 0x00)
#define  ICFG_USB_CTRL__DRD_FAST_SIM_MODE					7
#define  ICFG_USB_CTRL__DRD_FORCE_HOST_MODE					6
#define  ICFG_USB_CTRL__DRD_FORCE_DEVICE_MODE				5
#define  ICFG_USB_CTRL__XHC_CSR_RESET						4
#define  ICFG_USB_CTRL__BDC_CSR_RESET						3
#define  ICFG_USB_CTRL__DRD_SOFT_RESET						2
#define  ICFG_USB_CTRL__XHC_SOFT_RESET						1
#define  ICFG_USB_CTRL__BDC_SOFT_RESET						0
#define ICFG_USB_STAT_ADDR(base)							(base + 0x10)
#define  ICFG_USB_STAT__DRD_USB_VBUS_OVC					8
#define  ICFG_USB_STAT__DRD_USB_ID							7
#define  ICFG_USB_STAT__DRD_USB_VBUS_PRESENT				6
#define  ICFG_USB_STAT__DRD_USB_HOST						1
#define  ICFG_USB_STAT__DRD_USB_DEVICE						0
#define ICFG_USB_DRD_DBG_REG1_ADDR(base)					(base + 0x14)
#define  ICFG_USB_DBG_OVC_POLARITY							20
#define ICFG_USB_XHC_DBG_REG1_ADDR(base)					(base + 0x24)
#define ICFG_USB_XHC_DBG_REG2_ADDR(base)					(base + 0x28)
#define ICFG_USB_PHY_CTRL_ADDR(base)						(base + 0x2c)
#define  ICFG_USB_PHY_CTRL__SOFT_RESET						1

#define HX5_IPROC_WRAP_USBPHY_CTRL_0_ADDR(base)				(base + 0x00)
#define  HX5_IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO				18
#define  HX5_IPROC_WRAP_USBPHY_CTRL_0__PLL_CTRL_45			17
#define  HX5_IPROC_WRAP_USBPHY_CTRL_0__PLL_SUSPEND_EN		16
#define  HX5_IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB			15
#define  HX5_IPROC_WRAP_USBPHY_CTRL_0__RESETB				14
#define HX5_IPROC_WRAP_USBPHY_CTRL_2_ADDR(base)				(base + 0x08)
#define  HX5_IPROC_WRAP_USBPHY_CTRL_2__CORERDY				6
#define  HX5_IPROC_WRAP_USBPHY_CTRL_2__AFE_LDO_PWRDWNB		2
#define  HX5_IPROC_WRAP_USBPHY_CTRL_2__AFE_PLL_PWRDWNB		1
#define  HX5_IPROC_WRAP_USBPHY_CTRL_2__AFE_BG_PWRDWNB		0
#define HX5_IPROC_WRAP_MISC_STATUS_0_ADDR(base)				(base + 0x1c)
#define  HX5_IPROC_WRAP_MISC_STATUS_0__USBPHY_PWRON_FLAG	5
#define  HX5_IPROC_WRAP_MISC_STATUS_0__USBPHY_PLL_LOCK		0
#define HX5_IPROC_WRAP_MISC_STATUS_1_ADDR(base)				(base + 0x20)

#define HR4_IPROC_WRAP_USBPHY_CTRL_0_ADDR(base)				(base + 0x00)
#define  HR4_IPROC_WRAP_USBPHY_CTRL_0__RESETN				31
#define  HR4_IPROC_WRAP_USBPHY_CTRL_0__PORTRESETN			30
#define HR4_IPROC_WRAP_USBPHY_STATUS_0_ADDR(base)			(base + 0x18)
#define  HR4_IPROC_WRAP_USBPHY_STATUS_0__USB_PLL_LOCK_O		2


enum drd_support_dev_ids {
	XGS_IPROC_USB_HX5 = 0,
	XGS_IPROC_USB_HR4,
	XGS_IPROC_MAX_DEVS,
};

struct iproc_usb_priv {
	struct usb_phy phy;
	struct device *dev;
	struct device_node *dn;
	struct phy_device *mdio_phy;
	void __iomem *wrap_base;
	void __iomem *icfg_usb_base;
	int init_count;
	int usb_device;
};

extern void __iomem *get_iproc_wrap_ctrl_base(void);

#if defined(USB_DUMP_PHY_REGS) && (USB_DUMP_PHY_REGS == 1)
static int g_dump_phy = 0;
module_param(g_dump_phy, int, 0644);
MODULE_PARM_DESC(g_dump_phy, "Dump PHY register");
#endif /* defined(USB_DUMP_PHY_REGS) && (USB_DUMP_PHY_REGS == 1) */
/***************************************************************************
***************************************************************************/
#if defined(USB_DUMP_PHY_REGS) && (USB_DUMP_PHY_REGS == 1)
static u16 phy_rd_reg(struct phy_device *phydev, u16 base, u16 addr)
{
	if (phydev) {
		phy_write(phydev, 0x1f, base);
		return phy_read(phydev, addr);
	}
	return 0;
}

static void phy_wr_reg(struct phy_device *phydev, u16 base, u16 addr, u16 data)
{
	if (phydev) {
		phy_write(phydev, 0x1f, base);
		phy_write(phydev, addr, data);
	}
}

static void __dump_phy_register(struct phy_device *phydev)
{
	int i;
	int data_c[0x1d], data_p[0x1c];

	if (phydev) {
		for (i = 0 ; i <= 0x1c; i++) {
			data_c[i] = phy_rd_reg(phydev, 0x80a0, i);
		}

		for (i = 0 ; i <= 0x1b; i++) {
			data_p[i] = phy_rd_reg(phydev, 0x80c0, i);
		}

		printk("USB – Common Block Registers\n");
		printk("0x0000: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_c[0], data_c[1], data_c[2], data_c[3],
				data_c[4], data_c[5], data_c[6], data_c[7]);
		printk("0x0008: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_c[8], data_c[9], data_c[10], data_c[11],
				data_c[12], data_c[13], data_c[14], data_c[15]);
		printk("0x0010: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_c[16], data_c[17], data_c[18], data_c[19],
				data_c[20], data_c[21], data_c[22], data_c[23]);
		printk("0x0018: %.4x %.4x %.4x %.4x %.4x\n",
				data_c[24], data_c[25], data_c[26], data_c[27], data_c[28]);

		printk("USB – Port Block Registers");
		printk("0x0000: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_p[0], data_p[1], data_p[2], data_p[3],
				data_p[4], data_p[5], data_p[6], data_p[7]);
		printk("0x0008: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_p[8], data_p[9], data_p[10], data_p[11],
				data_p[12], data_p[13], data_p[14], data_p[15]);
		printk("0x0010: %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\n",
				data_p[16], data_p[17], data_p[18], data_p[19],
				data_p[20], data_p[21], data_p[22], data_p[23]);
		printk("0x0018: %.4x %.4x %.4x %.4x\n",
				data_p[24], data_p[25], data_p[26], data_p[27]);

		printk("\n");
	}
}
#endif /* defined(USB_DUMP_PHY_REGS) && (USB_DUMP_PHY_REGS == 1) */

static int iproc_drd_state_kthread(void *arg)
{
	struct iproc_usb_priv *iproc_usb_data = (struct iproc_usb_priv *)arg;
	void __iomem *icfg_usb_base;
	void __iomem *wrap_base;
	struct device *dev;
	u32 val, prev_val = 0;
	u32 curr_mode = 0;

	if (!iproc_usb_data) {
		return -EINVAL;
	}

	icfg_usb_base = iproc_usb_data->icfg_usb_base;
	dev = iproc_usb_data->dev;
	wrap_base = iproc_usb_data->wrap_base;

#if defined(USB_GET_RH_STATE) && USB_GET_RH_STATE == 1
	writel(0x0f100000, ICFG_USB_XHC_DBG_REG1_ADDR(icfg_usb_base));
#endif /* defined(USB_GET_RH_STATE) && USB_GET_RH_STATE == 1 */

	while(1) {
		val = readl(ICFG_USB_STAT_ADDR(icfg_usb_base));
		if (prev_val != val) {
			if (val & (1 << ICFG_USB_STAT__DRD_USB_DEVICE) && curr_mode != 1) {
				dev_info(dev, "drd status change (%.8x): device mode\n", val);
				curr_mode = 1;
			} else if (val & (1 << ICFG_USB_STAT__DRD_USB_HOST) && curr_mode != 2) {
				dev_info(dev, "drd status change (%.8x): host mode\n", val);
				curr_mode = 2;
			} else if (curr_mode != 0){
				dev_info(dev, "drd status change (%.8x): idle mode\n", val);
				curr_mode = 0;

				if (iproc_usb_data->usb_device == XGS_IPROC_USB_HX5) {
					/* WAR from USB DVT "FS mode stuck and stopping HS mode to work"
					* Reset the USB phy
					*/
					val = readl(ICFG_USB_PHY_CTRL_ADDR(icfg_usb_base));
					val &= ~(1 << ICFG_USB_PHY_CTRL__SOFT_RESET);
					writel(val, ICFG_USB_PHY_CTRL_ADDR(icfg_usb_base));
					udelay(100);
					val |= (1 << ICFG_USB_PHY_CTRL__SOFT_RESET);
					writel(val, ICFG_USB_PHY_CTRL_ADDR(icfg_usb_base));
				}
			}

			prev_val = val;
		}

#if defined(USB_DUMP_PHY_REGS) && (USB_DUMP_PHY_REGS == 1)
		/* For debug purpose */
		if (g_dump_phy) {
			__dump_phy_register(iproc_usb_data->mdio_phy);
			g_dump_phy = 0;
		}
#endif /* defined(USB_DUMP_PHY_REGS) && USB_DUMP_PHY_REGS == 1 */

#if defined(USB_GET_RH_STATE) && (USB_GET_RH_STATE == 1)
		{	/* Internal debug register */
			u32 prev_state = 0, curr_state = 0;

			curr_state = readl(ICFG_USB_XHC_DBG_REG2_ADDR(icfg_usb_base));
			if ((prev_state != curr_state) || g_dump_phy) {
				printk("RH_STATE: prev:%.8x, curr:%.8x\n", prev_state, curr_state);
				prev_state = curr_state;
			}
		}
#endif /* defined(USB_GET_RH_STATE) && USB_GET_RH_STATE == 1 */

		usleep_range(1000, 10000);
	}
	return 0;
}

static int _wait_pll_lock(struct device *dev, void __iomem *reg, uint32_t mask)
{
	volatile uint32_t val;
	uint32_t count = 0;

	do {
		val = readl((volatile uint32_t *)reg);
		if ((val & mask) == mask) {
			break;
		} else {
			udelay(10);
			count ++;
		}
	} while(count <= 10);

	if (count > 10) {
		dev_err(dev, "%s : PLL not lock! IPROC_WRAP_MISC_STATUS_0 = 0x%.8x\n",
							__FUNCTION__, val);
		return -ETIMEDOUT;
	}
	return 0;
}

static int hx5_usbphy_init(struct device *dev, struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base;
	void __iomem *icfg_usb_base;
	ulong mask, count = 0;
	u32 val;
	int ret = 0, retry = 5;

	if (!iproc_usb_data) {
		return -EINVAL;
	}

	wrap_base = iproc_usb_data->wrap_base;
	icfg_usb_base = iproc_usb_data->icfg_usb_base;

	while(retry--) {
		/* Enable the USB phy */
		val = readl(HX5_IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
		val |= (1 << HX5_IPROC_WRAP_USBPHY_CTRL_2__CORERDY);
		writel(val, HX5_IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

		/* check pll_lock */
		count = 0;
		mask = ((1 << HX5_IPROC_WRAP_MISC_STATUS_0__USBPHY_PLL_LOCK) |
				(1 << HX5_IPROC_WRAP_MISC_STATUS_0__USBPHY_PWRON_FLAG));

		ret = _wait_pll_lock(dev, HX5_IPROC_WRAP_MISC_STATUS_0_ADDR(wrap_base), mask);
		if (ret) {
			msleep(10);
			continue;
		}

		/* Bring the USB phy out of reset */
		val = readl(ICFG_USB_PHY_CTRL_ADDR(icfg_usb_base));
		val |= (1 << ICFG_USB_PHY_CTRL__SOFT_RESET);
		writel(val, ICFG_USB_PHY_CTRL_ADDR(icfg_usb_base));
		break;
	}

	return ret;
}

static int hr4_usbphy_init(struct device *dev, struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base;
	ulong mask, val;

	if (!iproc_usb_data) {
		return -EINVAL;
	}

	wrap_base = iproc_usb_data->wrap_base;

	val = readl((volatile uint32_t *)HR4_IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~((1 << HR4_IPROC_WRAP_USBPHY_CTRL_0__RESETN) |
		     (1 << HR4_IPROC_WRAP_USBPHY_CTRL_0__PORTRESETN));
	writel(val, (volatile uint32_t *)HR4_IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	msleep(1);
	val |= ((1 << HR4_IPROC_WRAP_USBPHY_CTRL_0__RESETN) |
		    (1 << HR4_IPROC_WRAP_USBPHY_CTRL_0__PORTRESETN));
	writel(val, (volatile uint32_t *)HR4_IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	msleep(10);

	mask = (1 << HR4_IPROC_WRAP_USBPHY_STATUS_0__USB_PLL_LOCK_O);
	return _wait_pll_lock(dev, HR4_IPROC_WRAP_USBPHY_STATUS_0_ADDR(wrap_base), mask);
}

static int iproc_drd_init(struct device *dev, struct usb_phy *phy)
{
	struct iproc_usb_priv *iproc_usb_data = container_of(phy, struct iproc_usb_priv, phy);
	void __iomem *wrap_base;
	void __iomem *icfg_usb_base;
	u32 val;
	int ret = 0;

	if (!iproc_usb_data) {
		return -EINVAL;
	}

	if (iproc_usb_data->init_count) {
		return 0;
	}

	dev_info(dev, "usb drd init.\n");

	wrap_base = iproc_usb_data->wrap_base;
	icfg_usb_base = iproc_usb_data->icfg_usb_base;

	if (iproc_usb_data->usb_device == XGS_IPROC_USB_HX5) {
		hx5_usbphy_init(dev, iproc_usb_data);
	} else if (iproc_usb_data->usb_device == XGS_IPROC_USB_HR4) {
		hr4_usbphy_init(dev, iproc_usb_data);
	} else {
		return -EINVAL;
	}

	val = readl(ICFG_USB_STAT_ADDR(icfg_usb_base));
	if (val & (1 << ICFG_USB_STAT__DRD_USB_VBUS_OVC)) {
		/* WAR for OVC */
		val = readl(ICFG_USB_DRD_DBG_REG1_ADDR(icfg_usb_base));
		val |= (1 << ICFG_USB_DBG_OVC_POLARITY);
		writel(val, ICFG_USB_DRD_DBG_REG1_ADDR(icfg_usb_base));
	}

	/* Set the soft reset and csr reset of XHC and BDC */
	val = readl(ICFG_USB_CTRL_ADDR(icfg_usb_base));
	val |= (1 << ICFG_USB_CTRL__DRD_SOFT_RESET);
	val |= (1 << ICFG_USB_CTRL__BDC_CSR_RESET);
	val |= (1 << ICFG_USB_CTRL__XHC_CSR_RESET);
	val |= (1 << ICFG_USB_CTRL__BDC_SOFT_RESET);
	val |= (1 << ICFG_USB_CTRL__XHC_SOFT_RESET);
	writel(val, ICFG_USB_CTRL_ADDR(icfg_usb_base));

	iproc_usb_data->init_count = 1;

	return ret;
}

static int iproc_usb_phy_init(struct usb_phy *phy)
{
	struct iproc_usb_priv *iproc_usb_data;
	struct device *dev;
	struct task_struct *kthread_task;
	int ret = 0;

	if (!phy) {
		return -EINVAL;
	}

	iproc_usb_data = container_of(phy, struct iproc_usb_priv, phy);
	if (!iproc_usb_data) {
		return -EINVAL;
	}
	dev = iproc_usb_data->dev;

	ret = iproc_drd_init(dev, &iproc_usb_data->phy);
	if (ret) {
		dev_err(dev, "Failed to init the usb drd\n");
		return ret;
	}

	kthread_task = kthread_create(iproc_drd_state_kthread, iproc_usb_data, "drd");
	if (IS_ERR(kthread_task)) {
		dev_err(dev, "Failed to creat drd kernel thread\n");
		return PTR_ERR(kthread_task);
	}
	wake_up_process(kthread_task);

	return ret;
}

static int xgs_iproc_drd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = pdev->dev.of_node;
	struct iproc_usb_priv *iproc_usb_data;
	struct device_node *mdio_phy_np = NULL;
	int ret;

	if (!of_device_is_available(dn)) {
		return -ENODEV;
	}

	iproc_usb_data = devm_kzalloc(dev, sizeof(*iproc_usb_data), GFP_KERNEL);
	if (!iproc_usb_data) {
		dev_err(dev, "devm_kzalloc() failed\n" );
		return -ENOMEM;
	}
	memset(iproc_usb_data, 0, sizeof(*iproc_usb_data));
	platform_set_drvdata(pdev, iproc_usb_data);

	iproc_usb_data->dev = dev;

	iproc_usb_data->wrap_base = get_iproc_wrap_ctrl_base();
	if (!iproc_usb_data->wrap_base) {
		dev_err(&pdev->dev, "can't iomap usb phy base address\n");
		ret = -ENOMEM;
		goto err;
	}

	iproc_usb_data->icfg_usb_base = (void *)of_iomap(dn, 0);
	if (!iproc_usb_data->icfg_usb_base) {
		dev_err(&pdev->dev, "can't iomap icfg usb base address\n");
		ret = -ENOMEM;
		goto err;
	}

	mdio_phy_np = of_parse_phandle(dn, "mdio-phy-handle", 0);
	if (mdio_phy_np) {
		iproc_usb_data->mdio_phy = of_phy_find_device(mdio_phy_np);
	} else {
		iproc_usb_data->mdio_phy = NULL;
	}

	if (of_device_is_compatible(dn, "brcm,usb-phy,hx5")) {
		iproc_usb_data->usb_device = XGS_IPROC_USB_HX5;
	} else if (of_device_is_compatible(dn, "brcm,usb-phy,hr4")) {
		iproc_usb_data->usb_device = XGS_IPROC_USB_HR4;
	} else {
		return -ENODEV;
	}

	iproc_usb_data->phy.dev = dev;
	iproc_usb_data->phy.type = USB_PHY_TYPE_USB2;
	iproc_usb_data->phy.init = iproc_usb_phy_init;
	iproc_usb_data->init_count = 0;

	ret = usb_add_phy_dev(&iproc_usb_data->phy);
	if (ret) {
		dev_err(&pdev->dev, "failed to add the phy device\n");
		goto err;
	}

	return 0;

err:
	if (iproc_usb_data->icfg_usb_base) {
		iounmap(iproc_usb_data->icfg_usb_base);
	}
	if (iproc_usb_data) {
		iounmap(iproc_usb_data);
	}

	return ret;
}

static int xgs_iproc_drd_remove(struct platform_device *pdev)
{
	struct iproc_usb_priv *iproc_usb_data = platform_get_drvdata(pdev);

	usb_remove_phy(&iproc_usb_data->phy);

	platform_set_drvdata(pdev, NULL);
	if (iproc_usb_data->icfg_usb_base) {
		iounmap(iproc_usb_data->icfg_usb_base);
	}

	if (iproc_usb_data) {
		iounmap(iproc_usb_data);
	}

	return 0;
}

static const struct of_device_id xgs_iproc_drd_dt_ids[] = {
	{ .compatible = "brcm,usb-phy,hx5", },
	{ .compatible = "brcm,usb-phy,hr4", },
	{ }
};
MODULE_DEVICE_TABLE(of, xgs_iproc_drd_dt_ids);

static struct platform_driver xgs_iproc_drd_driver =
{
	.driver = {
		.name = "usb-phy",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgs_iproc_drd_dt_ids),
	},
	.probe = xgs_iproc_drd_probe,
	.remove = xgs_iproc_drd_remove,
};

module_platform_driver(xgs_iproc_drd_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB DRD controller driver");
MODULE_LICENSE("GPL");
