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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/usb/phy.h>
#include <linux/usb/iproc_usb.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>


#define USB2_IDM_IO_CONTROL_DIRECT_ADDR(base)		(base + 0x408)
#define USB2_IDM_RESET_CONTROL_ADDR(base)		(base + 0x800)
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK 	BIT(1)
#define USB2_IDM_RESET_CONTROL__RESET			BIT(0)
#define USB2_IDM_IO_CONTROL_DIRECT__clk_enable		BIT(0)

/* HX4 */
#define HX4_WRAP_XGPLL_CTRL_0_ADDR(base)		(base + 0x1c)
#define HX4_WRAP_XGPLL_CTRL_4_ADDR(base)		(base + 0x2c)
#define HX4_WRAP_USBPHY_CTRL_ADDR(base) 		(base + 0x34)
#define HX4_WRAP_MISC_STATUS_ADDR(base) 		(base + 0x38)
#define IPROC_CLK_NDIV_40                               0x80
#define IPROC_CLK_NDIV_20                               0x8C
#define USB_CLK_NDIV_MASK                               0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK                          0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK                          0xFFFFFE00
#define USB_CLK_NDIV_40                                 0x30
#define USB_CLK_NDIV_20                                 0x60
#define HX4_XGPLL_CTRL_4__NDIV_INT_R			0
#define HX4_XGPLL_CTRL_4__NDIV_INT_WIDTH		8
#define HX4_XGPLL_CTRL_0__CH3_MDIV_R			8
#define HX4_XGPLL_CTRL_0__CH3_MDIV_WIDTH		8

/* KT2 */
#define KT2_PLL_CTRL_REG_3_ADDR(base)			(base + 0x0c)
#define KT2_PLL_CTRL_REG_5_ADDR(base)        		(base + 0x14)
#define KT2_WRAP_USBPHY_CTRL_ADDR(base) 		(base + 0x20)
#define KT2_WRAP_MISC_STATUS_ADDR(base) 		(base + 0x28)
#define KT2_PLL_CTRL_REG_3__NDIV_INT_R			0
#define KT2_PLL_CTRL_REG_3__NDIV_INT_WIDTH		10
#define KT2_PLL_CTRL_REG_5__CH1_MDIV_R			0
#define KT2_PLL_CTRL_REG_5__CH1_MDIV_WIDTH		8

/* SB2/GH/GH2/HR3 */
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ		BIT(26)
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB		BIT(25)
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB		BIT(24)
#define IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO		BIT(17)
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0		BIT(0)
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11		BIT(11)

/* SB2 */
#define SB2_WRAP_USBPHY_CTRL_0_ADDR(base)		(base + 0x28)
#define SB2_WRAP_USBPHY_CTRL_2_ADDR(base)		(base + 0x30)
#define SB2_WRAP_MISC_STATUS_ADDR(base) 		(base + 0x44)
#define IPROC_WRAP_TOP_STRAP_CTRL_ADDR(base)		(base + 0x70)
#define IPROC_WRAP_TOP_STRAP_CTRL__USB_DEVICE		BIT(10)

/* GH/GH2/HR3 */
#define GH_WRAP_USBPHY_CTRL_0_ADDR(base)		(base + 0x44)
#define GH_WRAP_USBPHY_CTRL_2_ADDR(base)		(base + 0x4c)
#define GH_WRAP_MISC_STATUS_ADDR(base)			(base + 0x58)
#define IPROC_WRAP_TOP_STRAP_STATUS_ADDR(base)		(base + 0xa4)
#define IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL		BIT(17)

/* GH2 */
#define USBH_Utmi_p0Ctl(base)				(base + 0x10)


struct iproc_usb_priv {
	struct usb_phy phy;
	struct phy_device *mdio_phy;
	void __iomem *wrap_base;
	void __iomem *idm_base;
	void __iomem *utmi_base;
	int usb_mode;
	int init_count;
};

extern void __iomem *get_iproc_wrap_ctrl_base(void);
extern void xgs_phy_wr_reg(struct phy_device *phydev, u32 regnum, u16 data);
extern u16 xgs_phy_rd_reg(struct phy_device *phydev, u32 regnum);
extern void xgs_sb2_usb_phy_wr_reg(struct phy_device *phydev, u32 regnum,
					u16 data);

/***************************************************************************
****************************************************************************
***************************************************************************/


/* check pll_lock */
static bool check_usbphy_pll_lock(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	void __iomem *wrap_addr = NULL;
	struct device_node *dn = iproc_usb_data->phy.dev->of_node;
	u32 val=0, mask;
	u32 count = 0;

	if (of_device_is_compatible(dn, "brcm,usb-phy-hx4"))
		wrap_addr = HX4_WRAP_MISC_STATUS_ADDR(wrap_base);
	else if (of_device_is_compatible(dn, "brcm,usb-phy-kt2"))
		wrap_addr = KT2_WRAP_MISC_STATUS_ADDR(wrap_base);
	else if (of_device_is_compatible(dn, "brcm,usb-phy-sb2"))
		wrap_addr = SB2_WRAP_MISC_STATUS_ADDR(wrap_base);
	else if (of_device_is_compatible(dn, "brcm,usb-phy-gh") ||
			of_device_is_compatible(dn, "brcm,usb-phy-hr3") ||
			of_device_is_compatible(dn, "brcm,usb-phy-gh2"))
		wrap_addr = GH_WRAP_MISC_STATUS_ADDR(wrap_base);

	if (!wrap_addr) {
		dev_warn(iproc_usb_data->phy.dev, "No wrap addr specified\n");
		return 0;
	}

	mask = IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK;
	do {
		val = readl(wrap_addr);
		if ((val & mask) == mask)
			break;

		udelay(10);
		count ++;
	} while(count <= 10);

	if (count <= 10)
		return 1;

	dev_warn(iproc_usb_data->phy.dev,
			"PLL not locked: IPROC_WRAP_MISC_STATUS = %x\n", val);
	return 0;
}

static int xgs_iproc_usb_phy_mode(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	struct device *dev = iproc_usb_data->phy.dev;
	struct device_node *dn = dev->of_node;
	int usb_mode = IPROC_USB_MODE_HOST;
	u32 __maybe_unused val;
	int __maybe_unused gpio_pin, ret;

	if (!wrap_base)
		dev_warn(dev, "no wrap base addr");

	if (of_device_is_compatible(dn, "brcm,usb-phy-hx4") ||
		of_device_is_compatible(dn, "brcm,usb-phy-kt2")) {
		/* gpio pin 4 to control host/device mode */
		gpio_pin = of_get_named_gpio(dev->of_node, "usbdev-gpio", 0);

		if (gpio_pin < 0) {
			/* default to 4 */
			dev_warn(dev, "No gpio pin set for USB device detection\n");
			gpio_pin = 4;
		}

		ret = devm_gpio_request(dev, gpio_pin, "usbdev-gpio");
		if (ret) {
			dev_warn(dev, "request gpio #%d fail\n", gpio_pin);
			/* Use default host mode */
			return usb_mode;
		}

		gpio_direction_input(gpio_pin);
		val = gpio_get_value(gpio_pin);
		if (val)
			usb_mode = IPROC_USB_MODE_DEVICE;

		devm_gpio_free(dev, gpio_pin);
	} else if (of_device_is_compatible(dn, "brcm,usb-phy-sb2")) {
		/* u-boot enable this bit to indicate usb in host mode */
		val = readl(IPROC_WRAP_TOP_STRAP_CTRL_ADDR(wrap_base));
		if (!(val & IPROC_WRAP_TOP_STRAP_CTRL__USB_DEVICE))
			usb_mode = IPROC_USB_MODE_DEVICE;
	} else if (of_device_is_compatible(dn, "brcm,usb-phy-gh") ||
			of_device_is_compatible(dn, "brcm,usb-phy-hr3") ||
			of_device_is_compatible(dn, "brcm,usb-phy-gh2")) {
		/* u-boot enable this bit to indicate usb in host mode */
		val = readl(IPROC_WRAP_TOP_STRAP_STATUS_ADDR(wrap_base));
		if (!(val & IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL))
			usb_mode = IPROC_USB_MODE_DEVICE;
	}

	dev_info(dev, "usb mode: %s\n",
			usb_mode == IPROC_USB_MODE_DEVICE ? "DEVICE" : "HOST");

	return usb_mode;
}

/* Returns USB PHY PLL ref clock in MHz for HX4/KT2 */
static u32 _get_usb_clk(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	struct device_node *dn = iproc_usb_data->phy.dev->of_node;
	u32 ndiv, mdiv, refclk;
	u32 val;

	if (of_device_is_compatible(dn, "brcm,usb-phy-hx4")) {
		val = readl(HX4_WRAP_XGPLL_CTRL_4_ADDR(wrap_base));
		ndiv = ((val >> HX4_XGPLL_CTRL_4__NDIV_INT_R) &
			~(0xFFFFFFFF << HX4_XGPLL_CTRL_4__NDIV_INT_WIDTH));

		val = readl(HX4_WRAP_XGPLL_CTRL_0_ADDR(wrap_base));
		mdiv = ((val >> HX4_XGPLL_CTRL_0__CH3_MDIV_R) &
			~(0xFFFFFFFF << HX4_XGPLL_CTRL_0__CH3_MDIV_WIDTH));
	} else /*if (of_device_is_compatible(dn, "brcm,usb-phy-kt2"))*/ {
		val = readl(KT2_PLL_CTRL_REG_3_ADDR(wrap_base));
		ndiv = ((val >> KT2_PLL_CTRL_REG_3__NDIV_INT_R) &
			~(0xFFFFFFFF << KT2_PLL_CTRL_REG_3__NDIV_INT_WIDTH));

		/* read channel 1 mdiv */
		val = readl(KT2_PLL_CTRL_REG_5_ADDR(wrap_base));
		mdiv = ((val >> KT2_PLL_CTRL_REG_5__CH1_MDIV_R) &
			~(0xFFFFFFFF << KT2_PLL_CTRL_REG_5__CH1_MDIV_WIDTH));
	}

	refclk = (25 * ndiv) / mdiv;

	return refclk;
}

static void hx4_clk_setup(void __iomem *wrap_base)
{
	u32 val, ndiv;
	void __iomem *wrap_addr = HX4_WRAP_USBPHY_CTRL_ADDR(wrap_base);

	val = readl(HX4_WRAP_XGPLL_CTRL_4_ADDR(wrap_base));
	ndiv = ((val >> HX4_XGPLL_CTRL_4__NDIV_INT_R) &
	       ~(0xFFFFFFFF << HX4_XGPLL_CTRL_4__NDIV_INT_WIDTH));

	if (ndiv == IPROC_CLK_NDIV_40) {
		val = readl(wrap_addr);
		val = (val & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_40;
		writel(val, wrap_addr);
		udelay(10);
		val = (val & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_40;
		writel(val, wrap_addr);
		udelay(10);
		val = (val & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_40;
		writel(val, wrap_addr);
		udelay(10);
	} else if (ndiv == IPROC_CLK_NDIV_20) {
		val = readl(wrap_addr);
		val = (val & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_20;
		writel(val, wrap_addr);
		udelay(10);
		val = (val & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_20;
		writel(val, wrap_addr);
		udelay(10);
		val = (val & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_20;
		writel(val, wrap_addr);
		udelay(10);
	}
}

static int iproc_usb_phy_hx4_config(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	void __iomem *wrap_addr = NULL;
	struct device_node *dn = iproc_usb_data->phy.dev->of_node;
	u32 ndiv, pdiv, miidata;
	u32 val;

	if (!wrap_base)
		return -EINVAL;

	if (iproc_usb_data->usb_mode == IPROC_USB_MODE_DEVICE) {
		ndiv = 1920 / _get_usb_clk(iproc_usb_data);
		pdiv = 1 << 12;
		miidata = pdiv + ndiv;

		/* Program NDIV and PDIV into 0x1C register */
		xgs_phy_wr_reg(iproc_usb_data->mdio_phy, 0x800c, miidata);
		mdelay(10);

		/* Program other PLL parameters into 0x1D register, disable suspend and
		put PHY into reset */
		miidata = 1 << 13 | 3 << 8 | 3 << 4 | 0xa;
		xgs_phy_wr_reg(iproc_usb_data->mdio_phy, 0x800d, miidata);
		mdelay(10);

		/* Program register 0x15, USB device mode set and get PHY out
		of reset */
		miidata = 1 << 2 | 1 << 1;
		xgs_phy_wr_reg(iproc_usb_data->mdio_phy, 0x8005, miidata);
		mdelay(10);

		/* Program register 0x19, set mdio mode */
		miidata = 1 << 7;
		xgs_phy_wr_reg(iproc_usb_data->mdio_phy, 0x8009, miidata);
		mdelay(10);

		/* get the PLL out of reset */
		miidata = xgs_phy_rd_reg(iproc_usb_data->mdio_phy, 0x800d);
		miidata |= (1 << 12);
		xgs_phy_wr_reg(iproc_usb_data->mdio_phy, 0x800d, miidata);
		mdelay(10);
	} else {
		if (of_device_is_compatible(dn, "brcm,usb-phy-hx4"))
			wrap_addr = HX4_WRAP_USBPHY_CTRL_ADDR(wrap_base);
		else if (of_device_is_compatible(dn, "brcm,usb-phy-kt2"))
			wrap_addr = KT2_WRAP_USBPHY_CTRL_ADDR(wrap_base);

		val = readl(wrap_addr);
		/* PLL_RESETB = 1 */
		val |= BIT(24);
		writel(val, wrap_addr);

		mdelay(10);

		/* check pll_lock */
		check_usbphy_pll_lock(iproc_usb_data);

		val = readl(wrap_addr);
		/* RESETB = 0 */
		val &= ~BIT(23);
		writel(val, wrap_addr);
		mdelay(100);

		if (of_device_is_compatible(dn, "brcm,usb-phy-hx4"))
			hx4_clk_setup(wrap_base);

		val = readl(wrap_addr);
		/* RESETB = 1 */
		val |= BIT(23);
		writel(val, wrap_addr);
		mdelay(1);
	}

	return 0;
}

static int iproc_usb_phy_sb2_config(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	u32 val;

	if (!wrap_base)
	    return -EINVAL;

	val = readl(SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val |= 0x0c000000;      /* 27:PHY_ISO & 26:PLL_SUSPEND_EN = 1 */
	writel(val, SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~0x03000000;     /* 25:PLL_RESETB & 24:RESETB = 0 */
	writel(val, SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	val = readl(SB2_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val &= ~0x03000000;     /* 25:AFE_BG_PWRDWNB & 24:AFE_LDO_PWRDWNB = 0 */
	writel(val, SB2_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	udelay(10);
	val |= 0x02000000;      /* 25:AFE_BG_PWRDWNB = 1 */
	writel(val, SB2_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	udelay(150);
	val |= 0x01000000;      /* 24:AFE_LDO_PWRDWNB = 1 */
	writel(val, SB2_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	udelay(160);

	val = readl(SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~0x08000000;     /* 27:PHY_ISO = 0 */
	writel(val, SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	udelay(20);
	val |= 0x02000000;      /* 25:PLL_RESETB = 1 */
	writel(val, SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	mdelay(20);

	/* check pll_lock */
	check_usbphy_pll_lock(iproc_usb_data);

	val = readl(SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val |= 0x01000000;      /* 24:RESETB = 1 */
	writel(val, SB2_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	udelay(2);

	/* adjust tx amplitude */
	xgs_sb2_usb_phy_wr_reg(iproc_usb_data->mdio_phy, 0x80aa, 0xeea0);

	return 0;
}

static int iproc_usb_phy_gh_config(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	u32 val;

	if (!wrap_base)
	    return -EINVAL;

	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	/* set phy_resetb to 0, pll_resetb to 0 */
	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_0__RESETB;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	/* set p1ctl[11] to 0 */
	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	/* set phy_iso to 0 */
	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	/* set phy_iddq to 0 */
	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	mdelay(1);

	/* set pll_resetb to 1, phy_resetb to 1 */
	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	val = readl(GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_0__RESETB;
	writel(val, GH_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

	mdelay(20);

	/* check pll_lock */
	 check_usbphy_pll_lock(iproc_usb_data);

	/* set non_drving to 0 */
	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val &= ~IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	/* set p1ctl[11] to 1 */
	val = readl(GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	val |= IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11;
	writel(val, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

	return 0;
}

static int iproc_usb_phy_gh2_config(struct iproc_usb_priv *iproc_usb_data)
{
	void __iomem *wrap_base = iproc_usb_data->wrap_base;
	void __iomem *utmi_base = NULL;
	u32 val;

	if (!wrap_base)
		return -EINVAL;

	/* This value is from DE team to set Internal Power Sequence Mode */
	val = readl(IPROC_WRAP_TOP_STRAP_STATUS_ADDR(wrap_base));
	if (val & IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL) {
		/* host mode */
		utmi_base = iproc_usb_data->utmi_base;
		if (!utmi_base)
			return -EINVAL;
		writel(0x0802, USBH_Utmi_p0Ctl(utmi_base));
	} else {
		/* device mode */
		writel(0x0806, GH_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
	}

	mdelay(20);

	/* check pll_lock */
	check_usbphy_pll_lock(iproc_usb_data);

	return 0;
}

static int iproc_usb_phy_init(struct usb_phy *phy)
{
	struct iproc_usb_priv *iproc_usb_data =
				container_of(phy, struct iproc_usb_priv, phy);
	struct device *dev = phy->dev;
	void __iomem *idm_base = iproc_usb_data->idm_base;
	int ret = 0;
	u32 val;

	if (iproc_usb_data->init_count)
		return 0;
	else
		iproc_usb_data->init_count++;

	if (!iproc_usb_data->wrap_base || !iproc_usb_data->idm_base)
		return -EINVAL;

	/* Put USB controller into reset state and disable clock */
	val = readl(USB2_IDM_RESET_CONTROL_ADDR(idm_base));
	val |= USB2_IDM_RESET_CONTROL__RESET;
	writel(val, USB2_IDM_RESET_CONTROL_ADDR(idm_base));

	val = readl(USB2_IDM_IO_CONTROL_DIRECT_ADDR(idm_base));
	val &= ~USB2_IDM_IO_CONTROL_DIRECT__clk_enable;
	writel(val, USB2_IDM_IO_CONTROL_DIRECT_ADDR(idm_base));

	if (of_device_is_compatible(dev->of_node, "brcm,usb-phy-hx4") ||
		of_device_is_compatible(dev->of_node, "brcm,usb-phy-kt2"))
		ret = iproc_usb_phy_hx4_config(iproc_usb_data);
	else if (of_device_is_compatible(dev->of_node, "brcm,usb-phy-sb2"))
		ret = iproc_usb_phy_sb2_config(iproc_usb_data);
	else if (of_device_is_compatible(dev->of_node, "brcm,usb-phy-gh") ||
		of_device_is_compatible(dev->of_node, "brcm,usb-phy-hr3"))
		ret = iproc_usb_phy_gh_config(iproc_usb_data);

	/* Enable clock to USB and get the USB out of reset  */
	val = readl(USB2_IDM_IO_CONTROL_DIRECT_ADDR(idm_base));
	val |= USB2_IDM_IO_CONTROL_DIRECT__clk_enable;
	writel(val, USB2_IDM_IO_CONTROL_DIRECT_ADDR(idm_base));

	mdelay(10);
	val = readl(USB2_IDM_RESET_CONTROL_ADDR(idm_base));
	val &= ~USB2_IDM_RESET_CONTROL__RESET;
	writel(val, USB2_IDM_RESET_CONTROL_ADDR(idm_base));
	mdelay(100);

	/* For GH2, PHY should be inited after RESET */
	if (of_device_is_compatible(dev->of_node, "brcm,usb-phy-gh2"))
		ret = iproc_usb_phy_gh2_config(iproc_usb_data);

	return ret;
}

static int xgs_iproc_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *mdio_phy_np = NULL;
	struct iproc_usb_priv *iproc_usb_data;
	struct resource *res;
	int gpio_pin;
	enum of_gpio_flags flags;
	u32 gpio_active_low;
	int ret, usb_mode;

	if (!of_device_is_available(dn))
		return -ENODEV;

	iproc_usb_data = devm_kzalloc(dev, sizeof(*iproc_usb_data), GFP_KERNEL);
	if (!iproc_usb_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, iproc_usb_data);

	iproc_usb_data->wrap_base = get_iproc_wrap_ctrl_base();
	if (!iproc_usb_data->wrap_base) {
		dev_err(dev, "No wrap_base addr in DT");
		return -ENXIO;
	}

	gpio_pin = of_get_named_gpio_flags(dn, "vbus-gpio", 0, &flags);
	if (gpio_pin < 0) {
		dev_err(dev, "No gpio pin set for USB power\n");
		return gpio_pin;
	}

	gpio_active_low = flags & OF_GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(dev, gpio_pin, "usbphy-vbus");
	if (ret != 0) {
		dev_err(dev, "Failed to request gpio #%d\n", gpio_pin);
		return ret;
	}

	iproc_usb_data->phy.dev = dev;
	usb_mode = xgs_iproc_usb_phy_mode(iproc_usb_data);

	iproc_usb_data->usb_mode = usb_mode;
	/* Save host/device mode in phy.flags for use by ECHI/OHCI drivers */
	iproc_usb_data->phy.flags = usb_mode;
	iproc_usb_data->phy.init = iproc_usb_phy_init;
	iproc_usb_data->phy.type = USB_PHY_TYPE_USB2;

	if (usb_mode == IPROC_USB_MODE_HOST) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		iproc_usb_data->idm_base =
			devm_ioremap_resource(dev, res);
		if (IS_ERR(iproc_usb_data->idm_base)) {
			dev_err(dev, "can't iomap usb2h idm base\n");
			return PTR_ERR(iproc_usb_data->idm_base);
		}

		/* required for GH2 */
		if (of_device_is_compatible(dev->of_node, "brcm,usb-phy-gh2")) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
			iproc_usb_data->utmi_base =
				devm_ioremap_resource(dev, res);
		}

		gpio_direction_output(gpio_pin, 1);
		/* turn off the power: if active low, then set 1 to turn off */
		if (gpio_active_low)
			gpio_set_value(gpio_pin, 1);
		else
			gpio_set_value(gpio_pin, 0);
	} else {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		iproc_usb_data->idm_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(iproc_usb_data->idm_base)) {
			dev_err(dev, "can't iomap usb2d idm base\n");
			return PTR_ERR(iproc_usb_data->idm_base);
		}
	}


	/* PHY controlled through MDIO */
	mdio_phy_np = of_parse_phandle(dn, "mdio-phy-handle", 0);
	if (mdio_phy_np)
		iproc_usb_data->mdio_phy = of_phy_find_device(mdio_phy_np);

	ret = usb_add_phy_dev(&iproc_usb_data->phy);
	if (ret)
		return ret;

	/* supply power for USB device connected to the host */
	if (usb_mode == IPROC_USB_MODE_HOST) {
		if (gpio_active_low)
			gpio_set_value(gpio_pin, 0);
		else
			gpio_set_value(gpio_pin, 1);
	}

	devm_gpio_free(dev, gpio_pin);

	return 0;
}

static int xgs_iproc_usb_phy_remove(struct platform_device *pdev)
{
	struct iproc_usb_priv *iproc_usb_data = platform_get_drvdata(pdev);

	if (iproc_usb_data)
		usb_remove_phy(&iproc_usb_data->phy);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id xgs_iproc_usb_phy_dt_ids[] = {
	{ .compatible = "brcm,usb-phy-hx4", },
	{ .compatible = "brcm,usb-phy-kt2", },
	{ .compatible = "brcm,usb-phy-gh", },
	{ .compatible = "brcm,usb-phy-sb2", },
	{ .compatible = "brcm,usb-phy-hr3", },
	{ .compatible = "brcm,usb-phy-gh2", },
	{ }
};
MODULE_DEVICE_TABLE(of, xgs_iproc_usb_phy_dt_ids);

static struct platform_driver xgs_iproc_usb_phy_driver =
{
	.driver = {
		.name = "xgs-usb-phy",
		.owner = THIS_MODULE,
		.of_match_table = xgs_iproc_usb_phy_dt_ids,
	},
	.probe = xgs_iproc_usb_phy_probe,
	.remove = xgs_iproc_usb_phy_remove,
};

module_platform_driver(xgs_iproc_usb_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB phy driver");
MODULE_LICENSE("GPL");
