/*
 * Xylon logiCVC frame buffer driver pixel clock control
 *
 * Copyright (C) 2016 Xylon d.o.o.
 * Author: Davor Joja <davor.joja@logicbricks.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "xylonfb_core.h"

struct xylonfb_pixclk {
	struct device *dev;
	struct device_node *dn;
	struct clk *clk;
};

#if defined(CONFIG_FB_XYLON_PIXCLK_LOGICLK)
static const struct of_device_id logiclk_of_match[] = {
	{ .compatible = "xylon,logiclk-1.02.b" },
	{/* end of table */}
};
static struct xylonfb_pixclk logiclk;
#endif

#if defined(CONFIG_FB_XYLON_PIXCLK_SI570)
static const struct of_device_id si570_of_match[] = {
	{ .compatible = "silabs,si570" },
	{/* end of table */}
};
static struct xylonfb_pixclk si570;
#endif

bool xylonfb_hw_pixclk_supported(struct device *dev, struct device_node *dn);
void xylonfb_hw_pixclk_unload(struct device_node *dn);
int xylonfb_hw_pixclk_set(struct device *dev, struct device_node *dn,
			  unsigned long pixclk_khz);

#if defined(CONFIG_FB_XYLON_PIXCLK)
static int xylonfb_hw_pixclk_set_freq(struct device *dev,
				      struct device_node *dn,
				      unsigned long freq_khz)
{
	struct clk *clk = NULL;

#if defined(CONFIG_FB_XYLON_PIXCLK_LOGICLK)
	if (dn == logiclk.dn)
		clk = logiclk.clk;
#endif

#if defined(CONFIG_FB_XYLON_PIXCLK_SI570)
	if (dn == si570.dn)
		clk = si570.clk;
#endif
	XYLONFB_DBG(INFO, "%s clk %p: freq_khz %d", __func__, clk, freq_khz);

	if (clk && clk_set_rate(clk, (freq_khz * 1000))) {
		dev_err(dev, "failed set pixel clock frequency\n");
		return -EINVAL;
	}

	return 0;
}
#endif

bool xylonfb_hw_pixclk_supported(struct device *dev, struct device_node *dn)
{
#if defined(CONFIG_FB_XYLON_PIXCLK)
	struct clk **clk;
	bool clk_dev = false;

#if defined(CONFIG_FB_XYLON_PIXCLK_LOGICLK)
	if (of_match_node(logiclk_of_match, of_get_parent(dn))) {
		clk = &logiclk.clk;
		logiclk.dev = dev;
		logiclk.dn = dn;
		clk_dev = true;
	}
#endif

#if defined(CONFIG_FB_XYLON_PIXCLK_SI570)
	if (of_match_node(si570_of_match, dn)) {
		clk = &si570.clk;
		si570.dev = dev;
		si570.dn = dn;
		clk_dev = true;
	}
#endif

	if (clk_dev) {
		*clk = devm_clk_get(dev, NULL);
		if (IS_ERR(*clk)) {
			dev_err(dev, "failed get pixel clock\n");
			return false;
		}
		if (clk_prepare_enable(*clk)) {
			dev_err(dev,
				"failed prepare/enable pixel clock\n");
			return false;
		}

		return true;
	} else {
		return false;
	}
#else
	return true;
#endif
}

void xylonfb_hw_pixclk_unload(struct device_node *dn)
{
	struct clk *clk = NULL;

#if defined(CONFIG_FB_XYLON_PIXCLK_LOGICLK)
	clk = logiclk.clk;
#endif
#if defined(CONFIG_FB_XYLON_PIXCLK_SI570)
	clk = si570.clk;
#endif

	if (clk)
		clk_disable_unprepare(clk);
}

int xylonfb_hw_pixclk_set(struct device *dev, struct device_node *dn,
			  unsigned long pixclk_khz)
{
#if defined(CONFIG_FB_XYLON_PIXCLK)
	return xylonfb_hw_pixclk_set_freq(dev, dn, pixclk_khz);
#else
	dev_warn(dev, "pixel clock control not supported\n");
	return -ENODEV;
#endif
}
