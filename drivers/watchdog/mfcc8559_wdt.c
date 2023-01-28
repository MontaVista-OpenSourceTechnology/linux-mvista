/*
 * mfcc8559_wdt.c - MMSI BMC watchdog driver.
 *
 *  Copyright (C) 2020 Mercury Mission Systems International SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/watchdog.h>
#include <linux/of.h>

#define WDT_VERSION "0.0.2"

#define WDT_REG_ID_VALUE 0x574447U /* "WDG" */

#define WDT_DEFAULT_TIMEOUT     60U  /* seconds */
#define WDT_MAX_TIMEOUT         65U  /* (1<<16) / 1000 seconds */
#define WDT_MIN_TIMEOUT         1U   /* seconds */

#define WDT_REG_VALUE            0x0000 /* RO, current value in  ms */
#define WDT_REG_TIMEOUT          0x0004 /* RW, timeout value in ms */
#define WDT_REG_CTRL             0x0008 /* RW */
/* #define WDT_REG_STATUS           0x000c */ /* RO, bit 0 expired */
#define WDT_REG_ID               0x0010
#define WDT_REG_REV              0x0014

/* WDT_REG_CTRL register bits */
#define WDT_REG_CTRL_ENABLE     (1U << 0)
#define WDT_REG_CTRL_RESET      (1U << 1) /* refresh the counter to 0 */
/* #define WDT_REG_CTRL_TRIG       (1 << 2) */ /* trigger the reset */
/* #define WDT_REG_CTRL_TEST       (1 << 3) */ /* disable reset signal */
/* #define WDT_REG_CTRL_MODE       (1 << 4) */ /* 0 reset mode, 1 interrupt mode */

/* flags bits */
#define FLAG_HW_WDT             (1U << 0)
#define FLAG_NOWAYOUT           (1U << 1)

struct wdt_device {
	u8 __iomem *base;
	struct watchdog_device wdt_dev;
	u32 flags;
	u32 timeout;
	struct device *dev;
};

static int wdt_is_running(struct wdt_device *w)
{
	u32 v = ioread32(w->base + WDT_REG_CTRL);
	return ((v & WDT_REG_CTRL_ENABLE) == WDT_REG_CTRL_ENABLE);
}

static int wdt_ping(struct watchdog_device *wdt)
{
	struct wdt_device *w = watchdog_get_drvdata(wdt);
	u32 v = ioread32(w->base + WDT_REG_CTRL);

	if ((v & WDT_REG_CTRL_ENABLE) == 0)
		return -EPERM;

	v |= WDT_REG_CTRL_RESET;
	iowrite32(v, w->base + WDT_REG_CTRL);
	dev_dbg(w->dev, "%s flags %x\n", __func__, w->flags);
	return 0;
}

static int wdt_set_timeout(struct watchdog_device *wdt, unsigned int timeout)
{
	struct wdt_device *w = watchdog_get_drvdata(wdt);

	/* "hardware" watchog has a fixed timeout */
	if (w->flags & FLAG_HW_WDT)
		return -ENOTSUPP;

	iowrite32(timeout * 1000U, w->base + WDT_REG_TIMEOUT);
	w->timeout = timeout;
	dev_dbg(w->dev, "%s %d\n", __func__, timeout);
	return 0;
}

static unsigned int wdt_get_timeleft(struct watchdog_device *wdt)
{
	struct wdt_device *w = watchdog_get_drvdata(wdt);
	u32 v = ioread32(w->base + WDT_REG_VALUE);
	dev_dbg(w->dev, "%s %d\n", __func__, v);
	return v / 1000;
}

static int wdt_stop(struct watchdog_device *wdt)
{
	struct wdt_device *w = watchdog_get_drvdata(wdt);
	u32 v = ioread32(w->base + WDT_REG_CTRL);
	v &= ~WDT_REG_CTRL_ENABLE;
	iowrite32(v, w->base + WDT_REG_CTRL);
	dev_dbg(w->dev, "%s flags %x\n", __func__, w->flags);
	return 0;
}

static int wdt_start(struct watchdog_device *wdt)
{
	struct wdt_device *w = watchdog_get_drvdata(wdt);
	u32 v = ioread32(w->base + WDT_REG_CTRL);
	v |= WDT_REG_CTRL_ENABLE;
	iowrite32(v, w->base + WDT_REG_CTRL);
	dev_dbg(w->dev, "%s flags %x\n", __func__, w->flags);
	return 0;
}

static const struct watchdog_info wdt_info = {
	.identity	= KBUILD_MODNAME,
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			      WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= wdt_start,
	.stop		= wdt_stop,
	.ping		= wdt_ping,
	.set_timeout	= wdt_set_timeout,
	.get_timeleft   = wdt_get_timeleft
};

static int wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wdt_device *w;
	struct resource *mem;
	int ret;
	u32 v, m;
	struct device_node *n;

	w = devm_kzalloc(dev, sizeof(struct wdt_device), GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	w->dev = dev;

	ret = of_property_read_u32(dev->of_node, "hw-wdog", &v);
	if ((ret == 0) || (v == 1U)) {
		w->flags |= FLAG_HW_WDT;
	} else {
		/* configurable watchdog */
		if (of_property_read_u32(dev->of_node, "timeout-secs",
					 &w->timeout)) {
			w->timeout = WDT_DEFAULT_TIMEOUT;
			dev_info(dev, "use default timeout %d\n", w->timeout);
		}

		n = of_get_child_by_name(dev->of_node, "nowayout");
		if (n)
			w->flags |= FLAG_NOWAYOUT;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no resource to map\n");
		return -ENODEV;
	}
	w->base = devm_ioremap_resource(dev, mem);
	if (IS_ERR(w->base))
		return PTR_ERR(w->base);

	v = ioread32(w->base + WDT_REG_ID);
	/* cppcheck-suppress misra-c2012-10.1; character standard */
	if ((v & 0xffffffU) != WDT_REG_ID_VALUE) {
		dev_err(dev, "invalid wdg_id\n");
		return -ENOTSUPP;
	}
	m = (w->flags & FLAG_HW_WDT) ? 'H' : 'P';
	if ((v >> 24) != m) {
		dev_err(dev, "unexpected watchdog type %x %x\n", v, m);
		return -ENOTSUPP;
	}

	w->wdt_dev.info = &wdt_info;
	w->wdt_dev.ops = &wdt_ops;
	w->wdt_dev.timeout = w->timeout;
	if (w->flags & FLAG_HW_WDT) {
		w->timeout = ioread32(w->base + WDT_REG_TIMEOUT) / 1000;
		w->wdt_dev.max_timeout = w->timeout;
		w->wdt_dev.min_timeout = w->timeout;
	} else {
		w->wdt_dev.max_timeout = WDT_MAX_TIMEOUT;
		w->wdt_dev.min_timeout = WDT_MIN_TIMEOUT;

		iowrite32(w->timeout * 1000, w->base + WDT_REG_TIMEOUT);
	}
	w->wdt_dev.parent = &pdev->dev;

	watchdog_init_timeout(&w->wdt_dev, w->timeout, &pdev->dev);
	watchdog_set_nowayout(&w->wdt_dev, (w->flags & FLAG_NOWAYOUT) ? 1 : 0);
	watchdog_set_drvdata(&w->wdt_dev, w);

	if (wdt_is_running(w))
		set_bit(WDOG_HW_RUNNING, &w->wdt_dev.status);

	ret = watchdog_register_device(&w->wdt_dev);
	if (unlikely(ret))
		return ret;

	platform_set_drvdata(pdev, w);

	v = ioread32(w->base + WDT_REG_REV);
	dev_info(dev, "probed sw %s, fw %d.%d.%d, timeout %d s, running %d\n",
		 WDT_VERSION, (v >> 16) & 0xff, (v >> 8) & 0xff, v & 0xff,
		 w->timeout, wdt_is_running(w));

	return 0;
}

static int wdt_remove(struct platform_device *pdev)
{
	struct wdt_device *w = platform_get_drvdata(pdev);
	if (!w)
		return -ENODEV;

	(void)wdt_stop(&w->wdt_dev);

	watchdog_unregister_device(&w->wdt_dev);
	watchdog_set_drvdata(&w->wdt_dev, NULL);

	dev_info(&pdev->dev, "%s driver removed\n", KBUILD_MODNAME);
	return 0;
}

/* cppcheck-suppress misra-c2012-9.2; proper initializer */
static const struct of_device_id wdt_dt_ids[2] = {
	{ .compatible = "mmsi,ifc-wdog" },
	{ }
};
MODULE_DEVICE_TABLE(of, wdt_dt_ids);

static struct platform_driver wdt_platform_driver = {
	.driver = {
		.name           = KBUILD_MODNAME,
		.of_match_table = wdt_dt_ids,
		.owner          = THIS_MODULE,
	},
	.probe  = wdt_probe,
	.remove = wdt_remove,
};

static int __init bmc_wdt_init(void)
{
	return platform_driver_register(&wdt_platform_driver);
}

static void __exit bmc_wdt_exit(void)
{
	platform_driver_unregister(&wdt_platform_driver);
}

module_init(bmc_wdt_init);
module_exit(bmc_wdt_exit);

MODULE_AUTHOR("MMSI");
MODULE_DESCRIPTION("MMSI BMC watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(WDT_VERSION);
