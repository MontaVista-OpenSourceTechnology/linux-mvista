/*
 * AC5 Watchdog driver
 *
 * Copyright (c) 2015, Linaro Ltd.
 * Author: Fu Wei <fu.wei@linaro.org>
 *         Suravee Suthikulpanit <Suravee.Suthikulpanit@amd.com>
 *         Al Stone <al.stone@linaro.org>
 *         Timur Tabi <timur@codeaurora.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AC5 Watchdog has two stage timeouts:
 * the first signal (WS0) is for alerting the system by interrupt,
 * the second one (WS1) is a real hardware reset.
 * More details about the hardware specification of this device:
 * ARM DEN0029B - Server Base System Architecture (SBSA)
 *
 * This driver can operate as a single stage watchdog
 * or a two stages watchdog, it's set up by the module parameter "action".
 * In the single stage mode, when the timeout is reached, your system
 * will be reset by WS1. The first signal (WS0) is ignored.
 * In the two stages mode, when the timeout is reached, the first signal (WS0)
 * will trigger panic. If the system is getting into trouble and cannot be reset
 * by panic or restart properly by the kdump kernel(if supported), then the
 * second stage (as long as the first stage) will be reached, system will be
 * reset by WS1. This function can help administrator to backup the system
 * context info by panic console output or kdump.
 *
 * SBSA GWDT:
 * if action is 1 (the two stages mode):
 * |--------WOR-------WS0--------WOR-------WS1
 * |----timeout-----(panic)----timeout-----reset
 *
 * if action is 0 (the single stage mode):
 * |------WOR-----WS0(ignored)-----WOR------WS1
 * |--------------timeout-------------------reset
 *
 * Note: Since this watchdog timer has two stages, and each stage is determined
 * by WOR, in the single stage mode, the timeout is (WOR * 2); in the two
 * stages mode, the timeout is WOR. The maximum timeout in the two stages mode
 * is half of that in the single stage mode.
 *
 */

#include <linux/io.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <asm/arch_timer.h>
#include <linux/arm-smccc.h>

/* AC5 SMCs, taken from ATF include/services/arm_arch_svc.h */
#define SMC_FID_READ_REG	0x80007FFE
#define SMC_FID_WRITE_REG	0x80007FFD

#define DRV_NAME		"ac5-gwd"
#define WATCHDOG_NAME	"AC5 Watchdog"

/* SBSA Generic Watchdog register definitions */
/* refresh frame */
#define SBSA_GWDT_WRR		0x000

/* control frame */
#define SBSA_GWDT_WCS		0x000
#define SBSA_GWDT_WOR		0x008
#define SBSA_GWDT_WCV		0x010

/* refresh/control frame */
#define SBSA_GWDT_W_IIDR	0xfcc
#define SBSA_GWDT_IDR		0xfd0

/* Watchdog Control and Status Register */
#define SBSA_GWDT_WCS_EN	BIT(0)
#define SBSA_GWDT_WCS_WS0	BIT(1)
#define SBSA_GWDT_WCS_WS1	BIT(2)

/**
 * struct sbsa_gwdt - Internal representation of the SBSA GWDT
 * @wdd:		kernel watchdog_device structure
 * @clk:		store the System Counter clock frequency, in Hz.
 * @refresh_base:	Virtual address of the watchdog refresh frame
 * @control_base:	Virtual address of the watchdog control frame
 */
struct sbsa_gwdt {
	struct watchdog_device	wdd;
	u32		clk;
	u32		refresh_base;
	u32 	control_base;
};

#define DEFAULT_TIMEOUT		10 /* seconds */

static unsigned int timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout,
		 "Watchdog timeout in seconds. (>=0, default="
		 __MODULE_STRING(DEFAULT_TIMEOUT) ")");

/*
 * action refers to action taken when watchdog gets WS0
 * 0 = skip
 * 1 = panic
 * defaults to skip (0)
 */
static int action;
module_param(action, int, 0);
MODULE_PARM_DESC(action, "after watchdog gets WS0 interrupt, do: "
		 "0 = skip(*)  1 = panic");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, S_IRUGO);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static inline u32 smc_readl(unsigned int addr)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(SMC_FID_READ_REG,  addr, 0,    0, 0, 0, 0, 0, &smc_res);
	return (u32)smc_res.a0;
}

static inline void smc_writel(unsigned int val, unsigned int addr)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(SMC_FID_WRITE_REG, addr, val, 0, 0, 0, 0, 0, &smc_res);
}

/*
 * watchdog operation functions
 */
static int sbsa_gwdt_set_timeout(struct watchdog_device *wdd,
				 unsigned int timeout)
{
	struct sbsa_gwdt *gwdt = watchdog_get_drvdata(wdd);

	wdd->timeout = timeout;

	if (action)
		smc_writel(gwdt->clk * timeout,
		       gwdt->control_base + SBSA_GWDT_WOR);
	else
		/*
		 * In the single stage mode, The first signal (WS0) is ignored,
		 * the timeout is (WOR * 2), so the WOR should be configured
		 * to half value of timeout.
		 */
		smc_writel(gwdt->clk / 2 * timeout,
		       gwdt->control_base + SBSA_GWDT_WOR);

	return 0;
}

static inline u64 lo_hi_smc_readq(unsigned int addr)
{
	u32 low, high;

	low = smc_readl(addr);
	high = smc_readl(addr + 4);
	low = smc_readl(addr); /* read twice, as a workaround to HW limitation */

	return low + ((u64)high << 32);
}


static unsigned int sbsa_gwdt_get_timeleft(struct watchdog_device *wdd)
{
	struct sbsa_gwdt *gwdt = watchdog_get_drvdata(wdd);
	u64 timeleft = 0;

	/*
	 * In the single stage mode, if WS0 is deasserted
	 * (watchdog is in the first stage),
	 * timeleft = WOR + (WCV - system counter)
	 */
	if (!action &&
	    !(smc_readl(gwdt->control_base + SBSA_GWDT_WCS) & SBSA_GWDT_WCS_WS0))
		timeleft += smc_readl(gwdt->control_base + SBSA_GWDT_WOR);

	timeleft += lo_hi_smc_readq(gwdt->control_base + SBSA_GWDT_WCV) -
		    arch_timer_read_counter();

	do_div(timeleft, gwdt->clk);

	return timeleft;
}

static int sbsa_gwdt_keepalive(struct watchdog_device *wdd)
{
	struct sbsa_gwdt *gwdt = watchdog_get_drvdata(wdd);

	/*
	 * Writing WRR for an explicit watchdog refresh.
	 * You can write anyting (like 0).
	 */
	smc_writel(0, gwdt->refresh_base + SBSA_GWDT_WRR);

	return 0;
}

static int sbsa_gwdt_start(struct watchdog_device *wdd)
{
	struct sbsa_gwdt *gwdt = watchdog_get_drvdata(wdd);

	/* writing WCS will cause an explicit watchdog refresh */
	smc_writel(SBSA_GWDT_WCS_EN, gwdt->control_base + SBSA_GWDT_WCS);

	return 0;
}

static int sbsa_gwdt_stop(struct watchdog_device *wdd)
{
	struct sbsa_gwdt *gwdt = watchdog_get_drvdata(wdd);

	/* Simply write 0 to WCS to clean WCS_EN bit */
	smc_writel(0, gwdt->control_base + SBSA_GWDT_WCS);

	return 0;
}

static irqreturn_t sbsa_gwdt_interrupt(int irq, void *dev_id)
{
	panic(WATCHDOG_NAME " timeout");

	return IRQ_HANDLED;
}

static const struct watchdog_info sbsa_gwdt_info = {
	.identity	= WATCHDOG_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE |
			  WDIOF_CARDRESET,
};

static const struct watchdog_ops sbsa_gwdt_ops = {
	.owner		= THIS_MODULE,
	.start		= sbsa_gwdt_start,
	.stop		= sbsa_gwdt_stop,
	.ping		= sbsa_gwdt_keepalive,
	.set_timeout	= sbsa_gwdt_set_timeout,
	.get_timeleft	= sbsa_gwdt_get_timeleft,
};

static int sbsa_gwdt_probe(struct platform_device *pdev)
{
#define AC5_ID      0xB400
#define AC5X_ID     0x9800
#define IML_ID      0xA000
#define IMM_ID      0xA200

#define WRITE_MASK(addr, mask, val)     \
	writel( (readl(addr) & (~(mask))) | val, addr);

#define MAP_CLR_BIT(addr, bit)  \
	reg = devm_ioremap(dev, addr, 1); \
	if (IS_ERR(reg))    \
		return PTR_ERR(reg); \
	WRITE_MASK(reg, bit, 0); \

	u32 rf_base, cf_base;
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct sbsa_gwdt *gwdt;
	struct resource *res;
	int ret, irq;
	void *__iomem reg;
	u32 status;

	gwdt = devm_kzalloc(dev, sizeof(*gwdt), GFP_KERNEL);
	if (!gwdt)
		return -ENOMEM;
	platform_set_drvdata(pdev, gwdt);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res))
		return PTR_ERR(res);
	cf_base = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (IS_ERR(res))
		return PTR_ERR(res);
	rf_base = res->start;

	/*
	 * Get the frequency of system counter from the cp15 interface of ARM
	 * Generic timer. We don't need to check it, because if it returns "0",
	 * system would panic in very early stage.
	 */
	gwdt->clk = arch_timer_get_cntfrq();
	gwdt->refresh_base = rf_base;
	gwdt->control_base = cf_base;

	wdd = &gwdt->wdd;
	wdd->parent = dev;
	wdd->info = &sbsa_gwdt_info;
	wdd->ops = &sbsa_gwdt_ops;
	wdd->min_timeout = 1;
	wdd->max_hw_heartbeat_ms = U32_MAX / gwdt->clk * 1000;
	wdd->timeout = DEFAULT_TIMEOUT;
	watchdog_set_drvdata(wdd, gwdt);
	watchdog_set_nowayout(wdd, nowayout);

	status = smc_readl(cf_base + SBSA_GWDT_WCS);
	if (status & SBSA_GWDT_WCS_WS1) {
		dev_warn(dev, "System reset by WDT.\n");
		wdd->bootstatus |= WDIOF_CARDRESET;
	}
	if (status & SBSA_GWDT_WCS_EN)
		set_bit(WDOG_HW_RUNNING, &wdd->status);

	if (action) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			action = 0;
			dev_warn(dev, "unable to get ws0 interrupt.\n");
		} else {
			/*
			 * In case there is a pending ws0 interrupt, just ping
			 * the watchdog before registering the interrupt routine
			 */
			smc_writel(0, rf_base + SBSA_GWDT_WRR);
			if (devm_request_irq(dev, irq, sbsa_gwdt_interrupt, 0,
					     pdev->name, gwdt)) {
				action = 0;
				dev_warn(dev, "unable to request IRQ %d.\n",
					 irq);
			}
		}
		if (!action)
			dev_warn(dev, "falling back to single stage mode.\n");
	}
	/*
	 * In the single stage mode, The first signal (WS0) is ignored,
	 * the timeout is (WOR * 2), so the maximum timeout should be doubled.
	 */
	if (!action)
		wdd->max_hw_heartbeat_ms *= 2;

	watchdog_init_timeout(wdd, timeout, dev);
	/*
	 * Update timeout to WOR.
	 * Because of the explicit watchdog refresh mechanism,
	 * it's also a ping, if watchdog is enabled.
	 */
	sbsa_gwdt_set_timeout(wdd, wdd->timeout);

	watchdog_stop_on_reboot(wdd);
	ret = devm_watchdog_register_device(dev, wdd);
	if (ret)
		return ret;

	/* CPSS-14280: 	WD HW need to trigger reset on WS1.
	   Enable GWD reset out */
	smc_writel(0xFFFFFFBF, 0x80210030);

	reg = devm_ioremap(dev, 0x7f90004c, 1);
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	switch ((readl(reg) >> 4) & 0xFF00) {
	case AC5_ID:
		MAP_CLR_BIT(0x840F800C, BIT(6));
		break;
	case AC5X_ID:
		MAP_CLR_BIT(0x944F800C, BIT(7));
		break;
	case IML_ID:
	case IMM_ID:
		MAP_CLR_BIT(0x840F800C, BIT(7));
		break;
	default:
		dev_warn(dev, "Failed to enable DFX reset path\n");
	}

	dev_info(dev, "Initialized with %ds timeout @ %u Hz, action=%d.%s\n",
		 wdd->timeout, gwdt->clk, action,
		 status & SBSA_GWDT_WCS_EN ? " [enabled]" : "");

	return 0;
}

/* Disable watchdog if it is active during suspend */
static int __maybe_unused sbsa_gwdt_suspend(struct device *dev)
{
	struct sbsa_gwdt *gwdt = dev_get_drvdata(dev);

	if (watchdog_active(&gwdt->wdd))
		sbsa_gwdt_stop(&gwdt->wdd);

	return 0;
}

/* Enable watchdog if necessary */
static int __maybe_unused sbsa_gwdt_resume(struct device *dev)
{
	struct sbsa_gwdt *gwdt = dev_get_drvdata(dev);

	if (watchdog_active(&gwdt->wdd))
		sbsa_gwdt_start(&gwdt->wdd);

	return 0;
}

static const struct dev_pm_ops sbsa_gwdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sbsa_gwdt_suspend, sbsa_gwdt_resume)
};

static const struct of_device_id sbsa_gwdt_of_match[] = {
	{ .compatible = "marvell,ac5-wd", },
	{},
};
MODULE_DEVICE_TABLE(of, sbsa_gwdt_of_match);

static const struct platform_device_id sbsa_gwdt_pdev_match[] = {
	{ .name = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(platform, sbsa_gwdt_pdev_match);

static struct platform_driver ac5_gwdt_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &sbsa_gwdt_pm_ops,
		.of_match_table = sbsa_gwdt_of_match,
	},
	.probe = sbsa_gwdt_probe,
	.id_table = sbsa_gwdt_pdev_match,
};

module_platform_driver(ac5_gwdt_driver);

MODULE_DESCRIPTION("AC5 Watchdog Driver");
MODULE_AUTHOR("Noam Liron <lnoam@marvell.com>");
MODULE_AUTHOR("Fu Wei <fu.wei@linaro.org>");
MODULE_AUTHOR("Suravee Suthikulpanit <Suravee.Suthikulpanit@amd.com>");
MODULE_AUTHOR("Al Stone <al.stone@linaro.org>");
MODULE_AUTHOR("Timur Tabi <timur@codeaurora.org>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
