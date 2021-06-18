// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/jiffies.h>
#include <asm/system_misc.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define K2_DDR3A_EMIF_CONFIG		 0x21010000
#define K2_DDR3A_EMIF_CONFIG_REGS_SIZE	 0x04
#define K2_DDR3_CTRL_STAT_REG		 (K2_DDR3A_EMIF_CONFIG | 0x004)
#define K2_DDR3_CTRL_PMCTL_REG		 (K2_DDR3A_EMIF_CONFIG | 0x038)
#define K2_DDR3_CTRL_STAT_SELF_REF	 BIT(27)
#define K2_DDR3_CTRL_PMCTL_LP_MODE	 0x0200 /* self-refresh (SR) mode */
#define K2_DDR3_CTRL_PMCTL_SR_TIM	 0x00 /* enter SR mode immediately */
#define LFA_IRQA_ENABLE			 0x90
#define LFA_IRQA_STATUS			 0x0B
#define IRQA_MASK			 BIT(3) | BIT(2) | BIT(0)
#define SYNC_WAIT_TIME			 (20 * HZ)

#define PLL_RESET_WRITE_KEY_MASK	 0xffff0000
#define PLL_RESET_WRITE_KEY		 0x5a69
#define PLL_RESET			 BIT(16)

struct fsp_reset_data {
	struct device *dev;
	struct regmap *rm;
	struct delayed_work wq;
	struct notifier_block nb;
	unsigned long status;
	u32 supervise;
	irqreturn_t (*irq_callback)(int, void *);
	int irq;
};

struct fsp_reset_match_data {
	int (*init)(struct fsp_reset_data*);
};

DEFINE_SPINLOCK(slock);

static irqreturn_t fsp_reset_call_rapid_reboot(struct fsp_reset_data *fr)
{
	int ret;
	char *cmd = "/sbin/reboot";
	char *argv[] = {
		cmd,
		NULL
	};
	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/bin:/usr/bin",
		NULL,
	};

	ret = call_usermodehelper(cmd, argv, envp, UMH_WAIT_EXEC);
	if (ret) {
		dev_err(fr->dev, "can't invoke rapid reboot");
		return IRQ_NONE;
	}

	dev_alert(fr->dev, "board is going to reboot now!");
	return IRQ_HANDLED;
}

static irqreturn_t fsp_reset_irq_k2(int irq, void *data)
{
	struct fsp_reset_data *fr = data;

	return fsp_reset_call_rapid_reboot(fr);
}

static int keystone_reset_init(struct fsp_reset_data *fr)
{
	struct device *dev = fr->dev;
	return 0;
}

static int fsp_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fsp_reset_data *fr;
	const struct of_device_id *id;
	const struct fsp_reset_match_data *data;
	int ret;

	fr = devm_kzalloc(dev, sizeof(*fr), GFP_KERNEL);
	if (!fr)
		return -ENOMEM;

	fr->irq = platform_get_irq(pdev, 0);
	if (fr->irq < 0) {
		pr_err("platform data for irq %d wasn't found\n", fr->irq);
		return fr->irq;
	}

	fr->dev = dev;

	id = of_match_device(dev->driver->of_match_table, dev);
	if (!id) {
		pr_err("failed to match data\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(dev->of_node, "supervise", &fr->supervise);
	if (ret < 0) {
		if (ret != -EINVAL) {
			dev_err(dev, "invalid 'supervise' property's value (%d)",
				ret);
			return ret;
		}

		dev_dbg(dev, "assuming slave role");
	}

	data = id->data;

	ret = data->init(fr);
	if (ret) {
		pr_err("can't initialize device\n");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, fr->irq, NULL, fr->irq_callback,
					IRQF_ONESHOT, dev_name(dev), fr);
	if (ret) {
		dev_err(dev, "request for irq %d failed\n", fr->irq);
		return ret;
	}

	return 0;
}

static const struct fsp_reset_match_data match_data_keystone = {
	.init = keystone_reset_init,
};

static struct of_device_id fsp_reset_match[] = {
	{
		.compatible = "nsn,fsp_reset",
		.data = &match_data_keystone,
	},
	{},
};
MODULE_DEVICE_TABLE(of, fsp_reset_match);

static struct platform_driver fsp_reset_driver = {
	.probe = fsp_reset_probe,
	.driver = {
		   .name = "fsp_reset",
		   .owner = THIS_MODULE,
		   .of_match_table = fsp_reset_match,
	},
};

module_platform_driver(fsp_reset_driver);

MODULE_AUTHOR("wojciech.tesluk@nokia.com");
MODULE_DESCRIPTION("FSP Reset Driver");
MODULE_LICENSE("GPL");

