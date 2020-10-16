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
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <fsmddg_fpga-master.h>

#define FSP_RAM_SELF_REFRESH_MODE	 0x0200
#define FSP_RAM_SELF_REFRESH_ADDRESS	 0x21010038
#define FSP_RAM_SELF_REFRESH_ADDR_SIZE	 0x04
#define RSTMUX8_OMODE_DEVICE_RESET	 5
#define RSTMUX8_OMODE_DEVICE_RESET_SHIFT 1
#define RSTMUX8_OMODE_DEVICE_RESET_MASK	 (BIT(1) | BIT(2) | BIT(3))
#define RSTMUX8_LOCK_MASK		 BIT(0)
#define LFA_IRQA_ENABLE			 0x90
#define LFA_IRQA_STATUS			 0x0B
#define IRQA_MASK			 BIT(3) | BIT(2) | BIT(0)
#define SYNC_WAIT_TIME			 (20 * HZ)

static void __iomem *reset_base;

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

static int fsp_reset_k2(struct notifier_block *this, unsigned long mode, void *cmd)
{
	if (reset_base) {
		writel(FSP_RAM_SELF_REFRESH_MODE, reset_base);
		udelay(300);
	}

	return NOTIFY_DONE;
}

static struct notifier_block fsp_restart_k2 = {
	.notifier_call = fsp_reset_k2,
	.priority = 192,
};

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

	reset_base = devm_ioremap(dev, FSP_RAM_SELF_REFRESH_ADDRESS,
			     FSP_RAM_SELF_REFRESH_ADDR_SIZE);
	if (!reset_base) {
		pr_err("ioremap failure\n");
		return -ENOMEM;
	}

	fr->irq_callback = fsp_reset_irq_k2;

	return register_restart_handler(&fsp_restart_k2);
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

