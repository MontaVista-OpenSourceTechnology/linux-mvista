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
#include <fsmddg_slave_sup.h>

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

/* XXX: this function should be removed from this module and:
 * a) we should start to use mainline keystone-reset.c, which
 *    configures rsmux through devicetree,
 * b) we should get back to the mainline keystone.c (it now
 *    includes nokia specific code) as soon as point a) is
 *    done.
 */
static int configure_rstmux(void)
{
	struct device_node *node;
	void __iomem *rstmux8;
	u32 val;

	node = of_find_compatible_node(NULL, NULL, "ti,keystone-reset");
	if (WARN_ON(!node))
		pr_warn("ti,keystone-reset node undefined\n");

	/* rstmux8 address is configured in the rstctrl node at index 1 */
	rstmux8 = of_iomap(node, 1);
	if (WARN_ON(!rstmux8)) {
		pr_warn("rstmux8 iomap error\n");
		return -ENODEV;
	}

	val = __raw_readl(rstmux8) & ~RSTMUX8_OMODE_DEVICE_RESET_MASK;
	if (!(val & RSTMUX8_LOCK_MASK)) {
		val |= (RSTMUX8_OMODE_DEVICE_RESET <<
					RSTMUX8_OMODE_DEVICE_RESET_SHIFT);
		__raw_writel(val, rstmux8);
	}
	iounmap(rstmux8);

	return 0;
}

static irqreturn_t fsp_reset_call_rapid_reboot(struct fsp_reset_data *fr)
{
	int ret;
	char *cmd = "/usr/bin/rapid-reboot";
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

static int fsp_reset_lionfish_sync(struct notifier_block *nb, unsigned long val,
				   void *data)
{
	struct fsp_reset_data *fr = container_of(nb, struct fsp_reset_data, nb);

	fr->status |= val;
	if ((fr->status & fr->supervise) == fr->supervise) {
		if (cancel_delayed_work_sync(&fr->wq)) {
			schedule_delayed_work(&fr->wq, 0);
		}
	}

	return NOTIFY_DONE;
}

static void fsp_reset_lionfish(struct work_struct *wq)
{
	struct delayed_work *delayed = to_delayed_work(wq);
	struct fsp_reset_data *fr = container_of(delayed, struct fsp_reset_data,
						 wq);
	unsigned long j = jiffies;
	unsigned long left;

	if (time_before_eq(delayed->timer.expires, j)) {
		dev_alert(fr->dev, "not synced with slaves, mask: 0x%lx",
			  fr->status);
	} else {
		left = SYNC_WAIT_TIME - (delayed->timer.expires - j);
		left = jiffies_to_msecs(left);
		dev_info(fr->dev, "synced with slaves after %lu msec", left);
	}

	fsp_reset_call_rapid_reboot(fr);
}

/* it is currently expected that fsp reset will be done by
 * invoking just 'reboot'. Machine-specific code will take
 * care of saving the RPRAM.
 */
static irqreturn_t fsp_reset_irq_lionfish(int irq, void *data)
{
	struct fsp_reset_data *fr = data;
	unsigned int val;
	int ret;
	const unsigned int reset_l = BIT(3);

	ret = regmap_read(fr->rm, LFA_IRQA_STATUS, &val);
	if (ret < 0) {
		dev_err(fr->dev, "couldn't read IRQA status\n");
		return IRQ_NONE;
	}

	dev_dbg(fr->dev, "LFA IRQA STATUS ->  %#x\n", val);

	/* clear all IRQA occurrences in the status register:
	 * - FCT_B_RST_IRQ - asserted remotely from control board,
	 * - LFA_B_RST_IRQ - asserted locally from LFA
	 * - WD_T1 - asserted by bmuc watchdog
	 */
	ret = regmap_update_bits(fr->rm, LFA_IRQA_STATUS, IRQA_MASK,
				 IRQA_MASK);
	if (ret < 0) {
		dev_err(fr->dev, "couldn't clear IRQA status\n");
		return IRQ_NONE;
	}

	if (fr->supervise && (val & reset_l)) {
		if (delayed_work_pending(&fr->wq))
			return IRQ_HANDLED;

		INIT_DELAYED_WORK(&fr->wq, fsp_reset_lionfish);
		schedule_delayed_work(&fr->wq, SYNC_WAIT_TIME);
	} else {
		return fsp_reset_call_rapid_reboot(fr);
	}

	return IRQ_HANDLED;
}

static irqreturn_t fsp_reset_irq_k2(int irq, void *data)
{
	struct fsp_reset_data *fr = data;

	return fsp_reset_call_rapid_reboot(fr);
}

static int lionfish_reset_init(struct fsp_reset_data *fr)
{
	struct device *dev = fr->dev;
	unsigned int val;
	int ret;

	fr->rm = fpga_master_node_to_regmap(dev);
	if (IS_ERR(fr->rm)) {
		ret = PTR_ERR(fr->rm);
		dev_err(dev, "couldn't get regmap: %d\n", ret);
		return ret;
	}
	/* enable all IRQA interrupts as these are disabled by default. */
	ret = regmap_update_bits(fr->rm, LFA_IRQA_ENABLE, IRQA_MASK, IRQA_MASK);
	if (ret < 0) {
		dev_err(dev, "couldn't enable IRQA\n");
		return ret;
	}

	if (fr->supervise) {
		fr->nb.priority = 255;
		fr->nb.notifier_call = fsp_reset_lionfish_sync;

		ret = slave_sup_notif_register(&fr->nb);
		if (ret) {
			dev_err(fr->dev, "couldn't register slave superviser %d",
				ret);
			return ret;
		}
	}

	fr->irq_callback = fsp_reset_irq_lionfish;

	/* just for debug, ignores error handling */
	regmap_read(fr->rm, LFA_IRQA_ENABLE, &val);
	dev_dbg(fr->dev, "LFA IRQA ENABLE ->  %#x\n", val);

	return 0;
}

static int keystone_reset_init(struct fsp_reset_data *fr)
{
	struct device *dev = fr->dev;
	int ret;

	reset_base = devm_ioremap(dev, FSP_RAM_SELF_REFRESH_ADDRESS,
			     FSP_RAM_SELF_REFRESH_ADDR_SIZE);
	if (!reset_base) {
		pr_err("ioremap failure\n");
		return -ENOMEM;
	}

	ret = configure_rstmux();
	if (ret)
		return ret;

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

static const struct fsp_reset_match_data match_data_lionfish = {
	.init = lionfish_reset_init,
};

static struct of_device_id fsp_reset_match[] = {
	{
		.compatible = "nsn,fsp_reset",
		.data = &match_data_keystone,
	},
	{
		.compatible = "nokia,baseband-reset",
		.data = &match_data_lionfish,
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

