/*!
 * \file bmuq_watchdog.c
 *
 * \brief BMUQ Watchdog Driver
 *
 * \version 0.3
 *
 * \history
 *  03.03.2016 as  0.1  First version\n
 *  06.09.2016 - replace hardcoded timeouts by symbolic names
 *  12.06.2020 - adapted to AGIB board
 *               PHASE2 timeout init removed since the corresponding
 *               registers do not exist
 *
 * \section DESCRIPTION
 *  Driver for BMUQ new wtachdog ingtroduced in FSM-r4.
 *
 *
 * \section AUTHORS
 *  Wojciech Tesluk (wt), wojciech.tesluk@nokia.com
 *
 * \endfile
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/watchdog.h>
#include <linux/completion.h>
#include <linux/of_platform.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <fsmddg_fpga-master.h>

/* --- DEFINES / VARIABLES -------------------------------------------------- */
#define DRIVERNAME              "bmuq_watchdog"
#define WATCHDOG_IDENTITY	"BMUQ Watchdog over i2c"
#define WATCHDOG_VERSION	1
#define WATCHDOG_FLAGS		(WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING)

/* register values for enabling and disabling the watchdog */
#define WDOG_ENABLE_VAL         0xC5
#define WDOG_DISABLE_VAL        0x00

#define K2WD_CONTROL	        0x0A
#define K2WD_FEED		0x0B
#define K2WD_PHASE1_TO		0x0C
#define DEFAULT_TIMEOUT		60
#define DEFAULT_MAX_TIMEOUT	255

static unsigned timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout (default="
			__MODULE_STRING(DEFAULT_TIMEOUT) "sec)");

struct watchdog_data {
	struct watchdog_device wd_dev;
	struct i2c_client *client;
	struct regmap *regmap;
};

struct watchdog_match_data {
	struct watchdog_info info;
	const struct watchdog_ops ops;

	int (*init)(struct watchdog_data *wd_data);
};

/* --- PROTOTYPES ----------------------------------------------------------- */

extern struct bus_type i2c_bus_type;

/* --- IMPLEMENTATION ------------------------------------------------------- */


static int compare(struct device *dv, void *data)
{
	if (NULL != dv) {
		if (NULL != dv->driver) {
			if (0 == strncmp(dv->driver->name, (const char *) data,
					 strlen((const char *) data)))
				return 1;
		}
	}
	return 0;
}

static struct i2c_client *find_i2c_client(void)
{
	struct device *i2c_bmuqdevice;

	/* Driver name in Nokia code was "bmu_ctrl", On AGIB it is "bmuqt" */
	i2c_bmuqdevice = bus_find_device(&i2c_bus_type, NULL, "bmuqt",
					 compare);

	if (i2c_bmuqdevice)
		return to_i2c_client(i2c_bmuqdevice);

	i2c_bmuqdevice = bus_find_device(&i2c_bus_type, NULL,
					 "bmuq_regmap", compare);
	if (i2c_bmuqdevice)
		return to_i2c_client(i2c_bmuqdevice);

	pr_err("i2c_bmuq device not yet found\n");
	return NULL;
}

static ssize_t writeBmuqWatchdogReg(struct i2c_client *client, uint8_t buf,
				    int reg)
{
	struct i2c_msg msg;
	char i2c_buf[2];
	ssize_t ret;

	if (NULL == client)
		return 0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = i2c_buf;
	msg.buf[0] = (char) reg;
	msg.buf[1] = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		ret = -EIO;
	}

	return (ret < 0) ? ret : 1;
}

static ssize_t readBmuqWatchdog(struct i2c_client *client, uint8_t * buf,
				int reg)
{
	struct i2c_msg msgs[2];
	char i2c_buf;
	ssize_t ret;

	if (NULL == client)
		return 0;

	*buf = 0x00;

	/* setup positioning header */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &i2c_buf;
	msgs[0].buf[0] = (char) reg;
	/* setup receive buffer */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	/* BMU can't evaluate I2C restart - use separate transfers */
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev,
			"%s: reg 0x%02X: i2c_transfer failed (%d)!\n",
			__func__, reg, (int) ret);
		ret = -EIO;
	}
	dev_dbg(&client->dev, "%s: reg 0x%02X value 0x%02X (process %s pid %d)\n",
		__func__, reg, *buf, current->comm, current->pid);

	return (ret < 0) ? ret : 1;
}

static int write2watchdogdevice(struct i2c_client *client, uint8_t address,
				uint8_t value)
{
	int ret;

	ret = writeBmuqWatchdogReg(client, value, address);
	return (ret == 1) ? 0 : ret;
}

static int read_wd_device(struct i2c_client *client, uint8_t *buf, uint8_t reg)
{
	return readBmuqWatchdog(client, buf, reg);
}
/*
 * BMUQr
 */
static int bmuq_wd_ping(struct watchdog_device *wd_dev)
{
	struct watchdog_data *data = watchdog_get_drvdata(wd_dev);

	dev_dbg(wd_dev->parent, "%s: ping the watchdog: \n", __func__);
	return write2watchdogdevice(data->client, K2WD_FEED, 0x01);
}

static int bmuq_wd_get_state(struct watchdog_device *wd_dev)
{
	int ret;
	uint8_t odp = 0;
	struct watchdog_data *data = watchdog_get_drvdata(wd_dev);

	ret = read_wd_device(data->client, &odp, K2WD_CONTROL);
	dev_dbg(wd_dev->parent, "%s: read_wd_device returned %i odp = %i\n",
		__func__, ret, odp);
	return (odp == WDOG_ENABLE_VAL) ? 1 : 0;
}

static int bmuq_wd_start(struct watchdog_device *wd_dev)
{
	dev_dbg(wd_dev->parent, "%s: start the watchdog: \n", __func__);
	return bmuq_wd_ping(wd_dev);
}

static int bmuq_wd_stop(struct watchdog_device *wd_dev)
{
	dev_dbg(wd_dev->parent, "%s: stop the watchdog: \n", __func__);
	return 0;
}

static int bmuq_wd_set_timeout(struct watchdog_device *wd_dev,
			       unsigned int timeout)
{
	struct watchdog_data *data = watchdog_get_drvdata(wd_dev);
	struct i2c_client *client = data->client;
	int isEnabled = bmuq_wd_get_state(wd_dev);

	dev_dbg(wd_dev->parent, "%s: Set T1 timeout to %i seconds.\n",
		__func__, timeout);

	write2watchdogdevice(client, K2WD_CONTROL, 0);
	write2watchdogdevice(client, K2WD_PHASE1_TO, (uint8_t) timeout);
	if (isEnabled)
		write2watchdogdevice(client, K2WD_CONTROL, WDOG_ENABLE_VAL);

	return write2watchdogdevice(client, K2WD_FEED, 0x01);
}

static unsigned int bmuq_wd_get_timeleft(struct watchdog_device *wd_dev)
{
	int ret;
	uint8_t odp = 0;
	struct watchdog_data *data = watchdog_get_drvdata(wd_dev);

	ret = read_wd_device(data->client, &odp, K2WD_PHASE1_TO);
	dev_info(wd_dev->parent, "%s: read_wd_device returned %i odp = %i\n",
		 __func__, ret, odp);
	return (unsigned int) odp;
}

static int bmuq_wd_init(struct watchdog_data *wd_data)
{
	wd_data->client = find_i2c_client();
	if (!wd_data->client) {
		dev_err(wd_data->wd_dev.parent, "cannot get i2c client\n");
		return -ENODEV;
	}

	if (bmuq_wd_get_state(&wd_data->wd_dev))
		set_bit(WDOG_ACTIVE, &wd_data->wd_dev.status);

	return 0;
}

/*
 * BMUC
 */
static int bmuc_write(struct watchdog_device *wd_dev, unsigned int reg,
		      unsigned int value, const char *func)
{
	struct watchdog_data *data = watchdog_get_drvdata(wd_dev);

	int ret = regmap_write(data->regmap, reg, value);
	if (ret)
		dev_err(wd_dev->parent, "%s: regmap write failed\n", func);

	return ret;
}
static int bmuc_wd_start(struct watchdog_device *wd_dev)
{
	dev_info(wd_dev->parent, "%s: ping (as start) BMUC watchdog\n", __func__);

	return wd_dev->ops->ping(wd_dev);
}

static int bmuc_wd_stop(struct watchdog_device *wd_dev)
{
	dev_info(wd_dev->parent, "%s: stop the BMUC watchdog\n", __func__);

	return bmuc_write(wd_dev, K2WD_CONTROL, 0, __func__);
}

static int bmuc_wd_ping(struct watchdog_device *wd_dev)
{
	dev_dbg(wd_dev->parent, "%s: ping BMUC watchdog\n", __func__);

	return bmuc_write(wd_dev, K2WD_FEED, 0x2, __func__);
}


static int bmuc_wd_set_timeout(struct watchdog_device *wd_dev,
			       unsigned int timeout)
{
	int ret;

	dev_info(wd_dev->parent, "%s: Set T1 timeout to %i seconds.\n",
		__func__, timeout);

	ret = bmuc_write(wd_dev, K2WD_PHASE1_TO, timeout, __func__);
	if (ret)
		return ret;

	wd_dev->timeout = timeout;

	return 0;
}

static int bmuc_wd_init(struct watchdog_data *wd_data)
{
	struct watchdog_device *wd_dev = &wd_data->wd_dev;
	unsigned int val, ret;

	wd_data->regmap = fpga_master_node_to_regmap(wd_dev->parent);
	if (IS_ERR(wd_data->regmap)) {
		dev_err(wd_dev->parent, "%s: cannot get regmap\n", __func__);
		return PTR_ERR(wd_data->regmap);
	}

	ret = wd_dev->ops->set_timeout(wd_dev, wd_dev->timeout);
	if (ret < 0) {
		dev_err(wd_dev->parent, "%s: cannot set timeout 1\n", __func__);
		return ret;
	}

	ret = regmap_read(wd_data->regmap, K2WD_CONTROL, &val);
	if (ret < 0) {
		dev_err(wd_dev->parent, "%s: cannot read status", __func__);
		return ret;
	}

	if (val == WDOG_ENABLE_VAL) {
		dev_info(wd_dev->parent, "%s: watchdog is enabled", __func__);
		set_bit(WDOG_HW_RUNNING, &wd_data->wd_dev.status);
	}

	return 0;
}

static const struct watchdog_match_data bmuq_wd_dev = {
	.info = {
		.identity = WATCHDOG_IDENTITY,
		.options = WATCHDOG_FLAGS,
		.firmware_version = WATCHDOG_VERSION,
	},
	.ops = {
		.owner = THIS_MODULE,
		.start = bmuq_wd_start,
		.stop = bmuq_wd_stop,
		.ping = bmuq_wd_ping,
		.set_timeout = bmuq_wd_set_timeout,
		.get_timeleft = bmuq_wd_get_timeleft,
	},
	.init = bmuq_wd_init,
};

static const struct watchdog_match_data bmuc_wd_dev = {
	.info = {
		.identity = "BMUC Watchdog",
		.options = WATCHDOG_FLAGS,
		.firmware_version = WATCHDOG_VERSION,
	},
	.ops = {
		.owner = THIS_MODULE,
		.start = bmuc_wd_start,
		.stop = bmuc_wd_stop,
		.ping = bmuc_wd_ping,
		.set_timeout = bmuc_wd_set_timeout,
	},
	.init = bmuc_wd_init,
};
static const struct of_device_id wd_id_table[] = {
	{
		.compatible = "nokia,bmuq_watchdog",
		.data = &bmuq_wd_dev
	},
	{
		.compatible = "nokia,bmuc_watchdog",
		.data = &bmuc_wd_dev
	},
	{ },
};
MODULE_DEVICE_TABLE(of, wd_id_table);

static int wd_probe(struct platform_device *pdev)
{
	int ret;
	struct watchdog_data *wd_data;
	struct watchdog_device *wd_dev;
	const struct of_device_id *match;
	const struct watchdog_match_data *data;

	wd_data = devm_kzalloc(&pdev->dev, sizeof(*wd_data), GFP_KERNEL);
	if (!wd_data)
		return -ENOMEM;

	match = of_match_device(wd_id_table, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to match device data\n");
		return -ENODEV;
	}
	data = match->data;

	wd_dev = &wd_data->wd_dev;
	wd_dev->parent = &pdev->dev;
	wd_dev->ops = &data->ops;
	wd_dev->timeout = DEFAULT_TIMEOUT;
	wd_dev->min_timeout = 1;
	wd_dev->max_timeout = DEFAULT_MAX_TIMEOUT;
	wd_dev->min_hw_heartbeat_ms = 1000;
	wd_dev->info = &data->info;

	watchdog_set_drvdata(wd_dev, wd_data);
	platform_set_drvdata(pdev, wd_data);
	watchdog_init_timeout(wd_dev, timeout, &pdev->dev);

	ret = data->init(wd_data);
	if (ret) {
		dev_err(&pdev->dev, "device init failed\n");
		return ret;
	}

	ret = devm_watchdog_register_device(&pdev->dev, wd_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s : watchdog_register_device failed with status %i\n",
			__func__, ret);
		return ret;
	}
	dev_info(&pdev->dev, "%s : BMUQ watchdog successuly loaded\n",
		 __func__);
	return 0;
}

static struct platform_driver wd_drv = {
	.probe = wd_probe,
	.driver = {
		.name = DRIVERNAME,
		.of_match_table = wd_id_table,
	},
};
module_platform_driver(wd_drv);

MODULE_AUTHOR("wojciech.tesluk@nokia.com");
MODULE_DESCRIPTION("BMUQ Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DRIVERNAME);
