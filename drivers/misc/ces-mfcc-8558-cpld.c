/*
 * ces-mfcc-8558.c - CES MFCC-8558 board CPLD driver.
 *
 *  Copyright (C) 2016 Creative Electronic Software, SA
 *  John Whitney <john.whitney@ces-noam.com>
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
#include <linux/of.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#ifdef CONFIG_SMBALERT
#include <linux/i2c-smbus.h>
#endif
#include <linux/io.h>
#include <asm/io.h>
#include "ces-mfcc-8558.h"

/**
 * Type Definitions and Constants
 */
#define MFCC8558_CPLD_VER_MAJOR             1
#define MFCC8558_CPLD_VER_MINOR             0
#define MFCC8558_CPLD_VER_REVISION          0

#if defined (CONFIG_WATCHDOG_CORE)
# define MFCC8558_CPLD_WDT_DEF_TIMEOUT      60
# define MFCC8558_CPLD_WDT_MIN_TIMEOUT      20
# define MFCC8558_CPLD_WDT_MAX_TIMEOUT      70
# define MFCC8558_CPLD_WDT_TICKS_PER_SEC    1526
#endif /* CONFIG_WATCHDOG_CORE */

#define MFCC8558_CPLD_REG_GENERAL_STS       0xE000
#define   MFCC8558_CPLD_GENSTS_MAINT_MODE     (1 << 0)
#define   MFCC8558_CPLD_GENSTS_BOOT_MODE      (1 << 1)
#define   MFCC8558_CPLD_GENSTS_PWR_FAIL       (1 << 3)
#define   MFCC8558_CPLD_GENSTS_PWR_OK         (1 << 8)
#define MFCC8558_CPLD_REG_BUS_SWITCH        0x1000
#define MFCC8558_CPLD_REG_ALARM_STS         0x2000
#define   MFCC8558_CPLD_ALARM_STS_VPWR        (1 << 0)
#define   MFCC8558_CPLD_ALARM_STS_3V3         (1 << 1)
#define   MFCC8558_CPLD_ALARM_STS_5V0         (1 << 2)
#define   MFCC8558_CPLD_ALARM_STS_1V8         (1 << 3)
#define   MFCC8558_CPLD_ALARM_STS_2V5         (1 << 4)
#define   MFCC8558_CPLD_ALARM_STS_1V0         (1 << 5)
#define   MFCC8558_CPLD_ALARM_STS_1V35        (1 << 6)
#define   MFCC8558_CPLD_ALARM_STS_1V5         (1 << 7)
#define   MFCC8558_CPLD_ALARM_STS_VOLTAGE     (0x00FF)
#define   MFCC8558_CPLD_ALARM_STS_CPU_TEMP    (1 << 8)
#define   MFCC8558_CPLD_ALARM_STS_DDR3_TEMP   (1 << 9)
#define   MFCC8558_CPLD_ALARM_STS_DCDC_TEMP   (1 << 10)
#define   MFCC8558_CPLD_ALARM_STS_TEMPERATURE (0x0700)
#define MFCC8558_CPLD_REG_GPIO_CSR          0x3000
#define MFCC8558_CPLD_REG_RESET_CAUSE       0x4000
#define   MFCC8558_CPLD_RST_CAUSE_MASK        0x0007
#define   MFCC8558_CPLD_RST_CAUSE_POWER       0x0000
#define   MFCC8558_CPLD_RST_CAUSE_HWL_WDOG    0x0001
#define   MFCC8558_CPLD_RST_CAUSE_PGM_WDOG    0x0002
#define   MFCC8558_CPLD_RST_CAUSE_XMC_RESET   0x0003
#define   MFCC8558_CPLD_RST_CAUSE_CPU_RESET   0x0004
#define   MFCC8558_CPLD_RST_CAUSE_OTHER       0x0007
#define MFCC8558_CPLD_REG_MISC_0            0x5000
#define MFCC8558_CPLD_REG_MISC_2            0x6000
#define MFCC8558_CPLD_REG_TSTAMP_LOW        0x7000
#define MFCC8558_CPLD_REG_TSTAMP_MED        0x8000
#define MFCC8558_CPLD_REG_TSTAMP_HIGH       0x9000
#define MFCC8558_CPLD_REG_WDG_HWL           0xA000
#define   MFCC8558_CPLD_WDOG_HWL_EN           (1 << 0)
#define MFCC8558_CPLD_REG_WDOG_PROG         0xB000
#define   MFCC8558_CPLD_WDOG_PROG_EN          (1 << 0)
#define   MFCC8558_CPLD_WDOG_PROG_TYPE        (1 << 1)
#define   MFCC8558_CPLD_WDOG_PROG_TYPE_INT    (0 << 1)
#define   MFCC8558_CPLD_WDOG_PROG_TYPE_RST    (1 << 1)
#define MFCC8558_CPLD_REG_WDOG_LIMIT        0xC000
#define MFCC8558_CPLD_REG_VERSION           0xF000
#define MFCC8558_CPLD_REG_VARIANT           0x10000
#define   MFCC8558_CPLD_VARIANT_AA            0x4141
#define   MFCC8558_CPLD_VARIANT_EA            0x4541

enum mfcc8558_variant {
	MFCC8558_VARIANT_AA,
	MFCC8558_VARIANT_EA,

	MFCC8558_VARIANT_COUNT
};

#ifdef CONFIG_SMBALERT
enum mfcc8558_smbalert_type {
	MFCC8558_SMBALERT_CPU,
	MFCC8558_SMBALERT_DDR3,
	MFCC8558_SMBALERT_DCDC,

	MFCC8558_SMBALERT_COUNT
};

struct mfcc8558_smbalert_info {
	struct i2c_adapter           *adapter;
	struct i2c_client            *ara;
	struct i2c_smbus_alert_setup  alert_data;
};

static struct mfcc8558_smbalert_info mfcc8558_smbalert_info[MFCC8558_SMBALERT_COUNT];

#endif /* CONFIG_SMBALERT */

struct mfcc8558_cpld_device {
	struct platform_device *pdev;
	struct mutex            mutex;
	void __iomem           *regs;
	uint16_t                alarm_cache;
	enum mfcc8558_variant   variant;
};

static struct mfcc8558_cpld_device *mfcc8558_cpld_default_device = NULL;

static uint16_t mfcc8558_cpld_read16(struct mfcc8558_cpld_device *dev, uint32_t offset)
{
	uint16_t value = in_be16(dev->regs + offset);
	return value;
}

static void mfcc8558_cpld_write16(struct mfcc8558_cpld_device *dev,
                                  uint32_t                     offset,
                                  uint16_t                     value)
{
	return out_be16(dev->regs + offset, value);
}

static uint64_t mfcc8558_cpld_get_timestamp(struct mfcc8558_cpld_device *dev)
{
	uint16_t low;
	uint16_t med;
	uint16_t high;
	uint64_t timestamp;

	do {
		high = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_TSTAMP_HIGH);
		do {
			med = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_TSTAMP_MED);
			low = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_TSTAMP_LOW);
		} while (med != mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_TSTAMP_MED));
	} while (high != mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_TSTAMP_HIGH));

	timestamp = ((uint64_t) high << 32) |
	            ((uint32_t) med  << 16) |
	            low;
	return timestamp;
}

uint64_t ces_mfcc8558_get_timestamp(void)
{
	struct mfcc8558_cpld_device *dev       = mfcc8558_cpld_default_device;
	uint64_t                     timestamp = 0xFFFFFFFFFFFFFFFFULL;

	if ((dev != NULL) && (dev->variant == MFCC8558_VARIANT_AA)) {
		timestamp = mfcc8558_cpld_get_timestamp (dev);
	}

	return timestamp;
}

static ssize_t mfcc8558_cpld_sysfs_pwralarm_show(struct class           *class,
                                                 struct class_attribute *attr,
                                                 char                   *buf)
{
	struct mfcc8558_cpld_device *dev    = mfcc8558_cpld_default_device;
	ssize_t                      result = 0;

	if (dev == NULL)
		result = sprintf(buf, "no device found\n");
	else {
		/*
		 * Read the current alarm status bits from the register, and
		 * merge them into the cached value.
		 */
		dev->alarm_cache |= mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                                         MFCC8558_CPLD_REG_ALARM_STS);
		/* Clear the status bits in the alarm status register. */
		mfcc8558_cpld_write16(mfcc8558_cpld_default_device,
		                      MFCC8558_CPLD_REG_ALARM_STS,
		                      0xFFFF);

		/* Process the voltage alarm bits in the cached value. */
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_VPWR)
			result += sprintf(buf + result, "%svpwr", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_3V3)
			result += sprintf(buf + result, "%s3.3v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_5V0)
			result += sprintf(buf + result, "%s5.0v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_1V8)
			result += sprintf(buf + result, "%s1.8v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_2V5)
			result += sprintf(buf + result, "%s2.5v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_1V0)
			result += sprintf(buf + result, "%s1.0v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_1V35)
			result += sprintf(buf + result, "%s1.35v", (result) ? "," : "");
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_1V5)
			result += sprintf(buf + result, "%s1.5v", (result) ? "," : "");

		/* Clear the voltage related bits from the cached value. */
		dev->alarm_cache &= ~MFCC8558_CPLD_ALARM_STS_VOLTAGE;

		if (result == 0)
			result = sprintf(buf, "none\n");
		else
			result += sprintf(buf + result, "\n");
	}

	return result;
}

static ssize_t mfcc8558_cpld_sysfs_tempalarm_show(struct class           *class,
                                                  struct class_attribute *attr,
                                                  char                   *buf)
{
	struct mfcc8558_cpld_device *dev    = mfcc8558_cpld_default_device;
	ssize_t                      result = 0;

	if (dev == NULL)
		result = sprintf(buf, "no device found\n");
	else {
		/*
		 * Read the current alarm status bits from the register, and
		 * merge them into the cached value.
		 */
		dev->alarm_cache |= mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                                         MFCC8558_CPLD_REG_ALARM_STS);
#ifdef CONFIG_SMBALERT
		/* Process the temperature alarm bits in the cached value. */
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_CPU_TEMP) {
			mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_CPU);
			result += sprintf(buf + result, "%scpu", (result) ? "," : "");
		}
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_DDR3_TEMP) {
			mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_DDR3);
			result += sprintf(buf + result, "%sddr3", (result) ? "," : "");
		}
		if (dev->alarm_cache & MFCC8558_CPLD_ALARM_STS_DCDC_TEMP) {
			mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_DCDC);
			result += sprintf(buf + result, "%sdcdc", (result) ? "," : "");
		}
#endif
		/* Clear the temperature related bits from the cached value. */
		dev->alarm_cache &= ~MFCC8558_CPLD_ALARM_STS_TEMPERATURE;

		if (result == 0)
			result = sprintf(buf, "none\n");
		else
			result += sprintf(buf + result, "\n");

		/* Clear the status bits in the alarm status register. */
		mfcc8558_cpld_write16(mfcc8558_cpld_default_device,
		                      MFCC8558_CPLD_REG_ALARM_STS,
		                      0xFFFF);
	}

	return result;
}

static ssize_t mfcc8558_cpld_sysfs_pwrok_show(struct class           *class,
                                              struct class_attribute *attr,
                                              char                   *buf)
{
	uint16_t value;
	ssize_t  result = 0;

	if (mfcc8558_cpld_default_device == NULL)
		result = sprintf(buf, "no device found\n");
	else {
		value = mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                             MFCC8558_CPLD_REG_GENERAL_STS);
		if (value & MFCC8558_CPLD_GENSTS_PWR_OK)
			result = sprintf(buf, "1\n");
		else
			result = sprintf(buf, "0\n");
	}

	return result;
}

static ssize_t mfcc8558_cpld_sysfs_opmode_show(struct class           *class,
                                               struct class_attribute *attr,
                                               char                   *buf)
{
	uint16_t value;
	ssize_t  result = 0;

	if (mfcc8558_cpld_default_device == NULL)
		result = sprintf(buf, "no device found\n");
	else {
		value = mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                             MFCC8558_CPLD_REG_GENERAL_STS);
		if (value & MFCC8558_CPLD_GENSTS_BOOT_MODE)
			result = sprintf(buf, "mission\n");
		else if (value & MFCC8558_CPLD_GENSTS_MAINT_MODE)
			result = sprintf(buf, "system maintenance\n");
		else
			result = sprintf(buf, "operating maintenance\n");
	}

	return result;
}

static ssize_t mfcc8558_cpld_sysfs_timestamp_show(struct class           *class,
                                                  struct class_attribute *attr,
                                                  char                   *buf)
{
	return sprintf(buf, "%lld\n", ces_mfcc8558_get_timestamp());
}

static ssize_t mfcc8558_cpld_sysfs_rst_cause_show(struct class           *class,
                                                  struct class_attribute *attr,
                                                  char                   *buf)
{
	uint16_t value;
	ssize_t  result;

	if (mfcc8558_cpld_default_device == NULL)
		result = sprintf(buf, "no device found\n");
	else {
		value = mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                             MFCC8558_CPLD_REG_RESET_CAUSE);
		switch (value & MFCC8558_CPLD_RST_CAUSE_MASK) {
		case MFCC8558_CPLD_RST_CAUSE_POWER:
			result = sprintf(buf, "power failure\n");
			break;
		case MFCC8558_CPLD_RST_CAUSE_HWL_WDOG:
			result = sprintf(buf, "hwl watchdog\n");
			break;
		case MFCC8558_CPLD_RST_CAUSE_PGM_WDOG:
			result = sprintf(buf, "pgm watchdog\n");
			break;
		case MFCC8558_CPLD_RST_CAUSE_XMC_RESET:
			result = sprintf(buf, "xmc reset\n");
			break;
		case MFCC8558_CPLD_RST_CAUSE_CPU_RESET:
			result = sprintf(buf, "cpu reset\n");
			break;
		case MFCC8558_CPLD_RST_CAUSE_OTHER:
		default:
			result = sprintf(buf, "other\n");
			break;
		}
	}

	return result;
}

/*
 * With the MFCC-8558, the class attributes are filled in dynamically as
 * some attributes are only valid on specific board variants.
 */
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_pwralarm);
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_pwrok);
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_rst_cause);
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_tempalarm);
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_opmode);
static CLASS_ATTR_RO(mfcc8558_cpld_sysfs_timestamp);

static struct attribute *mfcc8558_cpld_class_attrs[8] = { NULL };
ATTRIBUTE_GROUPS(mfcc8558_cpld_class);

static struct class mfcc8558_cpld_class = {
	.name  = "ces_mfcc_8558",
	.owner = THIS_MODULE,

	.class_groups = mfcc8558_cpld_class_groups,
};

#if defined (CONFIG_WATCHDOG_CORE)
/* Watchdog Operations */
static int mfcc8558_cpld_wdt_start(struct watchdog_device *wdd)
{
	struct mfcc8558_cpld_device *dev = watchdog_get_drvdata(wdd);

	mfcc8558_cpld_write16(dev,
	                      MFCC8558_CPLD_REG_WDOG_LIMIT,
	                      MFCC8558_CPLD_WDT_TICKS_PER_SEC * wdd->timeout);
	mfcc8558_cpld_write16(dev,
	                      MFCC8558_CPLD_REG_WDOG_PROG,
	                      (MFCC8558_CPLD_WDOG_PROG_EN |
	                       MFCC8558_CPLD_WDOG_PROG_TYPE_RST));
	return 0;
}

static int mfcc8558_cpld_wdt_stop(struct watchdog_device *wdd)
{
	struct mfcc8558_cpld_device *dev = watchdog_get_drvdata(wdd);

	mfcc8558_cpld_write16(dev, MFCC8558_CPLD_REG_WDOG_PROG, 0);
	return 0;
}

static int mfcc8558_cpld_wdt_ping(struct watchdog_device *wdd)
{
	struct mfcc8558_cpld_device *dev = watchdog_get_drvdata(wdd);

	mfcc8558_cpld_write16(dev,
	                      MFCC8558_CPLD_REG_WDOG_LIMIT,
	                      MFCC8558_CPLD_WDT_TICKS_PER_SEC * wdd->timeout);
	return 0;
}

static int  mfcc8558_cpld_wdt_timeout  = MFCC8558_CPLD_WDT_DEF_TIMEOUT;
static bool mfcc8558_cpld_wdt_nowayout = WATCHDOG_NOWAYOUT;
static const struct watchdog_info mfcc8558_cpld_wdt_info = {
	.options  = WDIOF_CARDRESET |
	            WDIOF_MAGICCLOSE |
	            WDIOF_KEEPALIVEPING,
	.identity = "MFCC-8558 Watchdog",
};

static struct watchdog_ops mfcc8558_cpld_wdt_ops = {
	.owner = THIS_MODULE,
	.start = mfcc8558_cpld_wdt_start,
	.stop  = mfcc8558_cpld_wdt_stop,
	.ping  = mfcc8558_cpld_wdt_ping,
};

static struct watchdog_device mfcc8558_cpld_wdt_device = {
	.info        = &mfcc8558_cpld_wdt_info,
	.ops         = &mfcc8558_cpld_wdt_ops,
	.timeout     = MFCC8558_CPLD_WDT_DEF_TIMEOUT,
	.min_timeout = MFCC8558_CPLD_WDT_MIN_TIMEOUT,
	.max_timeout = MFCC8558_CPLD_WDT_MAX_TIMEOUT,
};

module_param_named(nowayout, mfcc8558_cpld_wdt_nowayout, bool, S_IRUGO | S_IWUSR);
module_param_named(timeout, mfcc8558_cpld_wdt_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout,
                 "Watchdog timeout in seconds ("
                 __MODULE_STRING(MFCC8558_CPLD_WDT_MIN_TIMEOUT)
                 "-"
                 __MODULE_STRING(MFCC8558_CPLD_WDT_MAX_TIMEOUT)
                 ", default "
                 __MODULE_STRING(MFCC8558_CPLD_WDT_DEF_TIMEOUT)
                 ")");
#endif /* CONFIG_WATCHDOG_CORE */

/* Sysfs Operations */
static int mfcc8558_cpld_sysfs_init(void)
{
	struct mfcc8558_cpld_device *dev   = mfcc8558_cpld_default_device;
	int                          status;

	struct attribute **attr = mfcc8558_cpld_class_attrs;

	/* Set the attributes common to all variants. */
	*attr++ = &class_attr_mfcc8558_cpld_sysfs_pwralarm.attr;
	*attr++ = &class_attr_mfcc8558_cpld_sysfs_pwrok.attr;
	*attr++ = &class_attr_mfcc8558_cpld_sysfs_rst_cause.attr;
	*attr++ = &class_attr_mfcc8558_cpld_sysfs_tempalarm.attr;

	/* Set the attributes common to all specific variants. */
	switch (dev->variant) {
	case MFCC8558_VARIANT_AA:
	default:
		*attr++ = &class_attr_mfcc8558_cpld_sysfs_opmode.attr;
		*attr++ = &class_attr_mfcc8558_cpld_sysfs_timestamp.attr;
		break;
	case MFCC8558_VARIANT_EA:
		break;
	}

	status = class_register(&mfcc8558_cpld_class);
	if (status != 0) {
		printk(KBUILD_MODNAME ": Unable to register class '%s'.\n",
		       mfcc8558_cpld_class.name);
	}

	return status;
}

static void mfcc8558_cpld_sysfs_deinit(void)
{
	class_destroy(&mfcc8558_cpld_class);
}

#ifdef CONFIG_SMBALERT

/* SMBALERT Operations */
static int mfcc8558_cpld_smbalert_init(void)
{
	int                i;
	static const char *smbalert_name[MFCC8558_SMBALERT_COUNT] =
	{
		"CPU",
		"DDR3",
		"DCDC",
	};

	for (i = 0; i < MFCC8558_SMBALERT_COUNT; i++) {
		struct mfcc8558_smbalert_info *smbalert = &mfcc8558_smbalert_info[i];

		smbalert->alert_data.alert_edge_triggered = 1;

		smbalert->adapter = i2c_get_adapter(i + 4);
		if (smbalert->adapter == NULL) {
			printk(KBUILD_MODNAME ": Unable to acquire adapter %d for SMBALERT "
			       "notification for %s.\n",
			       i + 4,
			       smbalert_name[i]);
			continue;
		}

		smbalert->ara = i2c_setup_smbus_alert(smbalert->adapter,
		                                      &smbalert->alert_data);
		if (smbalert->ara == NULL) {
			printk(KBUILD_MODNAME ": Unable to register SMBALERT client "
			       "for %s.\n",
			       smbalert_name[i]);
			i2c_put_adapter (smbalert->adapter);
			smbalert->adapter = NULL;
			continue;
		}
	}

	return 0;
}

static void mfcc8558_cpld_smbalert_cleanup(void)
{
	int i;

	for (i = 0; i < MFCC8558_SMBALERT_COUNT; i++) {
		struct mfcc8558_smbalert_info *smbalert = &mfcc8558_smbalert_info[i];

		if (smbalert->ara != NULL) {
			i2c_unregister_device(smbalert->ara);
			smbalert->ara = NULL;
		}
		if (smbalert->adapter != NULL) {
			i2c_put_adapter(smbalert->adapter);
			smbalert->adapter = NULL;
		}
	}
}

void mfcc8558_cpld_handle_smbalert(enum mfcc8558_smbalert_type type)
{
	struct mfcc8558_smbalert_info *smbalert = &mfcc8558_smbalert_info[type];

	if (smbalert->ara) {
		i2c_handle_smbus_alert(smbalert->ara);
		flush_workqueue(system_wq);
	}

	return;
}
#endif

/* Module Handling Operations */
static int mfcc8558_cpld_of_probe(struct platform_device *pdev)
{
	struct mfcc8558_cpld_device *dev;
	struct resource             *resource;
	int                          result;
	uint16_t                     version;
	uint16_t                     variant;
	uint16_t                     value;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		printk(KBUILD_MODNAME ": Could not allocate memory.\n");
		result = -ENOMEM;
		goto out_error;
	}

	dev->pdev = pdev;
	mutex_init(&dev->mutex);

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource == NULL) {
		printk(KBUILD_MODNAME ": Could not find register resource.\n");
		result = -EINVAL;
		goto out_free_device;
	}

	dev->regs = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(dev->regs)) {
		printk(KBUILD_MODNAME ": Unable to map registers.\n");
		result = PTR_ERR(dev->regs);
		goto out_free_device;
	}

	/*
	 * Display the CPLD revision.
	 */
	version = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_VERSION);
	printk(KBUILD_MODNAME ": Found CPLD version %d.%d.%d\n",
	       (version >> 8) & 0xf,
	       (version >> 4) & 0xf,
	       (version & 0xf));

	/*
	 * Determine and display the board variant.
	 */
	variant = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_VARIANT);
	switch (variant) {
	case MFCC8558_CPLD_VARIANT_AA:
		dev->variant = MFCC8558_VARIANT_AA;
		break;
	case MFCC8558_CPLD_VARIANT_EA:
		dev->variant = MFCC8558_VARIANT_EA;
		break;
	default:
		printk(KBUILD_MODNAME ": Detected UNKNOWN BOARD VARIANT.\n");
		printk(KBUILD_MODNAME ": Treating as MFCC-8558AA.\n");
		dev->variant = MFCC8558_VARIANT_AA;
		break;
	}

	/*
	 * The watchdog devices are only available when the board is in "mission"
	 * mode.  Otherwise, the watchdog registers are pretty much read-only.
	 * If booting in maintenance mode, don't register a watchdog.
	 */
	value = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_GENERAL_STS);
	printk(KBUILD_MODNAME ": status %x\n", value);
#ifdef ENABLE_SW_WDOG
	if (value & MFCC8558_CPLD_GENSTS_BOOT_MODE) {
#if defined (CONFIG_WATCHDOG_CORE)
		result = watchdog_register_device(&mfcc8558_cpld_wdt_device);
		if (result != 0) {
			printk(KBUILD_MODNAME ": Unable to register watchdog.\n");
			goto out_unmap_regs;
		}
		watchdog_set_nowayout(&mfcc8558_cpld_wdt_device,
		                      mfcc8558_cpld_wdt_nowayout);
		watchdog_set_drvdata(&mfcc8558_cpld_wdt_device, dev);
#endif /* CONFIG_WATCHDOG_CORE */
	}
#endif /* ENABLE_SW_WDOG */
	/* Disable the HWL (boot) watchdog. */
	mfcc8558_cpld_write16(dev, MFCC8558_CPLD_REG_WDG_HWL, 0);
	printk(KBUILD_MODNAME ": Disabled hardware watchdog\n");

	platform_set_drvdata(pdev, dev);

	if (mfcc8558_cpld_default_device == NULL) {
		mfcc8558_cpld_default_device = dev;
		mfcc8558_cpld_sysfs_init();

		/* Clear out any existing alerts. */
		dev->alarm_cache = mfcc8558_cpld_read16(mfcc8558_cpld_default_device,
		                                        MFCC8558_CPLD_REG_ALARM_STS);
		dev->alarm_cache &= MFCC8558_CPLD_ALARM_STS_VOLTAGE;
#ifdef CONFIG_SMBALERT
		mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_CPU);
		mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_DDR3);
		mfcc8558_cpld_handle_smbalert(MFCC8558_SMBALERT_DCDC);
#endif
		mfcc8558_cpld_write16(dev, MFCC8558_CPLD_REG_ALARM_STS, 0xFFFF);
	}

	return 0;

#if defined (CONFIG_WATCHDOG_CORE)
out_unmap_regs:
	devm_iounmap(&pdev->dev, dev->regs);
#endif /* CONFIG_WATCHDOG_CORE */
out_free_device:
	devm_kfree(&pdev->dev, dev);
out_error:
	return result;
}

static int mfcc8558_cpld_of_remove(struct platform_device *pdev)
{
	struct mfcc8558_cpld_device *dev = platform_get_drvdata(pdev);
#if defined (CONFIG_WATCHDOG_CORE)
	uint16_t                     value;
#endif /* CONFIG_WATCHDOG_CORE */

	if (mfcc8558_cpld_default_device == dev) {
		mfcc8558_cpld_sysfs_deinit();
		mfcc8558_cpld_default_device = NULL;
	}

	platform_set_drvdata(pdev, NULL);

#if defined (CONFIG_WATCHDOG_CORE)
	/*
	 * The watchdog driver was only registered if the board was booted in
	 * "mission" mode.
	 */
	value = mfcc8558_cpld_read16(dev, MFCC8558_CPLD_REG_GENERAL_STS);
	if (value & MFCC8558_CPLD_GENSTS_BOOT_MODE) {
		watchdog_set_drvdata(&mfcc8558_cpld_wdt_device, NULL);
		watchdog_unregister_device (&mfcc8558_cpld_wdt_device);
	}
#endif /* CONFIG_WATCHDOG_CORE */

	devm_iounmap(&pdev->dev, dev->regs);
	devm_kfree(&pdev->dev, dev);

	return 0;
}

static struct of_device_id mfcc8558_cpld_of_match[] = {
	{ .compatible = "ces,mfcc-8558-cpld" },
	{}
};

static struct platform_driver mfcc8558_cpld_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = mfcc8558_cpld_of_match,
	},
	.probe  = mfcc8558_cpld_of_probe,
	.remove = mfcc8558_cpld_of_remove,
};

static int __init mfcc8558_cpld_init(void)
{
	printk(KBUILD_MODNAME ": version %d.%d.%d\n",
	       MFCC8558_CPLD_VER_MAJOR,
	       MFCC8558_CPLD_VER_MINOR,
	       MFCC8558_CPLD_VER_REVISION);

#ifdef CONFIG_SMBALERT
	mfcc8558_cpld_smbalert_init();
#endif

#if defined (CONFIG_WATCHDOG_CORE)
	if ((mfcc8558_cpld_wdt_timeout < MFCC8558_CPLD_WDT_MIN_TIMEOUT) ||
	    (mfcc8558_cpld_wdt_timeout > MFCC8558_CPLD_WDT_MAX_TIMEOUT)) {
		printk(KBUILD_MODNAME ": Invalid timeout parameter.\n");
		return -EINVAL;
	} else {
		mfcc8558_cpld_wdt_device.timeout = mfcc8558_cpld_wdt_timeout;
	}
#endif /* CONFIG_WATCHDOG_CORE */

	return platform_driver_register(&mfcc8558_cpld_of_driver);
}

static void __exit mfcc8558_cpld_exit(void)
{
	platform_driver_unregister(&mfcc8558_cpld_of_driver);
#ifdef CONFIG_SMBALERT
	mfcc8558_cpld_smbalert_cleanup();
#endif
	return;
}

MODULE_DEVICE_TABLE(of, mfcc8558_cpld_of_match);
MODULE_LICENSE("GPL");
module_init(mfcc8558_cpld_init);
module_exit(mfcc8558_cpld_exit);

EXPORT_SYMBOL(ces_mfcc8558_get_timestamp);
