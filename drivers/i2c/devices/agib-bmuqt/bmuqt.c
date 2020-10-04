/*
 * bmuqt.c - Control BMUQt. Adapted from following by Antti Garding.
 * bmu_ctrl.c - control BMU FPGA of FSPx in SM3
 * Copyright (C) Nokia 2014
 * \section AUTHORS
 *  Michael Lawnick, michael.lawnick@nsn.com, +49 89 5159 32847
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/of_platform.h>
#include <fsmddg_bmu.h>
#include <fsmddg_sfp.h>
#include "bmuqt.h"

static unsigned short slotid = 0;
module_param(slotid, short, S_IWUSR | S_IRUGO);
#define SHELF_MASK BIT(2)

#define DT_COMPATIBLE_STRING "bmuqt"

/* Register access functions */
static ssize_t readBMUReg(struct i2c_client *client, uint8_t *buf, int reg) {
	struct i2c_msg msgs[2];
	char i2c_buf;
	ssize_t ret;

	*buf = 0x00;

	/* setup positioning header */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &i2c_buf;
	msgs[0].buf[0] = (char)reg;
	/* setup receive buffer */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev,
				"%s: reg 0x%02X: i2c_transfer failed (%d)!\n",
				__func__, reg, (int)ret);
		ret = -EIO;
	}
	dev_dbg(&client->dev,
		"%s: reg 0x%02X value 0x%02X (process %s pid %d)\n",
		__func__, reg, *buf, current->comm, current->pid);

	return (ret < 0) ? ret : 1;
}

static ssize_t writeBMUReg(struct i2c_client *client, uint8_t buf, int reg) {
	struct i2c_msg msg;
	char i2c_buf[2];
	ssize_t ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = i2c_buf;
	msg.buf[0] = (char) reg;
	msg.buf[1] = buf;

	dev_dbg(&client->dev,
		"%s: reg 0x%02X value 0x%02X (process %s pid %d)\n",
		__func__, reg, buf, current->comm, current->pid);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if(ret != 1) {
		dev_err(&client->dev, "%s: reg 0x%02X data 0x%02X:"
			" i2c_transfer failed (%d)!\n",
			__func__, reg, buf, (int)ret);
		ret = -EIO;
	}

	return (ret < 0) ? ret : 1;
}

static void print_reg(struct i2c_client *client, const char *name, int reg)
{
	struct bmu_data *data = i2c_get_clientdata(client);
	uint8_t buf;
	ssize_t ret;

	mutex_lock(&data->device_lock);
	ret = readBMUReg(client, &buf, reg);
	mutex_unlock(&data->device_lock);

	if (ret < 0)
		dev_err(&client->dev, "read %s failed! (%zd)\n", name, ret);
	dev_info(&client->dev, "%s: %s(0x%X): 0x%02X", __func__, name, reg, buf);
}

/******************************************************************************
 * Writes specific bitfield in a device register
 *****************************************************************************/
static ssize_t bmu_bitfield_write(struct i2c_client *client,
	 uint8_t addr, uint8_t offset, uint8_t size, uint8_t value)
{
	struct bmu_data *data = i2c_get_clientdata(client);
	uint8_t mask;
	ssize_t ret = 0;
	uint8_t tmp = 0;

	mutex_lock(&data->device_lock);

	if (!((offset == 0) && (size == 8))) {
		ret = readBMUReg(client, &tmp, addr);
		if (ret < 0)
			goto bfwrite_exit;
	}

	mask = 0xFF;
	mask >>= (8 - size);

	value &= mask;
	value <<= offset;
	mask <<= offset;

	tmp &= (~mask);
	tmp |= value;

	ret = writeBMUReg(client, tmp, addr);
	if (ret < 0)
		goto bfwrite_exit;

bfwrite_exit:
	mutex_unlock(&data->device_lock);
	return ret;
}

/******************************************************************************
 * Reads specific bitfield from a device register
 *****************************************************************************/
static ssize_t bmu_bitfield_read(struct i2c_client *client,
	uint8_t addr, uint8_t offset, uint8_t size, uint8_t *value)
{
	struct bmu_data *data = i2c_get_clientdata(client);
	uint8_t mask;
	ssize_t ret = 0;
	uint8_t reg;

	*value = 0x00;

	mutex_lock(&data->device_lock);

	ret = readBMUReg(client, &reg, addr);
	if (ret < 0)
		goto bfread_exit;

	mask = 0xFF;
	mask >>= (8 - size);

	reg >>= offset;
	reg &= mask;

	*value = reg;

bfread_exit:
	mutex_unlock(&data->device_lock);
	return ret;
}

/* Sysfs register access macros */
SYSFS_BITFIELD_ENTRY_RD(FPGA_VERSION_REG, 0, 8, 0, fpga_version);

SYSFS_BITFIELD_ENTRY_RD(HARDWARE_VERSION_REG, 0, 4, 0, hardware_version);

SYSFS_BITFIELD_ENTRY_RW(DEBUG_DATA_INJECTION_REG, 4, 1, 0, ot_alarm);
SYSFS_BITFIELD_ENTRY_RW(DEBUG_DATA_INJECTION_REG, 3, 1, 0, clock_failure);
SYSFS_BITFIELD_ENTRY_RW(DEBUG_DATA_INJECTION_REG, 2, 1, 0, soc_failure);
SYSFS_BITFIELD_ENTRY_RW(DEBUG_DATA_INJECTION_REG, 0, 1, 0, peripheral_failure);

SYSFS_BITFIELD_ENTRY_RW(DEBUG_CONTROL_REG, 0, 8, 0, debug_control);

SYSFS_BITFIELD_ENTRY_RD(FNA_BP_ID_REG, 5, 3, 0, fna);
SYSFS_BITFIELD_ENTRY_RD(FNA_BP_ID_REG, 0, 4, 0, bp_id);

SYSFS_BITFIELD_ENTRY_RD(SOC_BOOT_COMPLETE_STATUS_REG, 0, 1, 0, soc_boot_complete_status);

SYSFS_BITFIELD_ENTRY_RW(HARQ_CONTROL_REG, 7, 1, 0, harq_enabled);

SYSFS_BITFIELD_ENTRY_RD(LAST_RESET_EVENT_AND_PERIPHERAL_RESET_REG, 4, 4, 0, last_reset_event);
SYSFS_BITFIELD_ENTRY_RW(LAST_RESET_EVENT_AND_PERIPHERAL_RESET_REG, 2, 1, 0, tpm_reset);
SYSFS_BITFIELD_ENTRY_RW(LAST_RESET_EVENT_AND_PERIPHERAL_RESET_REG, 1, 1, 0, eth_phy_1_reset);
SYSFS_BITFIELD_ENTRY_RW(LAST_RESET_EVENT_AND_PERIPHERAL_RESET_REG, 0, 1, 0, eth_phy_0_reset);

SYSFS_BITFIELD_ENTRY_RW(WATCHDOG_CONTROL_REG, 0, 8, 0, watchdog_enabled);

SYSFS_BITFIELD_ENTRY_RW(WATCHDOG_FEEDING_REG, 0, 1, 0, feed_watchdog);

SYSFS_BITFIELD_ENTRY_RW(WATCHDOG_TIMEOUT_PERIOD_REG, 0, 8, 0, watchdog_timeout_period);

SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_1_REG, 2, 1, 0, power_fail_p1v8);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_1_REG, 1, 1, 0, power_fail_p3v3_bmuq);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_1_REG, 0, 1, 0, power_fail_p5v3_com);

SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_2_REG, 3, 1, 0, power_fail_pll_1v8);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_2_REG, 2, 1, 0, power_fail_pll_2v5);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_2_REG, 1, 1, 0, power_fail_pll_3v3);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_2_REG, 0, 1, 0, power_fail_clk_4v0);

SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 7, 1, 0, power_fail_dvdd_3v3);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 6, 1, 0, power_fail_dvdd_0v85);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 5, 1, 0, power_fail_ddr3_vtt);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 4, 1, 0, power_fail_ddr3_vtt_or_ddr3_vref);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 3, 1, 0, power_fail_ddr3_vdd);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 2, 1, 0, power_fail_dvdd_1v8);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 1, 1, 0, power_fail_soc_cvdd1);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_3_REG, 0, 1, 0, power_fail_soc_cvdd);

SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_4_REG, 1, 1, 0, power_fail_eth_1v0);
SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_4_REG, 0, 1, 0, power_fail_eth_2v5);

SYSFS_BITFIELD_ENTRY_RD(POWER_FAIL_STATUS_5_REG, 0, 1, 0, power_fail_vusb);

SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 7, 1, 0, intake_air_over_temp_warning);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 6, 1, 0, intake_air_over_temp_alarm);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 5, 1, 0, cvdd_power_over_temp_warning);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 4, 1, 0, cvdd_power_over_temp_alarm);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 3, 1, 0, soc_ext_over_temp_warning);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 2, 1, 0, soc_ext_over_temp_alarm);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 1, 1, 0, soc_int_over_temp_warning);
SYSFS_BITFIELD_ENTRY_RD(OVER_TEMP_ALARM_REG, 0, 1, 0, soc_int_over_temp_alarm);

SYSFS_BITFIELD_ENTRY_RW(FPGA_EXT_SPI_FLASH_CTRL_STATUS_REG, 7, 1, 0, pid_erase_control);
SYSFS_BITFIELD_ENTRY_RD(FPGA_EXT_SPI_FLASH_CTRL_STATUS_REG, 6, 1, 0, pid_erase_status);
SYSFS_BITFIELD_ENTRY_RW(FPGA_EXT_SPI_FLASH_CTRL_STATUS_REG, 4, 1, 0, spi_flash_status);

SYSFS_BITFIELD_ENTRY_RW(STATUS_LED_CTRL_REG, 2,	1, 0, flashing_control);
SYSFS_BITFIELD_ENTRY_RW(STATUS_LED_CTRL_REG, 0, 2, 0, color_selection);

SYSFS_BITFIELD_ENTRY_RD(NO_POWER_GOOD_COUNTERS_REG, 6, 2, 0, clock_counter_value);
SYSFS_BITFIELD_ENTRY_RD(NO_POWER_GOOD_COUNTERS_REG, 4, 2, 0, soc_counter_value);
SYSFS_BITFIELD_ENTRY_RD(NO_POWER_GOOD_COUNTERS_REG, 0, 2, 0, peripheral_counter_value);

SYSFS_BITFIELD_ENTRY_RD(FPGA_IMAGE_TYPE_AND_MISC_CTRL_REG, 5, 1, 0, fpga_image_type);
SYSFS_BITFIELD_ENTRY_RD(FPGA_IMAGE_TYPE_AND_MISC_CTRL_REG, 4, 1, 0, fpga_version_type);
SYSFS_BITFIELD_ENTRY_RW(FPGA_IMAGE_TYPE_AND_MISC_CTRL_REG, 1, 1, 0, fault_logger_control);
SYSFS_BITFIELD_ENTRY_RW(FPGA_IMAGE_TYPE_AND_MISC_CTRL_REG, 0, 1, 0, fpga_reconfiguration_control);

SYSFS_BITFIELD_ENTRY_RW(UTC_TIMESTAMP_RESYNC_0_REG, 0, 8, 0, utc_timestamp_resync_0);

SYSFS_BITFIELD_ENTRY_RW(UTC_TIMESTAMP_RESYNC_1_REG, 0, 8, 0, utc_timestamp_resync_1);

SYSFS_BITFIELD_ENTRY_RW(UTC_TIMESTAMP_RESYNC_2_REG, 0, 8, 0, utc_timestamp_resync_2);

SYSFS_BITFIELD_ENTRY_RW(UTC_TIMESTAMP_RESYNC_3_REG, 0, 8, 0, utc_timestamp_resync_3);


/* FSPJ-BMUQ attributes for fsp */
static struct attribute *bmuqt_attributes[] = {
	&dev_attr_fpga_version.attr,
	&dev_attr_hardware_version.attr,
	&dev_attr_ot_alarm.attr,
	&dev_attr_clock_failure.attr,
	&dev_attr_soc_failure.attr,
	&dev_attr_peripheral_failure.attr,
	&dev_attr_debug_control.attr,
	&dev_attr_fna.attr,
	&dev_attr_bp_id.attr,
	&dev_attr_soc_boot_complete_status.attr,
	&dev_attr_harq_enabled.attr,
	&dev_attr_last_reset_event.attr,
	&dev_attr_tpm_reset.attr,
	&dev_attr_eth_phy_1_reset.attr,
	&dev_attr_eth_phy_0_reset.attr,
	&dev_attr_watchdog_enabled.attr,
	&dev_attr_feed_watchdog.attr,
	&dev_attr_watchdog_timeout_period.attr,
	&dev_attr_power_fail_p1v8.attr,
	&dev_attr_power_fail_p3v3_bmuq.attr,
	&dev_attr_power_fail_p5v3_com.attr,
	&dev_attr_power_fail_pll_1v8.attr,
	&dev_attr_power_fail_pll_2v5.attr,
	&dev_attr_power_fail_pll_3v3.attr,
	&dev_attr_power_fail_clk_4v0.attr,
	&dev_attr_power_fail_dvdd_3v3.attr,
	&dev_attr_power_fail_dvdd_0v85.attr,
	&dev_attr_power_fail_ddr3_vtt.attr,
	&dev_attr_power_fail_ddr3_vtt_or_ddr3_vref.attr,
	&dev_attr_power_fail_ddr3_vdd.attr,
	&dev_attr_power_fail_dvdd_1v8.attr,
	&dev_attr_power_fail_soc_cvdd1.attr,
	&dev_attr_power_fail_soc_cvdd.attr,
	&dev_attr_power_fail_eth_1v0.attr,
	&dev_attr_power_fail_eth_2v5.attr,
	&dev_attr_power_fail_vusb.attr,
	&dev_attr_intake_air_over_temp_warning.attr,
	&dev_attr_intake_air_over_temp_alarm.attr,
	&dev_attr_cvdd_power_over_temp_warning.attr,
	&dev_attr_cvdd_power_over_temp_alarm.attr,
	&dev_attr_soc_ext_over_temp_warning.attr,
	&dev_attr_soc_ext_over_temp_alarm.attr,
	&dev_attr_soc_int_over_temp_warning.attr,
	&dev_attr_soc_int_over_temp_alarm.attr,
	&dev_attr_pid_erase_control.attr,
	&dev_attr_pid_erase_status.attr,
	&dev_attr_spi_flash_status.attr,
	&dev_attr_flashing_control.attr,
	&dev_attr_color_selection.attr,
	&dev_attr_clock_counter_value.attr,
	&dev_attr_soc_counter_value.attr,
	&dev_attr_peripheral_counter_value.attr,
	&dev_attr_fpga_image_type.attr,
	&dev_attr_fpga_version_type.attr,
	&dev_attr_fault_logger_control.attr,
	&dev_attr_fpga_reconfiguration_control.attr,
	&dev_attr_utc_timestamp_resync_0.attr,
	&dev_attr_utc_timestamp_resync_1.attr,
	&dev_attr_utc_timestamp_resync_2.attr,
	&dev_attr_utc_timestamp_resync_3.attr,
	NULL
};

static const struct attribute_group bmuq_fspj_group = {
	.attrs = bmuqt_attributes,
};

/*
 * driver's probe function
 */
static int bmu_probe(struct i2c_client *new_client,
			 const struct i2c_device_id *id)
{
	struct bmu_data *data;
	int err;
	uint8_t version;
	int board_id = id->driver_data;
	bool same_slot;

	dev_info(&new_client->dev, "%s: device probe\n", __func__);

	/* Read the FPGA version */
	err = (int)readBMUReg(new_client, &version, FPGA_VERSION_REG);

	if (err != 1)
		return err;

	data = kzalloc(sizeof(struct bmu_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(new_client, data);
	mutex_init(&data->device_lock);
	mutex_init(&data->config_lock);

	data->board_id	= board_id;
	data->numGpio	= 0;
	data->numLed	= 0;
	data->ledId	= -1;
	data->gpioId	= -1;
	data->gpio_string[0]= '\0';
	data->led_string[0] = '\0';
	data->version = version;

	dev_info(&new_client->dev, "%s: slotid = %u\n", __func__, slotid);

	/* Register sysfs hooks depending of the BMU variant */
	switch (data->board_id) {
		case BOARD_FSP :
			dev_info(&new_client->dev,
				 "%s: Detected FSP-side BMUQ.\n", __func__);
			dev_info(&new_client->dev,
				 "%s: LEDs support enabled\n", __func__);
			err = sysfs_create_group(&new_client->dev.kobj,
						 &bmuq_fspj_group);
			kobject_uevent(&new_client->dev.kobj, KOBJ_ADD);
			break;
		default:
			dev_err(&new_client->dev,
				"%s: Unknown device type. Aborting...\n",
				__func__);
			err = -ENODEV;
			break;
	}

	print_reg(new_client, "Version", FPGA_VERSION_REG);

	if (err)
		goto exit_free;

	err = of_platform_populate(new_client->dev.of_node, NULL, NULL,
				   &new_client->dev);
	if (err)
		dev_err(&new_client->dev, "%s: of_platform_populate failed %d\n",
			__func__, err);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

/*
 * driver's remove function
 */
static int bmu_remove(struct i2c_client *client)
{
	struct bmu_data *data = i2c_get_clientdata(client);

	/* Remove sysfs hooks depending of the BMU variant */
	switch (data->board_id) {
		case BOARD_FSP :
			sysfs_remove_group(&client->dev.kobj,
					   &bmuq_fspj_group);
			break;
		default:
			break;
	}

	if (data->numGpio > 0)
	{
		data->numGpio = 0;
		data->gpioId = -1;
	}

	if (data->numLed > 0)
	{
		//led_twsi_device_unregister(data->ledId);
		data->numLed = 0;
		data->ledId = -1;
	}

	i2c_set_clientdata(client, NULL);
	kfree(data);
	dev_info(&client->dev, "%s: device removed\n", __func__);
	return 0;
}

/* Device variants */
static const struct i2c_device_id bmu_id[] = {
	{ DT_COMPATIBLE_STRING, BOARD_FSP },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmu_id);

static struct i2c_driver bmu_driver = {
	.driver = {
		.name   = DRIVERNAME,
		.owner  = THIS_MODULE,
	},
	.probe		= bmu_probe,
	.remove		= bmu_remove,
	.id_table	= bmu_id,
};

module_i2c_driver(bmu_driver);

MODULE_AUTHOR("Michael Lawnick <michael.lawnick@nsn.com>");
MODULE_DESCRIPTION("BMU control driver");
MODULE_LICENSE("GPL");
