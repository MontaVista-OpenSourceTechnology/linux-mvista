/*
	l-asc10.c - Driver for Lattice L-ASC10 Hardware Management Expander i2c
	interface

	Copyright (c) 2015 Maciej Sobkowski <maciej.sobkowski@nokia.com>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>

#define DRIVERNAME "asc"

#define ASC_CMD_READ_ID         0x02
#define ASC_CMD_WRITE_CFG_REG   0x31
#define ASC_CMD_WRITE_REG_WMASK 0x32
#define ASC_CMD_READ_CFG_REG    0x33
#define ASC_CMD_WRITE_MEAS_CTRL 0x51
#define ASC_CMD_READ_MEAS_CTRL  0x52

#define ASC_REG_TMON_MEAS_1_HIGH   0x80
#define ASC_REG_TMON_MEAS_1_LOW    0x81
#define ASC_REG_TMON_MEAS_2_HIGH   0x82
#define ASC_REG_TMON_MEAS_2_LOW    0x83
#define ASC_REG_TMON_MEAS_INT_HIGH 0x84
#define ASC_REG_TMON_MEAS_INT_LOW  0x85
#define ASC_REG_TMON_STAT_A        0x86
#define ASC_REG_TMON_STAT_B        0x87

#define ASC_CFG_REG_TMON_1_CONFIG_1   0x39
#define ASC_CFG_REG_TMON_1_CONFIG_3   0x3B
#define ASC_CFG_REG_TMON_1_CONFIG_4   0x3C
#define ASC_CFG_REG_TMON_1_CONFIG_7   0x3F
#define ASC_CFG_REG_TMON_2_CONFIG_1   0x42
#define ASC_CFG_REG_TMON_2_CONFIG_3   0x44
#define ASC_CFG_REG_TMON_2_CONFIG_4   0x45
#define ASC_CFG_REG_TMON_2_CONFIG_7   0x48
#define ASC_CFG_REG_TMON_INT_CONFIG_3 0x4D
#define ASC_CFG_REG_TMON_INT_CONFIG_4 0x4E
#define ASC_CFG_REG_TMON_INT_CONFIG_7 0x51

#define ASC_REG_ADC_AUX	0x00
#define ASC_REG_ADC_VALUE_LOW 0x01
#define ASC_REG_ADC_VALUE_HIGH 0x02

enum asc_tmon_sensors {
	tmon_1,
	tmon_2,
	tmon_int,
};

enum asc_adc_mux_selector{
	vmon_1 = 0x00,
	vmon_2,
	vmon_3,
	vmon_4,
	vmon_5,
	vmon_6,
	vmon_7,
	vmon_8,
	vmon_9 = 0x08,
	hvmon = 0x09,
};

struct attens {
	unsigned char atten[10];
};

/*
	ASC0 (0x60):
	ATTEN = 0: VMON2-VMON5, VMON9, HVMON
	ATTEN = 1: VMON1, VMON6, VMON7, VMON8

	ASC1 (0x61):
	ATTEN = 0: VMON1, VMON3-VMON6, HVMON
	ATTEN = 1: VMON2, VMON7-VMON9
*/
static const struct attens asc_adc_mux[2] = {
	{.atten = {0x80, 0, 0, 0, 0, 0x80, 0x80, 0x80, 0, 0}},
	{.atten = {0, 0x80, 0, 0, 0, 0, 0x80, 0x80, 0x80, 0}}
};

enum asc_tmon_diode_cfg {
	TMON_DISABLED = 0x00,
	TMON_BETA_PNP = 0x01,
	TMON_DIFFERENTIAL = 0x02,
	TMON_SINGLE_ENDED = 0x03,
	TMON_DIODE_CFG_MASK = 0x03,
};

struct asc_data {
	struct i2c_client *client;
	struct mutex lock;
};

static int asc_read_temp(struct i2c_client *client, int sensor_no, int *val);
static int asc_read_alarm(struct i2c_client *client, int sensor_no, int *alarm);
static int asc_read_hyst(struct i2c_client *client, int sensor_no, int *hyst);
static int asc_read_max(struct i2c_client *client, int sensor_no, int *max);
static int asc_write_hyst(struct i2c_client *client, int sensor_no, int hyst);
static int asc_write_max(struct i2c_client *client, int sensor_no, int max);
static int asc_read_voltage(struct i2c_client *client, int selector_no, int *voltage);
static int asc_write_adc_mux(struct i2c_client *client, uint8_t adc_mux);
static int asc_read_conv_info(struct i2c_client *client, int selector_no, int *conversion_info);

#define show_attr(name) \
static ssize_t show_##name(struct device *dev, \
			   struct device_attribute *da, char *buf) \
{ \
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da); \
	struct i2c_client *client = to_i2c_client(dev); \
	struct asc_data *data = i2c_get_clientdata(client); \
	int nr = attr->index; \
	int value = 0; \
	int err; \
	mutex_lock(&data->lock); \
	err = asc_read_##name(client, nr, &value);	\
	mutex_unlock(&data->lock); \
	return scnprintf(buf, PAGE_SIZE, "%d\n", value); \
}

#define set_attr(name) \
static ssize_t set_##name(struct device *dev, \
			  struct device_attribute *da, const char *buf, size_t count) \
{ \
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da); \
	struct i2c_client *client = to_i2c_client(dev); \
	struct asc_data *data = i2c_get_clientdata(client); \
	long value = 0; \
	int nr = attr->index; \
	int err = kstrtol(buf, 10, &value); \
	if (err < 0) { \
		dev_err(dev, "Failed to parse parameter (%d)!\n", err); \
		return err; \
	} \
	mutex_lock(&data->lock); \
	err = asc_write_##name(client, nr, value);	\
	mutex_unlock(&data->lock); \
	if (err < 0) \
		return err; \
	return count; \
}

show_attr(temp);
show_attr(alarm);
show_attr(hyst);
show_attr(max);

set_attr(hyst);
set_attr(max);

show_attr(voltage);
show_attr(conv_info);

static int asc_read_cfg_regs(
    struct i2c_client *, unsigned short, uint8_t, uint8_t *);
static int asc_write_cfg_regs_masked(
    struct i2c_client *, unsigned short, uint8_t, uint8_t *, uint8_t *);

static ssize_t show_tmon_enabled(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct asc_data *data = i2c_get_clientdata(client);
	uint8_t value1;
	uint8_t value2;
	int rc;
	unsigned enabled;

	mutex_lock(&data->lock);
	rc = asc_read_cfg_regs (client, 1, ASC_CFG_REG_TMON_1_CONFIG_1, &value1);
	if (rc != 0) {
		dev_err(dev, "Failed to read ASC_CFG_REG_TMON_1_CONFIG_1 (%d)!\n", rc);
	}
	else {
		rc = asc_read_cfg_regs (client, 1, ASC_CFG_REG_TMON_2_CONFIG_1, &value2);
		if (rc != 0) {
			dev_err(dev, "Failed to read ASC_CFG_REG_TMON_2_CONFIG_1 (%d)!\n", rc);
		}
	}
	mutex_unlock(&data->lock);

	if (rc != 0) {
		return rc;
	}

	dev_dbg(dev, "Diode cfg. is: TMON1 = 0x%X, TMON2 = 0x%X\n",
		(unsigned)(value1 & TMON_DIODE_CFG_MASK),
		(unsigned)(value2 & TMON_DIODE_CFG_MASK));

	enabled = (((value1 & TMON_DIODE_CFG_MASK) == TMON_DISABLED) &&
		   ((value2 & TMON_DIODE_CFG_MASK) == TMON_DISABLED)) ? 0 : 1;
	return scnprintf(buf, PAGE_SIZE, "%u\n", enabled);
}

static ssize_t set_tmon_enabled(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct asc_data *data = i2c_get_clientdata(client);
	uint8_t mask = ~((uint8_t)TMON_DIODE_CFG_MASK);
	uint8_t rvalue = 0;
	long param = 0;

	int rc = kstrtol (buf, 10, &param);
	if (rc != 0) {
		dev_err(dev, "Failed to parse parameter (%d)!\n", rc);
		return rc;
	}

	if ((param != 0) && (param != 1)) {
		dev_err(dev, "Invalid parameter (%ld)!\n", param);
		return -EINVAL;
	}

	rvalue = (param == 1) ? TMON_DIFFERENTIAL : TMON_DISABLED;

	mutex_lock(&data->lock);
	rc = asc_write_cfg_regs_masked (client, 1, ASC_CFG_REG_TMON_1_CONFIG_1, &rvalue, &mask);
	if (rc != 0) {
		dev_err(dev, "Failed to write ASC_CFG_REG_TMON_1_CONFIG_1 (%d)!\n", rc);
	}
	else {
		rc = asc_write_cfg_regs_masked (client, 1, ASC_CFG_REG_TMON_2_CONFIG_1, &rvalue, &mask);
		if (rc != 0) {
			dev_err(dev, "Failed to write ASC_CFG_REG_TMON_2_CONFIG_1 (%d)!\n", rc);
		}
	}
	mutex_unlock(&data->lock);

	if (rc != 0) {
		return rc;
	}
	return count;
}

static DEVICE_ATTR(tmon_enabled, S_IWUSR | S_IRUGO, show_tmon_enabled, set_tmon_enabled);

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, tmon_1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, tmon_2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp, NULL, tmon_int);

static SENSOR_DEVICE_ATTR(temp1_alarm, S_IRUGO, show_alarm, NULL, tmon_1);
static SENSOR_DEVICE_ATTR(temp2_alarm, S_IRUGO, show_alarm, NULL, tmon_2);
static SENSOR_DEVICE_ATTR(temp3_alarm, S_IRUGO, show_alarm, NULL, tmon_int);

static SENSOR_DEVICE_ATTR(temp1_hyst, S_IWUSR | S_IRUGO, show_hyst, set_hyst, tmon_1);
static SENSOR_DEVICE_ATTR(temp2_hyst, S_IWUSR | S_IRUGO, show_hyst, set_hyst, tmon_2);
static SENSOR_DEVICE_ATTR(temp3_hyst, S_IWUSR | S_IRUGO, show_hyst, set_hyst, tmon_int);

static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, show_max, set_max, tmon_1);
static SENSOR_DEVICE_ATTR(temp2_max, S_IWUSR | S_IRUGO, show_max, set_max, tmon_2);
static SENSOR_DEVICE_ATTR(temp3_max, S_IWUSR | S_IRUGO, show_max, set_max, tmon_int);

static SENSOR_DEVICE_ATTR(vmon1, S_IRUGO, show_voltage, NULL, vmon_1);
static SENSOR_DEVICE_ATTR(vmon2, S_IRUGO, show_voltage, NULL, vmon_2);
static SENSOR_DEVICE_ATTR(vmon3, S_IRUGO, show_voltage, NULL, vmon_3);
static SENSOR_DEVICE_ATTR(vmon4, S_IRUGO, show_voltage, NULL, vmon_4);
static SENSOR_DEVICE_ATTR(vmon5, S_IRUGO, show_voltage, NULL, vmon_5);
static SENSOR_DEVICE_ATTR(vmon6, S_IRUGO, show_voltage, NULL, vmon_6);
static SENSOR_DEVICE_ATTR(vmon7, S_IRUGO, show_voltage, NULL, vmon_7);
static SENSOR_DEVICE_ATTR(vmon8, S_IRUGO, show_voltage, NULL, vmon_8);
static SENSOR_DEVICE_ATTR(vmon9, S_IRUGO, show_voltage, NULL, vmon_9);
static SENSOR_DEVICE_ATTR(hvmon, S_IRUGO, show_voltage, NULL, hvmon);


static SENSOR_DEVICE_ATTR(vmon1_cvi, S_IRUGO, show_conv_info, NULL, vmon_1);
static SENSOR_DEVICE_ATTR(vmon2_cvi, S_IRUGO, show_conv_info, NULL, vmon_2);
static SENSOR_DEVICE_ATTR(vmon3_cvi, S_IRUGO, show_conv_info, NULL, vmon_3);
static SENSOR_DEVICE_ATTR(vmon4_cvi, S_IRUGO, show_conv_info, NULL, vmon_4);
static SENSOR_DEVICE_ATTR(vmon5_cvi, S_IRUGO, show_conv_info, NULL, vmon_5);
static SENSOR_DEVICE_ATTR(vmon6_cvi, S_IRUGO, show_conv_info, NULL, vmon_6);
static SENSOR_DEVICE_ATTR(vmon7_cvi, S_IRUGO, show_conv_info, NULL, vmon_7);
static SENSOR_DEVICE_ATTR(vmon8_cvi, S_IRUGO, show_conv_info, NULL, vmon_8);
static SENSOR_DEVICE_ATTR(vmon9_cvi, S_IRUGO, show_conv_info, NULL, vmon_9);
static SENSOR_DEVICE_ATTR(hvmon_cvi, S_IRUGO, show_conv_info, NULL, hvmon);

static struct attribute *asc_attributes[] = {
	&dev_attr_tmon_enabled.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp1_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp3_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_hyst.dev_attr.attr,
	&sensor_dev_attr_temp2_hyst.dev_attr.attr,
	&sensor_dev_attr_temp3_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_vmon1.dev_attr.attr,
	&sensor_dev_attr_vmon2.dev_attr.attr,
	&sensor_dev_attr_vmon3.dev_attr.attr,
	&sensor_dev_attr_vmon4.dev_attr.attr,
	&sensor_dev_attr_vmon5.dev_attr.attr,
	&sensor_dev_attr_vmon6.dev_attr.attr,
	&sensor_dev_attr_vmon7.dev_attr.attr,
	&sensor_dev_attr_vmon8.dev_attr.attr,
	&sensor_dev_attr_vmon9.dev_attr.attr,
	&sensor_dev_attr_hvmon.dev_attr.attr,
        &sensor_dev_attr_vmon1_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon2_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon3_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon4_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon5_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon6_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon7_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon8_cvi.dev_attr.attr,
	&sensor_dev_attr_vmon9_cvi.dev_attr.attr,
	&sensor_dev_attr_hvmon_cvi.dev_attr.attr,
	NULL,
};

static const struct attribute_group asc_group = {
	.attrs = asc_attributes,
};

static int asc_reg_read_byte(struct i2c_client *client, int reg, uint8_t *value)
{
	struct i2c_msg msgs[2];
	char i2c_buf;
	ssize_t ret;

	*value = 0x00;

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
	msgs[1].buf = value;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c_transfer failed (%d)!\n", (int)ret);
		ret = -EIO;
	}

	return (ret < 0) ? ret : 0;
}

static int asc_read_meas_reg(struct i2c_client *client, int reg, uint8_t *value)
{
	struct i2c_msg msgs[2];
	char i2c_buf[2];
	ssize_t ret;

	*value = 0x00;

	/* setup positioning header */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = i2c_buf;
	msgs[0].buf[0] = ASC_CMD_READ_MEAS_CTRL;
	msgs[0].buf[1] = (char)reg;

	/* setup receive buffer */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = value;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c_transfer failed (%d)!\n", (int)ret);
		ret = -EIO;
	}

	return (ret < 0) ? ret : 0;
}

static int asc_read_cfg_regs(struct i2c_client *client, unsigned short num,
			     uint8_t reg, uint8_t *value)
{
	struct i2c_msg msgs[2];
	char i2c_cmd[2];
	ssize_t ret;

	/* setup positioning header */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = i2c_cmd;
	msgs[0].buf[0] = ASC_CMD_READ_CFG_REG;
	msgs[0].buf[1] = (char) reg;

	/* setup receive buffer */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = num;
	msgs[1].buf = (char *) value;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c_transfer failed (%d)!\n", (int)ret);
		ret = -EIO;
	}

	return (ret < 0) ? ret : 0;
}

static int asc_write_cfg_regs_masked(struct i2c_client *client, unsigned short num,
				     uint8_t reg, uint8_t *value, uint8_t *mask)
{
	ssize_t ret;
	unsigned int i;
	struct i2c_msg msg;
	char *i2c_cmd = kzalloc((2 + 2 * num) * sizeof(char), GFP_KERNEL);

	if (!i2c_cmd)
		return -ENOMEM;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + (2 * num);
	msg.buf = i2c_cmd;
	msg.buf[0] = ASC_CMD_WRITE_REG_WMASK;
	msg.buf[1] = (char) reg;
	for (i = 0; i < num; i++) {
		msg.buf[2 * i + 2] = mask[i];
		msg.buf[2 * i + 3] = value[i];
	}

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "i2c_transfer failed (%d)!\n", (int)ret);
		ret = -EIO;
	}

	kfree(i2c_cmd);

	return (ret < 0) ? ret : 0;
}

#define TO_NEGATIVE(reg_val, sign_bit) \
	-((~reg_val & ((1 << sign_bit) - 1)) + 1)

static int asc_read_temp(struct i2c_client *client, int sensor_no, int *temp)
{
	uint8_t reg_high, reg_low;
	uint8_t byte_high, byte_low;
	int err;
	int reg_val;

	switch (sensor_no) {

	case tmon_1:
		reg_high = ASC_REG_TMON_MEAS_1_HIGH;
		reg_low  = ASC_REG_TMON_MEAS_1_LOW;
		break;

	case tmon_2:
		reg_high = ASC_REG_TMON_MEAS_2_HIGH;
		reg_low  = ASC_REG_TMON_MEAS_2_LOW;
		break;

	case tmon_int:
		reg_high = ASC_REG_TMON_MEAS_INT_HIGH;
		reg_low  = ASC_REG_TMON_MEAS_INT_LOW;
		break;

	default:
		return -EFAULT;
	}

	err = asc_read_meas_reg(client, reg_high, &byte_high);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", reg_high);
		return err;
	}

	err = asc_read_meas_reg(client, reg_low, &byte_low);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", reg_low);
		return err;
	}

	reg_val = 0x00;
	reg_val = (byte_high << 3) | (byte_low >> 5);

	/* 10-th bit is sign bit, so if set, we need to convert to negative */
	if (reg_val & (1 << 10))
		*temp = TO_NEGATIVE(reg_val, 10);
	else
		*temp = reg_val;

	/* Desired format of temperature is 1/1000th of degree
	 * Precision of value in register is .25th of degree
	 * We need to divide by four and multiply by 1000,
	 * for short multiply by 250
	 */
	*temp *= 250;

	return 0;
}

static int asc_read_alarm(struct i2c_client *client, int sensor_no, int *alarm)
{
	uint8_t status_reg;
	int err;
	int temp;

	err = asc_read_meas_reg(client, ASC_REG_TMON_STAT_A, &status_reg);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", ASC_REG_TMON_STAT_A);
		return err;
	}

	switch (sensor_no) {

	case tmon_1:
		*alarm = status_reg & 0x01;
		break;

	case tmon_2:
		*alarm = (status_reg >> 1) & 0x01;
		break;

	case tmon_int:
		*alarm = (status_reg >> 2) & 0x01;
		break;

	default:
		return -EFAULT;
	}

	// Due to HW problem additional reading of temperature has to be added.
	(void) asc_read_temp(client, sensor_no, &temp);

	return 0;
}

static int asc_read_hyst(struct i2c_client *client, int sensor_no, int *hyst)
{
	uint8_t reg_hyst;
	uint8_t value;
	int err;

	switch (sensor_no) {

	case tmon_1:
		reg_hyst  = ASC_CFG_REG_TMON_1_CONFIG_7;
		break;

	case tmon_2:
		reg_hyst  = ASC_CFG_REG_TMON_2_CONFIG_7;
		break;

	case tmon_int:
		reg_hyst  = ASC_CFG_REG_TMON_INT_CONFIG_7;
		break;

	default:
		return -EFAULT;
	}

	err = asc_read_cfg_regs(client, 1, reg_hyst, &value);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", reg_hyst);
		return err;
	}

	/* 6-th bit is sign bit, so if set, we need to convert to negative */
	if (value & (1 << 6))
		*hyst = TO_NEGATIVE(value, 6);
	else
		*hyst = value;

	/* Desired format of hysteresis is 1/1000th of degree
	 * Precision of value in register is 1 degree
	 * We need to multiply by 1000
	 * Hysteresis for overtemp is negative, desired is positive
	 * We need to multiply by -1
	 */

	*hyst *= -1000;

	return 0;
}

static int asc_write_hyst(struct i2c_client *client, int sensor_no, int hyst)
{
	uint8_t reg_hyst;
	int8_t value;
	uint8_t mask;
	int err;

	switch (sensor_no) {

	case tmon_1:
		reg_hyst  = ASC_CFG_REG_TMON_1_CONFIG_7;
		break;

	case tmon_2:
		reg_hyst  = ASC_CFG_REG_TMON_2_CONFIG_7;
		break;

	case tmon_int:
		reg_hyst  = ASC_CFG_REG_TMON_INT_CONFIG_7;
		break;

	default:
		return -EFAULT;
	}

	mask = 0x80;

	value = hyst / (-1000);

	if (hyst > 64000 || hyst < -63000) {
		dev_err(&client->dev, "Value %d is out of range\n", value);
		return -EINVAL;
	}

	if (value < 0)
		value = (~(-value) & 0x7F) + 0x01;

	err = asc_write_cfg_regs_masked(client, 1, reg_hyst, &value, &mask);
	if (err < 0) {
		dev_err(&client->dev, "Error writing byte to reg 0x%X\n", reg_hyst);
		return err;
	}

	return 0;
}

static int asc_read_max(struct i2c_client *client, int sensor_no, int *max)
{
	uint8_t reg_max_start;
	uint8_t value[2];
	int reg_val;
	int err;

	switch (sensor_no) {

	case tmon_1:
		reg_max_start  = ASC_CFG_REG_TMON_1_CONFIG_3;
		break;

	case tmon_2:
		reg_max_start  = ASC_CFG_REG_TMON_2_CONFIG_3;
		break;

	case tmon_int:
		reg_max_start  = ASC_CFG_REG_TMON_INT_CONFIG_3;
		break;

	default:
		return -EFAULT;
	}

	err = asc_read_cfg_regs(client, 2, reg_max_start, value);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", reg_max_start);
		return err;
	}

	reg_val = 0x00;
	reg_val = ((value[0] & 0xFE) << 1) | ((value[1] & 0x03));

	/* 8-th bit is sign bit, so if set, we need to convert to negative */
	if (reg_val & (1 << 8))
		*max = TO_NEGATIVE(reg_val, 8);
	else
		*max = reg_val;

	/* Desired format of max temp is 1/1000th of degree
	 * Precision of value in register is 1 degree
	 * We need to and multiply by 1000
	 */
	*max *= 1000;

	return 0;
}

static int asc_write_max(struct i2c_client *client, int sensor_no, int max)
{
	uint8_t reg_max_start;
	int16_t tmp_val;
	uint8_t value[2], mask[2];
	int err;

	switch (sensor_no) {

	case tmon_1:
		reg_max_start  = ASC_CFG_REG_TMON_1_CONFIG_3;
		break;

	case tmon_2:
		reg_max_start  = ASC_CFG_REG_TMON_2_CONFIG_3;
		break;

	case tmon_int:
		reg_max_start  = ASC_CFG_REG_TMON_INT_CONFIG_3;
		break;

	default:
		return -EFAULT;
	}

	tmp_val = max / 1000;

	if (max > 155000 || max < -64000) {
		dev_err(&client->dev, "Value %d is out of range\n", max);
		return -EINVAL;
	}

	if (tmp_val < 0)
		tmp_val = (~(-tmp_val) & 0x01FF) + 0x01;

	value[0] = (uint8_t) (tmp_val & 0x01FC) >> 1;
	value[1] = (uint8_t) (tmp_val & 0x0003);

	mask[0] = 0x01;
	mask[1] = 0xFC;

	err = asc_write_cfg_regs_masked(client, 2, reg_max_start, value, mask);
	if (err < 0) {
		dev_err(&client->dev, "Error writing byte to reg 0x%X\n", reg_max_start);
		return err;
	}

	return 0;
}

static int asc_write_adc_mux(struct i2c_client *client, uint8_t adc_mux)
{
	struct i2c_msg msg;
	int ret;
	char *i2c_cmd = kzalloc(3 * sizeof(char), GFP_KERNEL);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = i2c_cmd;
	msg.buf[0] = ASC_CMD_WRITE_MEAS_CTRL;
	msg.buf[1] = 0x00;
	msg.buf[2] = adc_mux;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "i2c_transfer failed (%d)!\n", (int)ret);
		ret = -EIO;
	}

	kfree(i2c_cmd);

	return (ret < 0) ? ret : 0;
	
}

static int asc_read_voltage(struct i2c_client *client, int selector_no, int *voltage)
{
	uint8_t byte_high, byte_low;
	uint8_t adx_mux_byte;
	int err;
	int reg_val;

	unsigned char target_asc_device = client->addr & 0x0F;

	if ((client->addr != 0x60) && (client->addr != 0x61)) {
		return EFAULT;
	}

	if ((selector_no < vmon_1) || (selector_no > hvmon)) {
		return EFAULT;
	}
	
	adx_mux_byte = asc_adc_mux[target_asc_device].atten[selector_no] | selector_no;

	err = asc_write_adc_mux(client, adx_mux_byte);
	if (err < 0) {
		return err;
	}

	err = asc_read_meas_reg(client, ASC_REG_ADC_VALUE_LOW, &byte_low);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", ASC_REG_ADC_VALUE_LOW);
		return err;
	}

	if (1 != (byte_low & 0x1)) {
		dev_err(&client->dev, "Conversion is in process: 0x%X\n", byte_low & 0x7);
		return -EAGAIN;
	}
	
	err = asc_read_meas_reg(client, ASC_REG_ADC_VALUE_HIGH, &byte_high);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n",ASC_REG_ADC_VALUE_HIGH );
		return err;
	}

	reg_val = 0x00;
	reg_val = (int)((byte_high << 5) | (byte_low >> 3));

	*voltage = reg_val * 2;

	return 0;
}

static int asc_read_conv_info(struct i2c_client *client, int selector_no, int *conversion_info)
{
	uint8_t byte_low;
	uint8_t adx_mux_byte;
	int err;
	unsigned char target_asc_device = client->addr & 0x0F;

	if ((client->addr != 0x60) && (client->addr != 0x61)) {
		return EFAULT;
	}

	if ((selector_no < vmon_1) || (selector_no > hvmon)) {
		return EFAULT;
	}
	
	adx_mux_byte = asc_adc_mux[target_asc_device].atten[selector_no] | selector_no;

	err = asc_write_adc_mux(client, adx_mux_byte);
	if (err < 0) {
		return err;
	}

	err = asc_read_meas_reg(client, ASC_REG_ADC_VALUE_LOW, &byte_low);
	if (err < 0) {
		dev_err(&client->dev, "Error reading byte from reg 0x%X\n", ASC_REG_ADC_VALUE_LOW);
		return err;
	}

	*conversion_info = (int)(byte_low & 0x7);
	return 0;
}

static int asc_probe(struct i2c_client *new_client, const struct i2c_device_id *id)
{
	struct asc_data *data;
	int err;
	uint8_t val;

	err = asc_reg_read_byte(new_client, ASC_CMD_READ_ID, &val);
	if (err < 0) {
		dev_err(&new_client->dev, "Couldn't read device ID register!\n");
		return err;
	}
	dev_dbg(&new_client->dev, "Read device ID = 0x%x\n", val);

	data = kzalloc(sizeof(struct asc_data), GFP_KERNEL);
	if (!data) {
		dev_err(&new_client->dev, "Memory allocation failed!\n");
		return -ENOMEM;
	}

	data->client = new_client;

	i2c_set_clientdata(new_client, data);

	mutex_init(&data->lock);

	err = sysfs_create_group(&new_client->dev.kobj, &asc_group);
	if (err) {
		dev_err(&new_client->dev, "Failed to create sysfs group (%d)!\n", err);
		kfree(data);
		return err;
	}

	return 0;
}


static int asc_remove(struct i2c_client *client)
{
	struct asc_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &asc_group);
	i2c_set_clientdata(client, NULL);
	kfree(data);

	return 0;
}

static const struct i2c_device_id asc_id[] = {
	{ DRIVERNAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asc_id);

static struct i2c_driver asc_driver = {
	.driver = {
		.name   = DRIVERNAME,
		.owner  = THIS_MODULE,
	},
	.probe		= asc_probe,
	.remove		= asc_remove,
	.id_table	= asc_id,
};

module_i2c_driver(asc_driver);

MODULE_AUTHOR("Maciej Sobkowski <maciej.sobkowski@nokia.com>");
MODULE_DESCRIPTION("L-ASC10 sensor driver");
MODULE_LICENSE("GPL");
