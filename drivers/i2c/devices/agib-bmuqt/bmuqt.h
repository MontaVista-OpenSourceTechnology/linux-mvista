/*!
 * \file bmuqt.h
 *
 * \brief BMU definitions. Adapted from Nokia's bmu_ctrl.h by Antti Garding.
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


#ifndef __BMUQT_H_
#define __BMUQT_H_ 1

#define DRIVERNAME "bmuqt"

#define MIN_BMU_FPGA_VERSION						0x0F
#define MIN_BMU_FPGA_VERSION_BTYPE_RECONFIG			0x26
#define MIN_BMU_FPGA_VERSION_BTYPE_RECONFIG_FTLF	0x16
#define MIN_BMU_FPGA_VERSION_RST_EVENT				0x29
#define MIN_BMU_FPGA_VERSION_BITSTREAM_TYPE			0x2C

#define FPGA_VERSION_REG                            0x00
#define HARDWARE_VERSION_REG                        0x01
#define DEBUG_DATA_INJECTION_REG                    0x02
#define DEBUG_CONTROL_REG                           0x03
#define FNA_BP_ID_REG                               0x04
#define SOC_BOOT_COMPLETE_STATUS_REG                0x06
#define HARQ_CONTROL_REG                            0x07
#define LAST_RESET_EVENT_AND_PERIPHERAL_RESET_REG   0x09
#define WATCHDOG_CONTROL_REG                        0x0A
#define WATCHDOG_FEEDING_REG                        0x0B
#define WATCHDOG_TIMEOUT_PERIOD_REG                 0x0C
#define POWER_FAIL_STATUS_1_REG                     0x0F
#define POWER_FAIL_STATUS_2_REG                     0x10
#define POWER_FAIL_STATUS_3_REG                     0x11
#define POWER_FAIL_STATUS_4_REG                     0x12
#define POWER_FAIL_STATUS_5_REG                     0x13
#define OVER_TEMP_ALARM_REG                         0x14
#define FPGA_EXT_SPI_FLASH_CTRL_STATUS_REG          0x16
#define STATUS_LED_CTRL_REG                         0x17
#define NO_POWER_GOOD_COUNTERS_REG                  0x1B
#define FPGA_IMAGE_TYPE_AND_MISC_CTRL_REG           0x1F
#define UTC_TIMESTAMP_RESYNC_0_REG                  0x23
#define UTC_TIMESTAMP_RESYNC_1_REG                  0x24
#define UTC_TIMESTAMP_RESYNC_2_REG                  0x25
#define UTC_TIMESTAMP_RESYNC_3_REG                  0x26


struct gpio_info {
	int  reg;
	int  mask;
	int  mode;
};

struct led_info {
	int reg;
	int shift;
};

typedef enum  {
	BOARD_FCT,
	BOARD_FSP,
	BOARD_FQG,
	BOARD_FQD,
	BOARD_FTLF,
	BOARD_FSPN,
	BOARD_FSPP,
	BOARD_FSPT,
	BOARD_FSPM,
	BOARD_FSPO,
	BOARD_TAD
} board_id;

/*
Example FSPE/P/M/O init strings:
gpio = "<idx> dsp1=7,1,1 dsp2=7,2,1 dsp3=7,4,1 dsp4=7,8,1 dsp5=7,16,1 dsp6=7,32,1 mcu_gpio=10,1,0 mcu_pll=10,2,0";
led = "<idx> unit=11,0";
sfp = "sfp1=<idx>,0,1,0,3,-1,-1,2 sfp2=<idx>,0,5,4,7,-1,-1,6"
*/

/* Client data (each client gets its own) */
#define MAX_LEN_CONFIG_LINE 255
struct bmu_data {
	/* Device type */
	int board_id;

	/* FPGA version read on probe time */
	uint8_t version;

	struct mutex device_lock;
	struct mutex config_lock;
	struct gpio_info *gpios;
	struct led_info *leds;
	struct sfp_gpio_info *sfp_gpios;
	int gpioId, numGpio;
	int ledId, numLed;
	int numSfp;
	char gpio_string[MAX_LEN_CONFIG_LINE+1];
	char led_string[MAX_LEN_CONFIG_LINE+1];
	char sfp_string[MAX_LEN_CONFIG_LINE+1];
};

/* for config line parsing */
#define MAX_BMU_CFG_PARAMS 8
#define MAX_BMU_NAME_LEN 80
struct conf_desc {
	int num_param;
	long params[MAX_BMU_CFG_PARAMS];
	char name[MAX_BMU_NAME_LEN + 1];
};



/* Bit and register access macros below */

/* Register-wide access sysfs functions */

#define show_bmu_reg(reg, hex, name)								\
static ssize_t show_bmu_reg_##name(struct device *dev,				\
			   struct device_attribute *attr, char *buf) {			\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val = 0; ssize_t ret;									\
	dev_dbg(dev, "Locking device\n");								\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU read\n");							\
	ret = readBMUReg(client, &val, reg);							\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if (ret > 0) {													\
		if(hex)														\
			ret = snprintf(buf, PAGE_SIZE - 1, "0x%02X\n", val);	\
		else														\
			ret = snprintf(buf, PAGE_SIZE - 1, "%d\n", val);		\
	}																\
	return ret;														\
}

#define set_bmu_reg(reg, name)										\
static ssize_t set_bmu_reg_##name(struct device *dev,				\
				struct device_attribute *attr,						\
				const char *buf, size_t count) {					\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val; ssize_t ret;										\
	unsigned long value = 0;										\
	if ((kstrtoul(buf, 0, &value) != 0) || (value > 255))			\
		return -EINVAL;												\
	dev_dbg(&client->dev,											\
			"%s: setting value 0x%02X (process %s pid %d)\n",		\
				__func__, (unsigned int)value,						\
				current->comm, current->pid);						\
	val = (uint8_t)value;											\
	dev_dbg(dev, "Locking device\n");								\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU write\n");							\
	ret = writeBMUReg(client, val, reg);							\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if(ret < 0)														\
		return ret;													\
	return count;													\
}

/******************************************************************************
 * Creates register sysfs entry - RW
 *****************************************************************************/
#define SYSFS_REGISTER_ENTRY_RW(reg, hex, name)						\
show_bmu_reg(reg, hex, name);										\
set_bmu_reg(reg, name);												\
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_bmu_reg_##name, set_bmu_reg_##name);


/******************************************************************************
 * Creates register sysfs entry - R
 *****************************************************************************/
#define SYSFS_REGISTER_ENTRY_RD(reg, hex, name)						\
show_bmu_reg(reg, hex, name);										\
static DEVICE_ATTR(name, S_IRUGO, show_bmu_reg_##name, NULL);


/* Bit access sysfs macros */

#define show_bmu_regbit(reg,bit)									\
static ssize_t show_bmu_regbit_##reg##bit(struct device *dev,		\
			   struct device_attribute *attr, char *buf) {			\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val = 0; ssize_t ret;									\
	dev_dbg(dev, "Locking device\n");								\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU read\n");							\
	ret = readBMUReg(client, &val, reg);							\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if (ret > 0)													\
		ret = snprintf(buf, PAGE_SIZE - 1, "%d\n",					\
						!(!(val&(1<<(bit-1)))));					\
	return ret;														\
}
#define set_bmu_regbit(reg,bit)										\
static ssize_t set_bmu_regbit_##reg##bit(struct device *dev,		\
				struct device_attribute *attr,						\
				const char *buf, size_t count) {					\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val = 0; ssize_t ret;									\
	dev_dbg(&client->dev,											\
			"%s: setting value %c (process %s pid %d)\n",			\
			__func__, buf[0], current->comm, current->pid);			\
	dev_dbg(dev, "Locking device\n");								\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU read/write\n");						\
	ret = readBMUReg(client, &val, reg);							\
	val = (val & ~(1<<(bit-1))) | ((buf[0]=='1'?1:0)<<(bit-1));		\
	ret = (ret>0 ? writeBMUReg(client, val, reg) : ret);			\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if(ret < 0)														\
		return ret;													\
	return count;													\
}

#define show_bmu_regbit_inv(reg,bit)								\
static ssize_t show_bmu_regbit_inv_##reg##bit(struct device *dev,	\
			   struct device_attribute *attr, char *buf) {			\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val = 0; ssize_t ret;									\
	dev_dbg(&client->dev,											\
			"%s: setting value %c (process %s pid %d)\n",			\
			__func__, buf[0], current->comm, current->pid);			\
	dev_dbg(dev, "Locking device\n");								\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU read\n");							\
	ret = readBMUReg(client, &val, reg);							\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if (ret > 0)													\
		ret = snprintf(buf, PAGE_SIZE - 1,							\
						"%d\n", !(val&(1<<(bit-1))));				\
	return ret;														\
}
#define set_bmu_regbit_inv(reg,bit)									\
static ssize_t set_bmu_regbit_inv_##reg##bit(struct device *dev,	\
				struct device_attribute *attr,						\
				const char *buf, size_t count) {					\
	struct i2c_client *client = to_i2c_client(dev);					\
	struct bmu_data *data = i2c_get_clientdata(client);				\
	uint8_t val = 0; ssize_t ret;									\
	dev_dbg(dev, "Locking device\n");									\
	dev_dbg(&client->dev,											\
			"%s: setting value !%c (process %s pid %d)\n",			\
			__func__, buf[0], current->comm, current->pid);			\
	mutex_lock(&data->device_lock);									\
	dev_dbg(dev, "Starting BMU read/write\n");							\
	ret = readBMUReg(client, &val, reg);							\
	val = (val & ~(1<<(bit-1))) | ((buf[0]=='1'?0:1)<<(bit-1));		\
	ret = (ret>0 ? writeBMUReg(client, val, reg) : ret);			\
	dev_dbg(dev, "Unlocking device\n");								\
	mutex_unlock(&data->device_lock);								\
	if(ret < 0)														\
		return ret;													\
	return count;													\
}


/* Bitfield access macros */

/******************************************************************************
 * Shows register bitfield
 *****************************************************************************/
#define SHOW_BITFIELD(addr, offset, size, hex, name, inv)			\
static ssize_t show_##name(struct device *dev,						\
							struct device_attribute *attr,			\
							char *buf)								\
{																	\
	struct i2c_client *client = to_i2c_client(dev);					\
	uint8_t val, mask = (0xFF >> (8 - size));						\
	ssize_t ret;													\
	ret = bmu_bitfield_read(client, addr, offset, size, &val);		\
	if (ret < 0)													\
	{																\
		dev_err(&client->dev, "%s: failed (%d)",					\
				__func__, (int)ret);								\
		return ret;													\
	}																\
	if (inv)														\
		val = (~val & mask);										\
	if(hex)															\
		return snprintf(buf, PAGE_SIZE - 1, "0x%02X\n", val);		\
	else															\
		return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);			\
}

/******************************************************************************
 * Sets register bitfield
 *****************************************************************************/
#define SET_BITFIELD(addr, offset, size, name, inv)					\
static ssize_t set_##name(struct device *dev,						\
							struct device_attribute *attr,			\
							const char *buf, size_t count)			\
{																	\
	struct i2c_client *client = to_i2c_client(dev);					\
	uint8_t val;													\
	unsigned long tmp;												\
	ssize_t ret;													\
	if ((kstrtoul(buf, 0, &tmp) != 0) || (tmp >= (1 << size)))			\
		return -EINVAL;												\
	dev_dbg(&client->dev,											\
			"%s: setting value %lu (process %s pid %d)\n",			\
			__func__, tmp, current->comm, current->pid);			\
	val = (uint8_t)tmp;												\
	if (inv)														\
		val = ~val;													\
	ret = bmu_bitfield_write(client, addr, offset, size, val);		\
	if (ret < 0)													\
	{																\
		dev_err(&client->dev,										\
				"%s: failed (%d)", __func__, (int)ret);				\
		return ret;													\
	}																\
	return count;													\
}


/******************************************************************************
 * Creates bitfield sysfs entry - RW
 *****************************************************************************/
#define SYSFS_BITFIELD_ENTRY_RW(addr, offset, size, hex, name)		\
SHOW_BITFIELD(addr, offset, size, hex, name, 0);					\
SET_BITFIELD(addr, offset, size, name, 0);							\
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_##name, set_##name);

#define SYSFS_BITFIELD_ENTRY_INV_RW(addr, offset, size, hex, name)		\
SHOW_BITFIELD(addr, offset, size, hex, name, 1);						\
SET_BITFIELD(addr, offset, size, name, 1);								\
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_##name, set_##name);

/******************************************************************************
 * Creates bitfield sysfs entry - R
 *****************************************************************************/
#define SYSFS_BITFIELD_ENTRY_RD(addr, offset, size, hex, name)		\
SHOW_BITFIELD(addr, offset, size, hex, name, 0);						\
static DEVICE_ATTR(name, S_IRUGO, show_##name, NULL);

#define SYSFS_BITFIELD_ENTRY_INV_RD(addr, offset, size, hex, name)		\
SHOW_BITFIELD(addr, offset, size, hex, name, 1);						\
static DEVICE_ATTR(name, S_IRUGO, show_##name, NULL);


#endif /* __BMUQT_H_ */
