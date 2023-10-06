/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (C) 2018 ROHM Semiconductors */

#ifndef __LINUX_MFD_TR8MCU_H__
#define __LINUX_MFD_TR8MCU_H__

#include <linux/types.h>

#define TR8MCU_REG_ID				0x01
#define TR8MCU_REG_CONTROL			0x02
#define TR8MCU_REG_CONFIG1			0x03
#define TR8MCU_REG_PIN_CONFIG		0x04
#define TR8MCU_REG_PIN_SET			0x05
#define TR8MCU_REG_PIN_GET			0x06
#define TR8MCU_REG_ADC_CONFIG		0x10
#define TR8MCU_REG_ADC				0x11
#define TR8MCU_REG_LSB				0x12
#define TR8MCU_REG_MSB				0x13

#define TR8MCU_REG_TOUCH_CONFIG		 0x20
#define TR8MCU_REG_TOUCH_WAITTIME	 0x21
#define TR8MCU_REG_TOUCH_XSCALE_LSB	 0x22
#define TR8MCU_REG_TOUCH_XSCALE_MSB	 0x23
#define TR8MCU_REG_TOUCH_YSCALE_LSB	 0x24
#define TR8MCU_REG_TOUCH_YSCALE_MSB	 0x25
#define TR8MCU_REG_TOUCH_XMIN_LSB	 0x26
#define TR8MCU_REG_TOUCH_XMIN_MSB	 0x27
#define TR8MCU_REG_TOUCH_YMIN_LSB	 0x28
#define TR8MCU_REG_TOUCH_YMIN_MSB	 0x29
#define TR8MCU_REG_TOUCH_XMAX_Z_LSB	 0x2A
#define TR8MCU_REG_TOUCH_XMAX_Z_MSB	 0x2B
#define TR8MCU_REG_TOUCH_YMAX_LSB	 0x2C
#define TR8MCU_REG_TOUCH_YMAX_MSB	 0x2D
#define TR8MCU_REG_TOUCH_AVERAGE	 0x2E

#define TR8MCU_REG_CONFIG1_RESET2USBBOOT	0x01
#define TR8MCU_REG_CONFIG1_SDVSEL			0x02


struct tr8mcu;

/* Read up to 4 bytes from the MCU
   Returns 0 or a negative error code
*/
int tr8mcu_read(struct tr8mcu * tr8mcu, u8 reg, int len, u32 * data);

/* Read up to 4 bytes from the MCU, specified by reg and param.
   Returns 0 or a negative error code
*/
int tr8mcu_read_with_param(struct tr8mcu * tr8mcu, u8 reg, u8 param, int len,
						   u32 * data);
/* Write up to 4 bytes to the MCU
   Returns 0 or a negative error code
*/
int tr8mcu_write(struct tr8mcu * tr8mcu, u8 reg, int len, u32 data);

#endif /* __LINUX_MFD_TR8TR8MCU_H__ */
