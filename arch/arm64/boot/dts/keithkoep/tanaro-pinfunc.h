/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 Keith & Koep GmbH
 *
 * This file works as abstraction layer between different Trizeps modules.
 * It is based on imx8mm-pinfuncs.h
 * 
 */

#ifndef __DTS_TANARO_PINFUNC_H
#define __DTS_TANARO_PINFUNC_H

#define KK_RSRVD_IN	            (1 << 0)
#define KK_RSRVD_OUT            (0 << 0)
#define KK_RSRVD_CHANGEABLE     (1 << 6)
#define KK_RSRVD_OUT_LO		    ((0 << 1) | KK_RSRVD_OUT)
#define KK_RSRVD_OUT_HI		    ((1 << 1) | KK_RSRVD_OUT)
#define KK_RSRVD_REQUEST(_idx)	((1 << 8) | ((_idx) << 16))
#define KK_RSRVD_EXPORT			(1 << 5)					// @+15.05.2017 S&B HL

#define PAD_GPIO_PU  (MX8MM_IOMUXC_PULLUP_ENABLE|MX8MM_IOMUXC_HYS|MX8MM_IOMUXC_SLOW|MX8MM_IOMUXC_DRIVE(6))
#define PAD_GPIO_PD  (MX8MM_IOMUXC_PULLDOWN_ENABLE|MX8MM_IOMUXC_HYS|MX8MM_IOMUXC_SLOW|MX8MM_IOMUXC_DRIVE(6))
#define PAD_GPIO_HYS (MX8MM_IOMUXC_HYS|MX8MM_IOMUXC_SLOW|MX8MM_IOMUXC_DRIVE(6))
#define PAD_GPIO     (MX8MM_IOMUXC_FAST|MX8MM_IOMUXC_DRIVE(6))
#define PAD_GPIO_OD  (MX8MM_IOMUXC_PULLUP_ENABLE|MX8MM_IOMUXC_HYS|MX8MM_IOMUXC_OPEN_DRAIN|MX8MM_IOMUXC_SLOW|MX8MM_IOMUXC_DRIVE(6))

/*
 * The pin function ID is a tuple of
 * <mux_reg conf_reg input_reg mux_mode input_val>
 */

#define TOUCH_INT                                           0x104 0x36C 0x000 0x5 0x0
#define touch_int           &gpio3 4
#define touch_int_parent    &gpio3
#define touch_int_pin       4
#define TOUCH_RESET                                         MX8MM_IOMUXC_GPIO1_IO00_GPIO1_IO0
#define touch_reset         &gpio1 0
#define DISPLAY_ENABLE                                      0x03C 0x2A4 0x000 0x0 0x0
#define display_enable      &gpio1 5
#define CAMERA_PWDN                                         0x034 0x29C 0x000 0x0 0x0
#define camera_pwdn         &gpio1 3
#define CAMERA_RESET                                        0x040 0x2A8 0x000 0x0 0x0
#define camera_reset        &gpio1 6

#endif
