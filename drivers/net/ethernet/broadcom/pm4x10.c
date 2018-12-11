/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/soc/bcm/iproc-cmic.h>
#include <asm/io.h>
#include "pm.h"
#include "merlin16_ucode.h"
#include "merlin16_regs.h"

#define JUMBO_MAXSZ			0x3fe8

#define IPROC_WRAP_MISC_CONTROL			0x10220c18
#define IPROC_WRAP_MISC_CONTROL_IPROC_PM_RST_L	1
extern void __iomem *get_iproc_wrap_ctrl_base(void);

#define debug(fmt, args...) do {} while (0)

/* TSC operation for data[2] (bit 64 in the whole string) */
#define TSC_WRITE			0x1
#define TSC_READ			0x0

/* S-channel address */
/* Per port registers */
#define XLMIB_GRx64(port)		(0x00000000 + port)
#define XLMIB_GRx127(port)		(0x00000100 + port)
#define XLMIB_GRx255(port)		(0x00000200 + port)
#define XLMIB_GRx511(port)		(0x00000300 + port)
#define XLMIB_GRx1023(port)		(0x00000400 + port)
#define XLMIB_GRx1518(port)		(0x00000500 + port)
#define XLMIB_GRx1522(port)		(0x00000600 + port)
#define XLMIB_GRx2047(port)		(0x00000700 + port)
#define XLMIB_GRx4095(port)		(0x00000800 + port)
#define XLMIB_GRx9216(port)		(0x00000900 + port)
#define XLMIB_GRx16383(port)		(0x00000a00 + port)
#define XLMIB_GRxPkt(port)		(0x00000b00 + port)
#define XLMIB_GRxUCA(port)		(0x00000c00 + port)
#define XLMIB_GRxMCA(port)		(0x00000d00 + port)
#define XLMIB_GRxBCA(port)		(0x00000e00 + port)
#define XLMIB_GRxFCS(port)		(0x00000f00 + port)
#define XLMIB_GRxCF(port)		(0x00001000 + port)
#define XLMIB_GRxPF(port)		(0x00001100 + port)
#define XLMIB_GRxUO(port)		(0x00001300 + port)
#define XLMIB_GRxWSA(port)		(0x00001500 + port)
#define XLMIB_GRxALN(port)		(0x00001600 + port)
#define XLMIB_GRxFLR(port)		(0x00001700 + port)
#define XLMIB_GRxOVR(port)		(0x00001a00 + port)
#define XLMIB_GRxJBR(port)		(0x00001b00 + port)
#define XLMIB_GRxMTUE(port)		(0x00001c00 + port)
#define XLMIB_GRxVLN(port)		(0x00001f00 + port)
#define XLMIB_GRxDVLN(port)		(0x00002000 + port)
#define XLMIB_GRxTRFU(port)		(0x00002100 + port)
#define XLMIB_GRxPOK(port)		(0x00002200 + port)
#define XLMIB_GRxUND(port)		(0x00003400 + port)
#define XLMIB_GRxFRG(port)		(0x00003500 + port)
#define XLMIB_GRxByt(port)		(0x00003d00 + port)
#define XLMIB_GTx64(port)		(0x00004000 + port)
#define XLMIB_GTx127(port)		(0x00004100 + port)
#define XLMIB_GTx255(port)		(0x00004200 + port)
#define XLMIB_GTx511(port)		(0x00004300 + port)
#define XLMIB_GTx1023(port)		(0x00004400 + port)
#define XLMIB_GTx1518(port)		(0x00004500 + port)
#define XLMIB_GTx1522(port)		(0x00004600 + port)
#define XLMIB_GTx2047(port)		(0x00004700 + port)
#define XLMIB_GTx4095(port)		(0x00004800 + port)
#define XLMIB_GTx9216(port)		(0x00004900 + port)
#define XLMIB_GTx16383(port)		(0x00004a00 + port)
#define XLMIB_GTxPOK(port)		(0x00004b00 + port)
#define XLMIB_GTxPkt(port)		(0x00004c00 + port)
#define XLMIB_GTxUCA(port)		(0x00004d00 + port)
#define XLMIB_GTxMCA(port)		(0x00004e00 + port)
#define XLMIB_GTxBCA(port)		(0x00004f00 + port)
#define XLMIB_GTxPF(port)		(0x00005000 + port)
#define XLMIB_GTxJBR(port)		(0x00005200 + port)
#define XLMIB_GTxFCS(port)		(0x00005300 + port)
#define XLMIB_GTxCF(port)		(0x00005400 + port)
#define XLMIB_GTxOVR(port)		(0x00005500 + port)
#define XLMIB_GTxFRG(port)		(0x00005c00 + port)
#define XLMIB_GTxErr(port)		(0x00005d00 + port)
#define XLMIB_GTxVLN(port)		(0x00005e00 + port)
#define XLMIB_GTxDVLN(port)		(0x00005f00 + port)
#define XLMIB_GTxUFL(port)		(0x00006100 + port)
#define XLMIB_GTxNCL(port)		(0x00006e00 + port)
#define XLMIB_GTxBYT(port)		(0x00006f00 + port)

#define XLPORT_CONFIG(port)		(0x00020000 + port)
#define XLMAC_CTRL(port)		(0x00060000 + port)
#define  XLMAC_CTRL__SW_LINK_STATUS			12
#define  XLMAC_CTRL__XGMII_IPG_CHECK_DISABLE		11
#define  XLMAC_CTRL__SOFT_RESET				6
#define  XLMAC_CTRL__LOCAL_LPBK				2
#define  XLMAC_CTRL__RX_EN				1
#define  XLMAC_CTRL__TX_EN				0
#define XLMAC_MODE(port)		(0x00060100 + port)
#define  XLMAC_MODE_OFFSET				0x1
#define  XLMAC_MODE__SPEED_MODE_L			6
#define  XLMAC_MODE__SPEED_MODE_R			4
#define  XLMAC_MODE__SPEED_MODE_WIDTH			3
#define   SPEED_MODE_LINK_10M				0x0
#define   SPEED_MODE_LINK_100M				0x1
#define   SPEED_MODE_LINK_1G				0x2
#define   SPEED_MODE_LINK_2G5				0x3
#define   SPEED_MODE_LINK_10G_PLUS			0x4
#define  XLMAC_MODE__SPEED_MODE_RESETVALUE		0x4
#define  XLMAC_MODE__NO_SOP_FOR_CRC_HG			3
#define  XLMAC_MODE__NO_SOP_FOR_CRC_HG_WIDTH		1
#define  XLMAC_MODE__NO_SOP_FOR_CRC_HG_RESETVALUE	0x0
#define  XLMAC_MODE__HDR_MODE_L				2
#define  XLMAC_MODE__HDR_MODE_R				0
#define  XLMAC_MODE__HDR_MODE_WIDTH 			3
#define   HDR_MODE_IEEE					0x0
#define   HDR_MODE_HG_PLUS				0x1
#define   HDR_MODE_HG_2					0x2
#define   HDR_MODE_SOP_ONLY_IEEE			0x5
#define XLMAC_TX_CTRL(port)		(0x00060400 + port)
#define  XLMAC_TX_CTRL__AVERAGE_IPG_R			12
#define  XLMAC_TX_CTRL__PAD_EN				4
#define  XLMAC_TX_CTRL__CRC_MODE_R			0
#define   CRC_MODE_APPEND				0x0
#define   CRC_MODE_KEEP					0x1
#define   CRC_MODE_REPLACE				0x2
#define   CRC_MODE_PER_PKT_MODE				0x3
#define XLMAC_TX_MAC_SA(port)		(0x00060500 + port)
#define XLMAC_RX_MAX_SIZE(port)		(0x00060800 + port)
#define  XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_L		13
#define  XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_R		0
#define  XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_WIDTH		14
#define XLMAC_RX_CTRL(port)		(0x00060600 + port)
#define  XLMAC_RX_CTRL__STRIP_CRC			2
#define XLMAC_RX_LSS_CTRL(port)		(0x00060a00 + port)
#define  XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_LINK_INTERRUPT	6
#define  XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_REMOTE_FAULT	5
#define  XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_LOCAL_FAULT		4
#define  XLMAC_RX_LSS_CTRL__REMOTE_FAULT_DISABLE		1
#define  XLMAC_RX_LSS_CTRL__LOCAL_FAULT_DISABLE			0
#define XLMAC_PAUSE_CTRL(port)		(0x00060d00 + port)
#define  XLMAC_PAUSE_CTRL__RX_PAUSE_EN			18
#define  XLMAC_PAUSE_CTRL__TX_PAUSE_EN			17
#define XLMAC_PFC_CTRL(port)		(0x00060e00 + port)
#define  XLMAC_PFC_CTRL__PFC_REFRESH_EN			32
/* General type registers */
#define XLPORT_MODE_REG			(0x02020a00)
#define  XLPORT_MODE_REG__RESET_MASK			0x3f
#define  XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_L	5
#define  XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_R	3
#define  XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_WIDTH	3
#define   XPORT0_CORE_PORT_MODE_QUAD			0x0
#define   XPORT0_CORE_PORT_MODE_TRI_012			0x1
#define   XPORT0_CORE_PORT_MODE_TRI_023			0x2
#define   XPORT0_CORE_PORT_MODE_DUAL			0x3
#define   XPORT0_CORE_PORT_MODE_SINGLE			0x4
#define  XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_L	2
#define  XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_R	0
#define  XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_WIDTH 	3
#define   XPORT0_PHY_PORT_MODE_QUAD			0x0
#define   XPORT0_PHY_PORT_MODE_TRI_012			0x1
#define   XPORT0_PHY_PORT_MODE_TRI_023			0x2
#define   XPORT0_PHY_PORT_MODE_DUAL			0x3
#define   XPORT0_PHY_PORT_MODE_SINGLE			0x4
#define XLPORT_ENABLE_REG		(0x02020b00)
#define  XLPORT_ENABLE_REG__PORT3			3
#define  XLPORT_ENABLE_REG__PORT2			2
#define  XLPORT_ENABLE_REG__PORT1			1
#define  XLPORT_ENABLE_REG__PORT0			0
#define XLPORT_MAC_CONTROL		(0x02021000)
#define  XLPORT_MAC_CONTROL__RX_DUAL_CYCLE_TDM_EN	5
#define  XLPORT_MAC_CONTROL__RX_NON_LINEAR_QUAD_TDM_EN	3
#define  XLPORT_MAC_CONTROL__RX_FLEX_TDM_ENABLE		2
#define  XLPORT_MAC_CONTROL__XMAC0_BYPASS_OSTS		1
#define  XLPORT_MAC_CONTROL__XMAC0_RESET		0
#define XLPORT_XGXS0_CTRL_REG		(0x02021400)
#define  XLPORT_XGXS0_CTRL_REG__RefSel			8
#define  XLPORT_XGXS0_CTRL_REG__RefCMOS       		7
#define  XLPORT_XGXS0_CTRL_REG__Pwrdwn_CML_LC 		6
#define  XLPORT_XGXS0_CTRL_REG__Pwrdwn_CML		5
#define  XLPORT_XGXS0_CTRL_REG__IDDQ			4
#define  XLPORT_XGXS0_CTRL_REG__PWRDWN			3
#define  XLPORT_XGXS0_CTRL_REG__Refin_EN		2
#define  XLPORT_XGXS0_CTRL_REG__RSTB_HW			0
#define XLPORT_WC_UCMEM_CTRL		(0x02021900)
#define  XLPORT_WC_UCMEM_CTRL__ACCESS_MODE		0
#define XLPORT_MIB_RESET		(0x02022400)
#define  XLPORT_MIB_RESET__CLR_CNT_L			3
#define  XLPORT_MIB_RESET__CLR_CNT_R			0
#define XLPORT_INTR_STATUS		(0x02022900)
#define XLPORT_INTR_ENABLE		(0x02022a00)
#define XLPORT_SOFT_RESET		(0x02020c00)
#define  XLPORT_SOFT_RESET__PORT3			3
#define  XLPORT_SOFT_RESET__PORT2			2
#define  XLPORT_SOFT_RESET__PORT1			1
#define  XLPORT_SOFT_RESET__PORT0			0
#define XLPORT_POWER_SAVE		(0x02020d00)
#define  XLPORT_POWER_SAVE__XPORT_CORE0			0

#define XLPORT_PORT_FIELD(reg, port)	reg##__PORT##port
#define XLPORT_PORT_FIELD_SET(_r, _p, _v)  { \
	if (_p == 0)    	val |= (1 << _r##__PORT0); \
	else if (_p == 1)   val |= (1 << _r##__PORT1); \
	else if (_p == 2)   val |= (1 << _r##__PORT2); \
	else if (_p == 3)   val |= (1 << _r##__PORT3); \
}

#define XLPORT_PORT_FIELD_CLEAR(_r, _p, _v)  { \
	if (_p == 0)    	val &= ~(1 << _r##__PORT0); \
	else if (_p == 1)   val &= ~(1 << _r##__PORT1); \
	else if (_p == 2)   val &= ~(1 << _r##__PORT2); \
	else if (_p == 3)   val &= ~(1 << _r##__PORT3); \
}

static u32 pm4x10_enabled = 0;


static inline void
xlmac_reg64_write(u32 addr, u64 val)
{
	iproc_cmic_schan_reg64_write(CMIC_BLOCK_TYPE_APM, addr, val);
}

static inline u64
xlmac_reg64_read(u32 addr)
{
	return iproc_cmic_schan_reg64_read(CMIC_BLOCK_TYPE_APM, addr);
}

static inline void
xlport_reg32_write(u32 addr, u32 val)
{
	iproc_cmic_schan_reg32_write(CMIC_BLOCK_TYPE_APM, addr, val);
}

static inline u32
xlport_reg32_read(u32 addr)
{
	return iproc_cmic_schan_reg32_read(CMIC_BLOCK_TYPE_APM, addr);
}


/* MDIO address for each lane in this PM */
static u32 lane_mdio_addr[4] = { 3, 4, 5, 6 };

static inline void
pm_phy_sbus_write(u32 lane, u32 addr, u32 val, u32 mask, u32 shift)
{
	u32 device, mem_data[4];

	/* TSC register address (indirect access) */
	if ((addr == 0x0002) || (addr == 0x0003) || ((addr <= 0xc340) && (addr >= 0x9000)))
		device = 0; /* PCS (TSC) */
	else
		device = 1; /* PMA/PMD (Physical Media Device or called serdes(merlin)) */

	mem_data[0] = (device << 27) | (lane_mdio_addr[lane] << 19) | (lane << 16) | addr;
	mem_data[1] = ((val << shift) << 16) | (~(mask << shift) & 0xffff);
	mem_data[2] = TSC_WRITE;
	mem_data[3] = 0;
	iproc_cmic_schan_ucmem_write(CMIC_BLOCK_TYPE_APM, mem_data);
}

static inline u32
pm_phy_sbus_read(u32 lane, u32 addr, u32 *val)
{
	u32 device, mem_data[4];

	/* TSC register address (indirect access) */
	if ((addr == 0x0002) || (addr == 0x0003) || ((addr <= 0xc340) && (addr >= 0x9000)))
		device = 0; /* PCS (TSC) */
	else
		device = 1; /* PMA/PMD (Physical Media Device or called serdes(merlin)) */

	mem_data[0] = (device << 27) | (lane_mdio_addr[lane] << 19) | (lane << 16) | addr;
	mem_data[1] = 0;
	mem_data[2] = TSC_READ;
	mem_data[3] = 0;
	iproc_cmic_schan_ucmem_write(CMIC_BLOCK_TYPE_APM, mem_data);
	*val = iproc_cmic_schan_ucmem_read(CMIC_BLOCK_TYPE_APM, mem_data);
	return 0;
}

static int __xlmac_credit_reset(int port)
{
	return 0;
}

static void __xlmac_enable_set(int port, bool enable)
{
	u64 ctrl, octrl;
	int soft_reset;

	ctrl = xlmac_reg64_read(XLMAC_CTRL(port));
	octrl = ctrl;
	/* Don't disable TX since it stops egress and hangs if CPU sends */
	ctrl |= (1 << XLMAC_CTRL__TX_EN);
	ctrl &= ~(1 << XLMAC_CTRL__RX_EN);
	if (enable) {
		ctrl |= (1 << XLMAC_CTRL__RX_EN);
	}

	if (ctrl == octrl) {
		/* SDK-49952 fix soluition :
		 *  >> to avoid the unexpected early return to prevent this problem.
		 *  1. Problem occurred for disabling process only.
		 *  2. To comply origianl designing senario, XLMAC_CTRLr.SOFT_RESETf is
		 *	  used to early check to see if this port is at disabled state
		 *	  already.
		 */
		soft_reset = ctrl & (1 << XLMAC_CTRL__SOFT_RESET);
		if ((enable) || (!enable && soft_reset)){
			return;
		}
	}

	ctrl |= (1 << XLMAC_CTRL__SOFT_RESET);
	if (enable) {
		/* Reset EP credit before de-assert SOFT_RESET */
		__xlmac_credit_reset(port);
		/* Deassert SOFT_RESET */
		ctrl &= ~(1 << XLMAC_CTRL__SOFT_RESET);
	}

	xlmac_reg64_write(XLMAC_CTRL(port), ctrl);
}

static int __xlmac_enable_get(int port)
{
	u64 ctrl;
	int tx_en, rx_en;

	ctrl = xlmac_reg64_read(XLMAC_CTRL(port));
	tx_en = ctrl & (1 << XLMAC_CTRL__TX_EN);
	rx_en = ctrl & (1 << XLMAC_CTRL__RX_EN);

	return (tx_en && rx_en);
}

static int __xlmac_speed_set(int port, int speed)
{
	u64 speed_cfg, val64;
	int enable;

	pm_phy_sbus_write(port, CKRST_CTRL_OSR_MODE_CONTROL, 0x8008, 0x8008, CKRST_CTRL_OSR_MODE_CONTROL_osr_mode_frc_val);

	if (speed == 1000) {
		speed_cfg = SPEED_MODE_LINK_1G;
		pm_phy_sbus_write(port, SC_X4_CONTROL_CONTROL, 0x0037, 0x01ff, SC_X4_CONTROL_CONTROL_sw_speed);
	} else if (speed == 100) {
		speed_cfg = SPEED_MODE_LINK_100M;
		pm_phy_sbus_write(port, SC_X4_CONTROL_CONTROL, 0x0036, 0x01ff, SC_X4_CONTROL_CONTROL_sw_speed);
	} else if (speed == 10) {
		speed_cfg = SPEED_MODE_LINK_10M;
		pm_phy_sbus_write(port, SC_X4_CONTROL_CONTROL, 0x0035, 0x01ff, SC_X4_CONTROL_CONTROL_sw_speed);
	} else {
		debug("(%s) Invalid xlport speed(%d)!\n", __func__, speed);
		return -1;
	}
	pm_phy_sbus_write(port, SC_X4_CONTROL_CONTROL, 0x0000, 0x0001, SC_X4_CONTROL_CONTROL_sw_speed_change);
	pm_phy_sbus_write(port, SC_X4_CONTROL_CONTROL, 0x0001, 0x0001, SC_X4_CONTROL_CONTROL_sw_speed_change);

	enable = __xlmac_enable_get(port);
	/* disable before updating the speed */
	if (enable) {
		__xlmac_enable_set(port, 0);
	}

	/* Update the speed */
	val64 = xlmac_reg64_read(XLMAC_MODE(port));
	val64 &= ~(0x70);
	val64 |= speed_cfg << XLMAC_MODE__SPEED_MODE_R;
	xlmac_reg64_write(XLMAC_MODE(port), val64);
	debug("(%s) XLMAC_MODE = 0x%llx\n",  __func__, xlmac_reg64_read(XLMAC_MODE(port)));

	if (enable) {
		__xlmac_enable_set(port, 1);
	}
	return 0;
}

static int __xlmac_rx_max_size_set(int port, int value)
{
	u64 val64;
	u64 mask64;

	val64 = xlmac_reg64_read(XLMAC_RX_MAX_SIZE(port));
	mask64 = (1 << XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_WIDTH) - 1;
	val64 &= ~(mask64 << XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_R);
	val64 |= value << XLMAC_RX_MAX_SIZE__RX_MAX_SIZE_R;
	xlmac_reg64_write(XLMAC_RX_MAX_SIZE(port), val64);

	return 0;
}

static int __xlmac_tx_mac_addr_set(int port, u8 *mac)
{
	u64 val64;

	/* set our local address */
	debug("GMAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	val64 = (u64)htonl(*(u32 *)&mac[2]);
	val64 |= ((u64)htons(*(u32 *)mac) << 32);
	xlmac_reg64_write(XLMAC_TX_MAC_SA(port), val64);

	debug("(%s) XLMAC_TX_MAC_SA = 0x%llx\n",  __func__, xlmac_reg64_read(XLMAC_TX_MAC_SA(port)));
	return 0;
}

static int __xlmac_init(int port)
{
	u64 val64;

	/* Disable Tx/Rx, assume that MAC is stable (or out of reset) */
	val64 = xlmac_reg64_read(XLMAC_CTRL(port));
	val64 &= ~(1 << XLMAC_CTRL__XGMII_IPG_CHECK_DISABLE);
	val64 &= ~(1 << XLMAC_CTRL__RX_EN);
	val64 &= ~(1 << XLMAC_CTRL__TX_EN);
	xlmac_reg64_write(XLMAC_CTRL(port), val64);

	/* XLMAC_RX_CTRL */
	val64 = xlmac_reg64_read(XLMAC_RX_CTRL(port));
	val64 &= ~(1 << XLMAC_RX_CTRL__STRIP_CRC);
	xlmac_reg64_write(XLMAC_RX_CTRL(port), val64);

	/* XLMAC_TX_CTRL */
	val64 = xlmac_reg64_read(XLMAC_TX_CTRL(port));
	val64 &= ~(0x3 << XLMAC_TX_CTRL__CRC_MODE_R);
	val64 |= (CRC_MODE_APPEND << XLMAC_TX_CTRL__CRC_MODE_R);
	val64 |= (1 << XLMAC_TX_CTRL__PAD_EN);
	xlmac_reg64_write(XLMAC_TX_CTRL(port), val64);

	/* PAUSE */
	val64 = xlmac_reg64_read(XLMAC_PAUSE_CTRL(port));
	val64 |= 1 << XLMAC_PAUSE_CTRL__RX_PAUSE_EN;
	val64 |= 1 << XLMAC_PAUSE_CTRL__TX_PAUSE_EN;
	xlmac_reg64_write(XLMAC_PAUSE_CTRL(port), val64);

	/* PFC */
	val64 = xlmac_reg64_read(XLMAC_PFC_CTRL(port));
	val64 |= ((u64)1 << XLMAC_PFC_CTRL__PFC_REFRESH_EN);
	xlmac_reg64_write(XLMAC_PFC_CTRL(port), val64);

	/* Set jumbo max size (8000 byte payload) */
	__xlmac_rx_max_size_set(port, JUMBO_MAXSZ);

	/* XLMAC_RX_LSS_CTRL */
	val64 = xlmac_reg64_read(XLMAC_RX_LSS_CTRL(port));
	val64 |= 1 << XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_LINK_INTERRUPT;
	val64 |= 1 << XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_REMOTE_FAULT;
	val64 |= 1 << XLMAC_RX_LSS_CTRL__DROP_TX_DATA_ON_LOCAL_FAULT;
	xlmac_reg64_write(XLMAC_RX_LSS_CTRL(port), val64);

	/* Disable loopback and bring XLMAC out of reset */
	val64 = xlmac_reg64_read(XLMAC_CTRL(port));
	val64 &= ~(1 << XLMAC_CTRL__SOFT_RESET);;
	val64 &= ~(1 << XLMAC_CTRL__LOCAL_LPBK);
	val64 |= 1 << XLMAC_CTRL__RX_EN;
	val64 |= 1 << XLMAC_CTRL__TX_EN;
	xlmac_reg64_write(XLMAC_CTRL(port), val64);

	return 0;
}

static int __tsc_reset(int in_reset)
{
	u32 val = xlport_reg32_read(XLPORT_XGXS0_CTRL_REG);

	if (in_reset) {
		/* Assert IDDQ */
		val |= (1 << XLPORT_XGXS0_CTRL_REG__IDDQ);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(1);

		/* Assert power down */
		val |= 1 << XLPORT_XGXS0_CTRL_REG__PWRDWN;
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(1);

		/* Reset XGXS */
		val &= ~(1 << XLPORT_XGXS0_CTRL_REG__RSTB_HW);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(11);
	} else {
		/* Select Internal LC Reference Input (default for A0) */
		val &= ~(7 << XLPORT_XGXS0_CTRL_REG__RefSel);
		val |= (5 << XLPORT_XGXS0_CTRL_REG__RefSel);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);

		/* Deassert IDDQ */
		val &= ~(1 << XLPORT_XGXS0_CTRL_REG__IDDQ);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(1);

		/* Reference clock selection */
		val |= (1 << XLPORT_XGXS0_CTRL_REG__Refin_EN);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);

		/* Deassert power down */
		val &= ~(1 << XLPORT_XGXS0_CTRL_REG__PWRDWN);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(1);

		/* Reset XGXS */
		val &= ~(0 << XLPORT_XGXS0_CTRL_REG__RSTB_HW);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(11);

		/* Bring XGXS out of reset */
		val |= (1 << XLPORT_XGXS0_CTRL_REG__RSTB_HW);
		xlport_reg32_write(XLPORT_XGXS0_CTRL_REG, val);
		msleep(1);
	}

	return 0;
}

/*****************************************************************************
*****************************************************************************/
static void pm4x10_xlport_mac_enable(int port)
{
	__xlmac_enable_set(port, 1);
}

static void pm4x10_xlport_mac_disable(int port)
{
	__xlmac_enable_set(port, 0);
}

int pm4x10_xlport_speed_set(int port, int speed)
{
	return __xlmac_speed_set(port, speed);
}

int pm4x10_xlport_mac_addr_set(int port, u8 *mac)
{
	return __xlmac_tx_mac_addr_set(port, mac);
}

static int pm4x10_xlport_max_packet_size_set(int port, int value)
{
	return __xlmac_rx_max_size_set(port, value);
}

int pm4x10_xlport_loopback_set(int port, int lb_type, int lb_en)
{
	u64 val64;
	u32 val32;

	switch(lb_type) {
		case pmLoopbackMac:
			val64 = xlmac_reg64_read(XLMAC_CTRL(port));
	    val64 &= ~(1 << XLMAC_CTRL__LOCAL_LPBK);
	    if (lb_en) {
		    debug("(%s) MAC port = %d, lb_en = %d\n", __func__, port, lb_en);
		    val64 |= (1 << XLMAC_CTRL__LOCAL_LPBK);
	    }
	    xlmac_reg64_write(XLMAC_CTRL(port), val64);
			break;

		case pmLoopbackPhy:
			if (lb_en) {
				debug("(%s) PHY port = %d, lb_en = %d\n", __func__, port, lb_en);
				pm_phy_sbus_write(port, MAIN0_LOOPBACK_CONTROL, (1 << port), 0x000f, MAIN0_LOOPBACK_CONTROL_local_pcs_loopback_enable);
				pm_phy_sbus_write(port, PMD_X4_OVERRIDE, 0x0043, 0x0043, PMD_X4_OVERRIDE_rx_lock_ovrd);
				pm_phy_sbus_write(port, PMD_X4_CONTROL, 0x0001, 0x0001, PMD_X4_CONTROL_tx_disable);
			} else {
				pm_phy_sbus_read(port, MAIN0_LOOPBACK_CONTROL, &val32);
				val32 &= ~(1 << port);
				pm_phy_sbus_write(port, MAIN0_LOOPBACK_CONTROL, val32, 0x000f, MAIN0_LOOPBACK_CONTROL_local_pcs_loopback_enable);
				pm_phy_sbus_write(port, PMD_X4_OVERRIDE, 0x0000, 0x0043, PMD_X4_OVERRIDE_rx_lock_ovrd);
				pm_phy_sbus_write(port, PMD_X4_CONTROL, 0x0000, 0x0001, PMD_X4_CONTROL_tx_disable);
			}
			break;

		default:
			break;
	}
	return 0;
}


int pm4x10_xlport_stats_get(int port, struct iproc_pm_stats *stats)
{
	stats->rx_frames = xlmac_reg64_read(XLMIB_GRxPkt(port));
	stats->rx_frame_good = xlmac_reg64_read(XLMIB_GRxPOK(port));
	stats->rx_bytes = xlmac_reg64_read(XLMIB_GRxByt(port));
	stats->rx_frame_64 = xlmac_reg64_read(XLMIB_GRx64(port));
	stats->rx_frame_127 = xlmac_reg64_read(XLMIB_GRx127(port));
	stats->rx_frame_255 = xlmac_reg64_read(XLMIB_GRx255(port));
	stats->rx_frame_511 = xlmac_reg64_read(XLMIB_GRx511(port));
	stats->rx_frame_1023 = xlmac_reg64_read(XLMIB_GRx1023(port));
	stats->rx_frame_1518 = xlmac_reg64_read(XLMIB_GRx1518(port));
	stats->rx_frame_1522 = xlmac_reg64_read(XLMIB_GRx1522(port));
	stats->rx_frame_jumbo = xlmac_reg64_read(XLMIB_GRx2047(port)) +
							xlmac_reg64_read(XLMIB_GRx4095(port)) +
							xlmac_reg64_read(XLMIB_GRx9216(port)) +
							xlmac_reg64_read(XLMIB_GRx16383(port));
	stats->rx_frame_unicast = xlmac_reg64_read(XLMIB_GRxUCA(port));
	stats->rx_frame_multicast = xlmac_reg64_read(XLMIB_GRxMCA(port));
	stats->rx_frame_broadcast = xlmac_reg64_read(XLMIB_GRxBCA(port));
	stats->rx_frame_control = xlmac_reg64_read(XLMIB_GRxCF(port));
	stats->rx_frame_pause = xlmac_reg64_read(XLMIB_GRxPF(port));
	stats->rx_frame_jabber = xlmac_reg64_read(XLMIB_GRxJBR(port));
	stats->rx_frame_fragment = xlmac_reg64_read(XLMIB_GRxFRG(port));
	stats->rx_frame_vlan = xlmac_reg64_read(XLMIB_GRxVLN(port));
	stats->rx_frame_dvlan = xlmac_reg64_read(XLMIB_GRxDVLN(port));
	stats->rx_frame_fcs_error = xlmac_reg64_read(XLMIB_GRxFCS(port));
	stats->rx_frame_unsupport = xlmac_reg64_read(XLMIB_GRxUO(port));
	stats->rx_frame_wrong_sa = xlmac_reg64_read(XLMIB_GRxWSA(port));
	stats->rx_frame_align_err = xlmac_reg64_read(XLMIB_GRxALN(port));
	stats->rx_frame_length_err = xlmac_reg64_read(XLMIB_GRxFLR(port));
	stats->rx_frame_oversize = xlmac_reg64_read(XLMIB_GRxOVR(port));
	stats->rx_frame_mtu_err = xlmac_reg64_read(XLMIB_GRxMTUE(port));
	stats->rx_frame_truncated_err = xlmac_reg64_read(XLMIB_GRxTRFU(port));
	stats->rx_frame_undersize = xlmac_reg64_read(XLMIB_GRxUND(port));
	stats->tx_frames = xlmac_reg64_read(XLMIB_GTxPkt(port));
	stats->tx_frame_good = xlmac_reg64_read(XLMIB_GTxPOK(port));
	stats->tx_bytes = xlmac_reg64_read(XLMIB_GTxBYT(port));
	stats->tx_frame_64 = xlmac_reg64_read(XLMIB_GTx64(port));
	stats->tx_frame_127 = xlmac_reg64_read(XLMIB_GTx127(port));
	stats->tx_frame_255 = xlmac_reg64_read(XLMIB_GTx255(port));
	stats->tx_frame_511 = xlmac_reg64_read(XLMIB_GTx511(port));
	stats->tx_frame_1023 = xlmac_reg64_read(XLMIB_GTx1023(port));
	stats->tx_frame_1518 = xlmac_reg64_read(XLMIB_GTx1518(port));
	stats->tx_frame_1522 = xlmac_reg64_read(XLMIB_GTx1522(port));
	stats->tx_frame_jumbo = xlmac_reg64_read(XLMIB_GTx2047(port)) +
							xlmac_reg64_read(XLMIB_GTx4095(port)) +
							xlmac_reg64_read(XLMIB_GTx9216(port)) +
							xlmac_reg64_read(XLMIB_GTx16383(port));
	stats->tx_frame_unicast = xlmac_reg64_read(XLMIB_GTxUCA(port));
	stats->tx_frame_multicast = xlmac_reg64_read(XLMIB_GTxMCA(port));
	stats->tx_frame_broadcast = xlmac_reg64_read(XLMIB_GTxBCA(port));
	stats->tx_frame_control = xlmac_reg64_read(XLMIB_GTxCF(port));
	stats->tx_frame_pause = xlmac_reg64_read(XLMIB_GTxPF(port));
	stats->tx_frame_jabber = xlmac_reg64_read(XLMIB_GTxJBR(port));
	stats->tx_frame_fragment = xlmac_reg64_read(XLMIB_GTxFRG(port));
	stats->tx_frame_vlan = xlmac_reg64_read(XLMIB_GTxVLN(port));
	stats->tx_frame_dvlan = xlmac_reg64_read(XLMIB_GTxDVLN(port));
	stats->tx_frame_fcs_error = xlmac_reg64_read(XLMIB_GTxFCS(port));
	stats->tx_frame_oversize = xlmac_reg64_read(XLMIB_GTxOVR(port));
	stats->tx_frame_error = xlmac_reg64_read(XLMIB_GTxErr(port));
	stats->tx_frame_fifo_underrun = xlmac_reg64_read(XLMIB_GTxUFL(port));
	stats->tx_frame_collision = xlmac_reg64_read(XLMIB_GTxNCL(port));

	return 0;
}


int pm4x10_xlport_mib_reset(int port)
{
	u32 val;

	/* MIB reset */
	val = xlport_reg32_read(XLPORT_MIB_RESET);
	val |= (1 << port) << XLPORT_MIB_RESET__CLR_CNT_R;
	xlport_reg32_write(XLPORT_MIB_RESET, val);

	val &= ~((1 << port) << XLPORT_MIB_RESET__CLR_CNT_R);
	xlport_reg32_write(XLPORT_MIB_RESET, val);

	return 0;
}

int pm4x10_pm_xlport_port_config(int port, int enable)
{
	u32 val;

	if (enable) {
		/* Soft reset */
		val = xlport_reg32_read(XLPORT_SOFT_RESET);
		XLPORT_PORT_FIELD_SET(XLPORT_SOFT_RESET, port, val);
		xlport_reg32_write(XLPORT_SOFT_RESET, val);

		XLPORT_PORT_FIELD_CLEAR(XLPORT_SOFT_RESET, port, val);
		xlport_reg32_write(XLPORT_SOFT_RESET, val);

		/* Port enable */
		val = xlport_reg32_read(XLPORT_ENABLE_REG);
		XLPORT_PORT_FIELD_SET(XLPORT_ENABLE_REG, port, val);
		xlport_reg32_write(XLPORT_ENABLE_REG, val);

		/* Init MAC */
		__xlmac_init(port);

#if 0 //FIXME
		/* LSS */
		val = xlport_reg32_read(XLPORT_FAULT_LINK_STATUS(port));
		val |= 1 << XLPORT_FAULT_LINK_STATUS__REMOTE_FAULT;
		val |= 1 << XLPORT_FAULT_LINK_STATUS__LOCAL_FAULT;
		xlport_reg32_write(XLPORT_FAULT_LINK_STATUS(port), val);
#endif /* 0 */

		/* MIB reset */
		pm4x10_xlport_mib_reset(port);

		/* Enable the port */
		pm4x10_xlport_mac_enable(port);
	} else {
		/* Port disable */
		val = xlport_reg32_read(XLPORT_ENABLE_REG);
		XLPORT_PORT_FIELD_CLEAR(XLPORT_ENABLE_REG, port, val);
		xlport_reg32_write(XLPORT_ENABLE_REG, val);

		/* Soft reset */
		val = xlport_reg32_read(XLPORT_SOFT_RESET);
		XLPORT_PORT_FIELD_CLEAR(XLPORT_SOFT_RESET, port, val);
		xlport_reg32_write(XLPORT_SOFT_RESET, val);

		/* Disable the port */
		pm4x10_xlport_mac_disable(port);
	}

	return 0;
}

static int pm4x10_pm_disable(void)
{
	u32 val;

	/* Put MAC in reset */
	val = xlport_reg32_read(XLPORT_MAC_CONTROL);
	val |= 1 << XLPORT_MAC_CONTROL__XMAC0_RESET;
	xlport_reg32_write(XLPORT_MAC_CONTROL, val);

	/* Put Serdes in reset*/
	__tsc_reset(1);

	return 0;
}

static inline u32
pm_phy_configure(int port)
{
	/* [0xc113] reset and enable tx lane */
	pm_phy_sbus_write(port, TX_X4_CONTROL0_MISC, 0x0003, 0x0003, TX_X4_CONTROL0_MISC_enable_tx_lane);

	/* [0xc137] low active per lane */
	pm_phy_sbus_write(port, RX_X4_CONTROL0_PMA_CONTROL_0, 0x0001, 0x0001, RX_X4_CONTROL0_PMA_CONTROL_0_rstb_lane);

	/* [0xc181] set base abilities */
	pm_phy_sbus_write(port, AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES, 0x0056,
				0x00ff, AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_sgmii_speed);

	/* [0xc183] enable CL72 */
	pm_phy_sbus_write(port, AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1, 0x0001,
				0x0001, AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_CL72);

	/* [0xc185] Support IEEE 802.3 */
	pm_phy_sbus_write(port, AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1, 0x0001,
				0x0001, AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1_base_selector);

	/* [0xd081] Active Low Lane Soft Reset for datapath */
	pm_phy_sbus_write(port, RXTXCOM_CKRST_CTRL_LANE_CLK_RESET_N_POWERDOWN_CONTROL, 0x0001,
				0x0001, RXTXCOM_CKRST_CTRL_LANE_CLK_RESET_N_POWERDOWN_CONTROL_ln_dp_s_rstb);

	/* [0xd0f4] Active Low Core Level Datapath Soft Reset */
	pm_phy_sbus_write(port, DIGCOM_TOP_USER_CONTROL_0, 0x0001, 0x0001, DIGCOM_TOP_USER_CONTROL_0_core_dp_s_rstb);
}


static inline u32
pm_ucode_download(u8 *ucode_image, u16 ucode_len)
{
	u32 get_val;
	u8 i, wrdata_lsb;
	u16 wrdata_lsw, ucode_len_padded, count = 0;

	/* Check array pointer */
	if (ucode_image == (u8 *)NULL) {
		debug("uCode Image is empty !!\n");
		return -1;
	}

	/* Check ucode size */
	if (ucode_len > (32768)) {  /* 16 x 2048 */
		debug("Can't fit all of the firmware into the device load table(max = 16 x 2048 bytes) \n");
		return -1;
	}

	/* [0xd200] Write to Clock control register 0 to enable micro core clock (m0) */
	pm_phy_sbus_write(0, MICRO_A_CLOCK_CONTROL0, 0x0001, 0x0001, MICRO_A_CLOCK_CONTROL0_micro_master_clk_en);

	/* [0xd201] Write to Reset control registers 0 to make micro_master_rstb = 1 */
	pm_phy_sbus_write(0, MICRO_A_RESET_CONTROL0, 0x0001, 0x0001, MICRO_A_RESET_CONTROL0_micro_master_rstb);

	/* [0xd201] Write to Reset control registers 0 to make micro_master_rstb = 0 */
	pm_phy_sbus_write(0, MICRO_A_RESET_CONTROL0, 0x0000, 0x0001, MICRO_A_RESET_CONTROL0_micro_master_rstb);

	/* [0xd201] Write to Reset control registers 0 to make micro_master_rstb = 1 */
	pm_phy_sbus_write(0, MICRO_A_RESET_CONTROL0, 0x0001, 0x0001, MICRO_A_RESET_CONTROL0_micro_master_rstb);

	/* [0xd202] Write to rmi to ahb control register 0 to initialize code RAMs */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0001, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_init);

	/* [0xd203] Read from ahb status register 0 to make sure all are done */
	for (i=0; i<100; ++i) {
		pm_phy_sbus_read(0, MICRO_A_AHB_STATUS0, &get_val);
		/* code/data RAM initialization process is complete */
		if (get_val == 0x1)	break;
		udelay(2500);
	}
	if (i == 100) {
		debug("code/data RAM initialization process is timeout !!\n");
		return -1;
	}

	/* [0xd202] Write to rmi to ahb control register 0 to clear intialize code/data RAM command */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0000, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_init);

	ucode_len_padded = ((ucode_len + 3) & 0xFFFC);		/* Aligning ucode size to 4-byte boundary */

	/* [0xd202] Write to rmi to ahb control register 0 to make Automatic increment write address enable */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0001, 0x0001, MICRO_A_AHB_CONTROL0_micro_autoinc_wraddr_en);

	/* [0xd202] Write to rmi to ahb control register 0 to select write data size = 16 bits */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0001, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_wrdatasize);

	/* [0xd205] Write to rmi to ahb write address MSW (bits 31:16) register with 0 */
	pm_phy_sbus_write(0, MICRO_A_AHB_WRADDR_MSW, 0x0000, 0xffff, MICRO_A_AHB_WRADDR_MSW_micro_ra_wraddr_msw);

	/* [0xd206] Write to rmi to ahb write data LSW (bits 15:0) register with 0 (starting address = 0x0) */
	pm_phy_sbus_write(0, MICRO_A_AHB_WRADDR_LSW, 0x0000, 0xffff, MICRO_A_AHB_WRADDR_LSW_micro_ra_wraddr_lsw);

	do {    /* ucode_image loaded 16bits at a time */
		/* wrdata_lsb read from ucode_image; zero padded to 4byte boundary */
		wrdata_lsb = (count < ucode_len) ? ucode_image[count] : 0x0;
		count++;
		/* wrdata_msb read from ucode_image; zero padded to 4byte boundary */
		wrdata_lsw = (count < ucode_len) ? ucode_image[count] : 0x0;
		count++;
		/* 16bit wrdata_lsw formed from 8bit msb and lsb values read from ucode_image */
		wrdata_lsw = ((wrdata_lsw << 8) | wrdata_lsb);

		/* [0xd206] Write to rmi to ahb write data LSW (bits 15:0) register with data */
		pm_phy_sbus_write(0, MICRO_A_AHB_WRDATA_LSW, wrdata_lsw, 0xffff, MICRO_A_AHB_WRDATA_LSW_micro_ra_wrdata_lsw);
	} while (count < ucode_len_padded);                 /* Loop repeated till entire image loaded (upto the 4byte boundary) */

	/* [0xd202] Write to rmi to ahb control register 0 to Select 32bit transfers as default */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0002, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_wrdatasize);

	/* [0xd200] Write to Clock control register 0 to enable micro core clock enable (m0) */
	pm_phy_sbus_write(0, MICRO_A_CLOCK_CONTROL0, 0x0001, 0x0001, MICRO_A_CLOCK_CONTROL0_micro_core_clk_en);

	return 0;
}

static void pm4x10_tsc_config_1(void)
{
#ifdef CONFIG_IPROC_EMULATION
	printf("skip %s... in emulation\n", __func__);
#else
	u32 val, id2, id3;
	int i;

  /* [0x0002] Read from IEEE phyID2 register */
	pm_phy_sbus_read(0, CL22_B0_phyid2, &id2);
	if (id2 != 0x600d) {
		printk("PHY ID2 should be 0x600d but get 0x%x....\n", id2);
		return;
	}

	/* [0x0003] Read from IEEE phyID3 register */
	pm_phy_sbus_read(0, CL22_B0_phyid3, &id3);
	if (id3 != 0x8770) {
		printk("PHY ID3 should be 0x8770 but get 0x%x....\n", id3);
		return;
	}
	printk("Get PHY (merlin16) ID2:%.4x, ID3:%.4x\n", id2, id3);

	/* [0x900e] Read from Serdes ID register */
	pm_phy_sbus_read(0, Main0_serdesID, &val);
	if ((val & 0x0000003f) != 0x00000012) {
		printk("Serdes ID should be TSCE(0x12) but get 0x%x....\n", (val & 0x0000003f));
		return;
	}

	/* [0x9010] Global PMD reset controls */
	pm_phy_sbus_write(0, PMD_X1_CONTROL, 0x0000, 0xffff, PMD_X1_CONTROL_core_dp_h_rstb);
	udelay(10);
	pm_phy_sbus_write(0, PMD_X1_CONTROL, 0x0003, 0xffff, PMD_X1_CONTROL_core_dp_h_rstb);
	udelay(10);

	/* [0xc010] Write to PMD lane reset control registers */
	for (i=0; i<4; ++i) {
	  pm_phy_sbus_write(i, PMD_X4_CONTROL, 0x0000, 0xc003, PMD_X4_CONTROL_ln_rx_dp_h_rstb);
	  udelay(10);
	  pm_phy_sbus_write(i, PMD_X4_CONTROL, 0xc003, 0xc003, PMD_X4_CONTROL_ln_rx_dp_h_rstb);
	}

	/* [0xd0f4] Read from DIGCOM_TOP_USER_CONTROL_0 to make sure uc_active */
	pm_phy_sbus_read(0, DIGCOM_TOP_USER_CONTROL_0, &val);
	if (val & (0x0001 << DIGCOM_TOP_USER_CONTROL_0_uc_active)) {
		debug("Merlin16 is still waiting for uC handshakes to wake up from datapath reset....\n");
		return;
	}

	/* [0x9000] Write to MAIN_CONTROL register to set refclk_sel = 156.25 MHz */
	pm_phy_sbus_write(0, MAIN0_SETUP, 0x0003, 0x0007, MAIN0_SETUP_refclk_sel);

	/* [0xd0f4] Write to TOP_USER_CONTROL_0 register to set heartbeat_count_1us = 0x271 */
	pm_phy_sbus_write(0, DIGCOM_TOP_USER_CONTROL_0, 0x0271, 0x03ff, DIGCOM_TOP_USER_CONTROL_0_heartbeat_count_1us);

	/* [0xd0f4] Read from DIGCOM_TOP_USER_CONTROL_0 to make sure uc_active & High PLL is active */
	pm_phy_sbus_read(0, DIGCOM_TOP_USER_CONTROL_0, &val);
	if (val & (0x0001 << DIGCOM_TOP_USER_CONTROL_0_uc_active)) {
		debug("Merlin16 is still waiting for uC handshakes to wake up from datapath reset....\n");
		return;
	}
	if (val & (0x0001 << DIGCOM_TOP_USER_CONTROL_0_afe_s_pll_pwrdn)) {
		debug("High PLL in Merlin16 is still down....\n");
		return;
	}

	/* [0xd0fa] Read from REVID1 to make sure num_lanes = 4 */
	pm_phy_sbus_read(0, DIGCOM_REVID1, &val);
	if ((uint16_t)(val >> DIGCOM_REVID1_revid_multiplicity) != 0x04) {
		debug("Lane number should be 4 but get %d....\n", i);
		return;
	}
#endif
}

static void pm4x10_tsc_config_2(void)
{
#ifdef CONFIG_IPROC_EMULATION
	printf("skip %s... in emulation\n", __func__);
#else
	u32 val;
	int i;

	/* [0xd200] Write to Clock control register 0 to enable clock to microcontroller subsystem */
	pm_phy_sbus_write(0, MICRO_A_CLOCK_CONTROL0, 0x0001, 0x0001, MICRO_A_CLOCK_CONTROL0_micro_master_clk_en);

	/* [0xd201] Write to Clock control register 0 to enable micro master reset (m0) */
	pm_phy_sbus_write(0, MICRO_A_RESET_CONTROL0, 0x0001, 0x0001, MICRO_A_RESET_CONTROL0_micro_master_rstb);

	/* [0xd202] Write to ahb control register 0 to init data RAM with all zeros */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0002, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_init);

	/* [0xd203] Read from ahb status registers 0 for micro_ra_initdone = 1 to indicate initialization done */
	for (i = 0; i < 100; ++i) {
		pm_phy_sbus_read(0, MICRO_A_AHB_STATUS0, &val);
		if (val == 0x1)		/* code/data RAM initialization process is complete */
			break;
		udelay(2500);
	}
	if (i == 100) {
		debug("code/data RAM initialization process is timeout !!\n");
		return;
	}

	/* [0xd202] Write to ahb control registers 0 to clear command for data RAM initialization */
	pm_phy_sbus_write(0, MICRO_A_AHB_CONTROL0, 0x0000, 0x0003, MICRO_A_AHB_CONTROL0_micro_ra_init);

	/* [0xd200] Write to Clock control registers 0 to enable micro core clock enable (m0) */
	pm_phy_sbus_write(0, MICRO_A_CLOCK_CONTROL0, 0x0001, 0x0001, MICRO_A_CLOCK_CONTROL0_micro_core_clk_en);

	/* [0xd201] Write to Reset control registers 0 to enable micro core reset (m0) */
	pm_phy_sbus_write(0, MICRO_A_RESET_CONTROL0, 0x0001, 0x0001, MICRO_A_RESET_CONTROL0_micro_core_rstb);

	/* [0xd0f4] Read from DIGCOM_TOP_USER_CONTROL_0 to make sure uc_active */
	for (i = 0; i < 10000; ++i) {
		pm_phy_sbus_read(0, DIGCOM_TOP_USER_CONTROL_0, &val);
		if ((val & (0x0001 << DIGCOM_TOP_USER_CONTROL_0_uc_active)) != 0) break;
		if (i > 10) udelay(1);
	}
	if (i == 10000) {
		debug("Merlin16 is still waiting for uC handshakes to wake up from datapath reset....\n");
		return;
	}
#endif
}

static int
pm_ucode_CRC_check(u16 ucode_len, u16 ucode_crc)
{
	int i;
	u32 val;

	for (i = 0; i < 100; ++i) {
		/* [0xd00d] Read from DSC uC Control register to check READY status */
		pm_phy_sbus_read(0, DSC_A_DSC_UC_CTRL, &val);
		if ((val & (1 << DSC_A_DSC_UC_CTRL_uc_dsc_ready_for_cmd)) != 0) {
			if ((val & (1 << DSC_A_DSC_UC_CTRL_uc_dsc_error_found)) != 0) {
				printk("uC dsc error found (0x%x)!!\n", val);
				return(-1);
			} else {
				break;  /* uC is ready for command */
			}
		}
		if (i > 10)
			udelay(10);
	}

	/* [0xd00e] Write to DSC uC Control register with ucode_len */
	pm_phy_sbus_write(0, DSC_A_DSC_SCRATCH, ucode_len, 0xffff, DSC_A_DSC_SCRATCH_uc_dsc_scratch);

	/* [0xd00d] Write to DSC uC Control register with CMD_CALC_CRC(0x14) command */
	pm_phy_sbus_write(0, DSC_A_DSC_UC_CTRL, CMD_CALC_CRC, 0xffff, DSC_A_DSC_UC_CTRL_uc_dsc_gp_uc_req);

	for (i = 0; i < 100; ++i) {
		/* [0xd00d] Read from DSC uC Control register to check READY status */
		pm_phy_sbus_read(0, DSC_A_DSC_UC_CTRL, &val);
		if ((val & (1 << DSC_A_DSC_UC_CTRL_uc_dsc_ready_for_cmd)) != 0) {
			if ((val & (1 << DSC_A_DSC_UC_CTRL_uc_dsc_error_found)) != 0) {
				printk("uC dsc error found (0x%x)!!\n", val);
				return(-1);
			} else {
				break;  /* uC is ready for command */
			}
		}
		if (i > 10)
			udelay(2500);
	}

	/* [0xd00e] Read from DSC uC Control register for the calculated CRC */
	pm_phy_sbus_read(0, DSC_A_DSC_SCRATCH, &val);
	if ((uint16_t)val != ucode_crc) {
		printk("ucode CRC check (expected = 0x%x, calculated = 0x%x) FAIL !!\n", ucode_crc, val);
		return(-1);
	} else {
		//printk("ucode CRC check (0x%x) : PASS !!\n", val);
		return(0);
	}
}

static void pm4x10_xlport_config(int port)
{
	u64 val64;

	/* Ethernet mode, 1G */
	val64 = xlmac_reg64_read(XLMAC_MODE(port));
	val64 &= ~(0x70);
	val64 |= SPEED_MODE_LINK_1G << XLMAC_MODE__SPEED_MODE_R;
	xlmac_reg64_write(XLMAC_MODE(port), val64);
	debug("(%s) XLMAC_MODE = 0x%llx\n",  __func__, xlmac_reg64_read(XLMAC_MODE(port)));

	val64 = xlmac_reg64_read(XLMAC_RX_CTRL(port));
	val64 &= ~(1 << XLMAC_RX_CTRL__STRIP_CRC);
	xlmac_reg64_write(XLMAC_RX_CTRL(port), val64);
	debug("(%s) XLMAC_RX_CTRL = 0x%llx\n",  __func__, xlmac_reg64_read(XLMAC_RX_CTRL(port)));

	val64 = xlmac_reg64_read(XLMAC_TX_CTRL(port));
	val64 &= ~(0x3 << XLMAC_TX_CTRL__CRC_MODE_R);
	val64 |= (CRC_MODE_REPLACE << XLMAC_TX_CTRL__CRC_MODE_R);
	val64 |= (1 << XLMAC_TX_CTRL__PAD_EN);
	xlmac_reg64_write(XLMAC_TX_CTRL(port), val64);
	debug("(%s) XLMAC_TX_CTRL = 0x%llx\n",  __func__, xlmac_reg64_read(XLMAC_TX_CTRL(port)));

	__xlmac_enable_set(port, 1);
}

static int pm4x10_pm_enable(void)
{
	u32 val, mask;
	int i;

	/* Power Save */
	xlport_reg32_write(XLPORT_POWER_SAVE, 0x0);

	/* Enable all xlports */
	val = xlport_reg32_read(XLPORT_ENABLE_REG);
	val |= 0x0000000f;
	xlport_reg32_write(XLPORT_ENABLE_REG, val);

#if 0
	/* Port configuration */
	val = xlport_reg32_read(XLPORT_MODE_REG);
	mask = (1 << XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_WIDTH) - 1;
	val &= ~(mask << XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_R);
	val |= (XPORT0_CORE_PORT_MODE_QUAD << XLPORT_MODE_REG__XPORT0_CORE_PORT_MODE_R);
	mask = (1 << XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_WIDTH) - 1;
	val &= ~(mask << XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_R);
	val |= (XPORT0_CORE_PORT_MODE_QUAD << XLPORT_MODE_REG__XPORT0_PHY_PORT_MODE_R);
	xlport_reg32_write(XLPORT_MODE_REG, val);
#endif

	/* Bring MAC OOR(out of reset) */
	val = xlport_reg32_read(XLPORT_MAC_CONTROL);
	val |= (1 << XLPORT_MAC_CONTROL__XMAC0_RESET);
	xlport_reg32_write(XLPORT_MAC_CONTROL, val);
	udelay(10);
	val &= ~(1 << XLPORT_MAC_CONTROL__XMAC0_RESET);
	xlport_reg32_write(XLPORT_MAC_CONTROL, val);

	for (i=0; i<4; ++i) {
		/* Resetting MIB counter */
		pm4x10_xlport_mib_reset(i);
		pm4x10_xlport_config(i);
	}

	return 0;
}

int pm4x10_pm_init(struct iproc_pm_ops *pm_ops, u8 land_idx)
{
	u32 val;
	volatile void __iomem *addr;

	if (!pm4x10_enabled) { /* check initialize for first time */
		/* Bring APM out of reset (to access IPROC_WRAP_MISC_CONTROL) */
		addr = (volatile void __iomem *)(get_iproc_wrap_ctrl_base() + 0x18);
		val = ioread32(addr);
		val &= ~(1 << IPROC_WRAP_MISC_CONTROL_IPROC_PM_RST_L);
		iowrite32(val, addr);
		msleep(1);

		val = ioread32(addr);
		val |= (1 << IPROC_WRAP_MISC_CONTROL_IPROC_PM_RST_L);
		iowrite32(val, addr);
		msleep(1);

		/* reset/clear whole APM */
		__tsc_reset(1);
		msleep(10);
		__tsc_reset(0);

		/* Enable PM4x10 */
		pm4x10_pm_enable();

		/* configure TSC registers before download ucode(part-1) */
		pm4x10_tsc_config_1();

		/* ucode download */
		pm_ucode_download(merlin16_ucode, merlin16_ucode_len);

		/* configure TSC registers before download ucode(part-2) */
		pm4x10_tsc_config_2();

		/* ucode CRC check */
		if (pm_ucode_CRC_check(merlin16_ucode_len, merlin16_ucode_crc) != 0) {
			printk("ucode CRC check error....\n");
			return -1;
    }

		pm4x10_enabled++;
	}

	/* Configure interface and abilities (including TSC and merlin16) */
	pm_phy_configure(land_idx);

	return 0;
}

int pm4x10_pm_deinit(struct iproc_pm_ops *pm_ops)
{
#if 0
	kfree(pm_ops);
	pm_ops = NULL;
#endif

	pm4x10_enabled--;
	if (!pm4x10_enabled) {
		pm4x10_pm_disable();
	}

	return 0;
}
