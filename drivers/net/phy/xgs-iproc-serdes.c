/*
 * Copyright (C) 2016 Broadcom Corporation
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

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include <linux/brcmphy.h>
#include <linux/phy/xgs_iproc_serdes.h>
#include "xgs_iproc_serdes_def.h"

/* the SERDES PHY ID for HX4/KT2/SB2/GH2/WH2 is the same */
#define PHY_ID_XGS_AMAC_SERDES	0x0143bff0

#define SERDES_ID_HX4_AMAC	0x828f4e00
#define SERDES_ID_KT2_AMAC	0x42814fc0
/* The amac serdes id (id0,id1) of SB2/GH2/WH2 is the same */
#define SERDES_ID_SB2_AMAC	0x02cf1a00
#define SERDES_ID_GH2_AMAC	0x02cf1a00	/* apply to WH2 */
/* When ID is the same, use id2 for further identification */
#define SERDES_ID2_HX4_AMAC	0x000f
#define SERDES_ID2_KT2_AMAC	0x03ff
#define SERDES_ID2_SB2_AMAC	0x8007
#define SERDES_ID2_GH2_AMAC	0x800f	/* apply to WH2 */

#define PHY_REG_BLK_ADDR	0x001f
#define PHY_REG_AER_BLK		0xffd0
#define PHY_REG_AER_OFFSET	0x001e
#define PHY_REG_BLK_ADDR_MASK	0x7ff0
#define PHY_REG_ADDR_MASK	0xf
#define PHY_REG_ADDR_32_MASK	0x8000

#define PHY_AER_REG_ADDR_AER(_addr)	(((_addr) >> XGXS16G_SERDES_LANE_SHIFT) \
						& 0xFFFF)
#define PHY_REG_ADDR_BLK(_addr) 	((_addr) & PHY_REG_BLK_ADDR_MASK)
#define PHY_REG_ADDR_REGAD(_addr)  (((_addr & PHY_REG_ADDR_32_MASK) >> 11) \
						| (_addr & PHY_REG_ADDR_MASK))

//#define BCMDBG
//#define BCMDBG_ERR
#ifdef BCMDBG
#define SERDES_ERROR(args) pr_err args
#define SERDES_TRACE(args) pr_info args
#elif defined(BCMDBG_ERR)
#define SERDES_ERROR(args) pr_err args
#define SERDES_TRACE(args)
#else
#define SERDES_ERROR(args)
#define SERDES_TRACE(args)
#endif /* BCMDBG */

/* For pcie/usb serdes and phy write */
void xgs_phy_wr_reg(struct phy_device *phydev, u32 regnum, u16 data)
{
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;

	phy_reg_blk  = regnum & PHY_REG_BLK_ADDR_MASK;
	phy_reg_addr = regnum & PHY_REG_ADDR_MASK;
	phy_reg_addr |= (regnum & PHY_REG_ADDR_32_MASK) ? 0x10 : 0x0;

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	phy_write(phydev, phy_reg_addr, data);
}

/* For pcie/usb serdes and phy read */
u16 xgs_phy_rd_reg(struct phy_device *phydev, u32 regnum)
{
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;
	int data;

	phy_reg_blk  = regnum & PHY_REG_BLK_ADDR_MASK;
	phy_reg_addr = regnum & PHY_REG_ADDR_MASK;
	phy_reg_addr |= (regnum & PHY_REG_ADDR_32_MASK) ? 0x10 : 0x0;

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	data = phy_read(phydev, phy_reg_addr);

	return (u16)data;
}

/* for SB2 USB PHY write */
void xgs_sb2_usb_phy_wr_reg(struct phy_device *phydev, u32 regnum, u16 data)
{
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;

	phy_reg_blk  = regnum & 0xfff0;
	phy_reg_addr = regnum & 0xf;

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	phy_write(phydev, phy_reg_addr, data);
}

/* for SB2 USB PHY read */
u16 xgs_sb2_usb_phy_rd_reg(struct phy_device *phydev, u32 regnum)
{
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;
	int data;

	phy_reg_blk  = regnum & 0xfff0;
	phy_reg_addr = regnum & 0xf;

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	data = phy_read(phydev, phy_reg_addr);

	return (u16)data;
}

/* SB2/GH2/WH2 amac serdes supports AER */
static u16 xgs_serdes_rd_reg(struct phy_device *phydev, u32 regnum)
{
	int data;
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;
	u32 phy_reg_aer=0;

	phy_reg_aer = PHY_AER_REG_ADDR_AER(regnum);
	phy_reg_blk  = PHY_REG_ADDR_BLK(regnum);
	phy_reg_addr = PHY_REG_ADDR_REGAD(regnum);

	if (phy_reg_aer) {
		phy_write(phydev, PHY_REG_BLK_ADDR, PHY_REG_AER_BLK);
		phy_write(phydev, PHY_REG_AER_OFFSET, phy_reg_aer);
	}

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	data = phy_read(phydev, phy_reg_addr);

	if (phy_reg_aer) {
		phy_write(phydev, PHY_REG_BLK_ADDR, PHY_REG_AER_BLK);
		phy_write(phydev, PHY_REG_AER_OFFSET, 0x0);
	}

	return (u16)data;
}

static void xgs_serdes_wr_reg(struct phy_device *phydev, u32 regnum, u16 data)
{
	u16 phy_reg_blk=0;
	u32 phy_reg_addr=0;
	u32 phy_reg_aer=0;

	phy_reg_aer = PHY_AER_REG_ADDR_AER(regnum);
	phy_reg_blk  = PHY_REG_ADDR_BLK(regnum);
	phy_reg_addr = PHY_REG_ADDR_REGAD(regnum);

	if (phy_reg_aer) {
		phy_write(phydev, PHY_REG_BLK_ADDR, PHY_REG_AER_BLK);
		phy_write(phydev, PHY_REG_AER_OFFSET, phy_reg_aer);
	}

	phy_write(phydev, PHY_REG_BLK_ADDR, phy_reg_blk);
	phy_write(phydev, phy_reg_addr, data);

	if (phy_reg_aer) {
		phy_write(phydev, PHY_REG_BLK_ADDR, PHY_REG_AER_BLK);
		phy_write(phydev, PHY_REG_AER_OFFSET, 0x0);
	}
}

static u32 serdes_get_id(struct phy_device *phy_dev)
{
	u16 serdes_id0, serdes_id1;
	u32 serdes_id;
	struct phy_device *phydev = phy_dev;

	serdes_id0 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID0r);
	serdes_id1 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID1r);
	serdes_id = (serdes_id0 << 16) | serdes_id1;

	return serdes_id;
}

void xgs_serdes_set_lane(struct phy_device *phy_dev, u32 lane)
{
	xgs_serdes_info_t *serdes_info;

	serdes_info = devm_kzalloc(&phy_dev->mdio.dev, sizeof(*serdes_info),
					GFP_KERNEL);
	if (!serdes_info) {
		dev_err(&phy_dev->mdio.dev, "Fail to allocate xgs_serdes_info\n");
		return;
	}

	serdes_info->lane = lane;
	phy_dev->priv = serdes_info;
}

static inline u32 xgs_serdes_get_lane(struct phy_device *phy_dev)
{
	xgs_serdes_info_t *lane_info = (xgs_serdes_info_t *) phy_dev->priv;

	return lane_info->lane;
}

bool xgs_serdes_hx4_amac(struct phy_device *phy_dev)
{
	return (serdes_get_id(phy_dev) == SERDES_ID_HX4_AMAC);
}

bool xgs_serdes_kt2_amac(struct phy_device *phy_dev)
{
	return (serdes_get_id(phy_dev) == SERDES_ID_KT2_AMAC);
}

/* Needed for HX4/KT2/GH2/WH2, WH2 has the same ID as GH2 */
static void xgs_serdes_reset_core(struct phy_device *phy_dev)
{
	u16 data16;
	u32 serdes_id;
	u16 serdes_id2;
	static u32 serdes_core_reset = 0;
	struct phy_device *phydev = phy_dev;

	if (phydev->phy_id != PHY_ID_XGS_AMAC_SERDES)
		return;

	/* Only reset once */
	if (serdes_core_reset)
		return;

	serdes_id = serdes_get_id(phydev);
	serdes_id2 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID2r);
	SERDES_TRACE(("-----SERDESID2: 0x%x\n", serdes_id2));

	if (!((serdes_id == SERDES_ID_HX4_AMAC) ||
		(serdes_id == SERDES_ID_KT2_AMAC) ||
		(serdes_id == SERDES_ID_GH2_AMAC)))
		return;

	/* GH2/WH2 specific code */
	if (serdes_id == SERDES_ID_GH2_AMAC) {
		if (serdes_id2 != SERDES_ID2_GH2_AMAC)
			return;

		/* Disable pll start sequencer */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr);
		data16 &= ~XGXSBLK0_CONTROL_PLL_SEQUENCER_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);
		serdes_core_reset = 1;
		return;
	}

	/* The following is HX4/KT2 related */
	/* unlock lane */
	data16 = xgs_serdes_rd_reg(phydev, XGXS16G_WC40_DIGITAL4_MISC3r);
	data16 &= ~(DIGITAL4_MISC3_LANEDISABLE_MASK);
	xgs_serdes_wr_reg(phydev, XGXS16G_WC40_DIGITAL4_MISC3r, data16);

	/* Reset the core */
	/* Stop PLL Sequencer and configure the core into correct mode */
	data16 = (XGXSBLK0_XGXSCONTROL_MODE_10G_IndLane <<
			XGXSBLK0_XGXSCONTROL_MODE_10G_SHIFT) |
				XGXSBLK0_XGXSCONTROL_HSTL_MASK |
				XGXSBLK0_XGXSCONTROL_CDET_EN_MASK |
				XGXSBLK0_XGXSCONTROL_EDEN_MASK |
				XGXSBLK0_XGXSCONTROL_AFRST_EN_MASK |
				XGXSBLK0_XGXSCONTROL_TXCKO_DIV_MASK;
	xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);

	/*
	 * Disable IEEE block select auto-detect.
	 * The driver will select desired block as necessary.
	 * By default, the driver keeps the XAUI block in IEEE address space.
	 */
	data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_MISCCONTROL1r);
	data16 &= ~(XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_AUTODET_MASK |
				XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_VAL_MASK);
	if (!XGXS16G_2p5G_ID(serdes_id2) && (serdes_id == SERDES_ID_HX4_AMAC))
		data16 |= XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_VAL_MASK;
	xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_MISCCONTROL1r, data16);

	/* disable in-band MDIO. PHY-443 */
	data16 = xgs_serdes_rd_reg(phydev, 0x8111);
	/* rx_inBandMdio_rst */
	data16 |= 1 << 3;
	xgs_serdes_wr_reg(phydev, 0x8111, data16);

	serdes_core_reset = 1;
}

static void xgs_serdes_reset(struct phy_device *phy_dev)
{
	u16 ctrl;
	struct phy_device *phydev = phy_dev;
	u32 serdes_id;
	u16 serdes_id2;
	u32 aer = 0;
	u32 aer_blk_reg = 0;

	if (phydev->phy_id != PHY_ID_XGS_AMAC_SERDES)
		return;

	serdes_id = serdes_get_id(phydev);
	serdes_id2 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID2r);

	/* AER required for GH2/WH2 serdes */
	if ((serdes_id == SERDES_ID_GH2_AMAC) &&
			(serdes_id2 == SERDES_ID2_GH2_AMAC))
		aer = xgs_serdes_get_lane(phy_dev) << XGXS16G_SERDES_LANE_SHIFT;

	/* de-assert reset */
	aer_blk_reg = aer | XGXS16G_IEEE0BLK_IEEECONTROL0r;
	ctrl = xgs_serdes_rd_reg(phydev, aer_blk_reg);
	ctrl |= IEEE0BLK_IEEECONTROL0_RST_HW_MASK;
	xgs_serdes_wr_reg(phydev, aer_blk_reg, ctrl);
	udelay(100);

	/* check if out of reset */
	if (xgs_serdes_rd_reg(phydev, aer_blk_reg) &
			IEEE0BLK_IEEECONTROL0_RST_HW_MASK)
		SERDES_ERROR(("amac serdes reset not completed.\n"));
}

static void xgs_serdes_init(struct phy_device *phy_dev)
{
	u16 data16;
	u32 serdes_id;
	u16 serdes_id2;
	u32 __maybe_unused aer_blk_reg, aer;
	struct phy_device *phydev = phy_dev;

#ifdef BCMDBG
	u16 tmp0, tmp1;
	tmp0 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID0r);
	tmp1 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID1r);
	SERDES_TRACE(("-----SERDESID0: 0x%x; SERDESID1: 0x%x\n", tmp0, tmp1));

	tmp0 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID2r);
	tmp1 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID3r);
	SERDES_TRACE(("-----SERDESID2: 0x%x;SERDESID3: 0x%x\n", tmp0, tmp1));
#endif /* BCMDBG */

	SERDES_TRACE(("%s: phyaddr %d\n",__FUNCTION__, phydev->mdio.addr));

	if (phydev->phy_id != PHY_ID_XGS_AMAC_SERDES)
		return;

	serdes_id = serdes_get_id(phydev);
	serdes_id2 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID2r);

	if ((serdes_id == SERDES_ID_SB2_AMAC) &&
		(serdes_id2 == SERDES_ID2_SB2_AMAC)) {
		/* Auto Negotiation 10M/100M/1G ¡V SGMII Slave */
		/* Disable pll start sequencer */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr);
		data16 &= ~XGXSBLK0_CONTROL_PLL_SEQUENCER_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);

		/* Set SGMII slave mode */
		xgs_serdes_wr_reg(phydev, XGXS16G_SERDESDIGITAL_CONTROL1000X1r,
				SERDESDIGITAL_CONTROL1000X1_SLAVE_MODE);

		/* Enable AN 10M/100M/1G */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_IEEE0BLK_IEEECONTROL0r);
		data16 |= IEEE0BLK_IEEECONTROL0_ENABLE_AN_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_IEEE0BLK_IEEECONTROL0r, data16);

		/* Enable pll start sequencer */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr);
		data16 |= XGXSBLK0_CONTROL_PLL_SEQUENCER_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);
	} else if ((serdes_id == SERDES_ID_GH2_AMAC) &&
			(serdes_id2 == SERDES_ID2_GH2_AMAC)) {
		aer = xgs_serdes_get_lane(phydev) << XGXS16G_SERDES_LANE_SHIFT;

		/* Disable IEEE block select auto-detect */
		data16 = 0;
		aer_blk_reg = (aer | XGXS16G_XGXSBLK0_MISCCONTROL1r);
		xgs_serdes_wr_reg(phydev, aer_blk_reg, data16);

		/* Disable lmtcal (broadcast to all lanes) */
		data16 = 0x83f8;
		aer_blk_reg = (aer | XGXS16G_RX3_CONTROL2r);
		xgs_serdes_wr_reg(phydev, aer_blk_reg, data16);

		/* Set SGMII slave mode */
		aer_blk_reg = (aer | XGXS16G_SERDESDIGITAL_CONTROL1000X1r);
		xgs_serdes_wr_reg(phydev, aer_blk_reg,
				SERDESDIGITAL_CONTROL1000X1_SLAVE_MODE);

		/* Enable AN 10M/100M/1G */
		aer_blk_reg = (aer | XGXS16G_IEEE0BLK_IEEECONTROL0r);
		data16 = xgs_serdes_rd_reg(phydev, aer_blk_reg);
		data16 |= IEEE0BLK_IEEECONTROL0_ENABLE_AN_MASK;
		xgs_serdes_wr_reg(phydev, aer_blk_reg, data16);
	} else if ((serdes_id == SERDES_ID_HX4_AMAC) ||
			(serdes_id == SERDES_ID_KT2_AMAC)) {
		/* unlock lane */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_WC40_DIGITAL4_MISC3r);
		data16 &= ~(DIGITAL4_MISC3_LANEDISABLE_MASK);
		xgs_serdes_wr_reg(phydev, XGXS16G_WC40_DIGITAL4_MISC3r, data16);

		/* disable CL73 BAM */
		data16 = xgs_serdes_rd_reg(phydev,
				XGXS16G_CL73_USERB0_CL73_BAMCTRL1r);
		data16 &= ~(CL73_USERB0_CL73_BAMCTRL1_CL73_BAMEN_MASK);
		xgs_serdes_wr_reg(phydev, XGXS16G_CL73_USERB0_CL73_BAMCTRL1r,
				data16);

		/* Set Local Advertising Configuration */
		data16 = MII_ANA_C37_FD | MII_ANA_C37_PAUSE |
				MII_ANA_C37_ASYM_PAUSE;
		xgs_serdes_wr_reg(phydev, XGXS16G_COMBO_IEEE0_AUTONEGADVr, data16);

		/* Disable BAM in Independent Lane mode. Over 1G AN not supported */
		data16 = 0;
		xgs_serdes_wr_reg(phydev, XGXS16G_BAM_NEXTPAGE_MP5_NEXTPAGECTRLr,
				data16);
		xgs_serdes_wr_reg(phydev, XGXS16G_BAM_NEXTPAGE_UD_FIELDr, data16);

		data16 = SERDESDIGITAL_CONTROL1000X1_CRC_CHECKER_DISABLE_MASK |
			SERDESDIGITAL_CONTROL1000X1_DISABLE_PLL_PWRDWN_MASK;
#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
		data16 |= SERDESDIGITAL_CONTROL1000X1_FIBER_MODE_1000X_MASK;
#endif
		/* Set SGMII mode */
		xgs_serdes_wr_reg(phydev, XGXS16G_SERDESDIGITAL_CONTROL1000X1r,
				data16);

		/* Set autoneg */
		data16 = IEEE0BLK_IEEECONTROL0_ENABLE_AN_MASK |
				IEEE0BLK_IEEECONTROL0_RESTART_AN_MASK;
#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
        if ((phydev->mdio.addr == 2) && of_machine_is_compatible("ericsson,npu1002"))
        data16 = 0;

		/* Set Full-duplex */
		data16 |= IEEE0BLK_IEEECONTROL0_FULL_DUPLEX;
#endif
		xgs_serdes_wr_reg(phydev, XGXS16G_COMBO_IEEE0_MIICNTLr, data16);

		/* Disable 10G parallel detect */
		data16 = 0;
		xgs_serdes_wr_reg(phydev, XGXS16G_AN73_PDET_PARDET10GCONTROLr,
				data16);

		/* Disable BAM mode and Teton mode */
		xgs_serdes_wr_reg(phydev, XGXS16G_BAM_NEXTPAGE_MP5_NEXTPAGECTRLr,
				data16);

		/* Enable lanes */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK1_LANECTRL0r);
		data16 |= XGXSBLK1_LANECTRL0_CL36_PCS_EN_RX_MASK |
				XGXSBLK1_LANECTRL0_CL36_PCS_EN_TX_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK1_LANECTRL0r, data16);

		/* Set elasticity fifo size to 13.5k to support 12k jumbo pkt size*/
		data16 = xgs_serdes_rd_reg(phydev,
					XGXS16G_SERDESDIGITAL_CONTROL1000X3r);
		data16 &= SERDESDIGITAL_CONTROL1000X3_FIFO_ELASICITY_TX_RX_MASK;
		data16 |= (1 << 2);
		xgs_serdes_wr_reg(phydev, XGXS16G_SERDESDIGITAL_CONTROL1000X3r,
				data16);

		/* Enable LPI passthru' for native mode EEE */
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_REMOTEPHY_MISC5r);
		data16 |= XGXS16G_REMOTEPHY_MISC5_LPI_MASK;
		xgs_serdes_wr_reg(phydev, XGXS16G_REMOTEPHY_MISC5r, data16);
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK7_EEECONTROLr);
		data16 |= 0x0007;
		xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK7_EEECONTROLr, data16);
	}
}

/* Needed for HX4/KT2/GH2/WH2 */
static void xgs_serdes_start_pll(struct phy_device *phy_dev)
{
	u16 data16;
	u32 serdes_id;
	u16 serdes_id2;
	u32 count = 100;
	struct phy_device *phydev = phy_dev;
	static u32 serdes_pll_started = 0;

	if (phydev->phy_id != PHY_ID_XGS_AMAC_SERDES)
		return;

	/* PLL started or not */
	if (serdes_pll_started)
		return;

	serdes_id = serdes_get_id(phydev);
	serdes_id2 = xgs_serdes_rd_reg(phydev, XGXS16G_SERDESID_SERDESID2r);

	if (!((serdes_id == SERDES_ID_HX4_AMAC) ||
		(serdes_id == SERDES_ID_KT2_AMAC) ||
		(serdes_id == SERDES_ID_GH2_AMAC)))
		return;

	/* Change PLL calibration threshold to 0xc for GH2/WH2*/
	if ((serdes_id == SERDES_ID_GH2_AMAC) &&
			(serdes_id2 == SERDES_ID2_GH2_AMAC)) {
		data16 = 0xc << XGXS16G_PLL2_CTRL_CAL_TH_SHIFT;
		xgs_serdes_wr_reg(phydev, XGXS16G_PLL2_CTRL1r, data16);
	}

	/* Start PLL Sequencer and wait for PLL to lock */
	data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr);
	data16 |= XGXSBLK0_XGXSCONTROL_START_SEQUENCER_MASK;
	xgs_serdes_wr_reg(phydev, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);

	/* wait for PLL to lock */
	while (count--) {
		data16 = xgs_serdes_rd_reg(phydev, XGXS16G_XGXSBLK0_XGXSSTATUSr);
		if (data16 & XGXSBLK0_XGXSSTATUS_TXPLL_LOCK_MASK)
			break;
		udelay(10);
	}
	if (!count)
		SERDES_ERROR(("amac serdes TXPLL did not lock\n"));
	else
		serdes_pll_started = 1;
}

static int xgs_serdes_config_init(struct phy_device *phydev)
{
	xgs_serdes_reset_core(phydev);
	xgs_serdes_reset(phydev);
	xgs_serdes_init(phydev);
	xgs_serdes_start_pll(phydev);

	return 0;
}

/*
 * 	REGADDR:  0x8304
 *	DESC:     1000X status 1 register
 *	SGMII_MODE       1 = sgmii mode0 = fiber mode (1000-X)
 *	LINK_STATUS      1 = link is up0 = link is down
 *	DUPLEX_STATUS    1 = full-duplex0 = half-duplex
 *	SPEED_STATUS     11 = 2.5G10 = gigabit01 = 100 mbps00 = 10 mbps
 */
static int xgs_serdes_read_status(struct phy_device *phydev)
{
	u16 link_stat;
	u32 serdes_lane;
	u32 reg;

	serdes_lane = xgs_serdes_get_lane(phydev);
	reg = (serdes_lane << XGXS16G_SERDES_LANE_SHIFT) |
				XGXS16G_SERDESDIGITAL_STATUS1000X1r;
	link_stat = xgs_serdes_rd_reg(phydev, reg);

	if (link_stat & 0x2)
		phydev->link = 1;
	else
		phydev->link = 0;

	if (link_stat & 0x4)
		phydev->duplex = 1;
	else
		phydev->duplex = 0;

	phydev->pause = 0;
	phydev->asym_pause = 0;
/*
	link_stat >>= 3;
	link_stat &= 0x3;
*/
	link_stat >>= SERDESDIGITAL_STATUS1000X1_SPEED_STATUS_SHIFT;
	link_stat &= ((1 << SERDESDIGITAL_STATUS1000X1_SPEED_STATUS_BITS) - 1);
	switch(link_stat) {
	case 0:
		phydev->speed = SPEED_10;
		break;
	case 1:
		phydev->speed = SPEED_100;
		break;
	case 2:
		phydev->speed = SPEED_1000;
		break;
	case 3:
		phydev->speed = SPEED_2500;
		break;
	};

	return 0;
}

static int xgs_serdes_config_aneg(struct phy_device *phydev)
{
	u32 serdes_lane;
	u32 reg;
	u16 data16;

	if (AUTONEG_ENABLE != phydev->autoneg)
		return 0;

	serdes_lane = xgs_serdes_get_lane(phydev);

	/* Enable AN 10M/100M/1G */
	reg = (serdes_lane << XGXS16G_SERDES_LANE_SHIFT) |
				XGXS16G_IEEE0BLK_IEEECONTROL0r;
	data16 = xgs_serdes_rd_reg(phydev, reg);
	//data16 |= IEEE0BLK_IEEECONTROL0_ENABLE_AN_MASK;
	data16 |= IEEE0BLK_IEEECONTROL0_RESTART_AN_MASK;
	xgs_serdes_wr_reg(phydev, reg, data16);

	return 0;
}

static int xgs_serdes_aneg_done(struct phy_device *phydev)
{
	u16 link_stat;
	u32 serdes_lane;
	u32 reg;

	serdes_lane = xgs_serdes_get_lane(phydev);
	reg = (serdes_lane << XGXS16G_SERDES_LANE_SHIFT) |
				XGXS16G_SERDESDIGITAL_STATUS1000X2r;
	link_stat = xgs_serdes_rd_reg(phydev, reg);
	if (link_stat & XGXS16G_SERDES_ANEG_MASK)
		return 1;

	return 0;
}

static struct mdio_device_id __maybe_unused xgs_serdes_tbl[] = {
	{ PHY_ID_XGS_AMAC_SERDES, 0xfffffff0 },
	{ }
};
MODULE_DEVICE_TABLE(mdio, xgs_serdes_tbl);

static struct phy_driver xgs_serdes_drivers[] = {
	{
		.phy_id		= PHY_ID_XGS_AMAC_SERDES,
		.phy_id_mask	= 0xfffffff0,
		.name		= "Broadcom XGS AMAC SERDES",
		.config_init	= xgs_serdes_config_init,
		.read_status	= xgs_serdes_read_status,
		.config_aneg	= xgs_serdes_config_aneg,
		.aneg_done	= xgs_serdes_aneg_done,
	}
};

module_phy_driver(xgs_serdes_drivers);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("XGS iProc AMAC serdes driver");
MODULE_LICENSE("GPL");
