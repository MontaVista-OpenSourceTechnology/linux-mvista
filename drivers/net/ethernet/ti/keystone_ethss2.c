/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Hao Zhang <hzhang@ti.com>
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
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/if_vlan.h>
#include <linux/of_mdio.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/ptp_classify.h>
#include <linux/filter.h>

#include "cpsw_ale.h"
#include "keystone_net.h"
#include "cpts.h"
#include "keystone_serdes.h"

#define NETCP2_DRIVER_NAME			"TI KeyStone Ethernet Driver 2"
#define NETCP2_DRIVER_VERSION			"v0.0.0"

#define CPSW2_MODULE_NAME			"keystone-cpsw2"

#define CPSW2_SGMII_IDENT(reg)			((reg >> 16) & 0xffff)
#define CPSW2_MAJOR_VERSION(reg)		(reg >> 8 & 0x7)
#define CPSW2_MINOR_VERSION(reg)		(reg & 0xff)
#define CPSW2_RTL_VERSION(reg)			((reg >> 11) & 0x1f)

#define DEVICE_EMACSL_RESET_POLL_COUNT		100

#define	CPSW2_TIMER_INTERVAL			(HZ / 10)

/* Soft reset register values */
#define SOFT_RESET_MASK				BIT(0)
#define SOFT_RESET				BIT(0)

#define MACSL_RX_ENABLE_CSF			BIT(23)
#define MACSL_RX_ENABLE_EXT_CTL			BIT(18)
#define MACSL_ENABLE				BIT(5)
#define MACSL_DEFAULT_CONFIG			(MACSL_ENABLE |\
						 MACSL_RX_ENABLE_EXT_CTL |\
						 MACSL_RX_ENABLE_CSF)
#define GMACSL_RET_WARN_RESET_INCOMPLETE	-2

#define CPSW2_NUM_PORTS				9
#define CPSW_CTL_P0_TX_CRC_REMOVE		BIT(13)
#define CPSW2_CTL_P0_ENABLE			BIT(2)
#define CPSW2_CTL_VLAN_AWARE			BIT(1)
#define CPSW2_REG_VAL_STAT_ENABLE_ALL(x)	((1 << (x)) - 1)

#define CPSW2_MASK_ALL_PORTS(x)			((1 << (x)) - 1)
#define CPSW2_MASK_PHYS_PORTS(x)		((1 << (x)) - 2)
#define CPSW2_MASK_NO_PORTS			0

#define CPSW2_STATS0_MODULE			0
#define CPSW2_STATS1_MODULE			1
#define CPSW2_STATS2_MODULE			2
#define CPSW2_STATS3_MODULE			3
#define CPSW2_STATS4_MODULE			4
#define CPSW2_STATS5_MODULE			5
#define CPSW2_STATS6_MODULE			6
#define CPSW2_STATS7_MODULE			7
#define CPSW2_STATS8_MODULE			8

#define MAX_SIZE_STREAM_BUFFER		        9504

/* Px_TS_CTL register */
#define CPSW2_TS_RX_ANX_F_EN			BIT(0)
#define CPSW2_TS_RX_VLAN_LT1_EN			BIT(1)
#define CPSW2_TS_RX_VLAN_LT2_EN			BIT(2)
#define CPSW2_TS_RX_ANX_D_EN			BIT(3)
#define CPSW2_TS_TX_ANX_F_EN			BIT(4)
#define CPSW2_TS_TX_VLAN_LT1_EN			BIT(5)
#define CPSW2_TS_TX_VLAN_LT2_EN			BIT(6)
#define CPSW2_TS_TX_ANX_D_EN			BIT(7)
#define CPSW2_TS_LT2_EN				BIT(8)
#define CPSW2_TS_RX_ANX_E_EN			BIT(9)
#define CPSW2_TS_TX_ANX_E_EN			BIT(10)
#define CPSW2_TS_TX_HOST_TS_EN			BIT(11)
#define CPSW2_TS_MSG_TYPE_EN_SHIFT		16
#define CPSW2_TS_MSG_TYPE_EN_MASK		0xffff

/* Px_TS_SEQ_LTYPE */
#define CPSW2_TS_LTYPE1_SHIFT			0
#define CPSW2_TS_LTYPE1_MASK			0xffff
#define CPSW2_TS_SEQ_ID_OFS_SHIFT		16
#define CPSW2_TS_SEQ_ID_OFS_MASK		0x3f

/* Px_TS_VLAN_LTYPE */
#define CPSW2_TS_VLAN_LTYPE1_SHIFT		0
#define CPSW2_TS_VLAN_LTYPE1_MASK		0xffff
#define CPSW2_TS_VLAN_LTYPE2_SHIFT		16
#define CPSW2_TS_VLAN_LTYPE2_MASK		0xffff

/* Px_TS_CTL_LTYPE2 */
#define CPSW2_TS_LTYPE2_SHIFT			0
#define CPSW2_TS_LTYPE2_MASK			0xffff
#define CPSW2_TS_107				BIT(16)
#define CPSW2_TS_129				BIT(17)
#define CPSW2_TS_130				BIT(18)
#define CPSW2_TS_131				BIT(19)
#define CPSW2_TS_132				BIT(20)
#define CPSW2_TS_319				BIT(21)
#define CPSW2_TS_320				BIT(22)
#define CPSW2_TS_TTL_NONZERO			BIT(23)
#define CPSW2_TS_UNI_EN				BIT(24)
#define CPSW2_TS_UNI_EN_SHIFT			24

/* Px_TS_CTL2 */
#define CPSW2_TS_MCAST_TYPE_EN_SHIFT		0
#define CPSW2_TS_MCAST_TYPE_EN_MASK		0xff
#define CPSW2_TS_DOMAIN_OFFSET_SHIFT		16
#define CPSW2_TS_DOMAIN_OFFSET_MASK		0x3f

#define CPSW2_TS_TX_ANX_ALL_EN		 \
		(CPSW2_TS_TX_ANX_D_EN	|\
		 CPSW2_TS_TX_ANX_E_EN	|\
		 CPSW2_TS_TX_ANX_F_EN)


#define CPSW2_TS_RX_ANX_ALL_EN		 \
		(CPSW2_TS_RX_ANX_D_EN	|\
		 CPSW2_TS_RX_ANX_E_EN	|\
		 CPSW2_TS_RX_ANX_F_EN)

#define CPSW2_TS_CTL_DST_PORT			(CPSW2_TS_319)
#define CPSW2_TS_CTL_DST_PORT_SHIFT		21

#define CPSW2_TS_CTL_MADDR_ALL	\
		(CPSW2_TS_107 | CPSW2_TS_129 | CPSW2_TS_130 | \
		 CPSW2_TS_131 | CPSW2_TS_132)

#define CPSW2_TS_CTL_MADDR_SHIFT		16

/* The PTP event messages - Sync, Delay_Req, Pdelay_Req, and Pdelay_Resp. */
#define EVENT_MSG_BITS			((1<<0) | (1<<1) | (1<<2) | (1<<3))

#define MAX_SLAVES				(CPSW2_NUM_PORTS - 1)

/* s: 0-based slave_num */
#define SGMII2_BASE(s) (((s) < 2) ? cpsw_dev->sgmii_port_regs : \
		cpsw_dev->sgmii_port_regs + SGMII_REGS_SIZE * 2)

#define IS_SGMII_MAC_PHY(i) \
	(((i) == SGMII_LINK_MAC_PHY) || ((i) == SGMII_LINK_MAC_PHY_MASTER))


/* CPSW Statistics register map size */
#define CPSW2_STATS_REGS_SIZE			0x200

/* CPSW slave port register map size */
#define CPSW2_SLAVE_REGS_SIZE			0x1000

/* CPSW SERDES */
#define CPSW2_SERDES_MAX_NUM			2
#define CPSW2_LANE_NUM_PER_SERDES		4

struct cpts2_port_ts_ctl {
	int	uni;
	u8	dst_port_map;
	u8	maddr_map;
	u8	ts_mcast_type;
};

/* slave_num: 0-based
 *  port_num: 1-based
 */
struct cpsw2_slave {
	struct cpsw2_slave_regs __iomem	*regs;
	int				 slave_num;
	int				 port_num;
	u32				 mac_control;
	struct phy_device		*phy;
	const char			*phy_id;
	struct cpsw_ale			*ale;
	u32				 link_interface;
	u8				 phy_port_t;
	struct cpts2_port_ts_ctl	 ts_ctl;
};

/* Offset 0x0000 */
struct cpsw2_ss_regs {
	u32	id_ver;
	u32	synce_count;
	u32	synce_mux;
	u32	rsvd;
	u32	ecc_control;
	u32	ecc_int_stat_raw;
	u32	ecc_int_stat_mask;
	u32	ecc_int_en;
};

/* Offset 0x20000 */
struct cpsw2_regs {
	u32	id_ver;
	u32	control;
	u32	rsvd0[2];
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	rsvd1;
	u32	tx_glb_out_flow_thresh_set;
	u32	tx_glb_out_flow_thresh_clr;
	u32	tx_glb_buf_thresh_set_l;
	u32	tx_glb_buf_thresh_set_h;
	u32	tx_glb_buf_thresh_clr_l;
	u32	tx_glb_buf_thresh_clr_h;
};

/* Offset 0x22000 + (0x1000 * slave port #) */
struct cpsw2_slave_regs {
	u32	rsvd0;
	u32	control;
	u32	rsvd1[2];
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	pri_ctl;
	u32	rx_pri_map;
	u32	rx_maxlen;
	u32	tx_blks_pri;
	u32	rsvd2[53];
	u32	rx_dscp_ipv4_map[8];
	u32	rx_dscp_ipv6_map[8];
	u32	pri_send[8];
	u32	pri_idle[8];
	u32	tx_dest_thresh_set_l;
	u32	tx_dest_thresh_set_h;
	u32	tx_dest_thresh_clr_l;
	u32	tx_dest_thresh_clr_h;
	u32	tx_glb_buf_thresh_set_l;
	u32	tx_glb_buf_thresh_set_h;
	u32	tx_glb_buf_thresh_clr_l;
	u32	tx_glb_buf_thresh_clr_h;
	u32	rsvd3[88];
	u32	tx_dest_out_flow_add_val_l;
	u32	tx_dest_out_flow_add_val_h;
	u32	sa_lo;
	u32	sa_hi;
	u32	ts_ctl;
	u32	ts_seq_ltype;
	u32	ts_vlan_ltype;
	u32	ts_ctl_ltype2;
	u32	ts_ctl2;
	u32	rsvd4[3];
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	mac_backoff_test;
	u32	mac_rx_pause_timer;
	u32	rsvd5[3];
	u32	mac_rxn_pause_timer[8];
	u32	mac_tx_pauset_imer;
	u32	rsvd6[3];
	u32	mac_txn_pause_timer[8];
	u32	mac_emu_ctl;
	u32	mac_tx_gap;
};

/* Offset 0x21000 */
struct cpsw2_host_regs {
	u32	rsvd0;
	u32	control;
	u32	rsvd1[2];
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	pri_ctl;
	u32	rx_pri_map;
	u32	rx_maxlen;
	u32	tx_blks_pri;
	u32	rsvd2[53];
	u32	rx_dscp_ipv4_map[8];
	u32	rx_dscp_ipv6_map[8];
	u32	pri_send[8];
	u32	pri_idle[8];
	u32	tx_dest_thresh_set_l;
	u32	tx_dest_thresh_set_h;
	u32	tx_dest_thresh_clr_l;
	u32	tx_dest_thresh_clr_h;
	u32	tx_glb_buf_thresh_set_l;
	u32	tx_glb_buf_thresh_set_h;
	u32	tx_glb_buf_thresh_clr_l;
	u32	tx_glb_buf_thresh_clr_h;
	u32	rsvd3[88];
	u32	src_id_a;
	u32	src_id_b;
	u32	rsvd4[6];
	u32	host_blks_pri;
};

/* Offset 0x3a000 + (0x1000 * port #)*/
struct cpsw2_hw_stats {
	u32	rx_good_frames;
	u32	rx_broadcast_frames;
	u32	rx_multicast_frames;
	u32	rx_pause_frames;
	u32	rx_crc_errors;
	u32	rx_align_code_errors;
	u32	rx_oversized_frames;
	u32	rx_jabber_frames;
	u32	rx_undersized_frames;
	u32	rx_fragments;
	u32	ale_drop;
	u32	ale_overrun_drop;
	u32	rx_bytes;
	u32	tx_good_frames;
	u32	tx_broadcast_frames;
	u32	tx_multicast_frames;
	u32	tx_pause_frames;
	u32	tx_deferred_frames;
	u32	tx_collision_frames;
	u32	tx_single_coll_frames;
	u32	tx_mult_coll_frames;
	u32	tx_excessive_collisions;
	u32	tx_late_collisions;
	u32	rx_ipg_error;	/* Rx inter packet Gap error, 10G only */
	u32	tx_carrier_sense_errors;
	u32	tx_bytes;
	u32	tx_64byte_frames;
	u32	tx_65_to_127byte_frames;
	u32	tx_128_to_255byte_frames;
	u32	tx_256_to_511byte_frames;
	u32	tx_512_to_1023byte_frames;
	u32	tx_1024byte_frames;
	u32	net_bytes;
	u32	rx_drop;
	u32	rx_port_mask_drop;
	u32	tx_drop;
	u32	ale_rate_limit_drop;
	u32	ale_vid_ingress_drop;
	u32	ale_da_eq_sa_drop;
	u32	rsvd0[3];
	u32	ale_unknown_ucast;
	u32	ale_unknown_ucast_bytes;
	u32	ale_unknown_mcast;
	u32	ale_unknown_mcast_bytes;
	u32	ale_unknown_bcast;
	u32	ale_unknown_bcast_bytes;
	u32	ale_pol_match;
	u32	ale_pol_match_red;
	u32	ale_pol_match_yellow;
	u32	rsvd1[44];
	u32	tx_mem_protect_err;
	u32	tx_pri0;
	u32	tx_pri1;
	u32	tx_pri2;
	u32	tx_pri3;
	u32	tx_pri4;
	u32	tx_pri5;
	u32	tx_pri6;
	u32	tx_pri7;
	u32	tx_pri0_bcnt;
	u32	tx_pri1_bcnt;
	u32	tx_pri2_bcnt;
	u32	tx_pri3_bcnt;
	u32	tx_pri4_bcnt;
	u32	tx_pri5_bcnt;
	u32	tx_pri6_bcnt;
	u32	tx_pri7_bcnt;
	u32	tx_pri0_drop;
	u32	tx_pri1_drop;
	u32	tx_pri2_drop;
	u32	tx_pri3_drop;
	u32	tx_pri4_drop;
	u32	tx_pri5_drop;
	u32	tx_pri6_drop;
	u32	tx_pri7_drop;
	u32	tx_pri0_drop_bcnt;
	u32	tx_pri1_drop_bcnt;
	u32	tx_pri2_drop_bcnt;
	u32	tx_pri3_drop_bcnt;
	u32	tx_pri4_drop_bcnt;
	u32	tx_pri5_drop_bcnt;
	u32	tx_pri6_drop_bcnt;
	u32	tx_pri7_drop_bcnt;
};

/* Offset 0x3e000 */
struct cpsw2_ale_regs {
	u32	ale_idver;
	u32	ale_status;
	u32	ale_control;
	u32	ale_control2;
	u32	ale_prescale;
	u32	ale_aging_timer;
	u32	rsvd0[2];
	u32	ale_tblctl;
	u32	rsvd1[4];
	u32	ale_tblw2;
	u32	ale_tblw1;
	u32	ale_tblw0;
	u32	ale_portctl[9];
	u32	rsvd2[11];
	u32	unkn_vlan;
	u32	unkn_mcast_flood;
	u32	unkn_reg_mcast_flood;
	u32	force_untag_egress;
	u32	rsvd3[8];
	u32	vlan_mask_mux[8];
	u32	rsvd4[8];
	u32	policer_port_oui;
	u32	policer_da_sa;
	u32	policer_vlan;
	u32	policer_ethertype_ipsa;
	u32	policer_ipda;
	u32	rsvd5;
	u32	policer_pir;
	u32	policer_cir;
	u32	policer_tbl_ctl;
	u32	policer_control;
	u32	policer_test_ctl;
	u32	policer_hit_status;
	u32	rsvd6;
	u32	thread_default;
	u32	thread_control;
	u32	thread_map;
};

/*
 * Statistic management
 */
struct netcp2_ethtool_stat {
	char desc[ETH_GSTRING_LEN];
	int type;
	u32 size;
	int offset;
};

#define for_each_slave(priv, func, arg...)				\
	do {								\
		int idx, port;						\
		port = (priv)->slave_port;				\
		if ((priv)->multi_if)					\
			(func)((priv)->slaves, ##arg);			\
		else							\
			for (idx = 0; idx < (priv)->num_slaves; idx++)	\
				(func)((priv)->slaves + idx, ##arg);	\
	} while (0)

#define FIELDINFO(_struct, field)	FIELD_SIZEOF(_struct, field),	\
						offsetof(_struct, field)
#define CPSW2_STATS0_INFO(field)	"CPSW_0:"#field, CPSW2_STATS0_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS1_INFO(field)	"CPSW_1:"#field, CPSW2_STATS1_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS2_INFO(field)	"CPSW_2:"#field, CPSW2_STATS2_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS3_INFO(field)	"CPSW_3:"#field, CPSW2_STATS3_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS4_INFO(field)	"CPSW_4:"#field, CPSW2_STATS4_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS5_INFO(field)	"CPSW_5:"#field, CPSW2_STATS5_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS6_INFO(field)	"CPSW_6:"#field, CPSW2_STATS6_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS7_INFO(field)	"CPSW_7:"#field, CPSW2_STATS7_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)
#define CPSW2_STATS8_INFO(field)	"CPSW_8:"#field, CPSW2_STATS8_MODULE,\
					FIELDINFO(struct cpsw2_hw_stats,\
						field)

static const struct netcp2_ethtool_stat et_stats[] = {
	/* CPSW module 0 */
	{CPSW2_STATS0_INFO(rx_good_frames)},
	{CPSW2_STATS0_INFO(rx_broadcast_frames)},
	{CPSW2_STATS0_INFO(rx_multicast_frames)},
	{CPSW2_STATS0_INFO(rx_pause_frames)},
	{CPSW2_STATS0_INFO(rx_crc_errors)},
	{CPSW2_STATS0_INFO(rx_align_code_errors)},
	{CPSW2_STATS0_INFO(rx_oversized_frames)},
	{CPSW2_STATS0_INFO(rx_jabber_frames)},
	{CPSW2_STATS0_INFO(rx_undersized_frames)},
	{CPSW2_STATS0_INFO(rx_fragments)},
	{CPSW2_STATS0_INFO(ale_drop)},
	{CPSW2_STATS0_INFO(ale_overrun_drop)},
	{CPSW2_STATS0_INFO(rx_bytes)},
	{CPSW2_STATS0_INFO(tx_good_frames)},
	{CPSW2_STATS0_INFO(tx_broadcast_frames)},
	{CPSW2_STATS0_INFO(tx_multicast_frames)},
	{CPSW2_STATS0_INFO(tx_pause_frames)},
	{CPSW2_STATS0_INFO(tx_deferred_frames)},
	{CPSW2_STATS0_INFO(tx_collision_frames)},
	{CPSW2_STATS0_INFO(tx_single_coll_frames)},
	{CPSW2_STATS0_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS0_INFO(tx_excessive_collisions)},
	{CPSW2_STATS0_INFO(tx_late_collisions)},
	{CPSW2_STATS0_INFO(rx_ipg_error)},
	{CPSW2_STATS0_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS0_INFO(tx_bytes)},
	{CPSW2_STATS0_INFO(tx_64byte_frames)},
	{CPSW2_STATS0_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS0_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS0_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS0_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS0_INFO(tx_1024byte_frames)},
	{CPSW2_STATS0_INFO(net_bytes)},
	{CPSW2_STATS0_INFO(rx_drop)},
	{CPSW2_STATS0_INFO(rx_port_mask_drop)},
	{CPSW2_STATS0_INFO(tx_drop)},
	{CPSW2_STATS0_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS0_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS0_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS0_INFO(ale_unknown_ucast)},
	{CPSW2_STATS0_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS0_INFO(ale_unknown_mcast)},
	{CPSW2_STATS0_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS0_INFO(ale_unknown_bcast)},
	{CPSW2_STATS0_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS0_INFO(ale_pol_match)},
	{CPSW2_STATS0_INFO(ale_pol_match_red)},
	{CPSW2_STATS0_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS0_INFO(tx_mem_protect_err)},
	{CPSW2_STATS0_INFO(tx_pri0_drop)},
	{CPSW2_STATS0_INFO(tx_pri1_drop)},
	{CPSW2_STATS0_INFO(tx_pri2_drop)},
	{CPSW2_STATS0_INFO(tx_pri3_drop)},
	{CPSW2_STATS0_INFO(tx_pri4_drop)},
	{CPSW2_STATS0_INFO(tx_pri5_drop)},
	{CPSW2_STATS0_INFO(tx_pri6_drop)},
	{CPSW2_STATS0_INFO(tx_pri7_drop)},
	{CPSW2_STATS0_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS0_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 1 */
	{CPSW2_STATS1_INFO(rx_good_frames)},
	{CPSW2_STATS1_INFO(rx_broadcast_frames)},
	{CPSW2_STATS1_INFO(rx_multicast_frames)},
	{CPSW2_STATS1_INFO(rx_pause_frames)},
	{CPSW2_STATS1_INFO(rx_crc_errors)},
	{CPSW2_STATS1_INFO(rx_align_code_errors)},
	{CPSW2_STATS1_INFO(rx_oversized_frames)},
	{CPSW2_STATS1_INFO(rx_jabber_frames)},
	{CPSW2_STATS1_INFO(rx_undersized_frames)},
	{CPSW2_STATS1_INFO(rx_fragments)},
	{CPSW2_STATS1_INFO(ale_drop)},
	{CPSW2_STATS1_INFO(ale_overrun_drop)},
	{CPSW2_STATS1_INFO(rx_bytes)},
	{CPSW2_STATS1_INFO(tx_good_frames)},
	{CPSW2_STATS1_INFO(tx_broadcast_frames)},
	{CPSW2_STATS1_INFO(tx_multicast_frames)},
	{CPSW2_STATS1_INFO(tx_pause_frames)},
	{CPSW2_STATS1_INFO(tx_deferred_frames)},
	{CPSW2_STATS1_INFO(tx_collision_frames)},
	{CPSW2_STATS1_INFO(tx_single_coll_frames)},
	{CPSW2_STATS1_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS1_INFO(tx_excessive_collisions)},
	{CPSW2_STATS1_INFO(tx_late_collisions)},
	{CPSW2_STATS1_INFO(rx_ipg_error)},
	{CPSW2_STATS1_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS1_INFO(tx_bytes)},
	{CPSW2_STATS1_INFO(tx_64byte_frames)},
	{CPSW2_STATS1_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS1_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS1_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS1_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS1_INFO(tx_1024byte_frames)},
	{CPSW2_STATS1_INFO(net_bytes)},
	{CPSW2_STATS1_INFO(rx_drop)},
	{CPSW2_STATS1_INFO(rx_port_mask_drop)},
	{CPSW2_STATS1_INFO(tx_drop)},
	{CPSW2_STATS1_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS1_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS1_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS1_INFO(ale_unknown_ucast)},
	{CPSW2_STATS1_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS1_INFO(ale_unknown_mcast)},
	{CPSW2_STATS1_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS1_INFO(ale_unknown_bcast)},
	{CPSW2_STATS1_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS1_INFO(ale_pol_match)},
	{CPSW2_STATS1_INFO(ale_pol_match_red)},
	{CPSW2_STATS1_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS1_INFO(tx_mem_protect_err)},
	{CPSW2_STATS1_INFO(tx_pri0_drop)},
	{CPSW2_STATS1_INFO(tx_pri1_drop)},
	{CPSW2_STATS1_INFO(tx_pri2_drop)},
	{CPSW2_STATS1_INFO(tx_pri3_drop)},
	{CPSW2_STATS1_INFO(tx_pri4_drop)},
	{CPSW2_STATS1_INFO(tx_pri5_drop)},
	{CPSW2_STATS1_INFO(tx_pri6_drop)},
	{CPSW2_STATS1_INFO(tx_pri7_drop)},
	{CPSW2_STATS1_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS1_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 2 */
	{CPSW2_STATS2_INFO(rx_good_frames)},
	{CPSW2_STATS2_INFO(rx_broadcast_frames)},
	{CPSW2_STATS2_INFO(rx_multicast_frames)},
	{CPSW2_STATS2_INFO(rx_pause_frames)},
	{CPSW2_STATS2_INFO(rx_crc_errors)},
	{CPSW2_STATS2_INFO(rx_align_code_errors)},
	{CPSW2_STATS2_INFO(rx_oversized_frames)},
	{CPSW2_STATS2_INFO(rx_jabber_frames)},
	{CPSW2_STATS2_INFO(rx_undersized_frames)},
	{CPSW2_STATS2_INFO(rx_fragments)},
	{CPSW2_STATS2_INFO(ale_drop)},
	{CPSW2_STATS2_INFO(ale_overrun_drop)},
	{CPSW2_STATS2_INFO(rx_bytes)},
	{CPSW2_STATS2_INFO(tx_good_frames)},
	{CPSW2_STATS2_INFO(tx_broadcast_frames)},
	{CPSW2_STATS2_INFO(tx_multicast_frames)},
	{CPSW2_STATS2_INFO(tx_pause_frames)},
	{CPSW2_STATS2_INFO(tx_deferred_frames)},
	{CPSW2_STATS2_INFO(tx_collision_frames)},
	{CPSW2_STATS2_INFO(tx_single_coll_frames)},
	{CPSW2_STATS2_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS2_INFO(tx_excessive_collisions)},
	{CPSW2_STATS2_INFO(tx_late_collisions)},
	{CPSW2_STATS2_INFO(rx_ipg_error)},
	{CPSW2_STATS2_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS2_INFO(tx_bytes)},
	{CPSW2_STATS2_INFO(tx_64byte_frames)},
	{CPSW2_STATS2_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS2_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS2_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS2_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS2_INFO(tx_1024byte_frames)},
	{CPSW2_STATS2_INFO(net_bytes)},
	{CPSW2_STATS2_INFO(rx_drop)},
	{CPSW2_STATS2_INFO(rx_port_mask_drop)},
	{CPSW2_STATS2_INFO(tx_drop)},
	{CPSW2_STATS2_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS2_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS2_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS2_INFO(ale_unknown_ucast)},
	{CPSW2_STATS2_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS2_INFO(ale_unknown_mcast)},
	{CPSW2_STATS2_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS2_INFO(ale_unknown_bcast)},
	{CPSW2_STATS2_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS2_INFO(ale_pol_match)},
	{CPSW2_STATS2_INFO(ale_pol_match_red)},
	{CPSW2_STATS2_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS2_INFO(tx_mem_protect_err)},
	{CPSW2_STATS2_INFO(tx_pri0_drop)},
	{CPSW2_STATS2_INFO(tx_pri1_drop)},
	{CPSW2_STATS2_INFO(tx_pri2_drop)},
	{CPSW2_STATS2_INFO(tx_pri3_drop)},
	{CPSW2_STATS2_INFO(tx_pri4_drop)},
	{CPSW2_STATS2_INFO(tx_pri5_drop)},
	{CPSW2_STATS2_INFO(tx_pri6_drop)},
	{CPSW2_STATS2_INFO(tx_pri7_drop)},
	{CPSW2_STATS2_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS2_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 3 */
	{CPSW2_STATS3_INFO(rx_good_frames)},
	{CPSW2_STATS3_INFO(rx_broadcast_frames)},
	{CPSW2_STATS3_INFO(rx_multicast_frames)},
	{CPSW2_STATS3_INFO(rx_pause_frames)},
	{CPSW2_STATS3_INFO(rx_crc_errors)},
	{CPSW2_STATS3_INFO(rx_align_code_errors)},
	{CPSW2_STATS3_INFO(rx_oversized_frames)},
	{CPSW2_STATS3_INFO(rx_jabber_frames)},
	{CPSW2_STATS3_INFO(rx_undersized_frames)},
	{CPSW2_STATS3_INFO(rx_fragments)},
	{CPSW2_STATS3_INFO(ale_drop)},
	{CPSW2_STATS3_INFO(ale_overrun_drop)},
	{CPSW2_STATS3_INFO(rx_bytes)},
	{CPSW2_STATS3_INFO(tx_good_frames)},
	{CPSW2_STATS3_INFO(tx_broadcast_frames)},
	{CPSW2_STATS3_INFO(tx_multicast_frames)},
	{CPSW2_STATS3_INFO(tx_pause_frames)},
	{CPSW2_STATS3_INFO(tx_deferred_frames)},
	{CPSW2_STATS3_INFO(tx_collision_frames)},
	{CPSW2_STATS3_INFO(tx_single_coll_frames)},
	{CPSW2_STATS3_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS3_INFO(tx_excessive_collisions)},
	{CPSW2_STATS3_INFO(tx_late_collisions)},
	{CPSW2_STATS3_INFO(rx_ipg_error)},
	{CPSW2_STATS3_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS3_INFO(tx_bytes)},
	{CPSW2_STATS3_INFO(tx_64byte_frames)},
	{CPSW2_STATS3_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS3_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS3_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS3_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS3_INFO(tx_1024byte_frames)},
	{CPSW2_STATS3_INFO(net_bytes)},
	{CPSW2_STATS3_INFO(rx_drop)},
	{CPSW2_STATS3_INFO(rx_port_mask_drop)},
	{CPSW2_STATS3_INFO(tx_drop)},
	{CPSW2_STATS3_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS3_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS3_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS3_INFO(ale_unknown_ucast)},
	{CPSW2_STATS3_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS3_INFO(ale_unknown_mcast)},
	{CPSW2_STATS3_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS3_INFO(ale_unknown_bcast)},
	{CPSW2_STATS3_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS3_INFO(ale_pol_match)},
	{CPSW2_STATS3_INFO(ale_pol_match_red)},
	{CPSW2_STATS3_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS3_INFO(tx_mem_protect_err)},
	{CPSW2_STATS3_INFO(tx_pri0_drop)},
	{CPSW2_STATS3_INFO(tx_pri1_drop)},
	{CPSW2_STATS3_INFO(tx_pri2_drop)},
	{CPSW2_STATS3_INFO(tx_pri3_drop)},
	{CPSW2_STATS3_INFO(tx_pri4_drop)},
	{CPSW2_STATS3_INFO(tx_pri5_drop)},
	{CPSW2_STATS3_INFO(tx_pri6_drop)},
	{CPSW2_STATS3_INFO(tx_pri7_drop)},
	{CPSW2_STATS3_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS3_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 4 */
	{CPSW2_STATS4_INFO(rx_good_frames)},
	{CPSW2_STATS4_INFO(rx_broadcast_frames)},
	{CPSW2_STATS4_INFO(rx_multicast_frames)},
	{CPSW2_STATS4_INFO(rx_pause_frames)},
	{CPSW2_STATS4_INFO(rx_crc_errors)},
	{CPSW2_STATS4_INFO(rx_align_code_errors)},
	{CPSW2_STATS4_INFO(rx_oversized_frames)},
	{CPSW2_STATS4_INFO(rx_jabber_frames)},
	{CPSW2_STATS4_INFO(rx_undersized_frames)},
	{CPSW2_STATS4_INFO(rx_fragments)},
	{CPSW2_STATS4_INFO(ale_drop)},
	{CPSW2_STATS4_INFO(ale_overrun_drop)},
	{CPSW2_STATS4_INFO(rx_bytes)},
	{CPSW2_STATS4_INFO(tx_good_frames)},
	{CPSW2_STATS4_INFO(tx_broadcast_frames)},
	{CPSW2_STATS4_INFO(tx_multicast_frames)},
	{CPSW2_STATS4_INFO(tx_pause_frames)},
	{CPSW2_STATS4_INFO(tx_deferred_frames)},
	{CPSW2_STATS4_INFO(tx_collision_frames)},
	{CPSW2_STATS4_INFO(tx_single_coll_frames)},
	{CPSW2_STATS4_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS4_INFO(tx_excessive_collisions)},
	{CPSW2_STATS4_INFO(tx_late_collisions)},
	{CPSW2_STATS4_INFO(rx_ipg_error)},
	{CPSW2_STATS4_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS4_INFO(tx_bytes)},
	{CPSW2_STATS4_INFO(tx_64byte_frames)},
	{CPSW2_STATS4_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS4_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS4_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS4_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS4_INFO(tx_1024byte_frames)},
	{CPSW2_STATS4_INFO(net_bytes)},
	{CPSW2_STATS4_INFO(rx_drop)},
	{CPSW2_STATS4_INFO(rx_port_mask_drop)},
	{CPSW2_STATS4_INFO(tx_drop)},
	{CPSW2_STATS4_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS4_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS4_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS4_INFO(ale_unknown_ucast)},
	{CPSW2_STATS4_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS4_INFO(ale_unknown_mcast)},
	{CPSW2_STATS4_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS4_INFO(ale_unknown_bcast)},
	{CPSW2_STATS4_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS4_INFO(ale_pol_match)},
	{CPSW2_STATS4_INFO(ale_pol_match_red)},
	{CPSW2_STATS4_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS4_INFO(tx_mem_protect_err)},
	{CPSW2_STATS4_INFO(tx_pri0_drop)},
	{CPSW2_STATS4_INFO(tx_pri1_drop)},
	{CPSW2_STATS4_INFO(tx_pri2_drop)},
	{CPSW2_STATS4_INFO(tx_pri3_drop)},
	{CPSW2_STATS4_INFO(tx_pri4_drop)},
	{CPSW2_STATS4_INFO(tx_pri5_drop)},
	{CPSW2_STATS4_INFO(tx_pri6_drop)},
	{CPSW2_STATS4_INFO(tx_pri7_drop)},
	{CPSW2_STATS4_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS4_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 5 */
	{CPSW2_STATS5_INFO(rx_good_frames)},
	{CPSW2_STATS5_INFO(rx_broadcast_frames)},
	{CPSW2_STATS5_INFO(rx_multicast_frames)},
	{CPSW2_STATS5_INFO(rx_pause_frames)},
	{CPSW2_STATS5_INFO(rx_crc_errors)},
	{CPSW2_STATS5_INFO(rx_align_code_errors)},
	{CPSW2_STATS5_INFO(rx_oversized_frames)},
	{CPSW2_STATS5_INFO(rx_jabber_frames)},
	{CPSW2_STATS5_INFO(rx_undersized_frames)},
	{CPSW2_STATS5_INFO(rx_fragments)},
	{CPSW2_STATS5_INFO(ale_drop)},
	{CPSW2_STATS5_INFO(ale_overrun_drop)},
	{CPSW2_STATS5_INFO(rx_bytes)},
	{CPSW2_STATS5_INFO(tx_good_frames)},
	{CPSW2_STATS5_INFO(tx_broadcast_frames)},
	{CPSW2_STATS5_INFO(tx_multicast_frames)},
	{CPSW2_STATS5_INFO(tx_pause_frames)},
	{CPSW2_STATS5_INFO(tx_deferred_frames)},
	{CPSW2_STATS5_INFO(tx_collision_frames)},
	{CPSW2_STATS5_INFO(tx_single_coll_frames)},
	{CPSW2_STATS5_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS5_INFO(tx_excessive_collisions)},
	{CPSW2_STATS5_INFO(tx_late_collisions)},
	{CPSW2_STATS5_INFO(rx_ipg_error)},
	{CPSW2_STATS5_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS5_INFO(tx_bytes)},
	{CPSW2_STATS5_INFO(tx_64byte_frames)},
	{CPSW2_STATS5_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS5_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS5_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS5_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS5_INFO(tx_1024byte_frames)},
	{CPSW2_STATS5_INFO(net_bytes)},
	{CPSW2_STATS5_INFO(rx_drop)},
	{CPSW2_STATS5_INFO(rx_port_mask_drop)},
	{CPSW2_STATS5_INFO(tx_drop)},
	{CPSW2_STATS5_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS5_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS5_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS5_INFO(ale_unknown_ucast)},
	{CPSW2_STATS5_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS5_INFO(ale_unknown_mcast)},
	{CPSW2_STATS5_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS5_INFO(ale_unknown_bcast)},
	{CPSW2_STATS5_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS5_INFO(ale_pol_match)},
	{CPSW2_STATS5_INFO(ale_pol_match_red)},
	{CPSW2_STATS5_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS5_INFO(tx_mem_protect_err)},
	{CPSW2_STATS5_INFO(tx_pri0_drop)},
	{CPSW2_STATS5_INFO(tx_pri1_drop)},
	{CPSW2_STATS5_INFO(tx_pri2_drop)},
	{CPSW2_STATS5_INFO(tx_pri3_drop)},
	{CPSW2_STATS5_INFO(tx_pri4_drop)},
	{CPSW2_STATS5_INFO(tx_pri5_drop)},
	{CPSW2_STATS5_INFO(tx_pri6_drop)},
	{CPSW2_STATS5_INFO(tx_pri7_drop)},
	{CPSW2_STATS5_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS5_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 6 */
	{CPSW2_STATS6_INFO(rx_good_frames)},
	{CPSW2_STATS6_INFO(rx_broadcast_frames)},
	{CPSW2_STATS6_INFO(rx_multicast_frames)},
	{CPSW2_STATS6_INFO(rx_pause_frames)},
	{CPSW2_STATS6_INFO(rx_crc_errors)},
	{CPSW2_STATS6_INFO(rx_align_code_errors)},
	{CPSW2_STATS6_INFO(rx_oversized_frames)},
	{CPSW2_STATS6_INFO(rx_jabber_frames)},
	{CPSW2_STATS6_INFO(rx_undersized_frames)},
	{CPSW2_STATS6_INFO(rx_fragments)},
	{CPSW2_STATS6_INFO(ale_drop)},
	{CPSW2_STATS6_INFO(ale_overrun_drop)},
	{CPSW2_STATS6_INFO(rx_bytes)},
	{CPSW2_STATS6_INFO(tx_good_frames)},
	{CPSW2_STATS6_INFO(tx_broadcast_frames)},
	{CPSW2_STATS6_INFO(tx_multicast_frames)},
	{CPSW2_STATS6_INFO(tx_pause_frames)},
	{CPSW2_STATS6_INFO(tx_deferred_frames)},
	{CPSW2_STATS6_INFO(tx_collision_frames)},
	{CPSW2_STATS6_INFO(tx_single_coll_frames)},
	{CPSW2_STATS6_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS6_INFO(tx_excessive_collisions)},
	{CPSW2_STATS6_INFO(tx_late_collisions)},
	{CPSW2_STATS6_INFO(rx_ipg_error)},
	{CPSW2_STATS6_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS6_INFO(tx_bytes)},
	{CPSW2_STATS6_INFO(tx_64byte_frames)},
	{CPSW2_STATS6_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS6_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS6_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS6_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS6_INFO(tx_1024byte_frames)},
	{CPSW2_STATS6_INFO(net_bytes)},
	{CPSW2_STATS6_INFO(rx_drop)},
	{CPSW2_STATS6_INFO(rx_port_mask_drop)},
	{CPSW2_STATS6_INFO(tx_drop)},
	{CPSW2_STATS6_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS6_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS6_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS6_INFO(ale_unknown_ucast)},
	{CPSW2_STATS6_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS6_INFO(ale_unknown_mcast)},
	{CPSW2_STATS6_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS6_INFO(ale_unknown_bcast)},
	{CPSW2_STATS6_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS6_INFO(ale_pol_match)},
	{CPSW2_STATS6_INFO(ale_pol_match_red)},
	{CPSW2_STATS6_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS6_INFO(tx_mem_protect_err)},
	{CPSW2_STATS6_INFO(tx_pri0_drop)},
	{CPSW2_STATS6_INFO(tx_pri1_drop)},
	{CPSW2_STATS6_INFO(tx_pri2_drop)},
	{CPSW2_STATS6_INFO(tx_pri3_drop)},
	{CPSW2_STATS6_INFO(tx_pri4_drop)},
	{CPSW2_STATS6_INFO(tx_pri5_drop)},
	{CPSW2_STATS6_INFO(tx_pri6_drop)},
	{CPSW2_STATS6_INFO(tx_pri7_drop)},
	{CPSW2_STATS6_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS6_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 7 */
	{CPSW2_STATS7_INFO(rx_good_frames)},
	{CPSW2_STATS7_INFO(rx_broadcast_frames)},
	{CPSW2_STATS7_INFO(rx_multicast_frames)},
	{CPSW2_STATS7_INFO(rx_pause_frames)},
	{CPSW2_STATS7_INFO(rx_crc_errors)},
	{CPSW2_STATS7_INFO(rx_align_code_errors)},
	{CPSW2_STATS7_INFO(rx_oversized_frames)},
	{CPSW2_STATS7_INFO(rx_jabber_frames)},
	{CPSW2_STATS7_INFO(rx_undersized_frames)},
	{CPSW2_STATS7_INFO(rx_fragments)},
	{CPSW2_STATS7_INFO(ale_drop)},
	{CPSW2_STATS7_INFO(ale_overrun_drop)},
	{CPSW2_STATS7_INFO(rx_bytes)},
	{CPSW2_STATS7_INFO(tx_good_frames)},
	{CPSW2_STATS7_INFO(tx_broadcast_frames)},
	{CPSW2_STATS7_INFO(tx_multicast_frames)},
	{CPSW2_STATS7_INFO(tx_pause_frames)},
	{CPSW2_STATS7_INFO(tx_deferred_frames)},
	{CPSW2_STATS7_INFO(tx_collision_frames)},
	{CPSW2_STATS7_INFO(tx_single_coll_frames)},
	{CPSW2_STATS7_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS7_INFO(tx_excessive_collisions)},
	{CPSW2_STATS7_INFO(tx_late_collisions)},
	{CPSW2_STATS7_INFO(rx_ipg_error)},
	{CPSW2_STATS7_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS7_INFO(tx_bytes)},
	{CPSW2_STATS7_INFO(tx_64byte_frames)},
	{CPSW2_STATS7_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS7_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS7_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS7_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS7_INFO(tx_1024byte_frames)},
	{CPSW2_STATS7_INFO(net_bytes)},
	{CPSW2_STATS7_INFO(rx_drop)},
	{CPSW2_STATS7_INFO(rx_port_mask_drop)},
	{CPSW2_STATS7_INFO(tx_drop)},
	{CPSW2_STATS7_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS7_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS7_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS7_INFO(ale_unknown_ucast)},
	{CPSW2_STATS7_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS7_INFO(ale_unknown_mcast)},
	{CPSW2_STATS7_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS7_INFO(ale_unknown_bcast)},
	{CPSW2_STATS7_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS7_INFO(ale_pol_match)},
	{CPSW2_STATS7_INFO(ale_pol_match_red)},
	{CPSW2_STATS7_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS7_INFO(tx_mem_protect_err)},
	{CPSW2_STATS7_INFO(tx_pri0_drop)},
	{CPSW2_STATS7_INFO(tx_pri1_drop)},
	{CPSW2_STATS7_INFO(tx_pri2_drop)},
	{CPSW2_STATS7_INFO(tx_pri3_drop)},
	{CPSW2_STATS7_INFO(tx_pri4_drop)},
	{CPSW2_STATS7_INFO(tx_pri5_drop)},
	{CPSW2_STATS7_INFO(tx_pri6_drop)},
	{CPSW2_STATS7_INFO(tx_pri7_drop)},
	{CPSW2_STATS7_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS7_INFO(tx_pri7_drop_bcnt)},
	/* CPSW module 8 */
	{CPSW2_STATS8_INFO(rx_good_frames)},
	{CPSW2_STATS8_INFO(rx_broadcast_frames)},
	{CPSW2_STATS8_INFO(rx_multicast_frames)},
	{CPSW2_STATS8_INFO(rx_pause_frames)},
	{CPSW2_STATS8_INFO(rx_crc_errors)},
	{CPSW2_STATS8_INFO(rx_align_code_errors)},
	{CPSW2_STATS8_INFO(rx_oversized_frames)},
	{CPSW2_STATS8_INFO(rx_jabber_frames)},
	{CPSW2_STATS8_INFO(rx_undersized_frames)},
	{CPSW2_STATS8_INFO(rx_fragments)},
	{CPSW2_STATS8_INFO(ale_drop)},
	{CPSW2_STATS8_INFO(ale_overrun_drop)},
	{CPSW2_STATS8_INFO(rx_bytes)},
	{CPSW2_STATS8_INFO(tx_good_frames)},
	{CPSW2_STATS8_INFO(tx_broadcast_frames)},
	{CPSW2_STATS8_INFO(tx_multicast_frames)},
	{CPSW2_STATS8_INFO(tx_pause_frames)},
	{CPSW2_STATS8_INFO(tx_deferred_frames)},
	{CPSW2_STATS8_INFO(tx_collision_frames)},
	{CPSW2_STATS8_INFO(tx_single_coll_frames)},
	{CPSW2_STATS8_INFO(tx_mult_coll_frames)},
	{CPSW2_STATS8_INFO(tx_excessive_collisions)},
	{CPSW2_STATS8_INFO(tx_late_collisions)},
	{CPSW2_STATS8_INFO(rx_ipg_error)},
	{CPSW2_STATS8_INFO(tx_carrier_sense_errors)},
	{CPSW2_STATS8_INFO(tx_bytes)},
	{CPSW2_STATS8_INFO(tx_64byte_frames)},
	{CPSW2_STATS8_INFO(tx_65_to_127byte_frames)},
	{CPSW2_STATS8_INFO(tx_128_to_255byte_frames)},
	{CPSW2_STATS8_INFO(tx_256_to_511byte_frames)},
	{CPSW2_STATS8_INFO(tx_512_to_1023byte_frames)},
	{CPSW2_STATS8_INFO(tx_1024byte_frames)},
	{CPSW2_STATS8_INFO(net_bytes)},
	{CPSW2_STATS8_INFO(rx_drop)},
	{CPSW2_STATS8_INFO(rx_port_mask_drop)},
	{CPSW2_STATS8_INFO(tx_drop)},
	{CPSW2_STATS8_INFO(ale_rate_limit_drop)},
	{CPSW2_STATS8_INFO(ale_vid_ingress_drop)},
	{CPSW2_STATS8_INFO(ale_da_eq_sa_drop)},
	{CPSW2_STATS8_INFO(ale_unknown_ucast)},
	{CPSW2_STATS8_INFO(ale_unknown_ucast_bytes)},
	{CPSW2_STATS8_INFO(ale_unknown_mcast)},
	{CPSW2_STATS8_INFO(ale_unknown_mcast_bytes)},
	{CPSW2_STATS8_INFO(ale_unknown_bcast)},
	{CPSW2_STATS8_INFO(ale_unknown_bcast_bytes)},
	{CPSW2_STATS8_INFO(ale_pol_match)},
	{CPSW2_STATS8_INFO(ale_pol_match_red)},
	{CPSW2_STATS8_INFO(ale_pol_match_yellow)},
	{CPSW2_STATS8_INFO(tx_mem_protect_err)},
	{CPSW2_STATS8_INFO(tx_pri0_drop)},
	{CPSW2_STATS8_INFO(tx_pri1_drop)},
	{CPSW2_STATS8_INFO(tx_pri2_drop)},
	{CPSW2_STATS8_INFO(tx_pri3_drop)},
	{CPSW2_STATS8_INFO(tx_pri4_drop)},
	{CPSW2_STATS8_INFO(tx_pri5_drop)},
	{CPSW2_STATS8_INFO(tx_pri6_drop)},
	{CPSW2_STATS8_INFO(tx_pri7_drop)},
	{CPSW2_STATS8_INFO(tx_pri0_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri1_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri2_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri3_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri4_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri5_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri6_drop_bcnt)},
	{CPSW2_STATS8_INFO(tx_pri7_drop_bcnt)},
};

#define ETHTOOL_PORT_STATS_NUM (ARRAY_SIZE(et_stats)/CPSW2_NUM_PORTS)
#define ETHTOOL_STATS_NUM(num_ports)	(ETHTOOL_PORT_STATS_NUM * (num_ports))

struct cpsw2_priv {
	struct device			*dev;
	struct clk			*cpgmac;
	struct netcp_device		*netcp_device;
	u32				 num_slaves;
	u32				 ale_ageout;
	u32				 ale_entries;
	u32				 ale_ports;
	u32				 sgmii_module_ofs;
	u32				 switch_module_ofs;
	u32				 host_port_reg_ofs;
	u32				 slave_reg_ofs;
	u32				 hw_stats_reg_ofs;
	u32				 ale_reg_ofs;
	u32				 cpts_reg_ofs;

	int				 host_port;
	u32				 rx_packet_max;

	struct cpsw2_regs __iomem	*regs;
	struct cpsw2_ss_regs __iomem	*ss_regs;
	struct cpsw2_hw_stats __iomem	*hw_stats_regs[CPSW2_NUM_PORTS];
	struct cpsw2_host_regs __iomem	*host_port_regs;
	struct cpsw2_ale_regs __iomem	*ale_reg;

	void __iomem			*sgmii_port_regs;

	struct cpsw_ale			*ale;
	atomic_t			 ale_refcnt;

	u32				 link[MAX_SLAVES];
	struct device_node		*phy_node[MAX_SLAVES];

	u32				 intf_tx_queues;

	u32				 multi_if;
	u32				 slaves_per_interface;
	u32				 num_interfaces;
	struct device_node		*interfaces;
	struct list_head		 cpsw_intf_head;

	u64				 hw_stats[ARRAY_SIZE(et_stats)];
	u32				 hw_stats_prev[ARRAY_SIZE(et_stats)];
	int				 init_serdes_at_probe;
	struct kobject			kobj;
	struct kobject			tx_pri_kobj;
	struct kobject			pvlan_kobj;
	struct kobject			port_ts_kobj[MAX_SLAVES];
	struct kobject			stats_kobj;
	spinlock_t			hw_stats_lock;
#ifdef CONFIG_TI_CPTS
	struct cpts			cpts;
#endif
	int				cpts_registered;
	int				force_no_hwtstamp;
	void __iomem			*serdes_regs[CPSW2_SERDES_MAX_NUM];
	u32				num_serdes;
	u32				serdes_lanes;
	struct serdes			serdes;
	u32				opened;
};

/* slave_port: 0-based (currently relevant only in multi_if mode)
 */
struct cpsw2_intf {
	struct net_device	*ndev;
	struct device		*dev;
	struct cpsw2_priv	*cpsw_priv;
	struct device_node	*phy_node;
	u32			 num_slaves;
	u32			 slave_port;
	struct cpsw2_slave	*slaves;
	u32			 intf_tx_queues;
	const char		*tx_chan_name;
	u32			 tx_queue_depth;
	struct netcp_tx_pipe	 tx_pipe;
	u32			 multi_if;
	struct list_head	 cpsw_intf_list;
	struct timer_list	 timer;
	u32			 link_state;
};

struct cpsw2_attribute {
	struct attribute attr;
	ssize_t (*show)(struct cpsw2_priv *cpsw_dev,
		struct cpsw2_attribute *attr, char *buf);
	ssize_t	(*store)(struct cpsw2_priv *cpsw_dev,
		struct cpsw2_attribute *attr, const char *, size_t);
	const struct cpsw2_mod_info *info;
	ssize_t info_size;
	void *context;
};
#define to_cpsw2_attr(_attr) container_of(_attr, struct cpsw2_attribute, attr)

#define to_cpsw2_dev(obj) container_of(obj, struct cpsw2_priv, kobj)
#define tx_pri_to_cpsw2_dev(obj) container_of(obj, struct cpsw2_priv, \
						tx_pri_kobj)
#define pvlan_to_cpsw2_dev(obj) container_of(obj, struct cpsw2_priv, pvlan_kobj)
#define stats_to_cpsw2_dev(obj) container_of(obj, struct cpsw2_priv, stats_kobj)

#define BITS(x)			(BIT(x) - 1)
#define BITMASK(n, s)		(BITS(n) << (s))
#define cpsw2_mod_info_field_val(r, i) \
	((r & BITMASK(i->bits, i->shift)) >> i->shift)

#define for_each_intf(i, priv) \
	list_for_each_entry((i), &(priv)->cpsw_intf_head, cpsw_intf_list)

#define __CPSW2_ATTR_FULL(_name, _mode, _show, _store, _info,	\
				_info_size, _ctxt)		\
	{ \
		.attr = {.name = __stringify(_name), .mode = _mode },	\
		.show	= _show,		\
		.store	= _store,		\
		.info	= _info,		\
		.info_size = _info_size,	\
		.context = (_ctxt),		\
	}

#define __CPSW2_ATTR(_name, _mode, _show, _store, _info) \
		__CPSW2_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), NULL)

#define __CPSW2_CTXT_ATTR(_name, _mode, _show, _store, _info, _ctxt) \
		__CPSW2_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), _ctxt)

struct cpsw2_mod_info {
	const char	*name;
	int		shift;
	int		bits;
};

struct cpsw2_parse_result {
	int control;
	int port;
	u32 value;
};

static ssize_t cpsw2_attr_info_show(const struct cpsw2_mod_info *info,
				int info_size, u32 reg, char *buf)
{
	int i, len = 0;

	for (i = 0; i < info_size; i++, info++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"%s=%d\n", info->name,
			(int)cpsw2_mod_info_field_val(reg, info));
	}

	return len;
}

static ssize_t cpsw2_attr_parse_set_command(struct cpsw2_priv *cpsw_dev,
				struct cpsw2_attribute *attr,
				const char *buf, size_t count,
				struct cpsw2_parse_result *res)
{
	char ctrl_str[33], tmp_str[9];
	int port = -1, value, len, control;
	unsigned long end;
	const struct cpsw2_mod_info *info = attr->info;

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		++buf;
		len = strcspn(buf, "=");
		if (len >= 8)
			return -ENOMEM;
		strncpy(tmp_str, buf, len);
		tmp_str[len] = '\0';
		if (kstrtoul(tmp_str, 0, &end))
			return -EINVAL;
		port = (int)end;
		buf += len;
	}

	if (*buf != '=')
		return -EINVAL;

	if (kstrtoul(buf + 1, 0, &end))
		return -EINVAL;

	value = (int)end;

	for (control = 0; control < attr->info_size; control++)
		if (strcmp(ctrl_str, info[control].name) == 0)
			break;

	if (control >= attr->info_size)
		return -ENOENT;

	res->control = control;
	res->port = port;
	res->value = value;

	dev_info(cpsw_dev->dev, "parsed command %s.%d=%d\n",
		attr->info[control].name, port, value);

	return 0;
}

static inline void cpsw2_info_set_reg_field(void __iomem *r,
		const struct cpsw2_mod_info *info, int val)
{
	u32 rv;

	rv = readl(r);
	rv = ((rv & ~BITMASK(info->bits, info->shift)) | (val << info->shift));
	writel(rv, r);
}

static ssize_t cpsw2_version_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = readl(&cpsw_dev->regs->id_ver);

	return snprintf(buf, PAGE_SIZE,
		"cpsw version %d.%d (%d) SGMII identification value 0x%x\n",
		 CPSW2_MAJOR_VERSION(reg), CPSW2_MINOR_VERSION(reg),
		 CPSW2_RTL_VERSION(reg), CPSW2_SGMII_IDENT(reg));
}

static struct cpsw2_attribute cpsw_version_attribute =
	__ATTR(version, S_IRUGO, cpsw2_version_show, NULL);

static const struct cpsw2_mod_info cpsw_controls[] = {
	{
		.name		= "vlan_aware",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "p0_enable",
		.shift		= 2,
		.bits		= 1,
	},
	{
		.name		= "p0_pass_pri_tagged",
		.shift		= 3,
		.bits		= 1,
	},
	{
		.name		= "p1_pass_pri_tagged",
		.shift		= 4,
		.bits		= 1,
	},
	{
		.name		= "p2_pass_pri_tagged",
		.shift		= 5,
		.bits		= 1,
	},
	{
		.name		= "p3_pass_pri_tagged",
		.shift		= 6,
		.bits		= 1,
	},
	{
		.name		= "p4_pass_pri_tagged",
		.shift		= 7,
		.bits		= 1,
	},
	{
		.name		= "p5_pass_pri_tagged",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "p6_pass_pri_tagged",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "p7_pass_pri_tagged",
		.shift		= 10,
		.bits		= 1,
	},
	{
		.name		= "p8_pass_pri_tagged",
		.shift		= 11,
		.bits		= 1,
	},
	{
		.name		= "p0_tx_crc_type",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "p0_tx_crc_remove",
		.shift		= 13,
		.bits		= 1,
	},
	{
		.name		= "p0_rx_pad",
		.shift		= 14,
		.bits		= 1,
	},
	{
		.name		= "p0_rx_pass_crc_err",
		.shift		= 15,
		.bits		= 1,
	},
};

static ssize_t cpsw2_control_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = readl(&cpsw_dev->regs->control);
	return cpsw2_attr_info_show(attr->info, attr->info_size, reg, buf);
}

static ssize_t cpsw2_control_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpsw2_mod_info *i;
	struct cpsw2_parse_result res;
	void __iomem *r = NULL;
	int ret;


	ret = cpsw2_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);
	r = &cpsw_dev->regs->control;

	cpsw2_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpsw2_attribute cpsw_control_attribute =
	__CPSW2_ATTR(control, S_IRUGO | S_IWUSR,
		cpsw2_control_show, cpsw2_control_store, cpsw_controls);

static const struct cpsw2_mod_info cpsw_ptypes[] = {
	{
		.name		= "escalate_pri_load_val",
		.shift		= 0,
		.bits		= 5,
	},
	{
		.name		= "port0_pri_type_escalate",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "port1_pri_type_escalate",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "port2_pri_type_escalate",
		.shift		= 10,
		.bits		= 1,
	},
	{
		.name		= "port3_pri_type_escalate",
		.shift		= 11,
		.bits		= 1,
	},
	{
		.name		= "port4_pri_type_escalate",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "port5_pri_type_escalate",
		.shift		= 13,
		.bits		= 1,
	},
	{
		.name		= "port6_pri_type_escalate",
		.shift		= 14,
		.bits		= 1,
	},
	{
		.name		= "port7_pri_type_escalate",
		.shift		= 15,
		.bits		= 1,
	},
	{
		.name		= "port8_pri_type_escalate",
		.shift		= 16,
		.bits		= 1,
	},
};

static ssize_t cpsw2_pri_type_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = readl(&cpsw_dev->regs->ptype);

	return cpsw2_attr_info_show(attr->info, attr->info_size, reg, buf);
}

static ssize_t cpsw2_pri_type_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpsw2_mod_info *i;
	struct cpsw2_parse_result res;
	void __iomem *r = NULL;
	int ret;


	ret = cpsw2_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);
	r = &cpsw_dev->regs->ptype;

	cpsw2_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpsw2_attribute cpsw_pri_type_attribute =
	__CPSW2_ATTR(priority_type, S_IRUGO | S_IWUSR,
			cpsw2_pri_type_show,
			cpsw2_pri_type_store,
			cpsw_ptypes);

static const struct cpsw2_mod_info cpsw_port_tx_pri_maps[] = {
	{
		.name		= "port_tx_pri_0",
		.shift		= 0,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_1",
		.shift		= 4,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_2",
		.shift		= 8,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_3",
		.shift		= 12,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_4",
		.shift		= 16,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_5",
		.shift		= 20,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_6",
		.shift		= 24,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_7",
		.shift		= 28,
		.bits		= 3,
	},
};

static ssize_t cpsw2_port_tx_pri_map_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_attribute *attr,
		     char *buf)
{
	int idx, len = 0, total_len = 0, port;
	struct cpsw2_intf *cpsw_intf;
	struct cpsw2_slave *slave;
	u32 reg;

	port = (int)(attr->context);

	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num != port)
				continue;
			reg = readl(&slave->regs->tx_pri_map);
			len = cpsw2_attr_info_show(attr->info, attr->info_size,
						reg, buf+total_len);
			total_len += len;
		} else {
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num != port)
					continue;
				reg = readl(&slave->regs->tx_pri_map);
				len = cpsw2_attr_info_show(attr->info,
					attr->info_size, reg, buf+total_len);
				total_len += len;
			}
		}
	}
	return total_len;
}

static ssize_t cpsw2_port_tx_pri_map_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpsw2_mod_info *i;
	struct cpsw2_parse_result res;
	struct cpsw2_intf *cpsw_intf;
	struct cpsw2_slave *slave;
	void __iomem *r = NULL;
	int ret, idx, port;

	port = (int)(attr->context);

	ret = cpsw2_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);

	/* Slave port */
	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num == port) {
				r = &slave->regs->tx_pri_map;
				goto set;
			}
		} else
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num == port) {
					r = &slave->regs->tx_pri_map;
					goto set;
				}
			}
	}

	if (!r)
		return  -ENOENT;

set:
	cpsw2_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpsw2_attribute cpsw_tx_pri_1_attribute =
	__CPSW2_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)1);

static struct cpsw2_attribute cpsw_tx_pri_2_attribute =
	__CPSW2_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)2);

static struct cpsw2_attribute cpsw_tx_pri_3_attribute =
	__CPSW2_CTXT_ATTR(3, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)3);

static struct cpsw2_attribute cpsw_tx_pri_4_attribute =
	__CPSW2_CTXT_ATTR(4, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)4);
static struct cpsw2_attribute cpsw_tx_pri_5_attribute =
	__CPSW2_CTXT_ATTR(5, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)5);

static struct cpsw2_attribute cpsw_tx_pri_6_attribute =
	__CPSW2_CTXT_ATTR(6, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)6);

static struct cpsw2_attribute cpsw_tx_pri_7_attribute =
	__CPSW2_CTXT_ATTR(7, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)7);

static struct cpsw2_attribute cpsw_tx_pri_8_attribute =
	__CPSW2_CTXT_ATTR(8, S_IRUGO | S_IWUSR,
			cpsw2_port_tx_pri_map_show,
			cpsw2_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)8);

static struct attribute *cpsw_tx_pri_default_attrs[] = {
	&cpsw_tx_pri_1_attribute.attr,
	&cpsw_tx_pri_2_attribute.attr,
	&cpsw_tx_pri_3_attribute.attr,
	&cpsw_tx_pri_4_attribute.attr,
	&cpsw_tx_pri_5_attribute.attr,
	&cpsw_tx_pri_6_attribute.attr,
	&cpsw_tx_pri_7_attribute.attr,
	&cpsw_tx_pri_8_attribute.attr,
	NULL
};

static ssize_t cpsw2_tx_pri_attr_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = tx_pri_to_cpsw2_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw2_tx_pri_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = tx_pri_to_cpsw2_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_tx_pri_sysfs_ops = {
	.show = cpsw2_tx_pri_attr_show,
	.store = cpsw2_tx_pri_attr_store,
};

static struct kobj_type cpsw_tx_pri_ktype = {
	.sysfs_ops = &cpsw_tx_pri_sysfs_ops,
	.default_attrs = cpsw_tx_pri_default_attrs,
};

static const struct cpsw2_mod_info cpsw_port_vlans[] = {
	{
		.name		= "port_vlan_id",
		.shift		= 0,
		.bits		= 12,
	},
	{
		.name		= "port_cfi",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "port_vlan_pri",
		.shift		= 13,
		.bits		= 3,
	},
};

static ssize_t cpsw2_port_vlan_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_attribute *attr,
		     char *buf)
{
	int idx, len = 0, total_len = 0, port;
	struct cpsw2_intf *cpsw_intf;
	struct cpsw2_slave *slave;
	u32 reg;

	port = (int)(attr->context);

	if (port == cpsw_dev->host_port) {
		/* Host port */
		reg = readl(&cpsw_dev->host_port_regs->port_vlan);
		len = cpsw2_attr_info_show(attr->info, attr->info_size,
					reg, buf);
		return len;
	}

	/* Slave ports */
	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num != port)
				continue;
			reg = readl(&slave->regs->port_vlan);
			len = cpsw2_attr_info_show(attr->info, attr->info_size,
					reg, buf+total_len);
			total_len += len;
		} else {
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num != port)
					continue;
				reg = readl(&slave->regs->port_vlan);
				len = cpsw2_attr_info_show(attr->info,
					attr->info_size, reg, buf+total_len);
				total_len += len;
			}
		}
	}
	return total_len;
}

static ssize_t cpsw2_port_vlan_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpsw2_mod_info *i;
	struct cpsw2_parse_result res;
	struct cpsw2_intf *cpsw_intf;
	struct cpsw2_slave *slave;
	void __iomem *r = NULL;
	int ret, idx, port;

	port = (int)(attr->context);

	ret = cpsw2_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);

	/* Host port */
	if (port == cpsw_dev->host_port) {
		r = &cpsw_dev->host_port_regs->port_vlan;
		goto set;
	}

	/* Slave port */
	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num == port) {
				r = &slave->regs->port_vlan;
				goto set;
			}
		} else
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num == port) {
					r = &slave->regs->port_vlan;
					goto set;
				}
			}
	}

	if (!r)
		return  -ENOENT;

set:
	cpsw2_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpsw2_attribute cpsw_pvlan_0_attribute =
	__CPSW2_CTXT_ATTR(0, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)0);

static struct cpsw2_attribute cpsw_pvlan_1_attribute =
	__CPSW2_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)1);

static struct cpsw2_attribute cpsw_pvlan_2_attribute =
	__CPSW2_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)2);

static struct cpsw2_attribute cpsw_pvlan_3_attribute =
	__CPSW2_CTXT_ATTR(3, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)3);

static struct cpsw2_attribute cpsw_pvlan_4_attribute =
	__CPSW2_CTXT_ATTR(4, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)4);
static struct cpsw2_attribute cpsw_pvlan_5_attribute =
	__CPSW2_CTXT_ATTR(5, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)5);

static struct cpsw2_attribute cpsw_pvlan_6_attribute =
	__CPSW2_CTXT_ATTR(6, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)6);

static struct cpsw2_attribute cpsw_pvlan_7_attribute =
	__CPSW2_CTXT_ATTR(7, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)7);

static struct cpsw2_attribute cpsw_pvlan_8_attribute =
	__CPSW2_CTXT_ATTR(8, S_IRUGO | S_IWUSR,
			cpsw2_port_vlan_show,
			cpsw2_port_vlan_store,
			cpsw_port_vlans, (void *)8);

static struct attribute *cpsw_pvlan_default_attrs[] = {
	&cpsw_pvlan_0_attribute.attr,
	&cpsw_pvlan_1_attribute.attr,
	&cpsw_pvlan_2_attribute.attr,
	&cpsw_pvlan_3_attribute.attr,
	&cpsw_pvlan_4_attribute.attr,
	&cpsw_pvlan_5_attribute.attr,
	&cpsw_pvlan_6_attribute.attr,
	&cpsw_pvlan_7_attribute.attr,
	&cpsw_pvlan_8_attribute.attr,
	NULL
};

static ssize_t cpsw2_pvlan_attr_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = pvlan_to_cpsw2_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw2_pvlan_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = pvlan_to_cpsw2_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_pvlan_sysfs_ops = {
	.show = cpsw2_pvlan_attr_show,
	.store = cpsw2_pvlan_attr_store,
};

static struct kobj_type cpsw_pvlan_ktype = {
	.sysfs_ops = &cpsw_pvlan_sysfs_ops,
	.default_attrs = cpsw_pvlan_default_attrs,
};

struct cpsw2_ts_attribute {
	struct attribute attr;
	ssize_t (*show)(struct cpsw2_priv *cpsw_dev,
		struct cpsw2_ts_attribute *attr, char *buf, void *);
	ssize_t	(*store)(struct cpsw2_priv *cpsw_dev,
		struct cpsw2_ts_attribute *attr, const char *, size_t, void *);
};
#define to_cpsw2_ts_attr(_attr) \
	container_of(_attr, struct cpsw2_ts_attribute, attr)

#define __CPSW2_TS_ATTR(_name, _mode, _show, _store)		\
	{ \
		.attr = {.name = __stringify(_name), .mode = _mode },	\
		.show	= _show,		\
		.store	= _store,		\
	}

#define pts_to_cpsw2_dev(obj) container_of(obj, struct cpsw2_priv, pts_kobj)

#define pts_n_to_cpsw2_dev(obj, n) \
	container_of(obj, struct cpsw2_priv, port_ts_kobj[n])

static struct cpsw2_slave *cpsw2_port_num_get_slave(struct cpsw2_priv *cpsw_dev,
						  int port)
{
	struct cpsw2_intf *cpsw_intf;
	struct cpsw2_slave *slave = NULL;
	int idx;

	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num == port)
				return slave;
		} else {
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num == port)
					return slave;
			}
		}
	}
	return NULL;
}

static ssize_t cpsw2_port_ts_uni_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_ts_attribute *attr,
		     char *buf, void *context)
{
	struct cpsw2_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	len = snprintf(buf, PAGE_SIZE, "%lu\n",
		((reg & CPSW2_TS_UNI_EN) >> CPSW2_TS_UNI_EN_SHIFT));

	return len;
}

static ssize_t cpsw2_port_ts_uni_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_ts_attribute *attr,
			      const char *buf, size_t count, void *context)
{
	struct cpsw2_slave *slave;
	int port, val;
	u32 reg, mode;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	if (kstrtoint(buf, 0, &val) < 0)
		return -EINVAL;


	if (val)
		mode = CPSW2_TS_UNI_EN;
	else
		mode = (slave->ts_ctl.maddr_map << CPSW2_TS_CTL_MADDR_SHIFT);

	reg = readl(&slave->regs->ts_ctl_ltype2);
	reg &= ~(CPSW2_TS_UNI_EN | CPSW2_TS_CTL_MADDR_ALL);
	reg |= mode;
	writel(reg, &slave->regs->ts_ctl_ltype2);

	slave->ts_ctl.uni = (val ? 1 : 0);
	return count;
}

static struct cpsw2_ts_attribute cpsw_pts_uni_attribute =
	__CPSW2_TS_ATTR(uni_en, S_IRUGO | S_IWUSR,
			cpsw2_port_ts_uni_show,
			cpsw2_port_ts_uni_store);

static ssize_t cpsw2_port_ts_maddr_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_ts_attribute *attr, char *buf, void *context)
{
	struct cpsw2_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	len = snprintf(buf, PAGE_SIZE, "%02x\n",
		(reg >> CPSW2_TS_CTL_MADDR_SHIFT) & 0x1f);
	return len;
}

static ssize_t cpsw2_port_ts_maddr_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_ts_attribute *attr,
			      const char *buf, size_t count, void *context)
{
	struct cpsw2_slave *slave;
	int port;
	u32 reg;
	u8 val;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	if (kstrtou8(buf, 0, &val) < 0)
		return -EINVAL;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	reg &= ~CPSW2_TS_CTL_MADDR_ALL;
	reg |= ((val & 0x1f) << CPSW2_TS_CTL_MADDR_SHIFT);
	writel(reg, &slave->regs->ts_ctl_ltype2);

	slave->ts_ctl.maddr_map = val & 0x1f;
	return count;
}

static struct cpsw2_ts_attribute cpsw_pts_maddr_attribute =
	__CPSW2_TS_ATTR(mcast_addr, S_IRUGO | S_IWUSR,
			cpsw2_port_ts_maddr_show,
			cpsw2_port_ts_maddr_store);

static ssize_t cpsw2_port_ts_dst_port_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_ts_attribute *attr, char *buf, void *context)
{
	struct cpsw2_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	len = snprintf(buf, PAGE_SIZE, "%01x\n",
		(reg >> CPSW2_TS_CTL_DST_PORT_SHIFT) & 0x3);
	return len;
}

static ssize_t cpsw2_port_ts_dst_port_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_ts_attribute *attr,
			      const char *buf, size_t count, void *context)
{
	struct cpsw2_slave *slave;
	int port;
	u32 reg;
	u8 val;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	if (kstrtou8(buf, 0, &val) < 0)
		return -EINVAL;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	reg &= ~CPSW2_TS_CTL_DST_PORT;
	reg |= ((val & 0x3) << CPSW2_TS_CTL_DST_PORT_SHIFT);
	writel(reg, &slave->regs->ts_ctl_ltype2);

	slave->ts_ctl.dst_port_map = val & 0x3;
	return count;
}

static struct cpsw2_ts_attribute cpsw_pts_dst_port_attribute =
	__CPSW2_TS_ATTR(dst_port, S_IRUGO | S_IWUSR,
			cpsw2_port_ts_dst_port_show,
			cpsw2_port_ts_dst_port_store);

static ssize_t cpsw2_port_ts_config_show(struct cpsw2_priv *cpsw_dev,
		     struct cpsw2_ts_attribute *attr, char *buf, void *context)
{
	struct cpsw2_slave *slave;
	int len, port, total_len = 0;
	u32 reg;
	char *p = buf;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	reg = readl(&slave->regs->ts_ctl);
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(&slave->regs->ts_seq_ltype);
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(&slave->regs->ts_vlan_ltype);
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(&slave->regs->ts_ctl_ltype2);
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(&slave->regs->ts_ctl2);
	len = snprintf(p, PAGE_SIZE, "%08x\n", reg);
	p += len;
	total_len += len;

	return total_len;
}

static ssize_t cpsw2_port_ts_config_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_ts_attribute *attr,
			      const char *buf, size_t count, void *context)
{
	struct cpsw2_slave *slave;
	unsigned long reg, val;
	int len, port;
	char tmp_str[4];
	u8 reg_num = 0;
	u32 __iomem *p;

	port = (int)context;

	slave = cpsw2_port_num_get_slave(cpsw_dev, port);
	if (!slave)
		return 0;

	len = strcspn(buf, " ");
	if (len > 1)
		return -ENOMEM;

	strncpy(tmp_str, buf, len);
	tmp_str[len] = '\0';
	if (kstrtou8(tmp_str, 0, &reg_num))
		return -EINVAL;

	buf += (len + 1);
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	switch (reg_num) {
	case 1:
		p = &slave->regs->ts_ctl;
		break;
	case 2:
		p = &slave->regs->ts_seq_ltype;
		break;
	case 3:
		p = &slave->regs->ts_vlan_ltype;
		break;
	case 4:
		p = &slave->regs->ts_ctl_ltype2;
		break;
	case 5:
		p = &slave->regs->ts_ctl2;
		break;
	default:
		return -EINVAL;
	}

	reg = readl(p);
	if (reg != val)
		writel(val, p);

	return count;
}

static struct cpsw2_ts_attribute cpsw_pts_config_attribute =
	__CPSW2_TS_ATTR(config, S_IRUGO | S_IWUSR,
			cpsw2_port_ts_config_show,
			cpsw2_port_ts_config_store);

static struct attribute *cpsw_pts_n_default_attrs[] = {
	&cpsw_pts_uni_attribute.attr,
	&cpsw_pts_maddr_attribute.attr,
	&cpsw_pts_dst_port_attribute.attr,
	&cpsw_pts_config_attribute.attr,
	NULL
};

static struct cpsw2_priv *cpsw2_port_ts_kobj_to_priv(struct kobject *kobj,
						   int *port)
{
	char *port_name[] = {"1", "2", "3", "4", "5", "6", "7", "8", NULL};
	struct cpsw2_priv *cpsw_dev;
	struct kobject *kobj_0;
	int i = 0;

	*port = -1;

	while (i < MAX_SLAVES && port_name[i]) {
		if (strncmp(port_name[i], kobject_name(kobj), 1) == 0)
			*port = i+1;
		i++;
	}

	if (*port < 0)
		return NULL;

	kobj_0 = kobj - (*port - 1);
	cpsw_dev = pts_n_to_cpsw2_dev(kobj_0, 0);
	return cpsw_dev;
}

static ssize_t cpsw2_pts_n_attr_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpsw2_ts_attribute *attribute = to_cpsw2_ts_attr(attr);
	struct cpsw2_priv *cpsw_dev;
	int port = -1;

	if (!attribute->show)
		return -EIO;

	cpsw_dev = cpsw2_port_ts_kobj_to_priv(kobj, &port);
	if (!cpsw_dev)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf, (void *)port);
}

static ssize_t cpsw2_pts_n_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpsw2_ts_attribute *attribute = to_cpsw2_ts_attr(attr);
	struct cpsw2_priv *cpsw_dev;
	int port = -1;

	if (!attribute->store)
		return -EIO;

	cpsw_dev = cpsw2_port_ts_kobj_to_priv(kobj, &port);
	if (!cpsw_dev)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count, (void *)port);
}

static const struct sysfs_ops cpsw_pts_n_sysfs_ops = {
	.show = cpsw2_pts_n_attr_show,
	.store = cpsw2_pts_n_attr_store,
};

static struct kobj_type cpsw_pts_n_ktype = {
	.sysfs_ops = &cpsw_pts_n_sysfs_ops,
	.default_attrs = cpsw_pts_n_default_attrs,
};

static void cpsw2_reset_mod_stats(struct cpsw2_priv *cpsw_dev, int stat_mod)
{
	void __iomem *base = cpsw_dev->hw_stats_regs[stat_mod];
	u32  __iomem *p;
	int i;

	for (i = 0; i < ETHTOOL_STATS_NUM(cpsw_dev->num_slaves + 1); i++) {
		if (et_stats[i].type == stat_mod) {
			cpsw_dev->hw_stats[i] = 0;
			p = base + et_stats[i].offset;
			cpsw_dev->hw_stats_prev[i] = readl_relaxed(p);
		}
	}
	return;
}

static ssize_t cpsw2_stats_mod_store(struct cpsw2_priv *cpsw_dev,
			      struct cpsw2_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long end;
	int stat_mod;

	if (kstrtoul(buf, 0, &end) != 0 || (end != 0))
		return -EINVAL;

	stat_mod = (int)(attr->context);
	spin_lock_bh(&cpsw_dev->hw_stats_lock);
	cpsw2_reset_mod_stats(cpsw_dev, stat_mod);
	spin_unlock_bh(&cpsw_dev->hw_stats_lock);
	return count;
}

static struct cpsw2_attribute cpsw_stats_0_attribute =
	__CPSW2_ATTR_FULL(0, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS0_MODULE);

static struct cpsw2_attribute cpsw_stats_1_attribute =
	__CPSW2_ATTR_FULL(1, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS1_MODULE);

static struct cpsw2_attribute cpsw_stats_2_attribute =
	__CPSW2_ATTR_FULL(2, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS2_MODULE);

static struct cpsw2_attribute cpsw_stats_3_attribute =
	__CPSW2_ATTR_FULL(3, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS3_MODULE);

static struct cpsw2_attribute cpsw_stats_4_attribute =
	__CPSW2_ATTR_FULL(4, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS4_MODULE);

static struct cpsw2_attribute cpsw_stats_5_attribute =
	__CPSW2_ATTR_FULL(5, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS5_MODULE);

static struct cpsw2_attribute cpsw_stats_6_attribute =
	__CPSW2_ATTR_FULL(6, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS6_MODULE);

static struct cpsw2_attribute cpsw_stats_7_attribute =
	__CPSW2_ATTR_FULL(7, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS7_MODULE);

static struct cpsw2_attribute cpsw_stats_8_attribute =
	__CPSW2_ATTR_FULL(8, S_IWUSR, NULL, cpsw2_stats_mod_store,
			NULL, 0, (void *)CPSW2_STATS8_MODULE);

static struct attribute *cpsw_stats_default_attrs[] = {
	&cpsw_stats_0_attribute.attr,
	&cpsw_stats_1_attribute.attr,
	&cpsw_stats_2_attribute.attr,
	&cpsw_stats_3_attribute.attr,
	&cpsw_stats_4_attribute.attr,
	&cpsw_stats_5_attribute.attr,
	&cpsw_stats_6_attribute.attr,
	&cpsw_stats_7_attribute.attr,
	&cpsw_stats_8_attribute.attr,
	NULL
};

static ssize_t cpsw2_stats_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = stats_to_cpsw2_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_stats_sysfs_ops = {
	.store = cpsw2_stats_attr_store,
};

static struct kobj_type cpsw_stats_ktype = {
	.sysfs_ops = &cpsw_stats_sysfs_ops,
	.default_attrs = cpsw_stats_default_attrs,
};

static struct attribute *cpsw_default_attrs[] = {
	&cpsw_version_attribute.attr,
	&cpsw_control_attribute.attr,
	&cpsw_pri_type_attribute.attr,
	NULL
};

static ssize_t cpsw2_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = to_cpsw2_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw2_attr_store(struct kobject *kobj, struct attribute *attr,
				   const char *buf, size_t count)
{
	struct cpsw2_attribute *attribute = to_cpsw2_attr(attr);
	struct cpsw2_priv *cpsw_dev = to_cpsw2_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_sysfs_ops = {
	.show = cpsw2_attr_show,
	.store = cpsw2_attr_store,
};

static struct kobj_type cpsw_ktype = {
	.sysfs_ops = &cpsw_sysfs_ops,
	.default_attrs = cpsw_default_attrs,
};

static void keystone_get_drvinfo2(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strncpy(info->driver, NETCP2_DRIVER_NAME, sizeof(info->driver));
	strncpy(info->version, NETCP2_DRIVER_VERSION, sizeof(info->version));
}

static u32 keystone_get_msglevel2(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	return netcp->msg_enable;
}

static void keystone_set_msglevel2(struct net_device *ndev, u32 value)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	netcp->msg_enable = value;
}

static void keystone_get_stat_strings2(struct net_device *ndev,
				   uint32_t stringset, uint8_t *data)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_device *netcp_device = netcp->netcp_device;
	struct cpsw2_priv *priv;
	int i;

	/* find the instance of the module registered to the netcp_device */
	priv = (struct cpsw2_priv *)netcp_device_find_module(netcp_device,
							CPSW2_MODULE_NAME);

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHTOOL_STATS_NUM(priv->num_slaves + 1); i++) {
			memcpy(data, et_stats[i].desc, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		break;
	}
}

static int keystone_get_sset_count2(struct net_device *ndev, int stringset)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_device *netcp_device = netcp->netcp_device;
	struct cpsw2_priv *priv;

	/* find the instance of the module registered to the netcp_device */
	priv = (struct cpsw2_priv *)netcp_device_find_module(netcp_device,
							CPSW2_MODULE_NAME);
	switch (stringset) {
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS:
		return ETHTOOL_STATS_NUM(priv->num_slaves + 1);
	default:
		return -EINVAL;
	}
}

static void cpsw2_update_stats(struct cpsw2_priv *cpsw_dev, uint64_t *data)
{
	void __iomem *base = NULL;
	u32  __iomem *p;
	u32 curr, delta;
	int i;

	for (i = 0; i < ETHTOOL_STATS_NUM(cpsw_dev->num_slaves + 1); i++) {
		base = cpsw_dev->hw_stats_regs[et_stats[i].type];
		p = base + et_stats[i].offset;
		curr = readl_relaxed(p);
		delta = curr - cpsw_dev->hw_stats_prev[i];
		cpsw_dev->hw_stats_prev[i] = curr;
		cpsw_dev->hw_stats[i] += delta;

		if (data)
			data[i] = cpsw_dev->hw_stats[i];
	}

	return;
}

static void keystone_get_ethtool_stats2(struct net_device *ndev,
				       struct ethtool_stats *stats,
				       uint64_t *data)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_device *netcp_device = netcp->netcp_device;
	struct cpsw2_priv *priv;

	/* find the instance of the module registered to the netcp_device */
	priv = (struct cpsw2_priv *)netcp_device_find_module(netcp_device,
							CPSW2_MODULE_NAME);
	if (priv) {
		spin_lock_bh(&priv->hw_stats_lock);
		cpsw2_update_stats(priv, data);
		spin_unlock_bh(&priv->hw_stats_lock);
	}

	return;
}

static int keystone_get_link_ksettings2(struct net_device *ndev,
				        struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phy = ndev->phydev;
	struct cpsw2_slave *slave;

	if (!phy)
		return -EINVAL;

	slave = (struct cpsw2_slave *)phy->priv;
	if (!slave)
		return -EINVAL;

	phy_ethtool_ksettings_get(phy, cmd);
	cmd->base.port = slave->phy_port_t;

	return 0;
}

static int keystone_set_link_ksettings2(struct net_device *ndev,
				        const struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phy = ndev->phydev;
	struct cpsw2_slave *slave;
	u8 port = cmd->base.port;
	u32 advertising, supported;
	u32 features;

	ethtool_convert_link_mode_to_legacy_u32(&advertising,
						cmd->link_modes.advertising);
	ethtool_convert_link_mode_to_legacy_u32(&supported,
						cmd->link_modes.supported);
	features = advertising & supported;

	if (!phy)
		return -EINVAL;

	slave = (struct cpsw2_slave *)phy->priv;
	if (!slave)
		return -EINVAL;

	if (port != slave->phy_port_t) {
		if ((port == PORT_TP) && !(features & ADVERTISED_TP))
			return -EINVAL;

		if ((port == PORT_AUI) && !(features & ADVERTISED_AUI))
			return -EINVAL;

		if ((port == PORT_BNC) && !(features & ADVERTISED_BNC))
			return -EINVAL;

		if ((port == PORT_MII) && !(features & ADVERTISED_MII))
			return -EINVAL;

		if ((port == PORT_FIBRE) && !(features & ADVERTISED_FIBRE))
			return -EINVAL;
	}

	slave->phy_port_t = port;
	return phy_ethtool_ksettings_set(phy, cmd);
}

#ifdef CONFIG_TI_CPTS
static int keystone_get_ts_info2(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_device *netcp_device = netcp->netcp_device;
	struct cpsw2_priv *priv;

	/* find the instance of the module registered to the netcp_device */
	priv = netcp_device_find_module(netcp_device, CPSW2_MODULE_NAME);
	if (!priv)
		return -EINVAL;

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->phc_index = priv->cpts.phc_index;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT);
	return 0;
}
#else
static int keystone_get_ts_info2(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE;
	info->phc_index = -1;
	info->tx_types = 0;
	info->rx_filters = 0;
	return 0;
}
#endif

static const struct ethtool_ops keystone_ethtool_ops = {
	.get_drvinfo		= keystone_get_drvinfo2,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= keystone_get_msglevel2,
	.set_msglevel		= keystone_set_msglevel2,
	.get_strings		= keystone_get_stat_strings2,
	.get_sset_count		= keystone_get_sset_count2,
	.get_ethtool_stats	= keystone_get_ethtool_stats2,
	.get_ts_info		= keystone_get_ts_info2,
	.get_link_ksettings	= keystone_get_link_ksettings2,
	.set_link_ksettings	= keystone_set_link_ksettings2,
};

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw2_set_slave_mac(struct cpsw2_slave *slave,
			       struct cpsw2_intf *cpsw_intf)
{
	struct net_device *ndev = cpsw_intf->ndev;

	writel(mac_hi(ndev->dev_addr), &slave->regs->sa_hi);
	writel(mac_lo(ndev->dev_addr), &slave->regs->sa_lo);
}

static inline int cpsw2_get_slave_port(struct cpsw2_priv *priv, u32 slave_num)
{
	if (priv->host_port == 0)
		return slave_num + 1;
	else
		return slave_num;
}

static void _cpsw2_adjust_link(struct cpsw2_slave *slave, bool *link)
{
	struct phy_device *phy = slave->phy;
	u32 mac_control = 0;
	u32 slave_port;

	if (!phy)
		return;

	slave_port = slave->port_num;

	if (phy->link) {
		mac_control = slave->mac_control;
		mac_control |= MACSL_DEFAULT_CONFIG;
		/* enable forwarding */
		cpsw_ale_control_set(slave->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

		if (phy->duplex)
			mac_control |= BIT(0);	/* FULLDUPLEXEN	*/
		else
			mac_control &= ~0x1;

		*link = true;
	} else {
		mac_control = 0;
		/* disable forwarding */
		cpsw_ale_control_set(slave->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);
	}

	if (mac_control != slave->mac_control) {
		phy_print_status(phy);
		writel(mac_control, &slave->regs->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw2_adjust_link(struct net_device *n_dev)
{
	struct cpsw2_slave *slave = NULL;
	struct netcp_priv *netcp = netdev_priv(n_dev);
	bool link = false;

	if (n_dev->phydev)
		slave = n_dev->phydev->priv;
	if (!slave)
		return;

	_cpsw2_adjust_link(slave, &link);

	if (link)
		netcp->phy_link_state_mask |= BIT(slave->slave_num);
	else
		netcp->phy_link_state_mask &= ~BIT(slave->slave_num);
}

/*
 * Reset the the mac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
static int cpsw2_port_reset(struct cpsw2_slave *slave)
{
	u32 i, v;

	/* Set the soft reset bit */
	writel(SOFT_RESET, &slave->regs->soft_reset);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = readl(&slave->regs->soft_reset);
		if ((v & SOFT_RESET_MASK) != SOFT_RESET)
			return 0;
	}

	/* Timeout on the reset */
	return GMACSL_RET_WARN_RESET_INCOMPLETE;
}

/*
 * Configure the mac sliver
 */
static void cpsw2_port_config(struct cpsw2_slave *slave, int max_rx_len)
{
	if (max_rx_len > MAX_SIZE_STREAM_BUFFER)
		max_rx_len = MAX_SIZE_STREAM_BUFFER;

	slave->mac_control = MACSL_DEFAULT_CONFIG;

	writel(max_rx_len, &slave->regs->rx_maxlen);
	writel(slave->mac_control, &slave->regs->mac_control);
}

static void cpsw2_slave_stop(struct cpsw2_slave *slave,
			    struct cpsw2_intf *cpsw_intf)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	/* disable forwarding */
	if (slave->ale)
		cpsw_ale_control_set(slave->ale, slave->port_num,
				ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);

	keystone_sgmii_rtreset(SGMII2_BASE(slave->slave_num),
				slave->slave_num, true);

	cpsw2_port_reset(slave);

	if (!slave->phy)
		return;

	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

static void cpsw2_slave_link(struct cpsw2_slave *slave,
			    struct cpsw2_intf *cpsw_intf)
{
	struct netcp_priv *netcp = netdev_priv(cpsw_intf->ndev);
	int sn = slave->slave_num;

	if (IS_SGMII_MAC_PHY(slave->link_interface)) {
		/* check only the bit in phy_link_state_mask
		 * that corresponds to the slave
		 */
		if (!(netcp->phy_link_state_mask & BIT(sn)))
			cpsw_intf->link_state &= ~BIT(sn);
	}
}

static void cpsw2_slave_open(struct cpsw2_slave *slave,
			    struct cpsw2_intf *cpsw_intf)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	u32 slave_port;

	keystone_sgmii_reset(SGMII2_BASE(slave->slave_num), slave->slave_num);

	keystone_sgmii_config(SGMII2_BASE(slave->slave_num), slave->slave_num,
				slave->link_interface);

	cpsw2_port_reset(slave);

	keystone_sgmii_rtreset(SGMII2_BASE(slave->slave_num),
					slave->slave_num, false);

	cpsw2_port_config(slave, cpsw_dev->rx_packet_max);

	cpsw2_set_slave_mac(slave, cpsw_intf);

	/* this slave port here is 1 based */
	slave_port = cpsw2_get_slave_port(cpsw_dev, slave->slave_num);

	/* hence port num here is also 1 based */
	slave->port_num = slave_port;
	slave->ale = cpsw_dev->ale;

	/* enable forwarding */
	cpsw_ale_control_set(cpsw_dev->ale, slave_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_mcast(cpsw_dev->ale, cpsw_intf->ndev->broadcast,
			   1 << slave_port, 0, 0, ALE_MCAST_FWD_2);

	if (IS_SGMII_MAC_PHY(slave->link_interface)) {
		slave->phy = of_phy_connect(cpsw_intf->ndev,
					    cpsw_intf->phy_node,
					    &cpsw2_adjust_link, 0,
					    PHY_INTERFACE_MODE_SGMII);
		if (IS_ERR_OR_NULL(slave->phy)) {
			dev_err(cpsw_dev->dev, "phy not found on slave %d\n",
				slave->slave_num);
			slave->phy = NULL;
		} else {
			dev_info(cpsw_dev->dev, "phy found: id is: 0x%s\n",
				 phydev_name(slave->phy));
			cpsw_intf->ndev->phydev = slave->phy;
			slave->phy_port_t = PORT_MII;
			phy_start(slave->phy);
		}
	}
}

static int cpsw2_init_ale(struct cpsw2_priv *cpsw_dev)
{
	struct cpsw_ale_params ale_params;

	memset(&ale_params, 0, sizeof(ale_params));

	ale_params.dev			= cpsw_dev->dev;
	ale_params.ale_regs		= (void *)((u32)cpsw_dev->ale_reg);
	ale_params.ale_ageout		= cpsw_dev->ale_ageout;
	ale_params.ale_entries		= cpsw_dev->ale_entries;
	ale_params.ale_ports		= cpsw_dev->ale_ports;

	cpsw_dev->ale = cpsw_ale_create(&ale_params);
	if (!cpsw_dev->ale) {
		dev_err(cpsw_dev->dev,
			"error initializing ale engine\n");
		return -ENODEV;
	}

	dev_info(cpsw_dev->dev, "Created a cpsw ale engine\n");

	cpsw_ale_start(cpsw_dev->ale);

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_BYPASS, cpsw_dev->multi_if ? 1 : 0);

	cpsw_ale_control_set(cpsw_dev->ale, 0, ALE_NO_PORT_VLAN, 1);

	cpsw_ale_control_set(cpsw_dev->ale, cpsw_dev->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

#if 0
	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_VLAN_MEMBER_1R4,
			     CPSW2_MASK_ALL_PORTS(cpsw_dev->num_slaves + 1));

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_MCAST_FLOOD_1R4,
			     CPSW2_MASK_PHYS_PORTS(cpsw_dev->num_slaves + 1));

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_REG_MCAST_FLOOD_1R4,
			     CPSW2_MASK_ALL_PORTS(cpsw_dev->num_slaves + 1));

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNTAGGED_EGRESS_1R4,
			     CPSW2_MASK_ALL_PORTS(cpsw_dev->num_slaves + 1));
#endif
	return 0;
}

static void cpsw2_init_host_port(struct cpsw2_priv *priv)
{
	/* Max length register */
	writel(MAX_SIZE_STREAM_BUFFER,
		     &priv->host_port_regs->rx_maxlen);

}

static void cpsw2_slave_init(struct cpsw2_slave *slave, struct cpsw2_priv *priv)
{
	void __iomem		*regs = priv->ss_regs;
	int			slave_num = slave->slave_num;

	slave->regs = regs + priv->slave_reg_ofs +
			(CPSW2_SLAVE_REGS_SIZE * slave_num);
}

static void cpsw2_add_mcast_addr(struct cpsw2_intf *cpsw_intf, u8 *addr)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_add_mcast(cpsw_dev->ale, addr,
			   CPSW2_MASK_ALL_PORTS(cpsw_dev->num_slaves + 1), 0, 0,
			   ALE_MCAST_FWD_2);
}

static void cpsw2_add_ucast_addr(struct cpsw2_intf *cpsw_intf, u8 *addr)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_add_ucast(cpsw_dev->ale, addr, cpsw_dev->host_port, 0, 0);
}

static void cpsw2_del_mcast_addr(struct cpsw2_intf *cpsw_intf, u8 *addr)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_del_mcast(cpsw_dev->ale, addr, 0, 0, 0);
}

static void cpsw2_del_ucast_addr(struct cpsw2_intf *cpsw_intf, u8 *addr)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_del_ucast(cpsw_dev->ale, addr, cpsw_dev->host_port, 0, 0);
}

static int cpsw2_add_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	if (!cpsw_dev->opened)
		return -ENXIO;

	dev_dbg(cpsw_dev->dev, "ethss adding address %pM, type %d\n",
		naddr->addr, naddr->type);

	switch (naddr->type) {
	case ADDR_MCAST:
	case ADDR_BCAST:
		cpsw2_add_mcast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_UCAST:
	case ADDR_DEV:
		cpsw2_add_ucast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_ANY:
		/* nothing to do for promiscuous */
	default:
		break;
	}

	return 0;
}

static int cpsw2_del_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	if (!cpsw_dev->opened)
		return -ENXIO;

	dev_dbg(cpsw_dev->dev, "ethss deleting address %pM, type %d\n",
		naddr->addr, naddr->type);

	switch (naddr->type) {
	case ADDR_MCAST:
	case ADDR_BCAST:
		cpsw2_del_mcast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_UCAST:
	case ADDR_DEV:
		cpsw2_del_ucast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_ANY:
		/* nothing to do for promiscuous */
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_TI_CPTS
static void cpsw2_hwtstamp(struct cpsw2_intf *cpsw_intf)
{
	struct cpsw2_priv *priv = cpsw_intf->cpsw_priv;
	struct cpsw2_slave *slave = cpsw_intf->slaves;
	u32 ts_en, seq_id, ctl, i;

	if (!priv->cpts.tx_enable && !priv->cpts.rx_enable) {
		writel(0, &slave->regs->ts_ctl);
		return;
	}

	seq_id = (30 << CPSW2_TS_SEQ_ID_OFS_SHIFT) | ETH_P_1588;
	ts_en = EVENT_MSG_BITS << CPSW2_TS_MSG_TYPE_EN_SHIFT;
	ctl = ETH_P_1588 | CPSW2_TS_TTL_NONZERO |
		(slave->ts_ctl.dst_port_map << CPSW2_TS_CTL_DST_PORT_SHIFT) |
		(slave->ts_ctl.uni ?  CPSW2_TS_UNI_EN :
			slave->ts_ctl.maddr_map << CPSW2_TS_CTL_MADDR_SHIFT);

	if (priv->cpts.tx_enable)
		ts_en |= (CPSW2_TS_TX_ANX_ALL_EN | CPSW2_TS_TX_VLAN_LT1_EN);

	if (priv->cpts.rx_enable)
		ts_en |= (CPSW2_TS_RX_ANX_ALL_EN | CPSW2_TS_RX_VLAN_LT1_EN);

	for (i = 0; i < cpsw_intf->num_slaves; i++, slave++) {
		writel(ts_en, &slave->regs->ts_ctl);
		writel(seq_id, &slave->regs->ts_seq_ltype);
		writel(ctl, &slave->regs->ts_ctl_ltype2);
	}
}

static int cpsw2_hwtstamp_ioctl(struct cpsw2_intf *cpsw_intf, struct ifreq *ifr)
{
	struct cpsw2_priv *priv = cpsw_intf->cpsw_priv;
	struct cpts *cpts = &priv->cpts;
	struct hwtstamp_config cfg;

	if (!cpts->reg)
		return -EOPNOTSUPP;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	/* reserved for future extensions */
	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		cpts->tx_enable = 0;
		break;
	case HWTSTAMP_TX_ON:
		cpts->tx_enable = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		cpts->rx_enable = 0;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		cpts->rx_enable = 1;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		cpts->rx_enable = 1;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}

	cpsw2_hwtstamp(cpsw_intf);

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}
#else
static inline int cpsw2_hwtstamp_ioctl(struct cpsw2_intf *cpsw_intf,
					struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}
#endif /*CONFIG_TI_CPTS*/

static int cpsw2_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;
	struct cpsw2_slave *slave = cpsw_intf->slaves;
	struct phy_device *phy = slave->phy;
	int ret = -EOPNOTSUPP;

	if (cpsw_intf->cpsw_priv->force_no_hwtstamp)
		return -EOPNOTSUPP;

	if (phy)
		ret = phy_mii_ioctl(phy, req, cmd);

	if ((cmd == SIOCSHWTSTAMP) && (ret == -EOPNOTSUPP))
		ret = cpsw2_hwtstamp_ioctl(cpsw_intf, req);

	return ret;
}

static void cpsw2_timer(struct timer_list *t)
{
	struct cpsw2_intf *cpsw_intf = from_timer(cpsw_intf, t, timer);
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	u32 sp = cpsw_intf->slave_port;
	u32 ns = cpsw_intf->num_slaves;
	u32 link_state;

	if (cpsw_dev->multi_if)
		link_state = keystone_sgmii_get_port_link(SGMII2_BASE(sp), sp);
	else {
		/* Single interface mode. Link is up if any one slave
		 * port is up.  It assumes slave port always starts from
		 * 0 and is consecutive.
		 */

		/* slave port (port > 2) status */
		link_state = keystone_sgmii_link_status(SGMII2_BASE(2),
						   max_t(u32, ns, 2) - 2);

		link_state <<= 2;

		/* slave port 0, 1 status */
		link_state |= keystone_sgmii_link_status(SGMII2_BASE(0),
						   min_t(u32, ns, 2));
	}
	cpsw_intf->link_state = link_state;

	/* if MAC-to-PHY, check phy link status also
	 * to conclude the intf link's status
	 */
	for_each_slave(cpsw_intf, cpsw2_slave_link, cpsw_intf);

	/* Is this the right logic?
	 *  multi_if & MAC_PHY: phy state machine already reported carrier
	 *  multi_if & !MAC_PHY: report carrier
	 * !multi_if: any one slave up means intf is up, reporting carrier
	 *            here corrects what phy state machine (if it exists)
	 *            might have reported.
	 */
	if (!cpsw_dev->multi_if ||
	    (cpsw_dev->multi_if &&
	     !IS_SGMII_MAC_PHY(cpsw_intf->slaves->link_interface))) {
		if (cpsw_intf->link_state)
			netif_carrier_on(cpsw_intf->ndev);
		else
			netif_carrier_off(cpsw_intf->ndev);
	}

	/* A timer runs as a BH, no need to block them */
	spin_lock(&cpsw_dev->hw_stats_lock);
	cpsw2_update_stats(cpsw_dev, NULL);
	spin_unlock(&cpsw_dev->hw_stats_lock);

	cpsw_intf->timer.expires = jiffies + (HZ/10);
	add_timer(&cpsw_intf->timer);

	return;
}

#ifdef CONFIG_TI_CPTS
#define PHY_TXTSTAMP(p)				\
		(p->skb->dev &&			\
		 p->skb->dev->phydev &&		\
		 p->skb->dev->phydev->drv &&	\
		 p->skb->dev->phydev->drv->txtstamp)

#define PHY_RXTSTAMP(p)				\
		(p->skb->dev &&			\
		 p->skb->dev->phydev &&		\
		 p->skb->dev->phydev->drv &&	\
		 p->skb->dev->phydev->drv->rxtstamp)

static bool phy_ptp_tstamp(const struct netcp_packet *p_info, bool is_tx)
{
	struct sk_buff *skb = p_info->skb;
	unsigned type = PTP_CLASS_NONE;

	if (is_tx && likely(PHY_TXTSTAMP(p_info)))
		type = ptp_classify_raw(skb);
	else if (!is_tx && likely(PHY_RXTSTAMP(p_info)))
		type = ptp_classify_raw(skb);
	else
		return false;

	if (type != PTP_CLASS_NONE)
		return true;
	return false;
}

static int cpsw2_txtstamp_complete(void *context, struct sk_buff *skb)
{
	struct cpsw2_intf *cpsw_intf = context;
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	return cpts_tx_timestamp(&cpsw_dev->cpts, skb);
}

static bool cpsw2_cpts_txtstamp(struct cpsw2_intf *cpsw_intf,
				const struct netcp_packet *p_info)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	struct sk_buff *skb = p_info->skb;
	unsigned type = PTP_CLASS_NONE;

	if (!cpsw_dev->cpts.tx_enable)
		return false;

	type = ptp_classify_raw(skb);
	if (type != PTP_CLASS_NONE)
		return true;
	return false;
}

static int cpsw2_mark_pkt_txtstamp(struct cpsw2_intf *cpsw_intf,
				  struct netcp_packet *p_info)
{
	if (!(skb_shinfo(p_info->skb)->tx_flags & SKBTX_HW_TSTAMP))
		return 0;

	if (phy_ptp_tstamp(p_info, true)) {
		skb_shinfo(p_info->skb)->tx_flags |= SKBTX_IN_PROGRESS;
		return 0;
	}

	if (cpsw2_cpts_txtstamp(cpsw_intf, p_info)) {
		p_info->txtstamp_complete = cpsw2_txtstamp_complete;
		p_info->ts_context = (void *)cpsw_intf;
		skb_shinfo(p_info->skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}

	return 0;
}

static int cpsw2_rxtstamp_complete(struct cpsw2_intf *cpsw_intf,
				struct netcp_packet *p_info)
{
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	if (p_info->rxtstamp_complete)
		return 0;

	if (phy_ptp_tstamp(p_info, false)) {
		p_info->rxtstamp_complete = true;
		return 0;
	}

	if (!cpts_rx_timestamp(&cpsw_dev->cpts, p_info->skb))
		p_info->rxtstamp_complete = true;

	return 0;
}

static inline void cpsw2_register_cpts(struct cpsw2_priv *cpsw_dev)
{
	if (!cpsw_dev->cpts.reg)
		return;

	if (cpsw_dev->cpts_registered < 0)
		/* Should not happen */
		return;

	if (cpsw_dev->cpts_registered > 0)
		goto done;

	/* Let cpts calculate the mult and shift */
	if (cpts_register(cpsw_dev->dev, &cpsw_dev->cpts,
			  cpsw_dev->cpts.cc.mult, cpsw_dev->cpts.cc.shift)) {
		dev_err(cpsw_dev->dev, "error registering cpts device\n");
		return;
	}

done:
	++cpsw_dev->cpts_registered;
}

static inline void cpsw2_unregister_cpts(struct cpsw2_priv *cpsw_dev)
{
	if (!cpsw_dev->cpts.reg)
		return;

	if (cpsw_dev->cpts_registered <= 0)
		return;

	--cpsw_dev->cpts_registered;

	if (cpsw_dev->cpts_registered)
		return;

	cpts_unregister(&cpsw_dev->cpts);
}

static void cpsw2_update_cpts_dt_params(struct cpsw2_priv *cpsw_dev,
				  struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "cpts_reg_ofs",
				   &cpsw_dev->cpts_reg_ofs);
	if (ret < 0)
		dev_err(cpsw_dev->dev,
			"missing cpts reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "cpts_rftclk_sel",
				   &cpsw_dev->cpts.rftclk_sel);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing cpts rftclk_sel, err %d\n", ret);
		cpsw_dev->cpts.rftclk_sel = 0;
	}

	ret = of_property_read_u32(node, "cpts_rftclk_freq",
				   &cpsw_dev->cpts.rftclk_freq);
	if (ret < 0) {
		dev_vdbg(cpsw_dev->dev,
			"cpts rftclk freq not defined\n");
		cpsw_dev->cpts.rftclk_freq = 0;
	}

	ret = of_property_read_u32(node, "cpts_ts_comp_length",
				&cpsw_dev->cpts.ts_comp_length);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing cpts ts_comp length, err %d\n", ret);
		cpsw_dev->cpts.ts_comp_length = 1;
	}

	if (of_property_read_u32(node, "cpts_ts_comp_polarity",
				&cpsw_dev->cpts.ts_comp_polarity))
		cpsw_dev->cpts.ts_comp_polarity = 1;

	if (of_property_read_u32(node, "cpts_clock_mult",
				&cpsw_dev->cpts.cc.mult)) {
		dev_err(cpsw_dev->dev,
			"Missing cpts_clock_mult property in the DT.\n");
		cpsw_dev->cpts.cc.mult = 0;
	}

	if (of_property_read_u32(node, "cpts_clock_shift",
				&cpsw_dev->cpts.cc.shift)) {
		dev_err(cpsw_dev->dev,
			"Missing cpts_clock_shift property in the DT.\n");
		cpsw_dev->cpts.cc.shift = 0;
	}

	if (of_property_read_u32(node, "cpts_clock_div",
				&cpsw_dev->cpts.cc_div)) {
		dev_err(cpsw_dev->dev,
			"Missing cpts_clock_div property in the DT.\n");
		cpsw_dev->cpts.cc_div = 1;
	}

	cpsw_dev->cpts.ignore_adjfreq =
		of_property_read_bool(node, "cpts-ignore-adjfreq");
}

#else
static inline int cpsw2_mark_pkt_txtstamp(struct cpsw2_intf *cpsw_intf,
					struct netcp_packet *p_info)
{
	return 0;
}

static inline int cpsw2_rxtstamp_complete(struct cpsw2_intf *cpsw_intf,
					struct netcp_packet *p_info)
{
	return 0;
}

static inline void cpsw2_register_cpts(struct cpsw2_priv *cpsw_dev)
{
}

static inline void cpsw2_unregister_cpts(struct cpsw2_priv *cpsw_dev)
{
}

static void cpsw2_update_cpts_dt_params(struct cpsw2_priv *cpsw_dev,
				  struct device_node *node)
{
}
#endif /* CONFIG_TI_CPTS */

static int cpsw2_serdes_init(struct cpsw2_priv *cpsw_dev)
{
	int i, total_slaves, slaves;
	int ret = 0;

	for (i = 0, total_slaves = cpsw_dev->num_slaves;
	     i < cpsw_dev->num_serdes;
	     i++, total_slaves -= cpsw_dev->serdes_lanes) {
		if (total_slaves <= 0)
			break;

		if (total_slaves > cpsw_dev->serdes_lanes)
			slaves = cpsw_dev->serdes_lanes;
		else
			slaves = total_slaves;

		serdes_reset(cpsw_dev->serdes_regs[i], slaves);
		ret = serdes_init(cpsw_dev->serdes_regs[i], &cpsw_dev->serdes,
				  slaves);
		if (ret < 0) {
			dev_err(cpsw_dev->dev,
				"cpsw serdes initialization failed\n");
			break;
		}
	}
	return ret;
}

static int cpsw2_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct cpsw2_intf *cpsw_intf = data;

	p_info->tx_pipe = &cpsw_intf->tx_pipe;

	return cpsw2_mark_pkt_txtstamp(cpsw_intf, p_info);
}

static int cpsw2_rx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct cpsw2_intf *cpsw_intf = data;

	return cpsw2_rxtstamp_complete(cpsw_intf, p_info);
}

#define	CPSW2_TXHOOK_ORDER	0
#define	CPSW2_RXHOOK_ORDER	0

static int cpsw2_open(void *intf_priv, struct net_device *ndev)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	struct netcp_priv *netcp = netdev_priv(ndev);
	int ret = 0;
	u32 reg;

	cpsw_dev->cpgmac = clk_get(cpsw_dev->dev, "clk_cpgmac");
	if (IS_ERR(cpsw_dev->cpgmac)) {
		ret = PTR_ERR(cpsw_dev->cpgmac);
		cpsw_dev->cpgmac = NULL;
		dev_err(cpsw_dev->dev, "unable to get Keystone CPGMAC"
			" clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(cpsw_dev->cpgmac);
	if (ret)
		goto clk_fail;

	reg = readl(&cpsw_dev->regs->id_ver);

	dev_info(cpsw_dev->dev, "initializing cpsw version %d.%d (%d) "
		 "SGMII identification value 0x%x\n",
		 CPSW2_MAJOR_VERSION(reg), CPSW2_MINOR_VERSION(reg),
		 CPSW2_RTL_VERSION(reg), CPSW2_SGMII_IDENT(reg));

	/* Use CPSW_NU P0 TX crc remove capability to remove FCS */
	netcp->hw_capabilities |= CPSW_HAS_P0_TX_CRC_REMOVE;

	ret = netcp_txpipe_open(&cpsw_intf->tx_pipe);
	if (ret)
		goto txpipe_fail;

	dev_dbg(cpsw_dev->dev, "opened TX channel %s: %p\n",
		cpsw_intf->tx_pipe.dma_chan_name,
		cpsw_intf->tx_pipe.dma_channel);

	if (atomic_inc_return(&cpsw_dev->ale_refcnt) == 1) {
		ret = cpsw2_init_ale(cpsw_dev);
		if (ret < 0) {
			atomic_dec(&cpsw_dev->ale_refcnt);
			goto ale_fail;
		}
		cpsw2_init_host_port(cpsw_dev);
	}

	for_each_slave(cpsw_intf, cpsw2_slave_init, cpsw_dev);

	for_each_slave(cpsw_intf, cpsw2_slave_stop, cpsw_intf);

	/* Serdes init */
	if (cpsw_dev->init_serdes_at_probe == 0)
		cpsw2_serdes_init(cpsw_dev);

	/* disable priority elevation and enable statistics on all ports */
	writel(0, &cpsw_dev->regs->ptype);

	/* Control register */
	__raw_writel(CPSW2_CTL_P0_ENABLE | CPSW_CTL_P0_TX_CRC_REMOVE,
			&cpsw_dev->regs->control);

	/* All statistics enabled by default */
	writel(CPSW2_REG_VAL_STAT_ENABLE_ALL(cpsw_dev->num_slaves + 1),
		     &cpsw_dev->regs->stat_port_en);

	for_each_slave(cpsw_intf, cpsw2_slave_open, cpsw_intf);

	timer_setup(&cpsw_intf->timer, cpsw2_timer, 0);
	mod_timer(&cpsw_intf->timer, jiffies + CPSW2_TIMER_INTERVAL);
	dev_dbg(cpsw_dev->dev, "%s(): cpsw2_timer = %p\n", __func__,
		cpsw2_timer);

	netcp_register_txhook(netcp, CPSW2_TXHOOK_ORDER,
			      cpsw2_tx_hook, cpsw_intf);

	if (!cpsw_dev->force_no_hwtstamp)
		netcp_register_rxhook(netcp, CPSW2_RXHOOK_ORDER,
				      cpsw2_rx_hook, cpsw_intf);

	/* Configure the streaming switch */
#define	PSTREAM_ROUTE_GLOBAL_DMA	0
	netcp_set_streaming_switch2(cpsw_dev->netcp_device, netcp->cpsw_port,
				   PSTREAM_ROUTE_GLOBAL_DMA);

	cpsw2_register_cpts(cpsw_dev);
	cpsw_dev->opened = 1;
	return 0;

ale_fail:
	netcp_txpipe_close(&cpsw_intf->tx_pipe);
txpipe_fail:
	clk_disable_unprepare(cpsw_dev->cpgmac);
clk_fail:
	clk_put(cpsw_dev->cpgmac);
	cpsw_dev->cpgmac = NULL;
	return ret;
}

static int cpsw2_close(void *intf_priv, struct net_device *ndev)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;
	struct cpsw2_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	struct netcp_priv *netcp = netdev_priv(ndev);

	del_timer_sync(&cpsw_intf->timer);

	for_each_slave(cpsw_intf, cpsw2_slave_stop, cpsw_intf);

	if (atomic_dec_return(&cpsw_dev->ale_refcnt) == 0) {
		cpsw_dev->ale = NULL;
	}

	if (!cpsw_dev->force_no_hwtstamp)
		netcp_unregister_rxhook(netcp, CPSW2_RXHOOK_ORDER,
					cpsw2_rx_hook, cpsw_intf);

	netcp_unregister_txhook(netcp, CPSW2_TXHOOK_ORDER, cpsw2_tx_hook,
				cpsw_intf);
	netcp_txpipe_close(&cpsw_intf->tx_pipe);

	clk_disable_unprepare(cpsw_dev->cpgmac);
	clk_put(cpsw_dev->cpgmac);

	cpsw2_unregister_cpts(cpsw_dev);
	cpsw_dev->opened = 0;
	return 0;
}

static int cpsw2_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct cpsw2_priv *cpsw_dev = inst_priv;
	struct cpsw2_intf *cpsw_intf, *tmp;
	int i;

	of_node_put(cpsw_dev->interfaces);

	list_for_each_entry_safe(cpsw_intf, tmp, &cpsw_dev->cpsw_intf_head,
				 cpsw_intf_list) {
		netcp_delete_interface(netcp_device, cpsw_intf->ndev);
	}
	BUG_ON(!list_empty(&cpsw_dev->cpsw_intf_head));

	iounmap(cpsw_dev->ss_regs);
	for (i = 0; i < cpsw_dev->num_serdes; i++)
		if (cpsw_dev->serdes_regs[i])
			iounmap(cpsw_dev->serdes_regs[i]);
	memset(cpsw_dev, 0x00, sizeof(*cpsw_dev));	/* FIXME: Poison */
	kfree(cpsw_dev);
	return 0;
}

static int init_slave(struct cpsw2_priv *cpsw_dev,
		      struct device_node *node, int slave_num)
{
	int ret = 0;

	ret = of_property_read_u32(node, "link-interface",
				   &cpsw_dev->link[slave_num]);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing link-interface value"
			"defaulting to mac-phy link\n");
		cpsw_dev->link[slave_num] = 1;
	}

	cpsw_dev->phy_node[slave_num] = of_parse_phandle(node, "phy-handle", 0);

	return 0;
}

static int cpsw2_create_cpts_sysfs(struct cpsw2_priv *cpsw_dev)
{
	struct kobject *pts_kobj;
	char *port_name[] = {"1", "2", "3", "4", "5", "6", "7", "8", NULL};
	int i, ret;

	pts_kobj = kobject_create_and_add("port_ts",
			kobject_get(&cpsw_dev->kobj));
	if (!pts_kobj) {
		dev_err(cpsw_dev->dev,
			"failed to create sysfs port_ts entry\n");
		kobject_put(&cpsw_dev->kobj);
		return -ENOMEM;
	}

	for (i = 0; (i < cpsw_dev->num_slaves) && port_name[i]; i++) {
		ret = kobject_init_and_add(&cpsw_dev->port_ts_kobj[i],
			&cpsw_pts_n_ktype, kobject_get(pts_kobj), port_name[i]);

		if (ret) {
			dev_err(cpsw_dev->dev,
				"failed to create sysfs port_ts/%s entry\n",
				port_name[i]);
			kobject_put(&cpsw_dev->port_ts_kobj[i]);
			kobject_put(pts_kobj);
			return ret;
		}
	}

	return 0;
}

static int cpsw2_create_sysfs_entries(struct cpsw2_priv *cpsw_dev)
{
	struct device *dev = cpsw_dev->dev;
	int ret;

	ret = kobject_init_and_add(&cpsw_dev->kobj, &cpsw_ktype,
		kobject_get(&dev->kobj), "cpsw");

	if (ret) {
		dev_err(dev, "failed to create cpsw sysfs entry\n");
		kobject_put(&cpsw_dev->kobj);
		kobject_put(&dev->kobj);
		return ret;
	}

	ret = kobject_init_and_add(&cpsw_dev->tx_pri_kobj,
		&cpsw_tx_pri_ktype,
		kobject_get(&cpsw_dev->kobj), "port_tx_pri_map");

	if (ret) {
		dev_err(dev, "failed to create sysfs port_tx_pri_map entry\n");
		kobject_put(&cpsw_dev->tx_pri_kobj);
		kobject_put(&cpsw_dev->kobj);
		return ret;
	}

	ret = kobject_init_and_add(&cpsw_dev->pvlan_kobj,
		&cpsw_pvlan_ktype,
		kobject_get(&cpsw_dev->kobj), "port_vlan");

	if (ret) {
		dev_err(dev, "failed to create sysfs port_vlan entry\n");
		kobject_put(&cpsw_dev->pvlan_kobj);
		kobject_put(&cpsw_dev->kobj);
		return ret;
	}

	ret = cpsw2_create_cpts_sysfs(cpsw_dev);
	if (ret)
		return ret;

	ret = kobject_init_and_add(&cpsw_dev->stats_kobj,
		&cpsw_stats_ktype,
		kobject_get(&cpsw_dev->kobj), "stats");

	if (ret) {
		dev_err(dev, "failed to create sysfs stats entry\n");
		kobject_put(&cpsw_dev->stats_kobj);
		kobject_put(&cpsw_dev->kobj);
		return ret;
	}

	return 0;
}

static int cpsw2_probe(struct netcp_device *netcp_device,
			struct device *dev,
			struct device_node *node,
			void **inst_priv)
{
	struct cpsw2_priv *cpsw_dev;
	struct device_node *slaves, *slave, *interfaces;
	void __iomem *regs;
	struct net_device *ndev;
	int slave_num = 0;
	int i, ret = 0;
	u32 temp[CPSW2_SERDES_MAX_NUM * 2];

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	cpsw_dev = devm_kzalloc(dev, sizeof(struct cpsw2_priv), GFP_KERNEL);
	if (!cpsw_dev) {
		dev_err(dev, "cpsw_dev memory allocation failed\n");
		return -ENOMEM;
	}
	*inst_priv = cpsw_dev;
	dev_dbg(dev, "%s(): cpsw_priv = %p\n", __func__, cpsw_dev);

	cpsw_dev->dev = dev;
	cpsw_dev->netcp_device = netcp_device;

	ret = of_property_read_u32(node, "num-serdes",
				   &cpsw_dev->num_serdes);
	if (ret < 0) {
		dev_err(dev, "missing num-serdes parameter\n");
		cpsw_dev->num_serdes = CPSW2_SERDES_MAX_NUM;
	}
	if (cpsw_dev->num_serdes > CPSW2_SERDES_MAX_NUM) {
		dev_err(dev, "invalid num_serdes, should not be bigger than"
			     " CPSW2_SERDES_MAX_NUM, reset the value to"
			     " CPSW2_SERDES_MAX_NUM\n");
		cpsw_dev->num_serdes = CPSW2_SERDES_MAX_NUM;
	}
	dev_dbg(dev, "num-serdes %u\n", cpsw_dev->num_serdes);

	ret = of_property_read_u32(node, "serdes-lanes",
				   &cpsw_dev->serdes_lanes);
	if (ret < 0) {
		dev_err(dev, "missing serdes-lanes parameter\n");
		cpsw_dev->serdes_lanes = CPSW2_LANE_NUM_PER_SERDES;
	}
	dev_dbg(dev, "serdes-lanes %u\n", cpsw_dev->serdes_lanes);

	if (of_property_read_u32_array(node, "serdes_reg", (u32 *)&(temp[0]),
					cpsw_dev->num_serdes * 2)) {
		dev_err(dev, "No serdes regs defined\n");
		ret = -ENODEV;
		goto exit;
	}

	for (i = 0; i < cpsw_dev->num_serdes; i++) {
		cpsw_dev->serdes_regs[i] = ioremap(temp[i*2], temp[i*2+1]);
		if (!cpsw_dev->serdes_regs[i]) {
			dev_err(dev, "can't map serdes regs\n");
			ret = -ENOMEM;
			goto exit;
		}
	}

	ret = of_property_read_u32(node, "serdes-ref-clk",
				   &cpsw_dev->serdes.clk);
	if (ret < 0) {
		dev_err(dev, "missing serdes-ref-clk parameter\n");
		cpsw_dev->serdes.clk = SERDES_CLOCK_156P25M;
	}
	dev_dbg(dev, "serdes-ref-clk %u\n", cpsw_dev->serdes.clk);

	ret = of_property_read_u32(node, "serdes-baud-rate",
				   &cpsw_dev->serdes.rate);
	if (ret < 0) {
		dev_err(dev, "missing serdes-baud-rate parameter\n");
		cpsw_dev->serdes.rate = SERDES_RATE_5G;
	}
	dev_dbg(dev, "serdes-baud-rate %u\n", cpsw_dev->serdes.rate);

	ret = of_property_read_u32(node, "serdes-rate-mode",
				   &cpsw_dev->serdes.rate_mode);
	if (ret < 0) {
		dev_err(dev, "missing serdes-rate-mode parameter\n");
		cpsw_dev->serdes.rate_mode = SERDES_QUARTER_RATE;
	}
	dev_dbg(dev, "serdes-rate-mode %u\n", cpsw_dev->serdes.rate_mode);

	ret = of_property_read_u32(node, "serdes-phy-intf",
				   &cpsw_dev->serdes.intf);
	if (ret < 0) {
		dev_err(dev, "missing serdes-phy-intf parameter\n");
		cpsw_dev->serdes.intf = SERDES_PHY_SGMII;
	}
	dev_dbg(dev, "serdes-phy-intf %u\n", cpsw_dev->serdes.intf);

	ret = of_property_read_u32(node, "serdes-loopback",
				   &cpsw_dev->serdes.loopback);
	if (ret < 0) {
		dev_err(dev, "missing serdes-loopback parameter\n");
		cpsw_dev->serdes.loopback = 0;
	}
	dev_dbg(dev, "serdes-loopback %u\n", cpsw_dev->serdes.loopback);

	ret = of_property_read_u32(node, "serdes_at_probe",
				   &cpsw_dev->init_serdes_at_probe);
	if (ret < 0) {
		dev_err(dev, "missing serdes_at_probe parameter\n");
		cpsw_dev->init_serdes_at_probe = 0;
	}
	dev_dbg(dev, "serdes_at_probe %u\n", cpsw_dev->init_serdes_at_probe);

	ret = of_property_read_u32(node, "num_slaves", &cpsw_dev->num_slaves);
	if (ret < 0) {
		dev_err(dev, "missing num_slaves parameter, err %d\n", ret);
		cpsw_dev->num_slaves = 2;
	}

	ret = of_property_read_u32(node, "sgmii_module_ofs",
				   &cpsw_dev->sgmii_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing sgmii module offset, err %d\n", ret);

	ret = of_property_read_u32(node, "switch_module_ofs",
				   &cpsw_dev->switch_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing switch module offset, err %d\n", ret);

	ret = of_property_read_u32(node, "host_port_reg_ofs",
				   &cpsw_dev->host_port_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing host port reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "slave_reg_ofs",
				   &cpsw_dev->slave_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing slave reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "hw_stats_reg_ofs",
				   &cpsw_dev->hw_stats_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing hw stats reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "ale_reg_ofs",
				   &cpsw_dev->ale_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing ale reg offset, err %d\n", ret);

	cpsw2_update_cpts_dt_params(cpsw_dev, node);

	ret = of_property_read_u32(node, "ale_ageout", &cpsw_dev->ale_ageout);
	if (ret < 0) {
		dev_err(dev, "missing ale_ageout parameter, err %d\n", ret);
		cpsw_dev->ale_ageout = 10;
	}

	ret = of_property_read_u32(node, "ale_entries", &cpsw_dev->ale_entries);
	if (ret < 0) {
		dev_err(dev, "missing ale_entries parameter, err %d\n", ret);
		cpsw_dev->ale_entries = 1024;
	}

	ret = of_property_read_u32(node, "ale_ports", &cpsw_dev->ale_ports);
	if (ret < 0) {
		dev_err(dev, "missing ale_ports parameter, err %d\n", ret);
		cpsw_dev->ale_ports = 2;
	}

	ret = of_property_read_u32(node, "intf_tx_queues",
				   &cpsw_dev->intf_tx_queues);
	if (ret < 0) {
		dev_err(dev, "missing intf_tx_queues parameter, err %d\n", ret);
		cpsw_dev->intf_tx_queues = 1;
	}

	if (of_find_property(node, "force_no_hwtstamp", NULL)) {
		cpsw_dev->force_no_hwtstamp = 1;
		dev_warn(dev, "***** No CPSW or PHY timestamping *****\n");
	}

	if (of_find_property(node, "multi-interface", NULL))
		cpsw_dev->multi_if = 1;

	ret = of_property_read_u32(node, "num-interfaces",
				   &cpsw_dev->num_interfaces);
	if (ret < 0) {
		dev_err(dev, "missing num-interfaces parameter\n");
		cpsw_dev->num_interfaces = 1;
	}

	ret = of_property_read_u32(node, "slaves-per-interface",
				   &cpsw_dev->slaves_per_interface);
	if (ret < 0) {
		dev_err(dev, "missing slaves-per_interface parameter\n");
		cpsw_dev->slaves_per_interface = 2;
	}

	/* cpsw sub-system regs base */
	if (of_property_read_u32_array(node, "cpsw-ss-reg", (u32 *)&(temp[0]),
				       2)) {
		dev_err(dev, "No cpsw-ss-reg defined\n");
		ret = -ENODEV;
		goto exit;
	}

	regs = ioremap(temp[0], temp[1]);
	if (!regs) {
		dev_err(dev, "can't map CPSW SS regs\n");
		ret = -ENOMEM;
		goto exit;
	}

	cpsw_dev->ss_regs = regs;
	cpsw_dev->sgmii_port_regs =
		regs + cpsw_dev->sgmii_module_ofs;
	cpsw_dev->regs = regs + cpsw_dev->switch_module_ofs;
	cpsw_dev->host_port_regs = regs + cpsw_dev->host_port_reg_ofs;

	for (i = 0; i < CPSW2_NUM_PORTS; i++)
		cpsw_dev->hw_stats_regs[i] =
			regs + cpsw_dev->hw_stats_reg_ofs +
			(CPSW2_STATS_REGS_SIZE * i);
	cpsw_dev->ale_reg	  = regs + cpsw_dev->ale_reg_ofs;
#ifdef CONFIG_TI_CPTS
	if (cpsw_dev->cpts_reg_ofs)
		cpsw_dev->cpts.reg = regs + cpsw_dev->cpts_reg_ofs;
#endif
	if (cpsw_dev->init_serdes_at_probe == 1)
		cpsw2_serdes_init(cpsw_dev);

	cpsw_dev->host_port = 0;
	cpsw_dev->rx_packet_max = NETCP_MAX_FRAME_SIZE;

	dev_dbg(dev, "num_slaves = %d\n", cpsw_dev->num_slaves);
	dev_dbg(dev, "ale_ageout = %d\n", cpsw_dev->ale_ageout);
	dev_dbg(dev, "ale_entries = %d\n", cpsw_dev->ale_entries);
	dev_dbg(dev, "ale_ports = %d\n", cpsw_dev->ale_ports);

	slaves = of_get_child_by_name(node, "slaves");
	if (!slaves) {
		dev_err(dev, "could not find slaves\n");
		ret = -ENODEV;
		goto exit;
	}

	for_each_child_of_node(slaves, slave) {
		init_slave(cpsw_dev, slave, slave_num);
			slave_num++;
	}

	of_node_put(slaves);

	interfaces = of_get_child_by_name(node, "interfaces");
	if (!interfaces)
		dev_err(dev, "could not find interfaces\n");

	cpsw_dev->interfaces = interfaces;

	/* Create the interface */
	INIT_LIST_HEAD(&cpsw_dev->cpsw_intf_head);
	if (cpsw_dev->multi_if)
		for (i = 0; i < cpsw_dev->num_interfaces; i++)
			netcp_create_interface(netcp_device, &ndev,
					       NULL, cpsw_dev->intf_tx_queues,
					       1, (i + 1));
	else
		netcp_create_interface(netcp_device, &ndev,
					       NULL, cpsw_dev->intf_tx_queues,
					       1, 0);

	/* init the hw stats */
	spin_lock_init(&cpsw_dev->hw_stats_lock);
	spin_lock_bh(&cpsw_dev->hw_stats_lock);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS0_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS1_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS2_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS3_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS4_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS5_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS6_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS7_MODULE);
	cpsw2_reset_mod_stats(cpsw_dev, CPSW2_STATS8_MODULE);
	spin_unlock_bh(&cpsw_dev->hw_stats_lock);

	ret = cpsw2_create_sysfs_entries(cpsw_dev);
	if (ret)
		goto exit;

	return 0;

exit:
	if (cpsw_dev->ss_regs)
		iounmap(cpsw_dev->ss_regs);
	for (i = 0; i < cpsw_dev->num_serdes; i++)
		if (cpsw_dev->serdes_regs[i])
			iounmap(cpsw_dev->serdes_regs[i]);

	*inst_priv = NULL;
	kfree(cpsw_dev);
	return ret;
}

static void cpsw2_slave_cpts_ctl_init(struct cpsw2_slave *slave)
{
	slave->ts_ctl.uni = 1;
	slave->ts_ctl.dst_port_map =
		(CPSW2_TS_CTL_DST_PORT >> CPSW2_TS_CTL_DST_PORT_SHIFT) & 0x3;
	slave->ts_ctl.maddr_map =
		(CPSW2_TS_CTL_MADDR_ALL >> CPSW2_TS_CTL_MADDR_SHIFT) & 0x1f;
}

static int cpsw2_attach(void *inst_priv, struct net_device *ndev,
		       void **intf_priv)
{
	struct cpsw2_priv *cpsw_dev = inst_priv;
	struct cpsw2_intf *cpsw_intf;
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct device_node *interface;
	int i = 0, ret = 0;
	char node_name[24];

	cpsw_intf = devm_kzalloc(cpsw_dev->dev,
				 sizeof(struct cpsw2_intf), GFP_KERNEL);
	if (!cpsw_intf) {
		dev_err(cpsw_dev->dev, "cpsw interface memory "
			"allocation failed\n");
		return -ENOMEM;
	}
	cpsw_intf->ndev = ndev;
	cpsw_intf->dev = cpsw_dev->dev;
	cpsw_intf->cpsw_priv = cpsw_dev;
	cpsw_intf->multi_if = cpsw_dev->multi_if;

	if (cpsw_dev->multi_if)
		snprintf(node_name, sizeof(node_name), "interface-%d",
			 netcp->cpsw_port - 1);
	else
		snprintf(node_name, sizeof(node_name), "interface-%d",
			 0);

	interface = of_get_child_by_name(cpsw_dev->interfaces, node_name);
	if (!interface) {
		dev_err(cpsw_dev->dev, "interface data not available\n");
		devm_kfree(cpsw_dev->dev, cpsw_intf);
		return -ENODEV;
	}
	ret = of_property_read_u32(interface, "slave_port",
				   &cpsw_intf->slave_port);
	if (ret < 0) {
		dev_err(cpsw_dev->dev, "missing slave_port paramater\n");
		return -EINVAL;
	}

	ret = of_property_read_string(interface, "tx-channel",
				      &cpsw_intf->tx_chan_name);
	if (ret < 0) {
		dev_err(cpsw_dev->dev, "missing tx-channel "
			"parameter, err %d\n", ret);
		cpsw_intf->tx_chan_name = "nettx";
	}
	dev_info(cpsw_dev->dev, "dma_chan_name %s\n", cpsw_intf->tx_chan_name);

	ret = of_property_read_u32(interface, "tx_queue_depth",
				   &cpsw_intf->tx_queue_depth);
	if (ret < 0) {
		dev_err(cpsw_dev->dev, "missing tx_queue_depth "
			"parameter, err %d\n", ret);
		cpsw_intf->tx_queue_depth = 32;
	}
	dev_dbg(cpsw_dev->dev, "tx_queue_depth %u\n",
		cpsw_intf->tx_queue_depth);

	of_node_put(interface);

	cpsw_intf->num_slaves = cpsw_dev->slaves_per_interface;

	cpsw_intf->slaves = devm_kzalloc(cpsw_dev->dev,
					 sizeof(struct cpsw2_slave) *
					 cpsw_intf->num_slaves, GFP_KERNEL);

	if (!cpsw_intf->slaves) {
		dev_err(cpsw_dev->dev, "cpsw interface slave memory "
			"allocation failed\n");
		devm_kfree(cpsw_dev->dev, cpsw_intf);
		return -ENOMEM;
	}

	if (cpsw_dev->multi_if) {
		cpsw_intf->slaves[i].slave_num = cpsw_intf->slave_port;
		cpsw_intf->slaves[i].link_interface =
			cpsw_dev->link[cpsw_intf->slave_port];
		cpsw_intf->phy_node = cpsw_dev->phy_node[cpsw_intf->slave_port];
		cpsw2_slave_cpts_ctl_init(&(cpsw_intf->slaves[i]));
	} else {
		for (i = 0; i < cpsw_intf->num_slaves; i++) {
			cpsw_intf->slaves[i].slave_num = i;
			cpsw_intf->slaves[i].link_interface = cpsw_dev->link[i];
			cpsw2_slave_cpts_ctl_init(&(cpsw_intf->slaves[i]));
		}
	}

	netcp_txpipe_init(&cpsw_intf->tx_pipe, netdev_priv(ndev),
			  cpsw_intf->tx_chan_name, cpsw_intf->tx_queue_depth);

	ndev->ethtool_ops = &keystone_ethtool_ops;

	list_add(&cpsw_intf->cpsw_intf_list, &cpsw_dev->cpsw_intf_head);

	*intf_priv = cpsw_intf;
	return 0;
}

static int cpsw2_release(void *intf_priv)
{
	struct cpsw2_intf *cpsw_intf = intf_priv;

	cpsw_intf->ndev->ethtool_ops = NULL;

	list_del(&cpsw_intf->cpsw_intf_list);

	netif_napi_del(&cpsw_intf->tx_pipe.dma_poll_napi);

	devm_kfree(cpsw_intf->dev, cpsw_intf->slaves);
	devm_kfree(cpsw_intf->dev, cpsw_intf);

	return 0;
}


static struct netcp_module cpsw_module = {
	.name		= CPSW2_MODULE_NAME,
	.owner		= THIS_MODULE,
	.probe		= cpsw2_probe,
	.open		= cpsw2_open,
	.close		= cpsw2_close,
	.remove		= cpsw2_remove,
	.attach		= cpsw2_attach,
	.release	= cpsw2_release,
	.add_addr	= cpsw2_add_addr,
	.del_addr	= cpsw2_del_addr,
	.ioctl		= cpsw2_ioctl,
};

int __init keystone_cpsw2_init(void)
{
	return netcp_register_module(&cpsw_module);
}
/* module_init(keystone_cpsw2_init); */

void __exit keystone_cpsw2_exit(void)
{
	netcp_unregister_module(&cpsw_module);
}
/* module_exit(keystone_cpsw2_exit); */

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hao Zhang <hzhang@ti.com>");
MODULE_DESCRIPTION("CPSW2 driver for Keystone devices");
