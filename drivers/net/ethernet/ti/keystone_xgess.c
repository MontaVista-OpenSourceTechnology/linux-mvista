/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
 * Authors: WingMan Kwok <w-kwok2@ti.com>
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "cpsw_ale.h"
#include "keystone_net.h"
#include "keystone_xserdes.h"

#define CPSW_MODULE_NAME	"keystone-cpswx"
#define NETCP_DRIVER_NAME	"TI KeyStone XGE Driver"
#define NETCP_DRIVER_VERSION	"v0.0.0"

#define CPSW_IDENT(reg)			((reg >> 16) & 0xffff)
#define CPSW_MAJOR_VERSION(reg)		(reg >> 8 & 0x7)
#define CPSW_MINOR_VERSION(reg)		(reg & 0xff)
#define CPSW_RTL_VERSION(reg)		((reg >> 11) & 0x1f)

#define DEVICE_N_GMACSL_PORTS			2
#define DEVICE_EMACSL_RESET_POLL_COUNT		100

#define	CPSW_TIMER_INTERVAL			(HZ / 10)
#define CPSW_WAIT_FOR_WORK_DONE 		100

/* Soft reset register values */
#define SOFT_RESET_MASK				BIT(0)
#define SOFT_RESET				BIT(0)

#define MACSL_RX_ENABLE_CSF			BIT(23)
#define MACSL_RX_ENABLE_EXT_CTL			BIT(18)
#define MACSL_XGMII_ENABLE			BIT(13)
#define MACSL_XGIG_MODE				BIT(8)
#define MACSL_GMII_ENABLE			BIT(5)
#define GMACSL_RET_WARN_RESET_INCOMPLETE	-2

#define SLAVE_LINK_IS_XGMII(s) \
	((s)->link_interface >= XGMII_LINK_MAC_PHY)

#define MACSL_SIG_ENABLE(s) \
		(SLAVE_LINK_IS_XGMII((s)) ?   \
		(MACSL_XGMII_ENABLE | MACSL_XGIG_MODE) : \
		MACSL_GMII_ENABLE)

#define MACSL_DEFAULT_CONFIG(s) \
		(MACSL_SIG_ENABLE((s)) | \
		 MACSL_RX_ENABLE_EXT_CTL)

#define CPSW_NUM_PORTS		                3
#define CPSW_CTL_P0_ENABLE			BIT(2)
#define CPSW_CTL_VLAN_AWARE			BIT(1)
#define CPSW_REG_VAL_STAT_ENABLE_ALL		0xf

#define CPSW_MASK_ALL_PORTS			7
#define CPSW_MASK_PHYS_PORTS			6
#define CPSW_MASK_NO_PORTS			0

#define CPSW_STATS0_MODULE			0
#define CPSW_STATS1_MODULE			1
#define CPSW_STATS2_MODULE			2

#define HOST_TX_PRI_MAP_DEFAULT			0x00000000
#define MAX_SIZE_STREAM_BUFFER		        9504

#define XGMII_LINK_STATE_INVALID		0xffffffff

/* CPSWX SERDES */
#define CPSWX_SERDES_MAX_NUM			1
#define CPSWX_SERDES_MAX_LANES			2

/* slave_num: 0-based
 *  port_num: 1-based
 */
struct cpswx_slave {
	struct cpswx_priv		*cpsw_dev;
	struct cpswx_slave_regs __iomem		*regs;
	struct cpswx_sliver_regs __iomem	*sliver;
	int				 slave_num;
	int				 port_num;
	u32				 mac_control;
	struct phy_device		*phy;
	const char			*phy_id;
	struct cpsw_ale			*ale;
	u32				 link_interface;
	u8				 phy_port_t;
					/* work trigger threshold
					 *   0: triger disabled
					 * > 1: trigger enabled
					 */
	u32				link_recover_thresh;
					/* 0:NOT, > 0:recovering */
	u32				link_recovering;
	struct delayed_work		link_recover_work;
};

/* 0x0000 */
struct cpswx_ss_regs {
	u32	id_ver;
	u32	synce_count;
	u32	synce_mux;
	u32	control;
};

/* 0x1000 */
struct cpswx_regs {
	u32	id_ver;
	u32	control;
	u32	emcontrol;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
	u32	cppi_thresh;
};

/* 0x1064, 0x1094 */
struct cpswx_slave_regs {
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	sa_lo;
	u32	sa_hi;
	u32	ts_ctl;
	u32	ts_seq_ltype;
	u32	ts_vlan;
	u32	ts_ctl_ltype2;
	u32	ts_ctl2;
	u32	control;
};

/* 0x1034 */
struct cpswx_host_regs {
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	src_id;
	u32	rx_pri_map;
	u32	rx_maxlen;
};

/* 0x1400, 0x1440 */
struct cpswx_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	em_control;
	u32	__reserved_1;
	u32	tx_gap;
	u32	rsvd[4];
};

struct cpswx_host_hw_stats {
	u32	rx_good_frames;
	u32	rx_broadcast_frames;
	u32	rx_multicast_frames;
	u32	__rsvd_0[3];
	u32	rx_oversized_frames;
	u32	__rsvd_1;
	u32	rx_undersized_frames;
	u32	__rsvd_2;
	u32	overrun_type4;
	u32	overrun_type5;
	u32	rx_bytes;
	u32	tx_good_frames;
	u32	tx_broadcast_frames;
	u32	tx_multicast_frames;
	u32	__rsvd_3[9];
	u32	tx_bytes;
	u32	tx_64byte_frames;
	u32	tx_65_to_127byte_frames;
	u32	tx_128_to_255byte_frames;
	u32	tx_256_to_511byte_frames;
	u32	tx_512_to_1023byte_frames;
	u32	tx_1024byte_frames;
	u32	net_bytes;
	u32	rx_sof_overruns;
	u32	rx_mof_overruns;
	u32	rx_dma_overruns;
};

/* 0x1900, 0x1a00 */
struct cpswx_hw_stats {
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
	u32	overrun_type4;
	u32	overrun_type5;
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
	u32	tx_underrun;
	u32	tx_carrier_sense_errors;
	u32	tx_bytes;
	u32	tx_64byte_frames;
	u32	tx_65_to_127byte_frames;
	u32	tx_128_to_255byte_frames;
	u32	tx_256_to_511byte_frames;
	u32	tx_512_to_1023byte_frames;
	u32	tx_1024byte_frames;
	u32	net_bytes;
	u32	rx_sof_overruns;
	u32	rx_mof_overruns;
	u32	rx_dma_overruns;
};

/* 0x1700 */
struct cpswx_ale_regs {
	u32	ale_idver;
	u32	__rsvd0;
	u32	ale_control;
	u32	__rsvd1;
	u32	ale_prescale;
	u32	ale_aging_timer;  /* +++FIXME: no description found in manual */
	u32	ale_unknown_vlan;
	u32	__rsvd3;
	u32	ale_tblctl;
	u32	__rsvd4[4];
	u32	ale_tblw2;
	u32	ale_tblw1;
	u32	ale_tblw0;
	u32	ale_portctl[3];
};

struct cpswx_serdes_priv {
	u32				num_serdes;
	int				init_serdes_at_probe;
	spinlock_t			tbus_lock;
	struct kserdes_config		serdes_config[CPSWX_SERDES_MAX_NUM];
};

struct cpswx_priv {
	struct device			*dev;
	struct clk			*clk;
	struct netcp_device		*netcp_device;
	u32				 num_slaves;
	u32				 ale_ageout;
	u32				 ale_entries;
	u32				 ale_ports;
	u32				 sgmii_module_ofs;
	u32				 pcsr_module_ofs;
	u32				 switch_module_ofs;
	u32				 host_port_reg_ofs;
	u32				 slave_reg_ofs;
	u32				 sliver_reg_ofs;
	u32				 hw_stats_reg_ofs;
	u32				 ale_reg_ofs;

	int				 host_port;
	u32				 rx_packet_max;

	struct cpswx_regs __iomem		*regs;
	struct cpswx_ss_regs __iomem		*ss_regs;
	struct cpswx_host_hw_stats __iomem	*host_hw_stats_regs;
	struct cpswx_hw_stats __iomem		*hw_stats_regs[2];
	struct cpswx_host_regs __iomem		*host_port_regs;
	struct cpswx_ale_regs __iomem		*ale_reg;

	void __iomem			*sgmii_port_regs;
	void __iomem			*pcsr_port_regs;

	struct cpsw_ale			*ale;
	atomic_t			 ale_refcnt;

	u32				 link[5];
	struct device_node		*phy_node[4];

	u32				 intf_tx_queues;

	u32				 multi_if;
	u32				 slaves_per_interface;
	u32				 num_interfaces;
	struct device_node		*interfaces;
	struct list_head		 cpsw_intf_head;

	u64				 hw_stats[96];
	int				 init_serdes_at_probe;
	struct kobject			kobj;
	struct kobject			tx_pri_kobj;
	struct kobject			pvlan_kobj;
	struct kobject			stats_kobj;
	spinlock_t			hw_stats_lock;
	struct cpswx_serdes_priv	serdes_priv;
	u32				opened;
};

/* slave_port: 0-based (currently relevant only in multi_if mode)
*/
struct cpswx_intf {
	struct net_device	*ndev;
	struct device		*dev;
	struct cpswx_priv	*cpsw_priv;
	struct device_node	*phy_node;
	u32			 num_slaves;
	u32			 slave_port;
	struct cpswx_slave	*slaves;
	u32			 intf_tx_queues;
	const char		*tx_chan_name;
	u32			 tx_queue_depth;
	struct netcp_tx_pipe	 tx_pipe;
	u32			 multi_if;
	struct list_head	 cpsw_intf_list;
	struct timer_list	 timer;
	u32			 link_state;
};

/*
 * Statistic management
 */
struct netcp_ethtool_stat {
	char desc[ETH_GSTRING_LEN];
	int type;
	u32 size;
	int offset;
};

/* +++FIXME: do we need the port?? */
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

#define FIELDINFO(_struct, field) FIELD_SIZEOF(_struct, field), \
					offsetof(_struct, field)

#define CPSW_STATS0_INFO(field)	"CPSW_0:"#field, CPSW_STATS0_MODULE, \
				FIELDINFO(struct cpswx_host_hw_stats, field)

#define CPSW_STATSA_INFO(field)	"CPSW_1:"#field, CPSW_STATS1_MODULE, \
				FIELDINFO(struct cpswx_hw_stats, field)
#define CPSW_STATSB_INFO(field)	"CPSW_2:"#field, CPSW_STATS2_MODULE, \
				FIELDINFO(struct cpswx_hw_stats, field)

static const struct netcp_ethtool_stat et_stats[] = {
	/* CPSW module 0 */
	{CPSW_STATS0_INFO(rx_good_frames)},
	{CPSW_STATS0_INFO(rx_broadcast_frames)},
	{CPSW_STATS0_INFO(rx_multicast_frames)},
	{CPSW_STATS0_INFO(rx_oversized_frames)},
	{CPSW_STATS0_INFO(rx_undersized_frames)},
	{CPSW_STATS0_INFO(overrun_type4)},
	{CPSW_STATS0_INFO(overrun_type5)},
	{CPSW_STATS0_INFO(rx_bytes)},
	{CPSW_STATS0_INFO(tx_good_frames)},
	{CPSW_STATS0_INFO(tx_broadcast_frames)},
	{CPSW_STATS0_INFO(tx_multicast_frames)},
	{CPSW_STATS0_INFO(tx_bytes)},
	{CPSW_STATS0_INFO(tx_64byte_frames)},
	{CPSW_STATS0_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATS0_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATS0_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATS0_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATS0_INFO(tx_1024byte_frames)},
	{CPSW_STATS0_INFO(net_bytes)},
	{CPSW_STATS0_INFO(rx_sof_overruns)},
	{CPSW_STATS0_INFO(rx_mof_overruns)},
	{CPSW_STATS0_INFO(rx_dma_overruns)},
	/* CPSW module 1 */
	{CPSW_STATSA_INFO(rx_good_frames)},
	{CPSW_STATSA_INFO(rx_broadcast_frames)},
	{CPSW_STATSA_INFO(rx_multicast_frames)},
	{CPSW_STATSA_INFO(rx_pause_frames)},
	{CPSW_STATSA_INFO(rx_crc_errors)},
	{CPSW_STATSA_INFO(rx_align_code_errors)},
	{CPSW_STATSA_INFO(rx_oversized_frames)},
	{CPSW_STATSA_INFO(rx_jabber_frames)},
	{CPSW_STATSA_INFO(rx_undersized_frames)},
	{CPSW_STATSA_INFO(rx_fragments)},
	{CPSW_STATSA_INFO(overrun_type4)},
	{CPSW_STATSA_INFO(overrun_type5)},
	{CPSW_STATSA_INFO(rx_bytes)},
	{CPSW_STATSA_INFO(tx_good_frames)},
	{CPSW_STATSA_INFO(tx_broadcast_frames)},
	{CPSW_STATSA_INFO(tx_multicast_frames)},
	{CPSW_STATSA_INFO(tx_pause_frames)},
	{CPSW_STATSA_INFO(tx_deferred_frames)},
	{CPSW_STATSA_INFO(tx_collision_frames)},
	{CPSW_STATSA_INFO(tx_single_coll_frames)},
	{CPSW_STATSA_INFO(tx_mult_coll_frames)},
	{CPSW_STATSA_INFO(tx_excessive_collisions)},
	{CPSW_STATSA_INFO(tx_late_collisions)},
	{CPSW_STATSA_INFO(tx_underrun)},
	{CPSW_STATSA_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSA_INFO(tx_bytes)},
	{CPSW_STATSA_INFO(tx_64byte_frames)},
	{CPSW_STATSA_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSA_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSA_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSA_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSA_INFO(tx_1024byte_frames)},
	{CPSW_STATSA_INFO(net_bytes)},
	{CPSW_STATSA_INFO(rx_sof_overruns)},
	{CPSW_STATSA_INFO(rx_mof_overruns)},
	{CPSW_STATSA_INFO(rx_dma_overruns)},
	/* CPSW module 2 */
	{CPSW_STATSB_INFO(rx_good_frames)},
	{CPSW_STATSB_INFO(rx_broadcast_frames)},
	{CPSW_STATSB_INFO(rx_multicast_frames)},
	{CPSW_STATSB_INFO(rx_pause_frames)},
	{CPSW_STATSB_INFO(rx_crc_errors)},
	{CPSW_STATSB_INFO(rx_align_code_errors)},
	{CPSW_STATSB_INFO(rx_oversized_frames)},
	{CPSW_STATSB_INFO(rx_jabber_frames)},
	{CPSW_STATSB_INFO(rx_undersized_frames)},
	{CPSW_STATSB_INFO(rx_fragments)},
	{CPSW_STATSB_INFO(overrun_type4)},
	{CPSW_STATSB_INFO(overrun_type5)},
	{CPSW_STATSB_INFO(rx_bytes)},
	{CPSW_STATSB_INFO(tx_good_frames)},
	{CPSW_STATSB_INFO(tx_broadcast_frames)},
	{CPSW_STATSB_INFO(tx_multicast_frames)},
	{CPSW_STATSB_INFO(tx_pause_frames)},
	{CPSW_STATSB_INFO(tx_deferred_frames)},
	{CPSW_STATSB_INFO(tx_collision_frames)},
	{CPSW_STATSB_INFO(tx_single_coll_frames)},
	{CPSW_STATSB_INFO(tx_mult_coll_frames)},
	{CPSW_STATSB_INFO(tx_excessive_collisions)},
	{CPSW_STATSB_INFO(tx_late_collisions)},
	{CPSW_STATSB_INFO(tx_underrun)},
	{CPSW_STATSB_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSB_INFO(tx_bytes)},
	{CPSW_STATSB_INFO(tx_64byte_frames)},
	{CPSW_STATSB_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSB_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSB_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSB_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSB_INFO(tx_1024byte_frames)},
	{CPSW_STATSB_INFO(net_bytes)},
	{CPSW_STATSB_INFO(rx_sof_overruns)},
	{CPSW_STATSB_INFO(rx_mof_overruns)},
	{CPSW_STATSB_INFO(rx_dma_overruns)},
};

#define ETHTOOL_STATS_NUM ARRAY_SIZE(et_stats)

struct cpswx_attribute {
	struct attribute attr;
	ssize_t (*show)(struct cpswx_priv *cpsw_dev,
		struct cpswx_attribute *attr, char *buf);
	ssize_t	(*store)(struct cpswx_priv *cpsw_dev,
		struct cpswx_attribute *attr, const char *, size_t);
	const struct cpswx_mod_info *info;
	ssize_t info_size;
	void *context;
};
#define to_cpswx_attr(_attr) container_of(_attr, struct cpswx_attribute, attr)

#define to_cpswx_dev(obj) container_of(obj, struct cpswx_priv, kobj)

#define tx_pri_to_cpswx_dev(obj) \
	container_of(obj, struct cpswx_priv, tx_pri_kobj)

#define pvlan_to_cpswx_dev(obj) \
	container_of(obj, struct cpswx_priv, pvlan_kobj)

#define stats_to_cpswx_dev(obj) \
	container_of(obj, struct cpswx_priv, stats_kobj)

#define BITS(x)			(BIT(x) - 1)
#define BITMASK(n, s)		(BITS(n) << (s))
#define cpsw_mod_info_field_val(r, i) \
	((r & BITMASK(i->bits, i->shift)) >> i->shift)

#define for_each_intf(i, priv) \
	list_for_each_entry((i), &(priv)->cpsw_intf_head, cpsw_intf_list)

#define __CPSW_ATTR_FULL(_name, _mode, _show, _store, _info,	\
				_info_size, _ctxt)		\
	{ \
		.attr = {.name = __stringify(_name), .mode = _mode },	\
		.show	= _show,		\
		.store	= _store,		\
		.info	= _info,		\
		.info_size = _info_size,	\
		.context = (_ctxt),		\
	}

#define __CPSW_ATTR(_name, _mode, _show, _store, _info) \
		__CPSW_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), NULL)

#define __CPSW_CTXT_ATTR(_name, _mode, _show, _store, _info, _ctxt) \
		__CPSW_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), _ctxt)

struct cpswx_mod_info {
	const char	*name;
	int		shift;
	int		bits;
};

struct cpswx_parse_result {
	int control;
	int port;
	u32 value;
};

static ssize_t cpsw_attr_info_show(const struct cpswx_mod_info *info,
				int info_size, u32 reg, char *buf)
{
	int i, len = 0;

	for (i = 0; i < info_size; i++, info++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"%s=%d\n", info->name,
			(int)cpsw_mod_info_field_val(reg, info));
	}

	return len;
}

static ssize_t cpsw_attr_parse_set_command(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count,
				struct cpswx_parse_result *res)
{
	char ctrl_str[33], tmp_str[9];
	int port = -1, value, len, control;
	unsigned long end;
	const struct cpswx_mod_info *info = attr->info;

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

static inline void cpsw_info_set_reg_field(void __iomem *r,
		const struct cpswx_mod_info *info, int val)
{
	u32 rv;

	rv = __raw_readl(r);
	rv = ((rv & ~BITMASK(info->bits, info->shift)) | (val << info->shift));
	__raw_writel(rv, r);
}

static ssize_t cpsw_version_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = __raw_readl(&cpsw_dev->regs->id_ver);

	return snprintf(buf, PAGE_SIZE,
		"cpsw version %d.%d (%d) SGMII identification value 0x%x\n",
		 CPSW_MAJOR_VERSION(reg), CPSW_MINOR_VERSION(reg),
		 CPSW_RTL_VERSION(reg), CPSW_IDENT(reg));
}

static struct cpswx_attribute cpsw_version_attribute =
	__ATTR(version, S_IRUGO, cpsw_version_show, NULL);

static ssize_t cpsw_prev_mode_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	struct cpswx_serdes_priv *spriv = &cpsw_dev->serdes_priv;
	struct kserdes_config *sconfig = &spriv->serdes_config[0];

	return snprintf(buf, PAGE_SIZE, "0x%x", sconfig->prev_cpu_ctrl_reg);
}

static struct cpswx_attribute cpsw_prev_mode_attribute =
	__ATTR(prev_mode, S_IRUGO, cpsw_prev_mode_show, NULL);

static const struct cpswx_mod_info cpsw_controls[] = {
	{
		.name		= "lane_reset",
		.shift		= 0,
		.bits		= 1,
	},
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
		.name		= "p0_tx_crc_type",
		.shift		= 12,
		.bits		= 1,
	},
};

static ssize_t cpsw_control_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = __raw_readl(&cpsw_dev->regs->control);
	return cpsw_attr_info_show(attr->info, attr->info_size, reg, buf);
}

void cpsw_serdes_lane_reset(struct cpswx_priv *cpsw_dev, u32 lane)
{
	struct cpswx_serdes_priv *spriv = &cpsw_dev->serdes_priv;
	struct kserdes_config *sc;
	int ret;

	sc = &spriv->serdes_config[0];
	ret = kserdes_phy_reset(sc, lane);
	if (ret)
		dev_dbg(cpsw_dev->dev, "Serdes Lane %u reset FAILED\n", lane);
}

static ssize_t cpsw_control_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpswx_mod_info *i;
	struct cpswx_parse_result res;
	void __iomem *r = NULL;
	int ret;


	ret = cpsw_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);
	r = &cpsw_dev->regs->control;

	if (res.control == 0) {
		pr_info("recover lane %u rx\n", res.value);
		cpsw_serdes_lane_reset(cpsw_dev, res.value);
		return -EINVAL;
	}

	cpsw_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpswx_attribute cpsw_control_attribute =
	__CPSW_ATTR(control, S_IRUGO | S_IWUSR,
		cpsw_control_show, cpsw_control_store, cpsw_controls);

static const struct cpswx_mod_info cpsw_ptypes[] = {
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
};

static ssize_t cpsw_pri_type_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	u32 reg;

	reg = __raw_readl(&cpsw_dev->regs->ptype);

	return cpsw_attr_info_show(attr->info, attr->info_size, reg, buf);
}

static ssize_t cpsw_pri_type_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpswx_mod_info *i;
	struct cpswx_parse_result res;
	void __iomem *r = NULL;
	int ret;


	ret = cpsw_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);
	r = &cpsw_dev->regs->ptype;

	cpsw_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpswx_attribute cpsw_pri_type_attribute =
	__CPSW_ATTR(priority_type, S_IRUGO | S_IWUSR,
			cpsw_pri_type_show,
			cpsw_pri_type_store,
			cpsw_ptypes);

static const struct cpswx_mod_info cpsw_flow_controls[] = {
	{
		.name		= "port0_flow_control_en",
		.shift		= 0,
		.bits		= 1,
	},
	{
		.name		= "port1_flow_control_en",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "port2_flow_control_en",
		.shift		= 2,
		.bits		= 1,
	},
};

static ssize_t cpsw_flow_control_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr, char *buf)
{
	u32 reg;

	reg = __raw_readl(&cpsw_dev->regs->flow_control);

	return cpsw_attr_info_show(attr->info, attr->info_size, reg, buf);
}

static ssize_t cpsw_flow_control_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpswx_mod_info *i;
	struct cpswx_parse_result res;
	void __iomem *r = NULL;
	int ret;


	ret = cpsw_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);
	r = &cpsw_dev->regs->flow_control;

	cpsw_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpswx_attribute cpsw_flow_control_attribute =
	__CPSW_ATTR(flow_control, S_IRUGO | S_IWUSR,
		cpsw_flow_control_show,
		cpsw_flow_control_store,
		cpsw_flow_controls);

static const struct cpswx_mod_info cpsw_port_tx_pri_maps[] = {
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

static ssize_t cpsw_port_tx_pri_map_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	int idx, len = 0, total_len = 0, port;
	struct cpswx_intf *cpsw_intf;
	struct cpswx_slave *slave;
	u32 reg;

	port = (int)(attr->context);

	/* Host port */
	if (port == cpsw_dev->host_port) {
		reg = __raw_readl(&cpsw_dev->host_port_regs->tx_pri_map);
		len = cpsw_attr_info_show(attr->info, attr->info_size,
					reg, buf);
		return len;
	}

	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num != port)
				continue;
			reg = __raw_readl(&slave->regs->tx_pri_map);
			len = cpsw_attr_info_show(attr->info, attr->info_size,
						reg, buf+total_len);
			total_len += len;
		} else {
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num != port)
					continue;
				reg = __raw_readl(&slave->regs->tx_pri_map);
				len = cpsw_attr_info_show(attr->info,
					attr->info_size, reg, buf+total_len);
				total_len += len;
			}
		}
	}
	return total_len;
}

static ssize_t cpsw_port_tx_pri_map_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpswx_mod_info *i;
	struct cpswx_parse_result res;
	struct cpswx_intf *cpsw_intf;
	struct cpswx_slave *slave;
	void __iomem *r = NULL;
	int ret, idx, port;

	port = (int)(attr->context);

	ret = cpsw_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	i = &(attr->info[res.control]);

	/* Host port */
	if (port == cpsw_dev->host_port) {
		r = &cpsw_dev->host_port_regs->tx_pri_map;
		goto set;
	}

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
	cpsw_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpswx_attribute cpsw_tx_pri_0_attribute =
	__CPSW_CTXT_ATTR(0, S_IRUGO | S_IWUSR,
			cpsw_port_tx_pri_map_show,
			cpsw_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)0);

static struct cpswx_attribute cpsw_tx_pri_1_attribute =
	__CPSW_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
			cpsw_port_tx_pri_map_show,
			cpsw_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)1);

static struct cpswx_attribute cpsw_tx_pri_2_attribute =
	__CPSW_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
			cpsw_port_tx_pri_map_show,
			cpsw_port_tx_pri_map_store,
			cpsw_port_tx_pri_maps, (void *)2);

static struct attribute *cpsw_tx_pri_default_attrs[] = {
	&cpsw_tx_pri_0_attribute.attr,
	&cpsw_tx_pri_1_attribute.attr,
	&cpsw_tx_pri_2_attribute.attr,
	NULL
};

static ssize_t cpsw_tx_pri_attr_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = tx_pri_to_cpswx_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw_tx_pri_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = tx_pri_to_cpswx_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_tx_pri_sysfs_ops = {
	.show = cpsw_tx_pri_attr_show,
	.store = cpsw_tx_pri_attr_store,
};

static struct kobj_type cpsw_tx_pri_ktype = {
	.sysfs_ops = &cpsw_tx_pri_sysfs_ops,
	.default_attrs = cpsw_tx_pri_default_attrs,
};

static const struct cpswx_mod_info cpsw_port_vlans[] = {
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

static ssize_t cpsw_port_vlan_show(struct cpswx_priv *cpsw_dev,
		     struct cpswx_attribute *attr,
		     char *buf)
{
	int idx, len = 0, total_len = 0, port;
	struct cpswx_intf *cpsw_intf;
	struct cpswx_slave *slave;
	u32 reg;

	port = (int)(attr->context);

	if (port == cpsw_dev->host_port) {
		/* Host port */
		reg = __raw_readl(&cpsw_dev->host_port_regs->port_vlan);
		len = cpsw_attr_info_show(attr->info, attr->info_size,
					reg, buf);
		return len;
	}

	/* Slave ports */
	for_each_intf(cpsw_intf, cpsw_dev) {
		if (cpsw_intf->multi_if) {
			slave = cpsw_intf->slaves;
			if (slave->port_num != port)
				continue;
			reg = __raw_readl(&slave->regs->port_vlan);
			len = cpsw_attr_info_show(attr->info, attr->info_size,
					reg, buf+total_len);
			total_len += len;
		} else {
			for (idx = 0; idx < cpsw_intf->num_slaves; idx++) {
				slave = cpsw_intf->slaves + idx;
				if (slave->port_num != port)
					continue;
				reg = __raw_readl(&slave->regs->port_vlan);
				len = cpsw_attr_info_show(attr->info,
					attr->info_size, reg, buf+total_len);
				total_len += len;
			}
		}
	}
	return total_len;
}

static ssize_t cpsw_port_vlan_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	const struct cpswx_mod_info *i;
	struct cpswx_parse_result res;
	struct cpswx_intf *cpsw_intf;
	struct cpswx_slave *slave;
	void __iomem *r = NULL;
	int ret, idx, port;

	port = (int)(attr->context);

	ret = cpsw_attr_parse_set_command(cpsw_dev, attr, buf, count, &res);
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
	cpsw_info_set_reg_field(r, i, res.value);
	return count;
}

static struct cpswx_attribute cpsw_pvlan_0_attribute =
	__CPSW_CTXT_ATTR(0, S_IRUGO | S_IWUSR,
			cpsw_port_vlan_show,
			cpsw_port_vlan_store,
			cpsw_port_vlans, (void *)0);

static struct cpswx_attribute cpsw_pvlan_1_attribute =
	__CPSW_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
			cpsw_port_vlan_show,
			cpsw_port_vlan_store,
			cpsw_port_vlans, (void *)1);

static struct cpswx_attribute cpsw_pvlan_2_attribute =
	__CPSW_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
			cpsw_port_vlan_show,
			cpsw_port_vlan_store,
			cpsw_port_vlans, (void *)2);

static struct attribute *cpsw_pvlan_default_attrs[] = {
	&cpsw_pvlan_0_attribute.attr,
	&cpsw_pvlan_1_attribute.attr,
	&cpsw_pvlan_2_attribute.attr,
	NULL
};

static ssize_t cpsw_pvlan_attr_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = pvlan_to_cpswx_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw_pvlan_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = pvlan_to_cpswx_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_pvlan_sysfs_ops = {
	.show = cpsw_pvlan_attr_show,
	.store = cpsw_pvlan_attr_store,
};

static struct kobj_type cpsw_pvlan_ktype = {
	.sysfs_ops = &cpsw_pvlan_sysfs_ops,
	.default_attrs = cpsw_pvlan_default_attrs,
};

static void cpsw_reset_mod_stats(struct cpswx_priv *cpsw_dev, int stat_mod)
{
	struct cpswx_host_hw_stats __iomem *cpsw_stats0;
	struct cpswx_hw_stats __iomem *cpsw_statsa;
	struct cpswx_hw_stats __iomem *cpsw_statsb;
	void __iomem *base;
	u32  __iomem *p;
	int i;

	cpsw_stats0 = cpsw_dev->host_hw_stats_regs;
	cpsw_statsa = cpsw_dev->hw_stats_regs[0];
	cpsw_statsb = cpsw_dev->hw_stats_regs[1];

	switch (stat_mod) {
	case CPSW_STATS0_MODULE:
		base = cpsw_stats0;
		break;
	case CPSW_STATS1_MODULE:
		base = cpsw_statsa;
		break;
	case CPSW_STATS2_MODULE:
		base  = cpsw_statsb;
		break;
	default:
		dev_err(cpsw_dev->dev, "Unknown stat module %d\n", stat_mod);
		return;
	}

	for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
		if (et_stats[i].type == stat_mod) {
			cpsw_dev->hw_stats[i] = 0;
			p = base + et_stats[i].offset;
			*p = 0xffffffff;
		}
	}
	return;
}

static ssize_t cpsw_stats_mod_store(struct cpswx_priv *cpsw_dev,
			      struct cpswx_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long end;
	int stat_mod;

	if (kstrtoul(buf, 0, &end) != 0 || (end != 0))
		return -EINVAL;

	stat_mod = (int)(attr->context);
	spin_lock_bh(&cpsw_dev->hw_stats_lock);
	cpsw_reset_mod_stats(cpsw_dev, stat_mod);
	spin_unlock_bh(&cpsw_dev->hw_stats_lock);
	return count;
}

static struct cpswx_attribute cpsw_stats_0_attribute =
	__CPSW_ATTR_FULL(0, S_IWUSR, NULL, cpsw_stats_mod_store,
			NULL, 0, (void *)CPSW_STATS0_MODULE);

static struct cpswx_attribute cpsw_stats_1_attribute =
	__CPSW_ATTR_FULL(1, S_IWUSR, NULL, cpsw_stats_mod_store,
			NULL, 0, (void *)CPSW_STATS1_MODULE);

static struct cpswx_attribute cpsw_stats_2_attribute =
	__CPSW_ATTR_FULL(2, S_IWUSR, NULL, cpsw_stats_mod_store,
			NULL, 0, (void *)CPSW_STATS2_MODULE);

static struct attribute *cpsw_stats_default_attrs[] = {
	&cpsw_stats_0_attribute.attr,
	&cpsw_stats_1_attribute.attr,
	&cpsw_stats_2_attribute.attr,
	NULL
};

static ssize_t cpsw_stats_attr_store(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = stats_to_cpswx_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_stats_sysfs_ops = {
	.store = cpsw_stats_attr_store,
};

static struct kobj_type cpsw_stats_ktype = {
	.sysfs_ops = &cpsw_stats_sysfs_ops,
	.default_attrs = cpsw_stats_default_attrs,
};
static struct attribute *cpsw_default_attrs[] = {
	&cpsw_version_attribute.attr,
	&cpsw_control_attribute.attr,
	&cpsw_pri_type_attribute.attr,
	&cpsw_flow_control_attribute.attr,
	&cpsw_prev_mode_attribute.attr,
	NULL
};

static ssize_t cpsw_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = to_cpswx_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(cpsw_dev, attribute, buf);
}

static ssize_t cpsw_attr_store(struct kobject *kobj, struct attribute *attr,
				   const char *buf, size_t count)
{
	struct cpswx_attribute *attribute = to_cpswx_attr(attr);
	struct cpswx_priv *cpsw_dev = to_cpswx_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(cpsw_dev, attribute, buf, count);
}

static const struct sysfs_ops cpsw_sysfs_ops = {
	.show = cpsw_attr_show,
	.store = cpsw_attr_store,
};

static struct kobj_type cpsw_ktype = {
	.sysfs_ops = &cpsw_sysfs_ops,
	.default_attrs = cpsw_default_attrs,
};

static void keystone_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strncpy(info->driver, NETCP_DRIVER_NAME, sizeof(info->driver));
	strncpy(info->version, NETCP_DRIVER_VERSION, sizeof(info->version));
}

static u32 keystone_get_msglevel(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	return netcp->msg_enable;
}

static void keystone_set_msglevel(struct net_device *ndev, u32 value)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	netcp->msg_enable = value;
}

static void keystone_get_stat_strings(struct net_device *ndev,
				   uint32_t stringset, uint8_t *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
			memcpy(data, et_stats[i].desc, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		break;
	}
}

static int keystone_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS:
		return ETHTOOL_STATS_NUM;
	default:
		return -EINVAL;
	}
}

static void cpswx_update_stats(struct cpswx_priv *cpsw_dev, uint64_t *data)
{
	struct cpswx_host_hw_stats __iomem *cpsw_stats0;
	struct cpswx_hw_stats __iomem *cpsw_statsa;
	struct cpswx_hw_stats __iomem *cpsw_statsb;
	void __iomem *base = NULL;
	u32  __iomem *p;
	u32 tmp = 0;
	int i;

	cpsw_stats0 = cpsw_dev->host_hw_stats_regs;
	cpsw_statsa = cpsw_dev->hw_stats_regs[0];
	cpsw_statsb = cpsw_dev->hw_stats_regs[1];

	for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
		switch (et_stats[i].type) {
		case CPSW_STATS0_MODULE:
			base = cpsw_stats0;
			break;
		case CPSW_STATS1_MODULE:
			base = cpsw_statsa;
			break;
		case CPSW_STATS2_MODULE:
			base = cpsw_statsb;
			break;
		default:
			dev_err(cpsw_dev->dev, "Unknown stat module %d\n",
				et_stats[i].type);
			return;
		}

		p = base + et_stats[i].offset;
		tmp = *p;
		cpsw_dev->hw_stats[i] = cpsw_dev->hw_stats[i] + tmp;
		if (data)
			data[i] = cpsw_dev->hw_stats[i];
		*p = tmp;
	}

	return;
}

static void keystone_get_ethtool_stats(struct net_device *ndev,
				       struct ethtool_stats *stats,
				       uint64_t *data)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct cpswx_priv *priv = netcp->cpswx_priv;

	if (priv) {
		spin_lock_bh(&priv->hw_stats_lock);
		cpswx_update_stats(priv, data);
		spin_unlock_bh(&priv->hw_stats_lock);
	}

	return;
}

static int keystone_get_link_ksettings(struct net_device *ndev,
				       struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phy = ndev->phydev;
	struct cpswx_slave *slave;

	if (!phy)
		return -EINVAL;

	slave = (struct cpswx_slave *)phy->priv;
	if (!slave)
		return -EINVAL;

	phy_ethtool_ksettings_get(phy, cmd);
	cmd->base.port = slave->phy_port_t;

	return 0;
}

static int keystone_set_link_ksettings(struct net_device *ndev,
				       const struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phy = ndev->phydev;
	struct cpswx_slave *slave;
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

	slave = (struct cpswx_slave *)phy->priv;
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

static const struct ethtool_ops keystone_ethtool_ops = {
	.get_drvinfo		= keystone_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= keystone_get_msglevel,
	.set_msglevel		= keystone_set_msglevel,
	.get_strings		= keystone_get_stat_strings,
	.get_sset_count		= keystone_get_sset_count,
	.get_ethtool_stats	= keystone_get_ethtool_stats,
	.get_link_ksettings     = keystone_get_link_ksettings,
	.set_link_ksettings     = keystone_set_link_ksettings,
};

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw_set_slave_mac(struct cpswx_slave *slave,
			       struct cpswx_intf *cpsw_intf)
{
	struct net_device *ndev = cpsw_intf->ndev;

	__raw_writel(mac_hi(ndev->dev_addr), &slave->regs->sa_hi);
	__raw_writel(mac_lo(ndev->dev_addr), &slave->regs->sa_lo);
}

static inline int cpsw_get_slave_port(struct cpswx_priv *priv, u32 slave_num)
{
	if (priv->host_port == 0)
		return slave_num + 1;
	else
		return slave_num;
}

static void _cpsw_adjust_link(struct cpswx_slave *slave, bool *link)
{
	struct phy_device *phy = slave->phy;
	u32 mac_control = 0;
	u32 slave_port;

	if (!phy)
		return;

	if (!slave->ale)
		return;

	slave_port = slave->port_num;

	if (phy->link) {
		mac_control = slave->mac_control;
		mac_control |= MACSL_DEFAULT_CONFIG(slave);
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
		__raw_writel(mac_control, &slave->sliver->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw_adjust_link(struct net_device *n_dev)
{
	struct cpswx_slave *slave = NULL;
	struct netcp_priv *netcp = netdev_priv(n_dev);
	bool link = false;

	if (n_dev->phydev)
		slave = n_dev->phydev->priv;
	if (!slave)
		return;

	_cpsw_adjust_link(slave, &link);

	if (link)
		netcp->phy_link_state_mask |= BIT(slave->slave_num);
	else
		netcp->phy_link_state_mask &= ~BIT(slave->slave_num);
}

/*
 * Reset the the mac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
static int cpsw_port_reset(struct cpswx_slave *slave)
{
	u32 i, v;

	/* Set the soft reset bit */
	__raw_writel(SOFT_RESET,
		     &slave->sliver->soft_reset);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(&slave->sliver->soft_reset);
		if ((v & SOFT_RESET_MASK) !=
		    SOFT_RESET)
			return 0;
	}

	/* Timeout on the reset */
	return GMACSL_RET_WARN_RESET_INCOMPLETE;
}

/*
 * Configure the mac sliver
 */
static void cpsw_port_config(struct cpswx_slave *slave, int max_rx_len)
{
	if (max_rx_len > MAX_SIZE_STREAM_BUFFER)
		max_rx_len = MAX_SIZE_STREAM_BUFFER;

	slave->mac_control = MACSL_DEFAULT_CONFIG(slave);

	__raw_writel(max_rx_len, &slave->sliver->rx_maxlen);

	__iowmb();
	__raw_writel(slave->mac_control, &slave->sliver->mac_control);
}

static void cpsw_slave_stop(struct cpswx_slave *slave,
			    struct cpswx_intf *cpsw_intf)
{
	u32 slave_port = cpsw_get_slave_port(slave->cpsw_dev, slave->slave_num);
	int cntr = 0;

	while (delayed_work_pending(&slave->link_recover_work)) {
		pr_warning("%s: work for iface %s (%d) still ongoing (%d)\n",
				__func__, netdev_name(cpsw_intf->ndev),
				slave->slave_num, cntr);
		if (cntr > CPSW_WAIT_FOR_WORK_DONE) {
			/* This is error by itself which can cause issues
			 * like wqueue still running, and deleting its
			 * objects */
			pr_err("%s: Work for slave (%d) never finished\n",
					__func__, slave->slave_num);
			break;
		}
		cntr++;
		msleep(100);
	}
	cpsw_ale_control_set(slave->cpsw_dev->ale, slave_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);
	cpsw_port_reset(slave);

	if (slave->link_recover_thresh ||
	    slave->link_recovering) {
		slave->link_recover_thresh = 0;
		slave->link_recovering = 0;
		dev_info(slave->cpsw_dev->dev,
			 "link_recover process cancelled: %s slave %d\n",
			 netdev_name(cpsw_intf->ndev), slave->slave_num);
	}

	if (!slave->phy)
		return;

	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

#define XGBE10_PCSR_OFFSET(x)           (x * 0x80)
#define XGBE10_PCSR_RX_STATUS(x)        (XGBE10_PCSR_OFFSET(x) + 0x0C)

static void cpsw_slave_link(struct cpswx_slave *slave,
			    struct cpswx_intf *cpsw_intf)
{
	struct netcp_priv *netcp = netdev_priv(cpsw_intf->ndev);
	struct cpswx_priv *priv = cpsw_intf->cpsw_priv;
	int sn = slave->slave_num;
	u32 pcsr_rx_blk_lock;

	if ((slave->link_interface == SGMII_LINK_MAC_PHY) ||
		(slave->link_interface == XGMII_LINK_MAC_PHY)) {
		/* check only the bit in phy_link_state_mask
		 * that corresponds to the slave
		 */
		if (!(netcp->phy_link_state_mask & BIT(sn)))
			cpsw_intf->link_state &= ~BIT(sn);
	} else if (slave->link_interface == XGMII_LINK_MAC_MAC_FORCED) {
		pcsr_rx_blk_lock = readl(priv->pcsr_port_regs +
					 XGBE10_PCSR_RX_STATUS(sn));
		pcsr_rx_blk_lock = ((pcsr_rx_blk_lock & BIT(30)) >> 30) << sn;
		if (cpsw_intf->link_state != pcsr_rx_blk_lock)
			cpsw_intf->link_state = pcsr_rx_blk_lock;
	}
}

static void cpswx_slave_link_recover(struct work_struct *work)
{
	struct cpswx_slave *slave = container_of(work, struct cpswx_slave,
					       link_recover_work.work);

	cpsw_serdes_lane_reset(slave->cpsw_dev, slave->slave_num);
}

static void cpsw_slave_open(struct cpswx_slave *slave,
			    struct cpswx_intf *cpsw_intf)
{
	struct cpswx_priv *priv = cpsw_intf->cpsw_priv;
	char name[32];		/* FIXME: Unused variable */
	u32 slave_port;
	int has_phy = 0;
	phy_interface_t phy_mode;

	snprintf(name, sizeof(name), "slave-%d", slave->slave_num);

	if (!SLAVE_LINK_IS_XGMII(slave)) {
		keystone_sgmii_reset(priv->sgmii_port_regs, slave->slave_num);

		keystone_sgmii_config(priv->sgmii_port_regs, slave->slave_num,
				slave->link_interface);
	} else
		keystone_pcsr_config(priv->pcsr_port_regs, slave->slave_num,
				slave->link_interface);

	cpsw_port_reset(slave);

	cpsw_port_config(slave, priv->rx_packet_max);

	cpsw_set_slave_mac(slave, cpsw_intf);

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	slave->port_num = slave_port;
	slave->ale = priv->ale;

	/* enable forwarding */
	cpsw_ale_control_set(priv->ale, slave_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_mcast(priv->ale, cpsw_intf->ndev->broadcast,
			   1 << slave_port, 0, 0, ALE_MCAST_FWD_2);

	if (slave->link_interface == SGMII_LINK_MAC_PHY) {
		has_phy = 1;
		phy_mode = PHY_INTERFACE_MODE_SGMII;
		slave->phy_port_t = PORT_MII;
	} else if (slave->link_interface == XGMII_LINK_MAC_PHY) {
		has_phy = 1;
		/* +++FIXME: PHY_INTERFACE_MODE_XGMII ?? */
		phy_mode = PHY_INTERFACE_MODE_NA;
		slave->phy_port_t = PORT_FIBRE;
	} else if (slave->link_interface == XGMII_LINK_MAC_MAC_FORCED) {
		cpsw_intf->link_state = XGMII_LINK_STATE_INVALID;
		INIT_DELAYED_WORK(&slave->link_recover_work,
				  cpswx_slave_link_recover);
	}

	if (has_phy) {
		slave->phy = of_phy_connect(cpsw_intf->ndev,
					    cpsw_intf->phy_node,
					    &cpsw_adjust_link, 0,
					    phy_mode);
		if (IS_ERR_OR_NULL(slave->phy)) {
			dev_err(priv->dev, "phy not found on slave %d\n",
				slave->slave_num);
			slave->phy = NULL;
		} else {
			/* set phy_dev->priv for adjust_link handler */
			slave->phy->priv = slave;
			dev_info(priv->dev, "phy found: id is: %s, drv: %s\n",
				 phydev_name(slave->phy),
				 (slave->phy->drv ?
				   (slave->phy->drv->name ?
					slave->phy->drv->name : "") : ""));
			cpsw_intf->ndev->phydev = slave->phy;
			phy_start(slave->phy);
		}
	}
	/* on else we do not have phy pointer, so we can`t assign the
	 * slave to the priv pointer
	 */
}

static int cpsw_init_ale(struct cpswx_priv *cpsw_dev,
				struct cpswx_intf *cpsw_intf)
{
	struct cpsw_ale_params ale_params;

	memset(&ale_params, 0, sizeof(ale_params));

	ale_params.dev		= cpsw_dev->dev;
	ale_params.ale_regs	= (void *)((u32)cpsw_dev->ale_reg);
	ale_params.ale_ageout	= cpsw_dev->ale_ageout;
	ale_params.ale_entries	= cpsw_dev->ale_entries;
	ale_params.ale_ports	= cpsw_dev->ale_ports;

	cpsw_dev->ale = cpsw_ale_create(&ale_params);
	if (!cpsw_dev->ale) {
		dev_err(cpsw_dev->dev, "error initializing ale engine\n");
		return -ENODEV;
	}

	dev_info(cpsw_dev->dev, "Created a cpsw ale engine\n");
	
	cpsw_ale_start(cpsw_dev->ale);

	cpsw_ale_control_set(cpsw_dev->ale, 0, ALE_BYPASS,
			cpsw_dev->multi_if ? 1 : 0);

	cpsw_ale_control_set(cpsw_dev->ale, 0, ALE_NO_PORT_VLAN, 1);

	cpsw_ale_control_set(cpsw_dev->ale, cpsw_dev->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_VLAN_MEMBER,
			     CPSW_MASK_ALL_PORTS);

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_MCAST_FLOOD,
			     CPSW_MASK_PHYS_PORTS);

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNKNOWN_REG_MCAST_FLOOD,
			     CPSW_MASK_ALL_PORTS);

	cpsw_ale_control_set(cpsw_dev->ale, 0,
			     ALE_PORT_UNTAGGED_EGRESS,
			     CPSW_MASK_ALL_PORTS);

	return 0;
}

static void cpsw_init_host_port(struct cpswx_priv *priv,
				struct cpswx_intf *cpsw_intf)
{
	/* Host Tx Pri */
	__raw_writel(HOST_TX_PRI_MAP_DEFAULT,
		     &priv->host_port_regs->tx_pri_map);

	/* Max length register */
	__raw_writel(MAX_SIZE_STREAM_BUFFER,
		     &priv->host_port_regs->rx_maxlen);
}

static void cpsw_slave_init(struct cpswx_slave *slave, struct cpswx_priv *priv)
{
	void __iomem		*regs = priv->ss_regs;
	int			slave_num = slave->slave_num;

	slave->regs	= regs + priv->slave_reg_ofs + (0x30 * slave_num);
	slave->sliver	= regs + priv->sliver_reg_ofs + (0x40 * slave_num);
	slave->cpsw_dev = priv;
}

static void cpsw_add_mcast_addr(struct cpswx_intf *cpsw_intf, u8 *addr)
{
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_add_mcast(cpsw_dev->ale, addr, CPSW_MASK_ALL_PORTS, 0, 0,
			   ALE_MCAST_FWD_2);
}

static void cpsw_add_ucast_addr(struct cpswx_intf *cpsw_intf, u8 *addr)
{
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_add_ucast(cpsw_dev->ale, addr, cpsw_dev->host_port, 0, 0);
}

static void cpsw_del_mcast_addr(struct cpswx_intf *cpsw_intf, u8 *addr)
{
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_del_mcast(cpsw_dev->ale, addr, 0, 0, 0);
}

static void cpsw_del_ucast_addr(struct cpswx_intf *cpsw_intf, u8 *addr)
{
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	cpsw_ale_del_ucast(cpsw_dev->ale, addr, cpsw_dev->host_port, 0, 0);
}

static int cpswx_add_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct cpswx_intf *cpsw_intf = intf_priv;
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	if (!cpsw_dev->opened)
		return -ENXIO;

	dev_dbg(cpsw_dev->dev, "xgess adding address %pM, type %d\n",
		naddr->addr, naddr->type);

	switch (naddr->type) {
	case ADDR_MCAST:
	case ADDR_BCAST:
		cpsw_add_mcast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_UCAST:
	case ADDR_DEV:
		cpsw_add_ucast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_ANY:
		/* nothing to do for promiscuous */
	default:
		break;
	}

	return 0;
}

static int cpswx_del_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct cpswx_intf *cpsw_intf = intf_priv;
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;

	if (!cpsw_dev->opened)
		return -ENXIO;

	dev_dbg(cpsw_dev->dev, "xgess deleting address %pM, type %d\n",
		naddr->addr, naddr->type);

	switch (naddr->type) {
	case ADDR_MCAST:
	case ADDR_BCAST:
		cpsw_del_mcast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_UCAST:
	case ADDR_DEV:
		cpsw_del_ucast_addr(cpsw_intf, naddr->addr);
		break;
	case ADDR_ANY:
		/* nothing to do for promiscuous */
	default:
		break;
	}

	return 0;
}

static int cpswx_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	struct cpswx_intf *cpsw_intf = intf_priv;
	struct cpswx_slave *slave = cpsw_intf->slaves;
	struct phy_device *phy = slave->phy;
	int ret;

	if (!phy)
		return -EOPNOTSUPP;

	ret = phy_mii_ioctl(phy, req, cmd);
	if ((cmd == SIOCSHWTSTAMP) && (ret == -ERANGE))
		ret = -EOPNOTSUPP;

	return ret;
}

/* Number of GBE_TIMER_INTERVAL */
#define LINK_RECOVER_THRESHOLD	6

extern int k2serdes_check_xge_link_status(void __iomem *serdes,
		void __iomem *sw_regs, u32 lane, int print);

extern void kserdes_relatch_att_boost(struct kserdes_config *sc, u32 lane);

static void cpswx_timer(struct timer_list *t)
{
	struct cpswx_intf *cpsw_intf = from_timer(cpsw_intf, t, timer);
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	u32 old_link_state = cpsw_intf->link_state;
	struct net_device *ndev = cpsw_intf->ndev;
	/* Assumes MULTI-INTERFACE */
	struct cpswx_slave *slave = cpsw_intf->slaves;
	int xge_link;
	struct cpswx_serdes_priv *spriv = &cpsw_dev->serdes_priv;
	struct kserdes_config *sconfig;

	sconfig = &spriv->serdes_config[0];
	/*
	 * if the slave's link_interface is not XGMII, link_state bit
	 * will not be set
	 */
	if (cpsw_dev->multi_if)
		cpsw_intf->link_state =
			keystone_sgmii_get_port_link(cpsw_dev->sgmii_port_regs,
						     cpsw_intf->slave_port);
	else
		cpsw_intf->link_state =
			keystone_sgmii_link_status(cpsw_dev->sgmii_port_regs,
						   cpsw_intf->num_slaves);

	/* if MAC-to-PHY, check phy link status also
	 * to conclude the intf link's status
	 */
	for_each_slave(cpsw_intf, cpsw_slave_link, cpsw_intf);

	kserdes_relatch_att_boost(sconfig, cpsw_intf->slave_port);

	xge_link = k2serdes_check_xge_link_status(
			sconfig->regs,sconfig->sw_regs,
			cpsw_intf->slave_port,
			0);

	if (!xge_link) {
		if (cpsw_intf->link_state) {
			pr_err("%s: sgmii link up, but xge showed down, execute recoverry!\n", __func__);
			cpsw_intf->link_state = 0;
			xge_link = k2serdes_check_xge_link_status(
				sconfig->regs,sconfig->sw_regs,
				cpsw_intf->slave_port, 1);
		}
	}

	/* FIXME: Don't aggregate link statuses in multi-interface case */
	if (cpsw_intf->link_state != old_link_state) {
		netdev_printk(KERN_INFO, cpsw_intf->ndev,
			      "Link is %s\n",
			      (cpsw_intf->link_state ? "Up" : "Down"));

		if (cpsw_intf->link_state) {
			/* link ON */
			if (!netif_carrier_ok(cpsw_intf->ndev))
				netif_carrier_on(cpsw_intf->ndev);

			if (slave->link_recover_thresh ||
			    slave->link_recovering) {
				slave->link_recover_thresh = 0;
				slave->link_recovering = 0;
				dev_info(slave->cpsw_dev->dev,
					 "link_recover process cancelled: %s slave %d\n",
					 netdev_name(ndev), slave->slave_num);
			}
		} else {
			/* link OFF */
			if (netif_carrier_ok(cpsw_intf->ndev))
				netif_carrier_off(cpsw_intf->ndev);

			/* from up to down */
			slave->link_recover_thresh = LINK_RECOVER_THRESHOLD;
			slave->link_recovering = 1;
			dev_info(slave->cpsw_dev->dev,
				 "link_recover process initiated: %s slave %d\n",
				 netdev_name(ndev), slave->slave_num);
		}
	} else {
		if (slave->link_recover_thresh) {
			if (++slave->link_recovering >=
					slave->link_recover_thresh) {
				schedule_delayed_work(&slave->link_recover_work,
						      0);
				slave->link_recovering = 1;
			}
		}
	}

	/* A timer runs as a BH, no need to block them */
	spin_lock(&cpsw_dev->hw_stats_lock);
	cpswx_update_stats(cpsw_dev, NULL);
	spin_unlock(&cpsw_dev->hw_stats_lock);

	cpsw_intf->timer.expires = jiffies + CPSW_TIMER_INTERVAL;
	add_timer(&cpsw_intf->timer);

	return;
}

static int cpsw_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct cpswx_intf *cpsw_intf = data;

	p_info->tx_pipe = &cpsw_intf->tx_pipe;
	return 0;
}

#define	CPSW_TXHOOK_ORDER	0

static int cpswx_open(void *intf_mod_priv, struct net_device *ndev)
{
	struct cpswx_intf *cpsw_intf = intf_mod_priv;
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	struct netcp_priv *netcp = netdev_priv(ndev);
	u32 xgmii_mode = 0;
	int ret = 0;
	u32 reg, i;

	cpsw_dev->clk = clk_get(cpsw_dev->dev, "clk_xge");
	if (IS_ERR(cpsw_dev->clk)) {
		ret = PTR_ERR(cpsw_dev->clk);
		cpsw_dev->clk = NULL;
		dev_err(cpsw_dev->dev,
			"unable to get Keystone XGE clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(cpsw_dev->clk);
	if (ret)
		goto clk_fail;

	reg = __raw_readl(&cpsw_dev->regs->id_ver);

	dev_info(cpsw_dev->dev,
	 "initialize cpsw version %d.%d (%d) CPSW identification value 0x%x\n",
	 CPSW_MAJOR_VERSION(reg), CPSW_MINOR_VERSION(reg),
	 CPSW_RTL_VERSION(reg), CPSW_IDENT(reg));

	ret = netcp_txpipe_open(&cpsw_intf->tx_pipe);
	if (ret)
		goto txpipe_fail;

	dev_dbg(cpsw_dev->dev, "opened TX channel %s: %p\n",
		cpsw_intf->tx_pipe.dma_chan_name,
		cpsw_intf->tx_pipe.dma_channel);

	/* initialize host and slave ports */
	if (atomic_inc_return(&cpsw_dev->ale_refcnt) == 1) {
		ret = cpsw_init_ale(cpsw_dev, cpsw_intf);
		if (ret < 0) {
			atomic_dec(&cpsw_dev->ale_refcnt);
			goto ale_fail;
		}
		cpsw_init_host_port(cpsw_dev, cpsw_intf);
	}

	for_each_slave(cpsw_intf, cpsw_slave_init, cpsw_dev);

	for_each_slave(cpsw_intf, cpsw_slave_stop, cpsw_intf);

	/* Enable correct MII mode at SS level */
	for (i = 0; i < cpsw_dev->num_slaves; i++)
		if (cpsw_dev->link[i] >= XGMII_LINK_MAC_PHY)
			xgmii_mode |= (1 << i);
	__raw_writel(xgmii_mode, &cpsw_dev->ss_regs->control);

	/* disable priority elevation and enable statistics on all ports */
	__raw_writel(0, &cpsw_dev->regs->ptype);

	/* Control register */
	__raw_writel(CPSW_CTL_P0_ENABLE, &cpsw_dev->regs->control);

	/* All statistics enabled by default */
	__raw_writel(CPSW_REG_VAL_STAT_ENABLE_ALL,
		     &cpsw_dev->regs->stat_port_en);

	for_each_slave(cpsw_intf, cpsw_slave_open, cpsw_intf);

	timer_setup(&cpsw_intf->timer, cpswx_timer, 0);
	mod_timer(&cpsw_intf->timer, jiffies + CPSW_TIMER_INTERVAL);
	dev_dbg(cpsw_dev->dev,
		"%s(): cpswx_timer = %p\n", __func__, cpswx_timer);

	netcp_register_txhook(netcp, CPSW_TXHOOK_ORDER,
			      cpsw_tx_hook, cpsw_intf);

#if 0
	/* Configure the streaming switch */
#define	PSTREAM_ROUTE_DMA	6
	netcp_set_streaming_switch(cpsw_dev->netcp_device, netcp->cpsw_port,
				   PSTREAM_ROUTE_DMA);
#endif

	cpsw_dev->opened = 1;
	return 0;

ale_fail:
	netcp_txpipe_close(&cpsw_intf->tx_pipe);
txpipe_fail:
	clk_disable_unprepare(cpsw_dev->clk);
clk_fail:
	cpsw_dev->clk = NULL;
	return ret;
}

static int cpswx_close(void *intf_modpriv, struct net_device *ndev)
{
	struct cpswx_intf *cpsw_intf = intf_modpriv;
	struct cpswx_priv *cpsw_dev = cpsw_intf->cpsw_priv;
	struct netcp_priv *netcp = netdev_priv(ndev);

	del_timer_sync(&cpsw_intf->timer);

	for_each_slave(cpsw_intf, cpsw_slave_stop, cpsw_intf);

	if (atomic_dec_return(&cpsw_dev->ale_refcnt) == 0) {
		cpsw_ale_stop(cpsw_dev->ale);
		cpsw_dev->ale = NULL;
	}

	netcp_unregister_txhook(netcp, CPSW_TXHOOK_ORDER, cpsw_tx_hook,
				cpsw_intf);
	netcp_txpipe_close(&cpsw_intf->tx_pipe);

	clk_disable_unprepare(cpsw_dev->clk);

	cpsw_dev->opened = 0;
	return 0;
}

static int cpswx_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct cpswx_priv *cpsw_dev = inst_priv;
	struct cpswx_intf *cpsw_intf, *tmp;
	struct device *dev = cpsw_dev->dev;

	of_node_put(cpsw_dev->interfaces);

	list_for_each_entry_safe(cpsw_intf, tmp, &cpsw_dev->cpsw_intf_head,
				 cpsw_intf_list) {
		netcp_delete_interface(netcp_device, cpsw_intf->ndev);
	}
	BUG_ON(!list_empty(&cpsw_dev->cpsw_intf_head));

	iounmap(cpsw_dev->ss_regs);
	memset(cpsw_dev, 0x00, sizeof(*cpsw_dev));	/* FIXME: Poison */
	devm_kfree(dev, cpsw_dev);
	return 0;
}

static int init_slave(struct cpswx_priv *cpsw_dev,
		      struct device_node *node, int slave_num)
{
	int ret = 0;

	ret = of_property_read_u32(node, "link-interface",
				   &cpsw_dev->link[slave_num]);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing link-interface value"
			"defaulting to mac-phy link\n");
		cpsw_dev->link[slave_num] = XGMII_LINK_MAC_PHY;
	}

	if (cpsw_dev->link[slave_num] == XGMII_LINK_MAC_PHY)
		cpsw_dev->phy_node[slave_num] =
			of_parse_phandle(node, "phy-handle", 0);

	return 0;
}

static int cpsw_create_sysfs_entries(struct cpswx_priv *cpsw_dev)
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
#define XGE_SERDES_BASE		0x0231e000
#define XGE_SERDES_SIZE		0x2000
#define XGE_SW_BASE		0x02f00000
#define XGE_SW_SIZE		0x00001000

/* returns number of serdeses configured */
static int cpswx_serdes_init(struct cpswx_priv *cpsw_dev)
{
	struct cpswx_serdes_priv *spriv = &cpsw_dev->serdes_priv;
	struct kserdes_config *sconfig;
	int i, j, serdes_configured = 0, ret = 0;

	spin_lock_init(&spriv->tbus_lock);

	for (i = 0; i < spriv->num_serdes; i++) {
		sconfig = &spriv->serdes_config[i];
		sconfig->tbus_lock = &spriv->tbus_lock;

		sconfig->regs = ioremap(XGE_SERDES_BASE, XGE_SERDES_SIZE);
		if (sconfig->regs == NULL)
			pr_err("%s: ioremap failed for regs\n", __func__);

		sconfig->sw_regs = ioremap(XGE_SW_BASE, XGE_SW_SIZE);
		if (sconfig->sw_regs == NULL)
			pr_err("%s: ioremap failed for sw_regs\n", __func__);

		ret = kserdes_provider_init(sconfig);

		if (ret < 0) {
			dev_err(cpsw_dev->dev,
				"cpswx serdes %d initialization failed: %d\n",
				i, ret);
			/* don't break, continue to the next serdes */
			continue;
		}

		for (j = 0; j < sconfig->lanes; j++) {
			if (!sconfig->lane[j].enable)
				continue;

			kserdes_phy_enable_rx(sconfig, j);
		}

		++serdes_configured;
	}

	return serdes_configured;
}

int cpswx_get_serdeses_properties(struct device *dev,
				  struct cpswx_serdes_priv *spriv,
				  struct device_node *node,
				  void __iomem *sw_ss_regs,
				  void __iomem *pcsr_port_regs)
{
	struct device_node *serdeses, *serdes;
	struct kserdes_config *sc;
	int ret;
	u32 i;

	serdeses = of_get_child_by_name(node, "serdeses");
	if (!serdeses) {
		dev_err(dev, "could not find serdeses node\n");
		return -ENODEV;
	}

	if (of_property_read_u32(serdeses, "serdes-at-probe",
				   &spriv->init_serdes_at_probe)) {
		dev_info(dev, "use default serdes-at-probe: 1\n");
		spriv->init_serdes_at_probe = 1;
	}
	dev_dbg(dev, "serdes-at-probe %u\n", spriv->init_serdes_at_probe);

	i = 0;
	for_each_child_of_node(serdeses, serdes) {
		sc = &spriv->serdes_config[i];

		ret = kserdes_of_parse(dev, sc, sw_ss_regs,
		                       pcsr_port_regs, serdes);
		if (ret)
			return -EINVAL;

		if (++i >= CPSWX_SERDES_MAX_NUM)
			break;
	}

	spriv->num_serdes = i;
	of_node_put(serdeses);
	dev_info(dev, "number of serdeses %u\n", spriv->num_serdes);
	return 0;
}

#define MDCTL50 0x02350AC8

static void disable_reset_isolation(void)
{
	u32 val;
	void __iomem *mdctl50_reg = ioremap(MDCTL50, 4);

	if (!mdctl50_reg) {
		pr_err("%s: Could not iomap reset isolation register\n", __func__);
		return;
	}
	val = __raw_readl(mdctl50_reg);
	if (val & BIT(12)) {
		val = val & ~BIT(12);
		__raw_writel(val, mdctl50_reg);
		pr_info("%s: KERN_10GE: Disabling XGE reset isolation, MDCTL50 = 0x%x\n", __func__, val);
	} else {
		pr_info("%s: KERN_10GE: Reset isolation was already disabled, MDCTL50 = 0x%x\n", __func__, val);
	}
	iounmap(mdctl50_reg);
	return;
}

static int cpswx_probe(struct netcp_device *netcp_device,
			struct device *dev,
			struct device_node *node,
			void **inst_priv)
{
	struct cpswx_priv *cpsw_dev;
	struct device_node *slaves, *slave, *interfaces;
	void __iomem *regs = NULL;
	struct net_device *ndev;
	int slave_num = 0;
	int i, pin, ret = 0;
	u32 temp[4];

	disable_reset_isolation();

	cpsw_dev = devm_kzalloc(dev, sizeof(struct cpswx_priv), GFP_KERNEL);
	if (!cpsw_dev) {
		dev_err(dev, "cpsw_dev memory allocation failed\n");
		return -ENOMEM;
	}

	*inst_priv = cpsw_dev;
	dev_dbg(dev, "%s(): cpsw_priv = %p\n", __func__, cpsw_dev);

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	cpsw_dev->dev = dev;
	cpsw_dev->netcp_device = netcp_device;

	ret = of_property_read_u32(node, "serdes_at_probe",
				   &cpsw_dev->init_serdes_at_probe);
	if (ret < 0) {
		dev_err(dev,
			"missing serdes_at_probe parameter, err %d\n", ret);
		cpsw_dev->init_serdes_at_probe = 0;
	}
	dev_dbg(dev, "serdes_at_probe %u\n", cpsw_dev->init_serdes_at_probe);

	ret = of_property_read_u32(node, "sgmii_module_ofs",
				   &cpsw_dev->sgmii_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing sgmii module offset, err %d\n", ret);

	ret = of_property_read_u32(node, "pcsr_module_ofs",
				   &cpsw_dev->pcsr_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing pcsr module offset, err %d\n", ret);

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

	ret = of_property_read_u32(node, "sliver_reg_ofs",
				   &cpsw_dev->sliver_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing sliver reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "hw_stats_reg_ofs",
				   &cpsw_dev->hw_stats_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing hw stats reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "ale_reg_ofs",
				   &cpsw_dev->ale_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing ale reg offset, err %d\n", ret);


	ret = of_property_read_u32(node, "num_slaves", &cpsw_dev->num_slaves);
	if (ret < 0) {
		dev_err(dev, "missing num_slaves parameter, err %d\n", ret);
		cpsw_dev->num_slaves = 2;
	}

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

	ret = of_property_read_u32(node,
				   "intf_tx_queues", &cpsw_dev->intf_tx_queues);
	if (ret < 0) {
		dev_err(dev, "missing intf_tx_queues parameter, err %d\n", ret);
		cpsw_dev->intf_tx_queues = 1;
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

	pin = of_get_named_gpio(node,"xfi_enable_gpio", 0);

	if (gpio_is_valid(pin)) {
		if (gpio_request(pin, "xfi_enable_gpio")) {
			dev_err(dev, "gpio %d request failed!\n", pin);
		} else {
			ret = gpio_direction_output(pin, 1);
			if (ret < 0)
				dev_err(dev, "gpio direction fail, err %d\n", ret);
			else
				dev_info(dev, "XFI clock enabled (gpio: %d)\n", pin);

			gpio_free(pin);
		}
	}
	/* Sub-sys regs base */
	if (of_property_read_u32_array(node, "reg", (u32 *)&(temp[0]), 2))
		dev_err(dev, "No reg defined\n");
	else
		regs = ioremap(temp[0], temp[1]);
	BUG_ON(!regs);

	cpsw_dev->ss_regs = regs;
	cpsw_dev->sgmii_port_regs	= regs + cpsw_dev->sgmii_module_ofs;
	cpsw_dev->pcsr_port_regs	= regs + cpsw_dev->pcsr_module_ofs;
	cpsw_dev->regs			= regs + cpsw_dev->switch_module_ofs;
	cpsw_dev->host_port_regs	= regs + cpsw_dev->host_port_reg_ofs;
	cpsw_dev->host_hw_stats_regs	= regs + cpsw_dev->hw_stats_reg_ofs;
	cpsw_dev->hw_stats_regs[0] = regs + cpsw_dev->hw_stats_reg_ofs + 0x100;
	cpsw_dev->hw_stats_regs[1] = regs + cpsw_dev->hw_stats_reg_ofs + 0x200;
	cpsw_dev->ale_reg		= regs + cpsw_dev->ale_reg_ofs;

	ret = of_property_read_u32(node, "host_port", &cpsw_dev->host_port);
	if (ret < 0) {
		dev_err(dev, "missing host_port parameter\n");
		cpsw_dev->host_port = 0;
	}
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

	ret = cpswx_get_serdeses_properties(dev, &cpsw_dev->serdes_priv,
	                                    node, cpsw_dev->ss_regs,
	                                    cpsw_dev->pcsr_port_regs);
	if (ret)
		goto exit;

	if (cpsw_dev->init_serdes_at_probe == 1) {
		cpsw_dev->clk = clk_get(cpsw_dev->dev, "clk_xge");
		if (IS_ERR(cpsw_dev->clk)) {
			ret = PTR_ERR(cpsw_dev->clk);
			cpsw_dev->clk = NULL;
			dev_err(cpsw_dev->dev,
				"unable to get Keystone XGE clock: %d\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(cpsw_dev->clk);
		if (ret)
			goto exit;

		/* needs the serdes pll to acces switch regs */
		ret = cpswx_serdes_init(cpsw_dev);
		if (ret != cpsw_dev->serdes_priv.num_serdes)
			goto exit;
	}

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

	/* init the hw stats lock */
	spin_lock_init(&cpsw_dev->hw_stats_lock);

	ret = cpsw_create_sysfs_entries(cpsw_dev);
	if (ret)
		goto exit;

	return 0;

exit:
	if (cpsw_dev->ss_regs)
		iounmap(cpsw_dev->ss_regs);
	*inst_priv = NULL;
	devm_kfree(dev, cpsw_dev);
	return ret;
}

static int cpswx_attach_serdes(struct cpswx_slave *slave,
			       struct cpswx_intf *cpsw_intf)
{
	phy_interface_t phy_mode;
	int has_phy = 0;
	int ret = 0;

	if (slave->link_interface == SGMII_LINK_MAC_PHY) {
		has_phy = 1;
		phy_mode = PHY_INTERFACE_MODE_SGMII;
		slave->phy_port_t = PORT_MII;
	} else if (slave->link_interface == XGMII_LINK_MAC_PHY) {
		has_phy = 1;
		/* +++FIXME: PHY_INTERFACE_MODE_XGMII ?? */
		phy_mode = PHY_INTERFACE_MODE_NA;
		slave->phy_port_t = PORT_FIBRE;
	}

	if (has_phy) {
		/* init the PHY to facilitate serdes link training */
		slave->phy = of_phy_connect(cpsw_intf->ndev,
					    cpsw_intf->phy_node,
					    &cpsw_adjust_link, 0,
					    phy_mode);
	}

	return ret;
}

static int cpswx_attach(void *inst_priv, struct net_device *ndev,
		       void **intf_priv)
{
	struct cpswx_priv *cpsw_dev = inst_priv;
	struct cpswx_intf *cpsw_intf;
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct device_node *interface;
	int i = 0, ret = 0;
	char node_name[24];

	cpsw_intf = devm_kzalloc(cpsw_dev->dev,
				 sizeof(struct cpswx_intf), GFP_KERNEL);
	if (!cpsw_intf) {
		dev_err(cpsw_dev->dev,
			"cpswx interface memory allocation failed\n");
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
		dev_err(cpsw_dev->dev,
			"missing tx-channel parameter, err %d\n", ret);
		cpsw_intf->tx_chan_name = "nettx";
	}
	dev_info(cpsw_dev->dev, "dma_chan_name %s\n", cpsw_intf->tx_chan_name);

	ret = of_property_read_u32(interface, "tx_queue_depth",
				   &cpsw_intf->tx_queue_depth);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing tx_queue_depth parameter, err %d\n", ret);
		cpsw_intf->tx_queue_depth = 32;
	}
	dev_dbg(cpsw_dev->dev, "tx_queue_depth %u\n",
		cpsw_intf->tx_queue_depth);

	of_node_put(interface);

	cpsw_intf->num_slaves = cpsw_dev->slaves_per_interface;

	cpsw_intf->slaves = devm_kzalloc(cpsw_dev->dev,
					 sizeof(struct cpswx_slave) *
					 cpsw_intf->num_slaves, GFP_KERNEL);

	if (!cpsw_intf->slaves) {
		dev_err(cpsw_dev->dev,
			"cpsw interface slave memory allocation failed\n");
		devm_kfree(cpsw_dev->dev, cpsw_intf);
		return -ENOMEM;
	}

	if (cpsw_dev->multi_if) {
		cpsw_intf->slaves[i].slave_num = cpsw_intf->slave_port;
		cpsw_intf->slaves[i].link_interface =
			cpsw_dev->link[cpsw_intf->slave_port];
		cpsw_intf->phy_node = cpsw_dev->phy_node[cpsw_intf->slave_port];
	} else {
		for (i = 0; i < cpsw_intf->num_slaves; i++) {
			cpsw_intf->slaves[i].slave_num = i;
			cpsw_intf->slaves[i].link_interface = cpsw_dev->link[i];
		}
	}

	netcp_txpipe_init(&cpsw_intf->tx_pipe, netdev_priv(ndev),
			  cpsw_intf->tx_chan_name, cpsw_intf->tx_queue_depth);

	ndev->ethtool_ops = &keystone_ethtool_ops;

	list_add(&cpsw_intf->cpsw_intf_list, &cpsw_dev->cpsw_intf_head);

	*intf_priv = cpsw_intf;

	netcp->cpswx_priv = inst_priv;

	for_each_slave(cpsw_intf, cpswx_attach_serdes, cpsw_intf);
	return ret;
}

static int cpswx_release(void *intf_modpriv)
{
	struct cpswx_intf *cpsw_intf = intf_modpriv;

	cpsw_intf->ndev->ethtool_ops = NULL;

	list_del(&cpsw_intf->cpsw_intf_list);

	netif_napi_del(&cpsw_intf->tx_pipe.dma_poll_napi);

	devm_kfree(cpsw_intf->dev, cpsw_intf->slaves);
	devm_kfree(cpsw_intf->dev, cpsw_intf);

	return 0;
}


static struct netcp_module cpsw_module = {
	.name		= CPSW_MODULE_NAME,
	.owner		= THIS_MODULE,
	.probe		= cpswx_probe,
	.open		= cpswx_open,
	.close		= cpswx_close,
	.remove		= cpswx_remove,
	.attach		= cpswx_attach,
	.release	= cpswx_release,
	.add_addr	= cpswx_add_addr,
	.del_addr	= cpswx_del_addr,
	.ioctl		= cpswx_ioctl,
};

int __init keystone_cpswx_init(void)
{
	return netcp_register_module(&cpsw_module);
}

void __exit keystone_cpswx_exit(void)
{
	netcp_unregister_module(&cpsw_module);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Paulraj <s-paulraj@ti.com>");
MODULE_DESCRIPTION("CPSW driver for Keystone 10GE devices");
