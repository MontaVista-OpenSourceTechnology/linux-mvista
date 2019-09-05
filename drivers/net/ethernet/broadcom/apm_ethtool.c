#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/phy.h>

#include "pm.h"
#include "apm.h"

static const struct {
	const char name[ETH_GSTRING_LEN];
} apm_stat_name[] = {
	{ "rx_frames" },
	{ "rx_frame_good" },
	{ "rx_bytes" },
	{ "rx_frame_64" },
	{ "rx_frame_127" },
	{ "rx_frame_255" },
	{ "rx_frame_511" },
	{ "rx_frame_1023" },
	{ "rx_frame_1518" },
	{ "rx_frame_1522" },
	{ "rx_frame_jumbo" },
	{ "rx_frame_unicast" },
	{ "rx_frame_multicast" },
	{ "rx_frame_broadcast" },
	{ "rx_frame_control" },
	{ "rx_frame_pause" },
	{ "rx_frame_jabber" },
	{ "rx_frame_fragment" },
	{ "rx_frame_vlan" },
	{ "rx_frame_dvlan" },
	{ "rx_frame_fcs_error" },
	{ "rx_frame_unsupport" },
	{ "rx_frame_wrong_sa" },
	{ "rx_frame_align_err" },
	{ "rx_frame_length_err" },
	{ "rx_frame_oversize" },
	{ "rx_frame_mtu_err" },
	{ "rx_frame_truncated_err" },
	{ "rx_frame_undersize" },
	{ "tx_frames" },
	{ "tx_frame_good" },
	{ "tx_bytes" },
	{ "tx_frame_64" },
	{ "tx_frame_127" },
	{ "tx_frame_255" },
	{ "tx_frame_511" },
	{ "tx_frame_1023" },
	{ "tx_frame_1518" },
	{ "tx_frame_1522" },
	{ "tx_frame_jumbo" },
	{ "tx_frame_unicast" },
	{ "tx_frame_multicast" },
	{ "tx_frame_broadcast" },
	{ "tx_frame_control" },
	{ "tx_frame_pause" },
	{ "tx_frame_jabber" },
	{ "tx_frame_fragment" },
	{ "tx_frame_vlan" },
	{ "tx_frame_dvlan" },
	{ "tx_frame_fcs_error" },
	{ "tx_frame_oversize" },
	{ "tx_frame_error" },
	{ "tx_frame_fifo_underrun" },
	{ "tx_frame_collision" },
};

static int apm_get_sset_count(struct net_device *net_dev, int sset)
{
	switch (sset) {
		case ETH_SS_STATS:
			return ARRAY_SIZE(apm_stat_name);
		default:
			return -EOPNOTSUPP;
	}

	return 0;
}

static void apm_get_strings(struct net_device *net_dev, u32 stringset,
			      u8 *data)
{
	if (stringset == ETH_SS_STATS) {
		memcpy(data, apm_stat_name, sizeof(apm_stat_name));
	}
}

static void apm_get_ethtool_stats(struct net_device *net_dev,
				    struct ethtool_stats *ss, uint64_t *data)
{
	struct apm *apm = netdev_priv(net_dev);
	struct iproc_pm_ops *pm_ops = apm->pm_ops;
	struct iproc_pm_stats stats;
	int i = 0;

	if (pm_ops) {
		pm_ops->port_stats(apm->land_idx, &stats);

		data[i++] = stats.rx_frames;
		data[i++] = stats.rx_frame_good;
		data[i++] = stats.rx_bytes;
		data[i++] = stats.rx_frame_64;
		data[i++] = stats.rx_frame_127;
		data[i++] = stats.rx_frame_255;
		data[i++] = stats.rx_frame_511;
		data[i++] = stats.rx_frame_1023;
		data[i++] = stats.rx_frame_1518;
		data[i++] = stats.rx_frame_1522;
		data[i++] = stats.rx_frame_jumbo;
		data[i++] = stats.rx_frame_unicast;
		data[i++] = stats.rx_frame_multicast;
		data[i++] = stats.rx_frame_broadcast;
		data[i++] = stats.rx_frame_control;
		data[i++] = stats.rx_frame_pause;
		data[i++] = stats.rx_frame_jabber;
		data[i++] = stats.rx_frame_fragment;
		data[i++] = stats.rx_frame_vlan;
		data[i++] = stats.rx_frame_dvlan;
		data[i++] = stats.rx_frame_fcs_error;
		data[i++] = stats.rx_frame_unsupport;
		data[i++] = stats.rx_frame_wrong_sa;
		data[i++] = stats.rx_frame_align_err;
		data[i++] = stats.rx_frame_length_err;
		data[i++] = stats.rx_frame_oversize;
		data[i++] = stats.rx_frame_mtu_err;
		data[i++] = stats.rx_frame_truncated_err;
		data[i++] = stats.rx_frame_undersize;
		data[i++] = stats.tx_frames;
		data[i++] = stats.tx_frame_good;
		data[i++] = stats.tx_bytes;
		data[i++] = stats.tx_frame_64;
		data[i++] = stats.tx_frame_127;
		data[i++] = stats.tx_frame_255;
		data[i++] = stats.tx_frame_511;
		data[i++] = stats.tx_frame_1023;
		data[i++] = stats.tx_frame_1518;
		data[i++] = stats.tx_frame_1522;
		data[i++] = stats.tx_frame_jumbo;
		data[i++] = stats.tx_frame_unicast;
		data[i++] = stats.tx_frame_multicast;
		data[i++] = stats.tx_frame_broadcast;
		data[i++] = stats.tx_frame_control;
		data[i++] = stats.tx_frame_pause;
		data[i++] = stats.tx_frame_jabber;
		data[i++] = stats.tx_frame_fragment;
		data[i++] = stats.tx_frame_vlan;
		data[i++] = stats.tx_frame_dvlan;
		data[i++] = stats.tx_frame_fcs_error;
		data[i++] = stats.tx_frame_oversize;
		data[i++] = stats.tx_frame_error;
		data[i++] = stats.tx_frame_fifo_underrun;
		data[i++] = stats.tx_frame_collision;
	}
}

static int apm_dump_phy_regs(struct apm *apm, int try_run, char *reg_buf)
{
	struct phy_device *phydev = apm->net_dev->phydev;
	int idx, len = 0;
	char *buf, tmp[32];
	u16 data = 0;

	if (phydev) {
		for (idx = 0; idx < 16; idx++) {
			if (try_run || !reg_buf) {
				buf = tmp;
			} else {
				buf = reg_buf + len;
				data = phy_read(phydev, idx);
			}
			len += sprintf(buf, "PHY REG %d: 0x%.4x\n", idx, data);
		}
	}
	return len;
}

static int apm_get_regs_len(struct net_device *dev)
{
	struct apm *apm = netdev_priv(dev);
	u32 len = 0;

	len += apm_dump_phy_regs(apm, 1, NULL);

	return len;
}

static void apm_get_regs(struct net_device *dev,
		struct ethtool_regs *regs, void *_p)
{
	struct apm *apm = netdev_priv(dev);
	u32 len = 0;

	regs->version = 0;

	/* Dump phy register */
	len += apm_dump_phy_regs(apm, 0, (char *)_p + len);
}

static void apm_get_drvinfo(struct net_device *net_dev,
			      struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, "apm", sizeof(info->driver));
	strlcpy(info->version, "0.1", sizeof(info->version));
	strlcpy(info->bus_info, "axi", sizeof(info->bus_info));
	info->regdump_len = apm_get_regs_len(net_dev);
}

static const struct ethtool_ops apm_ethtool_ops = {
	.get_regs			= apm_get_regs,
	.get_regs_len		= apm_get_regs_len,
	.get_strings		= apm_get_strings,
	.get_sset_count		= apm_get_sset_count,
	.get_ethtool_stats	= apm_get_ethtool_stats,
	.get_drvinfo		= apm_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings     = phy_ethtool_get_link_ksettings,
	.set_link_ksettings     = phy_ethtool_set_link_ksettings,
};

int apm_ethtool_init(struct net_device *net_dev)
{
	net_dev->ethtool_ops = &apm_ethtool_ops;
	return 0;
}
