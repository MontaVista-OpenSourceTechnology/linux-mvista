/*
 * Copyright (C) 2012 - 2014 Texas Instruments
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
#ifndef __KEYSTONE_NECTP_H__
#define __KEYSTONE_NECTP_H__

#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/keystone-dma.h>
#include <linux/interrupt.h>
#include <linux/u64_stats_sync.h>

/* Maximum Ethernet frame size supported by Keystone switch */
#define NETCP_MAX_FRAME_SIZE	9504
#define NETCP_MAX_MCAST_ADDR	16

/* Success return from a tx_hook asking for the packet to be dropped */
#define	NETCP_TX_DROP	1

#define SGMII_LINK_MAC_MAC_AUTONEG	0
#define SGMII_LINK_MAC_PHY		1
#define SGMII_LINK_MAC_MAC_FORCED	2
#define SGMII_LINK_MAC_FIBER		3
#define SGMII_LINK_MAC_PHY_NO_MDIO	4
#define SGMII_LINK_MAC_PHY_MASTER	5
#define SGMII_LINK_MAC_PHY_MASTER_NO_MDIO	6
#define SGMII_LINK_MAC_MAC_AN_SLAVE	7
#define XGMII_LINK_MAC_PHY		10
#define XGMII_LINK_MAC_MAC_FORCED	11

#define SGMII_REGS_SIZE			0x100

int keystone_sgmii_reset(void __iomem *sgmii_ofs, int port);
bool keystone_sgmii_rtreset(void __iomem *sgmii_ofs, int port, bool set);
int keystone_sgmii_link_status(void __iomem *sgmii_ofs, int ports);
int keystone_sgmii_get_port_link(void __iomem *sgmii_ofs, int port);
int keystone_sgmii_config(void __iomem *sgmii_ofs,
			  int port, u32 interface);

struct netcp_device;

enum netcp_rx_state {
	RX_STATE_INVALID,
	RX_STATE_INTERRUPT,
	RX_STATE_SCHEDULED,
	RX_STATE_POLL,
	RX_STATE_TEARDOWN,
};

enum netcp_tx_state {
	TX_STATE_INVALID,
	TX_STATE_INTERRUPT,
	TX_STATE_SCHEDULED,
	TX_STATE_POLL,
};

struct netcp_tx_pipe {
	struct netcp_priv		*netcp_priv;
	struct dma_chan			*dma_channel;
	const char			*dma_chan_name;
	u8				 dma_psflags;
	u8				 filler1;
	u16				 dma_queue;
	unsigned int			 dma_queue_depth;
	unsigned int			 dma_pause_threshold;
	unsigned int			 dma_resume_threshold;
	atomic_t			 dma_poll_count;
	enum netcp_tx_state		 dma_poll_state;
	struct napi_struct		 dma_poll_napi;
};

#define ADDR_NEW	BIT(0)
#define ADDR_VALID	BIT(1)

enum netcp_addr_type {
	ADDR_ANY,
	ADDR_DEV,
	ADDR_UCAST,
	ADDR_MCAST,
	ADDR_BCAST
};

struct netcp_addr {
	struct netcp_priv	*netcp;
	unsigned char		 addr[MAX_ADDR_LEN];
	enum netcp_addr_type	 type;
	unsigned int		 flags;
	struct list_head	 node;
};

struct netcp_stats {
	struct u64_stats_sync	syncp_rx ____cacheline_aligned_in_smp;
	u64			rx_packets;
	u64			rx_bytes;
	u32			rx_errors;
	u32			rx_dropped;

	struct u64_stats_sync	syncp_tx ____cacheline_aligned_in_smp;
	u64			tx_packets;
	u64			tx_bytes;
	u32			tx_dropped;
};

/* Flags for hw_capabilities */
#define	CPSW_HAS_P0_TX_CRC_REMOVE	BIT(0)

struct netcp_priv {
	/* Common stuff first */
	struct netcp_device		*netcp_device;
	struct net_device		*ndev;
	struct platform_device		*pdev;
	struct device			*dev;
	struct cpswx_priv		*cpswx_priv;
	int				 cpsw_port;
	/* Tx data path stuff */
	struct netcp_hook_list		*txhook_list_array;
	/* Rx data path stuff */
	struct dma_chan			*rx_channel;
	enum netcp_rx_state		 rx_state;
	struct netcp_hook_list		*rxhook_list_array;
	u32				hw_capabilities;
	struct napi_struct		 napi;
	/* Non data path stuff */
	u32				 msg_enable;
	spinlock_t			 lock;
	int				 rx_packet_max;
	const char			*rx_chan_name;
	u32				 phy_link_state_mask;
	struct list_head		 module_head;
	struct list_head		 interface_list;
	struct list_head		 addr_list;
	/* 64-bit netcp stats */
	struct netcp_stats		 stats;

	/* PktDMA configuration data */
	u32				 rx_queue_depths[KEYSTONE_QUEUES_PER_CHAN];
	u32				 rx_buffer_sizes[KEYSTONE_QUEUES_PER_CHAN];
};

#define NETCP_SGLIST_SIZE	(MAX_SKB_FRAGS + 2)
#define	NETCP_PSDATA_LEN	16

/* Maximum number of psdata words that are used in the Rx direction.
 * NOTE: this value should be updated if the drivers need to use more psdata.
 * Current usage is:
 * SA driver looks upto 4 words
 * PA driver looks upto 7 words
 */
#define	NETCP_MAX_RX_PSDATA_LEN	7

struct netcp_packet {
	struct sk_buff			*skb;
	struct netcp_priv		*netcp;
	struct netcp_tx_pipe		*tx_pipe;
	dma_cookie_t			 cookie;
	bool				 rxtstamp_complete;
	void				*ts_context;
	int				(*txtstamp_complete)(void *context,
						struct sk_buff *skb);
	void				*primary_bufptr;
	unsigned int			 primary_bufsiz;
	unsigned int			 primary_datsiz;
	u32				 eflags;
	u32				 epib[4];
	unsigned int			 psdata_len;
	int				 sg_ents;
	struct scatterlist		 sg[NETCP_SGLIST_SIZE]
							____cacheline_aligned;
	u32				 psdata[NETCP_PSDATA_LEN]
							____cacheline_aligned;
};

static inline u32 *netcp_push_psdata(struct netcp_packet *p_info, unsigned bytes)
{
	u32		*buf;
	unsigned	 words;

	if ((bytes & 0x03) != 0)
		return NULL;
	words = bytes >> 2;

	if ((p_info->psdata_len + words) > NETCP_PSDATA_LEN)
		return NULL;

	p_info->psdata_len += words;
	buf = &p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len];

	return buf;
}

static inline int netcp_align_psdata(struct netcp_packet *p_info, unsigned byte_align)
{
	int	padding;

	switch (byte_align) {
	case 0:
		padding = -EINVAL;
		break;
	case 1:
	case 2:
	case 4:
		padding = 0;
		break;
	case 8:
		padding = (p_info->psdata_len << 2) % 8;
		break;
	case 16:
		padding = (p_info->psdata_len << 2) % 16;
		break;
	default:
		padding = (p_info->psdata_len << 2) % byte_align;
		break;
	}

	return padding;
}

static inline int netcp_prepend_psdata(struct netcp_packet *p_info, u32 *data, unsigned len)
{
	if ((len + p_info->psdata_len) > NETCP_PSDATA_LEN)
		return -ENOBUFS;
	p_info->psdata_len += len;

	memcpy(&p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len],
			data, len * sizeof(u32));
	return 0;
}

struct netcp_module {
	const char		*name;
	struct module		*owner;
	struct list_head	 module_list;
	struct list_head	 interface_list;

	/* probe/remove: called once per NETCP instance */
	int			(*probe)(struct netcp_device *netcp_device,
					 struct device *device,
					 struct device_node *node,
					 void **inst_priv);
	int			(*remove)(struct netcp_device *netcp_device,
					  void *inst_priv);

	/* attach/release: called once per network interface */
	int			(*attach)(void *inst_priv, struct net_device *ndev,
					  void **intf_priv);
	int			(*release)(void *intf_priv);

	int			(*open)(void *intf_priv, struct net_device *ndev);
	int			(*close)(void *intf_priv, struct net_device *ndev);
	int			(*add_addr)(void *intf_priv,
					    struct netcp_addr *naddr);
	int			(*del_addr)(void *intf_priv,
					    struct netcp_addr *naddr);
	int			(*add_vid)(void *intf_priv, int vid);
	int			(*del_vid)(void *intf_priv, int vid);
	int			(*ioctl)(void *intf_priv, struct ifreq *req,
					 int cmd);
};

int netcp_register_module(struct netcp_module *module);
void netcp_unregister_module(struct netcp_module *module);

u32 netcp_get_streaming_switch(struct netcp_device *netcp_device, int port);
u32 netcp_get_streaming_switch2(struct netcp_device *netcp_device, int port);
u32 netcp_set_streaming_switch(struct netcp_device *netcp_device,
				int port, u32 new_value);
u32 netcp_set_streaming_switch2(struct netcp_device *netcp_device,
				 int port, u32 new_value);
int netcp_create_interface(struct netcp_device *netcp_device,
			   struct net_device **ndev_p,
			   const char *ifname_proto,
			   int tx_queues, int rx_queues,
			   int cpsw_port);
void netcp_delete_interface(struct netcp_device *netcp_device,
			    struct net_device *ndev);

int netcp_txpipe_init(struct netcp_tx_pipe *tx_pipe,
		struct netcp_priv *netcp_priv,
		const char *chan_name,
		int queue_depth);
int netcp_txpipe_open(struct netcp_tx_pipe *tx_pipe);
int netcp_txpipe_close(struct netcp_tx_pipe *tx_pipe);

struct dma_chan *netcp_get_rx_chan(struct netcp_priv *priv);
struct dma_chan *netcp_get_tx_chan(struct netcp_priv *priv);

typedef int netcp_hook_rtn(int order, void *data, struct netcp_packet *packet);

int netcp_register_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_unregister_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_register_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_unregister_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);

void *netcp_device_find_module(struct netcp_device *netcp_device,
		const char *name);
int xge_serdes_init(struct device_node *node);
int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface);
#endif
