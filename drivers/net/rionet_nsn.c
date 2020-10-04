/*
 * rionet - Ethernet driver over RapidIO messaging services
 *
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/rio_ids.h>
#include <linux/if_rio.h>
#include <linux/if_syscom_ether.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <linux/if_arp.h>
#include <linux/rionet-user.h>

#define DRV_NAME	"rionet"
#define DRV_VERSION	"0.3"
#define DRV_AUTHOR	"Matt Porter <mporter@kernel.crashing.org>"
#define DRV_DESC	"Ethernet over RapidIO"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");
#define RIONET_DEFAULT_MSGLEVEL \
			(NETIF_MSG_DRV		| \
			 NETIF_MSG_LINK		| \
			 NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR)


#define RIONET_TX_RING_SIZE	CONFIG_RIONET_NSN_TX_SIZE
#define RIONET_RX_RING_SIZE	CONFIG_RIONET_NSN_RX_SIZE
#define RIONET_MAX_NETS		8
#define RIONET_MSG_SIZE		RIO_MAX_MSG_SIZE
#define RIONET_MAX_LETTER	(4)

#define rio_letter csum_level
#define rio_id     hash
#define ARPHRD_RIO ARPHRD_VOID

#define ndev_type_syscom(ndev, let) \
	(((ndev)->type == ARPHRD_RIO) || \
	 (((struct rionet_private *)netdev_priv(ndev))->mbox_no == 3 && (let != 0)))

#define skb_type_syscom(skb) \
	(((skb)->protocol == htons(ETH_P_SYSCOM_FRAG)) || \
	 ((skb)->protocol == htons(ETH_P_SYSCOM)))

#define RIONET_MB_MASK				\
	((IS_ENABLED(CONFIG_RIONET_MBOX_0_ENABLE) ? (1<<0) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MBOX_1_ENABLE) ? (1<<1) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MBOX_2_ENABLE) ? (1<<2) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MBOX_3_ENABLE) ? (1<<3) : 0))

#define RIONET_MB0_RXLETTER_MASK				\
	((IS_ENABLED(CONFIG_RIONET_MB0_RX_LETTER0) ? (1<<0) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB0_RX_LETTER1) ? (1<<1) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB0_RX_LETTER2) ? (1<<2) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB0_RX_LETTER3) ? (1<<3) : 0))

#define RIONET_MB1_RXLETTER_MASK				\
	((IS_ENABLED(CONFIG_RIONET_MB1_RX_LETTER0) ? (1<<0) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB1_RX_LETTER1) ? (1<<1) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB1_RX_LETTER2) ? (1<<2) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB1_RX_LETTER3) ? (1<<3) : 0))

#define RIONET_MB2_RXLETTER_MASK				\
	((IS_ENABLED(CONFIG_RIONET_MB2_RX_LETTER0) ? (1<<0) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB2_RX_LETTER1) ? (1<<1) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB2_RX_LETTER2) ? (1<<2) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB2_RX_LETTER3) ? (1<<3) : 0))

#define RIONET_MB3_RXLETTER_MASK				\
	((IS_ENABLED(CONFIG_RIONET_MB3_RX_LETTER0) ? (1<<0) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB3_RX_LETTER1) ? (1<<1) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB3_RX_LETTER2) ? (1<<2) : 0) | \
	(IS_ENABLED(CONFIG_RIONET_MB3_RX_LETTER3) ? (1<<3) : 0))

#define RIONET_CONFIG(num) { \
	.eth = IS_ENABLED(CONFIG_RIONET_MB##num##_ETHERNET_ENCAPSULATION), \
	.tx_letter = CONFIG_RIONET_MB##num##_TX_LETTER_ID, \
	.rx_letter_mask = RIONET_MB##num##_RXLETTER_MASK }

static const struct rionet_config_s {
	int eth;
	int tx_letter;
	int rx_letter_mask;
} rionet_config[] = {
	RIONET_CONFIG(0), RIONET_CONFIG(1), RIONET_CONFIG(2), RIONET_CONFIG(3)
};

extern struct list_head rio_devices;
extern spinlock_t rio_global_list_lock;

struct rionet_private {
	struct rio_mport *mport;
	spinlock_t lock;
	spinlock_t tx_lock;
	u32 msg_enable;
	struct net_device *ndev;
	int tx_priority;
	int mbox_no;
	int tx_letter;
	int rx_letter_mask;
};
struct rionet_dev_priv {
	struct net_device *ndev[4];
};

#define is_rionet_capable(src_ops, dst_ops)			\
			((src_ops & RIO_SRC_OPS_DATA_MSG) &&	\
			 (dst_ops & RIO_DST_OPS_DATA_MSG))
#define dev_rionet_capable(dev) \
	is_rionet_capable(dev->src_ops, dev->dst_ops)

#define RIONET_MAC_MATCH(x)	(!memcmp((x), "\00\01\00\01", 4))
#define RIONET_GET_DESTID(x)	((*((u8 *)x + 4) << 8) | *((u8 *)x + 5))

/*
 * build a Ethernet MAC address by using a RapidIO ID
 */
static void srio_build_mac(uint16_t rio_id, uint8_t * mac, int mbox, int ether)
{
	if (ether) {
		mac[0] = 0xe;
		mac[1] = 0;
		mac[2] = 0;
		mac[3] = (uint8_t) (mbox & 3);
		mac[4] = (uint8_t) (rio_id >> 8);
		mac[5] = (uint8_t) rio_id;
	} else {
		mac[0] = (uint8_t) (rio_id >> 8);
		mac[1] = (uint8_t) rio_id;
	}
}

static int srio_change_mtu(struct net_device *dev, int new_mtu)
{
	int max_mtu = RIONET_MSG_SIZE;
	max_mtu -= dev->type == ARPHRD_ETHER ? ETH_HLEN : 0;

	if ((new_mtu < 1) || (new_mtu > max_mtu)) {
		netdev_err(dev, "RION: mtu must be between 1 and %d\n",
				max_mtu);
		return -EINVAL;
	}
	dev->mtu = new_mtu;
	return 0;
}

static int srio_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	int ivalue;

	switch (cmd) {
	case RIONET_NET_IOCTL_SETPRIO:
		ivalue = rq->ifr_ifru.ifru_ivalue;
		if ((ivalue >= 0) && (ivalue < 4)) {
			rnet->tx_priority = ivalue;
			return 0;
		} else if (ivalue == -1) {
			/* take priority out of the skb on send */
			rnet->tx_priority = ivalue;
			return 0;
		}
		return -EINVAL;

	case RIONET_NET_IOCTL_GETPRIO:
		rq->ifr_ifru.ifru_ivalue = rnet->tx_priority;
		return 0;
	}
	return -EOPNOTSUPP;
}

static void rionet_recv_skb(struct net_device *ndev, struct sk_buff *skb,
			    int sz, int letter, int sourceid)
{
	int proto, error;

	print_hex_dump_debug("RION INC: Message from MPORT:",
			     DUMP_PREFIX_OFFSET, 64, 1, skb->data, 64, 1);

#ifdef CONFIG_FSM4_AXM
	/* WA for PR114256: Do it for small messages only to not waste too much
	 * time by copying. Average syscom message size is ~150B
	 */
	if (sz < 256) {
		unsigned int truesize = skb->truesize;
		struct sk_buff *skb_new = netdev_alloc_skb_ip_align(ndev, sz);

		if (!skb_new) {
			dev_kfree_skb(skb);
			ndev->stats.rx_dropped++;
			return;
		}
		/* skb_put is done later */
		memcpy(skb_new->data, skb->data, sz);
		dev_kfree_skb(skb);
		skb = skb_new;
		netdev_dbg(ndev, "RION INC: SKB Truesize (Bef-Aft) %d-%d\n",
			   truesize, skb->truesize);
	}
#endif
	skb_reset_network_header(skb);
	skb_reset_mac_header(skb);
	skb_reset_transport_header(skb);
	skb_put(skb, sz);
	if (skb->len > 0x2000U)
		netdev_err(ndev, "skblen1: %x\n", skb->len);

	if (ndev_type_syscom(ndev, letter)) {
		skb->protocol = letter & 0x2 ?
			htons(ETH_P_SYSCOM_FRAG) :
			htons(ETH_P_SYSCOM);
		skb->dev = ndev;
		skb->rio_letter = letter;
		skb->rio_id = sourceid;
	} else {
		skb->protocol = eth_type_trans(skb, ndev);
	}

	print_hex_dump_debug("RION INC: Complete msg to netif_X():",
			     DUMP_PREFIX_OFFSET, 64, 1, skb, 64, 1);

	proto = skb->protocol;
	error = in_irq() ? netif_rx(skb) : netif_receive_skb(skb);
	if (error == NET_RX_DROP) {
		ndev->stats.rx_dropped++;
		if (net_ratelimit())
			netdev_info(ndev,
				    "RION INC: netif_X() proto=0x%x msg dropped\n",
				    ntohs(proto));
	} else {
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += sz;
	}
}

static struct kmem_cache *rionet_skb_cache;

static inline struct sk_buff *rionet_alloc_skb(void)
{
	struct skb_shared_info *shinfo;
	struct sk_buff *skb;
	unsigned size;
	u8 *data;

	skb = alloc_skb_head(GFP_ATOMIC);
	if (unlikely(!skb)) {
		return NULL;
	}

	data = kmem_cache_alloc(rionet_skb_cache, GFP_ATOMIC);
	if (unlikely(!data)) {
		kfree(skb);
		return NULL;
	}

	size = ksize(data);
	prefetchw(data + size);
	skb->truesize += size;
	size = SKB_WITH_OVERHEAD(size);

	skb->head = data;
	skb->data = data;
	skb_reset_tail_pointer(skb);
	skb->end = skb->tail + size;
	skb_reserve(skb, NET_SKB_PAD);

	shinfo = skb_shinfo(skb);
	memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
	atomic_set(&shinfo->dataref, 1);
	kmemcheck_annotate_variable(shinfo->destructor_arg);

	return skb;
}

static int rionet_queue_tx_msg(struct sk_buff **skb, struct net_device *ndev, u16 destid)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	int len = (*skb)->len;
	int tx_letter = skb_type_syscom(*skb) ? (*skb)->rio_letter : rnet->tx_letter;
	int rc;

#if CONFIG_RIONET_SKB_HEADROOM > 0
	unsigned int headroom = CONFIG_RIONET_SKB_HEADROOM;

	if (unlikely(skb_headroom(*skb) < headroom)) {
		struct sk_buff *new_skb = skb_copy_expand(*skb, headroom, 0, GFP_ATOMIC);

		netdev_dbg(ndev, "RION OUT: skb_copy_expand\n");
		if (!new_skb) {
			if (net_ratelimit())
				netdev_err(ndev, "RION OUT: No mem for skb_copy_expand\n");
			ndev->stats.tx_dropped++;
			return -ENOMEM;
		}
		dev_kfree_skb_any(*skb);
		*skb = new_skb;
	}
	__skb_push(*skb, headroom);
#endif

	rc = rio_mport_add_outb_message(rnet->mport, destid, rnet->mbox_no,
				tx_letter, 0, (*skb)->data, (*skb)->len, *skb);

	if (rc) {
		if (net_ratelimit())
			netdev_info(ndev, "RION OUT: Dropped due to rc=%d \n", rc);
		ndev->stats.tx_dropped++;
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += len;
	}
	return rc;
}

static int rionet_broadcast_capable(struct rio_dev *rdev)
{
/* TEMP: FSMr3 does not support setting up proper asm_vid value,
 * so ugly fix is used instead - solution to be found ...
 */
#ifdef CONFIG_NSN_BOARD_FCT
	if (((rdev->destid & 0xF) == 0xD) || rdev->host)
		return 1;
#else
	if ((rdev->host) || !(rdev->asm_vid == rdev->vid || rdev->asm_vid == 0))
		return 1;
#endif
	return 0;
}

#ifdef CONFIG_RAPIDIO_NSN
static int rionet_bcast_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	struct rio_dev *rdev;
	struct sk_buff *new_skb = skb;
	struct rio_mport *mport = rnet->mport;
	u16 destid = 0;
	u32 regVal;
	int rc = 0;

	if (mport) {
		rio_local_read_config_32(mport, 0x60, &regVal);
		destid = ((regVal & 0xf000) | 0x11);

		if ((destid & 0xf000) && destid != (regVal & 0xffff)) {
			/*
			 * First make copy because skb's content might be modified and also
			 * after each sending, skb is released
			 */
			new_skb = skb_copy(skb, GFP_ATOMIC);
			if (!new_skb) {
				if (net_ratelimit())
					netdev_err(ndev, "RION OUT: No mem, skb is not copied during broadcast\n");
				ndev->stats.tx_dropped++;
				return -ENOMEM;
			}
			rc = rionet_queue_tx_msg(&new_skb, ndev, destid);
			if (rc) {
				dev_kfree_skb_any(new_skb);
				return rc;
			}
		}
	} else {
		if (net_ratelimit())
			netdev_info(ndev, "RION OUT: broadcast TX failed\n");
	}

	spin_lock(&rio_global_list_lock);
	list_for_each_entry(rdev, &rio_devices, global_list) {
		u16 my_id;

		if (rdev == NULL || (rdev->pef & RIO_PEF_SWITCH)
				|| rdev->net == NULL || rdev->net->hport == NULL
				|| !rionet_broadcast_capable(rdev))
			continue;

		if((rdev->destid & 0xff) == 0x13)
			continue;

		my_id = rdev->destid;

		/*
		 * First make copy because skb's content might be modified and also
		 * after each sending, skb is released
		 */
		new_skb = skb_copy(skb, GFP_ATOMIC);
		if (!new_skb) {
			if (net_ratelimit())
				netdev_err(ndev, "RION OUT: No mem, skb is not copied during broadcast\n");
			spin_unlock(&rio_global_list_lock);
			ndev->stats.tx_dropped++;
			return -ENOMEM;
		}
		rc = rionet_queue_tx_msg(&new_skb, ndev, my_id);
		if (rc) {
			dev_kfree_skb_any(new_skb);
			break;
		}
	}
	spin_unlock(&rio_global_list_lock);

	if (!rc)
		dev_consume_skb_any(skb);

	return rc;
}
#else
static int rionet_bcast_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rio_dev *rdev;
	struct sk_buff *skbn = skb;
	int rc = 0;

	spin_lock(&rio_global_list_lock);
	list_for_each_entry(rdev, &rio_devices, global_list) {
		if (rdev == NULL || (rdev->pef & RIO_PEF_SWITCH)
				|| rdev->net == NULL || rdev->net->hport == NULL
				|| !rionet_broadcast_capable(rdev))
			continue;

#ifdef CONFIG_NSN_BOARD_FCT
		skbn = skb_copy(skb, GFP_ATOMIC);
		if (!skbn) {
			if (net_ratelimit())
				netdev_err(ndev, "RION OUT: No mem, skb is not copied during bcast\n");
			spin_unlock(&rio_global_list_lock);
			ndev->stats.tx_dropped++;
			return -ENOMEM;
		}
#else
		skb_get(skbn);
#endif
		rc = rionet_queue_tx_msg(&skbn, ndev, rdev->destid);
		if (rc) {
			dev_kfree_skb_any(skbn);
			break;
		}
	}
	spin_unlock(&rio_global_list_lock);

	if (!rc)
		dev_consume_skb_any(skb);

	return rc;
}
#endif

static int rionet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	int tx_letter = skb_type_syscom(skb) ? skb->rio_letter : rnet->tx_letter;
	u16 destid;
	unsigned long flags;
	int rc = NETDEV_TX_OK;

	spin_lock_irqsave(&rnet->tx_lock, flags);

	if (ndev_type_syscom(ndev, tx_letter)) {
		if (unlikely(skb->len > RIONET_MSG_SIZE))
			goto err_size;
		rc = rionet_queue_tx_msg(&skb, ndev, skb->rio_id);
	} else {
		const struct ethhdr *eth = (struct ethhdr *)skb->data;
		if (unlikely(skb->len > RIONET_MSG_SIZE))
			goto err_size;
		if (is_multicast_ether_addr(eth->h_dest)) {
			rc = rionet_bcast_xmit(skb, ndev);
		} else {
			destid = RIONET_GET_DESTID(eth->h_dest);
			rc = rionet_queue_tx_msg(&skb, ndev, destid);
		}
	}

	if (rc) {
		netif_stop_queue(ndev);
		rc = NETDEV_TX_BUSY;
	}

	spin_unlock_irqrestore(&rnet->tx_lock, flags);

	return rc;

err_size:
	spin_unlock_irqrestore(&rnet->tx_lock, flags);
	if (net_ratelimit()) {
		netdev_err(ndev, "RION OUT: TX packet %dB larger than %dB, "
				"dropped\n", skb->len, RIONET_MSG_SIZE);
	}
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return 0;
}

static void rionet_inb_msg_event(struct rio_mport *mport, void *dev_id, int mbox)
{
	struct sk_buff *skb;
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);
	int ret, let, sid, count, sz = RIONET_MSG_SIZE;

	spin_lock(&rnet->lock);
	for (let = 0; let < RIONET_MAX_LETTER; let++) {
		if (!(rnet->rx_letter_mask & (1 << let)))
			continue;

		count = 0;
		while (rio_mport_get_inb_message(mport, mbox, let, &sz, &sid,
						 (void **)&skb)) {
			rionet_recv_skb(ndev, skb, sz, let, sid);

			if (++count == RIONET_RX_RING_SIZE)
				break;
		}

		while (count) {
			skb = rionet_alloc_skb();
			if (!skb)
				break;

			ret = rio_mport_add_inb_buffer(mport, mbox, let,
						       skb->data, skb);
			if (ret) {
				dev_kfree_skb_any(skb);
				if (net_ratelimit())
					netdev_err(ndev,
						   "RION INC: add_inb_buf=%d\n",
						   ret);
				break;
			}
			count--;
		}
	}
	spin_unlock(&rnet->lock);
}

static void rionet_release_skb(void *cookie)
{
	kfree_skb((struct sk_buff *)cookie);
}

static void rionet_outb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int status, void *cookie)
{
	struct rionet_dev_priv *dpriv = dev_get_drvdata(&mport->dev);
	struct sk_buff *skb = cookie;
	int i;

	if (status) {
		skb->peeked = 1;
		dev_kfree_skb_any(skb);
	} else {
		dev_consume_skb_any(skb);
	}

	for (i = 0; i < ARRAY_SIZE(rionet_config); i++) {
		struct net_device *ndev = dpriv->ndev[i];
		if (unlikely(netif_queue_stopped(ndev)))
			netif_wake_queue(ndev);
	}
}

static int rionet_open(struct net_device *ndev)
{
	int i, j, rc = 0;
	struct rionet_private *rnet = netdev_priv(ndev);
	unsigned long flags = 0;

	srio_build_mac(rnet->mport->host_deviceid, ndev->dev_addr, rnet->mbox_no,
			ndev->type == ARPHRD_ETHER);

	if (netif_msg_ifup(rnet))
		netdev_info(ndev, "RION: open dev %s\n", ndev->name);

	if ((rc = rio_request_outb_mbox(rnet->mport,
					(void *)ndev,
					rnet->mbox_no,
					RIONET_TX_RING_SIZE,
					rionet_outb_msg_event)) < 0)
		goto out;

	rc = rio_request_inb_mbox_let(rnet->mport,
					  (void *)ndev,
					  rnet->mbox_no, rnet->rx_letter_mask,
					  RIONET_RX_RING_SIZE,
					  rionet_inb_msg_event);
	if (rc < 0) {
		rio_release_outb_mbox(rnet->mport, rnet->mbox_no);
		goto out;
	}

	/* Initialize inbound message ring */
	spin_lock_irqsave(&rnet->lock, flags);
	for (j = 0; j < RIONET_MAX_LETTER; j++) {
		if (!(rnet->rx_letter_mask & (1 << j)))
			continue;

		for (i = 0; i < RIONET_RX_RING_SIZE; i++) {
			struct sk_buff *skb = rionet_alloc_skb();

			if (unlikely(!skb))
				break;

			rc = rio_mport_add_inb_buffer(rnet->mport,
						      rnet->mbox_no, j,
						      skb->data, skb);
			if (rc) {
				dev_kfree_skb_any(skb);
				break;
			}
		}
	}
	spin_unlock_irqrestore(&rnet->lock, flags);

	netif_carrier_on(ndev);
	netif_start_queue(ndev);
out:
	return rc;
}

static int rionet_close(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	if (netif_msg_ifup(rnet))
		netdev_info(ndev, "RION: close %s\n", ndev->name);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	rio_release_inb_mbox_let(rnet->mport, rnet->mbox_no,
			rnet->rx_letter_mask, rionet_release_skb);
	rio_release_outb_mbox(rnet->mport, rnet->mbox_no);

	return 0;
}



static void rionet_get_drvinfo(struct net_device *ndev,
			       struct ethtool_drvinfo *info)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->fw_version, "n/a", sizeof(info->fw_version));
	strlcpy(info->bus_info, rnet->mport->name, sizeof(info->bus_info));
}

static u32 rionet_get_msglevel(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	return rnet->msg_enable;
}

static void rionet_set_msglevel(struct net_device *ndev, u32 value)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	rnet->msg_enable = value;
}

static const struct ethtool_ops rionet_ethtool_ops = {
	.get_drvinfo = rionet_get_drvinfo,
	.get_msglevel = rionet_get_msglevel,
	.set_msglevel = rionet_set_msglevel,
	.get_link = ethtool_op_get_link,
};

static const struct net_device_ops rionet_netdev_ops = {
	.ndo_open		= rionet_open,
	.ndo_stop		= rionet_close,
	.ndo_start_xmit		= rionet_start_xmit,
	.ndo_do_ioctl		= srio_ioctl,
	.ndo_change_mtu		= srio_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

/*** Raw RIO (without Ethernet emulation) ****************************/

static int raw_rio_parse_header(const struct sk_buff *skb, unsigned char *haddr)
{
	struct riohdr *hdr = (struct riohdr *)haddr;
	struct net_device *dev = skb->dev;
	struct rionet_private *rnet = netdev_priv(dev);
	uint16_t local;

	memset(hdr, 0, sizeof *hdr);
	riohdr_set_mbox(hdr, rnet->mbox_no);
	riohdr_set_letter(hdr, skb->rio_letter);

	local = dev->addr_len >= 2 ? dev->dev_addr[0] << 8 | dev->dev_addr[1] :
			             dev->dev_addr[0];

	if (skb->pkt_type == PACKET_OUTGOING) {
		hdr->destination = htons(skb->rio_id);
		hdr->source = htons(local);
	} else {
		hdr->destination = htons(local);
		hdr->source = htons(skb->rio_id);
	}

	return sizeof *hdr;
}

static int raw_rio_create_header(struct sk_buff *skb, struct net_device *dev,
		unsigned short type, const void *daddr, const void *saddr,
		unsigned int len)
{
	struct rionet_private *rnet = netdev_priv(dev);
	__be16 rio_id;

	if (saddr) return -EINVAL;

	if (type == ETH_P_SYSCOM) {
		skb->rio_letter = rnet->tx_letter & 0x1;
	} else if (type == ETH_P_SYSCOM_FRAG) {
		skb->rio_letter = rnet->tx_letter | 0x2;
	} else {
		return -EPROTONOSUPPORT;
	}

	memcpy(&rio_id, daddr, sizeof rio_id);
	skb->rio_id = ntohs(rio_id);

	skb_reset_network_header(skb);
	skb_reset_mac_header(skb);
	skb_reset_transport_header(skb);

	return 0;
}

const static struct header_ops raw_rio_header_ops = {
	.parse = raw_rio_parse_header,
	.create = raw_rio_create_header,
};

static void raw_rio_setup(struct net_device *dev)
{
	dev->type = ARPHRD_RIO;
	dev->addr_len = 2;
	dev->tx_queue_len = 1000;
	dev->priv_flags |= IFF_TX_SKB_SHARING;
	dev->header_ops = &raw_rio_header_ops;
}

/*********************************************************************/

static int eth_rio_parse_header(const struct sk_buff *skb, unsigned char *haddr)
{
	if (skb->protocol == htons(ETH_P_SYSCOM) ||
	    skb->protocol == htons(ETH_P_SYSCOM_FRAG)) {
		return raw_rio_parse_header(skb, haddr);
	} else {
		return eth_header_parse(skb, haddr);
	}
}

static int eth_rio_create_header(struct sk_buff *skb, struct net_device *dev,
		unsigned short type, const void *daddr, const void *saddr,
		unsigned int len)
{
	if (type == ETH_P_SYSCOM || type == ETH_P_SYSCOM_FRAG) {
		return raw_rio_create_header(skb, dev, type, daddr, saddr, len);
	} else {
		return eth_header(skb, dev, type, daddr, saddr, len);
	}
}

static const struct header_ops eth_rio_header_ops = {
	.parse = eth_rio_parse_header,
	.create = eth_rio_create_header,
	.cache = eth_header_cache,
	.cache_update = eth_header_cache_update,
};

/*********************************************************************/

static int rionet_add_mport_ifaces(struct device *dev,
			    struct class_interface *class_intf)
{
	int rc = -ENODEV, i;
	struct net_device *ndev = NULL;
	struct rionet_private *rnet;
	struct rio_mport *mport = to_rio_mport(dev);
	int mb_mask = RIONET_MB_MASK;
	struct rionet_dev_priv *dpriv = NULL;

	dpriv = kmalloc(sizeof(struct rionet_dev_priv), GFP_KERNEL);
	if (!dpriv)
		goto out;

	/*
	 * If first time through this net, make sure local device is rionet
	 * capable and setup netdev (this step will be skipped in later probes
	 * on the same net).
	 */
	for(i = 0; i < ARRAY_SIZE(rionet_config); i++) {
		if (!(mb_mask & (1<<i)))
			continue;
		/* Allocate our net_device structure */
		ndev = rionet_config[i].eth ? alloc_etherdev(sizeof *rnet) :
			alloc_netdev(sizeof(*rnet), "rio%d", NET_NAME_UNKNOWN,
				     raw_rio_setup);
		if (ndev == NULL) {
			rc = -ENOMEM;
			goto out;
		}

		netdev_info(ndev, "RION: add iface %d\n", i);

		if (rionet_config[i].eth) {
			ndev->mtu = RIONET_MSG_SIZE - ETH_HLEN;
			ndev->needed_headroom = ETH_HLEN + CONFIG_RIONET_SKB_HEADROOM;
			ndev->header_ops = &eth_rio_header_ops;
		} else {
			ndev->mtu = RIONET_MSG_SIZE;
			ndev->needed_headroom = CONFIG_RIONET_SKB_HEADROOM;
		}
		rnet = netdev_priv(ndev);
		rnet->mport = mport;
		rnet->mbox_no = i & 0x3;
		rnet->ndev = ndev;
		rnet->tx_letter = rionet_config[i].tx_letter;
		rnet->rx_letter_mask = rionet_config[i].rx_letter_mask;
		srio_build_mac(1, ndev->dev_addr, i, rionet_config[i].eth);
		ndev->netdev_ops = &rionet_netdev_ops;
		sprintf(ndev->name, "rio%dm%d", mport->id, i);
		netdev_info(ndev, "RION: add iface %s\n", ndev->name);
		ndev->features = NETIF_F_LLTX;
#ifdef CONFIG_RIONET_SCATTER_GATHER_SUPPORT
		ndev->features |=  NETIF_F_SG;
#endif
		ndev->ethtool_ops = &rionet_ethtool_ops;

		spin_lock_init(&rnet->lock);
		spin_lock_init(&rnet->tx_lock);

		rnet->msg_enable = RIONET_DEFAULT_MSGLEVEL;
		netif_carrier_off(ndev);
		rc = register_netdev(ndev);
		if (rc != 0)
			goto out;
		netdev_info(ndev, "RION: %d txid %d, rxmask 0x%x, type %d\n",
				i, rnet->tx_letter, rnet->rx_letter_mask,
				ndev->type);
		dpriv->ndev[i] = ndev;
	}
	dev_set_drvdata(dev, dpriv);

out:
	return rc;
}

static void rionet_remove_mport_ifaces(struct device *dev,
			struct class_interface *class_intf)
{
	struct net_device *ndev = NULL;
	u16 i;
	struct rionet_dev_priv *dpriv = dev_get_drvdata(dev);
	for (i = 0; i < 4; i++) {
		ndev = dpriv->ndev[i];
		if (ndev) {
			netdev_info(ndev, "RION: removing rionet device %s\n", ndev->name);
			unregister_netdev(ndev);
			free_netdev(ndev);
		}
	}
}

static struct class_interface rio_mport_interface __refdata = {
	.class = &rio_mport_class,
	.add_dev = rionet_add_mport_ifaces,
	.remove_dev = rionet_remove_mport_ifaces,
};

static int __init rionet_init(void)
{
	int rtn;

	rionet_skb_cache = kmem_cache_create("rio_skb",
			   SKB_DATA_ALIGN(RIONET_MSG_SIZE + NET_SKB_PAD) +
			   SKB_DATA_ALIGN(sizeof(struct skb_shared_info)),
			   NET_SKB_PAD, SLAB_CACHE_DMA, NULL);
	if (!rionet_skb_cache)
		return -ENOMEM;

	rtn = class_interface_register(&rio_mport_interface);
	if (rtn)
		kmem_cache_destroy(rionet_skb_cache);

	return rtn;
}

static void __exit rionet_exit(void)
{
	kmem_cache_destroy(rionet_skb_cache);
	class_interface_unregister(&rio_mport_interface);
}

module_init(rionet_init);
module_exit(rionet_exit);
