/*
 * Copyright (C) 2012 - 2014 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com>
 *	    Sandeep Paulraj <s-paulraj@ti.com>
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
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/net_tstamp.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/keystone-dma.h>

#include "keystone_net.h"

/* Read the e-fuse value as 32 bit values to be endian independent */
static inline int emac_arch_get_mac_addr(char *x,
					 void __iomem *efuse_mac)
{
	unsigned int addr0, addr1;

	addr1 = __raw_readl(efuse_mac + 4);
	addr0 = __raw_readl(efuse_mac);

	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;

	return 0;
}

static const char *netcp_node_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = node->name;
	if (!name)
		name = "unknown";
	return name;
}


/*
 *  Module management structures
 */
struct netcp_device {
	struct list_head	 device_list;
	struct list_head	 interface_head;
	struct list_head	 modpriv_head;
	struct platform_device	*platform_device;
	void __iomem		*streaming_switch;
};

struct netcp_inst_modpriv {
	struct netcp_device	*netcp_device;
	struct netcp_module	*netcp_module;
	struct list_head	 inst_list;
	void			*module_priv;
};

struct netcp_intf_modpriv {
	struct netcp_priv	*netcp_priv;
	struct netcp_module	*netcp_module;
	struct list_head	 intf_list;
	void			*module_priv;
};

static LIST_HEAD(netcp_devices);
static LIST_HEAD(netcp_modules);
static DEFINE_MUTEX(netcp_modules_lock);

static struct kmem_cache *netcp_pinfo_cache;

/*
 *  Module management routines
 */
#define for_each_netcp_module(module)			\
	list_for_each_entry(module, &netcp_modules, module_list)

#define for_each_netcp_device_module(netcp_device, inst_modpriv) \
	list_for_each_entry(inst_modpriv, \
		&((netcp_device)->modpriv_head), inst_list)

int netcp_register_module(struct netcp_module *module)
{
	struct netcp_module *tmp;
	struct netcp_device *netcp_device;
	int ret;

	BUG_ON(!module->name);
	BUG_ON(!module->probe);

	mutex_lock(&netcp_modules_lock);

	ret = -EEXIST;
	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, module->name))
			goto done;
	}

	list_add_tail(&module->module_list, &netcp_modules);

	list_for_each_entry(netcp_device, &netcp_devices, device_list) {
		struct platform_device *pdev = netcp_device->platform_device;
		struct device_node *node = pdev->dev.of_node;
		struct device_node *child;
		struct netcp_inst_modpriv *inst_modpriv;
		struct netcp_priv *netcp_priv;

		/* Find this module in the sub-tree for this device */
		for_each_child_of_node(node, child) {
			const char *name = netcp_node_name(child);
			if (!strcasecmp(module->name, name))
				break;
		}

		/* If module not used for this device, skip it */
		if (child == NULL)
			continue;

		inst_modpriv = kzalloc(sizeof(*inst_modpriv), GFP_KERNEL);
		if (!inst_modpriv) {
			dev_err(&pdev->dev, "Failed to allocate instance for for %s\n", pdev->name);
			continue;	/* FIXME: Fail instead? */
		}
		inst_modpriv->netcp_device = netcp_device;
		inst_modpriv->netcp_module = module;
		list_add_tail(&inst_modpriv->inst_list, &netcp_device->modpriv_head);

		dev_dbg(&pdev->dev, "%s(): probing module \"%s\"\n", __func__, module->name);

		ret = module->probe(netcp_device, &pdev->dev, child, &inst_modpriv->module_priv);
		if (ret) {
			dev_warn(&pdev->dev, "Probe of module %s failed with %d\n",
					module->name, ret);
			list_del(&inst_modpriv->inst_list);
			kfree(inst_modpriv);
			continue;
		}

		/* Attach module to interfaces */
		list_for_each_entry(netcp_priv, &netcp_device->interface_head, interface_list) {
			struct netcp_intf_modpriv *intf_modpriv;
			int found = 0;

			list_for_each_entry(intf_modpriv, &netcp_priv->module_head, intf_list) {
				if (intf_modpriv->netcp_module == module) {
					found = 1;
					break;
				}
			}

			if (!found) {
				intf_modpriv = kzalloc(sizeof(*intf_modpriv), GFP_KERNEL);
				if (!intf_modpriv) {
					dev_err(&pdev->dev, "Error allocating intf_modpriv for %s\n",
							module->name);
					continue;
				}

				intf_modpriv->netcp_priv = netcp_priv;
				intf_modpriv->netcp_module = module;
				list_add_tail(&intf_modpriv->intf_list, &netcp_priv->module_head);

				dev_dbg(&pdev->dev, "Attaching module \"%s\"\n", module->name);
				ret = module->attach(inst_modpriv->module_priv, netcp_priv->ndev, &intf_modpriv->module_priv);
				if (ret) {
					dev_info(&pdev->dev, "Attach of module %s declined with %d\n",
							module->name, ret);
					list_del(&intf_modpriv->intf_list);
					kfree(intf_modpriv);
					continue;
				}

			}
		}
	}

	ret = 0;

done:
	mutex_unlock(&netcp_modules_lock);
	return ret;
}
EXPORT_SYMBOL(netcp_register_module);

static struct netcp_module *__netcp_find_module(const char *name)
{
	struct netcp_module *tmp;

	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, name))
			return tmp;
	}
	return NULL;
}

static struct netcp_module *netcp_find_module(const char *name)
{
	struct netcp_module *module;

	mutex_lock(&netcp_modules_lock);
	module = __netcp_find_module(name);
	mutex_unlock(&netcp_modules_lock);
	return module;
}

static void *__netcp_device_find_module(struct netcp_device *netcp_device,
					 const char *name)
{
	struct netcp_inst_modpriv *tmp;

	for_each_netcp_device_module(netcp_device, tmp) {
		if (!strcasecmp(tmp->netcp_module->name, name))
			return tmp->module_priv;
	}
	return NULL;
}

void *netcp_device_find_module(struct netcp_device *netcp_device,
		const char *name)
{
	void *module;

	mutex_lock(&netcp_modules_lock);
	module = __netcp_device_find_module(netcp_device, name);
	mutex_unlock(&netcp_modules_lock);
	return module;
}
EXPORT_SYMBOL(netcp_device_find_module);

void netcp_unregister_module(struct netcp_module *module)
{
	struct netcp_device *netcp_device;
	struct netcp_module *module_tmp;

	mutex_lock(&netcp_modules_lock);

	list_for_each_entry(netcp_device, &netcp_devices, device_list) {
		struct netcp_priv *netcp_priv, *netcp_tmp;
		struct netcp_inst_modpriv *inst_modpriv, *inst_tmp;

		/* Release the module from each interface */
		list_for_each_entry_safe(netcp_priv, netcp_tmp, 
				&netcp_device->interface_head, interface_list) {
			struct netcp_intf_modpriv *intf_modpriv, *intf_tmp;

			list_for_each_entry_safe(intf_modpriv, intf_tmp,
					&netcp_priv->module_head, intf_list) {
				if (intf_modpriv->netcp_module == module) {
					module->release(intf_modpriv->module_priv);
					list_del(&intf_modpriv->intf_list);
					kfree(intf_modpriv);
					break;
				}
			}
		}

		/* Remove the module from each instance */
		list_for_each_entry_safe(inst_modpriv, inst_tmp,
				&netcp_device->modpriv_head, inst_list) {
			if (inst_modpriv->netcp_module == module) {
				module->remove(netcp_device, inst_modpriv->module_priv);
				list_del(&inst_modpriv->inst_list);
				kfree(inst_modpriv);
				break;
			}
		}
	}

	/* Remove the module from the module list */
	for_each_netcp_module(module_tmp) {
		if (module == module_tmp) {
			list_del(&module->module_list);
			break;
		}
	}

	mutex_unlock(&netcp_modules_lock);
}
EXPORT_SYMBOL(netcp_unregister_module);


/*
 *  Module TX and RX Hook management
 */
struct netcp_hook_list {
	netcp_hook_rtn		*hook_rtn;
	void			*hook_data;
	int			 order;
};

int netcp_register_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list *old_array;
	struct netcp_hook_list *new_array;
	struct netcp_hook_list *old_entry;
	struct netcp_hook_list *new_entry;
	int before = 0;
	int after = 0;
	unsigned long flags;

	spin_lock_irqsave(&netcp_priv->lock, flags);

	old_array = netcp_priv->txhook_list_array;
	if (old_array != NULL) {
		for (old_entry = old_array; old_entry->hook_rtn; ++old_entry) {
			if (old_entry->order > order)
				break;
			++before;
		}

		for (; old_entry->hook_rtn; ++old_entry)
			++after;
	}

	new_array = kmalloc((sizeof(*new_entry) * (before + after + 1)) +
			    sizeof(void *), GFP_ATOMIC);
	if (new_array == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOMEM;
	}

	old_entry = old_array;
	new_entry = new_array;
	while (before--)
		*new_entry++ = *old_entry++;

	new_entry->hook_rtn  = hook_rtn;
	new_entry->hook_data = hook_data;
	new_entry->order     = order;
	new_entry++;

	while (after--)
		*new_entry++ = *old_entry++;

	new_entry->hook_rtn = NULL;

	rcu_assign_pointer(netcp_priv->txhook_list_array, new_array);
	spin_unlock_irqrestore(&netcp_priv->lock, flags);
	synchronize_rcu();

	if (old_array != NULL)
		kfree(old_array);

	return 0;
}
EXPORT_SYMBOL(netcp_register_txhook);

int netcp_unregister_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list *old_array;
	struct netcp_hook_list *new_array;
	struct netcp_hook_list *old_entry;
	struct netcp_hook_list *new_entry;
	int before = 0;
	int after = 0;
	unsigned long flags;

	spin_lock_irqsave(&netcp_priv->lock, flags);

	old_array = netcp_priv->txhook_list_array;
	if (old_array == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOENT;
	}
		
	for (old_entry = old_array; old_entry->hook_rtn; ++old_entry) {
		if ((old_entry->order     == order) &&
		    (old_entry->hook_rtn  == hook_rtn) &&
		    (old_entry->hook_data == hook_data))
			break;
		++before;
	}
	
	if (old_entry->hook_rtn == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOENT;
	}
	old_entry++;

	for (; old_entry->hook_rtn; ++old_entry)
		++after;

	if ((before + after) == 0) {
		new_array = NULL;
	} else {
		new_array = kmalloc((sizeof(*new_entry) * (before + after)) +
				    sizeof(void *), GFP_ATOMIC);
		if (new_array == NULL) {
			spin_unlock_irqrestore(&netcp_priv->lock, flags);
			return -ENOMEM;
		}

		old_entry = old_array;
		new_entry = new_array;
		while (before--)
			*new_entry++ = *old_entry++;

		old_entry++;

		while (after--)
			*new_entry++ = *old_entry++;

		new_entry->hook_rtn = NULL;
	}

	rcu_assign_pointer(netcp_priv->txhook_list_array, new_array);
	spin_unlock_irqrestore(&netcp_priv->lock, flags);
	synchronize_rcu();

	kfree(old_array);
	return 0;
}
EXPORT_SYMBOL(netcp_unregister_txhook);

int netcp_register_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list *old_array;
	struct netcp_hook_list *new_array;
	struct netcp_hook_list *old_entry;
	struct netcp_hook_list *new_entry;
	int before = 0;
	int after = 0;
	unsigned long flags;

	spin_lock_irqsave(&netcp_priv->lock, flags);

	old_array = netcp_priv->rxhook_list_array;
	if (old_array != NULL) {
		for (old_entry = old_array; old_entry->hook_rtn; ++old_entry) {
			if (old_entry->order > order)
				break;
			++before;
		}

		for (; old_entry->hook_rtn; ++old_entry)
			++after;
	}

	new_array = kmalloc((sizeof(*new_entry) * (before + after + 1)) +
			    sizeof(void *), GFP_ATOMIC);
	if (new_array == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOMEM;
	}

	old_entry = old_array;
	new_entry = new_array;
	while (before--)
		*new_entry++ = *old_entry++;

	new_entry->hook_rtn  = hook_rtn;
	new_entry->hook_data = hook_data;
	new_entry->order     = order;
	new_entry++;

	while (after--)
		*new_entry++ = *old_entry++;

	new_entry->hook_rtn = NULL;

	rcu_assign_pointer(netcp_priv->rxhook_list_array, new_array);
	spin_unlock_irqrestore(&netcp_priv->lock, flags);
	synchronize_rcu();

	if (old_array != NULL)
		kfree(old_array);

	return 0;
}
EXPORT_SYMBOL(netcp_register_rxhook);

int netcp_unregister_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list *old_array;
	struct netcp_hook_list *new_array;
	struct netcp_hook_list *old_entry;
	struct netcp_hook_list *new_entry;
	int before = 0;
	int after = 0;
	unsigned long flags;

	spin_lock_irqsave(&netcp_priv->lock, flags);

	old_array = netcp_priv->rxhook_list_array;
	if (old_array == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOENT;
	}
		
	for (old_entry = old_array; old_entry->hook_rtn; ++old_entry) {
		if ((old_entry->order     == order) &&
		    (old_entry->hook_rtn  == hook_rtn) &&
		    (old_entry->hook_data == hook_data))
			break;
		++before;
	}
	
	if (old_entry->hook_rtn == NULL) {
		spin_unlock_irqrestore(&netcp_priv->lock, flags);
		return -ENOENT;
	}
	old_entry++;

	for (; old_entry->hook_rtn; ++old_entry)
		++after;

	if ((before + after) == 0) {
		new_array = NULL;
	} else {
		new_array = kmalloc((sizeof(*new_entry) * (before + after)) +
				    sizeof(void *), GFP_ATOMIC);
		if (new_array == NULL) {
			spin_unlock_irqrestore(&netcp_priv->lock, flags);
			return -ENOMEM;
		}

		old_entry = old_array;
		new_entry = new_array;
		while (before--)
			*new_entry++ = *old_entry++;

		old_entry++;

		while (after--)
			*new_entry++ = *old_entry++;

		new_entry->hook_rtn = NULL;
	}

	rcu_assign_pointer(netcp_priv->rxhook_list_array, new_array);
	spin_unlock_irqrestore(&netcp_priv->lock, flags);
	synchronize_rcu();

	kfree(old_array);
	return 0;
}
EXPORT_SYMBOL(netcp_unregister_rxhook);


#define NETCP_DEBUG (NETIF_MSG_HW	| NETIF_MSG_WOL		|	\
		    NETIF_MSG_DRV	| NETIF_MSG_LINK	|	\
		    NETIF_MSG_IFUP	| NETIF_MSG_INTR	|	\
		    NETIF_MSG_PROBE	| NETIF_MSG_TIMER	|	\
		    NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	|	\
		    NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	|	\
		    NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	|	\
		    NETIF_MSG_RX_STATUS)

#define NETCP_NAPI_WEIGHT_RX	64
#define NETCP_NAPI_WEIGHT_TX	64
#define NETCP_TX_TIMEOUT	(5 * HZ)
#define NETCP_MIN_PACKET_SIZE	ETH_ZLEN
#define NETCP_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static int netcp_rx_packet_max = NETCP_MAX_PACKET_SIZE;
static int netcp_debug_level;

#define for_each_module(netcp, intf_modpriv)			\
	list_for_each_entry(intf_modpriv, &netcp->module_head, intf_list)

static const char *netcp_rx_state_str(struct netcp_priv *netcp)
{
	static const char * const state_str[] = {
		[RX_STATE_INVALID]	= "invalid",
		[RX_STATE_INTERRUPT]	= "interrupt",
		[RX_STATE_SCHEDULED]	= "scheduled",
		[RX_STATE_POLL]		= "poll",
		[RX_STATE_TEARDOWN]	= "teardown",
	};

	if (netcp->rx_state < 0 || netcp->rx_state >= ARRAY_SIZE(state_str))
		return state_str[RX_STATE_INVALID];
	else
		return state_str[netcp->rx_state];
}

static inline void netcp_set_rx_state(struct netcp_priv *netcp,
				     enum netcp_rx_state state)
{
	netcp->rx_state = state;
	smp_wmb();
}

static inline bool netcp_is_alive(struct netcp_priv *netcp)
{
	return (netcp->rx_state == RX_STATE_POLL ||
		netcp->rx_state == RX_STATE_INTERRUPT);
}

#ifdef DEBUG
static void netcp_dump_packet(struct netcp_packet *p_info, const char *cause)
{
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	unsigned char *head, *tail;

	head = skb->data;
	tail = skb_tail_pointer(skb) - 16;

	dev_dbg(netcp->dev, "packet %p %s, size %d (%d): "
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x\n",
		p_info, cause, skb->len, p_info->sg[2].length,
		head[0x00], head[0x01], head[0x02], head[0x03],
		head[0x04], head[0x05], head[0x06], head[0x07],
		head[0x08], head[0x09], head[0x0a], head[0x0b],
		head[0x0c], head[0x0d], head[0x0e], head[0x0f],
		tail[0x00], tail[0x01], tail[0x02], tail[0x03],
		tail[0x04], tail[0x05], tail[0x06], tail[0x07],
		tail[0x08], tail[0x09], tail[0x0a], tail[0x0b],
		tail[0x0c], tail[0x0d], tail[0x0e], tail[0x0f]);
}
#endif

static inline void netcp_frag_free(bool is_frag, void *ptr)
{
	if (is_frag)
		put_page(virt_to_head_page(ptr));
	else
		kfree(ptr);
}

static void netcp_rx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	struct netcp_stats *rx_stats = &netcp->stats;
	struct sk_buff *skb;
	struct scatterlist *sg;
	enum dma_status status;
	unsigned int frags;
	struct netcp_hook_list *rx_hook;
	unsigned int len;

	status = dma_async_is_tx_complete(netcp->rx_channel,
					  p_info->cookie, NULL, NULL);
	WARN_ONCE((status != DMA_COMPLETE && status != DMA_ERROR),
		"in netcp_rx_complete dma status: %d\n", status);
	WARN_ONCE((netcp->rx_state != RX_STATE_POLL	 &&
		   netcp->rx_state != RX_STATE_TEARDOWN),
		"in netcp_rx_complete rx_state: (%d) %s\n",
			netcp->rx_state, netcp_rx_state_str(netcp));

	/* sg[3] describes the primary buffer */
	/* Build a new sk_buff for this buffer */
	dma_unmap_single(&netcp->pdev->dev, sg_dma_address(&p_info->sg[3]),
			p_info->primary_datsiz, DMA_FROM_DEVICE);
	skb = build_skb(p_info->primary_bufptr, p_info->primary_bufsiz);
	if (unlikely(!skb)) {
		 /* Free the primary buffer */
		netcp_frag_free((p_info->primary_bufsiz <= PAGE_SIZE), 
				p_info->primary_bufptr);
		 /* Free other buffers in the scatterlist */
		for (frags = 0, sg = sg_next(&p_info->sg[3]);
				frags < NETCP_SGLIST_SIZE - 4 && sg;
				++frags, sg = sg_next(sg)) {
			dma_unmap_page(&netcp->pdev->dev, sg_dma_address(sg),
					PAGE_SIZE, DMA_FROM_DEVICE);
			__free_page(sg_page(sg));
		}
		return;
	}
	p_info->skb = skb;
	skb->dev = netcp->ndev;

	/* Update data, tail, and len */
	skb_reserve(skb, NET_IP_ALIGN + NET_SKB_PAD);
	len = sg_dma_len(&p_info->sg[3]);
	__skb_put(skb, len);

	/* Fill in the page fragment list from sg[4] and later */
	for (frags = 0, sg = sg_next(&p_info->sg[3]);
			frags < NETCP_SGLIST_SIZE - 4 && sg;
			++frags, sg = sg_next(sg)) {
		dma_unmap_page(&netcp->pdev->dev, sg_dma_address(sg),
				PAGE_SIZE, DMA_FROM_DEVICE);
		len = sg_dma_len(sg);
		skb_add_rx_frag(skb, frags, sg_page(sg), sg->offset, len, PAGE_SIZE);
	}

	/* Remove FCS from the packet (last 4 bytes) for platforms
	 * that don't have capability to do this in CPSW
	 */
	if (!(netcp->hw_capabilities & CPSW_HAS_P0_TX_CRC_REMOVE))
		__pskb_trim(skb, skb->len - ETH_FCS_LEN);

	if (unlikely(netcp->rx_state == RX_STATE_TEARDOWN)) {
		dev_dbg(netcp->dev,
			"receive: reclaimed packet %p, status %d, state %s\n",
			p_info, status, netcp_rx_state_str(netcp));
		dev_kfree_skb(skb);
		kmem_cache_free(netcp_pinfo_cache, p_info);
		rx_stats->rx_dropped++;
		return;
	}

	if (unlikely(status != DMA_COMPLETE)) {
		dev_warn(netcp->dev,
			 "receive: reclaimed packet %p, status %d, state %s\n",
			 p_info, status, netcp_rx_state_str(netcp));
		dev_kfree_skb(skb);
		kmem_cache_free(netcp_pinfo_cache, p_info);
		rx_stats->rx_errors++;
		return;
	}

	if (unlikely(!skb->len)) {
		dev_warn(netcp->dev, "receive: zero length packet\n");
		dev_kfree_skb(skb);
		kmem_cache_free(netcp_pinfo_cache, p_info);
		rx_stats->rx_errors++;
		return;
	}

#ifdef DEBUG
	netcp_dump_packet(p_info, "rx");
#endif

	/* Call each of the RX hooks */
	p_info->rxtstamp_complete = false;
	rcu_read_lock();
	rx_hook = rcu_dereference(netcp->rxhook_list_array);
	if (rx_hook) {
		for (; rx_hook->hook_rtn; ++rx_hook) {
			int ret;
			ret = rx_hook->hook_rtn(rx_hook->order,
						rx_hook->hook_data,
						p_info);
			if (ret) {
				rcu_read_unlock();
				dev_err(netcp->dev, "RX hook %d failed: %d\n",
					rx_hook->order, ret);
				dev_kfree_skb(skb);
				kmem_cache_free(netcp_pinfo_cache, p_info);
				return;
			}
		}
	}
	rcu_read_unlock();

	u64_stats_update_begin(&rx_stats->syncp_rx);
	rx_stats->rx_packets++;
	rx_stats->rx_bytes += skb->len;
	u64_stats_update_end(&rx_stats->syncp_rx);

	kmem_cache_free(netcp_pinfo_cache, p_info);

	/* push skb up the stack */
	skb->protocol = eth_type_trans(skb, netcp->ndev);
	netif_receive_skb(skb);
}

/*
 *  Another ugly layering violation. We can only pass one
 *  value describing a secondary buffer, but we need both
 *  the page and dma addresses. -rrp
 */
#define	page_to_dma(dev,pg)	pfn_to_dma(dev, page_to_pfn(pg))

/* Release a free receive buffer */
static void netcp_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc)
{
	struct netcp_priv *netcp = arg;
	struct device *dev = &netcp->pdev->dev;

	if (q_num == 0) {
		struct netcp_packet *p_info = desc->callback_param;

		dma_unmap_single(dev, sg_dma_address(&p_info->sg[3]),
				p_info->primary_datsiz, DMA_FROM_DEVICE);
		netcp_frag_free((p_info->primary_bufsiz <= PAGE_SIZE), 
				p_info->primary_bufptr);
		kmem_cache_free(netcp_pinfo_cache, p_info);
	} else {
		struct page *page = desc->callback_param;
		dma_unmap_page(dev, page_to_dma(dev, page),
				PAGE_SIZE, DMA_FROM_DEVICE);
		__free_page(page);
	}
}

static void netcp_rx_complete2nd(void *data)
{
	struct page *page = data;

	WARN(1, "Attempt to complete secondary receive buffer!\n");
	/* FIXME: We need to unmap the page, but don't have the info needed */
	__free_page(page);
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *netcp_rxpool_alloc(void *arg,
		unsigned q_num, unsigned size)
{
	struct netcp_priv *netcp = arg;
	struct dma_async_tx_descriptor *desc;
	int err = 0;

	if (q_num == 0) {
		struct netcp_packet *p_info;
		void *bufptr;
		unsigned char *data;
		unsigned int bufsiz;

		/* Allocate a primary receive queue entry */
		p_info = kmem_cache_alloc(netcp_pinfo_cache, GFP_ATOMIC);
		if (!p_info) {
			dev_err(netcp->dev, "packet alloc failed\n");
			return NULL;
		}
		p_info->skb = NULL;
		p_info->netcp = netcp;

		bufsiz = SKB_DATA_ALIGN(size + NET_IP_ALIGN + NET_SKB_PAD) +
			 SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
		if (bufsiz <= PAGE_SIZE) {
			bufptr = netdev_alloc_frag(bufsiz);
			p_info->primary_bufsiz = bufsiz;
		} else {
			bufptr = kmalloc(bufsiz, GFP_ATOMIC);
			p_info->primary_bufsiz = 0;
		}
		if (!bufptr) {
			dev_warn(netcp->dev, "Primary RX buffer allocation failed\n");
			kmem_cache_free(netcp_pinfo_cache, p_info);
			return NULL;
		}
		p_info->primary_bufptr = bufptr;
		p_info->primary_datsiz = size;
		
		/* Same as skb_reserve(skb, NET_IP_ALIGN + NET_SKB_PAD) */
		data = bufptr + NET_IP_ALIGN + NET_SKB_PAD;

		sg_init_table(p_info->sg, NETCP_SGLIST_SIZE);
		sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
		sg_set_buf(&p_info->sg[1], p_info->psdata,
			   NETCP_MAX_RX_PSDATA_LEN * sizeof(u32));
		sg_set_buf(&p_info->sg[2], &p_info->eflags,
			   sizeof(p_info->eflags));
		sg_set_buf(&p_info->sg[3], data, size);

		p_info->sg_ents = 3 + dma_map_sg(&netcp->pdev->dev,
						 &p_info->sg[3], 1,
						 DMA_FROM_DEVICE);
		if (p_info->sg_ents != 4) {
			dev_err(netcp->dev, "dma map failed\n");
			netcp_frag_free((bufsiz <= PAGE_SIZE), bufptr);
			kmem_cache_free(netcp_pinfo_cache, p_info);
			return NULL;
		}

		desc = dmaengine_prep_slave_sg(netcp->rx_channel, p_info->sg,
					       4, DMA_DEV_TO_MEM,
					       DMA_HAS_EPIB | DMA_HAS_PSINFO |
					       DMA_HAS_EFLAGS);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(&netcp->pdev->dev, &p_info->sg[3], 1,
				     DMA_FROM_DEVICE);
			netcp_frag_free((bufsiz <= PAGE_SIZE), bufptr);
			kmem_cache_free(netcp_pinfo_cache, p_info);
			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
			}
			return NULL;
		}

		desc->callback_param = p_info;
		desc->callback = netcp_rx_complete;
		p_info->cookie = desc->cookie;

	} else {
		struct page *page;

		/* Allocate a secondary receive queue entry */
		struct scatterlist sg[1];

		page = alloc_page(GFP_ATOMIC | GFP_DMA32);
		if (!page) {
			dev_warn(netcp->dev, "page alloc failed for pool %d\n", q_num);
			return NULL;
		}

		sg_init_table(sg, 1);
		sg_set_page(&sg[0], page, PAGE_SIZE, 0);

		err = dma_map_sg(&netcp->pdev->dev, sg, 1, DMA_FROM_DEVICE);
		if (err != 1) {
			dev_warn(netcp->dev, "map error for pool %d\n", q_num);
			__free_page(page);
			return NULL;
		}

		desc = dmaengine_prep_slave_sg(netcp->rx_channel, sg, 1,
					       DMA_DEV_TO_MEM,
					       q_num << DMA_QNUM_SHIFT);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(&netcp->pdev->dev, sg, 1, DMA_FROM_DEVICE);
			__free_page(page);

			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
			}
			return NULL;
		}

		desc->callback_param = page;
		desc->callback = netcp_rx_complete2nd;
	}

	wmb();
	return desc;
}

/* NAPI poll */
static int netcp_poll(struct napi_struct *napi, int budget)
{
	struct netcp_priv *netcp = container_of(napi, struct netcp_priv, napi);
	unsigned packets;

	BUG_ON(netcp->rx_state != RX_STATE_SCHEDULED);
	netcp_set_rx_state(netcp, RX_STATE_POLL);

	packets = dma_poll(netcp->rx_channel, budget);

	if (packets < budget) {
		netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);
		napi_complete(&netcp->napi);
		dmaengine_resume(netcp->rx_channel);
	} else {
		netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	}

	dma_rxfree_refill(netcp->rx_channel);

	return packets;
}

static void netcp_rx_notify(struct dma_chan *chan, void *arg)
{
	struct netcp_priv *netcp = arg;
	enum netcp_rx_state rx_state;

	rx_state = netcp->rx_state;
	if (rx_state != RX_STATE_INTERRUPT) {
		WARN_ONCE((rx_state != RX_STATE_TEARDOWN),
			"rx_state == %d: %s",
			rx_state, netcp_rx_state_str(netcp));
		return;
	}

	dmaengine_pause(netcp->rx_channel);
	netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	napi_schedule(&netcp->napi);
}

static const char *netcp_tx_state_str(enum netcp_tx_state tx_state)
{
	static const char * const state_str[] = {
		[TX_STATE_INTERRUPT]	= "interrupt",
		[TX_STATE_POLL]		= "poll",
		[TX_STATE_SCHEDULED]	= "scheduled",
		[TX_STATE_INVALID]	= "invalid",
	};

	if (tx_state < 0 || tx_state >= ARRAY_SIZE(state_str))
		return state_str[TX_STATE_INVALID];
	else
		return state_str[tx_state];
}

static inline void netcp_set_txpipe_state(struct netcp_tx_pipe *tx_pipe,
					  enum netcp_tx_state new_state)
{
	dev_dbg(tx_pipe->netcp_priv->dev, "txpipe %s: %s -> %s\n",
		tx_pipe->dma_chan_name,
		netcp_tx_state_str(tx_pipe->dma_poll_state),
		netcp_tx_state_str(new_state));

	tx_pipe->dma_poll_state = new_state;
	smp_wmb();
}

static int netcp_tx_unmap_skb(struct device *dev, struct sk_buff *skb, int offset, int len)
{
	int start = skb_headlen(skb);
	int i, copy = start - offset;
	struct sk_buff *frag_iter;
	int elt = 0;

	if (copy > 0) {
		if (copy > len)
			copy = len;
		dma_unmap_single(dev, virt_to_dma(dev, skb->data + offset),
				copy, DMA_TO_DEVICE);
		elt++;
		if ((len -= copy) == 0)
			return elt;
		offset += copy;
	}

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		void *virt_addr;
		int end;

		WARN_ON(start > offset + len);

		end = start + skb_frag_size(&skb_shinfo(skb)->frags[i]);
		if ((copy = end - offset) > 0) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

			if (copy > len)
				copy = len;
			virt_addr = skb_frag_address(frag) + offset - start;
			dma_unmap_page(dev, virt_to_dma(dev, virt_addr),
					copy, DMA_TO_DEVICE);
			elt++;
			if (!(len -= copy))
				return elt;
			offset += copy;
		}
		start = end;
	}

	skb_walk_frags(skb, frag_iter) {
		int end;

		WARN_ON(start > offset + len);

		end = start + frag_iter->len;
		if ((copy = end - offset) > 0) {
			if (copy > len)
				copy = len;
			elt += netcp_tx_unmap_skb(dev, frag_iter,
						offset - start, copy);
			if ((len -= copy) == 0)
				return elt;
			offset += copy;
		}
		start = end;
	}
	BUG_ON(len);
	return elt;
}

static void netcp_tx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct sk_buff *skb = p_info->skb;
	struct netcp_priv *netcp = netdev_priv(skb->dev);
	struct netcp_tx_pipe *tx_pipe = p_info->tx_pipe;
	unsigned int ents;
	int poll_count;

	ents = netcp_tx_unmap_skb(&netcp->pdev->dev, skb, 0, skb->len);

#ifdef DEBUG
	netcp_dump_packet(p_info, "txc");
#endif

	if (p_info->txtstamp_complete)
		p_info->txtstamp_complete(p_info->ts_context, skb);

	poll_count = atomic_add_return(ents, &tx_pipe->dma_poll_count);
	if ((poll_count >= tx_pipe->dma_resume_threshold) &&
	    netif_subqueue_stopped(netcp->ndev, skb)) {
		u16 subqueue = skb_get_queue_mapping(skb);
		dev_dbg(netcp->dev, "waking subqueue %hu, %s poll count %d\n",
			subqueue, tx_pipe->dma_chan_name, poll_count);
		netif_wake_subqueue(netcp->ndev, subqueue);
	}

	dev_kfree_skb(skb);
	kfree(p_info);
}

static int netcp_tx_poll(struct napi_struct *napi, int budget)
{
	struct netcp_tx_pipe *tx_pipe =
		container_of(napi, struct netcp_tx_pipe, dma_poll_napi);
	int poll_count, packets;

	if (unlikely(tx_pipe->dma_poll_state != TX_STATE_SCHEDULED)) {
		WARN(1, "spurious netcp_tx_poll activation, txpipe %s state %d",
			tx_pipe->dma_chan_name, tx_pipe->dma_poll_state);
		return 0;
	}

	netcp_set_txpipe_state(tx_pipe, TX_STATE_POLL);

	packets = dma_poll(tx_pipe->dma_channel, budget);
	poll_count = atomic_read(&tx_pipe->dma_poll_count);

	if (packets < budget) {
		netcp_set_txpipe_state(tx_pipe, TX_STATE_INTERRUPT);
		napi_complete(&tx_pipe->dma_poll_napi);
		dmaengine_resume(tx_pipe->dma_channel);
	} else {
		netcp_set_txpipe_state(tx_pipe, TX_STATE_SCHEDULED);
	}

	dev_dbg(tx_pipe->netcp_priv->dev,
		"txpipe %s poll count %d, packets %d\n",
		tx_pipe->dma_chan_name, poll_count, packets);
	
	return packets;
}

static void netcp_tx_notify(struct dma_chan *chan, void *arg)
{
	struct netcp_tx_pipe *tx_pipe = arg;
	enum netcp_tx_state tx_state;

	tx_state = tx_pipe->dma_poll_state;
	if (tx_state != TX_STATE_INTERRUPT) {
		WARN_ONCE(true, "tx_state == %d: %s",
			tx_state, netcp_tx_state_str(tx_state));
		return;
	}

	dmaengine_pause(tx_pipe->dma_channel);
	netcp_set_txpipe_state(tx_pipe, TX_STATE_SCHEDULED);
	napi_schedule(&tx_pipe->dma_poll_napi);
}

/* Push an outcoming packet */
static int netcp_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_stats *tx_stats = &netcp->stats;
	struct dma_async_tx_descriptor *desc;
	struct netcp_tx_pipe *tx_pipe;
	struct netcp_hook_list *tx_hook;
	struct netcp_packet *p_info;
	int subqueue = skb_get_queue_mapping(skb);
	dma_cookie_t cookie;
	int real_sg_ents = 0;
	int poll_count;
	int ret = 0;

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		dev_err(netcp->dev, "packet alloc failed\n");
		return -ENOMEM;
	}

	u64_stats_update_begin(&tx_stats->syncp_tx);
	tx_stats->tx_packets++;
	tx_stats->tx_bytes += skb->len;
	u64_stats_update_end(&tx_stats->syncp_tx);

	p_info->netcp = netcp;
	p_info->skb = skb;

	/* Find out where to inject the packet for transmission */
	rcu_read_lock();
	tx_hook = rcu_dereference(netcp->txhook_list_array);
	if (tx_hook) {
		for (; tx_hook->hook_rtn; ++tx_hook) {
			ret = tx_hook->hook_rtn(tx_hook->order,
						tx_hook->hook_data,
						p_info);
			if (unlikely(ret != 0)) {
				if (ret < 0) {
					dev_err(netcp->dev, "TX hook %d "
						"rejected the packet: %d\n",
						tx_hook->order, ret);
				}
				rcu_read_unlock();
				dev_kfree_skb(skb);
				return (ret < 0) ? ret : NETDEV_TX_OK;
			}
		}
	}
	rcu_read_unlock();

	/* Make sure some TX hook claimed the packet */
	tx_pipe = p_info->tx_pipe;
	if (tx_pipe == NULL) {
		dev_err(netcp->dev, "No TX hook claimed the packet!\n");
		ret = -ENXIO;
		goto drop;
	}

	if (unlikely(skb->len < NETCP_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, NETCP_MIN_PACKET_SIZE);
		if (ret < 0) {
			/* If we get here, the skb has already been dropped */
			dev_warn(netcp->dev, "padding failed (%d), "
				 "packet dropped\n", ret);
			tx_stats->tx_dropped++;
			return ret;
		}
		skb->len = NETCP_MIN_PACKET_SIZE;
	}

#ifdef DEBUG
	netcp_dump_packet(p_info, "txs");
#endif

	skb_tx_timestamp(skb);

	sg_init_table(p_info->sg, NETCP_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], &p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len],
			p_info->psdata_len * sizeof(u32));
	real_sg_ents = skb_to_sgvec(skb, &p_info->sg[2], 0, skb->len);

	/* Reserve descriptors for this packet */
	poll_count = atomic_sub_return(real_sg_ents, &tx_pipe->dma_poll_count);
	if (poll_count < tx_pipe->dma_pause_threshold) {
		dev_dbg(netcp->dev, "pausing subqueue %d, %s poll count %d\n",
			subqueue, tx_pipe->dma_chan_name, poll_count);
		netif_stop_subqueue(ndev, subqueue);
		if (poll_count < 0) {
			ret = -ENOBUFS;
			goto drop;
		}
	}

	/* Map all the packet fragments	into the scatterlist */
	p_info->sg_ents = 2 + dma_map_sg(&netcp->pdev->dev, &p_info->sg[2],
					 real_sg_ents, DMA_TO_DEVICE);
	if (p_info->sg_ents != (real_sg_ents + 2)) {
		dev_warn(netcp->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto drop;
	}

	desc = dmaengine_prep_slave_sg(tx_pipe->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_MEM_TO_DEV,
				       (DMA_HAS_EPIB | DMA_HAS_PSINFO |
					tx_pipe->dma_psflags));

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(&netcp->pdev->dev, &p_info->sg[2], real_sg_ents,
			     DMA_TO_DEVICE);
		dev_warn(netcp->dev, "failed to prep slave dma\n");
		ret = -ENOBUFS;
		goto drop;
	}

	desc->callback_param = p_info;
	desc->callback = netcp_tx_complete;
	wmb();

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		dev_warn(netcp->dev, "failed to submit packet for dma: %d\n",
				cookie);
		goto drop;
	}

	netif_trans_update(ndev);
	return NETDEV_TX_OK;

drop:
	atomic_add(real_sg_ents, &tx_pipe->dma_poll_count);
	tx_stats->tx_dropped++;
	dev_kfree_skb(skb);
	kfree(p_info);
	return ret;
}


int netcp_txpipe_close(struct netcp_tx_pipe *tx_pipe)
{
	if (tx_pipe->dma_channel) {
		napi_disable(&tx_pipe->dma_poll_napi);
		dmaengine_pause(tx_pipe->dma_channel);
		dma_poll(tx_pipe->dma_channel, -1);
		dma_release_channel(tx_pipe->dma_channel);
		tx_pipe->dma_channel = NULL;
		netcp_set_txpipe_state(tx_pipe, TX_STATE_INVALID);

		dev_dbg(tx_pipe->netcp_priv->dev, "closed tx pipe %s\n",
			tx_pipe->dma_chan_name);
	}

	return 0;
}
EXPORT_SYMBOL(netcp_txpipe_close);

int netcp_txpipe_open(struct netcp_tx_pipe *tx_pipe)
{
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int ret, queue_depth;

	queue_depth = tx_pipe->dma_queue_depth;
	tx_pipe->dma_pause_threshold = (MAX_SKB_FRAGS < (queue_depth / 4)) ?
					MAX_SKB_FRAGS : (queue_depth / 4);
	tx_pipe->dma_resume_threshold = tx_pipe->dma_pause_threshold;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	tx_pipe->dma_channel = dma_request_channel_by_name(mask, tx_pipe->dma_chan_name);
	if (IS_ERR_OR_NULL(tx_pipe->dma_channel)) {
		dev_err(tx_pipe->netcp_priv->dev,
			"Could not get DMA channel \"%s\"\n",
			tx_pipe->dma_chan_name);
		tx_pipe->dma_channel = NULL;
		return -ENODEV;
	}

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.tx_queue_depth = queue_depth;
	ret = dma_keystone_config(tx_pipe->dma_channel, &config);
	if (ret) {
		dev_err(tx_pipe->netcp_priv->dev,
			"Could not configure DMA channel \"%s\": %d\n",
			tx_pipe->dma_chan_name, ret);
		dma_release_channel(tx_pipe->dma_channel);
		tx_pipe->dma_channel = NULL;
		return ret;
	}

	tx_pipe->dma_queue = dma_get_tx_queue(tx_pipe->dma_channel);
	atomic_set(&tx_pipe->dma_poll_count, queue_depth);
	netcp_set_txpipe_state(tx_pipe, TX_STATE_INTERRUPT);
	napi_enable(&tx_pipe->dma_poll_napi);
	dma_set_notify(tx_pipe->dma_channel, netcp_tx_notify, tx_pipe);


	dev_dbg(tx_pipe->netcp_priv->dev,
		"opened tx pipe %s, depth %d, pause/resume %d/%d\n",
		tx_pipe->dma_chan_name, queue_depth,
		tx_pipe->dma_pause_threshold, tx_pipe->dma_resume_threshold);
	return 0;
}
EXPORT_SYMBOL(netcp_txpipe_open);

int netcp_txpipe_init(struct netcp_tx_pipe *tx_pipe,
		struct netcp_priv *netcp_priv,
		const char *chan_name,
		int queue_depth)
{
	memset(tx_pipe, 0, sizeof(*tx_pipe));

	tx_pipe->netcp_priv = netcp_priv;
	tx_pipe->dma_chan_name = chan_name;
	tx_pipe->dma_queue_depth = queue_depth;

	netcp_set_txpipe_state(tx_pipe, TX_STATE_INVALID);
	netif_napi_add(netcp_priv->ndev, &tx_pipe->dma_poll_napi,
			netcp_tx_poll, NETCP_NAPI_WEIGHT_TX);

	dev_dbg(tx_pipe->netcp_priv->dev, "initialized tx pipe %s\n",
		tx_pipe->dma_chan_name);
	return 0;
}
EXPORT_SYMBOL(netcp_txpipe_init);

static struct netcp_addr *
netcp_addr_find(struct netcp_priv *netcp, const u8 *addr,
	       enum netcp_addr_type type)
{
	struct netcp_addr *naddr;

	list_for_each_entry(naddr, &netcp->addr_list, node) {
		if (naddr->type != type)
			continue;
		if (addr && memcmp(addr, naddr->addr, ETH_ALEN))
			continue;
		return naddr;
	}

	return NULL;
}

static struct netcp_addr *
netcp_addr_add(struct netcp_priv *netcp, const u8 *addr,
	       enum netcp_addr_type type)
{
	struct netcp_addr *naddr;

	naddr = kmalloc(sizeof(struct netcp_addr), GFP_ATOMIC);
	if (!naddr)
		return NULL;

	naddr->type = type;
	naddr->flags = 0;
	naddr->netcp = netcp;
	if (addr)
		memcpy(naddr->addr, addr, ETH_ALEN);
	else
		memset(naddr->addr, 0, ETH_ALEN);
	list_add_tail(&naddr->node, &netcp->addr_list);

	return naddr;
}

static void netcp_addr_del(struct netcp_addr *naddr)
{
	list_del(&naddr->node);
	kfree(naddr);
}

static void netcp_addr_clear_mark(struct netcp_priv *netcp)
{
	struct netcp_addr *naddr;

	list_for_each_entry(naddr, &netcp->addr_list, node)
		naddr->flags = 0;
}

static struct netcp_addr *
netcp_addr_add_mark(struct netcp_priv *netcp, const u8 *addr,
					enum netcp_addr_type type)
{
	struct netcp_addr *naddr;

	naddr = netcp_addr_find(netcp, addr, type);
	if (naddr) {
		naddr->flags |= ADDR_VALID;
		return naddr;
	}

	naddr = netcp_addr_add(netcp, addr, type);
	if (!WARN_ON(!naddr))
		naddr->flags |= ADDR_NEW;

	return naddr;
}

static void netcp_mod_del_addr(struct netcp_priv *netcp,
				struct netcp_addr *naddr)
{
	struct netcp_intf_modpriv *priv;
	struct netcp_module *module;

	dev_dbg(netcp->dev, "deleting address %pM, type %x\n",
		naddr->addr, naddr->type);
	for_each_module(netcp, priv) {
		module = priv->netcp_module;
		if (!module->del_addr)
			continue;
		module->del_addr(priv->module_priv, naddr);
	}
}

static void netcp_addr_sweep_del(struct netcp_priv *netcp)
{
	struct netcp_addr *naddr, *tmp;

	list_for_each_entry_safe(naddr, tmp, &netcp->addr_list, node) {
		if (naddr->flags & (ADDR_VALID | ADDR_NEW))
			continue;
		netcp_mod_del_addr(netcp, naddr);
		netcp_addr_del(naddr);
	}
}

static int netcp_mod_add_addr(struct netcp_priv *netcp,
				struct netcp_addr *naddr)
{
	struct netcp_intf_modpriv *priv;
	struct netcp_module *module;
	int error;

	dev_dbg(netcp->dev, "adding address %pM, type %x\n",
		naddr->addr, naddr->type);
	for_each_module(netcp, priv) {
		module = priv->netcp_module;
		if (!module->add_addr)
			continue;
		error = module->add_addr(priv->module_priv, naddr);
		if (error)
			break;
	}

	if (error)
		netcp_mod_del_addr(netcp, naddr);

	return error;
}

static void netcp_addr_sweep_add(struct netcp_priv *netcp)
{
	struct netcp_addr *naddr, *tmp;

	list_for_each_entry_safe(naddr, tmp, &netcp->addr_list, node) {
		if (!(naddr->flags & ADDR_NEW))
			continue;
		netcp_mod_add_addr(netcp, naddr);
	}
}

static void netcp_set_rx_mode(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netdev_hw_addr *ndev_addr;
	bool promisc;

	promisc = (ndev->flags & IFF_PROMISC ||
		   ndev->flags & IFF_ALLMULTI ||
		   netdev_mc_count(ndev) > NETCP_MAX_MCAST_ADDR);

	spin_lock(&netcp->lock);

	/* first clear all marks */
	netcp_addr_clear_mark(netcp);

	/* next add new entries, mark existing ones */
	netcp_addr_add_mark(netcp, ndev->broadcast, ADDR_BCAST);
	for_each_dev_addr(ndev, ndev_addr)
		netcp_addr_add_mark(netcp, ndev_addr->addr, ADDR_DEV);
	netdev_for_each_uc_addr(ndev_addr, ndev)
		netcp_addr_add_mark(netcp, ndev_addr->addr, ADDR_UCAST);
	netdev_for_each_mc_addr(ndev_addr, ndev)
		netcp_addr_add_mark(netcp, ndev_addr->addr, ADDR_MCAST);

	if (promisc)
		netcp_addr_add_mark(netcp, NULL, ADDR_ANY);

	/* finally sweep and callout into modules */
	netcp_addr_sweep_del(netcp);
	netcp_addr_sweep_add(netcp);

	spin_unlock(&netcp->lock);
}

struct dma_chan *netcp_get_rx_chan(struct netcp_priv *netcp)
{
	return netcp->rx_channel;
}
EXPORT_SYMBOL(netcp_get_rx_chan);

/* Open the device */
static int netcp_ndo_open(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_intf_modpriv *intf_modpriv;
	struct netcp_module *module;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;
	int i;

	netif_carrier_off(ndev);

	BUG_ON(netcp->rx_state != RX_STATE_INVALID);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	name = netcp->rx_chan_name;
	netcp->rx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(netcp->rx_channel)) {
		dev_err(netcp->dev, "Failed to open DMA channel \"%s\": %ld\n",
				name, PTR_ERR(netcp->rx_channel));
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction		= DMA_DEV_TO_MEM;
	config.scatterlist_size		= NETCP_SGLIST_SIZE;
	config.rxpool_allocator		= netcp_rxpool_alloc;
	config.rxpool_destructor	= netcp_rxpool_free;
	config.rxpool_param		= netcp;
	config.rxpool_thresh_enable	= DMA_THRESH_NONE;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN &&
		    netcp->rx_queue_depths[i] &&
		    netcp->rx_buffer_sizes[i]; ++i) {
		config.rxpools[i].pool_depth  = netcp->rx_queue_depths[i];
		config.rxpools[i].buffer_size = netcp->rx_buffer_sizes[i];
		dev_dbg(netcp->dev, "rx_pool[%d] depth %d, size %d\n", i,
				config.rxpools[i].pool_depth,
				config.rxpools[i].buffer_size);
	}
	config.rxpool_count = i;

	err = dma_keystone_config(netcp->rx_channel, &config);
	if (err) {
		dev_err(netcp->dev, "%d error configuring RX channel\n",
				err);
		goto fail;
	}
	dev_dbg(netcp->dev, "opened RX channel: %p\n", netcp->rx_channel);

	for_each_module(netcp, intf_modpriv) {
		module = intf_modpriv->netcp_module;
		if (module->open != NULL) {
			err = module->open(intf_modpriv->module_priv, ndev);
			if (err != 0) {
				dev_err(netcp->dev, "Open failed in %s\n",
						module->name);
				goto fail;
			}
		}
	}

	/*
	 * Since queues open with dma enabled, this order is critical:
	 * 1) The RX state must be set before the notifier runs or
	 *    we'll get a BUG assertion in netcp_rx_notify().
	 * 2) NAPI must be enabled before the notifier runs or the
	 *    NAPI schedule request will be lost.
	 * 3) The notifier must be registered before packets arrive
	 *    or they'll be completed in hard IRQ context (which is bad).
	 * 4) Once RX buffers are available packets may arrive immediately,
	 *    so fill the free queues LAST.
	 */
	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);
	napi_enable(&netcp->napi);
	dma_set_notify(netcp->rx_channel, netcp_rx_notify, netcp);
	dma_rxfree_refill(netcp->rx_channel);

	netif_tx_wake_all_queues(ndev);

	dev_info(netcp->dev, "netcp device %s opened\n", ndev->name);

	return 0;
fail:
	if (netcp->rx_channel) {
		dma_release_channel(netcp->rx_channel);
		netcp->rx_channel = NULL;
	}
	return err;
}

/* Close the device */
static int netcp_ndo_stop(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_intf_modpriv *intf_modpriv;
	struct netcp_module *module;
	int err = 0;

	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	napi_disable(&netcp->napi);
	dmaengine_pause(netcp->rx_channel);
	dma_rxfree_flush(netcp->rx_channel);

	netcp_set_rx_state(netcp, RX_STATE_TEARDOWN);
	dma_poll(netcp->rx_channel, -1);
	dma_release_channel(netcp->rx_channel);
	netcp->rx_channel = NULL;

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	netcp_addr_clear_mark(netcp);
	netcp_addr_sweep_del(netcp);

	for_each_module(netcp, intf_modpriv) {
		module = intf_modpriv->netcp_module;
		if (module->close != NULL) {
			err = module->close(intf_modpriv->module_priv, ndev);
			if (err != 0)
				dev_err(netcp->dev,
					"Close failed in module %s (%d)\n",
					module->name, err);
		}
	}

	dev_dbg(netcp->dev, "netcp device %s stopped\n", ndev->name);

	return 0;
}

static int netcp_ndo_ioctl(struct net_device *ndev,
			   struct ifreq *req, int cmd)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_intf_modpriv *intf_modpriv;
	struct netcp_module *module;
	int ret = -1, err = -EOPNOTSUPP;

	if (!netif_running(ndev))
		return -EINVAL;

	for_each_module(netcp, intf_modpriv) {
		module = intf_modpriv->netcp_module;
		if (!module->ioctl)
			continue;

		err = module->ioctl(intf_modpriv->module_priv, req, cmd);
		if ((err < 0) && (err != -EOPNOTSUPP))
			return err;
		if (err == 0)
			ret = err;
	}

	return (ret == 0) ? 0 : err;
}

void netcp_get_stats(struct net_device *ndev, struct rtnl_link_stats64 *stats)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_stats *p = &netcp->stats;

	u64 rxpackets, rxbytes, txpackets, txbytes;
	unsigned int start;
	do {
		start = u64_stats_fetch_begin_irq(&p->syncp_rx);
		rxpackets	= p->rx_packets;
		rxbytes		= p->rx_bytes;
	} while (u64_stats_fetch_retry_irq(&p->syncp_rx, start));

	do {
		start = u64_stats_fetch_begin_irq(&p->syncp_tx);
		txpackets	= p->tx_packets;
		txbytes		= p->tx_bytes;
	} while (u64_stats_fetch_retry_irq(&p->syncp_tx, start));

	stats->rx_packets = rxpackets;
	stats->rx_bytes = rxbytes;
	stats->tx_packets = txpackets;
	stats->tx_bytes = txbytes;

	/* The following are stored as 32 bit */
	stats->rx_errors = p->rx_errors;
	stats->rx_dropped = p->rx_dropped;
	stats->tx_dropped = p->tx_dropped;
}

static int netcp_ndo_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct netcp_priv *netcp = netdev_priv(ndev);

	/* MTU < 68 is an error for IPv4 traffic */
	if ((new_mtu < 68) ||
	    (new_mtu > (NETCP_MAX_FRAME_SIZE - ETH_HLEN - ETH_FCS_LEN))) {
		dev_err(netcp->dev, "Invalid mtu size = %d\n", new_mtu);
		return -EINVAL;
	}

	ndev->mtu = new_mtu;
	return 0;
}

static int netcp_ndo_set_mac_address(struct net_device *ndev, void *p)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_addr *naddr;
	int ret;

	ret = eth_mac_addr(ndev, p);
	if (ret) {
		dev_info(netcp->dev, "set mac addr %pM failed (%d).\n",
			((struct sockaddr *)p)->sa_data, ret);
		goto done;
	}

	naddr = netcp_addr_add_mark(netcp, ndev->dev_addr, ADDR_DEV);
	if (!naddr) {
		ret = -ENOMEM;
		goto done;
	}
	if (naddr->flags & ADDR_VALID)
		goto done;

	ret = netcp_mod_add_addr(netcp, naddr);
	if (ret) {
		/* lower modules are not ready yet */
		netcp_addr_del(naddr);
		ret = 0;
	}

done:
	return ret;
}

static void netcp_ndo_tx_timeout(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);

	dev_err(netcp->dev, "transmit timed out\n");
	/* FIXME: Need to unstall the DMAs */
	netif_tx_wake_all_queues(ndev);
}

static int netcp_rx_add_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_intf_modpriv *intf_modpriv;
	struct netcp_module *module;
	unsigned long flags;
	int err = 0;

	dev_dbg(netcp->dev, "adding rx vlan id: %d\n", vid);

	spin_lock_irqsave(&netcp->lock, flags);

	for_each_module(netcp, intf_modpriv) {
		module = intf_modpriv->netcp_module;
		if ((module->add_vid != NULL) && (vid != 0)) {
			err = module->add_vid(intf_modpriv->module_priv, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not add "
					"vlan id = %d\n", vid);
				return -ENODEV;
			}
		}
	}

	spin_unlock_irqrestore(&netcp->lock, flags);

	return 0;
}

static int netcp_rx_kill_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_intf_modpriv *intf_modpriv;
	struct netcp_module *module;
	int err = 0;
	unsigned long flags;

	dev_dbg(netcp->dev, "removing rx vlan id: %d\n", vid);

	spin_lock_irqsave(&netcp->lock, flags);

	for_each_module(netcp, intf_modpriv) {
		module = intf_modpriv->netcp_module;
		if (module->del_vid != NULL) {
			err = module->del_vid(intf_modpriv->module_priv, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not delete "
					"vlan id = %d\n", vid);
				return -ENODEV;
			}
		}
	}

	spin_unlock_irqrestore(&netcp->lock, flags);

	return 0;
}

static int netcp_setup_tc(struct net_device *dev, enum tc_setup_type type,
			  void *type_data)
{
	struct tc_mqprio_qopt *mqprio = type_data;
	u8 num_tc;
	int i;

	/* setup tc must be called under rtnl lock */
	ASSERT_RTNL();

	if (type != TC_SETUP_QDISC_MQPRIO)
		return -EOPNOTSUPP;

	mqprio->hw = TC_MQPRIO_HW_OFFLOAD_TCS;
	num_tc = mqprio->num_tc;

	/* Sanity-check the number of traffic classes requested */
	if ((dev->real_num_tx_queues <= 1) ||
	    (dev->real_num_tx_queues < num_tc))
		return -EINVAL;

	/* Configure traffic class to queue mappings */
	if (num_tc) {
		netdev_set_num_tc(dev, num_tc);
		for (i = 0; i < num_tc; i++)
			netdev_set_tc_queue(dev, i, 1, i);
	} else {
		netdev_reset_tc(dev);
	}

	return 0;
}

u32 netcp_get_streaming_switch(struct netcp_device *netcp_device, int port)
{
	u32	reg;

	reg = __raw_readl(netcp_device->streaming_switch);
	return (port == 0) ? reg : (reg >> ((port - 1) * 8)) & 0xff;
}
EXPORT_SYMBOL(netcp_get_streaming_switch);

u32 netcp_get_streaming_switch2(struct netcp_device *netcp_device, int port)
{
	u32 reg, offset = 0;
	void __iomem *thread_map = netcp_device->streaming_switch;

	if (port > 0)
		/* each port has 8 priorities, which needs 8 bytes setting */
		offset = (port - 1) * 8;

	reg = readl(thread_map + offset) & 0xff;
	return reg;
}
EXPORT_SYMBOL(netcp_get_streaming_switch2);

u32 netcp_set_streaming_switch(struct netcp_device *netcp_device,
				int port, u32 new_value)
{
	u32	reg;
	u32	old_value;

	reg = __raw_readl(netcp_device->streaming_switch);

	if (port == 0) {
		old_value = reg;
		reg = (new_value << 24) | (new_value << 16) |
			(new_value << 8) | new_value;
	} else {
		int shift = (port - 1) * 8;
		old_value = (reg >> shift) & 0xff;
		reg &= ~(0xff << shift);
		reg |= (new_value & 0xff) << shift;
	}

	__raw_writel(reg, netcp_device->streaming_switch);
	return old_value;
}
EXPORT_SYMBOL(netcp_set_streaming_switch);

u32 netcp_set_streaming_switch2(struct netcp_device *netcp_device,
				   int port, u32 new_value)
{
	u32 reg, offset;
	u32 old_value;
	int i;
	void __iomem *thread_map = netcp_device->streaming_switch;

	reg = (new_value << 24) | (new_value << 16) |
	      (new_value << 8) | (new_value);
	if (port == 0) {
		/* return 1st port priority 0 setting for all the ports */
		old_value = readl(thread_map);
		old_value &= 0xff;
		for (i = 0; i < 16; i++, thread_map += 4)
			writel(reg, thread_map);
	} else {
		/* each port has 8 priorities, which needs 8 bytes setting */
		offset = (port - 1) * 8;

		/* return priority 0 setting for the port */
		old_value = readl(thread_map + offset);
		old_value &= 0xff;
		writel(reg, thread_map + offset);
		writel(reg, thread_map + offset + 4);
	}

	return old_value;
}
EXPORT_SYMBOL(netcp_set_streaming_switch2);

static const struct net_device_ops netcp_netdev_ops = {
	.ndo_open		= netcp_ndo_open,
	.ndo_stop		= netcp_ndo_stop,
	.ndo_start_xmit		= netcp_ndo_start_xmit,
	.ndo_set_rx_mode	= netcp_set_rx_mode,
	.ndo_do_ioctl           = netcp_ndo_ioctl,
	.ndo_get_stats64	= netcp_get_stats,
	.ndo_change_mtu		= netcp_ndo_change_mtu,
	.ndo_set_mac_address	= netcp_ndo_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_vlan_rx_add_vid	= netcp_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= netcp_rx_kill_vid,
	.ndo_tx_timeout		= netcp_ndo_tx_timeout,
	.ndo_select_queue	= dev_pick_tx_zero,
	.ndo_setup_tc		= netcp_setup_tc,
};

int netcp_create_interface(struct netcp_device *netcp_device,
			   struct net_device **ndev_p,
			   const char *ifname_proto,
			   int tx_queues, int rx_queues, int cpsw_port)
{
	struct platform_device *pdev = netcp_device->platform_device;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *node_ifgroup;
	struct device_node *node_interface;
	struct netcp_inst_modpriv *inst_modpriv;
	struct netcp_priv *netcp;
	struct net_device *ndev;
	resource_size_t size;
	struct resource res;
	void __iomem *efuse = NULL;
	u32 efuse_mac = 0;
	const void *mac_addr;
	u8 efuse_mac_addr[6];
	int ret = 0;

	ndev = alloc_netdev_mqs(sizeof(struct netcp_priv),
			(ifname_proto ? ifname_proto : "eth%d"),
			NET_NAME_UNKNOWN,
			ether_setup, tx_queues, rx_queues);
	*ndev_p = ndev;
	if (!ndev) {
		dev_err(&pdev->dev, "Error allocating net_device\n");
		ret = -ENOMEM;
		goto probe_quit;
	}
	dev_dbg(&pdev->dev, "%s(): ndev = %p\n", __func__, ndev);
	ndev->features |= NETIF_F_SG;
	ndev->features |= NETIF_F_FRAGLIST;

	ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	ndev->hw_features = ndev->features;

	ndev->vlan_features |= NETIF_F_TSO |
				NETIF_F_TSO6 |
				NETIF_F_IP_CSUM |
				NETIF_F_IPV6_CSUM |
				NETIF_F_SG|
				NETIF_F_FRAGLIST;

	ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	/* MTU range: 68 - 9486 */
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = NETCP_MAX_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);

	netcp = netdev_priv(ndev);
	spin_lock_init(&netcp->lock);
	INIT_LIST_HEAD(&netcp->module_head);
	INIT_LIST_HEAD(&netcp->addr_list);
	netcp->netcp_device = netcp_device;
	netcp->pdev = netcp_device->platform_device;
	netcp->ndev = ndev;
	netcp->dev  = &ndev->dev;
	netcp->cpsw_port = cpsw_port;
	netcp->msg_enable = netif_msg_init(netcp_debug_level, NETCP_DEBUG);
	netcp->rx_packet_max = netcp_rx_packet_max;
	netcp_set_rx_state(netcp, RX_STATE_INVALID);
	u64_stats_init(&netcp->stats.syncp_rx);
	u64_stats_init(&netcp->stats.syncp_tx);

	netcp->cpswx_priv = NULL;

	node_ifgroup = of_get_child_by_name(node, "interfaces");
	if (!node_ifgroup) {
		dev_info(&pdev->dev, "could not find group \"interfaces\", "
				     "defaulting to parent\n");
		node_interface = node;
	} else {
		char node_name[24];
		snprintf(node_name, sizeof(node_name), "interface-%d",
			 (cpsw_port == 0) ? 0 : (cpsw_port - 1));
		node_interface = of_get_child_by_name(node_ifgroup, node_name);
		if (!node_interface) {
			dev_err(&pdev->dev, "could not find %s\n", node_name);
			of_node_put(node_ifgroup);
			ret = -ENODEV;
			goto probe_quit;
		}
		dev_dbg(&pdev->dev, "Using node \"%s\"\n", node_name);
	}

	ret = of_property_read_u32(node_interface, "efuse-mac", &efuse_mac);
	if (efuse_mac) {
		if (of_address_to_resource(node, 1, &res)) {
			dev_err(&pdev->dev, "could not find resource\n");
			ret = -ENODEV;
			goto probe_quit;
		}
		size = resource_size(&res);

		if (!devm_request_mem_region(&pdev->dev, res.start, size,
					     dev_name(&pdev->dev))) {
			dev_err(&pdev->dev, "could not reserve resource\n");
			ret = -ENOMEM;
			goto probe_quit;
		}

		efuse = devm_ioremap_nocache(&pdev->dev, res.start, size);
		if (!efuse) {
			dev_err(&pdev->dev, "could not map resource\n");
			devm_release_mem_region(&pdev->dev, res.start, size);
			ret = -ENOMEM;
			goto probe_quit;
		}

		emac_arch_get_mac_addr(efuse_mac_addr, efuse);
		if (is_valid_ether_addr(efuse_mac_addr))
			memcpy(ndev->dev_addr, efuse_mac_addr, ETH_ALEN);
		else
			random_ether_addr(ndev->dev_addr);

		devm_iounmap(&pdev->dev, efuse);
		devm_release_mem_region(&pdev->dev, res.start, size);
	} else {
		mac_addr = of_get_mac_address(node_interface);
		if (mac_addr)
			memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
		else
			random_ether_addr(ndev->dev_addr);
	}

	ret = of_property_read_string(node_interface, "rx-channel",
				      &netcp->rx_chan_name);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing \"rx-channel\" parameter\n");
		netcp->rx_chan_name = "netrx";
	}

	ret = of_property_read_u32_array(node_interface, "rx-queue-depth",
			netcp->rx_queue_depths, KEYSTONE_QUEUES_PER_CHAN);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing \"rx-queue-depth\" parameter\n");
		netcp->rx_queue_depths[0] = 128;
	}

	ret = of_property_read_u32_array(node_interface, "rx-buffer-size",
			netcp->rx_buffer_sizes, KEYSTONE_QUEUES_PER_CHAN);
	if (ret) {
		dev_err(&pdev->dev, "missing \"rx-buffer-size\" parameter\n");
		netcp->rx_buffer_sizes[0] = 1500;
	}

	if (node_ifgroup) {
		of_node_put(node_interface);
		of_node_put(node_ifgroup);
		node_interface = NULL;
		node_ifgroup = NULL;
	}

	/* NAPI register */
	netif_napi_add(ndev, &netcp->napi, netcp_poll, NETCP_NAPI_WEIGHT_RX);

	/* Register the network device */
	ndev->dev_id		= 0;
	ndev->watchdog_timeo	= NETCP_TX_TIMEOUT;
	ndev->netdev_ops	= &netcp_netdev_ops;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(netcp->dev, "Error registering net device\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}
	dev_info(&pdev->dev, "Created interface \"%s\"\n", ndev->name);
	list_add_tail(&netcp->interface_list, &netcp_device->interface_head);

	/* Notify each registered module of the new interface */
	list_for_each_entry(inst_modpriv, &netcp_device->modpriv_head, inst_list) {
		struct netcp_module *module = inst_modpriv->netcp_module;
		struct netcp_intf_modpriv *intf_modpriv;

		intf_modpriv = kzalloc(sizeof(*intf_modpriv), GFP_KERNEL);
		if (!intf_modpriv) {
			dev_err(&pdev->dev, "Error allocating intf_modpriv for %s\n",
					module->name);
			continue;
		}

		intf_modpriv->netcp_priv = netcp;
		intf_modpriv->netcp_module = module;
		list_add_tail(&intf_modpriv->intf_list, &netcp->module_head);

		dev_dbg(&pdev->dev, "Attaching module \"%s\"\n", module->name);
		ret = module->attach(inst_modpriv->module_priv, ndev, &intf_modpriv->module_priv);
		if (ret) {
			dev_info(&pdev->dev, "Attach of module %s declined with %d\n",
					module->name, ret);
			list_del(&intf_modpriv->intf_list);
			kfree(intf_modpriv);
			continue;
		}
	}

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
probe_quit:
	*ndev_p = NULL;
	return ret;
}
EXPORT_SYMBOL(netcp_create_interface);

int netcp_cpsw_port(const struct net_device *ndev)
{
	const struct netcp_priv *netcp = netdev_priv(ndev);
	return netcp->cpsw_port;
}
EXPORT_SYMBOL(netcp_cpsw_port);

void netcp_delete_interface(struct netcp_device *netcp_device,
			    struct net_device *ndev)
{
	struct netcp_intf_modpriv *intf_modpriv, *tmp;
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module *module;

	dev_info(&netcp_device->platform_device->dev,
			"Removing interface \"%s\"\n", ndev->name);

	/* Notify each of the modules that the interface is going away */
	list_for_each_entry_safe(intf_modpriv, tmp, &netcp->module_head, intf_list) {
		module = intf_modpriv->netcp_module;
		dev_dbg(&netcp_device->platform_device->dev,
				"Releasing module \"%s\"\n", module->name);
		if (module->release)
			module->release(intf_modpriv->module_priv);
		list_del(&intf_modpriv->intf_list);
		kfree(intf_modpriv);
	}
	WARN(!list_empty(&netcp->module_head),
			"%s interface module list is not empty!\n", ndev->name);

	list_del(&netcp->interface_list);
	netif_napi_del(&netcp->napi);

	unregister_netdev(ndev);
	free_netdev(ndev);
}
EXPORT_SYMBOL(netcp_delete_interface);


static int netcp_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	struct netcp_device *netcp_device;
	struct netcp_module *module;
	struct netcp_inst_modpriv *inst_modpriv;
	const char *name;
	u32 temp[4];
	int ret;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	/* Allocate a new NETCP device instance */
	netcp_device = kzalloc(sizeof(*netcp_device), GFP_KERNEL);
	if (!netcp_device) {
		dev_err(&pdev->dev, "failure allocating NETCP device\n");
		return -ENOMEM;
	}
	dev_dbg(&pdev->dev, "%s(): netcp_device = %p\n", __func__, netcp_device);

	/* Initialize the NETCP device instance */
	INIT_LIST_HEAD(&netcp_device->interface_head);
	INIT_LIST_HEAD(&netcp_device->modpriv_head);
	netcp_device->platform_device = pdev;
	platform_set_drvdata(pdev, netcp_device);

	/* Map the Streaming Switch */
	if (!of_property_read_u32_array(node, "streaming-regs",
					(u32 *)&(temp[0]), 2)) {
		netcp_device->streaming_switch =
			devm_ioremap_nocache(&pdev->dev, temp[0], temp[1]);
		if (!netcp_device->streaming_switch) {
			dev_err(&pdev->dev, "can't map streaming switch\n");
			ret = -ENOMEM;
			goto probe_quit;
		}
	}

	/* Add the device instance to the list */
	list_add_tail(&netcp_device->device_list, &netcp_devices);

	/* Probe any modules already registered */
	for_each_child_of_node(node, child) {
		name = netcp_node_name(child);
		module = netcp_find_module(name);
		if (!module)
			continue;

		inst_modpriv = kzalloc(sizeof(*inst_modpriv), GFP_KERNEL);
		if (!inst_modpriv) {
			dev_err(&pdev->dev, "Failed to allocate instance for for %s\n", name);
			continue;	/* FIXME: Fail instead? */
		}
		inst_modpriv->netcp_device = netcp_device;
		inst_modpriv->netcp_module = module;
		list_add_tail(&inst_modpriv->inst_list, &netcp_device->modpriv_head);

		dev_dbg(&pdev->dev, "%s(): probing module \"%s\"\n", __func__, name);
		ret = module->probe(netcp_device, &pdev->dev, child, &inst_modpriv->module_priv);
		if (ret) {
			dev_warn(&pdev->dev, "Probe of module %s failed with %d\n", name, ret);
			list_del(&inst_modpriv->inst_list);
			kfree(inst_modpriv);
			continue;
		}
	}

	return 0;

probe_quit:
	platform_set_drvdata(pdev, NULL);
	kfree(netcp_device);
	return ret;
}

static int netcp_remove(struct platform_device *pdev)
{
	struct netcp_device *netcp_device = platform_get_drvdata(pdev);
	struct netcp_inst_modpriv *inst_modpriv, *tmp;
	struct netcp_module *module;

	list_for_each_entry_safe(inst_modpriv, tmp, &netcp_device->modpriv_head, inst_list) {
		module = inst_modpriv->netcp_module;
		dev_dbg(&pdev->dev, "Removing module \"%s\"\n", module->name);
		module->remove(netcp_device, inst_modpriv->module_priv);
		list_del(&inst_modpriv->inst_list);
		kfree(inst_modpriv);
	}
	WARN(!list_empty(&netcp_device->interface_head),
			"%s interface list not empty!\n", pdev->name);

	/* Unmap the Streaming Switch */
	if (netcp_device->streaming_switch)
		devm_iounmap(&pdev->dev, netcp_device->streaming_switch);

	platform_set_drvdata(pdev, NULL);
	kfree(netcp_device);
	return 0;
}

static struct of_device_id of_match[] = {
	{ .compatible = "ti,keystone-netcp", },
	{},
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver netcp_driver = {
	.driver = {
		.name		= "keystone-netcp",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe = netcp_probe,
	.remove = netcp_remove,
};

extern int  keystone_cpsw_init(void);
extern void keystone_cpsw_exit(void);
extern int  keystone_cpsw2_init(void);
extern void keystone_cpsw2_exit(void);
#ifdef CONFIG_TI_KEYSTONE_XGE
extern int  keystone_cpswx_init(void);
extern void keystone_cpswx_exit(void);
#endif

static int __init netcp_init(void)
{
	int err;

	/* Create a cache for these commonly-used structures */
	netcp_pinfo_cache = kmem_cache_create("netcp_pinfo_cache",
			sizeof(struct netcp_packet), sizeof(void *),
			SLAB_HWCACHE_ALIGN, NULL);
	if (!netcp_pinfo_cache)
		return -ENOMEM;

	err = platform_driver_register(&netcp_driver);
	if (err)
		goto netcp_fail;

	err = keystone_cpsw_init();
	if (err)
		goto cpsw_fail;

	err = keystone_cpsw2_init();
	if (err)
		goto cpsw_fail;

#ifdef CONFIG_TI_KEYSTONE_XGE
	err = keystone_cpswx_init();
	if (err)
		goto cpsw_fail;
#endif

	return 0;

cpsw_fail:
	platform_driver_unregister(&netcp_driver);
netcp_fail:
	kmem_cache_destroy(netcp_pinfo_cache);
	netcp_pinfo_cache = NULL;
	return err;
}
module_init(netcp_init);

static void __exit netcp_exit(void)
{
	keystone_cpsw_exit();
	keystone_cpsw2_exit();
#ifdef CONFIG_TI_KEYSTONE_XGE
	keystone_cpswx_exit();
#endif
	platform_driver_unregister(&netcp_driver);
	kmem_cache_destroy(netcp_pinfo_cache);
	netcp_pinfo_cache = NULL;
}
module_exit(netcp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Ethernet driver");
