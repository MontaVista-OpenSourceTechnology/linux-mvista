/*
 * Virtual netdevice to workaround broken design of AXM CPUs and provide
 * ordered delivery of messages to upper layers.
 *
 * Author: Petr Malat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) "rioproxy: "fmt

#include <linux/module.h>
#include <linux/rtnetlink.h>
#include <linux/netdevice.h>
#include <linux/if_syscom_ether.h>
#include <uapi/linux/if_rio.h>
#include <linux/if_arp.h>
#include <linux/hash.h>
#include <net/syscom.h>

// TODO: This should come from kernel headers or the kernel should
// export a function to set/get rio_id/rio_letter from skb.
#define rio_id hash
#define rio_letter csum_level
#define rio_id_max 0xffff

struct rioproxy_table {
	int dev_idx, letter;
};

/** Private structure of rioproxy netdevice */
struct rioproxy_dev {
	const struct rioproxy_table *reliable, *unreliable, *unreliable_nofrag;
	int reliable_nmemb, unreliable_nmemb, unreliable_nofrag_nmemb, dev_nmemb;
	struct header_ops header_ops;
	struct net_device *dev[];
};

/** Bitmask of destinations with enabled workaround */
static unsigned long rioproxy_wa_enabled[BITS_TO_LONGS(rio_id_max + 1)];

// Inform others we are running with broken RIO implementation
#ifdef CONFIG_RIOPROXY_ON_BROKEN_CPU
#define rioproxy_tag_skb(skb) (void)(((struct syscom_frag_hdr *) \
		(skb)->data)->reserved = SYSCOM_FRAG_HDR_FLAG_AXM_WA)
#else
#define rioproxy_tag_skb(skb) (void)0
#endif

/** Send SKB and apply the workaround if needed */
static int rioproxy_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rioproxy_dev *rioproxy = netdev_priv(dev);

	if (likely(skb->protocol == htons(ETH_P_SYSCOM_FRAG) ||
			skb->protocol == htons(ETH_P_SYSCOM))) {
		struct net_device *tx_dev;
		struct sk_buff *skb_iter;
		int letter;

		BUG_ON(skb->rio_id > rio_id_max);

		if (test_bit(skb->rio_id, rioproxy_wa_enabled)) {
			const struct rioproxy_table *table;
			struct syscom_hdr *hdr;
			uint64_t stream;
			int idx, nmemb;

			if (skb->protocol == htons(ETH_P_SYSCOM_FRAG)) {
				struct syscom_frag_hdr *frag_hdr =
						(struct syscom_frag_hdr *)skb->data;
				// We do GSO, thus we must always receive the
				// first fragment with list attached to it
				BUG_ON(frag_hdr->frag);
				hdr = (struct syscom_hdr *)&frag_hdr[1];
			} else {
				hdr = (struct syscom_hdr *)skb->data;
			}

			if (hdr->ordered) {
				if (unlikely(skb->protocol == htons(ETH_P_SYSCOM))) {
					pr_err("Reliable non-fragmented syscom "
							"is not supported\n");
					return -EPROTONOSUPPORT;
				}
				table = rioproxy->reliable;
				nmemb = rioproxy->reliable_nmemb;
			} else if (skb->protocol == htons(ETH_P_SYSCOM_FRAG)) {
				table = rioproxy->unreliable;
				nmemb = rioproxy->unreliable_nmemb;
			} else {
				table = rioproxy->unreliable_nofrag;
				nmemb = rioproxy->unreliable_nofrag_nmemb;
			}

			memcpy(&stream, &hdr->destination, sizeof stream);
			idx = (int)hash_64(stream, 8) % nmemb;
			tx_dev = rioproxy->dev[table[idx].dev_idx];
			letter = table[idx].letter;
		} else {
			tx_dev = rioproxy->dev[0];
			letter = skb->rio_letter;
		}

		rioproxy_tag_skb(skb);
		skb->dev = tx_dev;
		skb->rio_letter = letter;
		for (skb_iter = skb_shinfo(skb)->frag_list; skb_iter;
				skb_iter = skb_iter->next) {
			rioproxy_tag_skb(skb_iter);
			skb_iter->dev = tx_dev;
			skb_iter->rio_letter = letter;
		}
	} else {
		skb->dev = rioproxy->dev[0];
	}

	return dev_queue_xmit(skb);
}

/** Receive hook checking if the workaround has to be enabled */
static rx_handler_result_t rioproxy_handle_frame(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct syscom_frag_hdr *frag_hdr = (struct syscom_frag_hdr *)skb->data;

	if (likely(skb->protocol == htons(ETH_P_SYSCOM_FRAG))) {
		BUG_ON(skb->rio_id > rio_id_max);

		if (frag_hdr->reserved & SYSCOM_FRAG_HDR_FLAG_AXM_WA) {
			if (unlikely(!test_bit(skb->rio_id, rioproxy_wa_enabled))) {
				pr_info("Enabling AXM workaround towards %04x\n",
						skb->rio_id);
				set_bit(skb->rio_id, rioproxy_wa_enabled);
			}
		} else {
			if (unlikely(test_bit(skb->rio_id, rioproxy_wa_enabled))) {
				pr_info("Disabling AXM workaround towards %04x\n",
						skb->rio_id);
				clear_bit(skb->rio_id, rioproxy_wa_enabled);
			}
		}
	}

#ifdef CONFIG_RIOPROXY_ON_BROKEN_CPU
	if (unlikely(skb->protocol == htons(ETH_P_SYSCOM))) {
		union {
			struct riohdr rio;
			char buf[MAX_ADDR_LEN];
		} hdr;
		int rtn;

		rtn = dev_parse_header(skb, hdr.buf);
		WARN_ONCE(sizeof hdr.rio != rtn, "Unsupported header format");
		if (riohdr_get_mbox(&hdr.rio) != 1 ||
				riohdr_get_letter(&hdr.rio) != 1)
			skb->protocol = htons(ETH_P_SYSCOM_FRAG);
	}
#endif

	return RX_HANDLER_PASS;
}

/** Netdevice operations */
static const struct net_device_ops rioproxy_netdev_ops = {
	.ndo_start_xmit	= rioproxy_ndo_start_xmit,
};

/** Setup callback of the rio device */
static void rioproxy_dev_setup(struct net_device *dev)
{
	struct rioproxy_dev *rioproxy = netdev_priv(dev);

	dev->priv_flags = IFF_NO_QUEUE;
	dev->header_ops = &rioproxy->header_ops;
	dev->netdev_ops = &rioproxy_netdev_ops;
	dev->type = ARPHRD_VOID;
	dev->features = NETIF_F_GSO_SYSCOM | NETIF_F_FRAGLIST | NETIF_F_HW_CSUM;
}

/** Call create header operation of the underlying device */
static int rioproxy_create_header(struct sk_buff *skb, struct net_device *dev,
                unsigned short type, const void *daddr, const void *saddr,
                unsigned int len)
{
	struct rioproxy_dev *rioproxy = netdev_priv(dev);

	return dev_hard_header(skb, rioproxy->dev[0], type, daddr, saddr, len);
}

/** Create rioproxy netdevice */
static struct net_device *rioproxy_instantiate(
		const char *devices[], int devices_nmemb,
		const struct rioproxy_table *table_reliable,
		int table_reliable_nmemb,
		const struct rioproxy_table *table_unreliable,
		int table_unreliable_nmemb,
		const struct rioproxy_table *table_unreliable_nofrag,
		int table_unreliable_nofrag_nmemb)
{
	struct rioproxy_dev *rioproxy;
	struct net_device *dev;
	int i, rtn;

	if (devices_nmemb < 1 || table_reliable_nmemb < 1 ||
	    table_unreliable_nmemb < 1 || table_unreliable_nofrag_nmemb < 1 ) {
		return ERR_PTR(-EINVAL);
	}

	dev = alloc_netdev(sizeof *rioproxy + devices_nmemb * sizeof *rioproxy->dev,
			"rioproxy%d", NET_NAME_UNKNOWN, rioproxy_dev_setup);
	if (!dev) {
		pr_err("Failed to allocate rioproxy device.\n");
		return ERR_PTR(-ENOMEM);
	}

	rioproxy = netdev_priv(dev);
	for (i = 0; i < devices_nmemb; i++) {
		rioproxy->dev[i] = dev_get_by_name(&init_net, devices[i]);
		if (!rioproxy->dev[i]) {
			pr_err("Device %s not found.\n", devices[i]);
			rtn = -ENODEV;
			goto err0;
		}
	}

	rtnl_lock();
	for (i = 0; i < devices_nmemb; i++) {
		rtn = netdev_rx_handler_register(rioproxy->dev[i],
				rioproxy_handle_frame, NULL);
		if (rtn) {
			pr_err("Can't attach RX handler to %s.\n", devices[i]);
			goto err1;
		}
	}
	rtnl_unlock();

	rioproxy->dev_nmemb = devices_nmemb;
	rioproxy->reliable = table_reliable;
	rioproxy->reliable_nmemb = table_reliable_nmemb;
	rioproxy->unreliable = table_unreliable;
	rioproxy->unreliable_nmemb = table_unreliable_nmemb;
	rioproxy->unreliable_nofrag = table_unreliable_nofrag;
	rioproxy->unreliable_nofrag_nmemb = table_unreliable_nofrag_nmemb;
	rioproxy->header_ops.parse = rioproxy->dev[0]->header_ops ?
			rioproxy->dev[0]->header_ops->parse : NULL;
	rioproxy->header_ops.create = rioproxy_create_header;

	dev->addr_len = rioproxy->dev[0]->addr_len;
	memcpy(dev->dev_addr, rioproxy->dev[0]->dev_addr, dev->addr_len);
	dev->hard_header_len = rioproxy->dev[0]->hard_header_len;
	dev->mtu = rioproxy->dev[0]->mtu;

	rtn = register_netdev(dev);
	if (rtn) {
		pr_err("Failed to register rioproxy device.\n");
		goto err2;
	}

	netif_carrier_on(dev);
	netif_start_queue(dev);

	return dev;

err2:	rtnl_lock();
err1:	while (--i >= 0) netdev_rx_handler_unregister(rioproxy->dev[i]);
	rtnl_unlock();
err0:	for (i = 0; i < devices_nmemb && rioproxy->dev[i]; i++) {
		dev_put(rioproxy->dev[i]);
	}
	free_netdev(dev);
	return ERR_PTR(rtn);
}

/** Destroy rioproxy netdevice */
static void rioproxy_destroy(struct net_device *dev)
{
	struct rioproxy_dev *rioproxy = netdev_priv(dev);
	int i;

	for (i = 0; i < rioproxy->dev_nmemb; i++) {
		rtnl_lock();
		netdev_rx_handler_unregister(rioproxy->dev[i]);
		rtnl_unlock();
		dev_put(rioproxy->dev[i]);
	}
	unregister_netdev(dev);
	free_netdev(dev);
}

/* Module initialization and de-initialization *************************/

/** Device instantiated by default */
static struct net_device *rioproxy0;

/** Module initialization */
static int __init rioproxy_init(void)
{
	static const char *devices[] = {"rio0m1", "rio0m3"};
	static const struct rioproxy_table table_reliable[] = {
			{0, 0}, {0, 2}, {1, 1}, {1, 2}, {1, 3} };
	static const struct rioproxy_table table_unreliable[] = { {0, 3} };
	static const struct rioproxy_table table_unreliable_nofrag[] = { {0, 1} };
	int rtn;

#ifdef CONFIG_RIOPROXY_ON_BROKEN_CPU
	pr_notice("Running with broken RIO implementation, "
			"asking peers to enable reordering workaround.\n");
#endif

	rioproxy0 = rioproxy_instantiate(
			devices, ARRAY_SIZE(devices),
			table_reliable, ARRAY_SIZE(table_reliable),
			table_unreliable, ARRAY_SIZE(table_unreliable),
			table_unreliable_nofrag, ARRAY_SIZE(table_unreliable_nofrag));
	if (IS_ERR(rioproxy0)) {
		return PTR_ERR(rioproxy0);
	}

	rtnl_lock();
	rtn = dev_open(rioproxy0);
	rtnl_unlock();
	if (rtn) {
		pr_err("Failed to bring the default device up: %d\n", rtn);
		rioproxy_destroy(rioproxy0);
		return rtn;
	}

	return 0;
}
module_init(rioproxy_init);

/** Module cleanup */
static void __exit rioproxy_exit(void)
{
	rioproxy_destroy(rioproxy0);
}
module_exit(rioproxy_exit);

MODULE_AUTHOR("Petr Malat");
MODULE_DESCRIPTION("Proxy RIO device to workaround AXM CPU bug");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_NETDEV("rioproxy0");
