/*
 * SYSCOM protocol stack for the Linux kernel
 * Author: Petr Malat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "common.h"

#include <linux/if_syscom_service.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>

#include "route.h"
#include "route-dev.h"

/** Netdev specific route record. */
struct syscom_route_record_dev {
	/** Parent route record */
	struct syscom_route_record super;
	/** Header prepended to the packet */
	uint8_t header[SYSCOM_ROUTE_DEV_HDR_MAX];
	/** The header length */
	uint8_t header_len;
	/** Device used for sending */
	struct net_device *dev;
};

/** Cast to the child type with a type check. */
static inline const struct syscom_route_record_dev *syscom_route_record_dev(
		const struct syscom_route_record *r)
{
	return (const struct syscom_route_record_dev *)r;
}

/** Route record destructor. */
static void syscom_route_record_dev_destroy(struct syscom_route_record *r)
{
	const struct syscom_route_record_dev *self = syscom_route_record_dev(r);

	dev_put(self->dev);
	kfree(r);
}

/** Netdevice unregister callback. If the route disappears, it returns true to
 *  indicate the record must be removed to release the device reference. */
static bool syscom_route_record_dev_netdev_unregister(
		const struct syscom_route_record *r, const struct net_device *dev)
{
	const struct syscom_route_record_dev *self = syscom_route_record_dev(r);

	return self->dev == dev;
}

#ifdef CONFIG_PROC_FS
/** Dump the record to the seq file. */
static void syscom_route_record_dev_seq_show(
		struct syscom_route_record *r, struct seq_file *seq)
{
	const struct syscom_route_record_dev *self = syscom_route_record_dev(r);
	int i;

	seq_printf(seq, "%-9s ", self->dev->name);
	for (i = 0; i < self->header_len; i++) {
		seq_printf(seq, "%02x", self->header[i]);
	}
	seq_printf(seq, "\n");

}
#endif // CONFIG_PROC_FS

/** Deliver the message over a net device. */
static int syscom_route_record_dev_deliver(const struct syscom_route_record *r,
		struct syscom_delivery *d)
{
	const struct syscom_route_record_dev *self = syscom_route_record_dev(r);
	int rtn, headroom = r->headroom + self->header_len;
	struct net_device *dev = self->dev;
	char header[self->header_len];
	struct sk_buff *skb, *i;

	dev_hold(dev);
	memcpy(header, self->header, sizeof header);
	rcu_read_unlock();

	rtn = syscom_delivery_get_skb(d, &skb, NULL, NULL, headroom, dev->mtu);
	if (unlikely(rtn)) {
		goto out;
	}

	skb->dev = dev;
	rtn = dev_hard_header(skb, dev, ntohs(skb->protocol), header,
			NULL, skb->len);
	for (i = skb_shinfo(skb)->frag_list; i && rtn >= 0; i = i->next) {
		i->dev = dev;
		rtn = dev_hard_header(i, dev, ntohs(skb->protocol), header,
				NULL, i->len);
	}
	if (rtn < 0) {
		kfree_skb(skb);
	} else {
		rtn = dev_queue_xmit(skb);
	}

out:	dev_put(dev);
	return rtn;
}

/** Route record operations for netdev routes. */
static const struct syscom_route_record_ops syscom_route_record_dev_ops = {
	.destroy = syscom_route_record_dev_destroy,
	.netdev_unregister = syscom_route_record_dev_netdev_unregister,
	.deliver = syscom_route_record_dev_deliver,
#ifdef CONFIG_PROC_FS
	.seq_show = syscom_route_record_dev_seq_show,
	.type = "DEV",
#endif // CONFIG_PROC_FS
};

static DEFINE_MUTEX(syscom_route_dev_lock);

static LIST_HEAD(down_hdr);

struct syscom_route_dev_auto_down {
	struct list_head down_hdr_list;
	uint8_t header_len;
	uint8_t header[SYSCOM_ROUTE_DEV_HDR_MAX];
	char if_name[IFNAMSIZ];
};

static struct syscom_route_dev_auto_down *syscom_route_dev_get_auto_down(
		const char *if_name, uint8_t *header, uint8_t header_len)
{
	struct syscom_route_dev_auto_down *d;

	lockdep_assert_held(&syscom_route_dev_lock);

	list_for_each_entry(d, &down_hdr, down_hdr_list) {
		if (d->header_len == header_len &&
		    !memcmp(d->header, header, header_len) &&
		    !strcmp(d->if_name, if_name)) {
			return d;
		}
	}

	return NULL;
}

/** Create a new netdev route record. */
int syscom_route_record_dev_add(__be16 dst_nid, uint8_t dst_nid_len,
		__be16 src_nid, struct net *if_net, const char *if_name,
		uint16_t frag_threshold, void *header, uint8_t header_len,
		int flags)
{
	struct syscom_route_record_dev *self;
	int err;

	if (header_len > sizeof self->header) {
		return -EINVAL;
	}

	self = kmalloc(sizeof *self, GFP_KERNEL);
	if (!self) {
		return -ENOMEM;
	}

	dev_load(if_net, if_name);
	self->dev = dev_get_by_name(if_net, if_name);
	if (!self->dev) {
		err = -ENODEV;
		goto err0;
	}
	self->header_len = header_len;
	memcpy(self->header, header, header_len);

	mutex_lock(&syscom_route_dev_lock);
	err = syscom_route_record_add(&self->super, dst_nid, dst_nid_len,
			src_nid, frag_threshold,
			LL_RESERVED_SPACE_EXTRA(self->dev, 8 + header_len),
			!!syscom_route_dev_get_auto_down(if_name, header,
			header_len),
			flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_ORDERED,
			flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_RELIABLE,
			&syscom_route_record_dev_ops);
	mutex_unlock(&syscom_route_dev_lock);
	if (!err) return 0;

	dev_put(self->dev);
err0:	kfree(self);
	return err;
}

/** Argument for up/down iterator */
struct syscom_route_dev_iter_cb_arg {
	void *header;
	uint8_t header_len;
	const char *if_name;
	bool up;
};

/** Up/down iterator */
static void syscom_route_dev_iter_cb(struct syscom_route_record *r, void *argp)
{
	struct syscom_route_dev_iter_cb_arg *arg = argp;
	struct syscom_route_record_dev *r_dev =
			(struct syscom_route_record_dev *)r;

	if (r->ops != &syscom_route_record_dev_ops) {
		return;
	}

	if (r_dev->header_len == arg->header_len &&
	    !memcmp(r_dev->header, arg->header, arg->header_len) &&
	    !strcmp(r_dev->dev->name, arg->if_name)) {
		syscom_route_up_down(r, arg->up);
	}
}

/** Put matching routes up or down */
static void syscom_route_dev_up_down(const char *if_name, void *header,
		uint8_t header_len, bool up)
{
	struct syscom_route_dev_iter_cb_arg arg = {
			.header = header, .header_len = header_len,
			.if_name = if_name, .up = up };
	syscom_route_record_iterate(syscom_route_dev_iter_cb, &arg);
}

/** Disable all routes matching specified header */
int syscom_route_dev_down(const char *if_name, void *header, uint8_t header_len,
		uint32_t flags)
{
	struct syscom_route_dev_auto_down *d;
	bool auto_down = flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_DEV_DOWN_AUTO;

	if (header_len > sizeof d->header) {
		return -EINVAL;
	}

	mutex_lock(&syscom_route_dev_lock);
	d = syscom_route_dev_get_auto_down(if_name, header, header_len);

	if (auto_down && !d) {
		d = kmalloc(sizeof *d, GFP_KERNEL);
		if (!d) {
			mutex_unlock(&syscom_route_dev_lock);
			return -ENOMEM;
		}
		memcpy(d->header, header, header_len);
		strcpy(d->if_name, if_name);
		d->header_len = header_len;
		list_add_tail(&d->down_hdr_list, &down_hdr);
	} else if (!auto_down && d) {
		list_del(&d->down_hdr_list);
		kfree(d);
	}

	syscom_route_dev_up_down(if_name, header, header_len, 0);
	mutex_unlock(&syscom_route_dev_lock);

	return 0;
}

/** Enable all routes matching specified header */
int syscom_route_dev_up(const char *if_name, void *header, uint8_t header_len)
{
	struct syscom_route_dev_auto_down *d;

	mutex_lock(&syscom_route_dev_lock);
	d = syscom_route_dev_get_auto_down(if_name, header, header_len);
	if (d) {
		list_del(&d->down_hdr_list);
		kfree(d);
	}

	syscom_route_dev_up_down(if_name, header, header_len, 1);
	mutex_unlock(&syscom_route_dev_lock);

	return 0;
}
