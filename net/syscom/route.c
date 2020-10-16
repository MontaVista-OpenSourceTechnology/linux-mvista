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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/seq_file_net.h>
#include <linux/netdevice.h>
#include <linux/if_syscom_ether.h>
#include <linux/highmem.h>

#include "route.h"
#include "raw.h"
#include "af.h"

/* Route tables ********************************************************/

/** Dummy structure to prevent dereferencing */
struct syscom_route_entry_dummy;

/** RCU protected route table entry */
typedef struct syscom_route_entry_dummy __rcu *syscom_route_entry_rcu;

/** Route table entry */
typedef struct syscom_route_entry_dummy *syscom_route_entry;

/** 256-ary tree node */
typedef syscom_route_entry_rcu syscom_route_node[256];

/** Tree for fast route lookups, RCU protected */
static syscom_route_node syscom_route_root;

/** Expands to casted line value
 * We need the cast to workaround inability of GCC to compile
 *   typeof(*incomplete_ptr) *ptr
 * which is used by RCU macros */
#define syscom_route_node_line(node, line) (*(void * __rcu *)&(*node)[line])

/** List of routes based on their mask length, RCU protected. */
static struct hlist_head route[17];

/** Lock to serialize route[] and syscom_route_root writers */
static DEFINE_SPINLOCK(route_lock);

/** Route entry types */
enum {
	/** Route entry is empty */
	EMPTY = 0,
	/** Route entry is not optimized, slow lookup must be performed */
	SLOW  = 1,
	/** Route entry points to syscom_route_record */
	ROUTE = 2,
	/** Route entry points to syscom_route_node */
	NODE = 3,
};

#define SLOW_PTR ((syscom_route_entry)SLOW)
#define EMPTY_PTR ((syscom_route_entry)EMPTY)

/** Expand to a NID mask from the mask length */
#define syscom_len2mask(x) htons(~((1 << (16 - (x))) - 1))

#define syscom_node_index(nid, level) ((level) == 0 ? (ntohs(nid) >> 8) : (ntohs(nid) & 0xff))

#define syscom_mask_iter(msk, level) (1 << (ffs(0x100 | syscom_node_index((msk), (level))) - 1))

#ifdef CONFIG_SYSCOM_DEBUG
#define SYSCOM_TYPE_BUG_ON(cond) BUG_ON(cond)
#else
#define SYSCOM_TYPE_BUG_ON(cond)
#endif

static __always_inline syscom_route_entry tlp(const syscom_route_node *t,
		int line)
{
	return rcu_dereference_protected(syscom_route_node_line(t, line),
			lockdep_is_held(&route_lock));
}

static __always_inline syscom_route_entry tl(const syscom_route_node *t, int line)
{
	return rcu_dereference_check(syscom_route_node_line(t, line),
			lockdep_is_held(&route_lock));
}

static __always_inline void tlu(syscom_route_node *t, int line,
		syscom_route_entry value)
{
	rcu_assign_pointer(syscom_route_node_line(t, line), (void*)value);
}

static __always_inline int to_TYPE(const syscom_route_entry ptr)
{
	SYSCOM_TYPE_BUG_ON(ptr != 0 && ((long)ptr & 3L) == 0);
	return (int)(long)ptr & 3L;
}

static __always_inline syscom_route_entry to_ROUTE(const struct syscom_route_record *r)
{
	SYSCOM_TYPE_BUG_ON((long)r & 3L);
	return ((void*)((long)(r) | ROUTE));
}

static __always_inline syscom_route_entry to_NODE(const syscom_route_node *t)
{
	SYSCOM_TYPE_BUG_ON((long)t & 3L);
	return ((void*)((long)(t) | NODE));
}

static __always_inline struct syscom_route_record *to_PTR_ROUTE(
		const syscom_route_entry ptr)
{
	SYSCOM_TYPE_BUG_ON(to_TYPE(ptr) != ROUTE);
	return (void*)((long)ptr & ~3L);
}

static __always_inline syscom_route_node *to_PTR_NODE(const syscom_route_entry ptr)
{
	SYSCOM_TYPE_BUG_ON(to_TYPE(ptr) != NODE);
	return (void*)((long)ptr & ~3L);
}

static void syscom_route_tree_optimize(void);

/** Remove route from the routing tree */
static void syscom_route_tree_del(const struct syscom_route_record *r)
{
	const uint8_t idx1 = syscom_node_index(r->dst_nid, 0);
	const uint8_t idx2 = syscom_node_index(r->dst_nid, 1);
	const int i_max = syscom_mask_iter(r->dst_nid_mask, 0);
	const int j_max = syscom_mask_iter(r->dst_nid_mask, 1);
	const syscom_route_entry match = to_ROUTE(r);
	int i, j;

	for (i = 0; i < i_max; i++) {
		syscom_route_entry rl = tlp(&syscom_route_root, idx1 | i);

		if (to_TYPE(rl) == NODE) {
			for (j = 0; j < j_max; j++) {
				syscom_route_entry ptr;
				ptr = tlp(to_PTR_NODE(rl), idx2 | j);
				if (ptr == match) {
					tlu(to_PTR_NODE(rl), idx2 | j, SLOW_PTR);
				}
			}
		} else if (rl == match) {
			tlu(&syscom_route_root, idx1 | i, SLOW_PTR);
		}
	}

	syscom_route_tree_optimize();
}

static int syscom_route_tree_add(syscom_route_node *node, int level,
		const struct syscom_route_record *r, bool replace_slow);

/** Either add the entry or allocate a new node and recurse */
static int syscom_route_tree_modify(syscom_route_node *node, int level,
		int index, const struct syscom_route_record *r,
		const syscom_route_entry initial_value,
		const bool replace_slow)
{
	if (level == 0 && syscom_node_index(r->dst_nid_mask, 1)) {
		int i;

		syscom_route_node *t = kmalloc(sizeof *t, GFP_ATOMIC);
		if (unlikely(!t)) {
			syscom_route_tree_del(r);
			return -ENOMEM;
		}
		atomic_inc(&syscom_stats.route_tree_nodes);

		for (i = 0; i < ARRAY_SIZE(*t); i++) {
			(*t)[i] = initial_value;
		}

		syscom_route_tree_add(t, level + 1, r, replace_slow);
		tlu(node, index, to_NODE(t));
	} else {
		tlu(node, index, to_ROUTE(r));
	}

	return 0;
}

/** Add entry to the route tree */
static int syscom_route_tree_add(syscom_route_node *node, int level,
		const struct syscom_route_record *r, bool replace_slow)
{
	const uint8_t idx = syscom_node_index(r->dst_nid, level);
	const int i_max = syscom_mask_iter(r->dst_nid_mask, level);
	int i;

	for (i = 0; i < i_max; i++) {
		const struct syscom_route_record *r2;
		const int iter = idx | i;
		syscom_route_entry e;
		int rtn;

		e = tlp(node, iter);
		switch (to_TYPE(e)) {
			case ROUTE:
				r2 = to_PTR_ROUTE(e);
				if ((r2->dst_nid_mask | r->dst_nid_mask) ==
						r2->dst_nid_mask) {
					break;
				}
				rtn = syscom_route_tree_modify(node, level,
						iter, r, e, replace_slow);
				if (rtn) {
					return rtn;
				}
				break;
			case EMPTY:
				rtn = syscom_route_tree_modify(node, level,
						iter, r, EMPTY_PTR, true);
				if (rtn) {
					return rtn;
				}
				break;
			case NODE:
				syscom_route_tree_add(to_PTR_NODE(e), level + 1,
						r, replace_slow);
				break;
			case SLOW:
				if (!replace_slow) break;
				rtn = syscom_route_tree_modify(node, level,
						iter, r, SLOW_PTR, true);
				if (rtn) {
					return rtn;
				}
				break;
		}
	}

	return 0;
}

/** Recursively compact the routing node */
static int syscom_route_tree_compact(syscom_route_node *t)
{
	const syscom_route_entry first = tlp(t, 0);
	int diff = 0, i;

	for (i = 0; i < ARRAY_SIZE(*t); i++) {
		const syscom_route_entry ptr = tlp(t, i);

		switch (to_TYPE(ptr)) {
			case SLOW:
				tlu(t, i, EMPTY_PTR);
				if (first != EMPTY_PTR) diff = 1;
				break;
			case NODE:
				if (!syscom_route_tree_compact(to_PTR_NODE(ptr))) {
					const syscom_route_entry ptr2 = tlp(to_PTR_NODE(ptr), 0);
					tlu(t, i, ptr2);
					kfree(to_PTR_NODE(ptr));
					atomic_dec(&syscom_stats.route_tree_nodes);
					if (first != ptr2) diff = 1;
				} else {
					diff = 1;
				}
				break;
			default:
				if (first != ptr) diff = 1;
				break;
		}
	}

	return diff;
}

/** Optimize the routing tree by removing slow lookups and compacting nodes */
static void syscom_route_tree_optimize_worker(struct work_struct *work)
{
	bool repeat = 0;
	int i;

	atomic_inc(&syscom_stats.route_tree_optimizations);

	spin_lock(&route_lock);
	for (i = ARRAY_SIZE(route) - 1; i >= 0; i--) {
		struct syscom_route_record *r;
		int err;
		hlist_for_each_entry_rcu(r, &route[i], hlist) {
			if (0 != (err = syscom_route_tree_add(&syscom_route_root, 0, r, true))) {
				pr_err("Failed optimizing routing tree: %d, will retry\n", err);
				repeat = 1;
			}
		}
	}
	syscom_route_tree_compact(&syscom_route_root);
	spin_unlock(&route_lock);

	if (repeat) {
		syscom_route_tree_optimize();
	} else {
		pr_debug("Routing tree optimized\n");
	}
}

static DECLARE_DEFERRABLE_WORK(syscom_route_tree_optimize_work,
		syscom_route_tree_optimize_worker);

/** Schedule optimization of the routing table */
static void syscom_route_tree_optimize(void)
{
	schedule_delayed_work(&syscom_route_tree_optimize_work, 2 * HZ);
}

/** Lookup the route, must be called within RCU read lock or route_lock */
static struct syscom_route_record *syscom_route_lookup(__be16 nid)
{
	int i;

	syscom_route_entry rl = tl(&syscom_route_root, syscom_node_index(nid, 0));
again:	switch (to_TYPE(rl)) {
		case EMPTY: return NULL;
		case SLOW: atomic_inc(&syscom_stats.route_tree_miss);
			for (i = ARRAY_SIZE(route) - 1; i >= 0; i--) {
				struct syscom_route_record *r;
				hlist_for_each_entry_rcu(r, &route[i], hlist) {
					if ((nid & r->dst_nid_mask) == r->dst_nid) {
						return r;
					}
				}
			}
			return NULL;
		case ROUTE: return to_PTR_ROUTE(rl);
		case NODE: rl = tl(to_PTR_NODE(rl), syscom_node_index(nid, 1));
			goto again;
	}

	BUG();

	return NULL;
}

/* Routing configuration interface *************************************/

/** Add route record to the lookup table */
int syscom_route_record_add(struct syscom_route_record *r, __be16 dst_nid,
		uint8_t dst_nid_len, __be16 src_nid, uint16_t frag_threshold,
		int needed_headroom, bool down, bool reject_ordered,
		bool reject_reliable, const struct syscom_route_record_ops *ops)
{
	struct syscom_route_record *r2;

	if (dst_nid_len > 16 || dst_nid & ~syscom_len2mask(dst_nid_len)) {
		return -EINVAL;
	}

	r->ops = ops;
	r->dst_nid = dst_nid;
	r->dst_nid_mask = syscom_len2mask(dst_nid_len);
	r->src_nid = src_nid;
	r->headroom = needed_headroom;
	r->frag_threshold = frag_threshold;
	r->flags = down << SYSCOM_ROUTE_RECORD_DOWN |
	           reject_ordered << SYSCOM_ROUTE_RECORD_REJECT_ORDERED |
	           reject_reliable << SYSCOM_ROUTE_RECORD_REJECT_RELIABLE;

	atomic_long_set(&r->send_bytes, 0);
	atomic_long_set(&r->send, 0);
	atomic_set(&r->hostdown, 0);
	atomic_set(&r->err, 0);

	spin_lock(&route_lock);

	// We do not need rculock, because we hold the route_lock
	r2 = syscom_route_lookup(r->dst_nid);
	if (r2 && dst_nid_len <= syscom_mask2len(r2->dst_nid_mask)) {
		hlist_for_each_entry(r2, &route[dst_nid_len], hlist) {
			if (r->dst_nid == r2->dst_nid) {
				spin_unlock(&route_lock);
				return -EEXIST;
			}
		}
	}

	hlist_add_head_rcu(&r->hlist, &route[dst_nid_len]);
	syscom_route_tree_add(&syscom_route_root, 0, r, false);
	spin_unlock(&route_lock);

	return 0;
}

/** Destroy the route record */
static inline void syscom_route_record_destroy(struct syscom_route_record *r)
{
	atomic_add(atomic_read(&r->err), &syscom_stats.route_err);
	r->ops->destroy(r);
}

/** Remove route record from the lookup table */
static struct syscom_route_record *_syscom_route_del(__be16 nid,
		uint8_t dst_nid_len)
{
	struct syscom_route_record *r;

	if (dst_nid_len > 16) return ERR_PTR(-EINVAL);

	spin_lock(&route_lock);
	// We do not need rculock, because we hold the spinlock
	r = syscom_route_lookup(nid);
	if (!r || dst_nid_len > syscom_mask2len(r->dst_nid_mask)) {
		goto noent;
	}
	if (dst_nid_len < syscom_mask2len(r->dst_nid_mask)) {
		hlist_for_each_entry(r, &route[dst_nid_len], hlist) {
			if (r->dst_nid == nid) {
				goto found;
			}
		}
		goto noent;
	}
found:	hlist_del_rcu(&r->hlist);
	syscom_route_tree_del(r);
	spin_unlock(&route_lock);

	return r;

noent:	spin_unlock(&route_lock);
	return ERR_PTR(-ENOENT);
}

/** Remove route record from the lookup table */
int syscom_route_del(__be16 nid, uint8_t dst_nid_len)
{
	struct syscom_route_record *r = _syscom_route_del(nid, dst_nid_len);

	if (IS_ERR(r)) {
		return PTR_ERR(r);
	}

	synchronize_rcu();
	syscom_route_record_destroy(r);
	return 0;
}

/** Schedule a delete */
int syscom_route_bulk_del(struct syscom_route_bulk_del_s *del,
		__be16 nid, uint8_t dst_nid_len)
{
	struct syscom_route_record *r = _syscom_route_del(nid, dst_nid_len);

	if (IS_ERR(r)) {
		return PTR_ERR(r);
	}

	r->to_destroy = del->to_destroy;
	del->to_destroy = r;
	if (test_bit(SYSCOM_ROUTE_RECORD_DEL_NEEDS_SYNC, &r->flags)) {
		del->needs_sync = 1;
	}

	return 0;
}

/** Complete bulk delete operation */
void syscom_route_bulk_del_complete(struct syscom_route_bulk_del_s *del)
{
	if (!del->to_destroy) return;

	synchronize_rcu();

	while (del->to_destroy) {
		struct syscom_route_record *r = del->to_destroy;
		del->to_destroy = r->to_destroy;
		syscom_route_record_destroy(r);
	}
}

#define syscom_route_for_each_safe(iter, tmp_int, node) \
	for (tmp_int = ARRAY_SIZE(route) - 1; tmp_int >= 0; tmp_int--) \
		hlist_for_each_entry_safe(iter, node, &route[tmp_int], hlist)

/** Notify routes about a device removal */
static int syscom_route_notify(struct notifier_block *nb,
		unsigned long action, void *infop)
{
	struct netdev_notifier_info *info = infop;
	struct syscom_route_record *r, *to_destroy = NULL;
	struct hlist_node *n;
	int i;

	if (action != NETDEV_UNREGISTER) {
		return 0;
	}

	spin_lock(&route_lock);
	syscom_route_for_each_safe(r, i, n) {
		if (r->ops->netdev_unregister &&
		    r->ops->netdev_unregister(r, info->dev)) {
			hlist_del_rcu(&r->hlist);
			syscom_route_tree_del(r);
			r->to_destroy = to_destroy;
			to_destroy = r;
		}
	}
	spin_unlock(&route_lock);

	synchronize_rcu();

	while (to_destroy) {
		struct syscom_route_record *r = to_destroy;
		to_destroy = r->to_destroy;
		syscom_route_record_destroy(r);
	}

	return 0;
}

/** Notifier callback block for netdevices events */
static struct notifier_block syscom_route_nb = {
	.notifier_call = syscom_route_notify,
};

/** Initialize the lookup table */
int syscom_route_init(void)
{
	int i;

	BUILD_BUG_ON(MAX_SKB_FRAGS < 16);

	for (i = 0; i < ARRAY_SIZE(route); i++) {
		INIT_HLIST_HEAD(&route[i]);
	}

	return register_netdevice_notifier(&syscom_route_nb);
}

/** De-initialize the lookup table, free all records */
void syscom_route_destroy(void)
{
	struct syscom_route_record *r;
	struct hlist_node *n;
	int i;

	i = unregister_netdevice_notifier(&syscom_route_nb);
	BUG_ON(i);

	syscom_route_for_each_safe(r, i, n) {
		syscom_route_tree_del(r);
		syscom_route_record_destroy(r);
	}

	cancel_delayed_work_sync(&syscom_route_tree_optimize_work);
}

#if NET_RX_SUCCESS != NET_XMIT_SUCCESS
#error "NET_RX_SUCCESS and NET_XMIT_SUCCESS aren't the same."
#elif NET_RX_DROP != NET_XMIT_DROP
#error "NET_RX_DROP and NET_XMIT_DROP aren't the same."
#endif

/** Process delivery request
 * @return -errno value, which can be forwarded to the user, or NET_XMIT_SUCCESS
 *         if the message is successfully routed or NET_XMIT_DROP if it's dropped
 */
static int syscom_route_deliver(struct syscom_delivery *d, __be16 nid,
		bool remote_delivery)
{
	struct syscom_route_record *r;
	mm_segment_t seg;
	int rtn;

again:	rcu_read_lock();
	r = syscom_route_lookup(nid);
	if (unlikely(!r)) {
		rtn = -ENETUNREACH;
		goto err;
	}

	if (!likely(remote_delivery || syscom_route_record_is_local(r))) {
		rtn = -EREMOTE;
		goto err;
	}

	if (unlikely(test_bit(SYSCOM_ROUTE_RECORD_DOWN, &r->flags))) {
		rtn = -EHOSTDOWN;
		atomic_inc(&r->hostdown);
		goto err;
	}

	if (unlikely((d->ordered &&
	     test_bit(SYSCOM_ROUTE_RECORD_REJECT_ORDERED, &r->flags)) ||
	    (d->reliable &&
	     test_bit(SYSCOM_ROUTE_RECORD_REJECT_RELIABLE, &r->flags)))) {
		rtn = -EMEDIUMTYPE;
		goto err;
	}

	d->src_nid = r->src_nid;
	d->frag_threshold = r->frag_threshold;
	seg = get_fs();
	rtn = r->ops->deliver(r, d);
	set_fs(seg);
	if (likely(!rtn)) {
		atomic_long_inc(&r->send);
		atomic_long_add(d->msg_size, &r->send_bytes);
	} else if (rtn == -EUNATCH) {
		// Route has changed during the delivery, repeat
		// TODO: Update timeout
		goto again;
	} else if (rtn == -EINPROGRESS) {
		rtn = 0;
	} else if (rtn != -EAGAIN) {
		atomic_inc(&r->err);
	}
	if (d->raw_skb) {
		syscom_raw_queue_skb(d->raw_skb, d->gfp_mask, rtn);
	}
	return rtn;

err:	rcu_read_unlock();
	return rtn;
}

/** Deliver a raw message according to the route table */
int syscom_route_deliver_buf(void *buf, size_t size, long timeo)
{
	struct syscom_hdr *hdr = buf;
	int len = ntohs(hdr->length), ttl = hdr->ttl, rtn;
	struct syscom_delivery_buf dbuf = {
		.kvec.iov_base = buf, // We make iovec read-only later
		.kvec.iov_len = len,
		.super = {
			.timeo = timeo,
			.gfp_mask = GFP_KERNEL,
			.backpressure = 1,
			.msg_size = len,
			.ordered = hdr->ordered,
			.ops = &syscom_delivery_buf_ops,
		},
	};
	bool forward;

	if (unlikely(sizeof *hdr > size || len > size)) {
		return -EBADMSG;
	}

	forward = READ_ONCE(syscom_forward) && hdr->ttl < 0x7;
	if (forward) hdr->ttl += 1;

	trace_syscom_route_deliver_start(hdr, &dbuf.super);
	rtn = syscom_route_deliver(&dbuf.super, hdr->dst.nid, forward);
	trace_syscom_route_deliver_done(rtn);

	hdr->ttl = ttl;

	return rtn;
}

/** Deliver a message according to the route table */
int syscom_route_deliver_msg(struct syscom_sock *src,
		__be16 nid, __be16 cpid, struct msghdr *msg, size_t size)
{
	struct syscom_delivery_msg dmsg = {
		.src = src,
		.msg = msg,
		.super = {
			.timeo = sock_sndtimeo(&src->sk,
					msg->msg_flags & MSG_DONTWAIT),
			.backpressure = 1,
			.gfp_mask = GFP_KERNEL,
			.ordered = src->ordered,
			.ops = &syscom_delivery_msg_ops,
		},
	};
	struct syscom_dgram_hdr dgram_hdr;
	int rtn;

	size += SYSCOM_DGRAM_RAW_HDR_DELTA;
	if (unlikely(size < sizeof(struct syscom_hdr))) {
		return -EBADMSG;
	}
	if (unlikely(size > SYSCOM_MTU)) {
		return -EMSGSIZE;
	}

	rtn = memcpy_from_msg(&dgram_hdr, msg, sizeof dgram_hdr);
	if (rtn) {
		return rtn;
	}

	dmsg.super.msg_size = size;
	dmsg.super.reliable = src->reliable ||
			!(dgram_hdr.flags & htons(SYSCOM_HDR_FLAG_UNRELIABLE));
	dmsg.hdr.msg_id = dgram_hdr.msg_id;
	dmsg.hdr.ordered = src->ordered;
	dmsg.hdr.src.nid = src->local.nid;
	dmsg.hdr.src.cpid = src->local.cpid;
	dmsg.hdr.dst.nid = nid;
	dmsg.hdr.dst.cpid = cpid;
	dmsg.hdr.length = htons(size);
	dmsg.hdr.flags = dgram_hdr.flags;

	trace_syscom_route_deliver_start(&dmsg.hdr, &dmsg.super);
	rtn = syscom_route_deliver(&dmsg.super, nid, 1);
	trace_syscom_route_deliver_done(rtn);

	return rtn;
}

/** Deliver a message according to the route table */
int syscom_route_deliver_rawmsg(struct syscom_sock *src, struct msghdr *msg,
		size_t size)
{
	struct syscom_delivery_msg dmsg = {
		.src = src,
		.msg = msg,
		.super = {
			.timeo = sock_sndtimeo(&src->sk,
					msg->msg_flags & MSG_DONTWAIT),
			.backpressure = 1,
			.gfp_mask = GFP_KERNEL,
			.ops = &syscom_delivery_rawmsg_ops,
		},
	};
	int rtn;

	if (unlikely(size < sizeof(struct syscom_hdr))) {
		return -EBADMSG;
	}
	if (unlikely(size > SYSCOM_MTU)) {
		return -EMSGSIZE;
	}

	rtn = memcpy_from_msg(&dmsg.hdr, msg, sizeof dmsg.hdr);
	if (rtn) {
		return rtn;
	}

	if (unlikely(size != ntohs(dmsg.hdr.length))) {
		return -EBADMSG;
	}

	dmsg.super.msg_size = ntohs(dmsg.hdr.length);
	dmsg.super.ordered = dmsg.hdr.ordered;
	dmsg.super.reliable = !(dmsg.hdr.flags & htons(SYSCOM_HDR_FLAG_UNRELIABLE));

	trace_syscom_route_deliver_start(&dmsg.hdr, &dmsg.super);
	rtn = syscom_route_deliver(&dmsg.super, dmsg.hdr.dst.nid, 1);
	trace_syscom_route_deliver_done(rtn);

	return rtn;
}

int _syscom_route_deliver_skb(struct sk_buff *skb, long timeo, gfp_t gfp_mask,
		bool forward, const char *name)
{
	struct syscom_delivery_skb dskb = {
		.skb = skb,
		.super = {
			.timeo = timeo,
			.gfp_mask = gfp_mask,
			.ops = &syscom_delivery_skb_ops,
		}
	};
	struct syscom_hdr *hdr = (struct syscom_hdr *)skb->data;
	int rtn, j;

	dskb.super.msg_size = ntohs(hdr->length);
	dskb.super.ordered = hdr->ordered;
	dskb.super.reliable = !(hdr->flags & htons(SYSCOM_HDR_FLAG_UNRELIABLE));

	trace_syscom_route_deliver_start(hdr, &dskb.super);
	rtn = syscom_route_deliver(&dskb.super, hdr->dst.nid, forward);
	trace_syscom_route_deliver_done(rtn);

	if (dskb.mapped) {
		BUG_ON(!dskb.skb);
		for (j = 0; j < skb_shinfo(dskb.skb)->nr_frags; j++) {
			kunmap(skb_frag_page(&skb_shinfo(dskb.skb)->frags[j]));
		}
	}

	if (unlikely(rtn)) {
		if (dskb.skb) {
			skb = dskb.skb;
			if (!name && skb->dev)
				name = netdev_name(skb->dev);
			syscom_delivery_error_skb(rtn, skb->data, skb, name);
			kfree_skb(skb);
		} else {
			syscom_delivery_error(rtn, NULL, name);
		}
	} else if (dskb.skb) {
		consume_skb(dskb.skb);
	}

	return rtn;
}

/** Deliver SKB according to the route table */
int syscom_route_deliver_skb(struct sk_buff *skb)
{
	struct syscom_hdr *hdr = (struct syscom_hdr *)skb->data;
	bool forward;

	// Check for the correct length (fragmented header is not supported)
	BUG_ON(skb->protocol != htons(ETH_P_SYSCOM));
	if (unlikely(sizeof *hdr > skb->len - skb->data_len ||
			ntohs(hdr->length) > skb->len)) {
		syscom_delivery_error_skb(-EBADMSG, hdr, skb,
				skb->dev ? netdev_name(skb->dev) : NULL);
		return -EBADMSG;
	}

	forward = READ_ONCE(syscom_forward) && hdr->ttl < 0x7;
	if (forward) hdr->ttl += 1;

	return _syscom_route_deliver_skb(skb, 0, GFP_ATOMIC, forward, NULL);
}

/* Proc FS (content of /proc/net/syscom_route) *************************/

/** Get the next syscom_route_record for the specified iterator */
static struct syscom_route_record *route_next(struct syscom_route_iter_state *iter)
{
	if (iter->node) {
		iter->node = hlist_next_rcu(iter->node);
		if (iter->node) {
			return hlist_entry_safe(rcu_dereference_raw(iter->node),
					struct syscom_route_record, hlist);
		}
	}

	for (iter->idx--; iter->idx >= 0; iter->idx--) {
		iter->node = hlist_first_rcu(&route[iter->idx]);
		if (iter->node) {
			return hlist_entry_safe(rcu_dereference_raw(iter->node),
					struct syscom_route_record, hlist);
		}
	}

	return NULL;
}

/** Get the first syscom_route_record for the specified iterator */
static struct syscom_route_record *route_first(struct syscom_route_iter_state *iter)
{
	iter->idx = ARRAY_SIZE(route);
	iter->node = NULL;

	return route_next(iter);
}

void syscom_route_record_iterate(
		void (*cb)(struct syscom_route_record *, void *), void *arg)
{
	struct syscom_route_iter_state iter;
	struct syscom_route_record *r;

	rcu_read_lock();
	for (r = route_first(&iter); r; r = route_next(&iter)) {
		cb(r, arg);
	}
	rcu_read_unlock();
}

#ifdef CONFIG_PROC_FS

const struct seq_operations syscom_route_seq_ops = {
	.start  = syscom_route_seq_start,
	.next   = syscom_route_seq_next,
	.stop   = syscom_route_seq_stop,
	.show   = syscom_route_seq_show,
};

/** Get the route record at the specified offset */
static struct syscom_route_record *route_idx(struct seq_file *seq, loff_t pos)
{
	struct syscom_route_iter_state *iter = seq->private;
	struct syscom_route_record *r;
	loff_t off = 0;

	for (r = route_first(iter); r; r = route_next(iter)) {
		if (off == pos)
			return r;
		++off;
	}

	return NULL;
}

/** Start callback of seq_file to dump the route table  */
void *syscom_route_seq_start(struct seq_file *seq, loff_t *pos)
{
	rcu_read_lock();
	return *pos ? route_idx(seq, *pos - 1) : SEQ_START_TOKEN;
}

/** Next callback of seq_file to dump the route table  */
void *syscom_route_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct syscom_route_iter_state *iter = seq->private;

	++*pos;
	return v == SEQ_START_TOKEN ? route_first(iter) : route_next(iter);
}

/** Stop callback of seq_file to dump the route table  */
void syscom_route_seq_stop(struct seq_file *seq, void *v)
{
	rcu_read_unlock();
}

/** Show callback of seq_file to dump the route table  */
int syscom_route_seq_show(struct seq_file *seq, void *v)
{
	struct syscom_route_record *r = v;

	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, "dst     src  type  fla       packets         bytes err hdown  frag dev/proto addr\n");
	} else {
		seq_printf(seq, "%04x/%-2d %04x %-5s %c%c%c %13lu %13lu %3u %5u %5u ",
			ntohs(r->dst_nid), syscom_mask2len(r->dst_nid_mask),
			ntohs(r->src_nid), r->ops->type,
			test_bit(SYSCOM_ROUTE_RECORD_DOWN, &r->flags) ? 'd' : 'u',
			test_bit(SYSCOM_ROUTE_RECORD_REJECT_RELIABLE, &r->flags) ? 'r' : '-',
			test_bit(SYSCOM_ROUTE_RECORD_REJECT_ORDERED, &r->flags) ? 'o' : '-',
			atomic_long_read(&r->send), atomic_long_read(&r->send_bytes),
			atomic_read(&r->err), atomic_read(&r->hostdown),
			r->frag_threshold);
		r->ops->seq_show(r, seq);
	}
	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 18, 0))
/** Open sequential file for inspection of the route table */
int syscom_route_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &syscom_route_seq_ops,
			    sizeof(struct syscom_route_iter_state));
}
#endif

#endif
