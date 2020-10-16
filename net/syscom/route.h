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

#ifndef SYSCOM_ROUTE_H
#define SYSCOM_ROUTE_H

#include "common.h"

#include <linux/list.h>
#include "delivery.h"

struct syscom_route_record;
struct net_device;
struct syscom_sock;
struct msghdr;

#ifdef CONFIG_PROC_FS
struct file;
struct inode;
struct seq_file;

void *syscom_route_seq_start(struct seq_file *seq, loff_t *pos);
void *syscom_route_seq_next(struct seq_file *seq, void *v, loff_t *pos);
void syscom_route_seq_stop(struct seq_file *seq, void *v);
int syscom_route_seq_show(struct seq_file *seq, void *v);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 18, 0))
int syscom_route_seq_open(struct inode *inode, struct file *file);
#endif
extern const struct seq_operations syscom_route_seq_ops;
#endif // CONFIG_PROC_FS

/* Route table iterator */
struct syscom_route_iter_state {
#ifdef CONFIG_PROC_FS
	struct seq_net_private p;
#endif // CONFIG_PROC_FS
	struct hlist_node *node;
	int idx;
};

/** Route record */
struct syscom_route_record_ops {
	/** Pointer to destructor (to release transport specific resources). */
	void (*destroy)(struct syscom_route_record *);
	/** Called if a netdevice is unregistered. If it returns true, the route
	 *  will be removed (optional). */
	bool (*netdev_unregister)(const struct syscom_route_record *,
			const struct net_device *);
#ifdef CONFIG_PROC_FS
	/** Print the route information into seq file */
	void (*seq_show)(struct syscom_route_record *, struct seq_file *);
	/** Record type (shown in /proc/net/syscom_route) */
	const char *type;
#endif // CONFIG_PROC_FS
	/** Deliver message on this route. Called with rcu_lock held to protect
	  * the route record and is responsible for unlocking it. */
	int (*deliver)(const struct syscom_route_record *,
			struct syscom_delivery *);
};

enum syscom_route_record_flags {
	SYSCOM_ROUTE_RECORD_DOWN,
	SYSCOM_ROUTE_RECORD_DEL_NEEDS_SYNC,
	SYSCOM_ROUTE_RECORD_REJECT_ORDERED,
	SYSCOM_ROUTE_RECORD_REJECT_RELIABLE,
};

/** Route record */
struct syscom_route_record {
	/** List to queue messages based on mask */
	struct hlist_node hlist;
	/** Destination node ID - the route lookup key */
	__be16 dst_nid;
	/** Destination node mask */
	__be16 dst_nid_mask;
	/** Source node ID used for automatic NID assignment */
	__be16 src_nid;
	/** Needed headroom for this route */
	short headroom;
	/** If the message is bigger than this, it's fragmented */
	uint16_t frag_threshold;
	/** Send messages counter */
	atomic_long_t send;
	/** Send bytes counter */
	atomic_long_t send_bytes;
	/** Error counter */
	atomic_t err;
	/** Number of messages send to a disabled route */
	atomic_t hostdown;
	/** Flags */
	unsigned long flags;
	/** Transport specific operations */
	const struct syscom_route_record_ops *ops;
	/** Used to build up a list of records to release under rcu lock */
	struct syscom_route_record *to_destroy;
};

/** Calculate length of the mask */
#define syscom_mask2len(x) (16 - fls((unsigned short)ntohs(~(x))))

int syscom_route_del(__be16 nid, uint8_t dst_nid_len);
int syscom_route_init(void);
void syscom_route_destroy(void);
int syscom_route_deliver_buf(void *buf, size_t size, long timeo);
int syscom_route_deliver_msg(struct syscom_sock *src,
			__be16 nid, __be16 cpid, struct msghdr *msg, size_t size);
int syscom_route_deliver_rawmsg(struct syscom_sock *src, struct msghdr *msg,
		size_t size);
int syscom_route_deliver_skb(struct sk_buff *skb);

int syscom_route_record_add(struct syscom_route_record *r, __be16 dst_nid,
		uint8_t dst_nid_len, __be16 src_nid, uint16_t frag_threshold,
		int needed_headroom, bool down, bool reject_ordered,
		bool reject_reliable, const struct syscom_route_record_ops *ops);

static inline bool syscom_route_record_is_local(
		const struct syscom_route_record *r)
{
	extern const struct syscom_route_record_ops
			syscom_route_record_local_ops;
	return r->ops == &syscom_route_record_local_ops;
}

void syscom_route_record_iterate(
		void (*cb)(struct syscom_route_record *, void *), void *arg);

static inline void syscom_route_up_down(struct syscom_route_record *r,
		bool up)
{
	if (up) {
		__clear_bit(SYSCOM_ROUTE_RECORD_DOWN, &r->flags);
	} else {
		__set_bit(SYSCOM_ROUTE_RECORD_DOWN, &r->flags);
	}
}

/** Opaque cookie for bulk route deletion */
struct syscom_route_bulk_del_s {
	/** True if we need to proceed before non-delete operation */
	bool needs_sync;
	/** List of routes scheduled for destruction */
	struct syscom_route_record *to_destroy;
};

/** Prepare bulk deletion */
static inline void syscom_route_bulk_del_prepare(
		struct syscom_route_bulk_del_s *del)
{
	*del = (struct syscom_route_bulk_del_s){ 0 };
}

void syscom_route_bulk_del_complete(struct syscom_route_bulk_del_s *del);

/** Submit postponed deletes, if further operations could depend on them */
static inline void syscom_route_bulk_del_sync(
		struct syscom_route_bulk_del_s *del)
{
	if (del->needs_sync) {
		syscom_route_bulk_del_complete(del);
		syscom_route_bulk_del_prepare(del);
	}
}

int syscom_route_bulk_del(struct syscom_route_bulk_del_s *del, __be16 nid,
		uint8_t dst_nid_len);

#endif // SYSCOM_ROUTE_H
