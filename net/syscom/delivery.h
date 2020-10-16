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

#ifndef SYSCOM_DELIVERY_H
#define SYSCOM_DELIVERY_H

#include "common.h"

#include <linux/if_syscom_ether.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/uio.h>
#include <net/syscom.h>

#include "raw.h"

struct syscom_delivery;

/** Virtual methods */
struct syscom_delivery_ops {
	/** Turn delivery into an IOV */
	int (*get_iov)(struct syscom_delivery *d, struct iov_iter *i);
	/** Turn delivery into an SKB */
	int (*get_skb)(struct syscom_delivery *d, struct sk_buff **head,
			int headroom, int mtu);
};

/** Structure to represent a delivery request. */
struct syscom_delivery {
	/** Can the sender do a back pressure */
	uint8_t backpressure:1;
	/** Is ordered delivery needed */
	uint8_t ordered:1;
	/** Is ordered delivery needed */
	uint8_t reliable:1;
	/** How long we can sleep */
	long timeo;
	/** Mask for allocations */
	gfp_t gfp_mask;
	/** Size of the syscom message */
	int msg_size;
	/** Fragmentation threshold */
	int frag_threshold;
	/** Default source NID */
	__be16 src_nid;
	/** SKB for raw sockets */
	struct sk_buff *raw_skb;
	/** Virtual methods pointer */
	const struct syscom_delivery_ops *ops;
};

struct syscom_delivery_buf {
	/** The parent class */
	struct syscom_delivery super;
	/** kvec describing the buffer */
	struct kvec kvec;
};

struct syscom_delivery_skb {
	/** The parent class */
	struct syscom_delivery super;
	/** SKB with a message, to be delivered */
	struct sk_buff *skb;
	/** kvec describing the skb */
	struct kvec skb_kvec[MAX_SKB_FRAGS + 1];
	bool mapped;
};

struct syscom_delivery_msg {
	/** The parent class */
	struct syscom_delivery super;
	/** Socket sending the message */
	struct syscom_sock *src;
	/** The message */
	struct msghdr *msg;
	/** Syscom header for the message */
	struct syscom_hdr hdr;
	/** Inlined iovec */
	struct iovec iovec[6];
};

void syscom_delivery_raw_from_iov(struct syscom_delivery *, struct iov_iter *);

void syscom_delivery_raw_from_skb(struct syscom_delivery *, struct sk_buff *);

static inline int syscom_delivery_get_iov(struct syscom_delivery *d,
		struct iov_iter *i, long *timeo)
{
	int rtn;

	if (timeo) *timeo = d->timeo;

	rtn = d->ops->get_iov(d, i);
	if (syscom_raw_delivery_needed() && !rtn) {
		syscom_delivery_raw_from_iov(d, i);
	}

	return rtn;
}

static inline int syscom_delivery_get_skb(struct syscom_delivery *d,
		struct sk_buff **head, long *timeo, bool *bp,
		int headroom, int mtu)
{
	int rtn;

	if (timeo) *timeo = d->timeo;
	if (bp) *bp = d->backpressure;

	if (unlikely(mtu < d->frag_threshold)) {
		pr_err_ratelimited("Routing misconfiguration - frag_threshold:"
				" %d > mtu: %d\n", d->frag_threshold, mtu);
		return -EMSGSIZE;
	}

	rtn = d->ops->get_skb(d, head, headroom, mtu);
	if (syscom_raw_delivery_needed() && !rtn) {
		syscom_delivery_raw_from_skb(d, *head);
	}

	return rtn;
}

struct work_struct *syscom_delivery_get_work(struct syscom_delivery *d,
		const char *name);

static inline long syscom_delivery_get_timeout(const struct syscom_delivery *d)
{
	return d->timeo;
}

static inline void syscom_delivery_set_timeout(struct syscom_delivery *d,
		long timeo)
{
	BUG_ON(d->timeo < timeo); // We shouldn't increase user provided timeo
	d->timeo = timeo;
}

static inline bool syscom_delivery_is_atomic(const struct syscom_delivery *d)
{
	return d->gfp_mask & __GFP_ATOMIC;
}

extern const struct syscom_delivery_ops syscom_delivery_buf_ops;
extern const struct syscom_delivery_ops syscom_delivery_skb_ops;
extern const struct syscom_delivery_ops syscom_delivery_msg_ops;
extern const struct syscom_delivery_ops syscom_delivery_rawmsg_ops;

#endif // SYSCOM_DELIVERY_H
