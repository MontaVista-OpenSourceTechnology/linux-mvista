// SPDX-License-Identifier: GPL-2.0
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

#include <linux/rcupdate.h>
#include <linux/slab.h>

#include "route.h"
#include "route-local.h"

/** Destructor for a local route record. */
static void syscom_route_record_local_destroy(struct syscom_route_record *r)
{
	kfree(r);
}

#ifdef CONFIG_PROC_FS
/** Dump the record to the seq file. */
static void syscom_route_record_local_seq_show(
		struct syscom_route_record *r, struct seq_file *seq)
{
	seq_printf(seq, "-         -\n");
}
#endif

/** Deliver the message to the local socket. */
static int syscom_route_record_local_deliver(const struct syscom_route_record *r,
		struct syscom_delivery *d) __releases(rcu)
{
	struct sk_buff *skb;
	long timeo;
	bool bp;
	int rtn;

	rcu_read_unlock();

	// We can't use nid and cpid from dst_sock, because they can be a wildcard
	// values, which would then make the wildcard visible in the dump instead
	// of the right address.
	rtn = syscom_delivery_get_skb(d, &skb, &timeo, &bp, 0,
			SYSCOM_MAX_MSGSIZE);
	if (rtn) {
		return rtn;
	}

	rtn = syscom_queue_rcv_skb(skb, timeo, bp);
	if (rtn == -ENOMEM) {
		// If we want to behave the same as a network connection,
		// we do not sleep here, but drop the message and return
		// success
		rtn = 0;
	}

	return rtn;
}

/** Route record operations for local routes.
 * Needs to be visible for syscom_route_record_is_local().
 */
const struct syscom_route_record_ops syscom_route_record_local_ops = {
	.destroy = syscom_route_record_local_destroy,
	.deliver = syscom_route_record_local_deliver,
#ifdef CONFIG_PROC_FS
	.seq_show = syscom_route_record_local_seq_show,
	.type = "LOCAL",
#endif
};

/** Create a new local route record (to deliver on local sockets). */
int syscom_route_record_local_add(__be16 dst_nid,
		uint8_t dst_nid_len, __be16 src_nid)
{
	struct syscom_route_record *self;
	int rtn;

	self = kmalloc(sizeof *self, GFP_KERNEL);
	if (!self) {
		return -ENOMEM;
	}

	rtn = syscom_route_record_add(self, dst_nid, dst_nid_len, src_nid,
			SYSCOM_MAX_MSGSIZE, 0, 0, 0, 0,
			&syscom_route_record_local_ops);
	if (rtn) {
		kfree(self);
	}

	return rtn;
}
