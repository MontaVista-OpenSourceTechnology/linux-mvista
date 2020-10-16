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

#include <asm/atomic.h>

#include "route-gw.h"
#include "route.h"
#include "gw.h"

/** Gateway specific route record. */
struct syscom_route_record_gw {
	/** Parent route record */
	struct syscom_route_record super;
	/** Socket used for the sending */
	struct syscom_gw __rcu *gw;
	/** List to maintain routes used by a gateway */
	struct list_head gw_routes;
	/** The gateway name (as gw member can be NULL, if NOGW mode is enabled) */
	char gw_name[SYSCOM_GW_NAME_MAX];
	/** How the route should behave if the gateway doesn't exist */
	enum {
		NOGW_FAIL,
		NOGW_BLOCK,
		NOGW_DESTROY,
	} nogw:2;
};

static LIST_HEAD(syscom_route_gw_pending);
static DEFINE_SPINLOCK(syscom_route_gw_pending_lock);
static atomic_t route_changes;
static DECLARE_WAIT_QUEUE_HEAD(syscom_gw_wq);

/** Cast to the child type with a type check. */
static inline struct syscom_route_record_gw *syscom_route_record_gw(
		const struct syscom_route_record *r)
{
	return (struct syscom_route_record_gw *)r;
}

/** Wake up tasks waiting for route gw change */
static void syscom_route_gw_change(void)
{
	atomic_inc(&route_changes);
	wake_up_interruptible_all(&syscom_gw_wq);
}

/** Route record destructor. */
static void syscom_route_record_gw_destroy(struct syscom_route_record *r)
{
	struct syscom_route_record_gw *self = syscom_route_record_gw(r);
	struct syscom_gw *gw;

	spin_lock(&syscom_route_gw_pending_lock);
	list_del(&self->gw_routes);
	spin_unlock(&syscom_route_gw_pending_lock);

	// We know we are the last user as we are called after
	// synchronize_rcu and we removed ourselves from gw_routes
	// list, where records are usable only with syscom_route_gw_pending_lock
	// held
	gw = rcu_dereference_protected(self->gw, 1);
	if (gw) {
		syscom_gw_put(gw);
	}

	kfree(r);
	syscom_route_gw_change();
}

#ifdef CONFIG_PROC_FS

/** Dump the record to the seq file. */
static void syscom_route_record_gw_seq_show(
		struct syscom_route_record *r, struct seq_file *seq)
{
	const struct syscom_route_record_gw *self = syscom_route_record_gw(r);

	seq_printf(seq, "%c%c        %s\n",
			rcu_access_pointer(self->gw) ? 'G' : '-',
			"FBD"[self->nogw], self->gw_name);
}
#endif // CONFIG_PROC_FS

/** Deliver the message over a socket. */
static int syscom_route_record_gw_deliver(const struct syscom_route_record *r,
		struct syscom_delivery *d) __releases(rcu)
{
	const struct syscom_route_record_gw *self = syscom_route_record_gw(r);
	struct syscom_gw *gw;
	struct iov_iter iov;
	int rtn, value;
	long timeo;

	gw = rcu_dereference(self->gw);
	if (likely(gw && syscom_gw_get(gw))) {
found:		gw = rcu_pointer_handoff(gw);
		rcu_read_unlock();

		if (unlikely((d->ordered && syscom_gw_test_bit(gw, REJECT_ORDERED)) ||
				(d->reliable && syscom_gw_test_bit(gw, REJECT_RELIABLE)))) {
			syscom_gw_put(gw);
			return -EMEDIUMTYPE;
		}

		if (syscom_delivery_is_atomic(d)) {
			struct work_struct *work;
			work = syscom_delivery_get_work(d, gw->name);
			if (!work) {
				rtn = -ENOMEM;
			} else {
				queue_work(gw->send_wq, work);
				rtn = -EINPROGRESS;
			}
		} else {
			rtn = syscom_delivery_get_iov(d, &iov, &timeo);
			if (!rtn) {
				rtn = syscom_gw_send(gw, &iov, timeo);
			}
		}
		syscom_gw_put(gw);
		return rtn;
	}

	switch (self->nogw) {
		case NOGW_FAIL:
			rtn = -ENOTCONN;
			break;
		case NOGW_BLOCK:
			if (syscom_delivery_get_timeout(d) > 0) {
				goto sleep;
			} else {
				rtn = -EAGAIN;
			}
			break;
		case NOGW_DESTROY:
			rtn = -EUNATCH;
			break;
		default:
			rtn = -EIO; // Make the compiler happy
	}
	rcu_read_unlock();
	return rtn;


sleep:	value = atomic_read(&route_changes);
	// TODO: Should we check if the routing changed as well? Desired
	//       behaviour should be discussed with architects.
	gw = rcu_dereference(self->gw);
	if (gw && syscom_gw_get(gw)) {
		goto found;
	}
	rcu_read_unlock();

	timeo = wait_event_interruptible_timeout(syscom_gw_wq,
			value != atomic_read(&route_changes),
			syscom_delivery_get_timeout(d));
	if (timeo >= 1) {
		syscom_delivery_set_timeout(d, timeo);
		return -EUNATCH; // Repeat delivery attempt
	}

	return timeo == -ERESTARTSYS ? -ERESTARTSYS : -EAGAIN;
}

/** Route record operations for socket routes. */
static const struct syscom_route_record_ops syscom_route_record_gw_ops = {
	.destroy = syscom_route_record_gw_destroy,
	.deliver = syscom_route_record_gw_deliver,
#ifdef CONFIG_PROC_FS
	.seq_show = syscom_route_record_gw_seq_show,
	.type = "GW",
#endif // CONFIG_PROC_FS
};

/** Create a new socket route record. */
int syscom_route_record_gw_add(__be16 dst_nid, uint8_t dst_nid_len,
	__be16 src_nid, const char *gw_name, int flags)
{
	struct syscom_route_record_gw *self;
	struct syscom_gw *gw;
	int rtn;

	if (flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_FAIL &&
	    flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_BLOCK) {
		return -EINVAL;
	}

	if (flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW &&
	    !(flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_FAIL ||
	      flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_BLOCK)) {
		return -EINVAL;
	}
	self = kmalloc(sizeof *self, GFP_KERNEL);
	if (!self) {
		return -ENOMEM;
	}

	if (flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_BLOCK) {
		self->nogw = NOGW_BLOCK;
	} else if (flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_FAIL) {
		self->nogw = NOGW_FAIL;
	} else {
		self->nogw = NOGW_DESTROY;
	}

	strbcpy(self->gw_name, gw_name, sizeof self->gw_name);

	syscom_gw_lock();
	gw = syscom_gw_lookup(gw_name);
	rcu_assign_pointer(self->gw, gw);
	if (gw) {
		syscom_gw_get(gw);
		list_add(&self->gw_routes, &gw->routes);
	} else if (flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW) {
		spin_lock(&syscom_route_gw_pending_lock);
		list_add(&self->gw_routes, &syscom_route_gw_pending);
		spin_unlock(&syscom_route_gw_pending_lock);
	} else {
		rtn = -ENOENT;
		goto err0;
	}

	rtn = syscom_route_record_add(&self->super, dst_nid, dst_nid_len,
			src_nid, 65535, 0, 0,
			flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_ORDERED,
			flags & SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_RELIABLE,
			&syscom_route_record_gw_ops);
	if (rtn) {
		goto err1;
	}
	__set_bit(SYSCOM_ROUTE_RECORD_DEL_NEEDS_SYNC, &self->super.flags);
	syscom_gw_unlock();
	return 0;

err1:	spin_lock(&syscom_route_gw_pending_lock);
	list_del(&self->gw_routes);
	spin_unlock(&syscom_route_gw_pending_lock);
	if (gw) syscom_gw_put(gw);
err0:	syscom_gw_unlock();
	kfree(self);
	return rtn;
}

/** Introduce a new gateway to the system. Serialized by gateways_lock. */
void syscom_route_gw_introduce_gw(struct syscom_gw *gw)
{
	struct syscom_route_record_gw *r, *tmp;
	bool changed = 0;

	spin_lock(&syscom_route_gw_pending_lock);
	list_for_each_entry_safe(r, tmp, &syscom_route_gw_pending, gw_routes) {
		if (!strcmp(r->gw_name, gw->name)) {
			list_move(&r->gw_routes, &gw->routes);
			// No need to check if we got the reference, because our
			// caller must have one
			rcu_assign_pointer(r->gw, syscom_gw_get(gw));
			changed = 1;
		}
	}
	spin_unlock(&syscom_route_gw_pending_lock);

	if (changed) {
		syscom_route_gw_change();
	}
}

/** Remove a gateway from the system. Serialized by gateways_lock. */
void syscom_route_gw_remove_gw(struct syscom_gw *gw)
{
	struct syscom_route_record_gw *r, *tmp;

	// NOGW_DESTROY sleeps
	might_sleep();

rst:	spin_lock(&syscom_route_gw_pending_lock);
	list_for_each_entry_safe(r, tmp, &gw->routes, gw_routes) {
		if (r->nogw == NOGW_DESTROY) {
			__be16 nid = r->super.dst_nid;
			uint8_t nid_len = syscom_mask2len(r->super.dst_nid_mask);
			spin_unlock(&syscom_route_gw_pending_lock);
			syscom_route_del(nid, nid_len);
			goto rst;
		} else {
			rcu_assign_pointer(r->gw, NULL);
			list_move(&r->gw_routes, &syscom_route_gw_pending);
			syscom_gw_put(gw); // Our caller has a reference too
		}
	}
	spin_unlock(&syscom_route_gw_pending_lock);
}
