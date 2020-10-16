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

#include <net/sctp/sctp.h>
#include <linux/workqueue.h>
#include <linux/rcupdate.h>
#include <linux/errqueue.h>
#include <linux/vmalloc.h>
#include <linux/atomic.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/bug.h>
#include <linux/net.h>
#include <linux/fs.h>
#include <linux/sctp.h>
#include <linux/poll.h>
#include <linux/crc16.h>
#include <linux/version.h>
#include <linux/sched/signal.h>
#include <asm/errno.h>

#include "route-gw.h"
#include "route.h"
#include "notify.h"
#include "gw.h"

#define SYSCOM_STREAM_SIZE(size) (SKB_TRUESIZE(size) + 64)
#define SYSCOM_GW_DELIM ':'
#define SYSCOM_GW_BUFSIZE (268*1024) // Should fit at least four messages
#define SYSCOM_GW_CMSG_SIZE 256
#define SYSCOM_SCTP_STREAM_NUM 256

#define syscom_gw_set_bit(gw, bit) \
		__set_bit(SYSCOM_GW_##bit##_BIT, &(gw)->flags)

#define syscom_gw_send_flags(noblock) \
		(MSG_NOSIGNAL | ((noblock) ? MSG_DONTWAIT : 0))

static DEFINE_MUTEX(gateways_lock);
static LIST_HEAD(gateways);
static struct workqueue_struct *syscom_wq;

struct syscom_gw_ops_s;

struct syscom_gw_ops_s {
	int (*init)(struct syscom_gw *gw, struct socket *sock, void *arg);
	int (*listen)(struct syscom_gw *gw, int backlog);
	int (*connect)(struct syscom_gw *gw, struct syscom_gw *parent_gw,
			struct sockaddr *addrs, int addrcnt, int addrsize);
	int (*send)(struct syscom_gw *gw, struct iov_iter *iov, bool noblock);
	/** Called when socket is marked ready. Can return -EAGAIN to be
	 *  rescheduled automatically or -ENOTCONN when the connection is
	 *  closed. Other values have no meaning. */
	int (*ready)(struct syscom_gw *gw);
	void (*destroy)(struct syscom_gw *gw);
	int (*destroy_sync_safe)(struct syscom_gw *gw, bool del_children);
	const struct syscom_gw_ops_s *connect_ops;
	char identifier;
};

static const struct syscom_gw_ops_s syscom_gw_sctp_seqpacket_base_ops;
static const struct syscom_gw_ops_s syscom_gw_sctp_seqpacket_child_ops;
static const struct syscom_gw_ops_s syscom_gw_dgram_base_ops;
static const struct syscom_gw_ops_s syscom_gw_dgram_child_ops;
static const struct syscom_gw_ops_s syscom_gw_stream_base_ops;
static const struct syscom_gw_ops_s syscom_gw_stream_child_a_ops;
static const struct syscom_gw_ops_s syscom_gw_stream_child_c_ops;

static struct syscom_gw *syscom_gw_spawn_child(struct syscom_gw *gw,
		const char *name, struct socket *sock, struct sockaddr *dstaddr,
		const struct syscom_gw_ops_s *ops, void *init_arg);

static int syscom_gw_no_listen(struct syscom_gw *gw, int backlog)
{
	return -EOPNOTSUPP;
}

static int syscom_gw_no_connect(struct syscom_gw *gw, struct syscom_gw *child_gw,
		struct sockaddr *addrs, int addrcnt, int addrsize)
{
	return -EOPNOTSUPP;
}

static int syscom_gw_no_send(struct syscom_gw *gw, struct iov_iter *iov,
		bool noblock)
{
	return -EOPNOTSUPP;
}

// static void syscom_gw_no_close(struct syscom_gw *gw) { }

/** Hold a lock to the list of gateways. As the list holds one reference to
 *  a gateway, the gateway can't go away. */
void syscom_gw_lock(void)
{
	mutex_lock(&gateways_lock);
}

void syscom_gw_unlock(void)
{
	mutex_unlock(&gateways_lock);
}

/** The blocking part of the GW destructor */
static void syscom_gw_kref_release_worker(struct work_struct *work)
{
	struct syscom_gw *gw = container_of(to_delayed_work(work),
			typeof(*gw), work);

	if (gw->send_wq) {
		destroy_workqueue(gw->send_wq);
	}

	gw->ops->destroy(gw);
	if (gw->hdr) kfree(gw->hdr);

	trace_syscom_gw_destroy(gw);
	syscom_notify_gw_remove(gw->name, gw->reason);
	if (gw->completion) complete(gw->completion);

	atomic_add(atomic_read(&gw->rx_drop), &syscom_stats.gw_rx_drops);
	atomic_add(atomic_read(&gw->tx_drop), &syscom_stats.gw_tx_drops);
	atomic_add(atomic_read(&gw->err), &syscom_stats.gw_errors);
	atomic_add(atomic_read(&gw->ready_would_block), &syscom_stats.gw_ready_would_block);
	syscom_max_update(&gw->rx_hw_us, &syscom_stats.gw_rx_hw_us);
	syscom_max_update(&gw->rx_sw_us, &syscom_stats.gw_rx_sw_us);
	syscom_max_update(&gw->rx_dl_us, &syscom_stats.gw_rx_dl_us);

	// Nobody can hold a reference now, but somebody might try to obtain it
	// until the next grace period after execution of syscom_gw_kref_release
	kfree_rcu(gw, rcu_head);
}

/** GW destructor called when reference counter drops to 0.
 *  This function must not block. */
void syscom_gw_kref_release(struct kref *kref)
{
	struct syscom_gw *self = container_of(kref, typeof(*self), kref);

	smp_mb();

	BUG_ON(!list_empty(&self->routes));
	BUG_ON(!syscom_gw_test_bit(self, STOP_WORK));

	INIT_DELAYED_WORK(&self->work, syscom_gw_kref_release_worker);
	schedule_delayed_work(&self->work, 0);
}

/** Look up a gateway by its name. Must be called under the gw_lock. */
struct syscom_gw *syscom_gw_lookup(const char *name)
{
	struct syscom_gw *iter;

	lockdep_assert_held(&gateways_lock);

	list_for_each_entry(iter, &gateways, gateways_list) {
		if (!strcmp(name, iter->name)) {
			return iter;
		}
	}

	return NULL;
}

static struct syscom_gw *syscom_gw_lookup_get(const char *name)
{
	struct syscom_gw *gw;

	syscom_gw_lock();
	gw = syscom_gw_lookup(name);
	if (gw) {
		syscom_gw_get(gw);
	}
	syscom_gw_unlock();

	return gw;
}

static int base64(char *out, int outlen, const void *inp, int inlen)
{
#define NEXT (i + 1 < inlen ? in[i + 1] : 0)
	const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"abcdefghijklmnopqrstuvwxyz0123456789_.";
	const uint8_t *in = inp;
	int i, o;

	for (i = o = 0; i < inlen && o < outlen - 1; o++) {
		switch (o & 0x3) {
			case 0: out[o] = table[in[i] >> 2];
				break;
			case 1: out[o] = table[(in[i] & 0x3) << 4 | NEXT >> 4];
				i++;
				break;
			case 2: out[o] = table[(in[i] & 0xf) << 2 | NEXT >> 6];
				i++;
				break;
			case 3: out[o] = table[in[i] & 0x3f];
				i++;
				break;
		}
	}

	if (o < outlen) out[o] = 0;
	return o;
}

static int syscom_gw_genname(struct syscom_gw *gw, const char *parent_name,
		const struct sockaddr *addr)
{
	struct sockaddr_in6 addrbuf = { .sin6_family = -1 };
	int (*getname)(struct socket *, struct sockaddr *);
	const char *pref, *delim;
	struct {
		__be32 addr;
		__be16 port;
	} __attribute__((__packed__)) aname;
	char name[10];
	int err = 0;

	lockdep_assert_held(&gateways_lock);

	if (parent_name) {
		static const char delim_str[] = {SYSCOM_GW_DELIM, 0};
		if (addr) {
			getname = NULL;
		} else {
			getname = kernel_getpeername;
			addr = (struct sockaddr *)&addrbuf;
		}
		pref = parent_name;
		delim = delim_str;
	} else {
		getname = kernel_getsockname;
		pref = delim = "";
		addr = (struct sockaddr *)&addrbuf;
	}

	BUG_ON(getname && !gw->sock);

	// Provide nice address based names if possible
	if (getname) {
		err = getname(gw->sock, (struct sockaddr *)&addrbuf);
	}
	if (err >= 0) {
		if (addr->sa_family == AF_INET) {
			struct sockaddr_in *a = (struct sockaddr_in *)addr;
			aname.addr = a->sin_addr.s_addr;
			aname.port = a->sin_port;
		} else if (addr->sa_family == AF_INET6) {
			struct sockaddr_in6 *a = (struct sockaddr_in6 *)addr;
			aname.addr = a->sin6_addr.s6_addr32[3];
			aname.port = a->sin6_port;
		} else {
			memcpy(&aname, (char*)addr + sizeof addr->sa_family,
					sizeof aname);
		}
		base64(name, sizeof name, &aname, sizeof aname);
		snprintf(gw->name, sizeof gw->name, "%s%s_%s", pref, delim, name);
	}

	if ((err < 0) || syscom_gw_lookup(gw->name)) {
		// We expect the number of sockets fit into int
		u32 ino = gw->sock ? sock_i_ino(gw->sock->sk) : (long)gw;

		base64(name, sizeof name, &ino, sizeof ino);
		snprintf(gw->name, sizeof gw->name, "%s%s_%s", pref, delim, name);
		if (syscom_gw_lookup(gw->name)) {
			return -EEXIST;
		}
	}

	return 0;
}

/** Validate gateway name */
static bool syscom_gw_is_name_valid(const char *name, bool child_name)
{
	const char *tmp;
	int delim, i;

	if (!isalpha(name[0])) {
		return false;
	}

	tmp = strnchr(name, SYSCOM_GW_BASENAME_MAX, SYSCOM_GW_DELIM);
	if (tmp) {
		if (!child_name) {
			return false;
		}
		delim = tmp - name;
	} else {
		if (child_name) {
			return false;
		}
		delim = strnlen(name, SYSCOM_GW_BASENAME_MAX);
		if (delim == SYSCOM_GW_BASENAME_MAX) {
			return false;
		}
	}

	for (i = 1; i < delim; i++) {
		if (!isalnum(name[i]) && name[i] != '_') {
			return false;
		}
	}

	if (!child_name) {
		return true;
	}

	if (!isalpha(name[delim + 1])) {
		return false;
	}
	for (i = 2; i <= SYSCOM_GW_NAME_MAX - SYSCOM_GW_BASENAME_MAX; i++) {
		if (name[delim + i] == '\0') {
			return true;
		}
		if (!isalnum(name[delim + i]) && name[delim + i] != '_') {
			return false;
		}
	}

	return false;
}


static void syscom_gw_worker(struct work_struct *work)
{
	struct syscom_gw *gw = container_of(
			to_delayed_work(work), typeof(*gw), work);
	int rtn;

	trace_syscom_gw_worker_start(gw);
	rtn = gw->ops->ready(gw);

	if (unlikely(rtn == -EAGAIN) && !syscom_gw_test_bit(gw, STOP_WORK)) {
		atomic_inc(&gw->ready_would_block);
		if (!queue_delayed_work(syscom_wq, &gw->work, HZ)) {
			goto put;
		}
		trace_syscom_gw_worker_done(gw, rtn);
		return;
	} else {
		if (unlikely(rtn == -ENOTCONN)) {
			rtn = syscom_gw_del(gw->name, 0, 1, rtn, 1, 0);
			// We may still be scheduled again while
			// we delete ourself, thus -ENOENT is valid
			BUG_ON(rtn != 0 && rtn != -ENOENT);
		}
	}

put:	trace_syscom_gw_worker_done(gw, rtn);
	syscom_gw_put(gw);
}

static inline void syscom_gw_stop_work(struct syscom_gw *gw)
{
	set_bit(SYSCOM_GW_STOP_WORK_BIT, &gw->flags);
	smp_mb__after_atomic();
}

int syscom_gw_send(struct syscom_gw *gw, struct iov_iter *iov, long timeo)
{
	int rtn;

	trace_syscom_gw_send_start(gw, iov, timeo);
	if (unlikely(syscom_gw_test_bit(gw, CONNECTING))) {
		unsigned long slept, start = jiffies;

		rtn = wait_on_bit_timeout(&gw->flags,
				SYSCOM_GW_CONNECTING_BIT,
				TASK_INTERRUPTIBLE, timeo);
		if (rtn) {
			goto err;
		}

		if (timeo != MAX_SCHEDULE_TIMEOUT) {
			slept = jiffies - start;
			if (slept > timeo) {
				rtn = -EAGAIN;
				goto err;
			}
			timeo -= slept;
			// TODO: Add the fast-path here:
			// Check if the connection was established
			// and continue
		}
		rtn = -EUNATCH;
		goto err;
	}

	if (syscom_gw_test_bit(gw, NOHDR)) {
		if (iov_iter_count(iov) < sizeof(struct syscom_hdr)) {
			rtn = -EBADMSG;
			goto out;
		}
		iov_iter_advance(iov, sizeof(struct syscom_hdr));
	}

	if (timeo == MAX_SCHEDULE_TIMEOUT) {
		rtn = gw->ops->send(gw, iov, false);
	} else while (1) {
		int init_cond = atomic_read(&gw->poll_wq_cond);
		rtn = gw->ops->send(gw, iov, true);
		if (rtn != -EAGAIN) {
			break;
		}

		timeo = wait_event_interruptible_timeout(gw->poll_wq,
				init_cond != atomic_read(&gw->poll_wq_cond),
				timeo);
		if (timeo <= 0) {
			rtn = timeo ?: -EAGAIN;
			break;
		}
	}

out:	if (rtn >= 0) {
		atomic_long_inc(&gw->send);
		rtn = 0;
	} else switch (rtn) {
		case -ERESTARTSYS:
			// TODO: Provide a restart block, which would handle
			// the timeout properly. ERESTARTSYS can be returned
			// by wait_event_interruptible_timeout above.
		case -EAGAIN:
		case -EINTR:
			break;
		case -ESRCH:
			// SCTP returns this on 4.19 kernel, which may be a bug.
			// This needs to be checked with upstream.
		case -ENOTCONN:
		case -EPIPE:
		case -ETIMEDOUT:
			// Connection died while sending, detach the route and
			// try sending again to honor the connection failure
			// behavior configured on the route. The actual gateway
			// will be cleaned up from its ready callback.
			syscom_gw_lock();
			syscom_route_gw_remove_gw(gw);
			syscom_gw_unlock();
			rtn = -EUNATCH;
			__attribute__((fallthrough)); 
		default:
			atomic_inc(&gw->err);
	}

err:	trace_syscom_gw_send_done(gw, rtn);
	return rtn;
}

static struct syscom_gw *syscom_gw_child_next(struct syscom_gw *self)
{
	struct syscom_gw *next;
	char *delim;

	if (self->gateways_list.next == &gateways) {
		return NULL;
	}

	next = list_next_entry(self, gateways_list);
	delim = strchr(self->name, SYSCOM_GW_DELIM);
	if (delim && !strncmp(self->name, next->name, delim - self->name + 1)) {
		return next;
	} else if (!delim &&
			!strncmp(self->name, next->name, strlen(self->name)) &&
			next->name[strlen(self->name)] == SYSCOM_GW_DELIM) {
		return next;
	}
	return NULL;
}

int syscom_gw_del(const char *name, bool del_children, bool del_routes,
		int reason, bool nonblock, unsigned long timeo)
{
	DECLARE_COMPLETION_ONSTACK(removed);
	struct syscom_gw *gw, *iter;
	int cnt = 0, rtn = 0;

	syscom_gw_lock();
	iter = gw = syscom_gw_lookup(name);
	if (!gw) {
		rtn = -ENOENT;
		goto err;
	}
	if (!del_routes) do {
		if (!list_empty(&iter->routes)) {
			rtn = -EBUSY;
			goto err;
		}
	} while (del_children && (iter = syscom_gw_child_next(iter)));
	if (!nonblock && gw->ops->destroy_sync_safe) {
		rtn = gw->ops->destroy_sync_safe(gw, del_children);
		if (rtn) {
			goto err;
		}
	}
	trace_syscom_gw_del(gw, del_children, del_routes, reason);
	do {
		iter = syscom_gw_child_next(gw);
		syscom_route_gw_remove_gw(gw);
		list_del(&gw->gateways_list);
		gw->reason = reason;
		syscom_gw_stop_work(gw);
		if (!nonblock) {
			gw->completion = &removed;
			cnt++;
		}
		reason = -ECHILD;
		smp_wmb();
		syscom_gw_put(gw);
	} while (del_children && (gw = iter));

err:	syscom_gw_unlock();

	while (cnt-- > 0) {
		timeo = wait_for_completion_timeout(&removed, timeo);
		if (!timeo) return -ETIMEDOUT;
	}

	return rtn;
}

static int __syscom_gw_bind(struct syscom_gw *gw, struct sockaddr *addrs, int addrcnt, int addrsize)
{
	int rtn;

	if (addrcnt < 1) {
		return -EINVAL;
	}

	if (addrcnt == 1) {
		rtn = kernel_bind(gw->sock, addrs, addrsize);
	} else if (gw->sock->sk->sk_protocol == IPPROTO_SCTP) {
		rtn = kernel_setsockopt(gw->sock, SOL_SCTP,
				SCTP_SOCKOPT_BINDX_ADD, (char*)addrs, addrsize);
	} else {
		rtn = -EINVAL;
	}

	return rtn;
}

static int syscom_gw_getname(struct syscom_gw *gw, bool local, int assoc,
		void **addr, int *addrcnt, int *addrsize)
{
	int as, rtn;

	*addr = kzalloc(8 * sizeof (struct sockaddr_in6), GFP_KERNEL);
	if (!*addr) {
		return -ENOMEM;
	}
	as = ksize(*addr);

	if (gw->sock->sk->sk_protocol == IPPROTO_SCTP) {
		struct sctp_getaddrs *sga = *addr;
		sga->assoc_id = assoc;
		rtn = kernel_getsockopt(gw->sock, SOL_SCTP,
				local ? SCTP_GET_LOCAL_ADDRS : SCTP_GET_PEER_ADDRS,
				*addr, &as);
		if (!rtn) {
			*addrcnt = sga->addr_num;
			as -= offsetof(struct sctp_getaddrs, addrs);
			memmove(*addr, sga->addrs, as);
		}
	} else if (gw->sock->sk->sk_protocol == IPPROTO_UDP && !local) {
		BUG_ON(as < gw->dest_addrlen);
		memcpy(*addr, &gw->dest_addr, gw->dest_addrlen);
		as = gw->dest_addrlen;
		*addrcnt = 1;
		rtn = 0;
	} else {
		int (*getname)(struct socket *, struct sockaddr *);
		getname = local ? kernel_getsockname : kernel_getpeername;
		rtn = getname(gw->sock, *addr);
		if (rtn >= 0) {
			as = rtn;
			rtn = 0;
		}
		if (rtn >= 0)
			*addrcnt = 1;
	}

	if (rtn < 0) {
		kfree(*addr);
		*addr = NULL;
		*addrsize = 0;
	} else {
		*addrsize = as;
	}

	return rtn;
}

static void syscom_gw_notify_bind(struct syscom_gw *gw)
{
	int addrcnt, addrsize;
	void *addr;

	if (syscom_gw_getname(gw, 1, 0, &addr, &addrcnt, &addrsize)) {
		atomic_inc(&syscom_stats.notify_errors);
	} else {
		syscom_notify_gw_bind(gw->name, addr, addrcnt, addrsize);
		kfree(addr);
	}
}

static void syscom_gw_notify_connect(struct syscom_gw *gw, bool accepted)
{
	int addrcnt, addrsize;
	void *addr;


	if (syscom_gw_getname(gw, 0, 0, &addr, &addrcnt, &addrsize)) {
		atomic_inc(&syscom_stats.notify_errors);
	} else {
		syscom_notify_gw_connect(gw->name, accepted, addr, addrcnt, addrsize);
		kfree(addr);
	}
}

static struct net *get_net_by_id(int id)
{
	unsigned long flags;
	struct net *peer;

	if (id < 0)
		return NULL;

	rcu_read_lock();
	spin_lock_irqsave(&init_net.nsid_lock, flags);
	peer = idr_find(&init_net.netns_ids, id);
	if (peer)
		get_net(peer);
	spin_unlock_irqrestore(&init_net.nsid_lock, flags);
	rcu_read_unlock();

	return peer;
}

static struct syscom_gw *syscom_gw_alloc(void)
{
	struct syscom_gw *gw;

	gw = kzalloc(sizeof *gw, GFP_KERNEL);
	if (gw) {
		kref_init(&gw->kref);
		INIT_LIST_HEAD(&gw->routes);
		init_waitqueue_head(&gw->poll_wq);
	}

	return gw;
}


int syscom_gw_add(const char *basename, char *genname,
		int netid, int domain, int type, int protocol, int flags,
		struct sockaddr *addrs, int addrcnt, int addrsize)
{
	struct syscom_gw *gw;
	struct net *net;
	int rtn;

	// Validate basename
	if (basename[0] && !syscom_gw_is_name_valid(basename, false)) {
		return -EINVAL;
	}

	// Alloc
	gw = syscom_gw_alloc();
	if (!gw) {
		return -ENOMEM;
	}

	net = netid == -1 ? get_net(&init_net) : get_net_by_id(netid);
	if (!net) {
		rtn = -EINVAL;
		goto err0;
	}
	rtn = sock_create_kern(net, domain, type, protocol, &gw->sock);
	put_net(net);
	if (rtn) {
		goto err0;
	}

	gw->sock->sk->sk_rcvbuf = gw->sock->sk->sk_sndbuf = SYSCOM_GW_BUFSIZE;
	if (type == SOCK_STREAM) {
		gw->ops = &syscom_gw_stream_base_ops;
	} else if (type == SOCK_SEQPACKET && protocol == IPPROTO_SCTP) {
		gw->ops = &syscom_gw_sctp_seqpacket_base_ops;
	} else {
		gw->ops = &syscom_gw_dgram_base_ops;
	}

	if (flags & SYSCOM_SERVICE_MSG_CONF_OP_GW_ADD_NOHDR) {
		syscom_gw_set_bit(gw, NOHDR);
	}
	if (flags & SYSCOM_SERVICE_MSG_CONF_OP_GW_ADD_M2M_EMULATION) {
		syscom_gw_set_bit(gw, M2M);
	}

	rtn = gw->ops->init(gw, gw->sock, NULL);
	if (rtn) {
		goto err1;
	}


	// Bind
	if (addrcnt) {
		rtn = __syscom_gw_bind(gw, addrs, addrcnt, addrsize);
		if (rtn) {
			goto err2;
		}
	}

	syscom_gw_lock();
	if (basename[0]) {
		strbcpy(gw->name, basename, sizeof gw->name);
		rtn = syscom_gw_lookup(gw->name) ? -EEXIST : 0;
	} else {
		rtn = syscom_gw_genname(gw, NULL, NULL);
	}
	if (rtn) {
		goto err3;
	}

	list_add(&gw->gateways_list, &gateways);
	trace_syscom_gw_create(gw);
	strcpy(genname, gw->name);

	syscom_route_gw_introduce_gw(gw);
	syscom_notify_gw_add(gw->name, 0);
	if (addrcnt) {
		syscom_gw_notify_bind(gw);
	}

	syscom_gw_unlock();

	return 0;

err3:	syscom_gw_unlock();
err2:	syscom_gw_stop_work(gw);
	syscom_gw_put(gw);
	return rtn;

err1:	sock_release(gw->sock);
err0:	kfree(gw);
	return rtn;
}

int syscom_gw_rename(const char *old_name, const char *new_name)
{
	struct syscom_gw *gw;
	const char *delim;
	int rtn;

	delim = strchr(old_name, SYSCOM_GW_DELIM);
	if (delim) {
		if(strncmp(old_name, new_name, delim - old_name + 1)) {
			return -EINVAL;
		}
	}
	if (!syscom_gw_is_name_valid(new_name, delim)) {
		return -EINVAL;
	}

	syscom_gw_lock();
	if (syscom_gw_lookup(new_name)) {
		rtn = -EEXIST;
		goto err0;
	}
	gw = syscom_gw_lookup(old_name);
	if (!gw) {
		rtn = -ENOENT;
		goto err0;
	}

	syscom_route_gw_remove_gw(gw);
	strbcpy(gw->name, new_name, sizeof gw->name);

	syscom_route_gw_introduce_gw(gw);
	syscom_notify_gw_rename(old_name, new_name);

	syscom_gw_unlock();

	return 0;

err0:	syscom_gw_unlock();
	return rtn;
}

struct syscom_poll_table {
	poll_table pt;
	struct syscom_gw *gw;
};

static int syscom_gw_poll_ready(wait_queue_entry_t *wq_entry, unsigned mode, int flags, void *key)
{
	struct syscom_gw *gw = wq_entry->private;

	trace_syscom_gw_ready(gw);
	atomic_inc(&gw->poll_wq_cond);
	wake_up(&gw->poll_wq);

	if (gw->ops->ready && syscom_gw_get(gw)) {
		if (!queue_delayed_work(syscom_wq, &gw->work, 0)) {
			syscom_gw_put(gw);
			return 1;
		}
		return 0;
	}

	return 1;
}

static void syscom_gw_poll_queue(struct file *file, wait_queue_head_t *head,
		struct poll_table_struct *pt)
{
	struct syscom_poll_table *syscom_pt = container_of(pt,
			typeof(*syscom_pt), pt);
	struct syscom_gw *gw = syscom_pt->gw;
	wait_queue_entry_t *wq_entry;

	BUG_ON(gw->wait_ready_cnt >= ARRAY_SIZE(gw->wait_ready));
	gw->wait_ready[gw->wait_ready_cnt].wait_address = head;
	wq_entry = &gw->wait_ready[gw->wait_ready_cnt++].wait;

	init_waitqueue_func_entry(wq_entry, syscom_gw_poll_ready);
	// We do not take a reference here, because we have to avoid circular
	// references. We unregister waiters before destroying the GW
	wq_entry->private = gw;

	add_wait_queue(head, wq_entry);
}

static void syscom_gw_register_poller(struct syscom_gw *gw)
{
	struct syscom_poll_table syscom_pt = { .gw = gw };
	__poll_t mask;

	if (!gw->ops->ready && !gw->ops->send) {
		// It doesn't make sense to poll the socket, if
		// there aren't callbacks defined.
		return;
	}

	INIT_DELAYED_WORK(&gw->work, syscom_gw_worker);
	init_poll_funcptr(&syscom_pt.pt, syscom_gw_poll_queue);
	mask = gw->sock->ops->poll(gw->sock->file, gw->sock, &syscom_pt.pt);

	if (mask) {
		atomic_inc(&gw->poll_wq_cond);
		wake_up(&gw->poll_wq);
		if (gw->ops->ready) {
			syscom_gw_get(gw); // Delayed work reference
			if (!queue_delayed_work(syscom_wq, &gw->work, 0)) {
				syscom_gw_put(gw);
			}
		}
	}
}

static void syscom_gw_unregister_poller(struct syscom_gw *gw)
{
	int i;
	for (i = 0; i < gw->wait_ready_cnt; i++) {
		remove_wait_queue(gw->wait_ready[i].wait_address,
				&gw->wait_ready[i].wait);
	}
	gw->wait_ready_cnt = 0;
}

int syscom_gw_listen(const char *name, int backlog)
{
	struct syscom_gw *gw;
	int rtn;


	if (backlog < 1) {
		return -EINVAL;
	}

	gw = syscom_gw_lookup_get(name);
	if (!gw) {
		return -ENOENT;
	}

	rtn = gw->ops->listen(gw, backlog);

	if (rtn == 0 && !syscom_gw_test_bit(gw, LISTENING)) {
		syscom_gw_set_bit(gw, LISTENING);
		syscom_gw_register_poller(gw);
	}

	syscom_gw_put(gw);
	return rtn;
}

int syscom_gw_bind(const char *name, struct sockaddr *addrs,
		int addrcnt, int addrsize)
{
	struct syscom_gw *gw;
	int rtn;

	gw = syscom_gw_lookup_get(name);
	if (!gw) {
		return -ENOENT;
	}

	rtn = __syscom_gw_bind(gw, addrs, addrcnt, addrsize);
	if (!rtn) {
		syscom_gw_notify_bind(gw);
	}

	syscom_gw_put(gw);
	return rtn;
}

struct syscom_gw_connect_arg_s {
	struct work_struct work;
	/** Connection timeout */
	unsigned long timeo;
	struct sockaddr *addrs;
	int addrcnt;
	int addrsize;
	/** Return value for sync operation */
	int rtn;
	bool kfree;
	struct syscom_gw *gw, *child_gw;
};

struct syscom_gw_worker_timer {
	struct timer_list timer;
	struct task_struct *task;
};

/** Terminate connect operation */
static void syscom_gw_connect_timeout(struct timer_list *t)
{
	struct syscom_gw_worker_timer *sgw_timer = from_timer(sgw_timer, t, timer);

	send_sig(SIGUSR1, sgw_timer->task, 0);
	trace_syscom_gw_connect_timeout(sgw_timer->task);
}

static void syscom_gw_connect_worker(struct work_struct *work)
{
	struct syscom_gw_connect_arg_s *arg =
			container_of(work, typeof(*arg), work);
	struct syscom_gw *gw = arg->gw, *child_gw = arg->child_gw;
	struct syscom_gw_worker_timer sgw_timer;
	int rtn;

	trace_syscom_gw_connect_start(child_gw, arg->timeo);
	if (arg->timeo != MAX_SCHEDULE_TIMEOUT) {
		sgw_timer.task = current;
		timer_setup_on_stack(&sgw_timer.timer,
				     syscom_gw_connect_timeout, 0);
		sgw_timer.timer.expires = jiffies + arg->timeo;
		allow_signal(SIGUSR1);
		add_timer(&sgw_timer.timer);
	}

	rtn = gw->ops->connect(gw, child_gw,
		arg->addrs, arg->addrcnt, arg->addrsize);

	if (arg->timeo != MAX_SCHEDULE_TIMEOUT) {
		disallow_signal(SIGUSR1);
		del_timer_sync(&sgw_timer.timer);
		flush_signals(current);
		destroy_timer_on_stack(&sgw_timer.timer);
	}

	if (arg->kfree) {
		kfree(arg);
	} else {
		if (rtn == -EINTR || rtn == -ERESTARTSYS) {
			arg->rtn = -ETIMEDOUT;
		} else {
			arg->rtn = rtn;
		}
	}

	if (rtn) {
		syscom_gw_del(child_gw->name, 0, 1, rtn, 1, 0);
	} else {
		syscom_gw_notify_connect(child_gw, 0);
	}

	clear_bit(SYSCOM_GW_CONNECTING_BIT, &child_gw->flags);
	smp_mb__after_atomic();
	wake_up_bit(&child_gw->flags, SYSCOM_GW_CONNECTING_BIT);

	trace_syscom_gw_connect_done(child_gw, rtn);
	syscom_gw_put(gw);
	syscom_gw_put(child_gw);
}

int syscom_gw_connect(const char *name, const char *childname, char *genname,
		struct sockaddr *addrs, int addrcnt, int addrsize,
		bool nonblock, unsigned long timeo)
{
	struct syscom_gw_connect_arg_s *connect_ptr, connect_buf;
	struct syscom_gw *gw;

	if (childname[0] && !syscom_gw_is_name_valid(childname, true)) {
		return -EINVAL;
	}

	if (addrcnt < 1) {
		return -EINVAL;
	}

	gw = syscom_gw_lookup_get(name);
	if (!gw) {
		return -ENOENT;
	}

	if (nonblock) {
		connect_ptr = kmalloc(sizeof *connect_ptr + addrsize + 7,
				GFP_KERNEL);
		if (!connect_ptr) {
			return -ENOMEM;
		}
		connect_ptr->addrs = (void*)PTR_ALIGN(&connect_ptr[1], 8);
		memcpy(connect_ptr->addrs, addrs, addrsize);
		INIT_WORK(&connect_ptr->work, syscom_gw_connect_worker);
	} else {
		connect_ptr = &connect_buf;
		connect_ptr->addrs = addrs;
		INIT_WORK_ONSTACK(&connect_ptr->work, syscom_gw_connect_worker);
	}
	connect_ptr->addrcnt = addrcnt;
	connect_ptr->addrsize = addrsize;
	connect_ptr->kfree = nonblock;
	connect_ptr->timeo = timeo;
	connect_ptr->gw = gw;

	connect_ptr->child_gw = syscom_gw_spawn_child(gw, childname, NULL,
			addrs, gw->ops->connect_ops, connect_ptr);
	if (IS_ERR(connect_ptr->child_gw)) {
		int rtn = PTR_ERR(connect_ptr->child_gw);
		if (nonblock) kfree(connect_ptr);
		return rtn;
	}

	strcpy(genname, connect_ptr->child_gw->name);

	// Worker owns connect_ptr->{gw,child_gw} references
	queue_work(nonblock ? system_long_wq : system_unbound_wq,
			&connect_ptr->work);

	if (!nonblock) {
		flush_work(&connect_ptr->work);
		return connect_ptr->rtn;
	}

	return -EINPROGRESS;
}

int syscom_gw_hdr(const char *name, const struct syscom_hdr *hdr)
{
	struct syscom_hdr *hdrbuf;
	struct syscom_gw *gw;
	int rtn = 0;

	gw = syscom_gw_lookup_get(name);
	if (!gw) {
		return -ENOENT;
	}

	if (!syscom_gw_test_bit(gw, NOHDR)) {
		syscom_gw_put(gw);
		return -ENOPROTOOPT;
	}

	syscom_gw_lock();
	if (gw->hdr) {
		rtn = -EALREADY;
		goto out;
	}

	hdrbuf = kmalloc(sizeof *hdrbuf, GFP_KERNEL);
	if (!hdrbuf) {
		rtn = -ENOMEM;
		goto out;
	}

	memcpy(hdrbuf, hdr, sizeof *hdrbuf);
	smp_store_release(&gw->hdr, hdrbuf);

out:	syscom_gw_unlock();
	syscom_gw_put(gw);
	return rtn;
}

static int syscom_gw_ts_set(struct syscom_gw *gw)
{
	const int rx_opt = SOF_TIMESTAMPING_RX_SOFTWARE |
			SOF_TIMESTAMPING_RX_HARDWARE |
			SOF_TIMESTAMPING_SOFTWARE |
			SOF_TIMESTAMPING_RAW_HARDWARE;
	int ts, rtn;

	// This can be called only if the GW is on gateways_list
	// or is being destroyed

	if (!gw->sock) {
		return 0;
	}

	ts = syscom_gw_rx_timestamping ? rx_opt : 0;
	// ts |= syscom_gw_tx_timestamping ? tx_opt : 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
	rtn = kernel_setsockopt(gw->sock, SOL_SOCKET, SO_TIMESTAMPING,
#else
	rtn = kernel_setsockopt(gw->sock, SOL_SOCKET, SO_TIMESTAMPING_OLD,
#endif
			(char*)&ts, sizeof ts);
	if (rtn) {
		pr_err("Can't enable timestamping on gw '%s': %d\n",
				gw->name, rtn);
	}

	return rtn;
}

int syscom_gw_ts_change(void)
{
	struct syscom_gw *gw;
	int tmp, rtn = 0;

	syscom_gw_lock();
	list_for_each_entry(gw, &gateways, gateways_list) {
		tmp = syscom_gw_ts_set(gw);
		rtn = rtn ?: tmp;
	}
	syscom_gw_unlock();

	return rtn;
}

static void _syscom_gw_attach_socket(struct syscom_gw *gw, struct socket *sock)
{
	BUG_ON(gw->sock);

	gw->sock = sock;
	sock->sk->sk_rcvbuf = sock->sk->sk_sndbuf = SYSCOM_GW_BUFSIZE;
}

static void syscom_gw_attach_socket(struct syscom_gw *gw, struct socket *sock)
{
	_syscom_gw_attach_socket(gw, sock);
	syscom_gw_register_poller(gw);
	syscom_gw_ts_set(gw);
}

int syscom_gw_open_socket(const char *name, int flags)
{
	struct syscom_gw *gw = syscom_gw_lookup_get(name);
	struct file *file;
	int err, fd;

	if (!gw) {
		return -ENOENT;
	}

	if (!gw->sock) {
		err = -ENOTSOCK;
		goto err0;
	}

	if (!gw->sock->file) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
/* FixMe: This cannot work as 'socket_file_ops' is private in all Linux releases */
#if 0
		// sock_alloc_file in 4.15+ kernel releases the socket on
		// failure, which we do not want.
		file = alloc_file_pseudo(SOCK_INODE(gw->sock), sock_mnt, name,
				O_RDWR | (flags & O_NONBLOCK),
				&socket_file_ops);
		if (!IS_ERR(file)) {
			gw->sock->file = file;
			file->private_data = gw->sock;
		}
#endif
		BUG();
#else
		file = sock_alloc_file(gw->sock, 0, name);
#endif
		if (IS_ERR(file)) {
			err = PTR_ERR(file);
			goto err0;
		}
	}
	file = get_file(gw->sock->file);

	fd = get_unused_fd_flags(flags & (O_CLOEXEC | O_NONBLOCK));
	if (fd < 0) {
		err = fd;
		goto err1;
	}

	fd_install(fd, file);
	syscom_gw_put(gw);
	return fd;

err1:	fput(file);
err0:	syscom_gw_put(gw);
	return err;
}

/** Spawn a child GW from the parent. The socket is always consumed. */
static struct syscom_gw *syscom_gw_spawn_child(struct syscom_gw *gw,
		const char *name, struct socket *sock, struct sockaddr *dstaddr,
		const struct syscom_gw_ops_s *ops, void *init_arg)
{
	struct syscom_gw *newgw;
	const char *delim;
	int err = -ENOMEM;

	newgw = syscom_gw_alloc();
	if (!newgw) {
		goto err0;
	}

	newgw->ops = ops;
	if (syscom_gw_test_bit(gw, NOHDR)) {
		syscom_gw_set_bit(newgw, NOHDR);
		if (gw->hdr) {
			newgw->hdr = kmalloc(sizeof *newgw->hdr, GFP_KERNEL);
			if (!newgw->hdr) {
				goto err1;
			}
			memcpy(newgw->hdr, gw->hdr, sizeof *newgw->hdr);
		}
	}

	err = ops->init(newgw, sock, init_arg);
	if (err) {
		goto err2;
	}

	if (sock) {
		_syscom_gw_attach_socket(newgw, sock);
		sock = NULL;
	} else {
		syscom_gw_set_bit(newgw, CONNECTING);
	}

	syscom_gw_lock();
	// The parent could be deleted in the mean time
	if (syscom_gw_test_bit(gw, STOP_WORK)) {
		err = -ECONNABORTED;
		goto err3;
	}

	// Name
	if (name[0]) {
		delim = strchr(name, SYSCOM_GW_DELIM);
		if (strncmp(name, gw->name, delim - name) ||
		    strlen(gw->name) != delim - name) {
			err = -EINVAL;
		} else if (syscom_gw_lookup(name)) {
			err = -EEXIST;
		} else {
			strbcpy(newgw->name, name, sizeof newgw->name);
			err = 0;
		}
	} else {
		err = syscom_gw_genname(newgw, gw->name, dstaddr);
	}
	if (err) {
		goto err3;
	}

	newgw->send_wq = alloc_ordered_workqueue("S-%s", WQ_HIGHPRI,
			newgw->name);
	if (!newgw->send_wq) {
		goto err3;
	}

	atomic_long_inc(dstaddr ? &gw->send : &gw->recv);

	syscom_gw_get(newgw); // List reference
	list_add(&newgw->gateways_list, &gw->gateways_list);
	syscom_gw_ts_set(newgw);
	trace_syscom_gw_create(newgw);

	syscom_route_gw_introduce_gw(newgw);
	syscom_notify_gw_add(newgw->name, !dstaddr);

	if (newgw->sock) {
		syscom_gw_notify_connect(newgw, !dstaddr);
		syscom_gw_register_poller(newgw);
	}
	syscom_gw_unlock();

	return newgw;

err3:	syscom_gw_unlock();
	ops->destroy(newgw);
err2:	if (newgw->hdr) kfree(newgw->hdr);
err1:	kfree(newgw);
err0:	atomic_inc(&gw->err);
	pr_err("Failed to spawn child '%s' on '%s': %d\n", name, gw->name, err);
	if (sock) {
		kernel_sock_shutdown(sock, SHUT_RDWR);
		sock_release(sock);
	}
	return ERR_PTR(err);
}

static void syscom_gw_delivery_start(struct syscom_gw *gw, struct msghdr *msg,
		const struct syscom_hdr *hdr)
{
	struct scm_timestamping *scmts = NULL;
	struct cmsghdr *cmsg;

	for (cmsg = CMSG_FIRSTHDR(msg); cmsg;
			cmsg = CMSG_NXTHDR(msg, cmsg)) {
		if (unlikely(!CMSG_OK(msg, cmsg))) {
			pr_err("Bogus cmsg\n");
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
		if (cmsg->cmsg_level == SOL_SOCKET &&
		    cmsg->cmsg_type == SCM_TIMESTAMPING) {
			scmts = CMSG_DATA(cmsg);
		}
#else
		if (cmsg->cmsg_level == SOL_SOCKET &&
		    cmsg->cmsg_type == SO_TIMESTAMPING_OLD)
			scmts = CMSG_DATA(cmsg);
#endif
	}

	gw->delivery_start = ktime_get();
	if (scmts) {
		if (scmts->ts[0].tv_sec || scmts->ts[0].tv_nsec) {
			syscom_latency_update(&gw->rx_sw_us,
					ktime_mono_to_real(gw->delivery_start),
					timespec_to_ktime(scmts->ts[0]),
					trace_syscom_gw_latency_grow,
					gw, SYSCOM_GW_LATENCY_RX_SW, hdr);
		}
	}
}

static void syscom_gw_delivery_complete(struct syscom_gw *gw,
		const struct syscom_hdr *hdr)
{
	syscom_latency_update(&gw->rx_dl_us, gw->delivery_start, ktime_get(),
			trace_syscom_gw_latency_grow, gw,
			SYSCOM_GW_LATENCY_RX_DL, hdr);
}

static void syscom_gw_destroy_sock(struct syscom_gw *gw)
{
	syscom_gw_unregister_poller(gw);

	if (gw->sock) {
		kernel_sock_shutdown(gw->sock, SHUT_RDWR);
		if (gw->sock->file) {
			fput(gw->sock->file);
		} else {
			sock_release(gw->sock);
		}
	}
	if (gw->m2m_sock) {
		sock_release(gw->m2m_sock);
	}
}

static void syscom_gw_destroy_rcvbuf(struct syscom_gw *gw)
{
	syscom_gw_unregister_poller(gw);
	vfree(gw->vec.iov_base);
}

static void syscom_gw_destroy_sock_rcvbuf(struct syscom_gw *gw)
{
	syscom_gw_destroy_sock(gw);
	syscom_gw_destroy_rcvbuf(gw);
}

static int syscom_gw_setopt(struct socket *sock)
{
	const struct { int opt; void *arg; int len; } opts[] = {
#define SOCK_OPT_INT(OPT, VAL) { SO_##OPT, (int[]){VAL}, sizeof(int) }
		SOCK_OPT_INT(PRIORITY, 3),
	};
	int i, rtn;

	for (i = 0; i < ARRAY_SIZE(opts); i++) {
		rtn = kernel_setsockopt(sock, SOL_SOCKET, opts[i].opt,
				opts[i].arg, opts[i].len);
		if (rtn) {
			pr_err("Failed to set socket option %d: %d\n",
					opts[i].opt, rtn);
			return rtn;
		}
	}

	return 0;
}

static int syscom_gw_basic_init(struct syscom_gw *gw,
		struct socket *sock, void *arg)
{
	if (sock)
		return syscom_gw_setopt(sock);
	return 0;
}

static int syscom_gw_init_rcvbuf(struct syscom_gw *gw,
		struct socket *sock, void *arg)
{
	syscom_gw_basic_init(gw, sock, arg);
	gw->vec.iov_len = SYSCOM_MTU;
	gw->vec.iov_base = vmalloc(SYSCOM_MTU);
	if (!gw->vec.iov_base) {
		return -ENOMEM;
	}

	return 0;
}

static int syscom_gw_sctp_setopt(struct socket *sock, bool init)
{
	const struct { int init; int opt; void *arg; int len; } opts[] = {
#define SCTP_OPT(I, OPT, ARG, ...) { I, SCTP_##OPT, &(struct sctp_##ARG)\
		{ __VA_ARGS__ }, sizeof(struct sctp_##ARG) }
#define SCTP_OPT_INT(I, OPT, VAL) { I, SCTP_##OPT, (int[]){VAL}, sizeof(int) }
		SCTP_OPT(1, INITMSG, initmsg,
			.sinit_num_ostreams = SYSCOM_SCTP_STREAM_NUM,
			.sinit_max_instreams = 65535,
			.sinit_max_attempts = 8,
			.sinit_max_init_timeo = 10000),
		SCTP_OPT(0, EVENTS, event_subscribe,
			.sctp_association_event = 1,
			.sctp_shutdown_event = 1),
		SCTP_OPT(0, RTOINFO, rtoinfo,
			.srto_initial = 50,
			.srto_max = 250,
			.srto_min = 50),
		SCTP_OPT(0, ASSOCINFO, assocparams,
			.sasoc_asocmaxrxt = 5),
		SCTP_OPT(0, MAXSEG, assoc_value,
			.assoc_value = 0),
		SCTP_OPT(0, PEER_ADDR_PARAMS, paddrparams,
			.spp_hbinterval = 3500,
			.spp_pathmaxrxt = 5,
			.spp_sackdelay = 5,
			.spp_flags = SPP_HB_ENABLE | SPP_SACKDELAY_ENABLE),
		SCTP_OPT_INT(0, NODELAY, 1),
		SCTP_OPT_INT(0, PARTIAL_DELIVERY_POINT, SYSCOM_MAX_MSGSIZE) };
	int rtn, i;

	for (i = 0; i < ARRAY_SIZE(opts); i++) {
		if (!init && opts[i].init) {
			// Some options must be set before a connection
			// attempt and then they are inherited
			continue;
		}

		rtn = kernel_setsockopt(sock, SOL_SCTP, opts[i].opt,
				opts[i].arg, opts[i].len);
		if (rtn) {
			pr_err("Failed to set SCTP option %d: %d\n",
					opts[i].opt, rtn);
			return rtn;
		}
	}

	return init ? 0 : syscom_gw_setopt(sock);
}

static struct socket *syscom_gw_sctp_peeloff(struct syscom_gw *gw, int assoc)
{
	struct socket *sock;
	int rtn;

	// This lock can be removed once SCTP stack is fixed
	lock_sock(gw->sock->sk);
	rtn = sctp_do_peeloff(gw->sock->sk, assoc, &sock);
	release_sock(gw->sock->sk);
	if (rtn) {
		// We can observe -EINVAL here for outgoing association, which
		// can be peeled of before we process the notification
		if (rtn == -EINVAL) {
			return ERR_PTR(-EALREADY);
		}
		atomic_inc(&gw->err);
		// TODO: Kill the existing association if needed
		return ERR_PTR(rtn);
	}

	rtn = syscom_gw_sctp_setopt(sock, false);
	if (rtn) {
		goto err;
	}

	return sock;

err:	kernel_sock_shutdown(sock, SHUT_RDWR);
	sock_release(sock);
	return ERR_PTR(rtn);
}

static int syscom_gw_sctp_peeloff_child(struct syscom_gw *gw, int assoc)
{
	struct syscom_gw *child_gw;
	struct socket *sock;

	sock = syscom_gw_sctp_peeloff(gw, assoc);
	if (IS_ERR(sock)) {
		return PTR_ERR(sock);
	}

	child_gw = syscom_gw_spawn_child(gw, "", sock, NULL,
			&syscom_gw_sctp_seqpacket_child_ops, sock);
	if (!IS_ERR(child_gw)) {
		syscom_gw_put(child_gw);
		return 0;
	} else {
		return PTR_ERR(child_gw);
	}
}

struct syscom_gw_sctp_conn_s {
	struct list_head conns_list;
	int addrcnt, addrsize;
	void *addr;
	int assoc;
};

static bool syscom_gw_sctp_conn_match(struct syscom_gw *gw, int assoc,
		struct syscom_gw_sctp_conn_s *conn)
{
	int addrcnt, addrsize;
	struct sockaddr_in *a1_in, *a2_in;
	struct sockaddr_in6 *a1_in6, *a2_in6;
	void *addr, *a1, *a2;
	int i, j;

	if (syscom_gw_getname(gw, 0, assoc, &addr, &addrcnt, &addrsize)) {
		return 0;
	}

	if (conn->addrcnt != addrcnt) goto fail;
	if (conn->addrsize != addrsize) goto fail;

	for (i = 0, a1 = addr; i < addrcnt; i++) {
		a1_in = a1; a1_in6 = a1;
		for (j = 0, a2 = conn->addr; j < addrcnt; j++) {
			a2_in = a2; a2_in6 = a2;
			if (a1_in->sin_family != a2_in->sin_family) {
				continue;
			}
			if (a1_in->sin_family == AF_INET) {
				if (a1_in->sin_port != a2_in->sin_port ||
				    memcmp(&a1_in->sin_addr, &a2_in->sin_addr,
						sizeof a2_in->sin_addr)) {
					a2 += sizeof a1_in;
					continue;
				}
				a1 += sizeof a1_in;
			} else if (a1_in->sin_family == AF_INET6) {
				if (a1_in6->sin6_port != a2_in6->sin6_port ||
				    memcmp(&a1_in6->sin6_addr, &a2_in6->sin6_addr,
						sizeof a2_in6->sin6_addr)) {
					a2 += sizeof a1_in6;
					continue;
				}
				a1 += sizeof a1_in6;
			} else {
				pr_warn("Matching unsupported family %d",
						a1_in->sin_family);
				return 0;
			}
			break;
		}
		if (j == addrcnt) goto fail;
	}

	kfree(addr);
	return 1;

fail:	kfree(addr);
	return 0;
}


/* STREAM gateway ***********************************************/

static int syscom_gw_stream_accept(struct syscom_gw *gw)
{
	struct syscom_gw *newgw;
	struct socket *newsock;
	int err;

	// It can happen that we are executed falsely for a connected socket
	// if there are data pending, because sk_user_data is copied from
	// the listening socket. This is not a problem, accept will
	// return EAGAIN, but we must make sure data are not left unnoticed
	// on the newly connected socket, thus we schedule receiving on it
	// after it's created.

	while (1) {
		err = kernel_accept(gw->sock, &newsock, O_NONBLOCK);
		if (err) {
			if (err != -EAGAIN) {
				atomic_inc(&gw->err);
				pr_warn("Error accepting a new connection: %d\n", err);
			}
			return 0;
		}

		newgw = syscom_gw_spawn_child(gw, "", newsock, NULL,
				&syscom_gw_stream_child_a_ops, NULL);
		if (!IS_ERR(newgw)) {
			syscom_gw_put(newgw);
		}
	}
}

static inline int syscom_kernel_recvmsg(struct socket *sock, struct msghdr *msg,
		struct kvec *vec)
{
	int controllen = msg->msg_controllen;
	int rtn = kernel_recvmsg(sock, msg, vec, 1, vec->iov_len, MSG_DONTWAIT);
	msg->msg_controllen = controllen - msg->msg_controllen;
	return rtn;
}

static int syscom_gw_stream_recv(struct syscom_gw *gw)
{
	int recv, nohdr = syscom_gw_test_bit(gw, NOHDR);
	struct syscom_hdr *hdr = gw->vec.iov_base;

	if (unlikely(gw->pending_data)) {
		recv = gw->pending_data;
		gw->pending_data = 0;
		goto try_to_send;
	} else {
		recv = 0;
	}

	while (1) {
		char cmsg[SYSCOM_GW_CMSG_SIZE] = { 0 };
		struct msghdr msg = {
			.msg_control = cmsg,
			.msg_controllen = sizeof cmsg };
		struct kvec vec;
		int rtn;

		hdr = gw->vec.iov_base;
		if (nohdr && recv == 0) {
			vec.iov_base = &hdr[1];
			vec.iov_len = gw->vec.iov_len - sizeof *hdr;
		} else {
			vec.iov_base = gw->vec.iov_base + recv;
			vec.iov_len = gw->vec.iov_len - recv;
		}

		rtn = syscom_kernel_recvmsg(gw->sock, &msg, &vec);
		if (rtn <= 0) {
			if (rtn == -EAGAIN) {
				gw->pending_data = recv;
				return 0;
			}
			if (rtn < 0) {
				pr_warn("Error receiving on gw '%s': %d\n", gw->name, recv);
				atomic_inc(&gw->err);
			} else if (recv) {
				pr_warn("Connection closed with %dB pending on gw '%s'\n", recv, gw->name);
				atomic_inc(&gw->err);
			}
			return -ENOTCONN;
		}
		recv += rtn;

		if (nohdr) {
			if (unlikely(!gw->hdr)) {
				pr_warn("Message received in nohdr mode without"
					" header being assigned on %s\n", gw->name);
				atomic_inc(&gw->rx_drop);
				continue;
			}
			memcpy(hdr, gw->hdr, sizeof *hdr);
			recv += sizeof *hdr;
			hdr->length = htons(recv);
		}
		atomic_long_inc(&gw->recv); // FIXME: Count properly (probably at return)
		syscom_gw_delivery_start(gw, &msg, hdr);

try_to_send:	while (recv >= sizeof *hdr && recv >= ntohs(hdr->length)) {
			int msglen = ntohs(hdr->length);

			rtn = syscom_route_deliver_buf(hdr, msglen, HZ/8);
			if (unlikely(rtn < 0)) {
				if (rtn == -EAGAIN) {
					gw->pending_data = recv;
					if (hdr != gw->vec.iov_base) {
						memmove(gw->vec.iov_base, hdr, recv);
					}
					return -EAGAIN;
				} else {
					syscom_delivery_error(rtn, hdr, gw->name);
					atomic_inc(&gw->rx_drop);
				}
			}
			syscom_gw_delivery_complete(gw, hdr);
			hdr = (struct syscom_hdr *)((char*)hdr + msglen);
			recv -= msglen;

			BUG_ON(recv > 0 && nohdr);
		}
		if (recv && hdr != gw->vec.iov_base) {
			memmove(gw->vec.iov_base, hdr, recv);
		}
	}
}

static int syscom_gw_stream_sock_copy(struct socket *orig, struct socket **copy,
		bool clear_port)
{
	int i, as, rtn, addrcnt = 1, one = 1;
	struct sockaddr *addr, *iter;
	struct sock *sk = orig->sk;
	struct socket *new;

	addr = kzalloc(8 * sizeof (struct sockaddr_in6), GFP_KERNEL);
	if (!addr) {
		return -ENOMEM;
	}
	as = ksize(addr);

	if (sk->sk_protocol == IPPROTO_SCTP) {
		struct sctp_getaddrs *sga = (struct sctp_getaddrs *)addr;
		rtn = kernel_getsockopt(orig, SOL_SCTP, SCTP_GET_LOCAL_ADDRS,
				(char*)addr, &as);
		if (!rtn) {
			addrcnt = sga->addr_num;
			as -= offsetof(struct sctp_getaddrs, addrs);
			memmove(addr, sga->addrs, as);
		}
	} else {
		rtn = kernel_getsockname(orig, addr);
		as = rtn;
	}

	if (rtn < 0)
		goto err0;

	if (clear_port) {
		for (i = 0, iter = addr; i < addrcnt; i++) {
			if (iter->sa_family == AF_INET) {
				struct sockaddr_in *a = (struct sockaddr_in*)iter;
				iter = (struct sockaddr*)&a[1];
				a->sin_port = 0;
			} else if (iter->sa_family == AF_INET6) {
				struct sockaddr_in6 *a = (struct sockaddr_in6*)iter;
				iter = (struct sockaddr*)&a[1];
				a->sin6_port = 0;
			} else {
				rtn = -EPROTONOSUPPORT;
				goto err0;
			}
			BUG_ON((char*)iter > (char*)addr + as);
		}
	}

	rtn = sock_create_kern(read_pnet(&sk->sk_net), sk->sk_family,
			SOCK_STREAM, sk->sk_protocol, &new);
	if (rtn) {
		goto err0;
	}

	rtn = kernel_setsockopt(new, SOL_SOCKET, SO_REUSEADDR, (char*)&one,
			sizeof one);
	if (rtn) {
		goto err1;
	}

	if (addrcnt == 1) {
		rtn = kernel_bind(new, addr, as);
	} else if (new->sk->sk_protocol == IPPROTO_SCTP) {
		rtn = kernel_setsockopt(new, SOL_SCTP, SCTP_SOCKOPT_BINDX_ADD,
				(char*)addr, as);
	}
	if (rtn) {
		goto err1;
	}

	kfree(addr);
	*copy = new;
	return 0;

err1:	sock_release(new);
err0:	kfree(addr);
	return rtn;
}

static int _syscom_gw_stream_connect(struct socket *sock,
		struct syscom_gw *child_gw, struct sockaddr *addrs,
		int addrcnt, int addrsize)
{
	int rtn;

	if (addrcnt == 1) {
		rtn = kernel_connect(sock, addrs, addrsize, 0);
	} else if (sock->sk->sk_protocol == IPPROTO_SCTP) {
		rtn = kernel_setsockopt(sock, SOL_SCTP,
				SCTP_SOCKOPT_CONNECTX, (char*)addrs, addrsize);
		if (rtn > 0) {
			// Discard the association number, we do not need it
			// in one-to-one communication.
			rtn = 0;
		}
	} else {
		rtn = -EINVAL;
	}

	if (!rtn) {
		syscom_gw_attach_socket(child_gw, sock);
	}

	return rtn;
}

static int syscom_gw_stream_connect(struct syscom_gw *gw,
		struct syscom_gw *child_gw, struct sockaddr *addrs,
		int addrcnt, int addrsize)
{
	int rtn;

	rtn = _syscom_gw_stream_connect(gw->sock, child_gw, addrs, addrcnt,
			addrsize);

	if (rtn == -EISCONN && syscom_gw_test_bit(gw, LISTENING) &&
			syscom_gw_test_bit(gw, M2M)) {
		if (!gw->m2m_sock) {
			// Fixme serialize
			rtn = syscom_gw_stream_sock_copy(gw->sock, &gw->m2m_sock, 1);
		}
		if (gw->m2m_sock) {
			struct socket *new;
			rtn = syscom_gw_stream_sock_copy(gw->m2m_sock, &new, 0);
			if (!rtn) {
				rtn = _syscom_gw_stream_connect(new, child_gw,
						addrs, addrcnt, addrsize);
				if (rtn) {
					sock_release(new);
				}
			}
		}
	}

	return rtn;
}

static int syscom_gw_stream_destroy_sync_safe(struct syscom_gw *gw,
		bool del_children)
{
	lockdep_assert_held(&gateways_lock);

	if (syscom_gw_test_bit(gw, LISTENING) || del_children) {
		// Listening gateway children do not take parent reference
		// or we will delete them
		return 0;
	} if (!syscom_gw_child_next(gw)) {
		// We do not have children
		return 0;
	}
	return -EDEADLOCK;
}

static int syscom_gw_stream_listen(struct syscom_gw *gw, int backlog)
{
	return kernel_listen(gw->sock, backlog);
}

static int syscom_gw_stream_send(struct syscom_gw *gw, struct iov_iter *iov,
		bool noblock)
{
	struct msghdr msg = { .msg_flags = syscom_gw_send_flags(noblock),
			.msg_iter = *iov };
	size_t size = iov_iter_count(iov);
	int rtn = 0;

	if (noblock) {
		// TODO: Provide proper poll for this case,
		// we poll on the socket currently
		if (0 == mutex_trylock(&gw->lock)) {
			return -EAGAIN;
		}
	} else {
		mutex_lock(&gw->lock);
	}

	if (!gw->sock) {
		return -EUNATCH;
	}

	// TCP stream socket is writable if
	//     sk->sk_wmem_alloc < (sk->sk_sndbuf >> 1)
	// -> at least half of the buffer must be available. To make this
	// algorithm work correctly, this half must be more than the message
	// size, as we need to write the message atomically
	BUG_ON(gw->sock->sk->sk_sndbuf >> 1 < SYSCOM_STREAM_SIZE(SYSCOM_MTU));

	if (noblock && sk_stream_wspace(gw->sock->sk) < SYSCOM_STREAM_SIZE(size)) {
		rtn = -EAGAIN;
	} else {
		rtn = sock_sendmsg(gw->sock, &msg);
		BUG_ON(rtn > 0 && rtn != size);
	}

	mutex_unlock(&gw->lock);

	return rtn;
}

static int syscom_gw_stream_init_child_accept(struct syscom_gw *gw,
		struct socket *sock, void *arg)
{
	mutex_init(&gw->lock);
	return syscom_gw_init_rcvbuf(gw, sock, NULL);
}

static int syscom_gw_stream_init_child_connect(struct syscom_gw *gw,
		struct socket *sock, void *init_arg)
{
	struct syscom_gw_connect_arg_s *connect_arg = init_arg;
	int rtn;

	rtn = syscom_gw_stream_init_child_accept(gw, sock, NULL);
	if (!rtn) {
		gw->parent = syscom_gw_get(connect_arg->gw);
	}

	return rtn;
}

static void syscom_gw_stream_destroy_child_accept(struct syscom_gw *gw)
{
	mutex_destroy(&gw->lock);
	syscom_gw_destroy_sock_rcvbuf(gw);
}

static void syscom_gw_stream_destroy_child_connect(struct syscom_gw *gw)
{
	syscom_gw_put(gw->parent);
	mutex_destroy(&gw->lock);
	syscom_gw_destroy_rcvbuf(gw);
}

static const struct syscom_gw_ops_s syscom_gw_stream_base_ops = {
	.init = syscom_gw_basic_init,
	.ready = syscom_gw_stream_accept,
	.listen = syscom_gw_stream_listen,
	.connect = syscom_gw_stream_connect,
	.send = syscom_gw_no_send,
	.destroy = syscom_gw_destroy_sock,
	.destroy_sync_safe = syscom_gw_stream_destroy_sync_safe,
	.connect_ops = &syscom_gw_stream_child_c_ops,
	.identifier = 'T',
};

static const struct syscom_gw_ops_s syscom_gw_stream_child_a_ops = {
	.init = syscom_gw_stream_init_child_accept,
	.ready = syscom_gw_stream_recv,
	.listen = syscom_gw_no_listen,
	.connect = syscom_gw_no_connect,
	.send = syscom_gw_stream_send,
	.destroy = syscom_gw_stream_destroy_child_accept,
	.identifier = 'a',
};

static const struct syscom_gw_ops_s syscom_gw_stream_child_c_ops = {
	.init = syscom_gw_stream_init_child_connect,
	.ready = syscom_gw_stream_recv,
	.listen = syscom_gw_no_listen,
	.connect = syscom_gw_no_connect,
	.send = syscom_gw_stream_send,
	.destroy = syscom_gw_stream_destroy_child_connect,
	.identifier = 'c',
};

/* DGRAM gateway ************************************************/

static int syscom_gw_dgram_listen(struct syscom_gw *gw, int backlog)
{
	return 0;
}

static int syscom_gw_dgram_init_child(struct syscom_gw *gw,
		struct socket *sock, void *init_arg)
{
	struct syscom_gw_connect_arg_s *connect_arg = init_arg;

	syscom_gw_set_bit(gw, REJECT_ORDERED);
	syscom_gw_set_bit(gw, REJECT_RELIABLE);
	gw->parent = syscom_gw_get(connect_arg->gw); // We reuse the parent GW socket
	memcpy(&gw->dest_addr, connect_arg->addrs, connect_arg->addrsize);
	gw->dest_addrlen = connect_arg->addrsize;

	return 0;
}

static void syscom_gw_dgram_destroy_child(struct syscom_gw *gw)
{
	syscom_gw_unregister_poller(gw);
	syscom_gw_put(gw->parent);
}

static int syscom_gw_dgram_connect(struct syscom_gw *gw,
		struct syscom_gw *child_gw, struct sockaddr *addrs, int addrcnt,
		int addrsize)
{
	if (addrcnt != 1) {
		return -EINVAL;
	}

	if (addrsize > sizeof gw->dest_addr) {
		return -EMSGSIZE;
	}

	syscom_gw_attach_socket(child_gw, gw->sock);

	return 0;
}

static int syscom_gw_dgram_destroy_sync_safe(struct syscom_gw *gw,
		bool del_children)
{
	lockdep_assert_held(&gateways_lock);

	if (del_children || !syscom_gw_child_next(gw)) {
		// We do not have or delete children
		return 0;
	}
	return -EDEADLOCK;
}

static int syscom_gw_dgram_send(struct syscom_gw *gw, struct iov_iter *iov,
		bool noblock)
{
	struct msghdr msg = {
		.msg_name = &gw->dest_addr,
		.msg_namelen = gw->dest_addrlen,
		.msg_flags = syscom_gw_send_flags(noblock),
		.msg_iter = *iov,
	};

	return sock_sendmsg(gw->sock, &msg);
}
static int syscom_gw_recv_notify(struct syscom_gw *gw, void *buf, int size)
{
	union sctp_notification *n = buf;

	if (gw->sock->sk->sk_protocol != IPPROTO_SCTP) {
		return 0;
	}

	if (size < sizeof n->sn_header || size < n->sn_header.sn_length) {
		return 0;
	}

	if (n->sn_header.sn_type == SCTP_SHUTDOWN_EVENT) {
		return -ENOTCONN;
	}

	if (n->sn_header.sn_type != SCTP_ASSOC_CHANGE) {
		return 0;
	}

	if (n->sn_assoc_change.sac_state == SCTP_COMM_UP) {
		struct syscom_gw_sctp_conn_s *conn;

		mutex_lock(&gw->lock);
		list_for_each_entry(conn, &gw->conns, conns_list) {
			if (conn->assoc >= 0) continue;
			if (syscom_gw_sctp_conn_match(gw, n->sn_assoc_change.sac_assoc_id, conn)) {
				conn->assoc = n->sn_assoc_change.sac_assoc_id;
				mutex_unlock(&gw->lock);
				return 0;
			}
		}
		mutex_unlock(&gw->lock);

		return syscom_gw_sctp_peeloff_child(gw,
				n->sn_assoc_change.sac_assoc_id);
	}

	if (n->sn_assoc_change.sac_state == SCTP_COMM_LOST ||
	    n->sn_assoc_change.sac_state == SCTP_SHUTDOWN_COMP) {
		return -ENOTCONN;
	}

	return 0;
}

static int syscom_gw_dgram_recv(struct syscom_gw *gw)
{
	const int nohdr = syscom_gw_test_bit(gw, NOHDR);
	int recv;

	if (unlikely(gw->pending_data)) {
		recv = gw->pending_data;
		gw->pending_data = 0;
		goto try_to_send;
	}

	while (1) {
		char cmsg[SYSCOM_GW_CMSG_SIZE] = { 0 };
		struct kvec vec = gw->vec;
		struct msghdr msg = {
			.msg_control = cmsg,
			.msg_controllen = sizeof cmsg };
		int rtn;

		if (nohdr) {
			vec.iov_base = (char *)vec.iov_base + sizeof *gw->hdr;
			vec.iov_len -= sizeof *gw->hdr;
		}

		recv = syscom_kernel_recvmsg(gw->sock, &msg, &vec);
		if (recv < 0) {
			if (recv == -EAGAIN) {
				return 0;
			}
			pr_warn("Error receiving on a socket: %d\n", recv);
			atomic_inc(&gw->err);
			return recv;
		}
		if (unlikely(msg.msg_flags & MSG_NOTIFICATION)) {
			rtn = syscom_gw_recv_notify(gw, vec.iov_base, recv);
			if (rtn == -ENOTCONN && strchr(gw->name, SYSCOM_GW_DELIM)) {
				return rtn;
			}
			continue;
		}

		if (nohdr) {
			struct syscom_hdr *hdr = gw->vec.iov_base;
			if (unlikely(!gw->hdr)) {
				atomic_inc(&gw->rx_drop);
				continue;
			}
			memcpy(hdr, gw->hdr, sizeof *hdr);
			recv += sizeof *hdr;
			hdr->length = htons(recv);
		}
		atomic_long_inc(&gw->recv);
		syscom_gw_delivery_start(gw, &msg, gw->vec.iov_base);

try_to_send:	rtn = syscom_route_deliver_buf(gw->vec.iov_base, recv, HZ/8);
		if (unlikely(rtn < 0)) {
			if (rtn == -EAGAIN) {
				gw->pending_data = recv;
				return -EAGAIN;
			} else {
				syscom_delivery_error(rtn, gw->vec.iov_base,
						gw->name);
				atomic_inc(&gw->rx_drop);
			}
		}
		syscom_gw_delivery_complete(gw, gw->vec.iov_base);
	}
}

static const struct syscom_gw_ops_s syscom_gw_dgram_base_ops = {
	.init = syscom_gw_init_rcvbuf,
	.ready = syscom_gw_dgram_recv,
	.listen = syscom_gw_dgram_listen,
	.connect = syscom_gw_dgram_connect,
	.send = syscom_gw_no_send,
	.destroy = syscom_gw_destroy_sock_rcvbuf,
	.destroy_sync_safe = syscom_gw_dgram_destroy_sync_safe,
	.connect_ops = &syscom_gw_dgram_child_ops,
	.identifier = 'D',
};

static const struct syscom_gw_ops_s syscom_gw_dgram_child_ops = {
	.init = syscom_gw_dgram_init_child,
	.listen = syscom_gw_no_listen,
	.connect = syscom_gw_no_connect,
	.send = syscom_gw_dgram_send,
	.destroy = syscom_gw_dgram_destroy_child,
	.identifier = 'd',
};

/* SCTP SEQPACKET gateway ***************************************/
static int syscom_gw_sctp_seqpacket_init(struct syscom_gw *gw,
		struct socket *sock, void *arg)
{
	int rtn;

	rtn = syscom_gw_init_rcvbuf(gw, sock, NULL);
	if (rtn) {
		return rtn;
	}

	if (sock) {
		rtn = syscom_gw_sctp_setopt(sock, true);
		if (rtn) {
			goto err;
		}
	}

	INIT_LIST_HEAD(&gw->conns);
	mutex_init(&gw->lock);

	return 0;

err:	syscom_gw_destroy_rcvbuf(gw);
	return rtn;
}

static int syscom_gw_sctp_seqpacket_connect(struct syscom_gw *gw,
		struct syscom_gw *child_gw, struct sockaddr *addrs, int addrcnt,
		int addrsize)
{
	struct syscom_gw_sctp_conn_s conn = {
		.addr = addrs, .addrcnt = addrcnt, .addrsize = addrsize,
		.assoc = -1 };
	struct socket *sock;
	int assoc;

	mutex_lock(&gw->lock);
	list_add(&conn.conns_list, &gw->conns);
	mutex_unlock(&gw->lock);

	assoc = kernel_setsockopt(gw->sock, SOL_SCTP, SCTP_SOCKOPT_CONNECTX,
			(char*)addrs, addrsize);

	if (assoc >= 0) {
		sock = syscom_gw_sctp_peeloff(gw, assoc);
	} else {
		sock = ERR_PTR(assoc);
	}

	mutex_lock(&gw->lock);
	list_del(&conn.conns_list);
	mutex_unlock(&gw->lock);

	if (conn.assoc >= 0) {
		if (assoc >= 0 && conn.assoc != assoc) {
			pr_warn("Unexpected double association on %s\n", gw->name);
			syscom_gw_sctp_peeloff_child(gw, conn.assoc);
		} else if (IS_ERR(sock)) {
			sock = syscom_gw_sctp_peeloff(gw, conn.assoc);
		}
	}

	if (!IS_ERR(sock)) {
		syscom_gw_attach_socket(child_gw, sock);
		return 0;
	}

	return PTR_ERR(sock);
}

static int syscom_gw_sctp_seqpacket_send(struct syscom_gw *gw,
		struct iov_iter *iov, bool noblock)
{
	if (syscom_gw_test_bit(gw, NOHDR)) {
		// We never reorder messages send without syscom header
		struct msghdr msg = { .msg_flags = syscom_gw_send_flags(noblock),
			.msg_iter = *iov };
		return sock_sendmsg(gw->sock, &msg);
	} else {
		struct syscom_hdr hdr;
		struct sctp_sndinfo *sndinfo;
		union {
			char data[CMSG_SPACE(sizeof *sndinfo)];
			struct cmsghdr cmsg;
		} cbuf = {
			.cmsg.cmsg_level = IPPROTO_SCTP,
			.cmsg.cmsg_type = SCTP_SNDINFO,
			.cmsg.cmsg_len = CMSG_LEN(sizeof *sndinfo) };
		struct msghdr msg = {
			.msg_flags = syscom_gw_send_flags(noblock),
			.msg_control = &cbuf, .msg_controllen = sizeof cbuf,
			.msg_iter = *iov };

		if (sizeof hdr != copy_from_iter(&hdr, sizeof hdr, iov)) {
			return -EFAULT;
		}
		iov_iter_revert(iov, sizeof hdr);

		sndinfo = CMSG_DATA(&cbuf.cmsg);
		sndinfo->snd_sid = crc16(0xFFFF, (u8*)&hdr.destination, 8) %
				SYSCOM_SCTP_STREAM_NUM;
		sndinfo->snd_ppid = (__force uint32_t)htonl(ETH_P_SYSCOM);
		if (!hdr.ordered) sndinfo->snd_flags = SCTP_UNORDERED;
		return sock_sendmsg(gw->sock, &msg);
	}
}

static void syscom_gw_sctp_seqpacket_destroy(struct syscom_gw *gw)
{
	mutex_destroy(&gw->lock);
	syscom_gw_destroy_sock_rcvbuf(gw);
}

static const struct syscom_gw_ops_s syscom_gw_sctp_seqpacket_base_ops = {
	.init = syscom_gw_sctp_seqpacket_init,
	.ready = syscom_gw_dgram_recv,
	.listen = syscom_gw_stream_listen,
	.connect = syscom_gw_sctp_seqpacket_connect,
	.send = syscom_gw_no_send,
	.destroy = syscom_gw_sctp_seqpacket_destroy,
	.connect_ops = &syscom_gw_sctp_seqpacket_child_ops,
	.identifier = 'S',
};

static const struct syscom_gw_ops_s syscom_gw_sctp_seqpacket_child_ops = {
	.init = syscom_gw_sctp_seqpacket_init,
	.ready = syscom_gw_dgram_recv,
	.listen = syscom_gw_no_listen,
	.connect = syscom_gw_no_connect,
	.send = syscom_gw_sctp_seqpacket_send,
	.destroy = syscom_gw_sctp_seqpacket_destroy,
	.identifier = 's',
};

int __init syscom_gw_init(void)
{
	syscom_wq = alloc_workqueue("syscom-gw",
			WQ_HIGHPRI | WQ_UNBOUND | WQ_SYSFS, 8);
	return syscom_wq ? 0 : -EIO;
}

void syscom_gw_destroy(void)
{
	destroy_workqueue(syscom_wq);
}

/* Proc FS (content of /proc/net/syscom_gw) *********************/

#ifdef CONFIG_PROC_FS

const struct seq_operations syscom_gw_seq_ops = {
	.start  = syscom_gw_seq_start,
	.next   = syscom_gw_seq_next,
	.stop   = syscom_gw_seq_stop,
	.show   = syscom_gw_seq_show,
};

/** Get the next syscom_gw for the specified iterator */
static struct syscom_gw *gw_next(struct syscom_gw_iter_state *iter)
{
	if (iter->gw) {
		iter->gw = list_next_entry(iter->gw, gateways_list);
	}
	if (&iter->gw->gateways_list == &gateways) {
		iter->gw = NULL;
	}

	return iter->gw;
}

/** Get the first syscom_gw for the specified iterator */
static struct syscom_gw *gw_first(struct syscom_gw_iter_state *iter)
{
	iter->gw = list_first_entry_or_null(&gateways,
			typeof(*iter->gw), gateways_list);
	return iter->gw;
}

/** Get the route record at the specified offset */
static struct syscom_gw *gw_idx(struct seq_file *seq, loff_t pos)
{
	struct syscom_gw_iter_state *iter = seq->private;
	struct syscom_gw *gw;
	loff_t off = 0;

	for (gw = gw_first(iter); gw; gw = gw_next(iter)) {
		if (off == pos)
			return gw;
		++off;
	}

	return NULL;
}

/** Start callback of seq_file to dump the route table  */
void *syscom_gw_seq_start(struct seq_file *seq, loff_t *pos)
{
	syscom_gw_lock();
	return *pos ? gw_idx(seq, *pos - 1) : SEQ_START_TOKEN;
}

/** Next callback of seq_file to dump the route table  */
void *syscom_gw_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct syscom_gw_iter_state *iter = seq->private;

	++*pos;
	return v == SEQ_START_TOKEN ? gw_first(iter) : gw_next(iter);
}

/** Stop callback of seq_file to dump the route table  */
void syscom_gw_seq_stop(struct seq_file *seq, void *v)
{
	syscom_gw_unlock();
}

/** Show callback of seq_file to dump the route table  */
int syscom_gw_seq_show(struct seq_file *seq, void *v)
{
	struct syscom_gw *gw = v;

	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, "name                  pkt_send   pkt_recv tx_drop rx_drop err rdbl rx_hw_us rx_sw_us rx_dl_us flags      inode\n");
	} else {
		seq_printf(seq, "%-19s %10lu %10lu %7u %7u %3u %4u %8lu %8lu %8lu %c%c%c%c%c%c%c %8lx\n",
				gw->name, atomic_long_read(&gw->send),
				atomic_long_read(&gw->recv), atomic_read(&gw->tx_drop),
				atomic_read(&gw->rx_drop), atomic_read(&gw->err),
				atomic_read(&gw->ready_would_block), atomic_long_read(&gw->rx_hw_us),
				atomic_long_read(&gw->rx_sw_us), atomic_long_read(&gw->rx_dl_us),
				gw->ops->identifier, syscom_gw_test_bit(gw, NOHDR) ? (gw->hdr ? 'h' : 'H') : '-',
				syscom_gw_test_bit(gw, LISTENING) ? syscom_gw_test_bit(gw, M2M) ? 'L' : 'l' : '-',
				syscom_gw_test_bit(gw, STOP_WORK) ? 's' : '-',
				gw->sock ? gw->sock->file ? 'F' : 'S' : '-',
				syscom_gw_test_bit(gw, REJECT_RELIABLE) ? 'r' : '-',
				syscom_gw_test_bit(gw, REJECT_ORDERED) ? 'o' : '-',
				gw->sock ? sock_i_ino(gw->sock->sk) : -1);
	}

	return 0;
}

#endif
