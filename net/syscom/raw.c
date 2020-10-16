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

#include <linux/list.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/ktime.h>

#include "af.h"
#include "raw.h"
#include "common.h"
#include "route.h"

/** Raw sockets list */
HLIST_HEAD(raw_sockets);
/** Lock for raw_sockets */
static DEFINE_SPINLOCK(raw_sockets_lock);

#define iter_cond(sk, skb) (sk && sk->sk_bound_dev_if && \
		(!skb->dev || skb->dev->ifindex != sk->sk_bound_dev_if))

static struct sock *syscom_raw_next(struct sock *sk, const struct sk_buff *skb)
{
	do {
		sk = hlist_entry_safe(rcu_dereference_raw(
				hlist_next_rcu(&sk->sk_node)),
				struct sock, sk_node);
	} while (iter_cond(sk, skb));
	return sk;
}

static struct sock *syscom_raw_first(const struct sk_buff *skb)
{
	struct sock *sk;
	sk = hlist_entry_safe(
			rcu_dereference_raw(hlist_first_rcu(&raw_sockets)),
			struct sock, sk_node);
	return iter_cond(sk, skb) ? syscom_raw_next(sk, skb) : sk;
}

/** Queue SKB on raw sockets. */
void syscom_raw_queue_skb(struct sk_buff *skb, gfp_t gfp_mask, int rtn)
{
	struct sock *sk, *next;
	ktime_t time;

	if (rtn == -EINPROGRESS || rtn == -EAGAIN) {
		consume_skb(skb);
		return;
	}

	time = ktime_get();

	rcu_read_lock();
	sk = syscom_raw_first(skb);
	if (!sk) {
		consume_skb(skb);
	}

	for (; sk; sk = next) {
		struct sk_buff *cskb;

		next = syscom_raw_next(sk, skb);

		atomic_long_inc(&syscom_sk(sk)->stat.rx_count);

		// Use GFP_ATOMIC as we are in RCU protected area
		cskb = next ? skb_clone(skb, GFP_ATOMIC) : skb;
		if (likely(cskb)) {
			int err;

			*(ktime_t*)cskb->cb = time;
			err = sock_queue_rcv_skb(sk, cskb);
			if (unlikely(err)) {
				// Do not complain about filtered messages
				if (err != -EPERM) {
					syscom_delivery_error(err, cskb->data,
							"raw");
					atomic_inc(&sk->sk_drops);
				}
				kfree_skb(cskb);
			}
		} else {
			syscom_delivery_error(-ENOMEM, skb->data, "raw clone");
			atomic_inc(&sk->sk_drops);
		}
	}
	rcu_read_unlock();
}

/** Initialize a newly created raw socket. */
void syscom_raw_create(struct syscom_sock *ssk)
{
	spin_lock(&raw_sockets_lock);
	sk_add_node_rcu(&ssk->sk, &raw_sockets);
	spin_unlock(&raw_sockets_lock);
}

/** release callback for SYSCOM raw sockets. */
static int syscom_raw_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	spin_lock(&raw_sockets_lock);
	sk_del_node_init_rcu(sk);
	spin_unlock(&raw_sockets_lock);

	syscom_release(syscom_sk(sk));

	return 0;
}

/** recvmsg callback for SYSCOM raw sockets. */
static int syscom_raw_recvmsg(struct socket *sock,
		struct msghdr *msg, size_t size, int flags)
{
	struct syscom_hdr *hdr;
	struct sk_buff *skb;
	int off = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 2, 0)
	int peeked;
#endif
	int err;
	int len;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 2, 0)
	skb = __skb_recv_datagram(sock->sk, flags, NULL, &peeked, &off, &err);
#else
	skb = __skb_recv_datagram(sock->sk, flags, NULL, &off, &err);
#endif
	if (!skb) {
		trace_syscom_sk_recv_err(syscom_sk(sock->sk), NULL, err);
		return err;
	}

	hdr = (struct syscom_hdr *)skb->data;

	len = ntohs(hdr->length);
	if (size > len)
		size = len;
	else if (size < len)
		msg->msg_flags |= MSG_TRUNC;
	if (size > skb->len)
		size = skb->len;

	err = skb_copy_datagram_msg(skb, 0, msg, size);
	if (likely(!err)) {
		err = syscom_recv_ts(syscom_sk(sock->sk), msg,
				*(ktime_t*)skb->cb);
		if (likely(!err)) err = size;
	}

	if (unlikely(err < 0)) {
		trace_syscom_sk_recv_err(syscom_sk(sock->sk), hdr, err);
	}
	skb_free_datagram_locked(sock->sk, skb);
	return err;
}

/** sendmsg callback for SYSCOM raw sockets. */
static int syscom_raw_sendmsg(struct socket *sock,
		struct msghdr *msg, size_t size)
{
	int rtn = syscom_route_deliver_rawmsg(syscom_sk(sock->sk), msg, size);
	return syscom_send_rtn(syscom_sk(sock->sk), rtn, size);
}

/** poll callback for SYSCOM raw sockets. */
static __poll_t syscom_raw_poll(struct file *file, struct socket *sock,
		poll_table *wait)
{
	struct sock *sk = sock->sk;
	__poll_t mask = 0;

	sock_poll_wait(file, sock, wait);

	if (sk->sk_err || !skb_queue_empty(&sk->sk_error_queue))
		mask |= EPOLLERR;
	if (!skb_queue_empty(&sk->sk_receive_queue))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (sk->sk_shutdown & RCV_SHUTDOWN)
		mask |= EPOLLRDHUP | EPOLLIN | EPOLLRDNORM;
	if (sk->sk_shutdown == SHUTDOWN_MASK)
		mask |= EPOLLHUP;

	return mask;
}

/** Protocol operations of a SYSCOM raw socket. */
const struct proto_ops syscom_raw_ops = {
	.family =       PF_SYSCOM,
	.owner =        THIS_MODULE,
	.release =      syscom_raw_release,
	.bind =         sock_no_bind,
	.connect =      sock_no_connect,
	.socketpair =   sock_no_socketpair,
	.accept =       sock_no_accept,
	.getname =      sock_no_getname,
	.poll =         syscom_raw_poll,
	.ioctl =        syscom_ioctl,
	.listen =       sock_no_listen,
	.shutdown =     sock_no_shutdown,
	.setsockopt =   syscom_setsockopt,
	.getsockopt =   syscom_getsockopt,
	.sendmsg =      syscom_raw_sendmsg,
	.recvmsg =      syscom_raw_recvmsg,
	.mmap =         sock_no_mmap,
	.sendpage =     sock_no_sendpage,
};
