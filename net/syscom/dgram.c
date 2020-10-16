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

#include <linux/socket.h>
#include <linux/netdevice.h>
#include <linux/errqueue.h>
#include <linux/uio.h>
#include <linux/version.h>

#include "dgram.h"
#include "route.h"
#include "af.h"
#include "raw.h"

static inline int syscom_gen_cpid(void)
{
	static atomic_t cpid_counter;
	return 0xe000 | (0xfff & atomic_inc_return(&cpid_counter));
}

static inline int syscom_sock_add_auto_addr(struct syscom_sock *ssk)
{
	uint16_t cpid, cpid_init;
	int rtn;

	cpid = cpid_init = syscom_gen_cpid();
	do {
		ssk->local.cpid = htons(cpid);
		rtn = syscom_sock_add(ssk, 1);
	} while (rtn == -EADDRINUSE && cpid_init != (cpid = syscom_gen_cpid()));

	return rtn;
}

/** bind callback for SYSCOM datagram sockets. */
static int syscom_dgram_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
	struct sockaddr_syscom *uaddr_syscom = (struct sockaddr_syscom *)uaddr;
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	int rtn;

	if (addr_len < sizeof(struct sockaddr_syscom)) {
		return -EINVAL;
	}
	if (uaddr->sa_family != AF_SYSCOM) {
		return -EINVAL;
	}

	lock_sock(sock->sk);
	if (ssk->sk.sk_state != SYSCOM_NEW &&
	    ssk->sk.sk_state != SYSCOM_BIND_FAILED) {
		rtn = ssk->sk.sk_state == SYSCOM_ROBBED ?
				-ESHUTDOWN : -EALREADY;
		goto out;
	}

	ssk->remote.nid = SOCKADDR_SYSCOM_ANY_N;
	ssk->remote.cpid = SOCKADDR_SYSCOM_ANY_N;
	ssk->local.nid = uaddr_syscom->nid;
	if (uaddr_syscom->cpid != htons(SOCKADDR_SYSCOM_CPID_AUTO)) {
		// Bind to an address
		ssk->local.cpid = uaddr_syscom->cpid;
		rtn = syscom_sock_add(ssk, 0);
	} else {
		rtn = syscom_sock_add_auto_addr(ssk);
	}

	if (rtn) {
		sock->sk->sk_state = rtn == -ESHUTDOWN ?
				SYSCOM_ROBBED : SYSCOM_BIND_FAILED;
	} else {
		sock->sk->sk_state = SYSCOM_BOUND;
	}
out:	release_sock(sock->sk);

	return rtn;
}

/** release callback for SYSCOM datagram sockets. */
static int syscom_dgram_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	syscom_sock_remove(syscom_sk(sk));
	syscom_release(syscom_sk(sk));

	return 0;
}

static int syscom_skb_copy_datagram_msg(struct sk_buff *skb, struct msghdr *msg,
		size_t size, int fullhdr)
{
	struct syscom_hdr *hdr;
	size_t skip, copied;
	int len, err;

	hdr = (struct syscom_hdr*)skb->data;

	if (fullhdr) {
		len = ntohs(hdr->length);
		copied = skip = 0;
	} else {
		const size_t uhdr_size = sizeof *hdr - sizeof hdr->msg_id -
				sizeof hdr->flags;
		// We do not check for error here, because the failure will
		// be detected by skb_copy_datagram_msg again
		(void)!copy_to_iter(&hdr->msg_id, sizeof hdr->msg_id,
				&msg->msg_iter);
		copied = sizeof hdr->msg_id;
		len = ntohs(hdr->length) - uhdr_size;
		skip = uhdr_size + copied;
	}

	if (size > len)
		size = len;
	else if (size < len)
		msg->msg_flags |= MSG_TRUNC;
	// BPF can truncate the message down to the syscom header
	if (unlikely(size > skb->len - skip + copied)) {
		size = skb->len - skip + copied;
	}

	err = skb_copy_datagram_msg(skb, skip, msg, size - copied);
	return err < 0 ? err : size;
}

static void syscom_dgram_fill_msg_name(struct msghdr *msg,
		const struct syscom_hdr *hdr, bool dst)
{
	if (msg->msg_name) {
		struct sockaddr_syscom *addr = msg->msg_name;
		addr->nid = dst ? hdr->dst.nid : hdr->src.nid;
		addr->cpid = dst ? hdr->dst.cpid : hdr->src.cpid;
		addr->sun_family = AF_SYSCOM;
		msg->msg_namelen = sizeof *addr;
	}
}

/** recvmsg callback for SYSCOM datagram sockets. */
static int syscom_dgram_recvmsg(struct socket *sock,
		struct msghdr *msg, size_t size, int flags)
{
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	struct syscom_hdr *hdr;
	struct sk_buff *skb;
	int err;
	int off = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 2, 0)
	int peeked;
#endif
	ktime_t time_now, time_queue;

	if (unlikely(flags & MSG_ERRQUEUE)) {
		// NOTE: We do not trace error queue receive operations
		struct {
			struct sock_extended_err ee;
			struct sockaddr_syscom offender;
		} errhdr;
		struct sock_exterr_skb *exterr;

		skb = sock_dequeue_err_skb(sock->sk);
		if (!skb) {
			err = -EAGAIN;
			goto err0;
		}

		exterr = SKB_EXT_ERR(skb);
		errhdr.ee = exterr->ee;

		put_cmsg(msg, SOL_SYSCOM, SYSCOM_SO_TX_REPORT,
				sizeof(errhdr), &errhdr);
		msg->msg_flags |= MSG_ERRQUEUE;

		hdr = (struct syscom_hdr*)skb->data;
		err = syscom_skb_copy_datagram_msg(skb, msg, size,
				ssk_flag(ssk, SYSCOM_RECV_FULLHDR));
		if (likely(err >= 0)) {
			syscom_dgram_fill_msg_name(msg, hdr, 1);
			consume_skb(skb);
		} else {
			trace_syscom_sk_recv_err(ssk, hdr, err);
			kfree_skb(skb);
		}

		return err;
	}

	if (unlikely(!skb_queue_empty(&ssk->sk_oob_queue))) {
		if (!sock_flag(&ssk->sk, SOCK_URGINLINE) &&
		    !(flags & MSG_OOB)) {
			err = -EBUSY;
			goto err0;
		}
		skb = skb_dequeue(&ssk->sk_oob_queue);
		if (skb) goto rcv;
	}
	if (unlikely(flags & MSG_OOB)) {
		err = sock_flag(&ssk->sk, SOCK_URGINLINE) ? -EINVAL : -EAGAIN;
		goto err0;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 2, 0)
	skb = __skb_recv_datagram(sock->sk, flags, NULL, &peeked, &off, &err);
#else
	skb = __skb_recv_datagram(sock->sk, flags, NULL, &off, &err);
#endif
	if (!skb) {
		goto err0;
	}

rcv:	wake_up_interruptible(&ssk->loopback_waiters);

	// Packets shorter than header are dropped by syscom_rcv
	hdr = (struct syscom_hdr*)skb->data;
	time_now = ktime_get();
	time_queue = *(ktime_t*)skb->cb;

	err = syscom_skb_copy_datagram_msg(skb, msg, size,
			ssk_flag(ssk, SYSCOM_RECV_FULLHDR));
	if (likely(err >= 0)) {
		if (skb->syscom_oob) {
			msg->msg_flags |= MSG_OOB;
		}

		syscom_dgram_fill_msg_name(msg, hdr, 0);
		syscom_latency_update(&ssk->stat.max_rx_latency, time_queue,
				time_now, trace_syscom_sk_recv_latency_grow,
				ssk, skb);

		size = err;
		err = syscom_recv_ts(syscom_sk(sock->sk), msg, time_queue);
		sock_recv_ts_and_drops(msg, &ssk->sk, skb);

		if (likely(!err)) err = size;
	}

	if (unlikely(err < 0)) {
		trace_syscom_sk_recv_err(ssk, hdr, err);
	}
	skb_free_datagram_locked(sock->sk, skb);
	return err;

err0:	if ((!err || err == -EAGAIN) && ssk->sk.sk_state == SYSCOM_ROBBED)
		err = -ESHUTDOWN;
	trace_syscom_sk_recv_err(ssk, NULL, err);
	return err;
}

/** sendmsg callback for SYSCOM datagram sockets. */
static int syscom_dgram_sendmsg(struct socket *sock,
		struct msghdr *msg, size_t size)
{
	struct syscom_sock *ssk;
	struct sock *sk = sock->sk;
	__be16 nid, cpid;
	int state, rtn = 0;

	ssk = syscom_sk(sk);
	state = READ_ONCE(ssk->sk.sk_state);

	if (state == SYSCOM_ROBBED) {
		rtn = -ESHUTDOWN;
		goto err0;
	}

	if (ssk_flag(ssk, SYSCOM_SEND_FULLHDR)) {
		if (unlikely(msg->msg_namelen != 0)) {
			rtn = -EINVAL;
			goto err0;
		}
		rtn = syscom_route_deliver_rawmsg(ssk, msg, size);
		return syscom_send_rtn(ssk, rtn, size);
	}

	if (state == SYSCOM_BIND_FAILED) {
		rtn = -EADDRNOTAVAIL;
		goto err0;
	}

	// Get the destination
	if (state == SYSCOM_CONNECTED) {
		if (msg->msg_name != NULL) {
			rtn = -EISCONN;
			goto err0;
		} else {
			nid = ssk->remote.nid;
			cpid = ssk->remote.cpid;
		}
	} else {
		struct sockaddr_syscom *addr;

		if (state == SYSCOM_NEW) {
			// Bind the socket automatically
			struct sockaddr_syscom addr = {
				.sun_family = AF_SYSCOM,
				.nid = SOCKADDR_SYSCOM_ANY_N,
			};
			rtn = syscom_dgram_bind(sock, (struct sockaddr *)&addr,
					sizeof addr);
			if (rtn) {
				BUG_ON(rtn > 0);
				return rtn;
			}
		}

		addr = (struct sockaddr_syscom *)msg->msg_name;
		if (addr == NULL) {
			rtn = -ENOTCONN;
			goto err0;
		} else if (msg->msg_namelen < sizeof *addr ||
				addr->sun_family != AF_SYSCOM) {
			rtn = -EINVAL;
			goto err0;
		} else {
			nid = addr->nid;
			cpid = addr->cpid;
		}
	}

	rtn = syscom_route_deliver_msg(ssk, nid, cpid, msg, size);
err0:	return syscom_send_rtn(ssk, rtn, size);
}

/** poll callback for SYSCOM datagram sockets. */
static __poll_t syscom_dgram_poll(struct file *file, struct socket *sock,
		poll_table *wait)
{
	struct sock *sk = sock->sk;
	struct syscom_sock *ssk = syscom_sk(sk);
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
	if (!skb_queue_empty(&ssk->sk_oob_queue))
		mask |= EPOLLPRI;
	if (sock_wspace(sk))
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (sk->sk_state == SYSCOM_ROBBED)
		mask |= EPOLLHUP;

	return mask;
}

/** getname callback for SYSCOM datagram sockets. */
static int syscom_getname(struct socket *sock, struct sockaddr *saddr, int peer)
{
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	DECLARE_SOCKADDR(struct sockaddr_syscom *, addr, saddr);

	lock_sock(sock->sk);

	if (peer) {
		if (sock->sk->sk_state != SYSCOM_CONNECTED) {
			release_sock(sock->sk);
			return -ENOTCONN;
		}
		addr->nid = ssk->remote.nid;
		addr->cpid = ssk->remote.cpid;
	} else {
		if (sock->sk->sk_state != SYSCOM_CONNECTED &&
		    sock->sk->sk_state != SYSCOM_BOUND) {
			release_sock(sock->sk);
			return -ESOCKTNOSUPPORT;
		}
		addr->nid = ssk->local.nid;
		addr->cpid = ssk->local.cpid;
	}

	release_sock(sock->sk);

	addr->sun_family = AF_SYSCOM;
	return sizeof(*addr);
}

/** connect callback for SYSCOM datagram sockets. */
static int syscom_dgram_connect(struct socket *sock, struct sockaddr *uaddr,
		int len, int flags)
{
	struct sockaddr_syscom *uaddr_syscom = (struct sockaddr_syscom *)uaddr;
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	int rtn = 0;

	if (len < sizeof *uaddr_syscom) {
		return -EINVAL;
	}

	if (uaddr->sa_family != AF_SYSCOM) {
		return -EAFNOSUPPORT;
	}

	if (uaddr_syscom->nid == SOCKADDR_SYSCOM_ORPHANS_N ||
	    uaddr_syscom->nid == SOCKADDR_SYSCOM_ANY_N ||
	    uaddr_syscom->cpid == SOCKADDR_SYSCOM_ORPHANS_N ||
	    uaddr_syscom->cpid == SOCKADDR_SYSCOM_ANY_N) {
		return -EINVAL;
	}

	lock_sock(sock->sk);
	switch (sock->sk->sk_state) {
		case SYSCOM_NEW:
			ssk->local.nid = SOCKADDR_SYSCOM_ANY_N;
			rtn = syscom_sock_add_auto_addr(ssk);
			if (rtn) goto unlock;
			break;
		case SYSCOM_BOUND:
			break;
		case SYSCOM_CONNECTED:
			rtn = -EISCONN;
			goto unlock;
		case SYSCOM_ROBBED:
			rtn = -ESHUTDOWN;
			goto unlock;
		default:
			rtn = -EADDRNOTAVAIL;
			goto unlock;
	}

	ssk->remote.nid = uaddr_syscom->nid;
	ssk->remote.cpid = uaddr_syscom->cpid;
	sock->sk->sk_state = SYSCOM_CONNECTED;

unlock:	release_sock(sock->sk);

	return rtn;
}

/** socketpair callback for SYSCOM datagram sockets. */
static int syscom_dgram_socketpair(struct socket *sock1, struct socket *sock2)
{
	struct syscom_sock *ssk1 = syscom_sk(sock1->sk);
	struct syscom_sock *ssk2 = syscom_sk(sock2->sk);
	int rtn;

	// This prevents delivery of messages after we bind the socket
	// as the remote CPID is 0 - SOCKADDR_SYSCOM_CPID_AUTO
	sock1->sk->sk_state = SYSCOM_CONNECTED;
	sock2->sk->sk_state = SYSCOM_CONNECTED;

	rtn = syscom_sock_add_auto_addr(ssk1);
	if (rtn) {
		goto err0;
	}

	rtn = syscom_sock_add_auto_addr(ssk2);
	if (rtn) {
		goto err1;
	}

	ssk1->remote.cpid = ssk2->local.cpid;
	ssk2->remote.cpid = ssk1->local.cpid;

	return 0;

err0:	sock1->sk->sk_state = SYSCOM_NEW;
err1:	sock2->sk->sk_state = SYSCOM_NEW;
	return rtn;
}

/** Protocol operations of a SYSCOM datagram socket. */
const struct proto_ops syscom_dgram_ops = {
	.family =       PF_SYSCOM,
	.owner =        THIS_MODULE,
	.release =      syscom_dgram_release,
	.bind =         syscom_dgram_bind,
	.connect =      syscom_dgram_connect,
	.socketpair =   syscom_dgram_socketpair,
	.accept =       sock_no_accept,
	.getname =      syscom_getname,
	.poll =         syscom_dgram_poll,
	.ioctl =        syscom_ioctl,
	.listen =       sock_no_listen,
	.shutdown =     sock_no_shutdown,
	.setsockopt =   syscom_setsockopt,
	.getsockopt =   syscom_getsockopt,
	.sendmsg =      syscom_dgram_sendmsg,
	.recvmsg =      syscom_dgram_recvmsg,
	.mmap =         sock_no_mmap,
	.sendpage =     sock_no_sendpage,
};
