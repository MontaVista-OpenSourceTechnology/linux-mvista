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

#include <linux/time.h>
#include <asm/ioctls.h>
#include <net/sock.h>

#include "af.h"
#include "gw.h"
#include "raw.h"
#include "route.h"
#include "route-dev.h"
#include "route-local.h"

/** setsockopt callback for SYSCOM sockets */
int syscom_setsockopt(struct socket *sock, int level, int optname,
		char __user *optval, unsigned int optlen)
{
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	int tmp;

	if (level != SOL_SYSCOM) {
		return -EINVAL;
	}

	if (optlen != sizeof tmp) {
		return -EINVAL;
	}
	if (get_user(tmp, (int __user *)optval)) {
		return -EFAULT;
	}

	switch (optname) {
		case SYSCOM_SO_BACKPRESSURE:
			if (tmp == 0) {
				ssk_reset_flag(ssk, SYSCOM_BACKPRESSURE);
			} else if (tmp == 1) {
				ssk_set_flag(ssk, SYSCOM_BACKPRESSURE);
			} else {
				return -EINVAL;
			}
			break;
		case SYSCOM_SO_RECV_FULLHDR:
			if (tmp == 0) {
				ssk_reset_flag(ssk, SYSCOM_RECV_FULLHDR);
			} else if (tmp == 1) {
				ssk_set_flag(ssk, SYSCOM_RECV_FULLHDR);
			} else {
				return -EINVAL;
			}
			break;
		case SYSCOM_SO_SEND_FULLHDR:
			if (tmp == 0) {
				ssk_reset_flag(ssk, SYSCOM_SEND_FULLHDR);
			} else if (tmp == 1) {
				ssk_set_flag(ssk, SYSCOM_SEND_FULLHDR);
			} else {
				return -EINVAL;
			}
			break;
		case SYSCOM_SO_QUEUETSTAMP:
			if (tmp == -1 || tmp == CLOCK_MONOTONIC ||
					 tmp == CLOCK_REALTIME) {
				ssk->tstamp = tmp;
			} else {
				return -EINVAL;
			}
			break;
		case SYSCOM_SO_TX_REPORT:
			ssk->report_fail_reliable = !!(tmp & SYSCOM_SO_TX_REPORT_RELIABLE_FAIL);
			ssk->report_fail_unreliable = !!(tmp & SYSCOM_SO_TX_REPORT_UNRELIABLE_FAIL);
			ssk->report_done_reliable = !!(tmp & SYSCOM_SO_TX_REPORT_RELIABLE_DONE);
			ssk->report_done_unreliable = !!(tmp & SYSCOM_SO_TX_REPORT_UNRELIABLE_DONE);
			break;
		default:
			return -ENOPROTOOPT;
	}

	return 0;
}

/** getsockopt callback for SYSCOM sockets */
int syscom_getsockopt(struct socket *sock, int level, int optname,
		char __user *optval, int __user *optlen)
{
	struct syscom_sock *ssk = syscom_sk(sock->sk);
	struct syscom_stats stats;
	int len, put_len, tmp;
	void *val;

	if (level != SOL_SYSCOM) {
		return -EINVAL;
	}
	if (get_user(len, optlen)) {
		return -EFAULT;
	}
	if (len < 0) {
		return -EINVAL;
	}

	switch (optname) {
		case SYSCOM_SO_BACKPRESSURE:
			put_len = sizeof tmp;
			tmp = ssk_flag(ssk, SYSCOM_BACKPRESSURE);
			val = &tmp;
			break;
		case SYSCOM_SO_RECV_FULLHDR:
			put_len = sizeof tmp;
			tmp = ssk_flag(ssk, SYSCOM_RECV_FULLHDR);
			val = &tmp;
			break;
		case SYSCOM_SO_SEND_FULLHDR:
			put_len = sizeof tmp;
			tmp = ssk_flag(ssk, SYSCOM_SEND_FULLHDR);
			val = &tmp;
			break;
		case SYSCOM_SO_QUEUETSTAMP:
			put_len = sizeof tmp;
			tmp = ssk->tstamp;
			val = &tmp;
			break;
		case SYSCOM_SO_TX_REPORT:
			put_len = sizeof tmp;
			tmp = ssk->report_fail_reliable * SYSCOM_SO_TX_REPORT_RELIABLE_FAIL |
			      ssk->report_fail_unreliable * SYSCOM_SO_TX_REPORT_UNRELIABLE_FAIL |
			      ssk->report_done_reliable * SYSCOM_SO_TX_REPORT_RELIABLE_DONE |
			      ssk->report_done_unreliable * SYSCOM_SO_TX_REPORT_UNRELIABLE_DONE;
			val = &tmp;
			break;
		case SYSCOM_SO_STATS:
			stats.rx_dropped = atomic_read(&ssk->sk.sk_drops);
			stats.tx_dropped = atomic_read(&ssk->stat.tx_drops);
			stats.rx_count = atomic_long_read(&ssk->stat.rx_count);
			stats.tx_count = atomic_long_read(&ssk->stat.tx_count);
			put_len = sizeof stats;
			val = &stats;
			break;
		default:
			return -ENOPROTOOPT;
	}

	if (len < put_len) {
		return -EINVAL;
	}
	if (put_user(put_len, optlen)) {
		return -EFAULT;
	}
	if (copy_to_user(optval, val, put_len)) {
		return -EFAULT;
	}

	return 0;
}

/** ioctl callback for SYSCOM sockets */
int syscom_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct syscom_route uroute;
	struct syscom_gw_open_socket open_sk;
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int err, val;

	switch (cmd) {
	case SYSCOM_ROUTE_ADD:
		if (copy_from_user(&uroute, (void __user *)arg, sizeof uroute)) {
			return -EFAULT;
		}
		if (uroute.type == SYSCOM_ROUTE_RIO) {
			return syscom_route_record_dev_add(uroute.dst_nid,
					uroute.dst_nid_len, SOCKADDR_SYSCOM_ANY_N,
					sock_net(sock->sk), uroute.rio.src_if,
					uroute.rio.frag_threshold,
					&uroute.rio.dst_addr,
					sizeof uroute.rio.dst_addr, 0);
		} else if (uroute.type == SYSCOM_ROUTE_LOCAL) {
			return syscom_route_record_local_add(uroute.dst_nid,
					uroute.dst_nid_len, SOCKADDR_SYSCOM_ANY_N);
		} else {
			return -EINVAL;
		}
	case SYSCOM_ROUTE_DEL:
		if (copy_from_user(&uroute, (void __user *)arg, sizeof uroute)) {
			return -EFAULT;
		}
		if (uroute.type != SYSCOM_ROUTE_DELETE) {
			return -EINVAL;
		}
		err = syscom_route_del(uroute.dst_nid, uroute.dst_nid_len);
		if (err) {
			return err;
		}
		break;
	case SYSCOM_GW_OPEN_SOCKET:
		if (copy_from_user(&open_sk, (void __user *)arg, sizeof open_sk)) {
			return -EFAULT;
		}
		open_sk.gw[sizeof(open_sk.gw) - 1] = 0;
		return syscom_gw_open_socket(open_sk.gw, open_sk.flags);
	case SIOCINQ:
		lock_sock(sk);
		skb = skb_peek(&sk->sk_receive_queue);
		if (skb) {
			struct syscom_hdr *hdr = (struct syscom_hdr*)skb->data;
			val = ntohs(hdr->length);
			if (sk->sk_type == SOCK_DGRAM &&
			    !ssk_flag(syscom_sk(sk), SYSCOM_RECV_FULLHDR)) {
				val -= sizeof *hdr - sizeof hdr->msg_id -
						sizeof hdr->flags;
			}
		} else {
			val = 0;
		}
		release_sock(sk);
		return put_user(val, (int __user *)arg);
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

#define update_stat(sock_atomic, glob_atomic, type) do { \
		sock##type##stat = atomic##type##read(&(sock_atomic)); \
		glob##type##stat = atomic##type##read(&(glob_atomic));	\
		while (unlikely(sock##type##stat > glob##type##stat)) { \
			glob##type##stat = sock##type##stat; \
			sock##type##stat = atomic##type##xchg(&(glob_atomic), \
					sock##type##stat); \
		} \
	} while (0)

void syscom_sock_update_stats(struct syscom_sock *ssk)
{
	unsigned long sock_long_stat, glob_long_stat;

	update_stat(ssk->stat.max_rx_latency, syscom_stats.sock_max_rx_latency, _long_);
	update_stat(ssk->stat.max_tx_latency, syscom_stats.sock_max_tx_latency, _long_);
	atomic_add(atomic_read(&ssk->stat.tx_drops), &syscom_stats.sock_tx_drops);
	atomic_add(atomic_read(&ssk->sk.sk_drops), &syscom_stats.sock_rx_drops);
}

#ifndef CONFIG_CPU_BIG_ENDIAN
void syscom_addr_to_be(struct sockaddr *addr, int addrsize)
{
	char *ptr = (char *)addr;
	__be16 be_family;
	uint16_t family;
	int cnt;

	for (cnt = 0; ptr - (char *)addr + sizeof family < addrsize; cnt++) {
		memcpy(&family, ptr, sizeof family);
		be_family = htons(family);
		memcpy(ptr, &be_family, sizeof family);

		switch (family) {
			case AF_INET: ptr += sizeof (struct sockaddr_in);
				break;
			case AF_INET6: ptr += sizeof (struct sockaddr_in6);
				break;
			default:
				pr_crit("Family %d is not supported\n", family);
				return;
		}
	}
}

void syscom_addr_from_be(struct sockaddr *addr, int addrsize)
{
	char *ptr = (char *)addr;
	__be16 be_family;
	uint16_t family;
	int cnt;

	for (cnt = 0; ptr - (char *)addr + sizeof family < addrsize; cnt++) {
		memcpy(&be_family, ptr, sizeof be_family);
		family = ntohs(be_family);
		memcpy(ptr, &family, sizeof family);

		switch (family) {
			case AF_INET: ptr += sizeof (struct sockaddr_in);
				break;
			case AF_INET6: ptr += sizeof (struct sockaddr_in6);
				break;
			default:
				pr_crit("Family %d is not supported\n", family);
				return;
		}
	}
}
#endif // CONFIG_CPU_BIG_ENDIAN

