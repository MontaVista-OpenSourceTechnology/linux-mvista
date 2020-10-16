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

#ifndef SYSCOM_COMMON_H
#define SYSCOM_COMMON_H

#define pr_fmt(fmt) "syscom: "fmt

#include <linux/version.h>
#include <linux/ktime.h>
#include <net/sock.h>

#include "af.h"
#include <trace/events/syscom.h>

int syscom_setsockopt(struct socket *sock, int level, int optname,
		char __user *optval, unsigned int optlen);

int syscom_getsockopt(struct socket *sock, int level, int optname,
		char __user *optval, int __user *optlen);

int syscom_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg);

/** Add the queue time stamp to ancillary data */
static inline int syscom_recv_ts(struct syscom_sock *ssk, struct msghdr *msg,
		ktime_t time_queue)
{
	if (ssk->tstamp == CLOCK_MONOTONIC) {
		struct timespec ts = ktime_to_timespec(time_queue);
		return put_cmsg(msg, SOL_SYSCOM, SYSCOM_SO_QUEUETSTAMP,
				sizeof ts, &ts);
	} else if (ssk->tstamp == CLOCK_REALTIME) {
		struct timespec ts;
		ts = ktime_to_timespec(ktime_mono_to_real(time_queue));
		return put_cmsg(msg, SOL_SYSCOM, SYSCOM_SO_QUEUETSTAMP,
				sizeof ts, &ts);
	}
	return 0;
}

static inline int syscom_send_rtn(struct syscom_sock *ssk, int rtn, int size)
{
	if (unlikely(rtn < 0)) {
		trace_syscom_sk_send_err(ssk, rtn);
		return rtn;
	}
	if (unlikely(rtn > 0)) {
		trace_syscom_sk_send_drop(ssk);
		atomic_inc(&ssk->stat.tx_drops);
	}
	atomic_long_inc(&ssk->stat.tx_count);
	return size;
}

void syscom_release(struct syscom_sock *ssk);

#ifdef CONFIG_CPU_BIG_ENDIAN
static inline void syscom_addr_to_be(struct sockaddr *addr, int addrsize) { }
static inline void syscom_addr_from_be(struct sockaddr *addr, int addrsize) { }
#else
void syscom_addr_to_be(struct sockaddr *addr, int addrsize);
void syscom_addr_from_be(struct sockaddr *addr, int addrsize);
#endif

#endif // SYSCOM_COMMON_H
