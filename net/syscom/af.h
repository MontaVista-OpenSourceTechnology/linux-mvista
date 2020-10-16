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

#ifndef SYSCOM_AF_H
#define SYSCOM_AF_H

#include <linux/if.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/if_syscom.h>
#include <linux/compiler.h>
#include <net/sock.h>

#include <if_syscom_ioctl.h>

#define SOCKADDR_SYSCOM_ORPHANS_N  ((__be16)htons(SOCKADDR_SYSCOM_ORPHANS))
#define SOCKADDR_SYSCOM_ANY_N     ((__be16)htons(SOCKADDR_SYSCOM_ANY))

/** Maximum transmission unit, currently limited by the SRIO transport */
#define SYSCOM_MTU ((4096-8)*16)

/** Maximum length of a syscom message */
#define SYSCOM_MAX_MSGSIZE 65535

/** Copy a string, emit warning when it doesn't fit */
#define strbcpy(dest, src, count) WARN_ON(-E2BIG == strscpy(dest, src, count))

extern int syscom_forward;
extern int syscom_trace_sndbuf_skbs;
extern int syscom_gw_rx_timestamping;
extern unsigned syscom_route_queue_bytes;

/** Socket states for sk_state */
enum syscom_sock_states {
	/** Socket is new - not bound or connected */
	SYSCOM_NEW,
	/** Socket is bound to a NID/CPID */
	SYSCOM_BOUND,
	/** Socket is connected */
	SYSCOM_CONNECTED,
	/** Weak socket which lost its address */
	SYSCOM_ROBBED,
	/** Bind was attempted, but failed. We do not autobind these sockets */
	SYSCOM_BIND_FAILED,
};


struct syscom_sock {
	/* WARNING: sk has to be the first member */
	struct sock sk;
	/** Various socket statistics */
	struct {
		atomic_long_t rx_bytes;
		atomic_long_t rx_count;
		atomic_long_t tx_bytes;
		atomic_long_t tx_count;
		atomic_long_t max_rx_latency;
		atomic_long_t max_tx_latency;
		// rx_drop is implemented by sk_drops
		atomic_t tx_drops;
	} stat;
	/** Local address */
	struct {
		/** Local NID */
		__be16 nid;
		/** Local CPID */
		__be16 cpid;
	} local;
	/** Remote address (if the socket is connected) */
	struct {
		/** Remote NID */
		__be16 nid;
		/** Remote CPID */
		__be16 cpid;
	} remote;
	/** Ordered delivery of non-raw messages */
	bool ordered;
	/** Ordered delivery of non-raw messages (first hop, next hop uses
	 * information from the header) */
	bool reliable;
	/** Queue for back pressure waiters */
	wait_queue_head_t loopback_waiters;
	/** Next socket with the same local address (for SO_REUSEADDR) */
	struct syscom_sock __rcu *next;
	/** Used to queue on unhashed list, using default sk_node would
	 *  require an additional rcu_synchronize call during bind */
	struct hlist_node unhashed;
	/** Used to queue on unhashed list, using default sk_node would
	 *  require an additional rcu_synchronize call during "robbing" */
	struct hlist_node robbed;
	/** Queue for OOB traffic */
	struct sk_buff_head sk_oob_queue;
	/** Socket flags */
	unsigned long ssk_flags;
	/** Timestamping configuration */
	int tstamp;
	/** Queue error SKB if reliable TX fails */
	unsigned report_fail_reliable:1;
	/** Queue error SKB if unreliable TX fails */
	unsigned report_fail_unreliable:1;
	/** Queue error SKB if reliable TX succeeds */
	unsigned report_done_reliable:1;
	/** Queue error SKB if unreliable TX succeeds */
	unsigned report_done_unreliable:1;
	/** List of SKBs in send queue (debugging feature) */
	struct list_head skb_snd_list;
	/** Protects skb_snd_list */
	spinlock_t skb_snd_list_lock;
};

int syscom_sock_add(struct syscom_sock *sk, bool ignore_weak);

void syscom_sock_remove(struct syscom_sock *sk);

void syscom_sock_dump_sndbuf(struct syscom_sock *ssk);

int syscom_queue_rcv_skb(struct sk_buff *skb, long timeo, bool bp);

typedef struct sk_buff *(*syscom_skb_alloc)(unsigned header_len,
		unsigned data_len, int *errcode, void *arg);

void _syscom_delivery_error(int rtn, void *hdrp, struct sk_buff *skb,
		const char *where, const char *file, int line);

#define syscom_delivery_error_skb(rtn, hdrp, skb, where) \
	_syscom_delivery_error(rtn, hdrp, skb, where, __FUNCTION__, __LINE__)

#define syscom_delivery_error(rtn, hdrp, where) \
	_syscom_delivery_error(rtn, hdrp, NULL, where, __FUNCTION__, __LINE__)

/** Cast struct sock to struct syscom_sock with a type check */
static inline struct syscom_sock *syscom_sk(struct sock *sk)
{
	return (struct syscom_sock *)sk;
}

/** Sycom socket flags */
enum syscom_sock_flags {
	SYSCOM_BACKPRESSURE,
	SYSCOM_RECV_FULLHDR,
	SYSCOM_SEND_FULLHDR,
	SYSCOM_WEAK_BIND,
};

static inline void ssk_set_flag(struct syscom_sock *ssk, enum syscom_sock_flags flag)
{
        __set_bit(flag, &ssk->ssk_flags);
}

static inline void ssk_reset_flag(struct syscom_sock *ssk, enum syscom_sock_flags flag)
{
        __clear_bit(flag, &ssk->ssk_flags);
}

static inline int ssk_flag(struct syscom_sock *ssk, enum syscom_sock_flags flag)
{
        return test_bit(flag, &ssk->ssk_flags);
}

/** Syscom statistics structure */
struct syscom_stats_s {
	/** How many times a slow route lookup was taken */
	atomic_t route_tree_miss;
	/** Number of route tree nodes */
	atomic_t route_tree_nodes;
	/** Number of route tree optimizations */
	atomic_t route_tree_optimizations;
	/** Sum of route errors among all destroyed routes */
	atomic_t route_err;
	/** Sum of messages in route work queues */
	atomic_t route_queue_bytes;

	/** Sum of tx_drop among all destroyed sockets */
	atomic_t sock_tx_drops;
	/** Sum of rx_drop among all destroyed sockets */
	atomic_t sock_rx_drops;
	/** Maximum RX latency among all destroyed sockets */
	atomic_long_t sock_max_rx_latency;
	/** Maximum TX latency among all destroyed sockets */
	atomic_long_t sock_max_tx_latency;

	/** Number of malformed messages discarded by the delivery code */
	atomic_t deliver_malformed;
	/** Number of messages discarded for being received out of order */
	atomic_t deliver_out_of_order;
	/** Number of messages discarded due to a reassemble timeout */
	atomic_t deliver_timeout;
	/** Number of messages received, which were not for the local host */
	atomic_t deliver_remote;
	/** Number of messages discarded due to a missing route */
	atomic_t deliver_net_unreach;
	/** Number of messages discarded due to a missing socket */
	atomic_t deliver_host_unreach;
	/** Number of messages discarded by the delivery code for another
	 * reason then one of these listed */
	atomic_t deliver_other_errors;

	/** Sum of dropped outgoing messages among all destroyed gateways */
	atomic_t gw_tx_drops;
	/** Sum of dropped incoming messages among all destroyed gateways */
	atomic_t gw_rx_drops;
	/** Sum of errors among all destroyed gateways */
	atomic_t gw_errors;
	/** Sum of ready_would_block among all destroyed gateways */
	atomic_t gw_ready_would_block;
	/** Maximum RX SW latency among destroyed gateways */
	atomic_long_t gw_rx_sw_us;
	/** Maximum RX HW latency among destroyed gateways */
	atomic_long_t gw_rx_hw_us;
	/** Maximum RX delivery latency among destroyed gateways */
	atomic_long_t gw_rx_dl_us;

	atomic_t notify_errors;
};

/** The statistics structure instance */
extern struct syscom_stats_s syscom_stats;

/** Abuse wifi_acked field to mark OOB messages */
#define syscom_oob wifi_acked

/** Compare addresses to be equal */
#define syscom_addr_eq(a1, a2) ((a1).cpid == (a2).cpid && (a1).nid == (a2).nid)

/** Size difference between raw and DGRAM headers */
#define SYSCOM_DGRAM_RAW_HDR_DELTA \
	(sizeof(struct syscom_hdr) - sizeof(struct syscom_dgram_hdr))

/** Update max value tracking stat */
static inline int _syscom_max_update(long v, atomic_long_t *max)
{
	long m = atomic_long_read(max);
	int updated = 0;

	while (unlikely(v > m)) {
		m = v;
		v = atomic_long_xchg(max, v);
		updated = 1;
	}

	return updated;
}

/** Update max value tracking stat */
static inline void syscom_max_update(atomic_long_t *val, atomic_long_t *max)
{
	long v = atomic_long_read(val);
	_syscom_max_update(v, max);
}

/** Update latency tracing atomic */
#define syscom_latency_update(max, start, end, trace, ...) do { \
		long delta = (long)ktime_to_us(ktime_sub((end), (start))); \
		if (_syscom_max_update(delta, max)) trace(__VA_ARGS__, delta); \
	} while (0)

#endif // SYSCOM_AF_H
