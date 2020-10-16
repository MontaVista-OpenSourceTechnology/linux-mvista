// SPDX-License-Identifier: GPL-2.0
/*
 * SYSCOM protocol stack for the Linux kernel
 * Author: Petr Malat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
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

#include <linux/version.h>
#include <trace/events/sock.h>
#include <linux/module.h>
#include <linux/radix-tree.h>
#include <linux/rcupdate.h>
#include <linux/rculist.h>
#include <asm/bitsperlong.h>
#include <linux/errqueue.h>
#include <linux/mutex.h>
#include <linux/jhash.h>
#include <linux/sysctl.h>
#include <net/sock.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/if_syscom_ether.h>
#include <linux/oom.h>
#include <net/syscom.h>
#include "af.h"
#include "dgram.h"
#include "route.h"
#include "route-local.h"
#include "route-dev.h"
#include "gw.h"
#include "raw.h"
#include "service-socket.h"
#include "notify.h"

/***** Socket hash ******************************************************/
#define SYSCOM_SK_HASH_SIZE 10
static DEFINE_MUTEX(syscom_sk_hash_lock);
static struct hlist_head syscom_sk_hash[jhash_size(SYSCOM_SK_HASH_SIZE)];
static HLIST_HEAD(unhashed_sockets);
static HLIST_HEAD(robbed_sockets);

static inline struct hlist_head *syscom_sock_hlist(
		__be16 local_nid, __be16 local_cpid)
{
	return &syscom_sk_hash[jhash_1word((local_nid << 16) | local_cpid, 0)
	                       & jhash_mask(SYSCOM_SK_HASH_SIZE)];
}

#define rcu_ssk_next_protected(ssk) rcu_dereference_protected((ssk)->next,\
		lockdep_is_held(&syscom_sk_hash_lock))

static void syscom_sock_rob(struct syscom_sock *ssk)
{
	struct sock *sk = &ssk->sk;
	struct socket_wq *wq;

	lock_sock(sk);
	sk->sk_state = SYSCOM_ROBBED;
	sk->sk_shutdown |= RCV_SHUTDOWN;
	// We can't use sk_node or unhashed without calling synchronize_rcu, but
	// until then the socket wouldn't be visible in proc. Use another list
	// instead..
	hlist_add_head_rcu(&ssk->robbed, &robbed_sockets);
	release_sock(sk);

	rcu_read_lock();
	wq = rcu_dereference(sk->sk_wq);
	if (skwq_has_sleeper(wq))
		wake_up_interruptible_sync(&wq->wait);
	rcu_read_unlock();

	sk_wake_async(sk, SOCK_WAKE_WAITD, POLL_HUP);
}

int syscom_sock_add(struct syscom_sock *new, bool new_bind_only)
{
	struct hlist_head *hlist;
	struct sock *sk;

	hlist = syscom_sock_hlist(new->local.nid, new->local.cpid);

	mutex_lock(&syscom_sk_hash_lock);

	// Unbind weak sockets
	if (!new_bind_only) sk_for_each(sk, hlist) {
		struct syscom_sock *ssk = syscom_sk(sk);
		bool all_week = 1;
		if (syscom_addr_eq(ssk->local, new->local)) {
			do {
				if (!ssk_flag(ssk, SYSCOM_WEAK_BIND)) {
					all_week = 0;
					break;
				}
			} while ((ssk = rcu_ssk_next_protected(ssk)));
			ssk = syscom_sk(sk);

			if (ssk_flag(new, SYSCOM_WEAK_BIND) && !all_week) {
				mutex_unlock(&syscom_sk_hash_lock);
				return -ESHUTDOWN;
			}
			if (!ssk_flag(new, SYSCOM_WEAK_BIND) && all_week) {
				syscom_sock_rob(ssk);
				sk_del_node_init_rcu(&ssk->sk);
				while ((ssk = rcu_ssk_next_protected(ssk))) {
					syscom_sock_rob(ssk);
					sock_put(&ssk->sk);
				}
			}
		}
	}

	sk_for_each(sk, hlist) {
		struct syscom_sock *ssk = syscom_sk(sk);
		if (syscom_addr_eq(ssk->local, new->local)) {
			if (new_bind_only) {
				mutex_unlock(&syscom_sk_hash_lock);
				return -EADDRINUSE;
			}
			if (new->sk.sk_reuse == SK_NO_REUSE) {
				mutex_unlock(&syscom_sk_hash_lock);
				return -EADDRINUSE;
			}
			if (ssk->sk.sk_reuse == SK_NO_REUSE &&
			    new->sk.sk_reuse != SK_FORCE_REUSE) {
				mutex_unlock(&syscom_sk_hash_lock);
				return -EADDRINUSE;
			}
			rcu_assign_pointer(new->next, rcu_ssk_next_protected(ssk));
			rcu_assign_pointer(ssk->next, new);
			sock_hold(&new->sk);
			goto out;
		}
	}

	rcu_assign_pointer(new->next, NULL);
	sk_add_node_rcu(&new->sk, hlist);
out:	hlist_del_init_rcu(&new->unhashed);
	mutex_unlock(&syscom_sk_hash_lock);

	return 0;
}

struct syscom_iter {
	struct syscom_sock *ssk;
	struct sk_buff *skb;
	const struct syscom_hdr *hdr;
	int how;
};

static struct syscom_sock *syscom_sock_skip(
		const struct syscom_iter *iter,
		struct syscom_sock *ssk)
{
	while (ssk && ssk->sk.sk_state == SYSCOM_CONNECTED) {
		if (iter->hdr->src.nid == ssk->remote.nid &&
		    iter->hdr->src.cpid == ssk->remote.cpid) {
			break;
		}
		ssk = rcu_dereference(ssk->next);
	}
	return ssk;
}

static struct syscom_sock *syscom_sock_lookup(
		const struct syscom_iter *iter,
		const __be16 nid, const __be16 cpid)
{
	struct sock *sk;

	sk_for_each_rcu(sk, syscom_sock_hlist(nid, cpid)) {
		struct syscom_sock *ssk = syscom_sk(sk);
		if (ssk->local.nid == nid && ssk->local.cpid == cpid) {
			return syscom_sock_skip(iter, ssk);
		}
	}

	return NULL;
}

static struct syscom_sock *syscom_iter_next(struct syscom_iter *iter)
{
	struct syscom_sock *old = iter->ssk;

	rcu_read_lock();
	if (iter->ssk) {
		iter->ssk = syscom_sock_skip(iter, rcu_dereference(iter->ssk->next));
		if (iter->ssk) {
			goto done;
		}
	}

	// We need new head
	switch (iter->how) {
#define SL(c, nid, cpid, next) \
			__attribute__((fallthrough)); \
		state_##c: __attribute__((unused)) case c: \
			iter->ssk = syscom_sock_lookup(iter, nid, cpid); \
			if (iter->ssk) { iter->how = next; break; }
		default: BUG();

		SL(1, iter->hdr->dst.nid, iter->hdr->dst.cpid, 11);
		SL(2, iter->hdr->dst.nid, SOCKADDR_SYSCOM_ANY_N, 21);
		SL(3, SOCKADDR_SYSCOM_ANY_N, iter->hdr->dst.cpid, 31);
		SL(4, iter->hdr->dst.nid, SOCKADDR_SYSCOM_ORPHANS_N, 8);
		SL(5, SOCKADDR_SYSCOM_ORPHANS_N, iter->hdr->dst.cpid, 33);
		SL(6, SOCKADDR_SYSCOM_ORPHANS_N, SOCKADDR_SYSCOM_ORPHANS_N, 7);
		SL(7, SOCKADDR_SYSCOM_ORPHANS_N, SOCKADDR_SYSCOM_ANY_N, 8);
		SL(8, SOCKADDR_SYSCOM_ANY_N, SOCKADDR_SYSCOM_ORPHANS_N, 9);
		SL(9, SOCKADDR_SYSCOM_ANY_N, SOCKADDR_SYSCOM_ANY_N, 0);
			__attribute__((fallthrough));
		case 0:	iter->ssk = NULL;
			iter->how = 100;
			break;

		SL(11, iter->hdr->dst.nid, SOCKADDR_SYSCOM_ANY_N, 12);
		SL(12, SOCKADDR_SYSCOM_ANY_N, iter->hdr->dst.cpid, 9);
		goto state_9;

		SL(21, SOCKADDR_SYSCOM_ANY_N, iter->hdr->dst.cpid, 9);
		SL(22, iter->hdr->dst.nid, SOCKADDR_SYSCOM_ORPHANS_N, 8);
		goto state_8;

		SL(31, iter->hdr->dst.nid, SOCKADDR_SYSCOM_ANY_N, 9);
		SL(32, SOCKADDR_SYSCOM_ORPHANS_N, iter->hdr->dst.cpid, 33);
		SL(33, SOCKADDR_SYSCOM_ORPHANS_N, SOCKADDR_SYSCOM_ANY_N, 9);
		goto state_9;
	}
	if (iter->ssk) {
done:		sock_hold(&iter->ssk->sk);
	}
	rcu_read_unlock();

	if (old) {
		sock_put(&old->sk);
	}

	return iter->ssk;
}

static struct syscom_sock *syscom_iter_first(struct syscom_iter *iter,
		struct sk_buff *skb)
{
	iter->ssk = NULL;
	iter->how = 1;
	iter->skb = skb;
	iter->hdr = (struct syscom_hdr *)skb->data;
	return syscom_iter_next(iter);
}

#define FILTERED_MASK 0x8000

int syscom_queue_rcv_skb(struct sk_buff *skb, long timeo, bool bp)
{
	struct syscom_sock *ssk, *next;
	struct syscom_iter iter;
	int delivered = 0;
	int err;

	for (ssk = syscom_iter_first(&iter, skb); ssk;
			sock_put(&ssk->sk), ssk = next) {
		struct sock *sk = &ssk->sk;
		struct sk_buff_head *list;
		bool sigurg_send = false;
		struct sk_buff *cskb;
		unsigned long flags;
		ktime_t time;

		sock_hold(sk);
		next = syscom_iter_next(&iter);

		time = ktime_get();
		trace_syscom_sk_queue_recv_skb(ssk, skb);

		// Here is a race-condition - the limit can be overcome if more SKBs
		// are queued at the same time. This is not seen as a problem in the
		// mainline kernel, thus we behave the same here.
		while (unlikely(atomic_read(&sk->sk_rmem_alloc) >= sk->sk_rcvbuf)) {
			if (ssk_flag(ssk, SYSCOM_BACKPRESSURE) && bp) {
				if (!timeo) {
					err = -EWOULDBLOCK;
					trace_sock_rcvqueue_full(sk, skb);
					goto drop2;
				}

				if (skb->syscom_oob && !sigurg_send) {
					sk_send_sigurg(sk);
					sigurg_send = true;
				}

				timeo = wait_event_interruptible_timeout(ssk->loopback_waiters,
						atomic_read(&sk->sk_rmem_alloc) < sk->sk_rcvbuf, timeo);
				if (timeo == -ERESTARTSYS) {
					if (delivered) {
						// We do not restart to prevent double delivery
						atomic_inc(&sk->sk_drops); // TODO: increment also remaining receivers
						err = -EINTR;
					} else {
						err = -ERESTARTSYS;
					}
					trace_sock_rcvqueue_full(sk, skb);
					goto drop2;
				}
			} else {
				err = -ENOMEM;
				goto drop;
			}
		}

		if (unlikely(next)) {
			cskb = skb_clone(skb, timeo ? GFP_KERNEL : GFP_ATOMIC);
			if (!cskb) {
				err = -ENOMEM;
				goto drop;
			}
			cskb->syscom_oob = skb->syscom_oob; // syscom_oob is not cloned
			prefetch(next);
		} else {
			cskb = skb;
		}

		delivered++;

		if (sk_filter_trim_cap(sk, cskb, sizeof(struct syscom_hdr))) {
			consume_skb(cskb);
			continue;
		}

		cskb->dev = NULL;
		*(ktime_t*)cskb->cb = time;
		skb_set_owner_r(cskb, sk);

		/* we escape from rcu protected region, make sure we don't leak
		 * a norefcounted dst
		 */
		skb_dst_force(cskb);

		if (likely(!cskb->syscom_oob) || sock_flag(sk, SOCK_URGINLINE)) {
			list = &sk->sk_receive_queue;
		} else {
			list = &ssk->sk_oob_queue;
		}

		atomic_long_inc(&ssk->stat.rx_count);
		spin_lock_irqsave(&list->lock, flags);
		sock_skb_set_dropcount(sk, cskb);
		if (unlikely(cskb->syscom_oob)) {
			if (!sigurg_send) {
				sk_send_sigurg(sk);
			}
			__skb_queue_head(list, cskb);
		} else {
			__skb_queue_tail(list, cskb);
		}
		spin_unlock_irqrestore(&list->lock, flags);

		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_data_ready(sk);
	}

	if (!delivered) {
		consume_skb(skb);
		return -EHOSTUNREACH;
	}

	return 0;

drop:	atomic_inc(&ssk->sk.sk_drops);
	atomic_long_inc(&ssk->stat.rx_count);
drop2:	trace_syscom_sk_recv_drop(ssk, skb, err);
	sock_put(&ssk->sk);
	if (next) sock_put(&next->sk);
	kfree_skb(skb);
	return err;
}

void syscom_sock_remove(struct syscom_sock *ssk)
{
	mutex_lock(&syscom_sk_hash_lock);
	if (!hlist_unhashed(&ssk->unhashed)) {
		hlist_del_init_rcu(&ssk->unhashed);
	} else if (!hlist_unhashed(&ssk->robbed)) {
		hlist_del_init_rcu(&ssk->robbed);
	} else if (sk_hashed(&ssk->sk)) {
		// Move the next socket behind us, that enforces it won't be
		// missed by RCU iterators.
		struct syscom_sock *new_head = rcu_ssk_next_protected(ssk);
		if (new_head) {
			hlist_add_behind_rcu(&new_head->sk.sk_node, &ssk->sk.sk_node);
		}
		sk_del_node_init_rcu(&ssk->sk);
	} else {
		struct sock *sk;
		sk_for_each_rcu(sk, syscom_sock_hlist(ssk->local.nid, ssk->local.cpid)) {
			struct syscom_sock *prev = syscom_sk(sk);
			if (syscom_addr_eq(ssk->local, prev->local)) {
				while (rcu_ssk_next_protected(prev) != ssk) {
					prev = rcu_ssk_next_protected(prev);
				}
				rcu_assign_pointer(prev->next, ssk->next);
				sock_put(&ssk->sk);
				break;
			}
		}
	}
	mutex_unlock(&syscom_sk_hash_lock);
}

/***********************************************************************/

struct syscom_stats_s syscom_stats;

void _syscom_delivery_error(int rtn, void *hdrp, struct sk_buff *skb,
		const char *where, const char *file, int line)
{
	struct ratelimit_state *rs;

#define handle_err(counter) do { \
	static DEFINE_RATELIMIT_STATE(_rs, 10, 3); \
	atomic_inc(&syscom_stats.deliver_##counter); \
	rs = &_rs; } while(0); break

	switch (rtn) {
		case -EBADMSG:      handle_err(malformed);
		case -EILSEQ:       handle_err(out_of_order);
		case -ETIMEDOUT:    handle_err(timeout);
		case -EREMOTE:      handle_err(remote);
		case -ENETUNREACH:  handle_err(net_unreach);
		case -EHOSTUNREACH: handle_err(host_unreach);
		default:            handle_err(other_errors);
	}

	if (!__ratelimit(rs)) {
		return;
	}

	if (!where) {
		where = "?";
	}

	if (rtn == -EBADMSG) {
		char copy[32];
		pr_warn("Malformed message discarded on %s. Detected by %s:%d.\n",
				where, file, line);
		if (skb) {
			pr_warn("SKB len: %d, data_len: %d\n", skb->len,
					skb->data_len);
		}
		if (hdrp && -EFAULT != probe_kernel_read(copy, hdrp,
		    sizeof copy)) {
			print_hex_dump(KERN_INFO, pr_fmt("data: "),
					DUMP_PREFIX_OFFSET, 16, 1, copy, 32, 0);
		}
	} else if (hdrp) {
		struct syscom_hdr *hdr = hdrp;
		pr_warn("Message %04x of %uB from %04x:%04x to %04x:%04x ttl %u"
		        " not delivered on %s, error %d. Detected by %s:%d.\n",
				ntohs(hdr->msg_id), ntohs(hdr->length),
				ntohs(hdr->src.nid), ntohs(hdr->src.cpid),
				ntohs(hdr->dst.nid), ntohs(hdr->dst.cpid),
				hdr->ttl, where, rtn, file, line);
	} else {
		pr_warn("Message not delivered on %s, error %d. Detected by "
				"%s:%d.\n", where, rtn, file, line);
	}

	if (skb && skb->dev) {
		char hdr[32];
		int hdrlen = dev_parse_header(skb, hdr);
		BUG_ON(hdrlen > sizeof hdr);
		print_hex_dump(KERN_INFO, pr_fmt("hwaddr: "), DUMP_PREFIX_NONE,
				16, 1, hdr, hdrlen, 0);
	}
}

int syscom_forward __read_mostly;
int syscom_trace_sndbuf_skbs __read_mostly;
int syscom_gw_rx_timestamping __read_mostly;
static int syscom_raw_receives_forward __read_mostly = 1;
static unsigned syscom_oom_threshold __read_mostly = 1 << 20;
unsigned syscom_route_queue_bytes __read_mostly = 4 << 20;

static int syscom_rcv(struct sk_buff *skb, struct net_device *dev,
		struct packet_type *pt, struct net_device *orig_dev)
{
	// No need to check the return value, route code will discard
	// messages with fragmented header
	pskb_may_pull(skb, sizeof(struct syscom_hdr));
	return syscom_route_deliver_skb(skb) ? NET_RX_DROP : NET_RX_SUCCESS;
}

static struct packet_type syscom_packet_type __read_mostly = {
	.type = cpu_to_be16(ETH_P_SYSCOM),
	.func = syscom_rcv,
};

static struct sk_buff *syscom_frag_gso_segment(struct sk_buff *skb,
		netdev_features_t features)
{
	if (unlikely(skb_cloned(skb))) {
		// SKB is cloned - somebody creates ETH_P_ALL socket,
		// which receives data before a GSO is done. In that
		// case we have to copy the message here, which can hurt
		// performance or lead to a drop if we are low on DMA
		// memory. Emit a warning in that case to avoid somebody
		// doing it in the production.
		static DEFINE_RATELIMIT_STATE(rs, 60*HZ, 2);

		if (__ratelimit(&rs)) {
			pr_notice("Offloaded capture detected, "
					"performance degraded.\n");
		}

		skb = pskb_copy_for_clone(skb, GFP_ATOMIC);
		if (!skb) {
			pr_err("Capture prevented regular delivery\n");
			return NULL;
		}
	} else {
		skb = skb_get(skb);
	}

	skb->next = skb_shinfo(skb)->frag_list;
	skb_shinfo(skb)->frag_list = NULL;
	skb_shinfo(skb)->gso_segs = 0;
	skb_shinfo(skb)->gso_size = 0;
	skb_shinfo(skb)->gso_type = 0;

	return skb;
}

static struct packet_offload syscom_frag_offload __read_mostly = {
	.type = cpu_to_be16(ETH_P_SYSCOM_FRAG),
	.callbacks.gso_segment = syscom_frag_gso_segment,
};

/***********************************************************************/

#define REASSEMBLY_TOUT HZ

#define OUT_OF_ORDER_ERR "Out-of-order delivery detected: "

static LIST_HEAD(syscom_reassembly_list);
static DEFINE_SPINLOCK(syscom_reassembly_lock);

struct syscom_frag {
	/** Fragment hash */
	uint64_t hash;
	/** Reassembly list */
	struct list_head reassembly_list;
	/** When was the first fragment received */
	unsigned long tstamp;
	/** How much data was received */
	unsigned int received;
	/** How many fragments was received */
	uint8_t frags_received;
#ifdef CONFIG_SYSCOM_REASSEMBLY_REPAIR
	/** Fragment number */
	uint8_t frag_no;
	/** Received fragments for earlier detection of a problem */
	uint16_t frag_mask;
#endif
};

static inline uint64_t syscom_frag_hash(const struct syscom_frag_hdr *hdr)
{
	return (uint64_t)hdr->tid << 32 | (uint32_t)hdr->src << 16 | hdr->total_length;
}

static inline void syscom_frag_init(struct sk_buff *skb, uint64_t hash,
		uint16_t fraglen)
{
	struct syscom_frag *frag = (struct syscom_frag*)skb->cb;
	BUILD_BUG_ON(sizeof *frag > sizeof skb->cb);

	frag->hash = hash;
	frag->tstamp = jiffies;
	frag->received = fraglen;
	frag->frags_received = 1;
	skb_frag_list_init(skb);
	skb->protocol = htons(ETH_P_SYSCOM);
	list_add_tail(&frag->reassembly_list, &syscom_reassembly_list);
}

static inline struct syscom_frag *syscom_frag_find(uint64_t hash)
{
	struct syscom_frag *frag, *tmp;

	list_for_each_entry_safe(frag, tmp, &syscom_reassembly_list,
			reassembly_list) {
		if (frag->hash == hash) {
			return frag;
		}
		if (unlikely(jiffies - frag->tstamp > REASSEMBLY_TOUT)) {
			struct sk_buff *t;
			t = container_of((void*)frag, struct sk_buff, cb);
			list_del(&frag->reassembly_list);
			syscom_delivery_error_skb(-ETIMEDOUT, t->data, t,
					t->dev ? netdev_name(t->dev) : "ASM");
			kfree_skb(t);
			continue;
		}
	}
	return NULL;
}

#ifdef CONFIG_SYSCOM_REASSEMBLY_REPAIR
static inline int check_order(struct sk_buff *skb)
{
	int i;

	if (((struct syscom_frag *)skb->cb)->frag_no != 0) {
		return 1;
	}

	for (skb = skb_shinfo(skb)->frag_list, i = 1; skb; skb = skb->next, i++) {
		if (((struct syscom_frag *)skb->cb)->frag_no != i) {
			return 1;
		}
	}

	return 0;
}

/* This functions tries to fix order of fragmented messages.
 * For use with broken underlying layers */
static inline int _syscom_rcv_frag(struct sk_buff *skb, struct net_device *dev,
		const struct syscom_frag_hdr *hdr, uint16_t len, uint16_t fraglen)
{
	uint64_t hash = syscom_frag_hash(hdr);
	struct syscom_frag *frag;
	struct sk_buff *head;

restart:frag = syscom_frag_find(hash);
	if (!frag) {
		frag = (struct syscom_frag *)skb->cb;
		syscom_frag_init(skb, hash, fraglen);
		frag->frag_mask = 1 << hdr->frag;
		frag->frag_no = hdr->frag;
		return NET_RX_SUCCESS;
	}
	head = container_of((void*)frag, struct sk_buff, cb);

	if (unlikely(frag->frag_mask & (1 << hdr->frag))) {
		list_del(&frag->reassembly_list);
		syscom_delivery_error_skb(-EILSEQ, head->data, head, netdev_name(dev));
		kfree_skb(head);
		goto restart;
	}

	if (unlikely(frag->received + fraglen > len)) {
		list_del(&frag->reassembly_list);
		syscom_delivery_error_skb(-EILSEQ, head->data, head, netdev_name(dev));
		kfree_skb(head);
		goto drop;
	}

	if (!skb_has_frag_list(head)) {
		skb_shinfo(head)->frag_list = skb;
		skb->next = NULL;
	} else {
		struct sk_buff *tail;
		for (tail = skb_shinfo(head)->frag_list; tail->next; tail = tail->next);
		tail->next = skb;
		skb->next = NULL;
	}
	((struct syscom_frag*)skb->cb)->frag_no = hdr->frag;
	frag->received += fraglen;
	frag->frags_received++;
	frag->frag_mask |= 1 << hdr->frag;

	if (frag->received == len) {
		int n;
		list_del(&frag->reassembly_list);

		// Fix order
		if (unlikely(check_order(head))) {
			struct sk_buff *s[17] = { 0 };
			struct syscom_hdr *h;

			s[frag->frag_no] = head;
			for (skb = skb_shinfo(head)->frag_list; skb; skb = skb->next) {
				n = ((struct syscom_frag *)skb->cb)->frag_no;
				if (s[n]) {
					goto drop2;
				}
				s[n] = skb;
			}

			if (!s[0]) {
				goto drop2;
			}
			skb_shinfo(s[0])->frag_list = s[1];

			for (n = 1; n < frag->frags_received; n++) {
				if (!s[0]) {
					goto drop2;
				}
				s[n]->next = s[n+1];
				skb_shinfo(s[n])->frag_list = NULL;
			}

			head = s[0]; h = (struct syscom_hdr *)head->data;
			pr_debug_ratelimited("Message %04x of %uB from %04x:%04x "
				"to %04x:%04x arrived out-of-order. Fixing...\n",
				ntohs(h->msg_id), ntohs(h->length),
				ntohs(h->src.nid), ntohs(h->src.cpid),
				ntohs(h->dst.nid), ntohs(h->dst.cpid));
		}

		head->protocol = htons(ETH_P_SYSCOM);
		for (skb = skb_shinfo(head)->frag_list; skb; skb = skb->next) {
			head->truesize += skb->truesize;
			head->len += skb->len;
			head->data_len += skb->len;
		}

		if (syscom_route_deliver_skb(head)) {
			return NET_RX_DROP;
		}
	}

	return NET_RX_SUCCESS;

drop2:	pr_err("Broken fragmentation\n");
	skb = head;
drop:	kfree_skb(skb);
	return NET_RX_DROP;
}
#else
static inline int _syscom_rcv_frag(struct sk_buff *skb, struct net_device *dev,
		const struct syscom_frag_hdr *hdr, uint16_t len, uint16_t fraglen)
{
	uint64_t hash = syscom_frag_hash(hdr);
	struct syscom_frag *frag;
	struct sk_buff *head;
	int delta; bool stolen;

	if (hdr->frag == 0) {
		syscom_frag_init(skb, hash, fraglen);
		return NET_RX_SUCCESS;
	}

	frag = syscom_frag_find(hash);
	if (unlikely(!frag)) {
		pr_err_ratelimited(OUT_OF_ORDER_ERR
				"First received frag: %d\n", hdr->frag);
		syscom_delivery_error_skb(-EBADMSG, (void *)hdr, skb, netdev_name(dev));
		goto drop;
	}

	head = container_of((void*)frag, struct sk_buff, cb);
	if (unlikely(frag->received + fraglen > len) ||
	    unlikely(frag->frags_received != hdr->frag)) {
		list_del(&frag->reassembly_list);
		if (frag->frags_received != hdr->frag) {
			pr_err_ratelimited(OUT_OF_ORDER_ERR
					"Unexpected frag %d, expected %d\n",
					hdr->frag, frag->frags_received);
		}
		syscom_delivery_error_skb(-EILSEQ, head->data, head, netdev_name(dev));
		kfree_skb(head);
		goto drop;
	}

	if (skb_try_coalesce(head, skb, &stolen, &delta)) {
		kfree_skb_partial(skb, stolen);
	} else {
		if (!skb_has_frag_list(head)) {
			skb_shinfo(head)->frag_list = skb;
			skb->next = NULL;
		} else {
			struct sk_buff *tail;
			for (tail = skb_shinfo(head)->frag_list; tail->next; tail = tail->next);
			tail->next = skb;
			skb->next = NULL;
		}
		head->truesize += skb->truesize;
		head->len += skb->len;
		head->data_len += skb->len;
	}
	frag->received += fraglen;
	frag->frags_received++;

	if (frag->received == len) {
		list_del(&frag->reassembly_list);
		if (syscom_route_deliver_skb(head)) {
			return NET_RX_DROP;
		}
	}

	return NET_RX_SUCCESS;

drop:	kfree_skb(skb);
	return NET_RX_DROP;
}
#endif

static int syscom_rcv_frag(struct sk_buff *skb, struct net_device *dev,
		struct packet_type *pt, struct net_device *orig_dev)
{
	const struct syscom_frag_hdr *hdr = (struct syscom_frag_hdr *)skb->data;
	uint16_t len;
	uint16_t fraglen;
	int rtn;

	if (unlikely(sizeof *hdr > skb->len)) {
		syscom_delivery_error_skb(-EBADMSG, NULL, skb, dev->name);
		goto drop;
	}
	if (unlikely(!pskb_pull(skb, sizeof *hdr))) {
		syscom_delivery_error_skb(-ENOMEM, NULL, skb, dev->name);
		goto drop;
	}

	len = ntohs(hdr->total_length);
	fraglen = hdr->frag_len1 << 8 | hdr->frag_len2;
	if (unlikely(fraglen > skb->len)) {
		syscom_delivery_error_skb(-EBADMSG, skb->data, skb, dev->name);
		goto drop;
	} else if (fraglen < skb->len) {
		skb->len = fraglen;
	}

	if (likely(len == fraglen)) {
		if (unlikely(hdr->frag != 0)) {
			syscom_delivery_error_skb(-EBADMSG, (void *)hdr, skb, dev->name);
			goto drop;
		}
		skb->protocol = htons(ETH_P_SYSCOM);
		return syscom_rcv(skb, dev, &syscom_packet_type, orig_dev);
	} else if (unlikely(fraglen > len)) {
		syscom_delivery_error_skb(-EBADMSG, (void *)hdr, skb, dev->name);
		goto drop;
	}

	spin_lock(&syscom_reassembly_lock);
	rtn = _syscom_rcv_frag(skb, dev, hdr, len, fraglen);
	spin_unlock(&syscom_reassembly_lock);

	return rtn;

drop:	kfree_skb(skb);
	return NET_RX_DROP;
}

/** Periodically check queue for incomplete messages. This is not needed for
 * resource reclaim, but for earlier reporting of a problem - otherwise the
 * timeout would be reported when the next fragmented message is received.
 */
static void syscom_frag_gb(struct work_struct *work)
{
	struct syscom_frag *frag, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&syscom_reassembly_lock, flags);
	list_for_each_entry_safe(frag, tmp, &syscom_reassembly_list,
			reassembly_list) {
		if (unlikely(jiffies - frag->tstamp > REASSEMBLY_TOUT)) {
			struct sk_buff *t;
			t = container_of((void*)frag, struct sk_buff, cb);
			list_del(&frag->reassembly_list);
			syscom_delivery_error_skb(-ETIMEDOUT, t->data, t,
					t->dev ? netdev_name(t->dev) : "GB");
			kfree_skb(t);
		}
	}
	spin_unlock_irqrestore(&syscom_reassembly_lock, flags);

	if (work) {
		schedule_delayed_work(to_delayed_work(work), 5*HZ);
	}
}

static DECLARE_DEFERRABLE_WORK(syscom_frag_gb_work, syscom_frag_gb);

static struct packet_type syscom_frag_packet_type __read_mostly = {
	.type = cpu_to_be16(ETH_P_SYSCOM_FRAG),
	.func = syscom_rcv_frag,
};


/***********************************************************************/

static void syscom_write_space(struct sock *sk)
{
	struct socket_wq *wq;

	if (sock_writeable(sk)) {
		rcu_read_lock();
		wq = rcu_dereference(sk->sk_wq);
		if (skwq_has_sleeper(wq))
			wake_up_interruptible_sync(&wq->wait);
		rcu_read_unlock();
		sk_wake_async(sk, SOCK_WAKE_SPACE, POLL_OUT);
	}
}

static void syscom_sock_destructor(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);

	/* Returns sk_wmem_alloc minus initial offset of one */
	WARN_ON(sk_wmem_alloc_get(sk) >= 0);
	WARN_ON(sk_hashed(sk));
	WARN_ON(!hlist_unhashed(&syscom_sk(sk)->unhashed));
	WARN_ON(!hlist_unhashed(&syscom_sk(sk)->robbed));
	/*WARN_ON(sk->sk_socket);
	if (!sock_flag(sk, SOCK_DEAD)) {
		printk(KERN_INFO "Attempt to release alive unix socket: %p\n", sk);
		return;
	}*/

	local_bh_disable();
	sock_prot_inuse_add(sock_net(sk), sk->sk_prot, -1);
	local_bh_enable();
}

static struct proto syscom_proto = {
	.name      = "SYSCOM",
	.owner     = THIS_MODULE,
	.obj_size  = sizeof(struct syscom_sock),
};

static int syscom_create(struct net *net, struct socket *sock, int protocol, int kern)
{
	struct syscom_sock *ssk;
	struct sock *sk;

	if (protocol && protocol != PF_SYSCOM) {
		return -EPROTONOSUPPORT;
	}

	sock->state = SS_UNCONNECTED;

	switch (sock->type) {
		case SOCK_SEQPACKET:
		case SOCK_DGRAM:
			sock->ops = &syscom_dgram_ops;
			break;
		case SOCK_RAW:
			sock->ops = &syscom_raw_ops;
			break;
		default:
			return -ESOCKTNOSUPPORT;
	}

	/* Not kernel internal socket */
	sk = sk_alloc(net, PF_SYSCOM, GFP_KERNEL, &syscom_proto, kern);
	if (!sk) {
		return -ENOBUFS;
	}

	sock_init_data(sock, sk);
	ssk = syscom_sk(sk);

	init_waitqueue_head(&ssk->loopback_waiters);
	skb_queue_head_init(&ssk->sk_oob_queue);
	spin_lock_init(&ssk->skb_snd_list_lock);
	INIT_LIST_HEAD(&ssk->skb_snd_list);
	ssk->tstamp = -1;
	ssk->ordered = sock->type == SOCK_SEQPACKET;
	ssk->reliable = sock->type == SOCK_SEQPACKET;
	sk->sk_write_space = syscom_write_space;
	sk->sk_destruct = syscom_sock_destructor;
	sk->sk_state = SYSCOM_NEW;
	sock_set_flag(sk, SOCK_RCU_FREE);

	local_bh_disable();
	sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
	local_bh_enable();

	if (sock->type == SOCK_RAW) {
		syscom_raw_create(ssk);
	} else {
		mutex_lock(&syscom_sk_hash_lock);
		hlist_add_head_rcu(&ssk->unhashed, &unhashed_sockets);
		mutex_unlock(&syscom_sk_hash_lock);
	}

	return 0;
}

static struct net_proto_family syscom_family_ops = {
	.family = PF_SYSCOM,
	.create = syscom_create,
	.owner  = THIS_MODULE,
};

/*** Per net operations ************************************************/

struct socket_iter_state {
	struct seq_net_private p;
	struct syscom_sock *ssk_head;
	struct syscom_sock *ssk_reuse;
	int idx;
};

#define SYSCOM_ITER_RAW_INIT -4
#define SYSCOM_ITER_RAW_LOOP -3
#define SYSCOM_ITER_UNHASHED_INIT -2
#define SYSCOM_ITER_UNHASHED_LOOP -1
#define SYSCOM_ITER_HASHED_INIT 0
#define SYSCOM_ITER_ROBBED_INIT (INT_MAX - 1)
#define SYSCOM_ITER_ROBBED_LOOP INT_MAX

#define hlist_first_entry_rcu(head, type, member) \
	hlist_entry_safe(rcu_dereference_raw(hlist_first_rcu(head)), type, member)

#define hlist_next_entry_rcu(pos, member) \
	hlist_entry_safe(rcu_dereference_raw(hlist_next_rcu(&(pos)->member)), typeof(*(pos)), member)

static struct syscom_sock *socket_next(struct socket_iter_state *iter)
{
	int i;

rst:	switch (iter->idx) {
		case SYSCOM_ITER_RAW_LOOP:
			iter->ssk_head = hlist_next_entry_rcu(
					iter->ssk_head, sk.sk_node);
			break;
		case SYSCOM_ITER_UNHASHED_LOOP:
			iter->ssk_head = hlist_next_entry_rcu(
					iter->ssk_head, unhashed);
			break;
		case SYSCOM_ITER_RAW_INIT:
			iter->ssk_head = hlist_first_entry_rcu(&raw_sockets,
					struct syscom_sock, sk.sk_node);
			if (iter->ssk_head) {
				iter->idx = SYSCOM_ITER_RAW_LOOP;
				return iter->ssk_head;
			}
			/* Fall through */
		case SYSCOM_ITER_UNHASHED_INIT:
			iter->ssk_head = hlist_first_entry_rcu(&unhashed_sockets,
					struct syscom_sock, unhashed);
			if (iter->ssk_head) {
				iter->idx = SYSCOM_ITER_UNHASHED_LOOP;
				return iter->ssk_head;
			}
			iter->idx = SYSCOM_ITER_HASHED_INIT;
			/* Fall through */
		default:
			if (iter->ssk_reuse) {
				struct syscom_sock *rtn;
				rtn = iter->ssk_reuse;
				iter->ssk_reuse = rcu_dereference(iter->ssk_reuse->next);
				return rtn;
			}
			if (iter->ssk_head) {
				iter->ssk_head = hlist_next_entry_rcu(
						iter->ssk_head, sk.sk_node);
				if (iter->ssk_head) {
					goto nh;
				}
			}
			for (i = iter->idx; i < ARRAY_SIZE(syscom_sk_hash); i++) {
				iter->ssk_head = hlist_first_entry_rcu(&syscom_sk_hash[i],
						struct syscom_sock, sk.sk_node);
				if (iter->ssk_head) {
					iter->idx = i + 1;
					goto nh;
				}
			}
			/* Fall through */
		case SYSCOM_ITER_ROBBED_INIT:
			iter->ssk_reuse = NULL;
			iter->ssk_head = hlist_first_entry_rcu(&robbed_sockets,
					struct syscom_sock, robbed);
			iter->idx = SYSCOM_ITER_ROBBED_LOOP;
			return iter->ssk_head;
		case SYSCOM_ITER_ROBBED_LOOP:
			iter->ssk_head = hlist_next_entry_rcu(
					iter->ssk_head, robbed);
			return iter->ssk_head;
	}
	if (!iter->ssk_head) {
		iter->idx++;
		goto rst;
	}
	return iter->ssk_head;

nh:	iter->ssk_reuse = rcu_dereference(iter->ssk_head->next);
	return iter->ssk_head;
}

static struct syscom_sock *socket_first(struct socket_iter_state *iter)
{
	iter->ssk_head = NULL;
	iter->ssk_reuse = NULL;
	iter->idx = SYSCOM_ITER_RAW_INIT;

	return socket_next(iter);
}

static struct syscom_sock *socket_idx(struct seq_file *seq, loff_t pos)
{
	struct socket_iter_state *iter = seq->private;
	struct syscom_sock *s;
	loff_t off = 0;

	for (s = socket_first(iter); s; s = socket_next(iter)) {
		if (off == pos)
			return s;
		++off;
	}

	return NULL;
}

#ifdef CONFIG_PROC_FS
static void *syscom_seq_start(struct seq_file *seq, loff_t *pos)
{
	rcu_read_lock();
	return *pos ? socket_idx(seq, *pos - 1) : SEQ_START_TOKEN;
}

static void *syscom_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct socket_iter_state *iter = seq->private;

	++*pos;

	return v == SEQ_START_TOKEN ? socket_first(iter) : socket_next(iter);
}

static void syscom_seq_stop(struct seq_file *seq, void *v)
{
	rcu_read_unlock();
}

static inline void syscom_addr_format(char *buf, __be16 addr, bool right,
		int type, int state)
{
	if (type == SOCK_RAW) {
		strcpy(buf, right ? "   -" : "-   ");
	} else if (state == SYSCOM_NEW) {
		strcpy(buf, right ? "   ~" : "~   ");
	} else if (addr == SOCKADDR_SYSCOM_ANY_N) {
		strcpy(buf, right ? "   *" : "*   ");
	} else {
		sprintf(buf, "%04x", ntohs(addr));
	}
}

static int syscom_seq_show(struct seq_file *seq, void *v)
{
	struct syscom_sock *r = v;
	char addr[4][9];

	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, "local     remote    st fla max_rx_us cur_rx_q lim_rx_q max_tx_us cur_tx_q lim_tx_q    rx_count rx_drop    tx_count tx_drop inode\n");
		return 0;
	}

	syscom_addr_format(addr[0], r->local.nid, 1, r->sk.sk_type, r->sk.sk_state);
	syscom_addr_format(addr[1], r->local.cpid, 0, r->sk.sk_type, r->sk.sk_state);
	syscom_addr_format(addr[2], r->remote.nid, 1, r->sk.sk_type, r->sk.sk_state);
	syscom_addr_format(addr[3], r->remote.cpid, 0, r->sk.sk_type, r->sk.sk_state);

	seq_printf(seq, "%s:%s %s:%s %2d %c%c%c %9lu %8u %8u %9lu %8u %8u %11lu %7u %11lu %7u %08lx\n",
			addr[0], addr[1], addr[2], addr[3], r->sk.sk_state,
			ssk_flag(r, SYSCOM_BACKPRESSURE) ? 'b' : '-',
			r->reliable ? 'r' : '-', r->ordered ? 'o' : '-',
			atomic_long_read(&r->stat.max_rx_latency),
			sk_rmem_alloc_get(&r->sk), r->sk.sk_rcvbuf,
			atomic_long_read(&r->stat.max_tx_latency),
			sk_wmem_alloc_get(&r->sk), r->sk.sk_sndbuf,
			atomic_long_read(&r->stat.rx_count),
			atomic_read(&r->sk.sk_drops),
			atomic_long_read(&r->stat.tx_count),
			atomic_read(&r->stat.tx_drops),
			sock_i_ino(&r->sk));

	return 0;
}

static const struct seq_operations syscom_sec_ops = {
	.start  = syscom_seq_start,
	.next   = syscom_seq_next,
	.stop   = syscom_seq_stop,
	.show   = syscom_seq_show,
};

static int show_stat(struct seq_file *seq, void *v)
{
	seq_printf(seq, "route_tree_misses: %u\n"
			"route_tree_nodes: %u\n"
			"route_tree_optimizations: %u\n"
			"route_errors: %u\n"
			"route_queue_bytes: %u\n"
			"sock_rx_drops: %u\n"
			"sock_tx_drops: %u\n"
			"sock_max_rx_us: %lu\n"
			"sock_max_tx_us: %lu\n"
			"deliver_malformed: %u\n"
			"deliver_out_of_order: %u\n"
			"deliver_timeout: %u\n"
			"deliver_remote: %u\n"
			"deliver_net_unreach: %u\n"
			"deliver_host_unreach: %u\n"
			"deliver_other_errors: %u\n"
			"gw_rx_drops: %u\n"
			"gw_tx_drops: %u\n"
			"gw_errors: %u\n"
			"gw_ready_would_block: %u\n"
			"gw_rx_hw_us: %lu\n"
			"gw_rx_sw_us: %lu\n"
			"gw_rx_dl_us: %lu\n"
			"notify_errors: %u\n"
			"snd_trace: %u\n",
			atomic_read(&syscom_stats.route_tree_miss),
			atomic_read(&syscom_stats.route_tree_nodes) + 1,
			atomic_read(&syscom_stats.route_tree_optimizations),
			atomic_read(&syscom_stats.route_err),
			atomic_read(&syscom_stats.route_queue_bytes),
			atomic_read(&syscom_stats.sock_tx_drops),
			atomic_read(&syscom_stats.sock_rx_drops),
			atomic_long_read(&syscom_stats.sock_max_rx_latency),
			atomic_long_read(&syscom_stats.sock_max_tx_latency),
			atomic_read(&syscom_stats.deliver_malformed),
			atomic_read(&syscom_stats.deliver_out_of_order),
			atomic_read(&syscom_stats.deliver_timeout),
			atomic_read(&syscom_stats.deliver_remote),
			atomic_read(&syscom_stats.deliver_net_unreach),
			atomic_read(&syscom_stats.deliver_host_unreach),
			atomic_read(&syscom_stats.deliver_other_errors),
			atomic_read(&syscom_stats.gw_rx_drops),
			atomic_read(&syscom_stats.gw_tx_drops),
			atomic_read(&syscom_stats.gw_errors),
			atomic_read(&syscom_stats.gw_ready_would_block),
			atomic_long_read(&syscom_stats.gw_rx_hw_us),
			atomic_long_read(&syscom_stats.gw_rx_sw_us),
			atomic_long_read(&syscom_stats.gw_rx_dl_us),
			atomic_read(&syscom_stats.notify_errors),
			syscom_trace_sndbuf_skbs);
	return 0;
}

#endif

#ifdef CONFIG_SYSCTL
static int zero = 0;
static int one = 1;

static int syscom_gw_rx_timestamping_change(struct ctl_table *table, int write,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int rtn = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (write && !rtn) {
		rtn = syscom_gw_ts_change();
	}
	return rtn;
}

static struct ctl_table syscom_ctl_table[] = {
	{
		.procname       = "forward",
		.data           = &syscom_forward,
		.proc_handler   = proc_dointvec_minmax,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{
		.procname       = "raw_receives_forward",
		.data           = &syscom_raw_receives_forward,
		.proc_handler   = proc_dointvec_minmax,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{
		.procname       = "oom_threshold",
		.data           = &syscom_oom_threshold,
		.proc_handler   = proc_douintvec,
		.maxlen         = sizeof(unsigned),
		.mode           = 0644,
	},
	{
		.procname       = "trace_sndbuf_skbs",
		.data           = &syscom_trace_sndbuf_skbs,
		.proc_handler   = proc_dointvec_minmax,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{
		.procname       = "gw_rx_timestamping",
		.data           = &syscom_gw_rx_timestamping,
		.proc_handler   = syscom_gw_rx_timestamping_change,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{
		.procname       = "route_queue_bytes",
		.data           = &syscom_route_queue_bytes,
		.proc_handler   = proc_douintvec,
		.maxlen         = sizeof(unsigned),
		.mode           = 0644,
	},
	{ }
};
struct ctl_table_header *ctl_forward_hdr;
#endif

static int syscom_net_init(struct net *net)
{
#ifdef CONFIG_PROC_FS
	if (!proc_create_net("syscom", 0, net->proc_net, &syscom_sec_ops,
			     sizeof(struct socket_iter_state))) {
		return -EIO;
	}
	if (!proc_create_net("syscom_route", 0, net->proc_net,
			     &syscom_route_seq_ops,
			     sizeof(struct syscom_route_iter_state))) {
		goto err0;
	}
	if (!proc_create_net("syscom_gw", 0, net->proc_net, &syscom_gw_seq_ops,
			     sizeof(struct syscom_gw_iter_state))) {
		goto err1;
	}
	if (!proc_create_net_single("syscom_stat", 0, net->proc_net, show_stat,
				    NULL)) {
		goto err2;
	}
#endif

#ifdef CONFIG_SYSCTL
	if (net == &init_net) {
		ctl_forward_hdr = register_net_sysctl(net,
				"net/syscom", syscom_ctl_table);
		if (!ctl_forward_hdr) {
			goto err3;
		}
	}
#endif
	return 0;

err3:
#ifdef CONFIG_PROC_FS
	remove_proc_entry("syscom_stat", net->proc_net);
err2:	remove_proc_entry("syscom_gw", net->proc_net);
err1:	remove_proc_entry("syscom_route", net->proc_net);
err0:	remove_proc_entry("syscom", net->proc_net);
#endif
	return -EIO;
}

static void syscom_net_exit(struct net *net)
{
#ifdef CONFIG_PROC_FS
	remove_proc_entry("syscom", net->proc_net);
	remove_proc_entry("syscom_route", net->proc_net);
	remove_proc_entry("syscom_gw", net->proc_net);
	remove_proc_entry("syscom_stat", net->proc_net);
#endif
#ifdef CONFIG_SYSCTL
	if (net == &init_net) {
		unregister_net_sysctl_table(ctl_forward_hdr);
	}
#endif
}

static struct pernet_operations syscom_net_ops = {
	.init = syscom_net_init,
	.exit = syscom_net_exit,
};

/*** Out of memory handling ********************************************/
static int syscom_oom_notify(struct notifier_block *self,
                          unsigned long not_used, void *param)
{
	struct socket_iter_state iter;
	struct syscom_sock *ssk, *maxsk;
	unsigned long *freed = param;

	rcu_read_lock();
	for (maxsk = ssk = socket_first(&iter); ssk; ssk = socket_next(&iter)) {
		if (atomic_read(&maxsk->sk.sk_rmem_alloc) <
		    atomic_read(&ssk->sk.sk_rmem_alloc)) {
			maxsk = ssk;
		}
	}
	if (maxsk) {
		sock_hold(&maxsk->sk);
	}
	rcu_read_unlock();

	if (maxsk) {
		int rmem = atomic_read(&maxsk->sk.sk_rmem_alloc);
		if (rmem < syscom_oom_threshold) {
			pr_info("OOM handling skipped, largest queue size is %d"
					" bytes on %016lx.\n", rmem,
					sock_i_ino(&maxsk->sk));
		} else {
			struct sockaddr_syscom addr;
			struct kvec vec = { };
			struct msghdr msg = {
				.msg_name = &addr,
				.msg_namelen = sizeof addr,
			};

			pr_crit("OOM handler sacrifices messages on %016lx "
					"because its queue size is %d bytes.",
					sock_i_ino(&maxsk->sk), rmem);
			maxsk->sk.sk_rcvbuf = syscom_oom_threshold;

			while (0 == kernel_recvmsg(maxsk->sk.sk_socket, &msg, &vec,
					1, vec.iov_len, MSG_DONTWAIT | MSG_OOB) ||
			       0 == kernel_recvmsg(maxsk->sk.sk_socket, &msg, &vec,
					1, vec.iov_len, MSG_DONTWAIT)) {
				atomic_inc(&maxsk->sk.sk_drops);
				*freed += 1;
			}
		}
		sock_put(&maxsk->sk);
	}

	return NOTIFY_OK;
}

static struct notifier_block oom_notifier_block = {
	.notifier_call = syscom_oom_notify
};

/*** Module initialization and removal *********************************/

static int __init af_syscom_init(void)
{
	int rtn;

	struct sk_buff *dummy_skb;
	BUILD_BUG_ON(sizeof(struct syscom_frag) > sizeof(dummy_skb->cb));

	rtn = syscom_gw_init();
	if (rtn) {
		return rtn;
	}

	rtn = syscom_route_init();
	if (rtn) {
		goto err0;
	}

	rtn = proto_register(&syscom_proto, 1);
	if (rtn) {
		pr_crit("Cannot create syscom_sock SLAB cache!\n");
		goto err1;
	}

	dev_add_offload(&syscom_frag_offload);
	dev_add_pack(&syscom_packet_type);
	dev_add_pack(&syscom_frag_packet_type);
	sock_register(&syscom_family_ops);
	rtn = syscom_service_socket_init();
	if (rtn) {
		pr_crit("Cannot create service socket!\n");
		goto err2;
	}

	rtn = syscom_notify_init();
	if (rtn) {
		pr_crit("Cannot create notification socket!\n");
		goto err3;
	}

	rtn = register_oom_notifier(&oom_notifier_block);
	if (rtn) {
		pr_crit("Cannot register OOM notifier!\n");
		goto err4;
	}

	register_pernet_subsys(&syscom_net_ops);
	schedule_delayed_work(&syscom_frag_gb_work, HZ);
	return 0;

err4:	syscom_notify_destroy();
err3:	syscom_service_socket_destroy();
err2:	sock_unregister(PF_SYSCOM);
	dev_remove_pack(&syscom_frag_packet_type);
	dev_remove_pack(&syscom_packet_type);
	dev_remove_offload(&syscom_frag_offload);
	proto_unregister(&syscom_proto);
err1:	syscom_route_destroy();
err0:	syscom_gw_destroy();
	return rtn;
}

static void __exit af_syscom_exit(void)
{
	unregister_oom_notifier(&oom_notifier_block);
	cancel_delayed_work_sync(&syscom_frag_gb_work);
	unregister_pernet_subsys(&syscom_net_ops);
	syscom_notify_destroy();
	syscom_service_socket_destroy();
	sock_unregister(PF_SYSCOM);
	dev_remove_pack(&syscom_frag_packet_type);
	dev_remove_pack(&syscom_packet_type);
	dev_remove_offload(&syscom_frag_offload);
	proto_unregister(&syscom_proto);
	syscom_route_destroy();
	syscom_gw_destroy();
	syscom_frag_gb(NULL);
}


module_init(af_syscom_init);
module_exit(af_syscom_exit);

MODULE_ALIAS_NETPROTO(PF_SYSCOM);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Petr Malat");
