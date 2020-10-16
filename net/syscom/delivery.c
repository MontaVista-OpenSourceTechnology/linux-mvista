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

#include <linux/errqueue.h>
#include <linux/highmem.h>

#include "delivery.h"
#include "af.h"

#define SYSCOM_FRAG_LEN 4096
#define SYSCOM_FRAG_MAX 16

static inline int syscom_frag_tid(void)
{
#if NR_CPUS <= 8
	static DEFINE_PER_CPU(int, tid);
	int rtn = (get_cpu_var(tid)++ << 3) | smp_processor_id();
	put_cpu_var(tid);
	return rtn;
#else
	static atomic_t tid;
	return atomic_inc_return(&tid);
#endif
}

static void syscom_skb_init_gso(struct sk_buff *skb, int len, int mtu)
{
	if (len > mtu) {
		skb_frag_list_init(skb);
		skb_shinfo(skb)->gso_segs = (len + mtu - 1)/mtu;
		// We do not account fragmentation header into gso size as
		// we could overflow then. It's not a problem as the size
		// is used by qdisc, and it's up to us if we want to account
		// with or without the fragmentation header
		skb_shinfo(skb)->gso_size = len;
		skb_shinfo(skb)->gso_type = SKB_GSO_SYSCOM;
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int syscom_frag_skb_from_iter(struct iov_iter *iter, int headroom,
		int mtu, const struct syscom_hdr *syscom_hdr,
		syscom_skb_alloc alloc,	void *alloc_arg, struct sk_buff **head)
{
	struct syscom_hdr syscom_hdr_buf;
	struct syscom_frag_hdr hdr;
	struct sk_buff **nextp, *new;
	int len, frag_len, rtn;

	if (mtu > SYSCOM_FRAG_LEN) mtu = SYSCOM_FRAG_LEN;
	mtu -= sizeof hdr;

	if (!syscom_hdr) {
		copy_from_iter(&syscom_hdr_buf, sizeof syscom_hdr_buf, iter);
		syscom_hdr = &syscom_hdr_buf;
	}
	len = ntohs(syscom_hdr->length);

	if (unlikely(len > mtu * SYSCOM_FRAG_MAX)) {
		return -EMSGSIZE;
	}

	/* Fill the initial header */
	hdr.src = syscom_hdr->src.nid;
	hdr.total_length = syscom_hdr->length;
	hdr.reserved = 0;
	hdr.tid = len <= mtu ? 0 : syscom_frag_tid();

	nextp = head;
	for (hdr.frag = 0; len > 0; hdr.frag++, len -= mtu) {
		frag_len = min_t(int, len, mtu);
		*nextp = alloc(headroom + sizeof hdr + frag_len, 0, &rtn, alloc_arg);
		if (!*nextp) {
			goto clean;
		}

		new = *nextp;
		nextp = &new->next;
		*nextp = NULL;

		new->protocol = htons(ETH_P_SYSCOM_FRAG);
		new->ip_summed = CHECKSUM_UNNECESSARY;

		hdr.frag_len1 = frag_len >> 8;
		hdr.frag_len2 = frag_len & 0xff;

		skb_reserve(new, headroom);
		skb_reset_network_header(new);
		memcpy(__skb_put(new, sizeof hdr), &hdr, sizeof hdr);
		if (hdr.frag == 0) {
			syscom_skb_init_gso(new, len, mtu);
			nextp = &skb_shinfo(new)->frag_list;
			memcpy(__skb_put(new, frag_len), syscom_hdr,
					sizeof *syscom_hdr);
			frag_len -= sizeof *syscom_hdr;
			skb_copy_datagram_from_iter(new, sizeof *syscom_hdr +
					sizeof hdr, iter, frag_len);
		} else {
			__skb_put(new, frag_len);
			skb_copy_datagram_from_iter(new, sizeof hdr, iter,
					frag_len);
		}
	}

	return 0;

clean:	kfree_skb(*head);
	return rtn;
}

static int syscom_skb_from_iter(struct iov_iter *iter, int headroom,
		const struct syscom_hdr *hdr, syscom_skb_alloc alloc,
		void *alloc_arg, struct sk_buff **head)
{
	unsigned long data_len = iov_iter_count(iter) - (hdr ? 0 : sizeof *hdr);
	int rtn, linear, total_len = sizeof *hdr + data_len;
	struct sk_buff *skb;

	skb = alloc(headroom + sizeof *hdr, data_len, &rtn, alloc_arg);
	if (!skb) {
		return rtn;
	}

	skb_reserve(skb, headroom);
	linear = min_t(int, skb_availroom(skb), total_len);
	skb_reset_network_header(skb);
	skb_put(skb, linear);
	skb->len = total_len;
	skb->data_len = total_len - linear;
	skb->protocol = htons(ETH_P_SYSCOM);

	if (hdr) {
		memcpy(skb->data, hdr, sizeof *hdr);
		rtn = skb_copy_datagram_from_iter(skb, sizeof *hdr, iter, data_len);
	} else {
		rtn = skb_copy_datagram_from_iter(skb, 0, iter, total_len);
	}
	if (rtn) {
		goto err;
	}

	*head = skb;
	return 0;

err:	kfree_skb(skb);
	return rtn;
}

struct syscom_skb_trace_s {
	/** To queue on ssk->skb_snd_list */
	struct list_head list;
	/** When we were queued on "SNDBUF" */
	ktime_t time_queue;
	/** Original destructor */
	void (*orig_destructor)(struct sk_buff *skb);
	/** Traced SKB */
	struct sk_buff *skb;
	/** We use canary field to check if the head has been overwritten.
	 *  With tracing disabled we would crash in skb_under_panic in that
	 *  case. */
	unsigned canary;
};

#define syscom_skb_trace_size round_up(sizeof(struct syscom_skb_trace_s), 8)
#define syscom_skb_trace_canary 0xDEAD05CB

static void syscom_wfree(struct sk_buff *skb)
{
	struct syscom_sock *ssk = syscom_sk(skb->sk);
	struct syscom_skb_trace_s *s;
	unsigned long flags;
	long delta;

	s = (struct syscom_skb_trace_s *)skb->head;
	BUG_ON(s->canary != syscom_skb_trace_canary);

	spin_lock_irqsave(&ssk->skb_snd_list_lock, flags);
	delta = ktime_to_us(ktime_sub(ktime_get(), s->time_queue));
	if (delta > ssk->stat.max_tx_latency.counter) {
		atomic_long_set(&ssk->stat.max_tx_latency, delta);
	}
	list_del(&s->list);
	spin_unlock_irqrestore(&ssk->skb_snd_list_lock, flags);

	skb->destructor = s->orig_destructor;
	s->orig_destructor(skb);
}

void syscom_sock_dump_sndbuf(struct syscom_sock *ssk)
{
	struct syscom_skb_trace_s *s;
	ktime_t ktime_now = ktime_get();
	unsigned long flags, ino;

	ino = ssk->sk.sk_socket ? SOCK_INODE(ssk->sk.sk_socket)->i_ino : 0;
	if (ssk->sk.sk_family != PF_SYSCOM) {
		pr_warn("Ignoring non-syscom socket %d/%lx\n",
				ssk->sk.sk_family, ino);
		return;
	}

	spin_lock_irqsave(&ssk->skb_snd_list_lock, flags);

	if (list_empty_careful(&ssk->skb_snd_list)) goto unlock;

	pr_notice("Pending SKBs on socket %lx %04x:%04x->%04x:%04x:\n",
			ino, ssk->local.nid, ssk->local.cpid,
			ssk->remote.nid, ssk->remote.cpid);
	list_for_each_entry(s, &ssk->skb_snd_list, list) {
		pr_notice(" * SKB %p, pending for %ld us, proto: %x\n",
				s->skb, (long)ktime_to_us(ktime_sub(ktime_now,
				s->time_queue)), htons(s->skb->protocol));
		print_hex_dump(KERN_NOTICE, pr_fmt("   payload: "),
				DUMP_PREFIX_OFFSET, 16, 1, s->skb->data,
				min_t(int, skb_headlen(s->skb), 64), 0);
	}

unlock:	spin_unlock_irqrestore(&ssk->skb_snd_list_lock, flags);
}

struct syscom_sk_alloc_skb_s {
	struct sk_buff *parent_skb;
	struct sock *sk;
	unsigned done_trace:1;
	unsigned allocated:1;
	unsigned noblock:1;
};

static void syscom_tx_done(struct sk_buff *skb)
{
	struct sk_buff *parent_skb = skb_shinfo(skb)->destructor_arg;

	// TODO: Review locking here
	// Peeked is a bitfield thus we may race with writing to other
	// bitfields.

	if (skb->peeked) parent_skb->peeked = 1;

	if (parent_skb->peeked) {
		kfree_skb(parent_skb);
	} else {
		consume_skb(parent_skb);
	}

	skb->destructor = sock_wfree;
	sock_wfree(skb);
}

static void syscom_tx_done_parent(struct sk_buff *skb)
{
	struct syscom_sock *ssk = syscom_sk(skb->sk);

	BUG_ON(!skb->sk);

	if ((skb->peeked && (ssk->report_fail_reliable ||
			ssk->report_fail_unreliable)) ||
	    (!skb->peeked && (ssk->report_done_reliable ||
			ssk->report_done_unreliable))) {
		struct sk_buff *errskb = skb_clone(skb, GFP_ATOMIC);
		struct sock_exterr_skb *exterr;

		if (!errskb) {
			goto err;
		}

		if (errskb->protocol == htons(ETH_P_SYSCOM_FRAG)) {
			skb_pull(errskb, sizeof(struct syscom_frag_hdr));
			errskb->protocol = htons(ETH_P_SYSCOM);
		}

		exterr = SKB_EXT_ERR(errskb);
		memset(exterr, 0, sizeof *exterr);
		exterr->ee.ee_errno = skb->peeked ? EIO : 0;
		exterr->ee.ee_origin = SO_EE_ORIGIN_TXSTATUS;
		if (sock_queue_err_skb(skb->sk, errskb)) {
			kfree_skb(errskb);
			goto err;
		}
	}

wfree:	skb->destructor = sock_wfree;
	sock_wfree(skb);
	return;

err:	pr_info("Can't report delivery error on socket %04x:%04x\n",
			ntohs(ssk->local.nid), ntohs(ssk->local.cpid));
	goto wfree;
}

static struct sk_buff *syscom_sk_alloc_skb(unsigned head_len, unsigned data_len,
		int *err, void *argp)
{
	int trace = READ_ONCE(syscom_trace_sndbuf_skbs);
	struct syscom_sk_alloc_skb_s *arg = argp;
	struct syscom_skb_trace_s *s;
	struct sk_buff *skb;

	if (unlikely(trace)) {
		head_len += syscom_skb_trace_size;
	}

	if (head_len + data_len < SKB_MAX_ALLOC) {
		head_len += data_len;
		data_len = 0;
	}
	data_len = PAGE_ALIGN(data_len);

	if (!arg->allocated) {
		skb = sock_alloc_send_pskb(arg->sk, head_len, data_len,
				arg->noblock, err, PAGE_ALLOC_COSTLY_ORDER);
		arg->allocated = 1;
	} else {
		// We overcommit memory for all fragments accept the first
		skb = alloc_skb_with_frags(head_len, data_len,
				PAGE_ALLOC_COSTLY_ORDER, err,
				arg->sk->sk_allocation);
		if (skb) {
			skb_set_owner_w(skb, arg->sk);
		}
	}

	if (unlikely(!skb)) {
		return NULL;
	}

	BUG_ON(skb->destructor != sock_wfree);
	skb->peeked = 0;
	if (arg->parent_skb) {
		skb_shinfo(skb)->destructor_arg = skb_get(arg->parent_skb);
		skb->destructor = syscom_tx_done;
	} else if (arg->done_trace) {
		arg->parent_skb = skb;
		skb->destructor = syscom_tx_done_parent;
	}

	if (unlikely(trace)) {
		struct syscom_sock *ssk = syscom_sk(arg->sk);
		unsigned long flags;

		s = (struct syscom_skb_trace_s*)skb->head;
		skb_reserve(skb, syscom_skb_trace_size);
		s->skb = skb; s->time_queue = ktime_get();
		s->canary = syscom_skb_trace_canary;
		s->orig_destructor = skb->destructor;
		skb->destructor = syscom_wfree;
		spin_lock_irqsave(&ssk->skb_snd_list_lock, flags);
		list_add_tail(&s->list, &ssk->skb_snd_list);
		spin_unlock_irqrestore(&ssk->skb_snd_list_lock, flags);
	}

	return skb;
}

struct syscom_alloc_skb_s {
	bool noblock;
};

static struct sk_buff *syscom_alloc_skb(unsigned head_len, unsigned data_len,
		int *err, void *argp)
{
	struct syscom_alloc_skb_s *arg = argp;
	struct sk_buff *skb;

	skb = __dev_alloc_skb(head_len + data_len,
			arg->noblock ? GFP_ATOMIC : GFP_KERNEL);
	if (!skb) {
		*err = -ENOMEM;
	}

	return skb;
}

int __syscom_skb_from_iter(struct iov_iter *iter, int headroom,
		int mtu, const struct syscom_hdr *hdr, struct sock *sk,
		struct sk_buff **skb, bool noblock, bool fragment)
{
	union {
		struct syscom_sk_alloc_skb_s arg_sk;
		struct syscom_alloc_skb_s arg_nosk;
	} arg;
	syscom_skb_alloc alloc;

	if (sk) {
		bool reliabe = !(htons(SYSCOM_HDR_FLAG_UNRELIABLE) & hdr->flags);
		struct syscom_sock *ssk = syscom_sk(sk);

		arg.arg_sk.done_trace = (reliabe && (ssk->report_fail_reliable
						|| ssk->report_done_reliable)) ||
				(!reliabe && (ssk->report_fail_unreliable
						|| ssk->report_done_unreliable));
		arg.arg_sk.sk = sk;
		arg.arg_sk.noblock = noblock;
		arg.arg_sk.allocated = 0;
		arg.arg_sk.parent_skb = NULL;
		alloc = syscom_sk_alloc_skb;
	} else {
		arg.arg_nosk.noblock = noblock;
		alloc = syscom_alloc_skb;
	}

	return fragment ?
		syscom_frag_skb_from_iter(iter, headroom, mtu, hdr,
		                          alloc, &arg, skb) :
		syscom_skb_from_iter(iter, headroom, hdr, alloc, &arg, skb);
}


static int syscom_frag_skb_from_skb(struct sk_buff *src, int headroom, int mtu,
		gfp_t gfp_mask, struct sk_buff **head)
{
	struct syscom_hdr *syscom_hdr = (struct syscom_hdr *)src->data;
	int len = ntohs(syscom_hdr->length), off, rtn;
	struct sk_buff **nextp, *new;
	struct syscom_frag_hdr hdr;

	if (mtu > SYSCOM_FRAG_LEN) mtu = SYSCOM_FRAG_LEN;
	mtu -= sizeof hdr;

	if (unlikely(len > mtu * SYSCOM_FRAG_MAX)) {
		return -EMSGSIZE;
	}

	/* Fill the initial header */
	hdr.src = syscom_hdr->src.nid;
	hdr.total_length = syscom_hdr->length;
	hdr.reserved = 0;
	hdr.tid = len <= mtu ? 0 : syscom_frag_tid();

	nextp = head;
	for (hdr.frag = 0, off = 0; off < len; hdr.frag++, off += mtu) {
		int frag_len = min_t(int, len - off, mtu);
		*nextp = __dev_alloc_skb(headroom + sizeof hdr + frag_len,
				gfp_mask);
		if (!*nextp) {
			rtn = -ENOMEM;
			goto clean;
		}

		new = *nextp;
		nextp = &new->next;
		*nextp = NULL;

		new->protocol = htons(ETH_P_SYSCOM_FRAG);
		new->priority = src->priority;
		new->syscom_oob = src->syscom_oob;
		new->ip_summed = CHECKSUM_UNNECESSARY;

		hdr.frag_len1 = frag_len >> 8;
		hdr.frag_len2 = frag_len & 0xff;

		skb_reserve(new, headroom);
		skb_reset_network_header(new);
		memcpy(skb_put(new, sizeof hdr), &hdr, sizeof hdr);
		rtn = skb_copy_bits(src, off, skb_put(new, frag_len), frag_len);
		if (rtn) {
			goto clean;
		}

		if (hdr.frag == 0) {
			syscom_skb_init_gso(new, len, mtu);
			nextp = &skb_shinfo(new)->frag_list;
		}
	}

	return 0;

clean:	kfree_skb(*head);
	return rtn;
}

/***** Raw socket support **********************************************/

void syscom_delivery_raw_from_iov(struct syscom_delivery *d,
		struct iov_iter *i)
{
	struct sk_buff *skb = __dev_alloc_skb(d->msg_size, d->gfp_mask);
	struct iov_iter iov = *i;

	if (skb) {
		skb->protocol = htons(ETH_P_SYSCOM);
		skb_reset_network_header(skb);
		skb->len = d->msg_size;
		if (!skb_copy_datagram_from_iter(skb, 0, &iov, d->msg_size)) {
			d->raw_skb = skb;
		} else {
			kfree_skb(skb);
		}
	}
}

void syscom_delivery_raw_from_skb(struct syscom_delivery *d,
		struct sk_buff *skb)
{
	if (skb->protocol == htons(ETH_P_SYSCOM)) {
		d->raw_skb = skb_clone(skb, d->gfp_mask);
	} else {
		d->raw_skb = NULL;
		d->ops->get_skb(d, &d->raw_skb, 0, SYSCOM_MAX_MSGSIZE);
	}
}

/***** Delivery worker *************************************************/

struct delivery_work_struct {
	struct work_struct work;
	struct sk_buff *skb;
	const char *name;
};

int _syscom_route_deliver_skb(struct sk_buff *skb, long timeo, gfp_t gfp_mask,
		bool forward, const char *name);

static void syscom_delivery_worker(struct work_struct *work)
{
	struct delivery_work_struct *dw = (struct delivery_work_struct *)work;
	int truesize = dw->skb->truesize;

	_syscom_route_deliver_skb(dw->skb, HZ/10, GFP_KERNEL,
			syscom_forward, dw->name);

	atomic_sub(truesize, &syscom_stats.route_queue_bytes);
}

struct work_struct *syscom_delivery_get_work(struct syscom_delivery *d,
		const char *name)
{
	struct delivery_work_struct *dw;

	if (syscom_route_queue_bytes <
			atomic_read(&syscom_stats.route_queue_bytes)) {
		return NULL;
	}

	dw = kmalloc(sizeof *dw, d->gfp_mask);
	if (!dw) {
		return NULL;
	}

	if (syscom_delivery_get_skb(d, &dw->skb, NULL, NULL, 0,
			SYSCOM_MAX_MSGSIZE)) {
		kfree(dw);
		return NULL;
	}

	atomic_add(dw->skb->truesize, &syscom_stats.route_queue_bytes);

	dw->name = name;
	INIT_WORK(&dw->work, syscom_delivery_worker);
	return &dw->work;
}

/***** Per delivery type methods ***************************************/

static int syscom_delivery_buf_get_iov(struct syscom_delivery *d,
		struct iov_iter *i)
{
	struct syscom_delivery_buf *dbuf = (struct syscom_delivery_buf*)d;
	iov_iter_kvec(i, READ | ITER_KVEC, &dbuf->kvec, 1, dbuf->kvec.iov_len);
	return 0;
}

static int syscom_delivery_buf_get_skb(struct syscom_delivery *d,
		struct sk_buff **skb, int headroom, int mtu)
{
	struct syscom_delivery_buf *dbuf = (struct syscom_delivery_buf*)d;
	struct iov_iter i;

	iov_iter_kvec(&i, READ | ITER_KVEC, &dbuf->kvec, 1, dbuf->kvec.iov_len);

	return __syscom_skb_from_iter(&i, headroom, mtu, NULL, NULL, skb,
			d->timeo == 0, d->msg_size > d->frag_threshold);
}

const struct syscom_delivery_ops syscom_delivery_buf_ops = {
	.get_iov = syscom_delivery_buf_get_iov,
	.get_skb = syscom_delivery_buf_get_skb,
};

static int syscom_delivery_skb_get_iov(struct syscom_delivery *d,
		struct iov_iter *i)
{
	struct syscom_delivery_skb *dskb = (struct syscom_delivery_skb*)d;
	int nmemb = 1;
	int rtn = 0;

	if (skb_shinfo(dskb->skb)->nr_frags) {
		if (skb_has_frag_list(dskb->skb) ||
				syscom_delivery_is_atomic(&dskb->super)) {
			rtn = skb_linearize(dskb->skb);
		} else {
			int j;
			for (j = 0; j < skb_shinfo(dskb->skb)->nr_frags; j++) {
				const skb_frag_t *frag = &skb_shinfo(dskb->skb)->frags[j];
				dskb->skb_kvec[nmemb].iov_base = kmap(skb_frag_page(frag));
				if (!dskb->skb_kvec[nmemb].iov_base) {
					while (--j >= 0) kunmap(skb_frag_page(&skb_shinfo(dskb->skb)->frags[j]));
					return -EIO;
				}
				dskb->skb_kvec[nmemb].iov_base += frag->page_offset;
				dskb->skb_kvec[nmemb++].iov_len = skb_frag_size(frag);
			}
			dskb->mapped = 1;
		}
	} else {
		struct sk_buff *iter;
		skb_walk_frags(dskb->skb, iter) {
			if (skb_shinfo(iter)->nr_frags) {
				rtn = skb_linearize(dskb->skb);
				nmemb = 1;
				break;
			}
			BUG_ON(nmemb >= ARRAY_SIZE(dskb->skb_kvec));
			dskb->skb_kvec[nmemb].iov_base = iter->data;
			dskb->skb_kvec[nmemb++].iov_len = iter->len;
		}
	}

	if (rtn) {
		return rtn;
	}

	dskb->skb_kvec[0].iov_base = dskb->skb->data;
	dskb->skb_kvec[0].iov_len = skb_headlen(dskb->skb);
	iov_iter_kvec(i, READ | ITER_KVEC, dskb->skb_kvec, nmemb, dskb->skb->len);

	return 0;
}

static int syscom_delivery_skb_get_skb(struct syscom_delivery *d,
		struct sk_buff **skb, int headroom, int mtu)
{
	struct syscom_delivery_skb *dskb = (struct syscom_delivery_skb*)d;
	struct syscom_hdr *hdr = (struct syscom_hdr *)dskb->skb->data;
	int rtn;

	if (ntohs(hdr->length) >= d->frag_threshold) {
		rtn = syscom_frag_skb_from_skb(dskb->skb, headroom, mtu,
				d->gfp_mask, skb);
		return rtn;
	}

	if (unlikely(headroom > skb_headroom(dskb->skb))) {
		WARN(1, "Increase headroom msg_id: %d src: "
				"%04x:%04x dst: %04x:%04x",
				ntohs(hdr->msg_id),
				ntohs(hdr->src.nid), ntohs(hdr->src.cpid),
				ntohs(hdr->dst.nid), ntohs(hdr->dst.cpid));
		rtn = pskb_expand_head(dskb->skb,
				headroom - skb_headroom(dskb->skb), 0,
				d->gfp_mask);
		if (rtn) {
			return rtn;
		}
	}

	*skb = dskb->skb;
	dskb->skb = NULL;
	return 0;
}

const struct syscom_delivery_ops syscom_delivery_skb_ops = {
	.get_iov = syscom_delivery_skb_get_iov,
	.get_skb = syscom_delivery_skb_get_skb,
};

static int syscom_delivery_lookup_msg_addr(__be16 *addr, int what,
		struct msghdr *msg, __be16 def)
{
	struct cmsghdr *cmsg;

	for (cmsg = CMSG_FIRSTHDR(msg); cmsg;
			cmsg = CMSG_NXTHDR(msg, cmsg)) {
		if (unlikely(!CMSG_OK(msg, cmsg))) {
			return -EINVAL;
		}

		if (cmsg->cmsg_level != SOL_SYSCOM) {
			continue;
		}

		if (cmsg->cmsg_type != what) {
			continue;
		}

		if (unlikely(cmsg->cmsg_len < CMSG_LEN(sizeof *addr))) {
			return -EINVAL;
		}

		memcpy(addr, CMSG_DATA(cmsg), sizeof *addr);
		if (unlikely(*addr == SOCKADDR_SYSCOM_ORPHANS_N) ||
		    unlikely(*addr == SOCKADDR_SYSCOM_ANY_N)) {
			return -EINVAL;
		}
		return 0;
	}

	if (def != SOCKADDR_SYSCOM_ORPHANS_N &&
	    def != SOCKADDR_SYSCOM_ANY_N) {
		*addr = def;
		return 0;
	}

	return -EADDRNOTAVAIL;
}

#define fixup_nid_cpid(addr, what, msg, def) do { \
	if (addr == SOCKADDR_SYSCOM_ORPHANS_N || \
	    addr == SOCKADDR_SYSCOM_ANY_N) { \
		rtn = syscom_delivery_lookup_msg_addr(&addr, what, msg, def); \
		if (rtn) goto err0; \
	} \
} while (0)

static int syscom_delivery_msg_get_iov(struct syscom_delivery *d,
		struct iov_iter *i)
{
	struct syscom_delivery_msg *dmsg = (struct syscom_delivery_msg*)d;
	struct iovec iovec_iter;
	struct iov_iter iov;
	int rtn, idx = 1;

	rtn = iov_iter_fault_in_readable(&dmsg->msg->msg_iter,
			d->msg_size - sizeof dmsg->hdr);
	if (rtn) { // Fail on wrong addresses
		return rtn;
	}

	// TODO: We should verify the msg_iter type, unfortunately
	// that's not possible on 4.4
	if (dmsg->msg->msg_iter.nr_segs + 1 >= ARRAY_SIZE(dmsg->iovec)) {
		return -EIO; // Replace with kmalloc if needed
	}
	fixup_nid_cpid(dmsg->hdr.src.nid, SYSCOM_SO_ADDR_NID, dmsg->msg,
			dmsg->super.src_nid);
	fixup_nid_cpid(dmsg->hdr.src.cpid, SYSCOM_SO_ADDR_CPID, dmsg->msg,
			SOCKADDR_SYSCOM_ANY_N);

	dmsg->iovec[0].iov_base = &dmsg->hdr;
	dmsg->iovec[0].iov_len = sizeof dmsg->hdr;
	iov_for_each(iovec_iter, iov, dmsg->msg->msg_iter) {
		dmsg->iovec[idx++] = iovec_iter;
	}
	iov_iter_init(i, READ | ITER_IOVEC, dmsg->iovec, idx, d->msg_size);
	set_fs(KERNEL_DS); // syscom_route_deliver will revert this

err0:	return rtn;
}

static int syscom_delivery_msg_get_skb(struct syscom_delivery *d,
		struct sk_buff **skb, int headroom, int mtu)
{
	struct syscom_delivery_msg *dmsg = (struct syscom_delivery_msg*)d;
	int rtn;

	fixup_nid_cpid(dmsg->hdr.src.nid, SYSCOM_SO_ADDR_NID,
			dmsg->msg, dmsg->super.src_nid);
	fixup_nid_cpid(dmsg->hdr.src.cpid, SYSCOM_SO_ADDR_CPID,
			dmsg->msg, SOCKADDR_SYSCOM_ANY_N);

	rtn = __syscom_skb_from_iter(&dmsg->msg->msg_iter, headroom, mtu,
			&dmsg->hdr, &dmsg->src->sk, skb, d->timeo == 0,
			d->msg_size > d->frag_threshold);

	if (likely(!rtn)) {
		(*skb)->priority = dmsg->src->sk.sk_priority;
		(*skb)->syscom_oob = dmsg->msg->msg_flags & MSG_OOB ? 1 : 0;
	}

err0:	return rtn;
}

const struct syscom_delivery_ops syscom_delivery_msg_ops = {
	.get_iov = syscom_delivery_msg_get_iov,
	.get_skb = syscom_delivery_msg_get_skb,
};

static int syscom_delivery_rawmsg_get_iov(struct syscom_delivery *d,
		struct iov_iter *i)
{
	struct syscom_delivery_msg *dmsg = (struct syscom_delivery_msg*)d;
	*i = dmsg->msg->msg_iter;
	iov_iter_revert(i, sizeof dmsg->hdr);
	return 0;
}

static int syscom_delivery_rawmsg_get_skb(struct syscom_delivery *d,
		struct sk_buff **skb, int headroom, int mtu)
{
	struct syscom_delivery_msg *dmsg = (struct syscom_delivery_msg*)d;
	int rtn;

	rtn = __syscom_skb_from_iter(&dmsg->msg->msg_iter, headroom, mtu,
			&dmsg->hdr, &dmsg->src->sk, skb, d->timeo == 0,
			d->msg_size > d->frag_threshold);

	if (likely(!rtn)) {
		(*skb)->priority = dmsg->src->sk.sk_priority;
		(*skb)->syscom_oob = dmsg->msg->msg_flags & MSG_OOB ? 1 : 0;
	}

	return rtn;
}

const struct syscom_delivery_ops syscom_delivery_rawmsg_ops = {
	.get_iov = syscom_delivery_rawmsg_get_iov,
	.get_skb = syscom_delivery_rawmsg_get_skb,
};
