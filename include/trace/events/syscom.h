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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM syscom

#ifndef TRACE_SYSCOM_H

#include <linux/netdevice.h>
#include <linux/tracepoint.h>

struct syscom_service_msg_conf_op_hdr_s;
struct syscom_service_msg_conf_s;
struct syscom_delivery;
struct syscom_sock;
struct syscom_hdr;
struct syscom_gw;

/** Part of the header we are interested in */
struct syscom_trace_hdr {
	__be16 msg_id;
	struct {
		__be16 nid;
		__be16 cpid;
	} dst;
	struct {
		__be16 nid;
		__be16 cpid;
	} src;
	__be16 length;
};

#define HDR_FIELDS \
	__field(__be16, msg_id) \
	__field(__be16, dst_nid) \
	__field(__be16, dst_cpid) \
	__field(__be16, src_nid) \
	__field(__be16, src_cpid) \
	__field(__be16, length)

#define HDR_ASSIGN(hdr) \
	memcpy(&__entry->msg_id, &((hdr)->msg_id), \
		sizeof(struct syscom_trace_hdr))

#define HDR_ASSIGN_NULL(hdr) \
	((hdr) ? memcpy(&__entry->msg_id, &((hdr)->msg_id), \
		sizeof(struct syscom_trace_hdr)) : \
	      memset(&__entry->msg_id, 0xff, \
		sizeof(struct syscom_trace_hdr)))

#define HDR_FMT "msg_id=%d src=%04x:%04x dst=%04x:%04x len=%d"

#define HDR_FMT_ARG \
	ntohs(__entry->msg_id), \
	ntohs(__entry->src_nid), \
	ntohs(__entry->src_cpid), \
	ntohs(__entry->dst_nid), \
	ntohs(__entry->dst_cpid), \
	ntohs(__entry->length)


#define SKB_FIELDS \
	HDR_FIELDS \
	__field(void *, shinfo)

#define SKB_ASSIGN(skb) \
	HDR_ASSIGN((struct syscom_hdr*)(skb)->data), \
	__entry->shinfo = skb_shinfo(skb)

#define SKB_ASSIGN_ANY(skb) \
	HDR_ASSIGN((struct syscom_hdr*)((skb)->data + \
		((skb)->protocol == htons(ETH_P_SYSCOM) ? 0 : \
			sizeof(struct syscom_frag_hdr)))), \
	__entry->shinfo = skb_shinfo(skb)

#define SKB_FMT HDR_FMT " shinfo=%p"

#define SKB_FMT_ARG HDR_FMT_ARG, __entry->shinfo


#define GW_FIELDS \
	__array(char, gw_name, 20)

#define GW_ASSIGN(gw) \
	strlcpy(__entry->gw_name, gw->name, sizeof __entry->gw_name)

#define GW_FMT "gw=%s"

#define GW_FMT_ARG __entry->gw_name


#define SSK_FIELDS \
	__field(ino_t, ino) \
	__field(int, rmem_alloc) \
	__field(int, wmem_alloc)

#define SSK_ASSIGN(ssk) \
	__entry->ino = ssk->sk.sk_socket ? SOCK_INODE(ssk->sk.sk_socket)->i_ino : 0, \
	__entry->rmem_alloc = sk_rmem_alloc_get(&ssk->sk), \
	__entry->wmem_alloc = sk_wmem_alloc_get(&ssk->sk)

#define SSK_FMT "sk_ino=0x%lx rmem_alloc=%d wmem_alloc=%d"

#define SSK_FMT_ARG \
	(unsigned long)__entry->ino, \
	__entry->rmem_alloc, \
	__entry->wmem_alloc

enum trace_syscom_gw_latency_type {
	SYSCOM_GW_LATENCY_RX_SW,
	SYSCOM_GW_LATENCY_RX_HW,
	SYSCOM_GW_LATENCY_RX_DL,
};

#endif // TRACE_SYSCOM_H

/* Trace socket *****************************************************/
#if !defined(TRACE_SYSCOM_H) || defined(TRACE_HEADER_MULTI_READ)
#define TRACE_SYSCOM_H

TRACE_EVENT(syscom_sk_send_err,
	TP_PROTO(struct syscom_sock *ssk, int rtn),

	TP_ARGS(ssk, rtn),

	TP_STRUCT__entry(
		SSK_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		SSK_ASSIGN(ssk);
		__entry->rtn = rtn;
	),

	TP_printk(SSK_FMT " rtn=%d", SSK_FMT_ARG, __entry->rtn)
);

TRACE_EVENT(syscom_sk_send_drop,
	TP_PROTO(struct syscom_sock *ssk),

	TP_ARGS(ssk),

	TP_STRUCT__entry(
		SSK_FIELDS
	),

	TP_fast_assign(
		SSK_ASSIGN(ssk);
	),

	TP_printk(SSK_FMT, SSK_FMT_ARG)
);

TRACE_EVENT(syscom_sk_recv_err,
	TP_PROTO(struct syscom_sock *ssk, struct syscom_hdr *hdr, int rtn),

	TP_ARGS(ssk, hdr, rtn),

	TP_STRUCT__entry(
		HDR_FIELDS
		SSK_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		HDR_ASSIGN_NULL(hdr);
		SSK_ASSIGN(ssk);
		__entry->rtn = rtn;
	),

	TP_printk(HDR_FMT " " SSK_FMT " rtn=%d", HDR_FMT_ARG, SSK_FMT_ARG,
		__entry->rtn)
);

TRACE_EVENT(syscom_sk_recv_drop,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb, int rtn),

	TP_ARGS(ssk, skb, rtn),

	TP_STRUCT__entry(
		SKB_FIELDS
		SSK_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		SKB_ASSIGN(skb);
		SSK_ASSIGN(ssk);
		__entry->rtn = rtn;
	),

	TP_printk(SKB_FMT " " SSK_FMT " rtn=%d", SKB_FMT_ARG, SSK_FMT_ARG,
		__entry->rtn)
);

TRACE_EVENT(syscom_sk_recv_latency_grow,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb, unsigned long latency),

	TP_ARGS(ssk, skb, latency),

	TP_STRUCT__entry(
		SKB_FIELDS
		SSK_FIELDS
		__field(unsigned long, latency)
	),

	TP_fast_assign(
		SKB_ASSIGN(skb);
		SSK_ASSIGN(ssk);
		__entry->latency = latency;
	),

	TP_printk(SKB_FMT " " SSK_FMT " latency=%lu", SKB_FMT_ARG, SSK_FMT_ARG,
		__entry->latency)
);

TRACE_EVENT(syscom_sk_queue_recv_skb,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb),

	TP_ARGS(ssk, skb),

	TP_STRUCT__entry(
		SKB_FIELDS
		SSK_FIELDS
		__field(int, ifindex)
	),

	TP_fast_assign(
		SKB_ASSIGN(skb);
		SSK_ASSIGN(ssk);
		__entry->ifindex = skb->dev ? skb->dev->ifindex : -1;
	),

	TP_printk(SKB_FMT " " SSK_FMT " dev=%d", SKB_FMT_ARG, SSK_FMT_ARG,
		__entry->ifindex)
);

TRACE_EVENT(syscom_sk_tx_done,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb),

	TP_ARGS(ssk, skb),

	TP_STRUCT__entry(
		SKB_FIELDS
		SSK_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		SKB_ASSIGN_ANY(skb);
		SSK_ASSIGN(ssk);
		__entry->rtn = skb->peeked ? -EIO : 0;
	),

	TP_printk(SKB_FMT " " SSK_FMT " rtn=%d", SKB_FMT_ARG, SSK_FMT_ARG,
		__entry->rtn)
);

/* Trace service ****************************************************/

TRACE_EVENT(syscom_service_loop_start,
	TP_PROTO(struct syscom_hdr *hdr, struct sockaddr_syscom *src_addr),

	TP_ARGS(hdr, src_addr),

	TP_STRUCT__entry(
		__field(__be16, msg_id)
		__field(__be16, src_nid)
		__field(__be16, src_cpid)
	),

	TP_fast_assign(
		__entry->msg_id = hdr->msg_id;
		__entry->src_nid = src_addr->nid;
		__entry->src_cpid = src_addr->cpid;
	),

	TP_printk("msg_id=%d src=%04x:%04x", ntohs(__entry->msg_id),
		ntohs(__entry->src_nid), ntohs(__entry->src_cpid))
);

TRACE_EVENT(syscom_service_loop_done,
	TP_PROTO(int send_return, int processing_err),

	TP_ARGS(send_return, processing_err),

	TP_STRUCT__entry(
		__field(int, send_return)
		__field(int, processing_err)
	),

	TP_fast_assign(
		__entry->send_return = send_return;
		__entry->processing_err = processing_err;
	),

	TP_printk("send=%d processing=%d",
		__entry->send_return, __entry->processing_err)
);

TRACE_EVENT(syscom_service_conf,
	TP_PROTO(struct syscom_service_msg_conf_s *msg),

	TP_ARGS(msg),

	TP_STRUCT__entry(
		__field(__be64, seq)
		__field(__be64, timeout)
	),

	TP_fast_assign(
		__entry->seq = msg->seq;
		__entry->timeout = msg->timeout;
	),

	TP_printk("seq=%llu timeout=0x%llx",
		(long long unsigned)be64_to_cpu(__entry->seq),
		(long long unsigned)be64_to_cpu(__entry->timeout))
);

TRACE_EVENT(syscom_service_conf_op_start,
	TP_PROTO(struct syscom_service_msg_conf_op_hdr_s *hdr),

	TP_ARGS(hdr),

	TP_STRUCT__entry(
		__field(__be16, op_id)
		__field(__be16, len)
	),

	TP_fast_assign(
		__entry->op_id = hdr->op_id;
		__entry->len = hdr->len;
	),

	TP_printk("op_id=0x%x len=%u", ntohs(__entry->op_id), ntohs(__entry->len))
);

TRACE_EVENT(syscom_service_conf_op_done,
	TP_PROTO(struct syscom_service_msg_conf_op_hdr_s *hdr),

	TP_ARGS(hdr),

	TP_STRUCT__entry(
		__field(__be16, op_id)
		__field(__be32, rtn)
	),

	TP_fast_assign(
		__entry->op_id = hdr->op_id;
		__entry->rtn = hdr->rtn;
	),

	TP_printk("op_id=0x%x rtn=%d", ntohs(__entry->op_id), ntohl(__entry->rtn))
);

/* Trace gateways ***************************************************/

TRACE_EVENT(syscom_gw_create,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		GW_FIELDS
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
	),

	TP_printk(GW_FMT, GW_FMT_ARG)
);

TRACE_EVENT(syscom_gw_destroy,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		GW_FIELDS
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
	),

	TP_printk(GW_FMT, GW_FMT_ARG)
);

TRACE_EVENT(syscom_gw_del,
	TP_PROTO(const struct syscom_gw *gw, int del_children, int del_routes,
			int reason),

	TP_ARGS(gw, del_children, del_routes, reason),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(int, del_children)
		__field(int, del_routes)
		__field(int, reason)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->del_children = del_children;
		__entry->del_routes = del_routes;
		__entry->reason = reason;
	),

	TP_printk(GW_FMT " del_children=%d del_routes=%d reason=%d",
			GW_FMT_ARG, __entry->del_children,
			__entry->del_routes, __entry->reason)
);

TRACE_EVENT(syscom_gw_ready,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		GW_FIELDS
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
	),

	TP_printk(GW_FMT, GW_FMT_ARG)
);

TRACE_EVENT(syscom_gw_worker_start,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		GW_FIELDS
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
	),

	TP_printk(GW_FMT, GW_FMT_ARG)
);

TRACE_EVENT(syscom_gw_worker_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->rtn = rtn;
	),

	TP_printk(GW_FMT " rtn=%d", GW_FMT_ARG, __entry->rtn)
);

TRACE_EVENT(syscom_gw_connect_start,
	TP_PROTO(const struct syscom_gw *gw, unsigned long timeo),

	TP_ARGS(gw, timeo),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(unsigned long, timeo)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->timeo = timeo;
	),

	TP_printk(GW_FMT " timeo=%lu", GW_FMT_ARG, __entry->timeo)
);

TRACE_EVENT(syscom_gw_connect_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->rtn = rtn;
	),

	TP_printk(GW_FMT " rtn=%d", GW_FMT_ARG, __entry->rtn)
);

TRACE_EVENT(syscom_gw_connect_timeout,
	TP_PROTO(void *arg),

	TP_ARGS(arg),

	TP_STRUCT__entry(
		__field(void*, arg)
	),

	TP_fast_assign(
		__entry->arg = arg;
	),

	TP_printk("%p", __entry->arg)
);

TRACE_EVENT(syscom_gw_send_start,
	TP_PROTO(const struct syscom_gw *gw, const struct iov_iter *iov, long timeo),

	TP_ARGS(gw, iov, timeo),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(int, len)
		__field(long, timeo)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->len = iov->count;
		__entry->timeo = timeo;
	),

	TP_printk(GW_FMT " timeo=%ld len=%d", GW_FMT_ARG, __entry->timeo,
			__entry->len)
);

TRACE_EVENT(syscom_gw_send_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		GW_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		GW_ASSIGN(gw);
		__entry->rtn = rtn;
	),

	TP_printk(GW_FMT " rtn=%d", GW_FMT_ARG, __entry->rtn)
);

TRACE_EVENT(syscom_gw_latency_grow,
	TP_PROTO(const struct syscom_gw *gw, int type,
			const struct syscom_hdr *hdr, long latency),

	TP_ARGS(gw, type, hdr, latency),

	TP_STRUCT__entry(
		HDR_FIELDS
		GW_FIELDS
		__field(int, type)
		__field(long, latency)
	),

	TP_fast_assign(
		HDR_ASSIGN(hdr);
		GW_ASSIGN(gw);
		__entry->type = type;
		__entry->latency = latency;
	),

	TP_printk(HDR_FMT " " GW_FMT " latency=%lu type=%d", HDR_FMT_ARG,
		GW_FMT_ARG, __entry->latency, __entry->type)
);

/* Trace route ******************************************************/

TRACE_EVENT(syscom_route_deliver_start,
	TP_PROTO(const struct syscom_hdr *hdr, const struct syscom_delivery *d),

	TP_ARGS(hdr, d),

	TP_STRUCT__entry(
		HDR_FIELDS
		__field(long, timeo)
	),

	TP_fast_assign(
		HDR_ASSIGN(hdr),
		__entry->timeo = d->timeo;
	),

	TP_printk(HDR_FMT " timeo=%ld", HDR_FMT_ARG, __entry->timeo)
);

TRACE_EVENT(syscom_route_deliver_done,
	TP_PROTO(const struct syscom_hdr *hdr, int rtn),

	TP_ARGS(hdr, rtn),

	TP_STRUCT__entry(
		HDR_FIELDS
		__field(int, rtn)
	),

	TP_fast_assign(
		HDR_ASSIGN(hdr);
		__entry->rtn = rtn;
	),

	TP_printk(HDR_FMT " rtn=%d", HDR_FMT_ARG, __entry->rtn)
);

#endif // TRACE_SYSCOM_H

/* This part must be outside protection */
#include <trace/define_trace.h>
