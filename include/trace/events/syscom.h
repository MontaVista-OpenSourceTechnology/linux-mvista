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

#define SKB_FIELDS \
	__field_struct(struct syscom_trace_hdr, syscom_trace_hdr)

#define SKB_ASSIGN(skb) \
	memcpy(&__entry->syscom_trace_hdr, \
		&(((struct syscom_hdr*)(skb)->data)->msg_id),\
		sizeof __entry->syscom_trace_hdr)

#define SKB_FMT "from: %04x:%04x to: %04x:%04x msg_id: %d"

#define SKB_FMT_ARG \
	ntohs(__entry->syscom_trace_hdr.src.nid), \
	ntohs(__entry->syscom_trace_hdr.src.cpid), \
	ntohs(__entry->syscom_trace_hdr.dst.nid), \
	ntohs(__entry->syscom_trace_hdr.dst.cpid), \
	ntohs(__entry->syscom_trace_hdr.msg_id)

#endif // TRACE_SYSCOM_H

/* Trace socket *****************************************************/
#if !defined(TRACE_SYSCOM_H) || defined(TRACE_HEADER_MULTI_READ)
#define TRACE_SYSCOM_H

TRACE_EVENT(syscom_sk_send_err,
	TP_PROTO(struct syscom_sock *ssk, int rtn),

	TP_ARGS(ssk, rtn),

	TP_STRUCT__entry(
		__field(ino_t, ino)
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
		__entry->rtn = rtn;
	),

	TP_printk("sk_ino=0x%lx rtn=%d", (unsigned long)__entry->ino, __entry->rtn)
);

TRACE_EVENT(syscom_sk_send_drop,
	TP_PROTO(struct syscom_sock *ssk),

	TP_ARGS(ssk),

	TP_STRUCT__entry(
		__field(ino_t, ino)
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
	),

	TP_printk("sk_ino=0x%lx", (unsigned long)__entry->ino)
);

TRACE_EVENT(syscom_sk_recv_err,
	TP_PROTO(struct syscom_sock *ssk, struct syscom_hdr *hdr, int rtn),

	TP_ARGS(ssk, hdr, rtn),

	TP_STRUCT__entry(
		__field(ino_t, ino)
		__field(__be16, msg_id)
		__field(__be16, src_nid)
		__field(__be16, src_cpid)
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
		__entry->msg_id = hdr ? hdr->msg_id : 0xffff;
		__entry->src_nid = hdr ? hdr->src.nid : 0xffff;
		__entry->src_cpid = hdr ? hdr->src.cpid : 0xffff;
		__entry->rtn = rtn;
	),

	TP_printk("sk_ino=0x%lx msg_id=%d src=%04x:%04x rtn=%d",
		(unsigned long)__entry->ino, ntohs(__entry->msg_id),
		ntohs(__entry->src_nid), ntohs(__entry->src_cpid), __entry->rtn)
);

TRACE_EVENT(syscom_sk_recv_drop,
	TP_PROTO(struct syscom_sock *ssk),

	TP_ARGS(ssk),

	TP_STRUCT__entry(
		__field(ino_t, ino)
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
	),

	TP_printk("sk_ino=0x%lx", (unsigned long)__entry->ino)
);

TRACE_EVENT(syscom_sk_recv_latency_grow,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb, unsigned long latency),

	TP_ARGS(ssk, skb, latency),

	TP_STRUCT__entry(
		__field(ino_t, ino)
		__field(unsigned long, latency)
		SKB_FIELDS
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
		__entry->latency = latency;
		SKB_ASSIGN(skb);
	),

	TP_printk("sk_ino=0x%lx latency=%lu " SKB_FMT,
		(unsigned long)__entry->ino, __entry->latency, SKB_FMT_ARG)
);

TRACE_EVENT(syscom_sk_queue_recv_skb,
	TP_PROTO(struct syscom_sock *ssk, struct sk_buff *skb),

	TP_ARGS(ssk, skb),

	TP_STRUCT__entry(
		__field(ino_t, ino)
		__field(int, ifindex)
		SKB_FIELDS
	),

	TP_fast_assign(
		__entry->ino = SOCK_INODE(ssk->sk.sk_socket)->i_ino;
		__entry->ifindex = skb->dev ? skb->dev->ifindex : -1;
		SKB_ASSIGN(skb);
	),

	TP_printk("sk_ino=0x%lx dev=%d " SKB_FMT, (unsigned long)__entry->ino,
		__entry->ifindex, SKB_FMT_ARG)
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
		__field(const struct syscom_gw *, gw)
	),

	TP_fast_assign(
		__entry->gw = gw;
	),

	TP_printk("gw=%s", __entry->gw->name)
);

TRACE_EVENT(syscom_gw_destroy,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
	),

	TP_fast_assign(
		__entry->gw = gw;
	),

	TP_printk("gw=%s", __entry->gw->name)
);

TRACE_EVENT(syscom_gw_del,
	TP_PROTO(const struct syscom_gw *gw, int del_children, int del_routes,
			int reason),

	TP_ARGS(gw, del_children, del_routes, reason),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
		__field(int, del_children)
		__field(int, del_routes)
		__field(int, reason)
	),

	TP_fast_assign(
		__entry->gw = gw;
	),

	TP_printk("gw=%s del_children=%d del_routes=%d reason=%d",
			__entry->gw->name, __entry->del_children,
			__entry->del_routes, __entry->reason)
);

TRACE_EVENT(syscom_gw_ready,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
	),

	TP_fast_assign(
		__entry->gw = gw;
	),

	TP_printk("gw=%s", __entry->gw->name)
);

TRACE_EVENT(syscom_gw_worker_start,
	TP_PROTO(const struct syscom_gw *gw),

	TP_ARGS(gw),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
	),

	TP_fast_assign(
		__entry->gw = gw;
	),

	TP_printk("gw=%s", __entry->gw->name)
);

TRACE_EVENT(syscom_gw_worker_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->gw = gw;
		__entry->rtn = rtn;
	),

	TP_printk("gw=%s rtn=%d", __entry->gw->name, __entry->rtn)
);

TRACE_EVENT(syscom_gw_connect_start,
	TP_PROTO(const struct syscom_gw *gw, unsigned long timeo),

	TP_ARGS(gw, timeo),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
		__field(unsigned long, timeo)
	),

	TP_fast_assign(
		__entry->gw = gw;
		__entry->timeo = timeo;
	),

	TP_printk("gw=%s timeo=%lu", __entry->gw->name, __entry->timeo)
);

TRACE_EVENT(syscom_gw_connect_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->gw = gw;
		__entry->rtn = rtn;
	),

	TP_printk("gw=%s rtn=%d", __entry->gw->name, __entry->rtn)
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
		__field(const struct syscom_gw *, gw)
		__field(const struct iov_iter *, iov)
		__field(long, timeo)
	),

	TP_fast_assign(
		__entry->gw = gw;
		__entry->iov = iov;
		__entry->timeo = timeo;
	),

	TP_printk("gw=%s timeo=%ld", __entry->gw->name, __entry->timeo)
);

TRACE_EVENT(syscom_gw_send_done,
	TP_PROTO(const struct syscom_gw *gw, int rtn),

	TP_ARGS(gw, rtn),

	TP_STRUCT__entry(
		__field(const struct syscom_gw *, gw)
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->gw = gw;
		__entry->rtn = rtn;
	),

	TP_printk("gw=%s rtn=%d", __entry->gw->name, __entry->rtn)
);

/* Trace route ******************************************************/

TRACE_EVENT(syscom_route_deliver_start,
	TP_PROTO(const struct syscom_hdr *hdr, const struct syscom_delivery *d),

	TP_ARGS(hdr, d),

	TP_STRUCT__entry(
		__field_struct(struct syscom_trace_hdr, hdr)
		__field(long, timeo)
	),

	TP_fast_assign(
		memcpy(&__entry->hdr, &hdr->msg_id, sizeof __entry->hdr);
		__entry->timeo = d->timeo;
	),

	TP_printk("msg_id=%d src=%04x:%04x dst=%04x:%04x len=%d timeo=%ld",
		ntohs(__entry->hdr.msg_id),
		ntohs(__entry->hdr.src.nid), ntohs(__entry->hdr.src.cpid),
		ntohs(__entry->hdr.dst.nid), ntohs(__entry->hdr.dst.cpid),
		ntohs(__entry->hdr.length), __entry->timeo)
);

TRACE_EVENT(syscom_route_deliver_done,
	TP_PROTO(int rtn),

	TP_ARGS(rtn),

	TP_STRUCT__entry(
		__field(int, rtn)
	),

	TP_fast_assign(
		__entry->rtn = rtn;
	),

	TP_printk("rtn=%d", __entry->rtn)
);

#endif // TRACE_SYSCOM_H

/* This part must be outside protection */
#include <trace/define_trace.h>
