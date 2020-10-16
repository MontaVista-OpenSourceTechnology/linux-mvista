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

#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/vmalloc.h>
#include <linux/module.h>

#include <linux/if_syscom_service.h>
#include "service-socket.h"
#include "route-local.h"
#include "route-dev.h"
#include "route-gw.h"
#include "route.h"
#include "gw.h"

#ifndef _K_SS_ALIGNSIZE
#define _K_SS_ALIGNSIZE (__alignof__ (struct sockaddr *))
#endif

static struct socket *service_sock;
static struct work_struct service_work;
static struct kvec service_vec;
static char cmsg_buf[CMSG_SPACE(sizeof(__be16))];
static struct cmsghdr *cmsg = (struct cmsghdr *)cmsg_buf;

static void syscom_service_loop(struct work_struct *work);
static DECLARE_WORK(service_work, &syscom_service_loop);

static void (*orig_data_ready)(struct sock *sk);
static void service_socket_data_ready(struct sock *sk)
{
	orig_data_ready(sk);
	schedule_work(&service_work);
}

static int syscom_service_conf(void *buf, int len, struct msghdr *msghdr)
{
	struct syscom_service_msg_conf_s *msg = buf;
	struct syscom_service_msg_conf_op_hdr_s *hdr;
	struct syscom_route_bulk_del_s del_cookie;
	unsigned long max_jiffies = MAX_JIFFY_OFFSET;
	int i, oplen, failed;

	// Avoid processing rubbish
	if (len < sizeof *msg) {
		return -1;
	}

	for (i = sizeof *msg; i < len - sizeof *hdr; i += oplen) {
		hdr = (struct syscom_service_msg_conf_op_hdr_s *)((char *)buf + i);
		oplen = ntohs(hdr->len);
		if (oplen < sizeof *hdr) {
			return -1;
		}
	}
	if (i != len) {
		return -1;
	}

	trace_syscom_service_conf(msg);
	if (msg->timeout && be64_to_cpu(msg->timeout) < SYSCOM_SERVICE_MAX_TIMEOUT) {
		struct cmsghdr *cmsg;
		for_each_cmsghdr(cmsg, msghdr) {
			if (cmsg->cmsg_level == SOL_SYSCOM &&
			    cmsg->cmsg_type == SYSCOM_SO_QUEUETSTAMP) {
				struct timespec *ts;
				ktime_t delta, end;

				BUG_ON(CMSG_SPACE(sizeof *ts) != cmsg->cmsg_len);
				ts = CMSG_DATA(cmsg);

				end = ktime_add_ms(timespec_to_ktime(*ts),
						be64_to_cpu(msg->timeout));

				// We have jiffy resolution, thus we do not
				// care about IRQ delay
				preempt_disable();
				delta = ktime_sub(end, ktime_get());
				max_jiffies = usecs_to_jiffies(ktime_to_us(delta));
				if (max_jiffies != MAX_JIFFY_OFFSET) {
					max_jiffies += jiffies;
				}
				preempt_enable();
				break;
			}
		}
	}

	syscom_route_bulk_del_prepare(&del_cookie);

	// Interpret the message;
	for (i = sizeof *msg, failed = 0; i < len - sizeof *hdr; i += oplen) {
		int rtn = -EAGAIN; unsigned long timeo, now = jiffies;
		hdr = (struct syscom_service_msg_conf_op_hdr_s *)((char *)buf + i);
		oplen = ntohs(hdr->len);

		if (ntohs(hdr->op_id) != syscom_service_msg_conf_op_id_route_del) {
			syscom_route_bulk_del_sync(&del_cookie);
		}

		trace_syscom_service_conf_op_start(hdr);

		timeo = max_jiffies == MAX_JIFFY_OFFSET ? MAX_SCHEDULE_TIMEOUT : max_jiffies - now;
		if (max_jiffies != MAX_JIFFY_OFFSET && time_before_eq(now, max_jiffies)) {
			rtn = -ETIMEDOUT;
		} else if (!failed) switch (ntohs(hdr->op_id)) {
#define CASE(exchange) case syscom_service_msg_conf_op_id_##exchange: { \
	struct syscom_service_msg_conf_op_##exchange##_s *op; \
	op = (struct syscom_service_msg_conf_op_##exchange##_s *)hdr; \
	if (oplen != sizeof *op) { rtn = -EINVAL; } else
#define ESAC break; }
			CASE(route_del) {
				rtn = syscom_route_bulk_del(&del_cookie,
						op->req.dst_nid, op->req.dst_nid_len);
			} ESAC
			CASE(route_add_local) {
				rtn = syscom_route_record_local_add(op->req.dst_nid,
						op->req.dst_nid_len, op->req.src_nid);
			} ESAC
			CASE(route_add_dev) {
				op->req.src_if[sizeof op->req.src_if - 1] = 0;

				if (op->req.header_len > sizeof op->req.header) {
					rtn = -EINVAL;
					break;
				}

				rtn = syscom_route_record_dev_add(op->req.dst_nid,
						op->req.dst_nid_len, op->req.src_nid,
						sock_net(service_sock->sk), op->req.src_if,
						ntohs(op->req.frag), op->req.header,
						op->req.header_len, ntohl(op->req.flags));
			} ESAC
			CASE(route_add_gw) {
				rtn = syscom_route_record_gw_add(op->req.dst_nid,
						op->req.dst_nid_len, op->req.src_nid,
						op->req.gw, ntohl(op->req.flags));
			} ESAC

			CASE(route_dev_down) {
				rtn = syscom_route_dev_down(op->req.src_if, op->req.header,
						op->req.header_len, ntohl(op->req.flags));
			} ESAC
			CASE(route_dev_up) {
				rtn = syscom_route_dev_up(op->req.src_if, op->req.header,
						op->req.header_len);
			} ESAC

			CASE(gw_del) {
				unsigned flags = ntohl(op->req.flags);
				rtn = syscom_gw_del(op->req.name,
						flags & SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_CHILDREN,
						flags & SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_ROUTES, 0,
						flags & SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_NONBLOCK,
						timeo);
			} ESAC
			CASE(gw_rename) {
				rtn = syscom_gw_rename(op->req.old_name, op->req.new_name);
			} ESAC
			case syscom_service_msg_conf_op_id_gw_add: {
				struct syscom_service_msg_conf_op_gw_add_s *op;
				int addrcnt, addrsize;
				void *storage;

				op = (struct syscom_service_msg_conf_op_gw_add_s *)hdr;
				if (oplen < sizeof *op) {
					rtn = -EINVAL;
					break;
				}

				addrcnt = ntohs(op->req.addr_cnt);
				addrsize = oplen - sizeof *op;
				if (ntohs(op->req.addr_cnt) > 0 &&
				    (long)&op[1] & (_K_SS_ALIGNSIZE - 1)) {
					storage = kmalloc(oplen - sizeof *op, GFP_KERNEL);
					if (!storage) {
						rtn = -ENOMEM;
						break;
					}
					memcpy(storage, &op[1], oplen - sizeof *op);
				} else {
					storage = &op[1];
				}

				syscom_addr_from_be(storage, addrsize);
				rtn = syscom_gw_add(op->req.name, op->rep.name, ntohl(op->req.net),
						ntohl(op->req.domain), ntohl(op->req.type),
						ntohl(op->req.protocol), ntohl(op->req.flags), storage,
						addrcnt, addrsize);
				if (storage != &op[1]) {
					kfree(storage);
				}
				break; }
			case syscom_service_msg_conf_op_id_gw_bind: {
				struct syscom_service_msg_conf_op_gw_bind_s *op;
				int addrcnt, addrsize;
				void *storage;

				op = (struct syscom_service_msg_conf_op_gw_bind_s *)hdr;
				if (oplen <= sizeof *op) {
					rtn = -EINVAL;
					break;
				}

				addrcnt = ntohs(op->req.addr_cnt);
				addrsize = oplen - sizeof *op;
				if ((long)&op[1] & (_K_SS_ALIGNSIZE - 1)) {
					storage = kmalloc(addrsize, GFP_KERNEL);
					if (!storage) {
						rtn = -ENOMEM;
						break;
					}
					memcpy(storage, &op[1], addrsize);
				} else {
					storage = &op[1];
				}

				syscom_addr_from_be(storage, addrsize);
				rtn = syscom_gw_bind(op->req.name, storage,
						addrcnt, addrsize);
				if (storage != &op[1]) {
					kfree(storage);
				}
				break; }
			CASE(gw_listen) {
				rtn = syscom_gw_listen(op->req.name, ntohl(op->req.backlog));
			} ESAC
			case syscom_service_msg_conf_op_id_gw_connect: {
				struct syscom_service_msg_conf_op_gw_connect_s *op;
				unsigned long op_timeo;
				int addrcnt, addrsize;
				void *storage;
				bool nonblock;

				op = (struct syscom_service_msg_conf_op_gw_connect_s *)hdr;
				if (oplen <= sizeof *op) {
					rtn = -EINVAL;
					break;
				}

				nonblock = ntohl(op->req.flags) & SYSCOM_SERVICE_MSG_CONF_OP_GW_CONNECT_NONBLOCK;
				op_timeo = ntohl(op->req.timeout);
				if (op_timeo >= SYSCOM_SERVICE_MAX_TIMEOUT ||
				    op_timeo == 0) {
					op_timeo = MAX_SCHEDULE_TIMEOUT;
				} else {
					op_timeo = msecs_to_jiffies(op_timeo);
				}
				if (!nonblock && op_timeo > timeo) {
					op_timeo = timeo;
				}
				addrcnt = ntohs(op->req.addr_cnt);
				addrsize = oplen - sizeof *op;
				if ((long)&op[1] & (_K_SS_ALIGNSIZE - 1)) {
					storage = kmalloc(addrsize, GFP_KERNEL);
					if (!storage) {
						rtn = -ENOMEM;
						break;
					}
					memcpy(storage, &op[1], addrsize);
				} else {
					storage = &op[1];
				}

				syscom_addr_from_be(storage, addrsize);
				rtn = syscom_gw_connect(op->req.name, op->req.childname,
						op->rep.childname, storage, addrcnt, addrsize,
						nonblock, op_timeo);
				if (storage != &op[1]) {
					kfree(storage);
				}
				break; }
			CASE(gw_hdr) {
				rtn = syscom_gw_hdr(op->req.name, &op->req.syscom_hdr);
			} ESAC
			default:
				pr_warn("Unknown operation %d\n", ntohs(hdr->op_id));
				rtn = -ENOSYS;
				break;
		}

		hdr->rtn = htonl(rtn);
		if (rtn < 0 && rtn != -EINPROGRESS) failed = 1;

		trace_syscom_service_conf_op_done(hdr);
	}

	syscom_route_bulk_del_complete(&del_cookie);

	return 0;
}

static void syscom_service_loop(struct work_struct *work)
{
	struct syscom_hdr *hdr = service_vec.iov_base;

	while (1) {
		char controlbuf[64];
		struct sockaddr_syscom addr;
		struct kvec vec = service_vec;
		struct msghdr msg = {
			.msg_name = &addr,
			.msg_namelen = sizeof addr,
			.msg_control = controlbuf,
			.msg_controllen = sizeof controlbuf,
		};
		int recv;

		recv = kernel_recvmsg(service_sock, &msg, &vec, 1,
				vec.iov_len, MSG_DONTWAIT);
		if (recv < 0) {
			if (recv != -EAGAIN) {
				pr_warn("Error receiving on the service socket: %d\n", recv);
			}
			break;
		}

		trace_syscom_service_loop_start(hdr, &addr);
		switch (ntohs(hdr->msg_id)) {
			case SYSCOM_SERVICE_MSG_ID_ECHO_REQUEST:
				hdr->msg_id = htons(SYSCOM_SERVICE_MSG_ID_ECHO_REPLY);
				break;
			case SYSCOM_SERVICE_MSG_ID_CONF_REQUEST:
				hdr->msg_id = htons(SYSCOM_SERVICE_MSG_ID_CONF_REPLY);
				if (syscom_service_conf(&hdr[1], recv - sizeof *hdr, &msg)) {
					pr_err("Malformed configuration message ignored.\n");
					trace_syscom_service_loop_done(0, -EBADMSG);
					continue;
				}
				break;
			default:
				pr_warn("Unknown message %d received on the service socket.\n",
						ntohs(hdr->msg_id));
				trace_syscom_service_loop_done(0, -ENOSYS);
				continue;
		}

		memset(&msg, 0, sizeof msg);
		msg.msg_name = &addr;
		msg.msg_namelen = sizeof addr;
		msg.msg_control = &cmsg_buf;
		msg.msg_controllen = sizeof cmsg_buf;
		msg.msg_flags = MSG_DONTWAIT;
		memcpy(CMSG_DATA(cmsg), &hdr->dst.nid, sizeof hdr->dst.nid);

		vec.iov_base = (char*)hdr + SYSCOM_DGRAM_RAW_HDR_DELTA;
		vec.iov_len = recv - SYSCOM_DGRAM_RAW_HDR_DELTA;
		memcpy(vec.iov_base, &hdr->msg_id, sizeof hdr->msg_id);

		recv = kernel_sendmsg(service_sock, &msg, &vec, 1, vec.iov_len);
		trace_syscom_service_loop_done(recv, 0);
	}
}

int syscom_service_socket_init(void)
{
	struct sockaddr_syscom addr = {
		.sun_family = AF_SYSCOM,
		.nid = SOCKADDR_SYSCOM_ORPHANS_N,
		.cpid = htons(SOCKADDR_SYSCOM_CPID_SERVICE),
	};
	int rtn, one = 1, monotonic = CLOCK_MONOTONIC;

	service_vec.iov_len = SYSCOM_MTU;
	service_vec.iov_base = vmalloc(SYSCOM_MTU);
	if (!service_vec.iov_base) {
		return -ENOMEM;
	}

	cmsg->cmsg_level = SOL_SYSCOM;
	cmsg->cmsg_type = SYSCOM_SO_ADDR_NID;
	cmsg->cmsg_len = CMSG_LEN(sizeof(__be16));

	// Use sock_create_kern() once its prototype is stable
	rtn = __sock_create(&init_net, PF_SYSCOM, SOCK_DGRAM, 0,
			&service_sock, 1);
	if (rtn) {
		goto err0;
	}

	orig_data_ready = service_sock->sk->sk_data_ready;
	service_sock->sk->sk_data_ready = service_socket_data_ready;

	rtn = kernel_setsockopt(service_sock, SOL_SYSCOM,
			SYSCOM_SO_RECV_FULLHDR, (void*)&one, sizeof one);
	if (rtn) {
		goto err1;
	}

	rtn = kernel_setsockopt(service_sock, SOL_SYSCOM,
			SYSCOM_SO_QUEUETSTAMP, (void*)&monotonic, sizeof monotonic);
	if (rtn) {
		goto err1;
	}

	rtn = kernel_setsockopt(service_sock, SOL_SYSCOM,
			SYSCOM_SO_BACKPRESSURE, (void*)&one, sizeof one);
	if (rtn) {
		goto err1;
	}

	rtn = kernel_bind(service_sock, (struct sockaddr *)&addr, sizeof addr);
	if (rtn) {
		goto err1;
	}

	rtn = syscom_route_record_local_add(0, 16, 0);
	if (rtn) {
		goto err1;
	}

	// Release the reference to allow module unloading, we destroy
	// the socket at the unload time on our own. Other option is
	// to move the service socket into a separate module.
	BUG_ON(service_sock->ops->owner != THIS_MODULE);
	BUG_ON(service_sock->sk->sk_prot_creator->owner != THIS_MODULE);
	module_put(service_sock->ops->owner);
	module_put(service_sock->sk->sk_prot_creator->owner);

	return 0;

err1:	sock_release(service_sock);
err0:	vfree(service_vec.iov_base);
	return rtn;
}

void syscom_service_socket_destroy(void)
{
	__module_get(service_sock->ops->owner);
	__module_get(service_sock->sk->sk_prot_creator->owner);
	syscom_route_del(0, 16);
	cancel_work_sync(&service_work);
	service_sock->sk->sk_data_ready = orig_data_ready;
	sock_release(service_sock);
	vfree(service_vec.iov_base);
}
