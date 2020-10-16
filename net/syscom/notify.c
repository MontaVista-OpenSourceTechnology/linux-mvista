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

#include <linux/ratelimit.h>
#include <linux/printk.h>
#include <linux/socket.h>
#include <linux/module.h>
#include <linux/net.h>

#include <linux/if_syscom.h>
#include <linux/if_syscom_notify.h>
#include "notify.h"
#include "af.h"

static struct socket *notify_sock;

static void syscom_notify_send(int msg_id, void *buf, size_t size,
		void *buf2, size_t size2)
{
	struct syscom_dgram_hdr dgram_hdr = { .msg_id = htons(msg_id) };
	struct syscom_notify_msg_hdr_s *msg_hdr = buf;
	static atomic_t seq;
	struct kvec vec[] = {
		{ .iov_base = &dgram_hdr, .iov_len = sizeof dgram_hdr },
		{ .iov_base = buf, .iov_len = size },
		{ .iov_base = buf2, .iov_len = size2 },
	};
	struct msghdr msg = { .msg_flags = MSG_DONTWAIT };
	int rtn, i, sz;

	msg_hdr->seq = htonl(atomic_inc_return(&seq));

	for (sz = i = 0; i < ARRAY_SIZE(vec); i++) sz += vec[i].iov_len;
	rtn = kernel_sendmsg(notify_sock, &msg, vec, ARRAY_SIZE(vec), sz);

	if (rtn < 0) {
		pr_err_ratelimited("Failed to send a notification %d: %d\n",
				msg_id, rtn);
		atomic_inc(&syscom_stats.notify_errors);
	}
}

void syscom_notify_gw_rename(const char *oldname, const char *newname)
{
	struct syscom_notify_msg_gw_added_s addmsg = { 0 };
	struct syscom_notify_msg_gw_removed_s rmmsg = { 0 };

	strncpy(addmsg.name, newname, sizeof addmsg.name);
	strncpy(addmsg.old_name, oldname, sizeof addmsg.old_name);
	addmsg.flags = htonl(SYSCOM_NOTIFY_MSG_GW_ADDED_FLAGS_RENAME);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_ADDED, &addmsg, sizeof addmsg, 0, 0);

	strncpy(rmmsg.name, oldname, sizeof rmmsg.name);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_REMOVED, &rmmsg, sizeof rmmsg, 0, 0);
}

void syscom_notify_gw_add(const char *name, bool accepted)
{
	struct syscom_notify_msg_gw_added_s msg = { 0 };

	strncpy(msg.name, name, sizeof msg.name);
	msg.flags = htonl(accepted ? SYSCOM_NOTIFY_MSG_GW_ADDED_FLAGS_ACCEPT : 0);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_ADDED, &msg, sizeof msg, 0, 0);
}

void syscom_notify_gw_remove(const char *name, int reason)
{
	struct syscom_notify_msg_gw_removed_s msg = { .reason = htonl(reason) };

	strncpy(msg.name, name, sizeof msg.name);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_REMOVED, &msg, sizeof msg, 0, 0);
}

void syscom_notify_gw_bind(const char *name, struct sockaddr *addr,
		int addrcnt, int addrsize)
{
	struct syscom_notify_msg_gw_bound_s msg = { .addrcnt = ntohs(addrcnt) };

	strncpy(msg.name, name, sizeof msg.name);
	syscom_addr_to_be(addr, addrsize);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_BOUND, &msg, sizeof msg,
			addr, addrsize);
}

void syscom_notify_gw_connect(const char *name, bool accepted, struct sockaddr *addr,
		int addrcnt, int addrsize)
{
	struct syscom_notify_msg_gw_connected_s msg = { .addrcnt = ntohs(addrcnt) };

	strncpy(msg.name, name, sizeof msg.name);
	syscom_addr_to_be(addr, addrsize);
	syscom_notify_send(SYSCOM_NOTIFY_MSG_ID_GW_CONNECTED, &msg, sizeof msg,
			addr, addrsize);
}

int syscom_notify_init(void)
{
	struct sockaddr_syscom addr = {
		.sun_family = AF_SYSCOM,
		.cpid = htons(SOCKADDR_SYSCOM_CPID_NOTIFY),
	};
	struct sock_filter code[] = {{6}};
	struct sock_fprog bpf = { .len = 1, .filter = code };
	int rtn, one = 1;

	rtn = sock_create_kern(&init_net, PF_SYSCOM, SOCK_DGRAM, 0, &notify_sock);
	if (rtn) {
		return rtn;
	}

	rtn = kernel_setsockopt(notify_sock, SOL_SOCKET,
			SO_REUSEADDR, (void*)&one, sizeof one);
	if (rtn) {
		goto err0;
	}

	rtn = kernel_setsockopt(notify_sock, SOL_SOCKET,
			SO_ATTACH_FILTER, (void*)&bpf, sizeof bpf);
	if (rtn) {
		goto err0;
	}

	rtn = kernel_bind(notify_sock, (struct sockaddr *)&addr, sizeof addr);
	if (rtn) {
		goto err0;
	}

	rtn = kernel_connect(notify_sock, (struct sockaddr *)&addr, sizeof addr, 0);
	if (rtn) {
		goto err0;
	}

	// Release the reference to allow module unloading, we destroy
	// the socket at the unload time on our own. Other option is
	// to move the notify socket into a separate module.
	BUG_ON(notify_sock->ops->owner != THIS_MODULE);
	BUG_ON(notify_sock->sk->sk_prot_creator->owner != THIS_MODULE);
	module_put(notify_sock->ops->owner);
	module_put(notify_sock->sk->sk_prot_creator->owner);

	return 0;

err0:	sock_release(notify_sock);
	return rtn;
}

void syscom_notify_destroy(void)
{
	__module_get(notify_sock->ops->owner);
	__module_get(notify_sock->sk->sk_prot_creator->owner);
	sock_release(notify_sock);
}
