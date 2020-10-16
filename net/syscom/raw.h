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

#ifndef SYSCOM_RAW_H
#define SYSCOM_RAW_H

#include <linux/list.h>

struct syscom_sock;
struct sk_buff;

extern struct hlist_head raw_sockets;

#define syscom_raw_delivery_needed() unlikely(!hlist_empty(&raw_sockets))

/** Queue SKB on raw sockets. The SKB is consumed. */
void syscom_raw_queue_skb(struct sk_buff *skb, gfp_t gfp_mask, int rtn);

void syscom_raw_create(struct syscom_sock *ssk);

extern const struct proto_ops syscom_raw_ops;

#endif // SYSCOM_RAW_H
