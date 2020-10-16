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

#ifndef SYSCOM_ROUTE_DEV_H
#define SYSCOM_ROUTE_DEV_H

struct syscom_route_record;
struct net_device;

int syscom_route_record_dev_add(__be16 dst_nid, uint8_t dst_nid_len,
		__be16 src_nid, struct net *if_net, const char *if_name,
		uint16_t frag_threshold, void *header, uint8_t header_len,
		int flags);

int syscom_route_dev_down(const char *if_name, void *header, uint8_t header_len,
		uint32_t flags);

int syscom_route_dev_up(const char *if_name, void *header, uint8_t header_len);

#endif // SYSCOM_ROUTE_DEV_H
