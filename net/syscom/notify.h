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

#ifndef SYSCOM_NOTIFY_H
#define SYSCOM_NOTIFY_H

void syscom_notify_gw_rename(const char *oldname, const char *newname);
void syscom_notify_gw_add(const char *name, bool accepted);
void syscom_notify_gw_remove(const char *name, int reason);
void syscom_notify_gw_bind(const char *name, struct sockaddr *addr,
		int addrcnt, int addrsize);
void syscom_notify_gw_connect(const char *name, bool accepted,
		struct sockaddr *addr, int addrcnt, int addrsize);

int syscom_notify_init(void);
void syscom_notify_destroy(void);

#endif // SYSCOM_NOTIFY_H
