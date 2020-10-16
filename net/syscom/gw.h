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

#ifndef SYSCOM_GW_H
#define SYSCOM_GW_H

#include <linux/kref.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/compiler.h>

#include <linux/if_syscom_service.h>

struct syscom_gw_ops_s;

struct syscom_gw {
	/** Routes referencing this gateway */
	struct list_head routes;
	/** Reference counting */
	struct kref kref;
	/** List of gateways in the system */
	struct list_head gateways_list;
	/** Number of send messages */
	atomic_long_t send;
	/** Number of received messages */
	atomic_long_t recv;
	/** Number of dropped outgoing messages */
	atomic_t tx_drop;
	/** Number of dropped incoming messages */
	atomic_t rx_drop;
	/** Number of errors */
	atomic_t err;
	/** Gateway name, kernel generated name are either generated from the
         *  socket inode number or the peer address. */
	char name[SYSCOM_GW_NAME_MAX];
	/** Socket we are gatewaying */
	struct socket *sock;
	/** Socket for m2m emulation */
	struct socket *m2m_sock;
	/** Wait for the socket to be ready */
	struct {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)
		wait_queue_t wait;
#else
		wait_queue_entry_t wait;
#endif
		wait_queue_head_t *wait_address;
	} wait_ready[4];
	/** Number of used wait_ready structures */
	int wait_ready_cnt;
	/** Sending/receiving worker */
	struct delayed_work work;
	/** Queue for internal poll */
	wait_queue_head_t poll_wq;
	/** Condition variable for poll_wq */
	atomic_t poll_wq_cond;
	/** Memory for receiving */
	struct kvec vec;
	/** Amount of data in vec */
	int pending_data;
	/** Syscom header prepended to messages when receiving header less data */
	struct syscom_hdr *hdr;
	/** Gateway flags */
	union syscom_gw_flags {
		struct {
		/** Strip syscom header while sending, push header while receiving */
		unsigned char nohdr:1;
		/** Map IP to NID and vice versa */
		unsigned char addr2nid:1;
		/** Map port to CPID and vice versa */
		unsigned char port2cpid:1;
		unsigned char fake_connect:1;
		unsigned char stop_work:1;
		unsigned char listening:1;
		unsigned char connecting:1;
		unsigned char m2m_emulation:1;
		/** Do not send ordered messages on this GW */
		unsigned char reject_ordered:1;
		/** Do not send reliable messages on this GW */
		unsigned char reject_reliable:1;
		};
		unsigned long flags;
	} flag;
	/** Destination address used by the DGRAM GW */
	struct sockaddr_in6 dest_addr;
	/** Destination address length */
	int dest_addrlen;
	/** Parent gateway */
	struct syscom_gw *parent;
	/** Lock for stream send or seqpacket accept */
	struct mutex lock;
	/** List of ongoing SCTP associations protected by lock */
	struct list_head conns;
	/** Sending workqueue if we need to send from IRQ */
	struct workqueue_struct *send_wq;
	/** Gateway operations */
	const struct syscom_gw_ops_s *ops;
	/** RCU head for releasing the gateway */
	struct rcu_head rcu_head;
	/** Completion for synchronized releasing of gateway */
	struct completion *completion;
	/** Reason for removal */
	int reason;
};

#ifdef CONFIG_PROC_FS
void *syscom_gw_seq_start(struct seq_file *seq, loff_t *pos);
void *syscom_gw_seq_next(struct seq_file *seq, void *v, loff_t *pos);
void syscom_gw_seq_stop(struct seq_file *seq, void *v);
int syscom_gw_seq_show(struct seq_file *seq, void *v);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 18, 0))
int syscom_gw_seq_open(struct inode *inode, struct file *file);
#endif

extern const struct seq_operations syscom_gw_seq_ops;

/** Route table iterator */
struct syscom_gw_iter_state {
	struct seq_net_private p;
	struct syscom_gw *gw;
};
#endif

void syscom_gw_kref_release(struct kref *kref);

/** Get a reference to the gateway, should be called under RCU lock (or gw lock
 * if the GW is still on the list) */
static inline struct syscom_gw *syscom_gw_get(struct syscom_gw *gw)
{
	return kref_get_unless_zero(&gw->kref) ? gw : NULL;
}

/** Put a reference to the gateway. */
static inline void syscom_gw_put(struct syscom_gw *gw)
{
	kref_put(&gw->kref, &syscom_gw_kref_release);
}

void syscom_gw_lock(void);

void syscom_gw_unlock(void);

int syscom_gw_add(const char *basename, char *genname,
		int netid, int domain, int type, int protocol, int flags,
		struct sockaddr *addrs, int addrcnt, int addrsize);

int syscom_gw_rename(const char *old_name, const char *new_name);

int syscom_gw_del(const char *name, bool del_children, bool del_routes,
		int reason, bool nonblock, unsigned long timeo);

int syscom_gw_bind(const char *name, struct sockaddr *addrs, int addrcnt, int addrsize);

int syscom_gw_listen(const char *name, int backlog);

int syscom_gw_connect(const char *name, const char *childname, char *genname,
		struct sockaddr *addrs, int addrcnt, int addrsize,
		bool nonblock, unsigned long timeo);

int syscom_gw_hdr(const char *name, const struct syscom_hdr *hdr);

struct syscom_gw *syscom_gw_lookup(const char *name);

int syscom_gw_send(struct syscom_gw *gw, struct iov_iter *iov, long timeo);

int syscom_gw_init(void);

void syscom_gw_destroy(void);

int syscom_gw_open_socket(const char *name, int flags);

#endif // SYSCOM_GW_H
