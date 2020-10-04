/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Texas Instruments Keystone Navigator Queue Management SubSystem
 * Queue Managers Monitor Header
 */

#ifndef __KNAV_QMSS_MONITOR_H
#define __KNAV_QMSS_MONITOR_H

#include <linux/rbtree.h>
#include <linux/rhashtable.h>
#include <linux/ring_buffer.h>

#include "hwqueue_internal.h"

enum knav_mqmss_watermark {
	WM_MIN,
	WM_LOW,
	WM_HIGH,
	NR_WATERMARK
};

#define KNAV_QMSSM_WM_MIN 1
#define KNAV_QMSSM_WM_MAX 4096

#define KNAV_QMSSM_ENABLE	1
#define KNAV_QMSSM_DISABLE	0

#define KNAV_QMSSM_FDQ_PER_CHAN 4

/*
 * knav_qmssm_record_item - statistics entry descriptor
 * qid - queue number
 * count - keep amount of available descriptors in a hardware queue qid
 */
struct knav_qmssm_record_item {
	uint16_t qid;
	uint16_t count;
} __packed;

/*
 * knav_qmssm_queue_property - describe property for queue under monitoring
 * new features and functionality could be added here
 * wmark - watermakr vector
 * enable - enable tracing
 * buffsize - sizeof buffer
 */
struct knav_qmssm_queue_property {
	atomic_t enable;
	unsigned int wmark[NR_WATERMARK];
	unsigned int buffsize;
};

/*
 * knav_qmssm_qdata - logical data struct to get/set property for queue
 * property - property for this knav queue entry
 * ring_buffer - buffer with collected statistics/logs
 * lchachee - last cache entry keep entry from rb to calc logging threshold
 * mqd - pointer to relative queue dentry
 * qh - keep knav queue wchich is event triggered
 */
struct knav_qmssm_qdata {
	struct knav_qmssm_queue_property property;
	struct ring_buffer *ring_buffer;
	struct knav_qmssm_record_item lchachee[KNAV_QMSSM_FDQ_PER_CHAN];
	struct monitor_queue_dentry *mqd;
};

/*
 * struct monitor_queue_entry - descride extended entry
 * wmark - dentry represent watermark file
 * enable - interface to enable/disable statistics collection
 * bufsize - change buffer size for current queue statistics collection
 * monitor_statas - show collected statistics
 * data - pointer to relative data
 */
struct monitor_queue_entry {
	struct dentry *wmark;
	struct dentry *enable;
	struct dentry *bufsize;
	struct dentry *monitor_stats;
	struct knav_qmssm_qdata *data;
};

/*
 * monitor_queue_dentry - struct represent item for registered queue
 * list - link all registered queues for monitoring for current device
 * qid - queue id number
 * lock - protect property access
 * root_qid - root dentry for queue
 * qmon - point to parent monitor device
 */
struct monitor_queue_dentry {
	struct list_head list;
	unsigned int qid;
	struct hwqueue *qh;
	struct mutex lock;
	struct dentry *root_qid;
	struct monitor_queue_entry *mqe;
	struct knav_qmssm *qmon;
};

/*
 * struct knav_qmssm_ilogger - interval logger periodic interval monitor
 * thread - kthread descriptor
 * mq_interval - dentry descriptor for interval
 * interval_ms - interval define monitor thread work cycle
 */
struct knav_qmssm_ilogger {
	struct task_struct *thread;
	struct dentry *mq_interval;
	u64 interval_ms;
};

/*
 * knav_qmssm - describe monitor for hardware queue device
 * hdev - device which is under monitoring
 * name - monitor instance name
 * mqlist - monitoring queues list queue_dentry HEAD is a list to keep queues
 * mqlock - protect for each list mqd for current device monitor
 * list  - link hardware queue devices monitors in a list
 * mq_root - hwq_monitor_%device_name% root debugfs dentry for monitored device
 * mq_register - interface for new queue register to monitor
 * mq_unregister - interface for queue unregister and stop monitoring
 * ilogger - interval thread logger for monitor device
 */
struct knav_qmssm {
	struct hwqueue_device *hdev;
	unsigned char *name;
	struct list_head mqlist;
	struct mutex mqlock;
	struct list_head list;
	struct dentry *mq_root;
	struct dentry *mq_register;
	struct dentry *mq_unregister;
	struct knav_qmssm_ilogger ilogger;
};

int knav_qmssm_register(struct hwqueue_device *hdev);
void knav_qmssm_unregister(struct hwqueue_device *hdev);

#endif /*__KNAV_QMSS_MONITOR_H*/
