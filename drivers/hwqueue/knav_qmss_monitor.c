/*
 * Texas Instruments Keystone Navigator Queue Management SubSystem
 * Queue Managers Monitor Implementation
 *
 * Contact: Vasyl Gomonovych <vasyl.gomonovych@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/rhashtable.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>
#include "knav_qmss_monitor.h"

static char *knav_qmssm_name;
module_param(knav_qmssm_name, charp, 0644);
MODULE_PARM_DESC(knav_qmssm_name, "Knav queue monitor name");

#define MAX_BOOTIME_MQ 4
static int knav_qmssm_qid[MAX_BOOTIME_MQ] = {-1, -1, -1, -1};
module_param_array(knav_qmssm_qid, int, NULL, 0644);
MODULE_PARM_DESC(knav_qmssm_qid, "Predefine knav queue ID for early monitoring start");

static u64 knav_qmssm_interval_ms;
module_param(knav_qmssm_interval_ms, ullong, 0644);
MODULE_PARM_DESC(knav_qmssm_interval_ms, "Knav queue logger interval");

static LIST_HEAD(knav_qmssm_list);
static DEFINE_MUTEX(knav_qmssm_lock);

#define TEMP_BUFF_SIZE 128

static const char knav_qmssm_prefix[] = "knav_qmssm";
static const char knav_qmssm_register_str[] = "register";
static const char knav_qmssm_unregister_str[] = "unregister";
static const char knav_qmssm_interval[] = "interval_ms";

static const char knav_qmssm_qid_enable[] = "enable";
static const char knav_qmssm_qid_wmark[] = "watermark";
static const char knav_qmssm_qid_bufsize[] = "buffer_size";
static const char knav_qmssm_qid_stats[] = "monitor_stats";

#define MONITOR_WMARK_MIN  64
#define MONITOR_WMARK_LOW  128
#define MONITOR_WMARK_HIGH 256

#define MONITOR_INTERVAL_MS	(10*1000) /* 10 sec */

#define MONITOR_BS   8192
#define MONITOR_BS_MAX   (64*1024)

#define qmssm_dev(qmon)	(qmon->hdev->dev)
#define mqd_dev(mqd)	(qmssm_dev(mqd->qmon))
#define data_dev(data)	(mqd_dev(data->mqd))
#define mqd_hdev(mqd)	(mqd->qmon->hdev)

#define property(mqd) (mqd->mqe->data->property)

#define INT_PTR(x) ((void *)(unsigned long)x)
#define PTR_INT(x) ((int)(unsigned long)x)

static inline void knav_qmssm_property_dump(struct monitor_queue_dentry *mqd)
{
	dev_dbg(mqd_dev(mqd), "dump: qid %d bs=%d, enable=%d, wmark=[%d %d %d]\n",
			mqd->qid,
			property(mqd).buffsize,
			atomic_read(&property(mqd).enable),
			property(mqd).wmark[WM_MIN],
			property(mqd).wmark[WM_LOW],
			property(mqd).wmark[WM_HIGH]);
}

static void knav_qmssm_dump(struct knav_qmssm *qmon)
{
	struct monitor_queue_dentry *mqd;

	rcu_read_lock();
	list_for_each_entry_rcu(mqd, &qmon->mqlist, list) {
		if (mqd->mqe)
			knav_qmssm_property_dump(mqd);
	}
	rcu_read_unlock();
}

static struct hwqueue *knav_qmssm_get_queue(struct hwqueue_device *hdev,
		unsigned int qid)
{
	struct hwqueue *qh = NULL;
	struct hwqueue_instance *inst = NULL;

	if (!hdev)
		return NULL;

	if (hdev->base_id <= qid && hdev->base_id + hdev->num_queues > qid) {
		qid -= hdev->base_id;
		inst = hdev->instances + (qid <<  hdev->inst_shift);
	}

	if (!inst)
		return NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(qh, &inst->handles, list) {
		if (hwqueue_get_id(qh) == qid)
			break;
	}
	rcu_read_unlock();

	return qh;
}

static int knav_qmssm_thread_logger(void *arg)
{
	struct knav_qmssm *qmon;
	struct monitor_queue_dentry *mqd;
	struct hwqueue *qh, *q[KNAV_QMSSM_FDQ_PER_CHAN];
	unsigned int count[KNAV_QMSSM_FDQ_PER_CHAN];
	unsigned int i = 0;
	unsigned int qid[KNAV_QMSSM_FDQ_PER_CHAN];
	ktime_t delay;
	int ret;

	qmon = (struct knav_qmssm *) arg;

	while (!kthread_should_stop()) {

		rcu_read_lock();
		list_for_each_entry_rcu(mqd, &qmon->mqlist, list) {

			qh = mqd->qh;
			if (!qh || !qh->inst)
				continue;

			if (!qh->monitor_cfg) {
				dev_info(qmssm_dev(qmon), "%d:%d",
					mqd->qid, qh->get_count(qh->inst));
				continue;
			}

			for (i = 0; i < KNAV_QMSSM_FDQ_PER_CHAN; i++) {
				q[i] = qh->monitor_cfg->fdq_arg[i];
				count[i] = 0;
				qid[i] = 0;
				if (!q[i] || !q[i]->inst)
					continue;

				count[i] = q[i]->get_count(q[i]->inst);
				qid[i] = hwqueue_get_id(q[i]);
			}
			dev_info(qmssm_dev(qmon), "%d:%d %d:%d %d:%d %d:%d",
				qid[0], count[0], qid[1], count[1],
				qid[2], count[2], qid[3], count[3]);
		}
		rcu_read_unlock();

		delay = ms_to_ktime(qmon->ilogger.interval_ms);
		set_current_state(TASK_INTERRUPTIBLE);
		ret = schedule_hrtimeout(&delay, HRTIMER_MODE_REL);
		set_current_state(TASK_RUNNING);
	}

	return 0;
}

static int knav_qmssm_logger_start(struct knav_qmssm *qmon)
{
	struct task_struct *thread;
	char name[TEMP_BUFF_SIZE] = {0};
	int len = 0;
	int ret = 0;

	if (!qmon)
		return -EINVAL;

	mutex_lock(&qmon->mqlock);

	if (qmon->ilogger.thread)
		goto out;

	len = snprintf(name, TEMP_BUFF_SIZE, "kthread_%s/%s\n",
			knav_qmssm_prefix,
			dev_name(qmssm_dev(qmon)));
	name[len] = '\0';

	thread = kthread_run(knav_qmssm_thread_logger, qmon, name);

	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		goto out;
	}

	qmon->ilogger.thread = thread;

out:
	mutex_unlock(&qmon->mqlock);

	return ret;
}

static void knav_qmssm_logger_stop(struct knav_qmssm *qmon)
{
	int ret;

	if (!qmon)
		return;

	mutex_lock(&qmon->mqlock);

	if (WARN_ON(!qmon->ilogger.thread)) {
		ret = -EINVAL;
		goto out;
	}

	send_sig(SIGTERM, qmon->ilogger.thread, 1);
	ret = kthread_stop(qmon->ilogger.thread);
	qmon->ilogger.thread = NULL;

out:
	mutex_unlock(&qmon->mqlock);
}

static int knav_qmssm_alloc_rb(struct knav_qmssm_qdata *data)
{
	int size;

	if (!data)
		return -EINVAL;

	size = data->property.buffsize;
	(*data).ring_buffer = ring_buffer_alloc(size, RB_FL_OVERWRITE);
	if (!data->ring_buffer) {
		dev_dbg(data_dev(data), "cannot allocate log buffer\n");
		return -ENOMEM;
	}
	dev_dbg(data_dev(data), "ring buffer allocated\n");

	return 0;
}

static int knav_qmssm_free_rb(struct knav_qmssm_qdata *data)
{
	if (!data) {
		dev_warn(data_dev(data), "%s() null data pointer\n", __func__);
		return -EINVAL;
	}

	if (data->ring_buffer) {
		ring_buffer_free(data->ring_buffer);
		data->ring_buffer = NULL;
		dev_dbg(data_dev(data), "ring buffer freed\n");
	}

	return 0;
}

static int knav_qmssm_write_rb(struct ring_buffer *buffer, void *entry,
		unsigned long size)
{
	int ret = 0;

	if (!buffer || !entry) {
		pr_err("%s() rb=%p entry=%p", __func__, buffer, entry);
		return -EINVAL;
	}

	ret = ring_buffer_write(buffer, size, entry);
	if (ret)
		pr_err("dropped log event %p size %lu\n", buffer, size);

	return 0;
}

static void knav_qmssm_read_rb(struct seq_file *filp,
		struct knav_qmssm_qdata *data)
{
	struct ring_buffer_event *event;
	struct knav_qmssm_record_item *entry;
	struct ring_buffer *rb;
	int cpu;
	int found = 1;
	u64 ts;
	unsigned long lost;
	unsigned long rem_nsec;

	rb = data->ring_buffer;

	if (!rb) {
		dev_err(data_dev(data), "%s null data\n", __func__);
		return;
	}

	if (ring_buffer_empty(rb)) {
		dev_dbg(data_dev(data), "log buffer empty qid %d\n",
					data->mqd->qid);
		found = 0;
	}

	while (found) {

		found = 0;
		for_each_online_cpu(cpu) {

			event = ring_buffer_consume(rb, cpu, &ts, &lost);
			if (!event)
				continue;
			else {
				entry = ring_buffer_event_data(event);
				found = 1;
				if (!entry)
					continue;

				rem_nsec = do_div(ts, 1000000000);
				seq_printf(filp, "[%5lu.%06lu]\t\t%d\t\t%d\n",
					(unsigned long)ts, rem_nsec,
					entry->qid, entry->count);
			}
		}
	}
}

static void knav_qmssm_write_data(struct knav_qmssm_qdata *data,
		struct knav_qmssm_record_item *entry)
{
	int ret = 0;

	if (!data || !entry) {
		pr_err("error ring buffer write\n");
		return;
	}

	ret = knav_qmssm_write_rb(data->ring_buffer, entry, sizeof(*entry));
	if (ret) {
		pr_err("ring buffer error to write qid %d\n", data->mqd->qid);
		return;
	}
}

static void knav_qmssm_start_rb(struct knav_qmssm_qdata *data)
{
	if (!data->ring_buffer)
		return;
	ring_buffer_record_on(data->ring_buffer);
	dev_dbg(data_dev(data), "%s() ring buffer start\n", __func__);
}

static void knav_qmssm_stop_rb(struct knav_qmssm_qdata *data)
{
	if (!data->ring_buffer)
		return;
	ring_buffer_record_off(data->ring_buffer);
	dev_dbg(data_dev(data), "%s() ring buffer stop\n", __func__);
}

static int knav_qmssm_start(struct monitor_queue_dentry *mqd)
{
	struct knav_qmssm_qdata *data;
	int ret = 0;

	if (!mqd || WARN_ON(!mqd->mqe))
		return -EINVAL;

	mutex_lock(&mqd->lock);

	data = mqd->mqe->data;
	if (!data)
		goto out0;

	if (!(&mqd->mqe->data->property.enable))
		goto out0;

	if (atomic_read(&property(mqd).enable) == KNAV_QMSSM_ENABLE)
		goto out0;

	ret = knav_qmssm_free_rb(data);
	if (ret)
		goto out0;

	ret = knav_qmssm_alloc_rb(data);
	if (ret)
		goto out0;

	knav_qmssm_start_rb(data);
	ret = hwqueue_enable_monitoring(mqd->qh);
	if (ret) {
		dev_dbg(mqd_dev(mqd), "cannot start not event-driven queue\n");
		goto out1;
	}
	dev_dbg(mqd_dev(mqd), "%s(%d) success\n", __func__, mqd->qid);
	atomic_set(&property(mqd).enable, KNAV_QMSSM_ENABLE);

out0:
	mutex_unlock(&mqd->lock);
	return ret;

out1:
	knav_qmssm_stop_rb(data);
	knav_qmssm_free_rb(data);
	mutex_unlock(&mqd->lock);
	return ret;
}

static int knav_qmssm_stop(struct monitor_queue_dentry *mqd)
{
	struct knav_qmssm_qdata *data;
	int ret = 0;

	if (!mqd)
		return -EINVAL;

	if (!mqd->mqe) {
		dev_err(mqd_dev(mqd), "%s skip not event-driven queue", __func__);
		return -EINVAL;
	}

	mutex_lock(&mqd->lock);
	data = mqd->mqe->data;

	if (atomic_read(&property(mqd).enable) == KNAV_QMSSM_DISABLE)
		goto out;

	ret = hwqueue_disable_monitoring(mqd->qh);
	if (ret) {
		dev_dbg(mqd_dev(mqd), "%s(%d) error\n", __func__, mqd->qid);
		goto out;
	}
	knav_qmssm_stop_rb(data);
	atomic_set(&property(mqd).enable, KNAV_QMSSM_DISABLE);
	dev_dbg(mqd_dev(mqd), "%s(%d) success\n", __func__, mqd->qid);

out:
	mutex_unlock(&mqd->lock);
	return ret;
}

/**
 * knav_qmssm_init_data() - allocate and initialize main data structures
 */
static int knav_qmssm_init_data(struct monitor_queue_dentry **mqd)
{
	struct knav_qmssm_qdata *data;
	struct knav_qmssm *qmon;

	if (!*mqd || !(*mqd)->mqe)
		return -EINVAL;

	qmon = (*mqd)->qmon;

	data = devm_kzalloc(qmssm_dev(qmon), sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	(*mqd)->mqe->data = data;
	data->mqd = *mqd;
	data->property.buffsize = MONITOR_BS;
	data->property.wmark[WM_MIN] = MONITOR_WMARK_MIN;
	data->property.wmark[WM_LOW] = MONITOR_WMARK_LOW;
	data->property.wmark[WM_HIGH] = MONITOR_WMARK_HIGH;
	atomic_set(&data->property.enable, KNAV_QMSSM_DISABLE);
	data->ring_buffer = NULL;
	memset(&data->lchachee, 0, sizeof(data->lchachee));

	return 0;
}

static int knav_qmssm_free_data(struct monitor_queue_dentry *mqd)
{
	struct knav_qmssm_qdata *data;

	if (!mqd || !mqd->mqe)
		return -EINVAL;

	data = mqd->mqe->data;
	mqd->mqe->data = NULL;

	knav_qmssm_free_rb(data);
	devm_kfree(mqd_dev(mqd), data);

	return 0;
}


static int knav_qmssm_set_enable(void *d, u64 enable)
{
	struct monitor_queue_dentry *mqd = d;
	int ret = 0;

	if (enable == KNAV_QMSSM_ENABLE)
		ret = knav_qmssm_start(mqd);
	else if (enable == KNAV_QMSSM_DISABLE)
		ret = knav_qmssm_stop(mqd);

	return ret;
}

static int knav_qmssm_get_enable(void *d, u64 *enable)
{
	struct monitor_queue_dentry *mqd = d;
	*enable = atomic_read(&property(mqd).enable);
	return 0;
}

static int knav_qmssm_set_bufsize(void *d, u64 bufsize)
{
	struct monitor_queue_dentry *mqd = d;

	mutex_lock(&mqd->lock);
	if (atomic_read(&property(mqd).enable) == KNAV_QMSSM_ENABLE)
		goto out;
	property(mqd).buffsize =
			bufsize > MONITOR_BS_MAX ? MONITOR_BS_MAX : bufsize;
out:
	mutex_unlock(&mqd->lock);
	return 0;
}

static int knav_qmssm_get_bufsize(void *d, u64 *bufsize)
{
	struct monitor_queue_dentry *mqd = d;
	struct ring_buffer *rb;
	long size;

	mutex_lock(&mqd->lock);
	rb = mqd->mqe->data->ring_buffer;
	if (rb)
		size = ring_buffer_size(rb, smp_processor_id());
	else
		size = property(mqd).buffsize;
	*bufsize = size;
	mutex_unlock(&mqd->lock);
	return 0;
}

/**
 * knav_qmssm_set_watermark() - write watermark vector
 */
static ssize_t knav_qmssm_set_watermark(struct file *filp,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	struct monitor_queue_dentry *mqd;
	char buff[TEMP_BUFF_SIZE] = {0};
	ssize_t len = 0;
	int num = 0;
	unsigned int wmark[NR_WATERMARK] = {0};
	unsigned int i = 0;
	int ret = count;

	if (!filp || *ppos != 0)
		return -EINVAL;

	mqd = (struct monitor_queue_dentry *)filp->private_data;

	if (count >= sizeof(buff))
		return -ENOSPC;

	mutex_lock(&mqd->lock);

	len = simple_write_to_buffer(buff, sizeof(buff)-1, ppos, buffer, count);

	if (len < 0) {
		ret = len;
		goto out;
	}

	buff[len] = '\n';

	num = sscanf(buff, "%u %u %u",
			&wmark[WM_MIN], &wmark[WM_LOW], &wmark[WM_HIGH]);
	if (num != NR_WATERMARK) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < NR_WATERMARK; ++i) {
		wmark[i] = wmark[i] < KNAV_QMSSM_WM_MIN ?
				KNAV_QMSSM_WM_MIN : wmark[i];
		wmark[i] = wmark[i] > KNAV_QMSSM_WM_MAX ?
				KNAV_QMSSM_WM_MAX : wmark[i];
		property(mqd).wmark[i] = wmark[i];
	}
out:
	mutex_unlock(&mqd->lock);
	return ret;
}

/**
 * knav_qmssm_get_watermark() - read watermark vector
 */
static ssize_t knav_qmssm_get_watermark(struct file *filp,
		char __user *buffer, size_t count, loff_t *ppos)
{
	struct monitor_queue_dentry *mqd;
	char buff[TEMP_BUFF_SIZE] = {0};
	ssize_t len = 0;
	int ret = 0;

	if (*ppos != 0 || !filp)
		return -EINVAL;

	mqd = (struct monitor_queue_dentry *)filp->private_data;

	mutex_lock(&mqd->lock);
	len = snprintf(buff, TEMP_BUFF_SIZE, "%u %u %u\n",
			property(mqd).wmark[WM_MIN],
			property(mqd).wmark[WM_LOW],
			property(mqd).wmark[WM_HIGH]);

	if (count < strlen(buff)) {
		ret = -ENOSPC;
		goto out;
	}
	ret = simple_read_from_buffer(buffer, count, ppos, buff, len);
out:
	mutex_unlock(&mqd->lock);
	return ret;
}

static ssize_t knav_qmssm_show_statistics(struct seq_file *filp, void *d)
{
	struct monitor_queue_dentry *mqd = filp->private;

	if (!mqd)
		return -EINVAL;

	if (atomic_read(&property(mqd).enable) == KNAV_QMSSM_DISABLE) {
		seq_printf(filp, "%s to enable use:\n", mqd->qmon->name);
		seq_printf(filp, "echo 1 > /sys/kernel/debug/%s/%d/enable\n\n",
				mqd->qmon->name, mqd->qid);
	}

	seq_puts(filp, "timestamp\t\t\tfdq number\tsubmit count\n");
	knav_qmssm_read_rb(filp, mqd->mqe->data);

	return 0;
}

static int knav_qmssm_open_statistics(struct inode *inode, struct file *filp)
{
	return single_open(filp, knav_qmssm_show_statistics, inode->i_private);
}

DEFINE_SIMPLE_ATTRIBUTE(queue_enable_fops, knav_qmssm_get_enable,
		knav_qmssm_set_enable, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(queue_bufsize_fops, knav_qmssm_get_bufsize,
		knav_qmssm_set_bufsize, "%llu\n");

static const struct file_operations knav_qmssm_watermark_fops = {
		.owner	= THIS_MODULE,
		.open	= simple_open,
		.read	= knav_qmssm_get_watermark,
		.write	= knav_qmssm_set_watermark,
};

static const struct file_operations knav_qmssm_stats_fops = {
		.owner		= THIS_MODULE,
		.open		= knav_qmssm_open_statistics,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
};

static inline bool knav_qmssm_readonly_queue(struct hwqueue *qh)
{
	unsigned int acc = qh->flags & O_ACCMODE;
	return acc == O_RDONLY;
}

static int knav_qmssm_register_queue(void *arg, u64 qid)
{
	struct knav_qmssm *qmon = NULL;
	struct monitor_queue_dentry *mqd = NULL;
	struct monitor_queue_entry *mqe = NULL;
	char hwq_monitor_qid[TEMP_BUFF_SIZE] = {0};
	int ret = 0;
	int len = 0;

	if (!arg)
		return -EINVAL;

	qmon = (struct knav_qmssm *)arg;

	if (qmon->hdev->num_queues <= qid)
		return ret;

	mutex_lock(&qmon->mqlock);

	rcu_read_lock();
	list_for_each_entry_rcu(mqd, &qmon->mqlist, list) {
		if (qid == mqd->qid) {
			dev_info(qmssm_dev(qmon),
					"%llu already registered\n", qid);
			ret = -EEXIST;
			break;
		}
	}
	rcu_read_unlock();
	if (ret)
		goto out0;

	mqd = devm_kzalloc(qmssm_dev(qmon), sizeof(*mqd), GFP_KERNEL);
	if (!mqd) {
		ret = -ENOMEM;
		goto out0;
	}
	len = snprintf(hwq_monitor_qid, TEMP_BUFF_SIZE, "%llu", qid);
	hwq_monitor_qid[len] = '\0';

	mqd->root_qid = debugfs_create_dir(hwq_monitor_qid, qmon->mq_root);
	mqd->qid = qid;
	mqd->qmon = qmon;
	mutex_init(&mqd->lock);
	mqd->mqe = NULL;
	mqd->qh = knav_qmssm_get_queue(qmon->hdev, qid);
	if (!mqd->qh)
		goto out1;

	if (knav_qmssm_readonly_queue(mqd->qh)) {

		mqe = devm_kzalloc(qmssm_dev(qmon), sizeof(*mqe), GFP_KERNEL);
		if (!mqe)
			goto out1;

		mqe->enable = debugfs_create_file(knav_qmssm_qid_enable,
				0660, mqd->root_qid, mqd, &queue_enable_fops);
		mqe->wmark = debugfs_create_file(knav_qmssm_qid_wmark,
				0660, mqd->root_qid, mqd, &knav_qmssm_watermark_fops);
		mqe->bufsize = debugfs_create_file(knav_qmssm_qid_bufsize,
				0660, mqd->root_qid, mqd, &queue_bufsize_fops);
		mqe->monitor_stats = debugfs_create_file(knav_qmssm_qid_stats,
				0440, mqd->root_qid, mqd, &knav_qmssm_stats_fops);

		mqd->mqe = mqe;

		ret = knav_qmssm_init_data(&mqd);
		if (ret)
			goto out2;
	}

	list_add_rcu(&mqd->list, &qmon->mqlist);

	knav_qmssm_dump(qmon);

	mutex_unlock(&qmon->mqlock);

	if (!qmon->ilogger.thread) {
		ret = knav_qmssm_logger_start(qmon);
		if (ret)
			dev_err(qmssm_dev(qmon), "cannot run monitor thread\n");
	}

	dev_dbg(qmssm_dev(qmon), "%s(%d)\n", __func__, mqd->qid);

	return PTR_INT(mqd);

out2:
	devm_kfree(qmssm_dev(qmon), mqe);

out1:
	debugfs_remove_recursive(mqd->root_qid);
	devm_kfree(qmssm_dev(qmon), mqd);
	ret = -EINVAL;
	dev_err(qmssm_dev(qmon), "cannot initialize monitor data\n");

out0:
	mutex_unlock(&qmon->mqlock);
	return ret;
}

static int knav_qmssm_unregister_queue_item(struct knav_qmssm *qmon,
		struct monitor_queue_dentry *mqd)
{
	int ret = 0;

	list_del_rcu(&mqd->list);
	synchronize_rcu();

	if (!mqd->mqe) {
		dev_info(mqd_dev(mqd), "free not event-driven queue");
		goto out;
	}

	ret = knav_qmssm_stop(mqd);
	if (ret)
		dev_err(qmssm_dev(qmon), "error stop monitoring\n");
	dev_info(qmssm_dev(qmon), "stop event-driven queue\n");

	ret = knav_qmssm_free_data(mqd);
	if (ret)
		dev_err(qmssm_dev(qmon), "error free monitoring data\n");

	debugfs_remove_recursive(mqd->root_qid);

	devm_kfree(qmssm_dev(qmon), mqd->mqe);

	dev_dbg(qmssm_dev(qmon), "free event-driven queue %d", mqd->qid);

out:
	devm_kfree(qmssm_dev(qmon), mqd);

	return ret;
}

static int knav_qmssm_unregister_queue(void *arg, u64 qid)
{
	struct knav_qmssm *qmon = NULL;
	struct monitor_queue_dentry *mqd = NULL;
	struct list_head *curr, *next;

	if (!arg)
		return -ENOENT;

	qmon = (struct knav_qmssm *)arg;

	mutex_lock(&qmon->mqlock);

	if (list_empty(&qmon->mqlist)) {
		dev_dbg(qmssm_dev(qmon), "no queues under monitoring\n");
		goto out;
	}

	list_for_each_safe(curr, next, &qmon->mqlist) {

		mqd = list_entry(curr, struct monitor_queue_dentry, list);
		if (mqd->qid != qid)
			continue;

		knav_qmssm_unregister_queue_item(qmon, mqd);

		break;
	}

	dev_dbg(qmssm_dev(qmon), "queue %llu unregistered\n", qid);

out:
	mutex_unlock(&qmon->mqlock);

	if (list_empty(&qmon->mqlist))
		knav_qmssm_logger_stop(qmon);

	return 0;
}

static int knav_qmssm_setup_parm_base_queue(struct knav_qmssm *qmon)
{
	struct monitor_queue_dentry *mqd;
	int i = 0, ret = 0;

	dev_dbg(qmssm_dev(qmon), "%s %s %llu", __func__,
			knav_qmssm_name,
			knav_qmssm_interval_ms);

	if (!knav_qmssm_name || strcmp(qmon->name, knav_qmssm_name))
		return 0;

	qmon->ilogger.interval_ms = knav_qmssm_interval_ms;

	for (i = 0; i < MAX_BOOTIME_MQ && knav_qmssm_qid[i] > 0; i++) {
		dev_dbg(qmssm_dev(qmon), "register boottime queue %d monitor %s\n",
				knav_qmssm_qid[i], knav_qmssm_name);
		ret = knav_qmssm_register_queue(qmon, knav_qmssm_qid[i]);
		mqd = INT_PTR(ret);
		if (IS_ERR(mqd))
			continue;
		knav_qmssm_start(mqd);
	}

	return 0;
}

static int knav_qmssm_set_interval(void *arg, u64 interval)
{
	struct knav_qmssm *qmon = arg;

	qmon->ilogger.interval_ms = interval;

	return 0;
}

static int knav_qmssm_get_interval(void *arg, u64 *interval)
{
	struct knav_qmssm *qmon = arg;

	*interval = qmon->ilogger.interval_ms;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(knav_register_queue_fops,
		NULL, knav_qmssm_register_queue, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(knav_unregister_queue_fops,
		NULL, knav_qmssm_unregister_queue, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(knav_interval_legger_fops,
		knav_qmssm_get_interval, knav_qmssm_set_interval, "%llu\n");

/**
 * knav_qmssm_event_callback() - main monitor handler callback
 */
void knav_qmssm_event_callback(void *arg)
{
	struct hwqueue *qh;
	struct hwqueue *q_submit;
	struct hwqueue_device *hdev;
	struct knav_qmssm *qmon;
	struct monitor_queue_dentry *mqd;
	struct knav_qmssm_record_item itm;

	int count = 0;
	int cdiff = 0, pdiff = 0;
	uint16_t qid = 0;
	int i = 0, j = 0;

	qh = (struct hwqueue *)arg;
	hdev = qh->inst->hdev;
	qmon = hdev->qmon;
	qid = hwqueue_get_id(qh);
	if (WARN_ON(!qh->monitor_cfg))
		return;

	rcu_read_lock();
	list_for_each_entry_rcu(mqd, &qmon->mqlist, list) {

		if (mqd->qid != qid)
			continue;

		for (i = 0; i < KNAV_QMSSM_FDQ_PER_CHAN; i++) {

			if (!qh->monitor_cfg->fdq_arg[i])
				break;

			q_submit = qh->monitor_cfg->fdq_arg[i];
			itm.count = q_submit->get_count(q_submit->inst);
			count = mqd->mqe->data->lchachee[i].count;

			for (j = WM_MIN; j < NR_WATERMARK; j++) {

				cdiff = property(mqd).wmark[j] - itm.count;
				pdiff = property(mqd).wmark[j] - count;

				if (!((cdiff < 0 && pdiff > 0) ||
						(cdiff > 0 && pdiff < 0)))
					continue;

				itm.qid = hwqueue_get_id(q_submit);
				knav_qmssm_write_data(mqd->mqe->data, &itm);
				memcpy(&mqd->mqe->data->lchachee[i],
						&itm, sizeof(itm));
			}
		}
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(knav_qmssm_event_callback);

void knav_qmssm_unregister(struct hwqueue_device *hdev)
{
	struct list_head *cd, *nd;
	struct list_head *cq, *nq;
	struct knav_qmssm *qmon;
	struct monitor_queue_dentry *mqd;

	if (WARN_ON(!hdev))
		return;

	mutex_lock(&knav_qmssm_lock);

	if (list_empty(&knav_qmssm_list))
		goto out;

	list_for_each_safe(cd, nd, &knav_qmssm_list) {

		qmon = list_entry(cd, struct knav_qmssm, list);
		if (qmon->hdev != hdev)
			continue;

		list_for_each_safe(cq, nq, &qmon->mqlist) {
			mqd = list_entry(cq, struct monitor_queue_dentry, list);
			knav_qmssm_unregister_queue_item(qmon, mqd);
		}

		debugfs_remove_recursive(qmon->mq_root);
		list_del(cd);
		kfree(qmon->name);
		devm_kfree(qmssm_dev(qmon), qmon);
	}
out:
	mutex_unlock(&knav_qmssm_lock);
}
EXPORT_SYMBOL(knav_qmssm_unregister);

int knav_qmssm_register(struct hwqueue_device *hdev)
{
	struct knav_qmssm *qmon = NULL;
	int ret = 0;
	int len = 0;

	char buff[TEMP_BUFF_SIZE] = {0};

	if (WARN_ON(!hdev))
		return -ENODEV;

	mutex_lock(&knav_qmssm_lock);

	qmon = devm_kzalloc(hdev->dev, sizeof(*qmon), GFP_KERNEL);
	if (!qmon) {
		ret = -ENOSPC;
		goto out;
	}

	qmon->hdev = hdev;

	len = snprintf(buff, TEMP_BUFF_SIZE, "%s_%s", knav_qmssm_prefix,
			dev_name(hdev->dev));
	buff[len] = '\0';
	qmon->name = kstrdup(buff, GFP_KERNEL);
	if (!qmon->name) {
		devm_kfree(hdev->dev, qmon);
		goto out;
	}

	qmon->mq_root = debugfs_create_dir(buff, NULL);

	qmon->mq_register = debugfs_create_file(knav_qmssm_register_str,
		0220, qmon->mq_root, qmon, &knav_register_queue_fops);

	qmon->mq_unregister = debugfs_create_file(knav_qmssm_unregister_str,
		0220, qmon->mq_root, qmon, &knav_unregister_queue_fops);

	qmon->ilogger.mq_interval = debugfs_create_file(knav_qmssm_interval,
		0220, qmon->mq_root, qmon, &knav_interval_legger_fops);

	qmon->ilogger.interval_ms = MONITOR_INTERVAL_MS;

	INIT_LIST_HEAD(&qmon->mqlist);
	mutex_init(&qmon->mqlock);
	list_add(&qmon->list, &knav_qmssm_list);
	hdev->qmon = qmon;
	dev_info(hdev->dev, "register %s\n", qmon->name);

	knav_qmssm_setup_parm_base_queue(qmon);

out:
	mutex_unlock(&knav_qmssm_lock);

	return ret;
}
EXPORT_SYMBOL(knav_qmssm_register);
