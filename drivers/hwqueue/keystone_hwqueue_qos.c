/*
 * Keystone hardware queue driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/hwqueue.h>
#include <linux/ktree.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <asm/div64.h>

#include "hwqueue_internal.h"
#include "keystone_hwqueue.h"
#include "keystone_qos.h"

#define	KHWQ_QOS_TIMER_INTERVAL	(HZ * 10)
#define BITMASK(s, n)		(BITS(n) << (s))

static int khwq_qos_write_cmd(struct khwq_qos_info *info, u32 cmd)
{
	struct khwq_pdsp_info *pdsp;
	struct khwq_device *kdev;
	unsigned long timeo;
	u32 result;

	pdsp = info->pdsp;
	kdev = info->kdev;

	dev_dbg(kdev->dev, "=== command <-- %08x\n", cmd);
	__raw_writel(cmd, &pdsp->qos_command[0]);

	timeo = jiffies + msecs_to_jiffies(QOS_COMMAND_TIMEOUT);
	do {
		result = __raw_readl(&pdsp->qos_command[0]);
		udelay(QOS_COMMAND_DELAY);
	} while ((result & 0xff) && time_before(jiffies, timeo));

	if (result & 0xff) {
		dev_err(kdev->dev, "=== result --> lockup [%x]!\n",
			result);
		return -EBUSY;
	}

	result = __raw_readl(&pdsp->qos_command[1]);

	if (result != QOS_RETCODE_SUCCESS) {
		dev_err(kdev->dev, "=== result --> %08x ! failed\n", result);
		return -EIO;
	} else {
		dev_dbg(kdev->dev, "=== result --> %08x\n", result);
		return 0;
	}
}

static void khwq_qos_dump_words(struct khwq_device *kdev, u32 ofs, u32 *data,
				int words, const char *dir)
{
	switch (words) {
	case 4: dev_dbg(kdev->dev, "==== [%04x] %s %08x %08x %08x %08x\n",
			ofs, dir, data[0], data[1], data[2], data[3]);
		break;
	case 3: dev_dbg(kdev->dev, "==== [%04x] %s %08x %08x %08x\n",
			ofs, dir, data[0], data[1], data[2]);
		break;
	case 2: dev_dbg(kdev->dev, "==== [%04x] %s %08x %08x\n",
			ofs, dir, data[0], data[1]);
		break;
	case 1: dev_dbg(kdev->dev, "==== [%04x] %s %08x\n",
			ofs, dir, data[0]);
	case 0:
		break;
	default:
		BUG();
		break;
	}
}

static void khwq_qos_write_words(struct khwq_device *kdev, u32 ofs,
				 u32 __iomem *out, u32 *in, int words)
{
	int i;
	khwq_qos_dump_words(kdev, ofs, in, words, "<--");
	for (i = 0; i < words; i++)
		__raw_writel(in[i], &out[i]);
}

static void khwq_qos_read_words(struct khwq_device *kdev, u32 ofs, u32 *out,
				u32 __iomem *in, int words)
{
	int i;
	for (i = 0; i < words; i++)
		out[i] = __raw_readl(&in[i]);
	khwq_qos_dump_words(kdev, ofs, out, words, "-->");
}

static int khwq_qos_write_shadow(struct khwq_qos_info *info, u32 options,
				 u32 ofs, u32 *data, u32 words)
{
	u32 __iomem *out = info->pdsp->command + QOS_SHADOW_OFFSET + ofs;
	struct khwq_device *kdev = info->kdev;
	int words_left = words;
	u32 command;
	int error;

	assert_spin_locked(&info->lock);

	while (data && words_left) {
		words = min(words_left, 4);
		khwq_qos_write_words(kdev, ofs, out, data, words);
		ofs	   += words * sizeof(u32);
		out	   += words;
		data	   += words;
		words_left -= words;
	}

	command = QOS_CMD_PORT_SHADOW | QOS_COPY_SHADOW_TO_ACTIVE | options;
	error = khwq_qos_write_cmd(info, command);

	return error;
}

static int khwq_qos_read_shadow(struct khwq_qos_info *info, u32 options,
				u32 ofs, u32 *data, u32 words)
{
	u32 __iomem *in = info->pdsp->command + QOS_SHADOW_OFFSET + ofs;
	struct khwq_device *kdev = info->kdev;
	int words_left = words;
	u32 command;
	int error;

	assert_spin_locked(&info->lock);

	command = QOS_CMD_PORT_SHADOW | QOS_COPY_ACTIVE_TO_SHADOW | options;
	error = khwq_qos_write_cmd(info, command);
	if (error)
		return error;

	while (data && words_left) {
		words = min(words_left, 4);
		khwq_qos_read_words(kdev, ofs, data, in, words);
		ofs	   += words * sizeof(u32);
		in	   += words;
		data	   += words;
		words_left -= words;
	}

	return 0;
}

static int khwq_qos_program_drop_sched(struct khwq_qos_info *info)
{
	u32 config[4];

	config[0] = (info->drop_cfg.qos_ticks << 8 |
		     info->drop_cfg.drop_ticks);
	config[1] = info->drop_cfg.seed[0];
	config[2] = info->drop_cfg.seed[1];
	config[3] = info->drop_cfg.seed[2];

	return khwq_qos_write_shadow(info, QOS_DROP_SCHED_CFG << 24, 0,
				     config, 4);
}

static int khwq_qos_request_stats(struct khwq_qos_info *info, int index)
{
	struct khwq_qos_stats *stats = &info->stats;
	struct khwq_device *kdev = info->kdev;
	u32 __iomem *ofs;
	u64 *to;
	u32 command;
	int error;
	
	spin_lock_bh(&info->lock);

	ofs = info->pdsp->command + QOS_STATS_OFFSET;

	command = (QOS_CMD_STATS_REQUEST | (0x8f << 8) |
			(index << 16));
	error = khwq_qos_write_cmd(info, command);
	if (error) {
		dev_err(kdev->dev, "failed to request stats for block %d\n",
			index);
		goto out;
	}

	to = (stats->data + index * 0x20);

	to[0] += __raw_readl(&ofs[0]);
	to[1] += __raw_readl(&ofs[2]);
	to[2] += __raw_readl(&ofs[4]);
	to[3] += __raw_readl(&ofs[5]);

	dev_dbg(kdev->dev, "%d %llx %llx %llx %llx\n",
				index, to[0], to[1], to[2], to[3]);

out:
	spin_unlock_bh(&info->lock);
	return error;
}

static int khwq_qos_update_stats(struct khwq_qos_info *info)
{
	struct khwq_qos_stats *stats = &info->stats;
	int i, error;
	
	for (i = (stats->count - 1); i >= stats->start; i--) {
		if (!test_bit(i, stats->avail)) {
			error = khwq_qos_request_stats(info, i);
			if (error)
				return error;
		}
	}

	return 0;
}

static void khwq_qos_timer(unsigned long arg)
{
	struct khwq_qos_info *info = (struct khwq_qos_info *)arg;
	struct khwq_device *kdev = info->kdev;
	int error;

	error = khwq_qos_update_stats(info);
	if (error) {
		dev_err(kdev->dev, "error updating stats\n");
		return;
	}

	info->timer.expires = jiffies + KHWQ_QOS_TIMER_INTERVAL;
	add_timer(&info->timer);

	return;
}

struct khwq_qos_attr {
	struct attribute	attr;
	int			offset;
};

#define __KHWQ_QOS_STATS_ATTR(_name, _offset)				\
(struct khwq_qos_attr) {					\
	.attr = { .name = __stringify(_name), .mode = S_IRUGO },	\
	.offset = _offset,						\
}

#define KHWQ_QOS_STATS_ATTR(_name, _offset)		\
	&__KHWQ_QOS_STATS_ATTR(_name, _offset).attr

static struct attribute *khwq_qos_stats_attrs[] = {
	KHWQ_QOS_STATS_ATTR(bytes_forwarded,	0x00),
	KHWQ_QOS_STATS_ATTR(bytes_discarded,	0x08),
	KHWQ_QOS_STATS_ATTR(packets_forwarded,	0x10),
	KHWQ_QOS_STATS_ATTR(packets_discarded,	0x18),
	NULL
};

static ssize_t khwq_qos_stats_attr_show(struct kobject *kobj,
					struct attribute *_attr, char *buf)
{
	struct khwq_qos_attr *attr;
	struct khwq_qos_stats_class *class;
	struct khwq_qos_info *info;
	struct khwq_qos_stats *stats;
	int offset, index, error;
	u64 *val;

	class = container_of(kobj, struct khwq_qos_stats_class, kobj);
	attr = container_of(_attr, struct khwq_qos_attr, attr);
	info = class->info;
	stats = &info->stats;
	index = class->stats_block_idx;
	offset = attr->offset;

	error = khwq_qos_request_stats(info, index);
	if (error)
		return error;

	val = stats->data + (index * 0x20) + offset;

	return snprintf(buf, PAGE_SIZE, "%lld\n", *val);
}

static struct kobj_type khwq_qos_stats_ktype = {
	.sysfs_ops = &(struct sysfs_ops) { .show = khwq_qos_stats_attr_show },
	.default_attrs = khwq_qos_stats_attrs,
};

static void khwq_qos_free_drop_policy(struct khwq_device *kdev,
				      struct khwq_qos_drop_policy *policy)
{
	list_del(&policy->list);
	kobject_del(&policy->kobj);
	kobject_put(&policy->kobj);
	devm_kfree(kdev->dev, policy);
}

static void khwq_qos_free_drop_policies(struct khwq_device *kdev,
					struct khwq_qos_info *info)
{
	struct khwq_qos_drop_policy *policy;

	while (!list_empty(&info->drop_policies)) {
		policy = list_first_entry(&info->drop_policies,
					  struct khwq_qos_drop_policy,
					  list);
		khwq_qos_free_drop_policy(kdev, policy);
	}
}

static int khwq_program_drop_policy(struct khwq_qos_info *info,
				    struct khwq_qos_drop_policy *policy,
				    bool sync)
{
	int error;
	u32 val, time_constant, diff;
	u32 thresh_recip;

	val = (policy->acct == QOS_BYTE_ACCT) ? BIT(0) : 0;

	error = khwq_qos_set_drop_cfg_unit_flags(info, policy->drop_cfg_idx,
						 val, false);
	if (error)
		return error;

	error = khwq_qos_set_drop_cfg_mode(info, policy->drop_cfg_idx,
					   policy->mode, false);
	if (error)
		return error;

	error = khwq_qos_set_drop_cfg_tail_thresh(info, policy->drop_cfg_idx,
						  policy->limit, false);
	if (error)
		return error;

	if (policy->mode == QOS_TAILDROP)
		return 0;

	error = khwq_qos_set_drop_cfg_red_low(info, policy->drop_cfg_idx,
					      policy->red_low, false);
	if (error)
		return error;

	error = khwq_qos_set_drop_cfg_red_high(info, policy->drop_cfg_idx,
					       policy->red_high, false);

	val = policy->half_life / 100;
	time_constant = ilog2((3 * val) + 1);

	error = khwq_qos_set_drop_cfg_time_const(info, policy->drop_cfg_idx,
						   time_constant, false);

	diff = ((policy->red_high - policy->red_low) >> time_constant);

	thresh_recip = 1 << 31;
	thresh_recip /= (diff >> 1);

	error = khwq_qos_set_drop_cfg_thresh_recip(info, policy->drop_cfg_idx,
						   thresh_recip, false);

	return error;
}

#define __KHWQ_QOS_DROP_POLICY_ATTR(_name)			\
(struct khwq_qos_attr) {					\
	.attr = { .name = __stringify(_name), .mode = S_IRUGO|S_IWUSR },\
	.offset = offsetof(struct khwq_qos_drop_policy, _name),	\
}

#define KHWQ_QOS_DROP_POLICY_ATTR(_name)		\
	&__KHWQ_QOS_DROP_POLICY_ATTR(_name).attr

static struct attribute *khwq_qos_drop_policy_attrs[] = {
	KHWQ_QOS_DROP_POLICY_ATTR(limit),
	KHWQ_QOS_DROP_POLICY_ATTR(red_low),
	KHWQ_QOS_DROP_POLICY_ATTR(red_high),
	KHWQ_QOS_DROP_POLICY_ATTR(half_life),
	KHWQ_QOS_DROP_POLICY_ATTR(max_drop_prob),
	NULL
};

static struct attribute *khwq_qos_drop_policy_taildrop_attrs[] = {
	KHWQ_QOS_DROP_POLICY_ATTR(limit),
	NULL
};

static ssize_t khwq_qos_drop_policy_attr_show(struct kobject *kobj,
					      struct attribute *_attr,
					      char *buf)
{
	struct khwq_qos_drop_policy *policy;
	struct khwq_qos_attr *attr;
	int offset;
	u32 *val;

	policy = container_of(kobj, struct khwq_qos_drop_policy, kobj);
	attr = container_of(_attr, struct khwq_qos_attr, attr);
	offset = attr->offset;

	val = (((void *)policy) + offset);

	return snprintf(buf, PAGE_SIZE, "%d\n", *val);
}

static ssize_t khwq_qos_drop_policy_attr_store(struct kobject *kobj,
					       struct attribute *_attr,
					       const char *buf, size_t size)
{
	struct khwq_qos_drop_policy *policy;
	struct khwq_qos_info *info;
	struct khwq_device *kdev;
	struct khwq_qos_attr *attr;
	int offset, error, field;
	u32 *val;

	policy = container_of(kobj, struct khwq_qos_drop_policy, kobj);
	attr = container_of(_attr, struct khwq_qos_attr, attr);
	offset = attr->offset;
	info = policy->info;
	kdev = info->kdev;

	error = kstrtouint(buf, 0, &field);
	if (error)
		return error;

	val = (((void *)policy) + offset);
	*val = field;

	error = khwq_program_drop_policy(info, policy, false);
	if (error) {
		khwq_qos_free_drop_policy(kdev, policy);
		policy->drop_cfg_idx = -1;
		return error;
	}

	error = khwq_qos_sync_drop_cfg(info, -1);
	if (error)
		dev_err(kdev->dev, "failed to sync drop configs\n");

	return size;
}

static struct kobj_type khwq_qos_policy_taildrop_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show	= khwq_qos_drop_policy_attr_show,
		.store	= khwq_qos_drop_policy_attr_store,
	},
	.default_attrs = khwq_qos_drop_policy_taildrop_attrs,
};

static struct kobj_type khwq_qos_policy_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show	= khwq_qos_drop_policy_attr_show,
		.store	= khwq_qos_drop_policy_attr_store,
	},
	.default_attrs = khwq_qos_drop_policy_attrs,
};

static int khwq_program_drop_policies(struct khwq_qos_info *info)
{
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_drop_policy *policy;
	int error = 0;

	for_each_policy(info, policy) {
		if (!policy->usecount && policy != info->default_drop_policy)
			continue;

		policy->drop_cfg_idx = khwq_qos_alloc_drop_cfg(info);
		if (policy->drop_cfg_idx < 0) {
			dev_err(kdev->dev, "too many drop policies\n");
			error = -EOVERFLOW;
			break;
		}

		error = khwq_program_drop_policy(info, policy, false);
		if (error) {
			khwq_qos_free_drop_policy(kdev, policy);
			policy->drop_cfg_idx = -1;
			break;
		}

		dev_dbg(kdev->dev, "added policy %s\n", policy->name);
	}

	if (!error) {
		error = khwq_qos_sync_drop_cfg(info, -1);
		if (error)
			dev_err(kdev->dev, "failed to sync drop configs\n");
	}

	return error;
}

static struct khwq_qos_stats_class *
khwq_qos_init_stats_class(struct khwq_qos_info *info, const char *name)
{
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_stats_class *class;
	struct khwq_qos_stats *stats = &info->stats;
	int idx, error;

	class = devm_kzalloc(kdev->dev, sizeof(*class), GFP_KERNEL);
	if (!class)
		return NULL;
	class->name = name;
	class->info = info;

	spin_lock_bh(&info->lock);

	idx = find_last_bit(stats->avail, stats->count);
	if (idx < stats->count)
		clear_bit(idx, stats->avail);
	else {
		spin_unlock_bh(&info->lock);
		return NULL;
	}

	spin_unlock_bh(&info->lock);

	class->stats_block_idx = idx;

	list_add_tail(&class->list, &info->stats_classes);

	error = kobject_init_and_add(&class->kobj, &khwq_qos_stats_ktype,
				info->kobj_stats, class->name);
	if (error) {
		dev_err(kdev->dev, "failed to create sysfs file\n");
		return NULL;
	}

	return class;
}

static void khwq_qos_free_stats_class(struct khwq_qos_info *info,
				      struct khwq_qos_stats_class *class)
{
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_stats *stats = &info->stats;
	
	spin_lock_bh(&info->lock);

	list_del(&class->list);
	set_bit(class->stats_block_idx, stats->avail);

	spin_unlock_bh(&info->lock);

	kobject_del(&class->kobj);
	kobject_put(&class->kobj);
	devm_kfree(kdev->dev, class);
}

static void khwq_qos_free_stats_classes(struct khwq_device *kdev,
					struct khwq_qos_info *info)
{
	struct khwq_qos_stats_class *class;

	while (!list_empty(&info->stats_classes)) {
		class = list_first_entry(&info->stats_classes,
					 struct khwq_qos_stats_class,
					 list);
		khwq_qos_free_stats_class(info, class);
	}
}

static struct khwq_qos_stats_class *
khwq_qos_find_stats_class(struct khwq_qos_info *info, const char *name)
{
	struct khwq_qos_stats_class *class;
	list_for_each_entry(class, &info->stats_classes, list)
		if (!strcmp(class->name, name))
			return class;
	return NULL;
}

static struct khwq_qos_drop_policy *
khwq_qos_find_drop_policy(struct khwq_qos_info *info, const char *name)
{
	struct khwq_qos_drop_policy *policy;
	list_for_each_entry(policy, &info->drop_policies, list)
		if (!strcmp(policy->name, name))
			return policy;
	return NULL;
}

static struct khwq_qos_drop_policy *
khwq_qos_get_drop_policy(struct khwq_device *kdev, struct khwq_qos_info *info,
			 struct device_node *node)
{
	struct khwq_qos_drop_policy *policy;
	u32 length, elements, temp[4];
	const char *name;
	int error;

	policy = devm_kzalloc(kdev->dev, sizeof(*policy), GFP_KERNEL);
	if (!policy)
		return ERR_PTR(-ENOMEM);

	policy->info = info;

	error = of_property_read_string(node, "label", &name);
	policy->name = (error < 0) ? node->name : name;
	if (khwq_qos_find_drop_policy(info, policy->name)) {
		dev_err(kdev->dev, "duplicate drop policy %s\n", policy->name);
		devm_kfree(kdev->dev, policy);
		return ERR_PTR(-EINVAL);
	}

	policy->mode = QOS_TAILDROP;

	policy->acct = QOS_BYTE_ACCT;
	if (of_find_property(node, "packet-units", NULL))
		policy->acct = QOS_PACKET_ACCT;

	error = of_property_read_u32(node, "limit", &policy->limit);
	if (error < 0)
		policy->limit = 0;

	if (!of_find_property(node, "random-early-drop", &length))
		goto done;

	policy->mode = QOS_RED;

	if (policy->acct == QOS_PACKET_ACCT) {
		dev_err(kdev->dev, "red policy must account bytes\n");
		devm_kfree(kdev->dev, policy);
		return ERR_PTR(-EINVAL);
	}

	elements = length / sizeof(u32);
	if (elements < 1 || elements > 4) {
		dev_err(kdev->dev, "invalid number of elements in red info\n");
		devm_kfree(kdev->dev, policy);
		return ERR_PTR(-EINVAL);
	}

	error = of_property_read_u32_array(node, "random-early-drop", temp,
					   elements);
	if (error < 0) {
		dev_err(kdev->dev, "could not obtain red info\n");
		devm_kfree(kdev->dev, policy);
		return ERR_PTR(-EINVAL);
	}

	policy->red_low = temp[0];

	policy->red_high = 2 * policy->red_low;
	if (elements > 1)
		policy->red_high = temp[1];

	policy->max_drop_prob = 2;
	if (elements > 2)
		policy->max_drop_prob = temp[2];
	if (policy->max_drop_prob >= 100) {
		dev_warn(kdev->dev, "invalid max drop prob %d on policy %s, taking defaults\n",
			 policy->max_drop_prob, policy->name);
		policy->max_drop_prob = 2;
	}

	policy->half_life = 2000;
	if (elements > 3)
		policy->half_life = temp[3];

done:
	if (of_find_property(node, "default", NULL)) {
		if (info->default_drop_policy) {
			dev_warn(kdev->dev, "duplicate default policy %s\n",
				 policy->name);
		} else
			info->default_drop_policy = policy;
	}
	
	if (policy->mode == QOS_RED)
		error = kobject_init_and_add(&policy->kobj,
				     &khwq_qos_policy_ktype,
				     info->kobj_policies, policy->name);
	else
		error = kobject_init_and_add(&policy->kobj,
				     &khwq_qos_policy_taildrop_ktype,
				     info->kobj_policies, policy->name);

	if (error) {
		dev_err(kdev->dev, "failed to create sysfs "
				"entries for policy %s\n", policy->name);
		devm_kfree(kdev->dev, policy);
		return ERR_PTR(-EINVAL);;
	}

	return policy;
}

static int khwq_qos_get_drop_policies(struct khwq_device *kdev,
				      struct khwq_qos_info *info,
				      struct device_node *node)
{
	struct khwq_qos_drop_policy *policy;
	struct device_node *child;
	int error = 0;

	for_each_child_of_node(node, child) {
		policy = khwq_qos_get_drop_policy(kdev, info, child);
		if (IS_ERR_OR_NULL(policy)) {
			error = PTR_ERR(policy);
			break;
		}
		list_add_tail(&policy->list, &info->drop_policies);
	}

	if (!error && !info->default_drop_policy) {
		dev_err(kdev->dev, "must specify a default drop policy!\n");
		error = -ENOENT;
	}

	if (!error)
		return 0;

	khwq_qos_free_drop_policies(kdev, info);

	return error;
}

static struct khwq_qos_shadow *
khwq_find_shadow(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		 int idx, int offset, bool internal)
{
	struct khwq_qos_shadow *shadow;
	struct khwq_device *kdev = info->kdev;

	shadow = &info->shadows[type];

	idx = khwq_qos_id_to_idx(idx);
	if (idx >= shadow->count || offset >= shadow->size) {
		dev_err(kdev->dev, "bad shadow access, idx %d, count %d, "
			"offset %d, size %d\n",
			idx, shadow->count, offset, shadow->size);
		return NULL;
	}

	if (!internal && test_bit(idx, shadow->avail)) {
		dev_err(kdev->dev, "idx %d not in use\n", idx);
		return NULL;
	}

	return shadow;
}

int khwq_qos_get(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		   const char *name, int idx, int offset, int startbit,
		   int nbits, u32 *value)
{
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_shadow *shadow;
	u32 *element;

	shadow = khwq_find_shadow(info, type, idx, offset, false);
	if (WARN_ON(!shadow))
		return -EINVAL;

	idx = khwq_qos_id_to_idx(idx);
	offset += idx * shadow->size;
	element = shadow->data + offset;
	*value = (*element >> startbit) & BITS(nbits);
	dev_dbg(kdev->dev, "=== %s(%d) --> %x\n", name, idx, *value);
	return 0;
}

int khwq_qos_set(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		   const char *name, int idx, int offset, int startbit,
		   int nbits, bool sync, u32 value, bool internal)
{
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_shadow *shadow;
	u32 *element;
	u32 outval;

	shadow = khwq_find_shadow(info, type, idx, offset, internal);
	if (WARN_ON(!shadow))
		return -EINVAL;

	idx = khwq_qos_id_to_idx(idx);
	offset += idx * shadow->size;
	element = shadow->data + offset;
	outval  = *element & ~BITMASK(startbit, nbits);
	WARN_ON(value & ~BITS(nbits));
	outval |= (value & BITS(nbits)) << startbit;
	dev_dbg(kdev->dev, "=== %s(%d) <-- %x [%08x --> %08x]\n", name, idx,
		value, *element, outval);
	*element = outval;

	set_bit(idx, shadow->dirty);
	if (sync)
		return shadow->sync ? shadow->sync(shadow, idx) : - ENOSYS;
	return 0;
}

int khwq_qos_control(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		       enum khwq_qos_control_type ctrl, int idx, u32 arg,
		       bool internal)
{
	struct khwq_qos_shadow *shadow;

	shadow = khwq_find_shadow(info, type, idx, 0, internal);
	if (WARN_ON(!shadow))
		return -EINVAL;

	idx = khwq_qos_id_to_idx(idx);
	if (!shadow->control)
		return -ENOSYS;
	return shadow->control(shadow, ctrl, idx, arg);
}

int khwq_qos_sync(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		    int idx, bool internal)
{
	struct khwq_pdsp_info *pdsp;
	struct khwq_qos_shadow *shadow;
	struct khwq_device *kdev = info->kdev;
	int error = 0;

	if (type < 0 || type >= QOS_MAX_SHADOW) {
		dev_err(kdev->dev, "bad shadow type %d\n", type);
		return -EINVAL;
	}

	if (idx >= 0) {
		shadow = khwq_find_shadow(info, type, idx, 0, internal);
		if (WARN_ON(!shadow))
			return -EINVAL;
		idx = khwq_qos_id_to_idx(idx);
		return shadow->sync ? shadow->sync(shadow, idx) : - ENOSYS;
	}

	/* sync all */
	for_each_pdsp(kdev, pdsp) {
		info = pdsp->qos_info;
		if (!info)
			continue;
		shadow = &info->shadows[type];

		error = shadow->sync ? shadow->sync(shadow, idx) : 0;
		if (error)
			break;
	}

	return error;
}

int khwq_qos_alloc(struct khwq_qos_info *info, enum khwq_qos_shadow_type type)
{
	struct khwq_pdsp_info *pdsp = info->pdsp;
	struct khwq_qos_shadow *shadow;
	int idx;

	shadow = &info->shadows[type];
	spin_lock_bh(&info->lock);

	idx = find_last_bit(shadow->avail, shadow->count);
	if (idx < shadow->count) {
		clear_bit(idx, shadow->avail);
		spin_unlock_bh(&info->lock);
		return khwq_qos_make_id(pdsp->id, idx);
	}

	spin_unlock_bh(&info->lock);

	return -ENOSPC;
}

int khwq_qos_free(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		    int idx)
{
	struct khwq_qos_shadow *shadow;

	shadow = khwq_find_shadow(info, type, idx, 0, false);
	if (WARN_ON(!shadow))
		return -EINVAL;

	idx = khwq_qos_id_to_idx(idx);
	if (WARN_ON(test_bit(idx, shadow->dirty)	||
		    test_bit(idx, shadow->running)	||
		    test_bit(idx, shadow->avail)))
	    return -EBUSY;

	set_bit(idx, shadow->avail);

	return 0;
}

static int khwq_qos_alloc_drop_queue(struct khwq_qos_info *info, int _idx)
{
	struct khwq_pdsp_info *pdsp = info->pdsp;
	struct khwq_qos_shadow *shadow;
	int idx, base, count;

	shadow = &info->shadows[QOS_DROP_QUEUE_CFG];
	base   = info->drop_sched_queue_base;
	count  = shadow->count;


	idx = _idx - base;

	if (test_and_clear_bit(idx, shadow->avail))
		return khwq_qos_make_id(pdsp->id, idx);
	else
		return -EBUSY;

	return -ENODEV;
}

static int khwq_qos_free_drop_queue(struct khwq_qos_info *info, int idx)
{
	return khwq_qos_free(info, QOS_DROP_QUEUE_CFG, idx);
}

#define	is_even(x)	(((x) & 1) == 0)

static inline int khwq_qos_alloc_sched_port(struct khwq_qos_info *info,
			  int parent_port_idx, bool port_pair)
{
	struct khwq_pdsp_info *pdsp = info->pdsp;
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_SCHED_PORT_CFG];
	int parent_idx, idx, idx_odd;

	parent_idx = (parent_port_idx < 0) ? shadow->count :
			khwq_qos_id_to_idx(parent_port_idx);

	spin_lock_bh(&info->lock);

	idx = find_last_bit(shadow->avail, parent_idx);
	if (idx >= parent_idx)
		goto fail;

	if (port_pair) {
		do {
			idx_odd = idx;
			idx = find_last_bit(shadow->avail, idx_odd);
			if (idx >= idx_odd)
				goto fail;
		} while (!is_even(idx) || (idx_odd != (idx + 1)));
		clear_bit(idx_odd, shadow->avail);
	}

	clear_bit(idx, shadow->avail);
	spin_unlock_bh(&info->lock);
	return khwq_qos_make_id(pdsp->id, idx);

fail:
	spin_unlock_bh(&info->lock);
	dev_err(info->kdev->dev, "QoS port allocation failure\n");
	return -ENOSPC;
}

static int khwq_qos_free_sched_port(struct khwq_qos_info *info, int idx)
{
	return khwq_qos_free(info, QOS_SCHED_PORT_CFG, idx);
}

static int khwq_qos_sched_port_enable(struct khwq_qos_shadow *shadow, int idx,
				       bool enable)
{
	struct khwq_qos_info *info = shadow->info;
	struct khwq_device *kdev = info->kdev;
	int error = 0, start = idx, end = idx + 1;
	u32 command;

	if (WARN_ON(idx >= shadow->count))
		return -EINVAL;

	if (idx < 0) {
		start = 0;
		end = shadow->count;
	}

	/* fail if our state is dirty */
	for (idx = start; idx < end; idx ++) {
		if (WARN_ON(test_bit(idx, shadow->dirty)))
			return -EBUSY;
	}

	for (idx = start; idx < end; idx ++) {
		if (enable && test_bit(idx, shadow->running)) {
			dev_warn(kdev->dev, "forced enable on running %s %d\n",
				 shadow->name, idx);
		}
		if (!enable && !test_bit(idx, shadow->running)) {
			dev_dbg(kdev->dev, "forced disable on halted %s %d\n",
				 shadow->name, idx);
		}
		command = (QOS_CMD_ENABLE_PORT			|
			   (enable ? QOS_ENABLE : QOS_DISABLE)	|
			   (shadow->start + idx) << 16);
		error = khwq_qos_write_cmd(info, command);
		if (error) {
			dev_err(kdev->dev, "failed to %s %s index %d\n",
				enable ? "enable" : "disable", shadow->name, idx);
			break;
		}
		if (enable)
			set_bit(idx, shadow->running);
		else
			clear_bit(idx, shadow->running);
	}

	return 0;
}

static int khwq_qos_sched_port_get_input(struct khwq_qos_shadow *shadow,
					 int idx, u32 queue)
{
	struct khwq_qos_info *info = shadow->info;
	struct khwq_device *kdev = info->kdev;

	if (WARN_ON(idx < 0 || idx >= shadow->count ||
		    test_bit(idx, shadow->avail)))
		return -EINVAL;
	if (queue >= shadow->info->inputs_per_port) {
		dev_err(kdev->dev, "requested queue %d out of range\n",
			queue);
		return -EINVAL;
	}

	return (shadow->start + idx) * info->inputs_per_port +
		queue + info->sched_port_queue_base;
}

static int __khwq_qos_control_sched_port(struct khwq_qos_shadow *shadow,
					 enum khwq_qos_control_type ctrl,
					 int idx, u32 arg)
{
	struct khwq_qos_info *info = shadow->info;
	int error = 0;

	spin_lock_bh(&info->lock);

	switch (ctrl) {

	case QOS_CONTROL_ENABLE:
		error = khwq_qos_sched_port_enable(shadow, idx, !!arg);
		break;

	case QOS_CONTROL_GET_INPUT:
		error = khwq_qos_sched_port_get_input(shadow, idx, arg);
		break;

	default:
		error = -ENOSYS;
		break;
	}

	spin_unlock_bh(&info->lock);

	return error;
}

static int khwq_qos_sync_shadow_unified(struct khwq_qos_shadow *shadow, int idx)
{
	struct khwq_qos_info *info = shadow->info;
	int count, error, offset;
	u32 *from;

	if (WARN_ON(idx >= shadow->count))
		return -EINVAL;

	spin_lock_bh(&info->lock);

	/* now fill in our data */
	if (idx < 0) {
		offset	= shadow->start * shadow->size;
		from	= shadow->data;
		count	= (shadow->size * shadow->count) / sizeof(u32);
	} else {
		offset	= (shadow->start + idx) * shadow->size;
		from	= shadow->data + idx * shadow->size;
		count	= shadow->size / sizeof(u32);
	}

	/* first get the pdsp to copy active config to shadow area */
	error = khwq_qos_read_shadow(info, shadow->type << 24, 0, NULL, 0);
	if (error)
		goto bail;

	/* now fill in our data and write it back */
	error = khwq_qos_write_shadow(info, shadow->type << 24, offset,
				      from, count);
	if (error)
		goto bail;

	if (idx < 0) {
		count = BITS_TO_LONGS(shadow->count) * sizeof(long);
		memset(shadow->dirty, 0, count);
	} else
		clear_bit(idx, shadow->dirty);

	error = 0;

bail:
	spin_unlock_bh(&info->lock);

	return error;
}

static int khwq_qos_sync_shadow_single(struct khwq_qos_shadow *shadow, int idx)
{
	struct khwq_qos_info *info = shadow->info;
	struct khwq_pdsp_info *pdsp = info->pdsp;
	int count, error = 0, start = idx, end = idx + 1;
	u32 __iomem *to;
	u32 *from;
	u32 command;

	if (WARN_ON(idx >= shadow->count))
		return -EINVAL;

	if (idx < 0) {
		start = 0;
		end = shadow->count;
	}

	spin_lock_bh(&info->lock);

	for (idx = start; idx < end; idx ++) {
		from	= shadow->data + (idx * shadow->size);
		to	= pdsp->command + QOS_SHADOW_OFFSET;
		count	= shadow->size / sizeof(u32);

		command = (shadow->start + idx) << 16 | shadow->type << 24;
		error = khwq_qos_write_shadow(info, command, 0, from, count);
		if (error)
			break;

		clear_bit(idx, shadow->dirty);
	}

	spin_unlock_bh(&info->lock);

	return error;
}

static void khwq_qos_free_shadow(struct khwq_qos_info *info,
				 enum khwq_qos_shadow_type type)
{
	struct khwq_qos_shadow *shadow = &info->shadows[type];
	struct khwq_device *kdev = info->kdev;

	if (shadow->data)
		devm_kfree(kdev->dev, shadow->data);
}

static int khwq_qos_init_shadow(struct khwq_qos_info *info,
				enum khwq_qos_shadow_type type, const char *name,
				struct device_node *node, bool unified)
{
	struct khwq_qos_shadow *shadow = &info->shadows[type];
	struct khwq_device *kdev = info->kdev;
	int error, size, alloc_size;
	u32 temp[3];

	shadow->info = info;
	shadow->name = name;

	error = of_property_read_u32_array(node, name, temp, 3);
	if (error < 0) {
		dev_err(kdev->dev, "invalid shadow config for %s\n",
			name);
		return -ENODEV;
	}

	shadow->start	= temp[0];
	shadow->count	= temp[1];
	shadow->size	= temp[2];
	shadow->type	= type;
	shadow->sync	= (unified ? khwq_qos_sync_shadow_unified :
				     khwq_qos_sync_shadow_single);
	if (type == QOS_SCHED_PORT_CFG)
		shadow->control	= __khwq_qos_control_sched_port;

	if (shadow->size % 4) {
		dev_err(kdev->dev, "misaligned shadow size for %s\n",
			name);
		return -ENODEV;
	}

	size = shadow->size * shadow->count;
	alloc_size = size + 3 * BITS_TO_LONGS(shadow->count) * sizeof(long);

	shadow->data = devm_kzalloc(kdev->dev, alloc_size, GFP_KERNEL);
	if (!shadow->data) {
		dev_err(kdev->dev, "shadow alloc failed for %s\n",
			name);
		return -ENOMEM;
	}
	shadow->dirty	= shadow->data + size;
	shadow->avail	= shadow->dirty + BITS_TO_LONGS(shadow->count);
	shadow->running	= shadow->avail  + BITS_TO_LONGS(shadow->count);

	/* mark all as available */
	memset(shadow->avail, 0xff,
	       BITS_TO_LONGS(shadow->count) * sizeof(long));

	return 0;
}

static int khwq_qos_init_stats(struct khwq_qos_info *info,
				struct device_node *node)
{
	struct khwq_qos_stats *stats = &info->stats;
	struct khwq_device *kdev = info->kdev;
	int error, size, alloc_size;
	u32 temp[2];

	error = of_property_read_u32_array(node, "statistics-profiles",
					   temp, 2);
	if (error < 0) {
		dev_err(kdev->dev, "invalid statistics config\n");
		return -ENODEV;
	}

	stats->start	= temp[0];
	stats->count	= temp[1];

	size = stats->count * sizeof(u64) * 4;
	alloc_size = size + 3 * BITS_TO_LONGS(stats->count) * sizeof(long);

	stats->data = devm_kzalloc(kdev->dev, alloc_size, GFP_KERNEL);
	if (!stats->data) {
		dev_err(kdev->dev, "stats alloc failed\n");
		return -ENOMEM;
	}

	stats->dirty	= stats->data + size;
	stats->avail	= stats->dirty + BITS_TO_LONGS(stats->count);
	stats->running	= stats->avail  + BITS_TO_LONGS(stats->count);

	/* mark all as available */
	memset(stats->avail, 0xff,
	       BITS_TO_LONGS(stats->count) * sizeof(long));

	return 0;
}

static void khwq_qos_free_stats(struct khwq_qos_info *info)
{
	struct khwq_qos_stats *stats = &info->stats;
	struct khwq_device *kdev = info->kdev;

	if (stats->data)
		devm_kfree(kdev->dev, stats->data);
}

static void __khwq_free_qos_range(struct khwq_device *kdev,
				  struct khwq_qos_info *info)
{
	int tree_index;

	khwq_qos_free_shadow(info, QOS_SCHED_PORT_CFG);
	khwq_qos_free_shadow(info, QOS_DROP_CFG_PROF);
	khwq_qos_free_shadow(info, QOS_DROP_OUT_PROF);
	khwq_qos_free_shadow(info, QOS_DROP_QUEUE_CFG);
	khwq_qos_free_stats(info);
	khwq_qos_free_drop_policies(kdev, info);
	khwq_qos_free_stats_classes(kdev, info);
	for (tree_index = 0; tree_index < info->qos_tree_count; ++tree_index)
		ktree_remove_tree(&info->qos_trees[tree_index]);
	kobject_del(info->kobj);
	kobject_put(info->kobj);
	kobject_del(info->kobj_stats);
	kobject_put(info->kobj_stats);
	kobject_del(info->kobj_policies);
	kobject_put(info->kobj_policies);
	devm_kfree(info->kdev->dev, info);
}

void khwq_free_qos_range(struct khwq_device *kdev,
			 struct khwq_range_info *range)
{
	if (range->qos_info)
		__khwq_free_qos_range(kdev, range->qos_info);
}

static int khwq_qos_prio_check(struct ktree_node *child, void *arg)
{
	struct khwq_qos_tree_node *node = arg;
	struct khwq_qos_tree_node *sibling = to_qnode(child);

	return ((sibling->priority != -1) &&
		(sibling->priority == node->priority)) ? -EINVAL : 0;
}

static int khwq_qos_lowprio_check(struct ktree_node *child, void *arg)
{
	struct khwq_qos_tree_node *node = arg;
	struct khwq_qos_tree_node *sibling = to_qnode(child);

	return ((sibling->low_priority != -1) &&
		(sibling->low_priority == node->low_priority)) ? -EINVAL : 0;
}

struct khwq_qos_drop_policy *
khwq_qos_inherited_drop_policy(struct khwq_qos_tree_node *node)
{
	for (; node; node = node->parent)
		if (node->drop_policy)
			return node->drop_policy;
	return NULL;
}

static int khwq_qos_drop_policy_check(struct ktree_node *node, void *arg)
{
	struct khwq_qos_info *info = arg;
	struct khwq_qos_tree_node *qnode = to_qnode(node);

	if (qnode->type != QOS_NODE_DEFAULT || qnode->drop_policy)
		return -EINVAL;
	return ktree_for_each_child(&qnode->node, khwq_qos_drop_policy_check,
				    info);
}

static void khwq_qos_get_tree_node(struct ktree_node *node)
{
	/* nothing for now */
}

static void khwq_qos_put_tree_node(struct ktree_node *node)
{
	struct khwq_qos_tree_node *qnode = to_qnode(node);
	kfree(qnode);
}

static ssize_t qnode_stats_class_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	struct khwq_qos_stats_class *class;

	class = qnode->stats_class;

	if (!class)
		return -ENODEV;

	return snprintf(buf, PAGE_SIZE, "%s\n", class->name);
}

static ssize_t qnode_drop_policy_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	struct khwq_qos_drop_policy *policy;

	policy = qnode->drop_policy;

	if (!policy)
		return -ENODEV;

	return snprintf(buf, PAGE_SIZE, "%s\n", policy->name);
}

static ssize_t qnode_weight_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->weight == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->weight);
}

static inline u64 khwq_qos_wrr_calc(
		u32 child_weight, u32 norm, u32 min_weight)
{
	u64 temp;

	temp = child_weight;
	temp *= norm;
	temp += min_weight / 2;
	do_div(temp, min_weight);

	return temp;
}

static void khwq_qos_wrr_norm(struct khwq_qos_tree_node *qnode, u32 credits[])
{
	int	i, child_low, child_high;
	u64	max_credits;
	u32	norm, min_credits;
	u32	min_weight, max_weight;

	memset(credits, 0, sizeof(qnode->child_weight));
	if (qnode->wrr_children == 0)
		return;

	/* Determine the range of WRR children */
	child_low = qnode->prio_children;
	child_high = child_low + qnode->wrr_children;

	/* Determine the lowest and highest WRR weights */
	max_weight = 0;
	min_weight = UINT_MAX;
	for (i = child_low; i < child_high; ++i) {
		if (qnode->child_weight[i] > max_weight)
			max_weight = qnode->child_weight[i];
		if (qnode->child_weight[i] < min_weight)
			min_weight = qnode->child_weight[i];
	}

	/* Calculate a conversion factor that won't cause overflow */
	norm = (qnode->acct == QOS_BYTE_ACCT) ?
			QOS_BYTE_NORMALIZATION_FACTOR :
			QOS_PACKET_NORMALIZATION_FACTOR;
	max_credits = khwq_qos_wrr_calc(max_weight, norm, min_weight);
	while (max_credits > (uint64_t)QOS_MAX_CREDITS) {
		norm /= 2;
		max_credits /= 2;
	}

	/* Warn if min_credits will end up very small */
	min_credits = khwq_qos_wrr_calc(min_weight, norm, min_weight);
	if (min_credits < QOS_MIN_CREDITS_WARN) {
		dev_warn(qnode->info->kdev->dev, "Warning: max/min weight "
			"ratio of %u on node %s may cause significant "
			"performance degradation!\n",
			(max_weight / min_weight), qnode->name);
	}

	/* Convert weights to credits */
	for (i = child_low; i < child_high; ++i)
		credits[i] = khwq_qos_wrr_calc(qnode->child_weight[i],
						norm, min_weight);
}

static ssize_t qnode_weight_store(struct khwq_qos_tree_node *qnode,
				  const char *buf, size_t size)
{
	struct khwq_qos_tree_node *parent = qnode->parent;
	struct khwq_qos_info *info = qnode->info;
	u32 wrr_credits[QOS_MAX_CHILDREN];
	unsigned int weight;
	int i, error, val, idx;

	if (!parent || (parent->wrr_children == 0))
		return -EINVAL;

	error = kstrtouint(buf, 0, &weight);
	if (error)
		return error;

	if (weight == 0 || weight > QOS_MAX_WEIGHT)
		return -EINVAL;

	qnode->weight = weight;
	parent->child_weight[qnode->parent_input] = weight;

	khwq_qos_wrr_norm(parent, wrr_credits);

	idx = parent->sched_port_idx;
	for (i = parent->child_count - 1; i >= 0; --i) {
		int port, queue;

		if (parent->is_joint_port && (i >= info->inputs_per_port)) {
			port = khwq_qos_id_odd(idx);
			queue = i - info->inputs_per_port;
		} else {
			port = idx;
			queue = i;
		}

		val = wrr_credits[i];
		khwq_qos_set_sched_wrr_credit(info, port, queue, val, (queue == 0));
	}

	return size;
}

static ssize_t qnode_priority_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->priority == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->priority);
}

static ssize_t qnode_lowprio_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->low_priority == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->low_priority);
}

static ssize_t qnode_output_rate_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->output_rate == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->output_rate);
}

static int qnode_output_rate_store_child(struct ktree_node *child, void *arg)
{
	struct khwq_qos_tree_node *qnode = to_qnode(child);
	struct khwq_qos_info *info = qnode->info;
	struct khwq_qos_tree_node *parent = arg;
	int idx = qnode->sched_port_idx;
	int val;

	val = parent->output_rate;
	val = (val + info->ticks_per_sec - 1) / info->ticks_per_sec;
	if (val != 0)
		++val;
	khwq_qos_set_sched_out_throttle(info, idx, val, true);

	return 0;
}

static ssize_t qnode_output_rate_store(struct khwq_qos_tree_node *qnode,
				       const char *buf, size_t size)
{
	struct khwq_qos_info *info = qnode->info;
	int idx = qnode->sched_port_idx;
	int error;
	unsigned int new_rate;
	u64 val64;

	error = kstrtouint(buf, 0, &new_rate);
	if (error)
		return error;
	
	val64 = new_rate;
	val64 <<= (qnode->acct == QOS_BYTE_ACCT) ?
			QOS_CREDITS_BYTE_SHIFT :
			QOS_CREDITS_PACKET_SHIFT;
	do_div(val64, info->ticks_per_sec);
	if (val64 > (u64)S32_MAX)
		return -ERANGE;

	qnode->output_rate = new_rate;

	khwq_qos_set_sched_cir_credit(info, idx, (u32)val64, true);

	ktree_for_each_child(&qnode->node,
			     qnode_output_rate_store_child, qnode);

	return size;
}

static ssize_t qnode_burst_size_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->burst_size == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->burst_size);
}

static ssize_t qnode_burst_size_store(struct khwq_qos_tree_node *qnode,
				      const char *buf, size_t size)
{
	struct khwq_qos_info *info = qnode->info;
	int idx = qnode->sched_port_idx;
	int error, field;
	u64 cir_max;
	u32 cir_credit;

	error = kstrtouint(buf, 0, &field);
	if (error)
		return error;
	
	error = khwq_qos_get_sched_cir_credit(info, idx, &cir_credit);
	if (error)
		return error;

	cir_max = (qnode->acct == QOS_BYTE_ACCT) ?
		(field << QOS_CREDITS_BYTE_SHIFT) :
		(field << QOS_CREDITS_PACKET_SHIFT);
	if (cir_max > (S32_MAX - cir_credit))
		return -EINVAL;

	qnode->burst_size = field;

	khwq_qos_set_sched_cir_max(info, idx, (u32)cir_max, true);

	return size;
}

static ssize_t qnode_overhead_bytes_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->overhead_bytes);
}

static ssize_t qnode_overhead_bytes_store(struct khwq_qos_tree_node *qnode,
					  const char *buf, size_t size)
{
	struct khwq_qos_info *info = qnode->info;
	int idx = qnode->sched_port_idx;
	int error, val;

	error = kstrtoint(buf, 0, &val);
	if (error)
		return error;
	
	qnode->overhead_bytes = val;

	khwq_qos_set_sched_overhead_bytes(info, idx,
			(val < 0) ? 0 : val, false);
	khwq_qos_set_sched_remove_bytes(info, idx,
			(val < 0) ? (-val) : 0, true);

	return size;
}

static ssize_t qnode_output_queue_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	if (qnode->output_queue == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", qnode->output_queue);
}

static ssize_t qnode_input_queues_show(struct khwq_qos_tree_node *qnode,
					     char *buf)
{
	ssize_t l = 0;
	int i;

	for (i = 0; i < qnode->num_input_queues; i++)
		if (qnode->input_queue[i].valid == true)
			l += snprintf(buf + l, PAGE_SIZE - l, "%d ",
					qnode->input_queue[i].queue);

	l += snprintf(buf + l, PAGE_SIZE - l, "\n");

	return l;
}

static ssize_t qnode_input_queues_store(struct khwq_qos_tree_node *qnode,
					  const char *buf, size_t size)
{
	struct khwq_qos_info *info = qnode->info;
	struct khwq_device *kdev = info->kdev;
	int error, field, i;

	error = kstrtoint(buf, 0, &field);
	if (error)
		return error;

	if (field < 0) {
		for (i = 0; i < QOS_MAX_INPUTS; i++) {
			if (qnode->input_queue[i].queue == -field) {
				qnode->input_queue[i].valid = false;
				khwq_qos_free_drop_queue(info,
					qnode->input_queue[i].drop_queue_idx);
			}
		}
	} else {
		error = khwq_qos_alloc_drop_queue(info, field);
		if (error < 0) {
		dev_err(kdev->dev,
			"failed to alloc input queue %d on node %s\n",
			field, qnode->name);
			return error;
		}
		for (i = 0; i < QOS_MAX_INPUTS; i++) {
			if (qnode->input_queue[i].valid == false) {
				qnode->input_queue[i].drop_queue_idx = error;
				qnode->input_queue[i].valid = true;
				qnode->input_queue[i].queue = field;
			}
		}
		qnode->num_input_queues++;
	}

	khwq_qos_stop(info);

	khwq_qos_start(info);

	return size;
}

struct khwq_qos_qnode_attribute {
	struct attribute attr;
	ssize_t (*show)(struct khwq_qos_tree_node *qnode, char *);
	ssize_t	(*store)(struct khwq_qos_tree_node *qnode,
			 const char *, size_t);
};

#define KHWQ_QOS_QNODE_ATTR(_name, _mode, _show, _store) \
	struct khwq_qos_qnode_attribute attr_qnode_##_name = \
	__ATTR(_name, _mode, _show, _store)

static KHWQ_QOS_QNODE_ATTR(stats_class, S_IRUGO, qnode_stats_class_show,
			   NULL);
static KHWQ_QOS_QNODE_ATTR(drop_policy, S_IRUGO, qnode_drop_policy_show,
			   NULL);
static KHWQ_QOS_QNODE_ATTR(priority, S_IRUGO,
			   qnode_priority_show, NULL);
static KHWQ_QOS_QNODE_ATTR(low_priority, S_IRUGO,
			   qnode_lowprio_show, NULL);
static KHWQ_QOS_QNODE_ATTR(output_queue, S_IRUGO,
			   qnode_output_queue_show,
			   NULL);
static KHWQ_QOS_QNODE_ATTR(weight, S_IRUGO|S_IWUSR,
			   qnode_weight_show, qnode_weight_store);
static KHWQ_QOS_QNODE_ATTR(output_rate, S_IRUGO|S_IWUSR,
			   qnode_output_rate_show, qnode_output_rate_store);
static KHWQ_QOS_QNODE_ATTR(burst_size, S_IRUGO|S_IWUSR,
			   qnode_burst_size_show, qnode_burst_size_store);
static KHWQ_QOS_QNODE_ATTR(overhead_bytes, S_IRUGO|S_IWUSR,
			   qnode_overhead_bytes_show,
			   qnode_overhead_bytes_store);
static KHWQ_QOS_QNODE_ATTR(input_queues, S_IRUGO,
			   qnode_input_queues_show,
			   qnode_input_queues_store);

static struct attribute *khwq_qos_qnode_sysfs_default_attrs[] = {
	&attr_qnode_output_rate.attr,
	&attr_qnode_burst_size.attr,
	&attr_qnode_overhead_bytes.attr,
	&attr_qnode_output_queue.attr,
	&attr_qnode_input_queues.attr,
	NULL
};

static struct attribute *khwq_qos_qnode_sysfs_priority_attrs[] = {
	&attr_qnode_stats_class.attr,
	&attr_qnode_drop_policy.attr,
	&attr_qnode_priority.attr,
	&attr_qnode_output_rate.attr,
	&attr_qnode_burst_size.attr,
	&attr_qnode_overhead_bytes.attr,
	&attr_qnode_input_queues.attr,
	NULL
};

static struct attribute *khwq_qos_qnode_sysfs_wrr_attrs[] = {
	&attr_qnode_stats_class.attr,
	&attr_qnode_drop_policy.attr,
	&attr_qnode_weight.attr,
	&attr_qnode_output_rate.attr,
	&attr_qnode_burst_size.attr,
	&attr_qnode_overhead_bytes.attr,
	&attr_qnode_input_queues.attr,
	NULL
};

static struct attribute *khwq_qos_qnode_sysfs_lowprio_attrs[] = {
	&attr_qnode_stats_class.attr,
	&attr_qnode_drop_policy.attr,
	&attr_qnode_low_priority.attr,
	&attr_qnode_output_rate.attr,
	&attr_qnode_burst_size.attr,
	&attr_qnode_overhead_bytes.attr,
	&attr_qnode_input_queues.attr,
	NULL
};

static ssize_t khwq_qos_qnode_attr_show(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	struct khwq_qos_qnode_attribute *qnode_attr;
	struct khwq_qos_tree_node *qnode;

	qnode = container_of(kobj, struct khwq_qos_tree_node, kobj);
	qnode_attr = container_of(attr, struct khwq_qos_qnode_attribute,
				   attr);

	if (!qnode_attr->show)
		return -ENOENT;

	return qnode_attr->show(qnode, buf);
}

static ssize_t khwq_qos_qnode_attr_store(struct kobject *kobj,
					  struct attribute *attr,
					  const char *buf, size_t size)
{
	struct khwq_qos_qnode_attribute *qnode_attr;
	struct khwq_qos_tree_node *qnode;

	qnode = container_of(kobj, struct khwq_qos_tree_node, kobj);
	qnode_attr = container_of(attr, struct khwq_qos_qnode_attribute,
				   attr);

	if (!qnode_attr->store)
		return -ENOENT;

	return qnode_attr->store(qnode, buf, size);
}

static struct kobj_type khwq_qos_qnode_default_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show = khwq_qos_qnode_attr_show,
		.store = khwq_qos_qnode_attr_store},
	.default_attrs = khwq_qos_qnode_sysfs_default_attrs,
};

static struct kobj_type khwq_qos_qnode_priority_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show = khwq_qos_qnode_attr_show,
		.store = khwq_qos_qnode_attr_store},
	.default_attrs = khwq_qos_qnode_sysfs_priority_attrs,
};

static struct kobj_type khwq_qos_qnode_lowprio_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show = khwq_qos_qnode_attr_show,
		.store = khwq_qos_qnode_attr_store},
	.default_attrs = khwq_qos_qnode_sysfs_lowprio_attrs,
};

static struct kobj_type khwq_qos_qnode_wrr_ktype = {
	.sysfs_ops = &(struct sysfs_ops) {
		.show = khwq_qos_qnode_attr_show,
		.store = khwq_qos_qnode_attr_store},
	.default_attrs = khwq_qos_qnode_sysfs_wrr_attrs,
};

static int khwq_qos_cmp(struct ktree_node *_a, struct ktree_node *_b,
			void *arg)
{
	struct khwq_qos_tree_node *a = to_qnode(_a);
	struct khwq_qos_tree_node *b = to_qnode(_b);

	if ((a->priority != -1) && (b->priority != -1))
		return a->priority - b->priority;
	if (a->priority != -1)
		return -1;
	if (b->priority != -1)
		return 1;

	if ((a->weight != -1) && (b->weight != -1))
		return 0;
	if (a->weight != -1)
		return -1;
	if (b->weight != -1)
		return 1;

	if ((a->low_priority != -1) && (b->low_priority != -1))
		return a->low_priority - b->low_priority;
	if (a->low_priority != -1)
		return -1;
	if (b->low_priority != -1)
		return 1;

	return 0;
}

static int khwq_qos_check_overflow(struct khwq_qos_tree_node *qnode,
				   int val)
{
	int tmp;

	tmp = (qnode->acct == QOS_BYTE_ACCT) ?
			(val & ~BITS(32 - QOS_CREDITS_BYTE_SHIFT)) :
			(val & ~BITS(32 - QOS_CREDITS_PACKET_SHIFT));

	if (tmp)
		return -EINVAL;
	
	return 0;
}

static int khwq_qos_tree_parse(struct khwq_qos_info *info,
				int tree_index,
				struct device_node *node,
				struct khwq_qos_tree_node *parent)
{
	struct khwq_qos_tree_node *qnode;
	struct khwq_device *kdev = info->kdev;
	int length, i, error = 0, elements, num_children;
	struct device_node *child;
	bool has_children;
	const char *name;
	struct kobject *parent_kobj;
	u32 temp[QOS_MAX_INPUTS];

	/* first find out if we are a leaf node */
	child = of_get_next_child(node, NULL);
	has_children = !!child;
	of_node_put(child);

	qnode = devm_kzalloc(kdev->dev, sizeof(*qnode), GFP_KERNEL);
	if (!qnode) {
		dev_err(kdev->dev, "failed to alloc qos node\n");
		return -ENOMEM;
	}

	if (!parent)
		parent_kobj = info->kobj;
	else
		parent_kobj = &parent->kobj;

	qnode->info = info;
	qnode->parent = parent;
	qnode->name = node->name;
	qnode->tree_index = tree_index;

	of_property_read_string(node, "label", &qnode->name);
	dev_dbg(kdev->dev, "processing node %s, parent %s%s\n",
		qnode->name, parent ? parent->name : "(none)",
		has_children ? "" : ", leaf");

	qnode->type = QOS_NODE_DEFAULT;
	if (of_find_property(node, "strict-priority", NULL))
		qnode->type = QOS_NODE_PRIO;
	if (of_find_property(node, "weighted-round-robin", NULL)) {
		if (qnode->type != QOS_NODE_DEFAULT) {
			dev_err(kdev->dev, "multiple node types in %s\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
		qnode->type = QOS_NODE_WRR;
	}
	if (of_find_property(node, "blended-scheduler", NULL)) {
		if (qnode->type != QOS_NODE_DEFAULT) {
			dev_err(kdev->dev, "multiple node types in %s\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
		qnode->type = QOS_NODE_BLENDED;
	}
	if (!parent && qnode->type == QOS_NODE_DEFAULT) {
		dev_err(kdev->dev, "root node %s must be wrr/prio/blended\n",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}
	if (!has_children && qnode->type != QOS_NODE_DEFAULT) {
		dev_err(kdev->dev, "leaf node %s must not be wrr/prio/blended\n",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}

	dev_dbg(kdev->dev, "node %s: type %d\n", qnode->name, qnode->type);

	qnode->weight = -1;
	if (!of_property_read_u32(node, "weight", &qnode->weight)) {
		dev_dbg(kdev->dev, "node %s: weight %d\n",
			qnode->name, qnode->weight);
		if (qnode->weight == 0 || qnode->weight > QOS_MAX_WEIGHT) {
			dev_err(kdev->dev,
				"node %s: weight must be between 1 and %u\n",
				qnode->name, QOS_MAX_WEIGHT);
			error = -EINVAL;
			goto error_free;
		}
		if (!parent ||
		    ((parent->type != QOS_NODE_WRR) &&
		     (parent->type != QOS_NODE_BLENDED))) {
			dev_err(kdev->dev, "node %s: unexpected weight\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
		++parent->wrr_children;
	} else if (parent && parent->type == QOS_NODE_WRR) {
		dev_err(kdev->dev, "node %s: missing weight on wrr child\n",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}

	qnode->priority = -1;
	if (!of_property_read_u32(node, "priority", &qnode->priority)) {
		dev_dbg(kdev->dev, "node %s: priority %d\n",
			qnode->name, qnode->priority);
		if (qnode->priority == -1) {
			dev_err(kdev->dev,
				"node %s: priority must be between 0 and %u\n",
				qnode->name, U32_MAX - 1);
			error = -EINVAL;
			goto error_free;
		}
		if (!parent ||
		    ((parent->type != QOS_NODE_PRIO) &&
		     (parent->type != QOS_NODE_BLENDED))) {
			dev_err(kdev->dev, "node %s: unexpected priority\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
		++parent->prio_children;
		error = ktree_for_each_child(&parent->node,
					     khwq_qos_prio_check,
					     qnode);
		if (error) {
			dev_err(kdev->dev, "node %s: duplicate priority %d\n",
				qnode->name, qnode->priority);
			error = -EINVAL;
			goto error_free;
		}
	} else if (parent && parent->type == QOS_NODE_PRIO) {
		dev_err(kdev->dev,
			"node %s: missing priority on strict-priority child\n",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}

	qnode->low_priority = -1;
	if (!of_property_read_u32(node, "low-priority", &qnode->low_priority)) {
		dev_dbg(kdev->dev, "node %s: low-priority %d\n",
			qnode->name, qnode->low_priority);
		if (qnode->low_priority == -1) {
			dev_err(kdev->dev,
				"node %s: low-priority must be between 0 and %u\n",
				qnode->name, U32_MAX - 1);
			error = -EINVAL;
			goto error_free;
		}
		if (!parent ||
		    (parent->type != QOS_NODE_BLENDED)) {
			dev_err(kdev->dev,
				"node %s: unexpected low-priority\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
		++parent->lowprio_children;
		error = ktree_for_each_child(&parent->node,
					     khwq_qos_lowprio_check,
					     qnode);
		if (error) {
			dev_err(kdev->dev,
				"node %s: duplicate low-priority %d\n",
				qnode->name, qnode->low_priority);
			error = -EINVAL;
			goto error_free;
		}
	}

	if (parent && (parent->type == QOS_NODE_BLENDED)) {
		if ((qnode->weight == -1) &&
		    (qnode->priority == -1) &&
		    (qnode->low_priority == -1)) {
			dev_err(kdev->dev,
				"node %s: missing weight or priority on "
				"blended-scheduler child\n",
				qnode->name);
			error = -EINVAL;
			goto error_free;
		}
	}

	qnode->acct = QOS_BYTE_ACCT;
	if (of_find_property(node, "byte-units", NULL))
		qnode->acct = QOS_BYTE_ACCT;
	else if (of_find_property(node, "packet-units", NULL))
		qnode->acct = QOS_PACKET_ACCT;
	else if (parent)
		qnode->acct = parent->acct;
	dev_dbg(kdev->dev, "node %s: accounting %s\n", qnode->name,
		qnode->acct == QOS_PACKET_ACCT ? "packet" : "bytes");

	qnode->output_queue = -1;
	error = of_property_read_u32(node, "output-queue", &qnode->output_queue);
	if (error && !parent) {
		dev_err(kdev->dev, "root qos node %s needs an output queue\n",
			qnode->name);
		goto error_free;
	}
	if (!error && parent) {
		dev_warn(kdev->dev, "output queue ignored on node %s\n",
			 qnode->name);
		qnode->output_queue = -1;
	}
	dev_dbg(kdev->dev, "node %s: output queue %d\n", qnode->name,
		qnode->output_queue);

	qnode->overhead_bytes = parent ? parent->overhead_bytes : 24;
	error = of_property_read_u32(node, "overhead-bytes",
				     &qnode->overhead_bytes);
	if (!error)
		dev_dbg(kdev->dev, "node %s: overhead bytes %d\n", qnode->name,
			qnode->overhead_bytes);

	error = of_property_read_string(node, "drop-policy", &name);
	if (!error) {
		qnode->drop_policy = khwq_qos_find_drop_policy(info, name);
		if (!qnode->drop_policy) {
			dev_err(kdev->dev, "invalid drop policy %s\n", name);
			error = -EINVAL;
			goto error_free;
		}
		qnode->drop_policy->usecount ++;
	}
	if (!has_children && !khwq_qos_inherited_drop_policy(qnode))
		qnode->drop_policy = info->default_drop_policy;

	dev_dbg(kdev->dev, "node %s: drop policy %s\n", qnode->name,
		qnode->drop_policy ? qnode->drop_policy->name : "(none)");

	error = of_property_read_string(node, "stats-class", &name);
	if (!error) {
		qnode->stats_class = khwq_qos_find_stats_class(info, name);
		if (!qnode->stats_class)
			qnode->stats_class = khwq_qos_init_stats_class(info, name);
		if (!qnode->stats_class) {
			dev_err(kdev->dev, "failed to create stats class %s\n", name);
			error = -ENODEV;
			goto error_free;
		}
		qnode->stats_class->usecount ++;
	}
	if (has_children && qnode->stats_class) {
		dev_err(kdev->dev, "unexpected stats class on non-leaf %s",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}
	dev_dbg(kdev->dev, "node %s: stats class %s\n", qnode->name,
		qnode->stats_class ? qnode->stats_class->name : "(none)");

	qnode->output_rate = parent ? parent->output_rate : -1;
	qnode->burst_size = parent ? parent->burst_size : -1;
	if (of_find_property(node, "output-rate", &length)) {
		elements = length / sizeof(u32);
		elements = max(elements, 2);
		error = of_property_read_u32_array(node, "output-rate", temp,
						   elements);
		if (error) {
			dev_err(kdev->dev, "error reading output-rate on %s\n",
				qnode->name);
			goto error_free;
		}

		error = khwq_qos_check_overflow(qnode, (temp[0] /
							info->ticks_per_sec));
		if (error) {
			dev_err(kdev->dev, "burst rate credit overflow\n");
			goto error_free;
		}

		qnode->output_rate = temp[0];

		if (elements > 1) {
			error = khwq_qos_check_overflow(qnode, temp[1]);
			if (error) {
				dev_err(kdev->dev, "burst size credit overflow\n");
				goto error_free;
			}

			qnode->burst_size = temp[1];
		}
	}
	dev_dbg(kdev->dev, "node %s: output rate %d, burst %d\n", qnode->name,
		qnode->output_rate, qnode->burst_size);

	if (of_find_property(node, "input-queues", &length)) {
		qnode->num_input_queues = length / sizeof(u32);
		if (qnode->num_input_queues >= QOS_MAX_INPUTS) {
			dev_err(kdev->dev, "too many input_queues to node %s\n",
				qnode->name);
			error = -EOVERFLOW;
			goto error_free;
		}
		error = of_property_read_u32_array(node, "input-queues",
						   temp,
						   qnode->num_input_queues);
		if (error) {
			dev_err(kdev->dev, "error getting input_queues on node %s\n",
				qnode->name);
			goto error_free;
		}
	}
	if (has_children && qnode->num_input_queues) {
		dev_err(kdev->dev, "unexpected input-queues on non-leaf %s",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}
	if (!has_children && !qnode->num_input_queues) {
		dev_err(kdev->dev, "expected input-queues on leaf %s",
			qnode->name);
		error = -EINVAL;
		goto error_free;
	}
	for (i = 0; i < qnode->num_input_queues; i++) {
		qnode->input_queue[i].queue = temp[i];
		qnode->input_queue[i].valid = false;
	}

	if (!parent)
		error = kobject_init_and_add(&qnode->kobj,
					     &khwq_qos_qnode_default_ktype,
					     parent_kobj, qnode->name);
	else {
		if ((parent->type == QOS_NODE_PRIO) ||
		    ((parent->type == QOS_NODE_BLENDED) &&
		     (qnode->priority != -1)))
			error = kobject_init_and_add(&qnode->kobj,
					     &khwq_qos_qnode_priority_ktype,
					     parent_kobj, qnode->name);
		else if ((parent->type == QOS_NODE_WRR) ||
			 ((parent->type == QOS_NODE_BLENDED) &&
			  (qnode->weight != -1)))
			error = kobject_init_and_add(&qnode->kobj,
					     &khwq_qos_qnode_wrr_ktype,
					     parent_kobj, qnode->name);
		else if ((parent->type == QOS_NODE_BLENDED) &&
			 (qnode->low_priority != -1))
			error = kobject_init_and_add(&qnode->kobj,
					     &khwq_qos_qnode_lowprio_ktype,
					     parent_kobj, qnode->name);
		else
			error = kobject_init_and_add(&qnode->kobj,
					     &khwq_qos_qnode_default_ktype,
					     parent_kobj, qnode->name);
	}
	if (error) {
		dev_err(kdev->dev, "failed to create sysfs "
			"entries for qnode %s\n", qnode->name);
		goto error_destroy;
	}

	if (!parent)
		ktree_set_root(&info->qos_trees[tree_index], &qnode->node);
	else
		ktree_add_child_last(&parent->node, &qnode->node);

	num_children = 0;
	for_each_child_of_node(node, child) {
		++num_children;
		error = khwq_qos_tree_parse(info, tree_index, child, qnode);
		if (error)
			goto error_destroy;
	}

	if (qnode->drop_policy) {
		error = ktree_for_each_child(&qnode->node,
					     khwq_qos_drop_policy_check,
					     info);
		if (error)
			goto error_destroy;
	}

	if (num_children <= info->inputs_per_port)
		qnode->is_joint_port = false;
	else {
		if (num_children <= (info->inputs_per_port * 2)) {
			qnode->is_joint_port = true;
			dev_dbg(kdev->dev, "node %s needs a joint port\n",
				 qnode->name);
		} else {
			dev_err(kdev->dev, "node %s has too many children\n",
				qnode->name);
			error = -EINVAL;
			goto error_destroy;
		}
	}
	if (qnode->is_joint_port && qnode->type == QOS_NODE_DEFAULT) {
		dev_err(kdev->dev, "joint port node %s must be wrr/prio\n",
			qnode->name);
		error = -EINVAL;
		goto error_destroy;
	}

	if (num_children > 1)
		ktree_sort_children(&qnode->node, khwq_qos_cmp, NULL);

	return 0;

error_destroy:
	ktree_remove_node(&qnode->node);
error_free:
	if (qnode && qnode->drop_policy)
		qnode->drop_policy->usecount--;
	if (qnode && qnode->stats_class)
		qnode->stats_class->usecount--;
	devm_kfree(kdev->dev, qnode);
	return error;
}

static int khwq_qos_tree_map_nodes(struct ktree_node *node, void *arg)
{
	struct khwq_qos_tree_node *qnode = to_qnode(node);
	struct khwq_qos_tree_node *parent = qnode->parent;
	struct khwq_qos_info *info = arg;
	struct khwq_device *kdev = info->kdev;
	int max_inputs;

	qnode->child_port_count	=  0;
	qnode->child_count	=  0;
	qnode->parent_input	=  0;
	qnode->is_drop_input	= false;

	if (qnode->drop_policy)
		qnode->is_drop_input = true;

	if (parent) {
		/* where do we plugin into our parent? */
		qnode->parent_input = parent->child_count;

		parent->child_weight[parent->child_count] = qnode->weight;
		/* provide our parent with info */
		parent->child_count++;

		/* inherit if parent is an input to drop sched */
		if (parent->is_drop_input)
			qnode->is_drop_input = true;
	}

	ktree_for_each_child(&qnode->node, khwq_qos_tree_map_nodes, info);

	qnode->has_sched_port = (qnode->type == QOS_NODE_PRIO	 ||
				 qnode->type == QOS_NODE_WRR	 ||
				 qnode->type == QOS_NODE_BLENDED ||
				 qnode->child_port_count);

	if (qnode->has_sched_port && parent)
		parent->child_port_count ++;

	max_inputs = (qnode->is_joint_port) ?
			info->inputs_per_port * 2 :
			info->inputs_per_port;

	if (qnode->child_count > max_inputs) {
		dev_err(kdev->dev, "too many input_queues (%d) to node %s\n",
			qnode->child_count, qnode->name);
		return -EOVERFLOW;
	}
	return 0;
}

static int khwq_qos_tree_alloc_nodes(struct ktree_node *node, void *arg)
{
	struct khwq_qos_info *info = arg;
	struct khwq_device *kdev = info->kdev;
	struct khwq_qos_tree_node *qnode = to_qnode(node);
	struct khwq_qos_tree_node *parent = qnode->parent;
	int error, i;

	if (qnode->has_sched_port) {
		error = khwq_qos_alloc_sched_port(info, 
				parent ? parent->sched_port_idx : -1,
				qnode->is_joint_port);
		if (error < 0) {
			error = khwq_qos_alloc_sched_port(info, -1,
					qnode->is_joint_port);
			if (error >= 0)
				dev_warn(kdev->dev, "node %s: non-optimal port"
					 " allocation\n", qnode->name);
		}
		if (error < 0) {
			dev_err(kdev->dev, "node %s: failed to alloc sched port [%d]\n",
				qnode->name, error);
			return error;
		}
		qnode->sched_port_idx = error;
	} else
		qnode->sched_port_idx = parent->sched_port_idx;


	if (parent) {
		if (WARN_ON(qnode->output_queue != -1))
			return -EINVAL;
		if (parent->type == QOS_NODE_DEFAULT)
			qnode->parent_input = parent->parent_input;
		if (parent->is_joint_port && (qnode->parent_input >= info->inputs_per_port))
			error = khwq_qos_control_sched_port(info,
					QOS_CONTROL_GET_INPUT,
					khwq_qos_id_odd(parent->sched_port_idx),
					qnode->parent_input - info->inputs_per_port);
		else
			error = khwq_qos_control_sched_port(info,
					QOS_CONTROL_GET_INPUT,
					parent->sched_port_idx,
					qnode->parent_input);
		if (WARN_ON(error < 0))
			return error;
		qnode->output_queue = error;
	}

	dev_dbg(kdev->dev, "node %s: mapped to output queue %d (port %d)\n",
		qnode->name, qnode->output_queue,
		khwq_qos_id_to_idx(qnode->sched_port_idx));

	if (qnode->drop_policy) {
		error = khwq_qos_alloc_drop_out(info);
		if (error < 0) {
			dev_err(kdev->dev, "node %s: failed to alloc sched port [%d]\n",
				qnode->name, error);
			return error;
		}
		qnode->drop_out_idx = error;
		dev_dbg(kdev->dev, "allocated drop out %d for node %s\n",
			khwq_qos_id_to_idx(qnode->drop_out_idx), qnode->name);
	}

	if (qnode->is_drop_input) {

		if (!qnode->drop_out_idx)
			qnode->drop_out_idx = parent->drop_out_idx;

		for (i = 0; i < qnode->num_input_queues; i++) {
			error = khwq_qos_alloc_drop_queue(info,
							  qnode->input_queue[i].queue);
			if (error < 0) {
				dev_err(kdev->dev,
					"failed to alloc input queue %d on node %s\n",
					qnode->input_queue[i].queue, qnode->name);
				return error;
			}
			qnode->input_queue[i].drop_queue_idx = error;
			qnode->input_queue[i].valid = true;
			dev_dbg(kdev->dev, "allocated drop queue %d for node %s\n",
				qnode->input_queue[i].queue, qnode->name);
		}
	}

	error = ktree_for_each_child(&qnode->node, khwq_qos_tree_alloc_nodes,
				     info);

	return error;
}

static int khwq_qos_tree_start_port(struct khwq_qos_info *info,
				      struct khwq_qos_tree_node *qnode)
{
	struct khwq_qos_tree_node *parent = qnode->parent;
	int error, val, idx = qnode->sched_port_idx;
	struct khwq_device *kdev = info->kdev;
	bool sync = false;
	int inputs, i, cir_credit;
	u32 wrr_credits[QOS_MAX_CHILDREN];
	u64 tmp;
	u64 cir_max;

	if (!qnode->has_sched_port)
		return 0;

	dev_dbg(kdev->dev, "programming sched port index %d for node %s\n",
		khwq_qos_id_to_idx(idx), qnode->name);

	val = (qnode->acct == QOS_BYTE_ACCT) ?
			(QOS_SCHED_FLAG_WRR_BYTES |
			 QOS_SCHED_FLAG_CIR_BYTES |
			 QOS_SCHED_FLAG_CONG_BYTES) : 0;
	if (parent && (parent->acct == QOS_BYTE_ACCT))
		val |= QOS_SCHED_FLAG_THROTL_BYTES;
	if (qnode->is_joint_port)
		val |= QOS_SCHED_FLAG_IS_JOINT;
	error = khwq_qos_set_sched_unit_flags(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	error = khwq_qos_set_sched_group_count(info, idx, 1, sync);
	if (WARN_ON(error))
		return error;

	val = qnode->output_queue;
	error = khwq_qos_set_sched_out_queue(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	val = qnode->overhead_bytes;
	error = khwq_qos_set_sched_overhead_bytes(info, idx,
			(val < 0) ? 0 : val, sync);
	if (WARN_ON(error))
		return error;
	error = khwq_qos_set_sched_remove_bytes(info, idx,
			(val < 0) ? (-val) : 0, sync);
	if (WARN_ON(error))
		return error;

	val = parent ? parent->output_rate : 0;
	val = (val + info->ticks_per_sec - 1) / info->ticks_per_sec;
	if (val != 0)
		++val;
	error = khwq_qos_set_sched_out_throttle(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	tmp = qnode->output_rate;
	tmp <<= (qnode->acct == QOS_BYTE_ACCT) ?
			QOS_CREDITS_BYTE_SHIFT :
			QOS_CREDITS_PACKET_SHIFT;
	do_div(tmp, info->ticks_per_sec);
	if (tmp > (u64)S32_MAX) {
		dev_warn(kdev->dev, "node %s output-rate is too large.\n",
				qnode->name);
		tmp = S32_MAX;
	}
	cir_credit = (u32)tmp;

	error = khwq_qos_set_sched_cir_credit(info, idx, cir_credit, sync);
	if (WARN_ON(error))
		return error;

	cir_max = qnode->burst_size;
	cir_max <<= (qnode->acct == QOS_BYTE_ACCT) ?
			QOS_CREDITS_BYTE_SHIFT :
			QOS_CREDITS_PACKET_SHIFT;
	if (cir_max > (S32_MAX - cir_credit)) {
		dev_warn(kdev->dev, "node %s burst-size is too large.\n",
				qnode->name);
		cir_max = S32_MAX - cir_credit;
	}
	error = khwq_qos_set_sched_cir_max(info, idx, (u32)cir_max, sync);
	if (WARN_ON(error))
		return error;

	inputs = (qnode->type == QOS_NODE_DEFAULT) ? 1 : qnode->child_count;

	error = khwq_qos_set_sched_total_q_count(info, idx, inputs, sync);
	if (WARN_ON(error))
		return error;

	error = khwq_qos_set_sched_sp_q_count(info, idx, qnode->prio_children,
						sync);
	if (WARN_ON(error))
		return error;

	error = khwq_qos_set_sched_wrr_q_count(info, idx, qnode->wrr_children,
						sync);
	if (WARN_ON(error))
		return error;

	khwq_qos_wrr_norm(qnode, wrr_credits);

	for (i = 0; i < inputs; i++) {
		int port, queue;

		if (qnode->is_joint_port && (i >= info->inputs_per_port)) {
			port = khwq_qos_id_odd(idx);
			queue = i - info->inputs_per_port;
		} else {
			port = idx;
			queue = i;
		}

		val = 0;
		error = khwq_qos_set_sched_cong_thresh(info, port, queue, val, sync);
		if (WARN_ON(error))
			return error;

		val = wrr_credits[i];
		error = khwq_qos_set_sched_wrr_credit(info, port, queue, val, sync);
		if (WARN_ON(error))
			return error;
	}

	error = khwq_qos_sync_sched_port(info, idx);
	if (error) {
		dev_err(kdev->dev, "error writing sched config for %s\n",
			qnode->name);
		return error;
	}

	error = khwq_qos_control_sched_port(info, QOS_CONTROL_ENABLE, idx,
					    true);
	if (error) {
		dev_err(kdev->dev, "error enabling sched port for %s\n",
			qnode->name);
		return error;
	}

	/* If this is a Lite-Joint port pair, configure the Odd port here */
	if (qnode->is_joint_port) {
		int odd_idx = khwq_qos_id_odd(idx);

		error = khwq_qos_set_sched_unit_flags(info, odd_idx,
					QOS_SCHED_FLAG_IS_JOINT, sync);
		if (WARN_ON(error))
			return error;

		val = (inputs <= info->inputs_per_port) ? 0 :
				inputs - info->inputs_per_port;
		error = khwq_qos_set_sched_total_q_count(info, odd_idx,
							 val, sync);
		if (WARN_ON(error))
			return error;

		val = (qnode->prio_children <= info->inputs_per_port) ? 0 :
				qnode->prio_children - info->inputs_per_port;
		error = khwq_qos_set_sched_sp_q_count(info, odd_idx, val, sync);
		if (WARN_ON(error))
			return error;

		val = ((qnode->prio_children + qnode->wrr_children)
					<= info->inputs_per_port) ? 0 :
			qnode->wrr_children -
				(info->inputs_per_port - qnode->prio_children);
		error = khwq_qos_set_sched_wrr_q_count(info, odd_idx, val, sync);
		if (WARN_ON(error))
			return error;

		error = khwq_qos_sync_sched_port(info, odd_idx);
		if (error) {
			dev_err(kdev->dev, "error writing sched config for %s\n",
				qnode->name);
			return error;
		}

		error = khwq_qos_control_sched_port(info, QOS_CONTROL_ENABLE,
						    odd_idx, false);
		if (error) {
			dev_err(kdev->dev, "error disabling sched port for %s\n",
				qnode->name);
			return error;
		}
	}

	return 0;
}

static int khwq_qos_tree_start_drop_out(struct khwq_qos_info *info,
				      struct khwq_qos_tree_node *qnode)
{
	struct khwq_qos_drop_policy *policy = qnode->drop_policy;
	int error, val, idx = qnode->drop_out_idx;
	struct khwq_device *kdev = info->kdev;
	bool sync = false;

	if (!policy)
		return 0;

	dev_dbg(kdev->dev, "programming drop out index %d for node %s\n",
		khwq_qos_id_to_idx(idx), qnode->name);

	val = qnode->output_queue;
	error = khwq_qos_set_drop_out_queue_number(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	val = (policy->max_drop_prob << 16) / 100;
	error = khwq_qos_set_drop_out_red_prob(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	val = khwq_qos_id_to_idx(policy->drop_cfg_idx);
	error = khwq_qos_set_drop_out_cfg_prof_idx(info, idx, val, sync);
	if (WARN_ON(error))
		return error;

	error = khwq_qos_set_drop_out_enable(info, idx, 1, sync);
	if (WARN_ON(error))
		return error;

	error = khwq_qos_set_drop_out_avg_depth(info, idx, 0, sync);
	if (WARN_ON(error))
		return error;

	return 0;
}

static int khwq_qos_tree_start_drop_queue(struct khwq_qos_info *info,
					    struct khwq_qos_tree_node *qnode)
{
	struct khwq_qos_stats_class *class = qnode->stats_class;
	struct khwq_device *kdev = info->kdev;
	int i, idx, error;
	bool sync = false;
	u32 val;

	if (!qnode->is_drop_input)
		return 0;

	for (i = 0; i < qnode->num_input_queues; i++) {
		if (qnode->input_queue[i].valid == false)
			continue;

		idx = qnode->input_queue[i].drop_queue_idx;

		dev_dbg(kdev->dev, "programming drop queue %d for node %s\n",
			khwq_qos_id_to_idx(idx), qnode->name);

		val = khwq_qos_id_to_idx(qnode->drop_out_idx);
		error = khwq_qos_set_drop_q_out_prof_idx(info, idx, val, sync);
		if (WARN_ON(error))
			return error;

		error = khwq_qos_set_drop_q_stat_blk_idx(info, idx,
							 class->stats_block_idx,
							 sync);
		if (WARN_ON(error))
			return error;

		error = khwq_qos_set_drop_q_stat_irq_pair_idx(info, idx,
							      1, sync);
		if (WARN_ON(error))
			return error;


		error = khwq_qos_set_drop_q_valid(info, idx, 1, sync);
		if (WARN_ON(error))
			return error;
	}
	return 0;
}

static int khwq_qos_tree_start_nodes(struct ktree_node *node, void *arg)
{
	struct khwq_qos_info *info = arg;
	struct khwq_qos_tree_node *qnode = to_qnode(node);
	struct khwq_device *kdev = info->kdev;
	int error;

	error = khwq_qos_tree_start_port(info, qnode);
	if (error)
		return error;

	error = khwq_qos_tree_start_drop_out(info, qnode);
	if (error)
		return error;

	error = khwq_qos_tree_start_drop_queue(info, qnode);
	if (error)
		return error;

	error = ktree_for_each_child(&qnode->node,
				     khwq_qos_tree_start_nodes, info);
	if (error)
		dev_err(kdev->dev, "error programming subtree at %s\n",
			qnode->name);
	return error;

}

static int khwq_qos_tree_init(struct khwq_qos_info *info)
{
	struct ktree_node *root;
	int error;
	int tree_index;

	for (tree_index = 0; tree_index < info->qos_tree_count; ++tree_index) {
		root = ktree_get_root(&info->qos_trees[tree_index]);
		if (WARN_ON(!root))
			return -ENODEV;

		error = khwq_qos_tree_map_nodes(root, info);
		if (WARN(error, "error %d\n", error)) {
			ktree_put_node(root);
			return error;
		}

		error = khwq_qos_tree_alloc_nodes(root, info);
		if (error) {
			ktree_put_node(root);
			return error;
		}

		ktree_put_node(root);
	}

	return 0;
}

int khwq_qos_tree_start(struct khwq_qos_info *info)
{
	struct ktree_node *root;
	int error;
	int tree_index;

	for (tree_index = 0; tree_index < info->qos_tree_count; ++tree_index) {
		root = ktree_get_root(&info->qos_trees[tree_index]);
		if (WARN_ON(!root))
			return -ENODEV;

		error = khwq_qos_tree_start_nodes(root, info);
		if (WARN(error, "error %d\n", error)) {
			ktree_put_node(root);
			return error;
		}

		ktree_put_node(root);
	}

	error = khwq_qos_sync_drop_queue(info, -1);
	if (error) {
		dev_err(info->kdev->dev, "error syncing drop queues\n");
		return error;
	}

	error = khwq_qos_sync_drop_out(info, -1);
	if (error) {
		dev_err(info->kdev->dev, "error syncing drop outs\n");
		return error;
	}

	return 0;
}

static int khwq_qos_stop_drop_queues(struct khwq_qos_info *info)
{
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_DROP_QUEUE_CFG];
	struct khwq_device *kdev = info->kdev;
	struct khwq_pdsp_info *pdsp;
	int i, error, idx;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			error = khwq_qos_set_drop_q_valid(info, idx, 0, false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_q_stat_blk_idx(info, idx, 0,
								 false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_q_stat_irq_pair_idx(info, idx,
								      0, false);
			if (WARN_ON(error))
			return error;

			error = khwq_qos_set_drop_q_out_prof_idx(info, idx, 0,
								 false);
			if (WARN_ON(error))
				return error;
		}
	}

	error = khwq_qos_sync_drop_queue(info, -1);
	if (error) {
		dev_err(kdev->dev, "error syncing drop queues\n");
		return error;
	}

	return 0;
}

static int khwq_qos_stop_drop_outs(struct khwq_qos_info *info)
{
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_DROP_OUT_PROF];
	struct khwq_device *kdev = info->kdev;
	struct khwq_pdsp_info *pdsp;
	int i, error, idx;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			error = khwq_qos_set_drop_out_enable(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_out_queue_number(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_out_red_prob(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_out_cfg_prof_idx(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_drop_out_avg_depth(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;
		}
	}

	error = khwq_qos_sync_drop_out(info, -1);
	if (error) {
		dev_err(kdev->dev, "error syncing drop out\n");
		return error;
	}

	return 0;
}

static int khwq_qos_stop_sched_port_queues(struct khwq_qos_info *info)
{
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_SCHED_PORT_CFG];
	struct khwq_device *kdev = info->kdev;
	struct khwq_pdsp_info *pdsp;
	int i, j, error, idx;
	u32 queues;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			error = khwq_qos_set_sched_unit_flags(info, idx, 0xf,
							      false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_get_sched_total_q_count(info, idx,
								&queues);
			if (WARN_ON(error))
				return error;
			if (queues > info->inputs_per_port)
				queues = info->inputs_per_port;

			for (j = 0; j < queues; j++) {
				error = khwq_qos_set_sched_cong_thresh(info,
								       idx, j,
								       1, false);
				if (WARN_ON(error))
					return error;

				error = khwq_qos_set_sched_wrr_credit(info, idx,
								      j, 0,
								      false);
				if (WARN_ON(error))
					return error;
			}

			error = khwq_qos_set_sched_out_queue(info, idx, 0,
							     false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_overhead_bytes(info, idx, 0,
								  false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_remove_bytes(info, idx, 0,
								  false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_out_throttle(info, idx, 0,
								false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_cir_credit(info, idx, 0,
							      false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_cir_max(info, idx, 0, false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_out_throttle(info, idx, 0,
								false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_total_q_count(info, idx, 0,
								false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_sp_q_count(info, idx, 0,
								false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_set_sched_wrr_q_count(info, idx, 0,
								false);
			if (WARN_ON(error))
				return error;

			error = khwq_qos_sync_sched_port(info, idx);
			if (error)
				return error;

			error = khwq_qos_control_sched_port(info,
							    QOS_CONTROL_ENABLE,
							    idx,
							    false);
			if (error) {
				dev_err(kdev->dev, "error disabling sched port "
					"%d\n", i);
				return error;
			}
		}
	}

	return 0;
}

int khwq_qos_tree_stop(struct khwq_qos_info *info)
{
	int error;

	error = khwq_qos_stop_sched_port_queues(info);
	if (error)
		return error;

	error = khwq_qos_stop_drop_queues(info);
	if (error)
		return error;

	error = khwq_qos_stop_drop_outs(info);
	if (error)
		return error;

	return 0;
}

static int khwq_qos_init_base(struct khwq_qos_info *info, int base, int num)
{
	struct khwq_device *kdev = info->kdev;
	int end = base + num;
	int sched_port_queues, drop_sched_queues;

	base = ALIGN(base, 32);
	info->drop_sched_queue_base = base;
	drop_sched_queues = info->shadows[QOS_DROP_QUEUE_CFG].count;
	base += drop_sched_queues;

	base = ALIGN(base, 32);
	info->sched_port_queue_base = base;
	sched_port_queues = (info->shadows[QOS_SCHED_PORT_CFG].count *
			     info->inputs_per_port);
	base += sched_port_queues;

	if (base >= end) {
		dev_err(kdev->dev, "too few queues (%d), need %d + %d\n",
			num, sched_port_queues, drop_sched_queues);
		return -ENODEV;
	} else {
		dev_info(kdev->dev, "qos: sched port @%d, drop sched @%d\n",
			 info->sched_port_queue_base,
			 info->drop_sched_queue_base);
		return 0;
	}
}

static int khwq_qos_init_queue(struct khwq_range_info *range,
			       struct hwqueue_instance *inst)
{
	return 0;
}

int khwq_qos_start(struct khwq_qos_info *info)
{
	struct khwq_device *kdev;
	u32 command;
	int error = 0;

	kdev = info->kdev;

	error = khwq_qos_tree_start(info);
	if (error) {
		dev_err(kdev->dev, "failed to program qos tree\n");
		return error;
	}

	/* Enable the drop scheduler */
	command = (QOS_CMD_ENABLE_PORT |
		   QOS_DROP_SCHED_ENABLE | QOS_ENABLE);
	error = khwq_qos_write_cmd(info, command);
	if (error)
		dev_err(kdev->dev, "failed to enable drop scheduler\n");

	init_timer(&info->timer);
	info->timer.data		= (unsigned long)info;
	info->timer.function		= khwq_qos_timer;
	info->timer.expires		= jiffies +
						KHWQ_QOS_TIMER_INTERVAL;
	add_timer(&info->timer);

	return error;
}

static ssize_t khwq_qos_out_prof_read(struct file *filp, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct khwq_qos_info *info = filp->private_data;
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_DROP_OUT_PROF];
	struct khwq_pdsp_info *pdsp;
	int i, buf_len = 8192, idx, error;
	unsigned long flags;
	size_t len = 0;
	ssize_t ret;
	char *buf;
	u32 temp;

	if (*ppos != 0)
		return 0;
	if (count < sizeof(buf))
		return -ENOSPC;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);

			spin_lock_irqsave(&info->lock, flags);

			len += snprintf(buf + len, buf_len - len,
					"output profile %d ", i);

			error = khwq_qos_get_drop_out_queue_number(info, idx,
								   &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"output q # %d ", temp);

			error = khwq_qos_get_drop_out_red_prob(info, idx,
							       &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"red prob %d ", temp);

			error = khwq_qos_get_drop_out_enable(info, idx, &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"enable %d ", temp);

			error = khwq_qos_get_drop_out_cfg_prof_idx(info, idx,
								   &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"config profile %d ", temp);

			error = khwq_qos_get_drop_out_avg_depth(info, idx,
								   &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"average q depth %d\n", temp);

			spin_unlock_irqrestore(&info->lock, flags);
		}
	}

free:
	ret = simple_read_from_buffer(buffer, len, ppos, buf, buf_len);
	kfree(buf);

	return ret;
}

static ssize_t khwq_qos_q_cfg_read(struct file *filp, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct khwq_qos_info *info = filp->private_data;
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_DROP_QUEUE_CFG];
	struct khwq_pdsp_info *pdsp;
	int i, buf_len = 4096, idx, error;
	unsigned long flags;
	size_t len = 0;
	ssize_t ret;
	char *buf;
	u32 temp;

	if (*ppos != 0)
		return 0;
	if (count < sizeof(buf))
		return -ENOSPC;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);

			spin_lock_irqsave(&info->lock, flags);

			len += snprintf(buf + len, buf_len - len,
					"q cfg %d ", i);

			error = khwq_qos_get_drop_q_stat_irq_pair_idx(info, idx,
								      &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"stats q pair # %d ", temp);

			error = khwq_qos_get_drop_q_stat_blk_idx(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"block %d ", temp);

			error = khwq_qos_get_drop_q_out_prof_idx(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"out prof %d\n", temp);

			spin_unlock_irqrestore(&info->lock, flags);
		}
	}

free:
	ret = simple_read_from_buffer(buffer, len, ppos, buf, buf_len);
	kfree(buf);

	return ret;
}

static ssize_t khwq_qos_drop_prof_read(struct file *filp, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct khwq_qos_info *info = filp->private_data;
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_DROP_CFG_PROF];
	struct khwq_pdsp_info *pdsp;
	int i, buf_len = 4096, idx, error;
	unsigned long flags;
	size_t len = 0;
	ssize_t ret;
	char *buf;
	u32 temp;

	if (*ppos != 0)
		return 0;
	if (count < sizeof(buf))
		return -ENOSPC;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);

			spin_lock_irqsave(&info->lock, flags);

			len += snprintf(buf + len, buf_len - len,
					"drop cfg prof %d ", i);

			error = khwq_qos_get_drop_cfg_unit_flags(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"unit flags %d ", temp);

			error = khwq_qos_get_drop_cfg_mode(info, idx, &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"mode %d ", temp);

			error = khwq_qos_get_drop_cfg_time_const(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"time const %d ", temp);

			error = khwq_qos_get_drop_cfg_tail_thresh(info, idx,
								  &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"tail thresh %d ", temp);

			error = khwq_qos_get_drop_cfg_red_low(info, idx, &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"red low %d ", temp);

			error = khwq_qos_get_drop_cfg_red_high(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"red high %d ", temp);

			error = khwq_qos_get_drop_cfg_thresh_recip(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"thresh recip %d\n", temp);

			spin_unlock_irqrestore(&info->lock, flags);
		}
	}

free:
	ret = simple_read_from_buffer(buffer, len, ppos, buf, buf_len);
	kfree(buf);

	return ret;
}

static ssize_t khwq_qos_sched_port_read(struct file *filp, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct khwq_qos_info *info = filp->private_data;
	struct khwq_qos_shadow *shadow = &info->shadows[QOS_SCHED_PORT_CFG];
	struct khwq_pdsp_info *pdsp;
	int i, j, buf_len = 4096, idx, error;
	unsigned long flags;
	size_t len = 0;
	ssize_t ret;
	char *buf;
	u32 temp, temp2, queues;

	if (*ppos != 0)
		return 0;
	if (count < sizeof(buf))
		return -ENOSPC;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pdsp = info->pdsp;

	for (i = shadow->start; i < (shadow->start + shadow->count); i++) {
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);

			spin_lock_irqsave(&info->lock, flags);

			len += snprintf(buf + len, buf_len - len,
					"port %d\n", i);

			error = khwq_qos_get_sched_unit_flags(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"unit flags %d ", temp);

			error = khwq_qos_get_sched_group_count(info, idx,
							       &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"group # %d ", temp);

			error = khwq_qos_get_sched_out_queue(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"out q %d ", temp);

			error = khwq_qos_get_sched_overhead_bytes(info, idx,
								  &temp);
			if (WARN_ON(error))
				goto free;

			error = khwq_qos_get_sched_remove_bytes(info, idx,
								  &temp2);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"overhead bytes %d ", temp - temp2);

			error = khwq_qos_get_sched_out_throttle(info, idx,
								&temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"throttle thresh %d ", temp);

			error = khwq_qos_get_sched_cir_credit(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"cir credit %d ", temp);

			error = khwq_qos_get_sched_cir_max(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"cir max %d\n", temp);

			error = khwq_qos_get_sched_total_q_count(info, idx,
								 &queues);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"total q's %d ", queues);

			error = khwq_qos_get_sched_sp_q_count(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"sp q's %d ", temp);

			error = khwq_qos_get_sched_wrr_q_count(info, idx,
								 &temp);
			if (WARN_ON(error))
				goto free;

			len += snprintf(buf + len, buf_len - len,
					"wrr q's %d\n", temp);

			for (j = 0; j < queues; j++) {
				len += snprintf(buf + len, buf_len - len,
					"queue %d ", j);

				error = khwq_qos_get_sched_cong_thresh(info,
								       idx, j,
								       &temp);
				if (WARN_ON(error))
					return error;

				len += snprintf(buf + len, buf_len - len,
					"cong thresh %d ", temp);

				error = khwq_qos_get_sched_wrr_credit(info, idx,
								      j, &temp);
				if (WARN_ON(error))
					return error;

				len += snprintf(buf + len, buf_len - len,
					"wrr credit %d\n", temp);
			}

			len += snprintf(buf + len, buf_len - len, "\n");

			spin_unlock_irqrestore(&info->lock, flags);
		}
	}
free:
	ret = simple_read_from_buffer(buffer, len, ppos, buf, buf_len);
	kfree(buf);

	return ret;
}

static int khwq_qos_debufs_generic_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

static const struct file_operations khwq_qos_out_profs_fops = {
	.owner	= THIS_MODULE,
	.open	= khwq_qos_debufs_generic_open,
	.read	= khwq_qos_out_prof_read,
	.llseek	= default_llseek,
};

static const struct file_operations khwq_qos_q_cfg_fops = {
	.owner	= THIS_MODULE,
	.open	= khwq_qos_debufs_generic_open,
	.read	= khwq_qos_q_cfg_read,
	.llseek	= default_llseek,
};

static const struct file_operations khwq_qos_drop_prof_fops = {
	.owner	= THIS_MODULE,
	.open	= khwq_qos_debufs_generic_open,
	.read	= khwq_qos_drop_prof_read,
	.llseek	= default_llseek,
};

static const struct file_operations khwq_qos_sched_port_fops = {
	.owner	= THIS_MODULE,
	.open	= khwq_qos_debufs_generic_open,
	.read	= khwq_qos_sched_port_read,
	.llseek	= default_llseek,
};

static int khwq_qos_open_queue(struct khwq_range_info *range,
			       struct hwqueue_instance *inst, unsigned flags)
{
	struct khwq_qos_info *info;
	struct khwq_device *kdev;
	int error = 0;

	info = range->qos_info;
	kdev = info->kdev;

	info->refcount++;
	if (info->refcount == 1) {
		error = khwq_qos_start(info);
		if (error)
			dev_err(kdev->dev, "failed to start qos\n");

		info->port_configs = debugfs_create_file("sched_ports", S_IRUSR,
							 info->root_dir, info,
							 &khwq_qos_sched_port_fops);

		info->out_profiles = debugfs_create_file("out_profiles",
							 S_IRUSR,
							 info->root_dir, info,
							 &khwq_qos_out_profs_fops);
	
		info->queue_configs = debugfs_create_file("queue_configs",
							  S_IRUSR,
							  info->root_dir, info,
							  &khwq_qos_q_cfg_fops);

		info->config_profiles = debugfs_create_file("config_profiles",
							    S_IRUSR,
							    info->root_dir,
							    info,
							    &khwq_qos_drop_prof_fops);

	}

	return error;
}

int khwq_qos_stop(struct khwq_qos_info *info)
{
	struct khwq_device *kdev;
	u32 command;
	int error = 0;

	kdev = info->kdev;

	/* Disable the drop scheduler */
	command = (QOS_CMD_ENABLE_PORT |
		   QOS_DROP_SCHED_ENABLE | QOS_DISABLE);
	error = khwq_qos_write_cmd(info, command);
	if (error)
		dev_err(kdev->dev, "failed to disable "
			"drop scheduler\n");

	error = khwq_qos_tree_stop(info);
	if (error) {
		dev_err(kdev->dev, "failed to close qos tree\n");
		return error;
	}

	del_timer_sync(&info->timer);

	return error;
}

static int khwq_qos_close_queue(struct khwq_range_info *range,
				struct hwqueue_instance *inst)
{
	struct khwq_qos_info *info;
	struct khwq_device *kdev;
	int error = 0;

	info = range->qos_info;
	kdev = info->kdev;

	info->refcount--;
	if (!info->refcount) {
		error = khwq_qos_stop(info);
		if (error)
			dev_err(kdev->dev, "failed to stop qos\n");

		debugfs_remove(info->port_configs);
		debugfs_remove(info->queue_configs);
		debugfs_remove(info->out_profiles);
		debugfs_remove(info->config_profiles);
	}

	return error;
}

static int khwq_qos_free_range(struct khwq_range_info *range)
{
	struct khwq_qos_info *info;
	struct khwq_qos_shadow *shadow;
	struct khwq_device *kdev;
	struct khwq_pdsp_info *pdsp;
	int i, idx;

	info = range->qos_info;
	pdsp = info->pdsp;
	kdev = info->kdev;

	debugfs_remove_recursive(info->root_dir);

	shadow = &info->shadows[QOS_SCHED_PORT_CFG];
	for (i = shadow->start; i < (shadow->start + shadow->count); i++)
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			khwq_qos_free_sched_port(info, idx);
		}

	shadow = &info->shadows[QOS_DROP_OUT_PROF];
	for (i = shadow->start; i < (shadow->start + shadow->count); i++)
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			khwq_qos_free_drop_out(info, idx);
		}

	shadow = &info->shadows[QOS_DROP_QUEUE_CFG];
	for (i = shadow->start; i < (shadow->start + shadow->count); i++)
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			khwq_qos_free_drop_queue(info, idx);
		}

	shadow = &info->shadows[QOS_DROP_CFG_PROF];
	for (i = shadow->start; i < (shadow->start + shadow->count); i++)
		if (!test_bit(i, shadow->avail)) {
			idx = khwq_qos_make_id(pdsp->id, i);
			khwq_qos_free_drop_cfg(info, idx);
		}

	return 0;
}

static int khwq_qos_set_notify(struct khwq_range_info *range,
			       struct hwqueue_instance *inst, bool enabled)
{
	return 0;
}

static int khwq_qos_push(struct hwqueue_instance *inst, dma_addr_t dma,
			 unsigned size, unsigned flags)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;
	struct khwq_pdsp_info *pdsp;
	struct khwq_qos_info *info;
	unsigned long irq_flags;
	u32 val;

	qmgr = khwq_find_qmgr(inst);
	if (!qmgr)
		return -ENODEV;

	info = range->qos_info;
	pdsp = info->pdsp;

	spin_lock_irqsave(&info->lock, irq_flags);

	while(__raw_readl(pdsp->command + QOS_PUSH_PROXY_OFFSET + 0x4));
	val = (id << 16) | (flags & BITS(17));
	__raw_writel(val, pdsp->command + QOS_PUSH_PROXY_OFFSET);
	
	val = (u32)dma | ((size / 16) - 1);
	__raw_writel(val, pdsp->command + QOS_PUSH_PROXY_OFFSET + 0x4);

	spin_unlock_irqrestore(&info->lock, irq_flags);

	return 0;
}

static int khwq_qos_init_range(struct khwq_range_info *range)
{
	struct khwq_pdsp_info *pdsp;
	struct khwq_qos_info *info;
	struct khwq_device *kdev;
	u32 command, magic, version;
	int error, i, idx, timer_config;
	unsigned long pdsp_clock_rate;
	char name[24];

	info = range->qos_info;
	pdsp = info->pdsp;
	kdev = info->kdev;

	pdsp_clock_rate = clk_get_rate(kdev->clk);

	range->inst_ops.push = khwq_qos_push;

	snprintf(name, sizeof(name), "qos-%d", pdsp->id);

	spin_lock_bh(&info->lock);

	magic = __raw_readl(pdsp->command + QOS_MAGIC_OFFSET);
	version = __raw_readl(pdsp->command + QOS_VERSION_OFFSET);

	if ((magic >> 16) != QOS_MAGIC_DROPSCHED) {
		dev_err(kdev->dev, "invalid qos magic word %x\n", magic);
		error = -EINVAL;
		goto fail;
	}

	dev_info(kdev->dev, "qos version 0x%x, magic %s\n", version,
		 ((magic >> 16) == QOS_MAGIC_DROPSCHED) ? "valid" : "invalid");

	for (i = 0 ; i < info->shadows[QOS_SCHED_PORT_CFG].count; i++) {
		idx = khwq_qos_make_id(pdsp->id, i);
		__khwq_qos_set_sched_overhead_bytes(info, idx,
						    QOS_DEFAULT_OVERHEAD_BYTES,
						    false);
		__khwq_qos_set_sched_remove_bytes(info, idx, 0, false);
	}

	for (i = 0 ; i < info->shadows[QOS_DROP_CFG_PROF].count; i++) {
		idx = khwq_qos_make_id(pdsp->id, i);
		__khwq_qos_set_drop_cfg_tail_thresh(info, idx, -1, false);
	}

	/* command for drop scheduler base */
	command = (QOS_CMD_SET_QUEUE_BASE | QOS_QUEUE_BASE_DROP_SCHED |
		   (info->drop_sched_queue_base << 16));
	error = khwq_qos_write_cmd(info, command);
	if (error) {
		dev_err(kdev->dev, "failed to set drop sched base\n");
		goto fail;
	}

	/* command for qos scheduler base */
	command = (QOS_CMD_SET_QUEUE_BASE | QOS_QUEUE_BASE_QOS_SCHED |
		   (info->sched_port_queue_base << 16));
	error = khwq_qos_write_cmd(info, command);
	if (error) {
		dev_err(kdev->dev, "failed to set qos sched base\n");
		goto fail;
	}

	/* calculate the timer config from the pdsp tick */
	timer_config = pdsp_clock_rate / info->ticks_per_sec;
	timer_config /= 2;
	command = (QOS_CMD_SET_TIMER_CONFIG | ((timer_config & 0xffff) << 16));
	error = khwq_qos_write_cmd(info, command);
	if (error) {
		dev_err(kdev->dev, "failed to set timer\n");
		goto fail;
	}

	error = khwq_qos_program_drop_sched(info);
	if (error) {
		dev_err(kdev->dev, "failed to initialize drop scheduler\n");
		goto fail;
	}

	spin_unlock_bh(&info->lock);

	error = khwq_program_drop_policies(info);
	if (error) {
		dev_err(kdev->dev, "failed to initialize drop policies\n");
		goto fail;
	}

	info->root_dir = debugfs_create_dir(name, NULL);
	if (!info->root_dir)
		goto fail;

	return 0;
fail:
	spin_unlock_bh(&info->lock);
	return error;
}

struct khwq_range_ops khwq_qos_range_ops = {
	.set_notify	= khwq_qos_set_notify,
	.init_queue	= khwq_qos_init_queue,
	.open_queue	= khwq_qos_open_queue,
	.close_queue	= khwq_qos_close_queue,
	.init_range	= khwq_qos_init_range,
	.free_range	= khwq_qos_free_range,
};

int khwq_init_qos_range(struct khwq_device *kdev, struct device_node *node,
			struct khwq_range_info *range)
{
	struct khwq_pdsp_info *pdsp = NULL;
	struct khwq_qos_info *info;
	struct device_node *child;
	struct device *dev = kdev->dev;
	u32 temp[7];
	int error;
	int tree_index;

	info = devm_kzalloc(kdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(kdev->dev, "failed to alloc qos info\n");
		return -ENOMEM;
	}

	info->kobj = kobject_create_and_add(node->name,
					kobject_get(&dev->kobj));
	if (!info->kobj) {
		dev_err(kdev->dev, "could not create sysfs entries for qos\n");
		devm_kfree(kdev->dev, info);
		return -ENODEV;
	}

	info->kobj_stats = kobject_create_and_add("statistics", info->kobj);
	if (!info->kobj_stats) {
		dev_err(kdev->dev, "could not create sysfs entries for "
			"qos statistics\n");
		devm_kfree(kdev->dev, info);
		return -ENODEV;
	}

	info->kobj_policies = kobject_create_and_add("drop-policies",
						     info->kobj);
	if (!info->kobj_policies) {
		dev_err(kdev->dev, "could not create sysfs entries for "
			"qos drop policies\n");
		devm_kfree(kdev->dev, info);
		return -ENODEV;
	}
	
	error = of_property_read_u32(node, "pdsp-id", &info->pdsp_id);
	if (error < 0) {
		dev_err(kdev->dev, "pdsp id must be specified\n");
		devm_kfree(kdev->dev, info);
		return error;
	}

	pdsp = khwq_find_pdsp(kdev, info->pdsp_id);
	if (!pdsp) {
		dev_err(kdev->dev, "pdsp id %d not found for range %s\n",
			info->pdsp_id, range->name);
		devm_kfree(kdev->dev, info);
		return -ENODEV;
	}

	if (pdsp->qos_info) {
		dev_err(kdev->dev, "pdsp id %d is busy\n", info->pdsp_id);
		devm_kfree(kdev->dev, info);
		return -EBUSY;
	}

	info->pdsp = pdsp;
	info->kdev = kdev;
	INIT_LIST_HEAD(&info->drop_policies);
	INIT_LIST_HEAD(&info->stats_classes);
	spin_lock_init(&info->lock);
	/* TODO: add refcount handlers */
	for (tree_index = 0; tree_index < QOS_MAX_TREES; ++tree_index)
		ktree_init(&info->qos_trees[tree_index],
			   khwq_qos_get_tree_node,
			   khwq_qos_put_tree_node);

	error = of_property_read_u32_array(node, "qos-cfg", temp, 7);
	if (error < 0) {
		dev_err(kdev->dev, "failed to obtain qos scheduler config\n");
		goto bail;
	}
	info->inputs_per_port	  = temp[0];
	info->drop_cfg.int_num	  = temp[1];
	info->drop_cfg.qos_ticks  = temp[2];
	info->drop_cfg.drop_ticks = temp[3];
	info->drop_cfg.seed[0]	  = temp[4];
	info->drop_cfg.seed[1]	  = temp[5];
	info->drop_cfg.seed[2]	  = temp[6];

	error = of_property_read_u32(node, "tick-per-sec",
				     &info->ticks_per_sec);
	if (error < 0)
		info->ticks_per_sec = 10000;

	error = khwq_qos_init_shadow(info, QOS_SCHED_PORT_CFG,
				     "sched-port-configs", node, false);
	if (error)
		goto bail;

	error = khwq_qos_init_shadow(info, QOS_DROP_CFG_PROF,
				     "drop-cfg-profiles", node, true);
	if (error)
		goto bail;

	error = khwq_qos_init_shadow(info, QOS_DROP_OUT_PROF,
				     "drop-out-profiles", node, true);
	if (error)
		goto bail;

	error = khwq_qos_init_shadow(info, QOS_DROP_QUEUE_CFG,
				     "drop-queue-configs", node, true);
	if (error)
		goto bail;

	error = khwq_qos_init_stats(info, node);
	if (error)
		goto bail;

	error = khwq_qos_init_base(info, range->queue_base, range->num_queues);
	if (error)
		goto bail;

	pdsp->qos_info  = info;
	range->qos_info = info;

	child = of_parse_phandle(node, "drop-policies", 0);
	if (!child)
		child = of_get_child_by_name(node, "drop-policies");
	if (!child) {
		dev_err(kdev->dev, "could not find drop policies\n");
		goto bail;
	}
	error = khwq_qos_get_drop_policies(kdev, info, child);
	if (error)
		goto bail;
	of_node_put(child);

	for (tree_index = 0;; ++tree_index) {
		child = of_parse_phandle(node, "qos-tree", tree_index);
		if (!child && (tree_index == 0))
			child = of_get_child_by_name(node, "qos-tree");
		if (!child) {
			if (tree_index != 0)
				break;
			else {
				dev_err(kdev->dev, "could not find qos tree\n");
				goto bail;
			}
		}
		if (tree_index >= QOS_MAX_TREES) {
			of_node_put(child);
			dev_err(kdev->dev, "too many qos trees\n");
			break;
		}

		error = khwq_qos_tree_parse(info, tree_index, child, NULL);
		of_node_put(child);
		if (error)
			goto bail;
	}
	info->qos_tree_count = tree_index;

	error = khwq_qos_tree_init(info);
	if (error)
		goto bail;

	range->ops = &khwq_qos_range_ops;

	return 0;

bail:
	__khwq_free_qos_range(kdev, info);

	range->qos_info	= NULL;
	range->ops	= NULL;
	if (pdsp)
		pdsp->qos_info	= NULL;

	return error;
}
