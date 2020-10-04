/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Reece Pollack <reece@theptrgroup.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/errqueue.h>

#include "keystone_net.h"

#define	QOS_TXHOOK_ORDER	20

struct qos_channel {
	const char		*tx_chan_name;
	u32			 tx_queue_depth;
	struct netcp_tx_pipe	 tx_pipe;
	struct kobject		 kobj;
};

struct qos_device {
	struct netcp_device		*netcp_device;
	struct device			*dev;
	struct device_node		*node;
	u32				 multi_if;
	struct kobject			 kobj;
};

struct qos_intf {
	struct qos_device		*qdev;
	struct net_device		*ndev;
	struct device			*dev;
	struct kobject			 kobj;
	int				 num_channels;
	int				 max_channels;
	struct qos_channel		 channels[1];
	/* NB: channels is allocated dynamically */
};

/*
 *  Sysfs stuff related to QoS TX Channels
 */
#define kobj_to_qchan(kobj) container_of(kobj, struct qos_channel, kobj)

static ssize_t qos_txchan_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	struct qos_channel *qos_chan = kobj_to_qchan(kobj);
	return snprintf(buf, PAGE_SIZE, "%s\n", qos_chan->tx_chan_name);
}

struct kobj_attribute qos_txchan_attr =
	__ATTR(tx-channel, S_IRUGO, qos_txchan_show, NULL);


static ssize_t qos_txdepth_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	struct qos_channel *qos_chan = kobj_to_qchan(kobj);
	return snprintf(buf, PAGE_SIZE, "%u\n", qos_chan->tx_queue_depth);
}

static ssize_t qos_txdepth_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	struct qos_channel *qos_chan = kobj_to_qchan(kobj);
	struct netcp_tx_pipe *tx_pipe = &qos_chan->tx_pipe;
	unsigned int val;

	if (kstrtouint(buf, 0, &val) < 0)
		return -EINVAL;

	qos_chan->tx_queue_depth = val;
	tx_pipe->dma_queue_depth = val;

	return count;
}

struct kobj_attribute qos_txdepth_attr =
	__ATTR(tx_queue_depth, S_IRUGO | S_IWUSR,
		qos_txdepth_show, qos_txdepth_store);

static struct attribute *qos_txchan_attrs[] = {
		&qos_txchan_attr.attr,
		&qos_txdepth_attr.attr,
		NULL
};

static struct kobj_type qos_chan_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.default_attrs = qos_txchan_attrs,
};

/* Sysfs stuff related to QoS Interfaces */
static struct kobj_type qos_intf_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
};

/* Sysfs stuff related to QoS Instances */
#define	kobj_to_qdev(kobj) container_of(kobj, struct qos_device, kobj)
static ssize_t qos_multi_if_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	struct qos_device *qos_dev = kobj_to_qdev(kobj);
	return snprintf(buf, PAGE_SIZE, "%u\n", qos_dev->multi_if);
}

struct kobj_attribute qos_multi_if_attr =
	__ATTR(multi-interface, S_IRUGO, qos_multi_if_show, NULL);

static struct attribute *qos_inst_attrs[] = {
		&qos_multi_if_attr.attr,
		NULL
};

static struct kobj_type qos_inst_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.default_attrs = qos_inst_attrs,
};


static int qos_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct qos_intf *qos_intf = data;
	struct sk_buff *skb = p_info->skb;

	dev_dbg(qos_intf->dev,
		"priority: %u, queue_mapping: %04x\n",
		skb->priority, skb_get_queue_mapping(skb));

	if (skb->queue_mapping < qos_intf->num_channels)
		p_info->tx_pipe =
			&qos_intf->channels[skb->queue_mapping].tx_pipe;
	else {
		dev_warn(qos_intf->dev,
			"queue mapping (%d) >= num chans (%d) QoS bypassed\n",
			 skb_get_queue_mapping(skb), qos_intf->num_channels);
	}

	return 0;
}


static int qos_close(void *intf_priv, struct net_device *ndev)
{
	struct qos_intf *qos_intf = intf_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	int i;

	netcp_unregister_txhook(netcp_priv, QOS_TXHOOK_ORDER, qos_tx_hook,
				qos_intf);

	for (i = 0; i < qos_intf->num_channels; ++i) {
		struct qos_channel *qchan = &qos_intf->channels[i];

		netcp_txpipe_close(&qchan->tx_pipe);
	}

	return 0;
}

static int qos_open(void *intf_priv, struct net_device *ndev)
{
	struct qos_intf *qos_intf = intf_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	int ret;
	int i;

	/* Open the QoS input queues */
	for (i = 0; i < qos_intf->num_channels; ++i) {
		struct qos_channel *qchan = &qos_intf->channels[i];

		ret = netcp_txpipe_open(&qchan->tx_pipe);
		if (ret)
			goto fail;
	}

	netcp_register_txhook(netcp_priv, QOS_TXHOOK_ORDER, qos_tx_hook,
			      intf_priv);

	return 0;

fail:
	qos_close(intf_priv, ndev);
	return ret;
}

static int init_channel(struct qos_intf *qos_intf,
			int index,
			struct device_node *node)
{
	struct qos_channel *qchan = &qos_intf->channels[index];
	int ret;

	ret = of_property_read_string(node, "tx-channel", &qchan->tx_chan_name);
	if (ret < 0) {
		dev_err(qos_intf->dev,
			"missing tx-channel parameter, err %d\n", ret);
		qchan->tx_chan_name = "qos";
	}
	dev_dbg(qos_intf->dev, "tx-channel \"%s\"\n", qchan->tx_chan_name);

	ret = of_property_read_u32(node, "tx_queue_depth",
				   &qchan->tx_queue_depth);
	if (ret < 0) {
		dev_err(qos_intf->dev,
			"missing tx_queue_depth parameter, err %d\n", ret);
		qchan->tx_queue_depth = 16;
	}
	dev_dbg(qos_intf->dev, "tx_queue_depth %u\n", qchan->tx_queue_depth);

	/* Create the per-channel entry and attributes */
	ret = kobject_init_and_add(&qchan->kobj, &qos_chan_ktype,
			kobject_get(&qos_intf->kobj), node->name);
	if (ret) {
		dev_err(qos_intf->dev,
			"failed to create %s/%s/%s sysfs entry\n",
			qos_intf->qdev->kobj.name, qos_intf->kobj.name,
			node->name);
		kobject_put(&qchan->kobj);
		kobject_put(&qos_intf->kobj);
		return ret;
	}

	return 0;
}

static int qos_attach(void *inst_priv, struct net_device *ndev,
		      void **intf_priv)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct qos_device *qos_dev = inst_priv;
	struct qos_intf *qos_intf;
	struct device_node *interface, *channel;
	char node_name[24];
	int i, ret;
	int max_channels, size;

	max_channels = netcp->ndev->num_tx_queues;
	size = sizeof(struct qos_intf) +
			((max_channels - 1) * sizeof(struct qos_channel));
	qos_intf = devm_kzalloc(qos_dev->dev, size, GFP_KERNEL);
	if (!qos_intf) {
		dev_err(qos_dev->dev,
			"qos interface memory allocation failed\n");
		return -ENOMEM;
	}
	qos_intf->max_channels = max_channels;

	qos_intf->qdev = qos_dev;
	qos_intf->ndev = ndev;
	qos_intf->dev = qos_dev->dev;

	snprintf(node_name, sizeof(node_name), "interface-%d",
		 (qos_dev->multi_if) ? (netcp->cpsw_port - 1) : 0);

	*intf_priv = qos_intf;

	interface = of_get_child_by_name(qos_dev->node, node_name);
	if (!interface) {
		dev_err(qos_intf->dev,
			"could not find interface %d node in device tree\n",
			(netcp->cpsw_port - 1));
		ret = -ENODEV;
		goto exit;
	}

	/* Create the per-interface sysfs entry */
	ret = kobject_init_and_add(&qos_intf->kobj, &qos_intf_ktype,
			kobject_get(&qos_intf->qdev->kobj), node_name);
	if (ret) {
		dev_err(qos_intf->dev, "failed to create %s/%s sysfs entry\n",
			qos_intf->qdev->kobj.name, node_name);
		kobject_put(&qos_intf->kobj);
		kobject_put(&qos_intf->qdev->kobj);
		goto exit;
	}

	qos_intf->num_channels = 0;
	for_each_child_of_node(interface, channel) {
		if (qos_intf->num_channels >= max_channels) {
			dev_err(qos_intf->dev,
				"too many QoS input channels defined\n");
			break;
		}
		init_channel(qos_intf, qos_intf->num_channels, channel);
		++qos_intf->num_channels;
	}

	of_node_put(interface);

	/* Initialize the QoS input queues */
	for (i = 0; i < qos_intf->num_channels; ++i) {
		struct qos_channel *qchan = &qos_intf->channels[i];

		netcp_txpipe_init(&qchan->tx_pipe, netdev_priv(ndev),
				  qchan->tx_chan_name,
				  qchan->tx_queue_depth);

		qchan->tx_pipe.dma_psflags = netcp->cpsw_port;
	}

	return 0;
exit:
	devm_kfree(qos_dev->dev, qos_intf);
	return ret;
}

static int qos_release(void *intf_priv)
{
	struct qos_intf *qos_intf = intf_priv;
	struct qos_device *qos_dev = qos_intf->qdev;
	int i;

	for (i = 0; i < qos_intf->max_channels; ++i) {
		struct qos_channel *qchan = &qos_intf->channels[i];

		if (qchan->tx_chan_name)
			netif_napi_del(&qchan->tx_pipe.dma_poll_napi);
	}

	devm_kfree(qos_dev->dev, qos_intf);

	return 0;
}

static int qos_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct qos_device *qos_dev = inst_priv;

	devm_kfree(qos_dev->dev, qos_dev);

	return 0;
}

static int qos_probe(struct netcp_device *netcp_device,
		    struct device *dev,
		    struct device_node *node,
		    void **inst_priv)
{
	struct qos_device *qos_dev;
	int ret = 0;

	qos_dev = devm_kzalloc(dev, sizeof(struct qos_device), GFP_KERNEL);
	if (!qos_dev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	*inst_priv = qos_dev;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	qos_dev->netcp_device = netcp_device;
	qos_dev->dev = dev;
	qos_dev->node = node;

	if (of_find_property(node, "multi-interface", NULL))
		qos_dev->multi_if = 1;

	/* Create the per-instance sysfs entry */
	ret = kobject_init_and_add(&qos_dev->kobj, &qos_inst_ktype,
		kobject_get(&dev->kobj), node->name);
	if (ret) {
		dev_err(dev, "failed to create %s sysfs entry\n",
			node->name);
		kobject_put(&qos_dev->kobj);
		kobject_put(&dev->kobj);
		goto exit;
	}

	return 0;

exit:
	qos_remove(netcp_device, qos_dev);
	*inst_priv = NULL;
	return ret;
}


static struct netcp_module qos_module = {
	.name		= "keystone-qos",
	.owner		= THIS_MODULE,
	.probe		= qos_probe,
	.open		= qos_open,
	.close		= qos_close,
	.remove		= qos_remove,
	.attach		= qos_attach,
	.release	= qos_release,
};

static int __init keystone_qos_init(void)
{
	return netcp_register_module(&qos_module);
}
module_init(keystone_qos_init);

static void __exit keystone_qos_exit(void)
{
	netcp_unregister_module(&qos_module);
}
module_exit(keystone_qos_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Reece Pollack <reece@theptrgroup.com");
MODULE_DESCRIPTION("Quality of Service driver for Keystone devices");
