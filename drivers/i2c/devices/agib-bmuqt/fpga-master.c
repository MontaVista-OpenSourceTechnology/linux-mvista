/* FPGA master framework
 *
 * Copyright (C) 2018 Nokia
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/acpi.h>
#include <fsmddg_fpga-master.h>

static LIST_HEAD(fpga_master_list);
static DEFINE_MUTEX(fpga_master_list_lock);

struct fpga_master {
	fpga_master_func_t ops_call;
	void *ops_priv;
	struct device *dev;
	struct regmap *regmap;
	struct list_head list;
	bool locked;
};

static int _fpga_master_fill(struct device *dev, void *data)
{
	struct platform_device *pdev;
	struct device_node *node;
	struct resource *res;
	const __be32 *cell;
	const char *name;
	int n, j, num_reg = 0, i = 0;
	u64 size;

	node = dev->of_node;
	if (!node) {
		pr_debug("%s: of_node is NULL\n", __func__);
		return 0;
	}

	pdev = to_platform_device(dev);

	// Ignore this node if there are any non IRQ resources registred already
	for (j = 0; j < pdev->num_resources; j++) {
		struct resource *r = &pdev->resource[j];
		if (resource_type(r) == IORESOURCE_MEM) {
			pr_debug("%s: '%s' already initialized\n", __func__, pdev->name);
			return 0;
		}
	}

	n = of_n_addr_cells(dev->of_node);

	while ((cell = of_get_address(node, num_reg, &size, NULL)))
		num_reg++;

	if (num_reg == 0)
		return 0;

	res = kcalloc(num_reg, sizeof(*res), GFP_KERNEL);
	if (!res)
		return 0;

	while ((cell = of_get_address(node, i, &size, NULL))) {
		name = NULL;
		of_property_read_string_index(node, "reg-names", i, &name);
		res[i].start = of_read_number(cell, n);
		res[i].end = res[i].start + size;
		res[i].name = name;
		res[i].flags = IORESOURCE_MEM;
		pr_debug("%s: [%d] %s - %llx\n", __func__, i, node->name, res[i].start);
		i++;
	}

	return platform_device_add_resources(pdev, res, num_reg);
}

static struct fpga_master *_fpga_master_find_dev(void *np)
{
	struct fpga_master *entry;

	list_for_each_entry(entry, &fpga_master_list, list)
		if (entry->dev->of_node == np || entry->dev->fwnode == np)
			return entry;

	return NULL;
}

static void *_fpga_master_find_parent_fwnode(const struct device *dev)
{
	struct fwnode_handle *ret = ERR_PTR(-EINVAL);
#ifdef CONFIG_ACPI
	struct acpi_device *acpi_dev = to_acpi_device_node(dev->fwnode);

	if (acpi_dev == NULL || acpi_dev->parent == NULL)
		return ret;

	ret = &acpi_dev->parent->fwnode;
	if (ret == NULL)
		return ERR_PTR(-EINVAL);

#endif
	return ret;
}

static void *_fpga_master_find_parent_of(const struct device *dev)
{
	struct device_node *np = dev->of_node;
	const __be32 *prop;

	if (np == NULL)
		return ERR_PTR(-EINVAL);

	prop = of_get_property(np, "fpga-master", NULL);
	if (prop) {
		np = of_find_node_by_phandle(be32_to_cpup(prop));
		if (!np)
			return ERR_PTR(-EPROBE_DEFER);
	} else if (np->parent) {
		np = np->parent;
	} else {
		return ERR_PTR(-EINVAL);
	}

	return np;
}

static void *_fpga_master_find_parent(const struct device *dev)
{
	if (dev->of_node)
		return _fpga_master_find_parent_of(dev);
	else if (dev->fwnode)
		return _fpga_master_find_parent_fwnode(dev);

	return ERR_PTR(-EINVAL);
}

/**
 * fpga_master_register - register regmap for specified node in DT
 *
 * @dev:  pointer to struct device
 * @regm: pointer to regmap that will be registered for this node
 * @ops_call: optional pointer to a callback function
 * @ops_priv: optional private data passed to callback function
 *
 * Returns valid pointer to fpga_master struct in case of success or ERR_PTR of:
 * - EINVAL - if this device was not instantiated from device tree
 * - EEXIST - if some regmap is already registered for this device tree node
 * - ENOMEM - if there was a memory allocation error
 */
struct fpga_master *fpga_master_register(struct device *dev,
					 struct regmap *regm,
					 fpga_master_func_t ops_call,
					 void *ops_priv)
{
	struct fpga_master *ret;
	void *node = dev->of_node;

	if (node == NULL)
		node = dev->fwnode;

	if (node == NULL)
		return ERR_PTR(-EINVAL);

	mutex_lock(&fpga_master_list_lock);
	if (_fpga_master_find_dev(node) != NULL) {
		ret = ERR_PTR(-EEXIST);
		goto out;
	}

	ret = kzalloc(sizeof(*ret), GFP_KERNEL);
	if (!ret) {
		ret = ERR_PTR(-ENOMEM);
		goto out;
	}

	ret->dev = dev;
	ret->regmap = regm;
	ret->ops_call = ops_call;
	ret->ops_priv = ops_priv;

	list_add_tail(&ret->list, &fpga_master_list);

	if (dev->of_node) { // The ACPI does this on its own
		of_platform_populate(dev->of_node, NULL, NULL, dev);
		device_for_each_child(dev, NULL, _fpga_master_fill);
	}

out:
	mutex_unlock(&fpga_master_list_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(fpga_master_register);

/**
 * fpga_master_unregister_regmap - unregister regmap registered by
 *                                 fpga_master_register
 *
 * @regm: pointer to regmap registered by fpga_master_register
 *
 * Returns 0 in case of a success or -EBUSY if this entry is locked (some
 * client already requested this regmap).
 */
int fpga_master_unregister_regmap(struct regmap *regm)
{
	struct fpga_master *entry;
	int ret = 0;

	mutex_lock(&fpga_master_list_lock);

	list_for_each_entry(entry, &fpga_master_list, list) {
		if (entry->regmap == regm) {
			if (!entry->locked) {
				list_del(&entry->list);
				kfree(entry);
			} else {
				ret = -EBUSY;
			}
			break;
		}
	}

	mutex_unlock(&fpga_master_list_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(fpga_master_unregister_regmap);

/**
 * fpga_master_get - Get handle for fpga master
 *
 * @dev: struct device of the client device
 *
 * Returns pointer to fpga master handle or ERR_PTR() of following codes:
 * - EPROBE_DEFER - returned in case of nodes using "fpga-master" property to
 *                  point to FPGA device if this device did not (yet?) register
 *                  regmap
 * - EINVAL       - returned if this function is called for a device that
 *                  does not have a parent in device tree or does not point
 *                  to a valid node using "fpga-master" property
 * - ENOENT       - returned if no regmap is registered for parent node in
 *                  device tree
 */
struct fpga_master *fpga_master_get(const struct device *dev)
{
	struct fpga_master *fpga;
	struct fpga_master *ret = ERR_PTR(-ENOENT);
	void *parent;

	parent = _fpga_master_find_parent(dev);
	if (IS_ERR(parent))
		return parent;

	mutex_lock(&fpga_master_list_lock);
	fpga = _fpga_master_find_dev(parent);
	if (!fpga)
		goto out;

	/* We are passing a reference to this regmap to someone and we
	 * can't control if is still valid so we have to block the device
	 * that registered it from unloading its module. We still can't block
	 * it from being unbinded but well..
	 */
	if (!fpga->locked) {
		if (try_module_get(fpga->dev->driver->owner))
			get_device(fpga->dev);
		else
			goto out;

		fpga->locked = true;
	}

	ret = fpga;
out:
	mutex_unlock(&fpga_master_list_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(fpga_master_get);

/**
 * fpga_master_node_to_regmap - Find a regmap registered for specified node
 *
 * @dev: struct device of the client device
 *
 * Returns pointer to struct regmap or ERR_PTR() of following codes:
 * - EPROBE_DEFER - returned in case of nodes using "fpga-master" property to
 *                  point to FPGA device if this device did not (yet?) register
 *                  regmap
 * - EINVAL       - returned if this function is called for a device that
 *                  does not have a parent in device tree or does not point
 *                  to a valid node using "fpga-master" property
 * - ENOENT       - returned if no regmap is registered for parent node in
 *                  device tree
 */
struct regmap *fpga_master_node_to_regmap(const struct device *dev)
{
	struct fpga_master *fpgam = fpga_master_get(dev);
	if (IS_ERR(fpgam))
		return ERR_PTR(PTR_ERR(fpgam));

	return fpgam->regmap;
}
EXPORT_SYMBOL_GPL(fpga_master_node_to_regmap);

int fpga_master_call(struct fpga_master *fpgam, int op, void *data)
{
	if (!fpgam->ops_call)
		return -ENOENT;

	return fpgam->ops_call(op, fpgam->ops_priv, data);
}
EXPORT_SYMBOL_GPL(fpga_master_call);

static void devm_fpga_master_release(struct device *dev, void *res)
{
	struct regmap *regm = *(struct regmap **)res;
	fpga_master_unregister_regmap(regm);
}

struct fpga_master *devm_fpga_master_register(struct device *dev,
					      struct regmap *regm,
					      fpga_master_func_t ops_call,
					      void *ops_priv)
{
	struct fpga_master *ret;
	struct regmap **ptr;

	ptr = devres_alloc(devm_fpga_master_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	ret = fpga_master_register(dev, regm, ops_call, ops_priv);
	if (!IS_ERR(ret)) {
		*ptr = regm;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return ret;
}
EXPORT_SYMBOL(devm_fpga_master_register);

struct fpga_master *devm_fpga_master_register_regmap(struct device *dev,
						     struct regmap *regm)
{
	return devm_fpga_master_register(dev, regm, NULL, NULL);
}
EXPORT_SYMBOL(devm_fpga_master_register_regmap);

static int __init fpga_master_init(void)
{
	return 0;
}

static void __exit fpga_master_exit(void)
{
}

module_init(fpga_master_init);
module_exit(fpga_master_exit);

MODULE_AUTHOR("Krzysztof Adamski <krzysztof.adamski@nokia.com");
MODULE_DESCRIPTION("FPGA master framework");
MODULE_LICENSE("GPL");
