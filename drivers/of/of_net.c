// SPDX-License-Identifier: GPL-2.0-only
/*
 * OF helpers for network devices.
 *
 * Initially copied out of arch/powerpc/kernel/prom_parse.c
 */
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>

/**
 * of_get_phy_mode - Get phy mode for given device_node
 * @np:	Pointer to the given device_node
 *
 * The function gets phy interface string from property 'phy-mode' or
 * 'phy-connection-type', and return its index in phy_modes table, or errno in
 * error case.
 */
int of_get_phy_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "phy-mode", &pm);
	if (err < 0)
		err = of_property_read_string(np, "phy-connection-type", &pm);
	if (err < 0)
		return err;

	for (i = 0; i < PHY_INTERFACE_MODE_MAX; i++)
		if (!strcasecmp(pm, phy_modes(i)))
			return i;

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(of_get_phy_mode);

static const void *of_get_mac_addr(struct device_node *np, const char *name)
{
	struct property *pp = of_find_property(np, name, NULL);

	if (pp && pp->length == ETH_ALEN && is_valid_ether_addr(pp->value))
		return pp->value;
	return NULL;
}

static const void *of_get_mac_addr_mtd(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct of_phandle_args args;
	struct device_node *mac_node = NULL;
	loff_t offset = 0;
	size_t len;
	unsigned int i;
	int args_count;
	int err;
	unsigned char mac_data[12], *mac = NULL;
	struct mtd_info *mtd = NULL;
	bool is_string = false;
	int data_len = 6;

	err = of_parse_phandle_with_args(np, "mtd-mac-address",
					 "#mtd-mac-address-cells", 0, &args);
	if (err)
		goto out_err;
	args_count = args.args_count;
	if (args_count > 2)
		args_count = 2;
	if (args_count)
		offset = of_read_number(args.args, args_count);

	mac_node = args.np;

	is_string = of_property_read_bool(mac_node, "mac-addr-string");
	if (is_string)
		data_len = 12;

	mtd = of_get_mtd_device_by_node(of_get_parent(mac_node));
	if (!mtd) {
		pr_warn("Unable to find MTD device for MAC address\n");
		err = -ENODEV;
		goto out_err;
	}

	err = mtd_read(mtd, offset, data_len, &len, mac_data);
	if (err) {
		pr_warn("Unable to read MAC address from MTD device: %d\n",
			err);
	} else if (len != data_len) {
		pr_warn("Unable to read full MAC address from MTD device: %d\n",
			(int) len);
		err = -ENXIO;
	} else {
		if (is_string) {
			char value[3];

			value[2] = 0;
			for (i = 0; i < 6; i++) {
				value[0] = mac_data[i * 2];
				value[1] = mac_data[i * 2 + 1];
				if (kstrtou8(value, 16, mac_data + i)) {
					err = -EINVAL;
					break;
				}
			}
		}
		if (!err) {
			mac = devm_kmemdup(&pdev->dev, mac_data, ETH_ALEN,
					   GFP_KERNEL);
			if (!mac)
				err = -ENOMEM;
		}
	}

out_err:
	if (mtd)
		put_mtd_device(mtd);
	if (mac_node)
		of_node_put(mac_node);
	put_device(&pdev->dev);

	if (err)
		return ERR_PTR(err);
	return mac;
}

static const void *of_get_mac_addr_nvmem(struct device_node *np)
{
	int ret;
	const void *mac;
	u8 nvmem_mac[ETH_ALEN];
	struct platform_device *pdev = of_find_device_by_node(np);

	if (!pdev)
		return ERR_PTR(-ENODEV);

	ret = nvmem_get_mac_address(&pdev->dev, &nvmem_mac);
	if (ret) {
		put_device(&pdev->dev);
		return ERR_PTR(ret);
	}

	mac = devm_kmemdup(&pdev->dev, nvmem_mac, ETH_ALEN, GFP_KERNEL);
	put_device(&pdev->dev);
	if (!mac)
		return ERR_PTR(-ENOMEM);

	return mac;
}

/**
 * Search the device tree for the best MAC address to use.  'mac-address' is
 * checked first, because that is supposed to contain to "most recent" MAC
 * address. If that isn't set, then 'local-mac-address' is checked next,
 * because that is the default address. If that isn't set, then the obsolete
 * 'address' is checked, just in case we're using an old device tree. If any
 * of the above isn't set, then try to get MAC address from nvmem cell named
 * 'mac-address'.
 *
 * Note that the 'address' property is supposed to contain a virtual address of
 * the register set, but some DTS files have redefined that property to be the
 * MAC address.
 *
 * All-zero MAC addresses are rejected, because those could be properties that
 * exist in the device tree, but were not set by U-Boot.  For example, the
 * DTS could define 'mac-address' and 'local-mac-address', with zero MAC
 * addresses.  Some older U-Boots only initialized 'local-mac-address'.  In
 * this case, the real MAC is in 'local-mac-address', and 'mac-address' exists
 * but is all zeros.
 *
 * Return: Will be a valid pointer on success and ERR_PTR in case of error.
*/
const void *of_get_mac_address(struct device_node *np)
{
	const void *addr;

	addr = of_get_mac_addr(np, "mac-address");
	if (addr)
		return addr;

	addr = of_get_mac_addr(np, "local-mac-address");
	if (addr)
		return addr;

	addr = of_get_mac_addr(np, "address");
	if (addr)
		return addr;

	addr = of_get_mac_addr_mtd(np);
	if (!IS_ERR_OR_NULL(addr))
		return addr;

	return of_get_mac_addr_nvmem(np);
}
EXPORT_SYMBOL(of_get_mac_address);
