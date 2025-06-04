// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Shared sysfs/debugfs macros used by various prestera modules
 *
 * Copyright (c) 2023, Marvell Technology, Inc.
 * Author: Elad Nachman <enachman@marvell.com>
 */

/*********************** A55 EDAC SYSFS macros *************************/

#define A55_DBGFS_TEMPLATE_WRITE(reg)					\
static ssize_t a55_edac_cpu_inject_##reg##_dbgfs_write_handler(struct file *file,	\
			       const char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_device_ctl_info *pci = file->private_data;		\
	struct a55_edac_private *drvdata = pci->pvt_info;		\
	if (!kstrtouint_from_user(data, count, 0, &drvdata->reg))			\
			return count;					\
	return 0;							\
}

#define A55_DBGFS_TEMPLATE_READ(reg)					\
static ssize_t a55_edac_cpu_inject_##reg##_dbgfs_read_handler(struct file *file,	\
			       char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_device_ctl_info *pci = file->private_data;		\
	struct a55_edac_private *drvdata = pci->pvt_info;		\
	char str[16];							\
	sprintf(str, "0x%08x", drvdata->reg);				\
	return simple_read_from_buffer(data, count, ppos,		\
				       str, strlen(str));		\
}

#define A55_DBGFS_TEMPLATE_FOPS(reg)						\
static const struct file_operations a55_edac_cpu_##reg##_fops = {	\
	.open = simple_open,						\
	.read = a55_edac_cpu_inject_##reg##_dbgfs_read_handler,		\
	.write = a55_edac_cpu_inject_##reg##_dbgfs_write_handler,		\
	.llseek = generic_file_llseek,					\
};

#define A55_TEMPLATE_FOPS_AR_ENTRY(reg) &a55_edac_cpu_##reg##_fops

#define A55_TEMPLATE_FOPS_DBGFS_ENTRY(reg) { A55_TEMPLATE_FOPS_AR_ENTRY(reg), #reg }

/*********************** PCI EDAC DEBUGFS macros *************************/

#define PCI_DBGFS_TEMPLATE_WRITE(reg)					\
static ssize_t prestera_edac_pci_inject_##reg##_dbgfs_write_handler(struct file *file,	\
			       const char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_pci_ctl_info *pci = file->private_data;		\
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;		\
	if (!kstrtouint_from_user(data, count, 0, &drvdata->reg))	\
			return count;					\
	return 0;							\
}

#define PCI_DBGFS_TEMPLATE_READ(reg)					\
static ssize_t prestera_edac_pci_inject_##reg##_dbgfs_read_handler(struct file *file,	\
			       char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_pci_ctl_info *pci = file->private_data;		\
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;		\
	char str[16];							\
	sprintf(str, "0x%08x", drvdata->reg);				\
	return simple_read_from_buffer(data, count, ppos,		\
				       str, strlen(str));		\
}

#define PCI_DBGFS_TEMPLATE_WRITE4(reg)					\
static ssize_t prestera_edac_pci_inject_##reg##_dbgfs_write_handler(struct file *file,	\
			       const char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_pci_ctl_info *pci = file->private_data;		\
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;		\
	char buf[128];							\
	ssize_t ret1, ret2;						\
	ret1 = simple_write_to_buffer(buf, sizeof(buf), ppos, data,	\
				     count);				\
	if (ret1 <=0)							\
			return -EINVAL;					\
	ret2 = sscanf(buf, "0x%08x 0x%08x 0x%08x 0x%08x",		\
		     &drvdata->reg[0], &drvdata->reg[1], 		\
		     &drvdata->reg[2], &drvdata->reg[3]); 		\
	if (ret2 != PRESTERA_PCI_NUM_HDR_REGS)				\
		return -EINVAL;						\
	return ret1;							\
}

#define PCI_DBGFS_TEMPLATE_READ4(reg)					\
static ssize_t prestera_edac_pci_inject_##reg##_dbgfs_read_handler(struct file *file,	\
			       char __user *data,			\
			       size_t count, loff_t *ppos)		\
{									\
	struct edac_pci_ctl_info *pci = file->private_data;		\
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;		\
	char str[64];							\
	sprintf(str, "0x%08x 0x%08x 0x%08x 0x%08x",			\
		drvdata->reg[0], drvdata->reg[1], 			\
		drvdata->reg[2], drvdata->reg[3]); 			\
	return simple_read_from_buffer(data, count, ppos,		\
				       str, strlen(str));		\
}


#define PCI_TEMPLATE_FOPS(reg)						\
static const struct file_operations prestera_edac_pci_##reg##_fops = {	\
	.open = simple_open,						\
	.read = prestera_edac_pci_inject_##reg##_dbgfs_read_handler,		\
	.write = prestera_edac_pci_inject_##reg##_dbgfs_write_handler,		\
	.llseek = generic_file_llseek,					\
};

#define PCI_TEMPLATE_FOPS_AR_ENTRY(reg) &prestera_edac_pci_##reg##_fops

#define PCI_TEMPLATE_FOPS_DBGFS_ENTRY(reg) { PCI_TEMPLATE_FOPS_AR_ENTRY(reg), #reg }

/************************ MC SYSFS macros ***************************/

#define SYSFS_TEMPLATE_SHOW(reg)						\
static ssize_t prestera_mc_inject_##reg##_show(struct device *dev,	\
			       struct device_attribute *attr,		\
			       char *data)				\
{									\
	struct mem_ctl_info *mci = to_mci(dev);				\
	struct prestera_lmc_pvt *pvt = mci->pvt_info;			\
	return sprintf(data, "%016llx\n", (u64)pvt->reg);		\
}

#define SYSFS_TEMPLATE_STORE(reg)						\
static ssize_t prestera_mc_inject_##reg##_store(struct device *dev,	\
			       struct device_attribute *attr,		\
			       const char *data, size_t count)		\
{									\
	struct mem_ctl_info *mci = to_mci(dev);				\
	struct prestera_lmc_pvt *pvt = mci->pvt_info;			\
	if (isdigit(*data)) {						\
		if (!kstrtoul(data, 0, &pvt->reg))			\
			return count;					\
	}								\
	return 0;							\
}
