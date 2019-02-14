/* SPDX-License-Identifier: GPL-2.0 */

/**
 * mtd3 flags
 *
 * The last eraseblock is reserved to store mtd3 flags (NPU1002 only).
 * There is a 64-bit magic value for each flag.
 * A flag is set if its magic is found at fixed position inside the
 * reserved erase block.
 *
 * NAND partition layout:
 * ======================
 *
 * total size: 1 GiB
 *
 * - mtd3 legacy size
 * ------------------
 *
 *   No reserved block for flags
 *
 *
 *       +----------------+
 *       |  LoadModule1   | 128 MiB
 *       |                |
 *       |----------------|
 *       |  LoadModule2   | 128 MiB
 *       |                |
 *       |----------------|
 *       |                |
 *       |   mtd3 util    |
 *       |   partition    | 256 MiB
 *       |                |
 *       |----------------|
 *       |                |
 *       |                |
 *       |                |
 *       |     Empty      |                                      -
 *       |                |
 *       |                |
 *       |                |
 *       |----------------|
 *       |Bad Block Table | 1 MiB
 *       +----------------+
 *
 *
 * - mtd3 extended size
 *   ------------------
 *
 *   Reserved block for flags after mtd3 partition
 *   and MTD3_FLAG_EXTENDED is set.
 *
 *       +----------------+
 *       |  LoadModule1   | 128 MiB
 *       |                |
 *       |----------------|
 *       |  LoadModule2   | 128 MiB
-*       |                |
 *       |----------------|
 *       |                |
 *       |   mtd3 util    |
 *       |   partition    |
 *       |                |
 *       |                | ~767 Mib
 *       |                |
 *       |                | (767 Mib - 128 KiB)
 *       |                |                                      -
 *       |                |
 *       |                |
 *       |----------------|               +-----------------------------------------
 *       |    Reserved    | 128 KiB ----> | Extended magic | ECC fixed magic |    ...
 *       |----------------|               +-----------------------------------------
 *       |Bad Block Table | 1 MiB               8 bytes         8 bytes
 *       +----------------+
 *
 * The partitions mtd0 and mtd1 are mapped to active loadmodule while mtd2 is mapped
 * to the passive one.
 * Extended mtd3 size (MTD3_FLAG_EXTENDED) was introduced in R3D353.
 * The driver initializes mtd3 size based on existence of extended magic in the reserved block.
 * It is possible to extend or shrink it run-time from userspace using /proc/mtd3_ctl
 *
 * MTD3_FLAG_ECC_FIXED is deprecated and checked only by 1.7 PRA2 and PRA3 SW releases.
 * ECC fix is replaced by erase fix solution (MTD3_FLAG_ERASE_FIXED) in 1.7 GA.
 */

#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define TO_MiB(x) ((x) << 20)

#define MTD3_FLAG_EXTENDED 0x01
#define MTD3_FLAG_ECC_FIXED 0x02 /* deprecated */
#define MTD3_FLAG_ERASE_FIXED 0x04

// magic values for flags
#define MTD3_EXTENDED_MAGIC	0x6465646e65747845UL /* as string: "dednetxE" */
#define MTD3_ECC_FIXED_MAGIC	0x6465786966434345UL /* as string: "dexifCCE" */
#define MTD3_ERASE_FIXED_MAGIC	0x7869666573617245UL /* as string: "xifesarE" */

#define MTD3_LEGACY_SIZE	256		// MiB
#define MTD3_EXTENDED_SIZE	767		// MiB, its last erase block will be reserved for mtd3 flags.
#define MTD3_EXTENDED_NAND_SIZE	0x40000000	// extended mtd3 is supported only on this NAND size

struct mtd_info* master_mtd = NULL;

struct mtd3_flag_descriptor {
	uint32_t flag;
	uint64_t magic;
};

static const struct mtd3_flag_descriptor mtd3_flags[] = {
	{ .flag = MTD3_FLAG_EXTENDED, .magic = MTD3_EXTENDED_MAGIC },
	{ .flag = MTD3_FLAG_ECC_FIXED, .magic = MTD3_ECC_FIXED_MAGIC },
	{ .flag = MTD3_FLAG_ERASE_FIXED, .magic = MTD3_ERASE_FIXED_MAGIC },
};

/**
 * erase_callback - MTD erasure call-back.
 * @ei: MTD erase information object.
 */
static void erase_callback(struct erase_info *ei)
{
	wake_up_interruptible((wait_queue_head_t *)ei->priv);
}

/**
 * do_sync_erase - synchronously erase a physical eraseblock.
 * Based on UBI do_sync_erase function
 *
 * This function synchronously erases physical eraseblock @addr and returns
 * zero in case of success and a negative error code in case of failure. If
 * %-EIO is returned, the physical eraseblock most probably went bad.
 */
static int do_sync_erase(struct mtd_info *master, uint64_t addr)
{
	int err, retries = 0;
	struct erase_info ei;
	wait_queue_head_t wq;
#define ERASE_RETRIES 3

retry:
	init_waitqueue_head(&wq);
	memset(&ei, 0, sizeof(struct erase_info));

	ei.mtd = master;
	ei.addr = addr;
	ei.len = master->erasesize;
	ei.callback = erase_callback;
	ei.priv = (unsigned long)&wq;

	err = mtd_erase(master, &ei);
	if (err) {
		if (retries++ < ERASE_RETRIES) {
			pr_err("error %d while erasing NAND block at %llx, retry",
			       err, (unsigned long long)addr);
			yield();
			goto retry;
		}
		pr_err("cannot erase NAND block at %llx, error %d",
		       (unsigned long long)addr, err);
		dump_stack();
		return err;
	}

	err = wait_event_interruptible(
		wq, ei.state == MTD_ERASE_DONE || ei.state == MTD_ERASE_FAILED);
	if (err) {
		pr_err("interrupted erasure of NAND block at %llx",
		       (unsigned long long)addr);
		return -EINTR;
	}

	if (ei.state == MTD_ERASE_FAILED) {
		if (retries++ < ERASE_RETRIES) {
			pr_err("error while erasing NAND block ar %llx, retry",
			       (unsigned long long)addr);
			yield();
			goto retry;
		}
		pr_err("cannot erase NAND block at %llx",
		       (unsigned long long)addr);
		dump_stack();
		return -EIO;
	}

	return 0;
}

static inline int64_t get_flags_address(struct mtd_info *master)
{
	return master->size - TO_MiB(1) - master->erasesize;
}

int read_mtd3_flags(struct mtd_info *master, uint32_t *flags)
{
	int64_t flag_address = get_flags_address(master);
	uint64_t *magic;
	const int bufsize = ARRAY_SIZE(mtd3_flags) * sizeof(*magic);
	char buf[bufsize];
	size_t len;
	int i;

	mtd_read(master, flag_address, bufsize, &len, (uint8_t *)buf);
	if (len != bufsize) {
		pr_emerg("%s() mtd_read error! req.len(%d) != read.len(%d)\n",
			 __func__, bufsize, len);
		return -EIO;
	}

	*flags = 0;
	magic = (uint64_t *)buf;

	for (i = 0; i < ARRAY_SIZE(mtd3_flags); i++, magic++) {
		if (*magic == mtd3_flags[i].magic)
			*flags |= mtd3_flags[i].flag;
	}

	return 0;
}

int write_mtd3_flags(struct mtd_info *master, uint32_t flags)
{
	uint64_t flag_address = get_flags_address(master);
	size_t len;
	char *buf;
	int ret = 0;
	uint64_t *magic;
	unsigned int i;

	/* Allocate memory for first page's writing (NPU1002: 2 KByte) */
	buf = kmalloc(master->writesize, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* Keep the content of other parts in the page */
	mtd_read(master, flag_address, master->writesize, &len, (uint8_t *)buf);
	if (len != master->writesize) {
		pr_emerg("%s() mtd_read error! req.len(%d) != read.len(%d)\n",
			 __func__, master->writesize, len);
		kfree(buf);
		return -EIO;
	}

	/* Wipe erase block of MTD3 flags (NPU1002: 128 KByte) */
	if (do_sync_erase(master, flag_address)) {
		kfree(buf);
		pr_err("%s() erase failed\n", __func__);
		return -EIO;
	}

	/* Set magic values */
	magic = (uint64_t *)buf;
	for (i = 0; i < ARRAY_SIZE(mtd3_flags); i++, magic++) {
		if (flags & mtd3_flags[i].flag)
			*magic = mtd3_flags[i].magic;
		else
			*magic = (uint64_t)0xffffffffffffffff;
	}

	/* Keep erased if buf is empty */
	for (i = 0; i < master->writesize; i++) {
		if (buf[i] != 0xff)
			break;
	}
	if (i >= master->writesize) {
		kfree(buf);
		return 0;
	}

	mtd_write(master, flag_address, master->writesize, &len,
		  (uint8_t *)buf);
	if (len != master->writesize) {
		pr_err("%s() mtd_write error! req.len(%d) != read.len(%d)\n",
		       __func__, master->writesize, len);
		ret = -EIO;
	}

	kfree(buf);
	return ret;
}

int set_mtd3_flags(struct mtd_info *master, uint32_t flags)
{
	uint32_t nandflags;

	if (read_mtd3_flags(master, &nandflags) < 0)
		return -EIO;

	if ((nandflags & flags) != flags) {
		nandflags |= flags;
		if (write_mtd3_flags(master, nandflags) < 0)
			return -EIO;
	}
	return 0;
}

int clear_mtd3_flags(struct mtd_info *master, uint32_t flags)
{
	uint32_t nandflags;

	if (read_mtd3_flags(master, &nandflags) < 0)
		return -EIO;

	if (nandflags & flags) {
		nandflags &= ~flags;
		if (write_mtd3_flags(master, nandflags) < 0)
			return -EIO;
	}
	return 0;
}

uint64_t npupart_get_mtd3_size(struct mtd_info *master)
{
	uint64_t legacy_size = TO_MiB(MTD3_LEGACY_SIZE);
	uint64_t extended_size = TO_MiB(MTD3_EXTENDED_SIZE) - master->erasesize;
	uint32_t flags;

	if (master->size != MTD3_EXTENDED_NAND_SIZE)
		return legacy_size;

	if (read_mtd3_flags(master, &flags) < 0)
		return legacy_size;

	if (flags & MTD3_FLAG_EXTENDED)
		return extended_size;
	else
		return legacy_size;
}

static ssize_t mtd3_ctl_write(struct file *f, const char __user *buf,
			      size_t size, loff_t *off)
{
	uint64_t old_size = TO_MiB(256);
	uint64_t extended_size =
		TO_MiB(MTD3_EXTENDED_SIZE) - master_mtd->erasesize;
	struct mtd_info *mtd3;
	char command;
	int rc;

	if (!master_mtd)
		return -ENODEV;
	if (size != 1)
		return -EINVAL;
	if (copy_from_user(&command, buf, 1))
		return -EFAULT;

	if (master_mtd->size != MTD3_EXTENDED_NAND_SIZE) {
		pr_err("extended mtd3 is supported only on 1GB flash\n");
		return -EIO;
	}

	switch (command) {
	case 'X': {
		// **** Extend mtd3 partition and write MTD3_EXTENDED_MAGIC ****
		mtd3 = get_mtd_device(NULL, 3);
		if (!mtd3)
			return -ENODEV;

		pr_info("Extending mtd3 size to 0x%llx\n",
			(unsigned long long)extended_size);
		mtd3->size = extended_size;

		rc = set_mtd3_flags(master_mtd, MTD3_FLAG_EXTENDED);
		put_mtd_device(mtd3);
		if (rc < 0)
			return rc;

		return size;
	}
	case 'S': {
		// **** Shrink mtd3 partition and clear MTD3_EXTENDED_MAGIC ****
		mtd3 = get_mtd_device(NULL, 3);
		if (!mtd3)
			return -ENODEV;

		pr_info("Shrinking mtd3 size to 0x%llx\n",
			(unsigned long long)old_size);
		mtd3->size = old_size;

		rc = clear_mtd3_flags(master_mtd, MTD3_FLAG_EXTENDED);
		put_mtd_device(mtd3);
		if (rc < 0)
			return rc;

		return size;
	}
	case 'E': {
		// **** Mark mtd3 partition as erase fixed ****
		// Note: erase fix involves ECC fix
		pr_info("Marking mtd3 partition as erase fixed\n");
		rc = set_mtd3_flags(master_mtd, MTD3_FLAG_ERASE_FIXED |
							MTD3_FLAG_ECC_FIXED);
		if (rc < 0)
			return rc;

		return size;
	}
	case 't': {
		// **** Clear erase fixed flag ****
		// Only for testing purposes!
		pr_warn("Clearing 'erase fixed' mtd3 flag!\n");
		rc = clear_mtd3_flags(master_mtd, MTD3_FLAG_ERASE_FIXED);
		if (rc < 0)
			return rc;

		return size;
	}
	default: {
		return -EINVAL;
	}
	}
}

static ssize_t mtd3_ctl_read(struct file *f, char __user *buf, size_t size,
			     loff_t *off)
{
	// Print mtd3 flags and total size of NAND flash
	char tmpbuf[64];
	long long nand_size = 0;
	uint32_t flags = 0;
	int rc;

	if (*off != 0)
		return 0;

	if (master_mtd) {
		nand_size = (long long)master_mtd->size;
		rc = read_mtd3_flags(master_mtd, &flags);
		if (rc < 0)
			return rc;
	}

	rc = snprintf(tmpbuf, sizeof(tmpbuf), "%x %llx\n", (unsigned int)flags,
		      nand_size);
	if (rc < 0)
		return -ENOBUFS;

	if (size < (size_t)rc) {
		// Out buffer is too small
		return -EINVAL;
	}

	if (copy_to_user(buf, tmpbuf, rc))
		return -EFAULT;

	*off = 1;
	return rc;
}

static struct proc_dir_entry *proc_file_entry;
static const struct file_operations npupart_mtd3_proc_file_fops = {
	.owner = THIS_MODULE,
	.write = mtd3_ctl_write,
	.read = mtd3_ctl_read,
};

void npu_parser_proc_init(void)
{
	proc_file_entry = proc_create("mtd3_ctl", 0, NULL, &npupart_mtd3_proc_file_fops);
	if (proc_file_entry == NULL)
		pr_err("Failed to create procfs entry");
}

void npu_parser_proc_remove(void)
{
	proc_remove(proc_file_entry);
}
