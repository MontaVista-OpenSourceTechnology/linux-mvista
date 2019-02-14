/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/bootmem.h>
#include <linux/module.h>

#include <linux/of.h>

#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
#define NUM_PARTS 12
#else
#define NUM_PARTS 4
#endif

#define TO_MiB(x) ((x) << 20)
#define BOOT_INFO_IMG_ACT (0x01 << 3)
#define KERN_MAGIC	0x00000001
#define DTB_MAGIC	0x00000002
#define FPGA_MAGIC	0x00000003
#define SKIP_MAGIC	0x00000004
#define SCRIPT_MAGIC	0x00000005
#define UBI_MAGIC	0x5542

#ifdef CONFIG_NPU_PARTS_MTD3_FLAGS
extern struct mtd_info* master_mtd;
uint64_t npupart_get_mtd3_size(struct mtd_info *);
void npu_parser_proc_init(void);
void npu_parser_proc_remove(void);
#endif

struct lm_header {
	u32 magic;
	u32 size;
	u32 crc32;
} __packed;

static int disable;
module_param(disable, int, 0444);
MODULE_PARM_DESC(disable, "Disable use of NPUPART - any non-zero value\n");

#ifdef CONFIG_NPU_PARTS_UBOOT_OK
static u8 get_bootinfo(void)
{
	u32 boot_info_prop;
	struct device_node *node;
	struct property *prop;
	int len = -1;

	node = of_find_node_by_path("/private");
	if (node == NULL) {
		pr_emerg(
			"Failed to fetch private properties from device tree!");
		return 0;
	}

	prop = of_find_property(node, "boot_info", &len);
	if (prop == NULL || len < 0 || len > 4) {
		pr_emerg(
			"Failed to fetch boot_info property from device tree!");
		return 0;
	}

	memcpy(&boot_info_prop, prop->value, len);

	of_node_put(node);

	return (u8)boot_info_prop & 0xFF;
}
#endif

static int get_rootfs_offset(struct mtd_info *master, u32 active_offset)
{
	struct lm_header *lmhdr;

	size_t len;
	u32 fs_offset = 0;
	bool find_nps_header = true;
	unsigned int b_cnt = 0;

	lmhdr = kmalloc(sizeof(*lmhdr), GFP_KERNEL);

	if (!lmhdr)
		return -ENOMEM;

	while (1) {
		if (mtd_block_isbad(master, active_offset + fs_offset)) {
			pr_emerg("Bad block found at 0x%08x\n",
				 active_offset + fs_offset);
			if (find_nps_header)
				active_offset += master->erasesize;
			else
				fs_offset += master->erasesize;
			continue;
		} else {
			if (find_nps_header) {
				pr_emerg("NPS header found at 0x%08x\n",
					 active_offset + fs_offset);
				fs_offset +=
					master->writesize; // first read from pagesize
				find_nps_header = false;
				continue;
			}
			if (b_cnt--) {
				fs_offset =
					(fs_offset & ~(master->erasesize - 1)) +
					master->erasesize;
				continue;
			} else {
				mtd_read(master, active_offset + fs_offset,
					 sizeof(*lmhdr), &len,
					 (uint8_t *)lmhdr);
			}
		}

		switch (lmhdr->magic) {
		case KERN_MAGIC:
		case DTB_MAGIC:
		case FPGA_MAGIC:
		case SKIP_MAGIC:
		case SCRIPT_MAGIC:
			b_cnt = ((lmhdr->size + sizeof(*lmhdr) +
				  master->erasesize - 1) &
				 ~(master->erasesize - 1)) /
				master->erasesize;

			pr_info("MAGIC(0x%08x) found at 0x%08x, %d eraseblock read is needed (len=0x%08x)\n",
				lmhdr->magic, active_offset + fs_offset, b_cnt,
				lmhdr->size);
			continue;

		default:

			if ((lmhdr->magic >> 16) == UBI_MAGIC) {
				pr_info("Root FS offset is 0x%08x bytes\n",
					active_offset + fs_offset);
				kfree(lmhdr);
				return fs_offset;
			} else {
				pr_emerg(
					"ROOTFS MAGIC (%x) is not found at offset: 0x%08x\n",
					UBI_MAGIC, active_offset + fs_offset);
				kfree(lmhdr);
				return -EINVAL;
			}
		}
	}
}

#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
static int create_mtd_partitions(struct mtd_info *master,
				 const struct mtd_partition **pparts,
				 struct mtd_part_parser_data *data)
{
	struct mtd_partition *npu_parts;
	u32 active_offset;
	u32 passive_offset;
	int rootfs_offset;
	u8 boot_info;

	npu_parts = kzalloc(sizeof(*npu_parts) * NUM_PARTS, GFP_KERNEL);
	if (!npu_parts)
		return -ENOMEM;

#ifdef CONFIG_NPU_PARTS_UBOOT_OK
	boot_info = get_bootinfo();
#else
	pr_emerg("FIXME: Trying to boot from bank0, hardcoded in %s!",
		 __FILE__);
	boot_info = 0;
#endif

	if (boot_info & BOOT_INFO_IMG_ACT) {
		active_offset = TO_MiB(320);
		passive_offset = TO_MiB(64);
	} else {
		active_offset = TO_MiB(64);
		passive_offset = TO_MiB(320);
	}

	/*TODO use rootfs offset passed from uboot as fallback*/
	rootfs_offset = get_rootfs_offset(master, active_offset);
	if (rootfs_offset < 0) {
		pr_emerg(
			"Giving up on npupart, falling back to hardcoded mapping in device tree!\n");
		kfree(npu_parts);
		return rootfs_offset;
	}

	npu_parts[4].name = "Boot (R/O)";
	npu_parts[4].offset = TO_MiB(0);
	npu_parts[4].size = TO_MiB(1);
	npu_parts[4].mask_flags = MTD_WRITEABLE;

	npu_parts[5].name = "MemTest (R/O)";
	npu_parts[5].offset = npu_parts[4].offset + npu_parts[4].size;
	npu_parts[5].size = TO_MiB(1);
	npu_parts[5].mask_flags = MTD_WRITEABLE;

	npu_parts[6].name = "BootEnv (R/O)";
	npu_parts[6].offset = npu_parts[5].offset + npu_parts[5].size;
	npu_parts[6].size = TO_MiB(1);
	npu_parts[6].mask_flags = MTD_WRITEABLE;

	npu_parts[8].name = "unused (R/W)";
	npu_parts[8].offset = npu_parts[6].offset + npu_parts[6].size;
	npu_parts[8].size = TO_MiB(1);
	npu_parts[8].mask_flags = 0;

	npu_parts[9].name = "IPROC RAM tuning(R/O)";
	npu_parts[9].offset = npu_parts[8].offset + npu_parts[8].size;
	npu_parts[9].size = TO_MiB(1);
	npu_parts[9].mask_flags = MTD_WRITEABLE;

	npu_parts[10].name = "NANDTest (R/O)";
	npu_parts[10].offset = npu_parts[9].offset + npu_parts[9].size;
	npu_parts[10].size = TO_MiB(1);
	npu_parts[10].mask_flags = MTD_WRITEABLE;

	npu_parts[11].name = "PBIST (R/O)";
	npu_parts[11].offset = npu_parts[10].offset + npu_parts[10].size;
	npu_parts[11].size = TO_MiB(58);
	npu_parts[11].mask_flags = MTD_WRITEABLE;

	npu_parts[0].name = "Header/Kernel (R/O)";
	npu_parts[0].offset = active_offset;
	npu_parts[0].size = rootfs_offset;
	npu_parts[0].mask_flags = MTD_WRITEABLE;

	npu_parts[1].name = "Root Disk (R/O)";
	npu_parts[1].offset = npu_parts[0].offset + npu_parts[0].size;
	npu_parts[1].size = TO_MiB(256) - npu_parts[0].size;
	npu_parts[1].mask_flags = MTD_WRITEABLE;

	npu_parts[2].name = "Passive Bank (R/W)";
	npu_parts[2].offset = passive_offset;
	npu_parts[2].size = TO_MiB(256);
	npu_parts[2].mask_flags = 0;

	npu_parts[3].name = "Util (R/W)";
	npu_parts[3].offset = TO_MiB(576);
	npu_parts[3].size = TO_MiB(1408);
	npu_parts[3].mask_flags = 0;

	npu_parts[7].name = "Crash (R/W)";
	npu_parts[7].offset = npu_parts[3].offset + npu_parts[3].size;
	npu_parts[7].size = TO_MiB(64);
	npu_parts[7].mask_flags = 0;

	*pparts = npu_parts;
	return NUM_PARTS;
}
#elif CONFIG_PPC_85xx
static int create_mtd_partitions(struct mtd_info *master,
				 const struct mtd_partition **pparts,
				 struct mtd_part_parser_data *data)
{
	struct mtd_partition *npu_parts;
	u32 active_offset;
	u32 passive_offset;
	int rootfs_offset;
	u8 boot_info;

#ifdef CONFIG_NPU_PARTS_MTD3_FLAGS
	master_mtd = master;
#endif

	npu_parts = kzalloc(sizeof(*npu_parts) * NUM_PARTS, GFP_KERNEL);
	if (!npu_parts)
		return -ENOMEM;

	boot_info = get_bootinfo();
	if (boot_info & BOOT_INFO_IMG_ACT) {
		active_offset = TO_MiB(128);
		passive_offset = 0;
	} else {
		active_offset = 0;
		passive_offset = TO_MiB(128);
	}

	/*TODO use rootfs offset passed from uboot as fallback*/
	rootfs_offset = get_rootfs_offset(master, active_offset);
	if (rootfs_offset < 0) {
		pr_emerg(
			"Giving up on npupart, falling back to hardcoded mapping in device tree!\n");
		kfree(npu_parts);
		return rootfs_offset;
	}

	npu_parts[0].name = "Header/Kernel (R/O)";
	npu_parts[0].offset = active_offset;
	npu_parts[0].size = rootfs_offset;
	npu_parts[0].mask_flags = MTD_WRITEABLE;

	npu_parts[1].name = "Root Disk (R/O)";
	npu_parts[1].offset = active_offset + npu_parts[0].size;
	npu_parts[1].size = TO_MiB(128) - rootfs_offset;
	npu_parts[1].mask_flags = MTD_WRITEABLE;

	npu_parts[2].name = "Passive Bank (R/W)";
	npu_parts[2].offset = passive_offset;
	npu_parts[2].size = TO_MiB(128);
	npu_parts[2].mask_flags = 0;

	npu_parts[3].name = "Util (R/W)";
	npu_parts[3].offset = npu_parts[0].size + npu_parts[1].size + npu_parts[2].size;
#ifdef CONFIG_NPU_PARTS_MTD3_FLAGS
	npu_parts[3].size = npupart_get_mtd3_size(master);
#else
	npu_parts[3].size = TO_MiB(256);
#endif
	npu_parts[3].mask_flags = 0;

	*pparts = npu_parts;
	return NUM_PARTS;
}
#else
static int create_mtd_partitions(struct mtd_info *master,
				 const struct mtd_partition **pparts,
				 struct mtd_part_parser_data *data)
{
	return -ENOTSUPP;
}
#endif	/* ifdef CONFIG_ML66_NPU_IPROC_PLATFORM */

static const struct of_device_id npupart_of_match_table[] = {
	{ .compatible = "ml66,npu-partitions" },
	{},
};
MODULE_DEVICE_TABLE(of, npupart_of_match_table);

static struct mtd_part_parser npu_parser = {
	.parse_fn = create_mtd_partitions,
	.name = "npupart",
	.of_match_table = npupart_of_match_table,
};

static int __init npu_parser_init(void)
{
	if (disable) {
		pr_info("NPUPART is disabled, will not be registered!\n");
		return 0;
	}

#ifdef CONFIG_NPU_PARTS_MTD3_FLAGS
	npu_parser_proc_init();
#endif

	return register_mtd_parser(&npu_parser);
}

static void __exit npu_parser_exit(void)
{
#ifdef CONFIG_NPU_PARTS_MTD3_FLAGS
	npu_parser_proc_remove();
#endif
	deregister_mtd_parser(&npu_parser);
}

module_init(npu_parser_init);
module_exit(npu_parser_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("norbert.a.nemeth@ericsson.com");
MODULE_DESCRIPTION("MTD partitioning for Ericsson ML66 NPUs");
