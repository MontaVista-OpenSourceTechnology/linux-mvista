/*
 * RapidIO generic memory map support
 *
 * Copyright 2016 Nokia
 * Radu Rendec <radu.rendec.ext@nokia.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/rio.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/crc32.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <asm/byteorder.h>

#ifdef __BIG_ENDIAN
#define ARCH_BE_FLAG RIO_MEM_MAP_FLAGS_BIG_ENDIAN
#else
#define ARCH_BE_FLAG 0
#endif

#ifdef CONFIG_CAVIUM_OCTEON2
#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-bootmem.h>
#endif

/* map descriptor count (number of used descriptors) */
static inline u32 map_count(struct rio_mem_map *map)
{
	return __be32_to_cpu(map->block->count);
}

/* map block usage (used size) */
static inline size_t map_bu(struct rio_mem_map *map)
{
	return sizeof(struct rio_mem_map_block) +
		map_count(map) * RIO_MEM_MAP_DESC_SIZE;
}

static inline void map_update_crc(struct rio_mem_map *map)
{
	map->block->crc = 0;
	map->block->crc = __cpu_to_be32(~crc32(~0, map->block, map_bu(map)));
}

static int __map_descriptor_alloc(struct rio_mem_map *map, u16 type,
		u64 addr, u32 size, u16 flags)
{
	struct rio_mem_map_descriptor *desc;
	u32 count = map_count(map);

	desc = &map->block->desc[count];
	map->block->count = __cpu_to_be32(count + 1);
	desc->flags = __cpu_to_be16(flags);
	desc->type = __cpu_to_be16(type);
	desc->dio_addr_h = __cpu_to_be32((u32)(addr >> 32));
	desc->dio_addr_l = __cpu_to_be32((u32)addr);
	desc->dio_size = __cpu_to_be32(size);

	return count;
}

static int __attribute__((__unused__)) map_descriptor_alloc_virt(
		struct rio_mem_map *map, u16 type,
		void *addr, size_t size, u16 flags)
{
	u64 rio_addr = RIO_MEM_MAP_MAPPING_FAILED;

	if (map_bu(map) + RIO_MEM_MAP_DESC_SIZE > map->size)
		return -ENOMEM;

	if (map->ops->map_virt)
		rio_addr = map->ops->map_virt(map, addr, size);

	if (rio_addr == RIO_MEM_MAP_MAPPING_FAILED)
		return -EFAULT;

	return __map_descriptor_alloc(map, type, rio_addr, size, flags);
}

static int __attribute__((__unused__)) map_descriptor_alloc_phys(
		struct rio_mem_map *map, u16 type,
		phys_addr_t addr, size_t size, u16 flags)
{
	u64 rio_addr = RIO_MEM_MAP_MAPPING_FAILED;

	if (map_bu(map) + RIO_MEM_MAP_DESC_SIZE > map->size)
		return -ENOMEM;

	if (map->ops->map_phys)
		rio_addr = map->ops->map_phys(map, addr, size);

	if (rio_addr == RIO_MEM_MAP_MAPPING_FAILED)
		return -EFAULT;

	return __map_descriptor_alloc(map, type, rio_addr, size, flags);
}

static int __attribute__((__unused__)) map_descriptor_alloc_named_phys(
		struct rio_mem_map *map,
		phys_addr_t addr, size_t size, u16 flags)
{
	u32 count, offs;

	if (map_bu(map) + RIO_MEM_MAP_NAMED_DESC_SIZE > map->size)
		return -ENOMEM;

	offs = __map_descriptor_alloc(map, RIO_MEM_MAP_TYPE_NAMED,
			addr, size, flags);

	count = map_count(map) + RIO_MEM_MAP_NAME_CNT;
	map->block->count = __cpu_to_be32(count);
	map->named_count++;

	return offs;
}

static int map_descriptor_find_named(struct rio_mem_map *map,
		char name[RIO_MEM_MAP_NAME_LEN])
{
	struct rio_mem_map_descriptor *desc = map->block->desc;
	u32 count = map_count(map);

	int i = count - map->named_count * (1 + RIO_MEM_MAP_NAME_CNT);
	while (i < count) {
		if (!strncmp(name, (char *)&desc[i + 1], RIO_MEM_MAP_NAME_LEN))
			return i;
		i += 1 + RIO_MEM_MAP_NAME_CNT;
	}

	return -1;
}

#ifdef CONFIG_OF
static void __attribute__((__unused__)) map_descriptor_add_of(
		struct rio_mem_map *map, u16 type,
		const char *dt_path, u16 flags)
{
	struct device_node *node;
	int n_addr_cells;
	int n_size_cells;
	const __be32 *reg_addr;
	int reg_len;
	u64 addr, size;

	node = of_find_node_by_path(dt_path);
	if (!node)
		return;

	reg_addr = of_get_property(node, "reg", &reg_len);
	if (!reg_addr)
		goto out_clean;
	reg_len /= 4;

	n_addr_cells = of_n_addr_cells(node);
	n_size_cells = of_n_size_cells(node);
	if (n_size_cells > 1)
		pr_warning("riomem: size of %d cells will be truncated for %s\n",
				n_size_cells, node->full_name);

	while (reg_len > n_addr_cells) {
		addr = of_read_number(reg_addr, n_addr_cells);
		reg_addr += n_addr_cells;
		reg_len -= n_addr_cells;

		if (n_size_cells > reg_len) {
			pr_warning("riomem: size cells workaround for %s\n",
					node->full_name);
			n_size_cells = reg_len;
		}
		size = of_read_number(reg_addr, n_size_cells);
		reg_addr += n_size_cells;
		reg_len -= n_size_cells;

		map_descriptor_alloc_phys(map, type, addr, size, flags);
	}

out_clean:
	of_node_put(node);
}
#endif

struct rio_mem_map *rio_mem_map_alloc(const struct rio_mem_map_ops *ops)
{
	struct rio_mem_map *map;

	if (!ops)
		return NULL;

	map = kzalloc(sizeof(struct rio_mem_map), GFP_KERNEL);
	if (!map)
		return NULL;

	if (ops->alloc_block(map)) {
		kfree(map);
		return NULL;
	}

	map->ops = ops;
	mutex_init(&map->lock);

	map->block->magic = __cpu_to_be32(RIO_MEM_MAP_BLOCK_MAGIC);
	map->block->version = __cpu_to_be32(RIO_MEM_MAP_BLOCK_VER);

	return map;
}

int rio_mem_map_init(struct rio_mem_map *map)
{

#ifdef CONFIG_OF_FLATTREE
	map_descriptor_alloc_virt(map, RIO_MEM_MAP_TYPE_DEVTREE,
			initial_boot_params, fdt_totalsize(initial_boot_params),
			RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);
#endif

	map_descriptor_alloc_virt(map, RIO_MEM_MAP_TYPE_KERNELLOG,
			log_buf_addr_get(), log_buf_len_get(),
			RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);

#ifndef CONFIG_CAVIUM_OCTEON2
#ifdef CONFIG_OF
	map_descriptor_add_of(map, RIO_MEM_MAP_TYPE_BOOTLOG,
			"/uboot/__uboot_log",
			RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);

	map_descriptor_add_of(map, RIO_MEM_MAP_TYPE_NVRAMDISK,
			"/uboot/__reset_safe_rd",
			RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);
#endif
#else
	{
		const struct cvmx_bootmem_named_block_desc *block_desc;
		block_desc = cvmx_bootmem_find_named_block("__uboot_log");
		if (block_desc) {
			map_descriptor_alloc_virt(map, RIO_MEM_MAP_TYPE_BOOTLOG,
					(void*)block_desc->base_addr, block_desc->size,
					RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);
		}
		block_desc = cvmx_bootmem_find_named_block("__reset_safe_rd");
		if(block_desc) {
			map_descriptor_alloc_virt(map, RIO_MEM_MAP_TYPE_NVRAMDISK,
					(void*)block_desc->base_addr, block_desc->size,
					RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);
		}
	}
#endif
	if (map->region)
		map_descriptor_alloc_virt(map, RIO_MEM_MAP_TYPE_SYNC,
				map->region[RIO_MEM_MAP_REGION_SYNC].get_addr(),
				map->region[RIO_MEM_MAP_REGION_SYNC].get_size(),
				RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);

	map_update_crc(map);

	return 0;
}

void rio_mem_map_free(struct rio_mem_map *map)
{
	if (map->ops && map->block)
		map->ops->free_block(map);
	kfree(map);
}

int rio_mem_map_op_alloc_block(struct rio_mem_map *map)
{
	if (map->block)
		return -EINVAL;

	map->block = (void *)get_zeroed_page(GFP_KERNEL);
	if (!map->block)
		return -ENOMEM;

	map->size = PAGE_SIZE;
	return 0;
}

void rio_mem_map_op_free_block(struct rio_mem_map *map)
{
	if (!map->block)
		return;

	free_page((unsigned long)map->block);
}

int rio_mem_map_add_named_region(struct rio_mem_map *map,
		char name[RIO_MEM_MAP_NAME_LEN], u64 phys_addr, u32 len)
{
	struct rio_mem_map_descriptor *desc = map->block->desc;
	int idx = map_descriptor_find_named(map, name);

	if (idx >= 0) {
		desc[idx].dio_addr_h = __cpu_to_be32((u32)(phys_addr >> 32));
		desc[idx].dio_addr_l = __cpu_to_be32((u32)(phys_addr));
		desc[idx].dio_size = __cpu_to_be32(len);
	} else {
		idx = map_descriptor_alloc_named_phys(map, phys_addr, len,
				RIO_MEM_MAP_FLAGS_READ | ARCH_BE_FLAG);
		if (idx < 0) {
			return idx;
		}
		memcpy(&desc[idx + 1], name, RIO_MEM_MAP_NAME_LEN);
	}
	map_update_crc(map);

	return 0;
}

int rio_mem_map_del_named_region(struct rio_mem_map *map,
		char name[RIO_MEM_MAP_NAME_LEN])
{
	u32 count = map_count(map);
	int idx, last_idx;

	idx = map_descriptor_find_named(map, name);
	if (idx < 0)
		return -ENODEV;

	last_idx = count - 1 - RIO_MEM_MAP_NAME_CNT;
	if (last_idx != idx)
		memmove(&map->block->desc[idx], &map->block->desc[last_idx],
				RIO_MEM_MAP_NAMED_DESC_SIZE);

	memset(&map->block->desc[last_idx], 0, RIO_MEM_MAP_NAMED_DESC_SIZE);
	map->block->count = __cpu_to_be32(last_idx);
	map->named_count--;
	map_update_crc(map);

	return 0;
}

void rio_mem_map_del_named_all(struct rio_mem_map *map)
{
	u32 count = map_count(map);

	if (!map->named_count)
		return;

	count = count - map->named_count * (1 + RIO_MEM_MAP_NAME_CNT);
	memset(&map->block->desc[count], 0,
			map->named_count * RIO_MEM_MAP_NAMED_DESC_SIZE);
	map->block->count = __cpu_to_be32(count);
	map->named_count = 0;
	map_update_crc(map);
}

EXPORT_SYMBOL_GPL(rio_mem_map_alloc);
EXPORT_SYMBOL_GPL(rio_mem_map_init);
EXPORT_SYMBOL_GPL(rio_mem_map_free);
EXPORT_SYMBOL_GPL(rio_mem_map_op_alloc_block);
EXPORT_SYMBOL_GPL(rio_mem_map_op_free_block);
EXPORT_SYMBOL_GPL(rio_mem_map_add_named_region);
EXPORT_SYMBOL_GPL(rio_mem_map_del_named_region);
EXPORT_SYMBOL_GPL(rio_mem_map_del_named_all);
