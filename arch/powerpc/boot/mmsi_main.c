// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) Paul Mackerras 1997.
 *
 * Updates for PPC64 by Todd Inglett, Dave Engebretsen & Peter Bergner.
 */
#include <stdarg.h>
#include <stddef.h>
#include "elf.h"
#include "page.h"
#include "string.h"
#include "stdio.h"
#include "ops.h"
#include "reg.h"

struct platform_ops platform_ops;
struct console_ops console_ops;

extern void mmsi_puts(const char *s);

static void serial_write(const char *buf, int len)
{
	mmsi_puts(buf);
}

int serial_console_init(void)
{
	console_ops.open = NULL;
	console_ops.write = serial_write;
	console_ops.close = NULL;
	console_ops.data = NULL;
	return 0;
}

void platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
		   unsigned long r6, unsigned long r7)
{
	u32 heapsize = 16*1024*1024 - (u32)_end;
	simple_alloc_init(_end, heapsize, 32, 64);

	serial_console_init();
}

static inline uint64_t readTime(void)
{
    uint32_t tbu, tbl, temp;

    __asm__ volatile(
        "mftbu %2\n"
        "mftb %0\n"
        "mftbu %1\n"
        "cmpw %2,%1\n"
        "bne $-0x10\n"
        : "=r"(tbl), "=r"(tbu), "=r"(temp)
        :
        : "cc");
    return (((uint64_t)tbu) << 32) | (uint64_t)tbl;
}

static uint32_t getUs(void)
{
    const uint32_t div = 38;
    const uint32_t c = (0x10000 + div - 1) / div;
    return (uint32_t)((readTime() * c) >> 16);
}

void start(void)
{
	kernel_entry_t kentry;
	unsigned char *vmlinuz_addr = (unsigned char *)_vmlinux_start;
	unsigned long vmlinuz_size = _vmlinux_end - _vmlinux_start;
	void *addr = 0;
	uint32_t time = getUs();

	long len = partial_decompress(vmlinuz_addr, vmlinuz_size,
				      addr, 16*1024*1024, 0);
	if (len < 0)
		fatal("Decompression failed with error code %ld\n", len);

	time = getUs() - time;
	printf("Linux kernel decompressed in %d ms at 0x%x\n",
	       time / 1000, (unsigned int)addr);

	kentry = (kernel_entry_t)addr;
	kentry(0, 0, NULL);

	fatal("Error: Linux kernel entry pointer failed\n");
}
