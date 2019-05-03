/*
 * Copyright (C) 2016 Broadcom Corporation
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

#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>


#define IPROC_IDM_COMPATIBLE "brcm,iproc-idm"

#define IDM_ERROR_LOG_ENABLE			0x33A
#define IDM_ERROR_LOG_CLEAR			0x3

#define IDM_RESET_CONTROL_REG(base)		(base + 0x800)
#define IDM_ERROR_LOG_CONTROL_REG(base) 	(base + 0x900)
#define IDM_ERROR_LOG_COMPLETE_REG(base)	(base + 0x904)
#define IDM_ERROR_LOG_STATUS_REG(base)		(base + 0x908)
#define IDM_ERROR_LOG_ADDR_LSB_REG(base)	(base + 0x90c)
#define IDM_ERROR_LOG_ID_REG(base)		(base + 0x914)
#define IDM_ERROR_LOG_FLAGS_REG(base)		(base + 0x91c)
#define IDM_INTERRUPT_STATUS_REG(base)		(base + 0xa00)

enum support_dev_ids {
	XGS_IPROC_HX4 = 0,
	XGS_IPROC_KT2,
	XGS_IPROC_HR2,
	XGS_IPROC_GH,
	XGS_IPROC_SB2,
	XGS_IPROC_HR3,
	XGS_IPROC_GH2,
	XGS_IPROC_WH2,
	XGS_IPROC_HX5,
	XGS_IPROC_MAX_DEVS,
};

static struct xgs_iproc_dev_infos_s {
	char dt_compat_str[32];
	u32 dev_id;
	u32 dmac_reset_offset;
} xgs_iproc_dev_infos[] = {
	{ "brcm,helix4", XGS_IPROC_HX4, 0x14800 },
	{ "brcm,katana2", XGS_IPROC_KT2, 0x14800 },
	{ "brcm,hurricane2", XGS_IPROC_HR2, 0x14800 },
	{ "brcm,greyhound", XGS_IPROC_GH, 0x14800 },
	{ "brcm,saber2", XGS_IPROC_SB2, 0xf800 },
	{ "brcm,hurricane3", XGS_IPROC_HR3, 0xf800 },
	{ "brcm,greyhound2", XGS_IPROC_GH2, 0xf800 },
	{ "brcm,wolfhound2", XGS_IPROC_WH2, 0xf800 },
	{ "brcm,helix5", XGS_IPROC_HX5, 0x10800 },
	{ "", 0, 0 }
};

#define IDM_BASE_ADDR_NUM		2
#define IDM_BASE_ADDR_MASK		0x10000000
#define IDM_BASE_ADDR_AREA(val) 	((val & IDM_BASE_ADDR_MASK) >> 28)
#define IDM_OFFSET_MASK 		0x0FFFFFFF

static struct xgs_iproc_idm_err_offset_s {
	u32 ihost_s1[XGS_IPROC_MAX_DEVS];
	u32 ihost_s0[XGS_IPROC_MAX_DEVS];
	u32 axi_pcie_sx[XGS_IPROC_MAX_DEVS];
	u32 ddr_s1[XGS_IPROC_MAX_DEVS];
	u32 ddr_s2[XGS_IPROC_MAX_DEVS];
	u32 cmic_s0[XGS_IPROC_MAX_DEVS];
	u32 apby_s0[XGS_IPROC_MAX_DEVS];
	u32 rom_s0[XGS_IPROC_MAX_DEVS];
	u32 nand_idm[XGS_IPROC_MAX_DEVS];
	u32 qspi_idm[XGS_IPROC_MAX_DEVS];
	u32 a9jtag_s0[XGS_IPROC_MAX_DEVS];
	u32 sram_s0[XGS_IPROC_MAX_DEVS];
	u32 apbz_s0[XGS_IPROC_MAX_DEVS];
	u32 axiic_ds_3[XGS_IPROC_MAX_DEVS];
	u32 apbw_idm[XGS_IPROC_MAX_DEVS];
	u32 apbx_idm[XGS_IPROC_MAX_DEVS];
	u32 axiic_ds_0[XGS_IPROC_MAX_DEVS];
	u32 periph_s0[XGS_IPROC_MAX_DEVS];
	u32 genres_s0[XGS_IPROC_MAX_DEVS];
} xgs_iproc_idm_err_offset = {
	/*       HX4,        KT2,        HR2,         GH,        SB2,        HR3,
	         GH2,        WH2,        HX5 */
	{ 0x00007000, 0x00007000, 0x00007000, 0x00006000, 0x00006000, 0x00006000,
	  0x00006000, 0x00006000,        0x0 },	/* IHOST_S1_IDM */
	{ 0x00008000, 0x00008000, 0x00008000, 0x00007000, 0x00007000, 0x00007000,
	  0x00007000, 0x00007000, 0x00014000 },	/* IHOST_S0_IDM */
	{ 0x0000b000, 0x0000c000, 0x0000b000, 0x00008000, 0x00008000, 0x00008000,
	  0x00008000, 0x00008000, 0x0000a000 },	/* AXI_PCIE_S0_IDM */ /* For KT2 AXI_PCIE_S1_IDM is used! */
	{ 0x00009000, 0x00009000, 0x00009000, 0x10002000, 0x10002000, 0x00004000,
	  0x10002000, 0x00004000, 0x0001a000 },	/* DDR_S1_IDM */
	{ 0x0000a000, 0x0000a000, 0x0000a000, 0x10003000, 0x10003000, 0x00005000,
	  0x10003000, 0x00005000, 0x0001b000 },	/* DDR_S2_IDM */
	{ 0x0000d000, 0x0000d000, 0x0000d000, 0x0000a000, 0x0000a000, 0x0000a000,
	  0x0000a000, 0x0000a000, 0x0000b000 },	/* CMICD_S0_IDM */
	{ 0x0000f000, 0x0000f000, 0x0000f000,        0x0,        0x0,        0x0,
	         0x0,        0x0, 0x0000c000 },	/* APBY_S0_IDM */
	{ 0x0001a000, 0x0001a000, 0x0001a000, 0x10004000, 0x10004000, 0x0001a000,
	  0x10004000, 0x0001a000,        0x0 },	/* ROM_S0_IDM */
	{ 0x0001b000, 0x0001b000, 0x0001b000, 0x10005000, 0x10005000, 0x0001d000,
	  0x10005000, 0x0001d000,        0x0 },	/* NAND_IDM */
	{ 0x0001c000, 0x0001c000, 0x0001c000, 0x10006000, 0x10006000, 0x0001f000,
	  0x10006000, 0x0001f000,        0x0 },	/* QSPI_IDM */
	{ 0x0001d000, 0x0001d000, 0x0001d000, 0x00019000, 0x00019000, 0x00019000,
	  0x00019000, 0x00019000,        0x0 },	/* A9JTAG_S0_IDM */
	{ 0x00020000, 0x00020000, 0x00020000, 0x0001b000, 0x0001b000,        0x0,
	  0x0001b000,        0x0,        0x0 },	/* SRAM_S0_IDM */
	{ 0x00021000, 0x00021000, 0x00021000,        0x0,        0x0,        0x0,
	         0x0,        0x0, 0x00017000 },	/* APBZ_S0_IDM */
	{ 0x00023000, 0x00023000, 0x00023000, 0x0001e000, 0x0001e000,        0x0,
	  0x0001e000,        0x0,        0x0 },	/* AXIIC_DS_3_IDM */
	{ 0x00031000, 0x00031000, 0x00031000,        0x0,        0x0,        0x0,
	         0x0,        0x0,        0x0 },	/* APBW_IDM */
	{ 0x00032000, 0x00032000, 0x00032000, 0x00030000, 0x00030000, 0x00030000,
	  0x00030000, 0x00030000,        0x0 },	/* APBX_IDM */
	{ 0x00041000, 0x00041000, 0x00041000, 0x00020000, 0x00020000, 0x00020000,
	  0x00020000, 0x00020000, 0x00053000 },	/* AXIIC_DS_0_IDM */
	{        0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
	         0x0,        0x0, 0x00018000 },	/* PERIPH_S0_IDM */
	{        0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
	         0x0,        0x0, 0x00019000 },	/* GENRES_S0_IDM */
};

struct xgs_iproc_idm {
	u32 curr_dev;
	void __iomem *idm_base[IDM_BASE_ADDR_NUM];
	u32 dmac_reset_offset;
};

static struct xgs_iproc_idm xgs_iproc_idm = { 0 };

/**********************************************************************************
***********************************************************************************/
void inline __iomem *get_iproc_idm_base(int idx)
{
	if (idx >= IDM_BASE_ADDR_NUM)
		return NULL;

	return xgs_iproc_idm.idm_base[idx];
}

int xgs_iproc_idm_dmac_reset(void)
{
	void __iomem *reset_base = NULL;

	if (xgs_iproc_idm.idm_base[0] == NULL ||
		xgs_iproc_idm.dmac_reset_offset == 0) {
		return -EINVAL;
	}

	reset_base = xgs_iproc_idm.idm_base[0] + xgs_iproc_idm.dmac_reset_offset;
	writel(readl(reset_base) & ~0x1, reset_base);

	return 0;
}

static int idm_error_log_dump(void __iomem *idm_addr)
{
	void __iomem *reg_addr;
	u32 val;

	reg_addr = (void __iomem *)IDM_ERROR_LOG_STATUS_REG(idm_addr);
	if ((val = readl(reg_addr)) > 0) {
		printk(KERN_DEBUG "%s: %d, %d\n", __func__, __LINE__, val);

		reg_addr = (void __iomem *)IDM_ERROR_LOG_ADDR_LSB_REG(idm_addr);
		val = readl(reg_addr);
		printk(KERN_DEBUG "%s: %d, %08x\n", __func__, __LINE__, val);

		reg_addr = (void __iomem *)IDM_ERROR_LOG_ID_REG(idm_addr);
		val = readl(reg_addr);
		printk(KERN_DEBUG "%s: %d, %08x\n", __func__, __LINE__, val);

		reg_addr = (void __iomem *)IDM_ERROR_LOG_FLAGS_REG(idm_addr);
		val = readl(reg_addr);
		printk(KERN_DEBUG "%s: %d, %08x\n", __func__, __LINE__, val);

		reg_addr = (void __iomem *)IDM_ERROR_LOG_COMPLETE_REG(idm_addr);
		writel(IDM_ERROR_LOG_CLEAR, reg_addr);

		reg_addr = (void __iomem *)IDM_ERROR_LOG_STATUS_REG(idm_addr);
		val = readl(reg_addr);
		printk(KERN_DEBUG "%s: %d, %d\n", __func__, __LINE__, val);
	}

	return 0;
}

static irqreturn_t idm_timeout_handler(int val, void *ptr)
{
	u32 idx, offset;
	void __iomem *idm_addr;
	u32 i, cnt, *idm_item;

	if (xgs_iproc_idm.curr_dev == XGS_IPROC_MAX_DEVS)
		return IRQ_HANDLED;

	cnt = sizeof(struct xgs_iproc_idm_err_offset_s) / (sizeof(u32) * XGS_IPROC_MAX_DEVS);
	for (i = 0; i < cnt; i++) {
		idm_item = (u32*)((u32*)(&xgs_iproc_idm_err_offset) + i * XGS_IPROC_MAX_DEVS);
		offset = idm_item[xgs_iproc_idm.curr_dev];
		if (offset != 0) {
			idx = IDM_BASE_ADDR_AREA(offset);
			if (xgs_iproc_idm.idm_base[idx]) {
				idm_addr = xgs_iproc_idm.idm_base[idx] + (offset & IDM_OFFSET_MASK);
				idm_error_log_dump(idm_addr);
			}
		}
	}

	return IRQ_HANDLED;
}

static int init_request_idm_timeout(void)
{
	void __iomem *idm_addr;
	void __iomem *reg_addr;
	u32 i, cnt, *idm_item;
	u32 idx, offset;

	if (xgs_iproc_idm.curr_dev == XGS_IPROC_MAX_DEVS)
		return IRQ_HANDLED;

	/* clear all pending idm interrupts */
	idm_timeout_handler(0, NULL);

	/* enable idm error log for all slaves */
	cnt = sizeof(struct xgs_iproc_idm_err_offset_s) / (sizeof(u32) * XGS_IPROC_MAX_DEVS);
	for (i = 0; i < cnt; i++) {
		idm_item = (u32*)((u32*)(&xgs_iproc_idm_err_offset) + i * XGS_IPROC_MAX_DEVS);
		offset = idm_item[xgs_iproc_idm.curr_dev];
		if (offset != 0) {
			idx = IDM_BASE_ADDR_AREA(offset);
			if (xgs_iproc_idm.idm_base[idx]) {
				idm_addr = xgs_iproc_idm.idm_base[idx] + (offset & IDM_OFFSET_MASK);
				reg_addr = (void __iomem *)IDM_ERROR_LOG_CONTROL_REG(idm_addr);
				writel(IDM_ERROR_LOG_ENABLE, reg_addr);
			}
		}
	}

	return 0;
}

#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
int xgs_iproc_idm_pcie_reset(void)
{
	void __iomem *idm_addr = xgs_iproc_idm.idm_base[0]
				+ (xgs_iproc_idm_err_offset.axi_pcie_sx[xgs_iproc_idm.curr_dev] & IDM_OFFSET_MASK);

	/* clear errors, resume timeout counter */
	idm_error_log_dump(idm_addr);

	writel(1, IDM_RESET_CONTROL_REG(idm_addr));
	/* Note that if there are any active transactions being handled by the
	 * reset at the time that a 0 is written to exit reset, the write will
	 * not succeed and the core will remain in soft reset.
	 * Consider using: udelay(500) in such case.
	 */
	writel(0, IDM_RESET_CONTROL_REG(idm_addr));

	return 0;
}
#endif /* CONFIG_ML66_NPU_IPROC_PLATFORM */

int xgs_iproc_idm_init(void)
{
	struct device_node *np;
	int idx, ret = 0;
	int irq, irqs_total;

	/* Get IDM base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_IDM_COMPATIBLE);
	if (!np) {
		pr_err("%s: No IDM node found\n", __func__);
		return -ENODEV;
	}

	xgs_iproc_idm.idm_base[0] = of_iomap(np, 0);
	if (!xgs_iproc_idm.idm_base[0]) {
		return -ENOMEM;
	}

	/*
	 * Second IDM base addr required for GH/SB2/GH2 IDM timeout handling.
	 * For other devices, the second IDM base addr is not used. So, it is
	 * fine even the addr is NULL.
	 */
	xgs_iproc_idm.idm_base[1] = of_iomap(np, 1);

	/* Get the current platform */
	idx = 0;
	while(1) {
		if (strlen(xgs_iproc_dev_infos[idx].dt_compat_str) == 0)
			return -EINVAL;
		if (of_machine_is_compatible(xgs_iproc_dev_infos[idx].dt_compat_str))
			break;
		idx++;
	}
	xgs_iproc_idm.curr_dev = xgs_iproc_dev_infos[idx].dev_id;
	xgs_iproc_idm.dmac_reset_offset = xgs_iproc_dev_infos[idx].dmac_reset_offset;

	/* Setup idm timeout handler for debug purpose */
	init_request_idm_timeout();

	irqs_total = of_irq_count(np);
	if (!irqs_total)
		return -EINVAL;

	for (idx = 0; idx < irqs_total; idx++) {
		irq = of_irq_get(np, idx);
		ret = request_irq(irq, (irq_handler_t)idm_timeout_handler,
				  IRQF_PERCPU, "IDM", NULL);
		if (ret != 0)
			printk(KERN_WARNING "%s: request_irq return = %d\n", __func__, ret);
	}

	return ret;
}
