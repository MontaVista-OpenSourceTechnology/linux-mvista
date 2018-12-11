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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>

#include "clk-iproc.h"

#define SB2_GEN_PLL_CTRL_1_OFFSET	0x04
#define SB2_GEN_PLL_CTRL_3_OFFSET	0x0C
#define SB2_GEN_PLL_CTRL_5_OFFSET	0x14
#define SB2_GEN_PLL_CTRL_1_PDIV_R	27
#define SB2_GEN_PLL_CTRL_3_NDIV_INT_R	20
#define SB2_GEN_PLL_CTRL_5_CH1_MDIV_R	8
#define SB2_GEN_PLL_CTRL_1_PDIV_WIDTH	4
#define SB2_GEN_PLL_CTRL_3_NDIV_INT_WIDTH	10
#define SB2_GEN_PLL_CTRL_5_CH1_MDIV_WIDTH	8

#define GEN_PLL_CTRL1_OFFSET		0x4
#define GEN_PLL_CTRL2_OFFSET		0x8
#define GEN_PLL_CTRL1_NDIV_INT_R	0
#define GEN_PLL_CTRL1_NDIV_INT_WIDTH	10
#define GEN_PLL_CTRL1_PDIV_R		10
#define GEN_PLL_CTRL2_CH3_MDIV_R	8
#define GEN_PLL_CTRL2_CH3_MDIV_WIDTH 	8
#define GEN_PLL_CTRL1_PDIV_WIDTH_3	3
#define GEN_PLL_CTRL1_PDIV_WIDTH_4	4


struct iproc_gen_pll {
	struct clk_hw hw;
	void __iomem *base;
	unsigned long rate;
};

#define to_iproc_gen_pll(phw) container_of(phw, struct iproc_gen_pll, hw)

static u32 genpll_pdiv_width;

static unsigned long iproc_axi_clk_recalc_rate(struct clk_hw *hw,
			unsigned long parent_rate)
{
	uint32_t ndiv, mdiv, pdiv;
	struct iproc_gen_pll *pll = to_iproc_gen_pll(hw);

	ndiv = readl(pll->base + GEN_PLL_CTRL1_OFFSET) >>
			GEN_PLL_CTRL1_NDIV_INT_R;
	ndiv &= (1 << GEN_PLL_CTRL1_NDIV_INT_WIDTH) - 1;
	if (ndiv == 0)
		ndiv = 1 << GEN_PLL_CTRL1_NDIV_INT_WIDTH;

	pdiv = readl(pll->base + GEN_PLL_CTRL1_OFFSET) >> GEN_PLL_CTRL1_PDIV_R;
	pdiv &= (1 << genpll_pdiv_width) -1;
	if (pdiv == 0)
		pdiv = 1 << genpll_pdiv_width;

	mdiv = readl(pll->base + GEN_PLL_CTRL2_OFFSET) >>
		GEN_PLL_CTRL2_CH3_MDIV_R;
	mdiv &= (1 << GEN_PLL_CTRL2_CH3_MDIV_WIDTH) - 1;
	if (mdiv == 0)
		mdiv = 1 << GEN_PLL_CTRL2_CH3_MDIV_WIDTH;

	pll->rate = parent_rate * ndiv / pdiv / mdiv;
	return pll->rate;
}

static unsigned long iproc_sb2_axi_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	uint32_t ndiv, mdiv, pdiv;
	struct iproc_gen_pll *pll = to_iproc_gen_pll(hw);

	ndiv = readl(pll->base + SB2_GEN_PLL_CTRL_3_OFFSET) >>
			SB2_GEN_PLL_CTRL_3_NDIV_INT_R;
	ndiv &= (1 << SB2_GEN_PLL_CTRL_3_NDIV_INT_WIDTH) - 1;

	mdiv = readl(pll->base + SB2_GEN_PLL_CTRL_5_OFFSET) >>
			SB2_GEN_PLL_CTRL_5_CH1_MDIV_R;
	mdiv &= (1 << SB2_GEN_PLL_CTRL_5_CH1_MDIV_WIDTH) - 1;

	pdiv = readl(pll->base + SB2_GEN_PLL_CTRL_1_OFFSET) >>
			SB2_GEN_PLL_CTRL_1_PDIV_R;
	pdiv &= (1 << SB2_GEN_PLL_CTRL_1_PDIV_WIDTH) - 1;

	pll->rate = parent_rate * ndiv / pdiv / mdiv;
	return pll->rate;
}


static struct clk_ops iproc_axi_clk_ops = {
	.recalc_rate = iproc_axi_clk_recalc_rate,
};

void __init xgs_iproc_axi_clk_setup(struct device_node *node)
{
	int ret;
	struct clk *clk;
	struct iproc_gen_pll *pll;
	struct clk_init_data init;
	const char *parent_name;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (WARN_ON(!pll))
		return;

	pll->base = of_iomap(node, 0);
	if (WARN_ON(!pll->base))
		goto err_free_pll;

	init.name = node->name;
	if (of_device_is_compatible(node, "axi-clk-sb2"))
		iproc_axi_clk_ops.recalc_rate = iproc_sb2_axi_clk_recalc_rate;
	if (of_device_is_compatible(node, "axi-clk-hx4") ||
		of_device_is_compatible(node, "axi-clk-hr2"))
		genpll_pdiv_width = GEN_PLL_CTRL1_PDIV_WIDTH_3;
	else
		genpll_pdiv_width = GEN_PLL_CTRL1_PDIV_WIDTH_4;

	init.ops = &iproc_axi_clk_ops;
	init.flags = 0;
	parent_name = of_clk_get_parent_name(node, 0);
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (WARN_ON(IS_ERR(clk)))
		goto err_iounmap;

	ret = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (WARN_ON(ret))
		goto err_clk_unregister;

	return;

err_clk_unregister:
	clk_unregister(clk);
err_iounmap:
	iounmap(pll->base);
err_free_pll:
	kfree(pll);
}
CLK_OF_DECLARE(xgs_iproc_axi_clk, "brcm,xgs-iproc-axi-clk",
	xgs_iproc_axi_clk_setup);


static void __init xgs_iproc_armpll_init(struct device_node *node)
{
         iproc_armpll_setup(node);
}
CLK_OF_DECLARE(xgs_iproc_armpll, "brcm,xgs-iproc-armpll",
	xgs_iproc_armpll_init);