// SPDX-License-Identifier: GPL-2.0
/*
 * Clock Controller Driver for Caninos Labrador
 *
 * Copyright (c) 2022-2023 ITEX - LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2018-2020 LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2014 Actions Semi Inc.
 * Author: David Liu <liuwei@actions-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include "clk-caninos.h"

static void __init
caninos_register_clkdev(struct caninos_clk_provider *ctx,
                        struct clk *clk, const char *name, int id)
{
	int ret;
	
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		pr_err("clkdev %s: uninitialized clock ret=%d.\n", name, ret);
		return;
	}
	if (id < 0 || id >= ctx->clk_data.clk_num) {
		pr_err("clkdev %s: invalid id=%d.\n", name, id);
		return;
	}
	if (!IS_ERR(ctx->clk_data.clks[id])) {
		pr_err("clkdev %s: lookup id=%d already in use.\n", name, id);
		return;
	}
	
	ctx->clk_data.clks[id] = clk;
	
	ret = clk_register_clkdev(clk, name, NULL);
	
	if (ret) {
		pr_err("clkdev %s: unable to register ret=%d.\n", name, ret);
		ctx->clk_data.clks[id] = ERR_PTR(-ENOENT);
	}
}

static void __init
caninos_clk_register_fixed(struct caninos_clk_provider *ctx,
                           const struct caninos_fixed_clock *clks, int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_fixed_clock *info = &clks[i];
		
		clk = clk_register_fixed_rate(NULL, info->name, info->parent_name,
		                              info->flags, info->fixed_rate);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

static void __init
caninos_clk_register_fixed_factor(struct caninos_clk_provider *ctx,
                                  const struct caninos_fixed_factor_clock *clks,
                                  int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_fixed_factor_clock *info = &clks[i];
		
		clk = clk_register_fixed_factor(NULL, info->name, info->parent_name,
		                                info->flags, info->mult, info->div);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

static void __init
caninos_clk_register_mux(struct caninos_clk_provider *ctx,
                         const struct caninos_mux_clock *clks, int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_mux_clock *info = &clks[i];
		
		clk = clk_register_mux(NULL, info->name, info->parent_names,
		                       info->num_parents, info->flags,
		                       ctx->reg_base + info->offset,
		                       info->shift, info->width,
		                       info->mux_flags, &ctx->lock);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

void __init 
caninos_clk_register_div(struct caninos_clk_provider *ctx,
                         const struct caninos_div_clock *clks, int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_div_clock *info = &clks[i];
		
		clk = caninos_register_divider_table(info, NULL, ctx->reg_base,
		                                     &ctx->lock);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

static void __init
caninos_clk_register_gate(struct caninos_clk_provider *ctx,
                          const struct caninos_gate_clock *clks, int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_gate_clock *info = &clks[i];
		
		clk = clk_register_gate(NULL, info->name,
		                        info->parent_name, info->flags,
		                        ctx->reg_base + info->offset, info->bit_idx,
		                        info->gate_flags, &ctx->lock);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

void __init
caninos_clk_register_pll(struct caninos_clk_provider *ctx,
                         const struct caninos_pll_clock *clks, int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_pll_clock *info = &clks[i];
		
		clk = caninos_register_pll(info, NULL, ctx->reg_base, &ctx->lock);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

static void __init
caninos_clk_register_composite(struct caninos_clk_provider *ctx,
                               const struct caninos_composite_clock *clks,
                               int num)
{
	struct clk *clk;
	int i;
	
	for (i = 0; i < num; i++)
	{
		const struct caninos_composite_clock *info = &clks[i];
		
		clk = caninos_register_composite(info, NULL, ctx->reg_base, &ctx->lock);
		
		caninos_register_clkdev(ctx, clk, info->name, info->id);
	}
}

struct caninos_clk_provider *__init
caninos_clk_init(struct device_node *np, unsigned long nr_clks)
{
	struct caninos_clk_provider *ctx = NULL;
	void __iomem *base = NULL;
	int ret, i;
	
	if (!np) {
		panic("%s: invalid device node.\n", __func__);
		return NULL;
	}
	
	base = of_iomap(np, 0);
	
	if (!base) {
		panic("%s: unable to map iomap.\n", __func__);
		return NULL;
	}
	
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	
	if (!ctx) {
		panic("%s: could not allocate clock provider context.\n", __func__);
		return NULL;
	}
	
	spin_lock_init(&ctx->lock);
	
	ctx->reg_base = base;
	ctx->clk_data.clk_num = nr_clks;
	ctx->clk_data.clks = kcalloc(nr_clks, sizeof(struct clk*), GFP_KERNEL);
	
	if (!ctx->clk_data.clks) {
		panic("%s: could not allocate clock lookup table.\n", __func__);
		kfree(ctx);
		return NULL;
	}
	
	for (i = 0; i < nr_clks; ++i) {
		ctx->clk_data.clks[i] = ERR_PTR(-ENOENT);
	}
	
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data);
	
	if (ret) {
		panic("%s: could not register clock provider.\n", __func__);
		kfree(ctx);
		ctx = NULL;
	}
	return ctx;
}

void __init
caninos_register_clk_tree(struct caninos_clk_provider *ctx,
                          const struct caninos_clock_tree *tree)
{
	caninos_clk_register_fixed(ctx, tree->fixed.clks, tree->fixed.num);
	caninos_clk_register_fixed_factor(ctx, tree->factor.clks, tree->factor.num);
	caninos_clk_register_pll(ctx, tree->pll.clks, tree->pll.num);
	caninos_clk_register_div(ctx, tree->div.clks, tree->div.num);
	caninos_clk_register_mux(ctx, tree->mux.clks, tree->mux.num);
	caninos_clk_register_gate(ctx, tree->gate.clks, tree->gate.num);
	caninos_clk_register_composite(ctx, tree->comp.clks, tree->comp.num);
}

