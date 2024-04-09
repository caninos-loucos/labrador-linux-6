// SPDX-License-Identifier: GPL-2.0
/*
 * Divider clock implementation for Caninos Labrador
 *
 * Copyright (c) 2022-2023 ITEX - LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2018-2020 LSI-TEC - Caninos Loucos
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

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include "clk-caninos.h"

struct clk *__init
caninos_register_divider_table(const struct caninos_div_clock *info,
                               struct device *dev, void __iomem *reg,
                               spinlock_t *lock)
{
	struct clk_div_table *table = NULL;
	unsigned int count = 0U, i;
	struct clk *clk;
	
	if (info->table)
	{
		/* calculate the number of elements in div_table */
		while (info->table[count].div != 0U) {
			count++;
		}
		if (count == 0U) {
			return ERR_PTR(-EINVAL);
		}
		
		/* allocate space for a copy of div_table */
		table = kcalloc(count + 1U, sizeof(*table), GFP_KERNEL);
		
		if (!table) {
			return ERR_PTR(-ENOMEM);
		}
		
		BUG_ON(table[count].div != 0U);
		
		/* copy div_table to allow __initconst*/
		for (i = 0; i < count; i++) {
			table[i] = info->table[i];
		}
	}
	
	clk = clk_register_divider_table(dev, info->name, info->parent_name,
	                                 info->flags, reg + info->offset,
	                                 info->shift, info->width, info->div_flags,
	                                 table, lock);
	
	if (IS_ERR(clk)) {
		kfree(table);
	}
	return clk;
}

