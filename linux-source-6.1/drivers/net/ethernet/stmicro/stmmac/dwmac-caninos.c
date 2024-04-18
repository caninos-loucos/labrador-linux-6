// SPDX-License-Identifier: GPL-2.0
/*
 * Caninos Labrador DWMAC specific glue layer
 * Copyright (c) 2019-2024 LSI-TEC - Caninos Loucos
 * Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 * Igor Ruschi Andrade E Lima <igor.lima@lsitec.org.br>
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

#include <linux/stmmac.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/of_net.h>
#include <linux/of_gpio.h>

#include "stmmac_platform.h"

struct caninos_priv_data {
	struct plat_stmmacenet_data *plat_dat;
	void __iomem *addr;
	int power_gpio;
};

#define CANINOS_RGMII_RATE 125000000
#define CANINOS_RMII_RATE 50000000

static int caninos_gmac_init(struct platform_device *pdev, void *priv)
{
	struct caninos_priv_data *gmac = priv;
	
	if (gpio_is_valid(gmac->power_gpio)) { /* power on the phy */
		gpio_set_value_cansleep(gmac->power_gpio, 1);
		msleep(500);
	}
	return 0;
}

static void caninos_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct caninos_priv_data *gmac = priv;
	
	if (gpio_is_valid(gmac->power_gpio)) { /* power off the phy */
		gpio_set_value_cansleep(gmac->power_gpio, 0);
		msleep(50);
	}
}

static int caninos_gmac_clks_config(void *priv, bool enabled)
{
	struct caninos_priv_data *gmac = priv;
	struct plat_stmmacenet_data *plat_dat = gmac->plat_dat;
	
	if (!enabled) {
		return 0;
	}
	if (phy_interface_mode_is_rgmii(plat_dat->phy_interface)) {
		clk_set_rate(plat_dat->pclk, CANINOS_RGMII_RATE);
		writel(0x1, gmac->addr);
	}
	else {
		clk_set_rate(plat_dat->pclk, CANINOS_RMII_RATE);
		writel(0x4, gmac->addr);
	}
	return 0;
}

static int caninos_gmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct device *dev = &pdev->dev;
	struct caninos_priv_data *gmac;
	int ret;
	
	gmac = devm_kzalloc(dev, sizeof(*gmac), GFP_KERNEL);
	
	if (!gmac) {
		return -ENOMEM;
	}
	
	gmac->addr = devm_ioremap(dev, 0xe024c0a0, 4);
	
	if (!gmac->addr) {
		dev_err(dev, "unable to map interface config reg\n");
		return -ENOMEM;
	}
	
	gmac->power_gpio = of_get_named_gpio(dev->of_node, "phy-power-gpio", 0);
	
	if (gpio_is_valid(gmac->power_gpio))
	{
		ret = devm_gpio_request(dev, gmac->power_gpio, "phy_power");
		
		if (ret) {
			dev_err(dev, "unable to request power gpio\n");
			return ret;
		}
		gpio_direction_output(gmac->power_gpio, 0);
		msleep(50);
	}
	
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	
	if (ret) {
		return ret;
	}
	
	plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	
	if (IS_ERR(plat_dat)) {
		return PTR_ERR(plat_dat);
	}
	
	gmac->plat_dat = plat_dat;
	plat_dat->clk_csr = 0x4;
	plat_dat->host_dma_width = 32;
	plat_dat->bsp_priv = gmac;
	plat_dat->has_gmac = 1;
	plat_dat->pmt = 1;
	plat_dat->tx_coe = 0;
	plat_dat->maxmtu = 1500;
	plat_dat->init = caninos_gmac_init;
	plat_dat->exit = caninos_gmac_exit;
	plat_dat->clks_config = caninos_gmac_clks_config;
	
	ret = caninos_gmac_clks_config(plat_dat->bsp_priv, true);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, plat_dat);
		return ret;
	}
	
	ret = caninos_gmac_init(pdev, plat_dat->bsp_priv);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, plat_dat);
		return ret;
	}
	
	ret = stmmac_dvr_probe(dev, plat_dat, &stmmac_res);
	
	if (ret) {
		caninos_gmac_exit(pdev, plat_dat->bsp_priv);
		stmmac_remove_config_dt(pdev, plat_dat);
		return ret;
	}
	return 0;
}

static const struct of_device_id caninos_dwmac_match[] = {
	{.compatible = "caninos,k7-gmac" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, caninos_dwmac_match);

static struct platform_driver caninos_dwmac_driver = {
	.probe  = caninos_gmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name = "caninos-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = caninos_dwmac_match,
	},
};

module_platform_driver(caninos_dwmac_driver);

MODULE_AUTHOR("LSI-TEC - Caninos Loucos");
MODULE_DESCRIPTION("Caninos Labrador DWMAC specific glue layer");
MODULE_LICENSE("GPL v2");
