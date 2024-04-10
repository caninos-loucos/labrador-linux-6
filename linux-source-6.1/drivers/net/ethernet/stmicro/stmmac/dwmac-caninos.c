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
	struct device *dev;
	struct clk *ref_clk;
	int power_gpio;
	int reset_gpio;
	unsigned int speed;
	struct pinctrl *pctl;
	struct pinctrl_state *rmii_state;
	struct pinctrl_state *rgmii_state;
};

static int caninos_gmac_get_gpios(struct caninos_priv_data *gmac)
{
	struct device *dev = gmac->dev;
	int ret;
	
	gmac->reset_gpio = of_get_named_gpio(dev->of_node, "phy-reset-gpio", 0);
	
	if (!gpio_is_valid(gmac->reset_gpio)) {
		dev_err(dev, "unable to get reset gpio\n");
		return -ENODEV;
	}
	
	ret = devm_gpio_request(dev, gmac->reset_gpio, "phy_reset");
	
	if (ret) {
		dev_err(dev, "unable to request reset gpio\n");
		return ret;
	}
	
	gmac->power_gpio = of_get_named_gpio(dev->of_node, "phy-power-gpio", 0);
	
	if (!gpio_is_valid(gmac->power_gpio)) {
		dev_info(dev, "not using power gpio");
		return 0;
	}
	
	ret = devm_gpio_request(dev, gmac->power_gpio, "phy_power");
	
	if (ret) {
		dev_err(dev, "unable to request power gpio\n");
		return ret;
	}
	
	return 0;
}

static int caninos_gmac_interface_config(void *priv, unsigned int speed)
{
	struct caninos_priv_data *gmac = priv;
	struct device *dev = gmac->dev;
	void __iomem *addr;
	
	if (gmac->speed == speed) {
		return 0;
	}
	
	addr = ioremap(0xe024c0a0, 4);
	
	if (!addr) {
		dev_err(dev, "unable to map interface config reg\n");
		return -ENOMEM;
	}
	
	if (gmac->speed) {
		clk_disable_unprepare(gmac->ref_clk);
	}
	
	if (speed == 1000U) {
		clk_set_rate(gmac->ref_clk, 125000000);
		clk_prepare_enable(gmac->ref_clk);
		pinctrl_select_state(gmac->pctl, gmac->rgmii_state);
		writel(0x1, addr);
	}
	else {
		clk_set_rate(gmac->ref_clk, 50000000);
		clk_prepare_enable(gmac->ref_clk);
		pinctrl_select_state(gmac->pctl, gmac->rmii_state);
		writel(0x4, addr);
	}
	
	gmac->speed = speed;
	iounmap(addr);
	
	if (speed == 1000U) {
		dev_info(dev, "phy interface mode set to rgmii\n");
	}
	else {
		dev_info(dev, "phy interface mode set to rmii\n");
	}
	return 0;
}

static void caninos_gmac_fix_speed(void *priv, unsigned int speed)
{
	caninos_gmac_interface_config(priv, speed);
}

static int caninos_gmac_init(struct platform_device *pdev, void *priv)
{
	struct caninos_priv_data *gmac = priv;
	phy_interface_t mode = gmac->plat_dat->phy_interface;
	struct device *dev = gmac->dev;
	int ret = 0;
	
	switch (mode)
	{
	case PHY_INTERFACE_MODE_RGMII:
		ret = caninos_gmac_interface_config(gmac, 1000U);
		break;
	case PHY_INTERFACE_MODE_RMII:
		ret = caninos_gmac_interface_config(gmac, 100U);
		break;
	default:
		dev_err(dev, "invalid phy interface %d\n", (int)mode);
		ret = -EINVAL;
		break;
	}
	if (ret) {
		return ret;
	}
	
	dev_info(dev, "power up and/or reset the phy\n");
	
	/* power up the phy */
	gpio_direction_output(gmac->reset_gpio, 1);
	if (gpio_is_valid(gmac->power_gpio)) {
		gpio_direction_output(gmac->power_gpio, 1);
	}
	msleep(150); /* time for power up */
	
	/* reset the phy */
	gpio_set_value_cansleep(gmac->reset_gpio, 0);
	usleep_range(12000, 15000); /* time for reset */
	gpio_set_value_cansleep(gmac->reset_gpio, 1);
	msleep(150); /* time required to access registers */
	
	return 0;
}

static void caninos_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct caninos_priv_data *gmac = priv;
	
	clk_disable_unprepare(gmac->ref_clk);
	gmac->speed = 0;
	
	gpio_set_value_cansleep(gmac->reset_gpio, 0);
	if (gpio_is_valid(gmac->power_gpio)) {
		gpio_set_value_cansleep(gmac->power_gpio, 0); /* power off the phy */
	}
}

static int caninos_gmac_probe(struct platform_device *pdev)
{
	struct stmmac_resources stmmac_res;
	struct device *dev = &pdev->dev;
	struct caninos_priv_data *gmac;
	int ret;
	
	gmac = devm_kzalloc(dev, sizeof(*gmac), GFP_KERNEL);
	
	if (!gmac) {
		return -ENOMEM;
	}
	
	gmac->dev = dev;
	gmac->speed = 0U;
	
	gmac->ref_clk = devm_clk_get(dev, "rmii");
	
	if (IS_ERR(gmac->ref_clk)) {
		dev_err(dev, "unable to get ref clock\n");
		return PTR_ERR(gmac->ref_clk);
	}
	
	gmac->pctl = devm_pinctrl_get(dev);
	
	if (IS_ERR(gmac->pctl)) {
		dev_err(dev, "devm_pinctrl_get() failed\n");
		return PTR_ERR(gmac->pctl);
	}
	
	gmac->rmii_state = pinctrl_lookup_state(gmac->pctl, "rmii");
	
	if (IS_ERR(gmac->rmii_state)) {
		dev_err(dev, "could not get pinctrl rmii state\n");
		return PTR_ERR(gmac->rmii_state);
	}
	
	gmac->rgmii_state = pinctrl_lookup_state(gmac->pctl, "rgmii");
	
	if (IS_ERR(gmac->rgmii_state)) {
		dev_err(dev, "could not get pinctrl rgmii state\n");
		return PTR_ERR(gmac->rgmii_state);
	}
	
	ret = caninos_gmac_get_gpios(gmac);
	
	if (ret) {
		return ret;
	}
	
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	
	if (ret) {
		dev_err(dev, "stmmac_get_platform_resources routine failed\n");
		return ret;
	}
	
	gmac->plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	
	if (IS_ERR(gmac->plat_dat)) {
		dev_err(dev, "stmmac_probe_config_dt routine failed\n");
		return PTR_ERR(gmac->plat_dat);
	}
	
	gmac->plat_dat->tx_coe = 0;
	gmac->plat_dat->riwt_off = 1; /* disable Rx Watchdog */
	gmac->plat_dat->bsp_priv = gmac;
	gmac->plat_dat->fix_mac_speed = caninos_gmac_fix_speed;
	gmac->plat_dat->maxmtu = 1518;
	gmac->plat_dat->has_gmac = 1;
	gmac->plat_dat->pmt = 1;
	gmac->plat_dat->force_thresh_dma_mode = 1;
	gmac->plat_dat->tx_fifo_size = 16384;
	gmac->plat_dat->rx_fifo_size = 16384;
	gmac->plat_dat->clk_csr = 0x4;
	gmac->plat_dat->init = caninos_gmac_init;
	gmac->plat_dat->exit = caninos_gmac_exit;
	
	ret = caninos_gmac_init(pdev, gmac);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, gmac->plat_dat);
		return ret;
	}
	
	ret = stmmac_dvr_probe(dev, gmac->plat_dat, &stmmac_res);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, gmac->plat_dat);
	}
	return ret;
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

