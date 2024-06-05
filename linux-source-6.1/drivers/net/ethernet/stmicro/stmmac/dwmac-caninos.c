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
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/kobject.h>

#include "stmmac_platform.h"

struct caninos_priv_data {
	struct plat_stmmacenet_data *plat;
	struct regmap_field *tx_delay_cfg;
	struct regmap_field *rx_delay_cfg;
	unsigned int curr_tx_delay;
	unsigned int curr_rx_delay;
	struct regmap *regmap;
	struct kobject kobj;
	struct device *dev;
	void __iomem *addr;
	int power_gpio;
};

#define CANINOS_RGMII_RATE 125000000
#define CANINOS_RMII_RATE 50000000

#define REGMAP_NAME "caninos,reset-regmap"
#define REG_DEVRST0 0x00
#define REG_DEVRST1 0x04

static void gmac_delay_kobj_release(struct kobject *kobj) {
	/* do nothing */
}

static int sysfs_convert_string(const char *buf, unsigned int *val)
{
	char *buf_cp = kstrdup(buf, GFP_KERNEL);
	int ret = -EIO;
	
	if (!buf_cp) {
		return -ENOMEM;
	}
	if ((sscanf(buf_cp, "%u", val) == 1) && (*val <= 0xF)){
		ret = 0;
	}
	kfree(buf_cp);
	return ret;
}

static ssize_t rx_delay_store(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              const char *buf, size_t count)
{
	struct caninos_priv_data *gmac = 
		container_of(kobj, struct caninos_priv_data, kobj);
	unsigned int val = 0;
	int ret;
	
	/* delays are only used in rgmii mode */
	if (!phy_interface_mode_is_rgmii(gmac->plat->phy_interface)) {
		return -EIO;
	}
	
	ret = sysfs_convert_string(buf, &val);
	
	if (!ret) {
		ret = regmap_field_write(gmac->rx_delay_cfg, val);
	}
	return ret ? ret : count;
}

static ssize_t tx_delay_store(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              const char *buf, size_t count)
{
	struct caninos_priv_data *gmac = 
		container_of(kobj,struct caninos_priv_data, kobj);
	unsigned int val = 0;
	int ret;
	
	/* delays are only used in rgmii mode */
	if (!phy_interface_mode_is_rgmii(gmac->plat->phy_interface)) {
		return -EIO;
	}
	
	ret = sysfs_convert_string(buf, &val);
	
	if (!ret) {
		ret = regmap_field_write(gmac->tx_delay_cfg, val);
	}
	return ret ? ret : count;
}

static ssize_t rx_delay_show(struct kobject *kobj,
                             struct kobj_attribute *attr, char *buf)
{
	struct caninos_priv_data *gmac = 
		container_of(kobj,struct caninos_priv_data, kobj);
	unsigned int val = 0;
	regmap_field_read(gmac->rx_delay_cfg, &val);
	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t tx_delay_show(struct kobject *kobj,
                             struct kobj_attribute *attr, char *buf)
{
	struct caninos_priv_data *gmac = 
		container_of(kobj,struct caninos_priv_data, kobj);
	unsigned int val = 0;
	regmap_field_read(gmac->tx_delay_cfg, &val);
	return sysfs_emit(buf, "%u\n", val);
}

static struct kobj_attribute rx_delay_attr = __ATTR_RW_MODE(rx_delay, 0644);

static struct kobj_attribute tx_delay_attr = __ATTR_RW_MODE(tx_delay, 0644);

static struct attribute *gmac_delay_attrs[] = {
	&rx_delay_attr.attr,
	&tx_delay_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = gmac_delay_attrs,
};

static struct kobj_type gmac_delay_ktype = {
	.release = gmac_delay_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

static int caninos_gmac_clks_config(void *priv, bool enabled)
{
	struct caninos_priv_data *gmac = priv;
	
	if (!enabled) {
		/* backup delays when interface is in rgmii mode */
		if (phy_interface_mode_is_rgmii(gmac->plat->phy_interface)) {
			regmap_field_read(gmac->tx_delay_cfg, &gmac->curr_tx_delay);
			regmap_field_read(gmac->rx_delay_cfg, &gmac->curr_rx_delay);
		}
		return 0;
	}
	if (phy_interface_mode_is_rgmii(gmac->plat->phy_interface)) {
		clk_set_rate(gmac->plat->pclk, CANINOS_RGMII_RATE);
		writel(0x1, gmac->addr);
		regmap_field_write(gmac->rx_delay_cfg, gmac->curr_rx_delay);
		regmap_field_write(gmac->tx_delay_cfg, gmac->curr_tx_delay);
	}
	else {
		clk_set_rate(gmac->plat->pclk, CANINOS_RMII_RATE);
		writel(0x4, gmac->addr);
		regmap_field_write(gmac->rx_delay_cfg, 0x0);
		regmap_field_write(gmac->tx_delay_cfg, 0x0);
	}
	return 0;
}

static void caninos_gmac_sysfs_remove(struct caninos_priv_data *gmac)
{
	if (gmac && gmac->kobj.state_initialized) {
		kobject_put(&gmac->kobj);
	}
}

static int caninos_gmac_sysfs_create(struct caninos_priv_data *gmac)
{
	struct device *dev = gmac->dev;
	int ret;
	
	/* create /sys/kernel/gmac */
	ret = kobject_init_and_add(&gmac->kobj, &gmac_delay_ktype,
	                           kernel_kobj, "gmac");
	
	if (ret) {
		kobject_put(&gmac->kobj);
		return dev_err_probe(dev, ret, "unable to create sysfs entry\n");
	}
	
	/* create group:
	 * /sys/kernel/gmac/tx_delay
	 * /sys/kernel/gmac/rx_delay
	 */
	ret = sysfs_create_group(&gmac->kobj, &attr_group);
	
	if (ret) {
		kobject_put(&gmac->kobj);
		return dev_err_probe(dev, ret, "unable to create sysfs group\n");
	}
	return 0;
}

static int caninos_gmac_probe_regmap(struct caninos_priv_data *gmac)
{
	struct reg_field tx_delay = REG_FIELD(REG_DEVRST1, 15, 18);
	struct reg_field rx_delay = REG_FIELD(REG_DEVRST1, 19, 22);
	struct device *dev = gmac->dev;
	int ret;
	
	gmac->regmap = syscon_regmap_lookup_by_phandle(dev->of_node, REGMAP_NAME);
	ret = IS_ERR(gmac->regmap) ? PTR_ERR(gmac->regmap) : 0;
	
	if (ret) {
		return dev_err_probe(dev, ret, "unable to get reset registers\n");
	}
	
	gmac->tx_delay_cfg = devm_regmap_field_alloc(dev, gmac->regmap, tx_delay);
	gmac->rx_delay_cfg = devm_regmap_field_alloc(dev, gmac->regmap, rx_delay);
	
	if (IS_ERR(gmac->tx_delay_cfg) || IS_ERR(gmac->rx_delay_cfg)) {
		return dev_err_probe(dev, -EINVAL, "unable to alloc regmap fields\n");
	}
	return 0;
}

static void caninos_gmac_phy_power_off(struct caninos_priv_data *gmac)
{
	if (gpio_is_valid(gmac->power_gpio)) { /* power off the phy */
		gpio_set_value_cansleep(gmac->power_gpio, 0);
	}
}

static void caninos_gmac_phy_power_on(struct caninos_priv_data *gmac)
{
	if (gpio_is_valid(gmac->power_gpio)) { /* power cycle the phy */
		gpio_set_value_cansleep(gmac->power_gpio, 0);
		msleep(50);
		gpio_set_value_cansleep(gmac->power_gpio, 1);
		msleep(150);
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
	ret = caninos_gmac_probe_regmap(gmac);
	
	if (ret) {
		return ret;
	}
	
	gmac->addr = devm_ioremap(dev, 0xe024c0a0, 4);
	
	if (!gmac->addr) {
		return dev_err_probe(dev, -ENOMEM, "unable to map rgmii cfg reg\n");
	}
	
	gmac->power_gpio = of_get_named_gpio(dev->of_node, "phy-power-gpio", 0);
	
	if (gpio_is_valid(gmac->power_gpio))
	{
		ret = devm_gpio_request(dev, gmac->power_gpio, "phy_power");
		
		if (ret) {
			return dev_err_probe(dev, ret, "unable to request power gpio\n");
		}
		gpio_direction_output(gmac->power_gpio, 0);
	}
	
	caninos_gmac_phy_power_on(gmac);
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	
	if (ret) {
		caninos_gmac_phy_power_off(gmac);
		return ret;
	}
	
	gmac->plat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	
	if (IS_ERR(gmac->plat)) {
		caninos_gmac_phy_power_off(gmac);
		return PTR_ERR(gmac->plat);
	}
	
	gmac->plat->clk_csr = 0x4;
	gmac->plat->host_dma_width = 32;
	gmac->plat->bsp_priv = gmac;
	gmac->plat->has_gmac = 1;
	gmac->plat->pmt = 1;
	gmac->plat->maxmtu = 1500;
	gmac->plat->clks_config = caninos_gmac_clks_config;
	
	if (phy_interface_mode_is_rgmii(gmac->plat->phy_interface)) {
		gmac->curr_tx_delay = 0x3;
		gmac->curr_rx_delay = 0x3;
	}
	
	ret = caninos_gmac_clks_config(gmac, true);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, gmac->plat);
		caninos_gmac_phy_power_off(gmac);
		return ret;
	}
	
	ret = stmmac_dvr_probe(dev, gmac->plat, &stmmac_res);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, gmac->plat);
		caninos_gmac_phy_power_off(gmac);
		return ret;
	}
	
	caninos_gmac_sysfs_create(gmac);
	dev_info(dev, "probe finished\n");
	return 0;
}

static int caninos_gmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct plat_stmmacenet_data *plat = priv->plat;
	int ret = stmmac_dvr_remove(&pdev->dev);
	
	caninos_gmac_sysfs_remove(plat->bsp_priv);
	stmmac_remove_config_dt(pdev, plat);
	caninos_gmac_phy_power_off(plat->bsp_priv);
	return ret;
}

static const struct of_device_id caninos_dwmac_match[] = {
	{.compatible = "caninos,k7-gmac" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, caninos_dwmac_match);

static struct platform_driver caninos_dwmac_driver = {
	.probe  = caninos_gmac_probe,
	.remove = caninos_gmac_remove,
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
