/*
 * Copyright (c) 2019-2024 LSI-TEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/mfd/caninos/atc260x.h>
#include <linux/dma-mapping.h>
#include <asm/system_misc.h>

#include "pmic-core.h"

#define MFD_CELL_OF_NAME(_name, _compat) \
	{ .name = _name, .of_compatible = _compat, }

static struct atc260x_dev pmic = { .dev = NULL };

static struct regmap_config atc2603c_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.max_register = ATC2603C_CHIP_VER,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static struct regmap_config atc2609a_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.max_register = ATC2609A_CHIP_VER,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static const struct mfd_cell sc_atc2603c_cells[] = {
	MFD_CELL_OF_NAME("caninos-audio", "caninos,atc2603c-audio"),
	MFD_CELL_OF_NAME("caninos-auxadc", "caninos,atc2603c-auxadc"),
	MFD_CELL_OF_NAME("caninos-battery", "caninos,atc2603c-battery"),
};

static void atc260x_prepare_pm_op(void)
{
	/* clear all wakeup sources except on/off long and reset */
	atc260x_reg_setbits(&pmic, ATC2603C_PMU_SYS_CTL0, 0xfbe0,
	                    BIT(13) | BIT(6));
	atc260x_reg_setbits(&pmic, ATC2603C_PMU_SYS_CTL4,
	                    BIT(9) | BIT(8) | BIT(7), 0);
	
	/* set bit 14 to enable s3 power state (sleep mode)
	 * this setting is ignored when current state is s1 (working mode) */
	atc260x_reg_setbits(&pmic, ATC2603C_PMU_SYS_CTL3,
	                    BIT(15) | BIT(14), BIT(14));
}

static int atc260x_system_control_setup(void)
{
	int ret;
	
	ret = atc260x_reg_write(&pmic, ATC2603C_PMU_SYS_CTL0, 0xE0CB);
	
	if (ret < 0) {
		dev_err(pmic.dev, "unable to write to PMU_SYS_CTL0 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(&pmic, ATC2603C_PMU_SYS_CTL1, 0xF);
	
	if (ret < 0) {
		dev_err(pmic.dev, "unable to write to PMU_SYS_CTL1 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(&pmic, ATC2603C_PMU_SYS_CTL2, 0x680);
	
	if (ret < 0) {
		dev_err(pmic.dev, "unable to write to PMU_SYS_CTL2 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(&pmic, ATC2603C_PMU_SYS_CTL3, 0x280);
	
	if (ret < 0) {
		dev_err(pmic.dev, "unable to write to PMU_SYS_CTL3 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(&pmic, ATC2603C_PMU_SYS_CTL5, 0x180);
	
	if (ret < 0) {
		dev_err(pmic.dev, "unable to write to PMU_SYS_CTL5 register\n");
		return ret;
	}
	
	return 0;
}

static void atc260x_poweroff(void)
{
	atc260x_prepare_pm_op();
	
	/* clear bit 0 to leave s1 power state */
	atc260x_reg_setbits(&pmic, ATC2603C_PMU_SYS_CTL1, BIT(0), 0);
	mdelay(500);
}

static int atc260x_restart(struct notifier_block *nb, unsigned long action,
                        void *data)
{
	atc260x_prepare_pm_op();
	
	/* set bit 10 to start reset */
	atc260x_reg_setbits(&pmic, ATC2603C_PMU_SYS_CTL0, BIT(10), BIT(10));
	mdelay(500);
	
	return NOTIFY_DONE;
}

static struct notifier_block atc260x_restart_nb = {
	.notifier_call = atc260x_restart,
	.priority = 200,
};

static int atc260x_i2c_probe(struct i2c_client *i2c,
                             const struct i2c_device_id *id)
{
	struct regmap *regmap;
	int ret;
	
	BUG_ON(!i2c || !id);
	
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "i2c bus is not functional\n");
		return -EPROBE_DEFER;
	}
	if (pmic.dev) {
		dev_err(&i2c->dev, "only one instance can be active at any time\n");
		return -EINVAL;
	}
	
	dma_coerce_mask_and_coherent(&i2c->dev, DMA_BIT_MASK(32));
	
	switch ((int)id->driver_data)
	{
	case ATC260X_ICTYPE_2603C:
		regmap = devm_regmap_init_i2c(i2c, &atc2603c_i2c_regmap_config);
		break;
	case ATC260X_ICTYPE_2609A:
		regmap = devm_regmap_init_i2c(i2c, &atc2609a_i2c_regmap_config);
		break;
	default:
		dev_err(&i2c->dev, "invalid ic type.\n");
		return -ENODEV;
	}
	
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&i2c->dev, "could not allocate register map: %d\n", ret);
		return ret;
	}
	
	pmic.dev =  &i2c->dev;
	pmic.regmap = regmap;
	pmic.ic_type = (int)id->driver_data;
	
	i2c_set_clientdata(i2c, &pmic);
	
	ret = atc260x_system_control_setup();
	
	if (ret) {
		dev_err(&i2c->dev, "failed to setup system control regs: %d\n", ret);
		i2c_set_clientdata(i2c, NULL);
		pmic.dev = NULL;
		return ret;
	}
	
	ret = devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_NONE,
	                           sc_atc2603c_cells,
	                           ARRAY_SIZE(sc_atc2603c_cells),
	                           NULL, 0, NULL);
	
	if (ret) {
		dev_err(&i2c->dev, "failed to add children devices: %d\n", ret);
		i2c_set_clientdata(i2c, NULL);
		pmic.dev = NULL;
		return ret;
	}
	
	pm_power_off = atc260x_poweroff;
	register_restart_handler(&atc260x_restart_nb);
	
	dev_info(&i2c->dev, "probe finished\n");
	return 0;
}

static void atc260x_i2c_remove(struct i2c_client *i2c)
{
	if (pm_power_off == atc260x_poweroff) {
		pm_power_off = NULL;
	}
	unregister_restart_handler(&atc260x_restart_nb);
	pmic.dev = NULL;
}

const struct i2c_device_id atc260x_i2c_id[] = {
	{ "atc2603c", ATC260X_ICTYPE_2603C },
	{ "atc2609a", ATC260X_ICTYPE_2609A },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, atc260x_i2c_id);

static struct i2c_driver atc260x_i2c_driver = {
	.driver = {
		.name = "caninos-pmic",
	},
	.id_table = atc260x_i2c_id,
	.probe = atc260x_i2c_probe,
	.remove = atc260x_i2c_remove,
};

module_i2c_driver(atc260x_i2c_driver);

MODULE_AUTHOR("Edgar Bernardi Righi <edgar.righi@lsitec.org.br>");
MODULE_DESCRIPTION("Caninos PMICs i2c device driver.");
MODULE_LICENSE("GPL");

