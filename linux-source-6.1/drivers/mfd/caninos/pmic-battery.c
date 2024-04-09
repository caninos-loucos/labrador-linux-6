/*
 * Copyright (c) 2023 LSI-TEC - Caninos Loucos
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
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/mfd/core.h>
#include <linux/mfd/caninos/atc260x.h>

#define DEFAULT_WQ system_power_efficient_wq
#define DEFAULT_DELAY round_jiffies_relative(HZ)

enum chg_state {
	STATE_NOT_CHARGING = 0,
	STATE_PRE_CHARGING,
	STATE_CC_CHARGING,
	STATE_CV_CHARGING,
};

enum bat_state {
	STATE_NO_BAT = 0,
	STATE_BAT_NCHG,
	STATE_BAT_IDLE,
	STATE_BAT_FULL,
};

struct bat_charger
{
	struct device *dev;
	struct atc260x_dev *pmic;
	struct delayed_work charge_work;
	struct kobject kobj;
	bool shutdown;
	enum chg_state chg_state;
	enum bat_state bat_state;
	bool detection;
};

static int pmu_set_bits(struct bat_charger *priv, uint reg, u16 mask)
{
	u16 val, org;
	int ret;
	
	ret = atc260x_reg_read(priv->pmic, reg);
	
	if (ret >= 0)
	{
		org = (u16)(ret);
		val = org | mask;
		
		ret = (org != val) ? atc260x_reg_write(priv->pmic, reg, val) : 0;
	}
	return ret;
}

static int pmu_clear_bits(struct bat_charger *priv, uint reg, u16 mask)
{
	u16 val, org;
	int ret;
	
	ret = atc260x_reg_read(priv->pmic, reg);
	
	if (ret >= 0)
	{
		org = (u16)(ret);
		val = org & ~mask;
		
		ret = (org != val) ? atc260x_reg_write(priv->pmic, reg, val) : 0;
	}
	return ret;
}

static int pmu_start_bat_detection(struct bat_charger *priv)
{
	int ret;
	
	/* disable battery detection (clear bit 5 of PMU_CHARGER_CTL1) */
	ret = pmu_clear_bits(priv, ATC2603C_PMU_CHARGER_CTL1, BIT(5));
	
	if (!ret) {
		/* start battery detection (set bit 5 of PMU_CHARGER_CTL1) */
		ret = pmu_set_bits(priv, ATC2603C_PMU_CHARGER_CTL1, BIT(5));
	}
	return ret;
}

static int pmu_bat_detection_finished(struct bat_charger *priv)
{
	int ret;
	
	/* when bat detection finishes bit 9 of PMU_CHARGER_CTL1 is set */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL1);
	
	if (ret >= 0) {
		ret = (ret & BIT(9)) ? 1 : 0;
	}
	return ret;
}

static int pmu_get_bat_detection_result(struct bat_charger *priv)
{
	int ret;
	
	/* when bat is detected bit 10 of PMU_CHARGER_CTL1 is set */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL1);
	
	if (ret >= 0) {
		ret = (ret & BIT(10)) ? 1 : 0;
	}
	return ret;
}

static int pmu_is_chg_possible(struct bat_charger *priv)
{
	int ret;
	
	/* check if SYSPWR is higher than BAT (bit 12 of PMU_CHARGER_CTL1) */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL1);
	
	if (ret >= 0) {
		ret = (ret & BIT(12)) ? 1 : 0;
	}
	return ret;
}

static int pmu_is_bat_full(struct bat_charger *priv)
{
	int ret;
	
	/* check if battery is full (bit 15 of PMU_CHARGER_CTL1) */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL1);
	
	if (ret >= 0) {
		ret = (ret & BIT(15)) ? 1 : 0;
	}
	return ret;
}

static int pmu_start_charging(struct bat_charger *priv)
{
	/* enable charge circuit (set bit 15 of PMU_CHARGER_CTL0) */
	return pmu_set_bits(priv, ATC2603C_PMU_CHARGER_CTL0, BIT(15));
}

static int pmu_stop_charging(struct bat_charger *priv)
{
	/* disable charge circuit (clear bit 15 of PMU_CHARGER_CTL0) */
	return pmu_clear_bits(priv, ATC2603C_PMU_CHARGER_CTL0, BIT(15));
}

static int pmu_is_charger_enabled(struct bat_charger *priv)
{
	int ret;
	
	/* check if charger circuit is enabled (bit 15 of PMU_CHARGER_CTL0) */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL0);
	
	if (ret >= 0) {
		ret = (ret & BIT(15)) ? 1 : 0;
	}
	return ret;
}

static int pmu_get_charging_phase(struct bat_charger *priv)
{
	int ret;
	
	/* get charging phase (bits 14:13 of PMU_CHARGER_CTL1) */
	ret = atc260x_reg_read(priv->pmic, ATC2603C_PMU_CHARGER_CTL1);
	
	if (ret >= 0) {
		ret = ((ret >> 13) & 0x3);
	}
	return ret;
}

static bool pmu_handle_charger(struct bat_charger *priv, enum chg_state *state)
{
	enum chg_state next_state = *state;
	int ret;
	
	ret = pmu_is_charger_enabled(priv);
	
	if (ret <= 0) { /* i2c error or not charging */
		next_state = STATE_NOT_CHARGING;
	}
	else {
		ret = pmu_get_charging_phase(priv);
		
		switch (ret)
		{
		case 1:
			next_state = STATE_PRE_CHARGING;
			break;
			
		case 2:
			next_state = STATE_CC_CHARGING;
			break;
			
		case 3:
			next_state = STATE_CV_CHARGING;
			break;
			
		default: /* i2c error or not charging */
			next_state = STATE_NOT_CHARGING;
			break;
		}
	}
	
	if (*state != next_state) {
		*state = next_state;
		return true;
	}
	return false;
}

static bool pmu_handle_battery(struct bat_charger *priv, enum bat_state *state)
{
	enum bat_state next_state = *state;
	int ret;
	
	if (READ_ONCE(priv->detection))
	{
		next_state = STATE_NO_BAT;
		WRITE_ONCE(priv->detection, false);
		
		/* note: detection takes 50ms */
		ret = pmu_bat_detection_finished(priv);
		
		if (ret > 0) /* detection finished */
		{
			ret = pmu_get_bat_detection_result(priv);
			
			if (ret > 0) { /* battery detected */
				next_state = STATE_BAT_IDLE;
			}
		}
	}
	
	if (next_state == STATE_BAT_IDLE)
	{
		/* check if charging is possible */
		ret = pmu_is_chg_possible(priv);
		
		if (ret <= 0) { /* i2c error or charging is not possible */
			next_state = STATE_BAT_NCHG;
		}
		else {
			/* check if battery is full */
			ret = pmu_is_bat_full(priv);
			
			if (ret < 0) { /* i2c error */
				next_state = STATE_BAT_NCHG;
			}
			if (ret > 0) { /* battery is full */
				next_state = STATE_BAT_FULL;
			}
		}
	}
	
	if (next_state != STATE_BAT_IDLE)
	{
		if (!pmu_start_bat_detection(priv)) {
			WRITE_ONCE(priv->detection, true);
		}
	}
	
	if (*state != next_state) {
		*state = next_state;
		return true;
	}
	return false;
}

static void battery_charger_worker(struct work_struct *work)
{
	struct bat_charger *priv;
	enum bat_state bat_state;
	enum chg_state chg_state;
	
	priv = container_of(work, struct bat_charger, charge_work.work);
	
	chg_state = READ_ONCE(priv->chg_state);
	bat_state = READ_ONCE(priv->bat_state);
	
	if (pmu_handle_battery(priv, &bat_state)) {
		WRITE_ONCE(priv->bat_state, bat_state);
	}
	
	if (bat_state == STATE_BAT_IDLE) {
		pmu_start_charging(priv);
	}
	else {
		pmu_stop_charging(priv);
	}
	
	if (pmu_handle_charger(priv, &chg_state)) {
		WRITE_ONCE(priv->chg_state, chg_state);
	}
	
	if (!READ_ONCE(priv->shutdown)) {
		queue_delayed_work(DEFAULT_WQ, &priv->charge_work, DEFAULT_DELAY);
	}
	else {
		pmu_stop_charging(priv);
	}
}

static ssize_t
bat_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct bat_charger *priv = container_of(kobj, struct bat_charger, kobj);
	const char *name;
	
	switch(READ_ONCE(priv->bat_state))
	{
	case STATE_NO_BAT:
		name = "NO_BATTERY";
		break;
		
	case STATE_BAT_NCHG:
		name = "NOT_READY";
		break;
		
	case STATE_BAT_IDLE:
		name = "READY";
		break;
		
	case STATE_BAT_FULL:
		name = "FULL";
		break;
		
	default:
		name = "NULL";
		break;
	}
	
	return sysfs_emit(buf, "%s\n", name);
}

static ssize_t
chg_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct bat_charger *priv = container_of(kobj, struct bat_charger, kobj);
	const char *name;
	
	switch(READ_ONCE(priv->chg_state))
	{
	case STATE_NOT_CHARGING:
		name = "NOT_CHARGING";
		break;
		
	case STATE_PRE_CHARGING:
		name = "PRE_CHARGING";
		break;
		
	case STATE_CC_CHARGING:
		name = "CC_CHARGING";
		break;
		
	case STATE_CV_CHARGING:
		name = "CV_CHARGING";
		break;
		
	default:
		name = "NULL";
		break;
	}
	
	return sysfs_emit(buf, "%s\n", name);
}

static struct kobj_attribute bat_attr = __ATTR_RO(bat_state);

static struct kobj_attribute chg_attr = __ATTR_RO(chg_state);

static struct attribute *battery_attrs[] = {
	&bat_attr.attr,
	&chg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = battery_attrs,
};

static void battery_kobj_release(struct kobject *kobj) {
	/* do nothing */
}

static struct kobj_type bat_ktype = {
	.release = battery_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

static int atc2603c_start_charger(struct bat_charger *priv)
{
	int ret;
	
	ret = atc260x_reg_write(priv->pmic, ATC2603C_PMU_CHARGER_CTL0, 0x325B);
	
	if (ret < 0) {
		dev_err(priv->dev, "unable to write to PMU_CHARGER_CTL0 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(priv->pmic, ATC2603C_PMU_CHARGER_CTL1, 0x1841);
	
	if (ret < 0) {
		dev_err(priv->dev, "unable to write to PMU_CHARGER_CTL1 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(priv->pmic, ATC2603C_PMU_CHARGER_CTL2, 0x0);
	
	if (ret < 0) {
		dev_err(priv->dev, "unable to write to PMU_CHARGER_CTL2 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(priv->pmic, ATC2603C_PMU_BAT_CTL0, 0x5680);
	
	if (ret < 0) {
		dev_err(priv->dev, "unable to write to PMU_BAT_CTL0 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(priv->pmic, ATC2603C_PMU_BAT_CTL1, 0xFC00);
	
	if (ret < 0) {
		dev_err(priv->dev, "unable to write to PMU_BAT_CTL1 register\n");
		return ret;
	}
	
	priv->chg_state = STATE_NOT_CHARGING;
	priv->bat_state = STATE_NO_BAT;
	priv->detection = false;
	priv->shutdown  = false;
	
	queue_delayed_work(DEFAULT_WQ, &priv->charge_work, 0);
	return 0;
}

static void atc2603c_stop_charger(struct bat_charger *priv)
{
	WRITE_ONCE(priv->shutdown, true);
	cancel_delayed_work_sync(&priv->charge_work);
}

static int atc2603c_platform_probe(struct platform_device *pdev)
{
	struct bat_charger *priv = NULL;
	int ret;
	
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	
	if (!priv) {
		return -ENOMEM;
	}
	
	priv->pmic = dev_get_drvdata(pdev->dev.parent);
	
	if (!priv->pmic) {
		return -EINVAL;
	}
	
	priv->dev = &pdev->dev;
	
	INIT_DELAYED_WORK(&priv->charge_work, battery_charger_worker);
	
	/* create /sys/power/caninos */
	ret = kobject_init_and_add(&priv->kobj, &bat_ktype, power_kobj, "caninos");
	
	if (ret) {
		kobject_put(&priv->kobj);
		dev_err(priv->dev, "unable to create sysfs entry\n");
		return ret;
	}
	
	/* create group:
	 * /sys/power/caninos/bat_state
	 * /sys/power/caninos/chg_state
	 */
	ret = sysfs_create_group(&priv->kobj, &attr_group);
	
	if (ret) {
		kobject_put(&priv->kobj);
		dev_err(priv->dev, "unable to create sysfs group\n");
		return ret;
	}
	
	ret = atc2603c_start_charger(priv);
	
	if (ret < 0) {
		kobject_put(&priv->kobj);
		return ret;
	}
	
	platform_set_drvdata(pdev, priv);
	dev_info(priv->dev, "probe finished\n");
	return 0;
}

static void atc2603c_platform_shutdown(struct platform_device *pdev)
{
	struct bat_charger *priv = platform_get_drvdata(pdev);
	
	if (priv) {
		kobject_put(&priv->kobj);
		atc2603c_stop_charger(priv);
	}
}

static int atc2603c_platform_remove(struct platform_device *pdev)
{
	struct bat_charger *priv = platform_get_drvdata(pdev);
	
	if (priv) {
		kobject_put(&priv->kobj);
		atc2603c_stop_charger(priv);
	}
	return 0;
}

static const struct of_device_id atc2603c_battery_of_match[] = {
	{.compatible = "caninos,atc2603c-battery",},
	{}
};
MODULE_DEVICE_TABLE(of, atc2603c_battery_of_match);

static struct platform_driver atc2603c_platform_driver = {
	.probe = atc2603c_platform_probe,
	.remove = atc2603c_platform_remove,
	.shutdown = atc2603c_platform_shutdown,
	.driver = {
		.name = "caninos-battery",
		.owner = THIS_MODULE,
		.of_match_table = atc2603c_battery_of_match,
	},
};

module_platform_driver(atc2603c_platform_driver);

MODULE_AUTHOR("Edgar Bernardi Righi <edgar.righi@lsitec.org.br>");
MODULE_DESCRIPTION("Caninos PMICs battery charging driver");
MODULE_LICENSE("GPL");

