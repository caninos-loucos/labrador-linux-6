#include <linux/mfd/caninos/regs_map_atc2603c.h>
#include <linux/mfd/caninos/atc260x.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of_platform.h>

#include "pmic-core.h"

#define ATC2603C_NGPIO 6

struct atc260x_gpio {
    struct gpio_chip gpio_chip;
    struct atc260x_dev *pmic;
};

static int atc260x_gpio_get_value(struct gpio_chip *gc, unsigned int offset) {
	struct atc260x_gpio *chip = gpiochip_get_data(gc);
	u16 ctl4;
	int val;

	ctl4 = atc260x_reg_read(chip->pmic, ATC2603C_PMU_SGPIO_CTL4);
	dev_dbg(chip->pmic->dev, "GPIO CTL4 value: %d\n", ctl4);
	val = (ctl4 & (BIT(offset))) >> offset;

	return (int) val;
}

static void atc260x_gpio_set_value(struct gpio_chip *gc, unsigned int offset, int value) {
	struct atc260x_gpio *chip = gpiochip_get_data(gc);
	u16 ctl4;
	int ret;
	ret = atc260x_reg_setbits(chip->pmic, ATC2603C_PMU_SGPIO_CTL4, BIT(offset), value << offset);

	if(ret)
		dev_err(chip->pmic->dev, "error writing registers");

	ctl4 = atc260x_reg_read(chip->pmic, ATC2603C_PMU_SGPIO_CTL4);

	dev_dbg(chip->pmic->dev, "GPIO CTL4 value: %d\n", ctl4);
}

static int atc260x_gpio_get_direction(struct gpio_chip *gc, unsigned int offset) { //Gets the direction of the GPIO pin
	struct atc260x_gpio *chip = gpiochip_get_data(gc);
	u16 ctl3, val;

	ctl3 = atc260x_reg_read(chip->pmic, ATC2603C_PMU_SGPIO_CTL3);
	dev_dbg(chip->pmic->dev, "GPIO CTL3 value: %d\n", ctl3);

	offset += 2;
	val = (ctl3 & BIT(offset)) >> offset;

	return (int) val;
}

static int atc260x_gpio_direction_input(struct gpio_chip *gc, unsigned int offset) {
	struct atc260x_gpio *chip = gpiochip_get_data(gc);
	int ret;

	offset += 2;
	ret = atc260x_reg_setbits(chip->pmic, ATC2603C_PMU_SGPIO_CTL3, BIT(offset), BIT(offset));
	if(ret)
		return ret;

	offset += 7;
	ret = atc260x_reg_setbits(chip->pmic, ATC2603C_PMU_SGPIO_CTL3, BIT(offset), 0);
	if(ret)
		return ret;

	return 0;
}

static int atc260x_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value) {
	struct atc260x_gpio *chip = gpiochip_get_data(gc);
	int ret;

	atc260x_gpio_set_value(gc, offset, value);

	offset += 2;
	ret = atc260x_reg_setbits(chip->pmic, ATC2603C_PMU_SGPIO_CTL3, BIT(offset), 0);
	if(ret)
		return ret;

	offset += 7;
	ret = atc260x_reg_setbits(chip->pmic, ATC2603C_PMU_SGPIO_CTL3, BIT(offset), BIT(offset));
	if(ret)
		return ret;

	return 0;
}

static int atc260x_gpio_simple_xlate(struct gpio_chip *gc,
				const struct of_phandle_args *gpiospec,
				u32 *flags)
{
	/*
	 * We're discouraging gpio_cells < 2, since that way you'll have to
	 * write your own xlate function (that will have to retrieve the GPIO
	 * number and the flags from a single gpio cell -- this is possible,
	 * but not recommended).
	 */
	if (gc->of_gpio_n_cells < 2) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

	if (gpiospec->args[0] >= gc->ngpio)
		return -EINVAL;

	if (flags)
		*flags = gpiospec->args[1];

	dev_info(gc->parent, "line %d got from dt", (int) gpiospec->args[0]);

	return gpiospec->args[0];
}


static int atc260x_gpio_probe(struct platform_device *pdev) {
    int ret;
    struct atc260x_gpio *chip;
    struct gpio_chip *gc;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	
	if (!chip) {
		return -ENOMEM;
	}
	
	chip->pmic = dev_get_drvdata(pdev->dev.parent);
	
	if (!chip->pmic) {
		dev_info(&pdev->dev, "pmic null");
		return -EINVAL;
	}

	gc = &chip->gpio_chip;

	gc->direction_input  = atc260x_gpio_direction_input;
	gc->direction_output = atc260x_gpio_direction_output;
	gc->get = atc260x_gpio_get_value;
	gc->set = atc260x_gpio_set_value;
	gc->get_direction = atc260x_gpio_get_direction;
	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->of_xlate = atc260x_gpio_simple_xlate;
	gc->ngpio = ATC2603C_NGPIO;
	gc->label = "atc260x-gpio";
	gc->base = -1;
	gc->parent = chip->pmic->dev;
	gc->owner = THIS_MODULE;
	gc->can_sleep = false;
	
	ret = devm_gpiochip_add_data(&pdev->dev, gc, chip);
	if(ret){
		dev_err(&pdev->dev, "error adding gpiochip data");
		return ret;
	}

	ret = platform_device_add_data(pdev, chip, sizeof(chip));
	if(ret){
		dev_err(&pdev->dev, "error adding platform data");
		return ret;
	}

	dev_info(&pdev->dev, "probe finished");

    return 0;
}

static int atc260x_gpio_remove (struct platform_device *pdev){
	struct atc260x_gpio *chip = dev_get_platdata(&pdev->dev);
	gpiochip_remove(&chip->gpio_chip);
	kfree(chip);
	return 0;
}

static const struct of_device_id atc260x_gpio_of_match[] = {
	{ .compatible = "caninos,atc2603c-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, atc260x_gpio_of_match);

static const struct platform_device_id atc260x_gpio_id_table[] = {
	{ "atc2603c-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, atc260x_gpio_id_table);

static struct platform_driver atc260x_gpio_driver = {
	.probe = atc260x_gpio_probe,
	.remove = atc260x_gpio_remove,
	.driver = {
		.name = "atc2603c-gpio",
		.owner = THIS_MODULE,
		.of_match_table = atc260x_gpio_of_match,
	},
	.id_table = atc260x_gpio_id_table,
}; 

module_platform_driver(atc260x_gpio_driver);

MODULE_AUTHOR("Ana Clara Forcelli <ana.forcelli@lsitec.org.br>");
MODULE_DESCRIPTION("ATC2603C GPIO driver");
MODULE_LICENSE("GPL");

