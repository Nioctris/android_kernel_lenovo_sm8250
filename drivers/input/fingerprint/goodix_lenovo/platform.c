// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix TEE fingerprint driver, Lenovo TB371FC variant: board glue
 *
 * Copyright (c) 2017 Goodix
 */
#define pr_fmt(fmt) "Goodixfp " fmt

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "gf_spi.h"

#define GF_VDD_UV	3000000

int gf_parse_dts(struct gf_dev *gf_dev)
{
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	int rc;

	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		return rc;
	}

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		rc = gf_dev->irq_gpio;
		goto err_irq;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}

	/* Hold the sensor in reset until the HAL powers it up. */
	gpio_direction_output(gf_dev->reset_gpio, 0);
	gpio_direction_input(gf_dev->irq_gpio);

	return 0;

err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->irq_gpio = -EINVAL;
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	struct device *dev = &gf_dev->spi->dev;

	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		devm_gpio_free(dev, gf_dev->irq_gpio);
		gf_dev->irq_gpio = -EINVAL;
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		devm_gpio_free(dev, gf_dev->reset_gpio);
		gf_dev->reset_gpio = -EINVAL;
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc;

	pr_info(" power_on in\n");

	if (gf_dev->vdd_enabled)
		return 0;

	/* VDD must ramp with the sensor held in reset. */
	if (gpio_is_valid(gf_dev->reset_gpio))
		gpio_set_value(gf_dev->reset_gpio, 0);
	msleep(40);

	rc = regulator_set_voltage(gf_dev->vdd, GF_VDD_UV, GF_VDD_UV);
	if (rc)
		pr_err("failed to set vdd to %d uV, rc = %d\n", GF_VDD_UV, rc);

	rc = regulator_enable(gf_dev->vdd);
	if (rc) {
		pr_info("enable regulato fail\n");
		return rc;
	}
	gf_dev->vdd_enabled = true;
	pr_info("regulator power on\n");

	if (gpio_is_valid(gf_dev->reset_gpio))
		gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(10);

	return 0;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	pr_info("power_off in\n");

	if (!gf_dev->vdd_enabled)
		return 0;

	regulator_disable(gf_dev->vdd);
	gf_dev->vdd_enabled = false;
	pr_info("regulator power_off\n");
	usleep_range(1000, 1100);

	return 0;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	pr_info("reset in\n");

	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}

	gpio_set_value(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	usleep_range(3000, 3100);
	gpio_set_value(gf_dev->reset_gpio, 1);
	if (delay_ms)
		msleep(delay_ms);

	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}

	return gpio_to_irq(gf_dev->irq_gpio);
}
