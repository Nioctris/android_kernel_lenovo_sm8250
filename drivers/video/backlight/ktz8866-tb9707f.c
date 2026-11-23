// SPDX-License-Identifier: GPL-2.0-only
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define KTZ8866_BL_BRT_LSB	0x04
#define KTZ8866_BL_BRT_MSB	0x05
#define KTZ8866_BL_EN		0x08
#define KTZ8866_LCD_BIAS_CFG1	0x09
#define KTZ8866_LCD_BOOST	0x0c
#define KTZ8866_LCD_VPOS	0x0d
#define KTZ8866_LCD_VNEG	0x0e

#define KTZ8866_BL_ENABLE	BIT(6)
#define KTZ8866_BL_SINKS	GENMASK(5, 0)
#define TB9707F_BL_SINKS	GENMASK(3, 0)
#define KTZ8866_BL_MAX		2047
#define TB9707F_BL_MAX		255

#define KTZ8866_BIAS_ENABLE	(BIT(7) | BIT(2) | BIT(1))
#define KTZ8866_BIAS_DISCHARGE	(BIT(4) | BIT(3))
#define KTZ8866_BIAS_EXT_EN	BIT(0)

static const struct regmap_config tb9707f_ktz8866_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x15,
};

static const struct regulator_ops tb9707f_lcd_bias_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc tb9707f_lcd_bias = {
	.name = "lcd-bias",
	.of_match = "lcd-bias",
	.ops = &tb9707f_lcd_bias_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.enable_reg = KTZ8866_LCD_BIAS_CFG1,
	.enable_mask = KTZ8866_BIAS_ENABLE | KTZ8866_BIAS_EXT_EN,
	.enable_val = KTZ8866_BIAS_ENABLE,
	.enable_time = 2000,
};

static int tb9707f_backlight_update(struct backlight_device *backlight)
{
	struct regmap *regmap = bl_get_data(backlight);
	unsigned int brightness, level;
	int ret;

	brightness = backlight->props.brightness;
	if (backlight->props.power != FB_BLANK_UNBLANK ||
	    backlight->props.fb_blank != FB_BLANK_UNBLANK ||
	    backlight->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;
	if (!brightness)
		return regmap_update_bits(regmap, KTZ8866_BL_EN,
					  KTZ8866_BL_ENABLE, 0);

	level = DIV_ROUND_CLOSEST(brightness * KTZ8866_BL_MAX, TB9707F_BL_MAX);
	ret = regmap_update_bits(regmap, KTZ8866_BL_BRT_LSB, GENMASK(2, 0),
				 level & GENMASK(2, 0));
	if (ret)
		return ret;
	ret = regmap_write(regmap, KTZ8866_BL_BRT_MSB, level >> 3);
	if (ret)
		return ret;

	return regmap_update_bits(regmap, KTZ8866_BL_EN,
				  KTZ8866_BL_ENABLE, KTZ8866_BL_ENABLE);
}

static int tb9707f_backlight_get(struct backlight_device *backlight)
{
	struct regmap *regmap = bl_get_data(backlight);
	unsigned int control, low, high, level;
	int ret;

	ret = regmap_read(regmap, KTZ8866_BL_EN, &control);
	if (ret)
		return ret;
	if (!(control & KTZ8866_BL_ENABLE))
		return 0;

	ret = regmap_read(regmap, KTZ8866_BL_BRT_LSB, &low);
	if (ret)
		return ret;
	ret = regmap_read(regmap, KTZ8866_BL_BRT_MSB, &high);
	if (ret)
		return ret;

	level = (high << 3) | (low & GENMASK(2, 0));
	return DIV_ROUND_CLOSEST(level * TB9707F_BL_MAX, KTZ8866_BL_MAX);
}

/* The only KTZ8866 on the Y700; used by the DSI panel power sequence. */
static struct backlight_device *tb9707f_bl;

/*
 * Panel LCD bias, sequenced like Lenovo's stock lcd_set_bias(): program the
 * boost and +/-5.5 V rails, raise OUTP then OUTN; on the way down drop the
 * backlight first, then OUTN and OUTP. The NT36523 is a TDDI panel, so this
 * also powers its touch controller.
 */
void dualktz8866_lcd_bias(bool on)
{
	struct regmap *regmap;

	if (!tb9707f_bl)
		return;
	regmap = bl_get_data(tb9707f_bl);

	if (on) {
		regmap_write(regmap, KTZ8866_LCD_BOOST, 0x28);
		regmap_write(regmap, KTZ8866_LCD_VPOS, 0x1e);
		regmap_write(regmap, KTZ8866_LCD_VNEG, 0x1e);
		regmap_write(regmap, KTZ8866_LCD_BIAS_CFG1, 0x9c);
		usleep_range(5000, 5100);
		regmap_write(regmap, KTZ8866_LCD_BIAS_CFG1, 0x9e);
		msleep(20);
	} else {
		regmap_update_bits(regmap, KTZ8866_BL_EN, KTZ8866_BL_ENABLE, 0);
		regmap_write(regmap, KTZ8866_LCD_BIAS_CFG1, 0x9c);
		usleep_range(5000, 5100);
		regmap_write(regmap, KTZ8866_LCD_BIAS_CFG1, 0x98);
	}
	pr_info("tb9707f-ktz8866: lcd bias %s\n", on ? "on" : "off");
}
EXPORT_SYMBOL(dualktz8866_lcd_bias);

/* Restore the backlight level after the panel was powered up again. */
void dualktz8866_bl_config(void)
{
	if (!tb9707f_bl)
		return;
	backlight_update_status(tb9707f_bl);
	pr_info("tb9707f-ktz8866: backlight restored to %d\n",
		tb9707f_bl->props.brightness);
}
EXPORT_SYMBOL(dualktz8866_bl_config);

static const struct backlight_ops tb9707f_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = tb9707f_backlight_update,
	.get_brightness = tb9707f_backlight_get,
};

static int tb9707f_ktz8866_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config cfg = { .dev = dev };
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = TB9707F_BL_MAX,
	};
	struct backlight_device *backlight;
	struct regulator_dev *bias;
	struct gpio_desc *enable;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &tb9707f_ktz8866_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Lenovo's stock DTBO uses the original GPIO property spelling. */
	if (of_device_is_compatible(dev->of_node, "ktz,ktz8866"))
		enable = devm_gpiod_get_from_of_node(dev, dev->of_node,
						    "ktz8866_hw_en", 0,
						    GPIOD_OUT_HIGH, "ktz8866-enable");
	else
		enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable))
		return PTR_ERR(enable);

	/* Four sinks and I2C-controlled bias, as configured by the Y700 ABL. */
	ret = regmap_update_bits(regmap, KTZ8866_BL_EN, KTZ8866_BL_SINKS,
				 TB9707F_BL_SINKS);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, KTZ8866_LCD_BIAS_CFG1,
				 KTZ8866_BIAS_DISCHARGE | KTZ8866_BIAS_EXT_EN,
				 KTZ8866_BIAS_DISCHARGE);
	if (ret)
		return ret;

	cfg.regmap = regmap;
	bias = devm_regulator_register(dev, &tb9707f_lcd_bias, &cfg);
	if (IS_ERR(bias))
		return PTR_ERR(bias);

	backlight = devm_backlight_device_register(dev, "ktz8866-backlight",
						   dev, regmap,
						   &tb9707f_backlight_ops,
						   &props);
	if (IS_ERR(backlight))
		return PTR_ERR(backlight);

	ret = tb9707f_backlight_get(backlight);
	if (ret < 0)
		return ret;
	backlight->props.brightness = ret;
	i2c_set_clientdata(client, backlight);
	tb9707f_bl = backlight;
	return 0;
}

static int tb9707f_ktz8866_remove(struct i2c_client *client)
{
	struct backlight_device *backlight = i2c_get_clientdata(client);

	tb9707f_bl = NULL;
	return backlight_device_set_brightness(backlight, 0);
}

static const struct of_device_id tb9707f_ktz8866_of_match[] = {
	{ .compatible = "lenovo,tb9707f-ktz8866" },
	{ .compatible = "ktz,ktz8866" },
	{ }
};
MODULE_DEVICE_TABLE(of, tb9707f_ktz8866_of_match);

static struct i2c_driver tb9707f_ktz8866_driver = {
	.driver = {
		.name = "tb9707f-ktz8866",
		.of_match_table = tb9707f_ktz8866_of_match,
	},
	.probe_new = tb9707f_ktz8866_probe,
	.remove = tb9707f_ktz8866_remove,
};
module_i2c_driver(tb9707f_ktz8866_driver);

MODULE_DESCRIPTION("KTZ8866 backlight and LCD bias for Lenovo Legion Y700");
MODULE_LICENSE("GPL");
