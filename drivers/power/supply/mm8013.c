// SPDX-License-Identifier: GPL-2.0
/*
 * Mitsumi MM8013 fuel gauge
 *
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 *
 * Based on the mainline driver, with the behaviour of the Lenovo TB371FC stock
 * driver (CONFIG_MM8013C06_BATTERY): the gauge is registered as the "bms"
 * supply that the Qualcomm SMB5 charger stack reads the battery state from,
 * the charge voltage register is exposed as VOLTAGE_MAX, values are cached
 * while the system is suspended and the state of charge is polled so user
 * space hears about changes.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define REG_BATID			0x00
 #define BATID_CMD			0x0008
 #define BATID_101			0x0101 /* 107 kOhm */
 #define BATID_102			0x0102 /* 10 kOhm */
#define REG_TEMPERATURE			0x06
#define REG_VOLTAGE			0x08
#define REG_FLAGS			0x0a
 #define MM8013_FLAG_OTC		BIT(15)
 #define MM8013_FLAG_OTD		BIT(14)
 #define MM8013_FLAG_BATHI		BIT(13)
 #define MM8013_FLAG_BATLOW		BIT(12)
 #define MM8013_FLAG_CHG_INH		BIT(11)
 #define MM8013_FLAG_FC			BIT(9)
 #define MM8013_FLAG_CHG		BIT(8)
 #define MM8013_FLAG_OCC		BIT(6)
 #define MM8013_FLAG_ODC		BIT(5)
 #define MM8013_FLAG_OT			BIT(4)
 #define MM8013_FLAG_UT			BIT(3)
 #define MM8013_FLAG_DSG		BIT(0)
#define REG_NOMINAL_CHARGE_CAPACITY	0x0c
#define REG_FULL_CHARGE_CAPACITY	0x0e
#define REG_AVERAGE_CURRENT		0x14
#define REG_AVERAGE_TIME_TO_EMPTY	0x16
#define REG_CYCLE_COUNT			0x2a
#define REG_STATE_OF_CHARGE		0x2c
#define REG_STATE_OF_HEALTH		0x2e
#define REG_CHARGE_VOLTAGE		0x30
#define REG_DESIGN_CAPACITY		0x3c

#define DECIKELVIN_TO_DECIDEGC(t)	((t) - 2731)

/*
 * The TB371FC pack is 10200 mAh. Lenovo's driver reports this for both the
 * design and the full charge capacity rather than the gauge's estimate.
 */
#define MM8013_DEFAULT_CAPACITY_UAH	10200000

#define MM8013_SOC_POLL_MS		10000
#define MM8013_BATID_RETRIES		3
#define MM8013_CMD_DELAY_MS		4

enum mm8013_cached {
	CACHE_FLAGS,
	CACHE_SOC,
	CACHE_VOLTAGE,
	CACHE_CURRENT,
	CACHE_TEMP,
	CACHE_CYCLE_COUNT,
	CACHE_REMAINING,
	CACHE_TIME_TO_EMPTY,
	CACHE_SOH,
	CACHE_CHARGE_VOLTAGE,
	CACHE_NUM,
};

struct mm8013_chip {
	struct i2c_client *client;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work soc_work;
	struct mutex lock;
	bool suspended;
	int battery_id;
	int last_soc;
	u32 cache[CACHE_NUM];
};

/*
 * Read a gauge register. While suspended, or when the bus read fails, hand
 * out the last good value instead, as the stock driver does.
 */
static u32 mm8013_read(struct mm8013_chip *chip, unsigned int reg,
		       enum mm8013_cached slot)
{
	u32 val;
	int ret;

	mutex_lock(&chip->lock);
	if (!chip->suspended) {
		ret = regmap_read(chip->regmap, reg, &val);
		if (!ret)
			chip->cache[slot] = val;
		else
			dev_err_ratelimited(&chip->client->dev,
					    "failed to read reg 0x%02x: %d\n",
					    reg, ret);
	}
	val = chip->cache[slot];
	mutex_unlock(&chip->lock);

	return val;
}

static int mm8013_read_battery_id(struct mm8013_chip *chip)
{
	u32 val;
	int i, ret = -EIO;

	for (i = 0; i < MM8013_BATID_RETRIES; i++) {
		ret = regmap_write(chip->regmap, REG_BATID, BATID_CMD);
		if (ret)
			continue;

		msleep(MM8013_CMD_DELAY_MS);

		ret = regmap_read(chip->regmap, REG_BATID, &val);
		if (ret)
			continue;

		if (val == BATID_102)
			return 2;
		if (val == BATID_101)
			return 1;

		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property mm8013_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_SOH,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_DEBUG_BATTERY,
};

static int mm8013_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct mm8013_chip *chip = power_supply_get_drvdata(psy);
	u32 regval;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		regval = mm8013_read(chip, REG_STATE_OF_CHARGE, CACHE_SOC);
		val->intval = min_t(u32, regval, 100);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		regval = mm8013_read(chip, REG_VOLTAGE, CACHE_VOLTAGE);
		val->intval = 1000 * regval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		/* Positive while discharging, as the Qualcomm gauges report. */
		regval = mm8013_read(chip, REG_AVERAGE_CURRENT, CACHE_CURRENT);
		val->intval = 1000 * (s16)regval;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		regval = mm8013_read(chip, REG_TEMPERATURE, CACHE_TEMP);
		val->intval = DECIKELVIN_TO_DECIDEGC((int)regval);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		regval = mm8013_read(chip, REG_CYCLE_COUNT, CACHE_CYCLE_COUNT);
		val->intval = (s16)regval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		regval = mm8013_read(chip, REG_CHARGE_VOLTAGE,
				     CACHE_CHARGE_VOLTAGE);
		val->intval = 1000 * regval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = MM8013_DEFAULT_CAPACITY_UAH;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/*
		 * Remaining charge in uAh, as the ABI and Android expect. The
		 * stock driver put the state of health here for ZUI; that is
		 * reported as SOH instead.
		 */
		regval = mm8013_read(chip, REG_NOMINAL_CHARGE_CAPACITY,
				     CACHE_REMAINING);
		val->intval = 1000 * regval;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		regval = mm8013_read(chip, REG_AVERAGE_TIME_TO_EMPTY,
				     CACHE_TIME_TO_EMPTY);
		/* The gauge reports minutes, 0xffff until it has an estimate. */
		if (regval == U16_MAX)
			return -ENODATA;
		val->intval = 60 * regval;
		break;
	case POWER_SUPPLY_PROP_SOH:
		regval = mm8013_read(chip, REG_STATE_OF_HEALTH, CACHE_SOH);
		val->intval = regval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		regval = mm8013_read(chip, REG_FLAGS, CACHE_FLAGS);
		if (regval & MM8013_FLAG_UT)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (regval & (MM8013_FLAG_ODC | MM8013_FLAG_OCC))
			val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;
		else if (regval & MM8013_FLAG_BATLOW)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (regval & MM8013_FLAG_BATHI)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (regval & (MM8013_FLAG_OT | MM8013_FLAG_OTD |
				   MM8013_FLAG_OTC))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mm8013_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct mm8013_chip *chip = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		mutex_lock(&chip->lock);
		if (chip->suspended) {
			mutex_unlock(&chip->lock);
			return -EBUSY;
		}
		ret = regmap_write(chip->regmap, REG_CHARGE_VOLTAGE,
				   val->intval / 1000);
		if (!ret) {
			chip->cache[CACHE_CHARGE_VOLTAGE] = val->intval / 1000;
			msleep(MM8013_CMD_DELAY_MS);
		}
		mutex_unlock(&chip->lock);
		return ret;
	default:
		return -EINVAL;
	}
}

static int mm8013_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_VOLTAGE_MAX;
}

static const struct power_supply_desc mm8013_desc = {
	.name			= "bms",
	.type			= POWER_SUPPLY_TYPE_BMS,
	.properties		= mm8013_battery_props,
	.num_properties		= ARRAY_SIZE(mm8013_battery_props),
	.get_property		= mm8013_get_property,
	.set_property		= mm8013_set_property,
	.property_is_writeable	= mm8013_property_is_writeable,
};

static void mm8013_soc_work(struct work_struct *work)
{
	struct mm8013_chip *chip = container_of(work, struct mm8013_chip,
						soc_work.work);
	int soc = mm8013_read(chip, REG_STATE_OF_CHARGE, CACHE_SOC);

	if (soc != chip->last_soc) {
		chip->last_soc = soc;
		power_supply_changed(chip->psy);
	}

	schedule_delayed_work(&chip->soc_work,
			      msecs_to_jiffies(MM8013_SOC_POLL_MS));
}

static const struct regmap_config mm8013_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x68,
	.use_single_rw = true,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int mm8013_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct mm8013_chip *chip;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return dev_err_probe(dev, -EIO,
				     "I2C_FUNC_SMBUS_WORD_DATA not supported\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->last_soc = -1;
	/* Until the first read: 25 degC, so a failed read can't look fatal. */
	chip->cache[CACHE_TEMP] = 2981;
	mutex_init(&chip->lock);
	INIT_DELAYED_WORK(&chip->soc_work, mm8013_soc_work);
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &mm8013_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Couldn't initialize regmap\n");

	chip->battery_id = mm8013_read_battery_id(chip);
	if (chip->battery_id < 0)
		return dev_err_probe(dev, chip->battery_id,
				     "MM8013 not found\n");

	psy_cfg.drv_data = chip;
	psy_cfg.of_node = dev->of_node;

	chip->psy = devm_power_supply_register(dev, &mm8013_desc, &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy),
				     "Couldn't register power supply\n");

	dev_info(dev, "battery id %d\n", chip->battery_id);

	schedule_delayed_work(&chip->soc_work, 0);

	return 0;
}

static int mm8013_remove(struct i2c_client *client)
{
	struct mm8013_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->soc_work);

	return 0;
}

static int __maybe_unused mm8013_suspend(struct device *dev)
{
	struct mm8013_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->soc_work);

	mutex_lock(&chip->lock);
	chip->suspended = true;
	mutex_unlock(&chip->lock);

	return 0;
}

static int __maybe_unused mm8013_resume(struct device *dev)
{
	struct mm8013_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	chip->suspended = false;
	mutex_unlock(&chip->lock);

	schedule_delayed_work(&chip->soc_work, 0);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mm8013_pm_ops, mm8013_suspend, mm8013_resume);

static const struct i2c_device_id mm8013_id_table[] = {
	{ "mm8013", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, mm8013_id_table);

static const struct of_device_id mm8013_match_table[] = {
	{ .compatible = "mitsumi,mm8013" },
	{ .compatible = "mitsumi,mm8013c06" },
	{}
};
MODULE_DEVICE_TABLE(of, mm8013_match_table);

static struct i2c_driver mm8013_i2c_driver = {
	.probe = mm8013_probe,
	.remove = mm8013_remove,
	.id_table = mm8013_id_table,
	.driver = {
		.name = "mm8013",
		.of_match_table = mm8013_match_table,
		.pm = &mm8013_pm_ops,
	},
};
module_i2c_driver(mm8013_i2c_driver);

MODULE_DESCRIPTION("MM8013 fuel gauge driver");
MODULE_LICENSE("GPL");
