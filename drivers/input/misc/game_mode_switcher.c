// SPDX-License-Identifier: GPL-2.0-only
/*
 * Side slide switch of the Lenovo Legion Y700 (TB-9707F).
 *
 * The switch is a single GPIO ("irq-gpio" of the ts,game_mode_switcher DT
 * node). Report every change of position as a key tap so a userspace key
 * handler can act on it: KEY_F13 when the line goes high, KEY_F14 when it
 * goes low. The current position is also readable from switcher_state.
 */

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define SWITCHER_DEBOUNCE_MS	50
#define SWITCHER_KEY_HIGH	KEY_F13
#define SWITCHER_KEY_LOW	KEY_F14

struct switcher {
	struct input_dev *input;
	struct delayed_work work;
	int gpio;
	int irq;
	int state;
};

static void switcher_report(struct switcher *sw, int state)
{
	unsigned int code = state ? SWITCHER_KEY_HIGH : SWITCHER_KEY_LOW;

	input_report_key(sw->input, code, 1);
	input_sync(sw->input);
	input_report_key(sw->input, code, 0);
	input_sync(sw->input);
}

static void switcher_work(struct work_struct *work)
{
	struct switcher *sw = container_of(to_delayed_work(work),
					   struct switcher, work);
	int state = !!gpio_get_value(sw->gpio);

	if (state == sw->state)
		return;
	sw->state = state;
	switcher_report(sw, state);
}

static irqreturn_t switcher_irq(int irq, void *data)
{
	struct switcher *sw = data;

	mod_delayed_work(system_wq, &sw->work,
			 msecs_to_jiffies(SWITCHER_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

static ssize_t switcher_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct switcher *sw = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!gpio_get_value(sw->gpio));
}
static DEVICE_ATTR_RO(switcher_state);

static int switcher_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct switcher *sw;
	int ret;

	sw = devm_kzalloc(dev, sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return -ENOMEM;

	sw->gpio = of_get_named_gpio(dev->of_node, "irq-gpio", 0);
	if (!gpio_is_valid(sw->gpio))
		return sw->gpio == -EPROBE_DEFER ? -EPROBE_DEFER : -ENODEV;

	ret = devm_gpio_request_one(dev, sw->gpio, GPIOF_IN, "game_mode_switcher");
	if (ret)
		return ret;

	sw->irq = gpio_to_irq(sw->gpio);
	if (sw->irq < 0)
		return sw->irq;

	sw->input = devm_input_allocate_device(dev);
	if (!sw->input)
		return -ENOMEM;
	sw->input->name = "game_mode_switcher";
	sw->input->phys = "game_mode_switcher/input0";
	sw->input->id.bustype = BUS_HOST;
	input_set_capability(sw->input, EV_KEY, SWITCHER_KEY_HIGH);
	input_set_capability(sw->input, EV_KEY, SWITCHER_KEY_LOW);

	ret = input_register_device(sw->input);
	if (ret)
		return ret;

	sw->state = !!gpio_get_value(sw->gpio);
	INIT_DELAYED_WORK(&sw->work, switcher_work);
	platform_set_drvdata(pdev, sw);

	ret = devm_request_irq(dev, sw->irq, switcher_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "game_mode_switcher", sw);
	if (ret)
		return ret;
	enable_irq_wake(sw->irq);

	ret = device_create_file(dev, &dev_attr_switcher_state);
	if (ret)
		dev_warn(dev, "cannot create switcher_state: %d\n", ret);

	dev_info(dev, "gpio %d, state %d\n", sw->gpio, sw->state);
	return 0;
}

static int switcher_remove(struct platform_device *pdev)
{
	struct switcher *sw = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_switcher_state);
	disable_irq_wake(sw->irq);
	devm_free_irq(&pdev->dev, sw->irq, sw);
	cancel_delayed_work_sync(&sw->work);
	return 0;
}

static const struct of_device_id switcher_of_match[] = {
	{ .compatible = "ts,game_mode_switcher" },
	{ }
};
MODULE_DEVICE_TABLE(of, switcher_of_match);

static struct platform_driver switcher_driver = {
	.probe = switcher_probe,
	.remove = switcher_remove,
	.driver = {
		.name = "game_mode_switcher",
		.of_match_table = switcher_of_match,
	},
};
module_platform_driver(switcher_driver);

MODULE_DESCRIPTION("Lenovo TB-9707F side switch");
MODULE_LICENSE("GPL v2");
