// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix TEE fingerprint driver, Lenovo TB371FC variant
 *
 * Copyright (C) 2016 Goodix
 *
 * The sensor is read over SPI by the Goodix trusted app; this driver only
 * owns its power, reset and interrupt and forwards interrupts and screen
 * state to the HAL over netlink. Matches the stock TB371FC kernel
 * (Goodix reference driver V1.2.11 as modified by Lenovo).
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": Goodixfp " fmt

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>

#include "gf_spi.h"

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 11

#define WAKELOCK_HOLD_TIME 500 /* in ms */

#define GF_SPIDEV_NAME		"goodix,fingerprint"
#define GF_DEV_NAME		"goodix_fp"
#define GF_INPUT_NAME		"qwerty"
#define CHRD_DRIVER_NAME	"goodix_fp_spi"
#define CLASS_NAME		"goodix_fp"

#define N_SPI_MINORS		32

static int SPIDEV_MAJOR;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct wakeup_source *fp_wakelock;
static struct class *gf_class;
static struct gf_dev gf;

static const struct gf_key_map maps[] = {
	{ EV_KEY, GF_KEY_INPUT_HOME },
	{ EV_KEY, GF_KEY_INPUT_MENU },
	{ EV_KEY, GF_KEY_INPUT_BACK },
	{ EV_KEY, GF_KEY_INPUT_POWER },
	{ EV_KEY, GF_NAV_INPUT_UP },
	{ EV_KEY, GF_NAV_INPUT_DOWN },
	{ EV_KEY, GF_NAV_INPUT_RIGHT },
	{ EV_KEY, GF_NAV_INPUT_LEFT },
	{ EV_KEY, GF_KEY_INPUT_CAMERA },
	{ EV_KEY, GF_NAV_INPUT_CLICK },
	{ EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
	{ EV_KEY, GF_NAV_INPUT_LONG_PRESS },
	{ EV_KEY, GF_NAV_INPUT_HEAVY },
};

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
		pr_warn("enable_irq in.\n");
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
		pr_warn("disable_irq in.\n");
	} else {
		pr_warn("IRQ has been disabled.\n");
	}
}

static void nav_event_input(struct gf_dev *gf_dev, gf_nav_event_t nav_event)
{
	uint32_t nav_input = 0;

	switch (nav_event) {
	case GF_NAV_FINGER_DOWN:
	case GF_NAV_FINGER_UP:
		return;
	case GF_NAV_DOWN:
		nav_input = GF_NAV_INPUT_DOWN;
		break;
	case GF_NAV_UP:
		nav_input = GF_NAV_INPUT_UP;
		break;
	case GF_NAV_LEFT:
		nav_input = GF_NAV_INPUT_LEFT;
		break;
	case GF_NAV_RIGHT:
		nav_input = GF_NAV_INPUT_RIGHT;
		break;
	case GF_NAV_CLICK:
		nav_input = GF_NAV_INPUT_CLICK;
		break;
	case GF_NAV_HEAVY:
		nav_input = GF_NAV_INPUT_HEAVY;
		break;
	case GF_NAV_LONG_PRESS:
		nav_input = GF_NAV_INPUT_LONG_PRESS;
		break;
	case GF_NAV_DOUBLE_CLICK:
		nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
		break;
	default:
		pr_warn("%s unknown nav event: %d\n", __func__, nav_event);
		break;
	}

	input_report_key(gf_dev->input, nav_input, 1);
	input_sync(gf_dev->input);
	input_report_key(gf_dev->input, nav_input, 0);
	input_sync(gf_dev->input);
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	char msg = GF_NET_EVENT_IRQ;

	__pm_wakeup_event(fp_wakelock, WAKELOCK_HOLD_TIME);
	sendnlmsg(&msg);

	return IRQ_HANDLED;
}

static int irq_setup(struct gf_dev *gf_dev)
{
	int status;

	gf_dev->irq = gf_irq_num(gf_dev);
	status = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);
	pr_err("status to request IRQ:%d\n", status);
	if (status) {
		pr_err("failed to request IRQ:%d\n", gf_dev->irq);
		gf_dev->irq = 0;
		return status;
	}

	enable_irq_wake(gf_dev->irq);
	gf_dev->irq_enabled = 1;

	return 0;
}

static void irq_cleanup(struct gf_dev *gf_dev)
{
	if (!gf_dev->irq)
		return;

	gf_disable_irq(gf_dev);
	disable_irq_wake(gf_dev->irq);
	free_irq(gf_dev->irq, gf_dev);
	gf_dev->irq = 0;
}

static void gf_kernel_key_input(struct gf_dev *gf_dev, struct gf_key *gf_key)
{
	uint32_t key_input;

	if (gf_key->key == GF_KEY_HOME)
		key_input = GF_KEY_INPUT_HOME;
	else if (gf_key->key == GF_KEY_POWER)
		key_input = GF_KEY_INPUT_POWER;
	else if (gf_key->key == GF_KEY_CAMERA)
		key_input = GF_KEY_INPUT_CAMERA;
	else
		key_input = gf_key->key;

	pr_info("%s: received key event[%d], key=%d, value=%d\n",
			__func__, key_input, gf_key->key, gf_key->value);

	if ((gf_key->key == GF_KEY_POWER || gf_key->key == GF_KEY_CAMERA) &&
			gf_key->value == 1) {
		input_report_key(gf_dev->input, key_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, key_input, 0);
		input_sync(gf_dev->input);
	}

	if (gf_key->key == GF_KEY_HOME) {
		input_report_key(gf_dev->input, key_input, gf_key->value);
		input_sync(gf_dev->input);
	}
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key;
	gf_nav_event_t nav_event = GF_NAV_NONE;
	struct gf_ioc_chip_info info;
	u8 netlink_route = NETLINK_TEST;
	int retval = 0;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	switch (cmd) {
	case GF_IOC_INIT:
		if (copy_to_user((void __user *)arg, &netlink_route, sizeof(u8))) {
			pr_err("GF_IOC_INIT failed\n");
			retval = -EFAULT;
		}
		break;
	case GF_IOC_EXIT:
		break;
	case GF_IOC_DISABLE_IRQ:
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_RESET:
		gf_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (void __user *)arg, sizeof(gf_key))) {
			pr_err("failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}
		gf_kernel_key_input(gf_dev, &gf_key);
		break;
	case GF_IOC_NAV_EVENT:
		if (copy_from_user(&nav_event, (void __user *)arg, sizeof(nav_event))) {
			pr_err("failed to copy nav event from user to kernel\n");
			retval = -EFAULT;
			break;
		}
		nav_event_input(gf_dev, nav_event);
		break;
	case GF_IOC_ENABLE_SPI_CLK:
	case GF_IOC_DISABLE_SPI_CLK:
		/* The SPI clock belongs to the TEE. */
		break;
	case GF_IOC_ENABLE_POWER:
		gf_power_on(gf_dev);
		msleep(10);
		break;
	case GF_IOC_DISABLE_POWER:
		gf_power_off(gf_dev);
		break;
	case GF_IOC_ENTER_SLEEP_MODE:
	case GF_IOC_GET_FW_INFO:
		break;
	case GF_IOC_REMOVE:
		pr_err("%s GF_IOC_REMOVE\n", __func__);
		irq_cleanup(gf_dev);
		gf_cleanup(gf_dev);
		break;
	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (void __user *)arg, sizeof(info))) {
			retval = -EFAULT;
			break;
		}
		pr_info("vendor_id : 0x%x\n", info.vendor_id);
		pr_info("mode : 0x%x\n", info.mode);
		pr_info("operation: 0x%x\n", info.operation);
		break;
	default:
		pr_warn("unsupport cmd:0x%x\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			pr_info("Found\n");
			status = 0;
			break;
		}
	}
	pr_info("open status (after find) = %d\n", status);

	if (status) {
		pr_info("No device for minor %d\n", iminor(inode));
		goto out;
	}

	gf_dev->users++;
	filp->private_data = gf_dev;
	nonseekable_open(inode, filp);
	pr_info("Succeed to open device. irq = %d\n", gf_dev->irq);

	if (gf_dev->users == 1) {
		status = gf_parse_dts(gf_dev);
		if (status)
			goto err_users;

		status = irq_setup(gf_dev);
		if (status) {
			gf_cleanup(gf_dev);
			goto err_users;
		}
	}
	gf_dev->device_available = 1;
	goto out;

err_users:
	gf_dev->users--;
	filp->private_data = NULL;
out:
	mutex_unlock(&device_list_lock);
	pr_info("open status (before return) = %d\n", status);
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;

	pr_err("release\n");

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	if (gf_dev && !--gf_dev->users) {
		pr_info("disble_irq. irq = %d\n", gf_dev->irq);
		irq_cleanup(gf_dev);
		gf_cleanup(gf_dev);
		gf_dev->device_available = 0;
		gf_power_off(gf_dev);
	}

	mutex_unlock(&device_list_lock);
	return 0;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif
	.open = gf_open,
	.release = gf_release,
};

#ifdef CONFIG_FB
static int goodix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct gf_dev *gf_dev = container_of(nb, struct gf_dev, notifier);
	struct fb_event *evdata = data;
	char msg;

	if (val != FB_EVENT_BLANK)
		return 0;

	pr_info("[info] %s go to the value = %d\n", __func__, (int)val);

	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	switch (*(int *)evdata->data) {
	case FB_BLANK_POWERDOWN:
		if (gf_dev->device_available == 1) {
			gf_dev->fb_black = 1;
			msg = GF_NET_EVENT_FB_BLACK;
			sendnlmsg(&msg);
		}
		break;
	case FB_BLANK_UNBLANK:
		if (gf_dev->device_available == 1) {
			gf_dev->fb_black = 0;
			msg = GF_NET_EVENT_FB_UNBLACK;
			sendnlmsg(&msg);
		}
		break;
	default:
		pr_info("%s defalut\n", __func__);
		break;
	}

	return NOTIFY_OK;
}
#endif

static int gf_probe(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	struct device *dev;
	unsigned long minor;
	int status;
	int i;

	pr_err("probe in!\n");

	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = pdev;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

	gf_dev->vdd = devm_regulator_get(&pdev->dev, "goodix_vdd");
	if (IS_ERR(gf_dev->vdd))
		return PTR_ERR(gf_dev->vdd);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor >= N_SPI_MINORS) {
		mutex_unlock(&device_list_lock);
		return -ENODEV;
	}

	gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
	dev = device_create(gf_class, &pdev->dev, gf_dev->devt, gf_dev,
			GF_DEV_NAME);
	if (IS_ERR(dev)) {
		gf_dev->devt = 0;
		mutex_unlock(&device_list_lock);
		return PTR_ERR(dev);
	}
	set_bit(minor, minors);
	list_add(&gf_dev->device_entry, &device_list);
	mutex_unlock(&device_list_lock);

	gf_dev->input = input_allocate_device();
	if (!gf_dev->input) {
		pr_err("%s, failed to allocate input device\n", __func__);
		status = -ENOMEM;
		goto err_dev;
	}

	for (i = 0; i < ARRAY_SIZE(maps); i++)
		input_set_capability(gf_dev->input, maps[i].type, maps[i].code);
	gf_dev->input->name = GF_INPUT_NAME;

	status = input_register_device(gf_dev->input);
	if (status) {
		pr_err("failed to register input device\n");
		input_free_device(gf_dev->input);
		gf_dev->input = NULL;
		goto err_dev;
	}

#ifdef CONFIG_FB
	gf_dev->notifier.notifier_call = goodix_fb_state_chg_callback;
	fb_register_client(&gf_dev->notifier);
#endif

	fp_wakelock = wakeup_source_register(NULL, "fp_wakelock");

	pr_info("version V%d.%d.%02d\n", VER_MAJOR, VER_MINOR, PATCH_LEVEL);
	pr_info("probe finish.\n");

	return 0;

err_dev:
	pr_info("Err: status = %d\n", status);
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	mutex_unlock(&device_list_lock);
	return status;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;

	pr_err("remove the device\n");

	wakeup_source_unregister(fp_wakelock);
#ifdef CONFIG_FB
	fb_unregister_client(&gf_dev->notifier);
#endif
	input_unregister_device(gf_dev->input);

	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct of_device_id gx_match_table[] = {
	{ .compatible = GF_SPIDEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, gx_match_table);

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status;

	pr_err("init in\n");

	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		return status;
	}
	SPIDEV_MAJOR = status;

	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}

	status = platform_driver_register(&gf_driver);
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME);
		pr_warn("Failed to register SPI driver.\n");
		return status;
	}

	netlink_init();
	pr_info("status = 0x%x\n", status);

	return 0;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	netlink_exit();
	platform_driver_unregister(&gf_driver);
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
