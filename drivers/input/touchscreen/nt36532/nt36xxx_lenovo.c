// SPDX-License-Identifier: GPL-2.0
/*
 * Lenovo TB371FC additions to the Novatek NT36532 touch driver, as in
 * Lenovo's stock kernel for the tablet:
 *
 *  - firmware commands for pen mode/type, palm rejection by panel
 *    direction, the edge grid zone and game mode, kept across resets
 *  - the stylus charger hand-off (pen sensing is off while it charges)
 *  - double-tap wake, armed through /proc/gesture_control or an
 *    EV_SYN/SYN_CONFIG event, with the keyboard cover's hall sensor
 *    putting the controller to sleep while the cover is closed
 *  - stylus state events and the lenovo_penraw recent pen sample buffer
 */

#include <linux/cdev.h>
#include <linux/ctn730.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/input.h>
#include <linux/input/nvt_lenovo.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "nt36xxx.h"
#include "nt36xxx_lenovo.h"

/* Host commands, written to EVENT_MAP_HOST_CMD. */
#define NVT_CMD_DEEP_SLEEP		0x11
#define NVT_CMD_GESTURE_MODE		0x13
#define NVT_CMD_GAME_MODE		0x79
#define NVT_CMD_PEN_MODE		0x7B
#define NVT_CMD_EDGE_GRID_ZONE		0x7F
#define NVT_CMD_RETRY			5

/* The FW reports stylus state as a function page event. */
#define NVT_FUNCPAGE_STYLUS		5
#define NVT_STYLUS_LEAVE		0
#define NVT_STYLUS_DETECT		1
#define NVT_STYLUS_ENTER		2

/* Keyboard cover hall sensor: TLMM GPIO 175, low while the cover is shut. */
#define NVT_HALL_TLMM_LABEL		"f000000.pinctrl"
#define NVT_HALL_TLMM_GPIO		175
#define NVT_HALL_CHECK_PERIOD_MS	1000

#define NVT_PENRAW_POINTS		32
#define NVT_PENRAW_REPORT_POINTS	16

uint8_t nvt_gesture_flag;
EXPORT_SYMBOL(nvt_gesture_flag);

static struct workqueue_struct *nvt_hall_check_wq;
static struct delayed_work nvt_hall_check_work;
static int nvt_hall_gpio = -ENODEV;
static unsigned long nvt_hall_timer;
/* The cover is closed and the controller was put to sleep for it. */
static bool hall_status;
/* The cover changed while the system was suspended. */
static bool nvt_hall_pm;
/* The cover was closed during system suspend; already asleep. */
static bool nvt_hall_pm_status;

/*******************************************************
 * Host commands
 *******************************************************/
/*
 * Send a host command and wait for the FW to acknowledge it by clearing
 * the command byte, retrying up to NVT_CMD_RETRY times.
 */
static int32_t nvt_lenovo_send_cmd(uint8_t *cmd, uint16_t len,
				   uint16_t ack_len)
{
	uint8_t buf[16];
	int32_t ret = -1;
	int32_t retry;

	if (mutex_lock_interruptible(&ts->lock))
		return -ERESTARTSYS;

	if (!bTouchIsAwake) {
		NVT_ERR("Touch is already suspend, ignore 0x%02X cmd\n",
			cmd[1]);
		goto out;
	}

	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		goto out;
	}

	for (retry = 0; retry < NVT_CMD_RETRY; retry++) {
		CTP_SPI_WRITE(ts->client, cmd, len);
		msleep(20);

		memset(buf, 0xFF, sizeof(buf));
		buf[0] = EVENT_MAP_HOST_CMD;
		CTP_SPI_READ(ts->client, buf, ack_len);
		if (buf[1] == 0x00)
			break;
	}

	if (retry == NVT_CMD_RETRY) {
		NVT_ERR("send Cmd 0x%02X failed, buf[1]=0x%02X\n", cmd[1],
			buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X success, tried %d times\n", cmd[1],
			retry);
		ret = 0;
	}

out:
	mutex_unlock(&ts->lock);
	return ret;
}

int32_t nvt_cmd_store(uint8_t cmd)
{
	uint8_t buf[2] = { EVENT_MAP_HOST_CMD, cmd };

	return nvt_lenovo_send_cmd(buf, sizeof(buf), 2);
}

int32_t nvt_ext_cmd_store(uint8_t cmd, uint8_t value)
{
	uint8_t buf[3] = { EVENT_MAP_HOST_CMD, cmd, value };

	return nvt_lenovo_send_cmd(buf, sizeof(buf), 2);
}

/* Enable or disable pen sensing; type selects the pen protocol. */
int32_t nvt_support_pen_set(uint8_t state, uint8_t type)
{
	uint8_t buf[4] = { EVENT_MAP_HOST_CMD, NVT_CMD_PEN_MODE, state, type };

	return nvt_lenovo_send_cmd(buf, sizeof(buf), sizeof(buf));
}

/*
 * The stylus charger takes the pen off the digitizer while it charges,
 * and gives it back with the mode last set through /proc/support_pen.
 */
int nvt_support_pen_set_nfc(u8 on)
{
	if (!ts)
		return 0;

	if (on == 1) {
		ts->pen_charging = 0;
		if (ts->pen_state == 1)
			nvt_support_pen_set(1, ts->pen_type);
		else
			NVT_LOG("The pen no connect!");
	} else {
		nvt_support_pen_set(0, 0);
		ts->pen_charging = 1;
	}

	return 0;
}
EXPORT_SYMBOL(nvt_support_pen_set_nfc);

/* Palm rejection by panel orientation. */
static const uint8_t nvt_edge_reject_cmd[] = { 0xBA, 0xBB, 0xBD, 0xBC };
static const char * const nvt_edge_reject_text[] = {
	"Vertical Direction!(0 degree)",
	"Right Up Direction!(270 degree)",
	"Vertical reverse Direction!(180 degree)",
	"Left Up Direction!(90 degree)",
};

int32_t nvt_edge_reject_set(int32_t direction)
{
	if (direction < 0 || direction >= ARRAY_SIZE(nvt_edge_reject_cmd))
		return 0;

	return nvt_cmd_store(nvt_edge_reject_cmd[direction]);
}

int32_t nvt_edge_grid_zone_set(uint8_t p0, uint8_t p1, uint16_t p2,
			       uint16_t p3)
{
	uint8_t buf[9] = {
		EVENT_MAP_HOST_CMD, NVT_CMD_EDGE_GRID_ZONE, 0x01, p0, p1,
		p2 & 0xFF, p2 >> 8, p3 & 0xFF, p3 >> 8,
	};

	return nvt_lenovo_send_cmd(buf, sizeof(buf), sizeof(buf));
}

/* 0: off, 1 and 2: the FW's two game mode levels. */
int32_t nvt_game_mode_set(uint8_t mode)
{
	static const uint8_t value[] = { 0x00, 0x02, 0x01 };

	if (mode >= ARRAY_SIZE(value))
		return 0;

	return nvt_ext_cmd_store(NVT_CMD_GAME_MODE, value[mode]);
}

/* Re-apply the Lenovo settings after the FW was (re)loaded. */
void nvt_restore_cmd(void)
{
	NVT_LOG("%s++\n", __func__);

	nvt_edge_reject_set(ts->edge_reject);
	nvt_edge_grid_zone_set(ts->grid_zone[0], ts->grid_zone[1],
			       ts->grid_zone[2], ts->grid_zone[3]);
	nvt_game_mode_set(ts->game_mode);
	if (ts->pen_charging)
		nvt_support_pen_set(0, 0);
	else
		nvt_support_pen_set(ts->pen_state, ts->pen_type);

	NVT_LOG("%s--\n", __func__);
}

static void nvt_restore_cmd_func(struct work_struct *work)
{
	nvt_restore_cmd();
	ts->fw_pen_mode = 0;
}

void nvt_lenovo_queue_restore(struct workqueue_struct *wq)
{
	queue_work(wq, &ts->restore_cmd_work);
}

/*******************************************************
 * Stylus state and pen samples
 *******************************************************/
/*
 * Returns nonzero if the event was a stylus state report, which carries
 * no touch data.
 */
int32_t nvt_check_stylus_state(uint8_t input_id, uint8_t *data)
{
	if (input_id != DATA_PROTOCOL || data[2] != NVT_FUNCPAGE_STYLUS)
		return 0;

	switch (data[3]) {
	case NVT_STYLUS_DETECT:
		NVT_LOG("Detect stylus mode\n");
		input_report_key(ts->input_dev, KEY_NVT_STYLUS_DETECT, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_NVT_STYLUS_DETECT, 0);
		input_sync(ts->input_dev);
		return 1;
	case NVT_STYLUS_ENTER:
		ts->fw_pen_mode = 1;
		NVT_LOG("Enter PEN mode\n");
		return 1;
	case NVT_STYLUS_LEAVE:
		ts->fw_pen_mode = 0;
		NVT_LOG("Leave PEN mode\n");
		return 1;
	default:
		NVT_ERR("invalid state %d!\n", data[3]);
		return -1;
	}
}

/* One pen sample as read through /dev/lenovo_penraw. */
struct nvt_pen_point {
	uint8_t frame_no;
	uint8_t reserved[7];
	uint16_t tool;
	int8_t tilt_x;
	int8_t tilt_y;
	uint8_t reserved2[4];
	uint64_t x;
	uint64_t y;
	uint64_t pressure;
};

struct nvt_pen_report {
	uint8_t num;
	uint8_t reserved[7];
	struct nvt_pen_point points[NVT_PENRAW_REPORT_POINTS];
};

#define PENRAW_IOC_GET_REPORT	_IOR('P', 0x00, struct nvt_pen_report)

static struct nvt_pen_point pen_buffer[NVT_PENRAW_POINTS];
static struct nvt_pen_report pen_report;
static uint8_t pen_buffer_wp;
static uint8_t pen_frame_no;
static uint8_t pen_report_num;
static DEFINE_SPINLOCK(pen_buffer_lock);

/* Record a pen sample, in pixels; all zero when the pen is lifted. */
void nvt_lenovo_report(int32_t unused, uint32_t x, uint32_t y, int8_t tilt_x,
		       int8_t tilt_y, uint32_t pressure)
{
	struct nvt_pen_point *point;
	unsigned long flags;

	spin_lock_irqsave(&pen_buffer_lock, flags);
	point = &pen_buffer[pen_buffer_wp];
	memset(point, 0, sizeof(*point));
	point->frame_no = pen_frame_no++;
	point->tool = 1;
	point->tilt_x = tilt_x;
	point->tilt_y = tilt_y;
	point->x = x / NVT_SUPER_RESOLUTION_N;
	point->y = y / NVT_SUPER_RESOLUTION_N;
	point->pressure = pressure;
	if (pen_report_num < NVT_PENRAW_REPORT_POINTS)
		pen_report_num++;
	pen_buffer_wp = (pen_buffer_wp + 1) % NVT_PENRAW_POINTS;
	spin_unlock_irqrestore(&pen_buffer_lock, flags);
}
EXPORT_SYMBOL(nvt_lenovo_report);

/* Hand out the most recent samples, oldest first. */
static long penraw_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	unsigned long flags;
	uint8_t i, rp;

	if (cmd != PENRAW_IOC_GET_REPORT) {
		pr_err("unsupported command %d\n", cmd);
		return -EFAULT;
	}

	spin_lock_irqsave(&pen_buffer_lock, flags);
	memset(&pen_report, 0, sizeof(pen_report));
	pen_report.num = pen_report_num;
	rp = pen_report_num < NVT_PENRAW_REPORT_POINTS ? 0 :
	     (pen_buffer_wp + NVT_PENRAW_REPORT_POINTS) % NVT_PENRAW_POINTS;
	for (i = 0; i < pen_report_num; i++) {
		pen_report.points[i] = pen_buffer[rp];
		rp = (rp + 1) % NVT_PENRAW_POINTS;
	}
	spin_unlock_irqrestore(&pen_buffer_lock, flags);

	if (copy_to_user((void __user *)arg, &pen_report, sizeof(pen_report)))
		return -EFAULT;

	return 0;
}

static int penraw_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int penraw_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations penraw_fops = {
	.owner = THIS_MODULE,
	.open = penraw_open,
	.release = penraw_close,
	.unlocked_ioctl = penraw_ioctl,
	.compat_ioctl = penraw_ioctl,
};

static dev_t penraw_devt;
static struct cdev penraw_char_dev;
static struct class *penraw_char_dev_class;

int32_t nvt_lenovo_register(void)
{
	struct device *dev;
	int ret;

	ret = alloc_chrdev_region(&penraw_devt, 0, 1, "lenovo_penraw_driver");
	if (ret) {
		pr_err("failed to alloc_chrdev_region()\n");
		return -EINVAL;
	}

	cdev_init(&penraw_char_dev, &penraw_fops);
	ret = cdev_add(&penraw_char_dev, penraw_devt, 1);
	if (ret) {
		pr_err("failed to cdev_add()\n");
		goto err_unregister;
	}

	penraw_char_dev_class = class_create(THIS_MODULE, "lenovo_penraw");
	if (IS_ERR(penraw_char_dev_class)) {
		pr_err("class_create()\n");
		ret = PTR_ERR(penraw_char_dev_class);
		goto err_cdev_del;
	}

	dev = device_create(penraw_char_dev_class, NULL, penraw_devt, NULL,
			    "lenovo_penraw");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto err_class_destroy;
	}

	return 0;

err_class_destroy:
	class_destroy(penraw_char_dev_class);
err_cdev_del:
	cdev_del(&penraw_char_dev);
err_unregister:
	unregister_chrdev_region(penraw_devt, 1);
	return -EINVAL;
}

void nvt_lenovo_unregister(void)
{
	device_destroy(penraw_char_dev_class, penraw_devt);
	class_destroy(penraw_char_dev_class);
	cdev_del(&penraw_char_dev);
	unregister_chrdev_region(penraw_devt, 1);
}

/*******************************************************
 * Charger mode
 *******************************************************/
#define NVT_CMD_CHARGER_OFF		0x51
#define NVT_CMD_CHARGER_ON		0x53

/* Tune the controller against USB charger noise. */
static int32_t nvt_set_charger(uint8_t on)
{
	uint8_t buf[2] = { EVENT_MAP_HOST_CMD };
	int32_t ret;

	NVT_LOG("nvt set charger: %d\n", on);
	msleep(20);

	mutex_lock(&ts->lock);
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("nvt Set event buffer index fail!\n");
		goto out;
	}

	buf[1] = on ? NVT_CMD_CHARGER_ON : NVT_CMD_CHARGER_OFF;
	ret = CTP_SPI_WRITE(ts->client, buf, sizeof(buf));
	if (ret < 0)
		NVT_ERR("nvt Write set charger command fail!\n");
	else
		NVT_LOG("nvt set charger %s cmd succeeded\n", on ? "on" : "off");
out:
	mutex_unlock(&ts->lock);
	return ret;
}

static void nvt_charger_notify_work(struct work_struct *work)
{
	switch (ts->usb_plug_status) {
	case 1:
		NVT_LOG("USB plug in");
		nvt_set_charger(1);
		break;
	case 0:
		NVT_LOG("USB plug out");
		nvt_set_charger(0);
		break;
	default:
		NVT_LOG("Charger flag:%d not currently required!\n",
			ts->usb_plug_status);
		break;
	}
}

static int nvt_usb_present(int *present)
{
	union power_supply_propval val = { 0 };
	struct power_supply *usb_psy;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy)
		return -ENODEV;

	ret = power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	power_supply_put(usb_psy);
	if (ret < 0)
		return ret;

	*present = val.intval;
	return 0;
}

static int nvt_charger_notifier_callback(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	int present, ret;

	if (ts->fw_loaded != 1 || event != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = nvt_usb_present(&present);
	if (ret == -ENODEV) {
		NVT_ERR("Couldn't get usbpsy\n");
		return -EINVAL;
	}
	if (ret < 0) {
		NVT_ERR("Couldn't get POWER_SUPPLY_PROP_PRESENT rc=%d\n", ret);
		return ret;
	}

	if (present != ts->usb_plug_status) {
		NVT_LOG("usb_plug_status = %d\n", present);
		ts->usb_plug_status = present;
		if (bTouchIsAwake && ts->charger_wq)
			queue_work(ts->charger_wq, &ts->charger_work);
	}

	return 0;
}

/* A freshly loaded FW needs the charger state again. */
void nvt_lenovo_fw_loaded(void)
{
	if (ts->charger_wq)
		queue_work(ts->charger_wq, &ts->charger_work);
}

int32_t nvt_charger_init(void)
{
	int present, ret;

	ts->usb_plug_status = -1;
	ts->charger_wq = alloc_workqueue("%s", WQ_UNBOUND | WQ_MEM_RECLAIM |
					 WQ_HIGHPRI, 1, "nvt_charger_wq");
	if (!ts->charger_wq) {
		NVT_ERR("allocate nvt_charger_notify_wq failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&ts->charger_work, nvt_charger_notify_work);

	ts->charger_notif.notifier_call = nvt_charger_notifier_callback;
	ret = power_supply_reg_notifier(&ts->charger_notif);
	if (ret) {
		NVT_ERR("Unable to register charger_notifier: %d\n", ret);
		ts->charger_notif.notifier_call = NULL;
	}

	ret = nvt_usb_present(&present);
	if (ret < 0) {
		NVT_ERR("Couldn't get POWER_SUPPLY_PROP_PRESENT rc=%d\n", ret);
	} else {
		ts->usb_plug_status = present;
		NVT_LOG("boot check usb_plug_status = %d\n", present);
	}

	return 0;
}

void nvt_charger_deinit(void)
{
	if (ts->charger_notif.notifier_call) {
		power_supply_unreg_notifier(&ts->charger_notif);
		ts->charger_notif.notifier_call = NULL;
	}
	if (ts->charger_wq) {
		destroy_workqueue(ts->charger_wq);
		ts->charger_wq = NULL;
	}
}

/*******************************************************
 * Double-tap wake and the keyboard cover
 *******************************************************/
/* input_dev->event: EV_SYN/SYN_CONFIG switches double-tap wake. */
int nvt_gesture_switch(struct input_dev *dev, unsigned int type,
		       unsigned int code, int value)
{
	NVT_LOG("%s:start\n", __func__);

	if (type == EV_SYN && code == SYN_CONFIG) {
		if (value == 1) {
			nvt_gesture_flag = 1;
			NVT_LOG("gesture enabled:%d", 1);
		} else if (value == 0) {
			nvt_gesture_flag = 0;
			NVT_LOG("gesture disabled:%d", 0);
		}
	}

	return 0;
}

static int nvt_hall_chip_match(struct gpio_chip *chip, void *data)
{
	return chip->label && !strcmp(chip->label, data);
}

/* Raw cover state: 0 while the cover is closed over the screen. */
static int nvt_hall_get(void)
{
	if (!gpio_is_valid(nvt_hall_gpio))
		return 1;

	return gpiod_get_raw_value(gpio_to_desc(nvt_hall_gpio));
}

/*
 * While double-tap wake keeps the controller scanning with the screen
 * off, closing the cover would wake the tablet: put the controller to
 * sleep instead. It is woken again by the next screen on.
 */
static void nvt_hall_check_func(struct work_struct *work)
{
	unsigned int elapsed = jiffies_to_msecs(jiffies - nvt_hall_timer);
	int hall = nvt_hall_get();
	uint8_t buf[2] = { EVENT_MAP_HOST_CMD, NVT_CMD_DEEP_SLEEP };

	if (elapsed > NVT_HALL_CHECK_PERIOD_MS && !bTouchIsAwake &&
	    hall == hall_status) {
		if (ts->dev_pm_suspend) {
			nvt_hall_pm = 1;
			NVT_LOG("The status of pm is suspend");
		} else if (nvt_hall_pm_status) {
			NVT_LOG("The status of double click has changed");
			hall_status = 1;
		} else if (!hall) {
			NVT_LOG("The keyboard close!");
			CTP_SPI_WRITE(ts->client, buf, sizeof(buf));
			hall_status = 1;
			nvt_hall_timer = jiffies;
		} else {
			NVT_LOG("The keyboard open!");
			hall_status = 0;
		}
	}

	queue_delayed_work(nvt_hall_check_wq, &nvt_hall_check_work,
			   msecs_to_jiffies(NVT_HALL_CHECK_PERIOD_MS));
}

/* Screen off: whether to arm double-tap wake (the cover must be open). */
bool nvt_lenovo_gesture_armed(void)
{
	return nvt_gesture_flag && nvt_hall_get();
}

void nvt_lenovo_hall_start(void)
{
	nvt_hall_pm = 0;
	nvt_hall_pm_status = 0;
	queue_delayed_work(nvt_hall_check_wq, &nvt_hall_check_work,
			   msecs_to_jiffies(NVT_HALL_CHECK_PERIOD_MS));
}

/* Screen on: the controller is reloaded, stop watching the cover. */
void nvt_lenovo_hall_stop(void)
{
	cancel_delayed_work_sync(&nvt_hall_check_work);
}

/* System resume: the cover was closed while SPI was unavailable. */
void nvt_lenovo_pm_resume(void)
{
	uint8_t buf[2] = { EVENT_MAP_HOST_CMD, NVT_CMD_DEEP_SLEEP };

	if (nvt_hall_pm && nvt_gesture_flag && !nvt_hall_pm_status &&
	    !nvt_hall_get()) {
		NVT_LOG("The keyboard still close!");
		CTP_SPI_WRITE(ts->client, buf, sizeof(buf));
		nvt_hall_pm_status = 1;
	}
}

/*******************************************************
 * /proc interface for Lenovo's touchscreen HAL
 *******************************************************/
static ssize_t nvt_proc_copy(char *kbuf, size_t size, const char __user *buf,
			     size_t count)
{
	if (count == 0 || count >= size)
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	return count;
}

static ssize_t gesture_control_write(struct file *file,
				     const char __user *buf, size_t count,
				     loff_t *ppos)
{
	char c;

	if (count == 0)
		return 0;
	if (get_user(c, buf))
		return -EFAULT;
	nvt_gesture_flag = (c == '1');

	return count;
}

static int gesture_control_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", nvt_gesture_flag);
	return 0;
}

static int gesture_control_open(struct inode *inode, struct file *file)
{
	return single_open(file, gesture_control_show, NULL);
}

static const struct file_operations gesture_control_fops = {
	.owner = THIS_MODULE,
	.open = gesture_control_open,
	.read = seq_read,
	.write = gesture_control_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t nvt_edge_reject_store(struct file *file,
				     const char __user *buf, size_t count,
				     loff_t *ppos)
{
	char kbuf[2] = { 0 };
	uint8_t direction;
	int ret;

	if (count == 0)
		return -EINVAL;
	if (copy_from_user(kbuf, buf, 1))
		return -EINVAL;
	ret = kstrtou8(kbuf, 16, &direction);
	if (ret < 0)
		return ret;

	ts->edge_reject = direction;
	nvt_edge_reject_set(direction);

	return count;
}

static int nvt_edge_reject_show(struct seq_file *m, void *v)
{
	if (ts->edge_reject >= 0 &&
	    ts->edge_reject < ARRAY_SIZE(nvt_edge_reject_text))
		seq_printf(m, "%s\n", nvt_edge_reject_text[ts->edge_reject]);
	else
		seq_puts(m, "Not Support!\n");

	return 0;
}

static int nvt_edge_reject_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_edge_reject_show, NULL);
}

static const struct file_operations nvt_edge_reject_fops = {
	.owner = THIS_MODULE,
	.open = nvt_edge_reject_open,
	.read = seq_read,
	.write = nvt_edge_reject_store,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t nvt_edge_grid_zone_store(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	char kbuf[16];
	int p[4] = { 0 };
	ssize_t ret;

	ret = nvt_proc_copy(kbuf, sizeof(kbuf), buf, count);
	if (ret < 0)
		return ret;

	NVT_LOG("buf=%s\n", kbuf);
	sscanf(kbuf, "%d,%d,%d,%d", &p[0], &p[1], &p[2], &p[3]);
	ts->grid_zone[0] = (uint8_t)p[0];
	ts->grid_zone[1] = (uint8_t)p[1];
	ts->grid_zone[2] = (uint16_t)p[2];
	ts->grid_zone[3] = (uint16_t)p[3];
	NVT_LOG("cmd_parm = %d,%d,%d,%d\n", ts->grid_zone[0], ts->grid_zone[1],
		ts->grid_zone[2], ts->grid_zone[3]);
	nvt_edge_grid_zone_set(ts->grid_zone[0], ts->grid_zone[1],
			       ts->grid_zone[2], ts->grid_zone[3]);

	return count;
}

static int nvt_edge_grid_zone_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Edge Grid Zone %d,%d,%d,%d\n", ts->grid_zone[0],
		   ts->grid_zone[1], ts->grid_zone[2], ts->grid_zone[3]);
	return 0;
}

static int nvt_edge_grid_zone_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_edge_grid_zone_show, NULL);
}

static const struct file_operations nvt_edge_grid_zone_fops = {
	.owner = THIS_MODULE,
	.open = nvt_edge_grid_zone_open,
	.read = seq_read,
	.write = nvt_edge_grid_zone_store,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t nvt_game_mode_store(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	char kbuf[2] = { 0 };
	uint8_t mode;
	int ret;

	if (count == 0)
		return -EINVAL;
	if (copy_from_user(kbuf, buf, 1))
		return -EINVAL;
	ret = kstrtou8(kbuf, 16, &mode);
	if (ret < 0)
		return ret;

	NVT_LOG("moto_apk_state %d!\n", mode);
	ts->game_mode = mode;
	nvt_game_mode_set(mode);

	return count;
}

static int nvt_game_mode_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", ts->game_mode);
	return 0;
}

static int nvt_game_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_game_mode_show, NULL);
}

static const struct file_operations nvt_game_mode_fops = {
	.owner = THIS_MODULE,
	.open = nvt_game_mode_open,
	.read = seq_read,
	.write = nvt_game_mode_store,
	.llseek = seq_lseek,
	.release = single_release,
};

/* "<enable>,<type>": type 1 is Lenovo's pen C, 0 pen B. */
static ssize_t nvt_support_pen_store(struct file *file,
				     const char __user *buf, size_t count,
				     loff_t *ppos)
{
	char kbuf[16];
	int state = 0, type = 0;
	ssize_t ret;

	ret = nvt_proc_copy(kbuf, sizeof(kbuf), buf, count);
	if (ret < 0)
		return ret;

	NVT_LOG("buf=%s\n", kbuf);
	sscanf(kbuf, "%d,%d", &state, &type);
	ts->pen_state = state;
	ts->pen_type = type;
	NVT_LOG("support pen state %d, %d!\n", ts->pen_state, ts->pen_type);

	if (ts->pen_charging) {
		NVT_LOG("The pen is chargering!");
	} else {
		NVT_LOG("The pen is ready!");
		nvt_support_pen_set(ts->pen_state, ts->pen_type);
	}

	return count;
}

static int nvt_support_pen_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Pen state %d!\n", ts->pen_state);
	return 0;
}

static int nvt_support_pen_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_support_pen_show, NULL);
}

static const struct file_operations nvt_support_pen_fops = {
	.owner = THIS_MODULE,
	.open = nvt_support_pen_open,
	.read = seq_read,
	.write = nvt_support_pen_store,
	.llseek = seq_lseek,
	.release = single_release,
};

static int nvt_get_fw_pen_state_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", ts->fw_pen_mode);
	return 0;
}

static int nvt_get_fw_pen_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_get_fw_pen_state_show, NULL);
}

static const struct file_operations nvt_get_fw_pen_state_fops = {
	.owner = THIS_MODULE,
	.open = nvt_get_fw_pen_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int nvt_fw_status_show(struct seq_file *m, void *v)
{
	uint8_t buf[6] = { EVENT_MAP_RESET_COMPLETE };

	if (mutex_lock_interruptible(&ts->lock))
		return -ERESTARTSYS;

	NVT_LOG("++\n");
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);
	CTP_SPI_READ(ts->client, buf, sizeof(buf));
	mutex_unlock(&ts->lock);
	NVT_LOG("--\n");

	seq_printf(m, "%02X\n", buf[1]);
	return 0;
}

static int nvt_fw_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_fw_status_show, NULL);
}

static const struct file_operations nvt_fw_status_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct {
	const char *name;
	umode_t mode;
	const struct file_operations *fops;
} nvt_lenovo_procs[] = {
	{ "gesture_control", 0664, &gesture_control_fops },
	{ "nvt_fw_status", 0444, &nvt_fw_status_fops },
	{ "panel_direction", 0644, &nvt_edge_reject_fops },
	{ "edge_grid_zone", 0644, &nvt_edge_grid_zone_fops },
	{ "game_mode", 0666, &nvt_game_mode_fops },
	{ "support_pen", 0666, &nvt_support_pen_fops },
	{ "fw_pen_state", 0664, &nvt_get_fw_pen_state_fops },
};

int32_t nvt_lenovo_proc_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nvt_lenovo_procs); i++) {
		if (!proc_create(nvt_lenovo_procs[i].name,
				 nvt_lenovo_procs[i].mode, NULL,
				 nvt_lenovo_procs[i].fops)) {
			NVT_ERR("create proc/%s Failed!\n",
				nvt_lenovo_procs[i].name);
			while (--i >= 0)
				remove_proc_entry(nvt_lenovo_procs[i].name,
						  NULL);
			return -ENOMEM;
		}
		NVT_LOG("create proc/%s Succeeded!\n",
			nvt_lenovo_procs[i].name);
	}

	return 0;
}

void nvt_lenovo_proc_deinit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nvt_lenovo_procs); i++)
		remove_proc_entry(nvt_lenovo_procs[i].name, NULL);
}

/*******************************************************
 * Setup
 *******************************************************/
int32_t nvt_lenovo_init(void)
{
	struct gpio_chip *tlmm;

	INIT_WORK(&ts->restore_cmd_work, nvt_restore_cmd_func);

	/*
	 * Only the TB371FC (NT36532 panel) has the keyboard cover hall sensor
	 * on this GPIO; elsewhere the line means something else, and reading
	 * it as a closed cover would keep double-tap wake from arming.
	 */
	if (strstr(saved_command_line, "nt36532")) {
		tlmm = gpiochip_find(NVT_HALL_TLMM_LABEL, nvt_hall_chip_match);
		if (tlmm)
			nvt_hall_gpio = tlmm->base + NVT_HALL_TLMM_GPIO;
		else
			NVT_ERR("no TLMM, keyboard cover state unavailable\n");
	}

	INIT_DELAYED_WORK(&nvt_hall_check_work, nvt_hall_check_func);
	nvt_hall_check_wq = alloc_workqueue("%s", WQ_MEM_RECLAIM, 1,
					    "nvt_hall_check_wq");
	if (!nvt_hall_check_wq) {
		NVT_ERR("nvt_hall_check_wq create workqueue failed\n");
		return -ENOMEM;
	}

	return 0;
}

/* The controller is up: let the stylus charger drive pen sensing. */
void nvt_lenovo_ready(void)
{
	ts->edge_reject = 0;
	ts->pen_charging = 0;
	check_tp_state(1);
}

void nvt_lenovo_deinit(void)
{
	check_tp_state(0);
	if (nvt_hall_check_wq) {
		cancel_delayed_work_sync(&nvt_hall_check_work);
		destroy_workqueue(nvt_hall_check_wq);
		nvt_hall_check_wq = NULL;
	}
	cancel_work_sync(&ts->restore_cmd_work);
}
