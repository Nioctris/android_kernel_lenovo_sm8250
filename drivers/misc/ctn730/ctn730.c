// SPDX-License-Identifier: GPL-2.0
/*
 * NXP CTN730 NFC wireless charger driver for the Lenovo stylus.
 *
 * Based on Lenovo's driver for the TB-Q706F, with the changes Lenovo made
 * for the TB371FC: the charger tells the touch controller to stop sensing
 * the pen while it charges and to resume when it is lifted, short "lost"
 * blips are filtered out, and a charge that never completes or a charger
 * stuck reporting the pen as present is recovered with a hard reset.
 */

#include <linux/alarmtimer.h>
#include <linux/ctn730.h>
#include <linux/input/nvt_lenovo.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>

#define STATUS_OK			0x00

#define MT_MASK				GENMASK(7, 6)
#define INSTRUCTION_MASK		GENMASK(5, 0)
#define RSP_BIT				BIT(6)
#define EVENT_BIT			BIT(7)

enum message_type {
	unkown = 0,
	Command,
	Response,
	Event,
};

enum instruction {
	HOST_RESET = 0x00,
	HOST_CTRL_WRITE_PARAMETER = 0x01,
	HOST_CTRL_READ_PARAMETER = 0x02,
	HOST_CTRL_DL_OPEN_SESSION = 0x03,
	HOST_CTRL_DL_COMMIT_SESSION = 0x04,
	HOST_CTRL_DL_WRITE_FLASH = 0x05,
	HOST_CTRL_BIST = 0x06,
	HOST_CTRL_WRITE_SESSION_PARAMETER = 0x07,
	HOST_CTRL_READ_SESSION_PARAMETER = 0x08,
	CHG_ENABLE = 0x10,
	CHG_DISABLE = 0x11,
	CHG_DEVICE_STATE = 0x12,
	CHG_CHARGING_STATE = 0x14,
	CHG_CHARGING_INFO = 0x15,
	APP_CTRL_CHN_ENABLE = 0x20,
	APP_CTRL_CHN_DISABLE = 0x21,
	APP_CTRL_CHN_STATE = 0x22,
	APP_CTRL_CHN_SEND_DATA = 0x23,
	APP_CTRL_CHN_RECIVE_DATA = 0x24,
};

enum ctn_status {
	status_unkown = 0,
	download_mode,
	initialized,
	detecting,
	detected,
	deactived,
	lost,
	version_missmatch,
	host_request_dischg,
	rf_on,
	rf_off,
	mac_getted,
	app_chn_enabled,
	app_chn_disabled,
	app_chn_connected,
	app_chn_reciced,
	need_to_app_chmn_enable,
	need_to_host_ctrl_enable_send_charging_info,
	need_to_host_ctrl_disable_send_charging_info,
	need_to_rf_on,
	need_to_rf_off,
	dl_open_session_ok,
	dl_flash_ok,
	dl_commit_ok,
	update_firmware_err,
	need_dowload_firmware,
	bist_err,
};

enum charge_status {
	charge_unkown = 0,
	charging,
	full,
	not_charging,
};

enum bist_id {
	BIST_RF_ON = 0x01,
	BIST_RF_OFF = 0x02,
	BIST_ACTIVATION = 0x04,
	BIST_READ_NTAG_BLOCKS = 0x09,
};

enum ext_cmd {
	ext_cmd_rf_on = 0x01,
	ext_cmd_update_firmware = 0x02,
};

#define ctn730_err(fmt, ...) \
	pr_err("ctn730: %s: " fmt, __func__, ##__VA_ARGS__)
#define ctn730_dbg(fmt, ...) \
	pr_debug("ctn730: %s: " fmt, __func__, ##__VA_ARGS__)

#define MAX_BUFFER_SIZE			256
#define DEV_DETC_INTERVAL		500
#define PEN_PARAM_LEN			32
#define CMD_DELAY_TIME			0
#define MAC_DELAY_TIME			50
#define STATIC_CHECK_DELAY_TIME		500
#define DETECT_CHECK_DELAY_TIME		2000
/* A "lost" followed by a new detection within this is a glitch. */
#define PEN_LOST_CHECK_DELAY_TIME	60
#define PEN_LOST_GLITCH_NS		(120 * NSEC_PER_MSEC)
/* Reset the charger if a charge has not finished after 90 minutes ... */
#define CHARGING_TIMEOUT_NS		(5400ULL * NSEC_PER_SEC)
/* ... or if it keeps reporting the pen as present for 5 minutes. */
#define PRESENT_STUCK_TIMEOUT_S		300
#define HARD_RESET_OFF_TIME		2000

struct info {
	int level;
	int attached;
	int charge_state;
	char mac[6];
	char pen_uid[8];
};

struct firmware_info {
	char *data;
	u32 user_ee_size;
	u32 user_app_size;
	u32 size;
	int index;
	u32 user_app_addr;
	u32 user_ee_addr;
	u32 crc;
	char version[2];
};

struct ctn730_dev {
	struct device *dev;
	struct i2c_client *client;
	struct miscdevice ctn730_device;
	struct info pen_info;
	struct firmware_info firmware;
	struct blocking_notifier_head nh;

	int ext_cmd;
	int ven_gpio;
	int rst_gpio;
	bool rst_requested;
	int irq_gpio;
	int irq;
	int wake_count;
	bool irq_enabled;
	bool have_mac;
	bool try_to_cmd;
	bool host_ctrl_flag;
	bool enabled;
	bool hardwareinfo;
	bool suspend;
	bool initialized;
	bool level_changed;
	int app_data_ok;
	spinlock_t irq_enabled_lock;
	struct delayed_work set_work;
	struct delayed_work reset_work;
	struct delayed_work update_firmware_work;
	struct delayed_work rf_on_work;
	struct delayed_work check_static_charge_work;
	struct delayed_work check_pen_lost_work;
	struct alarm charging_alarm;
	ktime_t lost_time;
	ktime_t detect_time;
	s64 lost_interval_ms;
	struct notifier_block drm_notifier;
	struct mutex bist_mutex;
	struct mutex report_info_mutex;
	int status;
	int bist_id;
	int report_info_id;
	bool report_info;
	bool need_report_info;
	char pen_param[PEN_PARAM_LEN];
	int pen_param_count;
	char version[2];
	ktime_t present_since;
};

static const char * const charge_state_text[] = {
	"Unknown",
	"Charging",
	"Full",
	"Notcharging",
};

static const char * const status_text[] = {
	"status_unkown",
	"download_mode",
	"initialized",
	"detecting",
	"detected",
	"deactived",
	"lost",
	"version_missmatch",
	"host_request_dischg",
	"rf_on",
	"rf_off",
	"mac_getted",
	"app_chn_enabled",
	"app_chn_disabled",
	"app_chn_connected",
	"app_chn_reciced",
	"need_to_app_chmn_enable",
	"need_to_host_ctrl_enable_send_charging_info",
	"need_to_host_ctrl_disable_send_charging_info",
	"need_to_rf_on",
	"need_to_rf_off",
	"dl_open_session_ok",
	"dl_flash_ok",
	"dl_commit_ok",
	"update_firmware_err",
	"need_dowload_firmware",
	"bist_err",
};

static struct ctn730_dev *ctn730;
static struct i2c_client *client_address;
static struct drm_panel *ctn730_panel;
/* The touch controller is ready to take pen mode requests. */
static int tp_state;
/* Suppresses the uevent for the reset event of a hard reset. */
static bool is_hard_reset;

int is_stylus_attached(void)
{
	return ctn730 ? ctn730->pen_info.attached : 0;
}
EXPORT_SYMBOL_GPL(is_stylus_attached);

int ctn730_notifier_register(struct notifier_block *nb)
{
	if (!ctn730)
		return -ENODEV;
	return blocking_notifier_chain_register(&ctn730->nh, nb);
}
EXPORT_SYMBOL_GPL(ctn730_notifier_register);

int ctn730_notifier_unregister(struct notifier_block *nb)
{
	if (!ctn730)
		return -ENODEV;
	return blocking_notifier_chain_unregister(&ctn730->nh, nb);
}
EXPORT_SYMBOL_GPL(ctn730_notifier_unregister);

static int ctn730_notifier_call_chain(struct ctn730_dev *ctn730_dev,
				      unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&ctn730_dev->nh, val, v);
}

static void ctn730_send_uevent(struct ctn730_dev *ctn730_dev)
{
	kobject_uevent(&ctn730_dev->ctn730_device.this_device->kobj,
		       KOBJ_CHANGE);
}

/*
 * The touch driver reports here when it is up. From then on the pen is
 * kept off the digitizer while it sits on the charger.
 */
int check_tp_state(int ready)
{
	struct ctn730_dev *ctn730_dev;

	ctn730_dev = client_address ? i2c_get_clientdata(client_address) : NULL;
	if (!ctn730_dev) {
		ctn730_err("failed to get CTN730\n");
		return 1;
	}

	if (ready & 1) {
		tp_state = 1;
		nvt_support_pen_set_nfc(ctn730_dev->pen_info.attached != 1);
		ctn730_err("TP is ready!");
	} else {
		tp_state = 0;
		ctn730_err("TP is not ready!");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(check_tp_state);

/**********************************************************
 * private functions
 **********************************************************/
static void ctn730_schedule_ext_work(struct ctn730_dev *ctn730_dev,
				     unsigned long delay)
{
	switch (ctn730_dev->ext_cmd) {
	case ext_cmd_rf_on:
		schedule_delayed_work(&ctn730_dev->rf_on_work, delay);
		break;
	case ext_cmd_update_firmware:
		schedule_delayed_work(&ctn730_dev->update_firmware_work, delay);
		break;
	default:
		schedule_delayed_work(&ctn730_dev->set_work, delay);
		break;
	}
}

static int ctn730_i2c_send(struct ctn730_dev *ctn730_dev, char *data,
			   int count)
{
	int ret = count;

	if (ctn730_dev == NULL) {
		ctn730_err("no ctn730_dev\n");
		return -ENODEV;
	}

	ctn730_err("%*ph\n", count, data);
	if (!ctn730_dev->enabled)
		return ret;

	ret = i2c_master_send(ctn730_dev->client, data, count);
	if (ret == count)
		return ret;

	ctn730_err("%s : i2c_master_send returned %d\n", __func__, ret);
	if (ret == -ENOTCONN) {
		msleep(1);
		ret = i2c_master_send(ctn730_dev->client, data, count);
		if (ret == count) {
			ctn730_err("%s : i2c_master_send retried ok\n",
				   __func__);
			return ret;
		}
		ctn730_schedule_ext_work(ctn730_dev, msecs_to_jiffies(1));
	} else {
		ctn730_schedule_ext_work(ctn730_dev, msecs_to_jiffies(100));
	}

	return -EIO;
}

static void ctn730_hard_reset(struct ctn730_dev *ctn730_dev)
{
	if (!gpio_is_valid(ctn730_dev->rst_gpio))
		return;

	gpio_direction_output(ctn730_dev->rst_gpio, 0);
	msleep(5);
	gpio_direction_output(ctn730_dev->rst_gpio, 1);
	ctn730_dev->initialized = 0;
}

static void ctn730_enable(struct ctn730_dev *ctn730_dev)
{
	ctn730_err("ctn730_enable\n");
	if (ctn730_dev->enabled)
		return;

	if (gpio_is_valid(ctn730_dev->ven_gpio))
		gpio_direction_output(ctn730_dev->ven_gpio, 1);
	ctn730_dev->enabled = 1;
	enable_irq(ctn730_dev->irq);
}

static void ctn730_disable(struct ctn730_dev *ctn730_dev)
{
	ctn730_err("ctn730_disenable\n");
	if (!ctn730_dev->enabled)
		return;

	disable_irq(ctn730_dev->irq);
	if (gpio_is_valid(ctn730_dev->ven_gpio)) {
		gpio_set_value(ctn730_dev->ven_gpio, 0);
		ctn730_dev->initialized = 0;
	} else {
		ctn730_err("invalid gpio ven_gpio,disable failed\n");
	}
	ctn730_dev->enabled = 0;
}

/* Power-cycle the charger, e.g. when it got stuck. */
static void ctn730_dev_hard_reset(struct ctn730_dev *ctn730_dev)
{
	ctn730_err("ctn730_dev_hard_reset\n");
	ctn730_disable(ctn730_dev);
	msleep(HARD_RESET_OFF_TIME);
	ctn730_enable(ctn730_dev);
}

static void ctn730_reset_init(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = HOST_RESET;
	cmd[2] = 0x00;
	len++;
	cmd[1] = len;
	if (ctn730_i2c_send(ctn730_dev, cmd, len + 2) >= 0)
		ctn730_dev->initialized = 1;
}

static void ctn730_get_mac(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = HOST_CTRL_BIST;
	cmd[2] = BIST_READ_NTAG_BLOCKS;
	len++;
	cmd[3] = 0x10;
	len++;
	cmd[4] = 1;
	len++;
	cmd[1] = len;
	ctn730_err("%*ph\n", len + 2, cmd);
	mutex_lock(&ctn730_dev->bist_mutex);
	if (ctn730_i2c_send(ctn730_dev, cmd, len + 2) >= 0)
		ctn730_dev->bist_id = BIST_READ_NTAG_BLOCKS;
	mutex_unlock(&ctn730_dev->bist_mutex);
}

static void ctn730_enable_chg(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = CHG_ENABLE;
	cmd[3] = (char)(DEV_DETC_INTERVAL >> 8);
	len++;
	cmd[2] = (char)(DEV_DETC_INTERVAL);
	len++;
	cmd[1] = len;
	ctn730_i2c_send(ctn730_dev, cmd, len + 2);
}

static void ctn730_disable_chg(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };

	cmd[0] = CHG_DISABLE;
	cmd[1] = 0;
	ctn730_i2c_send(ctn730_dev, cmd, 2);
}

static void ctn730_enable_app_channel(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };

	cmd[0] = APP_CTRL_CHN_ENABLE;
	cmd[1] = 0;
	ctn730_i2c_send(ctn730_dev, cmd, 2);
}

static void ctn730_disable_app_channel(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };

	cmd[0] = APP_CTRL_CHN_DISABLE;
	cmd[1] = 0;
	ctn730_i2c_send(ctn730_dev, cmd, 2);
}

static void ctn730_set_charging_info(struct ctn730_dev *ctn730_dev, bool on)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = HOST_CTRL_WRITE_SESSION_PARAMETER;
	cmd[2] = 0x00;
	len++;
	cmd[3] = on;
	len++;
	cmd[1] = len;
	if (ctn730_i2c_send(ctn730_dev, cmd, len + 2) >= 0) {
		mutex_lock(&ctn730_dev->report_info_mutex);
		ctn730_dev->report_info_id = on;
		mutex_unlock(&ctn730_dev->report_info_mutex);
	}
}

static void ctn730_app_set_param(struct ctn730_dev *ctn730_dev)
{
	char cmd[260] = { 0 };
	int len = 0;

	cmd[0] = APP_CTRL_CHN_SEND_DATA;
	cmd[2] = 0x01;
	len++;
	memcpy(cmd + 3, ctn730_dev->pen_param, ctn730_dev->pen_param_count);
	len += ctn730_dev->pen_param_count;
	cmd[1] = len;
	ctn730_i2c_send(ctn730_dev, cmd, len + 2);
}

static int ctn730_bist(struct ctn730_dev *ctn730_dev, int id)
{
	char cmd[10] = { 0 };
	int len = 0;
	int ret;

	cmd[0] = HOST_CTRL_BIST;
	cmd[2] = id;
	len++;
	cmd[1] = len;
	mutex_lock(&ctn730_dev->bist_mutex);
	ret = ctn730_i2c_send(ctn730_dev, cmd, len + 2);
	if (ret >= 0)
		ctn730_dev->bist_id = id;
	mutex_unlock(&ctn730_dev->bist_mutex);

	return ret;
}

static int ctn730_dl_open(struct ctn730_dev *ctn730_dev)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = HOST_CTRL_DL_OPEN_SESSION;
	cmd[2] = ctn730_dev->firmware.version[0];
	len++;
	cmd[3] = ctn730_dev->firmware.version[1];
	len++;
	cmd[1] = len;

	return ctn730_i2c_send(ctn730_dev, cmd, len + 2);
}

static int ctn730_dl_commit(struct ctn730_dev *ctn730_dev, u32 crc)
{
	char cmd[10] = { 0 };
	int len = 0;

	cmd[0] = HOST_CTRL_DL_COMMIT_SESSION;
	cmd[5] = (crc & 0xff000000) >> 24;
	len++;
	cmd[4] = (crc & 0xff0000) >> 16;
	len++;
	cmd[3] = (crc & 0xff00) >> 8;
	len++;
	cmd[2] = crc & 0xff;
	len++;
	cmd[1] = len;

	return ctn730_i2c_send(ctn730_dev, cmd, len + 2);
}

static int ctn730_dl_flash(struct ctn730_dev *ctn730_dev, char *data,
			   int addr)
{
	char cmd[150] = { 0 };
	int len = 0;

	cmd[0] = HOST_CTRL_DL_WRITE_FLASH;
	cmd[2] = addr & 0xff;
	len++;
	cmd[3] = (addr & 0xff00) >> 8;
	len++;
	cmd[4] = (addr & 0xff0000) >> 16;
	len++;
	memcpy(cmd + 5, data, 128);
	len += 128;
	cmd[1] = len;

	return ctn730_i2c_send(ctn730_dev, cmd, len + 2);
}

/* The pen left the charger (or never arrived): report it as detached. */
static void ctn730_clear_pen(struct ctn730_dev *ctn730_dev)
{
	ctn730_dev->pen_info.attached = 0;
	ctn730_dev->pen_info.level = -1;
	ctn730_dev->pen_info.charge_state = charge_unkown;
	ctn730_dev->have_mac = 0;
}

static void ctn730_clear_ids(struct ctn730_dev *ctn730_dev)
{
	memset(ctn730_dev->pen_info.mac, 0, sizeof(ctn730_dev->pen_info.mac));
	memset(ctn730_dev->pen_info.pen_uid, 0,
	       sizeof(ctn730_dev->pen_info.pen_uid));
}

static void ctn730_parse_event(struct ctn730_dev *ctn730_dev, int instruction,
			       char *data, int count, char cmd)
{
	char version[6];
	int charge_state = charging;
	int level;
	int ret;

	switch (instruction) {
	case HOST_RESET:
		if (data[0] == 0) {
			ctn730_dev->status = initialized;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
			ctn730_dev->version[0] = data[1];
			ctn730_dev->version[1] = data[2];
			ctn730_clear_pen(ctn730_dev);
			if (tp_state == 1) {
				ctn730_err("initialized :open pen mode\n");
				nvt_support_pen_set_nfc(1);
			}
			ctn730_clear_ids(ctn730_dev);
			if (!ctn730_dev->hardwareinfo) {
				snprintf(version, sizeof(version), "%02x%02x",
					 ctn730_dev->version[0],
					 ctn730_dev->version[1]);
				ctn730_dev->hardwareinfo = 1;
			}
			ctn730_notifier_call_chain(ctn730_dev,
				ctn730_dev->pen_info.attached, NULL);
			if (!is_hard_reset)
				ctn730_send_uevent(ctn730_dev);
			is_hard_reset = false;
			ctn730_err("version %02x%02x\n", ctn730_dev->version[0],
				   ctn730_dev->version[1]);
		}

		if (data[0] == 1) {
			ctn730_dev->status = ctn730_dev->initialized ?
				need_dowload_firmware : download_mode;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		ctn730_schedule_ext_work(ctn730_dev,
					 msecs_to_jiffies(CMD_DELAY_TIME));
		break;

	case CHG_DEVICE_STATE:
		if (data[0] == 0) {
			ctn730_dev->status = detected;
			cancel_delayed_work_sync(
				&ctn730_dev->check_static_charge_work);
			ctn730_dev->pen_info.attached = 1;
			memcpy(ctn730_dev->pen_info.pen_uid, data + 1, 7);
			ctn730_err("pen_uid:%*ph\n",
				   (int)sizeof(ctn730_dev->pen_info.pen_uid),
				   ctn730_dev->pen_info.pen_uid);
			ctn730_schedule_ext_work(ctn730_dev,
					msecs_to_jiffies(CMD_DELAY_TIME));
			ctn730_notifier_call_chain(ctn730_dev,
				ctn730_dev->pen_info.attached, NULL);
			ctn730_send_uevent(ctn730_dev);
		}
		if (data[0] == 1) {
			ctn730_dev->status = deactived;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		if (data[0] == 2) {
			ctn730_dev->pen_info.charge_state = charge_unkown;
			ctn730_dev->status = lost;
			ctn730_dev->have_mac = 0;
			ctn730_dev->pen_info.attached = 0;
			ctn730_dev->pen_info.level = -1;
			ctn730_dev->present_since = 0;
			alarm_cancel(&ctn730_dev->charging_alarm);
			ctn730_dev->detect_time = 0;
			ctn730_dev->lost_time = ktime_get();
			ctn730_clear_ids(ctn730_dev);
			ctn730_notifier_call_chain(ctn730_dev,
				ctn730_dev->pen_info.attached, NULL);
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
			/* Reported once it proves not to be a glitch. */
			schedule_delayed_work(&ctn730_dev->check_pen_lost_work,
				msecs_to_jiffies(PEN_LOST_CHECK_DELAY_TIME));
		}
		if (data[0] == 3) {
			ctn730_dev->status = version_missmatch;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		if (data[0] == 4 || data[0] == 5) {
			s64 delta;

			if (data[0] == 5)
				ctn730_dev->present_since = 0;
			ctn730_dev->detect_time = ktime_get();
			delta = ktime_to_ns(ktime_sub(ctn730_dev->detect_time,
						      ctn730_dev->lost_time));
			ctn730_dev->lost_interval_ms = delta / NSEC_PER_MSEC;
			if (delta <= PEN_LOST_GLITCH_NS) {
				cancel_delayed_work_sync(
					&ctn730_dev->check_pen_lost_work);
				ctn730_err(" cancel check pen lost\n");
			}
			ctn730_dev->detect_time = 0;
			ctn730_dev->lost_time = 0;
			if (!ctn730_dev->pen_info.attached) {
				ctn730_dev->pen_info.attached = 1;
				ctn730_notifier_call_chain(ctn730_dev, 1, NULL);
				ctn730_send_uevent(ctn730_dev);
			}
			ret = cancel_delayed_work_sync(
				&ctn730_dev->check_static_charge_work);
			ctn730_err("ret=%d\n", ret);
			schedule_delayed_work(
				&ctn730_dev->check_static_charge_work,
				msecs_to_jiffies(STATIC_CHECK_DELAY_TIME));
		}
		break;

	case CHG_CHARGING_STATE:
		if (data[0] == 0) {
			alarm_start_relative(&ctn730_dev->charging_alarm,
					     ns_to_ktime(CHARGING_TIMEOUT_NS));
			ctn730_err("charge_state %s\n",
				   charge_state_text[charging]);
			/* Keep the charging pen off the digitizer. */
			if (tp_state == 1)
				nvt_support_pen_set_nfc(0);
			mutex_lock(&ctn730_dev->report_info_mutex);
			ctn730_dev->need_report_info = 1;
			if (!ctn730_dev->report_info && ctn730_dev->have_mac) {
				ctn730_dev->status =
				    need_to_host_ctrl_enable_send_charging_info;
				schedule_delayed_work(&ctn730_dev->set_work,
					msecs_to_jiffies(CMD_DELAY_TIME));
			}
			mutex_unlock(&ctn730_dev->report_info_mutex);
		}
		if (data[0] == 1) {
			alarm_cancel(&ctn730_dev->charging_alarm);
			charge_state = full;
			ctn730_err("charge_state %s\n",
				   charge_state_text[charge_state]);
		}
		if (data[0] == 2 && ctn730_dev->host_ctrl_flag == 0) {
			charge_state = not_charging;
			ctn730_err("charge_state %s\n",
				   charge_state_text[charge_state]);
		}
		if (charge_state != ctn730_dev->pen_info.charge_state) {
			ctn730_dev->pen_info.charge_state = charge_state;
			ctn730_send_uevent(ctn730_dev);
		}
		break;

	case CHG_CHARGING_INFO:
		level = data[0];
		if (level != ctn730_dev->pen_info.level) {
			if (level == 0xff)
				level = -1;
			ctn730_dev->pen_info.level = level;
			ctn730_dev->level_changed = true;
			ctn730_err("level : %d\n", level);
			ctn730_send_uevent(ctn730_dev);
		}
		break;

	case CHG_DISABLE:
		if (data[0] == 0) {
			ctn730_dev->status = host_request_dischg;
			ctn730_dev->try_to_cmd = 1;
			schedule_delayed_work(&ctn730_dev->set_work,
					      msecs_to_jiffies(MAC_DELAY_TIME));
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		if (data[0] == 1)
			ctn730_err("disable_chg due to overtemp\n");
		break;

	case APP_CTRL_CHN_STATE:
		if (data[0] == 0) {
			ctn730_dev->status = app_chn_connected;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
			schedule_delayed_work(&ctn730_dev->set_work,
					      msecs_to_jiffies(CMD_DELAY_TIME));
		}
		if (data[0] == 1) {
			ctn730_dev->status = detected;
			ctn730_err("app_chn_disconnected ,treat as status %s\n",
				   status_text[ctn730_dev->status]);
		}
		break;

	case APP_CTRL_CHN_SEND_DATA:
		ctn730_err("APP_CTRL_CHN_SEND_DATA evt\n");
		break;

	case APP_CTRL_CHN_RECIVE_DATA:
		if (data[0] == 1) {
			ctn730_err("APP_CTRL_CHN_RECIVE_DATA last data\n");
			ctn730_dev->status = app_chn_reciced;
			schedule_delayed_work(&ctn730_dev->set_work,
					      msecs_to_jiffies(CMD_DELAY_TIME));
			if (data[1] == 0xaa && data[2] == 0x55 &&
			    data[3] == 0xcc && data[4] == 0x33)
				ctn730_dev->app_data_ok = 2;
			else
				ctn730_dev->app_data_ok = 1;
			ctn730_send_uevent(ctn730_dev);
		}
		if (data[0] == 0)
			ctn730_err("APP_CTRL_CHN_RECIVE_DATA more data\n");
		break;

	default:
		ctn730_err("recieve %02x %02x\n", cmd, count);
		break;
	}
}

static void ctn730_dl_response(struct ctn730_dev *ctn730_dev, char status,
			       int ok_status, const char *what)
{
	if (status == STATUS_OK) {
		ctn730_dev->status = ok_status;
		ctn730_err("status %s\n", status_text[ctn730_dev->status]);
	} else {
		ctn730_dev->status = update_firmware_err;
		ctn730_err("status %s failed\n", what);
	}
	ctn730_schedule_ext_work(ctn730_dev, msecs_to_jiffies(CMD_DELAY_TIME));
}

static void ctn730_parse_response(struct ctn730_dev *ctn730_dev,
				  int instruction, char *data, int count,
				  char cmd)
{
	switch (instruction) {
	case CHG_ENABLE:
		if (data[0] == STATUS_OK) {
			ctn730_dev->status = detecting;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
			schedule_delayed_work(
				&ctn730_dev->check_static_charge_work,
				msecs_to_jiffies(DETECT_CHECK_DELAY_TIME));
		} else {
			ctn730_err("CHG_ENABLE  Response error: %d\n", data[0]);
		}
		break;

	case CHG_DISABLE:
		if (data[0] == STATUS_OK)
			ctn730_err("CHG_DISABLE OK\n");
		else
			ctn730_err("CHG_DISABLE  Response error: %d\n",
				   data[0]);
		break;

	case HOST_CTRL_BIST:
		if (data[0] != STATUS_OK) {
			ctn730_dev->status = bist_err;
			schedule_delayed_work(&ctn730_dev->set_work,
					      msecs_to_jiffies(CMD_DELAY_TIME));
			ctn730_err("HOST_CTRL_BIST  Response error: %d\n",
				   data[0]);
			break;
		}

		mutex_lock(&ctn730_dev->bist_mutex);
		if (ctn730_dev->bist_id == BIST_RF_ON) {
			ctn730_dev->status = rf_on;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		if (ctn730_dev->bist_id == BIST_RF_OFF) {
			ctn730_dev->status = rf_off;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		if (ctn730_dev->bist_id == BIST_READ_NTAG_BLOCKS) {
			memcpy(ctn730_dev->pen_info.mac, data + 1, 6);
			ctn730_dev->status = mac_getted;
			ctn730_dev->have_mac = 1;
			ctn730_dev->level_changed = 1;
			ctn730_send_uevent(ctn730_dev);
			ctn730_err("pen_mac: %02X:%02X:%02X:%02X:%02X:%02X\n",
				   ctn730_dev->pen_info.mac[5],
				   ctn730_dev->pen_info.mac[4],
				   ctn730_dev->pen_info.mac[3],
				   ctn730_dev->pen_info.mac[2],
				   ctn730_dev->pen_info.mac[1],
				   ctn730_dev->pen_info.mac[0]);
			ctn730_err("recieve %02x %02x\n", cmd, count);
		}
		if (ctn730_dev->bist_id == BIST_ACTIVATION)
			ctn730_err("BIST_ACTIVATION:%*ph\n", count, data);
		mutex_unlock(&ctn730_dev->bist_mutex);

		if (ctn730_dev->ext_cmd != ext_cmd_rf_on &&
		    ctn730_dev->ext_cmd != ext_cmd_update_firmware)
			schedule_delayed_work(&ctn730_dev->set_work,
					      msecs_to_jiffies(CMD_DELAY_TIME));
		break;

	case APP_CTRL_CHN_ENABLE:
		if (data[0] == STATUS_OK) {
			ctn730_dev->status = app_chn_enabled;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		break;

	case APP_CTRL_CHN_SEND_DATA:
		if (data[0] == STATUS_OK) {
			ctn730_dev->status = detected;
			ctn730_err("APP_CTRL_CHN_SEND_DATA rsp ok, treat as detected\n");
		} else {
			ctn730_dev->app_data_ok = 1;
			ctn730_send_uevent(ctn730_dev);
		}
		break;

	case APP_CTRL_CHN_DISABLE:
		if (data[0] == STATUS_OK) {
			ctn730_dev->status = app_chn_disabled;
			ctn730_err("status %s\n",
				   status_text[ctn730_dev->status]);
		}
		break;

	case HOST_CTRL_WRITE_SESSION_PARAMETER:
		if (data[0] != STATUS_OK) {
			ctn730_err("status HOST_CTRL_WRITE_SESSION_PARAMETER failed\n");
			break;
		}
		ctn730_dev->status = detected;
		mutex_lock(&ctn730_dev->report_info_mutex);
		if (ctn730_dev->report_info_id == 1)
			ctn730_dev->report_info = 1;
		if (ctn730_dev->report_info_id == 0)
			ctn730_dev->report_info = 0;
		mutex_unlock(&ctn730_dev->report_info_mutex);
		ctn730_err("status HOST_CTRL_WRITE_SESSION_PARAMETER ok, treat as detected\n");
		break;

	case HOST_CTRL_DL_OPEN_SESSION:
		ctn730_dl_response(ctn730_dev, data[0], dl_open_session_ok,
				   "HOST_CTRL_DL_OPEN_SESSION");
		break;

	case HOST_CTRL_DL_COMMIT_SESSION:
		ctn730_dl_response(ctn730_dev, data[0], dl_commit_ok,
				   "HOST_CTRL_DL_COMMIT_SESSION");
		break;

	case HOST_CTRL_DL_WRITE_FLASH:
		ctn730_dl_response(ctn730_dev, data[0], dl_flash_ok,
				   "HOST_CTRL_DL_WRITE_FLASH");
		break;

	default:
		ctn730_err("recieve %02x %02x\n", cmd, count);
		break;
	}
}

static int ctn730_parse_message(struct ctn730_dev *ctn730_dev, char cmd,
				char *data, int count)
{
	int instruction = cmd & INSTRUCTION_MASK;

	ctn730_err("wk>enter ctn730_parse_message(),instruction: %d\n",
		   instruction);

	switch (cmd & MT_MASK) {
	case RSP_BIT:
		ctn730_parse_response(ctn730_dev, instruction, data, count, cmd);
		break;
	case EVENT_BIT:
		ctn730_parse_event(ctn730_dev, instruction, data, count, cmd);
		break;
	default:
		break;
	}

	return 0;
}

/*
 * A charger that keeps reporting a pen present without ever detecting it
 * is stuck; power-cycle it once that has gone on for five minutes.
 */
static bool ctn730_present_stuck(struct ctn730_dev *ctn730_dev, char *cmd,
				 char *data)
{
	struct timespec elapsed;

	if (cmd[0] != (EVENT_BIT | CHG_DEVICE_STATE) || cmd[1] != 1 ||
	    data[0] != 4) {
		ctn730_dev->present_since = 0;
		return false;
	}

	if (!ctn730_dev->present_since)
		ctn730_dev->present_since = ktime_get();
	elapsed = ktime_to_timespec(ktime_sub(ktime_get(),
					      ctn730_dev->present_since));
	if (elapsed.tv_sec <= PRESENT_STUCK_TIMEOUT_S)
		return false;

	ctn730_dev->wake_count++;
	ctn730_dev->present_since = 0;
	is_hard_reset = true;
	schedule_delayed_work(&ctn730_dev->reset_work, 0);

	return true;
}

/**********************************************************
 * Interrupt control and handler
 **********************************************************/
static irqreturn_t ctn730_dev_irq_handler(int irq, void *dev_id)
{
	struct ctn730_dev *ctn730_dev = dev_id;
	char cmd[2] = { 0 };
	char data[MAX_BUFFER_SIZE] = { 0 };
	int ret;

	if (ctn730_dev == NULL) {
		ctn730_err("no ctn730_dev\n");
		return IRQ_HANDLED;
	}

	if (ctn730_dev->suspend) {
		ctn730_err("suspended, wakeup event!\n");
		pm_wakeup_event(ctn730_dev->ctn730_device.this_device, 10000);
		ctn730_dev->suspend = 0;
		msleep(50);
	}

	if (!gpio_get_value(ctn730_dev->irq_gpio)) {
		ctn730_err("irq_gpio low do not read i2c\n");
		return IRQ_HANDLED;
	}

	ret = i2c_master_recv(ctn730_dev->client, cmd, 2);
	if (ret < 0) {
		ctn730_err("i2c_master_recv returned %d\n", ret);
		msleep(2);
		return IRQ_HANDLED;
	}
	if (cmd[1] > 0)
		i2c_master_recv(ctn730_dev->client, data, cmd[1]);

	if (ctn730_present_stuck(ctn730_dev, cmd, data)) {
		ctn730_err("ctn730_hard_reset \n");
		return IRQ_HANDLED;
	}

	ctn730_err("recieve %*ph %*ph\n", 2, cmd, cmd[1], data);
	ctn730_parse_message(ctn730_dev, cmd[0], data, cmd[1]);
	msleep(5);

	return IRQ_HANDLED;
}

static void set_work(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
						     set_work.work);

	if (ctn730_dev->ext_cmd) {
		ctn730_err("has ext_cmd, skip\n");
		return;
	}

	switch (ctn730_dev->status) {
	case download_mode:
		ctn730_reset_init(ctn730_dev);
		break;
	case initialized:
		ctn730_enable_chg(ctn730_dev);
		break;
	case detected:
		if (!ctn730_dev->have_mac) {
			ctn730_disable_chg(ctn730_dev);
			ctn730_dev->host_ctrl_flag = 1;
		}
		break;
	case host_request_dischg:
		ctn730_bist(ctn730_dev, BIST_RF_ON);
		break;
	case rf_on:
		if (!ctn730_dev->have_mac)
			ctn730_get_mac(ctn730_dev);
		break;
	case mac_getted:
		ctn730_bist(ctn730_dev, BIST_RF_OFF);
		break;
	case rf_off:
		ctn730_enable_chg(ctn730_dev);
		ctn730_dev->host_ctrl_flag = 0;
		break;
	case need_to_app_chmn_enable:
		ctn730_enable_app_channel(ctn730_dev);
		break;
	case app_chn_connected:
		ctn730_app_set_param(ctn730_dev);
		break;
	case app_chn_reciced:
		ctn730_disable_app_channel(ctn730_dev);
		break;
	case need_to_host_ctrl_enable_send_charging_info:
		ctn730_set_charging_info(ctn730_dev, true);
		break;
	case need_to_host_ctrl_disable_send_charging_info:
		ctn730_set_charging_info(ctn730_dev, false);
		break;
	case need_to_rf_on:
		ctn730_disable_chg(ctn730_dev);
		break;
	case need_to_rf_off:
		ctn730_bist(ctn730_dev, BIST_RF_OFF);
		break;
	case bist_err:
		ctn730_reset_init(ctn730_dev);
		break;
	default:
		break;
	}
}

static void reset_work(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
						     reset_work.work);

	ctn730_dev_hard_reset(ctn730_dev);
	if (ctn730_dev->wake_count) {
		pm_relax(ctn730_dev->dev);
		ctn730_dev->wake_count--;
	}
}

static enum alarmtimer_restart charging_alarmtimer_function(
		struct alarm *alarm, ktime_t now)
{
	struct ctn730_dev *ctn730_dev = container_of(alarm, struct ctn730_dev,
						     charging_alarm);

	if (ctn730_dev->suspend) {
		pm_stay_awake(ctn730_dev->dev);
		ctn730_dev->wake_count++;
	}
	schedule_delayed_work(&ctn730_dev->reset_work, 0);
	ctn730_err("charging time out ctn730 reset !!!\n");

	return ALARMTIMER_NORESTART;
}

/* Report the pen as gone, and give it back to the digitizer. */
static void ctn730_report_pen_gone(struct ctn730_dev *ctn730_dev,
				   const char *why)
{
	ctn730_clear_pen(ctn730_dev);
	if (tp_state == 1) {
		ctn730_err("%s :open pen mode\n", why);
		nvt_support_pen_set_nfc(1);
	}
	ctn730_clear_ids(ctn730_dev);
	ctn730_notifier_call_chain(ctn730_dev, ctn730_dev->pen_info.attached,
				   NULL);
	ctn730_send_uevent(ctn730_dev);
}

static void check_static_charge_work(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
					check_static_charge_work.work);

	ctn730_report_pen_gone(ctn730_dev, "check charge work");
}

static void check_pen_lost(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
					check_pen_lost_work.work);

	ctn730_err("enter check :is pen lost\n");
	ctn730_report_pen_gone(ctn730_dev, "pen_lost");
}

static void rf_on_work(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
						     rf_on_work.work);

	if (ctn730_dev->ext_cmd == 0)
		ctn730_hard_reset(ctn730_dev);
	if (ctn730_dev->ext_cmd != ext_cmd_rf_on)
		return;

	switch (ctn730_dev->status) {
	case download_mode:
		ctn730_reset_init(ctn730_dev);
		break;
	case initialized:
		ctn730_bist(ctn730_dev, BIST_RF_ON);
		break;
	default:
		ctn730_hard_reset(ctn730_dev);
		break;
	}
}

static void update_firmware_work(struct work_struct *work)
{
	struct ctn730_dev *ctn730_dev = container_of(work, struct ctn730_dev,
						     update_firmware_work.work);
	struct firmware_info *fw = &ctn730_dev->firmware;

	if (!ctn730_dev->enabled) {
		ctn730_enable(ctn730_dev);
		fw->index = 0;
		return;
	}

	switch (ctn730_dev->status) {
	case download_mode:
		ctn730_dl_open(ctn730_dev);
		break;
	case dl_open_session_ok:
	case dl_flash_ok:
		if (fw->index < fw->user_ee_size / 128) {
			if (ctn730_dl_flash(ctn730_dev,
					    fw->data + fw->index * 128,
					    fw->user_ee_addr +
					    fw->index * 128) >= 0)
				fw->index++;
		} else if (fw->index < fw->size / 128) {
			if (ctn730_dl_flash(ctn730_dev,
					    fw->data + fw->index * 128,
					    fw->user_app_addr +
					    (fw->index - fw->user_ee_size / 128) *
					    128) >= 0)
				fw->index++;
		} else if (fw->index == fw->size / 128) {
			if (ctn730_dl_commit(ctn730_dev, fw->crc) >= 0)
				fw->index++;
		}
		break;
	case dl_commit_ok:
		ctn730_dev->ext_cmd = 0;
		ctn730_dev->status = download_mode;
		vfree(fw->data);
		fw->data = NULL;
		schedule_delayed_work(&ctn730_dev->set_work,
				      msecs_to_jiffies(100));
		break;
	default:
		ctn730_disable(ctn730_dev);
		break;
	}
}

static int ctn730_parse_dt(struct device *dev, struct ctn730_dev *ctn730_dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *node_f;
	struct drm_panel *panel;
	struct pinctrl *pinctrl;
	int count, i, val;

	if (!node) {
		ctn730_err("no node\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_err(dev, "pins are not configured\n");

	val = of_get_named_gpio(node, "rst-gpios", 0);
	if (val < 0) {
		dev_err(dev, "RST GPIO error getting from OF node\n");
		return val;
	}
	ctn730_dev->rst_gpio = val;

	val = of_get_named_gpio(node, "enable-gpios", 0);
	if (val < 0) {
		dev_err(dev, "VEN GPIO error getting from OF node\n");
		return val;
	}
	ctn730_dev->ven_gpio = val;

	val = of_get_named_gpio(node, "interrupt-gpios", 0);
	if (val < 0) {
		dev_err(dev, "IRQ GPIO error getting from OF node\n");
		return val;
	}
	ctn730_dev->irq_gpio = val;

	count = of_count_phandle_with_args(node, "panel", NULL);
	ctn730_err("panel count:%d \n", count);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node_f = of_parse_phandle(node, "panel", i);
		panel = of_drm_find_panel(node_f);
		of_node_put(node_f);
		if (!IS_ERR(panel)) {
			ctn730_err("find panel\n");
			ctn730_panel = panel;
			return 0;
		}
	}

	return -EPROBE_DEFER;
}

static struct ctn730_dev *ctn730_from_dev(struct device *dev)
{
	return container_of(dev_get_drvdata(dev), struct ctn730_dev,
			    ctn730_device);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	if (simple_strtoul(buf, NULL, 0)) {
		ctn730_err("wk>ctn730_enable_chg\n");
		ctn730_enable_chg(ctn730_dev);
	} else {
		ctn730_err("wk>excute--ctn730_disable_chg()\n");
		ctn730_dev->ext_cmd = 0;
		ctn730_disable_chg(ctn730_dev);
		ctn730_dev->pen_info.attached = 0;
		if (tp_state == 1) {
			ctn730_err("enable_store :open pen mode\n");
			nvt_support_pen_set_nfc(1);
		}
		ctn730_dev->have_mac = 0;
	}

	return size;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%d\n", gpio_get_value(ctn730_dev->ven_gpio));
}

static ssize_t pen_cmd_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	if (ctn730_dev->status == detected && ctn730_dev->have_mac) {
		ctn730_dev->pen_param[0] = 0xaa;
		ctn730_dev->pen_param[1] = 0x55;
		ctn730_dev->pen_param[2] = 0xcc;
		ctn730_dev->pen_param[3] = 0x33;
		ctn730_dev->pen_param_count = 4;
		ctn730_dev->status = need_to_app_chmn_enable;
		schedule_delayed_work(&ctn730_dev->set_work, 0);
	} else {
		ctn730_err("status: %s have_mac %d,no pen_cmd\n",
			   status_text[ctn730_dev->status],
			   ctn730_dev->have_mac);
	}

	return size;
}

static ssize_t reget_mac_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	if (simple_strtoul(buf, NULL, 0)) {
		ctn730_dev->status = detected;
		ctn730_dev->have_mac = 0;
		schedule_delayed_work(&ctn730_dev->set_work, 0);
	}

	return size;
}

static ssize_t info_report_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	if (simple_strtoul(buf, NULL, 0))
		ctn730_dev->status =
			need_to_host_ctrl_enable_send_charging_info;
	else
		ctn730_dev->status =
			need_to_host_ctrl_disable_send_charging_info;
	schedule_delayed_work(&ctn730_dev->set_work, msecs_to_jiffies(100));

	return size;
}

static ssize_t rf_on_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	ctn730_dev->ext_cmd = simple_strtoul(buf, NULL, 0) ? ext_cmd_rf_on : 0;
	schedule_delayed_work(&ctn730_dev->rf_on_work, msecs_to_jiffies(10));

	return size;
}

static ssize_t param_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	if (size > PEN_PARAM_LEN) {
		ctn730_err("too many input\n");
		return -EINVAL;
	}
	memcpy(ctn730_dev->pen_param, buf, size);
	ctn730_dev->pen_param_count = size;
	ctn730_dev->status = need_to_app_chmn_enable;
	schedule_delayed_work(&ctn730_dev->set_work, msecs_to_jiffies(100));

	return size;
}

static ssize_t charge_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%s\n",
		       charge_state_text[ctn730_dev->pen_info.charge_state]);
}

static ssize_t mac_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);
	char *mac = ctn730_dev->pen_info.mac;

	return sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X\n", mac[5], mac[4],
		       mac[3], mac[2], mac[1], mac[0]);
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%02x%02x", ctn730_dev->version[0],
		       ctn730_dev->version[1]);
}

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%d\n", ctn730_dev->pen_info.level);
}

static ssize_t attached_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%d\n", ctn730_dev->pen_info.attached);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	return sprintf(buf, "%s\n", status_text[ctn730_dev->status]);
}

static ssize_t cn_test_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);

	ctn730_bist(ctn730_dev, BIST_ACTIVATION);
	return sprintf(buf, "%s\n", status_text[ctn730_dev->status]);
}

static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_WO(pen_cmd);
static DEVICE_ATTR_WO(reget_mac);
static DEVICE_ATTR_WO(info_report);
static DEVICE_ATTR_WO(rf_on);
static DEVICE_ATTR_WO(param);
static DEVICE_ATTR_RO(charge_state);
static DEVICE_ATTR_RO(mac);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(level);
static DEVICE_ATTR_RO(attached);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(cn_test);

static struct attribute *ctn730_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_pen_cmd.attr,
	&dev_attr_reget_mac.attr,
	&dev_attr_info_report.attr,
	&dev_attr_rf_on.attr,
	&dev_attr_param.attr,
	&dev_attr_charge_state.attr,
	&dev_attr_mac.attr,
	&dev_attr_version.attr,
	&dev_attr_level.attr,
	&dev_attr_attached.attr,
	&dev_attr_status.attr,
	&dev_attr_cn_test.attr,
	NULL
};
ATTRIBUTE_GROUPS(ctn730);

static int ctn730_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct ctn730_dev *ctn730_dev = ctn730_from_dev(dev);
	char *mac = ctn730_dev->pen_info.mac;
	int ret;

	if (ctn730_dev->app_data_ok) {
		ret = add_uevent_var(env, "WRITEPENDATA=%d",
				     ctn730_dev->app_data_ok == 2);
		ctn730_dev->app_data_ok = 0;
		if (ret)
			dev_err(dev, "failed to add uevent STATUS\n");
	}
	ret = add_uevent_var(env, "ATTACHED=%d", ctn730_dev->pen_info.attached);
	if (ret)
		dev_err(dev, "failed to add uevent STATUS\n");
	ret = add_uevent_var(env, "CHARGING_STATE=%s",
			charge_state_text[ctn730_dev->pen_info.charge_state]);
	if (ret)
		dev_err(dev, "failed to add uevent CHARGING_STATE\n");
	if (ctn730_dev->level_changed && ctn730_dev->pen_info.level != -1) {
		ret = add_uevent_var(env, "LEVEL=%d",
				     ctn730_dev->pen_info.level);
		if (ret)
			dev_err(dev, "failed to add uevent LEVEL\n");
		ctn730_dev->level_changed = false;
	}
	ret = add_uevent_var(env, "MAC=%02X:%02X:%02X:%02X:%02X:%02X", mac[5],
			     mac[4], mac[3], mac[2], mac[1], mac[0]);
	if (ret)
		dev_err(dev, "failed to add uevent LEVEL\n");

	return ret;
}

static const struct device_type ctn730_dev_type = {
	.name = "nfc_charger",
	.uevent = ctn730_uevent,
};

static int ctn730_drm_notifier_callback(struct notifier_block *self,
					unsigned long event, void *data)
{
	struct ctn730_dev *ctn730_dev = container_of(self, struct ctn730_dev,
						     drm_notifier);
	struct drm_panel_notifier *notifier_data = data;
	int status;
	int *blank;

	if (!notifier_data) {
		ctn730_err("no data\n");
		return -ENODATA;
	}
	blank = notifier_data->data;

	if (event == DRM_PANEL_EARLY_EVENT_BLANK &&
	    *blank == DRM_PANEL_BLANK_POWERDOWN) {
		mutex_lock(&ctn730_dev->report_info_mutex);
		ctn730_dev->need_report_info = 0;
		mutex_unlock(&ctn730_dev->report_info_mutex);
		status = need_to_host_ctrl_disable_send_charging_info;
	} else if (event == DRM_PANEL_EVENT_BLANK &&
		   *blank == DRM_PANEL_BLANK_UNBLANK) {
		mutex_lock(&ctn730_dev->report_info_mutex);
		ctn730_dev->need_report_info = 1;
		mutex_unlock(&ctn730_dev->report_info_mutex);
		status = need_to_host_ctrl_enable_send_charging_info;
	} else {
		return 0;
	}

	ctn730_err("need_report_info:%d report_info:%d\n",
		   ctn730_dev->need_report_info, ctn730_dev->report_info);
	if (ctn730_dev->pen_info.charge_state == charging &&
	    ctn730_dev->have_mac) {
		ctn730_dev->status = status;
		schedule_delayed_work(&ctn730_dev->set_work,
				      msecs_to_jiffies(CMD_DELAY_TIME));
	} else {
		ctn730_err("status: %s have_mac %d,no charging info cmd\n",
			   status_text[ctn730_dev->status],
			   ctn730_dev->have_mac);
	}

	return 0;
}

static int ctn730_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ctn730_dev *ctn730_dev;
	int ret;

	client_address = client;
	ctn730_dev = kzalloc(sizeof(*ctn730_dev), GFP_KERNEL);
	if (ctn730_dev == NULL) {
		ctn730_err("failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	ctn730 = ctn730_dev;
	BLOCKING_INIT_NOTIFIER_HEAD(&ctn730_dev->nh);

	ret = ctn730_parse_dt(&client->dev, ctn730_dev);
	if (ret < 0) {
		ctn730_err("parse device tree fail,probe fail\n");
		goto err_free;
	}

	/* Reset the charger into a known state. */
	if (gpio_request(ctn730_dev->rst_gpio, "rst-gpios"))
		ctn730_err("not able to get GPIO rst_gpio\n");
	else
		ctn730_dev->rst_requested = true;
	if (gpio_is_valid(ctn730_dev->rst_gpio)) {
		gpio_direction_output(ctn730_dev->rst_gpio, 1);
		msleep(20);
		gpio_set_value(ctn730_dev->rst_gpio, 0);
		msleep(20);
		gpio_set_value(ctn730_dev->rst_gpio, 1);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ctn730_err("need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_free;
	}

	ret = gpio_request(ctn730_dev->irq_gpio, "nfc_int");
	if (ret) {
		ctn730_err("not able to get GPIO irq_gpio\n");
		ret = -ENODEV;
		goto err_free;
	}
	ret = gpio_request(ctn730_dev->ven_gpio, "nfc_ven");
	if (ret) {
		ctn730_err("not able to get GPIO ven_gpio\n");
		ret = -ENODEV;
		goto err_free_irq_gpio;
	}

	ret = gpio_to_irq(ctn730_dev->irq_gpio);
	if (ret < 0) {
		ctn730_err("not able to map GPIO irq_gpio to an IRQ\n");
		ret = -ENODEV;
		goto err_free_gpios;
	}
	ctn730_dev->irq = ret;
	ctn730_dev->client = client;

	ret = gpio_direction_input(ctn730_dev->irq_gpio);
	if (ret < 0) {
		ctn730_err("not able to set irq_gpio as input\n");
		goto err_free_gpios;
	}
	ret = gpio_direction_output(ctn730_dev->ven_gpio, 0);
	if (ret < 0) {
		ctn730_err("not able to set ven_gpio as output\n");
		goto err_free_gpios;
	}

	spin_lock_init(&ctn730_dev->irq_enabled_lock);
	mutex_init(&ctn730_dev->bist_mutex);
	mutex_init(&ctn730_dev->report_info_mutex);

	ctn730_dev->ctn730_device.minor = MISC_DYNAMIC_MINOR;
	ctn730_dev->ctn730_device.name = "ctn730";
	ctn730_dev->ctn730_device.nodename = "ctn730";
	ctn730_dev->ctn730_device.groups = ctn730_groups;
	ret = misc_register(&ctn730_dev->ctn730_device);
	if (ret) {
		ctn730_err("misc_register failed\n");
		goto err_free_gpios;
	}
	ctn730_dev->ctn730_device.this_device->type = &ctn730_dev_type;
	ctn730_dev->dev = ctn730_dev->ctn730_device.this_device;

	ctn730_dev->drm_notifier.notifier_call = ctn730_drm_notifier_callback;
	if (ctn730_panel == NULL)
		ctn730_err("drm panel is  NULL\n");
	else if (drm_panel_notifier_register(ctn730_panel,
					     &ctn730_dev->drm_notifier))
		ctn730_err("Unable to register ctn730_drm_notifier\n");

	device_init_wakeup(ctn730_dev->dev, 1);
	i2c_set_clientdata(client, ctn730_dev);

	alarm_init(&ctn730_dev->charging_alarm, ALARM_BOOTTIME,
		   charging_alarmtimer_function);
	INIT_DELAYED_WORK(&ctn730_dev->update_firmware_work,
			  update_firmware_work);
	INIT_DELAYED_WORK(&ctn730_dev->rf_on_work, rf_on_work);
	INIT_DELAYED_WORK(&ctn730_dev->check_static_charge_work,
			  check_static_charge_work);
	INIT_DELAYED_WORK(&ctn730_dev->check_pen_lost_work, check_pen_lost);
	INIT_DELAYED_WORK(&ctn730_dev->set_work, set_work);
	INIT_DELAYED_WORK(&ctn730_dev->reset_work, reset_work);

	ret = devm_request_threaded_irq(ctn730_dev->dev, ctn730_dev->irq, NULL,
					ctn730_dev_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					ctn730_dev->ctn730_device.name,
					ctn730_dev);
	if (ret) {
		ctn730_err("request_irq failed\n");
		goto err_deregister;
	}
	disable_irq(ctn730_dev->irq);
	enable_irq_wake(ctn730_dev->irq);

	ctn730_enable(ctn730_dev);
	ctn730_dev->report_info = 1;
	ctn730_reset_init(ctn730_dev);
	ctn730_err("CTN730 successed\n");

	return 0;

err_deregister:
	if (ctn730_panel)
		drm_panel_notifier_unregister(ctn730_panel,
					      &ctn730_dev->drm_notifier);
	misc_deregister(&ctn730_dev->ctn730_device);
err_free_gpios:
	gpio_free(ctn730_dev->ven_gpio);
err_free_irq_gpio:
	gpio_free(ctn730_dev->irq_gpio);
err_free:
	if (ctn730_dev->rst_requested)
		gpio_free(ctn730_dev->rst_gpio);
	ctn730 = NULL;
	client_address = NULL;
	kfree(ctn730_dev);
	return ret;
}

static int ctn730_remove(struct i2c_client *client)
{
	struct ctn730_dev *ctn730_dev = i2c_get_clientdata(client);

	devm_free_irq(ctn730_dev->dev, ctn730_dev->irq, ctn730_dev);
	alarm_cancel(&ctn730_dev->charging_alarm);
	cancel_delayed_work_sync(&ctn730_dev->set_work);
	cancel_delayed_work_sync(&ctn730_dev->reset_work);
	cancel_delayed_work_sync(&ctn730_dev->update_firmware_work);
	cancel_delayed_work_sync(&ctn730_dev->rf_on_work);
	cancel_delayed_work_sync(&ctn730_dev->check_static_charge_work);
	cancel_delayed_work_sync(&ctn730_dev->check_pen_lost_work);
	if (ctn730_panel)
		drm_panel_notifier_unregister(ctn730_panel,
					      &ctn730_dev->drm_notifier);
	misc_deregister(&ctn730_dev->ctn730_device);
	gpio_free(ctn730_dev->irq_gpio);
	gpio_free(ctn730_dev->ven_gpio);
	if (ctn730_dev->rst_requested)
		gpio_free(ctn730_dev->rst_gpio);
	vfree(ctn730_dev->firmware.data);
	ctn730 = NULL;
	client_address = NULL;
	kfree(ctn730_dev);

	return 0;
}

static int ctn730_suspend(struct device *dev)
{
	struct ctn730_dev *ctn730_dev = dev_get_drvdata(dev);

	ctn730_dev->suspend = 1;
	if (ctn730_dev->report_info && ctn730_dev->status == detected &&
	    ctn730_dev->have_mac)
		ctn730_set_charging_info(ctn730_dev, false);

	return 0;
}

static int ctn730_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(ctn730_pm_ops, ctn730_suspend, ctn730_resume);

static const struct of_device_id ctn730_dt_match[] = {
	{ .compatible = "nxp,ctn730", },
	{ }
};
MODULE_DEVICE_TABLE(of, ctn730_dt_match);

static struct i2c_driver ctn730_driver = {
	.probe = ctn730_probe,
	.remove = ctn730_remove,
	.driver = {
		.name = "ctn730",
		.of_match_table = ctn730_dt_match,
		.pm = &ctn730_pm_ops,
	},
};
module_i2c_driver(ctn730_driver);

MODULE_DESCRIPTION("NXP CTN730 stylus wireless charger driver");
MODULE_LICENSE("GPL");
