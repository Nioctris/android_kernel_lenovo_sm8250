/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Lenovo TB371FC additions to the Novatek NT36532 touch driver.
 */
#ifndef _NT36XXX_LENOVO_H
#define _NT36XXX_LENOVO_H

#include <linux/input.h>
#include <linux/input/nvt_lenovo.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* Reported when the FW detects a stylus. */
#define KEY_NVT_STYLUS_DETECT	0x25e
/* Advertised by Lenovo's driver, never reported. */
#define KEY_NVT_LENOVO_RESERVED	KEY_NUMERIC_POUND

/* Gesture/function page events. */
#define DATA_PROTOCOL		30
#define FUNCPAGE_GESTURE	1

extern uint8_t bTouchIsAwake;

int32_t nvt_lenovo_init(void);
void nvt_lenovo_ready(void);
void nvt_lenovo_deinit(void);
int32_t nvt_lenovo_register(void);
void nvt_lenovo_unregister(void);
int32_t nvt_lenovo_proc_init(void);
void nvt_lenovo_proc_deinit(void);

int32_t nvt_cmd_store(uint8_t cmd);
int32_t nvt_ext_cmd_store(uint8_t cmd, uint8_t value);
int32_t nvt_support_pen_set(uint8_t state, uint8_t type);
int32_t nvt_edge_reject_set(int32_t direction);
int32_t nvt_edge_grid_zone_set(uint8_t p0, uint8_t p1, uint16_t p2,
			       uint16_t p3);
int32_t nvt_game_mode_set(uint8_t mode);
void nvt_restore_cmd(void);
void nvt_lenovo_queue_restore(struct workqueue_struct *wq);

int32_t nvt_check_stylus_state(uint8_t input_id, uint8_t *data);
void nvt_lenovo_report(int32_t unused, uint32_t x, uint32_t y, int8_t tilt_x,
		       int8_t tilt_y, uint32_t pressure);

int nvt_gesture_switch(struct input_dev *dev, unsigned int type,
		       unsigned int code, int value);
bool nvt_lenovo_gesture_armed(void);
void nvt_lenovo_hall_start(void);
void nvt_lenovo_hall_stop(void);
void nvt_lenovo_pm_resume(void);

#endif /* _NT36XXX_LENOVO_H */
