/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Lenovo TB371FC additions to the Novatek NT36532 touch driver that other
 * drivers use.
 */
#ifndef _LINUX_INPUT_NVT_LENOVO_H
#define _LINUX_INPUT_NVT_LENOVO_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532_HOSTDL_SPI)
/*
 * Double-tap wake is armed: the touch controller keeps scanning while the
 * screen is off, so the panel keeps its reset and LCD bias.
 */
extern uint8_t nvt_gesture_flag;
/* Pen sensing for the stylus charger: off while the pen charges. */
int nvt_support_pen_set_nfc(u8 on);
#else
#define nvt_gesture_flag 0
static inline int nvt_support_pen_set_nfc(u8 on) { return 0; }
#endif

#endif /* _LINUX_INPUT_NVT_LENOVO_H */
