/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NXP CTN730 NFC wireless charger for the Lenovo stylus, and its hand-off
 * with the touch controller, which must stop sensing the pen while the
 * pen sits on the charger.
 */
#ifndef _LINUX_CTN730_H
#define _LINUX_CTN730_H

#include <linux/errno.h>
#include <linux/notifier.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_NFC_NXP_CTN730)
int is_stylus_attached(void);
int ctn730_notifier_register(struct notifier_block *nb);
int ctn730_notifier_unregister(struct notifier_block *nb);
/* Called by the touch driver once it can take pen mode requests. */
int check_tp_state(int ready);
#else
static inline int is_stylus_attached(void) { return 0; }
static inline int ctn730_notifier_register(struct notifier_block *nb)
{
	return -ENODEV;
}
static inline int ctn730_notifier_unregister(struct notifier_block *nb)
{
	return -ENODEV;
}
static inline int check_tp_state(int ready) { return 0; }
#endif

#endif /* _LINUX_CTN730_H */
