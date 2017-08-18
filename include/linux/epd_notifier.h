/*
 * epd_notify.h: Electronic Paper Display (EPD) Notification Center
 *
 * (C) Copyright 2016 YotaDevices
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef _LINUX_EPD_NOTIF_H
#define _LINUX_EPD_NOTIF_H

#include <linux/types.h>

/*      Registered EDP driver     */
#define EPD_EVENT_REGISTERED		0x01
/*      Unregistered EDP driver   */
#define EPD_EVENT_UNREGISTERED		0x02
/*      EPD start/stop generate radio noise */
#define EPD_EVENT_RADIO_NOISE		0x03



/* EPD Radio noise event types */
#define EPD_RADIO_NOISE_START	1
#define EPD_RADIO_NOISE_STOP	2


struct epd_event {
	int type;
	int event_time;
};

extern int epd_register_client(struct notifier_block *nb);
extern int epd_unregister_client(struct notifier_block *nb);
extern int epd_notifier_call_chain(unsigned long val, void *v);

#endif /* _LINUX_EPD_NOTIF_H */
