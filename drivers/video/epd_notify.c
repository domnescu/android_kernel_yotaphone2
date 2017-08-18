/*
 *  linux/drivers/video/epd_notify.c
 *
 *  Copyright (C) 2016 YotaDevicess <dmitriy.ivanov@yotadevices.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(epd_display_list);

/**
 *	epd__register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int epd_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&epd_display_list, nb);
}
EXPORT_SYMBOL(epd_register_client);

/**
 *	epd_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int epd_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&epd_display_list, nb);
}
EXPORT_SYMBOL(epd_unregister_client);

/**
 * epd_notifier_call_chain - notify clients of epd_events
 *
 */
int epd_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&epd_display_list, val, v);
}
EXPORT_SYMBOL_GPL(epd_notifier_call_chain);
