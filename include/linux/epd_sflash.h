/* Copyright (C) 2016 Yota Devices
 * Author: Ivanov Dmitry <dmitriy.ivanov@yotadevices.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __EPD_SFLASH_H__
#define __EPD_SFLASH_H__

#define SF_WF_ADDR	0x00000886
#define SF_TEST_ADDR	0x00030050
#define SF_BORDER_ADDR	0x00070008
#define SF_VCOM_ADDR	0x00070010


extern bool epd_sflash_is_case_white(struct s1d13522fb_par *par);

extern int epd_sflash_get_vcom(struct s1d13522fb_par *par, int *val);

#endif	/* __EPD_SFLASH_H__ */
