/*
 * cyttsp3_ff_api.h
 * Cypress TrueTouch(TM) Standard Product V5 Device Access API module.
 * For use with Cypress touchscreen controllers.
 * Supported parts include:
 * Gen2/3
 *
 * Copyright (C) 2015 Parade Technologies, Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at http://www.paradetech.com/
 *
*/

#ifndef _LINUX_CYTTSP5_FORCE_FUNCTION_API_H
#define _LINUX_CYTTSP5_FORCE_FUNCTION_API_H
#define CY_USE_FORCE_FUNCTION
#define CY_USE_FF_TUNER_SUPPORT
//#define CY_FF_SHOW_TABLES /* for debugging */
#define CY_USE_GEN2
#define CY_USE_REG_ACCESS
#define CONFIG_TOUCHSCREEN_DEBUG
#define CY_I2C_FF_NAME	"cyttsp3_ff_module"

#include <linux/device.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/types.h>
#include <linux/device.h>
#include <linux/stat.h>
struct cyttsp3_force_function {
	void *handle;
	u16 operational;
	u16 ready;
	u16 *pressure_tbl;
	u16 *pressure_tbl_elements;
	u16 (*get_pressure)(void *handle, u16 *pressure_tbl, u16 *pressure_tbl_elements);
	u16 (*force)(struct device *dev, u16 xPos, u16 yPos, u16 zVal);
/*** ZTEMT start ***/
	bool FT_ready_flag;
	bool load_config_result;
/*ZTEMT end*/
};

extern struct cyttsp3_force_function _cyttsp3_force_function;
int cyttsp3_read_block_data(void *handle, u8 addr, u8 length, void *buf);
int cyttsp3_write_block_data(void *handle, u8 addr, u8 length, const void *buf);
int cyttsp3_set_touch_status(bool on);
//int cyttsp3_reset_baseline(void);
extern int id_number;
#endif /* _LINUX_CYTTSP5_FORCE_FUNCTION_API_H */
