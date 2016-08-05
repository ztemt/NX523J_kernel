/*
 * cyttsp2_ff_api.h
 * Cypress TrueTouch(TM) Standard Product V2 Force Function module.
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

#ifndef _LINUX_CYTTSP2_FF_API_H
#define _LINUX_CYTTSP2_FF_API_H

#define CY_USE_FORCE_FUNCTION

extern int cyttsp2_get_force(struct device *dev, int touch_num, 
	u16 xPos, u16 yPos, u16 zVal);
#endif /* _LINUX_CYTTSP2_FF_API_H */
