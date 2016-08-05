/*
 * Copyright (C) 2015 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

/* Defines generic platform structures for touch drivers */
#ifndef _LINUX_CYTTSP3_FF_PLATFORM_H
#define _LINUX_CYTTSP3_FF_PLATFORM_H

#include <linux/types.h>

struct cyttsp3_ff_settings {
	const uint8_t   *data;
	uint8_t         size;
	uint8_t         tag;
} __attribute__ ((packed));

struct cyttsp3_ff_firmware {
	const uint8_t   *img;
	uint32_t        size;
	const uint8_t   *ver;
	uint8_t         vsize;
} __attribute__ ((packed));

struct cyttsp3_ff_framework {
	const uint16_t  *abs;
	uint8_t         size;
} __attribute__ ((packed));

struct cyttsp3_ff_platform_data {
	int irq_gpio;
	int rst_gpio;
	struct cyttsp3_ff_settings   *sett[256];
	struct cyttsp3_ff_firmware   *fw;
	struct cyttsp3_ff_framework  *frmwrk;

	uint8_t         addr[2];
	uint16_t        flags;

	int         (*hw_reset)(struct cyttsp3_ff_platform_data *pdata);
	int         (*hw_recov)(int on, struct cyttsp3_ff_platform_data *pdata);
	int         (*irq_stat)(struct cyttsp3_ff_platform_data *pdata);
} __attribute__ ((packed));

#endif /* _LINUX_CYTTSP3_FF_PLATFORM_H */
