/*
 *
 * Author	Karsten xuexiaojun <xue.xiaojun@zte.com.cn>
 *
 *
 * Copyright 2015  
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
 */

#ifndef _EXT_MODEM_H_
#define _EXT_MODEM_H_

#define EXT_MODEM_USBSWITCH_HIGH	0x0001 /* pull up usbswitch pin*/
#define EXT_MODEM_USBSWITCH_LOW	0x0002 /* pull down usbswith pin */
#define EXT_MODEM_BOOST_HIGH	0x0003 /* pull pull usbswitch pin */
#define EXT_MODEM_BOOST_LOW		0x0004 /* pull down usbswitch pin */

extern int ext_modem_notifier_call_chain(unsigned long val);

#endif
