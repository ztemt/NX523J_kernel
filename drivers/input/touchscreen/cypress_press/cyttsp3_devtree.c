/*
 * cyttsp3_devtree.c
 * Cypress TrueTouch(TM) Standard Product Device Tree Support Module.
 *
 * Copyright (C) 2013-2014 Cypress Semiconductor
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
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

/* cyttsp */
#include <linux/cyttsp3_ff_platform.h>

/*** ZTEMT start ***/
int rst_gpio_number;
int irq_gpio_number;
#define CY_USE_AUTOLOAD_FW
/*ZTEMT end*/

static int cyttsp3_hw_reset(struct cyttsp3_ff_platform_data *pdata)
{
	int retval = 0;

	gpio_set_value(rst_gpio_number, 0);
	msleep(20);
	gpio_set_value(rst_gpio_number, 1);
	msleep(40);
	gpio_set_value(rst_gpio_number, 0);
	msleep(20);
	pr_info("%s: strobe Gen32 RST(%d) pin\n", __func__, rst_gpio_number);

	return retval;
}

/* cyttsp3_hw_recov() is used to switch INT direction and wake part */
#define CY_WAKE_DFLT                99	/* causes wake strobe on INT line
					 * in sample board configuration
					 * platform data->hw_recov() function
					 */
static int cyttsp3_hw_recov(int on, struct cyttsp3_ff_platform_data *pdata)
{
	//int irq_gpio = pdata->irq_gpio;
	int retval = 0;

	switch (on) {
	case 0:
		/* pulse the RST pin to reset the part */
		cyttsp3_hw_reset(pdata);
		retval = 0;
		break;
	case CY_WAKE_DFLT:
        printk("ztemt CY_WAKE_DFLT no need ,check it!!!\n");
        /*
		retval = gpio_request(irq_gpio_number, NULL);
		if (retval < 0) {
			pr_err("%s: Fail request IRQ pin r=%d\n", __func__, retval);
			pr_err("%s: Try free IRQ gpio=%d\n", __func__, irq_gpio_number);
			gpio_free(irq_gpio_number);
			retval = gpio_request(irq_gpio_number, NULL);
			if (retval < 0) {
				pr_err("%s: Fail 2nd request IRQ pin r=%d\n", __func__, retval);
			}
		}

		if (!(retval < 0)) {*/
			/* strobe (assert) INT pin to wake part */
			retval = gpio_direction_output(irq_gpio_number, 0);
			if (retval < 0) {
				pr_err("%s: Fail switch IRQ pin to OUT r=%d\n", __func__, retval);
			} else {
				/* 2ms wake pulse */
				udelay(2000);
				retval = gpio_direction_output(irq_gpio_number, 1);
				udelay(2000);
				//retval = gpio_direction_output(irq_gpio_number, 0);
				//udelay(2000);
				/* restore INT to input (de-assert INT) */
				retval = gpio_direction_input(irq_gpio_number);
				if (retval < 0) {
					pr_err("%s: Fail switch IRQ pin to IN r=%d\n", __func__, retval);
				}
			}
            /*
			gpio_free(irq_gpio_number);
		}*/
		break;
	default:
		retval = -ENOSYS;
		break;
	}

	return retval;
}

static int cyttsp3_irq_stat(struct cyttsp3_ff_platform_data *pdata)
{
	int irq_gpio = pdata->irq_gpio;
	return gpio_get_value(irq_gpio);
}

#if 0 //ZHWW, no use
static const uint8_t cyttsp_si_regs[] = {0, 0, 0, 0, 0, 0, 0x00, 0xFF, 0x0A};

static struct cyttsp3_ff_settings cyttsp_sett_si_regs = {
	.data = (uint8_t *)&cyttsp_si_regs[0],
	.size = sizeof(cyttsp_si_regs),
	.tag = 6,
};
#endif

static const uint8_t cyttsp_bl_keys[] = {0xAA, 0x55, 0x33, 0x68, 0x98, 0x0B, 0x1D, 0xAC};

static struct cyttsp3_ff_settings cyttsp_sett_bl_keys = {
	.data = (uint8_t *)&cyttsp_bl_keys[0],
	.size = sizeof(cyttsp_bl_keys),
	.tag = 0,
};

#ifdef CY_USE_AUTOLOAD_FW
#include "cyttsp3_img.h"
static struct cyttsp3_ff_firmware cyttsp3_firmware = {
	.img = cyttsp3_img,
	.size = sizeof(cyttsp3_img),
	.ver = cyttsp3_ver,
	.vsize = sizeof(cyttsp3_ver),
};
#else
static struct cyttsp3_ff_firmware cyttsp3_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

/* force function does not require touch signal ranges */
#define CYTTSP3_IGNORE_VALUE 0xFFFF
static const uint16_t cyttsp3_abs[] = {
	CYTTSP3_IGNORE_VALUE, 0, 0, 0, 0,
	CYTTSP3_IGNORE_VALUE, 0, 0, 0, 0,
	CYTTSP3_IGNORE_VALUE, 0, 0, 0, 0,
	CYTTSP3_IGNORE_VALUE, 0, 0, 0, 0,
	CYTTSP3_IGNORE_VALUE, 0, 0, 0, 0,
};

struct cyttsp3_ff_framework cyttsp3_framework = {
	.abs = (uint16_t *)&cyttsp3_abs[0],
	.size = ARRAY_SIZE(cyttsp3_abs),
};


static int cyttsp3_create_and_get_core_pdata(struct device_node *core_node, struct cyttsp3_ff_platform_data *pdata)
{
	u32 value = 0;
	int rc;
    printk("ztemt cyttsp3_create_and_get_core_pdata start \n");

#if 0
	/* Required fields */
	rc = of_property_read_u32(core_node, "cy,irq_gpio", &value);
	if (!rc)
		pdata->irq_gpio = value;

	rc = of_property_read_u32(core_node, "cy,rst_gpio", &value);
	if (!rc)
		pdata->rst_gpio = value;
#else
rst_gpio_number = of_get_named_gpio_flags(core_node, "cy,rst_gpio", 0, NULL);
pdata->rst_gpio = rst_gpio_number;
printk("ztemt--------pdata->reset_gpio------------: %ud\n",rst_gpio_number);
if (rst_gpio_number < 0)
    goto fail_free;

irq_gpio_number = of_get_named_gpio_flags(core_node, "cy,irq_gpio", 0, NULL);
pdata->irq_gpio = irq_gpio_number;
printk("ztemt--------pdata->irq_gpio------------: %ud\n",irq_gpio_number);
if (irq_gpio_number < 0)
    goto fail_free;

#endif

    rc = of_property_read_u32(core_node, "cy,flags", &value);
	if (!rc)
		pdata->flags = value;
    
	pdata->sett[0] = NULL;
	pdata->sett[1] = NULL;
	pdata->sett[2] = NULL;
	pdata->sett[3] = &cyttsp_sett_bl_keys;

	pdata->fw = &cyttsp3_firmware;
	pdata->frmwrk = &cyttsp3_framework;

	pdata->addr[0] = 0;
	pdata->addr[1] = 0;

	pr_debug("%s: irq_gpio:%d rst_gpio:%d flags:%d\n", __func__,
		pdata->irq_gpio, pdata->rst_gpio, pdata->flags);

	pdata->hw_reset = cyttsp3_hw_reset;
	pdata->hw_recov = cyttsp3_hw_recov;
	pdata->irq_stat = cyttsp3_irq_stat;

    printk("ztemt cyttsp3_create_and_get_core_pdata end \n");
    return 0;
fail_free:
    kfree(pdata);
    return -ENOMEM; 
}

int cyttsp3_devtree_create_and_get_pdata(struct device *adap_dev)
{
	struct cyttsp3_ff_platform_data *pdata;
	struct device_node *core_node;
	int rc = 0;
printk("ztemt cyttsp3_devtree_create_and_get_pdata start \n");
	if (!adap_dev->of_node)
		return 0;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	adap_dev->platform_data = pdata;

	/* There should be only one core node */
#if 1
	for_each_child_of_node(adap_dev->of_node, core_node) {
		const char *name;

		rc = of_property_read_string(core_node, "name", &name);
		if (!rc)
			pr_debug("%s: name:%s\n", __func__, name);

		rc = cyttsp3_create_and_get_core_pdata(core_node, pdata);
        if (rc)
            pr_debug("%s ERROR: failed to get GPIO \n", __func__);

		/* Increment reference count */
		of_node_get(core_node);
	}
#else
    cyttsp3_create_and_get_core_pdata(adap_dev->of_node, pdata);
#endif
    printk("ztemt cyttsp3_devtree_create_and_get_pdata end \n");

	return rc;
}
//EXPORT_SYMBOL_GPL(cyttsp3_devtree_create_and_get_pdata);

int cyttsp3_devtree_clean_pdata(struct device *adap_dev)
{
	struct cyttsp3_ff_platform_data *pdata;

	if (!adap_dev->of_node)
		return 0;

	pdata = dev_get_platdata(adap_dev);
	kfree(pdata);	
	return 0;
}
//EXPORT_SYMBOL_GPL(cyttsp3_devtree_clean_pdata);
