/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C touchscreen driver.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST242
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include <linux/cyttsp3_core.h>
#include <linux/cyttsp3_ff_api.h>

#include <linux/module.h>
#include <linux/gpio.h>

#include <linux/i2c.h>
#include <linux/slab.h>
//#define BUG_CAN_NOT_ENTER_SUSPEND_RESUME
#ifdef BUG_CAN_NOT_ENTER_SUSPEND_RESUME
#include <linux/device.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
#include <linux/of_device.h>
#endif

#define CY_I2C_DATA_SIZE  128
#define CY_FF_PRESSURE_VERSION	"2015072201"

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
int cyttsp3_devtree_create_and_get_pdata(struct device *adap_dev);
int cyttsp3_devtree_clean_pdata(struct device *adap_dev);
#endif

struct cyttsp_i2c {
	struct cyttsp_bus_ops ops;
	struct i2c_client *client;
	void *ttsp_client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
};

/*** ZTEMT start ***/
extern int rst_gpio_number;
extern int irq_gpio_number;
#define CYTTSP3_I2C_RST_GPIO    gpio_to_irq(rst_gpio_number)
#define CYTTSP3_I2C_IRQ_GPIO    gpio_to_irq(irq_gpio_number)
int id_number;
/*ZTEMT end*/

/*** ZTEMT Added by lixin, 2015/12/02 ***/
struct cyttsp3_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
static struct cyttsp3_pinctrl_info cyttsp3_pctrl;

static int cyttsp3_pinctrl_init(struct device *dev)
{
	cyttsp3_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(cyttsp3_pctrl.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	cyttsp3_pctrl.gpio_state_active = pinctrl_lookup_state(
					       cyttsp3_pctrl.pinctrl,
					       CY_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(cyttsp3_pctrl.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	cyttsp3_pctrl.gpio_state_suspend = pinctrl_lookup_state(
						cyttsp3_pctrl.pinctrl,
						CY_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(cyttsp3_pctrl.gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int cyttsp_power_on(struct device *dev)
{
	int rc;
    static struct regulator *vcc_ana;
    static struct regulator *vcc_i2c;

	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 3300000, 3300000);
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
    
    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }
    
    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }


	vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}
   
	if (regulator_count_voltages(vcc_i2c) > 0)
    {
 		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
        {
			dev_err(dev, "Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}
    
    rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_i2c set_opt failed rc=%d\n", rc);
        goto error_set_vtg_i2c;
    }

    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

    msleep(100);
    
    return 0;

error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
}

/*** ZTEMT END ***/
#if 0
void cyttsp3_print_buf(struct device *dev, u8 *dptr, int size, const char *data_name)
{
	int i, k;
	const char fmt[] = "%02X ";
	int max;
    u8 pr_buf[255];
	if (!size)
		return;

	max = 128;

	pr_buf[0] = 0;
	for (i = k = 0; i < size && k < max; i++, k += 3)
		scnprintf(pr_buf + k, 255, fmt, dptr[i]);

	if (size)
		dev_err(dev, "%s:  %s[0..%d]=%s%s\n", __func__, data_name, size - 1, pr_buf, "");
	else
		dev_err(dev, "%s:  %s[]\n", __func__, data_name);
}
#endif
//#define I2C_TEST_CODE
static int ttsp_i2c_read_block_data(void *handle, u8 addr,
	u8 length, void *data)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval = 0;
	#ifdef I2C_TEST_CODE
	static int test = 0;
	unsigned short save_addr,test_addr;
	#endif

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_i2c_read_block_data_exit;
	}

	if ((length == 0) || (length > CY_I2C_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_I2C_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_i2c_read_block_data_exit;
	}

	#ifdef I2C_TEST_CODE
	if(0 == test){
		test = 1;
		save_addr = ts->client->addr;
		for(test_addr=0; test_addr<128; test_addr++){
			ts->client->addr = test_addr;
			retval = i2c_master_send(ts->client, &addr, 1);
			if (retval != 1)
				pr_err("I2C_TEST_CODE: addr=0x%x fail\n", test_addr);
			else
				pr_err("I2C_TEST_CODE: addr=0x%x ok\n", test_addr);
			msleep(5);
		}
		ts->client->addr = save_addr;
	}
	#endif

	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		goto ttsp_i2c_read_block_data_exit;
	else if (retval != 1) {
		retval = -EIO;
		goto ttsp_i2c_read_block_data_exit;
	}

	retval = i2c_master_recv(ts->client, data, length);

ttsp_i2c_read_block_data_exit:
	return (retval < 0) ? retval : retval != length ? -EIO : 0;
}

static int ttsp_i2c_write_block_data(void *handle, u8 addr,
	u8 length, const void *data)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval = 0;

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_i2c_write_block_data_exit;
	}

	if ((length == 0) || (length > CY_I2C_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_I2C_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_i2c_write_block_data_exit;
	}

	ts->wr_buf[0] = addr;
	memcpy(&ts->wr_buf[1], data, length);

//	cyttsp3_print_buf(&(ts->client->dev), ts->wr_buf, length+1, "write_block_data");

	retval = i2c_master_send(ts->client, ts->wr_buf, length+1);

ttsp_i2c_write_block_data_exit:
	return (retval < 0) ? retval : retval != length+1 ? -EIO : 0;
}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
static struct of_device_id cyttsp3_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp3-i2c", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp3_i2c_of_match);
#endif

static int cyttsp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif	
	int retval = 0;

	pr_info("%s: Starting %s probe...\n", __func__, CY_I2C_NAME);

/*** ZTEMT Added by lixin, 2015/12/02 ***/
        retval = cyttsp3_pinctrl_init(&client->dev);
        if (retval < 0){
            return retval;
        }
            retval = pinctrl_select_state(cyttsp3_pctrl.pinctrl,cyttsp3_pctrl.gpio_state_active);
        if (retval){
            pr_err("%s:%d cyttsp3 cannot set pin to gpio_state_active state\n",__func__, __LINE__);
            return -EIO;
        }
/*** ZTEMT END ***/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: fail check I2C functionality\n", __func__);
		retval = -EIO;
		goto cyttsp_i2c_probe_exit;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp3_i2c_of_match), dev);
	if (match) {
		retval = cyttsp3_devtree_create_and_get_pdata(dev);
		if (retval < 0)
			return retval;
	}
#endif	

    retval = cyttsp_power_on(&client->dev);
    if (retval){
                pr_err("%s:%d cyttsp3 power on failed \n",__func__, __LINE__);
                return -EIO;
            }

	/* allocate and clear memory */
	ts = kzalloc(sizeof(struct cyttsp_i2c), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto cyttsp_i2c_probe_exit;
	}

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.dev = &client->dev;
	ts->ops.dev->bus = &i2c_bus_type;

	ts->ttsp_client = cyttsp_core_init(&ts->ops, &client->dev,
		client->irq, client->name);

	if (ts->ttsp_client == NULL) {
		kfree(ts);
		ts = NULL;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
		if (match)
			cyttsp3_devtree_clean_pdata(dev);
#endif
		retval = -ENODATA;
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		goto cyttsp_i2c_probe_exit;
	}

/*** ZTEMT start ***/
	if(!gpio_is_valid(rst_gpio_number))
		return -ENODEV;
   	 retval = gpio_request(rst_gpio_number, "CYTTSP3_I2C_RST_GPIO");
    	if (retval < 0) 
    	{
        	printk("Failed to request GPIO:%d, ERRNO:%d", (s32)rst_gpio_number, retval);
        	retval = -ENODEV;
    	}
   	    else
   	    {
        	gpio_direction_output(rst_gpio_number, 1);
		
        }

	if(!gpio_is_valid(irq_gpio_number))
		return -ENODEV;

	retval = gpio_request(irq_gpio_number, "CYTTSP3_I2C_IRQ_GPIO");
	if (retval < 0) {
			printk("Failed request CYTTSP3_I2C_IRQ_GPIO.\n");
			return -ENODEV;
		}
	gpio_direction_input(irq_gpio_number);

	id_number = 0;

/*ZTEMT end*/

	pr_info("%s: Registration complete\n", __func__);

cyttsp_i2c_probe_exit:
	return retval;
}

/* registered in driver struct */
static int cyttsp_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif
	struct cyttsp_i2c *ts;
	int retval = 0;

	ts = i2c_get_clientdata(client);
	if (ts == NULL) {
		pr_err("%s: client pointer error\n", __func__);
		retval = -EINVAL;
	} else {
		cyttsp_core_release(ts->ttsp_client);
		
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
		match = of_match_device(of_match_ptr(cyttsp3_i2c_of_match), dev);
		if (match)
			cyttsp3_devtree_clean_pdata(dev);
#endif
		kfree(ts);
	}
	return retval;
}

#ifdef BUG_CAN_NOT_ENTER_SUSPEND_RESUME
static int cyttsp_i2c_suspend(struct device *dev)
{
	void *ts = dev_get_drvdata(dev);//struct cyttsp_i2c *ts = dev_get_drvdata(dev);
	int rc;

	//pr_info("%s: cyttsp_i2c = %p\n", __func__, ts);
	rc = cyttsp_suspend(ts);
	if (rc == 0)
		pr_info("%s\n", __func__);//_cyttsp3_force_function.FT_ready_flag = false;
	else
		pr_err("%s: ERROR ztemt suspend false\n", __func__);
	return rc;
}

static int cyttsp_i2c_resume(struct device *dev)
{
    void *ts = dev_get_drvdata(dev);//struct cyttsp_i2c *ts = dev_get_drvdata(dev);
	int rc;

	//pr_info("%s: cyttsp_i2c = %p\n", __func__, ts);
	rc = cyttsp_resume(ts);
	if (rc == 0)
		pr_info("%s\n", __func__);//_cyttsp3_force_function.FT_ready_flag = true;
	else
		pr_err("%s: ERROR ztemt resume false\n", __func__);
	return rc;
}

const struct dev_pm_ops cyttsp3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp_i2c_suspend, cyttsp_i2c_resume)
};
#else
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cyttsp_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
//	struct cyttsp_i2c *ts = i2c_get_clientdata(client);
//	pr_info("%s: ztemt cyttsp_i2c_suspend\n",__func__);
//	return cyttsp_suspend(ts);
	return 0;
}

static int cyttsp_i2c_resume(struct i2c_client *client)
{
//	struct cyttsp_i2c *ts = i2c_get_clientdata(client);
//	pr_info("%s: ztemt cyttsp_i2c_resume\n",__func__);
//	return cyttsp_resume(ts);
	return 0;
}
#endif
#endif

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
		.of_match_table = cyttsp3_i2c_of_match,
#endif		
#ifdef BUG_CAN_NOT_ENTER_SUSPEND_RESUME
		.pm = &cyttsp3_pm_ops,
#endif
	},
	.probe = cyttsp_i2c_probe,
	.remove = cyttsp_i2c_remove,
	.id_table = cyttsp_i2c_id,
#ifndef BUG_CAN_NOT_ENTER_SUSPEND_RESUME
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cyttsp_i2c_suspend,
	.resume = cyttsp_i2c_resume,
#endif
#endif
};

static int __init cyttsp_i2c_init(void)
{
	int rc = i2c_add_driver(&cyttsp_i2c_driver);

	pr_info("%s: Cypress Force Function I2C Driver (Built %s) rc=%d\n",
		 __func__, CY_FF_PRESSURE_VERSION, rc);

	return rc;
}

static void __exit cyttsp_i2c_exit(void)
{
	i2c_del_driver(&cyttsp_i2c_driver);
}

fs_initcall_sync(cyttsp_i2c_init);
//module_init(cyttsp_i2c_init);
module_exit(cyttsp_i2c_exit);

//MODULE_ALIAS("i2c:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");

