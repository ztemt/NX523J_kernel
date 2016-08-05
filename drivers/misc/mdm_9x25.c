/* Copyright (c) 2015-2020, Code Nubia. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 * author xue.xiaojun@zte.com.cn
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include "linux/mdm_9x25.h"

struct pinctrl_inf{
    struct pinctrl *ext_modem_pinctrl;
	struct pinctrl_state *interrupt_gpio_init_state;
};

struct ext_modem_platform_data{
	struct mutex lock;
	struct dentry *dbg_dir;
    unsigned int gpio_power_on; //14
    unsigned int gpio_boost_enable; //41
    unsigned int gpio_sleep; //54
    unsigned int gpio_usb_switch;//62
    
    unsigned int gpio_usb_switch_two;//62
	unsigned int gpio_status_control;
    unsigned int usbswith_po;
	unsigned int gpio_status_irq; 
	u16 use_pinctrl;
    struct pinctrl_inf pctrl_info;
	struct notifier_block ext_mdm_notify;
};

/* reset modem */
static int ext_modem_pon_show(void *data, u64 *val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}
    
	return 0;
}

static int ext_modem_pon_store(void *data, u64 val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}
  
   gpio_direction_output(p_data->gpio_power_on,val);
   
	return 0;
}

static int ext_modem_sleep_show(void *data, u64 *val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int ext_modem_sleep_store(void *data, u64 val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

   
   gpio_direction_output(p_data->gpio_sleep,val);
   
	return 0;
}


static int ext_modem_boost_show(void *data, u64 *val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}


	return 0;
}

static int ext_modem_boost_store(void *data, u64 val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

      
   gpio_direction_output(p_data->gpio_boost_enable,val);

  
	return 0;
}


static int ext_modem_usbswitch_show(void *data, u64 *val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

	*val = p_data->gpio_usb_switch;

	return 0;
}

static int ext_modem_usbswitch_store(void *data, u64 val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

   
   gpio_direction_output(p_data->gpio_usb_switch,val);
  
	return 0;
}
static int ext_modem_usbswitch_two_show(void *data, u64 *val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

	*val = p_data->gpio_usb_switch_two;

	return 0;
}

static int ext_modem_usbswitch_two_store(void *data, u64 val)
{
	struct ext_modem_platform_data *p_data = data;

	if (!p_data) {
		pr_warning("%s: p_data struct not found\n", __func__);
		return -EINVAL;
	}

   
   gpio_direction_output(p_data->gpio_usb_switch_two,val);
  
	return 0;
}




static BLOCKING_NOTIFIER_HEAD(ext_modem_chain_head);

int register_ext_modem_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ext_modem_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_ext_modem_notifier);

int unregister_ext_modem_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ext_modem_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_ext_modem_notifier);

int ext_modem_notifier_call_chain(unsigned long val)
{
	int ret = blocking_notifier_call_chain(&ext_modem_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

EXPORT_SYMBOL_GPL(ext_modem_notifier_call_chain);



DEFINE_SIMPLE_ATTRIBUTE(ext_modem_pon_fops, ext_modem_pon_show,
			ext_modem_pon_store, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(ext_modem_sleep_fops, ext_modem_sleep_show,
			ext_modem_sleep_store, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(ext_modem_boost_fops, ext_modem_boost_show,
			ext_modem_boost_store, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ext_modem_usbswitch_fops, ext_modem_usbswitch_show,
			ext_modem_usbswitch_store, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(ext_modem_usbswitch_two_fops, ext_modem_usbswitch_two_show,
			ext_modem_usbswitch_two_store, "%llu\n");


static int ext_modem_get_pinctrl(struct device *dev,struct ext_modem_platform_data *p_data)
{

    int ret = 0;
	p_data->pctrl_info.ext_modem_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(p_data->pctrl_info.ext_modem_pinctrl)) {
		dev_err(dev,"%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}

    /* get all the states handles from Device Tree */
	p_data->pctrl_info.interrupt_gpio_init_state = pinctrl_lookup_state(p_data->pctrl_info.ext_modem_pinctrl, "init_state");
	if (IS_ERR(p_data->pctrl_info.interrupt_gpio_init_state)) {
		dev_err(dev, "%s: Unable to get pinctrl disable state handle, err: %ld\n",
				__func__, PTR_ERR(p_data->pctrl_info.interrupt_gpio_init_state));
		return -EINVAL;
	}

    ret = pinctrl_select_state(p_data->pctrl_info.ext_modem_pinctrl,
         p_data->pctrl_info.interrupt_gpio_init_state);
	if (ret < 0){
		dev_err(dev,"%s: Failed to select pinstate %d\n",
			__func__, ret);
    }

	return 0;
}


static int ext_modem_parse_dt(struct platform_device *pdev,struct ext_modem_platform_data *pdata)
{
	struct device_node *np = pdev->dev.of_node;
	pdata->gpio_power_on = of_get_named_gpio(np, "ext_modem,po-gpio", 0);
	pdata->gpio_boost_enable = of_get_named_gpio(np, "ext_modem,usb_boost_enable", 0);
	pdata->gpio_sleep = of_get_named_gpio(np, "ext_modem,sleep-gpio", 0);
	pdata->gpio_usb_switch = of_get_named_gpio(np, "ext_modem,usb-switch", 0);
	pdata->gpio_usb_switch_two = of_get_named_gpio(np, "ext_modem,usb-switch_two", 0);
    pdata->gpio_status_control = of_get_named_gpio(np, "ext_modem,status-control", 0);
    pdata->usbswith_po = of_get_named_gpio(np, "ext_modem,usbswith-po", 0);
    return 0;
}

int ext_modem_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
	struct ext_modem_platform_data *pdata = container_of(
		notify_block, struct ext_modem_platform_data, ext_mdm_notify);
    printk(KERN_ERR"ext_modem_notify %d\n\r",(int)mode);
	switch (mode) {
	case EXT_MODEM_USBSWITCH_HIGH:
        
        gpio_direction_output(pdata->gpio_usb_switch,1);
		break;
    case EXT_MODEM_USBSWITCH_LOW:
        
        gpio_direction_output(pdata->gpio_usb_switch,0);
        break;
    case EXT_MODEM_BOOST_HIGH:
        
        gpio_direction_output(pdata->gpio_boost_enable,1);
		break;
    case EXT_MODEM_BOOST_LOW:
        
        gpio_direction_output(pdata->gpio_boost_enable,0);
            break;

	default:
		break;
	}

	return NOTIFY_OK;
}


static irqreturn_t ext_modem_interrupt(int irq, void *dev_id)
{
	struct ext_modem_platform_data *p_data = dev_id;
	int val = gpio_get_value_cansleep(p_data->gpio_status_control);

    printk(KERN_ERR"ext_modem_interrupt %d",val);
    if(val == 1){
    /**9x25 connect with 8976, so disable boost, pull up s1, turn off S2, and then
      enable boost**/
      
    udelay(3000);
    gpio_direction_output(p_data->gpio_boost_enable,0);
    
    gpio_direction_output(p_data->gpio_usb_switch,1);
    gpio_direction_output(p_data->gpio_usb_switch_two,0);

    udelay(1000);
    gpio_direction_output(p_data->gpio_boost_enable,1);

    } else{
    /**pc connect with 9x25, so disable boost, low down S1 switch. turn on
     s2 and then enable boost **/
     
   udelay(3000);
   gpio_direction_output(p_data->gpio_boost_enable,0);
        
   gpio_direction_output(p_data->gpio_usb_switch,0);

   
   gpio_direction_output(p_data->gpio_usb_switch_two,1);
   udelay(1000);
   gpio_direction_output(p_data->gpio_boost_enable,1);

    }
	return IRQ_HANDLED;
}



static int __init ext_modem_probe(struct platform_device *pdev)
{
    
    struct ext_modem_platform_data *p_data = NULL;
    
    int ret = 0;

    p_data = kzalloc(sizeof(struct ext_modem_platform_data),GFP_KERNEL);

    ret = ext_modem_get_pinctrl(&pdev->dev,p_data);
    if(ret < 0){
        goto free_mem;
    }
    
    ext_modem_parse_dt(pdev,p_data);
    if (p_data->gpio_power_on && gpio_is_valid(p_data->gpio_power_on)) 
        gpio_request(p_data->gpio_power_on, "extm-po");
    if (p_data->gpio_boost_enable && gpio_is_valid(p_data->gpio_boost_enable))
        gpio_request(p_data->gpio_boost_enable, "extm-boost");
    if (p_data->gpio_sleep && gpio_is_valid(p_data->gpio_sleep))
        gpio_request(p_data->gpio_sleep, "extm-sleep");
    if (p_data->gpio_usb_switch && gpio_is_valid(p_data->gpio_usb_switch))
        gpio_request(p_data->gpio_usb_switch, "extm-usbswitch");
    if (p_data->gpio_usb_switch_two && gpio_is_valid(p_data->gpio_usb_switch_two))
        gpio_request(p_data->gpio_usb_switch_two, "extm-usbswitch_two");
    
    if (p_data->usbswith_po && gpio_is_valid(p_data->usbswith_po))
        gpio_request(p_data->usbswith_po, "extm-usb_po");
    
    gpio_direction_output(p_data->gpio_power_on, 0);
    msleep(5500);        
    gpio_direction_output(p_data->gpio_power_on, 1);
    gpio_direction_output(p_data->gpio_usb_switch, 1);
    gpio_direction_output(p_data->gpio_usb_switch_two,0);
    gpio_direction_output(p_data->gpio_boost_enable, 1);
    gpio_direction_output(p_data->gpio_sleep, 0);      
    gpio_direction_output(p_data->usbswith_po, 1);      

 
	p_data->dbg_dir = debugfs_create_dir("ext-modem", NULL);
	if (IS_ERR_OR_NULL(p_data->dbg_dir)) {
		dev_err(&pdev->dev, "%s: Unable to create debugfs directory\n",
			__func__);
		ret = PTR_ERR(p_data->dbg_dir);
      goto  free_gpio;
	}

	(void) debugfs_create_file("mdm-pon", S_IRUGO | S_IWUSR,
			p_data->dbg_dir, (void *)p_data, &ext_modem_pon_fops);
	(void) debugfs_create_file("mdm-sleep", S_IRUGO | S_IWUSR,
			p_data->dbg_dir, (void *)p_data, &ext_modem_sleep_fops);
    (void) debugfs_create_file("mdm-boost", S_IRUGO | S_IWUSR,
			p_data->dbg_dir, (void *)p_data, &ext_modem_boost_fops);
    (void) debugfs_create_file("mdm-usbswitch", S_IRUGO | S_IWUSR,
			p_data->dbg_dir, (void *)p_data, &ext_modem_usbswitch_fops);
    (void) debugfs_create_file("mdm-usbswitch-two", S_IRUGO | S_IWUSR,
			p_data->dbg_dir, (void *)p_data, &ext_modem_usbswitch_two_fops);

	ret = gpio_request(p_data->gpio_status_control, "extmodem-irq");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request extmodem-irq gpio\n");
		goto release_debugfs;
	}
    
    ret = gpio_direction_input(p_data->gpio_status_control);
    if (ret) {
        dev_err(&pdev->dev,"unable to set dir for gpio[%d],ret=%d\n",
            p_data->gpio_status_control, ret);
        goto free_extmodem_gpio;
    }

	p_data->gpio_status_irq= gpio_to_irq(p_data->gpio_status_control);
	if (p_data->gpio_status_irq < 0) {
		dev_err(&pdev->dev,
			"Failed to get irq number for extmodem-irq gpio\n");
		goto free_extmodem_gpio;
	}

    ret = request_irq(p_data->gpio_status_irq,
              ext_modem_interrupt, IRQ_TYPE_EDGE_BOTH,
              "extmodem-irq", p_data);
    
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Can't allocate extmodem-irq irq %d, ret= %d\n", p_data->gpio_status_irq,ret);
		goto free_extmodem_gpio;
	}

    
	p_data->ext_mdm_notify.notifier_call = ext_modem_notify;

    register_ext_modem_notifier(&p_data->ext_mdm_notify);

	platform_set_drvdata(pdev, p_data);

    printk(KERN_ERR"ext_modem_probe success!!!\r\n");
    return 0;

free_extmodem_gpio:
    gpio_free(p_data->gpio_status_control);

release_debugfs:
    
	debugfs_remove_recursive(p_data->dbg_dir); 
    
free_gpio:
    gpio_free(p_data->gpio_power_on);
    gpio_free(p_data->gpio_boost_enable);
    gpio_free(p_data->gpio_sleep);
    gpio_free(p_data->gpio_usb_switch);

free_mem:
    
    kfree(p_data);
    return ret;
}

static int ext_modem_remove(struct platform_device *pdev)
{
    struct ext_modem_platform_data *p_data = platform_get_drvdata(pdev);

    debugfs_remove_recursive(p_data->dbg_dir);
    gpio_free(p_data->gpio_power_on);
    gpio_free(p_data->gpio_boost_enable);
    gpio_free(p_data->gpio_sleep);
    gpio_free(p_data->gpio_usb_switch);
    pinctrl_put(p_data->pctrl_info.ext_modem_pinctrl);

    unregister_ext_modem_notifier(&p_data->ext_mdm_notify);

    kfree(p_data);

    return 0;
}

static void ext_modem_shutdown(struct platform_device *pdev)
{
    struct ext_modem_platform_data *p_data = platform_get_drvdata(pdev);

     
    kfree(p_data);
}

#ifdef CONFIG_PM_SLEEP
static int ext_modem_suspend(struct ext_modem_platform_data *p_data)
{
     gpio_direction_output(p_data->gpio_sleep,0);
	return 0;
}

static int ext_modem_resume(struct ext_modem_platform_data *p_data)
{
     gpio_direction_output(p_data->gpio_sleep, 1);
	return 0;
}
#endif




#ifdef CONFIG_PM_SLEEP
static int ext_modem_pm_suspend(struct device *dev)
{
	struct ext_modem_platform_data *p_data = dev_get_drvdata(dev);
    ext_modem_suspend(p_data);

	return 0;
}

static int ext_modem_pm_suspend_noirq(struct device *dev)
{
	//struct ext_modem_platform_data *p_data = dev_get_drvdata(dev);


	return 0;
}

static int ext_modem_pm_resume(struct device *dev)
{
	struct ext_modem_platform_data *p_data = dev_get_drvdata(dev);

    ext_modem_resume(p_data);

	dev_dbg(dev, "ext_modem  PM resume\n");

	return 0;
}
#endif


#ifdef CONFIG_PM_RUNTIME
static int ext_modem_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "EHCI runtime idle\n");
	return 0;
}

static int ext_modem_runtime_suspend(struct device *dev)
{
	struct ext_modem_platform_data *p_data = dev_get_drvdata(dev);

    ext_modem_suspend(p_data);

    return 0;
}

static int ext_modem_runtime_resume(struct device *dev)
{

	struct ext_modem_platform_data *p_data = dev_get_drvdata(dev);
    ext_modem_resume(p_data);
    return 0;
}
#endif



#ifdef CONFIG_PM
static const struct dev_pm_ops ext_modem_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ext_modem_pm_suspend, ext_modem_pm_resume)
	.suspend_noirq = ext_modem_pm_suspend_noirq,
	SET_RUNTIME_PM_OPS(ext_modem_runtime_suspend, ext_modem_runtime_resume,
				ext_modem_runtime_idle)
};
#endif


static const struct of_device_id ext_mdm_dt_match[] = {
	{ .compatible = "qcom,ext_mdm", },
	{}
};



static struct platform_driver ext_modem_driver = {
	.remove         = ext_modem_remove,
	.shutdown	= ext_modem_shutdown,
	.driver         = {
		.name = "ext_modem",     
#ifdef CONFIG_PM
        .pm = &ext_modem_dev_pm_ops,
#endif
		.owner = THIS_MODULE,
		.of_match_table = ext_mdm_dt_match,
	},
};

static int __init ext_modem_init(void)
{
	return platform_driver_probe(&ext_modem_driver, ext_modem_probe);
}

static void __exit ext_modem_exit(void)
{
	platform_driver_unregister(&ext_modem_driver);
}

module_init(ext_modem_init);
module_exit(ext_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ext modem driver");
MODULE_VERSION("2.0");
MODULE_ALIAS("ext_modem");
