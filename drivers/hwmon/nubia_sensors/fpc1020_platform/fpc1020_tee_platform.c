/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

//Added by nubia
#define FPC1020_DEBUG
#define FPC1020_NUBIA_MODIFY

#ifdef FPC1020_NUBIA_MODIFY
#define FPC1020_DEV_NAME        "fpc"
#define FPC1020_CLASS_NAME      "fpc"
#define FPC1020_DEV_MAJOR       154
#endif

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
	"fpc1020_ldo_enable",
	"fpc1020_ldo_disable",
};

struct fpc1020_data {
	struct device *dev;
	struct platform_device *pdev;
#ifdef FPC1020_NUBIA_MODIFY
	struct class *class;
	dev_t ndev;
#endif
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];	
	int irq_gpio;
	int rst_gpio;
	struct input_dev *idev;
	int irq_num;
	char idev_name[32];
	int event_type;
	int event_code;
	struct mutex lock;
	bool prepared;
};


/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;

#ifdef FPC1020_DEBUG
	printk("[fpc]%s(%d):name=%s\n", __func__, __LINE__, name);
#endif

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = select_pin_ctl(fpc1020, buf);
	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

#ifdef FPC1020_NUBIA_MODIFY
static int fpc_power_setup(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;

#ifdef FPC1020_DEBUG
	printk("[fpc]%s(%d):enable=%d\n", __func__, __LINE__, enable);
#endif

	if (enable) {
		rc = select_pin_ctl(fpc1020, "fpc1020_ldo_enable");
		msleep(10);
	} else {
		rc = select_pin_ctl(fpc1020, "fpc1020_ldo_disable");
	}

	return rc;

}

static ssize_t power_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = fpc_power_setup(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = fpc_power_setup(fpc1020, false);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(power_enable, S_IWUSR, NULL, power_enable_set);
#endif

static int hw_reset(struct  fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * Will setup clocks, GPIOs, and regulators to correctly initialize the touch
 * sensor to be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, SPI platform clocks, and reset line, all to set
 * the sensor in a correct power on or off state "electrical" wise.
 *
 * @see  spi_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct  fpc1020_data *fpc1020, bool enable)
{
	int rc;

#ifdef FPC1020_DEBUG
	printk("[fpc]%s(%d):enable=%d\n", __func__, __LINE__, enable);
#endif

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

#ifdef FPC1020_NUBIA_MODIFY
		select_pin_ctl(fpc1020, "fpc1020_ldo_enable");
#endif

		usleep_range(100, 1000);

		(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");
		usleep_range(100, 200);

	} else if (!enable && fpc1020->prepared) {
		rc = 0;

		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		usleep_range(100, 1000);

#ifdef FPC1020_NUBIA_MODIFY
		select_pin_ctl(fpc1020, "fpc1020_ldo_disable");
#endif

		fpc1020->prepared = false;
	} else {
		rc = 0;
	}
	mutex_unlock(&fpc1020->lock);
	return rc;
}

static ssize_t dev_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	printk("[fpc]%s(%d):buf=%s\n", __func__, __LINE__, buf);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		//rc = device_prepare(fpc1020, false);
		;
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(dev_prepare, S_IWUSR, NULL, dev_prepare_set);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_dev_prepare.attr,
#ifdef FPC1020_NUBIA_MODIFY
	&dev_attr_power_enable.attr,
#endif
	&dev_attr_hw_reset.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	input_event(fpc1020->idev, EV_MSC, MSC_SCAN, ++fpc1020->irq_num);
	input_sync(fpc1020->idev);
	dev_dbg(fpc1020->dev, "%s %d\n", __func__, fpc1020->irq_num);
	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

#ifdef FPC1020_DEBUG
	printk("%s(%d):label=%s, rc=%d\n", __func__, __LINE__, label, rc);
#endif

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);
	return 0;
}

static int fpc_open(struct inode *inode, struct file *filp)
{
	int rc = 0;

#ifdef FPC1020_DEBUG
	printk("%s(%d)\n", __func__, __LINE__);
#endif

	return rc;
}

static const struct file_operations fpc_fops = {
	.owner  = THIS_MODULE,
	.open   = fpc_open,
};

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	size_t i = 0;
	int irqf = 0;
	struct device_node *np = dev->of_node;
	u32 val = 0;
	const char *idev_name = NULL;

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	fpc1020->pdev = pdev;

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

#ifdef FPC1020_NUBIA_MODIFY
	rc = select_pin_ctl(fpc1020, "fpc1020_ldo_enable");
	if (rc)
		goto exit;
#endif

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;

	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;

	rc = of_property_read_u32(np, "fpc,event-type", &val);
	fpc1020->event_type = rc < 0 ? EV_MSC : val;

	rc = of_property_read_u32(np, "fpc,event-code", &val);
	fpc1020->event_code = rc < 0 ? MSC_SCAN : val;

	fpc1020->idev = devm_input_allocate_device(dev);
	if (!fpc1020->idev) {
		dev_err(dev, "failed to allocate input device\n");
		rc = -ENOMEM;
		goto exit;
	}
	input_set_capability(fpc1020->idev, fpc1020->event_type,
			fpc1020->event_code);

	if (!of_property_read_string(np, "input-device-name", &idev_name)) {
		fpc1020->idev->name = idev_name;
	} else {
		snprintf(fpc1020->idev_name, sizeof(fpc1020->idev_name),
			"fpc1020@%s", dev_name(dev));
		fpc1020->idev->name = fpc1020->idev_name;
	}
	rc = input_register_device(fpc1020->idev);
	if (rc) {
		dev_err(dev, "failed to register input device\n");
		goto exit;
	}

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

#ifdef FPC1020_NUBIA_MODIFY
	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);
	fpc1020->ndev = MKDEV(FPC1020_DEV_MAJOR, 0);
	device_create(fpc1020->class, NULL, fpc1020->ndev, NULL, FPC1020_DEV_NAME);
	rc = register_chrdev(MAJOR(fpc1020->ndev), FPC1020_DEV_NAME, &fpc_fops);
	if (rc) {
		dev_err(dev, "failed to register char device\n");
		goto exit;
	}
#endif

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		(void)device_prepare(fpc1020, true);
	}

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);

#ifdef FPC1020_NUBIA_MODIFY
	select_pin_ctl(fpc1020, "fpc1020_ldo_disable");
#endif

#ifdef FPC1020_NUBIA_MODIFY
	unregister_chrdev(MAJOR(fpc1020->ndev), FPC1020_DEV_NAME);
	device_destroy(fpc1020->class, fpc1020->ndev);
	class_destroy(fpc1020->class);
#endif

	dev_info(&pdev->dev, "%s\n", __func__);
	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);
	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
