/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Model dependent functions
 * 
 */

#include "melfas_mms400.h"
extern unsigned int mms_c_zone_flag;

/*used for touchscreen rim function added by ztemt*/
#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
enum {
	ZONE_DEFAULT = 0,
    ZONE_A,
    ZONE_C,
    ZONE_B,
};

enum {
	IGNORE_TURN_A = 0,
    IGNORE_TURN_C,
    IGNORE_TURN_A_TO_C,
    IGNORE_TURN_C_TO_A,
};

struct slot {
    char slot_zoneid;/* A 0,C 1,AtoC 2,CtoA 3 */
};
struct slot slot_ignore[MAX_FINGER_NUM];

#define ZTEMT_C_AREA_WIDTH	36
#define ZTEMT_B_AREA_WIDTH  70

void zte_ignore_zone_init(void) {
	int i = 0;
    for (i = 0; i < MAX_FINGER_NUM; i++) {
        slot_ignore[i].slot_zoneid = ZONE_DEFAULT;
    }
}

/* used for judge last zone and last action added by ztemt */
static int zte_ignore_zone(int point_id, int x, int y) {
    int i = point_id;

    if((slot_ignore[i].slot_zoneid) == ZONE_C) {//front point in C zone
        if(x < ZTEMT_C_AREA_WIDTH || x > (1080 - ZTEMT_C_AREA_WIDTH)) {//in C zone
            return IGNORE_TURN_C;
        } else if(x < ZTEMT_B_AREA_WIDTH || x > (1080 - ZTEMT_B_AREA_WIDTH)) {
	        slot_ignore[i].slot_zoneid = ZONE_B;//CtoB
			TP_LOG_INFO("\nC to B\n");
	        return IGNORE_TURN_C_TO_A;
        } else {//CtoA
            slot_ignore[i].slot_zoneid = ZONE_A;
			TP_LOG_INFO("\nC to A\n");
            return IGNORE_TURN_C_TO_A;
        }
    } else if((slot_ignore[i].slot_zoneid) == ZONE_A) {//front point in A zone

		return IGNORE_TURN_A;

    } else if ((slot_ignore[i].slot_zoneid) == ZONE_B) {
		if(x < ZTEMT_C_AREA_WIDTH || x > (1080 - ZTEMT_C_AREA_WIDTH)) {//in C zone
			slot_ignore[i].slot_zoneid = ZONE_C;
			TP_LOG_INFO("\nB to C\n");
	        return IGNORE_TURN_A_TO_C;
		} else if(x < ZTEMT_B_AREA_WIDTH || x > (1080 - ZTEMT_B_AREA_WIDTH)) {
			return IGNORE_TURN_A;
		} else {
			slot_ignore[i].slot_zoneid = ZONE_A;
			TP_LOG_INFO("\nB to A\n");
	        return IGNORE_TURN_A;
		}
	}
	else {//new point (slot_ignore[i].slot_zoneid) == ZONE_DEFAULT
		if(x < ZTEMT_C_AREA_WIDTH || x > (1080 - ZTEMT_C_AREA_WIDTH)){//in C zone
		    slot_ignore[i].slot_zoneid = ZONE_C;
			TP_LOG_INFO("start from zone C\n");
		    return IGNORE_TURN_C;
		} else if (x < ZTEMT_B_AREA_WIDTH || x > (1080 - ZTEMT_B_AREA_WIDTH)) {
			slot_ignore[i].slot_zoneid = ZONE_B;//in B zone
			TP_LOG_INFO("start from zone B\n");
			return IGNORE_TURN_A;
		} else {//in A zone
		    slot_ignore[i].slot_zoneid = ZONE_A;
			TP_LOG_INFO("start from zone A\n");
		    return IGNORE_TURN_A;
		}
    }
    return -1;
}

/* used for config input rim dev added by ztemt */
void zte_config_input_rim(struct mms_ts_info *info)
{
	struct input_dev *input_dev = info->input_rim_dev;

	TP_LOG_DEBUG("start\n");

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);

	//Key
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);

	TP_LOG_DEBUG("end\n");
	return;
}
#endif
/*used for touchscreen rim function added by ztemt end*/

/* POWER SUPPLY VOLTAGE RANGE */
#define MELFAS_VDD_MIN_UV	2600000
#define MELFAS_VDD_MAX_UV	3300000
#define MELFAS_VIO_MIN_UV	1750000
#define MELFAS_VIO_MAX_UV	1950000

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

int mms_regulator_control(struct i2c_client *client, int enable)
{
	int rc = 0;
	struct device *dev = &client->dev;
    static struct regulator *vdd_ana;
    static struct regulator *vcc_i2c;

	if (enable) {
		vdd_ana = regulator_get(dev, "vdd_ana");
		if (IS_ERR(vdd_ana)) {
			rc = PTR_ERR(vdd_ana);
			TP_LOG_ERROR("failed to get regulator vdd_ana = %d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(vdd_ana) > 0) {
			rc = regulator_set_voltage(vdd_ana, MELFAS_VDD_MIN_UV, MELFAS_VDD_MAX_UV);
			if (rc) {
				TP_LOG_ERROR("failed to set regulator vdd_ana = %d\n", rc);
				goto set_vtg_vdd_ana_failed;
			}
		}

	    rc = reg_set_optimum_mode_check(vdd_ana, 15000);
	    if (rc < 0) {
	        TP_LOG_ERROR("failed to set_opt regulator vdd_ana = %d\n", rc);
	        goto set_opt_vdd_ana_failed;
	    }

	    rc = regulator_enable(vdd_ana);
	    if (rc) {
	        TP_LOG_ERROR("failed to enable regulator vdd_ana = %d\n", rc);
	        goto en_vdd_ana_failed;
	    }

		vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			rc = PTR_ERR(vcc_i2c);
			TP_LOG_ERROR("failed to get regulator vcc_i2c = %d\n", rc);
			goto get_vcc_i2c_failed;
		}

		if (regulator_count_voltages(vcc_i2c) > 0) {
			rc = regulator_set_voltage(vcc_i2c, MELFAS_VIO_MIN_UV, MELFAS_VIO_MAX_UV);
			if (rc) {
				TP_LOG_ERROR("failed to set regulator vcc_i2c = %d\n", rc);
				goto set_vtg_vcc_i2c_failed;
			}
		}

	    rc = regulator_enable(vcc_i2c);
	    if (rc) {
	        TP_LOG_ERROR("failed to enable regulator vcc_i2c = %d\n", rc);
	        goto en_vcc_i2c_failed;
	    }

	    msleep(100);
	} else {
		if (regulator_count_voltages(vcc_i2c) > 0)
			regulator_set_voltage(vcc_i2c, 0, MELFAS_VIO_MAX_UV);
			regulator_put(vcc_i2c);

		if (regulator_count_voltages(vdd_ana) > 0)
			regulator_set_voltage(vdd_ana, 0, MELFAS_VDD_MAX_UV);
		regulator_put(vdd_ana);
	}

    return 0;

en_vcc_i2c_failed:
    if (regulator_count_voltages(vcc_i2c) > 0)
		regulator_set_voltage(vcc_i2c, 0, MELFAS_VIO_MAX_UV);
set_vtg_vcc_i2c_failed:
	regulator_put(vcc_i2c);
get_vcc_i2c_failed:
    regulator_disable(vdd_ana);
en_vdd_ana_failed:
    reg_set_optimum_mode_check(vdd_ana, 0);
set_opt_vdd_ana_failed:
	if (regulator_count_voltages(vdd_ana) > 0)
		regulator_set_voltage(vdd_ana, 0, MELFAS_VDD_MAX_UV);
set_vtg_vdd_ana_failed:
	regulator_put(vdd_ana);
	return rc;
}

#define MELFAS_TP_PINCTRL_ACTIVE   "melfas_pin_active"
#define MELFAS_TP_PINCTRL_SUSPEND  "melfas_pin_suspend"

int mms_pinctrl_init(struct mms_ts_info *info)
{
    info->pdata->pin_res.pinctrl = devm_pinctrl_get(&info->client->dev);
	if (IS_ERR_OR_NULL(info->pdata->pin_res.pinctrl)) {
        TP_LOG_ERROR("failed to get pinctrl\n");
	    return -EINVAL;
	}

    info->pdata->pin_res.gpio_state_active =
		pinctrl_lookup_state(info->pdata->pin_res.pinctrl, MELFAS_TP_PINCTRL_ACTIVE);
    if (IS_ERR_OR_NULL(info->pdata->pin_res.gpio_state_active)) {
        TP_LOG_ERROR("failed to get [%s] state pinctrl\n", MELFAS_TP_PINCTRL_ACTIVE);
	    return -EINVAL;
    }

    info->pdata->pin_res.gpio_state_suspend =
		pinctrl_lookup_state(info->pdata->pin_res.pinctrl, MELFAS_TP_PINCTRL_SUSPEND);
    if (IS_ERR_OR_NULL(info->pdata->pin_res.gpio_state_suspend)) {
        TP_LOG_ERROR("failed to get [%s] state pinctrl\n", MELFAS_TP_PINCTRL_SUSPEND);
	    return -EINVAL;
    }

    TP_LOG_INFO("success to mms_pinctrl_init\n");

	return 0;
}

int mms_pinctrl_enable(struct mms_ts_info *info, bool active)
{
	int rc = -EFAULT;
	struct pinctrl_state *pin_state;

	pin_state = active ? info->pdata->pin_res.gpio_state_active : \
		info->pdata->pin_res.gpio_state_suspend;

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(info->pdata->pin_res.pinctrl, pin_state);
		if (rc)
            TP_LOG_ERROR("failed to set [%s] pins\n",
				active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
		else
            TP_LOG_ERROR("success to set [%s] pins\n",
				active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
	} else {
	    TP_LOG_ERROR("invalid [%s] pinstate\n\n",
			active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
	}

    return rc;
}

/**
* Turn off power supply
*/
int mms_power_off(struct mms_ts_info *info)
{
	if (info->power_en == false) {
		TP_LOG_INFO("already power off\n");
		return 0;
	}

	//Control reset pin
	gpio_direction_output(info->pdata->gpio_reset, 0);
	msleep(10);

	info->power_en = false;

	TP_LOG_INFO("success to power off\n");

	return 0;
}

/**
* Turn on power supply
*/
int mms_power_on(struct mms_ts_info *info)
{
	if (info->power_en == true) {
		TP_LOG_INFO("already power on\n");
		return 0;
	}

	//Control reset pin
	gpio_direction_output(info->pdata->gpio_reset, 1);
	msleep(50);

	info->power_en = true;

	TP_LOG_INFO("success to power on\n");

	return 0;
}

/**
* Clear touch input events
*/
void mms_clear_input(struct mms_ts_info *info)
{
	int i = 0;
	for (i = 0; i< MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);

		#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
		input_mt_slot(info->input_rim_dev, i);
		input_mt_report_slot_state(info->input_rim_dev, MT_TOOL_FINGER, false);
		#endif
	}
	
	input_sync(info->input_dev);

	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_sync(info->input_rim_dev);
	#endif

	return;
}

/**
* Input event handler - Report touch input event
*/
void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	int i = 0;
	struct input_dev *input_actual_dev = NULL;

	TP_LOG_DEBUG("sz[%d] buf[0x%02X]\n", sz, buf[0]);

	for (i = 1; i < sz; i += info->event_size) {
		u8 *tmp = &buf[i];

		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
		int touch_major = tmp[4];
		int pressure = tmp[5];
		int palm = (tmp[0] & 0x10) >> 4;

		#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
		int ret = 0;
		#endif

		// Report input data
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			//Touchkey Event
			int key = tmp[0] & 0xf;
			int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
			int key_code = 0;

			//Report touchkey event
			switch (key) {
				case 1:
					key_code = KEY_MENU;
					TP_LOG_DEBUG("Key : KEY_MENU\n");
					break;
				case 2:
					key_code = KEY_HOME;
					TP_LOG_DEBUG("Key : KEY_HOME\n");
					break;
				case 3:
					key_code = KEY_BACK;
					TP_LOG_DEBUG("Key : KEY_BACK\n");
					break;
				default:
					TP_LOG_ERROR("unknown key code [%d]\n", key);
					continue;
					break;
			}

			input_report_key(info->input_dev, key_code, key_state);

			TP_LOG_DEBUG("Key = %d, state = %d\n", key_code, key_state);
		} else {
			//Report touchscreen event
			/* used for touchscreen rim function added by ztemt */
			#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
			if(mms_c_zone_flag) {
				ret = zte_ignore_zone(id, x, y);

				if (ret == IGNORE_TURN_C) {//C zone
					input_actual_dev = info->input_rim_dev;
				} else if (ret == IGNORE_TURN_A) {//A zone
					input_actual_dev = info->input_dev;
				} else if (ret == IGNORE_TURN_C_TO_A) {//C-A zone
					TP_LOG_INFO("ztemt C-A! id = %d\n", id);
					input_actual_dev = info->input_rim_dev;
					input_mt_slot(input_actual_dev, id);
					input_mt_report_slot_state(input_actual_dev, MT_TOOL_FINGER, 0);
					input_actual_dev = info->input_dev;
				} else if (ret == IGNORE_TURN_A_TO_C) {//A-C zone
					TP_LOG_INFO("ztemt A-C! id = %d\n", id);
					input_actual_dev = info->input_dev;
					input_mt_slot(input_actual_dev, id);
					input_mt_report_slot_state(input_actual_dev, MT_TOOL_FINGER, 0);
					input_actual_dev = info->input_rim_dev;
				} else {
					TP_LOG_ERROR("ztemt error!\n");
					input_actual_dev = info->input_dev;
				}
			} else {
				input_actual_dev = info->input_dev;
			}
			#else
			input_actual_dev = info->input_dev;
			#endif

			input_mt_slot(input_actual_dev, id);
			if((tmp[0] & MIP_EVENT_INPUT_PRESS) != 0) {
				//Press or Move
				input_mt_report_slot_state(input_actual_dev, MT_TOOL_FINGER, true);
				input_report_abs(input_actual_dev, ABS_MT_POSITION_X, x);
				input_report_abs(input_actual_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(input_actual_dev, ABS_MT_TOUCH_MAJOR, touch_major);

				if (palm == 1) {
					pressure = 1000;
					input_report_abs(input_actual_dev, ABS_MT_PRESSURE, pressure);
					TP_LOG_ERROR("Large Palm\n");
				} else {
					input_report_abs(input_actual_dev, ABS_MT_PRESSURE, pressure);
				}

				TP_LOG_DEBUG("%s - Touch : ID[%d] X[%d] Y[%d] P[%d] M[%d]\n",
					id, x, y, pressure, touch_major);
			} else {
				//Release
				input_mt_report_slot_state(input_actual_dev, MT_TOOL_FINGER, false);

				#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
				slot_ignore[id].slot_zoneid = ZONE_DEFAULT;
				#endif

				TP_LOG_DEBUG("Touch : ID[%d] X[%d] Y[%d] P[%d] M[%d] Release\n",
					id, x, y, pressure, touch_major);
			}
		}
	}

	input_sync(info->input_dev);
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_sync(info->input_rim_dev);
	#endif

	TP_LOG_DEBUG("end\n");
	return;
}

/**
* Wake-up event handler
*/
int mms_wakeup_event_handler(struct mms_ts_info *info, u8 *rbuf)
{
	u8 gesture_type = rbuf[1];
	u8 key = 0;

	TP_LOG_DEBUG("start\n");

	switch (gesture_type)
    {
        case DOUBLE_CLICK://double knock on
			key = KEY_F10;
			TP_LOG_INFO("double click\n");
			break;
	    default:
			return 0;
			break;
    }

	input_report_key(info->input_dev, key, 1);
	input_sync(info->input_dev);
	input_report_key(info->input_dev, key, 0);
	input_sync(info->input_dev);

	TP_LOG_DEBUG("end\n");
	return 0;
}

#if MMS_USE_DEVICETREE
/**
* Parse device tree
*/
int mms_parse_devicetree(struct device *dev, struct mms_ts_info *info)
{
	struct device_node *np = dev->of_node;
	u32 val = 0;
	int ret = 0;

	TP_LOG_INFO("start\n");

	//Read property
	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_x", &val);
	if (ret) {
		info->pdata->max_x = 1080;
		TP_LOG_ERROR("failed to read max_x, use default 1080\n");
	} else {		
		info->pdata->max_x = val;
		TP_LOG_INFO("max_x = %d\n", info->pdata->max_x);
	}

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_y", &val);
	if (ret) {
		info->pdata->max_y = 1920;
		TP_LOG_ERROR("failed to read max_y, use default 1920\n");
	} else {
		info->pdata->max_y = val;
		TP_LOG_INFO("max_y = %d\n", info->pdata->max_y);
	}

	//Get GPIO
	ret = of_get_named_gpio_flags(np, MMS_DEVICE_NAME",irq-gpio", 0, NULL);
	if (!gpio_is_valid(ret)) {
		info->pdata->gpio_intr = 915;
		TP_LOG_ERROR("failed to get irq-gpio, use default 915\n");
	} else {
		info->pdata->gpio_intr = ret;
		TP_LOG_INFO("irq-gpio = %d\n", info->pdata->gpio_intr);
	}

	ret = of_get_named_gpio_flags(np, MMS_DEVICE_NAME",reset-gpio", 0, NULL);
	if (!gpio_is_valid(ret)) {
		info->pdata->gpio_reset = 914;
		TP_LOG_ERROR("failed to get reset-gpio, use default 914\n");
	} else {
		info->pdata->gpio_reset = ret;
		TP_LOG_INFO("reset-gpio = %d\n", info->pdata->gpio_reset);
	}

	//Config GPIO
	ret = gpio_request(info->pdata->gpio_intr, "mms_int");
	if (ret < 0) {
		TP_LOG_ERROR("failed to request gpio_intr\n");
		goto error_exit;
	}
	gpio_direction_input(info->pdata->gpio_intr);

	//Set IRQ
	info->client->irq = gpio_to_irq(info->pdata->gpio_intr);
	TP_LOG_INFO("irq = %d\n", info->client->irq);

	ret = gpio_request(info->pdata->gpio_reset, "mms_rst");
   	if (ret < 0) {
		TP_LOG_ERROR("failed to request gpio_reset\n");
       	goto error_request_rst;
   	}
    gpio_direction_output(info->pdata->gpio_reset, 1);

	TP_LOG_INFO("end\n");
	return 0;

error_request_rst:
	gpio_free(info->pdata->gpio_intr);
error_exit:
	TP_LOG_ERROR("failed\n");
	return 1;
}
#endif

/**
* Config input interface	
*/
void mms_config_input(struct mms_ts_info *info)
{
	struct input_dev *input_dev = info->input_dev;

	TP_LOG_DEBUG("start\n");

	//Screen
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);

	//Key
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);

	//Gesture
	input_set_capability(input_dev, EV_KEY, KEY_F10);

#if MMS_USE_NAP_MODE
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER, input_dev->keybit);
#endif

	TP_LOG_DEBUG("end\n");
	return;
}

#if MMS_USE_CALLBACK
/**
* Callback - get charger status
*/
void mms_callback_charger(struct mms_callbacks *cb, int charger_status)
{
	TP_LOG_INFO("charger_status[%d]\n", charger_status);

	//...

	TP_LOG_DEBUG("end\n");
}

/**
* Callback - add callback funtions here
*/
//...

/**
* Config callback functions
*/
void mms_config_callback(struct mms_ts_info *info)
{
	TP_LOG_DEBUG("start\n");

	info->register_callback = info->pdata->register_callback;

	//callback functions
	info->callbacks.inform_charger = mms_callback_charger;
	//info->callbacks.inform_display = mms_callback_display;
	//...

	if (info->register_callback){
		info->register_callback(&info->callbacks);
	}

	TP_LOG_DEBUG("end\n");
	return;
}
#endif
