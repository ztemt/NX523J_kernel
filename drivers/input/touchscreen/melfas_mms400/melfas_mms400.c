/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * This module is tested on the Google AOSP (Nexus) platforms.
 *
 * Board Type : Maguro (Google Galaxy Nexus) - Android 4.3 with Kernel 3.0
 * DeviceTree Type : Hammerhead (Google Nexus 5) - Android 5.0 with Kernel 3.4
 *
 */

#include "melfas_mms400.h"

#if MMS_USE_NAP_MODE
struct wake_lock mms_wake_lock;
#endif

static unsigned int mms_wakeup_control_flag = 0;
static unsigned int mms_touch_mode_flag = 0;
unsigned int mms_c_zone_flag = 0;

static void mms_setup_fb_notifier(struct mms_ts_info *info);

/**
* Reboot chip
*
* Caution : IRQ must be disabled before mms_reboot and enabled after mms_reboot.
*/
void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);

	mms_power_off(info);
	mms_power_on(info);

	i2c_unlock_adapter(adapter);

	msleep(10);

	TP_LOG_INFO("success to reboot\n");
}

/**
* I2C Read
*/
int mms_i2c_read(struct mms_ts_info *info, char *write_buf,
	unsigned int write_len, char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res = 0;

	struct i2c_msg msg[] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

	while (retry--) {
		res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

		if (res == ARRAY_SIZE(msg)) {
			goto DONE;
		} else if (res < 0) {
			TP_LOG_ERROR("failed to i2c_transfer - errno[%d]\n", res);
		} else if (res != ARRAY_SIZE(msg)) {
			TP_LOG_ERROR("failed to i2c_transfer - size[%d] result[%d]\n",
				(int)ARRAY_SIZE(msg), res);
		} else {
			TP_LOG_ERROR("failed to i2c_transfer - unknown error [%d]\n", res);
		}
	}

	mms_reboot(info);
	return 1;

DONE:
	return 0;
}

/**
* I2C Read (Continue)
*/
int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf,
	int start_idx, unsigned int read_len)
{
	#define MAX_READ_LEN 255
	int retry = I2C_RETRY_COUNT;
	int res = 0;
	u8 rbuf[MAX_READ_LEN] = { 0 };

	while (retry--) {
		res = i2c_master_recv(info->client, rbuf, read_len);

		if (res == read_len) {
			goto DONE;
		} else if (res < 0) {
			TP_LOG_ERROR("failed to i2c_master_recv - errno [%d]\n", res);
		} else if (res != read_len) {
			TP_LOG_ERROR("failed to length match - read[%d] result[%d]\n",
				read_len, res);
		} else {
			TP_LOG_ERROR("failed to i2c_master_recv - unknown error [%d]\n", res);
		}
	}

	mms_reboot(info);
	return 1;

DONE:
	memcpy(&read_buf[start_idx], rbuf, read_len);
	return 0;
}

/**
* I2C Write
*/
int mms_i2c_write(struct mms_ts_info *info, char *write_buf,
	unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res = 0;

	while (retry--) {
		res = i2c_master_send(info->client, write_buf, write_len);

		if (res == write_len) {
			goto DONE;
		} else if (res < 0) {
			TP_LOG_ERROR("failed to i2c_master_send - errno [%d]\n", res);
		} else if (res != write_len) {
			TP_LOG_ERROR("failed to length match - write[%d] result[%d]\n",
				write_len, res);
		} else {
			TP_LOG_ERROR("failed to i2c_master_send - unknown error [%d]\n", res);
		}
	}

	mms_reboot(info);
	return 1;

DONE:
	return 0;
}

static s32 irq_is_disable = 0;
static s32 irq_is_wake = 0;

/**
 * mms_irq_enable - enable irq function.
 *
 */
void mms_irq_enable(int irq)
{
	if (irq_is_disable) {
		enable_irq(irq);
		irq_is_disable = 0;
	}
}

/**
 * mms_irq_disable - disable irq function.
 *
 */
void mms_irq_disable(int irq)
{
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(irq);
	}
}

/**
 * mms_irq_wake_enable - enable irq wake function.
 *
 */
void mms_irq_wake_enable(int irq)
{
	if (!irq_is_wake) {
		enable_irq_wake(irq);
		irq_is_wake = 1;
	}
}

/**
 * mms_irq_wake_disable - disable irq wake function.
 *
 */
void mms_irq_wake_disable(int irq)
{
	if (irq_is_wake) {
		irq_is_wake = 0;
		disable_irq_wake(irq);
	}
}


/**
* Enable device
*/
int mms_enable(struct mms_ts_info *info)
{
	if (info->enabled) {
		TP_LOG_ERROR("device already enabled\n");
		goto EXIT;
	}

	mutex_lock(&info->lock);

	mms_power_on(info);

	mms_irq_enable(info->client->irq);

	info->enabled = true;

	mutex_unlock(&info->lock);

	//Post-enable process
	if (info->disable_esd == true) {
		//Disable ESD alert
		mms_disable_esd_alert(info);
	}

EXIT:
	TP_LOG_INFO("success to enable\n");
	return 0;
}

/**
* Disable device
*/
int mms_disable(struct mms_ts_info *info)
{
	if (!info->enabled){
		TP_LOG_ERROR("device already disabled\n");
		goto EXIT;
	}

	mutex_lock(&info->lock);

	info->enabled = false;

	mms_irq_disable(info->client->irq);
	mms_power_off(info);

	mutex_unlock(&info->lock);

EXIT:
	TP_LOG_INFO("success to disable\n");
	return 0;
}

#if MMS_USE_INPUT_OPEN_CLOSE
/**
* Open input device
*/
static int mms_input_open(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	TP_LOG_INFO("start\n");

	if (info->init == true) {
		info->init = false;
	} else {
		mms_enable(info);
	}

	TP_LOG_INFO("end\n");

	return 0;
}

/**
* Close input device
*/
static void mms_input_close(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	TP_LOG_INFO("start\n");

	mms_disable(info);

	TP_LOG_INFO("end\n");

	return;
}
#endif

/**
* Get ready status
*/
int mms_get_ready_status(struct mms_ts_info *info)
{
	u8 wbuf[16] = { 0 };
	u8 rbuf[16] = { 0 };
	int ret = 0;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if ((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) ||
		(ret == MIP_CTRL_STATUS_READY)) {
		TP_LOG_INFO("status [0x%02X]\n", ret);
	} else {
		TP_LOG_ERROR("unknown status [0x%02X]\n", ret);
		goto ERROR;
	}

	if (ret == MIP_CTRL_STATUS_LOG) {
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if (mms_i2c_write(info, wbuf, 3)) {
			TP_LOG_ERROR("failed to mms_i2c_write\n");
		}
	}

	return ret;

ERROR:
	return -1;
}

/**
* Read chip firmware version
*/
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf)
{
	u8 rbuf[8] = { 0 };
	u8 wbuf[2] = { 0 };
	int i = 0;

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
		goto ERROR;
	};

	for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
		ver_buf[0 + i * 2] = rbuf[1 + i * 2];
		ver_buf[1 + i * 2] = rbuf[0 + i * 2];
	}

	return 0;

ERROR:
	TP_LOG_ERROR("failed to mms_get_fw_version\n");
	return 1;
}

/**
* Read chip firmware version for u16
*/
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16)
{
	u8 rbuf[8] = { 0 };
	int i = 0;

	if (mms_get_fw_version(info, rbuf)) {
		goto ERROR;
	}

	for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
		ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
	}

	return 0;

ERROR:
	TP_LOG_ERROR("failed to mms_get_fw_version_u16\n");
	return 1;
}

/**
* Disable ESD alert
*/
int mms_disable_esd_alert(struct mms_ts_info *info)
{
	u8 wbuf[4] = { 0 };
	u8 rbuf[4] = { 0 };

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
	wbuf[2] = 1;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if(rbuf[0] != 1){
		TP_LOG_ERROR("failed, rbuf[0] != 1\n");
		goto ERROR;
	}

	TP_LOG_INFO("end\n");
	return 0;

ERROR:
	return 1;
}

/**
* Enable gesture wakeup mode
*/
int mms_enable_gesture_wakeup_mode(struct mms_ts_info *info)
{
	u8 wbuf[4] = { 0 };
	u8 rbuf[4] = { 0 };

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_POWER_STATE;
	wbuf[2] = 1;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if (rbuf[0] != 1) {
		TP_LOG_ERROR("failed, rbuf[0] != 1\n");
		goto ERROR;
	}

	TP_LOG_DEBUG("end\n");
	return 0;

ERROR:
	return 1;
}

/**
* Enable normal active mode
*/
int mms_enable_nomal_active_mode(struct mms_ts_info *info)
{
	u8 wbuf[4] = { 0 };
	u8 rbuf[4] = { 0 };

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_POWER_STATE;
	wbuf[2] = 0;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if (rbuf[0] != 0) {
		TP_LOG_ERROR("failed, rbuf[0] != 0\n");
		goto ERROR;
	}

	TP_LOG_INFO("end\n");

	return 0;

ERROR:
	return 1;
}

/**
* Re-Calibration
*/
int mms_recalibration(struct mms_ts_info *info)
{
	u8 wbuf[4] = { 0 };
	u8 rbuf[4] = { 0 };

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_RECALIBRATE;
	wbuf[2] = 1;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if (rbuf[0] != 1) {
		TP_LOG_ERROR("failed, rbuf[0] != 1\n");
		goto ERROR;
	}

	TP_LOG_INFO("end\n");
	return 0;

ERROR:
	return 1;
}

/**
* Chracter Setting Bit
*/
#define SET_BIT(flag, g)                  ((flag) |= (0x1 << g))
#define CLEAR_BIT(flag, g)                ((flag) ^= (0x1 << g))

int mms_gesture_type_setting(struct mms_ts_info *info, u8 Gesture)
{
	u32 ulGestureType = 0;
	u8 wbuf[6] = { 0 };
	u8 rbuf[6] = { 0 };

	SET_BIT(ulGestureType, Gesture);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_GESTURE_TYPE;
	wbuf[2] = ulGestureType;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 4)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if ((rbuf[3] & 0x01) != 1) {
		TP_LOG_ERROR("failed, (rbuf[3] & 0x01) != 1\n");
		goto ERROR;
	}

	TP_LOG_INFO("end\n");

	return 0;

ERROR:
	return 1;
}

int mms_gesture_type_unsetting(struct mms_ts_info *info,u8 Gesture)
{
	u32 ulGestureType = 0;
	u8 wbuf[6] = { 0 };
	u8 rbuf[6] = { 0 };

	CLEAR_BIT(ulGestureType, Gesture);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_GESTURE_TYPE;
	wbuf[2] = ulGestureType;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to mms_i2c_write\n");
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 4)) {
		TP_LOG_ERROR("failed to mms_i2c_read\n");
		goto ERROR;
	}

	if ((rbuf[0] & 0x01) != 1) {
		TP_LOG_ERROR("failed, (rbuf[0] & 0x01) != 1\n");
		goto ERROR;
	}

	TP_LOG_INFO("end\n");
	return 0;

ERROR:
	return 1;
}

/**
* Alert event handler - ESD
*/
static int mms_alert_handler_esd(struct mms_ts_info *info, u8 *rbuf)
{
	u8 frame_cnt = rbuf[2];

	TP_LOG_DEBUG("start - frame_cnt[%d]\n", frame_cnt);

	if (frame_cnt == 0) {
		//sensor crack, not ESD
		info->esd_cnt++;
		TP_LOG_DEBUG("esd_cnt[%d]\n", info->esd_cnt);

		if (info->disable_esd == true) {
			mms_disable_esd_alert(info);
		} else if (info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
			//Disable ESD alert
			if (mms_disable_esd_alert(info)) {

			} else {
				info->disable_esd = true;
			}
		} else {
			//Reset chip
			mms_reboot(info);
		}
	} else {
		//ESD detected
		//Reset chip
		mms_reboot(info);
		info->esd_cnt = 0;
	}

	TP_LOG_DEBUG("end\n");
	return 0;
}

/**
* Alert event handler - Wake-up
*/
static int mms_alert_handler_wakeup(struct mms_ts_info *info, u8 *rbuf)
{
	if (mms_wakeup_event_handler(info, rbuf)) {
		goto ERROR;
	}

	TP_LOG_DEBUG("end\n");
	return 0;

ERROR:
	return 1;
}

/**
* Interrupt handler
*/
static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	u8 wbuf[8] = { 0 };
	u8 rbuf[256] = { 0 };
	unsigned int size = 0;
	int event_size = info->event_size;
	u8 category = 0;
	u8 alert_type = 0;

 	//Read first packet
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if (mms_i2c_read(info, wbuf, 2, rbuf, (1 + event_size))) {
		TP_LOG_ERROR("failed to read packet info\n");
		goto ERROR;
	}

	TP_LOG_DEBUG("info [0x%02X]\n", rbuf[0]);

	//Check event
	size = (rbuf[0] & 0x7F);

	TP_LOG_DEBUG("packet size [%d]\n", size);

	category = ((rbuf[0] >> 7) & 0x1);
	if (category == 0) {
		//Touch event
		if (size > event_size) {
			//Read next packet
			if (mms_i2c_read_next(info, rbuf, (1 + event_size), (size - event_size))) {
				TP_LOG_ERROR("failed to read next packet\n");
				goto ERROR;
			}
		}

		info->esd_cnt = 0;

		mms_input_event_handler(info, size, rbuf);
	} else {
		//Alert event
		wbuf[0] = MIP_R0_EVENT;
		wbuf[1] = MIP_R1_EVENT_PACKET_DATA;

		if (mms_i2c_read(info, wbuf, 2, rbuf, 2)) {
			TP_LOG_ERROR("failed to read touch event packet\n");
			goto ERROR;
		}
		alert_type = rbuf[0];

		TP_LOG_DEBUG("alert type [%d]\n", alert_type);

		if (alert_type == MIP_ALERT_ESD) {
			//ESD detection
			if (mms_alert_handler_esd(info, rbuf)) {
				goto ERROR;
			}
		} else if (alert_type == MIP_ALERT_WAKEUP) {
			//Wake-up gesture
			if (mms_alert_handler_wakeup(info, rbuf)) {
				goto ERROR;
			}
		} else {
			TP_LOG_ERROR("unknown alert type [%d]\n", alert_type);
			mms_reboot(info);
			goto ERROR;
		}
	}

	TP_LOG_DEBUG("end\n");

	return IRQ_HANDLED;

ERROR:
	if (RESET_ON_EVENT_ERROR) {
		mms_disable(info);
		mms_clear_input(info);
		mms_enable(info);
		TP_LOG_INFO("reset on error\n");
	}

	return IRQ_HANDLED;
}

void mms_touch_mode_set(struct mms_ts_info *info, int touch_mode)
{
	u8 wbuf[3] = { 0 };
	int mode = 0;

	mutex_lock(&info->touch_mode_lock);

	mms_touch_mode_flag = touch_mode;

	if (touch_mode == 0)
		mode = 0;
	else
		mode = 1;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_WINDOW_MODE;
	wbuf[2] = mode;
	if (mms_i2c_write(info, wbuf, 3)) {
		TP_LOG_ERROR("failed to set touch mode as %d\n", touch_mode);
		goto ERROR;
	}

	TP_LOG_INFO("success to set touch mode as %d\n", touch_mode);

ERROR:
	mutex_unlock(&info->touch_mode_lock);

	return;
}

void mms_fw_get_firmware_name(struct mms_ts_info *info, char *fw_name)
{
	u16 ver_chip[MMS_FW_MAX_SECT_NUM] = { 0 };
	int retires = 3;

	//Reboot chip
	mms_reboot(info);

	//Check chip firmware version
	while (retires--) {
		if (!mms_get_fw_version_u16(info, ver_chip)) {
			break;
		} else {
			mms_reboot(info);
		}
	}
	if (retires < 0) {
		memset(ver_chip, 0x0000, sizeof(ver_chip));
		TP_LOG_ERROR("failed to read chip firmware version, set to 0x0000\n");
	} else {
		TP_LOG_INFO("chip firmware version: 0x%04X 0x%04X 0x%04X 0x%04X\n",
			ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
	}

	if ((u8)(ver_chip[3] >> 8) == 1) {
		if (ver_chip[3] < 0x0117) {
			memcpy(fw_name, INTERNAL_A_FW_PATH, sizeof(INTERNAL_A_FW_PATH));
			TP_LOG_INFO("A firmware\n");
		} else if ((u8)ver_chip[3] % 2) {
			memcpy(fw_name, INTERNAL_B_FW_PATH, sizeof(INTERNAL_B_FW_PATH));
			TP_LOG_INFO("B firmware\n");
		} else {
			memcpy(fw_name, INTERNAL_A_FW_PATH, sizeof(INTERNAL_A_FW_PATH));
			TP_LOG_INFO("A firmware\n");
		}
	} else {
		memcpy(fw_name, INTERNAL_B_FW_PATH, sizeof(INTERNAL_B_FW_PATH));
		TP_LOG_INFO("default B firmware\n");
	}
	return;
}

/**
* Update firmware from kernel built-in binary
*/
int mms_fw_update_from_kernel(struct mms_ts_info *info)
{
	char fw_name[255] = { 0 };
	const struct firmware *fw;
	int retires = 3;
	int ret = 0;

	TP_LOG_INFO("start\n");

	mms_fw_get_firmware_name(info, fw_name);

	request_firmware(&fw, fw_name, &info->client->dev);

	if (!fw) {
		ret = -1;
		TP_LOG_ERROR("failed to request_firmware\n");
		goto ERROR;
	}

	do {
		ret = mms_flash_fw(info, fw->data, fw->size, false, true);
		if (ret >= fw_err_none) {
			break;
		}
	} while (--retires);

	if (!retires) {
		ret = -1;
		TP_LOG_ERROR("failed to mms_flash_fw\n");
		goto ERROR;
	}

	ret = 0;

	TP_LOG_INFO("success to end\n");

ERROR:
	release_firmware(fw);
	return ret;
}

/**
* Update firmware from external storage
*/
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force)
{
	struct file *fp;
	mm_segment_t old_fs;
	size_t fw_size = 0, nread = 0;
	int ret = 0;

	TP_LOG_INFO("start\n");

	mutex_lock(&info->lock);

	mms_irq_disable(info->client->irq);

	old_fs = get_fs();
	set_fs(KERNEL_DS);  
	fp = filp_open(EXTERNAL_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		TP_LOG_ERROR("failed to filp_open - path[%s]\n", EXTERNAL_FW_PATH);
		ret = fw_err_file_open;
		goto ERROR;
	}

 	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (0 < fw_size) {
		unsigned char *fw_data;
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
		TP_LOG_DEBUG("path [%s] size [%lu]\n", EXTERNAL_FW_PATH, fw_size);

		if (nread != fw_size) {
			TP_LOG_ERROR("failed to vfs_read - size[%d] read[%d]\n",
				(int)fw_size, (int)nread);
			ret = fw_err_file_read;
		} else {
			ret = mms_flash_fw(info, fw_data, fw_size, force, true);
		}

		kfree(fw_data);
	} else {
		TP_LOG_ERROR("fw_size [%d]\n", (int)fw_size);
		ret = fw_err_file_read;
	}

 	filp_close(fp, current->files);

ERROR:
	set_fs(old_fs);
	mms_irq_enable(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret == 0) {
		TP_LOG_DEBUG("end\n");
	} else {
		TP_LOG_DEBUG("failed\n");
	}

	return ret;
}

static ssize_t mms_sys_fw_update(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int result = 0;
	u8 data[255] = { 0 };
	int ret = 0;

	TP_LOG_INFO("start\n");

	memset(info->print_buf, 0, PAGE_SIZE);

	ret = mms_fw_update_from_storage(info, true);

	switch(ret) {
		case fw_err_none:
			sprintf(data, "F/W update success.\n");
			break;
		case fw_err_uptodate:
			sprintf(data, "F/W is already up-to-date.\n");
			break;
		case fw_err_download:
			sprintf(data, "F/W update failed : Download error\n");
			break;
		case fw_err_file_type:
			sprintf(data, "F/W update failed : File type error\n");
			break;
		case fw_err_file_open:
			sprintf(data, "F/W update failed : File open error [%s]\n", EXTERNAL_FW_PATH);
			break;
		case fw_err_file_read:
			sprintf(data, "F/W update failed : File read error\n");
			break;
		default:
			sprintf(data, "F/W update failed.\n");
			break;
	}

	strcat(info->print_buf, data);
	result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);

	TP_LOG_INFO("end\n");

	return result;
}
static DEVICE_ATTR(fw_update, 0444, mms_sys_fw_update, NULL);

static ssize_t mms_sys_wakeup_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mms_wakeup_control_flag);
}

static ssize_t mms_sys_wakeup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u8 recv = 0;

	if (kstrtou8(buf, 10, &recv))
		return -EINVAL;

	//ztemt for uniform sys node interface
	if(recv == 255)
		recv = 0;

	if ((recv != 0) && (recv != 1)) {
		TP_LOG_ERROR("wakeup gesture should be 0 or 1\n");
		return -EINVAL;
	}

	mms_wakeup_control_flag = recv;

	TP_LOG_INFO("success to set wakeup gesture as %d", mms_wakeup_control_flag);

	return count;
}

static DEVICE_ATTR(easy_wakeup_gesture, 0660, mms_sys_wakeup_show,
	mms_sys_wakeup_store);

static ssize_t mms_sys_touch_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mms_touch_mode_flag);
}

static ssize_t mms_sys_touch_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	u8 recv = 0;

	if (kstrtou8(buf, 10, &recv))
		return -EINVAL;

	if ((recv != 0) && (recv != 2)) {
		TP_LOG_ERROR("touch mode should be 0 or 2\n");
		return -EINVAL;
	}

	mms_touch_mode_set(info, (int)recv);

	return count;
}

static DEVICE_ATTR(touch_mode, 0660, mms_sys_touch_mode_show,
	mms_sys_touch_mode_store);

static ssize_t mms_sys_c_zone_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mms_c_zone_flag);
}

static ssize_t mms_sys_c_zone_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	u8 recv = 0;

	if (kstrtou8(buf, 10, &recv))
		return -EINVAL;

	if ((recv != 0) && (recv != 1)) {
		TP_LOG_ERROR("c zone should be 0 or 1\n");
		return -EINVAL;
	}

	mms_clear_input(info);

	mms_c_zone_flag = recv;

	TP_LOG_INFO("success to set c zone as %d", mms_c_zone_flag);

	return count;
}

static DEVICE_ATTR(c_zone, 0660, mms_sys_c_zone_show, mms_sys_c_zone_store);

/**
* Sysfs attr info
*/
static struct attribute *mms_attrs[] = {
	&dev_attr_fw_update.attr,
	&dev_attr_easy_wakeup_gesture.attr,
	&dev_attr_touch_mode.attr,
	&dev_attr_c_zone.attr,
	NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group mms_attr_group = {
	.attrs = mms_attrs,
};

/**
* Initial config
*/
static int mms_init_config(struct mms_ts_info *info)
{
	u8 wbuf[8] = { 0 };
	u8 rbuf[64] = { 0 };

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	mms_i2c_read(info, wbuf, 2, rbuf, 16);
	memcpy(info->product_name, rbuf, 16);
	TP_LOG_DEBUG("product_name[%s]\n", info->product_name);

	mms_get_fw_version(info, rbuf);
	memcpy(info->fw_version, rbuf, 8);
	TP_LOG_INFO("F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n",
		info->fw_version[0], info->fw_version[1], info->fw_version[2],
		info->fw_version[3], info->fw_version[4], info->fw_version[5],
		info->fw_version[6], info->fw_version[7]);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	mms_i2c_read(info, wbuf, 2, rbuf, 7);

#if 1
	//Set resolution using chip info
	info->max_x = (rbuf[0]) | (rbuf[1] << 8);
	info->max_y = (rbuf[2]) | (rbuf[3] << 8);
#else
	//Set resolution using platform data
	info->max_x = info->pdata->max_x;
	info->max_y = info->pdata->max_y;
#endif
	TP_LOG_DEBUG("max_x[%d] max_y[%d]\n", info->max_x, info->max_y);

	info->node_x = rbuf[4];
	info->node_y = rbuf[5];
	info->node_key = rbuf[6];
	TP_LOG_DEBUG("node_x[%d] node_y[%d] node_key[%d]\n", info->node_x,
		info->node_y, info->node_key);

	if (info->node_key > 0) {
		//Enable touchkey
		info->tkey_enable = true;
	}

	info->event_size = 8;

	TP_LOG_INFO("end\n");
	return 0;
}

/**
* Initial ts info
*/
static void mms_init_ts_info(struct mms_ts_info *info)
{
	info->irq = -1;
	info->init = true;
	info->power_en = false;
	info->enabled = false;
}

/**
* Initialize driver
*/
static int mms_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info = NULL;
	struct input_dev *input_dev = NULL;
	int ret = 0;

	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	struct input_dev *input_rim_dev = NULL;
	#endif

	TP_LOG_INFO("start\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		ret = -EIO;
		TP_LOG_ERROR("failed to i2c_check_functionality\n");
		goto err_exit;
	}

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		TP_LOG_ERROR("failed to kzalloc\n");
		goto err_kzalloc;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		TP_LOG_ERROR("failed to input_allocate_device\n");
		goto err_allo_input_dev;
	}
	info->input_dev = input_dev;

	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_rim_dev = input_allocate_device();
	if (!input_rim_dev) {
		ret = -ENOMEM;
		TP_LOG_ERROR("failed to input_allocate_device\n");
		goto err_allo_rim_dev;
	}
	info->input_rim_dev = input_rim_dev;
	#endif

	info->client = client;
	i2c_set_clientdata(client, info);

	//Initial config
	mms_init_ts_info(info);

	mutex_init(&info->lock);
	mutex_init(&info->touch_mode_lock);

	//Get platform data
#if MMS_USE_DEVICETREE
	if (client->dev.of_node) {
		info->pdata = devm_kzalloc(&client->dev, sizeof(struct mms_platform_data), GFP_KERNEL);
		if (!info->pdata) {
			ret = -ENOMEM;
			TP_LOG_ERROR("failed to devm_kzalloc\n");
			goto err_devm_kzalloc;
		}

		ret = mms_parse_devicetree(&client->dev, info);
		if (ret) {
			ret = -EINVAL;
			TP_LOG_ERROR("failed to mms_parse_devicetree\n");
			goto err_parse_dt;
		}
	} else
#endif
	{
		info->pdata = client->dev.platform_data;
		if (info->pdata == NULL) {
			ret = -EINVAL;
			TP_LOG_ERROR("failed to get pdata\n");
			goto err_pdata;
		}
	}

	snprintf(info->phys, sizeof(info->phys), "%s/input", dev_name(&client->dev));

	input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	//used for config input rim dev added by ztemt
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_rim_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen_RIM";
	input_rim_dev->phys = info->phys;
	input_rim_dev->id.bustype = BUS_I2C;
	input_rim_dev->dev.parent = &client->dev;
	#endif

#if MMS_USE_INPUT_OPEN_CLOSE
	input_dev->open = mms_input_open;
	input_dev->close = mms_input_close;
#endif

	//Set input event buffer size used for sync multi finger event data
	input_set_events_per_packet(input_dev, 200);
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_set_events_per_packet(input_rim_dev, 200);
	#endif

	//Create device
	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) {
		ret = -EIO;
		TP_LOG_ERROR("failed to input_register_device\n");
		goto err_input_reg_dev;
	}

	//used for register input rim dev added by ztemt
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_set_drvdata(input_rim_dev, info);
	ret = input_register_device(input_rim_dev);
	if (ret) {
		ret = -EIO;
		TP_LOG_ERROR("failed to register input_rim_dev\n");
		goto err_rim_input_reg_dev;
	}
	#endif

    ret = mms_pinctrl_init(info);
	if (ret) {
		TP_LOG_ERROR("failed to mms_pinctrl_init\n");
		goto err_pinctl_init;
	}

	ret = mms_pinctrl_enable(info, 1);
	if (ret) {
		TP_LOG_ERROR("failed to mms_pinctrl_enable\n");
		goto err_pinctl_enable;
	}
	mms_regulator_control(client, 1);

	//Power on
	mms_power_on(info);

	//Firmware update
#if MMS_USE_AUTO_FW_UPDATE
	/*
	info->fw_name = kstrdup(INTERNAL_FW_PATH, GFP_KERNEL);
	ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev, GFP_KERNEL, info, mms_fw_update_boot);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] request_firmware_nowait\n", __func__);
		ret = -EIO;
		//goto ERROR;
	}
	*/
	ret = mms_fw_update_from_kernel(info);
	if (ret) {
		TP_LOG_ERROR("failed to mms_fw_update_from_kernel\n");
		goto err_update_frm_kernel;
	}
#endif

	//Initial config
	mms_init_config(info);

	//Config input interface	
	mms_config_input(info);

	//used for config input rim dev added by ztemt
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	zte_config_input_rim(info);
	#endif

#if MMS_USE_CALLBACK
	//Config callback functions
	mms_config_callback(info);
#endif

	//Set interrup handler
	ret = request_threaded_irq(client->irq, NULL, mms_interrupt,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
	if (ret) {
		TP_LOG_ERROR("failed to request_threaded_irq\n");
		goto err_request_irq;
	}

	mms_irq_disable(info->client->irq);
	info->irq = client->irq;

#ifdef CONFIG_HAS_EARLYSUSPEND
	//Config early suspend
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = mms_early_suspend;
	info->early_suspend.resume = mms_late_resume;
	register_early_suspend(&info->early_suspend);

	TP_LOG_DEBUG("success to register_early_suspend\n");

#elif defined(CONFIG_FB)

	mms_setup_fb_notifier(info);

#endif

#if MMS_USE_DEV_MODE
	//Create dev node (optional)
	if (mms_dev_create(info)) {
		ret = -EAGAIN;
		TP_LOG_ERROR("failed to mms_dev_create\n");
		goto err_create_dev;
	}

	//Create dev
	info->class = class_create(THIS_MODULE, MMS_DEVICE_NAME);
	device_create(info->class, NULL, info->mms_dev, NULL, MMS_DEVICE_NAME);
#endif

#if MMS_USE_TEST_MODE
	//Create sysfs for test mode (optional)
	if (mms_sysfs_create(info)) {
		ret = -EAGAIN;
		TP_LOG_ERROR("failed to mms_sysfs_create\n");
		goto err_create_sysfs;
	}
#endif

#if MMS_USE_CMD_MODE
	//Create sysfs for command mode (optional)
	if (mms_sysfs_cmd_create(info)) {
		ret = -EAGAIN;
		TP_LOG_ERROR("failed to mms_sysfs_cmd_create\n");
		goto err_create_sysfs_cmd;
	}
#endif

	//Create sysfs
	if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
		ret = -EAGAIN;
		TP_LOG_ERROR("failed to sysfs_create_group\n");
		goto err_create_group;
	}

	if (sysfs_create_link(NULL, &client->dev.kobj, MMS_DEVICE_NAME)) {
		ret = -EAGAIN;
		TP_LOG_ERROR("failed to sysfs_create_link\n");
		goto err_create_link;
	}

	//used for init ignore zone added by ztemt
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	zte_ignore_zone_init();
	#endif

	//Enable device
	mms_enable(info);

	TP_LOG_INFO("success to probe MELFAS " CHIP_NAME " Touchscreen\n");
	return 0;

err_create_link:
	sysfs_remove_group(&client->dev.kobj, &mms_attr_group);
err_create_group:
#if MMS_USE_CMD_MODE
mms_sysfs_cmd_remove(info);
#endif
#if MMS_USE_CMD_MODE
err_create_sysfs_cmd:
#if MMS_USE_TEST_MODE
mms_sysfs_remove(info);
#endif
#endif
#if MMS_USE_TEST_MODE
err_create_sysfs:
#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif
#endif
#if MMS_USE_DEV_MODE
err_create_dev:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif
#endif
err_request_irq:
#if MMS_USE_AUTO_FW_UPDATE
err_update_frm_kernel:
#endif
err_pinctl_enable:
	devm_pinctrl_put(info->pdata->pin_res.pinctrl);
err_pinctl_init:
#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_unregister_device(info->input_rim_dev);
err_rim_input_reg_dev:
	input_unregister_device(info->input_dev);
#endif
err_input_reg_dev:
	gpio_free(info->pdata->gpio_reset);
	gpio_free(info->pdata->gpio_intr);
#if MMS_USE_DEVICETREE
err_parse_dt:
	devm_kfree(&client->dev, info->pdata);
err_devm_kzalloc:
#endif
err_pdata:
#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
err_allo_rim_dev:
#endif
err_allo_input_dev:
	kfree(info);
err_kzalloc:
err_exit:
	TP_LOG_ERROR("failed\n");
	return ret;
}

/**
* Remove driver
*/
static int mms_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0) {
		free_irq(info->irq, info);
	}

#if MMS_USE_CMD_MODE
	mms_sysfs_cmd_remove(info);
#endif

#if MMS_USE_TEST_MODE
	mms_sysfs_remove(info);
#endif

	sysfs_remove_group(&info->client->dev.kobj, &mms_attr_group);
	sysfs_remove_link(NULL, MMS_DEVICE_NAME);
	kfree(info->print_buf);

#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	input_unregister_device(info->input_dev);

	/* used for unregister input rim dev added by ztemt */
	#if defined(CONFIG_TOUCHSCREEN_RIM_FUNCTION)
	input_unregister_device(info->input_rim_dev);
	#endif

	kfree(info->fw_name);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
/**
* Device suspend event handler
*/
int mms_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;

	TP_LOG_DEBUG("start\n");

#if MMS_USE_GESTURE_WAKEUP_MODE
	if (mms_wakeup_control_flag == 1) {
		ret = mms_enable_gesture_wakeup_mode(info);
		if (ret == 0) {
			TP_LOG_INFO("success to open gesture\n");
		} else {
			TP_LOG_ERROR("failed to open gesture\n");
		}

		mms_irq_wake_enable(info->client->irq);
	} else {
		mms_disable(info);
	}
#endif

	mms_clear_input(info);

	TP_LOG_INFO("end\n");

	return 0;
}

/**
* Device resume event handler
*/
int mms_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;

	TP_LOG_DEBUG("start\n");

	mms_reboot(info);

	if (info->enabled == false) {
		ret = mms_enable(info);
	}

	if(mms_wakeup_control_flag == 1)
		mms_irq_wake_disable(info->client->irq);

	TP_LOG_INFO("end\n");

	return ret;
}
#endif

/* fb callback added by ztemt */
#if defined(CONFIG_FB)
static int mms_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct mms_ts_info *info = container_of(self, struct mms_ts_info, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

	TP_LOG_DEBUG("event = %lu\n", event);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;

		TP_LOG_ERROR("%s(%d)\n", *blank == 0 ? "FB_BLANK_UNBLANK" \
			: (*blank == 4) ? "FB_BLANK_POWERDOWN" : "UNKNOWN", *blank);

		if (*blank == FB_BLANK_UNBLANK) {
			mms_resume(&info->client->dev);
			mms_clear_input(info);
			TP_LOG_INFO("ztemt Wake!\n");
		} else if (*blank == FB_BLANK_POWERDOWN) {
			mms_suspend(&info->client->dev);
			TP_LOG_INFO("ztemt Sleep!\n");
		}
	}

	/*if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			mms_resume(&info->client->dev);
			mms_clear_input(info);
			dev_info(&info->client->dev, "ztemt %s: Wake!\n", __func__);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			mms_suspend(&info->client->dev);
			dev_info(&info->client->dev, "ztemt %s: Sleep!\n", __func__);
		}
	}*/

	return 0;
}

static void mms_setup_fb_notifier(struct mms_ts_info *info)
{
	int rc = 0;

	info->fb_notifier.notifier_call = mms_fb_notifier_callback;

	rc = fb_register_client(&info->fb_notifier);
	if (rc)
		TP_LOG_ERROR("failed to register fb_notifier: %d\n", rc);
}

/* fb callback added by ztemt */

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/**
* Early suspend handler
*/
void mms_early_suspend(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);

	mms_suspend(&info->client->dev);
}

/**
* Late resume handler
*/
void mms_late_resume(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);

	mms_resume(&info->client->dev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
* PM info
*/
const struct dev_pm_ops mms_pm_ops = {
#if 0
	SET_SYSTEM_SLEEP_PM_OPS(mms_suspend, mms_resume)
#else
	.suspend = NULL,
	.resume  = NULL,
#endif
};
#endif

#if MMS_USE_DEVICETREE
/**
* Device tree match table
*/
static const struct of_device_id mms_match_table[] = {
	{ .compatible = "melfas,"MMS_DEVICE_NAME,},
	{},
};
MODULE_DEVICE_TABLE(of, mms_match_table);
#endif

/**
* I2C Device ID
*/
static const struct i2c_device_id mms_id[] = {
	{MMS_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

/**
* I2C driver info
*/
static struct i2c_driver mms_driver = {
	.id_table = mms_id,
	.probe = mms_probe,
	.remove = mms_remove,
	.driver = {
		.name = MMS_DEVICE_NAME,
		.owner = THIS_MODULE,
#if MMS_USE_DEVICETREE
		.of_match_table = mms_match_table,
#endif
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm 	= &mms_pm_ops,
#endif
	},
};

/**
* Init driver
*/
static int __init mms_init(void)
{
	return i2c_add_driver(&mms_driver);
}

/**
* Exit driver
*/
static void __exit mms_exit(void)
{
	i2c_del_driver(&mms_driver);
}

module_init(mms_init);
module_exit(mms_exit);

MODULE_DESCRIPTION("MELFAS MMS400 Touchscreen");
MODULE_VERSION("2014.12.02");
MODULE_AUTHOR("Jee SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");
