/*
 * cyttsp3_force_function.c
 * Parade TrueTouch(TM) Standard Product Force Function test module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * Gen2
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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cyttsp3_core.h>
#include <linux/cyttsp3_ff_api.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>	/* This enables firmware class loader code */
#include <linux/i2c.h>

#define CY_FF_MODULE_VERSION	"2015112401"

#define CY_MAX_PRBUF_SIZE	PIPE_BUF
#define CY_CORE_FF_PROBE_STARTUP_DELAY_MS		3000

struct cyttsp {
	struct device *dev;
	struct i2c_client *client;
#ifdef CY_USE_FORCE_FUNCTION
	uint16_t *paramarray;
	bool is_parameters_upgrade_enabled;
	struct completion builtin_bin_mt_params_complete;
	u8 *config_data;
	int config_size;
	bool config_loading;
#ifdef CY_USE_FF_TUNER_SUPPORT
	struct dentry *ff_debugfs;
	u8 *ff_buf;
	u32 ff_buf_len;
	struct mutex ff_lock;
	u8 ff_exit;
	wait_queue_head_t ff_wait_q;
#endif /* CY_USE_FF_TUNER_SUPPORT */
#endif /* CY_USE_FORCE_FUNCTION */
	struct work_struct ff_probe_work;
	struct timer_list ff_probe_timer;
};

#ifdef CY_USE_FORCE_FUNCTION
#ifdef CY_USE_FF_TUNER_SUPPORT
#define CYTTSP3_FF_TUNER_FILE_NAME "ff_tuner"
#endif /* CY_USE_FF_TUNER_SUPPORT */
#endif /* CY_USE_FORCE_FUNCTION */

struct cyttsp3_mt_parameters_t {
	struct cyttsp *ts;
	uint16_t *Diff2Force;
	uint16_t *num_idx;
	uint16_t *qCurve;
	uint16_t *xCurve;
	uint16_t *yCurve;
};
static struct cyttsp3_mt_parameters_t *cyttsp3_mt_parameters;
static struct cyttsp *cyttsp_ts = NULL;

int ttsp_read_block_data(struct cyttsp *ts, u8 addr, u8 length, void *buf);
int ttsp_write_block_data(struct cyttsp *ts, u8 addr, u8 length, const void *buf);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
struct cyttsp3_force_function _cyttsp3_force_function;
EXPORT_SYMBOL_GPL(_cyttsp3_force_function);
#endif

#define CY_MAX_PRESSURE_PER_ELEMENT          128
#define CY_NUM_PEAK_SENSORS                  3
#define CY_NUM_SENSORS_AROUND_PEAK_SENSOR    9
#define CY_USE_LOCAL_PEAK

#ifndef CY_USE_LOCAL_PEAK
#define CY_USE_LOCAL_SUM
#endif

#ifdef CY_USE_FF_TUNER_SUPPORT
static int cyttsp3_ff_print(struct cyttsp *ts, u8 *buf, int buf_len,
		char *data_name)
{
	int len = strlen(data_name);
	int i, n;
	u8 *p;
	int remain;

	mutex_lock(&ts->ff_lock);
	if (!ts->ff_buf)
		goto exit;

	if (ts->ff_buf_len + (len + buf_len) > CY_MAX_PRBUF_SIZE)
		goto exit;

	if (len + buf_len == 0)
		goto exit;

	remain = CY_MAX_PRBUF_SIZE - ts->ff_buf_len;
	if (remain < len)
		len = remain;

	p = ts->ff_buf + ts->ff_buf_len;
	memcpy(p, data_name, len);
	ts->ff_buf_len += len;
	p += len;
	remain -= len;

	*p = 0;
	for (i = 0; i < buf_len; i++) {
		n = scnprintf(p, remain, "%02X ", buf[i]);
		if (!n)
			break;
		p += n;
		remain -= n;
		ts->ff_buf_len += n;
	}

	n = scnprintf(p, remain, "\n");
	if (!n)
		ts->ff_buf[ts->ff_buf_len] = 0;
	ts->ff_buf_len += n;
	wake_up(&ts->ff_wait_q);
exit:
	mutex_unlock(&ts->ff_lock);
	return 0;
}

static int ff_debugfs_open(struct inode *inode, struct file *filp)
{
	struct cyttsp *ts = inode->i_private;

	filp->private_data = inode->i_private;

	if (ts->ff_buf)
		return -EBUSY;

	ts->ff_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (!ts->ff_buf)
		return -ENOMEM;

	return 0;
}

static int ff_debugfs_close(struct inode *inode, struct file *filp)
{
	struct cyttsp *ts = filp->private_data;

	filp->private_data = NULL;

	kfree(ts->ff_buf);
	ts->ff_buf = NULL;

	return 0;
}

static ssize_t ff_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp *ts = filp->private_data;
	int size;
	int ret;

	wait_event_interruptible(ts->ff_wait_q,
			ts->ff_buf_len != 0 || ts->ff_exit);
	mutex_lock(&ts->ff_lock);
	if (ts->ff_exit) {
		mutex_unlock(&ts->ff_lock);
		return 0;
	}
	if (count > ts->ff_buf_len)
		size = ts->ff_buf_len;
	else
		size = count;
	if (!size) {
		mutex_unlock(&ts->ff_lock);
		return 0;
	}

	ret = copy_to_user(buf, ts->ff_buf, ts->ff_buf_len);
	if (ret == size)
		return -EFAULT;
	size -= ret;
	ts->ff_buf_len -= size;
	mutex_unlock(&ts->ff_lock);
	*ppos += size;
	return size;
}

static const struct file_operations ff_debugfs_fops = {
	.open = ff_debugfs_open,
	.release = ff_debugfs_close,
	.read = ff_debugfs_read,
};
#endif /* CY_USE_FF_TUNER_SUPPORT */

static u16 cyttsp3_default_pressure[] = {
	32, 96, 0, 0, 0, 0, 0, 0,
	0,   0, 0, 0, 0, 0, 0,
};

static u16 cyttsp3_pressure_tbl_elements = 0;


int cyttsp3_reset_baseline(void)
{
	u8 state = 0x01;
	int retval = 0;

	pr_err("Enter %s\n", __func__);

#if 0
	retval = cyttsp3_write_block_data(_cyttsp3_force_function.handle, 0x22, sizeof(state), &state);
#else
	retval = cyttsp3_write_block_data(_cyttsp3_force_function.handle, 0x1A, sizeof(state), &state);
#endif
	if (retval < 0) {
		pr_err("%s: Failed to write to state=%02X, r=%d\n",
			__func__, state, retval);
	}

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp3_reset_baseline);

#define ENABLE_SET_TOUCH_STATUS

#ifdef ENABLE_SET_TOUCH_STATUS
#define CY_REG_FORCE_CTRL   0x1A
#define TOUCH_ONOFF_BIT     0x02

int cyttsp3_set_touch_status(bool on)
{
	int retval = 0;
	u8 force_ctrl = 0;

	retval = cyttsp3_read_block_data(_cyttsp3_force_function.handle, CY_REG_FORCE_CTRL, sizeof(force_ctrl), &force_ctrl);

	if (retval < 0) {
		pr_err("%s: bus read fail on CY_REG_FORCE_CTRL r=%d\n", __func__, retval);
		return retval;
	}

	if(force_ctrl & TOUCH_ONOFF_BIT){
		if(on == false){
			force_ctrl = force_ctrl & (~TOUCH_ONOFF_BIT);
			retval = cyttsp3_write_block_data(_cyttsp3_force_function.handle, CY_REG_FORCE_CTRL, sizeof(force_ctrl), &force_ctrl);
			pr_debug("%s: Touch off, CY_REG_FORCE_CTRL=0x%02X\n", __func__, force_ctrl);
		}
	}
	else{
		if(on == true){
			force_ctrl = force_ctrl | TOUCH_ONOFF_BIT;
			retval = cyttsp3_write_block_data(_cyttsp3_force_function.handle, CY_REG_FORCE_CTRL, sizeof(force_ctrl), &force_ctrl);
			pr_debug("%s: Touch on, CY_REG_FORCE_CTRL=0x%02X\n", __func__, force_ctrl);
		}
	}

	if (retval < 0) {
		pr_err("%s: bus write fail on CY_REG_FORCE_CTRL r=%d\n", __func__, retval);
	}

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp3_set_touch_status);

#endif

static u16 cyttsp3_get_Diff(u16 *pressure_tbl)
{
	u32 pressure_sum = 0;
	int sensor_offset;
#ifdef CY_USE_LOCAL_PEAK
	int ii;

	pr_debug("%s: Enter 1\n", __func__);
	/* BLZJ : Sum the 9 sensors around the peak sensor. 
	 * Note that, we have to make sure sum_temp will
	 * not overflow */
	if ((pressure_tbl[4] >= pressure_tbl[7]) &&
		(pressure_tbl[4] >= pressure_tbl[10]))
		sensor_offset = 0;
	else if ((pressure_tbl[7] >= pressure_tbl[4]) &&
		(pressure_tbl[7] >= pressure_tbl[10]))
		sensor_offset = 3;
	else
		sensor_offset = 6;

	for (ii = 0; ii < CY_NUM_SENSORS_AROUND_PEAK_SENSOR; ii++)
		pressure_sum += (u32)pressure_tbl[sensor_offset + ii];

#else /* CY_USE_LOCAL_SUM */
	u32 area_sum[CY_NUM_PEAK_SENSORS];
	int ii, jj;

	pr_info("%s: Enter 2\n", __func__);
	memset(area_sum, 0, sizeof(area_sum));
	for (jj = 0; jj < CY_NUM_PEAK_SENSORS; jj++) {
		sensor_offset = CY_NUM_PEAK_SENSORS * jj;
		for (ii = 0; ii < CY_NUM_SENSORS_AROUND_PEAK_SENSOR; ii++)
			area_sum[jj] += (u32) pressure_tbl[sensor_offset + ii];
	}

	if ((area_sum[0] >= area_sum[1]) && (area_sum[0] >= area_sum[2]))
	{
		pressure_sum = area_sum[0];
	}
	else if ((area_sum[1] >= area_sum[0]) && (area_sum[1] >= area_sum[2]))
	{
		pressure_sum = area_sum[1];
	}
	else
	{
		pressure_sum = area_sum[2];
	}
#endif
	return (u16)pressure_sum;
}

#define CY_REG_BASE 0
#define CY_REG_DATATYPE_OFFSET      1
#define CY_REG_PRESSURE_OFFSET      2
#define CY_RW_REG_DATA_MAX          0xFF

static u16 cyttsp3_get_pressure(void *handle, u16 *pressure_tbl, u16 *pressure_tbl_elements)
{
//	struct cyttsp *ts = (struct cyttsp *)handle;
	int retval = 0;

#ifdef CY_NO_HW
	/* debug code if no ff hardware running */
	int ii;

	pr_debug("%s: Enter\n", __func__);
	for (ii = 0; ii < *pressure_tbl_elements; ii++) {
		pressure_tbl[ii] += 1;
		pressure_tbl[ii] %= CY_MAX_PRESSURE_PER_ELEMENT;
	}
	*pressure_tbl_elements = ii;
#else
	/* see BLZJ-38 */
	int ii, jj;
	u8 pressure_buf[*pressure_tbl_elements * sizeof(u16)];

	memset(pressure_buf, 0, sizeof(pressure_buf));

	retval = cyttsp3_read_block_data(handle,
		CY_REG_BASE + CY_REG_PRESSURE_OFFSET,
		1 * sizeof(u16), pressure_buf);

	for (ii = 0, jj = 0; ii < *pressure_tbl_elements; ii++, jj+=2)
		pressure_tbl[ii] = (pressure_buf[jj] * 256) + pressure_buf[jj+1];
#endif /* CY_NO_HW */

	pr_debug("%s:pressure_tbl=", __func__);
	for (ii = 0; ii < *pressure_tbl_elements; ii++)
		pr_debug("%d, ", pressure_tbl[ii]);
	pr_debug("\n");
	return retval;
}

u16 cyttsp3_force_function(struct device *dev, u16 xPos, u16 yPos, u16 zVal)
{
	u16 xIdx1, xIdx2;
	u16 yIdx1, yIdx2;
	u16 ii, jj;
	u32 x, x1, x2;
	u32 y, y1, y2;
	u32 f1, f2, f3, f4;
	u32 f_Q11, f_Q12, f_Q21, f_Q22;
	u32 fsum, Fxy, normDiff, Force;
	u32 divider;
	u16 Diff = 0;	// moved from input param list
	u16 *xCurve, *yCurve, *qCurve;
	u16 Diff2Force, num_idx;
	int retval = 0;
	u16 touchXPos = xPos;
	u16 touchYPos = yPos;

	pr_debug("%s: Enter 1\n", __func__);
	
	/* Get the current pressure array.
	 * This function can be built-in to this module or
	 * a call to another module.
	 */
	 if (_cyttsp3_force_function.get_pressure &&
	 	_cyttsp3_force_function.pressure_tbl &&
	 	_cyttsp3_force_function.pressure_tbl_elements) {
		pr_debug("%s: call %s=%p %s=%p %s=%p --%d\n", __func__,
			"get_pressure", _cyttsp3_force_function.get_pressure,
		 	"pressure_tbl", _cyttsp3_force_function.pressure_tbl,
		 	"tbl_elements",
			_cyttsp3_force_function.pressure_tbl_elements,
		 	*_cyttsp3_force_function.pressure_tbl_elements);
		retval = _cyttsp3_force_function.get_pressure
			(_cyttsp3_force_function.handle,
			_cyttsp3_force_function.pressure_tbl,
			_cyttsp3_force_function.pressure_tbl_elements);
			
			if (retval < 0) {
				pr_err("%s: Fail get pressure r=%d\n",
					__func__, retval);
				goto force_function_error_exit;
			}

		/* compute the Diff pressure sum value
		 */
		Diff = cyttsp3_get_Diff(_cyttsp3_force_function.pressure_tbl);
	}


	if (!cyttsp3_mt_parameters) {
		pr_debug("%s: %s=%d\n", __func__,
			"No paramsarray table from file; force=zVal", zVal);
		goto force_function_error_exit;
	}

	xCurve = cyttsp3_mt_parameters->xCurve;
	yCurve = cyttsp3_mt_parameters->yCurve;
	qCurve = cyttsp3_mt_parameters->qCurve;
	Diff2Force = *cyttsp3_mt_parameters->Diff2Force;
	num_idx = *cyttsp3_mt_parameters->num_idx;
	pr_debug("%s: Enter 2 %s=%p %s=%p %s=%p %s=%d %s=%d\n", __func__,
		"xcurve", xCurve, "yCurve", yCurve, "qcurve", qCurve,
		"Diff2Force", Diff2Force, "num_idx", num_idx);


	/* This code segment is to allow scaling touches during testing.
	 * This maps KEV touch panel to coordinate system inferred from
	 * xCurve and yCurve. Modify or comment out the scaling code
	 * as required.
	 */
	dev_dbg(dev, "Not Scaled touches xPos=%d yPos=%d\n", xPos, yPos);
	/* examples:
		xPos = (xPos * ( 612+108)) / 1200;
		yPos = (yPos * (1088+192)) / 2100;
	*/
	dev_dbg(dev, "    Scaled touches xPos=%d yPos=%d\n", xPos, yPos);

	x1 = xCurve[0];
	x2 = xCurve[num_idx-1];
	y1 = yCurve[0];
	y2 = yCurve[(num_idx*num_idx)-1];
	dev_dbg(dev, "%s:xPos=%d yPos=%d x1=%d x2=%d y1=%d y2=%d\n",
		__func__, xPos, yPos, x1, x2, y1, y2);
	xPos = xPos >= x2 ? x2-1 : xPos <= x1 ? x1+1 : xPos;
	yPos = yPos >= y2 ? y2-1 : yPos <= y1 ? y1+1 : yPos;
	dev_dbg(dev, "%s:xPos=%d yPos=%d\n", __func__, xPos, yPos);

	/* find X */
	xIdx1 = xIdx2 = num_idx-1;
	x1 = x2 = xCurve[num_idx-1];
	x = xPos;
	for (ii = 0; ii < num_idx-1; ii++) {
		if (x >= xCurve[ii] && x < xCurve[ii+1]) {
			xIdx1 = ii;
			xIdx2 = ii+1;
			x1 = xCurve[xIdx1];
			x2 = xCurve[xIdx2];
			break;
		}
	}

	/* find Y */
	yIdx1 = yIdx2 = num_idx-1;
	y1 = y2 = yCurve[(num_idx*num_idx)-1];
	y = yPos;
	for (jj = 0; jj < num_idx-1; jj++) {
		if (y >= yCurve[jj*num_idx] && y < yCurve[(jj+1)*num_idx]) {
			yIdx1 = jj;
			yIdx2 = jj+1;
			y1 = yCurve[yIdx1*num_idx];
			y2 = yCurve[yIdx2*num_idx];
			break;
		}
	}

	f_Q11 = qCurve[(yIdx1*num_idx) + xIdx1];
	f_Q12 = qCurve[(yIdx2*num_idx) + xIdx1];
	f_Q21 = qCurve[(yIdx1*num_idx) + xIdx2];
	f_Q22 = qCurve[(yIdx2*num_idx) + xIdx2];

	/* LocalCurve */
	divider = (x2-x1)*(y2-y1);
	if (divider == 0) {
		dev_dbg(dev, "bottom part is 0 x2=%d x1=%d y2=%d y1=%d\n",
			x1, x2, y1, y2);
		divider = 1;
	}

	/* no float */
	f1 = f_Q11*(x2-x)*(y2-y);
	f2 = f_Q21*(x-x1)*(y2-y);
	f3 = f_Q12*(x2-x)*(y-y1);
	f4 = f_Q22*(x-x1)*(y-y1);
	fsum = f1 + f2 + f3 + f4;
	Fxy = fsum / divider;

	/* NormalizedDiff = (Diff*100)/LocalCurve; */
	if (Fxy == 0) {
		dev_dbg(dev, "Fxy=%d\n", Fxy);
		normDiff = 0;
	} else {
		normDiff = (Diff * 100) / Fxy;
	}

	/* Force = NormalizedDiff / Parameters.Diff2Force; */
	Force = (normDiff * 255) / Diff2Force;
	if (Force > 255)
		Force = 255;
	else if (Force == 0)
		Force = 1;
	dev_dbg(dev, "Q11=%d Q12=%d\n", f_Q11, f_Q12);
	dev_dbg(dev, "Q21=%d Q22=%d\n", f_Q21, f_Q22);
	dev_dbg(dev, "f1=%d f2=%d f3=%d f4=%d divider=%d\n",
		f1, f2, f3, f4, divider);
	dev_dbg(dev, "Fxy=%d normDiff=%d Force=%d\n",
		Fxy, normDiff, Force);

#ifdef CY_USE_FF_TUNER_SUPPORT
	{
		u16 zForce = (u16)Force;
		u8 ff_print_buf
			[(*_cyttsp3_force_function.pressure_tbl_elements + 3) *
			sizeof(u16)];
		/* log the pressure table data */
		memset(ff_print_buf, 0, sizeof(ff_print_buf));
		ff_print_buf[0] = touchXPos / 256;
		ff_print_buf[1] = touchXPos % 256;
		ff_print_buf[2] = touchYPos / 256;
		ff_print_buf[3] = touchYPos % 256;
		ff_print_buf[4] = zForce / 256;
		ff_print_buf[5] = zForce % 256;
		ff_print_buf[6] = (*_cyttsp3_force_function.pressure_tbl) / 256;
		ff_print_buf[7] = (*_cyttsp3_force_function.pressure_tbl) % 256;
/*
		memcpy(&ff_print_buf[6],
			(u8 *)_cyttsp3_force_function.pressure_tbl,
			sizeof(ff_print_buf));
*/
		cyttsp3_ff_print(cyttsp3_mt_parameters->ts, ff_print_buf,\
			sizeof(ff_print_buf), "pressure_tbl=");
	}
#endif /* CY_USE_FF_TUNER_SUPPORT */

	return (u16)Force;
	
force_function_error_exit:
	return zVal;
}

int cyttsp2_get_force(struct device *dev, int touch_num,
	u16 xPos, u16 yPos, u16 zVal)
{
	/* only first touch at this time */
	if (touch_num == 0) {
		if (_cyttsp3_force_function.operational &&
			_cyttsp3_force_function.ready &&
			_cyttsp3_force_function.force &&
			_cyttsp3_force_function.get_pressure &&
			_cyttsp3_force_function.pressure_tbl) {
			return cyttsp3_force_function(dev, xPos, yPos, zVal);
		}
	}
	return -1;
}
EXPORT_SYMBOL_GPL(cyttsp2_get_force);

#include <linux/firmware.h>
#define CY_PARAM_FILE_NAME "cyttsp5_paramarray.bin"
static char *fname = CY_PARAM_FILE_NAME;
static int cyttsp3_get_parameters_from_bin(struct cyttsp *ts,
				const struct firmware *fw)
{
	int ii, jj;
	int num_idx;

	if (!fw) {
		dev_info(ts->dev, "%s: No builtin parameters\n", __func__);
		goto cyttsp3_get_parameters_from_bin_exit;
	}

	if (!fw->data || !fw->size) {
		dev_err(ts->dev, "%s: %s fw->data=%p, fw->size=%ld\n", __func__,
			"invalid parameters file", (void *)fw->data, fw->size);
		goto cyttsp3_get_parameters_from_bin_exit;
	}

	dev_err(ts->dev, "%s: Found force parameters\n", __func__);

	if (fw->size > (sizeof(u16) * 2)) {
		num_idx = (fw->data[3] * 256) + fw->data[2];
	} else {
		dev_err(ts->dev, "%s: short parameters file=%s size=%ld\n",
			__func__, fname, fw->size);
		for (ii = 0; ii < fw->size; ii++)
			dev_err(ts->dev, "%s:fw[%d]=%0X\n", __func__,
				ii, fw->data[ii]);
		goto cyttsp3_get_parameters_from_bin_exit;
	}

	if (fw->size == ((sizeof(u16) * 2) + 
		(3 * num_idx * num_idx * sizeof(u16)))) {
		ts->paramarray = kzalloc(fw->size, GFP_KERNEL);
		cyttsp3_mt_parameters = kzalloc(
			sizeof(struct cyttsp3_mt_parameters_t), GFP_KERNEL);
		if (ts->paramarray && cyttsp3_mt_parameters) {
			for (ii = 0, jj = 0; ii < fw->size; ii+=2, jj++)
				ts->paramarray[jj] =
					(fw->data[ii+1] * 256) + fw->data[ii];
			cyttsp3_mt_parameters->Diff2Force = &ts->paramarray[0];
			cyttsp3_mt_parameters->num_idx = &ts->paramarray[1];
			cyttsp3_mt_parameters->qCurve = &ts->paramarray[2];
			cyttsp3_mt_parameters->xCurve = &ts->paramarray[2 +
				(num_idx * num_idx)];
			cyttsp3_mt_parameters->yCurve = &ts->paramarray[2 +
				(2 * num_idx * num_idx)];
			cyttsp3_mt_parameters->ts = ts;
		} else {
			dev_err(ts->dev, "%s:fail get mt params  ma=%p mp=%p\n",
				__func__, (void *)ts->paramarray,
				(void *)cyttsp3_mt_parameters);
		}
	} else {
		dev_err(ts->dev, "%s: force parm file=%s is size=%ld expect%ld\n",
			__func__, fname, fw->size,
			((sizeof(u16) * 2) +
			(3 * num_idx * num_idx * sizeof(u16))));
		goto cyttsp3_get_parameters_from_bin_exit;
	}

#ifdef CY_FF_SHOW_TABLES
	if (cyttsp3_mt_parameters) {
		dev_info(ts->dev, "%s:Parameters=\n", __func__);
		dev_info(ts->dev, "%s:Diff2Force=%d\n", __func__,
			*cyttsp3_mt_parameters->Diff2Force);
		dev_info(ts->dev, "%s:num_idx   =%d\n", __func__,
			*cyttsp3_mt_parameters->num_idx);
		dev_info(ts->dev, "%s:qCurve=\n", __func__);
		for (ii = 0; ii < num_idx; ii++) {
			dev_info(ts->dev, "%s:(%d, %d, %d, %d, %d)\n", __func__,
				cyttsp3_mt_parameters->qCurve[(ii*num_idx)+0],
				cyttsp3_mt_parameters->qCurve[(ii*num_idx)+1],
				cyttsp3_mt_parameters->qCurve[(ii*num_idx)+2],
				cyttsp3_mt_parameters->qCurve[(ii*num_idx)+3],
				cyttsp3_mt_parameters->qCurve[(ii*num_idx)+4]);
		}
		dev_info(ts->dev, "%s:xCurve=\n", __func__);
		for (ii = 0; ii < num_idx; ii++) {
			dev_info(ts->dev, "(%s:%d, %d, %d, %d, %d)\n", __func__,
				cyttsp3_mt_parameters->xCurve[(ii*num_idx)+0],
				cyttsp3_mt_parameters->xCurve[(ii*num_idx)+1],
				cyttsp3_mt_parameters->xCurve[(ii*num_idx)+2],
				cyttsp3_mt_parameters->xCurve[(ii*num_idx)+3],
				cyttsp3_mt_parameters->xCurve[(ii*num_idx)+4]);
		}
		dev_info(ts->dev, "%s:yCurve=\n", __func__);
		for (ii = 0; ii < num_idx; ii++) {
			dev_info(ts->dev, "%s:(%d, %d, %d, %d, %d)\n", __func__,
				cyttsp3_mt_parameters->yCurve[(ii*num_idx)+0],
				cyttsp3_mt_parameters->yCurve[(ii*num_idx)+1],
				cyttsp3_mt_parameters->yCurve[(ii*num_idx)+2],
				cyttsp3_mt_parameters->yCurve[(ii*num_idx)+3],
				cyttsp3_mt_parameters->yCurve[(ii*num_idx)+4]);
		}
	}
#endif /* CY_FF_SHOW_TABLES */

	_cyttsp3_force_function.ready = 1;
	printk(KERN_ERR "%s: Enter 2 %s=%p %s=%p %s=%p %s=%d %s=%d\n", __func__,
		"xcurve", cyttsp3_mt_parameters->xCurve,
		"yCurve", cyttsp3_mt_parameters->yCurve,
		"qcurve", cyttsp3_mt_parameters->qCurve,
		"Diff2Force", *cyttsp3_mt_parameters->Diff2Force,
		"num_idx", *cyttsp3_mt_parameters->num_idx);

	return 0;

cyttsp3_get_parameters_from_bin_exit:
	printk(KERN_ERR "%s ztemt error!!\n", __func__);
	return -1;
}

/* config data sysfs object read function to allow output of param array data
 * eg:
 * echo 1 > mt_config_loading
 * cat cyttsp5_paramarray.bin > mt_config_data
 * echo 0 > mt_config_loading
 * 
 * cat mt_config_data > cyttsp5_paramarray.bin
 */
static ssize_t cyttsp3_mt_config_data_read(struct file *filp,
		struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);
	int ret;

	/* this function will dump out the same contents
	 * that were received from cyttsp5_paramarray.bin
	 */

	ret = ts->config_size;

	if (pos > ret)
		return -EINVAL;
	if (count > ret - pos)
		count = ret - pos;

	if (!count)
		return count;

	memcpy(buf, ts->config_data, count);

	return count;
}

static ssize_t cyttsp3_mt_config_data_write(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);
	u8 *p;

	dev_info(dev, "%s: offset:%lld count:%ld\n", __func__, offset, count);

	p = krealloc(ts->config_data, offset + count, GFP_KERNEL);
	if (!p) {
		kfree(ts->config_data);
		ts->config_data = NULL;
		return -ENOMEM;
	}
	ts->config_data = p;

	memcpy(&ts->config_data[offset], buf, count);
	ts->config_size += count;

	dev_info(dev, "%s: return count=%ld\n", __func__, count);
	return count;
}

static struct bin_attribute bin_attr_mt_config_data = {
	.attr = {
		.name = "mt_config_data",
		.mode = S_IRWXU|S_IRWXG|S_IRWXO,
	},
	.size = PAGE_SIZE,
	.read = cyttsp3_mt_config_data_read,
	.write = cyttsp3_mt_config_data_write,
};

static ssize_t cyttsp3_mt_config_loading_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	bool config_loading;

	config_loading = ts->config_loading;

	return sprintf(buf, "%d\n", config_loading);
}

static ssize_t cyttsp3_mt_config_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 pressure_buf[sizeof(cyttsp3_default_pressure)];
	int i, retval = 0;
	ssize_t count = 0;

	memset(pressure_buf, 0, sizeof(pressure_buf));

	retval = cyttsp3_read_block_data(_cyttsp3_force_function.handle,
		CY_REG_BASE + CY_REG_PRESSURE_OFFSET,
		sizeof(cyttsp3_default_pressure), pressure_buf);

	for (i = 0; i < 15; i++) {
		count += snprintf(buf, PAGE_SIZE, "%s %x", buf,
			pressure_buf[2 * i] << 8 | pressure_buf[2 * i + 1]);
	}
	count += snprintf(buf, PAGE_SIZE, "%s\n", buf);
	return count;
}

/*
 * 1: Start loading Parameters Array
 * 0: End loading - get file data; update curve arrays
 *-1: Exit loading
 */
static ssize_t cyttsp3_mt_config_loading_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	long value;
	struct firmware fw;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc < 0 || value < -1 || value > 1) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return size;
	}

	dev_info(dev, "%s: enter: value=%d ts=%p\n", __func__, (int)value, ts);
	if (value == 1)
		ts->config_loading = true;
	else if (value == -1)
		ts->config_loading = false;
	else if (value == 0 && ts->config_loading) {
		dev_info(dev, "%s: start getting parameters: value=%d ts=%p\n",
			__func__, (int)value, ts);
		ts->config_loading = false;
		if (ts->config_size == 0) {
			dev_err(dev, "%s: No parameters data\n", __func__);
			goto exit_free;
		}

		fw.data = ts->config_data;
		fw.size = ts->config_size;
		dev_info(dev, "%s: get parameters: fw-data=%p fw-size=%d\n",
			__func__, (void *)fw.data, (int)fw.size);
		rc = cyttsp3_get_parameters_from_bin(ts,
			(const struct firmware *)(&fw));
        if(rc == 0) {
            _cyttsp3_force_function.load_config_result = true;
        }
	}
	ts->config_size = 0;

exit_free:

	if (rc)
		return rc;

	return size;
}

static ssize_t cyttsp3_rw_data_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = _cyttsp3_force_function.handle;
	int retval = 0;
	u8 reg_data = 0;

	retval = cyttsp3_read_block_data(ts,
		CY_REG_BASE + CY_REG_DATATYPE_OFFSET, sizeof(reg_data), &reg_data);

	if (retval < 0)
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read DataType Failed\n");
	else
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read DataType=%02X(%d)\n",
			reg_data, reg_data);
}

static ssize_t cyttsp3_rw_data_type_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = _cyttsp3_force_function.handle;
	int retval = 0;
	unsigned long value = 0;
	u8 regdata = 0;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		retval = strict_strtoul(buf, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert value\n",
				__func__);
			goto cyttsp3_rw_data_type_store_exit;
		}
	}

	if (value > CY_RW_REG_DATA_MAX) {
		pr_err("%s: Invalid Register Data Range; no write\n",
			__func__);
	} else {
		regdata = (u8)value;
			retval = cyttsp3_write_block_data(ts, CY_REG_BASE + CY_REG_DATATYPE_OFFSET,
				sizeof(regdata), &regdata);
			if (retval < 0) {
				pr_err("%s: Failed write DataType=%02X(%d)\n"
					, __func__,
					regdata, regdata);
			}
	}

	retval = size;

cyttsp3_rw_data_type_store_exit:
	return retval;
}

static DEVICE_ATTR(rw_data_type, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	cyttsp3_rw_data_type_show, cyttsp3_rw_data_type_store);
static DEVICE_ATTR(mt_config_loading, S_IRWXU|S_IRWXG|S_IRWXO,
	cyttsp3_mt_config_loading_show, cyttsp3_mt_config_loading_store);
static DEVICE_ATTR(mt_config_raw, S_IRUSR |S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH,
	cyttsp3_mt_config_raw_show, NULL);
/* firmware class autoload at startup */
static void cyttsp3_mt_params_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct device *dev = context;
	struct cyttsp *ts = dev_get_drvdata(dev);
	int rc;

	dev_info(ts->dev, "%s: %s\n", __func__,
		"Enter - Get the parameters from the bin file");
	rc = cyttsp3_get_parameters_from_bin(ts, fw);

	release_firmware(fw);
	complete(&ts->builtin_bin_mt_params_complete);
	ts->is_parameters_upgrade_enabled = 0;
	return;
}

static int cyttsp3_get_parameters(struct cyttsp *ts)
{
	int retval;
	dev_info(ts->dev, "%s: Enter - call FW Class\n", __func__);
	init_completion(&ts->builtin_bin_mt_params_complete);
	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			fname, ts->dev, GFP_KERNEL, ts->dev,
			cyttsp3_mt_params_cont_builtin);
	if (retval < 0) {
		dev_err(ts->dev, "%s: %s=%s\n", __func__,
			"Fail request parameters class file load", fname);
		goto cyttsp3_get_parameters_exit;
	}

	/* wait until FW binary upgrade finishes */
#define CY_WAIT_FW_CLASS	5000
	retval = wait_for_completion_timeout
		(&ts->builtin_bin_mt_params_complete,
		msecs_to_jiffies(CY_WAIT_FW_CLASS));
	if (retval < CY_WAIT_FW_CLASS) {
		dev_info(ts->dev, "%s: %s=%d\n", __func__,
			"wait for FW class took ms", retval);
		retval = 0;
	}

cyttsp3_get_parameters_exit:
	return retval;
}

static int cyttsp3_ff_probe_complete(struct cyttsp *ts);
static void cyttsp3_ff_probe_work(struct work_struct *work)
{
	struct cyttsp *ts =
			container_of(work, struct cyttsp,
					ff_probe_work);
	int rc;
	rc = cyttsp3_ff_probe_complete(ts);
	pr_err("%s: FF Probe_complete returns rc=%d\n", __func__, rc);
}
static void cyttsp3_ff_probe_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	if (!ts)
		return;

	pr_err("%s: Probe timer triggered\n", __func__);

	if (!work_pending(&ts->ff_probe_work))
		schedule_work(&ts->ff_probe_work);
}

static int cyttsp_i2c_ff_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ii;
	int retval = 0;
	struct cyttsp *ts = NULL;

	pr_info("%s: Enter\n", __func__);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc context memory\n", __func__);
		goto error_alloc_data;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->dev->bus = &i2c_bus_type;
	
	
	pr_info("%s: ts=%p ts->dev=%p\n", __func__, ts, ts->dev);
	dev_set_drvdata(ts->dev, ts);
	cyttsp_ts = ts;

	dev_info(ts->dev, "%s: test dev_info with ts->dev=%p\n",
		__func__, ts->dev);
	pr_info("%s: continue....\n", __func__);

	cyttsp3_mt_parameters = NULL;
	cyttsp3_pressure_tbl_elements = sizeof(cyttsp3_default_pressure) /
		sizeof(u16);
	/* these items are cleared in the board config startup
	 * _cyttsp3_force_function.operational = 0;
	 * _cyttsp3_force_function.ready = 0;
	 */
	_cyttsp3_force_function.pressure_tbl =
		kzalloc(sizeof(cyttsp3_default_pressure), GFP_KERNEL);
	_cyttsp3_force_function.pressure_tbl_elements =
		&cyttsp3_pressure_tbl_elements;
	memcpy(_cyttsp3_force_function.pressure_tbl, cyttsp3_default_pressure,
		sizeof(cyttsp3_default_pressure));
	_cyttsp3_force_function.force = cyttsp3_force_function;
	_cyttsp3_force_function.get_pressure = cyttsp3_get_pressure;

	pr_info("%s: pressure_tbl[0:%d]=", __func__,
		*_cyttsp3_force_function.pressure_tbl_elements);
	for (ii = 0; ii < *_cyttsp3_force_function.pressure_tbl_elements; ii++)
		pr_info("%d ", _cyttsp3_force_function.pressure_tbl[ii]);
	pr_info("\n");

	retval = device_create_file(ts->dev, &dev_attr_mt_config_loading);
	if (retval)
		dev_err(ts->dev, "%s: %s\n", __func__,
			"Error, could not create mt_config_loading");

	retval = device_create_file(ts->dev, &dev_attr_rw_data_type);
	if (retval)
		dev_err(ts->dev, "%s: %s\n", __func__,
			"Error, could not create rw_data_type");

	retval = device_create_file(ts->dev, &dev_attr_mt_config_raw);
	if (retval)
		dev_err(ts->dev, "%s: %s\n", __func__,
			"Error, could not create mt_config_raw");

	retval = device_create_bin_file(ts->dev, &bin_attr_mt_config_data);
	if (retval)
		dev_err(ts->dev, "%s: %s=%d\n", __func__,
			"Error, could not create mt_config_data", retval);

#ifdef CY_USE_FF_TUNER_SUPPORT
	init_waitqueue_head(&ts->ff_wait_q);
	mutex_init(&ts->ff_lock);
	ts->ff_debugfs = debugfs_create_file(CYTTSP3_FF_TUNER_FILE_NAME,
			0777, NULL, ts, &ff_debugfs_fops);
#endif /* CY_USE_FF_TUNER_SUPPORT */

	/* Some host i2c busses start late and then run too slow */
	pr_err("%s:start wait for ff probe timer\n", __func__);
	INIT_WORK(&ts->ff_probe_work, cyttsp3_ff_probe_work);
	setup_timer(&ts->ff_probe_timer, cyttsp3_ff_probe_timer,
			(unsigned long)ts);
	mod_timer(&ts->ff_probe_timer, jiffies +
			msecs_to_jiffies(CY_CORE_FF_PROBE_STARTUP_DELAY_MS));

/*** ZTEMT start ***/
	_cyttsp3_force_function.FT_ready_flag = false;
	_cyttsp3_force_function.load_config_result = false;
/*ZTEMT end*/

	goto no_error;

error_alloc_data:
	pr_err("%s: Failed Initialization\n", __func__);
no_error:
	return retval;
}

static int cyttsp3_ff_probe_complete(struct cyttsp *ts)
{
	int retval = 0;

	/* load parameters array from .bin file */
	pr_info("%s: load parameters array from .bin file\n", __func__);
	cyttsp3_mt_parameters = NULL;
	retval = cyttsp3_get_parameters(ts);
	if (retval)
		dev_err(ts->dev, "%s: Error when getting parameters=%d\n",
				__func__, retval);

	/* cyttsp2_ff_core fills in handle and ready
	 * _cyttsp3_force_function.handle = (void *)ts;
	 * _cyttsp3_force_function.ready = 1;
	 */
	_cyttsp3_force_function.operational = 1;

	pr_debug("%s: force_ptr=%p, force_function=%p\n", __func__, 
		_cyttsp3_force_function.force, cyttsp3_force_function);

	return retval;
}

static int cyttsp_i2c_ff_remove(struct i2c_client *client)
{
	int retval = 0;

#ifdef CY_USE_FORCE_FUNCTION
	pr_info("%s: Enter\n", __func__);
	_cyttsp3_force_function.force = NULL;
	kfree(_cyttsp3_force_function.pressure_tbl);
	_cyttsp3_force_function.pressure_tbl = NULL;
	*_cyttsp3_force_function.pressure_tbl_elements = 0;

#endif /* CY_USE_FORCE_FUNCTION */

	return retval;
}

static const struct i2c_device_id cyttsp_i2c_ff_id[] = {
	{ CY_I2C_FF_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_ff_id);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
static struct of_device_id cyttsp_match_table[] = {
	{ .compatible = "cy,cyttsp3_ff_module",},
	{ },
};
#else
#define cyttsp_match_table NULL
#endif

static struct i2c_driver cyttsp_i2c_ff_driver = {
	.driver = {
		.name = CY_I2C_FF_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cyttsp_match_table,
	},
	.probe = cyttsp_i2c_ff_probe,
	.remove = cyttsp_i2c_ff_remove,
	.id_table = cyttsp_i2c_ff_id,
};

static int __init cyttsp_i2c_ff_init(void)
{
	int rc;

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
	memset(&_cyttsp3_force_function, 0, sizeof(struct cyttsp3_force_function));
#endif
	
	rc = i2c_add_driver(&cyttsp_i2c_ff_driver);

	pr_info("%s: Parade Force Function Module (Built %s) rc=%d\n",
		 __func__, CY_FF_MODULE_VERSION, rc);

	return rc;
}
//module_init(cyttsp_i2c_ff_init);
fs_initcall_sync(cyttsp_i2c_ff_init);
static void __exit cyttsp_i2c_ff_exit(void)
{

	return;
}
module_exit(cyttsp_i2c_ff_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product Force Function");
MODULE_AUTHOR("Parade Technologies");
