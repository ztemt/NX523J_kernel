/*
 * Core Source for:
 * Parade TrueTouch(TM) Standard Product (TTSP) force function core driver.
 * For use with Cypress Txx2xx and Txx3xx parts.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Parade Technologies at http://www.paradetech.com/
 *
 */

#include <linux/cyttsp3_ff_api.h>
#include <linux/cyttsp3_core.h>
#include <linux/cyttsp3_ff_platform.h>
//#include <linux/init-input.h>

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/version.h>	/* Required for kernel version checking */
#include <linux/firmware.h>	/* This enables firmware class loader code */

/* KEVKEV use this for standard Linux systems
 * for non-standard systems use the single threaded irq code as example code
 */
#if 1 //ZHWW enable
#define CY_USE_THREADED_IRQ
#endif

#define CY_CORE_PROBE_STARTUP_DELAY_MS		3000

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_VALID_APP(x)             ((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)       ((x) & 0x3F)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define IS_BOOTLOADERMODE(reg)      ((reg & 0x10) >> 4)
#define BL_WATCHDOG_DETECT(reg)     (reg & 0x02)
#ifdef CY_USE_GEN2
#define GET_EVNT_IDX(reg)           (reg & 0x03)
#endif /* --CY_USE_GEN2 */

/* maximum number of concurrent tracks */
#ifdef CY_USE_GEN2
#define CY_NUM_TCH_ID               2
#endif /* --CY_USE_GEN2 */
#ifdef CY_USE_GEN3
#define CY_NUM_TCH_ID               4
#endif /* --CY_USE_GEN3 */
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16

#ifdef CY_USE_WATCHDOG
/*
 * maximum number of touch reports with
 * current touches=0 before performing Driver reset
 */
#define CY_MAX_NTCH                 100
#endif /* --CY_USE_WATCHDOG */

#define CY_NTCH                     0 /* lift off */
#define CY_TCH                      1 /* touch down */
#define CY_SMALL_TOOL_WIDTH         10
#define CY_REG_BASE                 0x00
#define CY_REG_OP_START             0x1B
#define CY_REG_OP_END               0x1E
#define CY_REG_OP_FEATURE           0x1F
#define CY_REG_SI_START             0x17
#define CY_REG_SI_END               0x1F
#define CY_DELAY_DFLT               20 /* ms */
#define CY_DELAY_MAX                (500/CY_DELAY_DFLT) /* half second */
#define CY_HALF_SEC_TMO_MS          10000//ztemt 500
#define CY_TEN_SEC_TMO_MS           10000
#define CY_HNDSHK_BIT               0x80
#define CY_HST_MODE_CHANGE_BIT      0x08
#define CY_WAKE_DFLT                99 /* causes wake strobe on INT line
					* in sample board configuration
					* platform_data->hw_recov() function
					*/
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
#define CY_COMPOSITE_MODE           0x20

/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02

#define CY_NUM_ABS_SET              5 /* number of abs signal sets */
#define CY_IGNORE_VALUE             0xFFFF /* mark unused signals as ignore */

#define CY_BL_NUM_KEYS		8
#define CY_BL_PAGE_SIZE		16
#define CY_BL_NUM_PAGES		5
#define CY_BL_STATUS_MASK	0xB3
#define CY_BL_ERROR_MASK	0xFC
#define CY_BL_BUSY		0x80
#define CY_BL_READY_NO_APP	0x10
#define CY_BL_READY_APP		0x11
#define CY_BL_RUNNING		0x20
#define CY_BL_NOERR		0x00
#define CY_BL_ENTER_CMD_SIZE	11
#define CY_BL_EXIT_CMD_SIZE	11
#define CY_BL_WR_BLK_CMD_SIZE	79
#define CY_BL_VERS_SIZE		9
#define CY_BL_FW_NAME_SIZE	NAME_MAX
#define CY_MAX_PRBUF_SIZE	PIPE_BUF
#define CY_IRQ_DEASSERT		1
#define CY_IRQ_ASSERT		0

struct cyttsp_vers {
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 cid[3];
};

enum cyttsp_sett_flags {
	CY_NO_FLAGS = 0x00,
	CY_FLAG_FLIP = 0x08,
	CY_FLAG_INV_X = 0x10,
	CY_FLAG_INV_Y = 0x20,
};

enum cyttsp_powerstate {
	CY_IDLE_STATE = 0,	/* IC cannot be reached */
	CY_READY_STATE,		/* pre-operational; ready to go to ACTIVE */
	CY_ACTIVE_STATE,	/* app is running, IC is scanning */
	CY_LOW_PWR_STATE,	/* not currently used  */
	CY_SLEEP_STATE,		/* app is running, IC is idle */
	CY_BL_STATE,		/* bootloader is running */
	CY_LDR_STATE,		/* loader is running */
	CY_SYSINFO_STATE,	/* switching to sysinfo mode */
	CY_INVALID_STATE	/* always last in the list */
};

static char *cyttsp_powerstate_string[] = {
	/* Order must match enum cyttsp_powerstate above */
	"IDLE",
	"READY",
	"ACTIVE",
	"LOW_PWR",
	"SLEEP",
	"BOOTLOADER",
	"LOADER",
	"SYSINFO",
	"INVALID"
};

enum cyttsp_ic_grpnum {
	CY_IC_GRPNUM_RESERVED = 0,
	CY_IC_GRPNUM_OP_TAG,	/* Platform Data Operational tagged registers */
	CY_IC_GRPNUM_SI_TAG,	/* Platform Data Sysinfo tagged registers */
	CY_IC_GRPNUM_BL_KEY,	/* Platform Data Bootloader Keys */
	CY_IC_GRPNUM_OP_REG,	/* general Operational registers read/write */
	CY_IC_GRPNUM_SI_REG,	/* general Sysinfo registers read/write */
	CY_IC_GRPNUM_BL_REG,	/* general Bootloader registers read access */
	CY_IC_GRPNUM_NUM	/* always last */
};

/* Touch structure */
struct cyttsp_touch {
	__be16 x;
	__be16 y;
	u8 z;
} __packed;

/* TrueTouch Standard Product Gen3 interface definition */
struct cyttsp_status_regs {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
} __packed;

#ifdef CY_USE_GEN2
struct cyttsp_xydata_g2 {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	struct cyttsp_touch tch1;
	u8 evnt_idx;
	struct cyttsp_touch tch2;
	u8 tt_undef1;
	u8 gest_cnt;
	u8 gest_id;
	u8 tt_undef[11];
	u8 buttons;
	u8 alive;
	u8 tt_undef2;
	u8 act_dist;
	u8 reserved1;
} __packed;
#endif /* --CY_USE_GEN2 */

struct cyttsp_xydata {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	struct cyttsp_touch tch1;
	u8 touch12_id;
	struct cyttsp_touch tch2;
	u8 gest_cnt;
	u8 gest_id;
	struct cyttsp_touch tch3;
	u8 touch34_id;
	struct cyttsp_touch tch4;
	u8 tt_undef;
	u8 buttons;
	u8 alive;
	u8 act_dist;
	u8 feature;
} __packed;

/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
	u8 hst_mode;
	u8 mfg_stat;
	u8 mfg_cmd;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u8 bl_verh;
	u8 bl_verl;
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 tt_undef[4];
	u8 ca_cntrl;
	u8 scn_typ;
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
} __packed;

/* TTSP Bootloader Register Map interface definition */
struct cyttsp_bootloader_data {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 blver_hi;
	u8 blver_lo;
	u8 bld_blver_hi;
	u8 bld_blver_lo;
	u8 ttspver_hi;
	u8 ttspver_lo;
	u8 appid_hi;
	u8 appid_lo;
	u8 appver_hi;
	u8 appver_lo;
	u8 cid_0;
	u8 cid_1;
	u8 cid_2;
} __packed;

struct cyttsp_tch {
	struct cyttsp_touch *tch;
	u8 *id;
};

struct cyttsp_trk {
	bool tch;
	int abs[CY_NUM_ABS_SET];
};

struct cyttsp {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct mutex data_lock;		/* Used to prevent concurrent access */
	struct workqueue_struct *cyttsp_wq;
	struct work_struct cyttsp_resume_startup_work;
	char phys[32];
	const struct bus_type *bus_type;
	struct cyttsp3_ff_platform_data *platform_data;
	struct cyttsp_bus_ops *bus_ops;
	struct cyttsp_xydata xy_data;
	struct cyttsp_bootloader_data bl_data;
	struct cyttsp_sysinfo_data sysinfo_data;
	struct cyttsp_trk prv_trk[CY_NUM_TRK_ID];
	struct cyttsp_tch tch_map[CY_NUM_TCH_ID];
	struct completion bl_int_running;
	struct completion si_int_running;
	struct completion op_int_running;
	struct completion ld_int_running;
	enum cyttsp_powerstate power_state;
	bool irq_enabled;
	bool powered;
	bool was_suspended;
	u16 flags;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
/*** ZTEMT start ***/
#elif defined(CONFIG_FB)
	struct notifier_block fb_notifier;
/*ZTEMT end*/
#endif
#ifdef CY_USE_WATCHDOG
	struct work_struct work;
	struct timer_list timer;
	bool watchdog_enabled;
	int ntch_count;
	int prv_tch;
#endif /* --CY_USE_WATCHDOG */
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	char *fwname;
	bool waiting_for_fw;
	int ic_grpnum;
	int ic_grpoffset;
	bool ic_reflash_done;
#endif
#ifdef CY_USE_REG_ACCESS
	u8 rw_regid;
#endif
#ifndef CY_USE_THREADED_IRQ
	struct work_struct irq_work;
#endif
	struct work_struct core_probe_work;
	struct timer_list core_probe_timer;
};

static int _cyttsp_startup(struct cyttsp *ts);
static int _cyttsp_wakeup(struct cyttsp *ts);
static void _cyttsp_queue_startup(struct cyttsp *ts, bool was_suspended);
#ifndef CY_USE_THREADED_IRQ
static struct workqueue_struct *parade_wq;
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP3_DEVICETREE_SUPPORT
extern struct cyttsp3_force_function _cyttsp3_force_function;
#endif

/*** ZTEMT start ***/
extern int rst_gpio_number;
extern int irq_gpio_number;
/*ZTEMT end*/

int ttsp_read_block_data(struct cyttsp *ts, u8 addr, 
	u8 length, void *buf)
{
	int retval = 0;
	int tries = 0;

	if (buf == NULL || length == 0) {
		pr_err("%s: No data passed - buf=%p length=%d\n",
			__func__, buf, length);
		retval = -EINVAL;
		goto ttsp_read_block_data_exit;
	}

	for (tries = 0, retval = -1;
		(tries < CY_NUM_RETRY) && (retval < 0);
		tries++) {
		retval = ts->bus_ops->read(ts->bus_ops, addr, length, buf);
		if (retval < 0) {
			/*
			 * SPI can require retries based on data content
			 * I2C should not have errors
			 */
			msleep(CY_DELAY_DFLT);
		} else
			break;
	}

	if (retval < 0) {
		pr_err("%s: bus read block data fail r=%d\n",
			__func__, retval);
	}
ttsp_read_block_data_exit:
	return retval;
}
EXPORT_SYMBOL_GPL(ttsp_read_block_data);

int ttsp_write_block_data(struct cyttsp *ts, u8 addr,
	u8 length, const void *buf)
{
	int retval = 0;
	int tries = 0;

	if (buf == NULL || length == 0) {
		pr_err("%s: No data passed - buf=%p length=%d\n",
			__func__, buf, length);
		retval = -EINVAL;
		goto ttsp_write_block_data_exit;
	}

	for (tries = 0, retval = -1;
		(tries < CY_NUM_RETRY) && (retval < 0);
		tries++) {
		retval = ts->bus_ops->write(ts->bus_ops, addr, length, buf);
		if (retval < 0) {
			/*
			 * SPI can require retries based on data content
			 * I2C should not have errors
			 */
			msleep(CY_DELAY_DFLT);
		} else
			break;
	}

	if (retval < 0) {
		pr_err("%s: bus write block data fail r=%d\n",
			__func__, retval);
	}

ttsp_write_block_data_exit:
	return retval;
}
EXPORT_SYMBOL_GPL(ttsp_write_block_data);

int cyttsp3_read_block_data(void *handle, u8 addr, u8 length, void *buf)
{
	struct cyttsp *ts = (struct cyttsp *)handle;
	
	if (ts)
		return ttsp_read_block_data(ts, addr, length,buf);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(cyttsp3_read_block_data);

int cyttsp3_write_block_data(void *handle, u8 addr,
	u8 length, const void *buf)
{
	struct cyttsp *ts = (struct cyttsp *)handle;
	
	if (ts)
		return ttsp_write_block_data(ts, addr, length, buf);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(cyttsp3_write_block_data);
	
static void _cyttsp_change_state(struct cyttsp *ts,
	enum cyttsp_powerstate new_state)
{
	ts->power_state = new_state;
	pr_info("%s: %s\n", __func__,
		ts->power_state < CY_INVALID_STATE ?
		cyttsp_powerstate_string[ts->power_state] :
		"INVALID");
}

static void _cyttsp_dbg_pr_buf(struct cyttsp *ts,
	const u8 *dptr, int size, const char *data_name)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int i = 0;
	int max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(" truncated...");
	char *pbuf = NULL;

	if (ts->bus_ops->tsdebug >= CY_DBG_LVL_2) {
		pbuf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
		if (pbuf == NULL) {
			pr_err("%s: buf alloc error\n", __func__);
		} else {
			for (i = 0; i < size && i < max; i++)
				sprintf(pbuf, "%s %02X", pbuf, dptr[i]);
			cyttsp_dbg(ts, CY_DBG_LVL_2,
				"%s:  %s[0..%d]=%s%s\n", __func__,
				data_name, size-1, pbuf,
				size <= max ?
				"" : " truncated...");
			kfree(pbuf);
		}
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	return;
}

static int _cyttsp_hndshk(struct cyttsp *ts, u8 hst_mode)
{
	int retval = 0;
	u8 mode = 0;

	mode = hst_mode & CY_HNDSHK_BIT ?
		hst_mode & ~CY_HNDSHK_BIT :
		hst_mode | CY_HNDSHK_BIT;

	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		sizeof(mode), &mode);

	if (retval < 0) {
		pr_err("%s: bus write fail on handshake r=%d\n",
			__func__, retval);
	}

	return retval;
}

static int _cyttsp_load_status_regs(struct cyttsp *ts,
	struct cyttsp_status_regs *status_regs)
{
	int retval = 0;

	memset(status_regs, 0, sizeof(struct cyttsp_status_regs));

	retval = ttsp_read_block_data(ts, CY_REG_BASE,
		sizeof(struct cyttsp_status_regs), status_regs);

	if (retval < 0) {
		pr_err("%s: bus fail reading status regs r=%d\n",
			__func__, retval);
	}

	return retval;
}

static int _cyttsp_wait_ready(struct cyttsp *ts, struct completion *complete,
	u8 *cmd, size_t cmd_size, unsigned long timeout_ms)
{
	unsigned long timeout = 0;
	unsigned long uretval = 0;
	int retval = 0;

	timeout = msecs_to_jiffies(timeout_ms);
	init_completion(complete);
	if ((cmd != NULL) && (cmd_size != 0)) {
		retval = ttsp_write_block_data(ts, CY_REG_BASE, cmd_size, cmd);
		if (retval < 0) {
			pr_err("%s: bus write fail switch mode r=%d\n",
				__func__, retval);
			_cyttsp_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp_wait_ready_exit;
		}
	}
	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(complete, timeout);
	mutex_lock(&ts->data_lock);
	if (uretval == 0) {
		pr_err("%s: Switch Mode Timeout waiting"
			" for ready interrupt - try reading regs\n", __func__);
		/* continue anyway */
		retval = 0;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_2,
		"%s: Switch Mode: ret=%d uretval=%lu timeout=%lu",
		__func__, retval, uretval, timeout);

_cyttsp_wait_ready_exit:
	return retval;
}

static int _cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval = 0;

	memset(&(ts->bl_data), 0, sizeof(struct cyttsp_bootloader_data));

	retval = ttsp_read_block_data(ts, CY_REG_BASE,
		sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		pr_err("%s: bus fail reading Bootloader regs r=%d\n",
			__func__, retval);
		/* Calling process determines state change requirement */
		goto cyttsp_load_bl_regs_exit;
	}

	if (IS_BOOTLOADERMODE(ts->bl_data.bl_status)) {
		cyttsp_dbg(ts, CY_DBG_LVL_1,
			"%s: Bootloader Regs:\n"
			"  file=%02X status=%02X error=%02X\n"
			"  BL Version:          0x%02X%02X\n"
			"  Build BL Version:    0x%02X%02X\n"
			"  TTSP Version:        0x%02X%02X\n"
			"  Application ID:      0x%02X%02X\n"
			"  Application Version: 0x%02X%02X\n"
			"  Custom ID:           0x%02X%02X%02X\n",
			__func__,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error,
			ts->bl_data.blver_hi, ts->bl_data.blver_lo,
			ts->bl_data.bld_blver_hi, ts->bl_data.bld_blver_lo,
			ts->bl_data.ttspver_hi, ts->bl_data.ttspver_lo,
			ts->bl_data.appid_hi, ts->bl_data.appid_lo,
			ts->bl_data.appver_hi, ts->bl_data.appver_lo,
			ts->bl_data.cid_0, ts->bl_data.cid_1,
			ts->bl_data.cid_2);
	} else {
		cyttsp_dbg(ts, CY_DBG_LVL_1,
			"%s: Not Bootloader mode:\n"
			"  mode=%02X status=%02X error=%02X\n",
			__func__,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error);
	}

cyttsp_load_bl_regs_exit:
	return retval;
}

static int _cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	struct cyttsp_status_regs status_regs;
	int retval = 0;
	u8 bl_cmd[] = {0x00, 0xFF, 0xA5,/* BL file 0, cmd seq, exit cmd */
		0, 1, 2, 3, 4, 5, 6, 7};/* Default BL keys */

	memset(&status_regs, 0, sizeof(struct cyttsp_status_regs));
	if (ts->platform_data->sett[CY_IC_GRPNUM_BL_KEY] != NULL) {
		if ((ts->platform_data->sett[CY_IC_GRPNUM_BL_KEY]->data !=
			NULL) && (ts->platform_data->sett[CY_IC_GRPNUM_BL_KEY]
			->size <= CY_BL_NUM_KEYS)) {
			memcpy(&bl_cmd[3],
				ts->platform_data->sett
				[CY_IC_GRPNUM_BL_KEY]->data,
				ts->platform_data->sett
				[CY_IC_GRPNUM_BL_KEY]->size);
		} else {
			pr_err("%s: Too many platform bootloader keys--%s.\n",
				__func__, "using defaults instead");
		}
	} else {
		pr_err("%s: Missing platform bootloader keys--%s.\n",
			__func__, "using defaults instead");
	}

	_cyttsp_dbg_pr_buf(ts, bl_cmd, sizeof(bl_cmd), "bl_cmd");

	/* send exit command; wait for host mode interrupt */
	retval = _cyttsp_wait_ready(ts, &ts->bl_int_running,
		bl_cmd, sizeof(bl_cmd), CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		pr_err("%s: fail wait ready r=%d\n", __func__, retval);
		goto _cyttsp_exit_bl_mode_exit;
	}

	retval = _cyttsp_load_status_regs(ts, &status_regs);
	if (retval < 0) {
		pr_err("%s: Fail read status regs r=%d\n",
			__func__, retval);
		goto _cyttsp_exit_bl_mode_exit;
	}

	retval = _cyttsp_hndshk(ts, status_regs.hst_mode);
	if (retval < 0) {
		pr_err("%s: Fail write handshake r=%d\n",
			__func__, retval);
		/* continue anyway */
		retval = 0;
	}

	if (IS_BOOTLOADERMODE(status_regs.tt_mode)) {
		pr_err("%s: Fail exit bootloader mode r=%d\n",
			__func__, retval);
		retval = -EIO;
	} else if (GET_HSTMODE(status_regs.hst_mode) !=
		GET_HSTMODE(CY_OPERATE_MODE)) {
		pr_err("%s: Fail enter op mode=0x%02X\n",
			__func__, status_regs.hst_mode);
		retval = -EIO;
	} else
		_cyttsp_change_state(ts, CY_READY_STATE);

_cyttsp_exit_bl_mode_exit:
	return retval;
}

static int _cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval = 0;
#ifdef CY_USE_LEVEL_IRQ
	int tries = 0;
#endif
	u8 mode = CY_OPERATE_MODE + CY_HST_MODE_CHANGE_BIT;
	u8 *mode_ptr = &mode;
	size_t mode_size = sizeof(mode);

	/* switch to operational mode */
	_cyttsp_change_state(ts, CY_READY_STATE);
_cyttsp_set_operational_mode_retry:
	retval = _cyttsp_wait_ready(ts, &ts->op_int_running,
		mode_ptr, mode_size, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		pr_err("%s: fail wait ready r=%d\n", __func__, retval);
		goto _cyttsp_set_operational_mode_exit;
	}

	retval = ttsp_read_block_data(ts, CY_REG_BASE,
		sizeof(struct cyttsp_xydata), &(ts->xy_data));
	if (retval < 0) {
		pr_err("%s: Fail read status regs r=%d\n",
			__func__, retval);
		goto _cyttsp_set_operational_mode_exit;
	}

	retval = _cyttsp_hndshk(ts, ts->xy_data.hst_mode);
	if (retval < 0) {
		pr_err("%s: Fail write handshake r=%d\n",
			__func__, retval);
		/* continue anyway */
		retval = 0;
	}

	/*
	 * The CY_HST_MODE_CHANGE_BIT is set by the host when changing mode.
	 * When the mode change is complete, the device will pulse the
	 * interrupt line and clear this bit to signify that the
	 * mode change is complete.
	 */
	if (ts->xy_data.hst_mode & CY_HST_MODE_CHANGE_BIT) {
		/*
		 * due to race between last sysinfo heartbeat and
		 * the mode change interrupt, need to try waiting
		 * again without repeating the write operation
		 */
		if (mode_ptr != NULL) {
			mode_ptr = NULL;
			mode_size = 0;
			goto _cyttsp_set_operational_mode_retry;
		} else {
			pr_err("%s: Fail change mode=0x%02X\n",
				__func__, ts->xy_data.hst_mode);
			goto _cyttsp_set_operational_mode_exit;
		}
	}

	if ((GET_HSTMODE(ts->xy_data.hst_mode) !=
		GET_HSTMODE(CY_OPERATE_MODE)) &&
		!IS_BOOTLOADERMODE(ts->xy_data.tt_mode)) {
		pr_err("%s: Fail enter op mode"
			" hst_mode=0x%02X tt_mode=0x%02X\n",
			__func__, ts->xy_data.hst_mode, ts->xy_data.tt_mode);
		retval = -EIO;
		goto _cyttsp_set_operational_mode_exit;
	} else {
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Enter Operational mode"
			" hst_mode=0x%02X tt_mode=0x%02X\n",
			__func__, ts->xy_data.hst_mode, ts->xy_data.tt_mode);
		/* enable force function */
		_cyttsp3_force_function.operational = 1;
	}

_cyttsp_set_operational_mode_exit:
	if (retval < 0)
		_cyttsp_queue_startup(ts, false);
	return retval;
}

static int _cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval = 0;
	u8 mode = CY_SYSINFO_MODE + CY_HST_MODE_CHANGE_BIT;
	u8 *mode_ptr = &mode;
	size_t mode_size = sizeof(mode);

	/* switch to sysinfo mode */
	_cyttsp_change_state(ts, CY_SYSINFO_STATE);
_cyttsp_set_sysinfo_mode_retry:
	retval = _cyttsp_wait_ready(ts, &ts->si_int_running,
		mode_ptr, mode_size, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		pr_err("%s: fail wait ready r=%d\n", __func__, retval);
		goto _cyttsp_set_sysinfo_mode_exit;
	}

	/* sysinfo_data is filled by ISR */
	if (ts->sysinfo_data.hst_mode & CY_HST_MODE_CHANGE_BIT) {
		/*
		 * due to race between last bootloader heartbeat and
		 * the mode change interrupt, need to try waiting
		 * again without repeating the write operation
		 */
		if (mode_ptr != NULL) {
			mode_ptr = NULL;
			mode_size = 0;
			goto _cyttsp_set_sysinfo_mode_retry;
		} else {
			pr_err("%s: Fail change mode=0x%02X\n",
				__func__, ts->sysinfo_data.hst_mode);
			goto _cyttsp_set_sysinfo_mode_exit;
		}
	}

	if (GET_HSTMODE(ts->sysinfo_data.hst_mode) !=
		GET_HSTMODE(CY_SYSINFO_MODE)) {
		pr_err("%s: Fail enter Sysinfo mode hst_mode=0x%02X\n",
			__func__, ts->sysinfo_data.hst_mode);
		retval = -EIO;
	} else {
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Enter Sysinfo mode hst_mode=0x%02X\n",
			__func__, ts->sysinfo_data.hst_mode);
	}

_cyttsp_set_sysinfo_mode_exit:
	return retval;
}

static int _cyttsp_wait_sysinfo_heartbeat(struct cyttsp *ts)
{
	unsigned long timeout = 0;
	unsigned long uretval = 0;
	int retval = 0;

	timeout = msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX);
	init_completion(&ts->si_int_running);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->si_int_running, timeout);
	if (uretval == 0) {
		pr_err("%s: Timeout waiting for Sysinfo heartbeat"
			" uretval=%lu timeout=%lu\n",
			__func__, uretval, timeout);
		retval = -ETIME;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_2,
		"%s: Wait sysinfo irq:"
		" r=%d uretval=%lu timeout=%lu\n",
		__func__, retval, uretval, timeout);

	return retval;
}

static int _cyttsp_put_store_data(struct cyttsp *ts,
	int grpnum, size_t offset, size_t num_write, const u8 *ic_buf)
{
	int retval = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: Write group=%d bytes at offset=%ld\n",
		__func__, grpnum, offset);
	_cyttsp_dbg_pr_buf(ts, ic_buf, num_write, "Group_data");

	retval = ttsp_write_block_data(ts, offset, num_write, ic_buf);
	if (retval < 0) {
		pr_err("%s: Err write numbytes=%ld r=%d\n",
			__func__, num_write, retval);
	}

	return retval;
}

static int _cyttsp_put_si_data(struct cyttsp *ts,
	int grpnum, size_t offset, size_t num_write, const u8 *ic_buf)
{
	int retval = 0;

	mutex_unlock(&ts->data_lock);
	retval = _cyttsp_put_store_data(ts, offset, grpnum, num_write, ic_buf);
	if (retval < 0)
		pr_err("%s: fail put store data r=%d\n", __func__, retval);
	else {
		/* wait for data good */
		retval = _cyttsp_wait_sysinfo_heartbeat(ts);
		if (retval < 0) {
			pr_err("%s: Timeout waiting for sysinfo"
				" heartbeat r=%d\n", __func__, retval);
			/* continue; data should be good after timeout */
		}
	}
	mutex_lock(&ts->data_lock);

	return retval;
}

static int _cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	u8 size = 0;
	u8 tag = 0;
	u8 reg_offset = 0;
	u8 data_len = 0;
	int retval = 0;

	if ((ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG] == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG]->data == NULL)) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Sysinfo data table missing--%s",
			__func__, "IC defaults will be used\n");
		goto _cyttsp_set_sysinfo_regs_exit;
	}
	size = ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG]->size;
	tag = ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG]->tag;
	reg_offset = CY_REG_SI_START + tag;
	data_len = size - tag;  /* Overflows if tag > size; checked below */

	/* Do bounds checking on the data */
	if (reg_offset > CY_REG_SI_END) {
		pr_err("%s: Sysinfo reg data starts out of bounds--%s",
			__func__, "IC defaults will be used\n");
		goto _cyttsp_set_sysinfo_regs_exit;
	} else if (tag > size) { /* overflow if (size - tag) is negative */
		pr_err("%s: Invalid sysinfo data length--%s",
			__func__, "IC defaults will be used\n");
		goto _cyttsp_set_sysinfo_regs_exit;
	} else if ((reg_offset + data_len - 1) > CY_REG_SI_END) {
		pr_err("%s: Sysinfo reg data overflow--%s",
			__func__, "IC defaults will be used\n");
		goto _cyttsp_set_sysinfo_regs_exit;
	}

	_cyttsp_dbg_pr_buf(ts,
		&(ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG]->data[tag]),
		data_len, "write sysinfo_regs");

	retval = _cyttsp_put_si_data(ts,
		CY_IC_GRPNUM_SI_TAG, reg_offset, data_len,
		&(ts->platform_data->sett[CY_IC_GRPNUM_SI_TAG]->data[tag]));
	if (retval < 0) {
		pr_err("%s: Error writing Sysinfo data r=%d\n",
			__func__, retval);
	}

_cyttsp_set_sysinfo_regs_exit:
	return retval;
}

static int _cyttsp_hard_reset(struct cyttsp *ts)
{
	bool do_soft_reset = false;
	u8 mode = 0;
	int retval = 0;

#ifdef CY_NO_HW
	return retval;
#endif

	/* disable force function */
	_cyttsp3_force_function.operational = 0;
	_cyttsp_change_state(ts, CY_BL_STATE);

	if (ts->platform_data->hw_reset != NULL) {
		retval = ts->platform_data->hw_reset(ts->platform_data);
		if (retval < 0) {
			if (retval == -ENOSYS) {
				cyttsp_dbg(ts, CY_DBG_LVL_2,
					"%s: hw_reset not implemented\n",
					__func__);
			} else {
				pr_err("%s: fail on hard reset r=%d\n",
					__func__, retval);
			}
			do_soft_reset = true;
		} else {
			cyttsp_dbg(ts, CY_DBG_LVL_2,
				"%s: hard reset ok\n", __func__);
		}
	} else {
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: no hardware reset function r=%d\n",
			__func__, retval);
		do_soft_reset = true;
	}

	if (do_soft_reset) {
		mode = CY_SOFT_RESET_MODE;
		retval = ttsp_write_block_data(ts, CY_REG_BASE,
			sizeof(mode), &mode);
		if (retval < 0) {
			pr_err("%s: Fail writing soft reset command r=%d\n",
				__func__, retval);
			goto _cyttsp_hard_reset_exit;
		} else {
			cyttsp_dbg(ts, CY_DBG_LVL_2,
				"%s: soft reset ok\n", __func__);
		}
	}

	/* wait for interrupt to set ready completion */
	retval = _cyttsp_wait_ready(ts, &ts->bl_int_running,
		NULL, 0, CY_HALF_SEC_TMO_MS);
	if (retval < 0)
		pr_err("%s: fail wait ready r=%d\n", __func__, retval);

_cyttsp_hard_reset_exit:
	return retval;
}

static int _cyttsp_set_operational_regs(struct cyttsp *ts)
{
	int retval = 0;
	u8 size = 0;
	u8 tag = 0;
	u8 reg_offset = 0;
	u8 data_len = 0;

	if ((ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG] == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG]->data == NULL)) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Op data table missing--%s",
			__func__, "IC defaults will be used\n");
		goto _cyttsp_set_op_regs_exit;
	}
	size = ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG]->size;
	tag = ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG]->tag;
	reg_offset = CY_REG_OP_START + tag;
	data_len = size - tag;  /* Overflows if tag > size; checked below */

	/* Do bounds checking on the data */
	if (reg_offset > CY_REG_OP_END) {
		pr_err("%s: Op reg data starts out of bounds--%s\n",
			__func__, "IC defaults will be used");
		goto _cyttsp_set_op_regs_exit;
	} else if (tag > size) { /* overflow if (size - tag) is negative */
		pr_err("%s: Invalid op data length--%s\n",
			__func__, "IC defaults will be used");
		goto _cyttsp_set_op_regs_exit;
	} else if ((reg_offset + data_len - 1) > CY_REG_OP_END) {
		pr_err("%s: Op reg data overflow--%s\n",
			__func__, "IC defaults will be used");
		goto _cyttsp_set_op_regs_exit;
	}

	_cyttsp_dbg_pr_buf(ts,
		&(ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG]->data[tag]),
		data_len, "write op_regs");

	retval = ttsp_write_block_data(ts, reg_offset, data_len,
		&(ts->platform_data->sett[CY_IC_GRPNUM_OP_TAG]->data[tag]));

	if (retval < 0) {
		pr_err("%s: bus write fail on Op Regs r=%d\n",
			__func__, retval);
		_cyttsp_change_state(ts, CY_IDLE_STATE);
	}

_cyttsp_set_op_regs_exit:
	return retval;
}


static bool _cyttsp_chk_bl_startup(struct cyttsp *ts)
{
	return !(ts->bl_data.bl_status & CY_BL_BUSY) &&
		(ts->bl_data.bl_error == CY_BL_RUNNING);
}

static bool _cyttsp_chk_bl_terminate(struct cyttsp *ts)
{
	return (ts->bl_data.bl_status == CY_BL_READY_APP) &&
		(ts->bl_data.bl_error == CY_BL_NOERR);
}

static int _cyttsp_wait_bl_ready(struct cyttsp *ts,
	bool (*cond)(struct cyttsp *), int timeout_ms)
{
	unsigned long timeout = 0;
	unsigned long uretval = 0;
	int retval = 0;

	/* wait for interrupt to set ready completion */
	timeout = msecs_to_jiffies(timeout_ms);

	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->ld_int_running, timeout);
	mutex_lock(&ts->data_lock);

	if (uretval == 0) {
		pr_err("%s: Loader timeout waiting for loader ready\n",
			__func__);
		/* do not return error; continue in case of missed interrupt */
	}

	/* if cond == NULL then no status check is required */
	if (cond != NULL) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Loader Start: ret=%d uretval=%lu timeout=%lu\n",
			__func__, retval, uretval, timeout);

		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Load BL regs err r=%d\n",
				__func__, retval);
		} else if (cond(ts) == false) {
			if (uretval == 0) {
				pr_err("%s: BL loader timeout err"
					" ret=%d uretval=%lu timeout=%lu\n",
				__func__, retval, uretval, timeout);
				retval = -ETIME;
			} else {
				pr_err("%s: BL loader status err"
					" r=%d bl_f=%02X bl_s=%02X bl_e=%02X\n",
					__func__, retval, ts->bl_data.bl_file,
					ts->bl_data.bl_status,
					ts->bl_data.bl_error);
				retval = -EIO;
			}
		}
	}

	return retval;
}

static int _cyttsp_wr_blk_chunks(struct cyttsp *ts, u8 addr,
	u8 len, const u8 *values)
{
	int retval = 0;
	int block = 1;
	int length = len;
	u8 dataray[CY_BL_PAGE_SIZE + 1];

	memset(dataray, 0, sizeof(dataray));

	/* first page already includes the bl page offset */
	memcpy(dataray, values, sizeof(dataray));
	init_completion(&ts->ld_int_running);
	retval = ttsp_write_block_data(ts, addr, sizeof(dataray), dataray);
	if (retval < 0) {
		pr_err("%s: Write chunk err block=0 r=%d\n",
			__func__, retval);
		goto _cyttsp_wr_blk_chunks_exit;
	}

	values += CY_BL_PAGE_SIZE + 1;
	length -= CY_BL_PAGE_SIZE + 1;

	/* remaining pages require bl page offset stuffing */
	for (block = 1; block < CY_BL_NUM_PAGES; block++) {
		if (length < 0) {
			pr_err("%s: Write chunk length err block=%d r=%d\n",
				__func__, block, retval);
			goto _cyttsp_wr_blk_chunks_exit;
		}
		dataray[0] = CY_BL_PAGE_SIZE * block;

		retval = _cyttsp_wait_bl_ready(ts, NULL, 100);
		if (retval < 0) {
			pr_err("%s: wait ready timeout block=%d length=%d\n",
				__func__, block, length);
			goto _cyttsp_wr_blk_chunks_exit;
		}

		memcpy(&dataray[1], values, length >= CY_BL_PAGE_SIZE ?
			CY_BL_PAGE_SIZE : length);
		init_completion(&ts->ld_int_running);
		retval = ttsp_write_block_data(ts, addr,
			length >= CY_BL_PAGE_SIZE ?
			sizeof(dataray) : length + 1, dataray);
		if (retval < 0) {
			pr_err("%s: Write chunk err block=%d r=%d\n",
				__func__, block, retval);
			goto _cyttsp_wr_blk_chunks_exit;
		}

		values += CY_BL_PAGE_SIZE;
		length -= CY_BL_PAGE_SIZE;
	}

	retval = _cyttsp_wait_bl_ready(ts, NULL, 200);
	if (retval < 0) {
		pr_err("%s: last wait ready timeout block=%d length=%d\n",
			__func__, block, length);
		goto _cyttsp_wr_blk_chunks_exit;
	}

	retval = _cyttsp_load_bl_regs(ts);
	if (retval < 0) {
		pr_err("%s: Load BL regs err r=%d\n",
			__func__, retval);
		goto _cyttsp_wr_blk_chunks_exit;
	}

	if (((ts->bl_data.bl_status & CY_BL_STATUS_MASK) == CY_BL_READY_APP))
		retval = 0;
	else if (((ts->bl_data.bl_status  & CY_BL_STATUS_MASK) ==
		CY_BL_READY_NO_APP) &&
		((ts->bl_data.bl_error & CY_BL_ERROR_MASK) == CY_BL_RUNNING))
		retval = 0;
	else
		retval = -ETIMEDOUT;

_cyttsp_wr_blk_chunks_exit:
	return retval;
}

#define MAX_RETRY_BL_ENTER_CMD 2
static int _cyttsp_load_app(struct cyttsp *ts, const u8 *fw, int fw_size)
{
	int retval = 0;
	int loc = 0;
	int retry = 0;//ZHWW: add retry after sending key
    pr_err("%s: ztemt\n", __func__);

	/* sync up with the next bootloader heartbeat interrupt in order
	 * to remove haearbeat vs. repsonse interrupt ambiguity
	 */
	retval = _cyttsp_wait_ready(ts, &ts->bl_int_running,
		NULL, 0, CY_HALF_SEC_TMO_MS);
	if (retval < 0) {
		pr_err("%s: fail wait ready r=%d\n", __func__, retval);
		goto _cyttsp_load_app_exit;
	}

	_cyttsp_change_state(ts, CY_LDR_STATE);

	/* send bootload initiation command */
	pr_info("%s: Send BL Loader Enter\n", __func__);
	init_completion(&ts->ld_int_running);
	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		CY_BL_ENTER_CMD_SIZE, (void *)(&fw[loc]));
	if (retval < 0) {
		pr_err("%s: BL loader fail startup r=%d\n", __func__, retval);
		goto _cyttsp_load_app_exit;
	}
    pr_err("%s: ztemt 000\n", __func__);

	for(retry = 0; retry < MAX_RETRY_BL_ENTER_CMD; retry++){
		retval = _cyttsp_wait_bl_ready(ts, _cyttsp_chk_bl_startup,
			CY_TEN_SEC_TMO_MS);
		if (retval < 0) {
			pr_err("%s: BL loader startup err r=%d\n", __func__, retval);
			//goto _cyttsp_load_app_exit;
		}
		else{
			retry = MAX_RETRY_BL_ENTER_CMD;
			pr_err("%s: BL loader startup ok\n", __func__);
			break;
		}
	}
    pr_err("%s: ztemt 111 \n", __func__);

	loc += CY_BL_ENTER_CMD_SIZE;

	/* send bootload firmware load blocks */
	pr_info("%s: Send BL Loader Blocks\n", __func__);
	while ((fw_size - loc) > CY_BL_WR_BLK_CMD_SIZE) {
        pr_err("%s: BL loader block=%d"
			" f=%02X s=%02X e=%02X loc=%d\n", __func__,
			loc / CY_BL_WR_BLK_CMD_SIZE,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error, loc);

		retval = _cyttsp_wr_blk_chunks(ts, CY_REG_BASE,
			CY_BL_WR_BLK_CMD_SIZE, &fw[loc]);
		if (retval < 0) {
			pr_err("%s: BL loader fail"
				" r=%d block=%d loc=%d\n", __func__, retval,
				loc / CY_BL_WR_BLK_CMD_SIZE, loc);
			goto _cyttsp_load_app_exit;
		}

		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: BL Read error r=%d\n", __func__, retval);
			goto _cyttsp_load_app_exit;
		}

		if ((ts->bl_data.bl_status & CY_BL_BUSY) ||
			(ts->bl_data.bl_error != CY_BL_RUNNING)) {
			/* signal a status err */
			pr_err("%s: BL READY ERR on write block"
				" bl_file=%02X bl_status=%02X bl_error=%02X\n",
				__func__, ts->bl_data.bl_file,
				ts->bl_data.bl_status, ts->bl_data.bl_error);
			retval = -EIO;
			goto _cyttsp_load_app_exit;
		} else {
			/* point to next block */
			loc += CY_BL_WR_BLK_CMD_SIZE;
		}
	}

	/* send bootload terminate command */
	pr_info("%s: Send BL Loader Terminate\n", __func__);
	if (loc == (fw_size - CY_BL_EXIT_CMD_SIZE)) {
		init_completion(&ts->ld_int_running);
		retval = ttsp_write_block_data(ts, CY_REG_BASE,
			CY_BL_EXIT_CMD_SIZE, (void *)(&fw[loc]));
		if (retval < 0) {
			pr_err("%s: BL fail Terminate r=%d\n",
				__func__, retval);
			goto _cyttsp_load_app_exit;
		}

		retval = _cyttsp_wait_bl_ready(ts, _cyttsp_chk_bl_terminate,
			CY_TEN_SEC_TMO_MS);
		if (retval < 0) {
			pr_err("%s: BL Loader Terminate err r=%d\n",
				__func__, retval);
			/*
			 * if app valid bit is set
			 * return success and let driver
			 * try a normal restart anyway
			 */
			if (IS_VALID_APP(ts->bl_data.bl_status)) {
				pr_err("%s: BL Loader Valid App indicated"
					" bl_s=%02X\n", __func__,
					ts->bl_data.bl_status);
				retval = 0;
			}
		}
	} else {
		pr_err("%s: FW size mismatch\n", __func__);
		retval = -EINVAL;
		goto _cyttsp_load_app_exit;
	}

_cyttsp_load_app_exit:
	return retval;
}

static int _cyttsp_boot_loader(struct cyttsp *ts)
{
	int retval = 0;
	bool new_vers = false;
	struct cyttsp_vers *fw = NULL;

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	/*
	 * Prevent forced old firmware
	 * from being re-written by platform firmware
	 * on warm startup.
	 * Cold startup will restore old firmware
	 * to new platform firmware
	 */
	if (ts->ic_reflash_done) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Prevent writing over old firmware\n",
			__func__);
		goto _cyttsp_boot_loader_exit;
	}
#endif

	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: cannot upgrade firmware in %s state\n",
			__func__, cyttsp_powerstate_string[ts->power_state]);
		goto _cyttsp_boot_loader_exit;
	}

	if ((ts->platform_data->fw->ver == NULL) ||
		(ts->platform_data->fw->img == NULL)) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: empty version list or no image\n",
			__func__);
		goto _cyttsp_boot_loader_exit;
	}

	if (ts->platform_data->fw->vsize != CY_BL_VERS_SIZE) {
		pr_err("%s: bad fw version list size=%d\n",
			__func__, ts->platform_data->fw->vsize);
		goto _cyttsp_boot_loader_exit;
	}

	/* automatically update firmware if new version detected */
	fw = (struct cyttsp_vers *)ts->platform_data->fw->ver;
#ifdef CY_ANY_DIFF_NEW_VER
	new_vers =
		(((u16)fw->app_verh << 8) +
		(u16)fw->app_verl) !=
		(((u16)ts->bl_data.appver_hi << 8) +
		(u16)ts->bl_data.appver_lo);
#else
	new_vers =
		(((u16)fw->app_verh << 8) +
		(u16)fw->app_verl) >
		(((u16)ts->bl_data.appver_hi << 8) +
		(u16)ts->bl_data.appver_lo);
#endif

	if (!IS_VALID_APP(ts->bl_data.bl_status) || new_vers) {
		retval = _cyttsp_load_app(ts,
			ts->platform_data->fw->img,
			ts->platform_data->fw->size);
		if (retval < 0) {
			pr_err("%s: bus fail on load fw\n", __func__);
			_cyttsp_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp_boot_loader_exit;
		}

		/* reset TTSP Device back to bootloader mode */
		retval = _cyttsp_hard_reset(ts);
		if (retval < 0) {
			pr_err("%s: Fail on hard reset r=%d\n",
				__func__, retval);
		}
	}

_cyttsp_boot_loader_exit:
	return retval;
}

/* Driver version */
static ssize_t cyttsp_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		ts->input->name, CY_DRIVER_VERSION, CY_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, cyttsp_drv_ver_show, NULL);

/* Driver status */
static ssize_t cyttsp_drv_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver state is %s\n",
		cyttsp_powerstate_string[ts->power_state]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, cyttsp_drv_stat_show, NULL);
#ifdef ZTE_FACTORY_WRITE_BIN_BACK
#define PERSIST_FILE_PATH "/persist/sensors/cyttsp5_paramarray.bin"
unsigned char H2C(char c)
{
    if (c >='a' && c <= 'f')
        return c-'a'+10;
    if (c >='A' && c <= 'F')
        return c-'A'+10;
    if (c >='0' && c <= '9')
        return c-'0';
    return 0;
}

static ssize_t cyttsp_paramarray_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    int retval = 0;
    int i = 0;
    int j = 0;
    int length = (size>>1);
    char buf_temp[256];

    for(i = 0; i < length; i++){
        buf_temp[i] = (H2C(buf[j])<<4) + H2C(buf[j+1]);
        j = j + 2;
        pr_err("ztemt cyttsp buf_temp[%d]=%x\n", i,buf_temp[i]);
    }
    retval = pressure_file_write(PERSIST_FILE_PATH, buf_temp, length);
    dev_info(dev, "%s: pressure_file_write=%d\n", __func__, retval);

    if (retval < 0){
        pr_err("%s error %d\n", __func__, retval);
    }
    else{
        pr_err("%s ok\n", __func__);
    }

    return size;
}

static ssize_t cyttsp_paramarray_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int size = 0;
    char tmp_buf[256];
    int i;
    int retval = 0;

    size = pressure_file_read(PERSIST_FILE_PATH, tmp_buf, sizeof(tmp_buf));
    retval = snprintf(buf, CY_MAX_PRBUF_SIZE, "[cyttsp5_paramarray.bin] size:%d\n", size);
    for(i=0; i<size; i++){
        retval += snprintf(buf+retval, CY_MAX_PRBUF_SIZE - retval, "0x%2x,", tmp_buf[i]);
    }

    return retval;
}
static DEVICE_ATTR(paramarray, S_IWUGO|S_IRUGO, cyttsp_paramarray_show, cyttsp_paramarray_store);

#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Group Number */
static ssize_t cyttsp_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Group: %d\n", ts->ic_grpnum);
}
static ssize_t cyttsp_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_ic_grpnum_store_error_exit;
	}

	if (value > 0xFF) {
		value = 0xFF;
		pr_err("%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
	ts->ic_grpnum = value;

cyttsp_ic_grpnum_store_error_exit:
	retval = size;
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpnum_show, cyttsp_ic_grpnum_store);

/* Group Offset */
static ssize_t cyttsp_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Offset: %u\n", ts->ic_grpoffset);
}
static ssize_t cyttsp_ic_grpoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_ic_grpoffset_store_error_exit;
	}

	if (value > 0xFF) {
		value = 0xFF;
		pr_err("%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
	ts->ic_grpoffset = value;

cyttsp_ic_grpoffset_store_error_exit:
	retval = size;
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpoffset_show, cyttsp_ic_grpoffset_store);

/* Group Data */
static ssize_t _cyttsp_get_show_data(struct cyttsp *ts,
	size_t offset, size_t num_read, u8 *ic_buf, char *buf)
{
	ssize_t uretval = 0;
	int retval = 0;

	if (num_read > 0) {
		retval = ttsp_read_block_data(ts, offset, num_read, ic_buf);
		if (retval < 0) {
			pr_err("%s: Cannot read Group %d Data.\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot read Group %d Data.\n",
				ts->ic_grpnum);
		}
	}

	return uretval;
}
static ssize_t cyttsp_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int i = 0;
	ssize_t uretval = 0;
	int retval = 0;
	int num_read = 0;
	int grpsize = 0;
	u8 ic_buf[sizeof(struct cyttsp_xydata)];

	mutex_lock(&ts->data_lock);

	memset(ic_buf, 0, sizeof(ic_buf));

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_OP_TAG:
		if ((ts->ic_grpoffset + CY_REG_OP_START) >
			sizeof(struct cyttsp_xydata)) {
			grpsize = sizeof(struct cyttsp_xydata) -
				CY_REG_OP_START;
			goto cyttsp_ic_grpdata_show_error_exit;
		}
		num_read = sizeof(struct cyttsp_xydata) - (ts->ic_grpoffset +
			CY_REG_OP_START);
		uretval = _cyttsp_get_show_data(ts, ts->ic_grpoffset +
			CY_REG_OP_START, num_read, ic_buf, buf);
		if (uretval > 0)
			goto cyttsp_ic_grpdata_show_exit;
		break;
	case CY_IC_GRPNUM_SI_TAG:
		if ((ts->ic_grpoffset + CY_REG_SI_START) >
			sizeof(struct cyttsp_sysinfo_data)) {
			grpsize = sizeof(struct cyttsp_sysinfo_data) -
				CY_REG_SI_START;
			goto cyttsp_ic_grpdata_show_error_exit;
		}
		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot access SI Group %d Data.\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot access SI Group %d Data.\n",
					ts->ic_grpnum);
			goto cyttsp_ic_grpdata_show_exit;
		}

		num_read = sizeof(struct cyttsp_sysinfo_data) -
			(ts->ic_grpoffset + CY_REG_SI_START);
		uretval = _cyttsp_get_show_data(ts, ts->ic_grpoffset +
			CY_REG_SI_START, num_read, ic_buf, buf);

		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore op mode\n",
				__func__);
		} else {
			_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		}
		if (uretval > 0)
			goto cyttsp_ic_grpdata_show_exit;
		break;
	case CY_IC_GRPNUM_BL_KEY:
		if (ts->platform_data->sett[ts->ic_grpnum] == NULL) {
			pr_err("%s: Missing table for Group %d\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Missing table for Group %d\n",
				ts->ic_grpnum);
			goto cyttsp_ic_grpdata_show_exit;
		}
		if (ts->platform_data->sett[ts->ic_grpnum]->data == NULL) {
			pr_err("%s: Missing table data for Group %d\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Missing table for Group %d\n",
				ts->ic_grpnum);
			goto cyttsp_ic_grpdata_show_exit;
		}
		if (ts->ic_grpoffset >=
			ts->platform_data->sett[ts->ic_grpnum]->size) {
			grpsize = ts->platform_data->sett[ts->ic_grpnum]->size;
			goto cyttsp_ic_grpdata_show_error_exit;
		}
		num_read = ts->platform_data->sett[ts->ic_grpnum]->size -
			ts->ic_grpoffset;

		memcpy(ic_buf, &ts->platform_data->sett
			[ts->ic_grpnum]->data[ts->ic_grpoffset],
			num_read);
		break;
	case CY_IC_GRPNUM_OP_REG:
		grpsize = sizeof(struct cyttsp_xydata);
		if (ts->ic_grpoffset >= grpsize)
			goto cyttsp_ic_grpdata_show_error_exit;
		num_read = grpsize - ts->ic_grpoffset;
		uretval = _cyttsp_get_show_data(ts, ts->ic_grpoffset,
			num_read, ic_buf, buf);
		if (uretval > 0)
			goto cyttsp_ic_grpdata_show_exit;
		break;
	case CY_IC_GRPNUM_SI_REG:
		grpsize = sizeof(struct cyttsp_sysinfo_data);
		if (ts->ic_grpoffset >= grpsize)
			goto cyttsp_ic_grpdata_show_error_exit;
		num_read = grpsize - ts->ic_grpoffset;
		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot access SI Group %d Data.\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot access SI Group %d Data.\n",
					ts->ic_grpnum);
			goto cyttsp_ic_grpdata_show_exit;
		}
		uretval = _cyttsp_get_show_data(ts, ts->ic_grpoffset,
			num_read, ic_buf, buf);

		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore op mode\n",
				__func__);
		} else {
			_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		}

		if (uretval > 0)
			goto cyttsp_ic_grpdata_show_exit;
		break;
	case CY_IC_GRPNUM_BL_REG:
		grpsize = sizeof(struct cyttsp_bootloader_data);
		if (ts->ic_grpoffset >= grpsize)
			goto cyttsp_ic_grpdata_show_error_exit;
		num_read = grpsize - ts->ic_grpoffset;
		retval = _cyttsp_hard_reset(ts);
		if (retval < 0) {
			pr_err("%s: Cannot access BL Group %d Data.\n",
				__func__, ts->ic_grpnum);
			uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot access BL Group %d Data.\n",
					ts->ic_grpnum);
			goto cyttsp_ic_grpdata_show_exit;
		}
		uretval = _cyttsp_get_show_data(ts, ts->ic_grpoffset,
			num_read, ic_buf, buf);

		/* startup will restore to CY_ACTIVE_MODE if successful */
		retval = _cyttsp_startup(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore op mode\n",
				__func__);
		}

		if (uretval > 0)
			goto cyttsp_ic_grpdata_show_exit;
		break;
	default:
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d does not exist.\n",
			ts->ic_grpnum);
		goto cyttsp_ic_grpdata_show_exit;
	}

	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d, Offset %u:\n", ts->ic_grpnum, ts->ic_grpoffset);
	for (i = 0; i < num_read; i++)
		snprintf(buf, CY_MAX_PRBUF_SIZE, "%s0x%02X\n", buf, ic_buf[i]);
	uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s(%d bytes)\n", buf, num_read);

	goto cyttsp_ic_grpdata_show_exit;

cyttsp_ic_grpdata_show_error_exit:
	pr_err("%s: Offset %u exceeds group %d size of %d\n",
		__func__, ts->ic_grpoffset, ts->ic_grpnum, grpsize);
	uretval = snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Offset %u exceeds group %d size of %d\n",
		ts->ic_grpoffset, ts->ic_grpnum, grpsize);

cyttsp_ic_grpdata_show_exit:
	mutex_unlock(&ts->data_lock);
	return uretval;
}

static ssize_t cyttsp_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;
	const char *pbuf = buf;
	int scan_read = 0;
	int num_write = 0;
	int num_commas = 0;
	char scan_buf[5];	/* sizeof "0xHH" plus null byte */
	ssize_t grpsize = 0;
	u8 *ic_buf = NULL;

	mutex_lock(&ts->data_lock);

	memset(scan_buf, 0, sizeof(scan_buf));
	ic_buf = kzalloc(sizeof(struct cyttsp_xydata), GFP_KERNEL);
	if (ic_buf == NULL) {
		pr_err("%s: Error, kzalloc scan buffer memory\n", __func__);
		goto cyttsp_ic_grpdata_store_exit;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: pbuf=%p buf=%p size=%ld sizeof(scan_buf)=%ld\n",
		__func__, pbuf, buf, size, sizeof(scan_buf));

	while (pbuf <= ((buf + size) - (sizeof(scan_buf) - 1))) {
		num_commas = 0;
		while (((*pbuf == ' ') || (*pbuf == ',')) &&
			(pbuf < ((buf + size) - 4))) {
			if (*pbuf == ',')
				num_commas++;
			if (num_commas > 1) {
				pr_err("%s: Invalid data format. "
					"\",,\" not allowed.\n",
					__func__);
				retval = size;
				goto cyttsp_ic_grpdata_store_exit;
			}
			pbuf++;
		}

		if (num_write > sizeof(struct cyttsp_xydata)) {
			pr_err("%s: Num store data > max=%ld\n", __func__,
				sizeof(sizeof(struct cyttsp_xydata)));
			goto cyttsp_ic_grpdata_store_exit;
		}

		if (pbuf <= ((buf + size) - (sizeof(scan_buf) - 1))) {
			memset(scan_buf, 0, sizeof(scan_buf));
			for (scan_read = 0; scan_read < (sizeof(scan_buf) - 1);
				scan_read++)
				scan_buf[scan_read] = *pbuf++;
			retval = kstrtoul(scan_buf, 16, &value);
			if (retval < 0) {
				pr_err("%s: Invalid data format. "
					"Use \"0xHH,...,0xHH\" instead.\n",
					__func__);
				goto cyttsp_ic_grpdata_store_exit;
			}
			if (value > 0xFF) {
				/*pr_err("%s: invalid value=0x%08X; must be in"
					" the range 0x00 - 0xFF\n",
					__func__, (int)value);*/
				pr_err("%s: invalid value; must be in"
					" the range 0x00 - 0xFF\n",
					__func__);
				goto cyttsp_ic_grpdata_store_exit;
			}
			ic_buf[num_write] = value;
			cyttsp_dbg(ts, CY_DBG_LVL_3,
				"%s: ic_buf[%d] = 0x%02X\n",
				__func__, num_write, ic_buf[num_write]);
			num_write++;
		} else {
			pr_err("%s: Incomplete final value\n", __func__);
			goto cyttsp_ic_grpdata_store_exit;
		}
	}

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: After scan: grpnum=%d grpoffset=%d num_write=%d"
		" ic_buf[0]=%02X ic_buf[1]=%02X\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset,
		num_write, ic_buf[0], ic_buf[1]);
	if (num_write == 0) {
		pr_err("%s: No data scanned; num_write=%d grpnum=%d\n",
			__func__, num_write, ts->ic_grpnum);
		goto cyttsp_ic_grpdata_store_exit;
	}

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_OP_TAG:
		if ((ts->ic_grpoffset + CY_REG_OP_START) >
			sizeof(struct cyttsp_xydata)) {
			grpsize = sizeof(struct cyttsp_xydata) -
				CY_REG_OP_START;
			goto cyttsp_ic_grpdata_store_size_error_exit;
		}
		if (num_write > (sizeof(struct cyttsp_xydata) -
			(CY_REG_OP_START + ts->ic_grpoffset))) {
			pr_err("%s Too much data for op registers\n",
				__func__);
			goto cyttsp_ic_grpdata_store_exit;
		}
		retval = _cyttsp_put_store_data(ts, ts->ic_grpnum,
			ts->ic_grpoffset + CY_REG_OP_START, num_write, ic_buf);
		if (retval < 0) {
			pr_err("%s: fail put store data grpnum=%d r=%d\n",
				__func__, ts->ic_grpnum, retval);
		}
		break;
	case CY_IC_GRPNUM_SI_TAG:
		if ((ts->ic_grpoffset + CY_REG_SI_START) >
			sizeof(struct cyttsp_sysinfo_data)) {
			grpsize = sizeof(struct cyttsp_sysinfo_data) -
				CY_REG_SI_START;
			goto cyttsp_ic_grpdata_store_size_error_exit;
		}
		if (num_write > (sizeof(struct cyttsp_sysinfo_data) -
			(CY_REG_SI_START + ts->ic_grpoffset))) {
			pr_err("%s Too much data for sysinfo registers\n",
				__func__);
			goto cyttsp_ic_grpdata_store_exit;
		}
		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot write SI Group Data.\n",
			__func__);
			goto cyttsp_ic_grpdata_store_exit;
		}

		retval = _cyttsp_put_si_data(ts, ts->ic_grpnum,
			ts->ic_grpoffset + CY_REG_SI_START, num_write, ic_buf);
		if (retval < 0) {
			pr_err("%s: Error writing Sysinfo data r=%d\n",
				__func__, retval);
			/* continue anyway */
			retval = 0;
		}

		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore operating mode.\n",
				__func__);
		} else {
			_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		}
		break;
	case CY_IC_GRPNUM_BL_KEY:
		pr_err("%s: Group=%d is read only\n",
			__func__, ts->ic_grpnum);
		break;
	case CY_IC_GRPNUM_OP_REG:
		grpsize = sizeof(struct cyttsp_xydata);
		if ((ts->ic_grpoffset + num_write) > grpsize)
			goto cyttsp_ic_grpdata_store_size_error_exit;

		retval = _cyttsp_put_store_data(ts, ts->ic_grpnum,
			ts->ic_grpoffset, num_write, ic_buf);
		if (retval < 0) {
			pr_err("%s: fail put store data grpnum=%d r=%d\n",
				__func__, ts->ic_grpnum, retval);
		} else {
		}
		break;
	case CY_IC_GRPNUM_SI_REG:
		grpsize = sizeof(struct cyttsp_sysinfo_data);
		if ((ts->ic_grpoffset + num_write) > grpsize)
			goto cyttsp_ic_grpdata_store_size_error_exit;

		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot write SI Group Data.\n", __func__);
			goto cyttsp_ic_grpdata_store_exit;
		}

		retval = _cyttsp_put_si_data(ts, ts->ic_grpnum,
			ts->ic_grpoffset + CY_REG_SI_START, num_write, ic_buf);
		if (retval < 0) {
			pr_err("%s: Error writing Sysinfo data r=%d\n",
				__func__, retval);
			/* continue anyway */
			retval = 0;
		}

		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore operating mode.\n",
				__func__);
		} else {
			_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		}
		break;
	case CY_IC_GRPNUM_BL_REG:
		pr_err("%s: Group=%d is read only\n",
			__func__, ts->ic_grpnum);
		break;
	default:
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		break;
	}

	goto cyttsp_ic_grpdata_store_exit;

cyttsp_ic_grpdata_store_size_error_exit:
	pr_err("%s: Offset %u exceeds group %d size of %ld\n",
		__func__, ts->ic_grpoffset, ts->ic_grpnum, grpsize);
cyttsp_ic_grpdata_store_exit:
	if (ic_buf != NULL)
		kfree(ic_buf);
	mutex_unlock(&ts->data_lock);
	retval = size;
	return retval;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpdata_show, cyttsp_ic_grpdata_store);

static ssize_t cyttsp_drv_flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Driver Flags: 0x%04X\n", ts->flags);
}
static ssize_t cyttsp_drv_flags_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = kstrtoul(buf, 16, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_drv_flags_store_error_exit;
	}

	if (value > 0xFFFF) {
		pr_err("%s: value=%lu is greater than max;"
			" drv_flags=0x%04X\n", __func__, value, ts->flags);
	} else {
		ts->flags = value;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: drv_flags=0x%04X\n", __func__, ts->flags);

cyttsp_drv_flags_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(drv_flags, S_IRUSR | S_IWUSR,
	cyttsp_drv_flags_show, cyttsp_drv_flags_store);

static ssize_t cyttsp_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = _cyttsp_startup(ts);
	mutex_unlock(&(ts->data_lock));
	if (retval < 0) {
		pr_err("%s: fail hw_reset device restart r=%ld\n",
			__func__, retval);
	}

	retval = size;
	return retval;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, cyttsp_hw_reset_store);

static ssize_t cyttsp_hw_recov_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_hw_recov_store_error_exit;
	}

	if (ts->platform_data->hw_recov == NULL) {
		pr_err("%s: no hw_recov function\n", __func__);
		goto cyttsp_hw_recov_store_error_exit;
	}

	retval = ts->platform_data->hw_recov((int)value, ts->platform_data);
	if (retval < 0) {
		pr_err("%s: fail hw_recov(value=%d) function r=%ld\n",
			__func__, (int)value, retval);
	}

cyttsp_hw_recov_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(hw_recov, S_IWUSR, NULL, cyttsp_hw_recov_store);
#endif

static ssize_t cyttsp_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Application Version: 0x%02X%02X\n"
		"TTSP Version:        0x%02X%02X\n"
		"Applicaton ID:       0x%02X%02X\n"
		"Custom ID:           0x%02X%02X%02X\n"
		"FT_ready_status:%d   load_config_result:%d\n",
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1], ts->sysinfo_data.cid[2],
		_cyttsp3_force_function.FT_ready_flag, _cyttsp3_force_function.load_config_result);
}
static DEVICE_ATTR(ic_ver, S_IRUGO, cyttsp_ic_ver_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static ssize_t cyttsp_hw_irqstat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval = 0;
	struct cyttsp *ts = dev_get_drvdata(dev);

	if (ts->platform_data->irq_stat != NULL) {
		retval = ts->platform_data->irq_stat(ts->platform_data);
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	} else {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Function irq_stat() undefined.\n");
	}
}
static DEVICE_ATTR(hw_irqstat, S_IRUSR ,
	cyttsp_hw_irqstat_show, NULL);
#endif

/* Disable Driver IRQ */
static ssize_t cyttsp_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver IRQ is %s\n", ts->irq_enabled ? "ENABLED" : "DISABLED");
}
static ssize_t cyttsp_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval = 0;
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(&ts->data_lock);

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0)
		pr_err("%s: Failed to convert value\n", __func__);
	else {
		switch (value) {
		case 0:
			if (ts->irq_enabled == false) {
				pr_info("%s: Driver IRQ already DISABLED\n",
					__func__);
			} else {
				/* Disable IRQ */
				disable_irq(ts->irq);//ztemt disable_irq_nosync(ts->irq);
				pr_info("%s: Driver IRQ now DISABLED\n",
					__func__);
				ts->irq_enabled = false;
			}
			break;
		case 1:
			if (ts->irq_enabled == true) {
				pr_info("%s: Driver IRQ already ENABLED\n",
					__func__);
			} else {
				/* Enable IRQ */
				enable_irq(ts->irq);
				pr_info("%s: Driver IRQ now ENABLED\n",
					__func__);
				ts->irq_enabled = true;
			}
			break;
		default:
			pr_err("%s: value=%lu is out of range; enter"
				" either 0 (disable) or 1 (enable)\n",
				__func__, value);
		}
	}

	mutex_unlock(&ts->data_lock);
	retval = size;
	return retval;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	cyttsp_drv_irq_show, cyttsp_drv_irq_store);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Force firmware upgrade */
static int _cyttsp_firmware_class_loader(struct cyttsp *ts,
	const u8 *data, int size)
{
	int retval = 0;
	const u8 *img = NULL;
	int img_size = 0;

	if ((data == NULL) || (size == 0)) {
		pr_err("%s: empty firmware image data=%p size=%d\n",
			__func__, data, size);
		_cyttsp_change_state(ts, CY_IDLE_STATE);
		goto _cyttsp_firmware_class_loader_exit;
	} else {
		img = data + data[0] + 1;
		img_size = size - (data[0] + 1);

		retval = _cyttsp_hard_reset(ts);
		if (retval < 0) {
			pr_err("%s: Fail reset device on loader\n", __func__);
			goto _cyttsp_firmware_class_loader_exit;
		}

		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Fail read bootloader\n", __func__);
			goto _cyttsp_firmware_class_loader_exit;
		}

		cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: call load_app\n", __func__);
		retval = _cyttsp_load_app(ts, img, img_size);
		if (retval < 0)
			pr_err("%s: fail on load fw r=%d\n", __func__, retval);
	}

_cyttsp_firmware_class_loader_exit:
	return retval;
}
static void cyttsp_firmware_cont(const struct firmware *fw, void *context)
{
	int retval = 0;
	struct device *dev = context;
	struct cyttsp *ts = dev_get_drvdata(dev);

	if (fw == NULL) {
		pr_err("%s: firmware not found\n", __func__);
		goto cyttsp_firmware_cont_exit;
	}

	mutex_lock(&ts->data_lock);

	retval = _cyttsp_firmware_class_loader(ts, fw->data, fw->size);
	if (retval < 0) {
		/* mark an error and then try restarting anyway */
		pr_err("%s: Fail firmware class loader r=%d\n",
			__func__, retval);
	} else {
		/* prevent warm startup from overwriting the force load */
		ts->ic_reflash_done = true;
	}

	retval = _cyttsp_startup(ts);
	if (retval < 0) {
		pr_info("%s: Fail _cyttsp_startup after fw load r=%d\n",
			__func__, retval);
	}

	mutex_unlock(&ts->data_lock);
	release_firmware(fw);

cyttsp_firmware_cont_exit:
	ts->waiting_for_fw = false;
	return;
}
static ssize_t cyttsp_ic_reflash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s\n", ts->waiting_for_fw ?
		"Driver is waiting for firmware load" :
		"No firmware loading in progress");
}
static ssize_t cyttsp_ic_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int i = 0;
	int retval = 0;
	struct cyttsp *ts = dev_get_drvdata(dev);
	pr_err("%s: ztemt start\n",__func__);

	mutex_lock(&ts->data_lock);
	if (ts->waiting_for_fw) {
		pr_err("%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto cyttsp_ic_reflash_store_exit;
	}

	/*
	 * must configure FW_LOADER in .config file
	 * CONFIG_HOTPLUG=y
	 * CONFIG_FW_LOADER=y
	 * CONFIG_FIRMWARE_IN_KERNEL=y
	 * CONFIG_EXTRA_FIRMWARE=""
	 * CONFIG_EXTRA_FIRMWARE_DIR=""
	 */

	if (size > CY_BL_FW_NAME_SIZE) {
		pr_err("%s: Filename too long\n", __func__);
		retval = -ENAMETOOLONG;
		goto cyttsp_ic_reflash_store_exit;
	} else {
		/*
		 * name string must be in alloc() memory
		 * or is lost on context switch
		 * strip off any line feed character(s)
		 * at the end of the buf string
		 */
		for (i = 0; buf[i]; i++) {
			if (buf[i] < ' ')
				ts->fwname[i] = 0;
			else
				ts->fwname[i] = buf[i];
		}
	}

	pr_info("%s: Enabling firmware class loader\n", __func__);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		GFP_KERNEL, ts->dev, cyttsp_firmware_cont);
#else /* Kernel 2.6.32 */
	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		ts->dev, cyttsp_firmware_cont);
#endif
	if (retval < 0) {
		pr_err("%s: Fail request firmware class file load\n",
			__func__);
		ts->waiting_for_fw = false;
		goto cyttsp_ic_reflash_store_exit;
	} else {
		ts->waiting_for_fw = true;
		retval = size;
	}

cyttsp_ic_reflash_store_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(ic_reflash, S_IRWXU|S_IRWXG|S_IRWXO, cyttsp_ic_reflash_show, cyttsp_ic_reflash_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Driver debugging */
static ssize_t cyttsp_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u\n", ts->bus_ops->tsdebug);
}
static ssize_t cyttsp_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_LVL_0:
	case CY_DBG_LVL_1:
	case CY_DBG_LVL_2:
	case CY_DBG_LVL_3:
		pr_info("%s: Debug setting=%d\n", __func__, (int)value);
		ts->bus_ops->tsdebug = value;
		break;
#ifdef CY_USE_DEBUG_TOOLS
	case CY_DBG_SUSPEND:
		retval = cyttsp_suspend(ts);
		pr_info("%s: SUSPEND (ts=%p) r=%d\n", __func__, ts, retval);
		break;
	case CY_DBG_RESUME:
		retval = cyttsp_resume(ts);
		pr_info("%s: RESUME (ts=%p) r=%d\n", __func__, ts, retval);
		break;
	case CY_DBG_RESTART:
		pr_info("%s: RESTART (ts=%p)\n",
			__func__, ts);
		_cyttsp_queue_startup(ts, false);
		break;
#endif
	default:
		ts->bus_ops->tsdebug = CY_DBG_LVL_MAX;
		pr_err("%s: Invalid debug setting; set to max=%d\n",
			__func__, ts->bus_ops->tsdebug);
		break;
	}

	retval = size;

cyttsp_drv_debug_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_debug, S_IRUSR | S_IWUSR,
	cyttsp_drv_debug_show, cyttsp_drv_debug_store);
#endif

#ifdef CY_USE_REG_ACCESS
#define CY_RW_REGID_MAX             0x1F
#define CY_RW_REG_DATA_MAX          0xFF
static ssize_t cyttsp_drv_rw_regid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Read/Write Regid=%02X(%d)\n",
		ts->rw_regid, ts->rw_regid);
}
static ssize_t cyttsp_drv_rw_regid_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value = 0;

	mutex_lock(&ts->data_lock);
	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		retval = kstrtoul(buf, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_regid_store_exit;
		}
	}

	if (value > CY_RW_REGID_MAX) {
		ts->rw_regid = CY_RW_REGID_MAX;
		pr_err("%s: Invalid Read/Write Regid; set to max=%d\n",
			__func__, ts->rw_regid);
	} else
		ts->rw_regid = value;

	retval = size;

cyttsp_drv_rw_regid_store_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(drv_rw_regid, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
	cyttsp_drv_rw_regid_show, cyttsp_drv_rw_regid_store);


static ssize_t cyttsp_drv_rw_reg_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	u8 reg_data = 0;

	retval = ttsp_read_block_data(ts,
		CY_REG_BASE + ts->rw_regid, sizeof(reg_data), &reg_data);

	if (retval < 0)
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid(%02X(%d) Failed\n",
			ts->rw_regid, ts->rw_regid);
	else
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid=%02X(%d) Data=%02X(%d)\n",
			ts->rw_regid, ts->rw_regid, reg_data, reg_data);
}
static ssize_t cyttsp_drv_rw_reg_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value = 0;
	u8 regdata = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		retval = kstrtoul(buf, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_reg_data_store_exit;
		}
	}

	if (value > CY_RW_REG_DATA_MAX) {
		pr_err("%s: Invalid Register Data Range; no write\n",
			__func__);
	} else {
		regdata = (u8)value;
		if (ts->rw_regid == CY_REG_BASE && (GET_HSTMODE(regdata) !=
		GET_HSTMODE(CY_COMPOSITE_MODE)) &&
			!(regdata & (CY_SOFT_RESET_MODE |
			CY_DEEP_SLEEP_MODE))) {
			mutex_lock(&ts->data_lock);
			if (GET_HSTMODE(regdata) ==
				GET_HSTMODE(CY_OPERATE_MODE)) {
				retval = _cyttsp_set_operational_mode(ts);
				if (retval < 0) {
					pr_err("%s: Fail set operating mode.\n",
						__func__);
				} else {
					_cyttsp_change_state(ts,
						CY_ACTIVE_STATE);
				}
			} else if (GET_HSTMODE(regdata) ==
				GET_HSTMODE(CY_SYSINFO_MODE))  {
				retval = _cyttsp_set_sysinfo_mode(ts);
				if (retval < 0) {
					pr_err("%s: Fail set sysinfo mode.\n",
						__func__);
				}
			}
			mutex_unlock(&ts->data_lock);
		} else {
			retval = ttsp_write_block_data(ts, ts->rw_regid,
				sizeof(regdata), &regdata);
			if (retval < 0) {
				pr_err("%s: Failed write to Regid=%02X(%d)"
					" Data=%02X(%d)\n", __func__,
					ts->rw_regid, ts->rw_regid,
					regdata, regdata);
			}
		}
	}

	retval = size;

cyttsp_drv_rw_reg_data_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_rw_reg_data, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
	cyttsp_drv_rw_reg_data_show, cyttsp_drv_rw_reg_data_store);
#endif

static void cyttsp_ldr_init(struct cyttsp *ts)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_drv_debug))
		pr_err("%s: Cannot create drv_debug\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_flags))
		pr_err("%s: Cannot create drv_flags\n", __func__);
#endif


	if (device_create_file(ts->dev, &dev_attr_drv_irq))
		pr_err("%s: Cannot create drv_irq\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_stat))
		pr_err("%s: Cannot create drv_stat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_ver))
		pr_err("%s: Cannot create drv_ver\n", __func__);

#ifdef ZTE_FACTORY_WRITE_BIN_BACK
	if (device_create_file(ts->dev, &dev_attr_paramarray))
		pr_err("%s: Cannot create drv_ver\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_hw_irqstat))
		pr_err("%s: Cannot create hw_irqstat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_reset))
		pr_err("%s: Cannot create hw_reset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_recov))
		pr_err("%s: Cannot create hw_recov\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpdata))
		pr_err("%s: Cannot create ic_grpdata\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpnum))
		pr_err("%s: Cannot create ic_grpnum\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpoffset))
		pr_err("%s: Cannot create ic_grpoffset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_reflash))
		pr_err("%s: Cannot create ic_reflash\n", __func__);
#endif

	if (device_create_file(ts->dev, &dev_attr_ic_ver))
		pr_err("%s: Cannot create ic_ver\n", __func__);

#ifdef CY_USE_REG_ACCESS
	if (device_create_file(ts->dev, &dev_attr_drv_rw_regid))
		pr_err("%s: Cannot create drv_rw_regid\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_rw_reg_data))
		pr_err("%s: Cannot create drv_rw_reg_data\n", __func__);
#endif

	return;
}

static void cyttsp_ldr_free(struct cyttsp *ts)
{
	device_remove_file(ts->dev, &dev_attr_drv_irq);
	device_remove_file(ts->dev, &dev_attr_drv_ver);
	device_remove_file(ts->dev, &dev_attr_drv_stat);
	device_remove_file(ts->dev, &dev_attr_ic_ver);
#ifdef ZTE_FACTORY_WRITE_BIN_BACK
	device_remove_file(ts->dev, &dev_attr_paramarray);
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(ts->dev, &dev_attr_drv_debug);
	device_remove_file(ts->dev, &dev_attr_drv_flags);
	device_remove_file(ts->dev, &dev_attr_hw_irqstat);
	device_remove_file(ts->dev, &dev_attr_hw_reset);
	device_remove_file(ts->dev, &dev_attr_hw_recov);
	device_remove_file(ts->dev, &dev_attr_ic_grpdata);
	device_remove_file(ts->dev, &dev_attr_ic_grpnum);
	device_remove_file(ts->dev, &dev_attr_ic_grpoffset);
	device_remove_file(ts->dev, &dev_attr_ic_reflash);
#endif
#ifdef CY_USE_REG_ACCESS
	device_remove_file(ts->dev, &dev_attr_drv_rw_regid);
	device_remove_file(ts->dev, &dev_attr_drv_rw_reg_data);
#endif
}

static int _cyttsp_resume_sleep(struct cyttsp *ts)
{
	int retval = 0;
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)/*** ZTEMT add ***/
	u8 sleep = CY_DEEP_SLEEP_MODE;

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: Put the part back to sleep\n", __func__);

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(sleep), &sleep);
	if (retval < 0) {
		pr_err("%s: Failed to write sleep bit r=%d\n",
			__func__, retval);
	} else {
	}
#endif
	return retval;
}

static int _cyttsp_startup(struct cyttsp *ts)
{
	int retval = 0;

#ifdef CY_NO_HW
	pr_info("%s: Do not run startup\n", __func__);
	return retval;
#endif
    pr_err("%s ztemt\n", __func__);

	retval = _cyttsp_hard_reset(ts);
	if (retval < 0) {
		pr_err("%s: Fail on HW reset r=%d\n", __func__, retval);
		_cyttsp_change_state(ts, CY_INVALID_STATE);
		goto _cyttsp_startup_exit;
	}


	retval = _cyttsp_load_bl_regs(ts);
	if (retval < 0) {
		pr_err("%s: Fail on bl regs read r=%d\n", __func__, retval);
		_cyttsp_change_state(ts, CY_IDLE_STATE);
		goto _cyttsp_startup_exit;
	}

	retval = _cyttsp_boot_loader(ts);
	if (retval < 0) {
		pr_err("%s: Fail on boot loader r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;  /* We are now in IDLE state */
	}

	retval = _cyttsp_exit_bl_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail on exit bootloader r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;
	}


	retval = _cyttsp_set_sysinfo_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail on set sysinfo mode r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;
	}

	retval = _cyttsp_set_sysinfo_regs(ts);
	if (retval < 0) {
		pr_err("%s: Fail on set sysinfo regs r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;
	}

	retval = _cyttsp_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail on set op mode r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;
	}

	retval = _cyttsp_set_operational_regs(ts);
	if (retval < 0) {
		pr_err("%s: Fail on set op regs r=%d\n", __func__, retval);
		goto _cyttsp_startup_exit;
	}

	/* successful startup; switch to active state */
//	pr_info("%s: hm=%02X tv=%02X%02X ai=0x%02X%02X "
	pr_err("%s: hm=%02X tv=%02X%02X ai=0x%02X%02X "
		"av=0x%02X%02X ci=0x%02X%02X%02X\n", __func__,
		ts->sysinfo_data.hst_mode,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);

	_cyttsp_change_state(ts, CY_ACTIVE_STATE);

_cyttsp_startup_exit:
	if (ts->was_suspended) {
pr_err("%s: was suspended\n", __func__);
		ts->was_suspended = false;
		retval = _cyttsp_resume_sleep(ts);
		if (retval < 0) {
			pr_err("%s: Fail resume sleep, IC is active r=%d\n",
				__func__, retval);
		}
	}
	return retval;
}

static void _cyttsp_queue_startup(struct cyttsp *ts, bool was_suspended)
{

	_cyttsp_change_state(ts, CY_BL_STATE);


	ts->was_suspended = was_suspended;
	queue_work(ts->cyttsp_wq, &ts->cyttsp_resume_startup_work);
	pr_info("%s: startup queued\n", __func__);
}

static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = handle;
#ifdef CY_USE_LEVEL_IRQ
	int tries = 0;
#endif
	int retval = 0;
    pr_err("%s: ztemt cyttsp_irq = %p\n", __func__, ts);

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: GOT IRQ ps=%d\n", __func__, ts->power_state);

	mutex_lock(&ts->data_lock);

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: DO IRQ ps=%d\n", __func__, ts->power_state);

	switch (ts->power_state) {
	case CY_BL_STATE:
		complete(&ts->bl_int_running);
#ifdef CY_USE_LEVEL_IRQ
		udelay(2000);	/* irq pulse: heartbeat=1ms; exit_bl=500us */
#endif
		break;
	case CY_SYSINFO_STATE:
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(struct cyttsp_sysinfo_data),
			&(ts->sysinfo_data));
		if (retval < 0) {
			pr_err("%s: Fail read status and version regs r=%d\n",
				__func__, retval);
			goto _cyttsp_irq_sysinfo_exit;
		}
		retval = _cyttsp_hndshk(ts, ts->sysinfo_data.hst_mode);
		if (retval < 0) {
			pr_err("%s: Fail write handshake r=%d\n",
				__func__, retval);
			/* continue anyway */
			retval = 0;
		}
#ifdef CY_USE_LEVEL_IRQ
		udelay(100);	/* irq pulse: sysinfo mode switch=50us */
#endif
		complete(&ts->si_int_running);
_cyttsp_irq_sysinfo_exit:
		break;
	case CY_READY_STATE:
		complete(&ts->op_int_running);
#ifdef CY_USE_LEVEL_IRQ
		udelay(100);	/* irq pulse: operational mode switch=50us */
#endif
		break;
	case CY_LDR_STATE:
		complete(&ts->ld_int_running);
#ifdef CY_USE_LEVEL_IRQ
		udelay(2000);	/* irq pulse: bl_ready=1ms */
#endif
		break;
	case CY_SLEEP_STATE:
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: cyttsp Attempt to process touch after enter sleep or"
			" unexpected wake event\n", __func__);
		retval = _cyttsp_wakeup(ts); /* in case its really asleep */
		if (retval < 0) {
			pr_err("%s: wakeup fail r=%d\n",
				__func__, retval);
			_cyttsp_queue_startup(ts, true);
			break;
		}
		retval = _cyttsp_resume_sleep(ts);
		if (retval < 0) {
			pr_err("%s: resume suspend fail r=%d\n",
				__func__, retval);
			_cyttsp_queue_startup(ts, true);
			break;
		}
		break;
	case CY_IDLE_STATE:
		/* device now available; signal initialization */
		pr_info("%s: Received IRQ in IDLE state\n",
			__func__);
		/* Try to determine the IC's current state */
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Still unable to find IC after IRQ\n",
				__func__);
		} else if (IS_BOOTLOADERMODE(ts->bl_data.bl_status)) {
			pr_info("%s: BL mode found in IDLE state\n",
				__func__);
			/* change state to make sure any other heartbeats
			 * simply signal complete until the restart has
			 * been entered from the* startup queue
			 */
			_cyttsp_change_state(ts, CY_BL_STATE);
			_cyttsp_queue_startup(ts, false);
		} else {
			pr_err("%s: interrupt received in IDLE state -"
				" try processing touch\n",
				__func__);
			_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		}
		break;
	case CY_ACTIVE_STATE:
		break;
	case CY_INVALID_STATE:
		break;
	default:
		pr_err("%s: Unexpected interrupt ps=%d\n",
			__func__, ts->power_state);
		break;
	}

	mutex_unlock(&ts->data_lock);
	return IRQ_HANDLED;
}

static void cyttsp_ts_work_func(struct work_struct *work)
{
	struct cyttsp *ts =
		container_of(work, struct cyttsp, cyttsp_resume_startup_work);
	int retval = 0;

	mutex_lock(&ts->data_lock);

	retval = _cyttsp_startup(ts);
	if (retval < 0) {
		pr_err("%s: Startup failed with error code %d\n",
			__func__, retval);
		_cyttsp_change_state(ts, CY_INVALID_STATE);
	}

	mutex_unlock(&ts->data_lock);

	return;
}

static int _cyttsp_wakeup(struct cyttsp *ts)
{
	int retval = 0;
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)/*** ZTEMT add ***/
	struct cyttsp_status_regs status_regs;
	int wake = CY_WAKE_DFLT;

	memset(&status_regs, 0, sizeof(struct cyttsp_status_regs));

	if (ts->platform_data->hw_recov == NULL) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: no hw_recov function\n", __func__);
	} else {
		/* try wake using strobe on host alert pin */
		retval = ts->platform_data->hw_recov(wake, ts->platform_data);
		if (retval < 0) {
			if (retval == -ENOSYS) {
				cyttsp_dbg(ts, CY_DBG_LVL_3,
					"%s: no hw_recov wake=%d"
					" function\n", __func__, wake);
			} else {
				pr_err("%s: fail hw_recov(wake=%d)"
					" function r=%d\n",
					__func__, wake, retval);
				retval = -ENOSYS;
			}
		}
	}

	/* either device is already awake or a read will wake it up */
	retval = _cyttsp_load_status_regs(ts, &status_regs);
	if (retval < 0) {
		pr_err("%s: failed to access device on resume r=%d\n",
			__func__, retval);
		goto _cyttsp_wakeup_error_exit;
	}

	if (IS_BOOTLOADERMODE(status_regs.tt_mode)) {
		pr_err("%s: device in bootloader mode on wakeup"
			" tt_mode=0x%02X\n", __func__, status_regs.tt_mode);
		retval = -EIO;
		goto _cyttsp_wakeup_error_exit;
	}

	if (GET_HSTMODE(status_regs.hst_mode) !=
		GET_HSTMODE(CY_OPERATE_MODE)) {
		pr_err("%s: device not in op mode on wakeup hst_mode=0x%02X\n",
			__func__, status_regs.hst_mode);
		retval = -EIO;
		goto _cyttsp_wakeup_error_exit;
	}

	retval = _cyttsp_hndshk(ts, status_regs.hst_mode);
	if (retval < 0) {
		pr_err("%s: fail resume INT handshake(r=%d)\n",
			__func__, retval);
		/* continue anyway */
		retval = 0;
	}

	goto _cyttsp_wakeup_exit;

_cyttsp_wakeup_error_exit:
	/* caller must initiate startup sequence to restore touch */
	_cyttsp_change_state(ts, CY_IDLE_STATE);
	pr_err("%s: device reset required\n", __func__);
_cyttsp_wakeup_exit:
#endif
	return retval;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)/*** ZTEMT add ***/
int cyttsp_resume(void *handle)
{
	struct cyttsp *ts = handle;
	int retval = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Resuming...", __func__);

	mutex_lock(&ts->data_lock);
	pr_info("%s: ztemt cyttsp_resume power_state=%d \n",__func__,ts->power_state);

	switch (ts->power_state) {
	case CY_SLEEP_STATE:
		if (ts->irq_enabled) {
#if 1 //def CY_USE_LEVEL_IRQ //enable it for no irq when get gpio derection to input
			/* Workaround level interrupt unmasking issue */
			disable_irq(ts->irq);//ztemt disable_irq_nosync(ts->irq);
			udelay(5);
#endif
			enable_irq(ts->irq);
		}

		retval = _cyttsp_wakeup(ts);
		if (retval < 0) {
			pr_err("%s: wakeup fail r=%d\n",
				__func__, retval);
			_cyttsp_queue_startup(ts, false);
			break;
		}
		_cyttsp3_force_function.FT_ready_flag = true;//zte add 20160307
		_cyttsp_change_state(ts, CY_ACTIVE_STATE);
		break;
	case CY_ACTIVE_STATE:
		pr_err("%s: Already in %s state\n", __func__,
			cyttsp_powerstate_string[ts->power_state]);
		break;
	case CY_IDLE_STATE:
	case CY_READY_STATE:
	case CY_BL_STATE:
	case CY_LDR_STATE:
	case CY_SYSINFO_STATE:
	case CY_INVALID_STATE:
	default:
		pr_err("%s: Attempt to resume in %s state\n", __func__,
			cyttsp_powerstate_string[ts->power_state]);
		_cyttsp_queue_startup(ts, false);
		break;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Wake Up %s\n", __func__,
		(retval < 0) ? "FAIL" : "PASS");

	mutex_unlock(&ts->data_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_resume);

int cyttsp_suspend(void *handle)
{
	int retval = 0;
	struct cyttsp *ts = handle;
	u8 sleep = CY_DEEP_SLEEP_MODE;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Suspending...\n", __func__);

	if (ts->irq_enabled)
		disable_irq(ts->irq);//ztemt disable_irq_nosync(ts->irq);

	mutex_lock(&ts->data_lock);
	pr_info("%s: ztemt cyttsp_suspend power_state=%d \n",__func__,ts->power_state);

	if (ts->power_state == CY_ACTIVE_STATE) {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		if (ts->waiting_for_fw) {
			retval = -EBUSY;
			pr_err("%s: Suspend Blocked while waiting for"
				" fw load in %s state\n", __func__,
				cyttsp_powerstate_string[ts->power_state]);
			goto cyttsp_suspend_exit;
		}
#endif
#if 0
		retval = ttsp_write_block_data(ts, CY_REG_BASE +
			offsetof(struct cyttsp_xydata, hst_mode),
			sizeof(sleep), &sleep);
		if (retval < 0) {
			pr_err("%s: Failed to write sleep bit\n", __func__);
			if (ts->irq_enabled)
				enable_irq(ts->irq);
		} else
			_cyttsp_change_state(ts, CY_SLEEP_STATE);
//////////////////////////////////////////////////////
#else
            retval = ttsp_write_block_data(ts, CY_REG_BASE +
                offsetof(struct cyttsp_xydata, hst_mode),
                sizeof(sleep), &sleep);
            if (retval < 0) {
                if (ts->irq_enabled)
                    enable_irq(ts->irq);
                pr_info("%s: Failed to write sleep bit !\n", __func__);
            }else{
                _cyttsp3_force_function.FT_ready_flag = false;//zte add 20160307
                pr_info("%s: Success to write sleep bit !\n", __func__);
                _cyttsp_change_state(ts, CY_SLEEP_STATE);
        }
#endif
//////////////////////////////////////////////////////
	} else {
		switch (ts->power_state) {
		case CY_SLEEP_STATE:
			pr_err("%s: already in %s state\n", __func__,
				cyttsp_powerstate_string[ts->power_state]);
			break;
		case CY_LDR_STATE:
		case CY_SYSINFO_STATE:
		case CY_READY_STATE:
			retval = -EBUSY;
			pr_err("%s: Suspend Blocked while in %s state\n",
				__func__,
				cyttsp_powerstate_string[ts->power_state]);
			break;
		case CY_IDLE_STATE:
		case CY_LOW_PWR_STATE:
		case CY_BL_STATE:
		case CY_INVALID_STATE:
		default:
			pr_err("%s: Cannot enter suspend from %s state\n",
				__func__,
				cyttsp_powerstate_string[ts->power_state]);
			break;
		}

		if (ts->irq_enabled)
			enable_irq(ts->irq);
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
cyttsp_suspend_exit:
#endif
	mutex_unlock(&ts->data_lock);

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void cyttsp_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
	int retval = 0;
	pr_info("%s: ztemt cyttsp_early_suspend\n",__func__);

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: EARLY SUSPEND ts=%p\n", __func__, ts);
	retval = cyttsp_suspend(ts);
	if (retval < 0) {
		pr_err("%s: Early suspend failed with error code %d\n",
			__func__, retval);
	}
}

void cyttsp_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
	int retval = 0;
	pr_info("%s: ztemt cyttsp_late_resume\n",__func__);

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: LATE RESUME ts=%p\n", __func__, ts);
	retval = cyttsp_resume(ts);
	if (retval < 0) {
		pr_err("%s: Late resume failed with error code %d\n",
			__func__, retval);
	}
}
/*** ZTEMT start ***/
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct cyttsp *ts = container_of(self, struct cyttsp, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

    if (evdata && evdata->data && ts ) {
        if (event == FB_EVENT_BLANK && evdata) {
            blank = evdata->data;
            if (*blank == FB_BLANK_UNBLANK) {
                pr_info("%s: ztemt cyttsp FT fb UNBLANK!\n",__func__);
                if (cyttsp_resume(ts) != 0)
                    pr_err("%s: ztemt failed\n",__func__);
            } else if (*blank == FB_BLANK_POWERDOWN) {
                pr_info("%s: ztemt cyttsp  FT fb POWERDOWN!\n",__func__);
                if (cyttsp_suspend(ts) != 0)
                    pr_err("%s: ztemt cyttsp FT failed\n",__func__);
                }
            }
        }
	return 0;
}

static void cyttsp2_setup_fb_notifier(struct cyttsp *cd)
{
	int rc;

	cd->fb_notifier.notifier_call = fb_notifier_callback;

	rc = fb_register_client(&cd->fb_notifier);
	if (rc)
        pr_err("%s: Unable to register fb_notifier: %d\n",__func__, rc);
}
/*ZTEMT end*/
#endif

void cyttsp_core_release(void *handle)
{
	struct cyttsp *ts = handle;

	if (ts == NULL) {
		pr_err("%s: Null context pointer on driver release\n",
			__func__);
		goto cyttsp_core_release_exit;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
/*** ZTEMT start ***/
#elif defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notifier);
/*ZTEMT end*/
#endif
	cyttsp_ldr_free(ts);
	/* mutex must not be locked when destroy is called */
	if (mutex_is_locked(&ts->data_lock))
		mutex_unlock(&ts->data_lock);
	mutex_destroy(&ts->data_lock);
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	if (ts->cyttsp_wq != NULL) {
		destroy_workqueue(ts->cyttsp_wq);
		ts->cyttsp_wq = NULL;
	}
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->fwname != NULL) {
		kfree(ts->fwname);
		ts->fwname = NULL;
	}
#endif
	kfree(ts);
	ts = NULL;

cyttsp_core_release_exit:
	return;
}
EXPORT_SYMBOL_GPL(cyttsp_core_release);

static int cyttsp_open(struct input_dev *dev)
{
	int retval = 0;

	struct cyttsp *ts = input_get_drvdata(dev);
    pr_err("%s ztemt\n", __func__);

	if (ts == NULL) {
		pr_err("%s: NULL context pointer\n", __func__);
		retval = -ENOMEM;
		goto cyttsp_open_exit;
	}

	mutex_lock(&ts->data_lock);
	if (!ts->powered) {
		/* Communicate with the IC */
		retval = _cyttsp_startup(ts);
		if (retval < 0) {
			pr_err("%s: startup fail r=%d\n",__func__, retval);
/*** ZTEMT start ***/
			_cyttsp3_force_function.FT_ready_flag = false;
			pr_info("%s: ztemt FT_ready_flag = false\n",__func__);
/*ZTEMT end*/
			if (ts->power_state != CY_IDLE_STATE)
				_cyttsp_change_state(ts, CY_IDLE_STATE);
			ts->powered = false;
		} else {
			ts->powered = true;
/*** ZTEMT start ***/
			_cyttsp3_force_function.FT_ready_flag = true;
			pr_info("%s: ztemt FT_ready_flag = true\n",__func__);
/*ZTEMT end*/
		}

		pr_info("%s: Powered ON(%d) r=%d\n",
			__func__, (int)ts->powered, retval);
	}
	mutex_unlock(&ts->data_lock);
cyttsp_open_exit:
	return retval;
}

static void cyttsp_close(struct input_dev *dev)
{
	/*
	 * close() normally powers down the device
	 * this call simply returns unless power
	 * to the device can be controlled by the driver
	 */
	return;
}

#ifndef CY_USE_THREADED_IRQ
/* This is code for use with specialty host linux systems.
 * For standard Linux host systems, use the threaded_irq method.
 */
static void cyttsp3_irq_work_function(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, irq_work);
	pr_debug("%s: !!! -------- IRQ Work --------- !!!\n", __func__);
	cyttsp_irq(ts->irq, (void *)ts);
}

peint_handle * cyttsp3_irq_wrapper(void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;
	
	pr_debug("%s: !!! -------- IRQ Wrapper --------- !!!\n", __func__);
	queue_work(parade_wq, &ts->irq_work);
	return 0;
}

#endif /* not CY_USE_THREADED_IRQ */

static int cyttsp3_core_probe_complete(struct cyttsp *ts);
static void cyttsp3_core_probe_work(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, core_probe_work);
	int rc;
	rc = cyttsp3_core_probe_complete(ts);
	pr_err("%s: Core Probe_complete returns rc=%d\n", __func__, rc);
}
static void cyttsp3_core_probe_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	if (!ts)
		return;

	pr_err("%s: Core Probe timer triggered\n", __func__);

	if (!work_pending(&ts->core_probe_work))
		schedule_work(&ts->core_probe_work);
}

void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops,
	struct device *dev, int irq, char *name)
{
	struct input_dev *input_device;
	struct cyttsp *ts = NULL;
	//int retval = 0;

	/* disable force function */
	_cyttsp3_force_function.operational = 0;

	if (dev == NULL) {
		pr_err("%s: Error, dev pointer is NULL\n", __func__);
		goto error_alloc_data;
	}

	if (bus_ops == NULL) {
		pr_err("%s: Error, bus_ops pointer is NULL\n", __func__);
		goto error_alloc_data;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc context memory\n", __func__);
		goto error_alloc_data;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->fwname = kzalloc(CY_BL_FW_NAME_SIZE, GFP_KERNEL);
	if (ts->fwname == NULL) {
		pr_err("%s: Error, kzalloc fwname\n", __func__);
		goto error_alloc_failed;
	}
#endif
	ts->cyttsp_wq =
		create_singlethread_workqueue("cyttsp_resume_startup_w");
	if (ts->cyttsp_wq == NULL) {
		pr_err("%s: No memory for cyttsp_resume_startup_wq\n",
			__func__);
		goto error_alloc_failed;
	}

	ts->power_state = CY_INVALID_STATE;
	ts->powered = false;
	ts->was_suspended = false;

	ts->dev = dev;
	ts->bus_ops = bus_ops;
	ts->platform_data = dev->platform_data;
	if (ts->platform_data == NULL) {
		pr_err("%s: Error, platform data is NULL\n", __func__);
		goto error_alloc_failed;
	}

	if (ts->platform_data->frmwrk == NULL) {
		pr_err("%s: Error, platform data framework is NULL\n",
		__func__);
		goto error_alloc_failed;
	}

	if (ts->platform_data->frmwrk->abs == NULL) {
		pr_err("%s: Error, platform data in framwork array is NULL\n",
		__func__);
		goto error_alloc_failed;
	}

	mutex_init(&ts->data_lock);
	init_completion(&ts->bl_int_running);
	init_completion(&ts->si_int_running);
	init_completion(&ts->op_int_running);
	init_completion(&ts->ld_int_running);
	ts->flags = ts->platform_data->flags;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->waiting_for_fw = false;
	ts->ic_grpnum = 0;
	ts->ic_grpoffset = 0;
	ts->ic_reflash_done = false;
	ts->bus_ops->tsdebug = CY_DBG_LVL_0;
#endif
#ifdef CY_USE_REG_ACCESS
	ts->rw_regid = 0;
#endif

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (input_device == NULL) {
		pr_err("%s: Error, failed to allocate input device\n",
			__func__);
		goto error_init;
	}

	ts->input = input_device;
	input_device->name = name;
	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(dev));
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->dev;
	ts->bus_type = bus_ops->dev->bus;
	input_device->open = cyttsp_open;
	input_device->close = cyttsp_close;
	input_set_drvdata(input_device, ts);
	dev_set_drvdata(dev, ts);

	/* Add /sys files at beginning */
	cyttsp_ldr_init(ts);

	/* Some host i2c busses start late and then run too slow */
	dev_info(ts->dev, "%s:start wait for probe timer\n", __func__);
	INIT_WORK(&ts->core_probe_work, cyttsp3_core_probe_work);
	setup_timer(&ts->core_probe_timer, cyttsp3_core_probe_timer,
			(unsigned long)ts);
	mod_timer(&ts->core_probe_timer, jiffies +
			msecs_to_jiffies(CY_CORE_PROBE_STARTUP_DELAY_MS));
#if 0
	retval = gpio_request(rst_gpio_number, NULL);//ts->platform_data->rst_gpio
	if (retval < 0) {
		dev_err(dev, "%s: gpio_free called for rst_gpio retval =%d\n",
			__func__, retval);
		gpio_free(rst_gpio_number);//ts->platform_data->rst_gpio
		retval = gpio_request(rst_gpio_number, NULL);//ts->platform_data->rst_gpio
	}
	if (retval < 0) {
		dev_err(dev, "%s: Fail request gpio=%d\n", __func__,
			rst_gpio_number);//ts->platform_data->rst_gpio
	}
#endif

	goto no_init_error;

error_init:
error_alloc_failed:
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->fwname != NULL) {
		kfree(ts->fwname);
		ts->fwname = NULL;
	}
#endif
	if (ts != NULL) {
		mutex_destroy(&ts->data_lock);
		kfree(ts);
		ts = NULL;
	}
error_alloc_data:
	pr_err("%s: Failed Initialization\n", __func__);
no_init_error:
	return ts;
}

static int cyttsp3_core_probe_complete(struct cyttsp *ts)
{
	unsigned long irq_flags = 0;
	int retval = 0;

	/* enable interrupts */
	ts->irq = 0;

#ifdef CY_USE_THREADED_IRQ
	dev_dbg(ts->dev, "%s: PARADE Passed to gpio_to_irq:: platform_irq=%d ts_irq=%d\n",
		__func__, ts->platform_data->irq_gpio, ts->irq);
	ts->irq = gpio_to_irq(ts->platform_data->irq_gpio);
	dev_dbg(ts->dev, "%s: PARADE Return from gpio_to_irq:: ts->irq=%d\n",
		__func__, ts->irq);
	if (ts->irq <= 0) {
		pr_err("%s: Error, failed to allocate irq\n", __func__);
		goto probe_complete_error_init;
	}
	
#ifdef CY_USE_LEVEL_IRQ
	irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
#else
	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
#endif
	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: Initialize IRQ: flags=%08X\n",
		__func__, (unsigned int)irq_flags);

	retval = request_threaded_irq(ts->irq, NULL, cyttsp_irq,
		irq_flags, ts->input->name, ts);
#else /* use single thread irq handling */
	{
		u32 int_handle;
#ifdef CY_USE_LEVEL_IRQ
		irq_flags = TRIG_LEVL_LOW;
#else
		irq_flags = TRIG_EDGE_NEGATIVE;
#endif
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Initialize IRQ: flags=%08X\n",
			__func__, (unsigned int)irq_flags);
		/* PARADE Adding new work queue to cd struct */
		INIT_WORK(&ts->irq_work, cyttsp3_irq_work_function);
		parade_wq = create_singlethread_workqueue("parade_wq");
		if (!parade_wq) 
			dev_err(ts->dev,"%s Create workqueue failed.\n",__func__);

		pr_debug("%s: %s: ts_irq=%d irq_flags=%lx\n", __func__,
			"Parade - try to initialize interrupts",
			ts->irq, irq_flags);
		int_handle = sw_gpio_irq_request(irq_gpio_number,
			irq_flags, cyttsp3_irq_wrapper, ts);//ts->platform_data->irq_gpio
		if (!int_handle) {
			dev_err(ts->dev, "%s: %s\n", __func__,
				"PARADE error, could not request irq");
			retval = -1;
		}
		else {
			retval = 0;
			ctp_set_int_port_rate(irq_gpio_number, 1);//ts->platform_data->irq_gpio
			/* Debounce INT line */
			ctp_set_int_port_deb(irq_gpio_number, 0x07);//ts->platform_data->irq_gpio
			pr_info("%s: %s\n", __func__,
				"PARADE sw_gpio_ir1_request SUCCESS");
		}
	}

#endif /* CY_USE_THREADED_IRQ */

	if (retval < 0) {
		pr_err("%s: IRQ request failed r=%d\n",
			__func__, retval);
		ts->irq_enabled = false;
	}

	ts->irq_enabled = true;

	/* this enables the cyttsp3_open() call from the system */
	retval = input_register_device(ts->input);
	if (retval < 0) {
		pr_err("%s: Error, failed to register input device r=%d\n",
			__func__, retval);
		goto error_input_register_device;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_early_suspend;
	ts->early_suspend.resume = cyttsp_late_resume;
	register_early_suspend(&ts->early_suspend);
/*** ZTEMT start ***/
#elif defined(CONFIG_FB)
        cyttsp2_setup_fb_notifier(ts);
/*ZTEMT end*/
#endif

	INIT_WORK(&ts->cyttsp_resume_startup_work, cyttsp_ts_work_func);

	goto no_probe_error;

error_input_register_device:
	input_free_device(ts->input);
no_probe_error:
	_cyttsp3_force_function.handle = (void *)ts;
	_cyttsp3_force_function.ready = 1;
probe_complete_error_init:
pr_err("%s: core probe complete; set force_function ready ts=%p handle=%p\n",
	__func__, ts, _cyttsp3_force_function.handle);
	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard touchscreen FF driver core");
MODULE_AUTHOR("Parade Technologies");
