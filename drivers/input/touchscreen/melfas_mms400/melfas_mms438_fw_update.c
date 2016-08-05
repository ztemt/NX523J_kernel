/*
 * MELFAS MMS438/449/458 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Firmware update functions
 *
 */

#include "melfas_mms400.h"

//ISC Info
#define ISC_PAGE_SIZE				128

//ISC Command
#define ISC_CMD_ERASE_ALL		{0xFB,0x4A,0x00,0x15,0x00,0x00}
#define ISC_CMD_ERASE_PAGE		{0xFB,0x4A,0x00,0x8F,0x00,0x00}
#define ISC_CMD_READ_PAGE		{0xFB,0x4A,0x00,0xC2,0x00,0x00}
#define ISC_CMD_WRITE_PAGE		{0xFB,0x4A,0x00,0xA5,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE	{0xFB,0x4A,0x00,0x54,0x00,0x00}
#define ISC_CMD_READ_STATUS		{0xFB,0x4A,0x36,0xC2,0x00,0x00}
#define ISC_CMD_EXIT			{0xFB,0x4A,0x00,0x66,0x00,0x00}

//ISC Status
#define ISC_STATUS_BUSY			0x96
#define ISC_STATUS_DONE			0xAD

/**
* Read ISC status
*/
static int mms_isc_read_status(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	u8 cmd[6] =  ISC_CMD_READ_STATUS;
	u8 result = 0;
	int cnt = 100;
	int ret = 0;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 6,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &result,
			.len = 1,
		},
	};

	TP_LOG_DEBUG("start\n");

	do {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
			TP_LOG_ERROR("failed to i2c_transfer\n");
			return -1;
		}

		if (result == ISC_STATUS_DONE) {
			ret = 0;
			break;
		} else if (result == ISC_STATUS_BUSY) {
			ret = -1;
			msleep(1);
		} else {
			TP_LOG_ERROR("wrong value [0x%02X]\n", result);
			ret = -1;
			msleep(1);
		}
	} while (--cnt);

	if (!cnt) {
		TP_LOG_ERROR("count overflow - cnt [%d] status [0x%02X]\n", cnt, result);
		goto ERROR;
	}

	TP_LOG_DEBUG("end\n");

	return ret;

ERROR:
	return ret;
}

/**
* Command : Erase Page
*/
static int mms_isc_erase_page(struct mms_ts_info *info, int offset)
{
	u8 write_buf[6] = ISC_CMD_ERASE_PAGE;

	struct i2c_msg msg[1] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 6,
		},
	};

	TP_LOG_DEBUG("start\n");

	write_buf[4] = (u8)(((offset) >> 8) & 0xFF);
	write_buf[5] = (u8)(((offset) >> 0) & 0xFF);
	if (i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
		TP_LOG_ERROR("failed to i2c_transfer\n");
		goto ERROR;
	}

	if (mms_isc_read_status(info) != 0) {
		goto ERROR;
	}

	TP_LOG_DEBUG("Offset [0x%04X]\n", offset);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Read Page
*/
static int mms_isc_read_page(struct mms_ts_info *info, int offset, u8 *data)
{
	u8 write_buf[6] = ISC_CMD_READ_PAGE;

	struct i2c_msg msg[2] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 6,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = ISC_PAGE_SIZE,
		},
	};

	TP_LOG_DEBUG("start\n");

	write_buf[4] = (u8)(((offset) >> 8) & 0xFF);
	write_buf[5] = (u8)(((offset) >> 0) & 0xFF);
	if (i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
		TP_LOG_ERROR("failed to i2c_transfer\n");
		goto ERROR;
	}

	TP_LOG_DEBUG("Offset [0x%04X]\n", offset);

	return 0;

ERROR:
	return -1;
}

#if 0
/**
* Command : Write Page
*/
static int mms_isc_write_page(struct mms_ts_info *info, int offset,const u8 *data, int length)
{
	u8 write_buf[134] = ISC_CMD_WRITE_PAGE;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if( length > 128 ){
		dev_err(&info->client->dev, "%s [ERROR] page length overflow\n", __func__);
		goto ERROR;
	}
		
	write_buf[4] = (u8)(((offset)>>8)&0xFF );
	write_buf[5] = (u8)(((offset)>>0)&0xFF );
	
	memcpy( &write_buf[6], data, length);
	
	if(i2c_master_send(info->client, write_buf, length+6 )!=length+6){
		dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
		goto ERROR;
	}

	if(mms_isc_read_status(info) != 0){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE] - Offset[0x%04X] Length[%d]\n", __func__, offset, length);

	return 0;

ERROR:
	return -1;
}
#endif

/**
* Command : Program Page
*/
static int mms_isc_program_page(struct mms_ts_info *info, int offset,
	const u8 *data, int length)
{
	u8 write_buf[134] = ISC_CMD_PROGRAM_PAGE;

	TP_LOG_DEBUG("start\n");

	if (length > 128) {
		TP_LOG_ERROR("page length overflow\n");
		goto ERROR;
	}

	write_buf[4] = (u8)(((offset) >> 8) & 0xFF);
	write_buf[5] = (u8)(((offset) >> 0) & 0xFF);

	memcpy(&write_buf[6], data, length);

	if (i2c_master_send(info->client, write_buf, length + 6) != length + 6) {
		TP_LOG_ERROR("failed to i2c_master_send\n");
		goto ERROR;
	}

	if (mms_isc_read_status(info) != 0) {
		goto ERROR;
	}

	TP_LOG_DEBUG("Offset[0x%04X] Length[%d]\n", offset, length);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Exit ISC
*/
static int mms_isc_exit(struct mms_ts_info *info)
{
	u8 write_buf[6] = ISC_CMD_EXIT;

	TP_LOG_DEBUG("start\n");

	if (i2c_master_send(info->client, write_buf, 6) != 6) {
		TP_LOG_ERROR("failed to i2c_master_send\n");
		goto ERROR;
	}

	TP_LOG_DEBUG("end\n");

	return 0;

ERROR:
	return -1;
}

/**
* Flash chip firmware (main function)
*/
int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size,
	bool force, bool section)
{
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	//struct i2c_client *client = info->client;
	int i = 0;
	int retires = 3;
	int nRet = 0;
	int nStartAddr = 0;
	int nWriteLength = 0;
	int nLast = 0;
	int nOffset = 0;
	int nTransferLength = 0;
	int size = 0;
	u8 *data;
	u8 *cpydata;

	int offset = sizeof(struct mms_bin_hdr);

	bool update_flag = false;
	bool update_flags[MMS_FW_MAX_SECT_NUM] = {false, };

	u16 ver_chip[MMS_FW_MAX_SECT_NUM] = { 0 };
	u16 ver_file[MMS_FW_MAX_SECT_NUM] = { 0 };

	int offsetStart = 0;
	u8 initData[ISC_PAGE_SIZE] = { 0 };

	memset(initData, 0xFF, sizeof(initData));

	TP_LOG_INFO("start\n");

	//Read firmware file
	fw_hdr = (struct mms_bin_hdr *)fw_data;
	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);
	if (img == NULL) {
		nRet = -1;
		TP_LOG_ERROR("failed to kzalloc\n");
		goto err_kzalloc_img;
	}

	//Check firmware file
	if (memcmp(CHIP_FW_CODE, &fw_hdr->tag[4], 4)) {
		nRet = fw_err_file_type;
		TP_LOG_ERROR("F/W file is not for %s\n", CHIP_NAME);
		goto err_memcmp;
	}

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
		memset(ver_chip, 0xFFFF, sizeof(ver_chip));
		TP_LOG_ERROR("failed to read chip firmware version, set to [0xFFFF]\n");
	} else {
		TP_LOG_INFO("chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n",
			ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
	}

	//Set update flag
	TP_LOG_INFO("Firmware file info : Sections[%d] Offset[0x%08X] Length[0x%08X]\n",
		fw_hdr->section_num, fw_hdr->binary_offset, fw_hdr->binary_length);

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw_data + offset);
		ver_file[i] = img[i]->version;

		TP_LOG_INFO("Section[%d] Version[0x%04X] StartPage[%d] EndPage[%d] Offset[0x%08X] Length[0x%08X]\n",
			i, img[i]->version, img[i]->start_page, img[i]->end_page,
			img[i]->offset, img[i]->length);

		//Compare section version
		if ((ver_chip[i] < ver_file[i]) || (ver_chip[i] == 0xFFFF)) {
			//Set update flag
			update_flag = true;
			update_flags[i] = true;

			TP_LOG_INFO("Section[%d] need to be updated. Chip[0x%04X] File[0x%04X]\n",
				i, ver_chip[i], ver_file[i]);
		}
	}

	//Set force update flag
	if (force == true) {
		update_flag = true;
		update_flags[0] = true;
		update_flags[1] = true;
		update_flags[2] = true;
		update_flags[3] = true;

		TP_LOG_INFO("Force update\n");
	}

	//Exit when up-to-date
	if (update_flag == false) {
		nRet = fw_err_uptodate;
		TP_LOG_DEBUG("chip firmware is already up-to-date\n");
		goto err_no_update;
	}

	//Set start addr offset
	if (section == true) {
		if (update_flags[0] == true) {
			//boot
			offsetStart = img[0]->start_page;
		} else if (update_flags[1] == true) {
			//core
			offsetStart = img[1]->start_page;
		} else if (update_flags[2] == true) {
			//custom
			offsetStart = img[2]->start_page;
		} else if (update_flags[3] == true) {
			//param
			offsetStart = img[3]->start_page;
		}
	} else {
		offsetStart = 0;
	}

	offsetStart = offsetStart * 1024;

	//Load firmware data
	data = kzalloc(sizeof(u8) * fw_hdr->binary_length, GFP_KERNEL);
	if (data == NULL) {
		nRet = -1;
		TP_LOG_ERROR("failed to kzalloc\n");
		goto err_kzalloc_data;
	}
	size = fw_hdr->binary_length;

	cpydata = kzalloc(ISC_PAGE_SIZE, GFP_KERNEL);
	if (cpydata == NULL) {
		nRet = -1;
		TP_LOG_ERROR("failed to kzalloc\n");
		goto err_kzalloc_cpydata;
	}

	//Check firmware size
	if (size % ISC_PAGE_SIZE != 0) {
		size += (ISC_PAGE_SIZE - (size % ISC_PAGE_SIZE));
	}

	nStartAddr = 0;
	nWriteLength = size;
	nLast = nStartAddr + nWriteLength;

	if ((nLast) % 8 != 0 ) {
		nRet = fw_err_file_type;
		TP_LOG_ERROR("firmware size mismatch\n");
		goto err_nLast;
	} else {
		memcpy(data, fw_data + fw_hdr->binary_offset, fw_hdr->binary_length);
	}

	//Set address
	nOffset = nStartAddr + nWriteLength - ISC_PAGE_SIZE;
	nTransferLength = ISC_PAGE_SIZE;

	//Erase first page
	TP_LOG_INFO("Erase first page : Offset[0x%04X]\n", offsetStart);
	nRet = mms_isc_erase_page(info, offsetStart);
	if (nRet != 0) {
		TP_LOG_ERROR("failed to clear first page\n");
		goto err_erase_page;
	}

	//Flash firmware
	TP_LOG_INFO("Start Download : Offset Start[0x%04X] End[0x%04X]\n",
		nOffset, offsetStart);
	while (nOffset >= offsetStart) {
		TP_LOG_DEBUG("Downloading : Offset[0x%04X]\n", nOffset);

		//Program (erase and write) a page
		nRet = mms_isc_program_page(info, nOffset, &data[nOffset], nTransferLength);
		if (nRet != 0) {
			TP_LOG_ERROR("failed to isc_program_page\n");
			goto err_program_page;
		}

		//Verify (read and compare)
		if (mms_isc_read_page(info, nOffset, cpydata)) {
			TP_LOG_ERROR("failed to mms_isc_read_page\n");
			goto err_read_page;
		}

		if (memcmp(&data[nOffset], cpydata, ISC_PAGE_SIZE)) {
#if MMS_FW_UPDATE_DEBUG
			print_hex_dump(KERN_ERR, "Firmware Page Write : ", DUMP_PREFIX_OFFSET,
				16, 1, data, ISC_PAGE_SIZE, false);
			print_hex_dump(KERN_ERR, "Firmware Page Read : ", DUMP_PREFIX_OFFSET,
				16, 1, cpydata, ISC_PAGE_SIZE, false);
#endif
			TP_LOG_ERROR("failed to verify page\n");

			nRet = -1;
			goto err_verify;
		}

		nOffset -= nTransferLength;
	}

	//Exit ISC
	nRet = mms_isc_exit(info);
	if (nRet != 0) {
		TP_LOG_ERROR("failed to mms_isc_exit\n");
		goto err_isc_exit;
	}

	//Reboot chip
	mms_reboot(info);

	//Check chip firmware version
	if (mms_get_fw_version_u16(info, ver_chip)) {
		nRet = -1;
		TP_LOG_ERROR("failed to read chip firmware version after flash\n");
		goto err_get_version;
	} else {
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (ver_chip[i] != ver_file[i]) {
				TP_LOG_ERROR("version mismatch after flash. Section[%d] : \
					Chip[0x%04X] != File[0x%04X]\n", i, ver_chip[i], ver_file[i]);
				nRet = -1;
				goto err_cmp_version;
			}
		}
	}

	nRet = 0;
	TP_LOG_INFO("Firmware update completed\n");

err_cmp_version:
err_get_version:
	goto err_nLast;
err_isc_exit:
err_verify:
err_read_page:
err_program_page:
	mms_isc_exit(info);
err_erase_page:
err_nLast:
	if (cpydata)
		kfree(cpydata);
err_kzalloc_cpydata:
	if (data)
		kfree(data);
err_kzalloc_data:
err_no_update:
err_memcmp:
	if (img)
		kfree(img);
err_kzalloc_img:
	return nRet;
}
