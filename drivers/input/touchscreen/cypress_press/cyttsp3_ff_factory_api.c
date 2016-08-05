#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/workqueue.h>

#include <linux/thread_info.h>
#include <linux/fcntl.h>

#define DEBUG
#ifdef DEBUG
#define SENSOR_LOG_DEBUG(format, arg...) printk("<<Cyttsp_FT>>[%s:%d] "format"\n", __func__, __LINE__, ##arg)
#else
#define SENSOR_LOG_DEBUG(format, arg...)
#endif

int pressure_file_write(char *file_path, const char *write_buf, unsigned int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_write_retval = 0;
	//int retval = 0;

	SENSOR_LOG_DEBUG("%s: size = %d\n", __func__, count);
	if(NULL == file_path) {
		SENSOR_LOG_DEBUG("%s: file_path is NULL\n", __func__);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	file_p = filp_open(file_path, O_CREAT|O_RDWR | O_TRUNC, 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_DEBUG("[open file <%s>failed] \n", file_path);
		goto error;
	}

	vfs_write_retval = vfs_write(file_p, (char *)write_buf, count, &file_p->f_pos);
	if(vfs_write_retval < 0){
		SENSOR_LOG_DEBUG("[write file <%s>failed]\n", file_path);
		goto error;
	}


	filp_close(file_p, NULL);
	set_fs(old_fs);

	SENSOR_LOG_DEBUG("pressure_file_write ok\n");
	return 0;

error:
	return -1;
}
EXPORT_SYMBOL_GPL(pressure_file_write);

int pressure_file_read(char *file_path, const char *read_buf, unsigned int count)
{
	struct file *file_p;
	ssize_t	nread = 0;
	mm_segment_t old_fs;	
	//int retval = 0;
	//struct kstat file_kstat;
	unsigned int file_size;

	SENSOR_LOG_DEBUG("%s: 1 size = %d\n", __func__, count);
	if(NULL == file_path) {
		SENSOR_LOG_DEBUG("%s: file_path is NULL\n", __func__);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);  
	file_p = filp_open(file_path, O_RDONLY, 0666);//file_p = filp_open(file_path, O_CREAT|O_RDWR, 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_DEBUG("[open file <%s>failed]\n", file_path);
		goto error;
	}

	//read file size
	/*
	retval = vfs_stat(file_path, &file_kstat);
	file_size = (unsigned int)file_kstat.size;*/
	file_size = file_p->f_path.dentry->d_inode->i_size;	
	if(file_size > count){
		SENSOR_LOG_DEBUG("[file <%s> size %d larger than buffer size %d]\n", file_path, file_size, count);
		goto error;
	}
	count = file_size;
	SENSOR_LOG_DEBUG("%s: 2 size = %d\n", __func__, count);

    //read file
	nread = vfs_read(file_p, (char __user *)read_buf, count, &file_p->f_pos);
	if(nread != count){
		SENSOR_LOG_DEBUG("[read file <%s>failed]\n", file_path);
		goto error;
	}

	filp_close(file_p, NULL);
	set_fs(old_fs);

	SENSOR_LOG_DEBUG("pressure_file_read ok\n");
	return (int)nread;

error:
	return -1;
}
EXPORT_SYMBOL_GPL(pressure_file_read);


