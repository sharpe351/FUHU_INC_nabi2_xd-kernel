/*
 * ov5640.c - ov5640 sensor driver
 *
 * Copyright (c) 2011 - 2012, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      Abhinav Sinha <absinha@nvidia.com>
 *
 * Leverage soc380.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov5640.h>

#include "ov5640_tables.h"

/* Focuser single step & full scale transition time truth table
 * in the format of:
 *    index	mode		single step transition	full scale transition
 *	0	0			0			0
 *	1	1			50uS			51.2mS
 *	2	1			100uS			102.3mS
 *	3	1			200uS			204.6mS
 *	4	1			400uS			409.2mS
 *	5	1			800uS			818.4mS
 *	6	1			1600uS			1637.0mS
 *	7	1			3200uS			3274.0mS
 *	8	0			0			0
 *	9	2			50uS			1.1mS
 *	A	2			100uS			2.2mS
 *	B	2			200uS			4.4mS
 *	C	2			400uS			8.8mS
 *	D	2			800uS			17.6mS
 *	E	2			1600uS			35.2mS
 *	F	2			3200uS			70.4mS
 */

/* pick up the mode index setting and its settle time from the above table */
#define OV5640_VCM_DACMODE 0x3602
#define OV5640_TRANSITION_MODE 0x0B
#define SETTLETIME_MS 5

#define POS_LOW (0)
#define POS_HIGH (1023)
#define FPOS_COUNT 1024
#define FOCAL_LENGTH (10.0f)
#define FNUMBER (2.8f)

#define SIZEOF_I2C_TRANSBUF 64

#define CAMERA_DEBUG
#define SENSOR_MAX_RETRIES 3

struct ov5640_info {
	int mode;
	struct miscdevice miscdev_info;
	struct i2c_client *i2c_client;
	struct ov5640_platform_data *pdata;
	struct ov5640_config focuser;
	int af_fw_loaded;
	struct kobject *kobj;
	struct device *dev;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	int preview_x;
	int preview_y;

};

static struct ov5640_info *ov5640_sensor_info;
int init_check_ov5640 = 1;
int first_preview_ov5640 = 1;

static int ov5640_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[3] = {0};


	if (!client->adapter)
		return -ENODEV;
		

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* Write addr - high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);

	/* Read back data */
	if (err >= 0) {
		msg->len = 3;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
			*val = data[0];
			
		return 0;
	}
	//v4l_err(client, "read from offset 0x%x error %d\n", reg, err);
	return err;
}

static int ov5640_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s: i2c transfer failed, retrying %x %x %d\n",
				__func__, addr, val, err);
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}


static int ov5640_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	dev_err(&client->dev, "ov5640: i2c transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov5640_write_table(struct ov5640_info *info,
			      struct ov5640_reg table[],
			      struct ov5640_reg override_list[],
			      int num_override_regs)
{
	int err;
	struct ov5640_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	int i;
	u16 val;

	for (next = table; next->addr != OV5640_TABLE_END; next++) {
		if (next->addr == OV5640_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV5640_TABLE_END &&
			n_next->addr != OV5640_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov5640_write_bulk_reg(info->i2c_client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}

//&*&*&*CJ1_20130429: add for IQ test
#ifdef CAMERA_DEBUG

static int jj_table_5mp_size(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	printk("[camera][%s][%d]:firmware_name = (%s),strlen=(%d) \n",__FUNCTION__,__LINE__,firmware_name,strlen(firmware_name));
	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("[%s]error occured while opening file (%s)(%d).\n", __FUNCTION__,filepath,strlen(filepath));
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int jj_table_5mp_cmd(char *filename)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned char *buf;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	int fwsize = jj_table_5mp_size(filename);
	buf = kmalloc(fwsize + 1, GFP_ATOMIC);
		
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", filename);
	
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[%s]error occured while opening file (%s).\n", __FUNCTION__, filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, buf, fsize, &pos);
	filp_close(pfile, NULL);
	
	{
	int i=0;
	int j=0;
	char *buff_temp="0x1234";
	int para_count = 0;
	u16 addr=0;
	u8 val=0;

	memset(buff_temp, 0, sizeof(buff_temp));
	while (i<fsize)
	{
		//printk("buf[%d]=0x%x \n", i,buf[i]);
		if(buf[i]==',')
		{
			*(buff_temp+j) = '\0';
			para_count++;
			if(para_count == 1)
			{
				addr = simple_strtoul(buff_temp, NULL, 16);
			}
			else if(para_count == 2)
			{
				val = simple_strtoul(buff_temp, NULL, 16);
				para_count = 0;
				printk("[camera](%s)(addr=0x%x, val=0x%x)  \n", __FUNCTION__,addr,val);	
				 ov5640_write_reg(ov5640_sensor_info->i2c_client , addr, val);
			}
			else 
			{
				printk("[camera](%s)Error  \n", __FUNCTION__);
			}			
			
			j=0;
			i++;
			memset(buff_temp, 0, sizeof(buff_temp));
			continue;
		}
		else if(buf[i]<48)
		{
			j=0;
			i++;
		}
		else if(buf[i]>122)
		{
			j=0;
			i++;
		}
		else
		{
			*(buff_temp+j++) = *(buf+i++);
		}
	}
	*(buff_temp+j) = '\0';
	
	}	
	
	set_fs(old_fs);
	kfree(buf);
	return 0;
}

///echo "0x1234,0x0000," > sys/devices/platform/tegra-i2c.2/i2c-2/2-003c/camera_5mp_i2c_write
static ssize_t jj_camera_5mp_i2c_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int i=0;
	int j=0;
	char *buff_temp="0x1234";
	int para_count = 0;
	u16 addr=0;
	u8 val=0;

	if(buf == NULL)
		return -1;
		
	printk("[camera]Enter %s, buf is %s, count is %d\n", __FUNCTION__,buf, count);

	memset(buff_temp, 0, sizeof(buff_temp));
	while (i<count)
	{
		if(buf[i]==',')
		{
			*(buff_temp+j) = '\0';
			
			para_count++;
			
			if(para_count == 1)
			{
				addr = simple_strtoul(buff_temp, NULL, 16);
			}
			else if(para_count == 2)
			{
				val = simple_strtoul(buff_temp, NULL, 16);
			}
			else 
			{
				printk("(%s)(%d)Error  \n", __FUNCTION__,__LINE__);
			}			
			
			j=0;
			i++;
			memset(buff_temp, 0, sizeof(buff_temp));
			continue;
		}
		else
		{
			*(buff_temp+j++) = *(buf+i++);
		}
	}
	*(buff_temp+j) = '\0';

	para_count++;
	
	printk("(%s)(addr=0x%x, val=0x%x)  \n", __FUNCTION__,addr,val);	
	ov5640_write_reg(ov5640_sensor_info->i2c_client , addr, val);
		
	return count;
}
static DEVICE_ATTR(camera_5mp_i2c_write, 0664, NULL, jj_camera_5mp_i2c_write);

///echo "0x0F12," > sys/devices/platform/tegra-i2c.2/i2c-2/2-003c/camera_5mp_i2c_read
static ssize_t jj_camera_5mp_i2c_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int i=0;
	int j=0;
	char *buff_temp="0x1234";
	int para_count = 0;
	u16 addr=0;
	u8 val=0;

	if(buf == NULL)
		return -1;
	
	memset(buff_temp, 0, sizeof(buff_temp));
	while (i<count)
	{
		if(buf[i]==',')
		{
			*(buff_temp+j) = '\0';
			para_count++;
			if(para_count == 1)
			{
				addr = simple_strtoul(buff_temp, NULL, 16);
			}
			else 
			{
				printk("(%s)(%d)Error  \n", __FUNCTION__,__LINE__);
			}			
			
			j=0;
			i++;
			memset(buff_temp, 0, sizeof(buff_temp));
			continue;
		}
		else
		{
			*(buff_temp+j++) = *(buf+i++);
		}
	}
	*(buff_temp+j) = '\0';
	para_count++;
	
	ov5640_read_reg(ov5640_sensor_info->i2c_client, addr, &val);
	
	printk("(%s)ov5640_read_reg(addr=0x%x, val=[0x%x])  \n", __FUNCTION__,addr,val);	
		
	return count;
}
static DEVICE_ATTR(camera_5mp_i2c_read, 0664, NULL, jj_camera_5mp_i2c_read);

//echo "/sdcard/data/xxx.txt" > /sys/devices/platform/tegra-i2c.2/i2c-2/2-003c/camera_5mp_table				
static ssize_t jj_camera_5mp_table(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	
	if(buf != NULL)
	{
		char filename[128];
			
		sprintf(filename, "%s", buf);
	
		//printk("[camera]Enter %s, buf is %s, count is %d,table_name=%s \n", __FUNCTION__,buf, count,filename);
		*(filename+count-1) = '\0';
		jj_table_5mp_cmd(filename);
	}
	else
	{
		//return -1;
		char filename[]="/mnt/sdcard/xxx.txt";
		jj_table_5mp_cmd(filename);
	}
	
	return count;
}
static DEVICE_ATTR(camera_5mp_table, 0664, NULL, jj_camera_5mp_table);	
#endif
//&*&*&*CJ2_20130429: add for IQ test


//&*&*&*CJ1_20130429: add for Daig VI test
//echo 0 > sys/devices/platform/tegra-i2c.2/i2c-2/2-003c/diag_camera_5mp_power
static ssize_t diag_camera_5mp_power(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{	
	
	u16 power=0;
	
	power = simple_strtoul(buf, NULL, 10);

	if(power==0)//power off
	{
			ov5640_sensor_info->pdata->power_off();
			ov5640_sensor_info->pdata->suspend();
	}
	else if(power==1)//power on
	{
			ov5640_sensor_info->pdata->init();
			ov5640_sensor_info->pdata->power_on();
	}
	else //error command
	{	
	    printk("[%s][%d]error:power=%d \n",
			__func__, __LINE__, power);
	}
	
	return count;
}
static DEVICE_ATTR(diag_camera_5mp_power, 0664, NULL, diag_camera_5mp_power);
//&*&*&*CJ2_20130429: add for Daig VI test

static int ov5640_set_mode(struct ov5640_info *info, struct ov5640_mode *mode)
{
	int sensor_mode=0;
	int err=0;
//	int i;
//	u16 ov5640reg []= {0x3000,0x3003,0x3004,0x3503,0x3406,0x3034,0x3035,0x3036,0x3037,0x3500,0x3501,0x3502,0x350a,0x350b,0x350c,0x350d,0x380c,0x380d,0x380e,0x380f,0x56a1,0x3a08,0x3a09,0x3a0a,0x3a0b,0x3a0e,0x3a0d,
//	0x3c00,0x3c01,0x4800,0x4837,0x3800,0x3801,0x3802,0x3803,0x3804,0x3805,0x3806,0x3807,0x3808,0x3809,0x380a,0x380b,0x5001,0x3810,0x3811,0x3812,0x3813,0x4300,0x3a00,0x3a02,0x3a03,0x3a14,0x3a15};
//	u16 ov5640reg []= {0x3034,0x3035,0x3036,0x3C07,0x3814,0x3815,0x3800,0x3801,0x3802,0x3803,0x3804,0x3805,0x3806,0x3807,0x3808,0x3809,0x380A,0x380B,0x380C,0x380D,0x380E,0x380F,0x3813,0x3618,0x3612,0x3708,0x3709,
//	0x370C,0x3A02,0x3A03,0x3A0E,0x3A0D,0x3A14,0x3A15,0x4004,0x4713,0x4407,0x460B,0x5001,0x3008,0x3002,0x3006,0x4713,0x4407,0x460b,0x460c,0x3824,0x3a08,0x3a09,0x3a0a,0x3a0b,0x3a0e,0x3a0d,0x4003};
//	u8 val=0;

	pr_info("ov5640 %s x= %d, y=%d\n",__func__,mode->xres,mode->yres);	
	dev_info(info->dev, "%s: xres %u yres %u\n",
			__func__, mode->xres, mode->yres);
//	if (!info->af_fw_loaded) {
//		err = ov5640_write_table(info, tbl_af_firmware, NULL, 0);
//		if (err)
//			return err;
//		info->af_fw_loaded = 1;
//	}

	if(!((ov5640_sensor_info->preview_x == mode->xres)&&
		(ov5640_sensor_info->preview_y = mode->yres)))
  {

      	if (mode->xres == 2592 && mode->yres == 1944)
      		sensor_mode = OV5640_MODE_2592x1944;
      	else if (mode->xres == 1920 && mode->yres == 1080)
      		sensor_mode = OV5640_MODE_1920x1080;
      	else if (mode->xres == 1280 && mode->yres == 960)
      		sensor_mode = OV5640_MODE_1280x960;
//  			else if (mode->xres == 1280 && mode->yres == 720)
//  				sensor_mode = OV5640_MODE_1280x720;
      	else {
      		dev_info(info->dev, "%s: invalid resolution: %d %d\n",
      				__func__, mode->xres, mode->yres);
      		return -EINVAL;
      	}
      
      	err = ov5640_write_table(info, mode_table[sensor_mode],	NULL, 0);

//  		 msleep(20);
      
//      	for(i=0;i<ARRAY_SIZE(ov5640reg);i++)
//      	{
//      		ov5640_read_reg(ov5640_sensor_info->i2c_client, ov5640reg[i], &val);
//      //		printk("(%s)ov5640_read_reg(addr=0x%x, val=[0x%x])  \n", __FUNCTION__,ov5640reg[i],val);	
//      		printk("(addr=0x%x, val=[0x%x])  \n",ov5640reg[i],val);	
//      	}	
  }

	ov5640_sensor_info->preview_x = mode->xres;
	ov5640_sensor_info->preview_y = mode->yres;

	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int ov5640_set_af_mode(struct ov5640_info *info, u8 mode)
{
	pr_info("ov5640 %s \n",__func__);	
	dev_info(info->dev, "%s: mode %d\n", __func__, mode);
	if (mode == OV5640_AF_INIFINITY)
		return ov5640_write_table(info, tbl_release_focus, NULL, 0);

	if (mode == OV5640_AF_TRIGGER)
		return ov5640_write_table(info, tbl_single_focus, NULL, 0);

	return -EINVAL;
}

static int ov5640_get_af_status(struct ov5640_info *info, u8 *val)
{
	int err;

//	pr_info("ov5640 %s \n",__func__);	
	err = ov5640_read_reg(info->i2c_client, 0x3023, val);
	if (err)
		return -EINVAL;

//	dev_info(info->dev, "%s: value %02x\n", __func__, (u32)val);
	return 0;
}

static int ov5640_set_position(struct ov5640_info *info, u32 position)
{
	u8 data[4];

	pr_info("ov5640 %s \n",__func__);	
	if (position < info->focuser.pos_low ||
	    position > info->focuser.pos_high)
		return -EINVAL;

	data[0] = (OV5640_VCM_DACMODE >> 8) & 0xff;
	data[1] = OV5640_VCM_DACMODE & 0xff;
	data[2] = ((position & 0xf) << 4) | OV5640_TRANSITION_MODE;
	data[3] = (position * 0x3f0) >> 4;
	return ov5640_write_bulk_reg(info->i2c_client, data, 4);
}

static int ov5640_set_power(struct ov5640_info *info, u32 level)
{
	pr_info("ov5640 %s \n",__func__);	
	switch (level) {
	case OV5640_POWER_LEVEL_OFF:
	case OV5640_POWER_LEVEL_SUS:
		if (info->pdata && info->pdata->power_off)
			info->pdata->power_off();
//		info->af_fw_loaded = 0;
		info->mode = 0;
		break;
	case OV5640_POWER_LEVEL_ON:
		if (info->pdata && info->pdata->power_on)
			info->pdata->power_on();
		break;
	default:
		dev_err(info->dev, "unknown power level %d.\n", level);
		return -EINVAL;
	}

	return 0;
}

static long ov5640_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err=0;
	u8 val=0;
//	int i=0;
//	u16 ov5640reg []= {0x3034,0x3035,0x3036,0x3C07,0x3814,0x3815,0x3800,0x3801,0x3802,0x3803,0x3804,0x3805,0x3806,0x3807,0x3808,0x3809,0x380A,0x380B,0x380C,0x380D,0x380E,0x380F,0x3813,0x3618,0x3612,0x3708,0x3709,
//	0x370C,0x3A02,0x3A03,0x3A0E,0x3A0D,0x3A14,0x3A15,0x4004,0x4713,0x4407,0x460B,0x5001,0x3008,0x3002,0x3006,0x4713,0x4407,0x460b,0x460c,0x3824,0x3a08,0x3a09,0x3a0a,0x3a0b,0x3a0e,0x3a0d,0x4003};
	struct ov5640_info *info = file->private_data;

//	printk("[ov5640] %s, cmd=0x%x \n",	__func__,cmd);

	if(init_check_ov5640)
	{
		ov5640_write_table(info, mode_init, NULL, 0);

		init_check_ov5640 = 0;
		first_preview_ov5640 =1;
		ov5640_sensor_info->pdata->vcm_pwr_on();	
  	msleep(2);
	}

		if (!info->af_fw_loaded) {
		err = ov5640_write_table(info, tbl_af_firmware, NULL, 0);
  	printk("[ov5640] %s, cmd=0x%x  tbl_af_firmware #######\n",	__func__,cmd);
		if (err)
			return err;
		info->af_fw_loaded = 1;
	}
	
//  ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3023, 0x01); //release AF for noise issue.

	switch (cmd) {
	case OV5640_IOCTL_SET_SENSOR_MODE:
	{
		struct ov5640_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov5640_mode))) {
			return -EFAULT;
		}

		return ov5640_set_mode(info, &mode);
	}
	case OV5640_IOCTL_SET_PREVIEW:
	{

		struct ov5640_mode mode;
	
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov5640_mode))) {
			return -EFAULT;
		}
		
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3503, 0x00);	//for release AEC
 
//continous stop stream ++		
		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3000 , &val);	
		val = val |0x2;
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3000, val);
    
//		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3003 , &val);	
//		val = val |0x2;
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3003, val);
//continous stop stream --


//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3503, 0x00);	//for release AEC

    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3a00, 0x7C);	//2013/05/28

		err = ov5640_set_mode(info, &mode);

    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3821, 0x01);	//for enable binning for de-noise for preview
    
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3022, 0x80);	//for AF default window
		ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3008, 0x02);	// release sw pwd down.


//continous stop stream ++		
//		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3003 , &val);	
//		val = val & 0xfd;
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3003, val);

		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3000 , &val);	
		val = val & 0xfd;
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3000, val);
//continous stop stream --

  	msleep(300);
	
		return err;
	}
	case OV5640_IOCTL_SET_CAPTURE:
	{
		struct ov5640_mode mode;

		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov5640_mode))) {
			return -EFAULT;
		}

    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3503, 0x07);	//for lock AEC
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3406, 0x01);	// lock AWB

//continous stop stream ++		
		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3000 , &val);	
		val = val |0x2;
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3000, val);
    
//		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3003 , &val);	
//		val = val |0x2;
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3003, val);
//continous stop stream --
		
		
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3503, 0x07);	//for lock AEC
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3406, 0x01);	// lock AWB

    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3a00, 0x7C);	//2013/05/28

		err = ov5640_set_mode(info, &mode);
		
		ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3008, 0x02);	// release sw pwd down.

//continous stop stream ++		
//		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3003 , &val);	
//		val = val & 0xfd;
//    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3003, val);

		ov5640_read_reg(ov5640_sensor_info->i2c_client, 0x3000 , &val);	
		val = val & 0xfd;
    ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3000, val);
//continous stop stream --


   	msleep(300);

		return err;
	}
	case OV5640_IOCTL_GET_CONFIG:
	{
		if (copy_to_user((void __user *) arg,
				 &info->focuser,
				 sizeof(info->focuser))) {
			dev_err(info->dev, "%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}

		break;
	}
	case OV5640_IOCTL_GET_AF_STATUS:
	{
		int err;
		u8 val;

		if (!info->af_fw_loaded) {
			dev_err(info->dev, "OV5640 AF fw not loaded!\n");
			break;
		}

		err = ov5640_get_af_status(info, &val);
		if (err)
			return err;

		if (copy_to_user((void __user *) arg,
				 &val, sizeof(val))) {
			dev_err(info->dev, "%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
	case OV5640_IOCTL_SET_AF_MODE:
		if (!info->af_fw_loaded) {
			dev_err(info->dev, "OV5640 AF fw not loaded!\n");
			break;
		}
		return ov5640_set_af_mode(info, (u8)arg);
	case OV5640_IOCTL_POWER_LEVEL:
		return ov5640_set_power(info, (u32)arg);
	case OV5640_IOCTL_SET_FPOSITION:
		return ov5640_set_position(info, (u32)arg);
	case OV5640_IOCTL_GET_SENSOR_STATUS:
	{
		u8 status = 0;
		if (copy_to_user((void __user *)arg, &status,
				 1)) {
			dev_info(info->dev, "%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case OV5640_IOCTL_SET_WHITE_BALANCE:
	{
		u8 whitebalance;

		if (copy_from_user(&whitebalance,
					(const void __user *)arg,
					sizeof(whitebalance))) {
			return -EFAULT;
		}

		switch(whitebalance)
		{
			case YUV_Whitebalance_Auto:
				printk("enter auto white balance \n");
				err = ov5640_write_table(info, wb_auto, NULL, 0);
				break;
			case YUV_Whitebalance_Incandescent:
				printk("enter Incandescent white balance \n");
				err = ov5640_write_table(info, wb_incandescent, NULL, 0);
				break;
			case YUV_Whitebalance_Daylight:
				printk("enter Daylight white balance \n");
				err = ov5640_write_table(info, wb_daylight, NULL, 0);
				break;
			case YUV_Whitebalance_Fluorescent:
				printk("enter Fluorescent white balance \n");
				err = ov5640_write_table(info, wb_fluorescent, NULL, 0);
				break;
			case YUV_Whitebalance_CloudyDaylight:
				printk("enter CloudyDaylight white balance \n");
				err = ov5640_write_table(info, wb_cloudy, NULL, 0);
				break;
			default:
				break;
		}

		if (err)
			return err;

		return 0;
	}
	case OV5640_IOCTL_SET_SCENE_MODE:
	{
			return 0;
	}
  case OV5640_IOCTL_SET_EXPOSURE:
  {
    int exposure;

		if (copy_from_user(&exposure,
				   (const void __user *)arg,
				   sizeof(exposure))) {
			return -EFAULT;
		}

    switch(exposure)
    {
        case YUV_Exposure_0:
	     err = ov5640_write_table(info, Exposure_0, NULL, 0);
             break;
        case YUV_Exposure_1:
	     err = ov5640_write_table(info, Exposure_1, NULL, 0);
             break;
        case YUV_Exposure_2:
	     err = ov5640_write_table(info, Exposure_2, NULL, 0);
             break;
        case YUV_Exposure_Negative_1:
	     err = ov5640_write_table(info, Exposure_Negative_1, NULL, 0);
             break;
        case YUV_Exposure_Negative_2:
	     err = ov5640_write_table(info, Exposure_Negative_2, NULL, 0);
             break;
        default:
             break;
    }
	  if (err)
		   return err;

    return 0;
  }

	
	default:
		return -EINVAL;
	}
	return 0;
}

static int ov5640_open(struct inode *inode, struct file *file)
{
	pr_info("ov5640 %s \n",__func__);	

	file->private_data = ov5640_sensor_info;
	
	if (ov5640_sensor_info->pdata && ov5640_sensor_info->pdata->power_on)
	{
		ov5640_sensor_info->pdata->power_on();	
	}
	
	ov5640_sensor_info->preview_x = 0;
	ov5640_sensor_info->preview_y = 0;

//	init_check_ov5640 = 1;

	return 0;
}

int ov5640_release(struct inode *inode, struct file *file)
{
//	u8 val=0,i=0;
	pr_info("ov5640 %s\n", __func__);

	ov5640_sensor_info->preview_x = 0;
	ov5640_sensor_info->preview_y = 0;

  ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3022, 0x06); //release AF for noise issue.
  ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3603, 0x09); //release AF for noise issue.
  ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3602, 0x00); //release AF for noise issue.
  msleep(10);
  ov5640_write_reg(ov5640_sensor_info->i2c_client , 0x3022, 0x08); //release AF for noise issue.

	if (ov5640_sensor_info->pdata && ov5640_sensor_info->pdata->power_off)
		ov5640_sensor_info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations ov5640_fileops = {
	.owner = THIS_MODULE,
	.open = ov5640_open,
	.unlocked_ioctl = ov5640_ioctl,
	.release = ov5640_release,
};

static struct miscdevice ov5640_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov5640",
	.fops = &ov5640_fileops,
};

static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("ov5640 %s \n",__func__);
	
	dev_info(&client->dev, "ov5640: probing sensor.\n");

	ov5640_sensor_info = kzalloc(sizeof(struct ov5640_info), GFP_KERNEL);
	if (!ov5640_sensor_info) {
		pr_err("ov5640: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov5640_device);
	if (err) {
		pr_err("ov5640: Unable to register misc device!\n");
		kfree(ov5640_sensor_info);
		return err;
	}

	ov5640_sensor_info->dev = &client->dev;
	ov5640_sensor_info->pdata = client->dev.platform_data;
	ov5640_sensor_info->i2c_client = client;
	ov5640_sensor_info->focuser.settle_time = SETTLETIME_MS;
	ov5640_sensor_info->focuser.focal_length = FOCAL_LENGTH;
	ov5640_sensor_info->focuser.fnumber = FNUMBER;
	ov5640_sensor_info->focuser.pos_low = POS_LOW;
	ov5640_sensor_info->focuser.pos_high = POS_HIGH;
	ov5640_sensor_info->af_fw_loaded = 0;

	i2c_set_clientdata(client, ov5640_sensor_info);
	
//&*&*&*CJ1_20130429: add for IQ test
#ifdef CAMERA_DEBUG
	err = device_create_file(&client->dev, &dev_attr_camera_5mp_i2c_write);
	err = device_create_file(&client->dev, &dev_attr_camera_5mp_i2c_read);
	err = device_create_file(&client->dev, &dev_attr_camera_5mp_table);
#endif 
//&*&*&*CJ2_20130429: add for IQ test
	
	err = device_create_file(&client->dev, &dev_attr_diag_camera_5mp_power); //&*&*&*CJ_20130429: add for Daig VI test

	return 0;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov5640_device);
	kfree(info);
	return 0;
}

static int ov5640_suspend(struct i2c_client *client,pm_message_t state)
{
	
	ov5640_sensor_info->pdata->vcm_pwr_off();	
	msleep(2);
	ov5640_sensor_info->pdata->suspend();
	pr_info("ov5640 %s \n",__func__);	
	
	return 0;
}

static int ov5640_resume(struct i2c_client *client)
{
		
	ov5640_sensor_info->pdata->init();
	pr_info("ov5640 %s \n",__func__);
	
	
	
//	ov5640_sensor_info->pdata->vcm_pwr_off();	
//	msleep(2);
	
	ov5640_sensor_info->pdata->power_off();	
	
	ov5640_sensor_info->af_fw_loaded =0;

	init_check_ov5640 = 1;

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ "ov5640", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name = "ov5640",
		.owner = THIS_MODULE,
	},
	.probe = ov5640_probe,
	.remove = ov5640_remove,
	.id_table = ov5640_id,
	.suspend = ov5640_suspend,
	.resume = ov5640_resume,

};

static int __init ov5640_init(void)
{
	pr_info("ov5640 sensor driver loading\n");
	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit ov5640_exit(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_init);
module_exit(ov5640_exit);
