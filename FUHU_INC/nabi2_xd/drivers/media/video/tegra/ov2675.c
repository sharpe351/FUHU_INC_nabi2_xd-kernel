/*
 * ov2675.c - ov2675 sensor driver
 *
 * Copyright (c) 2011-2012, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      erik lilliebjerg <elilliebjerg@nvidia.com>
 *
 * Leverage ov2710.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#define DEBUG
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov2675.h>
#include <media/sensor_yuv.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include "ov2675_regs.h"
#define CAMERA_DEBUG

#define SENSOR_NAME "ov2675"

#define SIZEOF_I2C_TRANSBUF 64

struct ov2675_info {
	int mode;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	int preview_x;
	int preview_y;
};

static struct ov2675_info *ov2675_sensor_info;
int init_check_ov2675 = 1;

static int ov2675_read_reg(struct i2c_client *client, u16 reg, u8 *val)
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

static int ov2675_write_reg(struct i2c_client *client, u16 addr, u8 val)
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

static int ov2675_reg_poll16(struct i2c_client *client, u16 read_reg,
                              u8 mask, u8 val, int delay, int timeout)
{
	    u8 currval=0;
			while (timeout) 
			{
	
	        ov2675_read_reg(client, read_reg, &currval);
	        
					printk("[ov2675] %s, reg=0x%x, currval=0x%x, val=0x%x \n",	__func__,read_reg,currval,val);
	
	        if ((currval & mask) == val)
	                return 0;
	
	        msleep(delay);
	        timeout--;
			}
	
	    pr_err("[err](%s) read_reg=0x%x, currval=0x%x, val=0x%x \n",__FUNCTION__,read_reg,currval,val);
	
	return -ETIMEDOUT;

}

static int ov2675_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
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

	dev_err(&client->dev, "ov2675: i2c transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov2675_write_table(struct ov2675_info *info,
			      struct ov2675_reg table[],
			      struct ov2675_reg override_list[],
			      int num_override_regs)
{
	int err;
	struct ov2675_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	int i;
	u16 val;

	for (next = table; next->addr != SENSOR_TABLE_END; next++) {
		if (next->addr == SENSOR_WAIT_MS) {
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
		if (n_next->addr != SENSOR_TABLE_END &&
			n_next->addr != SENSOR_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov2675_write_bulk_reg(info->i2c_client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}

#ifdef CAMERA_DEBUG

static int jj_table_size(char *firmware_name)
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

static int jj_table_cmd(char *filename)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned char *buf;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	int fwsize = jj_table_size(filename);
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
				 ov2675_write_reg(ov2675_sensor_info->i2c_client , addr, val);
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

///echo "0x1234,0x0000," > sys/devices/platform/tegra-i2c.2/i2c-2/2-0030/camera_i2c_write
static ssize_t jj_camera_i2c_write(struct device *dev,
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
	ov2675_write_reg(ov2675_sensor_info->i2c_client , addr, val);
		
	return count;
}
static DEVICE_ATTR(camera_i2c_write, 0664, NULL, jj_camera_i2c_write);

///echo "0x0F12," > sys/devices/platform/tegra-i2c.2/i2c-2/2-0030/camera_i2c_read
static ssize_t jj_camera_i2c_read(struct device *dev,
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
	
	ov2675_read_reg(ov2675_sensor_info->i2c_client, addr, &val);
	
	printk("(%s)ov2675_read_reg(addr=0x%x, val=[0x%x])  \n", __FUNCTION__,addr,val);	
		
	return count;
}
static DEVICE_ATTR(camera_i2c_read, 0664, NULL, jj_camera_i2c_read);

//echo "/sdcard/data/xxx.txt" > /sys/devices/platform/tegra-i2c.2/i2c-2/2-0030/camera_table				
static ssize_t jj_camera_table(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	
	if(buf != NULL)
	{
		char filename[128];
			
		sprintf(filename, "%s", buf);
	
		//printk("[camera]Enter %s, buf is %s, count is %d,table_name=%s \n", __FUNCTION__,buf, count,filename);
		*(filename+count-1) = '\0';
		jj_table_cmd(filename);
	}
	else
	{
		//return -1;
		char filename[]="/mnt/sdcard/xxx.txt";
		jj_table_cmd(filename);
	}
		
	
	return count;
}
static DEVICE_ATTR(camera_table, 0664, NULL, jj_camera_table);			
#endif

//+jimmychi
static ssize_t diag_camera_power(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{	
	
	u16 power=0;
	
	power = simple_strtoul(buf, NULL, 10);

	if(power==0)//power off
	{
			ov2675_sensor_info->pdata->power_off();
			ov2675_sensor_info->pdata->suspend();
	}
	else if(power==1)//power on
	{
			ov2675_sensor_info->pdata->init();
			ov2675_sensor_info->pdata->power_on();
	}
	else //error command
	{	
	    printk("[%s][%d]error:power=%d \n",
			__func__, __LINE__, power);
	}
	
	return count;
}
static DEVICE_ATTR(diag_camera_power, 0664, NULL, diag_camera_power);
//-jimmychi

static int ov2675_set_mode(struct ov2675_info *info, struct sensor_mode *mode)
{
	
	if (mode->xres <= 800 && mode->yres <=600) // preview size
	{
			ov2675_write_table(info, SVGA_list, NULL, 0);
	    printk("[ov2675][preview size] %s: xres=%u, yres=%u \n",
			__func__, mode->xres, mode->yres);

	}
	else if(mode->xres==1280&&mode->yres==720)
	{
			ov2675_write_table(info, video_list, NULL, 0);
	    printk("[ov2675][video size] %s: xres=%u, yres=%u \n",
			__func__, mode->xres, mode->yres);
			
	}
	else	//video and still size
	{
			ov2675_write_table(info, UXVGA_list, NULL, 0);
	    printk("[ov2675][still size] %s: xres=%u, yres=%u \n",
			__func__, mode->xres, mode->yres);
	}

		ov2675_write_table(info, mirror_flip_off, NULL, 0);
		
	return 0;
}

u8 REG_3000,REG_3001,REG_3002,REG_3003,REG_301B,REG_3013,REG_3014;
static int ov2675_set_preview(struct ov2675_info *info, struct sensor_mode *mode)
{
//	int i;
//	u16 ov2675reg []= {0x3013,0x3014,0x3016,0x3300,0x3002,0x3003,0x3000,0x3001,0x302d,0x302e,
//	0x3070,0x3071,0x3072,0x3073,0x301c,0x301d};
//	u8 val=0;

#ifdef CAMERA_DEBUG
	char filename[]="/data/iq.txt";
#endif
		
	{
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3000, &REG_3000);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3001, &REG_3001);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3002, &REG_3002);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3003, &REG_3003);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x301b, &REG_301B);		
		printk("(%s +++++)REG_3000=0x%x, REG_3001=0x%x, REG_3002=0x%x, REG_3003=0x%x, REG_301B=0x%x \n",
		__FUNCTION__,REG_3000,REG_3001,REG_3002,REG_3003,REG_301B);
	}
	
	{
		ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3000, REG_3000);
		ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3001, REG_3001);
		ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3002, REG_3002);
		ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3003, REG_3003);
//	ov2675_write_table(ov2675_sensor_info->i2c_client, exposure_release, NULL, 0);
	}
	
		  ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3300, 0xfc);	//release AWB 2013/05/30
//	if(!((ov2675_sensor_info->preview_x == mode->xres)&&
//		(ov2675_sensor_info->preview_y = mode->yres)))
	{
			if (mode->xres <= 800 && mode->yres <= 600) // preview size
			{
					ov2675_write_table(info, SVGA_list, NULL, 0);
			    printk("[ov2675][preview size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
  		
			}
			else if(mode->xres==1280&&mode->yres==720)
			{
					ov2675_write_table(info, video_list, NULL, 0);
			    printk("[ov2675][video size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
					
			}
			else	//video and still size
			{
					ov2675_write_table(info, UXVGA_list, NULL, 0);
			    printk("[ov2675][still size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
			}
	}
	
			ov2675_write_table(info, mirror_flip_off, NULL, 0);
		
//		ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3017, 0x60);
		ov2675_write_table(info, exposure_release, NULL, 0);
        msleep(200); //&*&*&*cj_20130301, fix-Camera : #redmine-bug39 [Camrea] DUT preview windows will show crash screen immediately.
        						 

	ov2675_sensor_info->preview_x = mode->xres;
	ov2675_sensor_info->preview_y = mode->yres;
	
	{
		//ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x301b, REG_301B);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3000, &REG_3000);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3001, &REG_3001);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3002, &REG_3002);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3003, &REG_3003);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x301b, &REG_301B);		
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3013, &REG_3013);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3014, &REG_3014);		
		printk("(%s -----)REG_3000=0x%x, REG_3001=0x%x, REG_3002=0x%x, REG_3003=0x%x, REG_301B=0x%x, REG_3013=0x%x, REG_3014=0x%x \n",
		__FUNCTION__,REG_3000,REG_3001,REG_3002,REG_3003,REG_301B,REG_3013,REG_3014);
	}

//      	for(i=0;i<ARRAY_SIZE(ov2675reg);i++)
//      	{
//      		ov2675_read_reg(ov2675_sensor_info->i2c_client, ov2675reg[i], &val);
//      //		printk("(%s)ov2675_read_reg(addr=0x%x, val=[0x%x])  \n", __FUNCTION__,ov2675reg[i],val);	
//      		printk("(addr=0x%x, val=[0x%x])  \n",ov2675reg[i],val);	
//      	}	
	
#ifdef CAMERA_DEBUG
	jj_table_cmd(filename);
#endif	
		
	return 0;
}

static int ov2675_set_capture(struct ov2675_info *info, struct sensor_mode *mode)
{
//	int i;
//	u16 ov2675reg []= {0x3013,0x3014,0x3016,0x3300,0x3002,0x3003,0x3000,0x3001,0x302d,0x302e,
//	0x3070,0x3071,0x3072,0x3073,0x301c,0x301d};
//	u8 val=0;

#ifdef CAMERA_DEBUG
	char filename[]="/data/iq.txt";
#endif
	
	{
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3000, &REG_3000);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3001, &REG_3001);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3002, &REG_3002);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3003, &REG_3003);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x301b, &REG_301B);		
		printk("(%s +++++)REG_3000=0x%x, REG_3001=0x%x, REG_3002=0x%x, REG_3003=0x%x, REG_301B=0x%x \n",
		__FUNCTION__,REG_3000,REG_3001,REG_3002,REG_3003,REG_301B);
	}
	
	if(!((ov2675_sensor_info->preview_x == mode->xres)&&
		(ov2675_sensor_info->preview_y = mode->yres)))
	{
			ov2675_write_table(info, exposure_fixed, NULL, 0);
		  ov2675_write_reg(ov2675_sensor_info->i2c_client , 0x3300, 0xdc);	//lock AWB 2013/05/30
			
			if (mode->xres <= 800 && mode->yres <= 600) // preview size
			{
					ov2675_write_table(info, SVGA_list, NULL, 0);
			    printk("[ov2675][preview size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
  		
			}
			else if(mode->xres==1280&&mode->yres==720)
			{
					ov2675_write_table(info, video_list, NULL, 0);
			    printk("[ov2675][video size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
					
			}
			else	//video and still size
			{
					ov2675_write_table(info, UXVGA_list, NULL, 0);
			    printk("[ov2675][still size] %s: xres=%u, yres=%u \n",
					__func__, mode->xres, mode->yres);
			}
			msleep(200);
	}
	
		ov2675_write_table(info, mirror_flip_off, NULL, 0);

#ifdef CAMERA_DEBUG	
		jj_table_cmd(filename);
#endif		
		
	{
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3000, &REG_3000);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3001, &REG_3001);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3002, &REG_3002);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3003, &REG_3003);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x301b, &REG_301B);		
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3013, &REG_3013);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3014, &REG_3014);		
		printk("(%s -----)REG_3000=0x%x, REG_3001=0x%x, REG_3002=0x%x, REG_3003=0x%x, REG_301B=0x%x, REG_3013=0x%x, REG_3014=0x%x \n",
		__FUNCTION__,REG_3000,REG_3001,REG_3002,REG_3003,REG_301B,REG_3013,REG_3014);
	}


//      	for(i=0;i<ARRAY_SIZE(ov2675reg);i++)
//      	{
//      		ov2675_read_reg(ov2675_sensor_info->i2c_client, ov2675reg[i], &val);
//      //		printk("(%s)ov2675_read_reg(addr=0x%x, val=[0x%x])  \n", __FUNCTION__,ov2675reg[i],val);	
//      		printk("(addr=0x%x, val=[0x%x])  \n",ov2675reg[i],val);	
//      	}	
	
			return 0;
}

#define INFO_PID_H 		0x300A
#define INFO_PID_L 		0x300B
#define INFO_OTP 		  0x308F

#define INFO_PID_H_value	0x26
#define INFO_PID_L_value	0x56
#define INFO_OTP_value	  0x80


static long ov2675_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int err=0;
	struct ov2675_info *info = file->private_data;
	int poll_timeout;
	
	printk("[ov2675] %s, cmd=0x%x \n",	__func__,cmd);
	
	if(init_check_ov2675)
	{
		poll_timeout = ov2675_reg_poll16(info->i2c_client, INFO_PID_H,		0xFF, INFO_PID_H_value, 5, 10);
		if(poll_timeout)
		{
				printk("[ov2675] INFO_PID_H poll_timeout=%d \n",poll_timeout);
		}
		
		poll_timeout = ov2675_reg_poll16(info->i2c_client, INFO_PID_L,		0xFF, INFO_PID_L_value, 5, 10);
		if(poll_timeout)
		{
				printk("[ov2675] INFO_PID_L poll_timeout=%d \n",poll_timeout);
		}

		ov2675_write_table(info, initial_list, NULL, 0);
		
		ov2675_write_table(info, mirror_flip_off, NULL, 0);
	  
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3000, &REG_3000);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3001, &REG_3001);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3002, &REG_3002);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x3003, &REG_3003);
		ov2675_read_reg(ov2675_sensor_info->i2c_client, 0x301b, &REG_301B);
	
		init_check_ov2675 = 0;	
	}
		
	pr_info("ov2675 %s:cmd=0x%x \n",__func__,cmd);
	
	switch (cmd) {
		
		case SENSOR_IOCTL_SET_MODE:
			{
				struct sensor_mode mode;
				if (copy_from_user(&mode,
							(const void __user *)arg,
							sizeof(struct sensor_mode))) {
					return -EFAULT;
				}

				return ov2675_set_mode(info, &mode);
			}
		case SENSOR_IOCTL_SET_CAPTURE:
			{
				struct sensor_mode mode;
				if (copy_from_user(&mode,
							(const void __user *)arg,
							sizeof(struct sensor_mode))) {
					return -EFAULT;
				}

				return ov2675_set_capture(info, &mode);
			}
		case SENSOR_IOCTL_SET_PREVIEW:
			{
				struct sensor_mode mode;
				if (copy_from_user(&mode,
							(const void __user *)arg,
							sizeof(struct sensor_mode))) {
					return -EFAULT;
				}

				return ov2675_set_preview(info, &mode);
			}
		case SENSOR_IOCTL_GET_STATUS:
			{
				return 0;
			}
#if 1
		case SENSOR_IOCTL_GET_AF_STATUS:
			{
				return 0;
			}
		case SENSOR_IOCTL_SET_AF_MODE:
			{
				return 0;
			}
		case SENSOR_IOCTL_SET_COLOR_EFFECT:
			{
				return 0;
			}
		case SENSOR_IOCTL_SET_WHITE_BALANCE:
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
						err = ov2675_write_table(info, wb_auto, NULL, 0);
						break;
					case YUV_Whitebalance_Incandescent:
						printk("enter Incandescent white balance \n");
						err = ov2675_write_table(info, wb_incandescent, NULL, 0);
						break;
					case YUV_Whitebalance_Daylight:
						printk("enter Daylight white balance \n");
						err = ov2675_write_table(info, wb_daylight, NULL, 0);
						break;
					case YUV_Whitebalance_Fluorescent:
						printk("enter Fluorescent white balance \n");
						err = ov2675_write_table(info, wb_fluorescent, NULL, 0);
						break;
					case YUV_Whitebalance_CloudyDaylight:
						printk("enter CloudyDaylight white balance \n");
						err = ov2675_write_table(info, wb_cloudy, NULL, 0);
						break;
					default:
						break;
				}

				if (err)
					return err;

				return 0;
			}
		case SENSOR_IOCTL_SET_SCENE_MODE:
			{
				return 0;
			}
#endif
        case SENSOR_IOCTL_SET_EXPOSURE:
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
	                 err = ov2675_write_table(info, Exposure_0, NULL, 0);
                         break;
                    case YUV_Exposure_1:
	                 err = ov2675_write_table(info, Exposure_1, NULL, 0);
                         break;
                    case YUV_Exposure_2:
	                 err = ov2675_write_table(info, Exposure_2, NULL, 0);
                         break;
                    case YUV_Exposure_Negative_1:
	                 err = ov2675_write_table(info, Exposure_Negative_1, NULL, 0);
                         break;
                    case YUV_Exposure_Negative_2:
	                 err = ov2675_write_table(info, Exposure_Negative_2, NULL, 0);
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


static int ov2675_open(struct inode *inode, struct file *file)
{
	pr_info("ov2675 %s \n",__func__);	

	file->private_data = ov2675_sensor_info;
	
	if (ov2675_sensor_info->pdata && ov2675_sensor_info->pdata->power_on)
	{
		ov2675_sensor_info->pdata->power_on();	
	}
	
	ov2675_sensor_info->preview_x = 0;
	ov2675_sensor_info->preview_y = 0;

//	init_check_ov2675 = 1;

	
	return 0;
}

int ov2675_release(struct inode *inode, struct file *file)
{
	pr_info("ov2675 %s \n",__func__);
	
	ov2675_sensor_info->preview_x = 0;
	ov2675_sensor_info->preview_y = 0;
	
	if (ov2675_sensor_info->pdata && ov2675_sensor_info->pdata->power_off)
		ov2675_sensor_info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static int ov2675_suspend(struct i2c_client *client,pm_message_t state)
{
	
	ov2675_sensor_info->pdata->suspend();
	pr_info("ov2675 %s \n",__func__);	
	
	return 0;
}

static int ov2675_resume(struct i2c_client *client)
{
		
	ov2675_sensor_info->pdata->init();
	pr_info("ov2675 %s \n",__func__);
	
	msleep(2);
	
	ov2675_sensor_info->pdata->power_off();	

	init_check_ov2675 = 1;

	return 0;
}

static const struct file_operations ov2675_fileops = {
	.owner = THIS_MODULE,
	.open = ov2675_open,
	.unlocked_ioctl = ov2675_ioctl,
	.release = ov2675_release,
};

static struct miscdevice ov2675_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov2675",
	.fops = &ov2675_fileops,
};

static int ov2675_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;
	
	pr_info("ov2675 %s \n",__func__);

	ov2675_sensor_info = kzalloc(sizeof(struct ov2675_info), GFP_KERNEL);
	if (!ov2675_sensor_info) {
		pr_err("ov2675: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov2675_device);
	if (err) {
		pr_err("ov2675: Unable to register misc device!\n");
		kfree(ov2675_sensor_info);
		return err;
	}
	ov2675_sensor_info->pdata = client->dev.platform_data;
	ov2675_sensor_info->i2c_client = client;

	i2c_set_clientdata(client, ov2675_sensor_info);
	
#ifdef CAMERA_DEBUG
	err = device_create_file(&client->dev, &dev_attr_camera_i2c_write);
	err = device_create_file(&client->dev, &dev_attr_camera_i2c_read);
	err = device_create_file(&client->dev, &dev_attr_camera_table);
#endif 
	err = device_create_file(&client->dev, &dev_attr_diag_camera_power); //+jimmychi	

	return err;	
}

static int ov2675_remove(struct i2c_client *client)
{
	struct ov2675_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov2675_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov2675_id[] = {
	{ "ov2675", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov2675_id);

static struct i2c_driver ov2675_i2c_driver = {
	.driver = {
		.name = "ov2675",
		.owner = THIS_MODULE,
	},
	.probe = ov2675_probe,
	.remove = ov2675_remove,
	.id_table = ov2675_id,
	.suspend = ov2675_suspend,
	.resume = ov2675_resume,
};

static int __init ov2675_init(void)
{
	pr_info("ov2675 sensor driver loading\n");
	return i2c_add_driver(&ov2675_i2c_driver);
}

static void __exit ov2675_exit(void)
{
	i2c_del_driver(&ov2675_i2c_driver);
}

module_init(ov2675_init);
module_exit(ov2675_exit);

