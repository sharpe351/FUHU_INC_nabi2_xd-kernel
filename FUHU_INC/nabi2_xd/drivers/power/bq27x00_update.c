/*
 * BQ27x00 gauge update driver
 *
 *
 * Copyright (C) 2013 Aimar Liu <aimar.ts.liu@foxconn.com>
 *
 *
 * Based on bq27541 EVM sample work by Copyright (C) 2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include "bq27x00_update.h"

// bq27425-g2 definition
#define BQ27425_CNTL_STATUS_SS		         BIT(5)
#define BQ27425_FLAGS_CFGUPMODE		         BIT(4)
#define BQ27425_FLAGS_ITPOR		             BIT(5)

#define BQ27425_CMD_CNTL                     0x00
#define BQ27425_CMD_FLAGS                    0x06
#define BQ27425_CMD_OPERATION_CONFIGURATION  0x3a
#define BQ27425_CMD_DESIGN_CAPACITY          0x3c
#define bq27425_ECMD_DFCLS                   0x3e
#define bq27425_ECMD_DFBLK                   0x3f
#define bq27425_ECMD_DFD                     0x40
#define bq27425_ECMD_DFDCKS                  0x60
#define bq27425_ECMD_DFDCNTL                 0x61

#define BQ27425_SUBCLASS_SAFETY              0x02
#define BQ27425_SUBCLASS_CHARGE_TERMINATION  0x24
#define BQ27425_SUBCLASS_DISCHARGE           0x31
#define BQ27425_SUBCLASS_CURRENT_THRESHOLD   0x51
#define BQ27425_SUBCLASS_STATE               0x52

#define BQ27425_CNTL_STATUS         0x0000
#define BQ27425_CNTL_SET_CFGUPDATE  0x0013
#define BQ27425_CNTL_SEALED         0x0020
#define BQ27425_CNTL_FORCE_RESET    0x0041
#define BQ27425_CNTL_SOFT_RESET     0x0042
// end

#define BQ27x00_BUFFERSIZE          32

static DEFINE_MUTEX(bq27x00_update_mutex);

static unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
  unsigned int tmp;
  
  tmp = ((msb << 8) & 0xFF00);
  return ((unsigned int)(tmp + lsb) & 0x0000FFFF);  
}

static void transUnsignedInt2Bytes(unsigned int data, unsigned char* bytes)
{
  
  bytes[1] = (unsigned char)((data & 0xFF00) >> 8);
  bytes[0] = (unsigned char)(data & 0x00FF);
  
  return;
}

static int bq27x00_write_code(struct i2c_client *client, u8 reg, u16 wt_value, bool single)
{
	struct i2c_msg msg[1];
	unsigned char data[3];
	int err;
	
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = data;

	data[0] = reg;
	data[1] = wt_value & 0xff;

	if (single) {
		msg->len = 2;
	} else {
		data[2] = (wt_value >> 8) & 0xff;
		msg->len = 3;
	}
	
	err = i2c_transfer(client->adapter, msg, 1);
	if (err == ARRAY_SIZE(msg))
		return 0;

	return (err < 0) ? err : -EIO;
}

static int bq27x00_read_code(struct i2c_client *client, u8 reg, bool single)
{
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		//ret = get_unaligned_le16(data);
		ret = transBytes2UnsignedInt(data[1], data[0]);
	else
		ret = data[0];

	return ret;
}

static int bq27x00_read_data(struct i2c_client *client, u8 reg, char *rd_data, int bytes)
{
	struct i2c_msg msg[2];
	int ret;
	
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = rd_data;
	msg[1].len = bytes;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	if (ret == ARRAY_SIZE(msg))
		return 0;

	return (ret < 0) ? ret : -EIO;
}

static int bq27x00_write_data(struct i2c_client *client, u8 reg, char *wt_data, int bytes)
{
	struct i2c_msg msg[1];
	unsigned char data[bytes + 1];
	int err;
	int i;
	
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = bytes+1;
	msg->buf = data;

	data[0] = reg;
	for(i=1;i<(bytes+1);i++)
		data[i] = wt_data[i-1];	
	
	err = i2c_transfer(client->adapter, msg, 1);
	
	if (err == ARRAY_SIZE(msg))
		return 0;

	return (err < 0) ? err : -EIO;
}

static int bq27x00_read_block_data(struct i2c_client *client, u8 subclass, int offset, unsigned char *blockdata)
{
	unsigned char RxData[BQ27x00_BUFFERSIZE];
	int numBlock, ret;

	numBlock = offset/BQ27x00_BUFFERSIZE;

	// Write & Read Block Data (32 bytes of data) in Manufacturer Info Block X
	ret = bq27x00_write_code(client, bq27425_ECMD_DFDCNTL, 0x00, true); // BlockDataControl() = 0x00
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write DFDCNTL(0x61) failure\n" ,__func__);
		return ret;
	}
  	msleep(100);
	ret = bq27x00_write_code(client, bq27425_ECMD_DFCLS, subclass, true); // Write the subclass value
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write DFCLS(0x3e) failure\n" ,__func__);
		return ret;
	}
  	msleep(100);
	ret = bq27x00_write_code(client, bq27425_ECMD_DFBLK, numBlock, true); // Select offset within the flash  
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write DFBLK(0x3f) failure\n" ,__func__);
		return ret;
	}
  	msleep(1000);

	ret = bq27x00_read_data(client, bq27425_ECMD_DFD, RxData, BQ27x00_BUFFERSIZE); 
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read DFD(0x40) failure\n" ,__func__);
		return ret;
	}

	memcpy(blockdata, RxData, BQ27x00_BUFFERSIZE);

	return 0;
}

static int bq27x00_write_block_data(struct i2c_client *client, u8 subclass, int offset, int wt_data, int data_len)
{
	unsigned char rData[BQ27x00_BUFFERSIZE];
	unsigned char wData[2];
	int val, ret, i, checksum;
	int sum = 0;
	
	val = bq27x00_read_block_data(client, subclass, offset, rData);
	if (val < 0)  
		dev_err(&client->dev, "(%s): read block data [subclass:%d][offset:%d]failure\n", __func__, subclass, offset);

	if(data_len == 2)
	{
		transUnsignedInt2Bytes(wt_data, wData); 
		rData[(offset%BQ27x00_BUFFERSIZE) + 1] = wData[0];
		rData[offset%BQ27x00_BUFFERSIZE] = wData[1];
	}
	else if(data_len == 1)
	{
		rData[offset%BQ27x00_BUFFERSIZE] = (unsigned char)wt_data;
	}
	else
		return -EINVAL;

	ret = bq27x00_write_data(client, bq27425_ECMD_DFD, rData, BQ27x00_BUFFERSIZE);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)write DFD(0x40) failure\n" ,__func__);
		return ret;
	}

	for (i = 0; i < BQ27x00_BUFFERSIZE; i++)          // Compute the checksum of the block
  	{
    	sum += rData[i];                       // Calculate the sum of the values  
  	}
	checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
	dev_info(&client->dev, "(%s)checksum :0x%02x\n", __func__, checksum);

	ret = bq27x00_write_code(client, bq27425_ECMD_DFDCKS, checksum, true); // Write checksum value
	if(ret < 0) {
		dev_err(&client->dev, "(%s):write DFDCKS(0x60) failure\n" ,__func__);
		return ret;
	}
  	msleep(1000);

	return ret;
}

static int enter_config_update_mode(struct i2c_client *client, bool enter)
{	
	int ret, status;

	if(!client)
		return -ENODEV;

	if(enter){
		ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_SET_CFGUPDATE, false);
		if(ret < 0) {
			dev_err(&client->dev, "(%s):write SET_CFGUPDATE(0x0013) failure\n" ,__func__);
			return ret;
		}
	}
	else{
		ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_SOFT_RESET, false);
		if(ret < 0) {
			dev_err(&client->dev, "(%s):write SOFT_RESET(0x0042) failure\n" ,__func__);
			return ret;
		}
	}
	msleep(1000);  // need to wait 1 sec

	status = bq27x00_read_code(client, BQ27425_CMD_FLAGS, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read CMD_FLAGS(0x06) failure\n" ,__func__);
		return ret;
	}

	ret = (status & 0x00FF) & BQ27425_FLAGS_CFGUPMODE;

	if(ret)
		dev_info(&client->dev, "(%s)Enter Config Mode :0x%04x\n" ,__func__, status);
	else
		dev_info(&client->dev, "(%s)Leave Config Mode :0x%04x\n" ,__func__, status);

	return ret;
}

static int place_to_sealed(struct i2c_client *client)
{
	int ret;
		
	if(!client)
		return -ENODEV;

	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_SEALED, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write SEALED(0x0020) failure\n" ,__func__);
		return ret;
	}

	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_STATUS, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write control_status(0x0000) failure\n" ,__func__);
		return ret;
	}
	ret = bq27x00_read_code(client, BQ27425_CMD_CNTL, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read control_status(0x00) failure\n" ,__func__);
		return ret;
	}

	if((ret >> 8) & BQ27425_CNTL_STATUS_SS)
		return 0;

	return -1;
}

static int enable_to_unsealed(struct i2c_client *client)
{
	int ret;
		
	if(!client)
		return -ENODEV;
	
	/* unseal key */
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x0414, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x0414 failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x3672, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x3672 failure\n" ,__func__);
		return ret;
	}
	msleep(100);

	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_STATUS, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write control_status(0x0000) failure\n" ,__func__);
		return ret;
	}
	ret = bq27x00_read_code(client, BQ27425_CMD_CNTL, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read control_status failure\n" ,__func__);
		return ret;
	}

	if(!((ret >> 8) & BQ27425_CNTL_STATUS_SS))
		return 0;
	
	return -1;
}

static int enter_to_full_unsealed(struct i2c_client *client)
{
	int ret = 0;
	/* ;--------------------------------------------------------
	* ;Unseal device
	* ;--------------------------------------------------------*/
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x0414, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x0414 failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x3672, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x3672 failure\n" ,__func__);
		return ret;
	}
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0xFFFF, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0xFFFF failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0xFFFF, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0xFFFF failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	msleep(1000);
	
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, BQ27425_CNTL_STATUS, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write control_status(0x0000) failure\n" ,__func__);
		return ret;
	}
	ret = bq27x00_read_code(client, BQ27425_CMD_CNTL, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read control_status failure\n" ,__func__);
		return ret;
	}

	if(!((ret >> 8) & BQ27425_CNTL_STATUS_SS))
		return 0;
	
	return -1;
}

static int bq27x00_force_reset(struct bq27x00_update_access_resource *access_resource)
{
	struct i2c_client *client = access_resource->client;
	int ret, i, check;
	int timeout = 3;

	//step1. read all registers of control register
	for(i = 0x00; i<=0x3c; i+=2)
	{
		ret = bq27x00_read_code(client, i, false);
		dev_info(&client->dev, "[REG-0x%02x] is 0x%04x\n", i, ret);
	}

	//step2. unsealed
	ret = enable_to_unsealed(client);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): place unsealed failure\n", __func__);
		return -EPERM;
	}

	do {
		//step3. force reset
		ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x0041, false);
		if(ret < 0) {
			dev_err(&client->dev, "(%s): force reset failure\n", __func__);
			return -EIO;
		}
		msleep(6000);
		ret = bq27x00_read_code(client, BQ27425_CMD_FLAGS, false);
		if (ret < 0) {
			dev_err(&client->dev, "(%s)read CMD_FLAGS(0x06) failure\n" ,__func__);
			return ret;
		}
		check = (ret & 0x00FF) & BQ27425_FLAGS_ITPOR;
		if(check)
			dev_info(&client->dev, "(%s)RESET or POR \n" ,__func__);
		else
			dev_info(&client->dev, "(%s)Force Reset again :0x%04x\n" ,__func__, ret);
	} while( !check && --timeout);

	return check; // return 1 if success
}

static int bq27x00_read_resource(struct bq27x00_update_access_resource *access_resource, struct bq27x00_block_data_info *bd_info)
{
	struct i2c_client *client = access_resource->client;
	struct bq27x00_block_data_info *info = bd_info;
	unsigned char rData[BQ27x00_BUFFERSIZE];
	int ret;

	ret = enable_to_unsealed(client);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): place unsealed failure\n", __func__);
		return -EPERM;
	}
	
	mutex_lock(&bq27x00_update_mutex);
	ret = bq27x00_read_block_data(client, info->subclass, info->offset, rData);	
	if (ret < 0) {
		dev_err(&client->dev, "(%s): read block data [subclass:%d][offset:%d]failure\n", __func__, info->subclass, info->offset);
		goto err;
	}
	if(info->length == 2)
		info->data = transBytes2UnsignedInt(rData[info->offset%BQ27x00_BUFFERSIZE], rData[(info->offset%BQ27x00_BUFFERSIZE)+1]);
	else
		info->data = rData[info->offset%BQ27x00_BUFFERSIZE];
	
err:
	mutex_unlock(&bq27x00_update_mutex);
	place_to_sealed(client);
	
	return ret;
}

static int bq27x00_write_resource(struct bq27x00_update_access_resource *access_resource, struct bq27x00_block_data_info *bd_info)
{
	struct i2c_client *client = access_resource->client;
	struct bq27x00_block_data_info *info = bd_info;
	int ret;
	int timeout = 3;

	ret = enable_to_unsealed(client);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): place unsealed failure\n", __func__);
		return -EPERM;
	}
	
	mutex_lock(&bq27x00_update_mutex);
	ret = enter_config_update_mode(client, 1);
	if(ret <= 0)
		goto err_update_mode;
	ret = bq27x00_write_block_data(client, info->subclass, info->offset, info->data, info->length);	
	if (ret < 0) {
		dev_err(&client->dev, "(%s): write block data [subclass:%d][offset:%d]failure\n", __func__, info->subclass, info->offset);
		goto err;
	}

	do {
		ret = enter_config_update_mode(client, 0);
	} while(ret && --timeout);

err:
err_update_mode:	
	mutex_unlock(&bq27x00_update_mutex);
	place_to_sealed(client);

	if (timeout == 0)
		return -EIO;
	
	return ret;
}

static int bq27x00_write_operation_configuration(struct bq27x00_update_access_resource *access_resource, u16 config)
{
	struct i2c_client *client = access_resource->client;
	int ret;
	
	ret = enable_to_unsealed(client);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): place unsealed failure\n", __func__);
		return -EPERM;
	}

	mutex_lock(&bq27x00_update_mutex);
	ret = bq27x00_write_code(client, BQ27425_CMD_OPERATION_CONFIGURATION, config, false);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): write OPERATION_CONFIGURATION failure\n", __func__);
	}

	mutex_unlock(&bq27x00_update_mutex);
	place_to_sealed(client);

	return ret;
}

static int bq27x00_read_operation_configuration(struct bq27x00_update_access_resource *access_resource)
{
	struct i2c_client *client = access_resource->client;
	int ret;

	ret = bq27x00_read_code(client, BQ27425_CMD_OPERATION_CONFIGURATION, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read CMD_OPERATION_CONFIGURATION(0x3a) failure\n" ,__func__);
		return ret;
	}
	
	dev_info(&client->dev, "(%s)Operation Configuration :0x%04x\n" ,__func__, ret);
	
	return ret;
}

static ssize_t bq27x00_read_bd_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27x00_update_access_resource *access_resource;
	struct bq27x00_block_data_info *info;
	int num = 1;
	unsigned long val;
	char *tok;
	int error;

	access_resource = kzalloc(sizeof(*access_resource), GFP_KERNEL);
	if (!access_resource) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_update_access_resource data\n", __func__);
		return -ENOMEM;
	}
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_block_data_info data\n", __func__);
		error = -ENOMEM;
		goto err_alloc;
	}
	
	access_resource->client = client;
	/* echo [subID],[offset],[length] > read_bd*/
	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = kstrtoul(tok, 10, &val);
		if (error)
			break;
		if(num == 1)
			info->subclass = val;
		else if(num == 2)
			info->offset = val;
		else if(num == 3)
			info->length = val;
		num++;
	}
	if(num != 4)
	{
		dev_err(&client->dev, "(%s)the number of parameters not match.\n", __func__);
		goto err;
	}

	error = bq27x00_read_resource(access_resource, info);
	if(error < 0)
	{
		dev_err(&client->dev, "(%s)bq27x00_read_resource error.\n", __func__);
		error = -EPERM;
		goto err;
	}

	dev_info(&client->dev, "(%s)[subclass:%d][offset:%d] :%d\n" ,__func__, info->subclass, info->offset, info->data);

err:
	kfree(info);
err_alloc:
	kfree(access_resource);
	return count;
}
static DEVICE_ATTR(read_bd, 0664, NULL, bq27x00_read_bd_store);

static ssize_t bq27x00_write_bd_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27x00_update_access_resource *access_resource;
	struct bq27x00_block_data_info *info;
	int num = 1;
	unsigned long val;
	char *tok;
	int error;
	
	access_resource = kzalloc(sizeof(*access_resource), GFP_KERNEL);
	if (!access_resource) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_update_access_resource data\n", __func__);
		return -ENOMEM;
	}
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_block_data_info data\n", __func__);
		error = -ENOMEM;
		goto err_alloc;
	}
	access_resource->client = client;
	/* echo [subID],[offset],[data],[writebyte] > blockdata */
	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = kstrtoul(tok, 10, &val);
		if (error)
			break;
		if(num == 1)
			info->subclass = val;
		else if(num == 2)
			info->offset = val;
		else if(num == 3)
			info->data = val;
		else if(num == 4)
			info->length = val;
		num++;
	}
	if(num != 5)
	{
		printk("[bq27x00 gauge]the number of parameters not match.\n");
		goto err;
	}

	error = bq27x00_write_resource(access_resource, info);
	if(error < 0)
	{
		dev_err(&client->dev, "(%s)bq27x00_write_resource error.\n", __func__);
		error = -EPERM;
	}

err:
	kfree(info);
err_alloc:
	kfree(access_resource);
	return count;
}
static DEVICE_ATTR(write_bd, 0664, NULL, bq27x00_write_bd_store);

static ssize_t bq27x00_force_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27x00_update_access_resource *access_resource;
	unsigned long val;
	int ret;
	
	access_resource = kzalloc(sizeof(*access_resource), GFP_KERNEL);
	if (!access_resource) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_update_access_resource data\n", __func__);
		return -ENOMEM;
	}
	
	access_resource->client = client;
	ret = kstrtoul(buf, 10, &val);
	if (ret || val<0)
		goto err;

	if(val == 1)
	{
		ret = bq27x00_force_reset(access_resource);
		if(ret != 1)
			dev_err(&client->dev, "(%s)bq27x00_force_reset fail, Try again.\n", __func__);
	}
	
err:
	kfree(access_resource);
	return count;
}
static DEVICE_ATTR(force_reset, 0664, NULL, bq27x00_force_reset_store);

static ssize_t bq27x00_control_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	
	ret = bq27x00_read_code(client, BQ27425_CMD_OPERATION_CONFIGURATION, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s)read CMD_OPERATION_CONFIGURATION(0x3a) failure\n", __func__);
		return 0;
	}
	
	dev_info(&client->dev, "(%s)Operation Configuration :0x%04x\n", __func__, ret);

	return sprintf(buf, "Operation Configuration : %u\n", ret);
}

static ssize_t bq27x00_control_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 16, &val);
	if (ret < 0)
		return ret;
		
	ret = enable_to_unsealed(client);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): place unsealed failure\n", __func__);
		goto error;
	}
	
	ret = bq27x00_write_code(client, BQ27425_CMD_OPERATION_CONFIGURATION, val, false);
	if(ret < 0) {
		dev_err(&client->dev, "(%s): write OPERATION_CONFIGURATION failure, 0x%04x\n", __func__, val);
	}
	
	place_to_sealed(client);
	
error:
	return count;
}
static DEVICE_ATTR(control, 0664, bq27x00_control_show, bq27x00_control_store);

static struct attribute *bq27x00_update_attributes[] = {
	&dev_attr_read_bd.attr,
	&dev_attr_write_bd.attr,
	&dev_attr_control.attr,
	&dev_attr_force_reset.attr,
	NULL
};

static const struct attribute_group bq27x00_update_attr_group = {
	.attrs = bq27x00_update_attributes,
};

/* -------------------------------------------------------------------------------------- */

#define BQ27425_UPDATE_MODE_I2C_ADDR 0x16
#define BQ27425_NORMAL_MODE_I2C_ADDR 0xAA
//#define BQ27425_FIRMWARE_NAME "01.NJ5.BQ27425.20130207.dffs"
#define BQ27425_FIRMWARE_NAME "01.NXD.BQ27425.20130429.dffs"
//#define BQ27425_NJ5_DESIGN_CAPACITY 2500
#define BQ27425_NJ5_DESIGN_CAPACITY 8200
#define BQ27425_NJ5_HW_VERSION 0x0425
#define BQ27425_NJ5_FIRMWARE_VERSION 0x0205

static int bq27x00_write_read_i2c(struct i2c_client *client, u8 ctrl_reg, u16 ctrl_val, bool single)
{
	int ret;
	
	ret = bq27x00_write_code(client, ctrl_reg, ctrl_val, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x%04x to 0x%02x reg failure (ret:%d)\n", __func__, ctrl_val, ctrl_reg, ret);
		return ret;
	}
	
	ret = bq27x00_read_code(client, ctrl_reg, single);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):read 0x%02x reg failure (ret:%d)\n", __func__, ctrl_reg, ret);
		return ret;
	}

	return ret;
}

static int bq27x00_check_version(struct i2c_client *client)
{
	int hw, sw;
	/* ;--------------------------------------------------------
        * ;Verify Existing Firmware Version
        * ;-------------------------------------------------------- */
    hw = bq27x00_write_read_i2c(client, BQ27425_CMD_CNTL, 0x0001, false);
	if (hw < 0) {
		dev_err(&client->dev, "(%s)read hw version failure\n", __func__);
		return hw;
	}

	sw = bq27x00_write_read_i2c(client, BQ27425_CMD_CNTL, 0x0002, false);
	if (sw < 0) {
		dev_err(&client->dev, "(%s)read sw version failure\n", __func__);
		return sw;
	}
	dev_info(&client->dev, "(%s)h/w version: 0x%04x, firmware version: 0x%04x\n", __func__, hw, sw);

	if(hw == BQ27425_NJ5_HW_VERSION && sw == BQ27425_NJ5_FIRMWARE_VERSION)
		return 0;
	else
	{
		dev_err(&client->dev, "(%s)h/w and s/w version incorrect.\n", __func__);
		return -EFAULT;
	}	
}

static int bq27x00_goto_rom_mode(struct i2c_client *client)
{
	int ret;
	/* ;--------------------------------------------------------
	 * ;Go To ROM Mode
	 * ;--------------------------------------------------------*/
	ret = bq27x00_write_code(client, BQ27425_CMD_CNTL, 0x0F00, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x0F00(goto ROM mode) failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	msleep(1000);
	
	return ret;
}

static int bq27x00_exit_rom_mode(struct bq27x00_update_access_resource *access_resource)
{
	struct i2c_client *client = access_resource->client;
	struct i2c_client *client_up = access_resource->client_up;
	int ret;
	/* ;--------------------------------------------------------
	 * ;exit ROM Mode
	 * ;--------------------------------------------------------*/
	if(client_up == NULL)
	{
		dev_err(&client->dev, "(%s):not such rom mode(0x16) device \n" ,__func__);
		return -ENODEV;
	}
	
	ret = bq27x00_write_code(client_up, 0x00, 0x0F, true);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x0F(exit ROM mode) failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	ret = bq27x00_write_code(client_up, 0x64, 0x000F, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x0F(exit ROM mode) failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	msleep(4000);
	
	return ret;
}

static int bq27425_rom_mode_prepare(struct bq27x00_update_access_resource *access_resource)
{
	struct i2c_client *client = access_resource->client;
	int err, ret;
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {{0}};
	u8 buff[] = {0x1F, 0x00, 0x00, 0x00, 0xD2, 0xFF};
	
	//adapter = i2c_get_adapter(4);
	adapter = access_resource->client->adapter;
	if (!adapter){
		dev_err(&client->dev, "%s: bus adapter is null\n", __func__);
		err = -ENODEV;
		return err;
	}
	else {
		info.addr = BQ27425_UPDATE_MODE_I2C_ADDR/2;
		if (!access_resource->client_up)
			access_resource->client_up = i2c_new_device(adapter, &info);
		//i2c_put_adapter(adapter);
		if (!access_resource->client_up){
			dev_err(&client->dev, "%s: update mode client is null\n", __func__);
			err = -ENXIO;
			return err;
		}
	}

	ret = bq27x00_write_data(access_resource->client_up, 0x00, buff, ARRAY_SIZE(buff));
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 6 bytes data(@ROM mode) failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	ret = bq27x00_write_code(access_resource->client_up, 0x64, 0x01F0, false);
	if (ret < 0) {
		dev_err(&client->dev, "(%s):write 0x01F0(@ROM mode) failure (ret:%d)\n" ,__func__, ret);
		return ret;
	}
	msleep(1);
	ret = bq27x00_read_code(access_resource->client_up, 0x66, true);
	if(ret != 0x00)
	{
		dev_err(&client->dev, "(%s):checksum 0x01F0(@ROM mode) from 0x66 reg failure (ret:%d)\n" ,__func__, ret);
		ret = -EINVAL;
		return ret;
	}

	return 0;
}

static int bq27425_rom_mode_parse_and_write_firmware_data(struct bq27x00_update_access_resource *access_resource, char *firmware)
{
	struct i2c_client *client = access_resource->client_up;
	char *templine, *byte;
	const char *first_char;
	int count = 0, err = 0, i = 0, j;
	u8 buffer[8], reciver[2];
	int delay = 20;

	if(client == NULL)
		return -ENODEV;
	
	/* 6. start to parse */	
	while ((templine = strsep (&firmware, "\n")) != NULL) // get line
	{
		count = 0;
		memset(buffer, 0, ARRAY_SIZE(buffer));
		
		first_char = templine; //save first char
		i++;  // count number of line
		if(*first_char == 'E')  // EOF (\r) 
		{
			dev_info(&access_resource->client->dev, "%d: End of firmware \n", i);
			break;
		}
		if(!strncmp(first_char, ";", 1)) // It's comments, skip.
			continue; 
		templine+=3; // skip ':' and ' ', shift to i2c address

		if(!strncmp(first_char, "X", 1)) 
			delay = simple_strtoul(strsep (&templine, " "), NULL, 10);
		else 
		{
			while ((byte = strsep (&templine, " ")) != NULL) //translate string to bytes
			{
			
				buffer[count] = (u8)simple_strtoul(byte, NULL, 16);
				count++;
			}
			if(!(buffer[0] == BQ27425_NORMAL_MODE_I2C_ADDR) && !(buffer[0] == BQ27425_UPDATE_MODE_I2C_ADDR)){
				dev_err(&access_resource->client->dev, "%s: client address is wrong!!\n", __func__);
				return -EFAULT;
			}
			else if(buffer[0] == BQ27425_NORMAL_MODE_I2C_ADDR)
				client = access_resource->client;
			else if(buffer[0] == BQ27425_UPDATE_MODE_I2C_ADDR)
				client = access_resource->client_up;
		}
		switch(*first_char){
			case 'W': // write data, "W: I2CAddr RegAddr Byte0 Byte1 Byte2!K".
				err = bq27x00_write_data(client, buffer[1], buffer+2, count-2);
				if (err < 0) 
				{
					printk("Write(%d): ", i);
					for(j=0; j<count; j++)
						printk("0x%02x ", buffer[j]);
					printk("error!!\n");
					return err;
				}
				break;
			case 'X': // delay time
				//printk("Delay: %d\n", delay);
				msleep(delay);
				break;
			case 'C':  // check data, "C: i2cAddr RegAddr Byte0 Byte1 Byte2".
				memset(reciver, 0, ARRAY_SIZE(reciver));
				err = bq27x00_read_data(client, buffer[1], reciver, count-2);
				if (err < 0) 
				{
					printk("Read and Check(%d): ", i);
					for(j=0; j<count; j++)
						printk("0x%02x ", buffer[j]);
					printk("error!!\n");
					return err;
				}
				for(j=0; j<count-2;j++)
				{
					if(reciver[j] != buffer[j+2])
					{
						dev_err(&client->dev, "%s: data compare fail!! [0x%02x][0x%02x]\n", __func__, reciver[0], reciver[1]);
						return -EINVAL;
					}
				}
				break;
			default:
				break;
		}
	}
	return err;
}

static int bq27425_check_rom_mode(struct bq27x00_update_access_resource *access_resource)
{
	struct bq27x00_update_access_resource *access_res = access_resource;
	struct i2c_client *client = access_res->client;
	int ret, timeout;

	ret = bq27425_rom_mode_prepare(access_res);
	if(ret == 0)
	{
		dev_info(&client->dev, "(%s)rom mode test success\n", __func__);
	
		timeout = 3;
		do {
		ret = bq27x00_exit_rom_mode(access_res);
		} while(ret && --timeout);
		if(ret == 0)
			dev_info(&client->dev, "(%s)Exit rom mode\n", __func__);
	}
	 
	return ret;
}

static int bq27425_update_firmware(struct bq27x00_update_access_resource *access_resource)
{
	struct bq27x00_update_access_resource *access_res = access_resource;
	struct i2c_client *client = access_res->client;
	struct bq27x00_block_data_info *info;
	char *p;
	int ret = 0;
	const struct firmware *fw;
	const char *fw_bq27425 = BQ27425_FIRMWARE_NAME;
	const int designCapacity = BQ27425_NJ5_DESIGN_CAPACITY;
	int timeout = 3;

	/* 1. check whether firmware already update (Desgin capacity = 2500mA). */
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "(%s)failed to allocate bq27x00_block_data_info data\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	info->subclass = 82;
	info->offset = 12;
	info->length = 2;
	if((ret = bq27x00_read_resource(access_res, info)) < 0){
		dev_err(&client->dev, "(%s): read disgin capacity data error (%d)!!!\n", __func__, info->data);
		goto read_blk_err;
	}
	if(info->data == designCapacity) {
		dev_info(&client->dev, "(%s)battery cell arguments already update(%d)\n", __func__, info->data);
		kfree(info);
		return 0;
	}
	
	dev_info(&client->dev, "(%s)start downloading battery cell arguments\n", __func__);

	/* 2. check h/w and s/w version */
	if((ret = bq27x00_check_version(client)) < 0)
		goto version_err;

	/* 3. place to full unsealed */
	if((ret = enter_to_full_unsealed(client)) < 0)
		goto full_unsealed_err;

	/* 4. requests device.c (init) to download firmware from /etc/firmware/ */
	ret = request_firmware(&fw, fw_bq27425, &client->dev);
	if (ret != 0) {
		dev_err(&client->dev, "(%s)update firmware failure.\n" ,__func__);
		goto req_err;
	}
	// back a copy
	p = kmalloc(fw->size, GFP_KERNEL);  //fw->size = page_count * page_size; not file size. 
	if (p != NULL) {
		memcpy(p, fw->data, fw->size);
	}
	else {
		dev_err(&client->dev, "(%s): alloc memory buffer error.\n", __func__);
		goto alloc_err;
	}
	
	/* 5. go to rom mode. Take case the statements, no back way !!!*/
	if((ret = bq27x00_goto_rom_mode(client)) < 0)
		goto enter_rom_mode_err;

	dev_info(&client->dev, "(%s)Enter rom mode\n", __func__);

	/* 5.1 rom mode configuration.  */
	ret = bq27425_rom_mode_prepare(access_res);
	if (ret < 0){
		dev_err(&client->dev, "(%s): prepare rom mode fail!!!.\n", __func__);
		goto rom_mode_err;
	}
	
	/* 6. parse firmware data and write data */
	do {
		ret = bq27425_rom_mode_parse_and_write_firmware_data(access_res, p);
		if(!ret)
			dev_info(&client->dev, "(%s)finish downloading battery cell arguments\n", __func__);
		else
			dev_err(&client->dev, "(%s)something wrong on updating bq27425 cell arguments, retry %d\n", __func__, timeout);
	} while(ret && --timeout);

rom_mode_err:
	timeout = 3;
	do {
	ret = bq27x00_exit_rom_mode(access_res);
	} while(ret && --timeout);
	if(ret == 0)
		dev_info(&client->dev, "(%s)Exit rom mode\n", __func__);
enter_rom_mode_err:
	kfree(p);
alloc_err:
	release_firmware(fw);
req_err:
	place_to_sealed(client);
full_unsealed_err:
version_err:
read_blk_err:
	kfree(info);
	return ret;
}

static ssize_t bq27x00_rom_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	static struct bq27x00_update_access_resource *access_res = NULL;
	int ret, timeout;
	unsigned long val;

	if(access_res == NULL){
		access_res = kzalloc(sizeof(struct bq27x00_update_access_resource), GFP_KERNEL);
		if (!access_res) {
			dev_err(&client->dev, "(%s)failed to allocate bq27x00_update_access_resource data structure\n", __func__);
			return -ENOMEM;
		}
		access_res->client = client;
	}

	ret = kstrtoul(buf, 10, &val);
	if (ret < 0)
		return ret;

	if(val == 1)
	{
		enter_to_full_unsealed(client);
		ret = bq27x00_goto_rom_mode(client);
		if(ret == 0)
			dev_info(&client->dev, "(%s)Enter rom mode\n", __func__);
	}
	else if(val == 0)
	{
		timeout = 3;
		do {
			ret = bq27x00_exit_rom_mode(access_res);
		} while(ret && --timeout);
		if(ret == 0)
			dev_info(&client->dev, "(%s)Exit rom mode\n", __func__);
	}
	else if(val == 66)
	{
		ret = bq27425_rom_mode_prepare(access_res);
		if(ret == 0)
			dev_info(&client->dev, "(%s)rom mode test success\n", __func__);
	}
	
error:
	return count;
}
static DEVICE_ATTR(rom_mode, 0664, NULL, bq27x00_rom_mode_store);

static struct attribute *bq27x00_rom_mode_attributes[] = {
	&dev_attr_rom_mode.attr,
	NULL
};

static const struct attribute_group bq27x00_rom_mode_attr_group = {
	.attrs = bq27x00_rom_mode_attributes,
};

static int bq27x00_open_resource(struct bq27x00_update_access_resource *access_resource, struct i2c_client *client)
{
	int retval;
	if(!client || !access_resource)
		return -ENODATA;
	
	access_resource->client = client;
	
	retval = sysfs_create_group(&client->dev.kobj, &bq27x00_update_attr_group);
	if (retval)
	{
		dev_err(&client->dev, "(%s) error in creating update sysfs attribute" , __func__);
		return retval;
	}

	retval = sysfs_create_group(&client->dev.kobj, &bq27x00_rom_mode_attr_group);
	if (retval)
	{
		dev_err(&client->dev, "(%s) error in creating rom_mode sysfs attribute" , __func__);
		return retval;
	}
	
	return 0;
}

struct bq27x00_update_access_resource* get_bq27x00_update_instance(void)
{
	static struct bq27x00_update_access_resource instance = {
	
		NULL, NULL, 0, bq27x00_open_resource, bq27x00_read_resource, bq27x00_write_resource,
			bq27x00_write_operation_configuration, bq27x00_read_operation_configuration, 
			bq27x00_force_reset, bq27425_update_firmware, bq27425_check_rom_mode
	};

	return &instance;
	
}
EXPORT_SYMBOL(get_bq27x00_update_instance);


