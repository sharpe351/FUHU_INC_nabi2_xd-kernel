/*
 * BQ275xx battery update driver
 *
 *
 * Copyright (C) 2012 Aimar Liu <aimar.ts.liu@foxconn.com>
 *
 *
 * Based on a bq27541 EVM work by Copyright (C) 2010 Texas Instruments, Inc.
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
#include "bq27541.h"

#define FUAL_GAUGE_RESET
#define RESET_COMMAND 0x41

#define BUFFERSIZE             32           // # of bytes for Tx & Rx buffers
#define TAPER_CURRENT          250

struct bq275xx_data_flash_info{
	int is_seal;
	int subclass;
	int offset;
}*bq275xx_data;

static unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
  unsigned int tmp;
  
  tmp = ((msb << 8) & 0xFF00);
  return ((unsigned int)((tmp + lsb) & 0x0000FFFF));  
}

static void transUnsignedInt2Bytes(unsigned int data, unsigned char* bytes)
{
  unsigned char bytesdata[2];
  
  bytes[1] = (unsigned char)((data & 0xFF00) >> 8);
  bytes[0] = (unsigned char)(data & 0x00FF);
  
  return;
}

static int bq275xx_write_code(struct i2c_client *client, u8 reg, char *wt_value, int bytes)
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
		data[i] = wt_value[i-1];	
	
	err = i2c_transfer(client->adapter, msg, 1);
	return err;
}

static int bq275xx_read_code(struct i2c_client *client, u8 reg, char *rt_value, int bytes)
{
	struct i2c_msg msg[1];
	unsigned char data[bytes];
	int err;
	int i;
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	/* send memory address */
	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		msg->len = bytes;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {		
			for(i = 0;i<bytes;i++)
			{
				rt_value[i]=data[i];
			}
			return 0;
		}
	}
	return err;
}

static int bq275xx_read_block(struct i2c_client *client, int subclass, int offset, unsigned char *data)
{
	unsigned char RxData[32];
	int i, numBlock;
	unsigned char TxData[1];

	numBlock = offset/BUFFERSIZE;

	// Write & read back 32 bytes of data in Manufacturer Info Block X
	TxData[0] = 0x00;
  	bq275xx_write_code(client, bq27541CMD_DFDCNTL, TxData, 1);// BlockDataControl() = 0x00
  	msleep(100);
	TxData[0] = subclass;
  	bq275xx_write_code(client, bq27541CMD_DFCLS, TxData, 1);    // Write the subclass value
  	msleep(100);
	TxData[0] = numBlock;
  	bq275xx_write_code(client, bq27541CMD_DFBLK, TxData, 1);// Select offset within the flash  
  	msleep(1000);
	printk("***** Data flash Summary [Subclass ID: %d]*****\n", subclass);
	bq275xx_read_code(client, bq27541CMD_ADF, RxData, 32); 
	for (i = 0; i < BUFFERSIZE; i++)          // Compute the checksum of the block
  	{
		data[i] = RxData[i];
		//printk("DataBlock[0x%02x]: 0x%02x\n", bq27541CMD_ADF+i, RxData[i]);
  	}
	printk("***** End of Data Flash Block %d*****\n", numBlock);
	
	return 0;
}

static ssize_t bq275xx_blockdata_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char RxData[32];

	bq275xx_read_block(client, bq275xx_data->subclass, bq275xx_data->offset, RxData);
	
	return sprintf(buf, "%u\n", "read block data success");
}

static ssize_t bq275xx_blockdata_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned long val;
	int error, i;
	char *tok;
	unsigned int num = 1;
	unsigned int subID = 0;
	unsigned int offset = 0;
	int data = 0;
	unsigned int numbytes = 0;
	unsigned char TxData[32];
	unsigned char RxData[32];
	unsigned char WordData[2];
	unsigned int sum = 0;
	unsigned int checksum;
	unsigned int numBlock = 0;

	/* echo [subID],[offset],[data],[writebyte] > blockdata */
	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = strict_strtoul(tok, 10, &val);
		if (error)
			return error;
		if(num == 1)
			subID = val;
		else if(num == 2)
			offset = val;
		else if(num == 3)
			data = val;
		else if(num == 4)
			numbytes = val;
		num++;
	}
	if(num != 5)
	{
		printk("[bq275xx gauge]the number of parameters not match.\n");
		return count;
	}

	numBlock = offset % BUFFERSIZE;
	error = bq275xx_read_block(client, subID, offset, TxData);
	if(!error)
	{
		bq275xx_data->subclass = subID;
		bq275xx_data->offset = offset;
	}

	transUnsignedInt2Bytes(data, WordData);
	if(numbytes == 1)
		TxData[numBlock] = WordData[0];
	else
	{
		TxData[numBlock+1] = WordData[0];
		TxData[numBlock] = WordData[1];
	}
	
  	printk("[bq275xx gauge]write data....0x%02x%02x\n", WordData[1], WordData[0]);
  	for (i = 0; i < BUFFERSIZE; i++)          // Write 32 bytes to Info Block A
  	{
    	bq275xx_write_code(client, (bq27541CMD_ADF+i), &TxData[i], 1);  
 	}
	
	for (i = 0; i < BUFFERSIZE; i++)          // Compute the checksum of the block
  	{
    	sum += TxData[i];                       // Calculate the sum of the values  
  	}
	checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
	printk("[bq275xx gauge]write checksum....0x%02x\n", checksum);
  	bq275xx_write_code(client, bq27541CMD_DFDCKS, &checksum, 1); // Write checksum value

#if 0
	bq275xx_read_code(client, bq27541CMD_ADF, RxData, 32);  // Read the contents of the block
  	for (i = 0; i < BUFFERSIZE; i++)          // Check if writes were successful
  	{
    	if (TxData[i] != RxData[i])             // Tx & Rx data values match?
    	{
      		printk("[bq275xx gauge]Data not match on DataBlock[0x%02x]: %d\n", 0x40+i, RxData[i]);   // Signal error condition occurred
    	}
  	}
#endif
	return count;
}
static DEVICE_ATTR(blockdata, 0666, bq275xx_blockdata_show, bq275xx_blockdata_store);

static ssize_t bq275xx_en_dataflash_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char TxData[2];
	unsigned char RxData[2];
	int val;
	
	TxData[0] = 0x00;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);// BlockDataControl() = 0x00
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_CNTL_LSB, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]Control register is 0x%04x\n", val);
	
	return sprintf(buf, "%u\n", val);
}

static ssize_t bq275xx_en_dataflash_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char TxData[2];
	int val, error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val) {
		/* unseal key */
		TxData[0] = 0x14;
		TxData[1] = 0x04;
		bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
		TxData[0] = 0x72;
		TxData[1] = 0x36;
		bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
		/* full-access key */
		TxData[0] = 0xff;
		TxData[1] = 0xff;
		bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
		TxData[0] = 0xff;
		TxData[1] = 0xff;
		bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
		bq275xx_data->is_seal = 0;
		printk("[bq275xx gas-gauge]Enable unseal and full-access mode success\n");
	}
	else
	{
		TxData[0] = 0x20;
		TxData[1] = 0x00;
		bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
		bq275xx_data->is_seal = 1;
		printk("[bq275xx gas-gauge]Place into seal mode.\n");
	}
	return count;
}
static DEVICE_ATTR(en_dataflash, 0666, bq275xx_en_dataflash_show, bq275xx_en_dataflash_store);

static ssize_t bq275xx_readinfo_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char RxData[32];
	unsigned int num = 1;
	unsigned int subID = 0;
	unsigned int offset = 0;
	unsigned long val;
	char *tok;
	int error;

	/* echo [subID],[offset] > readinfo*/
	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = strict_strtoul(tok, 10, &val);
		if (error)
			return error;
		if(num == 1)
			subID = val;
		else if(num == 2)
			offset = val;
		num++;
	}

	if(num != 3)
	{
		printk("[bq275xx gauge]the number of parameters not match.\n");
		return count;
	}

	bq275xx_read_block(client, subID, offset, RxData);
	return count;
}
static DEVICE_ATTR(readinfo, 0666, NULL, bq275xx_readinfo_store);

#ifdef FUAL_GAUGE_RESET
static ssize_t bq275xx_control_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char TxData[2];
	unsigned char RxData[2];
	int val, i;
	
	TxData[0] = 0x02;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);// BlockDataControl() = 0x00
	msleep(100);
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_CNTL_LSB, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]FW_VERSION is 0x%04x\n", val);

	TxData[0] = 0x03;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);// BlockDataControl() = 0x00
	msleep(100);
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_CNTL_LSB, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]HW_VERSION is 0x%04x\n", val);

	TxData[0] = 0x01;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);// BlockDataControl() = 0x00
	msleep(100);
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_CNTL_LSB, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]DEVICE_TYPE is 0x%04x\n", val);


	for(i = 0x00; i<=0x2C; i+=2)
	{
		RxData[0] = 0x00;
  		RxData[1] = 0x00;
  		bq275xx_read_code(client, i, RxData, 2);
		val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  		printk("[bq275xx gas-gauge][REG-0x%02x] is 0x%04x\n",i, val);
	}
	
	return sprintf(buf, "%u\n", val);
}

static ssize_t bq275xx_control_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char TxData[2];
	unsigned char RxData[2];
	int val, error;
	unsigned int offset = 0;
	unsigned int value = 0;
	char *tok;
	unsigned int num = 1;

	/* echo [offset],[value] > control*/

	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = strict_strtoul(tok, 10, &val);
		if (error)
			return error;
		if(num == 1)
			offset = val;
		else if(num == 2)
			value = val;
		num++;
	}

	if(num != 3)
	{
		printk("[bq275xx gauge]the number of parameters not match.\n");
		return count;
	}
	
	/* Force Reset */
	TxData[0] = value;
	TxData[1] = 0x00;
	bq275xx_write_code(client, offset, TxData, 2);
	msleep(100);
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, offset, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]after write offset[0x%02x] data is 0x%04x\n", offset, val);
	return count;
}
static DEVICE_ATTR(control, 0666, bq275xx_control_show, bq275xx_control_store);
#endif

static struct attribute *bq275xx_attributes[] = {
	&dev_attr_en_dataflash.attr,
	&dev_attr_blockdata.attr,
	&dev_attr_readinfo.attr,
	#ifdef FUAL_GAUGE_RESET
	&dev_attr_control.attr,
	#endif
	NULL
};

static const struct attribute_group bq275xx_attr_group = {
	.attrs = bq275xx_attributes,
};

int bq275xx_force_reset(struct i2c_client *client)
{
	unsigned char TxData[2];
	unsigned char RxData[2];
	unsigned char enData[2];
	int val, i;
	//step1. read all registers of control register
	for(i = 0x00; i<=0x2C; i+=2)
	{
		RxData[0] = 0x00;
  		RxData[1] = 0x00;
  		bq275xx_read_code(client, i, RxData, 2);
		val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  		printk("[bq275xx gas-gauge][REG-0x%02x] is 0x%04x\n",i, val);
	}
	//step2. unsealed
	/* unseal key */
	enData[0] = 0x14;
	enData[1] = 0x04;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	enData[0] = 0x72;
	enData[1] = 0x36;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	/* full-access key */
	enData[0] = 0xff;
	enData[1] = 0xff;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	enData[0] = 0xff;
	enData[1] = 0xff;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	printk("[bq275xx gas-gauge]Enable unseal and full-access mode success\n");
	//step3. force reset
	/* Force Reset */
	TxData[0] = RESET_COMMAND;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
	msleep(100);
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_CNTL_LSB, RxData, 2);
	val = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq275xx gas-gauge]after write offset[0x%02x] data is 0x%04x\n", bq27541CMD_CNTL_LSB, val);

	return 0;
}
EXPORT_SYMBOL(bq275xx_force_reset);

int bq275xx_parameter_update(struct i2c_client *client)
{
	unsigned char enData[2];
	unsigned char TxData[32];
	unsigned char WordData[2];
	unsigned int numBlock = 0;
	unsigned int sum = 0;
	unsigned int checksum;
	int i;
	/* unseal key */
	enData[0] = 0x14;
	enData[1] = 0x04;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	enData[0] = 0x72;
	enData[1] = 0x36;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	/* full-access key */
	enData[0] = 0xff;
	enData[1] = 0xff;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	msleep(100);
	enData[0] = 0xff;
	enData[1] = 0xff;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, enData, 2);
	bq275xx_data->is_seal = 0;
	msleep(100);
	printk("[bq275xx gas-gauge]Enable unseal and full-access mode success\n");

	numBlock = bq275xx_data->offset % BUFFERSIZE;
	bq275xx_read_block(client, bq275xx_data->subclass, bq275xx_data->offset, TxData);
	transUnsignedInt2Bytes(TAPER_CURRENT, WordData); /* change taper current from 100mA to 300mA */
	if((TxData[numBlock+1] == WordData[0]) && (TxData[numBlock] == WordData[1]))
	{
		printk("[bq275xx gas-gauge]taper current data already update\n");
		goto finish;
	}
	TxData[numBlock+1] = WordData[0];
	TxData[numBlock] = WordData[1];
		
  	printk("[bq275xx gauge]write data....0x%02x%02x\n", WordData[1], WordData[0]);
  	for (i = 0; i < BUFFERSIZE; i++)          // Write 32 bytes to Info Block A
  	{
    	bq275xx_write_code(client, (bq27541CMD_ADF+i), &TxData[i], 1);  
 	}
	
	for (i = 0; i < BUFFERSIZE; i++)          // Compute the checksum of the block
  	{
    	sum += TxData[i];                       // Calculate the sum of the values  
  	}
	checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
	printk("[bq275xx gauge]write checksum....0x%02x\n", checksum);
  	bq275xx_write_code(client, bq27541CMD_DFDCKS, &checksum, 1); // Write checksum value
  	msleep(1000);

finish:
	TxData[0] = 0x20;
	TxData[1] = 0x00;
	bq275xx_write_code(client, bq27541CMD_CNTL_LSB, TxData, 2);
	bq275xx_data->is_seal = 1;
	printk("[bq275xx gas-gauge]Place into seal mode.\n");
	sysfs_remove_group(&client->dev.kobj, &bq275xx_attr_group);

	return 0;
}
EXPORT_SYMBOL(bq275xx_parameter_update);

int bq275xx_init_development_mode(struct i2c_client *client)
{
	int retval;
	unsigned char RxData[2];
	unsigned char RxData_Name[8];
	retval = sysfs_create_group(&client->dev.kobj, &bq275xx_attr_group);
	if (retval)
	{
		dev_err(&client->dev, "%s() error in creating sysfs attribute" , __func__);
		goto err;
	}
	bq275xx_data = kzalloc(sizeof(*bq275xx_data), GFP_KERNEL);
	if (!bq275xx_data) {
		dev_err(&client->dev, "failed to allocate bq275xx_data info data\n");
		retval = -ENOMEM;
		goto alloc_err;
	}

	bq275xx_data->subclass = 0x24;
	bq275xx_data->offset = 2;
	bq275xx_data->is_seal = 1;
	// Read Design Capacity (units = mAH)
	#if 1 //skip those information to avoid race condition with charger's callback by aimar 20121019
	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_DCAP_LSB, RxData, 2);
  	retval = transBytes2UnsignedInt(RxData[1], RxData[0]);
  	printk("[bq27541 gauge]Design Capacity is %d\n", retval);
  	// Read Device Name Length
  	RxData[0] = 0x00;
  	RxData[1] = 0x00;
  	bq275xx_read_code(client, bq27541CMD_DNAMELEN, RxData, 1);
  	printk("[bq27541 gauge]Device Name Length is %d\n", RxData[0]); 
	retval = RxData[0];
  	// Read Device Name (Rx buffer should end up with ASCII chars for 'bq27541')
  	bq275xx_read_code(client, bq27541CMD_DNAME, RxData_Name, retval);
	RxData_Name[7] = 0;
  	printk("[bq27541 gauge]Device Name is %s\n", RxData_Name);
	#endif
	return 0;

alloc_err:
	sysfs_remove_group(&client->dev.kobj, &bq275xx_attr_group);
err:
	return retval;
}
EXPORT_SYMBOL(bq275xx_init_development_mode);

