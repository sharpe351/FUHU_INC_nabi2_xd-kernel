/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 * Copyright (C) 2011 NVIDIA Corporation.
 * Copyright (C) 2013 Foxconn Corporation <aimar.ts.liu@foxconn.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
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

/*
 * This patch adds support for BQ27425 (TI) chip. This chip is same as
 * BQ27500 with few registers removed and register address map changed.
 * Define SOC register address plus offset for bq27425 instead of 
 * register address alone to align with other register reads.
 */
 
/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27425-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/power/bq27x00_battery.h>
#include <linux/regulator/bq2419x.h>
#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
#include "bq27x00_update.h"
#include <linux/wakelock.h>
#endif

//&*&*&*HC1_20120525
#ifdef CONFIG_NABI_FAKE_BATTERY
#include <linux/io.h>
#include <mach/iomap.h>
#endif
//&*&*&*HC2_20120525

//&*&*&*HC1_20120712
#include <linux/fs.h>
#include <asm/uaccess.h>
//&*&*&*HC2_20120712

#define DRIVER_VERSION			"1.2.0"

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26
#define BQ27x00_REG_NAC			0x0C /* Nominal available capacity */
#define BQ27x00_REG_LMD			0x12 /* Last measured discharge */
#define BQ27x00_REG_CYCT		0x2A /* Cycle count total */
#define BQ27x00_REG_AE			0x22 /* Available energy */
#define BQ27x00_POWER_AVG		0x24

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76 /* Initial last measured discharge */
#define BQ27000_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27000_FLAG_FC			BIT(5)
#define BQ27000_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27500_REG_SOC			0x2C
#define BQ27500_REG_DCAP		0x3C /* Design capacity */
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27500_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27500_FLAG_BAT_DET	BIT(3)
#define BQ27500_FLAG_FC			BIT(9)
#define BQ27500_FLAG_OTC		BIT(15)
#define BQ27500_FLAG_UTC		BIT(14)

/* bq27425 register addresses are same as bq27x00 addresses minus 4 */
#define BQ27425_REG_CNTL		0x00
#define BQ27425_REG_OFFSET		0x04
#define BQ27425_POWER_AVG		0x1C
#define BQ27425_REG_SOC			0x20 
#define BQ27425_REG_DCAP		0x40 /* Design capacity */


#define BQ27000_RS			20 /* Resistor sense */
#define BQ27x00_POWER_CONSTANT		(256 * 29200 / 1000)

/* bq27425-g1 control register sub-commands*/
#define BQ27425_CNTL_DEVICE_TYPE	0x0001
#define BQ27425_CNTL_FW_VERSION		0x0002
#define BQ27425_CNTL_HW_VERSION  	0x0003
#define BQ27425_CNTL_BAT_INSERT     0x000C
#define BQ27425_CNTL_SET_CFGUPDATE  0x0013
#define BQ27425_CNTL_SEALED         0x0020
#define BQ27425_CNTL_FORCE_RESET    0x0041
#define BQ27425_CNTL_SOFT_RESET     0x0042

#define BQ27425_LOW_BATTERY_SOC1    5
#define BQ27425_BATTERY_FULL_CAPACITY_THRESHOLD 89

/* bq27x00 requires 3 to 4 second to update charging status */
#define CHARGING_STATUS_UPDATE_DELAY_SECS	4

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
	int (*ctrl_read)(struct bq27x00_device_info *di, u8 ctrl_reg,
				u16 ctrl_func_reg);
	int (*write)(struct bq27x00_device_info *di, u8 reg, u16 val,
				bool single);
};

enum bq27x00_chip { BQ27000, BQ27500, BQ27425};

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
	int now_current;
	int voltage;
	int remainingCapacity;
	int fullChargeCapacity;
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;
	struct delayed_work low_bat_delay_work;

	struct power_supply	bat;
	struct power_supply	ac;
	struct power_supply	usb;
	int battery_online;
	int ac_online;	
	int usb_online;
	int charging_status;
	//&*&*&*AL1_20130503 update_charging_health
	int charging_health;
	//&*&*&*AL2_20130503 update_charging_health
	
	struct bq27x00_access_methods bus;

	struct mutex lock;
	struct mutex update_lock;

	int irq;
	low_battery_callback_t low_bat_cb;
	void	*low_bat_cb_data;
	
	//&*&*&*HC1_20120525
	#ifdef CONFIG_NABI_FAKE_BATTERY
	bool fake_battery;
	#endif
	//&*&*&*HC2_20120525
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	//&*&*&*AL1_20130220 don't update and access before firmware complete
	int init_complete;
	//&*&*&*AL2_20130220 don't update and access before firmware complete
	struct bq27x00_update_access_resource *up_res;
	struct delayed_work download_firmware_work;
	struct wake_lock wakelock_download;
	int re_download;
	#endif
	int old_RemainingCapacity;
	int remainingCapacity_count;
	//&*&*&*AL1_20130429 add debug-on attribute to track battery capacity.
	u8 is_debug;
	//&*&*&*AL2_20130429 add debug-on attribute to track battery capacity.
	u8 reset_gauge;
};

static struct bq27x00_device_info *g_bq27x00_di = NULL;

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property bq27425_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	//&*&*&*AL1_20130503 update_charging_health
	POWER_SUPPLY_PROP_HEALTH,
	//&*&*&*AL2_20130503 update_charging_health
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static enum power_supply_property bq27425_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property bq27425_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static unsigned int poll_interval = 20;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

//&*&*&*HC1_20120525
#ifdef CONFIG_NABI_FAKE_BATTERY
#define PMC_SCRATCH20		0xa0
#define FAKE_BATTERY_MODE	BIT(25)
bool is_fake_battery()
{
	void __iomem *reset = IO_ADDRESS(TEGRA_PMC_BASE + 0x00);
	u32 reg;
	bool ret = false;

	reg = readl_relaxed(reset + PMC_SCRATCH20);

	if (reg & FAKE_BATTERY_MODE) {
		reg &= ~(FAKE_BATTERY_MODE);
		writel_relaxed(reg, reset + PMC_SCRATCH20);
		ret = true;
	}

	printk("%s, fack_battery=%d \n", __func__, ret);
	
	return ret;
}
#endif
//&*&*&*HC2_20120525

int register_low_battery_callback(low_battery_callback_t cb, void *args)
{
	struct bq27x00_device_info *di = g_bq27x00_di;
	if (!di)
		return -ENODEV;

	di->low_bat_cb = cb;
	di->low_bat_cb_data = args;
	
	return 0;
}

/*
 * Common code for BQ27x00 devices
 */

static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	if (di->chip == BQ27425)
		return di->bus.read(di, reg - BQ27425_REG_OFFSET, single);
	return di->bus.read(di, reg, single);
}

static inline int bq27x00_ctrl_read(struct bq27x00_device_info *di,
					u8 ctrl_reg, u16 ctrl_func_reg)
{
	return di->bus.ctrl_read(di, ctrl_reg, ctrl_func_reg);
}

static inline int bq27x00_write(struct bq27x00_device_info *di, u8 reg,
		u16 val, bool single)
{
	return di->bus.write(di, reg, val, single);
}

/*
 * Higher versions of the chip like BQ27425 and BQ27500
 * differ from BQ27000 and BQ27200 in calculation of certain
 * parameters. Hence we need to check for the chip type.
 */
static bool bq27xxx_is_chip_version_higher(struct bq27x00_device_info *di)
{
	if (di->chip == BQ27425 || di->chip == BQ27500)
		return true;
	return false;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc;

	if (di->chip == BQ27500)
		rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	else if (di->chip == BQ27425)
		rsoc = bq27x00_read(di, BQ27425_REG_SOC, false);
	else
		rsoc = bq27x00_read(di, BQ27000_REG_RSOC, true);

	if (rsoc < 0)
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (bq27xxx_is_chip_version_higher(di))
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 * fixed. Do not report nominal available capaciy if battery is not calibrated
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	int flags;
	bool is_bq27500 = di->chip == BQ27500;
	bool is_higher = bq27xxx_is_chip_version_higher(di);

	flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !is_bq27500);
	if (flags >= 0 && !is_higher && (flags & BQ27000_FLAG_CI))
		return -ENODATA;

	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (bq27xxx_is_chip_version_higher(di))
		ilmd = bq27x00_read(di, BQ27425_REG_DCAP, false);
	else
		ilmd = bq27x00_read(di, BQ27000_REG_ILMD, true);

	if (ilmd < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

	if (bq27xxx_is_chip_version_higher(di))
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_energy(struct bq27x00_device_info *di)
{
	int ae;

	ae = bq27x00_read(di, BQ27x00_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27500)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (bq27xxx_is_chip_version_higher(di))
		temp -= 2731;
	else
		temp = ((temp * 5) - 5463) / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_pwr_avg(struct bq27x00_device_info *di, u8 reg)
{
	int tval;
 	
	tval = bq27x00_read(di, reg, false);
    if (tval < 0) {
 	   dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
 		      reg, tval);
       return tval;
    }
 
    if (di->chip == BQ27500 || 	di->chip == BQ27425)
       return tval;
    else
       return (tval * BQ27x00_POWER_CONSTANT) / BQ27000_RS;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_health(struct bq27x00_device_info *di)
{
	int tval;

	tval = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if (bq27xxx_is_chip_version_higher(di)) {
		//&*&*&*AL1_20130503 update_charging_health
		#ifndef CONFIG_REGULATOR_BQ2419X
		//if (tval & BQ27500_FLAG_SOCF)
			//tval = POWER_SUPPLY_HEALTH_DEAD;
		if (tval & BQ27500_FLAG_UTC)
			tval = POWER_SUPPLY_HEALTH_COLD;
		else if (tval & BQ27500_FLAG_OTC)
			tval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
		#else
		if(di->charging_health == BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT)
			tval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if(di->charging_health == BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_COLD)
			tval = POWER_SUPPLY_HEALTH_COLD;
		else if(di->charging_health == BQ2419X_CHARGING_STATUS_FAULT_TIMEOUT)
			tval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if(di->charging_health == BQ2419X_CHARGING_STATUS_FAULT_BATTERY)
			tval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if(di->charging_health == BQ2419X_CHARGING_STATUS_STOP)
			tval = POWER_SUPPLY_HEALTH_UNKNOWN;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		#endif
		return tval;
		//&*&*&*AL2_20130503 update_charging_health
	} else {
		if (tval & BQ27000_FLAG_EDV1)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	}

	return -1;
}

//&*&*&*AL1_20130429 add debug-on attribute to track battery capacity. 
static ssize_t bq27425_debug_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)

{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	unsigned long val;
	int ret;
	
	if(!di)
		return count;

	ret = kstrtoul(buf, 10, &val);
	if (ret || val<0)
		goto ret;

	di->is_debug = val;
	dev_info(di->dev, "%s: debug is: %s\n", __func__, di->is_debug?"on":"off");
	
ret:	
	return count;
}
static DEVICE_ATTR(debug, 0664, NULL, bq27425_debug_store);

static ssize_t bq27425_fix_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	u8 is_reset;

	if(!di)
		return 1;

	is_reset = di->reset_gauge;
	dev_info(di->dev, "%s: is_reset: %d\n", __func__, is_reset);
	di->reset_gauge = 0;
	
	return sprintf(buf, "%u\n",  is_reset);
}
static DEVICE_ATTR(fix, 0664, bq27425_fix_show, NULL);

static struct attribute *bq27425_attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_fix.attr,
	NULL
};

static const struct attribute_group bq27425_attr_group = {
	.attrs = bq27425_attributes,
};
//&*&*&*AL2_20130429 add debug-on attribute to track battery capacity. 

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27500 = di->chip == BQ27500;
	bool is_bq27425 = di->chip == BQ27425;

	mutex_lock(&di->update_lock);
	
	//&*&*&*AL1_20130221 don't update and access before firmware complete
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
		if(!di->init_complete)
			goto report;
	#endif
	//&*&*&*AL2_20130220 don't update and access before firmware complete
		
	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !is_bq27425);
	if (cache.flags >= 0) {
		if (!is_bq27500 && !is_bq27425
				&& (cache.flags & BQ27000_FLAG_CI)) {
			dev_info(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			cache.capacity = bq27x00_battery_read_rsoc(di);
			if (!is_bq27425) {
				cache.energy = bq27x00_battery_read_energy(di);
				cache.time_to_empty = bq27x00_battery_read_time(di,	BQ27x00_REG_TTE);
				cache.time_to_empty_avg = bq27x00_battery_read_time(di,	BQ27x00_REG_TTECP);
				cache.time_to_full = bq27x00_battery_read_time(di, BQ27x00_REG_TTF);
			}
			cache.charge_full = bq27x00_battery_read_lmd(di);
			cache.health = bq27x00_battery_read_health(di);
		}
		cache.temperature = bq27x00_battery_read_temperature(di);
		if (!is_bq27425)
			cache.cycle_count = bq27x00_battery_read_cyct(di);
		if(is_bq27425)
			cache.power_avg = bq27x00_battery_read_pwr_avg(di, BQ27425_POWER_AVG);
		else if(is_bq27500)
			cache.power_avg = bq27x00_battery_read_pwr_avg(di, BQ27x00_POWER_AVG);
		
		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27x00_battery_read_ilmd(di);
	}

report:
	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		di->cache = cache;
		if(di->bat.dev)
			power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
	mutex_unlock(&di->update_lock);
}

#ifdef CONFIG_BATTERY_BQ27x00_SHOW_BATTERY_INFORMATION
//&*&*&*HC1_20130503, mod code to prevent possible exception
//&*&*&*HC1_20120712
static char tsensor_temp[32];
static char *tsensor_temperature="/sys/class/hwmon/hwmon0/device/tsensor_temperature";
static void get_cpu_temperature(void)
{
	struct file *tsensor_tmp = NULL;
	mm_segment_t old_fs;
	char    buf[128];
	loff_t offset = 0;

	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	buf[127] = 0;
	sprintf(buf, tsensor_temperature,0);
	tsensor_tmp = filp_open(buf, O_RDONLY, 0);
	if (tsensor_tmp != NULL) {
		if (tsensor_tmp->f_op != NULL &&
			tsensor_tmp->f_op->read != NULL) {
			tsensor_tmp->f_op->read(tsensor_tmp,
					tsensor_temp,
					32,
					&offset);
			// only print the temperatue information here while read operation is success
			printk("CPU Temperature: %s", tsensor_temp);
		}	
		else
			pr_err("f_op might be null\n");

		filp_close(tsensor_tmp, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}
	set_fs(old_fs);

}
//&*&*&*HC2_20120712
//&*&*&*HC2_20130503, mod code to prevent possible exception
#endif

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);
	
	//&*&*&*HC1_20120525
	#ifdef CONFIG_NABI_FAKE_BATTERY
	if (!di->fake_battery) {
	#endif	
	//&*&*&*HC2_20120525

	bq27x00_update(di);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		//set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}

	//&*&*&*HC1_20120525
	#ifdef CONFIG_NABI_FAKE_BATTERY	
	} else {
		printk("Fake battery \n");
	}
	#endif
	//&*&*&*HC2_20120525

	//&*&*&*AL1_20130220 don't update and access before firmware complete
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	if(!di->init_complete)
		return;
	#endif
	//&*&*&*AL2_20130220 don't update and access before firmware complete

	#ifdef CONFIG_BATTERY_BQ27x00_SHOW_BATTERY_INFORMATION
	/* show the reports on kernel messasge */
	di->cache.now_current = (int)(((s16)bq27x00_read(di, BQ27x00_REG_AI, false)) * 1000);
	di->cache.voltage = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	di->cache.remainingCapacity = bq27x00_read(di, 0x10, false);
	di->cache.fullChargeCapacity = bq27x00_read(di, 0x12, false);
	printk("\n---------- Battery Information --------\n");
	printk("Capacity(SOC): %d\n", di->cache.capacity);
	printk("Voltage: %dmV\n", di->cache.voltage);
	printk("Current: %duA\n", di->cache.now_current);
	printk("temperature: %d (°0.1C)\n", di->cache.temperature);
	printk("---------------- END -----------------\n");

	//&*&*&*AL1_20130429 add debug-on attribute to track battery capacity.
	if(di->is_debug){
	printk("\n- Battery Information (Standard Commands Registers) -\n");
	printk("Control Status(0x00/0x01): 0x%04x\n", bq27x00_ctrl_read(di, BQ27425_REG_CNTL, 0x0000));
	printk("Temperature(0x02/0x03): %d (°0.1C)\n", di->cache.temperature);
	printk("Voltage(0x04/0x05): %dmV\n", di->cache.voltage);
	printk("Flags(0x06/0x07): 0x%04x\n", bq27x00_read(di, BQ27x00_REG_FLAGS, false));
	printk("NominalAvailableCapacity(0x08/0x09): %dmAh\n", bq27x00_read(di, 0x0C, false));
	printk("FullAvailableCapacity(0x0A/0x0B): %dmAh\n", bq27x00_read(di, 0x0E, false));
	printk("RemainingCapacity(0x0C/0x0D): %dmAh\n", di->cache.remainingCapacity);
	printk("FullChargeCapacity(0x0E/0x0F):%dmAh\n", di->cache.fullChargeCapacity);
	printk("AverageCurrent(0x10/0x11): %duA\n", di->cache.now_current);
	printk("Debug1(0x16/0x17): 0x%04x\n", bq27x00_read(di, 0x1a, false));
	printk("AveragePower((0x18/0x19): %dmW\n", bq27x00_read(di, 0x1c, false));
	printk("StateOfCharge(SOC)(0x1C/0x1D): %d\n", di->cache.capacity);
	printk("IntTemperature(0x1E/0x1F): %d (°0.1C)\n", bq27x00_read(di, 0x22, false)-2731);
	printk("StateOfHealth(0x20/0x21): 0x%04x\n", bq27x00_read(di, 0x24, false));
	printk("Debug2(0x2C/0x2D): 0x%04x\n", bq27x00_read(di, 0x30, false));
	printk("Debug3(0x32/0x33): 0x%04x\n", bq27x00_read(di, 0x36, false));
	printk("OperationConfiguration(0x3a/0x3b): 0x%04x\n", bq27x00_read(di, 0x3e, false));
	printk("DesignCapacity(0x3c/0x3d): %d\n", bq27x00_read(di, 0x40, false));
	printk("---------------- END -----------------\n\n");
	}
	//&*&*&*AL1_20130429 add debug-on attribute to track battery capacity.

	//&*&*&*HC1_20120712
	get_cpu_temperature();
	//&*&*&*HC2_20120712
	#endif

	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	//&*&*&*AL1_20130410 fix not match charging complete
	if(	(di->ac_online) &&
		(di->cache.capacity >= BQ27425_BATTERY_FULL_CAPACITY_THRESHOLD) && 
		(di->cache.capacity < 100) &&
		(di->cache.now_current < 340000) &&
		(di->cache.voltage > 4200))
		schedule_delayed_work(&di->download_firmware_work, HZ*1);
	//&*&*&*AL2_20130410 fix not match charging complete
	//&*&*&*AL1_20130418 to fix gauge hangis 
	if((di->charging_status == POWER_SUPPLY_STATUS_DISCHARGING) && (di->old_RemainingCapacity == di->cache.remainingCapacity) && (di->cache.now_current != 0))
	{
		di->remainingCapacity_count++;
		dev_err(di->dev, "battery remaining capacity repeat the same value (%d) times on discharging....keep tracking\n", di->remainingCapacity_count);
		if(di->remainingCapacity_count >= 9)
		{
			dev_err(di->dev, "battery remaining capacity(%d) always repeat the same value 9 times on discharging to execute soft reset....\n", di->cache.remainingCapacity);
			schedule_delayed_work(&di->download_firmware_work, HZ*1);
			di->remainingCapacity_count = 0;
		}
	}
	else if(di->old_RemainingCapacity != di->cache.remainingCapacity)
	{
		di->remainingCapacity_count = 0;
		di->old_RemainingCapacity = di->cache.remainingCapacity;
	}
	//&*&*&*AL2_20130418 to fix gauge hang
	//&*&*&*AL1_20130509 to fix gauge caculate remaining capacity too low
	if(di->cache.fullChargeCapacity < 7000)
	{
		schedule_delayed_work(&di->download_firmware_work, HZ*1);
	}
	//&*&*&*AL2_20130509 to fix gauge caculate remaining capacity too low
	#endif
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (bq27xxx_is_chip_version_higher(di)) {
		/* bq27500 returns signed value */
		val->intval = (int)((s16)curr) * 1000;
	} else {
		flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
		if (flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}

	return 0;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if (bq27xxx_is_chip_version_higher(di)) {
		if (di->cache.flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;

	return 0;
}

static int bq27x00_battery_capacity_level(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int level;

	if (bq27xxx_is_chip_version_higher(di)) {
		if (di->cache.flags & BQ27500_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27500_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27500_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27000_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27000_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

static int bq27425_battery_present(struct bq27x00_device_info *di,
					union power_supply_propval *val)
{
	int ret;

	ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	if (ret & BQ27500_FLAG_BAT_DET)
		val->intval = 1;
	else
		val->intval = 0;

	return 0;
}

static char bq27425_serial[5];
static int bq27425_get_battery_serial_number(struct bq27x00_device_info *di)
{
	int ret;

	if (di->chip == BQ27425) {
		ret = bq27x00_ctrl_read(di, BQ27425_REG_CNTL,
					BQ27425_CNTL_DEVICE_TYPE);
		ret = sprintf(bq27425_serial, "%04x", ret);
		return 0;
	} else {
		return 1;
	}
}

int bq27425_fake_battery_get_property(enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
			val->intval = 1; // not charging
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:	
			val->intval = 4000;

		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:		
			val->intval = 10000000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 2500;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:		
			val->intval = 2500;	
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_TEMP:
			val->intval = 250;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	//&*&*&*HC1_20120525
	#ifdef CONFIG_NABI_FAKE_BATTERY
	if (di->fake_battery) {
		return  bq27425_fake_battery_get_property(psp, val);
	}
	#endif
	//&*&*&*HC2_20120525

	//&*&*&*AL1_20130221 update fake status before firmware complete
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	if(!di->init_complete)
		return  bq27425_fake_battery_get_property(psp, val);
	#endif
	//&*&*&*AL2_20130220 update fake status before firmware complete

	//&*&*&*HC1_20130424, will cause exception and debug later 
	#if 0
	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);
	#endif
	//&*&*&*HC2_20130424, will cause exception and debug later 

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->battery_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		//&*&*&*AL1_20130201 take off charing status from gasgauge, change to bq2419x charging update
		//if(di->init_complete)
			//ret = bq27x00_battery_status(di, val);
		//else
		//&*&*&*AL2_20130201 take off charing status from gasgauge, change to bq2419x charging update
		val->intval = di->charging_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		//ret = bq27425_battery_present(di, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		//ret = bq27x00_simple_value(di->cache.capacity, val);
		val->intval = bq27x00_battery_read_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27x00_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		//ret = bq27x00_simple_value(di->cache.temperature, val);
		val->intval = di->cache.temperature;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27x00_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		if (bq27425_get_battery_serial_number(di))
			return -EINVAL;
		else
			val->strval = bq27425_serial;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		//ret = bq27x00_simple_value(di->cache.health, val);
		val->intval = bq27x00_battery_read_health(di);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		//set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}

static int bq27425_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = container_of(psy, struct bq27x00_device_info, ac);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->ac_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq27425_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = container_of(psy, struct bq27x00_device_info, usb);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->usb_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

void bq27425_charger_status_on_suspend(enum charging_status status, void *data)
{
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)data;	
	if (status == BQ2419X_CHARGING_STATUS_FULL)	
		di->charging_status = POWER_SUPPLY_STATUS_FULL;
}
EXPORT_SYMBOL(bq27425_charger_status_on_suspend);

static void bq27425_charger_status(enum charging_status status, enum cable_type chrg_type, void *data)
{	
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)data;	
	
	mutex_lock(&di->update_lock);
	di->ac_online = 0;	
	di->usb_online = 0;	
	di->battery_online = 0;	
	//&*&*&*AL1_20130503 update_charging_health
	di->charging_health = status;
	//&*&*&*AL2_20130503 update_charging_health
	if (chrg_type == BQ2419X_AC_CABLE_SUPPLY)	
		di->ac_online = 1;	
	else if (chrg_type == BQ2419X_USB_CABLE_SUPPLY)	
		di->usb_online = 1;
	else
		di->battery_online = 1;
	
	if (status == BQ2419X_CHARGING_STATUS_CONNECT)	
		di->charging_status = POWER_SUPPLY_STATUS_CHARGING;	
	else if (status == BQ2419X_CHARGING_STATUS_FULL)	
		di->charging_status = POWER_SUPPLY_STATUS_FULL;	
	else if (status >= BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT)	
		//&*&*&*AL1_20130315 change charging status from NOT_CHARGING to DISCHARGING
		di->charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		//di->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;	
		//&*&*&*AL1_20130315 change charging status from NOT_CHARGING to DISCHARGING
	else
	{
		// already charging complete at boot-up 
		if((chrg_type >= BQ2419X_AC_CABLE_SUPPLY) && (di->cache.capacity >= 100))
			di->charging_status = POWER_SUPPLY_STATUS_FULL;
		else
			di->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	mutex_unlock(&di->update_lock);

	//&*&*&*AL1_20130220 don't update and access before firmware complete
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	if(!di->init_complete)
		return;
	#endif
	//&*&*&*AL2_20130220 don't update and access before firmware complete

	// update charging status, don't do bq27x00_update()
	if(di->bat.dev)
		power_supply_changed(&di->bat);	
		
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = bq27425_ac_props;
	di->ac.num_properties = ARRAY_SIZE(bq27425_ac_props);
	di->ac.get_property = bq27425_ac_get_property;
	di->ac.external_power_changed = NULL;
	ret = power_supply_register(di->dev, &di->ac);
	if (ret) {
		dev_err(di->dev, "failed to register ac\n");
	}

	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = bq27425_usb_props;
	di->usb.num_properties = ARRAY_SIZE(bq27425_usb_props);
	di->usb.get_property = bq27425_usb_get_property;
	di->usb.external_power_changed = NULL;
	ret = power_supply_register(di->dev, &di->usb);
	if (ret) {
		dev_err(di->dev, "failed to register usb\n");
	}

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	//di->chip = BQ27425;
	if (di->chip == BQ27425) {
		di->bat.properties = bq27425_battery_props;
		di->bat.num_properties = ARRAY_SIZE(bq27425_battery_props);
	} else {
		di->bat.properties = bq27x00_battery_props;
		di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	}
	di->bat.get_property = bq27x00_battery_get_property;
	//&*&*&*AL1_20130204take off the schedule work on bq27x00_external_power_changed, bq27x00_battery_poll can do.
	//di->bat.external_power_changed = bq27x00_external_power_changed;
	di->bat.external_power_changed = NULL;
	//&*&*&*AL2_20130204 take off the schedule work on bq27x00_external_power_changed, bq27x00_battery_poll can do.

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);
	mutex_init(&di->update_lock);

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	schedule_delayed_work(&di->work, 1);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	/*
	 * power_supply_unregister call bq27x00_battery_get_property which
	 * call bq27x00_battery_poll.
	 * Make sure that bq27x00_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
	mutex_destroy(&di->update_lock);
}


/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
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
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg,
				u16 val, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	unsigned char i2c_data[3];
	int ret, len;

	i2c_data[0] = reg;
	i2c_data[1] = val & 0xff;

	if (single) {
		len = 2;
	} else {
		i2c_data[2] = (val >> 8) & 0xff;
		len = 3;
	}

	ret = i2c_master_send(client, i2c_data, len);
	if (ret == len)
		return 0;

	return (ret < 0) ? ret : -EIO;
}

static int bq27x00_ctrl_read_i2c(struct bq27x00_device_info *di,
					u8 ctrl_reg, u16 ctrl_func_reg)
{
	int ret = bq27x00_write(di, ctrl_reg, ctrl_func_reg, false);
	if (ret < 0) {
		dev_err(di->dev, "write control reg failure : 0x%04x\n", ctrl_func_reg);
		return ret;
	}

	ret = bq27x00_read_i2c(di, ctrl_reg, false);
	//ret = bq27425_read_i2c(di, ctrl_reg, false);
	if (ret < 0) {
		dev_err(di->dev, "read control reg failure\n");
		return ret;
	}

	return ret;
}

static void bq27x00_low_bat_worker(struct work_struct *work)
{
	struct bq27x00_device_info *di = container_of(work, struct bq27x00_device_info, low_bat_delay_work.work);

	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	if(di->re_download)
	{
		di->re_download = 0;
		goto update_proccess;
	}
	#endif
	
	di->cache.capacity = bq27x00_battery_read_rsoc(di);
	if(di->cache.capacity <= BQ27425_LOW_BATTERY_SOC1)
	{
		if (di->low_bat_cb)
			di->low_bat_cb(di->low_bat_cb_data);
		
		//bq27x00_battery_capacity_level(di, val);
		dev_info(di->dev, "low bat alert (%d)\n", di->cache.capacity);
		bq27x00_update(di);
	}
#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
update_proccess:
#endif
	enable_irq(di->irq);
}

#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
static int bq27x00_battery_insert(struct bq27x00_device_info *di)
{
	u16 read_data;
	int timeout = 3;
	/* Send subcommand BAT_INSERT to indicate battery presentation */
	do {
		bq27x00_ctrl_read(di, BQ27425_REG_CNTL, BQ27425_CNTL_BAT_INSERT);
		msleep(2000);
		read_data = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
		dev_info(di->dev, "battery present: 0x%04x, count:%d\n", read_data, timeout);
	} while((!(read_data & BQ27500_FLAG_BAT_DET)) && --timeout);

	if(timeout == 0)
		return -EFAULT;
	else
		return 0;
}

static void bq27x00_download_firmware(struct work_struct *work)
{
	struct bq27x00_device_info *di = container_of(work, struct bq27x00_device_info, download_firmware_work.work);
	struct bq27x00_block_data_info *info;
	int ret;
	int fcc;

	if(!di->up_res)
		return;

	if(di->init_complete)
		cancel_delayed_work_sync(&di->work);

	wake_lock(&di->wakelock_download);
	ret = di->up_res->update_firmware(di->up_res);
	if(ret == 0)
	{
		info = kzalloc(sizeof(*info), GFP_KERNEL);
		if (!info) {
			dev_err(di->dev, "(%s)failed to allocate bq27x00_block_data_info data\n", __func__);
			goto init_finish;
		}

		fcc = bq27x00_read(di, 0x12, false);
		if(fcc < 7000)
		{
			dev_info(di->dev, "battery fullChargeCapacity is too low: %d\n", fcc);
			info->subclass = 82;
			info->offset = 12;
			info->length = 2;
			if(di->init_complete)
				di->reset_gauge = 1;
			info->data = 8100; 
			di->up_res->write_blockdate(di->up_res, info);
			msleep(6000);
			if(di->init_complete){
				di->re_download = 1;
				di->reset_gauge = 1;
			}
			ret = di->up_res->update_firmware(di->up_res);
		}
		/* 
		 * Fine tuning to meet chager complete 
		 * (reference DATA BLOCK SUMMARY of bq27425-g2 spec.)
		 */
		info->subclass = 82;
		info->offset = 32;
		info->length = 2;
		di->up_res->read_blockdata(di->up_res, info);
		dev_info(di->dev, "battery taper voltage: %d\n", info->data);
		if(info->data != 4100)
		{
			if(di->init_complete)
				di->reset_gauge = 1;
			info->data = 4100; 
			di->up_res->write_blockdate(di->up_res, info);
			msleep(5000);
		}
		info->subclass = 82;
		info->offset = 30;
		info->length = 2;
		di->up_res->read_blockdata(di->up_res, info);
		dev_info(di->dev, "battery taper current: %d\n", info->data);
		if(info->data != 300 || di->init_complete)
		{
			if(di->init_complete)
				di->reset_gauge = 1;
			info->data = 300; 
			di->up_res->write_blockdate(di->up_res, info);
			msleep(5000);
		}
		kfree(info);
	}

init_finish:
	ret = bq27x00_battery_insert(di);
	//&*&*&*AL1_20130315 to avoid entering rom mode not expectantly.
	if(ret < 0)
	{
		ret = di->up_res->check_rom_mode(di->up_res);
		if(ret < 0)
			dev_info(di->dev, "NOT in rom mode: %d, read i2c 0x16 failure.\n", ret);
		else
		{
			dev_info(di->dev, "IN rom mode: %d, read i2c 0x16 success, and already left rom mode\n", ret);
			ret = bq27x00_battery_insert(di);
		}
	}
	//&*&*&*AL2_20130315 to avoid entering rom mode not expectantly.
	wake_unlock(&di->wakelock_download);
	di->init_complete = 1;
	schedule_delayed_work(&di->work, 1 * HZ);   // update the battery status
}
#endif

static irqreturn_t bq27x00_low_bat_irq_handler(int irq, void *data)
{
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)data;

	disable_irq_nosync(irq);
	schedule_delayed_work(&di->low_bat_delay_work, 10);

	return IRQ_HANDLED;
}

static int bq27x00_gpio_and_irq_init(struct bq27x00_device_info *di)
{
	int gpio_num, ret;
	gpio_num = TEGRA_IRQ_TO_GPIO(di->irq);
		
	ret = gpio_request(gpio_num, "bq27425 GPOUT");
	if (ret) {
		dev_err(di->dev, "couldn't request bq27425 gpio: %d\n", gpio_num);
		goto err_request;
	}
	ret = gpio_direction_input(gpio_num);

	#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra_gpio_enable(gpio_num);
	#endif

	INIT_DELAYED_WORK(&di->low_bat_delay_work, bq27x00_low_bat_worker);
	
	ret = request_threaded_irq(di->irq, NULL, bq27x00_low_bat_irq_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "Low Bat", di);
	if (ret) {			
		dev_err(di->dev, "can't allocate bq27425 low bat irq %d\n", di->irq);
		goto err_request;		
	}
	
	enable_irq_wake(di->irq);

err_request:
	return ret;

}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	#ifndef CONFIG_BATTERY_BQ27x00_UPDATE
	u16 read_data;
	int timeout = 3;
	#endif
	int retval = 0;
	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	struct bq27x00_update_access_resource *up_res;
	#endif

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	g_bq27x00_di = di;

	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27x00_read_i2c;
	di->bus.ctrl_read = &bq27x00_ctrl_read_i2c;
	di->bus.write = &bq27x00_write_i2c;
	di->irq = client->irq;
	di->is_debug = 1;

	/* Let's see whether this adapter can support what we need. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "insufficient functionality!\n");
		retval = -ENODEV;
		goto batt_failed_3;
	}

	#ifdef CONFIG_BATTERY_BQ27x00_UPDATE
	wake_lock_init(&di->wakelock_download, WAKE_LOCK_SUSPEND, "bq27x00-update");
	up_res = get_bq27x00_update_instance();
	if(up_res)
	{
		retval = up_res->open(up_res, client);
		if(!retval)
			di->up_res = up_res;
		INIT_DELAYED_WORK(&di->download_firmware_work, bq27x00_download_firmware);
		schedule_delayed_work(&di->download_firmware_work, HZ*1);
	}
	#endif

	#ifndef CONFIG_BATTERY_BQ27x00_UPDATE
	/* Send subcommand BAT_INSERT to indicate battery presentation */
	do {
		bq27x00_ctrl_read(di, BQ27425_REG_CNTL, BQ27425_CNTL_BAT_INSERT);
		msleep(2000);
		read_data = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
		dev_err(&client->dev, "battery present: 0x%04x, count:%d\n", read_data, timeout);
	} while((!(read_data & BQ27500_FLAG_BAT_DET)) && --timeout);
	#endif
	
	//&*&*&*HC1_20120525
	#ifdef CONFIG_NABI_FAKE_BATTERY
	di->fake_battery = is_fake_battery();
	#endif
	//&*&*&*HC2_20120525

	//&*&*&*AL1_20130429 add debug-on attribute to track battery capacity. 
	retval = sysfs_create_group(&client->dev.kobj, &bq27425_attr_group);
	if (retval)
		dev_err(&client->dev, "%s: error in creating sysfs attribute" , __func__);
	//&*&*&*AL2_20130429 add debug-on attribute to track battery capacity. 
	
	retval = bq27x00_powersupply_init(di);
	if (retval < 0)
		goto batt_failed_3;

	retval = register_charger_callback(bq27425_charger_status, di);
	if (retval < 0)		
		dev_err(&client->dev, "register bq2419x callback error\n");

	i2c_set_clientdata(client, di);

	bq27x00_gpio_and_irq_init(di);

	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27x00_battery_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);
	int ret = 0;

	cancel_delayed_work_sync(&di->work);
	
	ret = cancel_delayed_work_sync(&di->low_bat_delay_work);
	dev_info(di->dev, "cancel low bat alert handler is %d\n", ret);
	if(ret)
		enable_irq(di->irq);

	return 0;
}

static int bq27x00_battery_resume(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	schedule_delayed_work(&di->work, 6*HZ);

	return 0;
}

#endif

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
	{ "bq27425", BQ27425 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static struct i2c_driver bq27x00_battery_driver = {
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
	.driver = {
		.name = "bq27x00-battery",
	},
#if defined(CONFIG_PM)
	.suspend = bq27x00_battery_suspend,
	.resume = bq27x00_battery_resume,
#endif
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#else

static inline int bq27x00_battery_i2c_init(void) { return 0; }
static inline void bq27x00_battery_i2c_exit(void) {};

#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
			bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int __devinit bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27000;

	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

static int __devexit bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static struct platform_driver bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = __devexit_p(bq27000_battery_remove),
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void) { return 0; }
static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
