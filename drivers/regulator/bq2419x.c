/*
 * Support for TI bq24190, bq24192, bq24192i, bq24193, bq24196 Single Input with 4.5A charging current (USB/AC Adpater)
 * 1-Cell Battery Charger with OTG .
 *
 * Copyright (c) 2012 Foxconn, Aimar Liu
 * First version created on 2012.04
 * Modify for Tegra3 by Aimar Liu 2012.12
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/regulator/bq2419x.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#define BQ2419X_LED_CONTROL 1

#define BQ2419X_INPUT_SOURCE_CONTROL          0x00
#define BQ2419X_POWER_ON_CONFIGURATION        0x01
#define BQ2419X_CHARGE_CURRENT                0x02
#define BQ2419X_PRE_CHARGE_AND_TERM_CURRENT	  0x03
#define BQ2419X_CHARGE_VOLTAGE      	        0x04
#define BQ2419X_CHARGE_TERM_AND_TIMER  	      0x05
#define BQ2419X_IR_THERMAL_THRESHOLD	        0x06
#define BQ2419X_MISC_OPERATION	              0x07
#define BQ2419X_SYSTEM_STATUS	                0x08
#define BQ2419X_FAULT	                        0x09
#define BQ2419X_RVERSION	                    0x0A

#define BQ2419X_USB_CURRENT_LIMIT             500
#define BQ2419X_AC_CHARGING_CURRENT_LIMIT     1800
#define BQ2419X_AC_INPUT_CURRENT_LIMIT        2000

#define BQ2419X_ILIM              (7 << 0)
#define BQ2419X_ILIM_500          (2 << 0)
#define BQ2419X_ILIM_1500         (5 << 0)
#define BQ2419X_ILIM_2000         (6 << 0)

#define BQ2419X_CHARGING_STATUS  (3 << 4)
#define BQ2419X_NOT_CHARGING     0
#define BQ2419X_PRE_CHARGING     1
#define BQ2419X_FAST_CHARGING    2
#define BQ2419X_TERMINATION_DONE 3

#define BQ2419X_VBUS_VALID       (1 << 2)
#define BQ2419X_VBUS_STATUS      (3 << 6)
#define BQ2419X_BATTERY          0
#define BQ2419X_USB_HOST         1
#define BQ2419X_AC_ADAPTER       2
#define BQ2419X_OTG              3

#define BQ2419X_CHG_CONFIG       (3 << 4)
#define BQ2419X_CHARGE_DISABLE   (0 << 4)
#define BQ2419X_CHARGE_BATTERY   (1 << 4)
#define BQ2419X_CHARGE_OTG       (2 << 4)

#define BQ2419X_WATCHDOG_TIMEOUT (1 << 7)
#define BQ2419X_CHARGE_FAULT (3 << 4)
#define BQ2419X_INPUT_FAULT 1
#define BQ2419X_THERMAL_SHUTDOWN 2
#define BQ2419X_CHARGE_SAFETY_TIMER_EXPIRATION 3
#define BQ2419X_BAT_FAULT (1 << 3)
#define BQ2419X_NTC_FAULT (3 << 0)
#define BQ2419X_TS1_COLD 1
#define BQ2419X_TS1_HOT 2

struct bq2419x_chip {
	struct i2c_client		*client;
	struct bq2419x_mach_info	*pdata;
	
	struct regulator_dev 	*regu_dev;
	struct regulator_dev  *otg_regu_dev;
	
	struct delayed_work		bq2419x_interrupt_delay_work;
	//&*&*&*AL1_20130201 add delay_work for bootup update
	struct delayed_work		bq2419x_update_delay_work;
	//&*&*&*AL2_20130201 add delay_work for bootup update
	struct early_suspend  early_suspend;
	struct mutex mutex;	/* reentrant protection for struct */
	
	/* Update to Gas-Gauge */
	void	*charger_cb_data;
	charge_callback_t	charger_cb;
	
	/* State Of Connect */
	enum cable_type online_cable_type;
	enum charging_status online_charging_status;
	enum charging_mode online_mode;
	int is_suspend;
	int is_charger_enable;
};

static struct bq2419x_chip *bq2419x_charger = NULL;

static void bq2419x_update_status(struct bq2419x_chip *chip);
static void bq2419x_early_suspend(struct early_suspend *h);
static void bq2419x_late_resume(struct early_suspend *h);
extern void max77663_OTG_alert(void* data);
extern void bq27425_charger_status_on_suspend(enum charging_status status, void *data);

int otg_removed = 0;
EXPORT_SYMBOL(otg_removed);

static int bq2419x_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d, reg addr 0x%02X\n", __func__, ret, reg);

	return ret;
}

static int bq2419x_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d, reg addr 0x%02X\n", __func__, ret, reg);

	return ret;
}

static int bq2419x_charger_enable(struct bq2419x_chip *chip, int enable)
{
	struct i2c_client *client = chip->client;
	int val = 0;
	int ret = 0;

	val = bq2419x_read_reg(client, BQ2419X_POWER_ON_CONFIGURATION);
	//&*&*&*AL1_20130206 It should be cleaned before configuration
	val = val & ~BQ2419X_CHG_CONFIG;
	//&*&*&*AL2_20130206 It should be cleaned before configuration
	if(enable)
		val = val | BQ2419X_CHARGE_BATTERY;
	else
		val = val | BQ2419X_CHARGE_DISABLE;
	ret = bq2419x_write_reg(client, BQ2419X_POWER_ON_CONFIGURATION, val);
	if (ret < 0){
		dev_err(&client->dev, "%s: BQ2419X config charge as %d fail!!(0x%02x)\n", __func__, enable, val);
		return ret;
	}
	
	dev_info(&client->dev, "%s: BQ2419X Charge %s(0x%02x)\n", __func__, enable ? "Enable":"Disable", val);
	msleep(100);
	bq2419x_update_status(chip);

	//&*&*&*AL1_20130301 to let battery discharging with AC cable for diag testing
	val = bq2419x_read_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL);
	val = val & ~BQ2419X_ILIM;
	if(enable){
		if(chip->online_cable_type == BQ2419X_AC_CABLE_SUPPLY)
			val = val | BQ2419X_ILIM_2000;
		else
			val = val | BQ2419X_ILIM_500;
	}
	ret = bq2419x_write_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL, val);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: BQ2419X config ILIM to 1.5A fail!!(0x%02x)\n", __func__, val);
	//&*&*&*AL1_20130301 to let battery discharging with AC cable for diag testing

	//&*&*&*AL1_20130421 to avoid cable plug-in/out during run-in testing.
	if(enable)
		chip->is_charger_enable = 1;
	else
		chip->is_charger_enable = 0;
	//&*&*&*AL2_20130421 to avoid cable plug-in/out during run-in testing.
	
	return 0;
}

static ssize_t bq2419x_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bq2419x_chip *chip = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&chip->mutex);
	if (val)
		bq2419x_charger_enable(chip, 1);
	else 
		bq2419x_charger_enable(chip, 0);
	mutex_unlock(&chip->mutex);

	return count;
}
static DEVICE_ATTR(enable, 0664, NULL, bq2419x_enable_store);

static ssize_t bq2419x_dump_reg_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bq2419x_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = chip->client;
	unsigned long val;
	char *tok;
	int first = 1, reg = 0;
	int error, ret;
	
	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = strict_strtoul(tok, 10, &val);
		if (error)
			return error;
		if(first){
			reg = val;
			first = 0;
		}
	}
	ret = bq2419x_write_reg(client, reg, val);
	if (ret < 0){
		dev_err(&chip->client->dev, "%s: BQ2419X Write 0x%02x : 0x%02x fail!!\n", __func__, reg, (int)val);
		return ret;
	}
	
	ret = bq2419x_read_reg(client, reg);
	dev_info(&chip->client->dev, "0x%02x : 0x%02x\n", reg, ret);
	
	return count;
}
	
static ssize_t bq2419x_dump_reg_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bq2419x_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = chip->client;
	int reg;
	u8 val = 0x00;
	
	dev_info(&chip->client->dev, "%s: Registers: \n", __func__);
	printk("=============================\n");
	for(reg = 0; reg < 10; reg++)
	{
		val = bq2419x_read_reg(client, reg);
		printk("[0x%02x] : 0x%02x\n", reg, val);
	}
	printk("=============================\n");
	
	return 1;
}
static DEVICE_ATTR(dump_reg, 0664, bq2419x_dump_reg_show, bq2419x_dump_reg_store);

static struct attribute *bq2419x_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_dump_reg.attr,
	NULL
};

static const struct attribute_group bq2419x_attr_group = {
	.attrs = bq2419x_attributes,
};

int register_charger_callback(charge_callback_t cb, void *args)
{
	struct bq2419x_chip *chip = bq2419x_charger;
	if (!chip)
		return -ENODEV;

	chip->charger_cb = cb;
	chip->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_charger_callback);

int update_charger_status_to_gasgauge(struct bq2419x_chip *chip)
{
	//&*&*&*AL1_20130625 to avoid device hang while suspend
	if(chip->is_suspend)
	{
		if (chip->charger_cb && chip->charger_cb_data)
			bq27425_charger_status_on_suspend(chip->online_charging_status, chip->charger_cb_data);
		return 0;
	}
	#if 0
	if(chip->is_suspend && (chip->online_cable_type < BQ2419X_AC_CABLE_SUPPLY))
		return 0;
	#endif
	//&*&*&*AL2_20130625 to avoid device hang while suspend
	
	if (chip->charger_cb)
	{
		chip->charger_cb(chip->online_charging_status, chip->online_cable_type,
						chip->charger_cb_data);
		//&*&*&*AL1_20130516 to control led while charging fault
		#ifdef BQ2419X_LED_CONTROL
		if(chip->online_charging_status >= BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT)
		{
			dev_err(&chip->client->dev, "%s: Got something fault(status %d, type %d) to re-detect again in 10sec later.\n", __func__, chip->online_charging_status, chip->online_cable_type);
			schedule_delayed_work(&chip->bq2419x_update_delay_work, 10*HZ);
		}
		#endif 
		//&*&*&*AL2_20130516 to control led while charging fault
	}
	else
	{
		dev_err(&chip->client->dev, "%s: Gauge hasn't registered on bootup yet. To report again in 3000ms later.\n", __func__);
		schedule_delayed_work(&chip->bq2419x_update_delay_work, 10*HZ);
	}
	
	return 0;
}

static void bq2419x_update_status(struct bq2419x_chip *chip)
{
	u8 value = 0;
	
	value = bq2419x_read_reg(chip->client, BQ2419X_SYSTEM_STATUS);
	//dev_info(&chip->client->dev, "%s: charger status 0x%02x\n", __func__, value);
	if(value & BQ2419X_VBUS_VALID){
			chip->online_mode = BQ2419X_BATTERY_CHARGING_MODE;
			dev_info(&chip->client->dev, "%s: %s\n", __func__, "BATTERY CHARGING MODE");
			switch((value & BQ2419X_CHARGING_STATUS) >> 4){
				case BQ2419X_NOT_CHARGING:
					//&*&*&*AL1_20130516 to control led while charging fault
					#ifdef BQ2419X_LED_CONTROL
					if(chip->online_charging_status >= BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT)
						;
					else
					#endif 
					//&*&*&*AL2_20130516 to control led while charging fault
						chip->online_charging_status = BQ2419X_CHARGING_STATUS_DISCONNECT;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "CHARGING STATUS DISCONNECT");
					break;
				case BQ2419X_PRE_CHARGING:
				case BQ2419X_FAST_CHARGING:
					chip->online_charging_status = BQ2419X_CHARGING_STATUS_CONNECT;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "CHARGING STATUS CONNECT");
					break;
				case BQ2419X_TERMINATION_DONE:
					chip->online_charging_status = BQ2419X_CHARGING_STATUS_FULL;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "CHARGING STATUS FULL");
					break;
				default:
					dev_err(&chip->client->dev, "%s: didn't match any charing status.\n", __func__);
					break;
			}
			switch((value & BQ2419X_VBUS_STATUS) >> 6){
				case BQ2419X_BATTERY:
					chip->online_cable_type = BQ2419X_BATTERY_SUPPLY;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "BATTERY SUPPLY");
					break;
				case BQ2419X_USB_HOST:
					chip->online_cable_type = BQ2419X_USB_CABLE_SUPPLY;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "USB SUPPLY");
					break;
				case BQ2419X_AC_ADAPTER:
					chip->online_cable_type = BQ2419X_AC_CABLE_SUPPLY;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "AC SUPPLY");
					break;
				case BQ2419X_OTG:
					chip->online_mode = BQ2419X_BOOST_MODE;
					chip->online_cable_type = BQ2419X_ID_BOOST_SUPPLY;
					dev_info(&chip->client->dev, "%s: %s\n", __func__, "BOOST SUPPLY");
					dev_info(&chip->client->dev, "%s: change to %s\n", __func__, "BOOST MODE");
					break;
				default:
					dev_err(&chip->client->dev, "%s: didn't match any cable type.\n", __func__);
					break;
			}
	}
	//&*&*&*AL1_20130207, indicate cable type and charging status on OTG mode
	else if(((value & BQ2419X_VBUS_STATUS) >> 6) == BQ2419X_OTG){
		chip->online_mode = BQ2419X_BOOST_MODE;
		dev_info(&chip->client->dev, "%s: change to %s\n", __func__, "BOOST MODE");
		chip->online_charging_status = BQ2419X_CHARGING_STATUS_DISCONNECT;
		dev_info(&chip->client->dev, "%s: %s\n", __func__, "CHARGING STATUS DISCONNECT");
		chip->online_cable_type = BQ2419X_ID_BOOST_SUPPLY;
		dev_info(&chip->client->dev, "%s: %s\n", __func__, "BOOST SUPPLY");
	}
	//&*&*&*AL2_20130207, indicate cable type and charging status on OTG mode
	else{
		chip->online_mode = BQ2419X_SUPPLEMENT_MODE;
		dev_info(&chip->client->dev, "%s: %s\n", __func__, "SUPPLEMENT MODE");
		chip->online_charging_status = BQ2419X_CHARGING_STATUS_DISCONNECT;
		dev_info(&chip->client->dev, "%s: %s\n", __func__, "CHARGING STATUS DISCONNECT");
		chip->online_cable_type = BQ2419X_BATTERY_SUPPLY;
		dev_info(&chip->client->dev, "%s: %s\n", __func__, "BATTERY SUPPLY");
	}
	
	update_charger_status_to_gasgauge(chip);
}

static void bq2419x_process_fault(struct bq2419x_chip *chip, u8 value)
{
	u8 fault_value = value;
	dev_err(&chip->client->dev, "%s: charger fault (err: 0x%02x)\n", __func__, value);

	//chip->online_charging_status = BQ2419X_CHARGING_STATUS_STOP;
	
	switch((fault_value & BQ2419X_CHARGE_FAULT ) >> 4)
	{
		case BQ2419X_INPUT_FAULT:
			dev_err(&chip->client->dev, "%s: Input fault(OVP or bad source).\n", __func__);
			chip->online_charging_status = BQ2419X_CHARGING_STATUS_DISCONNECT;
			break;
		case BQ2419X_THERMAL_SHUTDOWN:
			dev_err(&chip->client->dev, "%s: Thermal shutdown.\n", __func__);
			chip->online_charging_status = BQ2419X_CHARGING_STATUS_STOP;
			break;
		case BQ2419X_CHARGE_SAFETY_TIMER_EXPIRATION:
			dev_err(&chip->client->dev, "%s: Charge timer expiration.\n", __func__);
			chip->online_charging_status = BQ2419X_CHARGING_STATUS_FAULT_TIMEOUT;
			break;
		default:
			break;
	}
	if(fault_value & BQ2419X_BAT_FAULT)
	{
		dev_err(&chip->client->dev, "%s: BAT OVP fault.\n", __func__);
		chip->online_charging_status = BQ2419X_CHARGING_STATUS_FAULT_BATTERY;
	}
	switch((fault_value & BQ2419X_NTC_FAULT ))
	{
		case BQ2419X_TS1_COLD:
			dev_err(&chip->client->dev, "%s: Battery Cold.\n", __func__);
			chip->online_charging_status = BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_COLD;
			break;
		case BQ2419X_TS1_HOT:
			dev_err(&chip->client->dev, "%s: Battery Hot.\n", __func__);
			chip->online_charging_status = BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT;
			break;
		default:
			break;
	}
	
	update_charger_status_to_gasgauge(chip);
}

static void bq2419x_update_delay_worker(struct work_struct *work)
{
	struct bq2419x_chip *chip = container_of(work, struct bq2419x_chip, bq2419x_update_delay_work.work);

	bq2419x_update_status(chip);
}

static void bq2419x_irq_delay_worker_interrupt(struct work_struct *work)
{
	struct bq2419x_chip *chip = container_of(work, struct bq2419x_chip, bq2419x_interrupt_delay_work.work);
	struct bq2419x_mach_info *pdata = chip->pdata;
	u8 fault_value, state_value;

	fault_value = bq2419x_read_reg(chip->client, BQ2419X_FAULT);
	state_value = bq2419x_read_reg(chip->client, BQ2419X_SYSTEM_STATUS);
	
	if(((state_value & BQ2419X_CHARGING_STATUS) >> 4) == BQ2419X_TERMINATION_DONE)
		bq2419x_update_status(chip);

	if(fault_value != 0)
		bq2419x_process_fault(chip, fault_value);

	enable_irq(gpio_to_irq(pdata->pin_int));
}

static irqreturn_t bq2419x_int_irq_handler(int irq, void *data)
{
	struct bq2419x_chip *chip = (struct bq2419x_chip *)data;

	disable_irq_nosync(irq);
	schedule_delayed_work(&chip->bq2419x_interrupt_delay_work, 20);

	return IRQ_HANDLED;
}

static int bq2419x_configure_charge(struct bq2419x_chip *chip, int max_uA)
{
		struct bq2419x_mach_info *pdata = chip->pdata;
		int val, ret;
		
		if(max_uA >= BQ2419X_AC_CHARGING_CURRENT_LIMIT)
		{
			gpio_set_value(pdata->pin_psel, 1);
			//&*&*&*AL1_20130129 add to config ILIMIT
			val = bq2419x_read_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL);
			val = val & ~BQ2419X_ILIM;
			val = val | BQ2419X_ILIM_2000;
			ret = bq2419x_write_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL, val);
			if (ret < 0)
				dev_err(&chip->client->dev, "%s: BQ2419X config ILIM to 1.5A fail!!(0x%02x)\n", __func__, val);
			//&*&*&*AL1_20130129 add to config ILIMIT
		}
		else
			gpio_set_value(pdata->pin_psel, 0);

		msleep(100);
		
		//&*&*&*AL1_20130507 disable fast charge timer during usb charging
		if(max_uA >= BQ2419X_AC_CHARGING_CURRENT_LIMIT)
		{
			val = bq2419x_read_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER);
			val = val | (1 << 3);  // Enable Charging Safety Timer REG05 bit[3]  
			bq2419x_write_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER, val);
		}
		else
		{
			val = bq2419x_read_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER);
			val = val & ~(1 << 3);  // Disable Charging Safety Timer REG05 bit[3]  
			bq2419x_write_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER, val);
		}
		//&*&*&*AL2_20130507 disable fast charge timer during usb charging
			
		//&*&*&*AL1_20130421 to avoid cable plug-in/out during run-in testing.
		if(!chip->is_charger_enable)
		{
			val = bq2419x_read_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL);
			val = val & ~BQ2419X_ILIM;
			ret = bq2419x_write_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL, val);
			if (ret < 0)
				dev_err(&chip->client->dev, "%s: BQ2419X config ILIM to 100mA fail!!(0x%02x)\n", __func__, val);
		}
		//&*&*&*AL2_20130421 to avoid cable plug-in/out during run-in testing.

		return 0;
}

static int bq2419x_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct bq2419x_chip *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	int ret = 0;
	int maxima_uA = max_uA/1000;
	
	if (!maxima_uA) {
		/* Disable charger */
		ret = bq2419x_configure_charge(chip, maxima_uA);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error in configuring"
				"charge to disable\n", __func__);
			return ret;
		}
		dev_info(&client->dev, "%s: set charge current to %dmA(Disable charging)\n", __func__, maxima_uA);
	}
	else {
		if(maxima_uA == BQ2419X_USB_CURRENT_LIMIT)
			//&*&*&*AL1_20130421 to avoid cable plug-in/out during run-in testing.
			ret = bq2419x_configure_charge(chip, maxima_uA);
			//&*&*&*AL1_20130421 to avoid cable plug-in/out during run-in testing.
		else if(maxima_uA == BQ2419X_AC_CHARGING_CURRENT_LIMIT)
		{
			/* configure charger */
			ret = bq2419x_configure_charge(chip, maxima_uA);
			if (ret < 0) {
				dev_err(&client->dev, "%s: error in configuring"
					"charge to enable\n", __func__);
				return ret;
			}
		}
		dev_info(&client->dev, "%s: set charging current to %dmA(Enable charging)\n", __func__, (maxima_uA==1800)?1500:500 );
	}

	bq2419x_update_status(chip);
	
	return 0;
}

static int bq2419x_is_otg_enabled(struct regulator_dev *otg_rdev)
{	
	struct bq2419x_chip *chip = rdev_get_drvdata(otg_rdev);
	int ret = 0;

	if(chip->online_mode == BQ2419X_BOOST_MODE)
		ret = 1;
		
	return ret;
}

static int bq2419x_enable_otg(struct regulator_dev *otg_rdev)
{
	struct bq2419x_chip *chip = rdev_get_drvdata(otg_rdev);
	int val = 0;
	int ret = 0;

	if (!chip)
		return -EFAULT;

	//to distinguish OTG and AC/USB cable on max77663 PMIC
	val = 1;
	max77663_OTG_alert(&val);
	
	val = bq2419x_read_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION);
	val = val & ~BQ2419X_CHG_CONFIG;
	val = val | BQ2419X_CHARGE_OTG;
	ret = bq2419x_write_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION, val);
	if (ret < 0){
		dev_err(&chip->client->dev, "%s: BQ2419X config OTG to enable fail!!(0x%02x)\n", __func__, val);
		return ret;
	}
	
	dev_info(&chip->client->dev, "BQ2419X OTG Enable(0x%02x)\n", val);
	msleep(300); //change 300ms to wait for status updat by charger
	bq2419x_update_status(chip);
	
	return 0;
}

static int bq2419x_disable_otg(struct regulator_dev *otg_rdev)
{
	struct bq2419x_chip *chip = rdev_get_drvdata(otg_rdev);
	int val = 0;
	int ret = 0;

	if (!chip)
		return -EFAULT;

	val = bq2419x_read_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION);
	val = val & ~BQ2419X_CHG_CONFIG;
	val = val | BQ2419X_CHARGE_BATTERY;
	bq2419x_write_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION, val);
	if (ret < 0){
		dev_err(&chip->client->dev, "%s: BQ2419X config OTG to disable fail!!(0x%02x)\n", __func__, val);
		return ret;
	}
	
	dev_info(&chip->client->dev, "BQ2419X OTG Disable(0x%02x)\n", val);
	
	//to distinguish OTG and AC/USB cable on max77663 PMIC
	msleep(100); //Let AC_OK irq doing first before setting the flag
	val = 0;
	max77663_OTG_alert(&val);
	bq2419x_update_status(chip);

	return 0;
}

static void bq2419x_device_init(struct bq2419x_chip *chip)
{
	u8 value;

	/* let bq24190 enter host mode then stay the mode */
	value = bq2419x_read_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION);
	value = value | (1 << 6);  // Reset REG01 bit[6]  
	bq2419x_write_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION, value);
	value = bq2419x_read_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER);
	value = value & ~(3 << 4);  // Disable Watchdog REG05 bit[4:5]  
	bq2419x_write_reg(chip->client, BQ2419X_CHARGE_TERM_AND_TIMER, value);
	/* Config input current limit */
	value = bq2419x_read_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL);
	value = value & 0xF6;  // Config input current limit to 2A (REG00 bit[0:2] : 110)  
	bq2419x_write_reg(chip->client, BQ2419X_INPUT_SOURCE_CONTROL, value);
	/* Config charging current limit to 1.92A(REG00 bit[2:7] : 010110) */ 
	//bq2419x_write_reg(chip->client, BQ2419X_CHARGE_CURRENT, 0x30);
	bq2419x_write_reg(chip->client, BQ2419X_CHARGE_CURRENT, 0x58);
	/* Config termination current limit to 256mA(REG03 bit[0:3] : 0001) */ 
	value = bq2419x_read_reg(chip->client, BQ2419X_PRE_CHARGE_AND_TERM_CURRENT);
	value = value & 0xF0;  // REG03 bit[0:3] = 0x0  
	value = value | 0x01;  // REG03 bit[0:3] = 0001
	bq2419x_write_reg(chip->client, BQ2419X_PRE_CHARGE_AND_TERM_CURRENT, value);
	/* Enable battery charging as default */
	//&*&*&*AL1_20130421 to avoid cable plug-in/out during run-in testing.
	chip->is_charger_enable = 1;
	//&*&*&*AL2_20130421 to avoid cable plug-in/out during run-in testing.
	value = bq2419x_read_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION);
	value = value & ~BQ2419X_CHG_CONFIG;
	value = value | BQ2419X_CHARGE_BATTERY;
	bq2419x_write_reg(chip->client, BQ2419X_POWER_ON_CONFIGURATION, value);
	
	/*avoid to bootup mis-detection */
	bq2419x_update_status(chip);
}

static int bq2419x_gpio_and_irq_init(struct bq2419x_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct bq2419x_mach_info *pdata = chip->pdata; 
	int ret, irq_num;
	
	/*
	If the input is detected as USB host, OTG=high, input limit=500mA and when OTG=low, input limit=100mA.
	During boost operation, the OTG=high, suspend and when OTG=low, boost operation.
	note : OTG pin always high.
	*/
	ret = gpio_request(pdata->pin_psel, "charging_input_limit");
	if (ret) {
		dev_err(&client->dev, "%s: couldn't request PSEL gpio: %d\n",
			__func__, pdata->pin_psel);
		goto err_gpio_request;
	}
	ret = gpio_direction_output(pdata->pin_psel, 0);

	ret = gpio_request(pdata->pin_int, "charge_int");
	if (ret) {
		dev_err(&client->dev, "%s: couldn't request charge INT GPIO: %d\n",
			__func__, pdata->pin_int);
		goto err_gpio_request_2;
	}
	ret = gpio_direction_input(pdata->pin_int);
	//request bq2419x interrrupt
	irq_num = gpio_to_irq(pdata->pin_int); 

	ret = request_threaded_irq(irq_num, NULL, bq2419x_int_irq_handler, 
		IRQF_TRIGGER_FALLING, "bq2419x_charger", chip);
	if (ret) {			
		dev_err(&chip->client->dev, "%s: can't allocate bq2419x int pin's irq %d\n", __func__, irq_num);
		goto err_request_irq;		
	}
	enable_irq_wake(irq_num);

	return 0;

err_request_irq:
	gpio_free(pdata->pin_int);
err_gpio_request_2:
	gpio_free(pdata->pin_psel);
err_gpio_request:
	return ret;
}

static int bq2419x_get_version(struct i2c_client *client)
{
	u8 chip_ver = 0;
	int ret = 0;

  chip_ver = bq2419x_read_reg(client, BQ2419X_RVERSION);
		
	switch (chip_ver)
	{
		case 0x23:
			dev_info(&client->dev, "BQ24190 USB/Adaptor Charger(0x%02x) ", chip_ver);
			break;
		case 0x2B:
			dev_info(&client->dev, "BQ24192/BQ24193/BQ24196 USB/Adaptor Charger(0x%02x) ", chip_ver);
			break;
		case 0x1B:
			dev_info(&client->dev, "BQ24192I USB/Adaptor Charger(0x%02x) ", chip_ver);
			break;
		default:
			dev_info(&client->dev, "Doesn't detect any device(0x%02x) ", chip_ver);
			ret = -1;
			break;
	}
	
	return ret;
}

//&*&*&*AL1_20130207, add reboot notifier for warm-reset
static int bq2419x_prepare_for_shutdown(struct notifier_block *this, 
		unsigned long code, void *unused)
{
	if(!bq2419x_charger || !bq2419x_charger->otg_regu_dev)
		return NOTIFY_DONE;
	
	dev_info(&bq2419x_charger->client->dev, "cut off OTG power before shut down.\n");
	
	if(bq2419x_is_otg_enabled(bq2419x_charger->otg_regu_dev))
		bq2419x_disable_otg(bq2419x_charger->otg_regu_dev);

	return NOTIFY_DONE;
}

static struct notifier_block bq2419x_shutdown_notifier = {
		.notifier_call = bq2419x_prepare_for_shutdown,
		.next = NULL,
		.priority = 0
};
//&*&*&*AL2_20130207, add reboot notifier for warm-reset

static struct regulator_ops bq2419x_regulator_ops = {
	.set_current_limit = bq2419x_set_current_limit,
};

static struct regulator_desc bq2419x_regulator_desc = {
	.name  = "vbus_charger",
	.ops   = &bq2419x_regulator_ops,
	.type  = REGULATOR_CURRENT,
	.owner = THIS_MODULE,
};

static struct regulator_ops bq2419x_otg_regulator_ops = {
	.enable = bq2419x_enable_otg,	
	.disable = bq2419x_disable_otg,	
	.is_enabled = bq2419x_is_otg_enabled,
};

static struct regulator_desc bq2419x_otg_regulator_desc = {
	.name  = "vbus_otg",
	.ops   = &bq2419x_otg_regulator_ops,
	.type  = REGULATOR_CURRENT,
	.owner = THIS_MODULE,
};

static int __devinit bq2419x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bq2419x_mach_info *pdata = client->dev.platform_data;
	struct bq2419x_chip *chip;
	int ret;
	
	if (!pdata || !pdata->pin_int)
		return -EINVAL;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	if(bq2419x_get_version(client))
		return -ENODEV;

	chip = kzalloc(sizeof(struct bq2419x_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

 	chip->client = client;
	chip->pdata = pdata;
	bq2419x_charger = chip;
	
	i2c_set_clientdata(client, chip);
	INIT_DELAYED_WORK(&chip->bq2419x_interrupt_delay_work, bq2419x_irq_delay_worker_interrupt);
	INIT_DELAYED_WORK(&chip->bq2419x_update_delay_work, bq2419x_update_delay_worker);

	chip->regu_dev = regulator_register(&bq2419x_regulator_desc, &client->dev, pdata->init_data, chip);
	if (IS_ERR(chip->regu_dev)) {
		dev_err(&client->dev, "%s: failed to register: %s\n", __func__, bq2419x_regulator_desc.name);
		ret = PTR_ERR(chip->regu_dev);
		goto err_vbus_reg;
	}
	
	chip->otg_regu_dev = regulator_register(&bq2419x_otg_regulator_desc, &client->dev, pdata->otg_init_data, chip);
	if (IS_ERR(chip->otg_regu_dev)) {
		dev_err(&client->dev, "%s: failed to register: %s\n", __func__, bq2419x_otg_regulator_desc.name);
		ret = PTR_ERR(chip->otg_regu_dev);
		goto err_otg_reg;
	}

	#if defined (CONFIG_HAS_EARLYSUSPEND)
	chip->early_suspend.suspend = bq2419x_early_suspend;
	chip->early_suspend.resume = bq2419x_late_resume;
	register_early_suspend(&chip->early_suspend);
	#endif

	//&*&*&*AL1_20130207, add reboot notifier for warm-reset
	ret = register_reboot_notifier(&bq2419x_shutdown_notifier);
	if (ret)
		dev_err(&client->dev, "%s failed to register reboot notifier\n", __func__);
	//&*&*&*AL2_20130207, add reboot notifier for warm-reset
	
	mutex_init(&chip->mutex);
	ret = sysfs_create_group(&client->dev.kobj, &bq2419x_attr_group);
	if (ret)
		dev_err(&client->dev, "%s: error in creating sysfs attribute" , __func__);
	
	bq2419x_device_init(chip);
		
	ret = bq2419x_gpio_and_irq_init(chip);
	if(ret){
		dev_err(&client->dev, "%s: initial bq2419x gpio fault!!!\n", __func__);
		goto err_gpio_and_irq_init;
	}
		
	return 0;

err_gpio_and_irq_init:
	//&*&*&*AL1_20130207, add reboot notifier for warm-reset
	unregister_reboot_notifier(&bq2419x_shutdown_notifier);
	//&*&*&*AL2_20130207, add reboot notifier for warm-reset
	unregister_early_suspend(&chip->early_suspend);
	regulator_unregister(chip->otg_regu_dev);
err_otg_reg:
	regulator_unregister(chip->regu_dev);
err_vbus_reg:
	kfree(chip);
	return ret;
}

static int __devexit bq2419x_remove(struct i2c_client *client)
{
	struct bq2419x_chip *chip = i2c_get_clientdata(client);
	struct bq2419x_mach_info *pdata = chip->pdata;

	free_irq(gpio_to_irq(pdata->pin_int), chip);
	gpio_free(chip->pdata->pin_int);
	//&*&*&*AL1_20130207, add reboot notifier for warm-reset
	unregister_reboot_notifier(&bq2419x_shutdown_notifier);
	//&*&*&*AL2_20130207, add reboot notifier for warm-reset
	unregister_early_suspend(&chip->early_suspend);
	regulator_unregister(chip->regu_dev);
	regulator_unregister(chip->otg_regu_dev);

	kfree(chip);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void bq2419x_early_suspend(struct early_suspend *h)
{
	struct bq2419x_chip *chip = container_of(h, struct bq2419x_chip, early_suspend);
	
	chip->is_suspend = 1;
	otg_removed = 1;
}

void bq2419x_late_resume(struct early_suspend *h)
{
	struct bq2419x_chip *chip = container_of(h, struct bq2419x_chip, early_suspend);
	
	chip->is_suspend = 0;
	otg_removed = 0;

	bq2419x_update_status(chip);
}
#endif

#if defined (CONFIG_PM)
static int bq2419x_charger_suspend(struct i2c_client *client,	pm_message_t state)
{
	struct bq2419x_chip *chip = i2c_get_clientdata(client);
	struct bq2419x_mach_info *pdata = chip->pdata;
	int ret;
	
	ret = cancel_delayed_work_sync(&chip->bq2419x_interrupt_delay_work);
	dev_info(&chip->client->dev, "%s: cancel interrupt delay work %d\n", __func__, ret);
	if(ret)
		enable_irq(gpio_to_irq(pdata->pin_int));

	return 0;
}

static int bq2419x_charger_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static const struct i2c_device_id bq2419x_id[] = {
	{ "bq24196", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bq2419x_id);

static struct i2c_driver bq2419x_i2c_driver = {
	.driver	= {
		.name	= "bq2419x",
	},
	.probe		= bq2419x_probe,
	.remove		= __devexit_p(bq2419x_remove),
#if defined (CONFIG_PM)
	.suspend = bq2419x_charger_suspend,
	.resume = bq2419x_charger_resume,
#endif
	.id_table	= bq2419x_id,
};

static int __init bq2419x_init(void)
{
	return i2c_add_driver(&bq2419x_i2c_driver);
}
module_init(bq2419x_init);

static void __exit bq2419x_exit(void)
{
	i2c_del_driver(&bq2419x_i2c_driver);
}
module_exit(bq2419x_exit);

MODULE_AUTHOR("Aimar Liu <aimar.ts.liu@foxconn.com>");
MODULE_DESCRIPTION("TI bq2419x One Cell Charger driver");
MODULE_LICENSE("GPL");
