/*
 * Support for TI bq2419x (USB/AC Adpater)
 * 1-Cell Li-Ion Charger connected via GPIOs.
 *
 * Copyright (c) 2012 Foxconn Aimar Liu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _BQ22419X_H
#define _BQ22419X_H

enum cable_type {
	BQ2419X_BATTERY_SUPPLY = 0,
	BQ2419X_VBUS_VALID_SUPPLY,
	BQ2419X_ID_BOOST_SUPPLY,
	BQ2419X_AC_CABLE_SUPPLY,
	BQ2419X_USB_CABLE_SUPPLY,
};
enum charging_status {
	BQ2419X_CHARGING_STATUS_DISCONNECT,
	BQ2419X_CHARGING_STATUS_CONNECT,
	BQ2419X_CHARGING_STATUS_FULL,
	BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_HOT,
	BQ2419X_CHARGING_STATUS_FAULT_TEMPERATURE_COLD,
	BQ2419X_CHARGING_STATUS_FAULT_TIMEOUT,
	BQ2419X_CHARGING_STATUS_FAULT_BATTERY,
	BQ2419X_CHARGING_STATUS_STOP,
};
enum charging_mode {
	BQ2419X_SUPPLEMENT_MODE,
	BQ2419X_BATTERY_CHARGING_MODE,
	BQ2419X_BOOST_MODE,
};

struct regulator_init_data;
struct regulator_consumer_supply;

struct bq2419x_mach_info {
	//int pin_nce;    // pull_down, always enable
	//int pin_iusb;   // pull_up, always 500mA on USB host
	//int pin_stat;   // not use
	//int pin_pg;     // not use for vbus detection
	int pin_int;
	int pin_psel;
	struct regulator_init_data *init_data;
	struct regulator_init_data *otg_init_data;
};

typedef void (*charge_callback_t)(enum charging_status state, 
enum cable_type cable_type, void *args);

extern int register_charger_callback(charge_callback_t cb, void *args);

#endif


