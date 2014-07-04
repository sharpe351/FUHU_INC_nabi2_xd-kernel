/*
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "gpio-names.h"
#include "wakeups.h"
//&*&*&*HC1_20130220, modify wakeup source
#include <linux/wakeup-source.h>
#include "wakeups-t3.h"

u8 g_pmic_wk_top_irq;
u8 g_pmic_wk_lv2_irq;
u64 g_tegra_wk_status;

u8 g_cable_wk_irq;//&*&*&*HC_20120609

#define TEGRA_WAKE_PWR_INT_MASK		(0x1<<TEGRA_WAKE_PWR_INT)
#define TEGRA_WAKE_GPIO_PI5_MASK		(0x1<<TEGRA_WAKE_GPIO_PI5)
#define TEGRA_WAKE_GPIO_PW3_MASK	(0x1<<TEGRA_WAKE_GPIO_PW3)
#define TEGRA_WAKE_GPIO_PU6_MASK		(0x1<<TEGRA_WAKE_GPIO_PU6)
#define TEGRA_WAKE_GPIO_PS4_MASK		(0x1<<TEGRA_WAKE_GPIO_PS4)
//&*&*&*HC2_20130220, modify wakeup source

static struct tegra_wake_info tegra_wake_event_data_t3[] = {
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5), POLARITY_NONE},	/* wake0 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV1), POLARITY_NONE},	/* wake1 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PL1), POLARITY_NONE},	/* wake2 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6), POLARITY_NONE},	/* wake3 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN7), POLARITY_NONE},	/* wake4 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB6), POLARITY_NONE},	/* wake5 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), POLARITY_NONE},	/* wake6 */
	#ifdef CONFIG_NABI_WAKEUP_BT
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6), POLARITY_NONE},	/* wake7 */
	#else
	{-EINVAL, POLARITY_NONE},							/* not used */
	#endif
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7), POLARITY_NONE},	/* wake8 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2), POLARITY_NONE},	/* wake9 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PAA1), POLARITY_NONE},	/* wake10 */
	#ifdef CONFIG_NABI_WAKEUP_WIFI
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW3), POLARITY_NONE},	/* wake11 */
	#else
	{-EINVAL, POLARITY_NONE},							/* not used */
	#endif	
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW2), POLARITY_NONE},	/* wake12 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6), POLARITY_NONE},	/* wake13 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PDD3), POLARITY_NONE},	/* wake14 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2), POLARITY_NONE},	/* wake15 */
	{INT_RTC, POLARITY_NONE},				/* wake16 */
	{INT_KBC, POLARITY_NONE},				/* wake17 */
	{INT_EXTERNAL_PMU, POLARITY_NONE},			/* wake18 */
	{INT_USB, POLARITY_EDGE_ANY}, /* TEGRA_USB1_VBUS, */		/* wake19 */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_VBUS, */		/* wake20 */
	{INT_USB, POLARITY_EDGE_ANY}, /* TEGRA_USB1_ID, */		/* wake21 */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_ID, */		/* wake22 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI5), POLARITY_NONE},	/* wake23 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV0), POLARITY_NONE},	/* wake24 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4), POLARITY_NONE},	/* wake25 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS5), POLARITY_NONE},	/* wake26 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0), POLARITY_NONE},	/* wake27 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS6), POLARITY_NONE},	/* wake28 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7), POLARITY_NONE},	/* wake29 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN2), POLARITY_NONE},	/* wake30 */
	{-EINVAL, POLARITY_NONE}, /* not used */			/* wake31 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4), POLARITY_NONE},	/* wake32 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ0), POLARITY_NONE},	/* wake33 */
	#if 0 // KAI
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2), POLARITY_NONE},	/* wake34 */
	#else // NJ5
	{-EINVAL, POLARITY_NONE},	/* wake34 */ /*NJ5 Board_Strap1*/
	#endif	
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6), POLARITY_NONE},	/* wake35 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB1), POLARITY_NONE},	/* wake36 */
	{-EINVAL, POLARITY_NONE}, /* TEGRA_USB3_VBUS, */		/* wake37 */
	{-EINVAL, POLARITY_NONE}, /* TEGRA_USB3_ID, */		/* wake38 */
	#if 0 // KAI
	{INT_USB, POLARITY_LEVEL_HI}, /* TEGRA_USB1_UTMIP, */		/* wake39 */
	#else // NJ5
	{-EINVAL, POLARITY_LEVEL_HI}, /* TEGRA_USB1_UTMIP, */		/* wake39 */
	#endif
	{INT_USB2, POLARITY_LEVEL_HI}, /* TEGRA_USB2_UTMIP, */	/* wake40 */
	{INT_USB3, POLARITY_LEVEL_HI}, /* TEGRA_USB3_UTMIP, */	/* wake41 */
	{INT_USB2, POLARITY_LEVEL_HI}, /* TEGRA_USB2_UHSIC, */	/* wake42 */
};

struct tegra_wake_info *tegra_wake_event_data = tegra_wake_event_data_t3;
unsigned int tegra_wake_event_data_size = ARRAY_SIZE(tegra_wake_event_data_t3);


//&*&*&*HC1_20130220, modify wakeup source
static struct wkup_source nabi_wkup_source[] = {
		{.name="wkup_pwr_key",	.reason=WKUP_PWR_KEY},		// PMU_INT, EN0
		{.name="wkup_rtc",		.reason=WKUP_RTC},			// PMU_INT, RTC
		{.name="wkup_cable",		.reason=WKUP_CABLE},		// PMU_INT, ACOK
		{.name="wkup_sd",		.reason=WKUP_SD},			// GPIO_PI2
		{.name="wkup_wifi",		.reason=WKUP_WIFI},			// GPIO_PV1
		{.name="wkup_bt",		.reason=WKUP_BT},			// GPIO_PS6		
		{.name="wkup_hdmi",		.reason=WKUP_HDMI},			// GPIO_PN7		
		{.name="wkup_batt_low",	.reason=WKUP_BATT_LOW},		// PMU_INT, GPIO1	
		{.name="wkup_keypad",	.reason=WKUP_KEYPAD},		// Tegra KBC
		{.name="wkup_sim_card",	.reason=WKUP_SIM_CARD},		// 	
		{.name="wkup_chg_full",	.reason=WKUP_CHG_FULL},		// PMU_INT, GPIO1	
		{.name="wkup_unknown",  	.reason=WKUP_UNKNOWN},
};

/*
 * Get wake-up reason after resuming
 */
int tegra3_get_wakeup_reason(void)
{
	enum wkup_reason wakeup = WKUP_UNKNOWN;
	u8 screen_on = false;

	if (g_tegra_wk_status & TEGRA_WAKE_PWR_INT_MASK) {

		if (g_pmic_wk_top_irq & PMIC_TOPIRQ_ONOFF_MASK)	 {
			if (g_pmic_wk_lv2_irq & PMIC_LV2IRQ_EN0_MASK)
				wakeup = WKUP_PWR_KEY;
			else if (g_pmic_wk_lv2_irq & PMIC_LV2IRQ_ACOK_MASK)
				wakeup = WKUP_CABLE;
		} else if (g_pmic_wk_top_irq & PMIC_TOPIRQ_RTC_MASK) {
			wakeup = WKUP_RTC;
		} else if (g_pmic_wk_top_irq & PMIC_TOPIRQ_GPIO_MASK) {
			if (g_pmic_wk_lv2_irq & PMIC_LV2IRQ_GPIO1_MASK) {
				//charger INT
			} else if (g_pmic_wk_lv2_irq & PMIC_LV2IRQ_GPIO6_MASK)
			;	//VDD_CORE_REQ
		}		
	} else if (g_tegra_wk_status & TEGRA_WAKE_GPIO_PI5_MASK) {
		wakeup = WKUP_SD;
	} else if (g_tegra_wk_status & TEGRA_WAKE_GPIO_PW3_MASK) {
		wakeup = WKUP_WIFI;
	} else if (g_tegra_wk_status & TEGRA_WAKE_GPIO_PU6_MASK) {
		wakeup = WKUP_BT;		
	} else if (g_tegra_wk_status & TEGRA_WAKE_GPIO_PS4_MASK) {
		wakeup = WKUP_BATT_LOW;		
	}	

	printk("%s, g_pmic_wk_top_irq=0x%x \n", __func__, g_pmic_wk_top_irq);
	printk("%s, g_pmic_wk_lv2_irq=0x%x \n", __func__, g_pmic_wk_lv2_irq);
	printk("%s, g_tegra_wk_status_L=0x%x \n", __func__, (u32) (g_tegra_wk_status&0xFFFFFFFF));
	printk("%s, g_tegra_wk_status_H=0x%x \n", __func__, (u32) ((g_tegra_wk_status>>32)&0xFFFFFFFF));
	printk("%s, wakeup reason is [%d - %s]\n", __func__, wakeup, nabi_wkup_source[wakeup].name);

	if (get_suspend_state() != PM_SUSPEND_ON) {

		#ifdef CONFIG_NABI_WAKEUP_SD_CARD
		if (wakeup == WKUP_SD) {
			printk("wakeup by sd card\n");
			screen_on = true;
		}
		#endif

		#ifdef CONFIG_NABI_WAKEUP_CABLE
		if (wakeup == WKUP_CABLE) {
			printk("wakeup by cable\n");

			if (g_cable_wk_irq == 0) {
				printk("send event\n");
				g_cable_wk_irq = 1;
				screen_on = true;
			}
		}
		#endif

		#ifdef CONFIG_NABI_WAKEUP_WIFI
		//&*&*&*HC1_20130222, disable screen on
		#if 0
		if (wakeup == WKUP_WIFI) {
			printk("wakeup by wifi\n");
			screen_on = true;
		}
		#endif
		//&*&*&*HC2_20130222, disable screen on
		#endif

		#ifdef CONFIG_NABI_WAKEUP_BT
		if (wakeup == WKUP_BT) {
			printk("wakeup by bt\n");
			screen_on = true;
		}
		#endif
	}

	if (screen_on)
		SendPowerbuttonEvent();

	return wakeup;
}
EXPORT_SYMBOL(tegra3_get_wakeup_reason);
//&*&*&*HC2_20130220, modify wakeup source
