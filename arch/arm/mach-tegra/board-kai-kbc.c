/*
 * arch/arm/mach-tegra/board-kai-kbc.c
 * Keys configuration for Nvidia tegra3 kai platform.
 *
 * Copyright (C) 2012 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/max77663-core.h>
#include <linux/gpio_scrollwheel.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-kai.h"

#include "gpio-names.h"
#include "devices.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

//&*&*&*HC1_20130411, add NV's patch for bug 1267529
/*
  * Subject: [PATCH 3/3] arm: tegra: kai: register MAX77663_IRQ_ONOFF_EN0_RISING for power key pressed
  * register MAX77663_IRQ_ONOFF_EN0_RISING for power key pressed
  * register MAX77663_IRQ_ONOFF_EN0_FALLING for power key released
  * Do not need to listen MAX77663_IRQ_ONOFF_EN0_1SEC any more
  */  

#define GPIO_IKEY(_id, _irq, _iswake, _deb, _active_low)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
		.active_low = _active_low,	\
	}

//&*&*&*HC1_20130425, modify for NXD
static struct gpio_keys_button kai_keys[] = {
#if 0	//Kai
	[0] = GPIO_KEY(KEY_MENU, PR2, 0),
	[1] = GPIO_KEY(KEY_BACK, PQ1, 0),
	[2] = GPIO_KEY(KEY_HOME, PQ0, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PR1, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PR0, 0),
	[6] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_FALLING, 0, 100),
	[7] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_1SEC, 0, 3000),
#else	//NXD
	[0] = GPIO_KEY(KEY_VOLUMEDOWN, PQ0, 0),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PQ1, 0),
	[2] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_FALLING, 0, -1, 1),
	[3] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_RISING, 0, -1, 0),
#endif
};
//&*&*&*HC2_20130425, modify for NXD
//&*&*&*HC2_20130411, add NV's patch for bug 1267529

static struct gpio_keys_platform_data kai_keys_platform_data = {
	.buttons	= kai_keys,
	.nbuttons	= ARRAY_SIZE(kai_keys),
};

static struct platform_device kai_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &kai_keys_platform_data,
	},
};

int __init kai_keys_init(void)
{
#if 0	//Kai
	pr_info("Registering gpio keys\n");
#else   //NXD
	int i;

	pr_info("Registering gpio keys\n");

	/* Enable gpio mode for other pins */
	for (i = 0; i < kai_keys_platform_data.nbuttons; i++) {
		if (kai_keys_platform_data.buttons[i].gpio < 0)
			continue;
		tegra_gpio_enable(kai_keys_platform_data.buttons[i].gpio);
	}
#endif
	platform_device_register(&kai_keys_device);

	return 0;
}
