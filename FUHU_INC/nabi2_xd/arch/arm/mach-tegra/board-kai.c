/*
 * arch/arm/mach-tegra/board-kai.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>

#include <linux/skbuff.h>
// BCM4330 +
//#include <linux/ti_wilink_st.h>
#include <linux/rfkill-gpio.h>
// BCM4330 -
#include <linux/regulator/consumer.h>
//&*&*&*AL1_20130207, don't remove this
#include <linux/smb349-charger.h>
//&*&*&*AL2_20130207, don't remove this
#ifdef CONFIG_REGULATOR_BQ2419X
#include <linux/regulator/bq2419x.h>
#endif
#include <linux/leds.h>
#include <linux/i2c/at24.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <linux/mfd/tlv320aic3xxx-registers.h>
#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include <sound/tlv320aic325x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/tegra_fiq_debugger.h>
#include <linux/nfc/bcm2079x.h> //NJ5+

#include "board.h"
#include "clock.h"
#include "board-kai.h"
#include "board-touch.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "wdt-recovery.h"

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
static struct throttle_table throttle_freqs_tj[] = {
	      /*    CPU,    CBUS,    SCLK,     EMC */
	      { 1000000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  437000,  NO_CAP,  NO_CAP },
	      {  620000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  247000,  204000,  375000 },
	      {  475000,  247000,  204000,  204000 },
	      {  475000,  247000,  204000,  204000 },
	{ CPU_THROT_LOW,  247000,  204000,  102000 },
};
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct throttle_table throttle_freqs_tskin[] = {
	      /*    CPU,    CBUS,    SCLK,     EMC */
	      { 1000000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  760000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  NO_CAP,  NO_CAP,  NO_CAP },
	      {  620000,  437000,  NO_CAP,  NO_CAP },
	      {  620000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  NO_CAP,  NO_CAP },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  352000,  250000,  375000 },
	      {  475000,  247000,  204000,  375000 },
	      {  475000,  247000,  204000,  204000 },
	      {  475000,  247000,  204000,  204000 },
	{ CPU_THROT_LOW,  247000,  204000,  102000 },
};
#endif

static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = ARRAY_SIZE(throttle_freqs_tj),
		.throt_tab = throttle_freqs_tj,
	},
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = ARRAY_SIZE(throttle_freqs_tskin),
		.throt_tab = throttle_freqs_tskin,
	},
#endif
};

//&*&*&*HC1_20130204, modify thermal configuration
/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	//.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT, /* KAI */
	.shutdown_device_id = THERMAL_DEVICE_ID_TSENSOR, /* NJ5 */		
	.temp_shutdown = 90000,
#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	//.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT, /* KAI */
	.throttle_edp_device_id = THERMAL_DEVICE_ID_TSENSOR, /* NJ5 */	
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	//.temp_throttle = 85000, /* KAI */
	/* merge NV's patch:0001-arm-tegra-ft2-change-thermal-throttle-to-75C.patch */
	.temp_throttle = 75000, /* NJ5 */
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
#endif
};
//&*&*&*HC2_20130204, modify thermal configuration

// BCM4330 +
#if 1
static struct rfkill_gpio_platform_data cardhu_bt_rfkill_pdata[] = {
	{
		.name           = "bt_rfkill",
		.shutdown_gpio  = TEGRA_GPIO_PU0,
		.reset_gpio     = TEGRA_GPIO_INVALID,
		.type           = RFKILL_TYPE_BLUETOOTH,
	},
};

static struct platform_device cardhu_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &cardhu_bt_rfkill_pdata,
	},
};

static struct resource cardhu_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device cardhu_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bluesleep_resources),
	.resource       = cardhu_bluesleep_resources,
};

static noinline void __init cardhu_setup_bluesleep(void)
{
	platform_device_register(&cardhu_bluesleep_device);
	return;
}

#else

/* wl128x BT, FM, GPS connectivity chip */
struct ti_st_plat_data kai_wilink_pdata = {
	.nshutdown_gpio = TEGRA_GPIO_PU0,
	.dev_name = BLUETOOTH_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &kai_wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static noinline void __init kai_bt_st(void)
{
	pr_info("kai_bt_st");

	platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
}

static struct resource kai_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
			.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device kai_bluesleep_device = {
	.name		= "bluesleep",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(kai_bluesleep_resources),
	.resource	= kai_bluesleep_resources,
};

static noinline void __init kai_tegra_setup_tibluesleep(void)
{
	platform_device_register(&kai_bluesleep_device);
}
#endif
// BCM4330 -

static __initdata struct tegra_clk_init_table kai_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

#if 0 //Kai
static struct pn544_i2c_platform_data nfc_pdata = {
	.irq_gpio = TEGRA_GPIO_PX0,
	.ven_gpio = TEGRA_GPIO_PS7,
	.firm_gpio = TEGRA_GPIO_PR3,
};

static struct i2c_board_info __initdata kai_nfc_board_info[] = {
	{
		I2C_BOARD_INFO("pn544", 0x28),
		.platform_data = &nfc_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX0),
	},
};
#endif

#if 1 //NJ5
static struct bcm2079x_platform_data bcm2079x_pdata = {
	.irq_gpio = TEGRA_GPIO_PV0,
	.en_gpio = TEGRA_GPIO_PP2,
	.wake_gpio= TEGRA_GPIO_PP0,
};

static struct i2c_board_info i2c_bcm2079x[] __initdata = {
	{
		I2C_BOARD_INFO("bcm2079x-i2c", 0x77),		
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV0),
		.platform_data = &bcm2079x_pdata,
	},
};

#endif

static struct tegra_i2c_platform_data kai_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data kai_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data kai_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data kai_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data kai_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

#if 0 //Kai
struct max17048_battery_model max17048_mdata = {
	.rcomp          = 170,
	.soccheck_A     = 252,
	.soccheck_B     = 254,
	.bits           = 19,
	.alert_threshold = 0x00,
	.one_percent_alerts = 0x40,
	.alert_on_reset = 0x40,
	.rcomp_seg      = 0x0800,
	.hibernate      = 0x3080,
	.vreset         = 0x9696,
	.valert         = 0xD4AA,
	.ocvtest        = 55600,
};

static struct at24_platform_data eeprom_info = {
	.byte_len	= (256*1024)/8,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
	.setup		= get_mac_addr,
};

static struct i2c_board_info kai_i2c4_max17048_board_info[] = {
	{
		I2C_BOARD_INFO("max17048", 0x36),
		.platform_data = &max17048_mdata,
	},
};

static struct i2c_board_info kai_eeprom_mac_add = {
	I2C_BOARD_INFO("at24", 0x56),
	.platform_data = &eeprom_info,
};
#endif

#if defined(CONFIG_LEDS_NXD)
static struct gpio_led nxd_leds[] = {
	[0] = {
		.name				= "led0",
		.default_trigger	= "battery-charging",
		.gpio				= TEGRA_GPIO_PN1,
		.default_state 		= LEDS_GPIO_DEFSTATE_KEEP,
		//.flags      = LED_CORE_SUSPENDRESUME,
	},
	[1] = {
		.name				= "led1",
		.default_trigger	= "battery-full",
		.gpio				= TEGRA_GPIO_PN2,
		.default_state 		= LEDS_GPIO_DEFSTATE_KEEP,		
		//.flags      = LED_CORE_SUSPENDRESUME,
	},
};

const struct gpio_led_platform_data nxd_pdata_led = {
	.num_leds = ARRAY_SIZE(nxd_leds),
	.leds = nxd_leds,
};
#endif

//Battery Charger bq24196
#ifdef CONFIG_REGULATOR_BQ2419X
static struct regulator_consumer_supply bq2419x_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", NULL),
};

static struct regulator_consumer_supply bq2419x_otg_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus_otg", NULL),
};

static struct regulator_init_data bq2419x_init_data = {
	.constraints = {
		.name        			= "vbus_charger",
		.min_uV                 = 0,
		.max_uV                 = 4400000,
		.min_uA                 = 0,
		.max_uA                 = 2500000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_CURRENT | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(bq2419x_vbus_supply),
	.consumer_supplies      = bq2419x_vbus_supply,
};

static struct regulator_init_data bq2419x_otg_init_data = {
	.constraints = {
		.name         			= "vbus_otg",
		.min_uV                 = 0,
		.max_uV                 = 5000000,
		.min_uA                 = 0,
		.max_uA                 = 1300000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_CURRENT | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(bq2419x_otg_vbus_supply),
	.consumer_supplies      = bq2419x_otg_vbus_supply,
};

static struct bq2419x_mach_info bq2419x_pdata = {
	.pin_int = MAX77663_GPIO_BASE + MAX77663_GPIO1,
	.pin_psel = TEGRA_GPIO_PP1,
	.init_data  = &bq2419x_init_data,
	.otg_init_data = &bq2419x_otg_init_data,
	/*
	  * set input_source_control_register00 = 0x37 for 3000mA
	  * use default to power-on_configuration_register01 = 0x1D (minVoltage = 3.4v)
	  * set charge_current_control_register02 = 0x00.  (fast charge current = 500)
	  * set pre_charge_current_control_register03 = 0x00. (Pre-charge current = 128mA, Termination current = 128mA) 
	  * set charge_voltage_control_register04 = 0xB2. (charge voltage limit = 4.208V, Batlow = 3.0, VRECHG = 100mV) 
	  * set charge_termination_timer_control_register05 = 0x8A. (disable watchdog after i2c write (always HOST mode), Fast charge timeout = 8hr) 
	  * set IR_Compensation_and_thermal_Regulator_Control_register06 = 0x03. (chip thermal threshold for shutdown = 120 degree) 
	  * set Misc_operation_control_register07 = 0xD3. (enable dpdm detection, Q4, batteryGood, charging fault interrupt, battery fault interrupt) 
	*/
};

static struct i2c_board_info bq2419x_board_info = {
	I2C_BOARD_INFO("bq24196", 0x6b), 
	.platform_data= &bq2419x_pdata,
};
#endif
// End.

#ifdef CONFIG_BATTERY_BQ27x00
static struct i2c_board_info bq27x00_board_info = {
    I2C_BOARD_INFO("bq27425", 0x55),
    .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4),
};
#endif

//CL2N+
#ifdef CONFIG_BATTERY_BQ27541
static struct i2c_board_info kai_i2c4_bq27541_board_info[] = {
	{
		I2C_BOARD_INFO("bq27541", 0x55),
	},
};
#endif
//CL2N-

#ifdef CONFIG_CHARGER_SMB349
static struct regulator_consumer_supply smb349_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", NULL),
};

static struct regulator_consumer_supply smb349_otg_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus_otg", NULL),
};
#endif

#if 0 //Kai
static struct smb349_charger_platform_data smb349_charger_pdata = {
	.max_charge_current_mA = 1000,
	.charging_term_current_mA = 100,
	.consumer_supplies = smb349_vbus_supply,
	.num_consumer_supplies = ARRAY_SIZE(smb349_vbus_supply),
	.otg_consumer_supplies = smb349_otg_vbus_supply,
	.num_otg_consumer_supplies = ARRAY_SIZE(smb349_otg_vbus_supply),
};
#else //CL2N
#ifdef CONFIG_CHARGER_SMB349
static struct smb349_charger_platform_data smb349_charger_pdata = {
	.max_charge_current_mA = 2000,
	.charging_term_current_mA = 200,
	.consumer_supplies = smb349_vbus_supply,
	.num_consumer_supplies = ARRAY_SIZE(smb349_vbus_supply),
	.otg_consumer_supplies = smb349_otg_vbus_supply,
	.num_otg_consumer_supplies = ARRAY_SIZE(smb349_otg_vbus_supply),
	.stat_gpio = MAX77663_GPIO_BASE + MAX77663_GPIO1,
	.irq_gpio = MAX77663_IRQ_BASE + MAX77663_IRQ_GPIO1,
	.configuration_data = {0x6A/*0x00*//*input current*/, 0x40/*taper current*/, 0xFF, 0xFF, 0x38/*recharge current=100mA*/, 0x06 /*500mA for other charger*/, 0xFF, 0x40/*min system voltage and termal enable*/, 
						   0xFF, 0xFF/*OTG active low: 0x20*/, 0xFF, 0x4E,//0x8E/*temperature monitor:0~50*/ 
						   0x80, 0x98, /*<-- interrupt mask*/
						   0xFF, 0xFF, 0x0F/*low battery threshold:3.58*/},
};
static struct i2c_board_info kai_i2c4_smb349_board_info[] = {
	{
		I2C_BOARD_INFO("smb349", 0x1B),
		.platform_data = &smb349_charger_pdata,
	},
};
#endif
#endif

#if 0 //Kai
static struct i2c_board_info __initdata rt5640_board_info = {
	I2C_BOARD_INFO("rt5640", 0x1c),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

static struct i2c_board_info __initdata rt5639_board_info = {
	I2C_BOARD_INFO("rt5639", 0x1c),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};
#else //NJ5
static struct aic325x_pdata kai_aic3206_pdata = {
	.debounce_time_ms = 512,
	.cspin = TEGRA_CODEC_SPI_CS,
};

static struct spi_board_info __initdata aic3206_spi_board_info[] = {
	{
		.modalias = "tlv320aic325x",
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_1,
		//.max_speed_hz = 12000000,
		.max_speed_hz = 4000000,
		.platform_data = &kai_aic3206_pdata,
		.irq = 0, 
	},
};
#endif
static void kai_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	tegra_i2c_device1.dev.platform_data = &kai_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &kai_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &kai_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &kai_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &kai_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

#ifdef CONFIG_CHARGER_SMB349
	i2c_register_board_info(4, kai_i2c4_smb349_board_info,
		ARRAY_SIZE(kai_i2c4_smb349_board_info));
#endif

#if 0 //Kai
	if (board_info.fab == BOARD_FAB_A00)
		i2c_register_board_info(4, &rt5640_board_info, 1);
	else
		i2c_register_board_info(4, &rt5639_board_info, 1);

	i2c_register_board_info(4, &kai_eeprom_mac_add, 1);

	i2c_register_board_info(4, kai_i2c4_max17048_board_info,
		ARRAY_SIZE(kai_i2c4_max17048_board_info));

	i2c_register_board_info(0, kai_nfc_board_info, 1);
#endif
//CL2N+
#ifdef CONFIG_BATTERY_BQ27541
	i2c_register_board_info(4, kai_i2c4_bq27541_board_info,
		ARRAY_SIZE(kai_i2c4_bq27541_board_info));
#endif
//CL2N-

#ifdef CONFIG_REGULATOR_BQ2419X
	i2c_register_board_info(4, &bq2419x_board_info, 1);
#endif
#ifdef CONFIG_BATTERY_BQ27x00
	i2c_register_board_info(4, &bq27x00_board_info, 1);
#endif
//NJ5+
   i2c_register_board_info(0, i2c_bcm2079x,	ARRAY_SIZE(i2c_bcm2079x));
//NJ5-

}

static struct platform_device *kai_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data kai_uart_pdata;
static struct tegra_uart_platform_data kai_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0)
		debug_port_id = 3;

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		kai_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		kai_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		kai_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		kai_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		kai_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		kai_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
}

static void __init kai_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	kai_uart_pdata.parent_clk_list = uart_parent_clk;
	kai_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	kai_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	kai_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	kai_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &kai_uart_pdata;
	tegra_uartb_device.dev.platform_data = &kai_uart_pdata;
	tegra_uartc_device.dev.platform_data = &kai_uart_pdata;
	tegra_uartd_device.dev.platform_data = &kai_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &kai_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(kai_uart_devices,
				ARRAY_SIZE(kai_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *kai_spi_devices[] __initdata = {
	&tegra_spi_device1,
	&tegra_spi_device2,
};

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};
#if 0 //Kai
static struct tegra_spi_platform_data kai_spi1_pdata = {
		.is_dma_based           = true,
		.max_dma_buffer         = (128),
		.is_clkon_always        = false,
		.max_rate               = 100000000,
};
#else //CL2N
static struct tegra_spi_platform_data kai_spi1_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= true,
	.max_rate		= 100000000,
};

static struct tegra_spi_platform_data kai_spi2_pdata = {
		.is_dma_based           = true,
		.max_dma_buffer         = (128),
		.is_clkon_always        = false,
		.max_rate               = 100000000,
};

#endif

static void __init kai_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}

	kai_spi1_pdata.parent_clk_list = spi_parent_clk;
	kai_spi1_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device1.dev.platform_data = &kai_spi1_pdata;

	kai_spi2_pdata.parent_clk_list = spi_parent_clk;
	kai_spi2_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device2.dev.platform_data = &kai_spi2_pdata;
	
	platform_add_devices(kai_spi_devices,
				ARRAY_SIZE(kai_spi_devices));

//CL2N+
	/*register TI AIC3206 codec on SPI bus*/
	spi_register_board_info(aic3206_spi_board_info, ARRAY_SIZE(aic3206_spi_board_info));
//CL2N-
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};
#if 0 //Kai
static struct tegra_asoc_platform_data kai_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
		.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= -1,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
	},
};

static struct platform_device kai_audio_device = {
	.name	= "tegra-snd-rt5640",
	.id	= 0,
	.dev	= {
		.platform_data = &kai_audio_pdata,
	},
};
#else //NJ5
static struct tegra_asoc_platform_data kai_audio_device_aic3206_platform_data ={

	.gpio_spkr_en = TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	/*
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= -1,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
	},
	*/
/* comment because of compiler error
	.audio_port_id		= {
		[HIFI_CODEC] = 1,
		[BASEBAND] = -1,
		[BT_SCO] = 3,
	},
	.baseband_param		= {
		.rate = -1,
		.channels = -1,
	},
*/	
};

static struct platform_device kai_audio_device_aic3206 = {
	.name	= "tegra-snd-aic325x",
	.id	= 0,
	.dev	= {
	.platform_data  = &kai_audio_device_aic3206_platform_data,
	},
};
#endif

static struct gpio_led kai_led_info[] = {
	{
		.name			= "statled",
		.default_trigger	= "default-on",
		.gpio			= TEGRA_GPIO_STAT_LED,
		.active_low		= 1,
		.retain_state_suspended	= 0,
		.default_state		= LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data kai_leds_pdata = {
	.leds		= kai_led_info,
	.num_leds	= ARRAY_SIZE(kai_led_info),
};

static struct platform_device kai_leds_gpio_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_leds_pdata,
	},
};

static struct platform_device *kai_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	//&kai_audio_device, /*remove temporarily for Foxconn*/
	&kai_audio_device_aic3206, //NJ5
	&kai_leds_gpio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
// BCM4330 +
	&cardhu_bt_rfkill_device
// BCM4330 -
};
#if 0 //for Kai
static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "sbc1",       "pll_p",        52000000,       true},
	{ NULL,         NULL,           0,              0},
};

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern3",    "pll_p",        41000000,       true},
	{ "clk_out_3",  "extern3",      40800000,       true},
	{ NULL,         NULL,           0,              0},
};

static int __init kai_touch_init(void)
{
	int touch_id;

	gpio_request(KAI_TS_ID1, "touch-id1");
	gpio_direction_input(KAI_TS_ID1);

	gpio_request(KAI_TS_ID2, "touch-id2");
	gpio_direction_input(KAI_TS_ID2);

	touch_id = gpio_get_value(KAI_TS_ID1) << 1;
	touch_id |= gpio_get_value(KAI_TS_ID2);

	pr_info("touch-id %d\n", touch_id);

	/* Disable TS_ID GPIO to save power */
	gpio_direction_output(KAI_TS_ID1, 0);
	tegra_pinmux_set_pullupdown(KAI_TS_ID1_PG, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_tristate(KAI_TS_ID1_PG, TEGRA_TRI_TRISTATE);
	gpio_direction_output(KAI_TS_ID2, 0);
	tegra_pinmux_set_pullupdown(KAI_TS_ID2_PG, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_tristate(KAI_TS_ID2_PG, TEGRA_TRI_TRISTATE);

	switch (touch_id) {
	case 0:
		pr_info("Raydium PCB based touch init\n");
		tegra_clk_init_from_table(spi_clk_init_table);
		touch_init_raydium(TEGRA_GPIO_PEE1, TEGRA_GPIO_PN5, 0);
/*
		rm31080a_kai_spi_board[0].platform_data =
			&rm31080ts_kai_007_data;
		rm31080a_kai_spi_board[0].irq =
			gpio_to_irq(TEGRA_GPIO_PEE1);
		touch_init_raydium(TEGRA_GPIO_PEE1,
					TEGRA_GPIO_PN5,
					&rm31080ts_kai_007_data,
					&rm31080a_kai_spi_board[0],
					ARRAY_SIZE(rm31080a_kai_spi_board));
*/
		break;
	case 1:
		pr_info("Raydium On-Board touch init\n");
		tegra_clk_init_from_table(spi_clk_init_table);
		tegra_clk_init_from_table(touch_clk_init_table);
		clk_enable(tegra_get_clock_by_name("clk_out_3"));

		touch_init_raydium(TEGRA_GPIO_PEE1, TEGRA_GPIO_PN5, 1);
/*
		rm31080a_kai_spi_board[0].platform_data =
			&rm31080ts_kai_107_data;
		rm31080a_kai_spi_board[0].irq =
			gpio_to_irq(TEGRA_GPIO_PEE1);
		touch_init_raydium(TEGRA_GPIO_PEE1,
					TEGRA_GPIO_PN5,
					&rm31080ts_kai_107_data,
					&rm31080a_kai_spi_board[0],
					ARRAY_SIZE(rm31080a_kai_spi_board));
*/
		break;
	case 3:
		pr_info("Synaptics PCB based touch init\n");
		touch_init_synaptics_kai();
		break;
	default:
		pr_err("touch_id error, no touch %d\n", touch_id);
	}
	return 0;
}
#endif	//for Kai

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "usb_vbus_otg",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

//NJ5+
/**++++20130110, JimmySu add MStar touch driver**/
#ifdef CONFIG_TOUCHSCREEN_MSG21XX

static struct i2c_board_info __initdata msg21xx_i2c_info[] = {
	{
		I2C_BOARD_INFO("ms-msg21xx", 0x26),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
//		.platform_data = &msg21xx_ts_info,
	}
};

static int __init touch_init_msg21xx_kai(void)
{
	pr_debug("%s\n", __func__);

	i2c_register_board_info(1, msg21xx_i2c_info, 1);

	return 0;
}

#endif

#ifdef CONFIG_TOUCHSCREEN_RM31080A
//static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
//	/* name         parent          rate            enabled */
//	{ "sbc1",       "pll_p",        52000000,       true},
//	{ NULL,         NULL,           0,              0},
//};

//static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
//	/* name         parent          rate            enabled */
//	{ "extern3",    "pll_p",        41000000,       false/*true*/},
//	{ "clk_out_3",  "extern3",      40800000,       false/*true*/},
//	{ NULL,         NULL,           0,              0},
//};
#endif
static int __init kai_touch_init(void)
{

#ifdef CONFIG_TOUCHSCREEN_MSG21XX
	return touch_init_msg21xx_kai();
#elif  defined (CONFIG_TOUCHSCREEN_RM31080A)
	pr_info("Raydium touch init\n");

////	tegra_clk_init_from_table(spi_clk_init_table);
//	tegra_clk_init_from_table(touch_clk_init_table);
//	clk_enable(tegra_get_clock_by_name("clk_out_3"));
//	touch_init_raydium(TEGRA_GPIO_PEE1, TEGRA_GPIO_PN5, 1);
/*
		rm31080a_kai_spi_board[0].platform_data =
			&rm31080ts_kai_107_data;
		rm31080a_kai_spi_board[0].irq =
			gpio_to_irq(TEGRA_GPIO_PEE1);
		touch_init_raydium(TEGRA_GPIO_PEE1,
					TEGRA_GPIO_PN5,
					&rm31080ts_kai_107_data,
					&rm31080a_kai_spi_board[0],
					ARRAY_SIZE(rm31080a_kai_spi_board));
*/

	return 0;

#endif
}
//NJ5-
/**----20130110, JimmySu add MStar touch driver**/

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = NULL,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,

	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

#if CONFIG_USB_SUPPORT
static void kai_usb_init(void)
{
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
	platform_device_register(&tegra_ehci2_device);
}

//+&*&*&*YT_121016,JB: remove icera modem gpio settings.
/*static void kai_modem_init(void)
{
	int ret;

	ret = gpio_request(TEGRA_GPIO_W_DISABLE, "w_disable_gpio");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_W_DISABLE);
	else
		gpio_direction_output(TEGRA_GPIO_W_DISABLE, 1);


	ret = gpio_request(TEGRA_GPIO_MODEM_RSVD1, "Port_V_PIN_0");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_MODEM_RSVD1);
	else
		gpio_direction_input(TEGRA_GPIO_MODEM_RSVD1);


	ret = gpio_request(TEGRA_GPIO_MODEM_RSVD2, "Port_H_PIN_7");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_MODEM_RSVD2);
	else
		gpio_direction_output(TEGRA_GPIO_MODEM_RSVD2, 1);

}*/
//-&*&*&*YT_121016,JB: remove icera modem gpio settings.

#else
static void kai_usb_init(void) { }
//+&*&*&*YT_121016,JB: remove icera modem gpio settings.
//static void kai_modem_init(void) { }
//-&*&*&*YT_121016,JB: remove icera modem gpio settings.
#endif
#if 0 //Kai
static void kai_audio_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	if (board_info.fab == BOARD_FAB_A01) {
		kai_audio_pdata.codec_name = "rt5639.4-001c";
		kai_audio_pdata.codec_dai_name = "rt5639-aif1";
	} else if (board_info.fab == BOARD_FAB_A00) {
		kai_audio_pdata.codec_name = "rt5640.4-001c";
		kai_audio_pdata.codec_dai_name = "rt5640-aif1";
	}
}
#endif
static void __init tegra_kai_init(void)
{
	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(kai_clk_init_table);
	kai_pinmux_init();
	kai_i2c_init();
	kai_spi_init();
	kai_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	kai_edp_init();
#endif
	kai_uart_init();
	//NJ5+
	kai_tsensor_init();
	//NJ5-
	//kai_audio_init(); //Kai-
	platform_add_devices(kai_devices, ARRAY_SIZE(kai_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	kai_sdhci_init();
	kai_regulator_init();
	kai_suspend_init();
//	kai_touch_init();
	kai_keys_init();
	kai_panel_init();
	kai_touch_init();
// BCM4330 +
//	kai_tegra_setup_tibluesleep();
//	kai_bt_st();
	cardhu_setup_bluesleep();
// BCM4330 -
	kai_sensors_init();
	kai_pins_state_init();
	kai_emc_init();
	tegra_release_bootloader_fb();
//+&*&*&*YT_121016,JB: remove icera modem gpio settings.
	/*kai_modem_init();*/
//-&*&*&*YT_121016,JB: remove icera modem gpio settings.
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
#if defined(CONFIG_LEDS_NXD)
	gpio_led_register_device(0, &nxd_pdata_led);
#endif
}

static void __init kai_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_kai_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	kai_ramconsole_reserve(SZ_1M);
}

MACHINE_START(KAI, "kai")
	.boot_params	= 0x80000100,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_kai_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_kai_init,
MACHINE_END
