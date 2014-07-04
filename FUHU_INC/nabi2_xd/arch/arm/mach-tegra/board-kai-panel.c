/*
 * arch/arm/mach-tegra/board-kai-panel.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include <linux/nabijr_board_version.h>

#include <linux/reboot.h>

#include <linux/clk.h>

#include "board.h"
#include "board-kai.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"
#include "clock.h"  //NXD++ add for touch power-on sequence

#include <mach/hardware.h>

#define PANEL_10_CPT_1366_768

#ifdef PANEL_10_CPT_1366_768
#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE
#define DSI_PANEL_RESET 1
#endif

#ifdef PANEL_10_CPT_1366_768
#define kai_bl_pwm			TEGRA_GPIO_PH0 /* LCD1_BL_PWM */
#define kai_lcd_en_3v3		TEGRA_GPIO_PI7 /* LCD_VDD_EN_3V3 */
#define kai_panel_vdd_en		TEGRA_GPIO_PW1 /* EN_VDD_PNL1_LCD */
#define kai_lcd_id1		TEGRA_GPIO_PG2 /* EN_VDD_PNL1_LCD */
#define kai_lcd_id2		TEGRA_GPIO_PG3 /* EN_VDD_PNL1_LCD */

#else

#if 1 /*for NJ5 panel*/
/* kai default display board pins */
#define kai_vdd_bl_enb			TEGRA_GPIO_PH2 /* 1.EN_VDD_BL1 */
/* SNN_LCD1_BL_EN TEGRA_GPIO_PH2 NC */
#define kai_vdd_enb		TEGRA_GPIO_PW1 /* 3.EN_VDD_PNL */
#define kai_bl_pwm			TEGRA_GPIO_PH0 /* 4.LCD1_BL_PWM */
#define kai_panel_mode			TEGRA_GPIO_PG2 /* 5.LCD_MODE0 */
#define kai_panel_dith			TEGRA_GPIO_PG6 /* 7.BPP (Low:24bpp, High:18bpp) */
#define kai_panel_ud			TEGRA_GPIO_PG0 /* 8.LCD_UD */
#define kai_panel_lr			TEGRA_GPIO_PG1 /* 9.LCD_LR */
#define kai_panel_rst			TEGRA_GPIO_PG7 /* 11.RESET* */
#define kai_panel_avdd_en		TEGRA_GPIO_PH6 /* 13.LCD_AVDD_EN */

#else  /*for kai panel*/
/* kai default display board pins */
##define kai_lvds_avdd_en		TEGRA_GPIO_PH6
#define kai_lvds_stdby			TEGRA_GPIO_PG5
#define kai_lvds_rst			TEGRA_GPIO_PG7
#define kai_lvds_shutdown		TEGRA_GPIO_PN6
#define kai_lvds_rs			TEGRA_GPIO_PV6
#define kai_lvds_lr			TEGRA_GPIO_PG1

#endif

#endif  /*PANEL_10_CPT_1366_768*/

/* kai A00 display board pins */
#define kai_lvds_rs_a00		TEGRA_GPIO_PH1

/* common pins( backlight ) for all display boards */
#define kai_bl_enb			TEGRA_GPIO_PH3
#define kai_bl_pwm			TEGRA_GPIO_PH0
#define kai_hdmi_hpd			TEGRA_GPIO_PN7

#ifdef CONFIG_TEGRA_DC
static struct regulator *kai_hdmi_reg;
static struct regulator *kai_hdmi_pll;
static struct regulator *kai_hdmi_vddio;
#endif

int cl2n_panel_status;  /*20120607, jimmySu add to judge display status */

static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct regulator *kai_lvds_reg;
static struct regulator *kai_lvds_vdd_panel;

#ifdef PANEL_10_CPT_1366_768
static struct regulator *vdd_lcm_core;
static struct regulator *vdd_dsi_csi;
#endif

/*20120515, JimmySu add board ID*/
static int panel_board_id =0x1;

static tegra_dc_bl_output kai_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 20, 21,
	22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 32, 34, 34, 36, 36,
	38, 39, 40, 40, 41, 42, 42, 43,
	44, 44, 45, 46, 46, 47, 48, 48,
	49, 50, 50, 51, 52, 53, 54, 54,
	55, 56, 57, 58, 58, 59, 60, 61,
	62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 72, 72, 73, 74, 75, 76,
	76, 77, 78, 79, 80, 81, 82, 83,
	85, 86, 87, 89, 90, 91, 92, 92,
	93, 94, 95, 96, 96, 97, 98, 99,
	100, 100, 101, 102, 103, 104, 104, 105,
	106, 107, 108, 108, 109, 110, 112, 114,
	116, 118, 120, 121, 122, 123, 124, 125,
	126, 127, 128, 129, 130, 131, 132, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 149,
	150, 151, 151, 152, 153, 153, 154, 155,
	155, 156, 157, 157, 158, 159, 159, 160,
	162, 164, 166, 168, 170, 172, 174, 176,
	178, 180, 181, 181, 182, 183, 183, 184,
	185, 185, 186, 187, 187, 188, 189, 189,
	190, 191, 192, 193, 194, 195, 196, 197,
	198, 199, 200, 201, 201, 202, 203, 203,
	204, 205, 205, 206, 207, 207, 208, 209,
	209, 210, 211, 211, 212, 212, 213, 213,
	214, 215, 215, 216, 216, 217, 217, 218,
	219, 219, 220, 222, 226, 230, 232, 234,
	236, 238, 240, 244, 248, 251, 253, 255
};

static p_tegra_dc_bl_output bl_output;

/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/
static struct clk *emc_clk_lock;
/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/

static int kai_backlight_init(struct device *dev)
{
#ifndef PANEL_10_CPT_1366_768
	int ret;
#endif
	bl_output = kai_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(kai_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");


	tegra_gpio_disable(kai_bl_pwm);

#ifndef PANEL_10_CPT_1366_768
	ret = gpio_request(kai_vdd_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(kai_vdd_bl_enb, 1);
	if (ret < 0)
		gpio_free(kai_vdd_bl_enb);
	else
		tegra_gpio_enable(kai_vdd_bl_enb);

	return ret;
#else
	return 0;
#endif

};

static void kai_backlight_exit(struct device *dev)
{
	/* int ret; */
	/*ret = gpio_request(kai_vdd_bl_enb, "backlight_enb");*/
#ifndef PANEL_10_CPT_1366_768
	gpio_set_value(kai_vdd_bl_enb, 0);
#endif
	
#if 0 //Kai, 20120607, JimmySu remove unsed setting
	gpio_free(kai_vdd_bl_enb);
#endif
	return;
}

static int kai_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

//	gpio_set_value(kai_vdd_bl_enb, !!brightness); /*20120607, JimmySu remove unsed setting*/

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data kai_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 100/*224*/,
	.pwm_period_ns	= 1000000/*6670000*//*100000*/, /*1KHz*//*150Hz*//*10KHz*/
	.init		= kai_backlight_init,
	.exit		= kai_backlight_exit,
	.notify		= kai_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= kai_disp1_check_fb,
};

static struct platform_device kai_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_backlight_data,
	},
};

#if 0  /*disable clk for NXD new touch structure*/
static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern3",    "pll_p",        41000000,       false/*true*/},
	{ "clk_out_3",  "extern3",      40800000,       false/*true*/},
	{ NULL,         NULL,           0,              0},
};
#endif
static int kai_panel_enable(void)
{
//#ifndef PANEL_10_CPT_1366_768
	static int first = 0;
//#endif

/*20121017, JimmySu add patch, emc clk not less than 102M when screen on, change 102->204MHz*/
	if (emc_clk_lock) {
		clk_set_rate(emc_clk_lock, 204000000/*102000000*/);
		clk_enable(emc_clk_lock);
	}
/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/

	cl2n_panel_status = 1; /*20120607, jimmySu add to judge display status */

/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
#ifdef PANEL_10_CPT_1366_768

    	tegra_gpio_disable(TEGRA_GPIO_PT5);
	tegra_gpio_disable(TEGRA_GPIO_PT6);

	 if (vdd_dsi_csi == NULL) {
	       vdd_dsi_csi = regulator_get(NULL, "lcm_mipi");
		  if (WARN_ON(IS_ERR(vdd_dsi_csi))) {
			   pr_err("%s: couldn't get regulator avdd_dsi_csi: %ld\n",
			          __func__, PTR_ERR(vdd_dsi_csi));
			   return PTR_ERR(vdd_dsi_csi);
		  }
	 }
	 regulator_enable(vdd_dsi_csi);

/*touch 1V8 enable*/
	gpio_set_value(TEGRA_GPIO_PH3, 1);

	 mdelay(10);


	if (kai_lvds_reg == NULL) {
		kai_lvds_reg = regulator_get(NULL, "vdd_lvds");
		if (WARN_ON(IS_ERR(kai_lvds_reg))) {
			pr_err("%s: couldn't get regulator vdd_lvds: %ld\n",
			       __func__, PTR_ERR(kai_lvds_reg));
			return PTR_ERR(kai_lvds_reg);
		}
	}
	regulator_enable(kai_lvds_reg);

	if (kai_lvds_vdd_panel == NULL) {
		kai_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(kai_lvds_vdd_panel))) {
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(kai_lvds_vdd_panel));
			return PTR_ERR(kai_lvds_vdd_panel);
		}
	}
	regulator_enable(kai_lvds_vdd_panel);

	gpio_set_value(kai_lcd_en_3v3, 1);
	mdelay(1);

	 if (vdd_lcm_core == NULL) {
	       vdd_lcm_core = regulator_get(NULL, "core_vdd_lcm");
		  if (WARN_ON(IS_ERR(vdd_lcm_core))) {
			   pr_err("%s: couldn't get regulator core_vdd_lcm: %ld\n",
			          __func__, PTR_ERR(vdd_lcm_core));
			   return PTR_ERR(vdd_lcm_core);
		  }
	 }
	 regulator_enable(vdd_lcm_core);

	mdelay(30);

	tegra_gpio_disable(TEGRA_GPIO_PEE0); //touch TS_CLK
	tegra_gpio_disable(TEGRA_GPIO_PX0);
	tegra_gpio_disable(TEGRA_GPIO_PX1);
	tegra_gpio_disable(TEGRA_GPIO_PX2);
	tegra_gpio_disable(TEGRA_GPIO_PX3);
	
	 //touch power-on sequence
//	tegra_clk_init_from_table(touch_clk_init_table);
	//clk_enable(tegra_get_clock_by_name("clk_out_3"));
//	clk_disable(tegra_get_clock_by_name("clk_out_3"));
	if (first == 0){
		touch_init_raydium(TEGRA_GPIO_PEE1, TEGRA_GPIO_PN5, 1);
		first = 1;
	}else{
		gpio_set_value(TEGRA_GPIO_PN5, 1); //touch RST high
	}
//	mdelay(70);
//	tegra_gpio_disable(kai_bl_pwm); /*20120608, JimmySu prevent flash screen while shutdown device*/
	
#else
// panel default setting
	gpio_set_value(kai_panel_lr, 1);
	gpio_set_value(kai_panel_ud, 1);
	gpio_set_value(kai_panel_mode, 0);
	gpio_set_value(kai_panel_dith, 1);

	if (kai_lvds_reg == NULL) {
		kai_lvds_reg = regulator_get(NULL, "vdd_lvds");
		if (WARN_ON(IS_ERR(kai_lvds_reg))) {
			pr_err("%s: couldn't get regulator vdd_lvds: %ld\n",
			       __func__, PTR_ERR(kai_lvds_reg));
			return PTR_ERR(kai_lvds_reg);
		}
	}
	regulator_enable(kai_lvds_reg);

	if (kai_lvds_vdd_panel == NULL) {
		kai_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(kai_lvds_vdd_panel))) {
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(kai_lvds_vdd_panel));
			return PTR_ERR(kai_lvds_vdd_panel);
		}
	}
	regulator_enable(kai_lvds_vdd_panel);

	mdelay(1);
	gpio_set_value(kai_panel_rst, 1);
	mdelay(4);
	gpio_set_value(kai_panel_avdd_en, 1);

	if (first != 0){ 
	mdelay(40);
	gpio_set_value(kai_panel_rst, 0);
	mdelay(1);
	gpio_set_value(kai_panel_rst, 1);
	}else{
		first =1;
	}
	
	mdelay(160);/*msleep(160);*/  /*20120822 JimmySu modify for A2110 10" panel timing*/
	gpio_set_value(kai_vdd_bl_enb, 1);
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/
#endif /*PANEL_10_CPT_1366_768*/
	return 0;
}

static int kai_panel_disable(void)
{
	cl2n_panel_status = 0; /*++++20120607, jimmySu add to judge display status */

/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/
	if (emc_clk_lock) {
		clk_disable(emc_clk_lock);
	}
/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/

/*++++20120608, JimmySu prevent flash screen while shutdown device*/
//	tegra_gpio_enable(kai_bl_pwm);
//	gpio_request(kai_bl_pwm, "backlight-pwm-sleep");
//	gpio_direction_output(kai_bl_pwm, 0);
//	gpio_set_value(kai_bl_pwm, 0);
/*----20120608, JimmySu prevent flash screen while shutdown device*/

#ifdef PANEL_10_CPT_1366_768

	tegra_gpio_enable(TEGRA_GPIO_PT5);
	gpio_request(TEGRA_GPIO_PT5, "i2c-scl-sleep");
	gpio_direction_output(TEGRA_GPIO_PT5, 0);
	gpio_set_value(TEGRA_GPIO_PT5, 0);

	tegra_gpio_enable(TEGRA_GPIO_PT6);
	gpio_request(TEGRA_GPIO_PT6, "i2c-sda-sleep");
	gpio_direction_output(TEGRA_GPIO_PT6, 0);
	gpio_set_value(TEGRA_GPIO_PT6, 0);

//	mdelay(200);
	/*shut down interface signal*/
	mdelay(1);

	regulator_disable(vdd_lcm_core);
	regulator_put(vdd_lcm_core);
	vdd_lcm_core = NULL;

	mdelay(1);

	gpio_set_value(kai_lcd_en_3v3, 0);
	
	regulator_disable(kai_lvds_reg);
	regulator_put(kai_lvds_reg);
	kai_lvds_reg = NULL;

	regulator_disable(kai_lvds_vdd_panel);
	regulator_put(kai_lvds_vdd_panel);
	kai_lvds_vdd_panel = NULL;

	regulator_disable(vdd_dsi_csi);
	regulator_put(vdd_dsi_csi);
	vdd_dsi_csi = NULL;

	mdelay(30/*10*/);
	/*touch 1V8 disable*/
	gpio_set_value(TEGRA_GPIO_PH3, 0);
	gpio_set_value(TEGRA_GPIO_PN5, 0);

	tegra_gpio_enable(TEGRA_GPIO_PEE0);
	gpio_request(TEGRA_GPIO_PEE0, "ts-clk-sleep");
	gpio_direction_output(TEGRA_GPIO_PEE0, 0);
	gpio_set_value(TEGRA_GPIO_PEE0, 0);

	tegra_gpio_enable(TEGRA_GPIO_PX0);
	gpio_request(TEGRA_GPIO_PX0, "spi2- mosi-sleep");
	gpio_direction_output(TEGRA_GPIO_PX0, 0);
	gpio_set_value(TEGRA_GPIO_PX0, 0);

	tegra_gpio_enable(TEGRA_GPIO_PX1);
	gpio_request(TEGRA_GPIO_PX1, "spi2-miso-sleep");
	gpio_direction_output(TEGRA_GPIO_PX1, 0);
	gpio_set_value(TEGRA_GPIO_PX1, 0);

	tegra_gpio_enable(TEGRA_GPIO_PX2);
	gpio_request(TEGRA_GPIO_PX2, "spi2-sck-sleep");
	gpio_direction_output(TEGRA_GPIO_PX2, 0);
	gpio_set_value(TEGRA_GPIO_PX2, 0);

	tegra_gpio_enable(TEGRA_GPIO_PX3);
	gpio_request(TEGRA_GPIO_PX3, "spi2-cs0-sleep");
	gpio_direction_output(TEGRA_GPIO_PX3, 0);
	gpio_set_value(TEGRA_GPIO_PX3, 0);
#else
/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
	gpio_set_value(kai_vdd_bl_enb, 0);
	mdelay(120);
	gpio_set_value(kai_panel_avdd_en, 0);
	mdelay(60);
	gpio_set_value(kai_panel_rst, 0);

	regulator_disable(kai_lvds_reg);
	regulator_put(kai_lvds_reg);
	kai_lvds_reg = NULL;

	regulator_disable(kai_lvds_vdd_panel);
	regulator_put(kai_lvds_vdd_panel);
	kai_lvds_vdd_panel = NULL;

	gpio_set_value(kai_panel_lr, 0);
	gpio_set_value(kai_panel_ud, 0);
	gpio_set_value(kai_panel_mode, 0);
	gpio_set_value(kai_panel_dith, 0);
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/
#endif

	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int kai_hdmi_vddio_enable(void)
{
	int ret;
	if (!kai_hdmi_vddio) {
		kai_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(kai_hdmi_vddio)) {
			ret = PTR_ERR(kai_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			kai_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(kai_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int kai_hdmi_vddio_disable(void)
{
	if (kai_hdmi_vddio) {
		regulator_disable(kai_hdmi_vddio);
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
	}
	return 0;
}

static int kai_hdmi_enable(void)
{
	int ret;
	if (!kai_hdmi_reg) {
		kai_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(kai_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_reg);
		}
	}
	ret = regulator_enable(kai_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!kai_hdmi_pll) {
		kai_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(kai_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			kai_hdmi_pll = NULL;
			regulator_put(kai_hdmi_reg);
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_pll);
		}
	}
	ret = regulator_enable(kai_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int kai_hdmi_disable(void)
{
	regulator_disable(kai_hdmi_reg);
	regulator_put(kai_hdmi_reg);
	kai_hdmi_reg = NULL;

	regulator_disable(kai_hdmi_pll);
	regulator_put(kai_hdmi_pll);
	kai_hdmi_pll = NULL;
	return 0;
}

static struct resource kai_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by kai_panel_init() */
		.end	= 0,	/* Filled in by kai_panel_init() */
		.flags	= IORESOURCE_MEM,
	},

#ifdef CONFIG_TEGRA_DSI_INSTANCE_1
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSIB_BASE,
		.end	= TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#else
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
		
};

static struct resource kai_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif


#ifdef PANEL_10_CPT_1366_768
static struct tegra_dsi_out kai_dsi = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,

//	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,


	.refresh_rate = 59,
	.rated_refresh_rate = 59,

	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_has_frame_buffer = true,
#ifdef CONFIG_TEGRA_DSI_INSTANCE_1
	.dsi_instance = 1,
#else
	.dsi_instance = 0,
#endif
	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,

	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END, //&*&*&*JJ1 DSI_PKT_SEQ_1_LO (40090288  --> 40090308)
};
#endif


static struct tegra_dc_mode kai_panel_modes[] = {
	{
#ifdef PANEL_10_CPT_1366_768
		/* 1366x768@60Hz */

		.pclk = 70000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 2,
		.h_back_porch = 50,
		.v_back_porch = 4,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 100,
		.v_front_porch = 4,

/*
		.pclk = 68000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 4,
		.v_sync_width = 5,
		.h_back_porch = 10,
		.v_back_porch = 10,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 50,
		.v_front_porch = 10,
*/
#else	
		
/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
		/* 800x480@60Hz */
		.pclk = 26400000, /*type pclk =33.3MHz, min pclk=26.4MHz*/
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 1,
		.v_sync_width = 1,
		.h_back_porch = 45,
		.v_back_porch = 22,
		.h_active = 800,
		.v_active = 480,
		.h_front_porch = 16, /*type value=210*/
		.v_front_porch = 7, /*type value=22*/
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/
#endif
	},
};

#ifndef PANEL_10_CPT_1366_768
/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
static struct tegra_dc_out_pin kai_dc_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_HIGH,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_DATA_ENABLE,
		.pol	= TEGRA_DC_OUT_PIN_POL_HIGH,
	},
};
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/
#endif

static struct tegra_dc_sd_settings kai_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &kai_backlight_device,
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_fb_data kai_fb_data = {
	.win		= 0,
#ifdef PANEL_10_CPT_1366_768
	.xres		= 1366,
	.yres		= 768,
#else
	.xres		= 800, /** 20130110, JimmySu add chimei NJ5 panel driver**/
	.yres		= 480,
#endif
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data kai_hdmi_fb_data = {
	.win		= 0,
#ifdef PANEL_10_CPT_1366_768
	.xres		= 1366,
	.yres		= 768,
#else
	.xres		= 800,  /** 20130110, JimmySu add chimei NJ5 panel driver**/
	.yres		= 480,
#endif
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out kai_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.parent_clk	= "pll_d2_out0",  /*20130523, JimmySu change HDMI clock source*/

	.dcc_bus	= 3,
	.hotplug_gpio	= kai_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= kai_hdmi_enable,
	.disable	= kai_hdmi_disable,

	.postsuspend	= kai_hdmi_vddio_disable,
	.hotplug_init	= kai_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data kai_disp2_pdata = {
	.flags		= 0,
	.default_out	= &kai_disp2_out,
	.fb		= &kai_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};
#endif

#ifdef PANEL_10_CPT_1366_768
static struct tegra_dc_out kai_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.dsi		= &kai_dsi,

	.flags		= DC_CTRL_MODE,
	.sd_settings	= &kai_sd_settings,


	.modes		= kai_panel_modes,
	.n_modes	= ARRAY_SIZE(kai_panel_modes),

	.enable		= kai_panel_enable,
	.disable	= kai_panel_disable,

	.height = 125,
	.width = 223,
	
};
#else
static struct tegra_dc_out kai_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &kai_sd_settings,
	.parent_clk	= "pll_p",
	.parent_clk_backup = "pll_d2_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 24/*18*/, /*CL2N*/
	.dither		= TEGRA_DC_ORDERED_DITHER,

/**20130110, JimmySu add chimei NJ5 panel driver**/
	.height = 65, /* mm */
	.width = 108, /* mm */

	.modes		= kai_panel_modes,
	.n_modes	= ARRAY_SIZE(kai_panel_modes),

/**20130110, JimmySu add chimei NJ5 panel driver**/
	.out_pins	= kai_dc_out_pins,
	.n_out_pins	= ARRAY_SIZE(kai_dc_out_pins),

	.enable		= kai_panel_enable,
	.disable	= kai_panel_disable,
};
#endif

#ifdef CONFIG_TEGRA_DC
static struct tegra_dc_platform_data kai_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &kai_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &kai_fb_data,
};

static struct nvhost_device kai_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= kai_disp1_resources,
	.num_resources	= ARRAY_SIZE(kai_disp1_resources),
	.dev = {
		.platform_data = &kai_disp1_pdata,
	},
};

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &kai_disp1_device.dev;
}

static struct nvhost_device kai_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= kai_disp2_resources,
	.num_resources	= ARRAY_SIZE(kai_disp2_resources),
	.dev = {
		.platform_data = &kai_disp2_pdata,
	},
};
#else
static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout kai_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by kai_panel_init() */
		.size		= 0,	/* Filled in by kai_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data kai_nvmap_data = {
	.carveouts	= kai_carveouts,
	.nr_carveouts	= ARRAY_SIZE(kai_carveouts),
};

static struct platform_device kai_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_nvmap_data,
	},
};
#endif

static struct platform_device *kai_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&kai_nvmap_device,
#endif
	&tegra_pwfm0_device,
	&kai_backlight_device,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend kai_panel_early_suspender;

static void kai_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_store_default_gov();
	cpufreq_change_gov(cpufreq_conservative_gov);
#endif
}

static void kai_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_gov();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

/*20121005, JimmySu add reboot notifier for warm-reset*/
#if 0  // remove notify function for panel power-off timing
static int panel_prepare_for_shutdown(struct notifier_block *this,
		unsigned long cmd, void *p)
{

//	printk("panel_prepare_for_shutdown\n");	
	if (cl2n_panel_status == 0)
		return NOTIFY_DONE;

	tegra_gpio_enable(kai_bl_pwm);
	gpio_request(kai_bl_pwm, "backlight-pwm-sleep");
	gpio_direction_output(kai_bl_pwm, 0);
	gpio_set_value(kai_bl_pwm, 0);

#ifdef PANEL_10_CPT_1366_768
//	regulator_disable(vdd_lcm_core);
	mdelay(1);
	gpio_set_value(kai_lcd_en_3v3, 0);
//	regulator_disable(kai_lvds_reg);
//	regulator_disable(kai_lvds_vdd_panel);
//	regulator_disable(vdd_dsi_csi);
#else
/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
	gpio_set_value(kai_vdd_bl_enb, 0);
	mdelay(120);
	gpio_set_value(kai_panel_avdd_en, 0);
	mdelay(60);
	gpio_set_value(kai_panel_rst, 0);

	regulator_disable(kai_lvds_reg);
	regulator_disable(kai_lvds_vdd_panel);

	gpio_set_value(kai_panel_lr, 0);
	gpio_set_value(kai_panel_ud, 0);
	gpio_set_value(kai_panel_mode, 0);
	gpio_set_value(kai_panel_dith, 0);
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/
#endif

	return NOTIFY_DONE;
}

static struct notifier_block panel_shutdown_notifier = {
		.notifier_call = panel_prepare_for_shutdown,
		.next = NULL,
		.priority = 0
};
#endif
/*20121005, JimmySu add reboot notifier for warm-reset*/

int __init kai_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	panel_board_id = nabijr_get_board_strap();

#if defined(CONFIG_TEGRA_NVMAP)
	kai_carveouts[1].base = tegra_carveout_start;
	kai_carveouts[1].size = tegra_carveout_size;
#endif

#ifdef PANEL_10_CPT_1366_768

//kai_lcd_en_3v3		TEGRA_GPIO_PI7 /* LCD_VDD_EN_3V3 */
//kai_lcd_id1		TEGRA_GPIO_PG2 /* LCD_ID1 */
//kai_lcd_id2		TEGRA_GPIO_PG3 /* LCD_ID2 */

// touch 1V8 enable config
	gpio_request(TEGRA_GPIO_PH3, "VDD_1V8_TP");
	gpio_direction_output(TEGRA_GPIO_PH3, 1);
	tegra_gpio_enable(TEGRA_GPIO_PH3);

	gpio_request(kai_lcd_en_3v3, "LCD_VDD_EN_3V3");
	gpio_direction_output(kai_lcd_en_3v3, 1);
	tegra_gpio_enable(kai_lcd_en_3v3);

	gpio_request(kai_lcd_id1, "LCD_ID1");
	gpio_direction_input(kai_lcd_id1);
	tegra_gpio_enable(kai_lcd_id1);

	gpio_request(kai_lcd_id2, "LCD_ID2");
	gpio_direction_input(kai_lcd_id2);
	tegra_gpio_enable(kai_lcd_id2);

#else
/**++++ 20130110, JimmySu add chimei NJ5 panel driver**/
	gpio_request(kai_panel_avdd_en, "panel_avdd_en");
	gpio_direction_output(kai_panel_avdd_en, 1);
	tegra_gpio_enable(kai_panel_avdd_en);
	
	gpio_request(kai_panel_rst, "panel_rst");
	gpio_direction_output(kai_panel_rst, 1);
	tegra_gpio_enable(kai_panel_rst);

	gpio_request(kai_panel_lr, "panel_lr");
	gpio_direction_output(kai_panel_lr, 1);
	tegra_gpio_enable(kai_panel_lr);

	gpio_request(kai_panel_ud, "panel_ud");
	gpio_direction_output(kai_panel_ud, 1);
	tegra_gpio_enable(kai_panel_ud);

	gpio_request(kai_panel_dith, "panel_dith");
	gpio_direction_output(kai_panel_dith, 1);
	tegra_gpio_enable(kai_panel_dith);

	gpio_request(kai_panel_mode, "panel_mode");
	gpio_direction_output(kai_panel_mode, 0);
	tegra_gpio_enable(kai_panel_mode);
/**---- 20130110, JimmySu add chimei NJ5 panel driver**/

	tegra_gpio_enable(kai_hdmi_hpd);
	gpio_request(kai_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(kai_hdmi_hpd);

#endif /*PANEL_10_CPT_1366_768*/

/*20121005, JimmySu add reboot notifier for warm-reset*/
//	err = register_reboot_notifier(&panel_shutdown_notifier);
//	if (err)
//		printk("panel Failed to register reboot notifier\n");
/*20121005, JimmySu add reboot notifier for warm-reset*/


/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/
	emc_clk_lock = clk_get_sys("floor.emc", NULL);
/*20121017, JimmySu add patch, emc clk not less than 102M when screen on*/

#ifdef CONFIG_HAS_EARLYSUSPEND
	kai_panel_early_suspender.suspend = kai_panel_early_suspend;
	kai_panel_early_suspender.resume = kai_panel_late_resume;
	kai_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&kai_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(kai_gfx_devices,
				ARRAY_SIZE(kai_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&kai_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&kai_disp1_device);

	res = nvhost_get_resource_byname(&kai_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&kai_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

#ifdef PANEL_10_CPT_1366_768
	kai_dsi.chip_id = tegra_get_chipid();
	kai_dsi.chip_rev = tegra_get_revision();
#endif /*PANEL_10_CPT_1366_768*/

	return err;
}
