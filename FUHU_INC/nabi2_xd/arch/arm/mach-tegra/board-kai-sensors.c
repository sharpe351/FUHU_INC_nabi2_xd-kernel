
/*
 * arch/arm/mach-tegra/board-kai-sensors.c
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/jsa1127.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
//&*&*&cj1_20121220:Add camera for nj5
#include <media/ov2675.h>
#include <media/sensor_yuv.h>
//#include <media/ov5640.h>
//&*&*&cj2_20121220:Add camera for nj5
#include "board.h"
#include "board-kai.h"

//&*&*&cj1_20121220:Add camera for nj5

#define CAM_MCLK_EN_GPIO		TEGRA_GPIO_PCC0 //&*&*&*cj_20120612:add for MCLK power sequence  //CAM_MCLK
#define CAM_VDD_GPIO	TEGRA_GPIO_PS0	//ROW8 1.8V

/* rear camera */
#define CAM1_RST_GPIO			TEGRA_GPIO_PBB3
#define CAM1_POWER_DWN_GPIO		TEGRA_GPIO_PBB5

#define CAM1_LDO_CAM_GPIO	TEGRA_GPIO_PS7	//ROW15 2.8V
#define CAM1_LDO_VCM_GPIO	TEGRA_GPIO_PS6	//ROW14 2.8V

/* front camera */
#define CAM2_RST_GPIO			TEGRA_GPIO_PBB4
#define CAM2_POWER_DWN_GPIO		TEGRA_GPIO_PBB6

#define CAM2_LDO_GPIO	TEGRA_GPIO_PR7	//ROW7 2.8V


struct kai_cam_gpio {
	int gpio;
	const char *label;
	int value;
	int active_high;
};

#define TEGRA_CAM_GPIO(_gpio, _label, _value, _active_high) \
	{					\
		.gpio = _gpio,			\
		.label = _label,		\
		.value = _value,		\
		.active_high = _active_high	\
	}

struct kai_cam_power_rail {
	struct regulator *cam_1v8_reg;
	struct regulator *cam_2v8_reg;
	struct kai_cam_gpio *gpio_pwdn;
	struct kai_cam_gpio *gpio_rst;
};

enum CAM_INDEX {
	CAM_REAR,
	CAM_FRONT,
};

static struct kai_cam_gpio kai_cam_gpio_data[] = {
	[0] = TEGRA_CAM_GPIO(CAM1_POWER_DWN_GPIO, "cam1_power_en", 1, 1),
	[1] = TEGRA_CAM_GPIO(CAM1_RST_GPIO, "cam1_reset", 0, 0),
	[2] = TEGRA_CAM_GPIO(CAM2_POWER_DWN_GPIO, "cam2_power_en", 1, 1),
	[3] = TEGRA_CAM_GPIO(CAM2_RST_GPIO, "cam2_reset", 0, 0),
};

static struct kai_cam_power_rail kai_cam_pwr[] = {
	[0] = {
		.cam_1v8_reg = NULL,
		.cam_2v8_reg = NULL,
		.gpio_pwdn   = &kai_cam_gpio_data[0],
		.gpio_rst    = &kai_cam_gpio_data[1],
	},
	[1] = {
		.cam_1v8_reg = NULL,
		.cam_2v8_reg = NULL,
		.gpio_pwdn   = &kai_cam_gpio_data[2],
		.gpio_rst    = &kai_cam_gpio_data[3],
	},
};

//&*&*&cj2_20121220:Add camera for nj5

//&*&*&cj1_20120806:Add camera for CL2-N
static int kai_nxd_cam5MP_init(void)
{	
	struct kai_cam_power_rail *cam_pwr0 = &kai_cam_pwr[CAM_REAR];
//	struct kai_cam_power_rail *cam_pwr1 = &kai_cam_pwr[CAM_FRONT];
	printk("[camera](%s) \n",__FUNCTION__);
	gpio_set_value(CAM_VDD_GPIO, 1);//1.8 high
	msleep(1); 
	gpio_set_value(CAM1_LDO_CAM_GPIO, 1);//2.8v_5M high
////	gpio_set_value(CAM1_LDO_VCM_GPIO, 1);//2.8v_5M high
//	gpio_set_value(CAM2_LDO_GPIO, 1);//2.8v_2M high
	msleep(1);
	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	msleep(2);
	gpio_set_value(cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high);//pwdn_5M low
//	gpio_set_value(cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high);//pwdn_2M low
	msleep(2);
	gpio_set_value(cam_pwr0->gpio_rst->gpio, !cam_pwr0->gpio_rst->active_high);//rst_2M high
//	gpio_set_value(cam_pwr1->gpio_rst->gpio, !cam_pwr1->gpio_rst->active_high);//rst_2M high
	msleep(2);

	printk("%s, gpio_pwdn0=%d, value=%d, get=%d\n", __func__, cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high, gpio_get_value(cam_pwr0->gpio_pwdn->gpio));
//	printk("%s, gpio_pwdn1=%d, value=%d, get=%d\n", __func__, cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high, gpio_get_value(cam_pwr1->gpio_pwdn->gpio));

	return 0;
}


static int kai_nxd_cam2MP_init(void)
{	
//	struct kai_cam_power_rail *cam_pwr0 = &kai_cam_pwr[CAM_REAR];
	struct kai_cam_power_rail *cam_pwr1 = &kai_cam_pwr[CAM_FRONT];
	printk("[camera](%s) \n",__FUNCTION__);
	gpio_set_value(CAM_VDD_GPIO, 1);//1.8 high
	msleep(1); 
//	gpio_set_value(CAM1_LDO_CAM_GPIO, 1);//2.8v_5M high
//	gpio_set_value(CAM1_LDO_VCM_GPIO, 1);//2.8v_5M high
	gpio_set_value(CAM2_LDO_GPIO, 1);//2.8v_2M high
	msleep(1);
	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	msleep(2);
//	gpio_set_value(cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high);//pwdn_5M low
	gpio_set_value(cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high);//pwdn_2M low
	msleep(2);
//	gpio_set_value(cam_pwr0->gpio_rst->gpio, !cam_pwr0->gpio_rst->active_high);//rst_5M high
	gpio_set_value(cam_pwr1->gpio_rst->gpio, !cam_pwr1->gpio_rst->active_high);//rst_2M high
	msleep(2);

//	printk("%s, gpio_pwdn0=%d, value=%d, get=%d\n", __func__, cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high, gpio_get_value(cam_pwr0->gpio_pwdn->gpio));
	printk("%s, gpio_pwdn1=%d, value=%d, get=%d\n", __func__, cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high, gpio_get_value(cam_pwr1->gpio_pwdn->gpio));

	return 0;
}

static int kai_nxd_cam_suspend(void)
{
	
	struct kai_cam_power_rail *cam_pwr0 = &kai_cam_pwr[CAM_REAR];	
	struct kai_cam_power_rail *cam_pwr1 = &kai_cam_pwr[CAM_FRONT];	
	printk("[camera](%s) \n",__FUNCTION__);

//	gpio_set_value(cam_pwr0->gpio_pwdn->gpio, cam_pwr0->gpio_pwdn->active_high);//pwdn_5M high
//	gpio_set_value(cam_pwr1->gpio_pwdn->gpio, cam_pwr1->gpio_pwdn->active_high);//pwdn_2M high
	
	msleep(1);
	
	gpio_set_value(cam_pwr0->gpio_rst->gpio, cam_pwr0->gpio_rst->active_high);//rst_5M low
	gpio_set_value(cam_pwr1->gpio_rst->gpio, cam_pwr1->gpio_rst->active_high);//rst_2M low

	msleep(1);
	
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low

//	gpio_set_value(CAM1_LDO_VCM_GPIO, 0);//2.8v_5M low
	gpio_set_value(CAM1_LDO_CAM_GPIO, 0);//2.8v_5M low
	gpio_set_value(CAM2_LDO_GPIO, 0);//2.8v_2M low

	gpio_set_value(CAM_VDD_GPIO, 0);//1.8v low

	msleep(5);
	
	gpio_set_value(cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high);//pwdn_2M low
	gpio_set_value(cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high);//pwdn_2M low


//	printk("%s, gpio_pwdn0=%d, value=%d, get=%d\n", __func__, cam_pwr0->gpio_pwdn->gpio, !cam_pwr0->gpio_pwdn->active_high, gpio_get_value(cam_pwr0->gpio_pwdn->gpio));
//	printk("%s, gpio_pwdn1=%d, value=%d, get=%d\n", __func__, cam_pwr1->gpio_pwdn->gpio, !cam_pwr1->gpio_pwdn->active_high, gpio_get_value(cam_pwr1->gpio_pwdn->gpio));
	
	return 0;
}

static int kai_camera_init(void)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(kai_cam_gpio_data); i++) {
		ret = gpio_request(kai_cam_gpio_data[i].gpio,
				   kai_cam_gpio_data[i].label);

		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, kai_cam_gpio_data[i].gpio);
			goto fail_free_gpio;
		}
		gpio_direction_output(kai_cam_gpio_data[i].gpio,
				      kai_cam_gpio_data[i].value);
		gpio_export(kai_cam_gpio_data[i].gpio, false);
		tegra_gpio_enable(kai_cam_gpio_data[i].gpio);
	}
	
	tegra_gpio_enable(CAM1_LDO_CAM_GPIO);
	ret = gpio_request(CAM1_LDO_CAM_GPIO, "cam1_ldo_cam_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM1_LDO_CAM_GPIO");
			goto fail_cam1_cam_gpio;
	}
	tegra_gpio_enable(CAM1_LDO_VCM_GPIO);
	ret = gpio_request(CAM1_LDO_VCM_GPIO, "cam1_ldo_vcm_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM1_LDO_VCM_GPIO");
			goto fail_cam1_vcm_gpio;
	}
	tegra_gpio_enable(CAM2_LDO_GPIO);
	ret = gpio_request(CAM2_LDO_GPIO, "cam2_ldo_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM2_LDO_GPIO");
			goto fail_cam2_gpio;
	}
	
	tegra_gpio_enable(CAM_VDD_GPIO);
	ret = gpio_request(CAM_VDD_GPIO, "cam_vdd_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_VDD_GPIO");
			goto fail_vdd_gpio;
	}
	gpio_direction_output(CAM1_LDO_CAM_GPIO,0);
	gpio_direction_output(CAM1_LDO_VCM_GPIO,0);
	gpio_direction_output(CAM2_LDO_GPIO,0);
	gpio_direction_output(CAM_VDD_GPIO,0);

	ret = gpio_request(CAM_MCLK_EN_GPIO, "mclk_disable");
	if (ret < 0)
	{
			pr_err("CAM_MCLK_EN_GPIO gpio_request failed\n");
			goto fail_mclk_gpio;
	}
	gpio_direction_output(CAM_MCLK_EN_GPIO,0);
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);

	gpio_direction_output(CAM1_POWER_DWN_GPIO,0);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM2_POWER_DWN_GPIO,0);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);
		
	return 0;

fail_free_gpio:
	pr_err("%s failed!", __func__);
	while(i--)
		gpio_free(kai_cam_gpio_data[i].gpio);
	return ret;

fail_mclk_gpio:
	gpio_free(CAM_MCLK_EN_GPIO);
	return ret;

fail_vdd_gpio:
	gpio_free(CAM_VDD_GPIO);
	return ret;
fail_cam1_cam_gpio:
	gpio_free(CAM1_LDO_CAM_GPIO);
	return ret;
fail_cam1_vcm_gpio:
	gpio_free(CAM1_LDO_VCM_GPIO);
	return ret;
fail_cam2_gpio:
	gpio_free(CAM2_LDO_GPIO);
	return ret;
		
}

static int kai_ov5640_power_on(void)
{
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];	
	printk("[camera](%s) \n",__FUNCTION__);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_2M low

//	printk("%s, gpio_pwdn=%d, value=%d, get=%d\n", __func__, cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high, gpio_get_value(cam_pwr->gpio_pwdn->gpio));
	
	msleep(1);

	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	
	msleep(1);
	
	
	return 0;
}

static int kai_ov5640_power_off(void)
{
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];
	printk("[camera](%s) \n",__FUNCTION__);

	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low
	
	msleep(1);	
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_2M high

//	printk("%s, gpio_pwdn=%d, value=%d, get=%d\n", __func__, cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high, gpio_get_value(cam_pwr->gpio_pwdn->gpio));

	msleep(1);	
		
	
	return 0;
}

static int kai_ov2675_power_on(void)
{
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_FRONT];	
	printk("[camera](%s) \n",__FUNCTION__);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_2M low

//	printk("%s, gpio_pwdn=%d, value=%d, get=%d\n", __func__, cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high, gpio_get_value(cam_pwr->gpio_pwdn->gpio));
	
	msleep(1);

	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	
	msleep(1);
	
	
	return 0;
}

static int kai_ov2675_power_off(void)
{
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_FRONT];
	printk("[camera](%s) \n",__FUNCTION__);

	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low
	
	msleep(1);	
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_2M high

//	printk("%s, gpio_pwdn=%d, value=%d, get=%d\n", __func__, cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high, gpio_get_value(cam_pwr->gpio_pwdn->gpio));

	msleep(1);	
		
	
	return 0;
}


static int kai_ov5640_vcm_on(void)
{
	gpio_set_value(CAM1_LDO_VCM_GPIO, 1);//2.8v_5M high
	return 0;
}

static int kai_ov5640_vcm_off(void)
{
	gpio_set_value(CAM1_LDO_VCM_GPIO, 0);//2.8v_5M low
	return 0;
}
	
struct ov5640_platform_data kai_ov5640_data = {
	.init = kai_nxd_cam5MP_init,
	.suspend = kai_nxd_cam_suspend,
	.power_on = kai_ov5640_power_on,
	.power_off = kai_ov5640_power_off,
	.vcm_pwr_on = kai_ov5640_vcm_on,
	.vcm_pwr_off = kai_ov5640_vcm_off,
};

struct yuv_sensor_platform_data kai_ov2675_data = {
	.init = kai_nxd_cam2MP_init,
	.suspend = kai_nxd_cam_suspend,
	.power_on = kai_ov2675_power_on,
	.power_off = kai_ov2675_power_off,
};

static struct i2c_board_info kai_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ov5640", 0x3C),  //0x78
		.platform_data = &kai_ov5640_data,
	},
	{
		I2C_BOARD_INFO("ov2675", 0x30),  //0x60
		.platform_data = &kai_ov2675_data,
	},
};

//&*&*&cj2_20120806:Add camera for CL2-N

static void jsa1127_configure_platform(void)
{
	/* regulator_get regulator_enable*/
}

static struct jsa1127_platform_data kai_jsa1127_pdata = {
    .configure_platform = jsa1127_configure_platform,
};

static struct i2c_board_info kai_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("jsa1127", 0x39),
		.platform_data = &kai_jsa1127_pdata,
	},
	{
		I2C_BOARD_INFO("bma2x2", 0x10),
	},
	{
		I2C_BOARD_INFO("bmm050", 0x12),
	},
};

static inline void kai_msleep(u32 t)
{
        /*
        If timer value is between ( 10us - 20ms),
        usleep_range() is recommended.
        Please read Documentation/timers/timers-howto.txt.
        */
        usleep_range(t*1000, t*1000 + 500);
}


int __init kai_sensors_init(void)
{

//&*&*&cj1_20121221:Add camera for nj5
	kai_camera_init();
	kai_nxd_cam5MP_init();
	kai_nxd_cam2MP_init();
	kai_ov5640_power_off();
	kai_ov2675_power_off();
	
	i2c_register_board_info(2, kai_i2c2_board_info,
		ARRAY_SIZE(kai_i2c2_board_info));
//&*&*&cj2_20121221:Add camera for nj5
	i2c_register_board_info(0, kai_i2c0_board_info,
		ARRAY_SIZE(kai_i2c0_board_info));

	return 0;
}
