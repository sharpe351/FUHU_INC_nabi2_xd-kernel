/*
 * ov5640.h - header for YUV camera sensor OV5640 driver.
 *
 * Copyright (C) 2012 NVIDIA Corporation.
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

#ifndef __OV5640_H__
#define __OV5640_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV5640_IOCTL_SET_SENSOR_MODE		_IOW('o', 1, struct ov5640_mode)
#define OV5640_IOCTL_GET_SENSOR_STATUS	_IOR('o', 2, __u8)
#define OV5640_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define OV5640_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define OV5640_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define OV5640_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define OV5640_IOCTL_GET_AF_STATUS      _IOR('o', 7, __u8)
#define OV5640_IOCTL_SET_EXPOSURE       _IOW('o', 8, int)
#define OV5640_IOCTL_SET_PREVIEW				_IOW('o', 9, struct ov5640_mode)
#define OV5640_IOCTL_SET_CAPTURE				_IOW('o', 10, struct ov5640_mode)
#define OV5640_IOCTL_SET_FPOSITION      _IOW('o', 11, __u32)
#define OV5640_IOCTL_GET_CONFIG					_IOR('o', 12, struct ov5640_config)
#define OV5640_IOCTL_POWER_LEVEL				_IOW('o', 13, __u32)



#define OV5640_POWER_LEVEL_OFF		0
#define OV5640_POWER_LEVEL_ON		1
#define OV5640_POWER_LEVEL_SUS		2

enum {
      YUV_ColorEffect = 0,
      YUV_Whitebalance,
      YUV_SceneMode,
      YUV_Exposure
};

enum {
      YUV_ColorEffect_Invalid = 0,
      YUV_ColorEffect_Aqua,
      YUV_ColorEffect_Blackboard,
      YUV_ColorEffect_Mono,
      YUV_ColorEffect_Negative,
      YUV_ColorEffect_None,
      YUV_ColorEffect_Posterize,
      YUV_ColorEffect_Sepia,
      YUV_ColorEffect_Solarize,
      YUV_ColorEffect_Whiteboard
};

enum {
      YUV_Whitebalance_Invalid = 0,
      YUV_Whitebalance_Auto,
      YUV_Whitebalance_Incandescent,
      YUV_Whitebalance_Fluorescent,
      YUV_Whitebalance_WarmFluorescent,
      YUV_Whitebalance_Daylight,
      YUV_Whitebalance_CloudyDaylight,
      YUV_Whitebalance_Shade,
      YUV_Whitebalance_Twilight,
      YUV_Whitebalance_Custom
};

enum {
      YUV_SceneMode_Invalid = 0,
      YUV_SceneMode_Auto,
      YUV_SceneMode_Action,
      YUV_SceneMode_Portrait,
      YUV_SceneMode_Landscape,
      YUV_SceneMode_Beach,
      YUV_SceneMode_Candlelight,
      YUV_SceneMode_Fireworks,
      YUV_SceneMode_Night,
      YUV_SceneMode_NightPortrait,
      YUV_SceneMode_Party,
      YUV_SceneMode_Snow,
      YUV_SceneMode_Sports,
      YUV_SceneMode_SteadyPhoto,
      YUV_SceneMode_Sunset,
      YUV_SceneMode_Theatre,
      YUV_SceneMode_Barcode
};

enum {
      YUV_Exposure_Negative_2 = -2,
      YUV_Exposure_Negative_1,
      YUV_Exposure_0,
      YUV_Exposure_1,
      YUV_Exposure_2
};

struct ov5640_mode {
	int xres;
	int yres;
};

struct ov5640_config {
	__u32 settle_time;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
};

enum {
	OV5640_AF_INIFINITY,
	OV5640_AF_TRIGGER,
};

#ifdef __KERNEL__
struct ov5640_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	int (*vcm_pwr_on)(void);
	int (*vcm_pwr_off)(void);
	int (*init)(void);
	int (*suspend)(void);
};
#endif /* __KERNEL__ */

#endif  /* __OV5640_H__ */
