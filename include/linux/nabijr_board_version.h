/*
 *	include/linux/nabijr_board_version.h
 */

#ifndef _LINUX_NABIJR_BOARD_VERSION_H
#define _LINUX_NABIJR_BOARD_VERSION_H

// arch/arm/mach-tegra/board.h
extern int nabijr_get_board_strap(void);

enum nabijr_board_version
{
  /* NABIJR Board Version */
	NABIJR_BOARD_VER_EVT    = 0x0,
	NABIJR_BOARD_VER_DVT    = 0x1,
	NABIJR_BOARD_VER_PVT    = 0x2,
	NABIJR_BOARD_VER_MP     = 0x3,
	NABIJR_BOARD_VER_DVT2   = 0x4,
};

#endif
