/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: junghyun, kim <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/hdmi.h>

#include "s5pxx18_hdmi.h"

#define DEFAULT_SAMPLE_RATE		48000
#define DEFAULT_BITS_PER_SAMPLE		16
#define DEFAULT_AUDIO_CODEC		HDMI_AUDIO_PCM
#define DEFAULT_HDMIPHY_TX_LEVEL	23

/*
 * HDMI preset configs
 */
static const struct hdmi_preset hdmi_conf_800x480p66 = {
		.mode = {
				.pixelclock = 32000000,
				.h_as = 800, .h_sw = 48, .h_bp = 60, .h_fp = 40, .h_si = 0,
				.v_as = 480, .v_sw = 3, .v_bp = 30, .v_fp = 13, .v_si = 0,
				.refresh = 66,
				.name = "800x480p@66",
				.flags = 0,
		},

};

static struct hdmi_preset hdmi_conf_480p60 = {
	.mode = {
		 .pixelclock = 27027000,
		 .h_as = 720, .h_sw = 62, .h_bp = 60, .h_fp = 16, .h_si = 0,
		 .v_as = 480, .v_sw = 6, .v_bp = 30, .v_fp = 9, .v_si = 0,
		 .refresh = 60,
		 .name = "720x480p@60",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x8a, 0x00},
		 .v2_blank = {0x0d, 0x02},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x0d, 0x02},
		 .h_line = {0x5a, 0x03},
		 .hsync_pol = {0x01},
		 .vsync_pol = {0x01},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x0e, 0x00},
		 .h_sync_end = {0x4c, 0x00},
		 .v_sync_line_bef_2 = {0x0f, 0x00},
		 .v_sync_line_bef_1 = {0x09, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x5a, 0x03,	/* h_fsz */
	       0x8a, 0x00, 0xd0, 0x02,	/* hact */
	       0x0d, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0xe0, 0x01,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 3,
};

static struct hdmi_preset hdmi_conf_480p59_94 = {
	.mode = {
		 .pixelclock = 27000000,
		 .h_as = 720, .h_sw = 61, .h_bp = 60, .h_fp = 16, .h_si = 0,
		 .v_as = 480, .v_sw = 6, .v_bp = 30, .v_fp = 9, .v_si = 0,
		 .refresh = 59,
		 .name = "720x480p@59.94",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x8a, 0x00},
		 .v2_blank = {0x0d, 0x02},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x0d, 0x02},
		 .h_line = {0x5a, 0x03},
		 .hsync_pol = {0x01},
		 .vsync_pol = {0x01},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x0e, 0x00},
		 .h_sync_end = {0x4c, 0x00},
		 .v_sync_line_bef_2 = {0x0f, 0x00},
		 .v_sync_line_bef_1 = {0x09, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x5a, 0x03,	/* h_fsz */
	       0x8a, 0x00, 0xd0, 0x02,	/* hact */
	       0x0d, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0xe0, 0x01,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 3,
};

/* this preset is used for setting 800x480 display from seed studio */
static const struct hdmi_preset hdmi_conf_800x480p66Capacitive = {
	.mode = {
		.pixelclock = 32000000,
		.h_as = 800, .h_sw = 48, .h_bp = 40, .h_fp = 40, .h_si = 0,
		.v_as = 480, .v_sw = 3, .v_bp = 29, .v_fp = 13, .v_si = 0,
		.refresh = 66,
		.name = "800x480pCapactive@66",
		.flags = 0
	},
};

static struct hdmi_preset hdmi_conf_576p50 = {
	.mode = {
		 .pixelclock = 27000000,
		 .h_as = 720, .h_sw = 64, .h_bp = 68, .h_fp = 12, .h_si = 0,
		 .v_as = 576, .v_sw = 5, .v_bp = 39, .v_fp = 5, .v_si = 0,
		 .refresh = 50,
		 .name = "720x576p@50",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x90, 0x00},
		 .v2_blank = {0x71, 0x02},
		 .v1_blank = {0x31, 0x00},
		 .v_line = {0x71, 0x02},
		 .h_line = {0x60, 0x03},
		 .hsync_pol = {0x01},
		 .vsync_pol = {0x01},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x0a, 0x00},
		 .h_sync_end = {0x4a, 0x00},
		 .v_sync_line_bef_2 = {0x0a, 0x00},
		 .v_sync_line_bef_1 = {0x05, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x60, 0x03,	/* h_fsz */
	       0x90, 0x00, 0xd0, 0x02,	/* hact */
	       0x71, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x31, 0x00, 0x40, 0x02,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 18,
};

/* this preset is used for setting 1024x600 display from waveshare "7 */
static const struct hdmi_preset hdmi_conf_1024x600p43Capacitive = {
	.mode = {
		.pixelclock = 32000000,
		.h_as = 1024, .h_sw = 48, .h_bp = 40, .h_fp = 40, .h_si = 0,
		.v_as = 600, .v_sw = 3, .v_bp = 29, .v_fp = 13, .v_si = 0,
		.refresh = 43,
		.name = "1024x600p@43",
		.flags = 0,
	},
};

struct hdmi_preset hdmi_conf_2Ddynamic_detect = {
	.mode = {
		.pixelclock = 0xffffffff,
		.h_as = 0xff, .h_sw = 0xff, .h_bp = 0xff, .h_fp = 0xff,
		.h_si = 0, .v_as = 0xff, .v_sw = 0xff, .v_bp = 0xff,
		.v_fp = 0xff, .v_si = 0,
		.refresh = 0xff,
		.name = "2DxDynamicDetect",
		.flags = 0,
	},
};

static struct hdmi_preset hdmi_conf_720p50 = {
	.mode = {
		 .pixelclock = 74250000,
		 .h_as = 1280, .h_sw = 40, .h_bp = 220, .h_fp = 440, .h_si = 0,
		 .v_as = 720, .v_sw = 5, .v_bp = 20, .v_fp = 5, .v_si = 0,
		 .refresh = 50,
		 .name = "1280x720p@50",
		 .flags = 0,
	},
	.core = {
		 .h_blank = {0xbc, 0x02},
		 .v2_blank = {0xee, 0x02},
		 .v1_blank = {0x1e, 0x00},
		 .v_line = {0xee, 0x02},
		 .h_line = {0xbc, 0x07},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0xb6, 0x01},
		 .h_sync_end = {0xde, 0x01},
		 .v_sync_line_bef_2 = {0x0a, 0x00},
		 .v_sync_line_bef_1 = {0x05, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
	},
	.tg = {
	       0x00,		/* cmd */
	       0xbc, 0x07,	/* h_fsz */
	       0xbc, 0x02, 0x00, 0x05,	/* hact */
	       0xee, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x1e, 0x00, 0xd0, 0x02,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	},
	.vic = 19,
};

static struct hdmi_preset hdmi_conf_720p60 = {
	.mode = {
		 .pixelclock = 74250000,
		 .h_as = 1280, .h_sw = 40, .h_bp = 220, .h_fp = 110, .h_si = 0,
		 .v_as = 720, .v_sw = 5, .v_bp = 20, .v_fp = 5, .v_si = 0,
		 .refresh = 60,
		 .name = "1280x720p@60",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {370 % 256, 370 >> 8},
		 .v2_blank = {750 % 256, 750 >> 8},
		 .v1_blank = {30, 0},
		 .v_line = {0xee, 0x02},
		 .h_line = {0x72, 0x06},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x6c, 0x00},
		 .h_sync_end = {0x94, 0x00},
		 .v_sync_line_bef_2 = {0x0a, 0x00},
		 .v_sync_line_bef_1 = {0x05, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x72, 0x06,	/* h_fsz */
	       0x72, 0x01, 0x00, 0x05,	/* hact */
	       0xee, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x1e, 0x00, 0xd0, 0x02,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 4,
};

/* future design inc 1280x800, ELI101-CPW */
static struct hdmi_preset hdmi_conf_800p60 = {
	.mode = {
		 .pixelclock = 73020000,
		 .h_as = 1280, .h_sw = 60, .h_bp = 26, .h_fp = 95, .h_si = 0,
		 .v_as = 800, .v_sw = 15, .v_bp = 2, .v_fp = 16, .v_si = 0,
		 .refresh = 60,
		 .name = "1280x800p@60",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {370 % 256, 370 >> 8},
		 .v2_blank = {750 % 256, 750 >> 8},
		 .v1_blank = {30, 0},
		 .v_line = {0xee, 0x02},
		 .h_line = {0x72, 0x06},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x6c, 0x00},
		 .h_sync_end = {0x94, 0x00},
		 .v_sync_line_bef_2 = {0x0a, 0x00},
		 .v_sync_line_bef_1 = {0x05, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x72, 0x06,	/* h_fsz */
	       0x72, 0x01, 0x00, 0x05,	/* hact */
	       0xee, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x1e, 0x00, 0xd0, 0x02,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 4,
};

static struct hdmi_preset hdmi_conf_1080p60 = {
	.mode = {
		 .pixelclock = 148500000,
		 .h_as = 1920, .h_sw = 44, .h_bp = 148, .h_fp = 88, .h_si = 0,
		 .v_as = 1080, .v_sw = 5, .v_bp = 36, .v_fp = 4, .v_si = 0,
		 .refresh = 60,
		 .name = "1920x1080p@60",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x18, 0x01},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0x98, 0x08},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x56, 0x00},
		 .h_sync_end = {0x82, 0x00},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x98, 0x08,	/* h_fsz */
	       0x18, 0x01, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 16,
};

static struct hdmi_preset hdmi_conf_1080p50 = {
	.mode = {
		 .pixelclock = 148500000,
		 .h_as = 1920, .h_sw = 44, .h_bp = 148, .h_fp = 528, .h_si = 0,
		 .v_as = 1080, .v_sw = 5, .v_bp = 36, .v_fp = 4, .v_si = 0,
		 .refresh = 50,
		 .name = "1920x1080p@50",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0xd0, 0x02},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0x50, 0x0a},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x0e, 0x02},
		 .h_sync_end = {0x3a, 0x02},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x50, 0x0a,	/* h_fsz */
	       0xd0, 0x02, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 31,
};

static struct hdmi_preset hdmi_conf_1080p24 = {
	.mode = {
		 .pixelclock = 0,
		 .h_as = 1920, .h_sw = 44, .h_bp = 148, .h_fp = 638, .h_si = 0,
		 .v_as = 1080, .v_sw = 5, .v_bp = 36, .v_fp = 4, .v_si = 0,
		 .refresh = 24,
		 .name = "1920x1080p@24",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x3e, 0x03},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0xbe, 0x0a},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x7c, 0x02},
		 .h_sync_end = {0xa8, 0x02},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0xbe, 0x0a,	/* h_fsz */
	       0x3e, 0x03, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 32,
};

static struct hdmi_preset hdmi_conf_720p59_94 = {
	.mode = {
		 .pixelclock = 74175000,
		 .h_as = 1280, .h_sw = 40, .h_bp = 220, .h_fp = 110, .h_si = 0,
		 .v_as = 720, .v_sw = 5, .v_bp = 20, .v_fp = 5, .v_si = 0,
		 .refresh = 59,
		 .name = "1280x720p@59.94",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x72, 0x01},
		 .v2_blank = {0xee, 0x02},
		 .v1_blank = {0x1e, 0x00},
		 .v_line = {0xee, 0x02},
		 .h_line = {0x72, 0x06},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x6c, 0x00},
		 .h_sync_end = {0x94, 0x00},
		 .v_sync_line_bef_2 = {0x0a, 0x00},
		 .v_sync_line_bef_1 = {0x05, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x72, 0x06,	/* h_fsz */
	       0x72, 0x01, 0x00, 0x05,	/* hact */
	       0xee, 0x02,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x1e, 0x00, 0xd0, 0x02,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 4,
};

static struct hdmi_preset hdmi_conf_1080p59_94 = {
	.mode = {
		 .pixelclock = 148352000,
		 .h_as = 1920, .h_sw = 44, .h_bp = 148, .h_fp = 88, .h_si = 0,
		 .v_as = 1080, .v_sw = 5, .v_bp = 36, .v_fp = 4, .v_si = 0,
		 .refresh = 59,
		 .name = "1920x1080p@59.94",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x18, 0x01},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0x98, 0x08},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x56, 0x00},
		 .h_sync_end = {0x82, 0x00},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x98, 0x08,	/* h_fsz */
	       0x18, 0x01, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 16,
};

static struct hdmi_preset hdmi_conf_1080p60_sb_h = {
	.mode = {
		 .pixelclock = 0,
		 .h_as = 1920,
		 .v_as = 1080,
		 .refresh = 60,
		 .name = "",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x18, 0x01},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0x98, 0x08},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x56, 0x00},
		 .h_sync_end = {0x82, 0x00},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x98, 0x08,	/* h_fsz */
	       0x18, 0x01, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 16,
};

static struct hdmi_preset hdmi_conf_1080p60_tb = {
	.mode = {
		 .pixelclock = 0,
		 .h_as = 1920,
		 .v_as = 1080,
		 .refresh = 60,
		 .name = "",
		 .flags = 0,
		 },
	.core = {
		 .h_blank = {0x18, 0x01},
		 .v2_blank = {0x65, 0x04},
		 .v1_blank = {0x2d, 0x00},
		 .v_line = {0x65, 0x04},
		 .h_line = {0x98, 0x08},
		 .hsync_pol = {0x00},
		 .vsync_pol = {0x00},
		 .int_pro_mode = {0x00},
		 .v_blank_f0 = {0xff, 0xff},
		 .v_blank_f1 = {0xff, 0xff},
		 .h_sync_start = {0x56, 0x00},
		 .h_sync_end = {0x82, 0x00},
		 .v_sync_line_bef_2 = {0x09, 0x00},
		 .v_sync_line_bef_1 = {0x04, 0x00},
		 .v_sync_line_aft_2 = {0xff, 0xff},
		 .v_sync_line_aft_1 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_2 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_1 = {0xff, 0xff},
		 .v_blank_f2 = {0xff, 0xff},
		 .v_blank_f3 = {0xff, 0xff},
		 .v_blank_f4 = {0xff, 0xff},
		 .v_blank_f5 = {0xff, 0xff},
		 .v_sync_line_aft_3 = {0xff, 0xff},
		 .v_sync_line_aft_4 = {0xff, 0xff},
		 .v_sync_line_aft_5 = {0xff, 0xff},
		 .v_sync_line_aft_6 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_3 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_4 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_5 = {0xff, 0xff},
		 .v_sync_line_aft_pxl_6 = {0xff, 0xff},
		 .vact_space_1 = {0xff, 0xff},
		 .vact_space_2 = {0xff, 0xff},
		 .vact_space_3 = {0xff, 0xff},
		 .vact_space_4 = {0xff, 0xff},
		 .vact_space_5 = {0xff, 0xff},
		 .vact_space_6 = {0xff, 0xff},
		 /* other don't care */
		 },
	.tg = {
	       0x00,		/* cmd */
	       0x98, 0x08,	/* h_fsz */
	       0x18, 0x01, 0x80, 0x07,	/* hact */
	       0x65, 0x04,	/* v_fsz */
	       0x01, 0x00, 0x33, 0x02,	/* vsync */
	       0x2d, 0x00, 0x38, 0x04,	/* vact */
	       0x33, 0x02,	/* field_chg */
	       0x48, 0x02,	/* vact_st2 */
	       0x00, 0x00,	/* vact_st3 */
	       0x00, 0x00,	/* vact_st4 */
	       0x01, 0x00, 0x01, 0x00,	/* vsync top/bot */
	       0x01, 0x00, 0x33, 0x02,	/* field top/bot */
	       0x00,		/* 3d FP */
	       },
	.vic = 16,
};

static struct hdmi_preset hdmi_conf_768p60 = {
	.mode = {
		 .pixelclock = 65000000,
		 .h_as = 1024, .h_sw = 136, .h_bp = 160, .h_fp = 24, .h_si = 0,
		 .v_as = 768, .v_sw = 6, .v_bp = 29, .v_fp = 3, .v_si = 0,
		 .refresh = 60,
		 .name = "1024x768p@60",
		 .flags = 0,
		 },
};

static const struct hdmi_preset hdmi_conf_1024p60 = {
	.mode = {
		 .pixelclock = 108000000,
		 .h_as = 1280, .h_sw = 112, .h_bp = 248, .h_fp = 48, .h_si = 0,
		 .v_as = 1024, .v_sw = 3, .v_bp = 38, .v_fp = 1, .v_si = 0,
		 .refresh = 60,
		 .name = "1280x1024p@60",
		 .flags = 0,
		 },
};
/*
 * PHY preset data tables
 */
static const u8 hdmiphy_preset_25_2[32] = {
	0x52, 0x3f, 0x55, 0x40, 0x01, 0x00, 0xc8, 0x82,
	0xc8, 0xbd, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x01, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xf4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_25_175[32] = {
	0xd1, 0x1f, 0x50, 0x40, 0x20, 0x1e, 0xc8, 0x81,
	0xe8, 0xbd, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xf4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_27[32] = {
	0xd1, 0x22, 0x51, 0x40, 0x08, 0xfc, 0xe0, 0x98,
	0xe8, 0xcb, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xe4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_27_027[32] = {
	0xd1, 0x2d, 0x72, 0x40, 0x64, 0x12, 0xc8, 0x43,
	0xe8, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xe3, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_32[32] = {
	0xd1, 0x22, 0x51, 0x40, 0x08, 0xfc, 0xe0, 0x98,
	0xe8, 0xcb, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xe4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_54[32] = {
	0x51, 0x2d, 0x35, 0x40, 0x01, 0x00, 0xc8, 0x82,
	0xc8, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80, 0x06,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xe4, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_54_054[32] = {
	0xd1, 0x2d, 0x32, 0x40, 0x64, 0x12, 0xc8, 0x43,
	0xe8, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xe3, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_65[32] = {
	0xD1, 0x36, 0x34, 0x40, 0x0C, 0x04, 0xC8, 0x82,
	0xE8, 0x45, 0xD9, 0x45, 0xA0, 0xAC, 0x80, 0x08,
	0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66, 0x54,
	0xBD, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_74_175[32] = {
	0xd1, 0x1f, 0x10, 0x40, 0x5b, 0xef, 0xc8, 0x81,
	0xe8, 0xb9, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x56,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xa6, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_74_25[32] = {
	0xd1, 0x1f, 0x10, 0x40, 0x40, 0xf8, 0xc8, 0x81,
	0xe8, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x08,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xa5, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_108[32] = {
	0x51, 0x2d, 0x15, 0x40, 0x01, 0x00, 0xc8, 0x82,
	0xc8, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80, 0x08,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0xc7, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_148_352[32] = {
	0xd1, 0x1f, 0x00, 0x40, 0x5b, 0xef, 0xc8, 0x81,
	0xe8, 0xb9, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x0a,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0x4b, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset_148_5[32] = {
	0xd1, 0x1f, 0x00, 0x40, 0x40, 0xf8, 0xc8, 0x81,
	0xe8, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x08,
	0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
	0x4b, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80, 0x10,
};

u8 hdmiphy_dynamic_preset[32];

static const struct hdmi_format _format_2d = {
	.vformat = HDMI_VIDEO_FORMAT_2D,
};

static const struct hdmi_format _format_sbh = {
	.vformat = HDMI_VIDEO_FORMAT_3D,
	.type_3d = HDMI_3D_TYPE_SB_HALF,
};

static const struct hdmi_format _format_tb = {
	.vformat = HDMI_VIDEO_FORMAT_3D,
	.type_3d = HDMI_3D_TYPE_TB,
};

static const struct hdmi_format _format_fp = {
	.vformat = HDMI_VIDEO_FORMAT_3D,
	.type_3d = HDMI_3D_TYPE_FP,
};

const struct hdmi_conf hdmi_conf[] = {
	{ /* 0 : 720x480p@59.94 */
	 .preset = &hdmi_conf_480p59_94,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_27,
	 .support = true,
	},
	{ /* 1 : 720x480p@60 */
	 .preset = &hdmi_conf_480p60,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_27_027,
	 .support = true,
	},
	{ /* 2 : 720x576p@50 */
	 .preset = &hdmi_conf_576p50,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_27,
	 .support = true,
	},

	{ /* 3 : 800x480@66 */
	 .preset = &hdmi_conf_800x480p66,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_32,
	 .support = true,
	},
	{ /* 4 : 1280x720p@50 */
	 .preset = &hdmi_conf_720p50,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_74_25,
	 .support = true,
	},

	{ /* 5 : 1280x720p@59.94 */
	 .preset = &hdmi_conf_720p59_94,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_74_175,
	 .support = true,
	},

	{ /* 6 :1280x720p@60 */
	 .preset = &hdmi_conf_720p60,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_74_25,
	 .support = true,
	},

	{ /* 7 : 1920x1080p@50 */
	 .preset = &hdmi_conf_1080p50,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_148_5,
	 .support = true,
	},

	{ /* 8 : 1920x1080p@59.94 */
	 .preset = &hdmi_conf_1080p59_94,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_148_352,
	 .support = true,
	},

	{ /* 9 : 1920x1080p@60 */
	 .preset = &hdmi_conf_1080p60,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_148_5,
	 .support = true,
	},

	{ /* 10 : 1024x768@60 */

	 .preset = &hdmi_conf_768p60,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_65,
	 .support = true,
	},

	{ /* 11 : 1280x1024@60 */

	 .preset = &hdmi_conf_1024p60,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_108,
	 .support = true,
	},
	{ /* 14  : 1920x1080p@24 */
	 .preset = &hdmi_conf_1080p24,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_preset_74_25,
	 .support = false,
	},
	{
	 .preset = &hdmi_conf_1080p60_sb_h,
	 .format = &_format_sbh,
	},
	{
	 .preset = &hdmi_conf_1080p60_tb,
	 .format = &_format_tb,
	},
	{ /* dynamic mode */
	 .preset = &hdmi_conf_2Ddynamic_detect,
	 .format = &_format_2d,
	 .phy_data = hdmiphy_dynamic_preset,
	 .support = true,
	},
};

const int num_hdmi_presets = ARRAY_SIZE(hdmi_conf);
/* Note: list this array with pixel clock in asendant order,
 * the function hdmi_get_closest_timing expecting it that way
 */
const struct hdmi_vic_conf hdmi_vic_conf[] = {
	{
		.hdmiphy_preset = hdmiphy_preset_25_2,
		.pixelclock = 25200000,
		.htotal = 800, /* guesswork */
		.vtotal = 525,
		.refresh = 60,
	},
	/* 800x600@60 */
	{
		.hdmiphy_preset = hdmiphy_preset_54,
		.pixelclock = 54000000,
		.htotal = 1125, /* guesswork */
		.vtotal = 800,
		.refresh = 60,
	},
	/* 1024x768@60 */
	{
		.hdmiphy_preset = hdmiphy_preset_65,
		.pixelclock = 64990000,
		.htotal = 1344, /* 1024 + 24(fp) + 136(sync) + 160(bp) */
		.vtotal = 806, /* 768 + 3(fp) + 6(sync) + 29(bp) */
		.refresh = 60,
	},
	/* 1280x720@60 */
	{
		.hdmiphy_preset = hdmiphy_preset_74_25,
		.pixelclock = 74250000,
		.htotal = 1650, /* 1280 + 110 + 40 + 220 */
		.vtotal = 750, /* 720 + 5 + 5 + 20 */
		.refresh = 60,
	},
	/* 1280x1024 */
	{
		.hdmiphy_preset = hdmiphy_preset_108,
		.pixelclock = 108000000,
		.htotal = 1688, /* 1280 + 48 + 112 + 248 */
		.vtotal = 1066, /* 1024 + 1 + 3 + 38 */
		.refresh = 60,
	},
	/* 1920x1080@60 */
	{
		.hdmiphy_preset = hdmiphy_preset_148_5,
		.pixelclock = 148500000,
		.htotal = 2100, /* 1920 + 88 + 44 + 148 */
		.vtotal = 1125, /* 1080 + 4 + 5 + 36 */
		.refresh = 60,
	},
};
const int num_dynamic_clock_tries = ARRAY_SIZE(hdmi_vic_conf);

/* This function is for finding the closest frequency and giving the
 * corresponding variables back to the caller
 */
int hdmi_get_closest_timing(int pixelclock, u32 *htotal,
		u32 *vtotal, int *refresh)
{
	int i;

	if (pixelclock < 1000 || htotal == NULL ||
			vtotal == NULL || refresh == NULL) {
		pr_debug("%s<<ERR>>Invalid parameters received\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < num_dynamic_clock_tries; i++) {
		if (hdmi_vic_conf[i].pixelclock >= pixelclock) {
			memcpy(hdmiphy_dynamic_preset,
					hdmi_vic_conf[i].hdmiphy_preset, 32);
			*htotal = hdmi_vic_conf[i].htotal;
			*vtotal = hdmi_vic_conf[i].vtotal;
			*refresh = hdmi_vic_conf[i].refresh;
			return 0;
		}
	}

	return -EINVAL;
}
