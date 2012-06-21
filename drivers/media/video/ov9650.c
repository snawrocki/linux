/*
 * OV965x CMOS Image Sensor driver
 *
 * Copyright (c) 2012, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/ov9650.h>

#include "ov9650.h"

static int debug = 10;

#define CONFIG_OV965X_VGA
#define CHIP_DELAY 		0xff

#define DRIVER_NAME "OV9650"

static const char * const ov965x_supply_names[] = {
	"AVDD",	 /* Analog power supply, 2.45...2.8 VDC */
	"DVDD",	 /* 1.8 VDC +/- 10% for digital core logic */
	"DOVDD", /* Digital power supply for I/O */
};
#define OV965X_NUM_SUPPLIES ARRAY_SIZE(ov965x_supply_names)

struct ov965x_ctrls {
	struct v4l2_ctrl_handler handler;
};

enum s5k6aa_gpio_id {
	GPIO_PWDN,
	GPIO_RST,
	NUM_GPIOS,
};

struct ov965x {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	enum v4l2_mbus_type bus_type;

	struct regulator_bulk_data supplies[OV965X_NUM_SUPPLIES];
	int gpios[NUM_GPIOS];

	/* external master clock frequency */
	unsigned long mclk_frequency;

	/* protects the struct members below */
	struct mutex lock;

	/* sensor matrix scan window */
	/* struct v4l2_rect ccd_rect; */

	struct ov965x_ctrls ctrls;

	unsigned int streaming:1;
	unsigned int apply_cfg:1;
	unsigned int power;
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov965x, ctrls.handler)->sd;
}

static inline struct ov965x *to_ov965x(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov965x, sd);
}

/* static inline int ov965x_reg_write(struct i2c_client *client, unsigned int addr, */
/* 				   unsigned int value) */
/* { */
/* 	pr_info("addr: %#X, value: %#x", addr, value); */
/* 	return i2c_smbus_write_byte_data(client, addr, value); */
/* } */

static int ov965x_i2c_read(struct i2c_client *client, u8 addr, u8 *val)
{
	u8 buf = addr;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &buf
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1) {
		msg.flags = I2C_M_RD;
		ret = i2c_transfer(client->adapter, &msg, 1);

		if (ret == 1)
			*val = buf;
	}

	v4l2_dbg(3, debug, client, "i2c_read: 0x%02X : 0x%02x. ret: %d\n",
		 addr, *val, ret);

	return ret == 1 ? 0 : ret;
}

static int ov965x_i2c_write(struct i2c_client *client, u8 addr, u8 val)
{
	u8 buf[2] = { addr, val };

	int ret = i2c_master_send(client, buf, 2);

	v4l2_dbg(3, debug, client, "i2c_write: 0x%02X : 0x%02x. ret: %d\n",
		 addr, val, ret);

	return ret == 2 ? 0 : ret;
}

/* The command register write, assumes Command_Wr_addH = 0x7000. */
static int ov965x_write(struct i2c_client *c, u8 addr, u8 val)
{
	return ov965x_i2c_write(c, addr, val);
}

/* The command register read, assumes Command_Rd_addH = 0x7000. */
static int ov965x_read(struct i2c_client *client, u8 addr, u8 *val)
{
	return ov965x_i2c_read(client, addr, val);
}


struct i2c_regval {
	u8 addr;
	u8 value;
};

#if 1
struct i2c_regval ov965x_init_regs[] = {
	{REG_COM7, 0x80}, /*  SCCB reset, output format  */
	{REG_COM7, 0x80}, /*  SCCB reset, output format  */
	{REG_OFCON, 0x43}, /*  Power down register  */
	{REG_ACOM, 0x12}, /*  reserved  */
	{REG_ADC, 0x91}, /*  reserved  */
	{REG_COM5, 0x00}, /*  System clock options  */

	{REG_MVFP, 0x0c}, /*  Mirror / vflip  */
	{REG_BLUE, 0x80}, /*  blue gain  */
	{REG_RED, 0x80}, /*  red gain  */
	{REG_GAIN, 0x00}, /*  Gain lower 8 bits (rest in vref)  */
	{REG_AEC, 0xf0}, /*  More bits of AEC value  */
	{REG_COM1, 0x00}, /*  Control 1  */
	{REG_COM3, 0x04}, /*  Output byte swap, signals reassign  */
	{REG_COM4, 0x80}, /*  Vario Pixels   */

	{REG_CLKRC,0x80}, /*  Clock control, try to run at 24Mhz  */
	{REG_COM7, 0x40}, /*  SCCB reset, output format  */
	{REG_COM2, 0x01}, /*  Output drive  */

	{REG_COM9,0x2e}, /*  Control 9  - gain ceiling  */
	{REG_COM10,0x00}, /*  Slave mode, HREF vs HSYNC, signals negate  */
	{REG_HSTART, 0x1d+8}, /*  Horiz start high bits  */
	{REG_HSTOP, 0xbd+8}, /*  Horiz stop high bits  */
	{REG_HREF, 0xbf}, /*  HREF pieces  */

	{REG_VSTART, 0x00}, /*  Vert start high bits  */
	{REG_VSTOP, 0x80}, /*  Vert stop high bits  */
	{REG_VREF, 0x12}, /*  Pieces of GAIN, VSTART, VSTOP  */

	{REG_EDGE, 0xa6}, /*  Edge enhancement factor  */
	{REG_COM16, 0x02}, /*  Color matrix coeff double option  */
	{REG_COM17, 0x08}, /*  Single Frame out, Banding filter  */
	{REG_PSHFT, 0x00}, /*  Pixel delay after HREF  */
	{0x16, 0x06},
	{REG_CHLF, 0xc0}, /*  reserved  */
	{0x34,0xbf},
	{0xa8,0x80},
	{0x96,0x04},
	{REG_TSLB, 0x00}, /*  YUYV format  */
	{0x8e,0x00},
	{REG_COM12,0x77}, /*  HREF option, UV average  */

	{0x8b,0x06},
	{0x35,0x91},
	{0x94,0x88},
	{0x95,0x88},
	{REG_COM15,0xc1}, /*  Output range, RGB 555 vs 565 option  */

	{REG_GRCOM,0x3f}, /*  Analog BLC & regulator  */
	{REG_COM6,0x42|1}, /*  HREF & ADBLC options  */
	{REG_COM8,0xe5}, /*  AGC/AEC options  */
	{REG_COM13,0x90}, /*  Gamma selection, Color matrix enable, UV delay */
	{REG_HV,0x80}, /*  Manual banding filter MSB  */

	{0x5c,0x96},
	{0x5d,0x96},
	{0x5e,0x10},
	{0x59,0xeb},
	{0x5a,0x9c},
	{0x5b,0x55},
	{0x43,0xf0},
	{0x44,0x10},
	{0x45,0x55},
	{0x46,0x86},
	{0x47,0x64},
	{0x48,0x86},
	{0x5f,0xe0},
	{0x60,0x8c},
	{0x61,0x20},
	{0xa5,0xd9},
	{0xa4,0x74},
	{REG_COM23,0x02}, /*  Color bar test,color gain  */
	{REG_COM8,(1<<5)}, /*  AGC/AEC options  */
	{0x4f,0x3a},
	//0xe7
	{0x50,0x3d},
	{0x51,0x03},
	{0x52,0x12},
	{0x53,0x26},
	{0x54,0x38},
	{0x55,0x40},
	{0x56,0x40},
	{0x57,0x40},
	{0x58,0x0d},
	{REG_COM22,0x23}, /*  Edge enhancement, denoising  */
	{REG_COM14,0x02}, /*  Edge enhancement options  */
	{0xa9,0xb8},
	{0xaa,0x92},
	{0xab,0x0a},
	{REG_DBLC1,0xdf}, /*  Digital BLC  */
	{REG_DBLC_B,0x00}, /*  Digital BLC B chan offset  */
	{REG_DBLC_R,0x00}, /*  Digital BLC R chan offset  */
	{REG_DBLC_GB,0x00}, /*  Digital BLC GB chan offset  */
	{REG_TSLB, 0x00}, /*  YUYV format  */
	{REG_AEW,0x70}, /*  AGC upper limit  */
	{REG_AEB,0x64}, /*  AGC lower limit  */
	{REG_VPT,0xc3}, /*  AGC/AEC fast mode op region  */
	{REG_EXHCH,0x00}, /*  Dummy pixel insert MSB  */
	{REG_EXHCL,0x00}, /*  Dummy pixel insert LSB  */
	{REG_COM11,(1<<7)|(3<<5)}, /*  Night mode, banding filter enable  */
	//0x2a,0x12 0x19
	{REG_GSP,0x40}, /*  Gamma 1  */
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},

	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
	{0x7c,0x04},
	{0x7d,0x07},
	{0x7e,0x10},
	{0x7f,0x28},
	{0x80,0x36},
	{0x81,0x44},
	{0x82,0x52},
	{0x83,0x60},
	{0x84,0x6c},
	{0x85,0x78},
	{0x86,0x8c},
	{0x87,0x9e},
	{0x88,0xbb},
	{0x89,0xd2},
	{0x8a,0xe6},
	{REG_MBD,0x41}, /*  Manual banding filter value, when COM11[0] is high  */
	{REG_LCC5,0x00}, /*  Lens Correction Option 5  */
	{REG_COM14,0x00}, /*  Edge enhancement options  */
	{REG_EDGE,0xa4}/*  Edge enhancement factor  */
};
#endif

#if 0
/* #if defined(CONFIG_OV965X_VGA) */
struct i2c_regval ov965x_init_regs[] = {
	{0x12, 0x80},	/* Camera Soft reset. Self cleared after reset. */
	{CHIP_DELAY, 10},
	//{0x11,0x80},
	{0x6a,0x3e},
	{0x3b,0x09},//09
	{0x13,0x8f},//e0//8f
	{0x01,0x80},
	{0x02,0x80},
	{0x00,0x00},
	{0x10,0x00},
	{0x35,0x91},
	{0x0e,0xa0},
	{0x1e,0x34},//14
	{0xA8,0x80},
	//////////////VGA//////////
	{0x04,0x00},
	{0x0c,0x04},
	{0x0d,0x80},
	{0x11,0x81},
	{0x12,0x40},
	{0x37,0x91},
	{0x38,0x12},
	{0x39,0x43},
	/////////////END///Square_Chiu
	{0x18,0xc6},
	{0x17,0x26},
	{0x32,0xad},
	{0x03,0x00},
	{0x1a,0x3d},
	{0x19,0x01},
	{0x3f,0xa6},
	{0x14,0x2e},
	{0x15,0x10},
	{0x41,0x02},
	{0x42,0x08},
	{0x1b,0x00},
	{0x16,0x06},
	{0x33,0xe2},
	{0x34,0xbf},
	{0x96,0x04},
	{0x3a,0x00},
	{0x8e,0x00},
	{0x3c,0x77},
	{0x8B,0x06},
	{0x94,0x88},
	{0x95,0x88},
	{0x40,0xc1},
	{0x29,0x3f},
	{0x0f,0x42},
	{0x3d,0x92},
	{0x69,0x40},
	{0x5C,0xb9},
	{0x5D,0x96},
	{0x5E,0x10},
	{0x59,0xc0},
	{0x5A,0xaf},
	{0x5B,0x55},
	{0x43,0xf0},
	{0x44,0x10},
	{0x45,0x68},
	{0x46,0x96},
	{0x47,0x60},
	{0x48,0x80},
	{0x5F,0xe0},
	{0x60,0x8c},
	{0x61,0x20},
	{0xa5,0xd9},
	{0xa4,0x74},
	{0x8d,0x02},
	//{0x13,0xe7},
	{0x4f,0x3a},
	{0x50,0x3d},
	{0x51,0x03},
	{0x52,0x12},
	{0x53,0x26},
	{0x54,0x36},
	{0x55,0x45},
	{0x56,0x40},
	{0x57,0x40},
	{0x58,0x0d},
	{0x8C,0x23},
	{0x3E,0x02},
	{0xa9,0xb8},
	{0xaa,0x92},
	{0xab,0x0a},
	{0x8f,0xdf},
	{0x90,0x00},
	{0x91,0x00},
	{0x9f,0x00},
	{0xa0,0x00},
	{0x3A,0x01},
	{0x24,0x70},
	{0x25,0x64},
	{0x26,0xc3},
	{0x2a,0x00},
	{0x2b,0x00},
	{0x6c,0x40},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},
	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
	{0x7c,0x04},
	{0x7d,0x07},
	{0x7e,0x10},
	{0x7f,0x28},
	{0x80,0x36},
	{0x81,0x44},
	{0x82,0x52},
	{0x83,0x60},
	{0x84,0x6c},
	{0x85,0x78},
	{0x86,0x8c},
	{0x87,0x9e},
	{0x88,0xbb},
	{0x89,0xd2},
	{0x8a,0xe6},
	//{0x15, 0x12},	// PCLK reverse

};
#endif

/*
#elif defined(CONFIG_OV965X_QVGA)
*/
#if 0
struct i2c_regval ov965x_init_regs[] = {
	{0x12, 0x80},	// Camera Soft reset. Self cleared after reset.
	{CHIP_DELAY, 10},
	{0x11,0x80},
	{0x6a,0x3e},
	{0x3b,0x09},//09
	{0x13,0x8f},//e0//8f
	{0x01,0x80},
	{0x02,0x80},
	{0x00,0x00},
	{0x10,0x00},
	{0x39,0x43},
	{0x38,0x12},
	{0x37,0x00},
	{0x35,0x91},
	{0x0e,0xa0},
	{0x1e,0x14},
	{0xA8,0x80},
	{0x12, 0x10}, /* QVGA */
	{0x04,0x00},
	{0x0c,0x04},
	{0x0d,0x80},
	{0x18,0xc6},
	{0x17,0x26},
	{0x32,0xad},
	{0x03,0x00},
	{0x1a,0x3d},
	{0x19,0x01},
	{0x3f,0xa6},
	{0x14,0x2e},
	{0x15,0x10},
	{0x41,0x02},
	{0x42,0x08},
	{0x1b,0x00},
	{0x16,0x06},
	{0x33,0xe2},
	{0x34,0xbf},
	{0x96,0x04},
	{0x3a,0x00},
	{0x8e,0x00},
	{0x3c,0x77},
	{0x8B,0x06},
	{0x94,0x88},
	{0x95,0x88},
	{0x40,0xc1},
	{0x29,0x3f},
	{0x0f,0x42},
	{0x3d,0x92},
	{0x69,0x40},
	{0x5C,0xb9},
	{0x5D,0x96},
	{0x5E,0x10},
	{0x59,0xc0},
	{0x5A,0xaf},
	{0x5B,0x55},
	{0x43,0xf0},
	{0x44,0x10},
	{0x45,0x68},
	{0x46,0x96},
	{0x47,0x60},
	{0x48,0x80},
	{0x5F,0xe0},
	{0x60,0x8c},
	{0x61,0x20},
	{0xa5,0xd9},
	{0xa4,0x74},
	{0x8d,0x02},
	//{0x13,0xe7},
	{0x4f,0x3a},
	{0x50,0x3d},
	{0x51,0x03},
	{0x52,0x12},
	{0x53,0x26},
	{0x54,0x36},
	{0x55,0x45},
	{0x56,0x40},
	{0x57,0x40},
	{0x58,0x0d},
	{0x8C,0x23},
	{0x3E,0x02},
	{0xa9,0xb8},
	{0xaa,0x92},
	{0xab,0x0a},
	{0x8f,0xdf},
	{0x90,0x00},
	{0x91,0x00},
	{0x9f,0x00},
	{0xa0,0x00},
	{0x3A,0x01},
	{0x24,0x70},
	{0x25,0x64},
	{0x26,0xc3},
	{0x2a,0x00},
	{0x2b,0x00},
	{0x6c,0x40},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},
	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
	{0x7c,0x04},
	{0x7d,0x07},
	{0x7e,0x10},
	{0x7f,0x28},
	{0x80,0x36},
	{0x81,0x44},
	{0x82,0x52},
	{0x83,0x60},
	{0x84,0x6c},
	{0x85,0x78},
	{0x86,0x8c},
	{0x87,0x9e},
	{0x88,0xbb},
	{0x89,0xd2},
	{0x8a,0xe6},
	//{0x15, 0x12},	// PCLK reverse
};
#endif

/*
#elif defined(CONFIG_OV965X_SVGA)
struct i2c_regval ov965x_init_reg[] =
{
	// Only for VGA Mode
	{0x25,0x64},
	{0x26,0xc3},
	{0x2a,0x00},
	{0x2b,0x00},
	{0x6c,0x40},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},
	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
};
#elif defined(CONFIG_OV965X_SXGA)
struct i2c_regval ov965x_init_reg[] = {
	{0x25,0x64},
	{0x26,0xc3},
	{0x2a,0x00},
	{0x2b,0x00},
	{0x6c,0x40},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},
	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
};
#endif */

/*
static struct s3c_fimc_camera ov965x_data = {
	.id 		= CONFIG_VIDEO_FIMC_CAM_CH,
	.type		= CAM_TYPE_ITU,
	.mode		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,//YCRYCB,
	.clockrate	= 24000000,
	.width		= 640,//800,
	.height		= 480,//600,

	.polarity	= {
		.pclk	= 0,
		.vsync	= 1,
		.href	= 0,
		.hsync	= 0,
	},
};*/

struct ov965x_pixfmt {
	enum v4l2_mbus_pixelcode code;
	u32 colorspace;
};

static const struct ov965x_pixfmt ov965x_formats[] = {
	{ V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG },
};

static void ov965x_try_format(struct ov965x *ov965x,
			      struct v4l2_mbus_framefmt *mf)
{
	unsigned int index;

	/* v4l_bound_align_image(&mf->width, OV965X_WIN_WIDTH_MIN, */
	/* 		      OV965X_WIN_WIDTH_MAX, 1, */
	/* 		      &mf->height, OV965X_WIN_HEIGHT_MIN, */
	/* 		      OV965X_WIN_HEIGHT_MAX, 1, 0); */

	mf->width = 640;
	mf->height = 480;

	index = 0; /* ov965x_get_pixfmt_index(ov965x, mf); */

	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	mf->code	= ov965x_formats[index].code;
	mf->field	= V4L2_FIELD_NONE;
}

static int ov965x_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov965x *ov965x = to_ov965x(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, 0);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = ov965x->format;

	return 0;
}

static int ov965x_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov965x *ov965x = to_ov965x(sd);
	struct v4l2_mbus_framefmt *mf = NULL;
	int ret = 0;

	ov965x_try_format(ov965x, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		if (fh != NULL) {
			mf = v4l2_subdev_get_try_format(fh, fmt->pad);
			*mf = fmt->format;
		}
	} else {
		if (ov965x->streaming) {
			ret = -EBUSY;
		} else {
			ov965x->format = fmt->format;
			ov965x->apply_cfg = 1;
		}
	}

	return ret;
}

static int ov965x_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov965x *ov965x = to_ov965x(sd);

	int ret = 0, i;

	if (!on || ov965x->streaming)
		return 0;

	pr_info("on: %d\n", on);

	/* ret = i2c_smbus_write_byte_data(client, 0x12, 0x80); */
	/* msleep(20); */
	/* pr_info("ret= %d\n", ret); */

	/* ret = i2c_smbus_write_byte_data(client, 0x11, 0x80); */
	/* msleep(20); */
	/* pr_info("ret= %d\n", ret); */

	for (i = 0; i < ARRAY_SIZE(ov965x_init_regs) && !ret; i++) {
		if (ov965x_init_regs[i].addr == 0xff) {
			msleep(ov965x_init_regs[i].value);
			continue;
		}
		//pr_info("\n");
		ret = ov965x_write(client, ov965x_init_regs[i].addr,
				   ov965x_init_regs[i].value);
	}
	ov965x->streaming = 1;

	return ret;
}

static int ov965x_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format = v4l2_subdev_get_try_format(fh, 0);

	v4l2_info(sd, "%s:%d\n", __func__, __LINE__);

	format->colorspace = ov965x_formats[0].colorspace;
	format->code = ov965x_formats[0].code;
	format->width = 640;
	format->height = 480;
	format->field = V4L2_FIELD_NONE;

	return 0;
}

static void ov965x_gpio_set(int gpio, int val)
{
	if (gpio_is_valid(gpio)) {
		gpio_set_value(gpio, val);
		pr_info("set gpio %d to %d\n", gpio, val);
	}
}

static int ov965x_set_power(struct v4l2_subdev *sd, int on)
{
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */
	struct ov965x *ov965x = to_ov965x(sd);

	pr_info("on: %d\n", on);

	if (on) {
		/* int i, ret = 0; */
		/* u8 value; */

		ov965x_gpio_set(ov965x->gpios[GPIO_PWDN], 0);
		ov965x_gpio_set(ov965x->gpios[GPIO_RST], 0);
#if 0
		for (i = 0; i < 0xff && ret == 0; i++) {
			ret = ov965x_i2c_read(client, i, &value);
			pr_info("%02X : %02x\n", i, value);
		}
#endif
	} else {
		ov965x_gpio_set(ov965x->gpios[GPIO_RST], 1);
		ov965x_gpio_set(ov965x->gpios[GPIO_PWDN], 1);
		ov965x->streaming = 0;
	}
	msleep(25);

	return 0;
}

static const struct v4l2_subdev_pad_ops ov965x_pad_ops = {
	/* .enum_mbus_code = ov965x_enum_mbus_code, */
	/* .enum_frame_size = ov965x_enum_frame_size, */
	.get_fmt = ov965x_get_fmt,
	.set_fmt = ov965x_set_fmt,
};

static const struct v4l2_subdev_video_ops ov965x_video_ops = {
	.s_stream = ov965x_s_stream,
};

static const struct v4l2_subdev_internal_ops ov965x_sd_internal_ops = {
	.open = ov965x_open,
};

static const struct v4l2_subdev_core_ops ov965x_core_ops = {
	.s_power = ov965x_set_power,
};

static const struct v4l2_subdev_ops ov965x_subdev_ops = {
	.core = &ov965x_core_ops,
	.pad = &ov965x_pad_ops,
	.video = &ov965x_video_ops,
};

static int ov965x_initialize_ctrls(struct ov965x *ov965x)
{
	return 0;
}

/*
 * GPIO setup
 */
static void ov965x_free_gpios(struct ov965x *ov965x)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov965x->gpios); i++) {
		if (!gpio_is_valid(ov965x->gpios[i]))
			continue;
		gpio_free(ov965x->gpios[i]);
		ov965x->gpios[i] = -EINVAL;
	}
}

static int ov965x_configure_gpios(struct ov965x *ov965x,
				  const struct ov9650_platform_data *pdata)
{
	int ret, i;

	ov965x->gpios[GPIO_PWDN] = pdata->gpio_pwdn;
	ov965x->gpios[GPIO_RST]  = pdata->gpio_reset;

	for (i = 0; i < ARRAY_SIZE(ov965x->gpios); i++) {
		int gpio = ov965x->gpios[i];
		if (!gpio_is_valid(gpio))
			continue;

		ret = gpio_request_one(gpio, GPIOF_OUT_INIT_HIGH, "OV965X");
		if (ret < 0)
			goto error;

		if (gpio_is_valid(gpio)) {
			pr_info("set gpio %d to 1\n", gpio);
			gpio_set_value(gpio, 1);
			gpio_export(gpio, 0);
		}
		ov965x->gpios[i] = gpio;
	}
	return 0;
error:
	ov965x_free_gpios(ov965x);
	return ret;
}

static int ov965x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct ov9650_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	struct ov965x *ov965x;
	int ret;

	if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}

	if (pdata->mclk_frequency == 0) {
		dev_err(&client->dev, "MCLK frequency not specified\n");
		return -EINVAL;
	}

	ov965x = devm_kzalloc(&client->dev, sizeof(*ov965x), GFP_KERNEL);
	if (!ov965x)
		return -ENOMEM;

	mutex_init(&ov965x->lock);

	ov965x->mclk_frequency = pdata->mclk_frequency;

	sd = &ov965x->sd;
	v4l2_i2c_subdev_init(sd, client, &ov965x_subdev_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));

	sd->internal_ops = &ov965x_sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ov965x->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ov965x->pad, 0);
	if (ret < 0)
		return ret;

	ret = ov965x_configure_gpios(ov965x, pdata);
	if (ret)
		goto err_me;

#if 0
	for (i = 0; i < OV965X_NUM_SUPPLIES; i++)
		ov965x->supplies[i].supply = ov965x_supply_names[i];

	ret = regulator_bulk_get(&client->dev, OV965X_NUM_SUPPLIES,
				 ov965x->supplies);
	if (ret) {
		dev_err(&client->dev, "Failed to get regulators\n");
		goto out_gpio;
	}
#endif
	ret = ov965x_initialize_ctrls(ov965x);
	if (ret)
		goto err_gpio;

	ov965x->format.width = 640;
	ov965x->format.height = 480;

	/* j = 50; */
	/* while (j-- > 0) { */
	/* 	ov965x_set_power(sd, j & 1); */
	/* 	msleep(1000); */
	/* } */

	ov965x_set_power(sd, 1);
	msleep(300);
	/* ov965x_set_power(sd, 0); */
	/* msleep(1000); */
	/* ov965x_set_power(sd, 1); */
	/* msleep(1000); */

	/* j = 50; */
	/* while (j-- > 0) { */
		ret = ov965x_write(client, 0x01, 0x83);
		pr_info_ratelimited("I2C write: ret= %d\n", ret);
	/* 	msleep_interruptible(20); */
	/* } */

	u8 value;
	ret = ov965x_read(client, 0x01, &value);
	pr_info("I2C read: ret: %d, value: %#x\n", ret, value);

	ov965x_set_power(sd, 0);

	/* ov965x->ccd_rect.width = OV965X_WIN_WIDTH_MAX; */
	/* ov965x->ccd_rect.height	= OV965X_WIN_HEIGHT_MAX; */
	/* ov965x->ccd_rect.left = 0; */
	/* ov965x->ccd_rect.top = 0; */

	return 0;

/* err_reg: */
	/* regulator_bulk_free(OV965X_NUM_SUPPLIES, ov965x->supplies); */
err_gpio:
	ov965x_free_gpios(ov965x);
err_me:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int ov965x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov965x *ov965x = to_ov965x(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);

	ov965x_free_gpios(ov965x);
	return 0;
}

static const struct i2c_device_id ov965x_id[] = {
	{ "OV9650", 0 },
	{ "OV9652", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c,ov965x_id);

static struct i2c_driver ov965x_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= ov965x_probe,
	.remove		= ov965x_remove,
	.id_table	= ov965x_id,
};

module_i2c_driver(ov965x_i2c_driver);

MODULE_AUTHOR("Sylwester Nawrocki <sylvester.nawrocki@gmail.com>");
MODULE_DESCRIPTION("OV9650/OV9652 CMOS Image Sensors driver");
MODULE_LICENSE("GPL");

