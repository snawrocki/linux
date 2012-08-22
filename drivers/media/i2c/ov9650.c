/*
 * OV9650/OV9652 CMOS Image Sensor driver
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
#include <linux/slab.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/ov9650.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define DRIVER_NAME		"OV9650"
#define CHIP_DELAY 		0xff

/*
 * OV9650 register definitions
 */
#define OV9650_MANUFACT_ID	0x7fa2
#define OV9650_PRODUCT_ID	0x9650

#define REG_GAIN		0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE		0x01	/* blue gain */
#define REG_RED			0x02	/* red gain */
#define REG_VREF		0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1		0x04	/* Control 1 */
#define  COM1_CCIR656		0x40	/* CCIR656 enable */
#define REG_BAVE		0x05	/* U/B Average level */
#define REG_GbAVE		0x06	/* Y/Gb Average level */
#define REG_GrAVE		0x07	/* Cr Average Level */
#define REG_RAVE		0x08	/* V/R Average level */
#define REG_COM2		0x09	/* Control 2 */
#define  COM2_SSLEEP		0x10	/* Soft sleep mode */
#define REG_PID			0x0a	/* Product ID MSB */
#define REG_VER			0x0b	/* Product ID LSB */
#define REG_COM3		0x0c	/* Output byte swap, signals reassign */
#define  COM3_SWAP		0x40	/* Byte swap */
#define  COM3_VARIOPIXEL1	0x04
#define REG_COM4		0x0d	/* Vario Pixels  */
#define  COM4_VARIOPIXEL2	0x80
#define REG_COM5		0x0e	/* System clock options */
#define  COM5_SLAVE_MODE	0x10
#define  COM5_SYSTEMCLOCK48MHZ	0x80
#define REG_COM6		0x0f	/* HREF & ADBLC options */
#define REG_AEC			0x10	/* More bits of AEC value */
#define REG_CLKRC		0x11	/* Clocl control */
#define  CLK_EXT		0x40	/* Use external clock directly */
#define  CLK_SCALE		0x3f	/* Mask for internal clock scale */
#define REG_COM7		0x12	/* SCCB reset, output format */
#define  COM7_RESET		0x80	/* Register reset */
#define  COM7_FMT_MASK		0x38
#define  COM7_FMT_VGA		0x40
#define	 COM7_FMT_CIF		0x20	/* CIF format */
#define  COM7_FMT_QVGA		0x10	/* QVGA format */
#define  COM7_FMT_QCIF		0x08	/* QCIF format */
#define	 COM7_RGB		0x04	/* bits 0 and 2 - RGB format */
#define	 COM7_YUV		0x00	/* YUV */
#define	 COM7_BAYER		0x01	/* Bayer format */
#define	 COM7_PBAYER		0x05	/* "Processed bayer" */
#define REG_COM8		0x13	/* AGC/AEC options */
#define  COM8_FASTAEC		0x80	/* Enable fast AGC/AEC */
#define  COM8_AECSTEP		0x40	/* Unlimited AEC step size */
#define  COM8_BFILT		0x20	/* Band filter enable */
#define  COM8_AGC		0x04	/* Auto gain enable */
#define  COM8_AWB		0x02	/* White balance enable */
#define  COM8_AEC		0x01	/* Auto exposure enable */
#define REG_COM9		0x14	/* Control 9  - gain ceiling */
#define REG_COM10		0x15	/* Slave mode, HREF vs HSYNC, signals negate */
#define  COM10_HSYNC		0x40	/* HSYNC instead of HREF */
#define  COM10_PCLK_HB		0x20	/* Suppress PCLK on horiz blank */
#define  COM10_HREF_REV		0x08	/* Reverse HREF */
#define  COM10_VS_LEAD		0x04	/* VSYNC on clock leading edge */
#define  COM10_VS_NEG		0x02	/* VSYNC negative */
#define  COM10_HS_NEG		0x01	/* HSYNC negative */
#define REG_HSTART		0x17	/* Horiz start high bits */
#define REG_HSTOP		0x18	/* Horiz stop high bits */
#define REG_VSTART		0x19	/* Vert start high bits */
#define REG_VSTOP		0x1a	/* Vert stop high bits */
#define REG_PSHFT		0x1b	/* Pixel delay after HREF */
#define REG_MIDH		0x1c	/* Manuf. ID high */
#define REG_MIDL		0x1d	/* Manuf. ID low */
#define REG_MVFP		0x1e	/* Mirror / vflip */
#define  MVFP_MIRROR		0x20	/* Mirror image */
#define  MVFP_FLIP		0x10	/* Vertical flip */
#define REG_BOS			0x20	/* B channel Offset  */
#define REG_GBOS		0x21	/* Gb channel Offset  */
#define REG_GROS		0x22	/* Gr channel Offset  */
#define REG_ROS			0x23	/* R channel Offset  */
#define REG_AEW			0x24	/* AGC upper limit */
#define REG_AEB			0x25	/* AGC lower limit */
#define REG_VPT			0x26	/* AGC/AEC fast mode op region */
#define REG_BBIAS		0x27	/* B channel output bias */
#define REG_GBBIAS		0x28	/* Gb channel output bias */
#define REG_GRCOM		0x29	/* Analog BLC & regulator */
#define REG_EXHCH		0x2A	/* Dummy pixel insert MSB */
#define REG_EXHCL		0x2B	/* Dummy pixel insert LSB */
#define REG_RBIAS		0x2C	/* R channel output bias */
#define REG_ADVFL		0x2D	/* LSB of dummy pixel insert */
#define REG_ADVFH		0x2E	/* MSB of dummy pixel insert */
#define REG_YAVE		0x2F	/* Y/G channel average value */
#define REG_HSYST		0x30	/* HSYNC rising edge delay LSB*/
#define REG_HSYEN		0x31	/* HSYNC falling edge delay LSB*/
#define REG_HREF		0x32	/* HREF pieces */
#define REG_CHLF		0x33	/* reserved */
#define REG_ADC			0x37	/* reserved */
#define REG_ACOM		0x38	/* reserved */
#define REG_OFCON		0x39	/* Power down register */
#define  OFCON_PWRDN		0x08	/* Power down bit */
#define REG_TSLB		0x3a	/* YUVU format */
#define  TSLB_YUYV_MASK		0x0c	/* UYVY or VYUY - see com13 */
#define REG_COM11		0x3b	/* Night mode, banding filter enable */
#define  COM11_NIGHT		0x80	/* NIght mode enable */
#define  COM11_NMFR		0x60	/* Two bit NM frame rate */
#define  COM11_BANDING		0x01	/* banding filter */
#define REG_COM12		0x3c	/* HREF option, UV average */
#define  COM12_HREF		0x80	/* HREF always */
#define REG_COM13		0x3d	/* Gamma selection, Color matrix enable, UV delay*/
#define  COM13_GAMMA		0x80	/* Gamma enable */
#define	 COM13_UVSAT		0x40	/* UV saturation auto adjustment */
#define  COM13_UVSWAP		0x01	/* V before U - w/TSLB */
#define REG_COM14		0x3e	/* Edge enhancement options */
#define REG_EDGE		0x3f	/* Edge enhancement factor */
#define REG_COM15		0x40	/* Output range, RGB 555 vs 565 option */
#define  COM15_R10F0		0x00	/* Data range 10 to F0 */
#define	 COM15_R01FE		0x80	/* 01 to FE */
#define  COM15_R00FF		0xc0	/* 00 to FF */
#define  COM15_RGB565		0x10	/* RGB565 output */
#define  COM15_RGB555		0x30	/* RGB555 output */
#define  COM15_SWAPRB		0x04	/* Swap R&B */
#define REG_COM16		0x41	/* Color matrix coeff double option */
#define  COM16_AWBGAIN		0x08	/* AWB gain enable */
#define REG_COM17		0x42	/* Single Frame out, Banding filter */
/* n = 1...9, 0x4f..0x57 */
#define	REG_MTX(__n)		(0x4f + (__n) - 1)
#define REG_MTXS		0x58
/* Lens Correction Option 1...5, __n = 0...5 */
#define REG_LCC(__n)		(0x62 + (__n) - 1)
#define  LCC5_LCC_ENABLE	0x01	/* LCC5, enable lens correction */
#define  LCC5_LCC_COLOR		0x04
#define REG_MANU		0x67	/* Manual U value */
#define REG_MANV		0x68	/* Manual V value */
#define REG_HV			0x69	/* Manual banding filter MSB */
#define REG_MBD			0x6a	/* Manual banding filter value */
#define REG_GSP			0x6c	/* Gamma curve */
#define  GSP_LEN		15
#define REG_GST			0x7c	/* Gamma curve */
#define  GST_LEN		15
#define REG_COM22		0x8c	/* Edge enhancement, denoising */
#define  COM22_WHTPCOR		0x02	/* White pixel correction enable */
#define  COM22_WHTPCOROPT	0x01	/* White pixel correction option */
#define  COM22_DENOISE		0x10	/* White pixel correction option */
#define REG_COM23		0x8d	/* Color bar test,color gain */
#define REG_DBLC1		0x8f	/* Digital BLC */
#define REG_DBLC_B		0x90	/* Digital BLC B chan offset */
#define REG_DBLC_R		0x91	/* Digital BLC R chan offset */
#define REG_DM_LNL		0x92	/* Dummy line low 8 bits */
#define REG_DM_LNH		0x93	/* Dummy line high 8 bits */
#define REG_LCCFB		0x9d	/* Lens Correction B chan */
#define REG_LCCFR		0x9e	/* Lens Correction R chan */
#define REG_DBLC_GB		0x9f	/* Digital BLC GB chan offset */
#define REG_DBLC_GR		0xa0	/* Digital BLC GR chan offset */
#define REG_AECHM		0xa1	/* Exposure Value - AEC MSB 6 bits */
#define REG_BD50ST		0xa2	/* Banding Filter Value 50Hz */
#define REG_BD60ST		0xa3	/* Banding Filter Value 60Hz */

struct ov965x_ctrls {
	struct v4l2_ctrl_handler handler;
};

enum gpio_id {
	GPIO_PWDN,
	GPIO_RST,
	NUM_GPIOS,
};

struct ov965x {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	enum v4l2_mbus_type bus_type;
	int gpios[NUM_GPIOS];
	/* external master clock frequency */
	unsigned long mclk_frequency;

	/* protects the struct members below */
	struct mutex lock;

	/* sensor matrix scan window */
	/* struct v4l2_rect ccd_rect; */
	u8 reg_com7;

	struct ov965x_ctrls ctrls;

	unsigned int streaming:1;
	unsigned int apply_fmt:1;
	unsigned int power;
};

struct i2c_regval {
	u8 addr;
	u8 value;
};

struct i2c_regval ov965x_init_regs[] = {
	//{ REG_COM7, 0x80 },	/* SCCB reset, output format  */
	//{ CHIP_DELAY, 20 },
	{ REG_COM2, 0x10 },	/* Set soft sleep mode */
	{ REG_OFCON, 0x43 },	/* Power down register  */
	{ REG_ACOM, 0x12 },	/* reserved  */
	{ REG_ADC, 0x91 },	/* reserved  */
	{ REG_COM5, 0x00 },	/* System clock options  */

	{ REG_MVFP, 0x0c },	/* Mirror / vflip  */
	{ REG_BLUE, 0x80 },	/* blue gain  */
	{ REG_RED, 0x80 },	/* red gain  */
	{ REG_GAIN, 0x00 },	/* Gain lower 8 bits (rest in vref)  */
	{ REG_AEC, 0xf0 },	/* More bits of AEC value  */
	{ REG_COM1, 0x00 },	/* Control 1  */
	{ REG_COM3, 0x04 },	/* Output byte swap, signals reassign  */
	{ REG_COM4, 0x80 },	/* Vario Pixels   */

	{ REG_CLKRC, 0x83 },	/* Clock control, try to run at 24Mhz  */

	{ REG_COM7, 0x04 },	/* SCCB reset, output format  */
	{ REG_COM2, 0x01 },	/* Output drive, soft sleep mode */

	{ REG_COM9, 0x2e },	/*  Control 9  - gain ceiling  */
	{ REG_COM10, 0x00 },	/* Slave mode, HREF vs HSYNC, signals negate */
	{ REG_HSTART, 0x1d + 8 }, /*  Horiz start high bits  */
	{ REG_HSTOP, 0xbd + 8 }, /*  Horiz stop high bits  */
	{ REG_HREF, 0xbf },	/* HREF pieces  */

	{ REG_VSTART, 0x00 },	/* Vert start high bits  */
	{ REG_VSTOP, 0x80 },	/* Vert stop high bits  */
	{ REG_VREF, 0x12 },	/* Pieces of GAIN, VSTART, VSTOP  */

	{ REG_EDGE, 0xa6 },	/* Edge enhancement factor  */
	{ REG_COM16, 0x02 },	/* Color matrix coeff double option  */
	{ REG_COM17, 0x08 },	/* Single Frame out, Banding filter  */
	{ REG_PSHFT, 0x00 },	/* Pixel delay after HREF  */
	{ 0x16, 0x06 },
	{ REG_CHLF, 0xc0 },	/* reserved  */
	{ 0x34, 0xbf },
	{ 0xa8, 0x80 },
	{ 0x96, 0x04 },
	{ REG_TSLB, 0x00 },	/* YUYV format  */
	{ 0x8e, 0x00 },
	{ REG_COM12, 0x77 },	/* HREF option, UV average  */

	{ 0x8b, 0x06 },
	{ 0x35, 0x91 },
	{ 0x94, 0x88 },
	{ 0x95, 0x88 },
	{ REG_COM15, 0xc1 },	/* Output range, RGB 555 vs 565 option */

	{ REG_GRCOM, 0x3f },	/* Analog BLC & regulator */
	{ REG_COM6, 0x42 | 1 }, /* HREF & ADBLC options */
	{ REG_COM8, 0xe5 }, 	/* AGC/AEC options */
	{ REG_COM13, 0x90 },	/* Gamma selection, Color matrix enable,
				   UV delay */
	{ REG_HV, 0x80 },	/*  Manual banding filter MSB  */
	{ 0x5c, 0x96 },
	{ 0x5d, 0x96 },
	{ 0x5e, 0x10 },
	{ 0x59, 0xeb },
	{ 0x5a, 0x9c },
	{ 0x5b, 0x55 },
	{ 0x43, 0xf0 },
	{ 0x44, 0x10 },
	{ 0x45, 0x55 },
	{ 0x46, 0x86 },
	{ 0x47, 0x64 },
	{ 0x48, 0x86 },
	{ 0x5f, 0xe0 },
	{ 0x60, 0x8c },
	{ 0x61, 0x20 },
	{ 0xa5, 0xd9 },
	{ 0xa4, 0x74 },

	{ REG_COM23, 0x02 }, /*  Color bar test, color gain  */
	{ REG_COM8, (1 << 5) | 0x7 }, /*  AGC/AEC options  */

	{ REG_COM22, 0x23 }, /*  Edge enhancement, denoising  */
	{ REG_COM14, 0x02 }, /*  Edge enhancement options  */
	{ 0xa9, 0xb8 },
	{ 0xaa, 0x92 },
	{ 0xab, 0x0a },
	{ REG_DBLC1, 0xdf }, /*  Digital BLC  */
	{ REG_DBLC_B, 0x00 }, /*  Digital BLC B chan offset  */
	{ REG_DBLC_R, 0x00 }, /*  Digital BLC R chan offset  */
	{ REG_DBLC_GB, 0x00 }, /*  Digital BLC GB chan offset  */
	{ REG_TSLB, 0x00 }, /*  YUYV format  */
	{ REG_AEW, 0x70 }, /*  AGC upper limit  */
	{ REG_AEB, 0x64 }, /*  AGC lower limit  */
	{ REG_VPT, 0xc3 }, /*  AGC/AEC fast mode op region  */
	{ REG_EXHCH, 0x00 }, /*  Dummy pixel insert MSB  */
	{ REG_EXHCL, 0x00 }, /*  Dummy pixel insert LSB  */
	{ REG_COM11, (1 << 7) | (3 << 5)}, /*  Night mode, banding filter enable  */

	{ REG_MBD, 0x41 },	/* Manual banding filter value, when COM11[0] is high  */
	{ REG_LCC(5), 0x00 },	/* Lens Correction Option 5  */
	{ REG_COM14, 0x00 },	/* Edge enhancement options  */
	{ REG_EDGE, 0xa4 }	/* Edge enhancement factor  */
};

struct ov965x_framesize {
	u16 width;
	u16 height;
	u8 reg_com7;
};

static const struct ov965x_framesize ov965x_framesizes[] = {
	{ 640, 480, 0x40 }, /* VGA */
	{ 320, 240, 0x10 }, /* QVGA */
	{ 352, 288, 0x20 }, /* CIF */
	{ 176, 144, 0x08 }, /* QCIF */
};

struct ov965x_pixfmt {
	enum v4l2_mbus_pixelcode code;
	u32 colorspace;
};

static const struct ov965x_pixfmt ov965x_formats[] = {
	{ V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG },
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov965x, ctrls.handler)->sd;
}

static inline struct ov965x *to_ov965x(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov965x, sd);
}

static int ov965x_read(struct i2c_client *client, u8 addr, u8 *val)
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

static int ov965x_write(struct i2c_client *client, u8 addr, u8 val)
{
	u8 buf[2] = { addr, val };

	int ret = i2c_master_send(client, buf, 2);

	v4l2_dbg(3, debug, client, "i2c_write: 0x%02X : 0x%02x. ret: %d\n",
		 addr, val, ret);

	return ret == 2 ? 0 : ret;
}

#if 0
static int ov965x_write_array(struct ov965x *ov965x,
			      const struct i2c_regval *regs,
			      unsigned int len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov965x->sd);
	int ret, i;

	for (i = 0; i < len && !ret; i++)
		ret = ov965x_write(client, regs[i].addr, regs[i].value);

	return ret;
}
#endif

static int ov965x_set_gamma_curve(struct ov965x *ov965x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov965x->sd);
	static const u8 gamma_curve[] = {
		0x40, 0x30, 0x4b, 0x60, 0x70, 0x70, 0x70, 0x70,
		0x60, 0x60, 0x50, 0x48, 0x3a, 0x2e, 0x28, 0x22,
		0x04, 0x07, 0x10, 0x28,	0x36, 0x44, 0x52, 0x60,
		0x6c, 0x78, 0x8c, 0x9e, 0xbb, 0xd2, 0xe6
	};
	u8 addr = REG_GSP;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(gamma_curve) && !ret; i++) {
		ret = ov965x_write(client, addr, gamma_curve[i]);
		addr++;
	}

	return ret;
};

static int ov965x_set_color_matrix(struct ov965x *ov965x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov965x->sd);
	static const u8 mtx[] = {
		/* MTX1..MTX9, MTXS */
		0x3a, 0x3d, 0x03, 0x12, 0x26, 0x38, 0x40, 0x40, 0x40, 0x0d
	};
	u8 addr = REG_MTX(1);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtx); i++) {
		int ret = ov965x_write(client, addr, mtx[i]);
		if (ret < 0)
			return ret;;
		addr++;
	}

	return 0;
}

static void ov965x_get_default_format(struct v4l2_mbus_framefmt *mf)
{
	mf->width = ov965x_framesizes[0].width;
	mf->height = ov965x_framesizes[0].height;
	mf->colorspace = ov965x_formats[0].colorspace;
	mf->code = ov965x_formats[0].code;
	mf->field = V4L2_FIELD_NONE;
}

static int ov965x_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov965x_formats))
		return -EINVAL;

	code->code = ov965x_formats[code->index].code;
	return 0;
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

static int __ov965x_try_frame_size(struct v4l2_mbus_framefmt *mf,
				   const struct ov965x_framesize **size)
{
	unsigned int min_err = ~0;
	int i = ARRAY_SIZE(ov965x_framesizes);
	const struct ov965x_framesize *fsize = &ov965x_framesizes[0],
		*match = NULL;

	while (i--) {
		int err = abs(fsize->width - mf->width)
				+ abs(fsize->height - mf->height);

		if (err < min_err) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}
	if (match) {
		mf->width  = match->width;
		mf->height = match->height;
		if (size)
			*size = match;
		return 0;
	}
	return -EINVAL;
}

static void ov965x_try_format(struct ov965x *ov965x,
			      struct v4l2_mbus_framefmt *mf)
{
	const struct ov965x_framesize *size = NULL;
	unsigned int index;

	__ov965x_try_frame_size(mf, &size);

	if (size)
		ov965x->reg_com7 = size->reg_com7;

	index = 0; /* ov965x_get_pixfmt_index(ov965x, mf); */

	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	mf->code	= ov965x_formats[index].code;
	mf->field	= V4L2_FIELD_NONE;
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
			ov965x->apply_fmt = 1;
		}
	}

	return ret;
}

static int ov965x_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov965x *ov965x = to_ov965x(sd);
	int ret = 0, i;

	pr_info("on: %d\n", on);

	if (!on) {
		/* sleep mode  */
		ov965x_write(client, REG_COM2, 0x11);
		return 0; /* DEBUG */
	}

	/* if (!on || (on && ov965x->streaming)) */
	/* 	return 0; */
	/* pr_info("on: %d\n", on); */

	for (i = 0; i < ARRAY_SIZE(ov965x_init_regs) && !ret; i++) {
		if (ov965x_init_regs[i].addr == CHIP_DELAY) {
			msleep(ov965x_init_regs[i].value);
			continue;
		}

		ret = ov965x_write(client, ov965x_init_regs[i].addr,
				   ov965x_init_regs[i].value);
		if (ret < 0)
			break;
	}
	if (!ret)
		ret = ov965x_set_gamma_curve(ov965x);
	if (!ret)
		ret = ov965x_set_color_matrix(ov965x);

	if (ov965x->apply_fmt) {
		ov965x_write(client, REG_COM7, ov965x->reg_com7);
		/* Output drive, disable soft sleep mode  */
		ov965x_write(client, REG_COM2, 0x01);
		//ov965x->apply_fmt = 1;
	}

	if (!ret)
		ov965x->streaming = on;

	return ret;
}

static int ov965x_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf = v4l2_subdev_get_try_format(fh, 0);

	ov965x_get_default_format(mf);
	v4l2_info(sd, "%s:%d\n", __func__, __LINE__);
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
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov965x *ov965x = to_ov965x(sd);
	int ret = 0;

	pr_info("on: %d\n", on);

	/* TODO: Add reference counting */

	if (on) {
		ov965x_gpio_set(ov965x->gpios[GPIO_PWDN], 0);
		ov965x_gpio_set(ov965x->gpios[GPIO_RST], 0);
		usleep_range(25000, 26000);
		/* soft reset */
		//ov965x_write(client, REG_COM7, 0x80);
		/* sleep mode */
		//msleep(50);
		//ov965x_write(client, REG_COM2, 0x11);
	} else {
		ov965x_gpio_set(ov965x->gpios[GPIO_RST], 1);
		ov965x_gpio_set(ov965x->gpios[GPIO_PWDN], 1);
		ov965x->streaming = 0;
	}

	return ret;
}

static const struct v4l2_subdev_pad_ops ov965x_pad_ops = {
	.enum_mbus_code = ov965x_enum_mbus_code,
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

	ret = ov965x_initialize_ctrls(ov965x);
	if (ret)
		goto err_gpio;

	ov965x_get_default_format(&ov965x->format);
	return 0;

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
MODULE_DEVICE_TABLE(i2c, ov965x_id);

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

