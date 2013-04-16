/*
 * linux/drivers/video/s3c2410fb.h
 *	Copyright (c) 2004 Arnaud Patard
 *
 *  S3C2410 LCD Framebuffer Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#ifndef __S3C2410FB_H
#define __S3C2410FB_H

/* LCD control registers */
#define S3C2410_LCDCON1			0x00
#define S3C2410_LCDCON1_CLKVAL(x)	((x) << 8)
#define S3C2410_LCDCON1_ENVID		1
#define S3C2410_LCDCON1_MODEMASK	0x1e
#define S3C2410_LCDCON1_MMODE		(1 << 7)

#define S3C2410_LCDCON2			0x04
#define S3C2410_LCDCON2_VBPD(x)		((x) << 24)
#define S3C2410_LCDCON2_LINEVAL(x)	((x) << 14)
#define S3C2410_LCDCON2_VFPD(x)		((x) << 6)
#define S3C2410_LCDCON2_VSPW(x)		((x) << 0)

#define S3C2410_LCDCON2_GET_VBPD(x)	(((x) >> 24) & 0xff)
#define S3C2410_LCDCON2_GET_VFPD(x)	(((x) >>  6) & 0xff)
#define S3C2410_LCDCON2_GET_VSPW(x)	(((x) >>  0) & 0x3f)

#define S3C2410_LCDCON3			0x08
#define S3C2410_LCDCON3_HBPD(x)		((x) << 19)
#define S3C2410_LCDCON3_WDLY(x)		((x) << 19)
#define S3C2410_LCDCON3_HOZVAL(x)	((x) << 8)
#define S3C2410_LCDCON3_HFPD(x)		((x) << 0)
#define S3C2410_LCDCON3_LINEBLANK(x)	((x) << 0)

#define S3C2410_LCDCON3_GET_HBPD(x)	(((x) >> 19) & 0x7f)
#define S3C2410_LCDCON3_GET_HFPD(x)	(((x) >>  0) & 0xff)

/* LDCCON4 changes for STN mode on the S3C2412 */

#define S3C2410_LCDCON4			0x0c
#define S3C2410_LCDCON4_MVAL(x)		((x) << 8)
#define S3C2410_LCDCON4_HSPW(x)		((x) << 0)
#define S3C2410_LCDCON4_WLH(x)		((x) << 0)
#define S3C2410_LCDCON4_GET_HSPW(x)	(((x) >>  0) & 0xff)

#define S3C2410_LCDCON5			0x10

/* framebuffer start addressed */
#define S3C2410_LCDSADDR1		0x14
#define S3C2410_LCDSADDR2		0x18
#define S3C2410_LCDSADDR3		0x1c

#define S3C2410_LCDBANK(x)		((x) << 21)
#define S3C2410_LCDBASEU(x)		(x)

#define S3C2410_OFFSIZE(x)		((x) << 11)
#define S3C2410_PAGEWIDTH(x)		(x)

/* colour lookup and miscellaneous controls */

#define S3C2410_REDLUT			0x20
#define S3C2410_GREENLUT		0x24
#define S3C2410_BLUELUT			0x28

#define S3C2410_DITHMODE		0x4c
#define S3C2410_TPAL			0x50

#define S3C2410_TPAL_EN			(1 << 24)

/* interrupt info */
#define S3C2410_LCDINTPND		0x54
#define S3C2410_LCDSRCPND		0x58
#define S3C2410_LCDINTMSK		0x5c
#define S3C2410_LCDINT_FIWSEL		(1 << 2)
#define S3C2410_LCDINT_FRSYNC		(1 << 1)
#define S3C2410_LCDINT_FICNT		(1 << 0)

/* s3c2442 extra stn registers */

#define S3C2442_REDLUT			0x20
#define S3C2442_GREENLUT		0x24
#define S3C2442_BLUELUT			0x28
#define S3C2442_DITHMODE		0x20

#define S3C2410_LPCSEL			0x60

#define S3C2410_TFTPAL(x)		(0x400 + ((x) * 4))

/* S3C2412 registers */

#define S3C2412_TPAL			0x20

#define S3C2412_LCDINTPND		0x24
#define S3C2412_LCDSRCPND		0x28
#define S3C2412_LCDINTMSK		0x2c
#define S3C2412_TCONSEL			0x30
#define S3C2412_LCDCON6			0x34
#define S3C2412_LCDCON7			0x38
#define S3C2412_LCDCON8			0x3c
#define S3C2412_LCDCON9			0x40

#define S3C2412_REDLUT(x)		(0x44 + ((x) * 4))
#define S3C2412_GREENLUT(x)		(0x60 + ((x) * 4))
#define S3C2412_BLUELUT(x)		(0x98 + ((x) * 4))

#define S3C2412_FRCPAT(x)		(0xb4 + ((x)*4))

/* General registers. */

/*
 * Base of the LCD registers, where INTPND, INTSRC and then INTMSK
 * are available.
 */
#define S3C2410_LCDINTBASE		0x54
#define S3C2412_LCDINTBASE		0x24

#define S3C24XX_LCDINTPND		0x00
#define S3C24XX_LCDSRCPND		0x04
#define S3C24XX_LCDINTMSK		0x08


enum s3c_drv_type {
	DRV_S3C2410,
	DRV_S3C2412,
};

struct s3c2410fb_info {
	struct device		*dev;
	struct clk		*clk;

	struct resource		*mem;
	void __iomem		*io;
	void __iomem		*irq_base;

	enum s3c_drv_type	drv_type;
	struct s3c2410fb_hw	regs;

	unsigned long		clk_rate;
	unsigned int		palette_ready;

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif

	/* keep these registers in case we need to re-write palette */
	u32			palette_buffer[256];
	u32			pseudo_pal[16];
};

#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */

int s3c2410fb_init(void);

#endif
