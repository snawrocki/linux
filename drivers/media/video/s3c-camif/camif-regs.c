/*
 * Samsung s3c244x/s3c64xx SoC CAMIF driver
 *
 * Copyright (c) 2012, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#define DEBUG
#include "camif-regs.h"
#include <linux/delay.h>

static inline void camif_write(struct camif_dev *camif, unsigned int offset,
			       u32 val)
{
	/* if (camif->variant->rev32 && offset > 0x64) */
	/* 	offset += 0x20; */
	writel(val, camif->io_base + offset);
}

static inline u32 camif_read(struct camif_dev *camif, unsigned int offset)
{
	return readl(camif->io_base + offset);
}

static void camif_get_burst(u32 hsize, u32 *mburst, u32 *rburst)
{
	switch ((hsize / 4) % 16) {
	case 0:
		*mburst = 16;
		*rburst = 16;
		break;

	case 4:
		*mburst = 16;
		*rburst = 4;
		break;

	case 8:
		*mburst = 16;
		*rburst = 8;
		break;

	case 12:
		*mburst = 8;
		*rburst = 4;
		break;

	default:
		*mburst = 4;
		*rburst = (hsize / 4) % 4;
	}
}

void camif_hw_reset(struct camif_dev *camif)
{
	u32 cfg;

	cfg = camif_read(camif, S3C_CAMIF_REG_CISRCFMT);
	/* FIXME: set CISRCFMT_ITU601_8BIT bit only for R.601 bus type */
	cfg |= CISRCFMT_ITU601_8BIT;
	camif_write(camif, S3C_CAMIF_REG_CISRCFMT, cfg);

	/* S/W reset */
	cfg = camif_read(camif, S3C_CAMIF_REG_CIGCTRL);
	/* cfg |= (CIGCTRL_SWRST | CIGCTRL_IRQ_LEVEL); */
	cfg |= CIGCTRL_SWRST;
	camif_write(camif, S3C_CAMIF_REG_CIGCTRL, cfg);
	usleep_range(10000, 15000);

	cfg = camif_read(camif, S3C_CAMIF_REG_CIGCTRL);
	cfg &= ~CIGCTRL_SWRST;
	camif_write(camif, S3C_CAMIF_REG_CIGCTRL, cfg);
	usleep_range(10000, 15000);
}

void camif_hw_clear_pending_irq(struct camif_vp *vp)
{
	u32 cfg = camif_read(vp->camif, S3C_CAMIF_REG_CIGCTRL);
	cfg &= ~CIGCTRL_IRQ_CLR(vp->id);
	camif_write(vp->camif, S3C_CAMIF_REG_CIGCTRL, cfg);
	pr_debug("CIGCTRL: %#x", cfg);
}

u32 camif_hw_get_interrupt_source(struct camif_vp *vp)
{
	u32 intsrc = camif_read(vp->camif, S3C_CAMIF_REG_CISTATUS(vp->id, 0));
	return intsrc; // & S3C_CAMIF_REG_CISTATUS_IRQ_MASK);
}

/*
 * Sets video test pattern (off, color bar, horizontal or vertical gradient).
 * External sensor pixel clock must be active for the test pattern to work.
 */
void camif_hw_set_test_pattern(struct camif_dev *camif, unsigned int pattern)
{
	u32 cfg = camif_read(camif, S3C_CAMIF_REG_CIGCTRL);
	cfg &= ~CIGCTRL_TESTPATTERN_MASK;
	cfg |= (pattern << 27);
	camif_write(camif, S3C_CAMIF_REG_CIGCTRL, cfg);
}

static const u32 src_pixfmt_map[8][2] = {
	{ V4L2_MBUS_FMT_YUYV8_2X8, CISRCFMT_ORDER422_YCBYCR },
	{ V4L2_MBUS_FMT_YVYU8_2X8, CISRCFMT_ORDER422_YCRYCB },
	{ V4L2_MBUS_FMT_UYVY8_2X8, CISRCFMT_ORDER422_CBYCRY },
	{ V4L2_MBUS_FMT_VYUY8_2X8, CISRCFMT_ORDER422_CRYCBY },
};

/* Set camera input pixel format and resolution */
void camif_hw_set_source_format(struct camif_dev *camif)
{
	enum v4l2_mbus_pixelcode pixelcode = camif->inp_fmt->mbus_code;
	struct camif_frame *f = &camif->inp_frame;
	unsigned int i = ARRAY_SIZE(src_pixfmt_map);
	u32 cfg;

	while (i-- >= 0) {
		if (src_pixfmt_map[i][0] == pixelcode)
			break;
	}

	if (i == 0 && src_pixfmt_map[i][0] != pixelcode) {
		dev_err(camif->dev,
			"Unsupported pixel code, falling back to %#08x\n",
			src_pixfmt_map[i][0]);
	}

	cfg = camif_read(camif, S3C_CAMIF_REG_CISRCFMT);
	cfg &= ~(CISRCFMT_ORDER422_MASK | CISRCFMT_SIZE_CAM_MASK);
	cfg |= (f->f_width << 16) | f->f_height;
	cfg |= src_pixfmt_map[i][1];
	camif_write(camif, S3C_CAMIF_REG_CISRCFMT, cfg);
}

/* Set the camera host input window offsets (cropping) */
void camif_hw_set_camera_crop(struct camif_dev *camif)
{
	struct camif_frame *f = &camif->inp_frame;
	u32 hoff2, voff2;
	u32 cfg;

	/* TODO: requirement for s3c2444x: left = f_width - rect.width / 2 */
	cfg = camif_read(camif, S3C_CAMIF_REG_CIWDOFST);
	cfg &= ~CIWDOFST_OFST_MASK;
	//cfg |= (f->rect.left << 16) | f->rect.top;
	//cfg |= CIWDOFST_WINOFSEN;
	camif_write(camif, S3C_CAMIF_REG_CIWDOFST, cfg);

	if (camif->variant->ip_revision == S3C6410_CAMIF_IP_REV) {
		hoff2 = f->f_width - f->rect.width - f->rect.left;
		voff2 = f->f_height - f->rect.height - f->rect.top;
		cfg = (hoff2 << 16) | voff2;
		camif_write(camif, S3C_CAMIF_REG_CIWDOFST2, cfg);
	}
}

void camif_hw_clear_fifo_overflow(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	u32 cfg;

	cfg = camif_read(camif, S3C_CAMIF_REG_CIWDOFST);
	if (vp->id == 0)
		cfg |= (CIWDOFST_CLROVCOFIY | CIWDOFST_CLROVCOFICB |
			CIWDOFST_CLROVCOFICR);
	else
		cfg |= (CIWDOFST_CLROVPRFIY | CIWDOFST_CLROVPRFICB |
			CIWDOFST_CLROVPRFICR);
	camif_write(camif, S3C_CAMIF_REG_CIWDOFST, cfg);
}

/* Set video bus signals polarity */
void camif_hw_set_camera_bus(struct camif_dev *camif)
{
	unsigned int flags = camif->pdata.sensor.flags;

	u32 cfg = camif_read(camif, S3C_CAMIF_REG_CIGCTRL);

	cfg &= ~(CIGCTRL_INVPOLPCLK | CIGCTRL_INVPOLVSYNC |
		 CIGCTRL_INVPOLHREF);

	if (flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
		cfg |= CIGCTRL_INVPOLPCLK;

	if (flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		cfg |= CIGCTRL_INVPOLVSYNC;
	/*
	 * HREF is normally high during frame active data
	 * transmission and low during horizontal synchronization
	 * period. Thus HREF active high means HSYNC active low.
	 */
	if (flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
		cfg |= CIGCTRL_INVPOLHREF; /* HREF active low */

	pr_debug("Setting CIGCTRL to: %#x\n", cfg);

	camif_write(camif, S3C_CAMIF_REG_CIGCTRL, cfg);
}

void camif_hw_set_output_addr(struct camif_vp *vp,
			      struct camif_addr *paddr, int i)
{
	struct camif_dev *camif = vp->camif;

	pr_debug("dst_buf[%d]: 0x%X, cb: 0x%X, cr: 0x%X",
		 i, paddr->y, paddr->cb, paddr->cr);

	camif_write(camif, S3C_CAMIF_REG_CIYSA(vp->id, i), paddr->y);
	if (vp->id == VP_CODEC) {
		camif_write(camif, S3C_CAMIF_REG_CICOCBSA(i), paddr->cb);
		camif_write(camif, S3C_CAMIF_REG_CICOCRSA(i), paddr->cr);
	}
}

#if 0
void camif_hw_set_out_order(struct camif_dev *camif, struct camif_frame *f)
{
	static const u32 pixcode[4][2] = {
		{ V4L2_MBUS_FMT_YUYV8_2X8, S3C_CAMIF_REG_CIODMAFMT_YCBYCR },
		{ V4L2_MBUS_FMT_YVYU8_2X8, S3C_CAMIF_REG_CIODMAFMT_YCRYCB },
		{ V4L2_MBUS_FMT_UYVY8_2X8, S3C_CAMIF_REG_CIODMAFMT_CBYCRY },
		{ V4L2_MBUS_FMT_VYUY8_2X8, S3C_CAMIF_REG_CIODMAFMT_CRYCBY },
	};
	u32 cfg = camif_read(camif, S3C_CAMIF_REG_CIODMAFMT);
	unsigned int i = ARRAY_SIZE(pixcode);

	while (i-- >= 0)
		if (pixcode[i][0] == camif->mbus_code)
			break;
	cfg &= ~S3C_CAMIF_REG_CIODMAFMT_YCBCR_ORDER_MASK;
	writel(cfg | pixcode[i][1], io_base + S3C_CAMIF_REG_CIODMAFMT);
}
#endif

static void camif_hw_set_out_dma_size(struct camif_vp *vp)
{
	struct camif_frame *frame = &vp->out_frame;
	u32 cfg;

	cfg = camif_read(vp->camif, S3C_CAMIF_REG_CITRGFMT(vp->id, 0));
	cfg &= ~CITRGFMT_TARGETSIZE_MASK;
	cfg |= (frame->f_width << 16) | frame->f_height;
	camif_write(vp->camif, S3C_CAMIF_REG_CITRGFMT(vp->id, 0), cfg);
}

void camif_hw_set_output_dma(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	struct camif_frame *frame = &vp->out_frame;
	/* struct camif_dma_offset *offset = &frame->dma_offset; */
	const struct camif_fmt *fmt = vp->out_fmt;
	u32 cfg;

	u32 y1burst = 0, y2burst = 0, c1burst = 4, c2burst = 2;

	/* FIXME: Composition onto DMA output buffer for s3c64xx */
#if 0
	/* Set the input dma offsets. */
	cfg = (offset->y_v << 16) | offset->y_h;
	writel(cfg, regs + S3C_CAMIF_REG_CISSY(vp->id));

	cfg = (offset->cb_v << 16) | offset->cb_h;
	writel(cfg, regs + S3C_CAMIF_REG_CISSY(vp->id));

	cfg = (offset->cr_v << 16) | offset->cr_h;
	writel(cfg, regs + S3C_CAMIF_REG_CISSY(vp->id));
#endif
	camif_hw_set_out_dma_size(vp);


	/* Configure chroma components order. */
	cfg = camif_read(camif, S3C_CAMIF_REG_CICTRL(vp->id, 0));

#if 0
	cfg &= ~(CIOCTRL_ORDER2P_MASK | CIOCTRL_ORDER422_MASK |
		 CIOCTRL_YCBCR_PLANE_MASK | CIOCTRL_RGB16FMT_MASK);

	if (fmt->colplanes == 1)
		cfg |= vp->out_order_1p;
	else if (fmt->colplanes == 2)
		cfg |= vp->out_order_2p | CIOCTRL_YCBCR_2PLANE;
	else if (fmt->colplanes == 3)
		cfg |= CIOCTRL_YCBCR_3PLANE;

	if (fmt->color == IMG_FMT_RGB565)
		cfg |= CIOCTRL_RGB565;
	else if (fmt->color == IMG_FMT_RGB555)
		cfg |= CIOCTRL_ARGB1555;

	writel(cfg, regs + S3C_CAMIF_REG_CIOCTRL);
#endif

	cfg &= ~CICTRL_BURST_MASK;

	switch (fmt->color) {
	case IMG_FMT_RGB565:
		break;
		/* YUYV */
	default:
		WARN_ON(frame->f_width % 4);

		camif_get_burst(2 * frame->f_width, &y1burst, &y2burst);

		/*
		 * According to S3C6410 User's Manual:
		 *
		 * "When Codec output format is YCbCr 4:2:2 interleave,
		 * ScalerBypass_Co = 0 and ScaleUp_V_Co = 1 , Wanted main
		 * burst length = 16 and Wanted remained burst length != 16
		 * is not allowed."
		 *
		 * So let's just disallow such configuration to be on safe side.
		 */
		if (y1burst == 16 && y2burst != 16)
			y1burst = 8;

		y1burst /= 2;
		y2burst /= 2;
		c1burst = y1burst / 2;
		c2burst = y2burst / 2;
		break;

	/* default: */
	/* 	WARN_ON(frame->width % 8); */
	/* 	fimc_get_burst(frame->width, &y1burst, &y2burst); */
	/* 	fimc_get_burst(frame->width / 2, &c1burst, &c2burst); */
	}

	pr_debug("y1burst = %d, y2burst = %d, c1burst = %d, c2burst = %d",
		y1burst, y2burst, c1burst, c2burst);

	cfg |= CICTRL_YBURST1(y1burst);
	cfg |= CICTRL_YBURST2(y2burst);
	cfg |= CICTRL_CBURST1(c1burst);
	cfg |= CICTRL_CBURST2(c2burst);

	camif_write(camif, S3C_CAMIF_REG_CICTRL(vp->id, 0), cfg);
}

void camif_hw_set_input_path(struct camif_vp *vp)
{
	u32 cfg = camif_read(vp->camif, S3C_CAMIF_REG_MSCTRL(vp->id));

	/* if (ctx->in_path == CAMIF_IO_DMA) */
	/* 	cfg |= MSCTRL_SEL_DMA_CAM; */
	/* else */
		cfg &= ~MSCTRL_SEL_DMA_CAM;

	camif_write(vp->camif, S3C_CAMIF_REG_MSCTRL(vp->id), cfg);
}

#if 0
/* Enable/disable output DMA, set output pixel size and offsets (composition) */
void camif_hw_set_output_dma(struct camif_dev *camif, struct camif_frame *f,
			     bool enable)
{
	u32 cfg = camif_read(camif, S3C_CAMIF_REG_CIGCTRL);

	if (!enable) {
		cfg |= S3C_CAMIF_REG_CIGCTRL_ODMA_DISABLE;
		camif_write(camif, S3C_CAMIF_REG_CIGCTRL);
		return;
	}

	cfg &= ~S3C_CAMIF_REG_CIGCTRL_ODMA_DISABLE;
	camif_write(camif, S3C_CAMIF_REG_CIGCTRL);

	camif_hw_set_out_order(camif, f);
	camif_hw_set_dma_window(camif, f);
}
#endif

void camif_hw_set_target_format(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	struct camif_frame *frame = &vp->out_frame;
	u32 cfg;

	pr_debug("fw: %d, fh: %d color: %d\n", frame->f_width,
		frame->f_height, vp->out_fmt->color);

	cfg = camif_read(camif, S3C_CAMIF_REG_CITRGFMT(vp->id, 0));

	//cfg &= ~(CITRGFMT_OUTFMT_MASK | CITRGFMT_TARGETSIZE_MASK);

	cfg &= ~CITRGFMT_TARGETSIZE_MASK;
	cfg &= ~(CITRGFMT_IN422 | CITRGFMT_OUT422);

#if 0 /* TODO */
	switch (vp->out_fmt->color) {
	case IMG_FMT_RGB565...IMG_FMT_RGB888:
		cfg |= CITRGFMT_OUTFORMAT_RGB;
		break;
	case IMG_FMT_YCBCR420:
		cfg |= CITRGFMT_OUTFORMAT_YCBCR420;
		break;
	case IMG_FMT_YCBYCR422...IMG_FMT_CRYCBY422:
		/* if (frame->fmt->colplanes == 1) */
		/* 	cfg |= CAMIF_REG_CITRGFMT_YCBCR422_1P; */
		/* else */
			cfg |= CITRGFMT_OUTFORMAT_YCBCR422;
		break;
	}
#endif
	cfg |= (CITRGFMT_IN422);

	if (vp->out_fmt->color != IMG_FMT_YCBCR420)
		cfg |= CITRGFMT_OUT422;

	/* if (ctx->rotation == 90 || ctx->rotation == 270) */
	/* 	cfg |= (frame->height << 16) | frame->width; */
	/* else */
	cfg |= (frame->f_width << 16) | frame->f_height;
	camif_write(camif, S3C_CAMIF_REG_CITRGFMT(vp->id, 0), cfg);


	cfg = camif_read(camif, S3C_CAMIF_REG_CITAREA(vp->id, 0));
	cfg &= ~CITAREA_MASK;
	cfg |= (frame->f_width * frame->f_height);
	camif_write(camif, S3C_CAMIF_REG_CITAREA(vp->id, 0), cfg);
}

/* Return an index to a buffer actually being written. */
u32 camif_hw_get_frame_index(struct camif_vp *vp)
{
	unsigned int addr = S3C_CAMIF_REG_CISTATUS(vp->id, 0);
	u32 cfg = camif_read(vp->camif, addr);
	return (cfg & CISTATUS_FRAMECNT_MASK) >> 26;
}

static void camif_hw_set_prescaler(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	struct camif_scaler *sc = &vp->scaler;
	u32 cfg, shfactor, addr;

	addr = S3C_CAMIF_REG_CISCPRERATIO(vp->id, 0);

	shfactor = 10 - (sc->hfactor + sc->vfactor);
	cfg = shfactor << 28;

	cfg |= (sc->pre_hratio << 16) | sc->pre_vratio;
	camif_write(camif, addr, cfg);

	cfg = (sc->pre_dst_width << 16) | sc->pre_dst_height;
	camif_write(camif, S3C_CAMIF_REG_CISCPREDST(vp->id, 0), cfg);
}

void camif_hw_set_scaler(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	struct camif_scaler *scaler = &vp->scaler;
	unsigned int color = vp->out_fmt->color;
	u32 cfg;

	camif_hw_set_prescaler(vp);
	cfg = camif_read(camif, S3C_CAMIF_REG_CISCCTRL(vp->id, 0));

#if 0
	cfg &= ~(CISCCTRL_CSCR2Y_WIDE | CISCCTRL_CSCY2R_WIDE |
		 CISCCTRL_SCALEUP_H | CISCCTRL_SCALEUP_V |
		 CISCCTRL_SCALERBYPASS | CISCCTRL_ONE2ONE |
		 CISCCTRL_INRGB_FMT_MASK | CISCCTRL_OUTRGB_FMT_MASK |
		 CISCCTRL_INTERLACE | CISCCTRL_MAIN_RATIO_MASK);

	//if (!(vp->flags & CAMIF_COLOR_RANGE_NARROW))
		cfg |= CISCCTRL_CSCR2Y_WIDE | CISCCTRL_CSCY2R_WIDE;

	if (!sc->enable)
		cfg |= CISCCTRL_SCALERBYPASS;
	if (sc->scaleup_h)
		cfg |= CISCCTRL_SCALEUP_H;
	if (sc->scaleup_v)
		cfg |= CISCCTRL_SCALEUP_V;
	if (sc->copy_mode)
		cfg |= CISCCTRL_ONE2ONE;

	switch (color) {
	case IMG_FMT_RGB565:
		cfg |= CISCCTRL_OUTRGB_FMT_RGB565;
		break;
	case IMG_FMT_RGB666:
		cfg |= CISCCTRL_OUTRGB_FMT_RGB666;
		break;
	case IMG_FMT_RGB888:
		cfg |= CISCCTRL_OUTRGB_FMT_RGB888;
		break;
	}
#endif
	cfg &= ~(CISCCTRL_SCALEUP_MASK | CISCCTRL_SCALERBYPASS |
		 CISCCTRL_MAIN_RATIO_MASK);

	if (scaler->enable) {
		if (scaler->scaleup_h)
			cfg |= CISCCTRL_SCALEUP_H;

		if (scaler->scaleup_v)
			cfg |= CISCCTRL_SCALEUP_V;
	} else {
		cfg |= CISCCTRL_SCALERBYPASS;
	}

	cfg |= (scaler->main_hratio & 0x1ff) << 16;
	cfg |= scaler->main_vratio & 0x1ff;

	camif_write(camif, S3C_CAMIF_REG_CISCCTRL(vp->id, 0), cfg);

	pr_debug("main_hratio: 0x%X  main_vratio: 0x%X",
		 scaler->main_hratio, scaler->main_vratio);
}

void camif_hw_enable_scaler(struct camif_vp *vp, bool on)
{
	u32 addr = S3C_CAMIF_REG_CISCCTRL(vp->id, 0);
	u32 cfg;

	cfg = camif_read(vp->camif, addr);
	if (on)
		cfg |= CISCCTRL_SCALERSTART;
	else
		cfg &= ~CISCCTRL_SCALERSTART;
	camif_write(vp->camif, addr, cfg);
}

void camif_hw_set_lastirq(struct camif_vp *vp, int enable)
{
	u32 addr = S3C_CAMIF_REG_CICTRL(vp->id, 0);
	u32 cfg;

	cfg = camif_read(vp->camif, addr);
	if (enable)
		cfg |= CICTRL_LASTIRQ_ENABLE;
	else
		cfg &= ~CICTRL_LASTIRQ_ENABLE;
	camif_write(vp->camif, addr, cfg);
}

void camif_hw_enable_capture(struct camif_vp *vp)
{
	u32 cfg = camif_read(vp->camif, S3C_CAMIF_REG_CIIMGCPT(0));

	cfg |= CIIMGCPT_IMGCPTEN;

	//s3c64xx | CIIMGCPT_CPT_FREN_ENABLE(vp->id);

	if (vp->scaler.enable)
		cfg |= CIIMGCPT_IMGCPTEN_SC(vp->id);

	camif_write(vp->camif, S3C_CAMIF_REG_CIIMGCPT(0), cfg);

	cfg = camif_read(vp->camif, S3C_CAMIF_REG_CIIMGCPT(0));
	pr_info("CIIMGCPT: %#x\n", cfg);
}

void camif_hw_disable_capture(struct camif_vp *vp)
{
	u32 cfg = camif_read(vp->camif, S3C_CAMIF_REG_CIIMGCPT(0));
	cfg &= ~(CIIMGCPT_IMGCPTEN | CIIMGCPT_IMGCPTEN_SC(vp->id));
	camif_write(vp->camif, S3C_CAMIF_REG_CIIMGCPT(0), cfg);
}

/* Locking: the caller holds camif->slock */
void camif_activate_capture(struct camif_vp *vp)
{
	pr_debug("sc: %d\n", vp->scaler.enable);
	camif_hw_enable_scaler(vp, vp->scaler.enable);
	camif_hw_enable_capture(vp);
}

void camif_deactivate_capture(struct camif_vp *vp)
{
	camif_hw_set_lastirq(vp, true);
	camif_hw_disable_capture(vp);
	camif_hw_enable_scaler(vp, false);
	camif_hw_set_lastirq(vp, false);
	pr_debug("\n");
}

void camif_hw_dump_regs(struct camif_dev *camif, const char *label)
{
	struct {
		u32 offset;
		const char * const name;
	} registers[] = {
		{ S3C_CAMIF_REG_CISRCFMT, "CISRCFMT" },
		{ S3C_CAMIF_REG_CIWDOFST, "CIWDOFST" },
		{ S3C_CAMIF_REG_CIGCTRL, "CIGCTRL\t" },
		{ S3C_CAMIF_REG_CIWDOFST2, "CIWDOFST2" },

		{ S3C_CAMIF_REG_CIYSA(0, 0), "CICOYSA0" },
		{ S3C_CAMIF_REG_CICOCBSA(0), "CICOCBSA0" },
		{ S3C_CAMIF_REG_CICOCRSA(0), "CICOCRSA0" },
		{ S3C_CAMIF_REG_CIYSA(0, 1), "CICOYSA1" },
		{ S3C_CAMIF_REG_CICOCBSA(1), "CICOCBSA1" },
		{ S3C_CAMIF_REG_CICOCRSA(1), "CICOCRSA1" },
		{ S3C_CAMIF_REG_CIYSA(0, 2), "CICOYSA2" },
		{ S3C_CAMIF_REG_CICOCBSA(2), "CICOCBSA2" },
		{ S3C_CAMIF_REG_CICOCRSA(2), "CICOCRSA2" },
		{ S3C_CAMIF_REG_CIYSA(0, 3), "CICOYSA3" },
		{ S3C_CAMIF_REG_CICOCBSA(3), "CICOCBSA3" },
		{ S3C_CAMIF_REG_CICOCRSA(3), "CICOCRSA3" },

		{ S3C_CAMIF_REG_CIYSA(1, 0), "CIPRYSA0" },
		{ S3C_CAMIF_REG_CIYSA(1, 1), "CIPRYSA1" },
		{ S3C_CAMIF_REG_CIYSA(1, 2), "CIPRYSA2" },
		{ S3C_CAMIF_REG_CIYSA(1, 3), "CIPRYSA3" },

		{ S3C_CAMIF_REG_CITRGFMT(0, 0), "CICOTRGFMT" },
		{ S3C_CAMIF_REG_CITRGFMT(1, 0), "CIPRTRGFMT" },

		{ S3C_CAMIF_REG_CICTRL(0, 0), "CICOCTRL" },
		{ S3C_CAMIF_REG_CICTRL(1, 0), "CIPRCTRL" },

		{ S3C_CAMIF_REG_CISCPREDST(0, 0), "CICOSCPREDST" },
		{ S3C_CAMIF_REG_CISCPREDST(1, 0), "CIPRSCPREDST" },

		{ S3C_CAMIF_REG_CISCPRERATIO(0, 0), "CICOSCPRERATIO" },
		{ S3C_CAMIF_REG_CISCPRERATIO(1, 0), "CIPRSCPRERATIO" },

		{ S3C_CAMIF_REG_CISCCTRL(0, 0), "CICOSCCTRL" },
		{ S3C_CAMIF_REG_CISCCTRL(1, 0), "CIPRSCCTRL" },

		{ S3C_CAMIF_REG_CITAREA(0, 0), "CICOTAREA" },
		{ S3C_CAMIF_REG_CITAREA(1, 0), "CIPRTAREA" },

		{ S3C_CAMIF_REG_CISTATUS(0, 0), "CICOSTATUS" },
		{ S3C_CAMIF_REG_CISTATUS(1, 0), "CIPRSTATUS" },

		{ S3C_CAMIF_REG_CIIMGCPT(0), "CIIMGCPT" },
	};
	u32 i;

	pr_info("--- %s ---\n", label);
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 cfg = readl(camif->io_base + registers[i].offset);
		printk(KERN_INFO "%s:\t0x%08x\n", registers[i].name, cfg);
	}
}
