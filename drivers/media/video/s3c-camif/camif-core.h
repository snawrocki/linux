/*
 * s3c244x/s3c64xx SoC series Camera Interface (CAMIF) driver
 *
 * Copyright (c) 2012, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef CAMIF_CORE_H_
#define CAMIF_CORE_H_

#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <media/media-entity.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/s3c_camif.h>

#define S3C_CAMIF_DRIVER_NAME	"s3c-camif"
#define CAMIF_VP_COUNT		2
#define CAMIF_REQ_BUFS_MIN	4
#define CAMIF_MAX_OUT_BUFS	4
#define CAMIF_MAX_PIX_WIDTH	4096
#define CAMIF_MAX_PIX_HEIGHT	4096
#define CAMIF_DEF_WIDTH		640
#define CAMIF_DEF_HEIGHT	480

#define S3C244X_CAMIF_IP_REV	0x20 /* 2.0 */
#define S3C6410_CAMIF_IP_REV	0x32 /* 3.2 */

/* Bit index definitions for struct fimc_lite::state */
enum {
	ST_CAMIF_LPM,
	ST_CAMIF_PENDING,
	ST_CAMIF_RUN,
	ST_CAMIF_STREAM,
	ST_CAMIF_SUSPENDED,
	ST_CAMIF_OFF,
	ST_CAMIF_IN_USE,
	ST_CAMIF_CONFIG,
	ST_SENSOR_STREAM,
};

#define CAMIF_SD_PAD_SINK	0
#define CAMIF_SD_PAD_SOURCE	1
#define CAMIF_SD_PADS_NUM	2

enum img_fmt {
	IMG_FMT_RGB565 = 0x10,
	IMG_FMT_RGB666,
	IMG_FMT_RGB888,
	IMG_FMT_YCBCR420 = 0x20,
	IMG_FMT_YCBCR422P,
};

#define img_fmt_is_rgb(x) (!!((x) & 0x10))

/* color format description */
struct camif_fmt {
	/* media bus pixel code, -1 if not applicable */
	enum v4l2_mbus_pixelcode mbus_code;
	/* the format description */
	char	*name;
	/* the fourcc code for this format, 0 if not applicable */
	u32	fourcc;
	/* the corresponding enum img_fmt */
	u32	color;
	/* number of physically contiguous data planes */
	u16	colplanes;
	u16	flags;
	/* bits per pixel */
	u8	depth;
#define FMT_FLAGS_CAM	(1 << 0)
#define FMT_FLAGS_M2M	(1 << 1)
};

/**
 * struct camif_frame - source/target frame properties
 * @f_width: full pixel width
 * @f_height: full pixel height
 * @rect: crop/composition rectangle
 */
struct camif_frame {
	u16 f_width;
	u16 f_height;
	struct v4l2_rect rect;
};

/* CAMIF clocks enumeration */
enum {
	CLK_CAM,
	CLK_GATE,
	CLK_MAX_NUM,
};

struct s3c_camif_pix_limits {
	u16 max_out_width;
	u16 max_sc_out_width;
	u16 out_width_align;
	u16 max_height;
};

/*
 * Camera port pins, without CAMRST, which is unused by this host
 * driver and is intended to be controlled directly by sensor
 * subdev drivers, if required.
 */
#define CAMIF_NUM_GPIOS 	12

struct s3c_camif_variant {
	/* Pixel limits for the codec and preview paths */
	struct s3c_camif_pix_limits pix_limits[2];
	/* IP revision: 0x20 for s3c244x, 0x32 for s3c6410 */
	u8 ip_revision;
	/* Camera port pins, without CAMRST*/
	int gpios[CAMIF_NUM_GPIOS];
};

struct s3c_camif_drvdata {
	const struct s3c_camif_variant *variant;
	unsigned long bus_clk_freq;
};

struct camif_scaler {
	u8 scaleup_h;
	u8 scaleup_v;
	u8 copy_mode;
	u8 enable;
	u32 hfactor;
	u32 vfactor;
	u32 pre_hratio;
	u32 pre_vratio;
	u32 pre_dst_width;
	u32 pre_dst_height;
	u32 main_hratio;
	u32 main_vratio;
	u32 real_width;
	u32 real_height;
};

struct camif_dev;

/**
 * struct camif_vp - CAMIF data processing path abstraction (codec/preview)
 * @irq_queue:	interrupt handling waitqueue
 * @irq:	an interrupt number for the selected data path
 * @id:		CAMIF id, 0 - codec, 1 - preview
 */
struct camif_vp {
	wait_queue_head_t	irq_queue;
	int			irq;
	u16			id;
	struct camif_dev	*camif;
	struct media_pad	pad;
	struct video_device	vdev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct vb2_queue	vb_queue;
	struct list_head	pending_buf_q;
	struct list_head	active_buf_q;
	unsigned int		active_buf_count;
	unsigned int		buf_index;
	unsigned int		frame_sequence;
	unsigned int		reqbufs_count;
	unsigned int		ref_count;
	struct camif_scaler	scaler;
	const struct camif_fmt	*out_fmt;
	unsigned long		payload;
	struct camif_frame	out_frame;
	unsigned long		state;
};

/* Video processing path enumeration */
#define VP_CODEC	0
#define VP_PREVIEW	1
#define CAMIF_VP_NUM	2

/**
 * struct camif_dev - the CAMIF driver private data structure
 * @media_dev:    top-level media device structure
 * @v4l2_dev:	  root v4l2_device
 * @camif_sd:     camera interface ("catchcam") subdev
 * @sensor_sd:    image sensor subdev registered with the CAMIF driver
 * @pipeline:	  video entity pipeline description
 * @ctrl_handler: v4l2 control handler (owned by @camif_sd)
 * @test_pattern: test pattern controls
 * @subdev:       CAMIF sub-devices for the CODEC and PREVIEW paths
 * @alloc_ctx:    memory buffer allocator context
 * @variant:      variant information for the platform device
 * @dev:	  pointer to CAMIF device
 * @pdata:	  pointer to the device platform data
 * @clock:	  clocks required for CAMIF operation
 * @lock:	  mutex protecting this data structure
 * @slock:	  spinlock protecting CAMIF registers
 * @io_base:	  start address of the CAMIF mmapped I/O registers
 * @state:	  flags used to synchronize m2m and capture mode operation
 */
struct camif_dev {
	struct media_device		media_dev;
	struct v4l2_device		v4l2_dev;
	struct v4l2_subdev		subdev;
	struct camif_frame		inp_frame;
	const struct camif_fmt		*inp_fmt;
	struct media_pad		pads[CAMIF_SD_PADS_NUM];
	struct v4l2_subdev		*sensor;
	struct media_pipeline		*m_pipeline;

	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_ctrl		*test_pattern;

	struct camif_vp			vp[CAMIF_VP_NUM];
	struct vb2_alloc_ctx		*alloc_ctx;

	const struct s3c_camif_variant 	*variant;
	struct device			*dev;
	struct s3c_camif_plat_data	pdata;
	struct clk			*clock[CLK_MAX_NUM];
	struct mutex			lock;
	spinlock_t 			slock;
	void __iomem			*io_base;
	unsigned long			state;
};

/**
 * struct camif_addr - Y/CB/Cr plane DMA start address structure
 * @y:	 luminance plane physical address
 * @cb:	 Cb plane physical address
 * @cr:	 Cr plane physical address
 */
struct camif_addr {
	u32 y;
	u32 cb;
	u32 cr;
};

/**
 * struct camif_buffer - video buffer structure
 * @vb:    vb2 buffer
 * @list:  list head for the buffers queue
 * @paddr: DMA start addresses
 */
struct camif_buffer {
	struct vb2_buffer vb;
	struct list_head list;
	struct camif_addr paddr;
	unsigned int index;
};

const struct camif_fmt *s3c_camif_find_format(const u32 *pixelformat,
				      const u32 *mbus_code, int index);
int s3c_camif_register_video_node(struct camif_dev *camif, int idx);
void s3c_camif_unregister_video_node(struct camif_dev *camif, int idx);
irqreturn_t s3c_camif_irq_handler(int irq, void *priv);
int s3c_camif_create_subdev(struct camif_dev *camif);
void s3c_camif_unregister_subdev(struct camif_dev *camif);
int s3c_camif_set_defaults(struct camif_dev *camif);
/* int s3c_camif_get_scaler_factor(u32 src, u32 tar, u32 *ratio, u32 *shift); */
int s3c_camif_set_scaler_info(struct camif_vp *vp);


static inline bool s3c_camif_active(struct camif_dev *camif)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&camif->slock, flags);
	ret = camif->state & BIT(ST_CAMIF_RUN) ||
		camif->state & BIT(ST_CAMIF_PENDING);
	spin_unlock_irqrestore(&camif->slock, flags);
	return ret;
}

static inline void camif_active_queue_add(struct camif_vp *vp,
					 struct camif_buffer *buf)
{
	list_add_tail(&buf->list, &vp->active_buf_q);
	vp->active_buf_count++;
}

static inline struct camif_buffer *camif_active_queue_pop(
					struct camif_vp *vp)
{
	struct camif_buffer *buf = list_first_entry(&vp->active_buf_q,
					      struct camif_buffer, list);
	list_del(&buf->list);
	vp->active_buf_count--;
	return buf;
}

static inline void camif_pending_queue_add(struct camif_vp *vp,
					   struct camif_buffer *buf)
{
	list_add_tail(&buf->list, &vp->pending_buf_q);
}

static inline struct camif_buffer *camif_pending_queue_pop(
					struct camif_vp *vp)
{
	struct camif_buffer *buf = list_first_entry(&vp->pending_buf_q,
					      struct camif_buffer, list);
	list_del(&buf->list);
	return buf;
}

#endif /* CAMIF_CORE_H_ */
