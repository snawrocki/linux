/*
 * s3c24xx/s3c64xx SoC series Camera Interface (CAMIF) driver
 *
 * Copyright (c) 2012, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#define DEBUG
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "camif-regs.h"
#include "camif-core.h"

static int debug;
module_param(debug, int, 0644);

static int buf_set = 0;

static int s3c_camif_hw_init(struct camif_dev *camif, struct camif_vp *vp)
{
	unsigned long flags;
	int ret = 0;

	if (camif->sensor == NULL || vp->out_fmt == NULL)
		return -EINVAL;

	spin_lock_irqsave(&camif->slock, flags);
	pr_info("\n");
	//camif_prepare_dma_offset(ctx, &ctx->d_frame);
	//camif_set_yuv_order(ctx);

	camif_hw_clear_fifo_overflow(vp);
	camif_hw_set_camera_bus(camif);
	camif_hw_set_source_format(camif);
	//camif_hw_set_camera_crop(camif);

	camif_hw_set_test_pattern(camif, camif->test_pattern->val);

	WARN_ON(s3c_camif_set_scaler_info(vp));
	camif_hw_set_scaler(vp);

	camif_hw_set_target_format(vp);
	//camif_hw_set_rotation(ctx);

	camif_hw_set_output_dma(vp);
	clear_bit(ST_CAMIF_CONFIG, &camif->state);

	spin_unlock_irqrestore(&camif->slock, flags);

	camif_hw_dump_regs(camif, __func__);

	return ret;
}

/*
 * Reinitialize the driver so it is ready to start the streaming again.
 * Set camif->state to indicate stream off and the hardware shut down state.
 * If not suspending (@suspend is false), return any buffers to videobuf2.
 * Otherwise put any owned buffers onto the pending buffers queue, so they
 * can be re-spun when the device is being resumed. Also perform CAMIF
 * software reset and disable streaming on the whole pipeline if required.
 */
static int s3c_camif_reinit(struct camif_vp *vp, bool suspend)
{
	struct camif_dev *camif = vp->camif;
	struct camif_buffer *buf;
	unsigned long flags;
	bool streaming;

	pr_info("\n");
	spin_lock_irqsave(&camif->slock, flags);
	streaming = camif->state & (1 << ST_SENSOR_STREAM);

	camif->state &= ~(1 << ST_CAMIF_RUN | 1 << ST_CAMIF_OFF |
			  1 << ST_CAMIF_STREAM | 1 << ST_SENSOR_STREAM);
	if (suspend)
		camif->state |= (1 << ST_CAMIF_SUSPENDED);
	else
		camif->state &= ~(1 << ST_CAMIF_PENDING |
				 1 << ST_CAMIF_SUSPENDED);

	/* Release unused buffers */
	while (!suspend && !list_empty(&vp->pending_buf_q)) {
		buf = camif_pending_queue_pop(vp);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	/* If suspending put unused buffers onto pending queue */
	while (!list_empty(&vp->active_buf_q)) {
		buf = camif_active_queue_pop(vp);
		if (suspend)
			camif_pending_queue_add(vp, buf);
		else
			vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&camif->slock, flags);

	camif_hw_reset(camif);

	//camif_hw_dump_regs(camif, __func__);

	if (!streaming)
		return 0;

	return v4l2_subdev_call(camif->sensor, video, s_stream, 1);
}

static int s3c_camif_stop_capture(struct camif_vp *vp, bool suspend)
{
	struct camif_dev *camif = vp->camif;
	unsigned long flags;

	if (!s3c_camif_active(camif))
		return 0;

	spin_lock_irqsave(&camif->slock, flags);
	set_bit(ST_CAMIF_OFF, &camif->state);
	camif_deactivate_capture(vp);
	spin_unlock_irqrestore(&camif->slock, flags);

	wait_event_timeout(vp->irq_queue,
			   !test_bit(ST_CAMIF_OFF, &camif->state),
			   (2 * HZ / 10)); /* 200 ms */

	return s3c_camif_reinit(vp, suspend);
}

/* Must be called  with camif.slock spinlock held. */
static void s3c_camif_config_update(struct camif_dev *camif,
				    struct camif_vp *vp)
{
	//camif_hw_set_window_offset(camif);
	//camif_hw_set_dma_window(camif, &vp->out_frame);
	camif_hw_set_test_pattern(camif, camif->test_pattern->val);
	clear_bit(ST_CAMIF_CONFIG, &camif->state);
}

static int camif_prepare_addr(struct camif_vp *vp, struct vb2_buffer *vb,
			      struct camif_addr *paddr)
{
	struct camif_frame *frame = &vp->out_frame;
	u32 pix_size;

	if (vb == NULL || frame == NULL)
		return -EINVAL;

	/* pix_size = frame->rect.width * frame->rect.height; */
	pix_size = frame->f_width * frame->f_height;

	pr_debug("colplanes: %d, pix_size: %d\n",
		 vp->out_fmt->colplanes, pix_size);

	paddr->y = vb2_dma_contig_plane_dma_addr(vb, 0);

	switch (vp->out_fmt->colplanes) {
	case 1:
		paddr->cb = 0;
		paddr->cr = 0;
		break;
	case 2:
		/* decompose Y into Y/Cb */
		paddr->cb = (u32)(paddr->y + pix_size);
		paddr->cr = 0;
		break;
	case 3:
		paddr->cb = (u32)(paddr->y + pix_size);
		/* decompose Y into Y/Cb/Cr */
		if (vp->out_fmt->color == IMG_FMT_YCBCR420)
			paddr->cr = (u32)(paddr->cb + (pix_size >> 2));
		else /* 422 */
			paddr->cr = (u32)(paddr->cb + (pix_size >> 1));
		break;
	default:
		return -EINVAL;
	}

	pr_debug("PHYS_ADDR: y= 0x%X  cb= 0x%X cr= 0x%X\n",
		 paddr->y, paddr->cb, paddr->cr);

	return 0;
}

irqreturn_t s3c_camif_irq_handler(int irq, void *priv)
{
	struct camif_vp *vp = priv;
	struct camif_dev *camif = vp->camif;
	unsigned int ip_rev = camif->variant->ip_revision;
	struct camif_buffer *vbuf;
	u32 status;

	spin_lock(&camif->slock);

	if (ip_rev == S3C6410_CAMIF_IP_REV)
		camif_hw_clear_pending_irq(vp);

	//intsrc = camif_hw_get_status(vp);
	status = readl(camif->io_base + S3C_CAMIF_REG_CISTATUS(vp->id, 0));
	//	pr_info("STATUS[%d]: %#x", vp->id, status);

	if (status & ((1 << 31) | (1 << 30) | (1 << 29))) {
		camif_hw_clear_fifo_overflow(vp);
		goto unlock;
	}

	/* camif_hw_clear_fifo_overflow(vp); */
	status = readl(camif->io_base + S3C_CAMIF_REG_CISTATUS(vp->id, 0));
	//pr_info("*STATUS[%d]: %#x", vp->id, status);

	//status &= ~((1 << 31) | (1 << 30) | (1 << 29));
	//writel(status, camif->io_base + S3C_CAMIF_REG_CISTATUS(vp->id, 0));

	if (test_and_clear_bit(ST_CAMIF_OFF, &camif->state)) {
		wake_up(&vp->irq_queue);
		goto done;
	}

	if (!list_empty(&vp->active_buf_q) &&
	    test_bit(ST_CAMIF_RUN, &camif->state)) {
		struct timeval *tv;
		struct timespec ts;

		ktime_get_real_ts(&ts);
		vbuf = camif_active_queue_pop(vp);

		tv = &vbuf->vb.v4l2_buf.timestamp;
		tv->tv_sec = ts.tv_sec;
		tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
		vbuf->vb.v4l2_buf.sequence = vp->frame_sequence++;

		vb2_buffer_done(&vbuf->vb, VB2_BUF_STATE_DONE);
	}

	if (!list_empty(&vp->pending_buf_q)) {
		vbuf = camif_pending_queue_pop(vp);
		camif_hw_set_output_addr(vp, &vbuf->paddr, vbuf->index);

		vbuf->index = vp->buf_index;

		/* Move the buffer to the capture active queue */
		camif_active_queue_add(vp, vbuf);

		//pr_info("next frame: %d, done frame: %d",
		//	camif_hw_get_frame_index(camif), vbuf->index);

		if (++vp->buf_index >= CAMIF_MAX_OUT_BUFS)
			vp->buf_index = 0;
	}

	if (vp->active_buf_count == 0) {
		if (++vp->buf_index >= CAMIF_MAX_OUT_BUFS)
			vp->buf_index = 0;
	} else {
		set_bit(ST_CAMIF_RUN, &camif->state);
	}

	if (test_bit(ST_CAMIF_CONFIG, &camif->state))
		s3c_camif_config_update(camif, vp);
done:
	if (vp->active_buf_count == 1) {
		///camif_deactivate_capture(vp);
		//clear_bit(ST_CAMIF_STREAM, &camif->state);
	}

	//pr_info("frame: %d, active_buf_count: %d",
	//	 /* camif_hw_get_frame_index(camif),*/ 0, vp->active_buf_count);
unlock:
	spin_unlock(&camif->slock);
	return IRQ_HANDLED;
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct camif_vp *vp = vb2_get_drv_priv(vq);
	struct camif_dev *camif = vp->camif;
	int ret;

	pr_info("\n");
	vp->frame_sequence = 0;

	ret = s3c_camif_hw_init(camif, vp);
	if (ret) {
		s3c_camif_reinit(vp, false);
		return ret;
	}

	set_bit(ST_CAMIF_PENDING, &camif->state);
	buf_set = 0;

	if (vp->active_buf_count >= CAMIF_REQ_BUFS_MIN &&
	    !test_and_set_bit(ST_CAMIF_STREAM, &camif->state)) {
		camif_activate_capture(vp);
		camif_hw_dump_regs(camif, __func__);

		if (!test_and_set_bit(ST_SENSOR_STREAM, &camif->state))
			v4l2_subdev_call(camif->sensor, video, s_stream, 1);
	}
	/* if (debug > 0) */
		/* camif_hw_dump_regs(camif, __func__); */

	return 0;
}

static int stop_streaming(struct vb2_queue *vq)
{
	struct camif_vp *vp = vb2_get_drv_priv(vq);
	struct camif_dev *camif = vp->camif;

	pr_info("\n");
	if (!s3c_camif_active(camif))
		return -EINVAL;

	return s3c_camif_stop_capture(vp, false);
}

static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *pfmt,
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], void *allocators[])
{
	const struct v4l2_pix_format *pix = NULL;
	struct camif_vp *vp = vb2_get_drv_priv(vq);
	struct camif_dev *camif = vp->camif;
	struct camif_frame *frame = &vp->out_frame;
	const struct camif_fmt *fmt = vp->out_fmt;
	unsigned int size;

	pr_debug("\n");
	if (pfmt) {
		pix = &pfmt->fmt.pix;
		fmt = s3c_camif_find_format(&pix->pixelformat, NULL, -1);
		size = (pix->width * pix->height * fmt->depth) / 8;
	} else {
		size = (frame->f_width * frame->f_height * fmt->depth) / 8;
	}

	if (fmt == NULL)
		return -EINVAL;
	*num_planes = 1;

	if (pix)
		sizes[0] = max(size, pix->sizeimage);
	else
		sizes[0] = size;
	allocators[0] = camif->alloc_ctx;

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct camif_vp *vp = vb2_get_drv_priv(vb->vb2_queue);

	pr_debug("\n");
	if (vp->out_fmt == NULL)
		return -EINVAL;

	if (vb2_plane_size(vb, 0) < vp->payload) {
		v4l2_err(&vp->vdev, "User buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), vp->payload);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, vp->payload);

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct camif_buffer *buf = container_of(vb, struct camif_buffer, vb);
	struct camif_vp *vp = vb2_get_drv_priv(vb->vb2_queue);
	struct camif_dev *camif = vp->camif;
	unsigned long flags;

	//pr_info("\n");
	spin_lock_irqsave(&camif->slock, flags);
	camif_prepare_addr(vp, &buf->vb, &buf->paddr);

	if (!test_bit(ST_CAMIF_SUSPENDED, &camif->state) &&
	    !test_bit(ST_CAMIF_STREAM, &camif->state) &&
	    vp->active_buf_count < CAMIF_MAX_OUT_BUFS) {
		/* Schedule an empty buffer in H/W */
		buf->index = vp->buf_index;

		//pr_info("buf->index: %d\n", buf->index);
		camif_hw_set_output_addr(vp, &buf->paddr, buf->index);

		camif_active_queue_add(vp, buf);
		if (++vp->buf_index >= CAMIF_MAX_OUT_BUFS)
			vp->buf_index = 0;
	} else {
		camif_pending_queue_add(vp, buf);
	}

	if (vb2_is_streaming(&vp->vb_queue) &&
	    vp->active_buf_count >= CAMIF_REQ_BUFS_MIN &&
	    !test_and_set_bit(ST_CAMIF_STREAM, &camif->state)) {
		camif_activate_capture(vp);
		spin_unlock_irqrestore(&camif->slock, flags);

		if (!test_and_set_bit(ST_SENSOR_STREAM, &camif->state)) {
			camif_hw_dump_regs(camif, __func__);
			v4l2_subdev_call(camif->sensor, video, s_stream, 1);
		}
		return;
	}
	spin_unlock_irqrestore(&camif->slock, flags);
}

static void camif_lock(struct vb2_queue *vq)
{
	struct camif_vp *vp = vb2_get_drv_priv(vq);
	//	pr_info("\n");
	mutex_lock(&vp->camif->lock);
}

static void camif_unlock(struct vb2_queue *vq)
{
	struct camif_vp *vp = vb2_get_drv_priv(vq);
	//	pr_info("\n");
	mutex_unlock(&vp->camif->lock);
}

static const struct vb2_ops s3c_camif_qops = {
	.queue_setup	 = queue_setup,
	.buf_prepare	 = buffer_prepare,
	.buf_queue	 = buffer_queue,
	.wait_prepare	 = camif_unlock,
	.wait_finish	 = camif_lock,
	.start_streaming = start_streaming,
	.stop_streaming	 = stop_streaming,
};

static int s3c_camif_open(struct file *file)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	int ret;

	pr_info("\n");
	if (mutex_lock_interruptible(&camif->lock))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	set_bit(ST_CAMIF_IN_USE, &camif->state);
	//ret = pm_runtime_get_sync(camif->dev);
	if (ret < 0) {
		v4l2_fh_release(file);
		if (vp->ref_count == 0)
			clear_bit(ST_CAMIF_IN_USE, &camif->state);
		goto unlock;
	}

	ret = v4l2_subdev_call(camif->sensor, core, s_power, 1);
	if (ret < 0) {
		v4l2_fh_release(file);
		if (vp->ref_count == 0)
			clear_bit(ST_CAMIF_IN_USE, &camif->state);
		goto unlock;
	}

	//camif_hw_reset(camif); /* DEBUG! */
	//camif_hw_clear_fifo_overflow(vp);

	vp->ref_count++;
unlock:
	mutex_unlock(&camif->lock);
	return ret;
}

static int s3c_camif_close(struct file *file)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	int ret;

	pr_info("\n");
	if (mutex_lock_interruptible(&camif->lock))
		return -ERESTARTSYS;

	if (--vp->ref_count == 0) {
		clear_bit(ST_CAMIF_IN_USE, &camif->state);
		s3c_camif_stop_capture(vp, false);
		v4l2_subdev_call(camif->sensor, video, s_stream, 0);
		v4l2_subdev_call(camif->sensor, core, s_power, 0);
		clear_bit(ST_CAMIF_SUSPENDED, &camif->state);
	}

//	pm_runtime_put(camif->dev);

	if (vp->ref_count == 0)
		vb2_queue_release(&vp->vb_queue);

	ret = v4l2_fh_release(file);
	mutex_unlock(&camif->lock);
	return ret;
}

static unsigned int s3c_camif_poll(struct file *file,
				   struct poll_table_struct *wait)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	int ret;

	if (mutex_lock_interruptible(&camif->lock))
		return -ERESTARTSYS;

	ret = vb2_poll(&vp->vb_queue, file, wait);
	mutex_unlock(&camif->lock);

	return ret;
}

static int s3c_camif_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	int ret;

	pr_info("\n");
	if (mutex_lock_interruptible(&camif->lock))
		return -ERESTARTSYS;

	ret = vb2_mmap(&vp->vb_queue, vma);
	mutex_unlock(&camif->lock);

	return ret;
}

static const struct v4l2_file_operations s3c_camif_fops = {
	.owner		= THIS_MODULE,
	.open		= s3c_camif_open,
	.release	= s3c_camif_close,
	.poll		= s3c_camif_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= s3c_camif_mmap,
};

/*
 * Video node IOCTLs
 */

static int s3c_camif_vidioc_querycap(struct file *file, void *priv,
				     struct v4l2_capability *cap)
{
	pr_info("\n");
	strlcpy(cap->driver, S3C_CAMIF_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, S3C_CAMIF_DRIVER_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform", sizeof(cap->bus_info));

	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_STREAMING;
	return 0;
}

static int s3c_camif_vidioc_enum_input(struct file *file, void *priv,
				       struct v4l2_input *input)
{
	struct camif_vp *vp = video_drvdata(file);
	struct v4l2_subdev *sensor = vp->camif->sensor;

	pr_info("\n");
	if (input->index || sensor == NULL)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(input->name, sensor->name, sizeof(input->name));
	return 0;
}

static int s3c_camif_vidioc_s_input(struct file *file, void *priv,
				    unsigned int i)
{
	pr_info("\n");
	return i == 0 ? 0 : -EINVAL;
}

static int s3c_camif_vidioc_g_input(struct file *file, void *priv,
				    unsigned int *i)
{
	pr_info("\n");
	*i = 0;
	return 0;
}

static int s3c_camif_vidioc_enum_fmt(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	const struct camif_fmt *fmt;

	pr_info("fmt index: %d\n", f->index);
	fmt = s3c_camif_find_format(NULL, NULL, f->index);
	if (!fmt)
		return -EINVAL;

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	pr_info("fmt (%d): %s\n", f->index, f->description);
	return 0;
}

static int s3c_camif_vidioc_g_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct camif_vp *vp = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct camif_frame *frame = &vp->out_frame;
	const struct camif_fmt *fmt = vp->out_fmt;

	pr_info("\n");
	pix->bytesperline = (frame->f_width * fmt->depth) / 8;
	pix->sizeimage = pix->bytesperline * frame->f_height;

	pix->pixelformat = fmt->fourcc;
	pix->width = frame->f_width;
	pix->height = frame->f_height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static int __camif_video_try_format(struct camif_vp *vp,
				    struct v4l2_pix_format *pix,
				    const struct camif_fmt **ffmt)
{
	struct camif_dev *camif = vp->camif;
	const struct s3c_camif_pix_limits *pix_lim;
	u32 bpl = pix->bytesperline;
	const struct camif_fmt *fmt;

	fmt = s3c_camif_find_format(&pix->pixelformat, NULL, 0);
	if (WARN_ON(fmt == NULL))
		return -EINVAL;
	if (ffmt)
		*ffmt = fmt;

	pix_lim = &camif->variant->pix_limits[vp->id];

	pr_info("%dx%d (max width: %d)\n", pix->width, pix->height,
		pix_lim->max_sc_out_width);

	v4l_bound_align_image(&pix->width, 8, pix_lim->max_sc_out_width,
			      ffs(pix_lim->out_width_align) - 1,
			      &pix->height, 8, pix_lim->max_height, 0, 0);

	pr_info("%dx%d\n", pix->width, pix->height);

	if ((bpl == 0 || ((bpl * 8) / fmt->depth) < pix->width))
		pix->bytesperline = (pix->width * fmt->depth) / 8;

	if (pix->sizeimage == 0)
		pix->sizeimage = (pix->width * pix->height * fmt->depth) / 8;

	pix->pixelformat = fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->field = V4L2_FIELD_NONE;
	return 0;
}

static int s3c_camif_vidioc_try_fmt(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_info("\n");
	return __camif_video_try_format(vp, &f->fmt.pix, NULL);
}

static int s3c_camif_vidioc_s_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct camif_vp *vp = video_drvdata(file);
	struct camif_frame *inp_frame = &vp->camif->inp_frame;
	struct camif_frame *out_frame = &vp->out_frame;
	const struct camif_fmt *fmt = NULL;
	int ret;

	pr_info("\n");
	if (vb2_is_busy(&vp->vb_queue))
		return -EBUSY;

	ret = __camif_video_try_format(vp, &f->fmt.pix, &fmt);
	if (ret < 0)
		return ret;

	vp->out_fmt = fmt;
	vp->payload = (pix->width * pix->height * fmt->depth) / 8;
	//	vp->payload = pix->width * pix->bytesperline;
	out_frame->f_width = pix->width;
	out_frame->f_height = pix->height;

	pr_info("%dx%d. payload: %lu. fmt: %s. %d %d. sizeimage: %d. bpl: %d\n",
		out_frame->f_width, out_frame->f_height, vp->payload, fmt->name,
		pix->width * pix->height * fmt->depth, fmt->depth,
		pix->sizeimage, pix->bytesperline);
	/* DEBUG */
#if 0
	if (pix->width == inp_frame->f_width &&
	    pix->height == inp_frame->f_height)
		vp->scaler.enable = 1; // !!
	else
		vp->scaler.enable = 1;
#endif
	return 0;
}

static int camif_pipeline_validate(struct camif_dev *camif)
{
#if 0
	struct v4l2_subdev *sd = &camif->subdev;
	struct v4l2_subdev_format sink_fmt, src_fmt;
	struct media_pad *pad;
	int ret;

	while (1) {
		/* Retrieve format at the sink pad */
		pad = &sd->entity.pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;
		/* Don't call CAMIF subdev operation to avoid nested locking */
		if (sd == &camif->subdev) {
			struct camif_frame *ff = &camif->out_frame;
			sink_fmt.format.width = ff->f_width;
			sink_fmt.format.height = ff->f_height;
			sink_fmt.format.code = camif->fmt->mbus_code;
		} else {
			sink_fmt.pad = pad->index;
			sink_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
			ret = v4l2_subdev_call(sd, pad, get_fmt, NULL,
					       &sink_fmt);
			if (ret < 0 && ret != -ENOIOCTLCMD)
				return -EPIPE;
		}
		/* Retrieve format at the source pad */
		pad = media_entity_remote_source(pad);
		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		sd = media_entity_to_v4l2_subdev(pad->entity);
		src_fmt.pad = pad->index;
		src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &src_fmt);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return -EPIPE;

		if (src_fmt.format.width != sink_fmt.format.width ||
		    src_fmt.format.height != sink_fmt.format.height ||
		    src_fmt.format.code != sink_fmt.format.code)
			return -EPIPE;
	}
#endif
	return 0;
}

static int s3c_camif_streamon(struct file *file, void *priv,
			      enum v4l2_buf_type type)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	struct media_entity *sensor = &camif->sensor->entity;
	int ret;

	pr_info("\n");
	if (s3c_camif_active(camif))
		return -EBUSY;

	ret = media_entity_pipeline_start(sensor, camif->m_pipeline);
	if (ret < 0)
		return ret;

	ret = camif_pipeline_validate(camif);
	if (ret < 0) {
		media_entity_pipeline_stop(sensor);
		return ret;
	}

	return vb2_streamon(&vp->vb_queue, type);
}

static int s3c_camif_streamoff(struct file *file, void *priv,
			       enum v4l2_buf_type type)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	int ret;

	pr_info("\n");
	ret = vb2_streamoff(&vp->vb_queue, type);
	if (ret == 0)
		media_entity_pipeline_stop(&camif->sensor->entity);
	return ret;
}

static int s3c_camif_reqbufs(struct file *file, void *priv,
			     struct v4l2_requestbuffers *rb)
{
	struct camif_vp *vp = video_drvdata(file);
	int ret;

	pr_info("rb count: %d\n", rb->count);
	rb->count = max_t(u32, CAMIF_REQ_BUFS_MIN, rb->count);

	ret = vb2_reqbufs(&vp->vb_queue, rb);
	if (ret == 0)
		vp->reqbufs_count = rb->count;

	pr_info("rb count adjusted by the driver: %d\n", rb->count);
	return ret;
}

static int s3c_camif_querybuf(struct file *file, void *priv,
			      struct v4l2_buffer *buf)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_info("\n");
	return vb2_querybuf(&vp->vb_queue, buf);
}

static int s3c_camif_qbuf(struct file *file, void *priv,
			  struct v4l2_buffer *buf)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_debug("\n");
	return vb2_qbuf(&vp->vb_queue, buf);
}

static int s3c_camif_dqbuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_debug("\n");
	return vb2_dqbuf(&vp->vb_queue, buf, file->f_flags & O_NONBLOCK);
}

static int s3c_camif_create_bufs(struct file *file, void *priv,
				 struct v4l2_create_buffers *create)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_info("\n");
	return vb2_create_bufs(&vp->vb_queue, create);
}

static int s3c_camif_prepare_buf(struct file *file, void *priv,
				 struct v4l2_buffer *b)
{
	struct camif_vp *vp = video_drvdata(file);
	pr_info("\n");
	return vb2_prepare_buf(&vp->vb_queue, b);
}

static int s3c_camif_g_selection(struct file *file, void *priv,
				 struct v4l2_selection *sel)
{
	struct camif_vp *vp = video_drvdata(file);

	pr_info("\n");

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = vp->out_frame.f_width;
		sel->r.height = vp->out_frame.f_height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE_ACTIVE:
		sel->r = vp->out_frame.rect;
		return 0;
	}

	return -EINVAL;
}

static void __camif_try_compose(struct camif_dev *camif, struct camif_vp *vp,
				struct v4l2_rect *r)
{
	struct camif_frame *frame = &vp->out_frame;
	//struct v4l2_rect *comp_rect = &vp->out_frame.rect;

	pr_info("\n");

	/* s3c244x doesn't support composition */
	if (!camif->variant->ip_revision == 32) {
		//*r = vp->out_frame.rect;
		return;
	}

	/* Adjust left/top if the composing rectangle got out of bounds */
	r->left = clamp_t(u32, r->left, 0, frame->f_width - r->width);
	//r->left = round_down(r->left, camif->variant->out_hor_offs_align);
	r->top  = clamp_t(u32, r->top, 0, vp->out_frame.f_height - r->height);

	v4l2_dbg(1, debug, &camif->v4l2_dev, "(%d,%d)/%dx%d, source fmt: %dx%d",
		 r->left, r->top, r->width, r->height,
		 frame->f_width, frame->f_height);
}

static int s3c_camif_s_selection(struct file *file, void *priv,
				 struct v4l2_selection *sel)
{
	struct camif_vp *vp = video_drvdata(file);
	struct camif_dev *camif = vp->camif;
	struct v4l2_rect rect = sel->r;
	unsigned long flags;

	pr_info("\n");

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    sel->target != V4L2_SEL_TGT_COMPOSE_ACTIVE)
		return -EINVAL;

	__camif_try_compose(camif, vp, &rect);

	sel->r = rect;
	spin_lock_irqsave(&camif->slock, flags);
	vp->out_frame.rect = rect;
	set_bit(ST_CAMIF_CONFIG, &camif->state);
	spin_unlock_irqrestore(&camif->slock, flags);

	return 0;
}

static const struct v4l2_ioctl_ops s3c_camif_ioctl_ops = {
	.vidioc_querycap	 = s3c_camif_vidioc_querycap,
	.vidioc_enum_input	 = s3c_camif_vidioc_enum_input,
	.vidioc_g_input		 = s3c_camif_vidioc_g_input,
	.vidioc_s_input		 = s3c_camif_vidioc_s_input,
	.vidioc_enum_fmt_vid_cap = s3c_camif_vidioc_enum_fmt,
	.vidioc_try_fmt_vid_cap	 = s3c_camif_vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap	 = s3c_camif_vidioc_s_fmt,
	.vidioc_g_fmt_vid_cap	 = s3c_camif_vidioc_g_fmt,
	.vidioc_g_selection	 = s3c_camif_g_selection,
	.vidioc_s_selection	 = s3c_camif_s_selection,
	.vidioc_reqbufs		 = s3c_camif_reqbufs,
	.vidioc_querybuf	 = s3c_camif_querybuf,
	.vidioc_prepare_buf	 = s3c_camif_prepare_buf,
	.vidioc_create_bufs	 = s3c_camif_create_bufs,
	.vidioc_qbuf		 = s3c_camif_qbuf,
	.vidioc_dqbuf		 = s3c_camif_dqbuf,
	.vidioc_streamon	 = s3c_camif_streamon,
	.vidioc_streamoff	 = s3c_camif_streamoff,
};

static int s3c_camif_video_s_ctrl(struct v4l2_ctrl *ctrl)
{
	/* struct camif_dev *camif = container_of(ctrl->handler, struct camif_dev, */
	/* 				       ctrl_handler); */
	pr_info("\n");
	/* set_bit(ST_CAMIF_CONFIG, &camif->state); */
	return 0;
}

/* Codec and preview video node control ops */
static const struct v4l2_ctrl_ops s3c_camif_video_ctrl_ops = {
	.s_ctrl = s3c_camif_video_s_ctrl,
};

int s3c_camif_register_video_node(struct camif_dev *camif, int idx)
{
	struct camif_vp *vp = &camif->vp[idx];
	struct v4l2_ctrl_handler *ctrl_handler = &vp->ctrl_handler;
	struct vb2_queue *q = &vp->vb_queue;
	struct video_device *vfd = &vp->vdev;
	int ret;

	pr_info("\n");

	vp->id = idx;
	vp->camif = camif;
	vp->out_fmt = s3c_camif_find_format(NULL, NULL, 0);

	snprintf(vfd->name, sizeof(vfd->name), "camif-%s",
		 vp->id == 0 ? "codec" : "preview");

	vfd->fops = &s3c_camif_fops;
	vfd->ioctl_ops = &s3c_camif_ioctl_ops;
	vfd->v4l2_dev = &camif->v4l2_dev;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->lock = &camif->lock;
	vp->ref_count = 0;
	vp->reqbufs_count = 0;

	INIT_LIST_HEAD(&vp->pending_buf_q);
	INIT_LIST_HEAD(&vp->active_buf_q);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->ops = &s3c_camif_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct camif_buffer);
	q->drv_priv = vp;

	vb2_queue_init(q);

	vp->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vfd->entity, 1, &vp->pad, 0);
	if (ret)
		goto err;

	video_set_drvdata(vfd, vp);

	v4l2_ctrl_handler_init(ctrl_handler, 1);
	v4l2_ctrl_new_std(ctrl_handler, &s3c_camif_video_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	ret = ctrl_handler->error;
	if (ret < 0)
		goto err_ctrl;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto err_vd;

	vfd->ctrl_handler = ctrl_handler;

	v4l2_info(&camif->v4l2_dev, "registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));
	return 0;

 err_vd:
	v4l2_ctrl_handler_free(ctrl_handler);
 err_ctrl:
	media_entity_cleanup(&vfd->entity);
 err:
	video_device_release(vfd);
	return ret;
}

void s3c_camif_unregister_video_node(struct camif_dev *camif, int idx)
{
	struct video_device *vfd = &camif->vp[idx].vdev;

	pr_info("\n");
	if (vfd) {
		video_unregister_device(vfd);
		media_entity_cleanup(&vfd->entity);
		v4l2_ctrl_handler_free(vfd->ctrl_handler);
	}
}

/*
 *  Camera input interface subdev operations
 */

static int s3c_camif_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_mbus_code_enum *code)
{
	const struct camif_fmt *fmt;

	pr_info("\n");

	fmt = s3c_camif_find_format(NULL, NULL, code->index);
	if (!fmt)
		return -EINVAL;
	code->code = fmt->mbus_code;
	return 0;
}

static int s3c_camif_subdev_get_fmt(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_format *fmt)
{
	struct camif_dev *camif = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct camif_frame *f = &camif->inp_frame;

	pr_info("\n");

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
		return 0;
	}
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	mutex_lock(&camif->lock);
	mf->code = camif->inp_fmt->mbus_code;
	pr_info("code: 0x%04x\n", mf->code);

	/* FIXME */
	switch (fmt->pad) {
	case CAMIF_SD_PAD_SINK:
		/* full camera input frame size */
		mf->width = f->f_width;
		mf->height = f->f_height;
		break;
	case CAMIF_SD_PAD_SOURCE:
		/* crop rectangle at camera interface input */
		mf->width = f->rect.width;
		mf->height = f->rect.height;
		break;
	}
	mutex_unlock(&camif->lock);
	return 0;
}

static const struct camif_fmt *__camif_subdev_try_format(struct camif_dev *camif,
					 struct v4l2_mbus_framefmt *mf, int pad)
{
	const struct s3c_camif_variant *variant = camif->variant;
	const struct s3c_camif_pix_limits *pix_lim;
	const struct camif_fmt *fmt;

	pr_info("\n");
	/* FIXME */
	pix_lim = &variant->pix_limits[VP_CODEC];

	fmt = s3c_camif_find_format(NULL, &mf->code, 0);
	BUG_ON(fmt == NULL);
	mf->code = fmt->mbus_code;

	if (pad == CAMIF_SD_PAD_SINK) {
		v4l_bound_align_image(&mf->width, 8, CAMIF_MAX_PIX_WIDTH,
				      ffs(pix_lim->out_width_align) - 1,
				      &mf->height, 8, CAMIF_MAX_PIX_HEIGHT, 0,
				      0);
	} else {
		struct v4l2_rect *r = &camif->inp_frame.rect;
		v4l_bound_align_image(&mf->width, 8, r->width,
				      ffs(pix_lim->out_width_align) - 1,
				      &mf->height, 8, r->height,
				      0, 0);
	}

	v4l2_dbg(1, debug, &camif->subdev, "%dx%d\n", mf->width, mf->height);

	return fmt;
}

static int s3c_camif_subdev_set_fmt(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_format *fmt)
{
	struct camif_dev *camif = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct camif_frame *sink = &camif->inp_frame;
	const struct camif_fmt *ffmt;

	v4l2_dbg(1, debug, sd, "pad%d: code: 0x%x, %dx%d",
		 fmt->pad, mf->code, mf->width, mf->height);

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mutex_lock(&camif->lock);

	if (vb2_is_busy(&camif->vp[VP_CODEC].vb_queue) ||
	    vb2_is_busy(&camif->vp[VP_PREVIEW].vb_queue)) {
		mutex_unlock(&camif->lock);
		return -EBUSY;
	}

	ffmt = __camif_subdev_try_format(camif, mf, fmt->pad);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		*mf = fmt->format;
		mutex_unlock(&camif->lock);
		return 0;
	}

	switch (fmt->pad) {
	case CAMIF_SD_PAD_SINK:
		sink->f_width = mf->width;
		sink->f_height = mf->height;
		camif->inp_fmt = ffmt;
		/* Set sink crop rectangle */
		sink->rect.width = mf->width;
		sink->rect.height = mf->height;
		sink->rect.left = 0;
		sink->rect.top = 0;
		/* Reset source crop rectangle */
		camif->vp[VP_CODEC].out_frame.rect = sink->rect;
		camif->vp[VP_PREVIEW].out_frame.rect = sink->rect;
		break;
	case CAMIF_SD_PAD_SOURCE:
		/* FIXME */
		/* Allow changing format only on sink pad */
		mf->code = camif->inp_fmt->mbus_code;
		mf->width = sink->rect.width;
		mf->height = sink->rect.height;
		break;
	}

	mutex_unlock(&camif->lock);
	return 0;
}

static int s3c_camif_subdev_get_selection(struct v4l2_subdev *sd,
					  struct v4l2_subdev_fh *fh,
					  struct v4l2_subdev_selection *sel)
{
	struct camif_dev *camif = v4l2_get_subdevdata(sd);
	struct camif_frame *f = &camif->inp_frame;

	if ((sel->target != V4L2_SUBDEV_SEL_TGT_CROP_ACTUAL &&
	    sel->target != V4L2_SUBDEV_SEL_TGT_CROP_BOUNDS) ||
	    sel->pad != CAMIF_SD_PAD_SINK)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = *v4l2_subdev_get_try_crop(fh, sel->pad);
		return 0;
	}

	mutex_lock(&camif->lock);
	if (sel->target == V4L2_SUBDEV_SEL_TGT_CROP_ACTUAL) {
		sel->r = f->rect;
	} else {
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = f->f_width;
		sel->r.height = f->f_height;
	}
	mutex_unlock(&camif->lock);

	v4l2_dbg(1, debug, sd, "%s: (%d,%d) %dx%d, f_w: %d, f_h: %d",
		 __func__, f->rect.left, f->rect.top, f->rect.width,
		 f->rect.height, f->f_width, f->f_height);

	return 0;
}

static void __camif_try_crop(struct camif_dev *camif, struct v4l2_rect *r)
{
	struct camif_frame *frame = &camif->inp_frame;

	v4l_bound_align_image(&r->width, 0, frame->f_width, 0,
			      &r->height, 0, frame->f_height, 0, 0);

	/* Adjust left/top if cropping rectangle got out of bounds */
	r->left = clamp_t(u32, r->left, 0, frame->f_width - r->width);
	r->left = round_down(r->left, 8 /* FIXME */);
	/* camif->variant->win_hor_offs_align); */
	r->top  = clamp_t(u32, r->top, 0, frame->f_height - r->height);

	v4l2_dbg(1, debug, &camif->v4l2_dev, "(%d,%d)/%dx%d, sink fmt: %dx%d",
		 r->left, r->top, r->width, r->height,
		 frame->f_width, frame->f_height);
}

static int s3c_camif_subdev_set_selection(struct v4l2_subdev *sd,
					  struct v4l2_subdev_fh *fh,
					  struct v4l2_subdev_selection *sel)
{
	struct camif_dev *camif = v4l2_get_subdevdata(sd);
	struct camif_frame *f = &camif->inp_frame;

	if (sel->target != V4L2_SUBDEV_SEL_TGT_CROP_ACTUAL ||
	    sel->pad != CAMIF_SD_PAD_SINK)
		return -EINVAL;

	mutex_lock(&camif->lock);
	__camif_try_crop(camif, &sel->r);

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_crop(fh, sel->pad) = sel->r;
	} else {
		unsigned long flags;
		spin_lock_irqsave(&camif->slock, flags);
		f->rect = sel->r;
		/* Set same crop rectangle on the capture video nodes */
		/* FIXME */
		camif->vp[VP_CODEC].out_frame.rect = sel->r;
		camif->vp[VP_PREVIEW].out_frame.rect = sel->r;
		set_bit(ST_CAMIF_CONFIG, &camif->state);
		spin_unlock_irqrestore(&camif->slock, flags);
	}
	mutex_unlock(&camif->lock);

	v4l2_dbg(1, debug, sd, "%s: (%d,%d) %dx%d, f_w: %d, f_h: %d",
		 __func__, f->rect.left, f->rect.top, f->rect.width,
		 f->rect.height, f->f_width, f->f_height);

	return 0;
}

static const struct v4l2_subdev_pad_ops s3c_camif_subdev_pad_ops = {
	.enum_mbus_code = s3c_camif_subdev_enum_mbus_code,
	.get_selection = s3c_camif_subdev_get_selection,
	.set_selection = s3c_camif_subdev_set_selection,
	.get_fmt = s3c_camif_subdev_get_fmt,
	.set_fmt = s3c_camif_subdev_set_fmt,
};

static struct v4l2_subdev_ops s3c_camif_subdev_ops = {
	.pad = &s3c_camif_subdev_pad_ops,
};

static int s3c_camif_subdev_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camif_dev *camif = container_of(ctrl->handler, struct camif_dev,
					       ctrl_handler);
	pr_info("\n");
	set_bit(ST_CAMIF_CONFIG, &camif->state);
	return 0;
}

static const struct v4l2_ctrl_ops s3c_camif_subdev_ctrl_ops = {
	.s_ctrl	= s3c_camif_subdev_s_ctrl,
};

static const struct v4l2_ctrl_config s3c_camif_priv_ctrl = {
	.ops	= &s3c_camif_subdev_ctrl_ops,
	.id	= V4L2_CTRL_CLASS_USER | 0x1001,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "Test Pattern",
	.min	= 0,
	.max	= 3,
	.step	= 1,
	.def	= 0,
	/* .is_private = 1, */
};

int s3c_camif_create_subdev(struct camif_dev *camif)
{
	struct v4l2_ctrl_handler *handler = &camif->ctrl_handler;
	struct v4l2_subdev *sd = &camif->subdev;
	int ret;

	pr_info("\n");

	v4l2_subdev_init(sd, &s3c_camif_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strlcpy(sd->name, "S3C-CAMIF", sizeof(sd->name));

	camif->pads[CAMIF_SD_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	camif->pads[CAMIF_SD_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&sd->entity, CAMIF_SD_PADS_NUM,
				camif->pads, 0);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(handler, 1);
	camif->test_pattern = v4l2_ctrl_new_custom(handler,
					&s3c_camif_priv_ctrl, NULL);
	if (handler->error) {
		media_entity_cleanup(&sd->entity);
		return handler->error;
	}

	sd->ctrl_handler = handler;
	v4l2_set_subdevdata(sd, camif);

	return 0;
}

void s3c_camif_unregister_subdev(struct camif_dev *camif)
{
	struct v4l2_subdev *sd = &camif->subdev;

	pr_info("\n");
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&camif->ctrl_handler);
	v4l2_set_subdevdata(sd, NULL);
}

int s3c_camif_set_defaults(struct camif_dev *camif)
{
	struct camif_frame *f;
	int i;

	for (i = 0; i < CAMIF_VP_NUM; i++) {
		struct camif_vp *vp = &camif->vp[VP_CODEC];
		vp->out_fmt = s3c_camif_find_format(NULL, NULL, 0);

		f = &vp->out_frame;
		f->f_width = CAMIF_DEF_WIDTH;
		f->f_height = CAMIF_DEF_HEIGHT;
		f->rect.width = CAMIF_DEF_WIDTH;
		f->rect.height = CAMIF_DEF_HEIGHT;
		/* Scaler is always enabled */
		vp->scaler.enable = 1;
	}

	f = &camif->inp_frame;
	f->f_width = CAMIF_DEF_WIDTH;
	f->f_height = CAMIF_DEF_HEIGHT;
	f->rect.width = CAMIF_DEF_WIDTH;
	f->rect.height = CAMIF_DEF_HEIGHT;

	camif->inp_fmt = s3c_camif_find_format(NULL, NULL, 0);

	return 0;
}
