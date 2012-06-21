/*
 * s3c24xx/s3c64xx SoC series Camera Interface (CAMIF) driver
 *
 * Copyright (c) 2012, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "camif-core.h"

static char *camif_clocks[CLK_MAX_NUM] = {
	"camif-upll", "camif",
};

static const struct camif_fmt camif_formats[] = {
	{
		.name		= "YUV 4:2:2 planar, Y/Cb/YCr",
		.fourcc		= V4L2_PIX_FMT_YUV422P,
		.depth		= 16,
		.color		= IMG_FMT_YCBCR422P,
		.colplanes	= 3,
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
	}, {
		.name		= "YUV 4:2:0 planar, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.depth		= 12,
		.color		= IMG_FMT_YCBCR420,
		.colplanes	= 3,
#if 0
	}, {
		.name		= "RGB565, 16 bpp",
		.fourcc		= V4L2_PIX_FMT_RGB565X,
		.depth		= 16,
		.color		= IMG_FMT_RGB565,
		.colplanes	= 1,
	}, {
		.name		= "BGR666",
		.fourcc		= V4L2_PIX_FMT_BGR666,
		.depth		= 32,
		.color		= IMG_FMT_RGB666,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "XRGB8888, 32 bpp",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= 32,
		.color		= IMG_FMT_RGB888,
		.colplanes	= 1,
#endif
	}
};

/**
 * s3c_camif_find_format - lookup fimc color format by fourcc or media bus code
 * @pixelformat: fourcc to match, ignored if null
 * @mbus_code: media bus code to match, ignored if null
 * @index: index to the camif_formats array, ignored if negative
 */
const struct camif_fmt *s3c_camif_find_format(const u32 *pixelformat,
					      const u32 *mbus_code, int index)
{
	const struct camif_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
	int id = 0;

	if (index >= (int)ARRAY_SIZE(camif_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(camif_formats); ++i) {
		fmt = &camif_formats[i];
		if (pixelformat && fmt->fourcc == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == id)
			def_fmt = fmt;
		id++;
	}
	return def_fmt;
}

#if 0
int camif_check_scaler_ratio(struct camif_ctx *ctx, int sw, int sh,
			    int dw, int dh, int rotation)
{
	if (rotation == 90 || rotation == 270)
		swap(dw, dh);

	if (!ctx->scaler.enabled)
		return (sw == dw && sh == dh) ? 0 : -EINVAL;

	if ((sw >= SCALER_MAX_HRATIO * dw) || (sh >= SCALER_MAX_VRATIO * dh))
		return -EINVAL;

	return 0;
}
#endif

static int camif_get_scaler_factor(u32 src, u32 tar, u32 *ratio, u32 *shift)
{
	u32 sh = 6;

	if (src >= 64 * tar)
		return -EINVAL;

	while (sh--) {
		u32 tmp = 1 << sh;
		if (src >= tar * tmp) {
			*shift = sh, *ratio = tmp;
			return 0;
		}
	}
	*shift = 0, *ratio = 1;
	return 0;
}

int s3c_camif_set_scaler_info(struct camif_vp *vp)
{
	struct camif_dev *camif = vp->camif;
	//const struct camif_variant *variant = camif->variant;
	struct device *dev = camif->dev;
	struct camif_scaler *sc = &vp->scaler;
	struct camif_frame *s_frame = &camif->inp_frame;
	struct camif_frame *d_frame = &vp->out_frame;
	int tx, ty, sx, sy;
	int ret;

	/* if (ctx->rotation == 90 || ctx->rotation == 270) { */
	/* 	ty = d_frame->width; */
	/* 	tx = d_frame->height; */
	/* } else { */
		tx = d_frame->f_width;
		ty = d_frame->f_height;
	/* } */
	if (tx <= 0 || ty <= 0) {
		dev_err(dev, "Invalid target size: %dx%d", tx, ty);
		return -EINVAL;
	}

	sx = s_frame->rect.width;
	sy = s_frame->rect.height;
	if (sx <= 0 || sy <= 0) {
		dev_err(dev, "Invalid source size: %dx%d", sx, sy);
		return -EINVAL;
	}
	sc->real_width = sx;
	sc->real_height = sy;

	ret = camif_get_scaler_factor(sx, tx, &sc->pre_hratio, &sc->hfactor);
	if (ret)
		return ret;

	ret = camif_get_scaler_factor(sy, ty,  &sc->pre_vratio, &sc->vfactor);
	if (ret)
		return ret;

	sc->pre_dst_width = sx / sc->pre_hratio;
	sc->pre_dst_height = sy / sc->pre_vratio;

	pr_info("H: ratio: %d. shift: %d\n", sc->pre_hratio, sc->hfactor);
	pr_info("V: ratio: %d. shift: %d\n", sc->pre_vratio, sc->vfactor);
	pr_info("Source: %dx%d, Target: %dx%d\n", sx, sy, tx, ty);

	/* if (variant->has_mainscaler_ext) { */
	/* 	sc->main_hratio = (sx << 14) / (tx << sc->hfactor); */
	/* 	sc->main_vratio = (sy << 14) / (ty << sc->vfactor); */
	/* } else { */
		sc->main_hratio = (sx << 8) / (tx << sc->hfactor);
		sc->main_vratio = (sy << 8) / (ty << sc->vfactor);
	/* } */

	sc->scaleup_h = (tx >= sx) ? 1 : 0;
	sc->scaleup_v = (ty >= sy) ? 1 : 0;

	/* check to see if input and output size/format differ */
	/* if (s_frame->fmt->color == d_frame->fmt->color */
	/* 	&& s_frame->width == d_frame->width */
	/* 	&& s_frame->height == d_frame->height) */
	/* 	sc->copy_mode = 1; */
	/* else */
		sc->copy_mode = 0;

	return 0;
}

static int camif_register_sensor(struct camif_dev *camif)
{
	struct s3c_camif_sensor_info *sensor = &camif->pdata.sensor;
	struct v4l2_device *v4l2_dev = &camif->v4l2_dev;
	struct i2c_adapter *adapter;
	struct v4l2_subdev *sd;

	camif->sensor = NULL;

	if (sensor->i2c_board_info == NULL)
		return -EINVAL;

	adapter = i2c_get_adapter(sensor->i2c_bus_num);
	if (adapter == NULL) {
		v4l2_warn(v4l2_dev, "failed to get I2C adapter %d\n",
			  sensor->i2c_bus_num);
		return -EPROBE_DEFER;
	}

	sd = v4l2_i2c_new_subdev_board(v4l2_dev, adapter,
				       sensor->i2c_board_info, NULL);
	if (sd == NULL) {
		i2c_put_adapter(adapter);
		v4l2_warn(v4l2_dev, "failed to acquire subdev %s\n",
			  sensor->i2c_board_info->type);
		return -EPROBE_DEFER;
	}
	/* v4l2_set_subdev_hostdata(sd, sensor); */
	/* sd->grp_id = SENSOR_GROUP_ID; */
	camif->sensor = sd;

	v4l2_info(v4l2_dev, "registered sensor subdevice %s\n", sd->name);
	return 0;
}

static void camif_unregister_sensor(struct camif_dev *camif)
{
	struct v4l2_subdev *sd = camif->sensor;
	struct i2c_client *client = sd ? v4l2_get_subdevdata(sd) : NULL;
	struct i2c_adapter *adapter;

	if (client) {
		adapter = client->adapter;
		v4l2_device_unregister_subdev(sd);
		camif->sensor = NULL;
		i2c_unregister_device(client);
		if (adapter)
			i2c_put_adapter(adapter);
	}
}

static int camif_create_media_links(struct camif_dev *camif)
{
	return 0;
}

static int camif_register_video_nodes(struct camif_dev *camif)
{
	int ret = s3c_camif_register_video_node(camif, VP_CODEC);
	if (ret < 0)
		return ret;

	return s3c_camif_register_video_node(camif, VP_PREVIEW);
}

static void camif_unregister_video_nodes(struct camif_dev *camif)
{
	s3c_camif_unregister_video_node(camif, VP_CODEC);
	s3c_camif_unregister_video_node(camif, VP_PREVIEW);
}

static void camif_unregister_media_entities(struct camif_dev *camif)
{
	camif_unregister_video_nodes(camif);
	camif_unregister_sensor(camif);
	s3c_camif_unregister_subdev(camif);
}

/*
 * Media device
 */
static int camif_media_dev_register(struct camif_dev *camif)
{
	struct media_device *md = &camif->media_dev;
	struct v4l2_device *v4l2_dev = &camif->v4l2_dev;
	unsigned int ip_rev = camif->variant->ip_revision;
	int ret;

	snprintf(md->model, sizeof(md->model), "SAMSUNG S3C%s CAMIF",
		 ip_rev == S3C6410_CAMIF_IP_REV ? "6410" : "244X");
	strlcpy(md->bus_info, "platform", sizeof(md->bus_info));
	md->hw_revision = ip_rev;
	md->driver_version = KERNEL_VERSION(1,0,0);

	md->dev = camif->dev;

	strlcpy(v4l2_dev->name, "s3c-camif", sizeof(v4l2_dev->name));
	v4l2_dev->mdev = md;

	ret = v4l2_device_register(camif->dev, v4l2_dev);
	if (ret < 0)
		return ret;

	ret = media_device_register(md);
	if (ret < 0)
		v4l2_device_unregister(v4l2_dev);

	return ret;
}

static void camif_clk_put(struct camif_dev *camif)
{
	int i;

	for (i = 0; i < CLK_MAX_NUM; i++) {
		if (camif->clock[i])
			clk_put(camif->clock[i]);
	}
}

static int camif_clk_get(struct camif_dev *camif)
{
	int i;

	for (i = 0; i < CLK_MAX_NUM; i++) {
		camif->clock[i] = clk_get(camif->dev, camif_clocks[i]);
		if (!IS_ERR(camif->clock[i]))
			continue;
		dev_err(camif->dev, "failed to get clock: %s\n",
			camif_clocks[i]);
		return -ENXIO;
	}

	return 0;
}

/*
 * The CAMIF device has two relatively independent data processing paths
 * that can source data from memory or the common camera input frontend.
 * Register interrupts for each data processing path (camif_vp).
 */
static int camif_request_irqs(struct platform_device *pdev,
			      struct camif_dev *camif)
{
	struct resource *res;
	int ret, i;

	for (i = 0; i < CAMIF_VP_COUNT; i++) {
		struct camif_vp *vp = &camif->vp[i];

		init_waitqueue_head(&vp->irq_queue);

		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res) {
			dev_err(&pdev->dev, "failed to get IRQ %d\n", i);
			return -ENXIO;
		}
		pr_err("IRQ: %d\n", res->start);
		ret = devm_request_irq(&pdev->dev, res->start,
				       s3c_camif_irq_handler, 0,
				       dev_name(&pdev->dev), vp);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to install IRQ: %d\n", ret);
			break;
		}
	}

	return ret;
}

static int s3c_camif_configure_port(struct camif_dev *camif)
{
	const int *gpios = camif->variant->gpios;
	int i, ret;

	for (i = 0; i < CAMIF_NUM_GPIOS; i++) {
		ret = gpio_request(gpios[i], "camif");
		if (ret == 0) {
			ret = s3c_gpio_cfgpin(gpios[i], S3C_GPIO_SFN(2));
		} else {
			for (--i; i >= 0; i--)
				gpio_free(gpios[i]);
			dev_err(camif->dev, "Failed to configure GPIO %d\n", i);
			return ret;
		}
		s3c_gpio_setpull(gpios[i], S3C_GPIO_PULL_NONE);
	}

	return 0;
}

static int s3c_camif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s3c_camif_plat_data *pdata = dev->platform_data;
	struct s3c_camif_drvdata *drvdata;
	struct camif_dev *camif;
	struct resource *mres;
	int ret = 0;

	camif = devm_kzalloc(dev, sizeof(*camif), GFP_KERNEL);
	if (!camif)
		return -ENOMEM;

	spin_lock_init(&camif->slock);
	mutex_init(&camif->lock);

	camif->dev = dev;

	if (!dev->of_node) {
		/* if (pdata == NULL) */
		/* 	return -EINVAL; */
		if (pdata)
			camif->pdata = *pdata;
	}

	drvdata = (void *)platform_get_device_id(pdev)->driver_data;
	camif->variant = drvdata->variant;

	mres = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	camif->io_base = devm_request_and_ioremap(dev, mres);
	if (camif->io_base == NULL) {
		dev_err(dev, "Failed to obtain I/O memory\n");
		return -ENOENT;
	}

	ret = s3c_camif_configure_port(camif);
	if (ret < 0 )
		return ret;

	ret = camif_request_irqs(pdev, camif);
	if (ret < 0 )
		return ret;

	ret = s3c_camif_create_subdev(camif);
	if (ret < 0)
		return ret;

	ret = camif_clk_get(camif);
	if (ret < 0)
		goto err_clk;

	platform_set_drvdata(pdev, camif);

	clk_set_rate(camif->clock[CLK_CAM], 24000000);

	ret = clk_enable(camif->clock[CLK_GATE]);
	pr_err("CLK_GATE enable: %d, frequency= %lu\n", ret,
	       clk_get_rate(camif->clock[CLK_GATE]));

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err_pm;

	/* Initialize contiguous memory allocator */
	camif->alloc_ctx = vb2_dma_contig_init_ctx(dev);
	if (IS_ERR(camif->alloc_ctx)) {
		ret = PTR_ERR(camif->alloc_ctx);
		goto err_alloc;
	}

	ret = camif_media_dev_register(camif);
	if (ret < 0)
		goto err_mdev;

	ret = v4l2_device_register_subdev(&camif->v4l2_dev, &camif->subdev);
	if (ret < 0)
		goto err_sens;

	ret = camif_register_sensor(camif);
	if (ret < 0)
		goto err_sens;

	ret = camif_create_media_links(camif);
	if (ret < 0)
		goto err_sens;

	s3c_camif_set_defaults(camif);

	ret = v4l2_device_register_subdev_nodes(&camif->v4l2_dev);
	if (ret < 0)
		goto err_sens;

	ret = camif_register_video_nodes(camif);
	if (ret < 0)
		goto err_sens;

	pm_runtime_put(dev);
	return 0;

err_sens:
	media_device_unregister(&camif->media_dev);
	camif_unregister_media_entities(camif);
err_mdev:
	vb2_dma_contig_cleanup_ctx(camif->alloc_ctx);
err_alloc:
	pm_runtime_put(dev);
err_pm:
	camif_clk_put(camif);
err_clk:
	s3c_camif_unregister_subdev(camif);
	return ret;
}

static int __devexit s3c_camif_remove(struct platform_device *pdev)
{
	struct camif_dev *camif = platform_get_drvdata(pdev);

	s3c_camif_unregister_subdev(camif);
	media_device_unregister(&camif->media_dev);
	camif_unregister_media_entities(camif);
	/* clk_disable(camif->clock[CLK_GATE]); */
	camif_clk_put(camif);
	return 0;
}

static int s3c_camif_runtime_resume(struct device *dev)
{
	/* struct camif_dev *camif = dev_get_drvdata(dev); */
	/* clk_enable(camif->clock); */
	return 0;
}

static int s3c_camif_runtime_suspend(struct device *dev)
{
	/* struct camif_dev *camif = dev_get_drvdata(dev); */
	/* clk_disable(camif->clock); */
	return 0;
}

static const struct s3c_camif_variant s3c244x_camif_variant = {
	.pix_limits = {
		[VP_CODEC] = {
			.max_out_width		= 4096,
			.max_sc_out_width	= 2048,
			.out_width_align	= 8,	/* FIXME */
			.max_height		= 4096, /* FIXME */
		},
		[VP_PREVIEW] = {
			/* FIXME */
			.max_out_width		= 4096,
			.max_sc_out_width	= 2048,
			.out_width_align	= 8,	/* FIXME */
			.max_height		= 4096, /* FIXME */
		}
	},
	.ip_revision = S3C244X_CAMIF_IP_REV,
	.gpios = {
		S3C2410_GPJ(0), S3C2410_GPJ(1), S3C2410_GPJ(2),
		S3C2410_GPJ(3), S3C2410_GPJ(4), S3C2410_GPJ(5),
		S3C2410_GPJ(6), S3C2410_GPJ(7), S3C2410_GPJ(8),
		S3C2410_GPJ(9), S3C2410_GPJ(10), S3C2410_GPJ(11),
	},
};

static struct s3c_camif_drvdata s3c244x_camif_drvdata = {
	.variant	= &s3c244x_camif_variant,
	.bus_clk_freq	= 24000000UL,
};

static struct platform_device_id s3c_camif_driver_ids[] = {
	{
		.name		= "s3c2440-camif",
		.driver_data	= (unsigned long)&s3c244x_camif_drvdata,
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, s3c_camif_driver_ids);

static const struct dev_pm_ops s3c_camif_pm_ops = {
	.runtime_suspend	= s3c_camif_runtime_suspend,
	.runtime_resume		= s3c_camif_runtime_resume,
};

static struct platform_driver s3c_camif_driver = {
	.probe		= s3c_camif_probe,
	.remove		= __devexit_p(s3c_camif_remove),
	.id_table	= s3c_camif_driver_ids,
	.driver = {
		.name	= S3C_CAMIF_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &s3c_camif_pm_ops,
	}
};

module_platform_driver(s3c_camif_driver);

MODULE_AUTHOR("Sylwester Nawrocki <sylvester.nawrocki@gmail.com>");
MODULE_DESCRIPTION("s3c244x/s3c64xx SoC camera interface (CAMIF) driver");
MODULE_LICENSE("GPL");
