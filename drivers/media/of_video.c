/*
 * OF helpers for the Video4Linux API
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 * Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/of_video.h>
#include <media/v4l2-mediabus.h>

/**
 * v4l2_mbus_of_get_polarity - returns value of 'video-bus-polarity' property
 * @node: device tree node to parse
 * @flags: output media bus polarity flags (V4L2_MBUS_*)
 *
 * Returns 0 on success, or an error code if the property was not found
 * or couldn't be read.
 */
int v4l2_mbus_of_get_polarity(struct device_node *node, unsigned int *flags)
{
	unsigned int tmp = 0;
	int ret;
	u32 val;

	ret = of_property_read_u32(node, "video-bus-polarity", &val);
	if (ret < 0)
		return ret;

	if (val & 0x01)
		tmp |= V4L2_MBUS_HSYNC_ACTIVE_HIGH;
	else
		tmp |= V4L2_MBUS_HSYNC_ACTIVE_LOW;
	if (val & 0x02)
		tmp |= V4L2_MBUS_VSYNC_ACTIVE_HIGH;
	else
		tmp |= V4L2_MBUS_VSYNC_ACTIVE_LOW;
	if (val & 0x04)
		tmp |= V4L2_MBUS_PCLK_SAMPLE_RISING;
	else
		tmp |= V4L2_MBUS_PCLK_SAMPLE_FALLING;
	if (val & 0x08)
		tmp |= V4L2_MBUS_DATA_ACTIVE_HIGH;
	else
		tmp |= V4L2_MBUS_DATA_ACTIVE_LOW;
	if (val & 0x10)
		tmp |= V4L2_MBUS_FIELD_EVEN_HIGH;
	else
		tmp |= V4L2_MBUS_FIELD_EVEN_LOW;

	*flags = tmp;

	return 0;
}
