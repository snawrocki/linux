/*
 * V4L2 asynchronous subdevice registration API
 *
 * Copyright (C) 2012, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef V4L2_ASYNC_H
#define V4L2_ASYNC_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>

#include <media/v4l2-subdev.h>

struct device;
struct v4l2_device;

enum v4l2_async_bus_type {
	V4L2_ASYNC_BUS_PLATFORM,
	V4L2_ASYNC_BUS_I2C,
};

struct v4l2_async_hw_device {
	enum v4l2_async_bus_type bus_type;
	union {
		struct {
			const char *name;
		} platform;
		struct {
			int adapter_id;
			unsigned short address;
		} i2c;
	} match;
};

/**
 * struct v4l2_async_subdev - device descriptor
 * @hw:		this device descriptor
 * @list:	member in the group
 * @dev:	corresponding hardware device (I2C, platform,...)
 * @sdpd:	embedded subdevice platform data
 * @role:	this subdevice role in the video pipeline
 */
struct v4l2_async_subdev {
	struct v4l2_async_hw_device hw;
	struct list_head list;
	struct device *dev;
	struct v4l2_subdev_platform_data sdpd;
	enum v4l2_subdev_role role;
};

/**
 * struct v4l2_async_group - list of device descriptors
 * @list:		member in the v4l2 group list
 * @group:		head of device list
 * @done:		head of probed device list
 * @platform_notifier:	platform bus notifier block
 * @i2c_notifier:	I2C bus notifier block
 * @v4l2_dev:		link to the respective struct v4l2_device
 * @bind:		callback, called upon BUS_NOTIFY_BIND_DRIVER for each
 *			subdevice
 * @complete:		callback, called once after all subdevices in the group
 *			have been bound
 */
struct v4l2_async_group {
	struct list_head list;
	struct list_head group;
	struct list_head done;
	struct notifier_block platform_notifier;
	struct notifier_block i2c_notifier;
	struct v4l2_device *v4l2_dev;
	int (*bind)(struct v4l2_async_group *group,
		    struct v4l2_async_subdev *asd);
	int (*bound)(struct v4l2_async_group *group,
		    struct v4l2_async_subdev *asd);
	int (*complete)(struct v4l2_async_group *group);
};

int v4l2_async_group_init(struct v4l2_device *v4l2_dev,
			  struct v4l2_async_group *group,
			  struct v4l2_async_subdev *asd, int cnt);
int v4l2_async_group_probe(struct v4l2_async_group *group);
void v4l2_async_group_release(struct v4l2_async_group *group);

#endif
