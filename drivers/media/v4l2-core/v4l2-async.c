/*
 * V4L2 asynchronous subdevice registration API
 *
 * Copyright (C) 2012, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

static bool match_i2c(struct device *dev, struct v4l2_async_hw_device *hw_dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return hw_dev->bus_type == V4L2_ASYNC_BUS_I2C &&
		hw_dev->match.i2c.adapter_id == client->adapter->nr &&
		hw_dev->match.i2c.address == client->addr;
}

static bool match_platform(struct device *dev, struct v4l2_async_hw_device *hw_dev)
{
	return hw_dev->bus_type == V4L2_ASYNC_BUS_PLATFORM &&
		!strcmp(hw_dev->match.platform.name, dev_name(dev));
}

/*
 * I think, notifiers on different busses can run concurrently, so, we have to
 * protect common data, e.g. sub-device lists.
 */
static int async_notifier_cb(struct v4l2_async_group *group,
		unsigned long action, struct device *dev,
		bool (*match)(struct device *, struct v4l2_async_hw_device *))
{
	struct v4l2_device *v4l2_dev = group->v4l2_dev;
	struct v4l2_async_subdev *asd;
	bool done;
	int ret;

	if (action != BUS_NOTIFY_BOUND_DRIVER &&
	    action != BUS_NOTIFY_BIND_DRIVER)
		return NOTIFY_DONE;

	/* Asynchronous: have to lock */
	mutex_lock(&v4l2_dev->group_lock);

	list_for_each_entry(asd, &group->group, list) {
		if (match(dev, &asd->hw))
			break;
	}

	if (&asd->list == &group->group) {
		/* Not our device */
		mutex_unlock(&v4l2_dev->group_lock);
		return NOTIFY_DONE;
	}

	asd->dev = dev;

	if (action == BUS_NOTIFY_BIND_DRIVER) {
		/*
		 * Provide platform data to the driver: it can complete probing
		 * now.
		 */
		dev->platform_data = &asd->sdpd;
		mutex_unlock(&v4l2_dev->group_lock);
		if (group->bind)
			group->bind(group, asd);
		return NOTIFY_OK;
	}

	/* BUS_NOTIFY_BOUND_DRIVER */
	if (asd->hw.bus_type == V4L2_ASYNC_BUS_I2C)
		asd->sdpd.subdev = i2c_get_clientdata(to_i2c_client(dev));
	/*
	 * Non-I2C subdevice drivers should take care to assign their subdevice
	 * pointers
	 */
	ret = v4l2_device_register_subdev(v4l2_dev,
					  asd->sdpd.subdev);
	if (ret < 0) {
		mutex_unlock(&v4l2_dev->group_lock);
		/* FIXME: error, clean up world? */
		dev_err(dev, "Failed registering a subdev: %d\n", ret);
		return NOTIFY_OK;
	}
	list_move(&asd->list, &group->done);

	/* Client probed & all subdev drivers collected */
	done = list_empty(&group->group);

	mutex_unlock(&v4l2_dev->group_lock);

	if (group->bound)
		group->bound(group, asd);

	if (done && group->complete)
		group->complete(group);

	return NOTIFY_OK;
}

static int platform_cb(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct device *dev = data;
	struct v4l2_async_group *group = container_of(nb, struct v4l2_async_group,
						     platform_notifier);

	return async_notifier_cb(group, action, dev, match_platform);
}

static int i2c_cb(struct notifier_block *nb,
		  unsigned long action, void *data)
{
	struct device *dev = data;
	struct v4l2_async_group *group = container_of(nb, struct v4l2_async_group,
						     i2c_notifier);

	return async_notifier_cb(group, action, dev, match_i2c);
}

/*
 * Typically this function will be called during bridge driver probing. It
 * installs bus notifiers to handle asynchronously probing subdevice drivers.
 * Once the bridge driver probing completes, subdevice drivers, waiting in
 * EPROBE_DEFER state are re-probed, at which point they get their platform
 * data, which allows them to complete probing.
 */
int v4l2_async_group_probe(struct v4l2_async_group *group)
{
	struct v4l2_async_subdev *asd, *tmp;
	bool i2c_used = false, platform_used = false;
	int ret;

	/* This group is inactive so far - no notifiers yet */
	list_for_each_entry_safe(asd, tmp, &group->group, list) {
		if (asd->sdpd.subdev) {
			/* Simulate a BIND event */
			if (group->bind)
				group->bind(group, asd);

			/* Already probed, don't wait for it */
			ret = v4l2_device_register_subdev(group->v4l2_dev,
							  asd->sdpd.subdev);

			if (ret < 0)
				return ret;

			list_move(&asd->list, &group->done);
			continue;
		}

		switch (asd->hw.bus_type) {
		case V4L2_ASYNC_BUS_PLATFORM:
			platform_used = true;
			break;
		case V4L2_ASYNC_BUS_I2C:
			i2c_used = true;
		}
	}

	if (list_empty(&group->group)) {
		if (group->complete)
			group->complete(group);
		return 0;
	}

	/* TODO: so far bus_register_notifier() never fails */
	if (platform_used) {
		group->platform_notifier.notifier_call = platform_cb;
		bus_register_notifier(&platform_bus_type,
				      &group->platform_notifier);
	}

	if (i2c_used) {
		group->i2c_notifier.notifier_call = i2c_cb;
		bus_register_notifier(&i2c_bus_type,
				      &group->i2c_notifier);
	}

	return 0;
}
EXPORT_SYMBOL(v4l2_async_group_probe);

int v4l2_async_group_init(struct v4l2_device *v4l2_dev,
			  struct v4l2_async_group *group,
			  struct v4l2_async_subdev *asd, int cnt)
{
	int i;

	if (!group)
		return -EINVAL;

	INIT_LIST_HEAD(&group->group);
	INIT_LIST_HEAD(&group->done);
	group->v4l2_dev = v4l2_dev;

	for (i = 0; i < cnt; i++)
		list_add_tail(&asd[i].list, &group->group);

	/* Protect the global V4L2 device group list */
	mutex_lock(&v4l2_dev->group_lock);
	list_add_tail(&group->list, &v4l2_dev->group_head);
	mutex_unlock(&v4l2_dev->group_lock);

	return 0;
}
EXPORT_SYMBOL(v4l2_async_group_init);

void v4l2_async_group_release(struct v4l2_async_group *group)
{
	struct v4l2_async_subdev *asd, *tmp;

	/* Also no problem, if notifiers haven't been registered */
	bus_unregister_notifier(&platform_bus_type,
				&group->platform_notifier);
	bus_unregister_notifier(&i2c_bus_type,
				&group->i2c_notifier);

	mutex_lock(&group->v4l2_dev->group_lock);
	list_del(&group->list);
	mutex_unlock(&group->v4l2_dev->group_lock);

	list_for_each_entry_safe(asd, tmp, &group->done, list) {
		v4l2_device_unregister_subdev(asd->sdpd.subdev);
		/* If we handled USB devices, we'd have to lock the parent too */
		device_release_driver(asd->dev);
		asd->dev->platform_data = NULL;
		if (device_attach(asd->dev) <= 0)
			dev_dbg(asd->dev, "Failed to re-probe to %s\n", asd->dev->driver ?
				asd->dev->driver->name : "(none)");
		list_del(&asd->list);
	}
}
EXPORT_SYMBOL(v4l2_async_group_release);
