/*
 * V4L2 clock service
 *
 * Copyright (C) 2012, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>

static DEFINE_MUTEX(clk_lock);
static LIST_HEAD(v4l2_clks);

static struct v4l2_clk *__find_clk(const char *dev_id, const char *id)
{
	struct v4l2_clk *tclk, *clk = ERR_PTR(-ENODEV);

	list_for_each_entry(tclk, &v4l2_clks, list) {
		if (!strstr(dev_id, tclk->dev_id))
			continue;

		if (!id || (id && strcmp(tclk->id, id))) {
			clk = tclk;
			break;
		}
	}
	return clk;
}

struct v4l2_clk *v4l2_clk_get(struct v4l2_subdev *sd, const char *id)
{
	struct v4l2_clk *clk;

	mutex_lock(&clk_lock);
	clk = __find_clk(sd->name, id);
	mutex_unlock(&clk_lock);

	if (!IS_ERR(clk) && !try_module_get(clk->ops->owner))
		clk = ERR_PTR(-ENODEV);

	return clk;
}
EXPORT_SYMBOL(v4l2_clk_get);

void v4l2_clk_put(struct v4l2_clk *clk)
{
	if (!IS_ERR(clk))
		module_put(clk->ops->owner);
}
EXPORT_SYMBOL(v4l2_clk_put);

int v4l2_clk_enable(struct v4l2_clk *clk)
{
	if (!clk->ops->enable)
		return -ENOSYS;
	return clk->ops->enable(clk);
}
EXPORT_SYMBOL(v4l2_clk_enable);

void v4l2_clk_disable(struct v4l2_clk *clk)
{
	if (clk->ops->disable)
		clk->ops->disable(clk);
}
EXPORT_SYMBOL(v4l2_clk_disable);

unsigned long v4l2_clk_get_rate(struct v4l2_clk *clk)
{
	if (!clk->ops->get_rate)
		return -ENOSYS;
	return clk->ops->get_rate(clk);
}
EXPORT_SYMBOL(v4l2_clk_get_rate);

int v4l2_clk_set_rate(struct v4l2_clk *clk, unsigned long rate)
{
	if (!clk->ops->set_rate)
		return -ENOSYS;
	return clk->ops->set_rate(clk, rate);
}
EXPORT_SYMBOL(v4l2_clk_set_rate);

struct v4l2_clk *v4l2_clk_register(const struct v4l2_clk_ops *ops,
				   const char *dev_id,
				   const char *id, void *priv)
{
	struct v4l2_clk *clk;

	if (!ops || !dev_id)
		return ERR_PTR(-EINVAL);

	mutex_lock(&clk_lock);
	clk = __find_clk(dev_id, id);
	mutex_unlock(&clk_lock);

	if (!IS_ERR(clk))
		return ERR_PTR(-EEXIST);

	clk = kzalloc(sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);

	clk->ops = ops;
	clk->id = id;
	clk->dev_id = dev_id;
	clk->priv = priv;

	mutex_lock(&clk_lock);
	list_add_tail(&clk->list, &v4l2_clks);
	mutex_unlock(&clk_lock);

	return clk;
}
EXPORT_SYMBOL(v4l2_clk_register);

void v4l2_clk_unregister(struct v4l2_clk *clk)
{
	mutex_lock(&clk_lock);
	list_del(&clk->list);
	mutex_unlock(&clk_lock);

	kfree(clk);
}
EXPORT_SYMBOL(v4l2_clk_unregister);
