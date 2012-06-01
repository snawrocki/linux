#ifndef OF_MEDIA
#define OF_MEDIA

#include <linux/of.h>

#if defined(CONFIG_OF_MEDIA_VIDEO)
int v4l2_mbus_of_get_polarity(struct device_node *node,
			      unsigned int *out_flags);

#else
int v4l2_mbus_of_get_polarity(struct device_node *node,
			      unsigned int *out_flags)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF_MEDIA_VIDEO */

#endif /* OF_MEDIA */
