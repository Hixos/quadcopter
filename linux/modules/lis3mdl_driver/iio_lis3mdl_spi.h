/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef HX_LIS3MDL_H
#define HX_LIS3MDL_H

#include <linux/regmap.h>

/*
 * Buffer size max case: 2bytes per channel, 3 channels in total +
 *			 8bytes timestamp channel (s64)
 */
#define HX_LIS3MDL_MAX_BUFFER_SIZE (ALIGN(2 * 3, sizeof(s64)) + sizeof(s64))

struct hx_lis3mdl_data {
	struct regmap *regmap;
	int gain;
	int odr;
	bool enabled;
	s64 hw_timestamp;

	int irq;
	struct iio_trigger *trig;

	char buffer_data[HX_LIS3MDL_MAX_BUFFER_SIZE] ____cacheline_aligned;
};

#endif /* HX_LIS3MDL_H */
