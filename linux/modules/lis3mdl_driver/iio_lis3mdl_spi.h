/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef HX_LIS3MDL_H
#define HX_LIS3MDL_H

#include <linux/regmap.h>

struct hx_lis3mdl_data {
    struct regmap* regmap;
    int gain;
    int odr;
};

#endif /* HX_LIS3MDL_H */
