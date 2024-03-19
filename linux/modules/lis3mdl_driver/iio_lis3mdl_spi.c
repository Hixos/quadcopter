// SPDX-License-Identifier: GPL-2.0-only

#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>

#include "iio_lis3mdl_spi.h"

#define HX_LIS3MDL_SCALE_MICRO_4G 146
#define HX_LIS3MDL_SCALE_MICRO_8G 292
#define HX_LIS3MDL_SCALE_MICRO_12G 437
#define HX_LIS3MDL_SCALE_MICRO_16G 584

#define HX_LIS3MDL_ODR_0_625 625
#define HX_LIS3MDL_ODR_1_25 1250
#define HX_LIS3MDL_ODR_2_5 2500
#define HX_LIS3MDL_ODR_5 5000
#define HX_LIS3MDL_ODR_10 10000
#define HX_LIS3MDL_ODR_20 20000
#define HX_LIS3MDL_ODR_40 40000
#define HX_LIS3MDL_ODR_80 80000
#define HX_LIS3MDL_ODR_155 155000

#define HX_LIS3MDL_ODR_BITS_0_625 0b0000
#define HX_LIS3MDL_ODR_BITS_1_25 0b0010
#define HX_LIS3MDL_ODR_BITS_2_5 0b0100
#define HX_LIS3MDL_ODR_BITS_5 0b0110
#define HX_LIS3MDL_ODR_BITS_10 0b1000
#define HX_LIS3MDL_ODR_BITS_20 0b1010
#define HX_LIS3MDL_ODR_BITS_40 0b1100
#define HX_LIS3MDL_ODR_BITS_80 0b1110
#define HX_LIS3MDL_ODR_BITS_155 0b0001

#define HX_LIS3MDL_NUM_ODR 9
static int hx_lis3mdl_odr_list[] = { HX_LIS3MDL_ODR_0_625, HX_LIS3MDL_ODR_1_25,
				     HX_LIS3MDL_ODR_2_5,   HX_LIS3MDL_ODR_5,
				     HX_LIS3MDL_ODR_10,	   HX_LIS3MDL_ODR_20,
				     HX_LIS3MDL_ODR_40,	   HX_LIS3MDL_ODR_80,
				     HX_LIS3MDL_ODR_155 };

#define HX_LIS3MDL_NUM_FS 4
static int hx_lis3mdl_fs_list[] = { 4, 8, 12, 16 };

enum hx_lis3mdl_registers {
	HX_LIS3MDL_REG_WHO_AM_I = 0x0F,
	HX_LIS3MDL_REG_CTRL_REG_1 = 0x20,
	HX_LIS3MDL_REG_CTRL_REG_2 = 0x21,
	HX_LIS3MDL_REG_CTRL_REG_3 = 0x22,
	HX_LIS3MDL_REG_CTRL_REG_4 = 0x23,
	HX_LIS3MDL_REG_CTRL_REG_5 = 0x24,
	HX_LIS3MDL_REG_OUT_X_L = 0x28,
	HX_LIS3MDL_REG_OUT_Y_L = 0x2A,
	HX_LIS3MDL_REG_OUT_Z_L = 0x2C
};

enum hx_lis3mdl_scan_index {
	HX_LIS3MDL_SCAN_MAG_X = 0,
	HX_LIS3MDL_SCAN_MAG_Y = 1,
	HX_LIS3MDL_SCAN_MAG_Z = 2,
	HX_LIS3MDL_SCAN_TIMESTAMP = 3,
};

static const struct of_device_id hx_lis3mdl_of_match[] = {
	{ .compatible = "hx,hxlis3mdl" },
	{}
};

MODULE_DEVICE_TABLE(of, hx_lis3mdl_of_match);

static const struct regmap_config hx_lis3mdl_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#define HX_LIS3MDL_NUM_CHAN 4
static const struct iio_chan_spec hx_lis3mdl_channels[] = {
	{
		.type = IIO_MAGN,
		.channel2 = IIO_MOD_X,
		.address = HX_LIS3MDL_REG_OUT_X_L,
		.scan_index = HX_LIS3MDL_SCAN_MAG_X,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1
	},
	{
		.type = IIO_MAGN,
		.channel2 = IIO_MOD_Y,
		.address = HX_LIS3MDL_REG_OUT_Y_L,
		.scan_index = HX_LIS3MDL_SCAN_MAG_Y,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) ,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1
	},
	{
		.type = IIO_MAGN,
		.channel2 = IIO_MOD_Z,
		.address = HX_LIS3MDL_REG_OUT_Z_L,
		.scan_index = HX_LIS3MDL_SCAN_MAG_Z,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1
	},
	{
		.type = IIO_TIMESTAMP,
		.channel = -1,
		.scan_index = HX_LIS3MDL_SCAN_TIMESTAMP,
		.scan_type = {
			.sign = 's',
			.realbits = 64,
			.storagebits = 64,
			.shift = 0,
			.endianness = IIO_LE,
		}
	}
};

static int hx_lis3mdl_check_id(struct iio_dev *indio_dev)
{
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);
	struct regmap *regmap = mdata->regmap;

	unsigned int whoami;
	regmap_read(regmap, HX_LIS3MDL_REG_WHO_AM_I, &whoami);

	if (whoami != 0x3D) {
		dev_err(&indio_dev->dev,
			"Invalid WHO_AM_I: 0x%02X, expected 0x3D.\n", whoami);
		return -EINVAL;
	}
	dev_info(&indio_dev->dev, "Init ok. WHO_AM_I: 0x%02X.\n", whoami);
	return 0;
}

static int hx_lis3mdl_configure(struct iio_dev *indio_dev)
{
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);
	struct regmap *regmap = mdata->regmap;

	int err;

	// FS 4 gauss
	err = regmap_write(regmap, HX_LIS3MDL_REG_CTRL_REG_2, 0x00);
	if (err < 0)
		goto configure_error;

	// ODR 0.625 Hz, XY UHP mode
	err = regmap_write(regmap, HX_LIS3MDL_REG_CTRL_REG_1, 0x60);
	if (err < 0)
		goto configure_error;

	// Z UHP mode
	err = regmap_write(regmap, HX_LIS3MDL_REG_CTRL_REG_4, 0x0C);
	if (err < 0)
		goto configure_error;

	// Continuous conversion mode, low power mode disabled
	err = regmap_write(regmap, HX_LIS3MDL_REG_CTRL_REG_3, 0x00);
	if (err < 0)
		goto configure_error;

	return 0;

configure_error:
	dev_err(&indio_dev->dev, "Could not configure device: %d", err);
	return err;
}

static int hx_lis3mdl_update_bits(struct iio_dev *indio_dev, unsigned int reg,
				  unsigned int mask, unsigned int value)
{
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);
	return regmap_update_bits(sdata->regmap, reg, mask,
				  value << __ffs(mask));
}

static int hx_lis3mdl_sensor_init(struct iio_dev *indio_dev)
{
	int err;

	err = hx_lis3mdl_check_id(indio_dev);
	if (err < 0)
		return err;

	err = hx_lis3mdl_configure(indio_dev);
	if (err < 0)
		return err;

	return 0;
}

ssize_t hx_lis3mdl_sysfs_sampling_frequency_avail(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	int i, len = 0;

	for (i = 0; i < HX_LIS3MDL_NUM_ODR; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%d ",
				 hx_lis3mdl_odr_list[i] / 1000,
				 hx_lis3mdl_odr_list[i] % 1000);
	}
	buf[len - 1] = '\n';

	return len;
}
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(hx_lis3mdl_sysfs_sampling_frequency_avail);

ssize_t hx_lis3mdl_sysfs_scale_avail(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < HX_LIS3MDL_NUM_FS; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 hx_lis3mdl_fs_list[i]);
	}
	buf[len - 1] = '\n';

	return len;
}
static IIO_DEVICE_ATTR(in_magn_scale_available, S_IRUGO,
		       hx_lis3mdl_sysfs_scale_avail, NULL, 0);

static struct attribute *hx_lis3mdl_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_magn_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group hx_lis3mdl_attribute_group = {
	.attrs = hx_lis3mdl_attributes,
};

static int hx_lis3mdl_read_axis(struct iio_dev *indio_dev,
				struct iio_chan_spec const *ch, int *val)
{
	int err;
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);

	__le16 buf;
	err = regmap_bulk_read(mdata->regmap, ch->address, &buf, sizeof(buf));
	if (err < 0)
		goto read_error;

	*val = le16_to_cpu(buf);

read_error:
	return err;
}

static int hx_lis3mdl_write_odr(struct iio_dev *indio_dev, int odr_mhz)
{
	int bits;
	switch (odr_mhz) {
	case HX_LIS3MDL_ODR_0_625:
		bits = HX_LIS3MDL_ODR_BITS_0_625;
		break;
	case HX_LIS3MDL_ODR_1_25:
		bits = HX_LIS3MDL_ODR_BITS_1_25;
		break;
	case HX_LIS3MDL_ODR_2_5:
		bits = HX_LIS3MDL_ODR_BITS_2_5;
		break;
	case HX_LIS3MDL_ODR_5:
		bits = HX_LIS3MDL_ODR_BITS_5;
		break;
	case HX_LIS3MDL_ODR_10:
		bits = HX_LIS3MDL_ODR_BITS_10;
		break;
	case HX_LIS3MDL_ODR_20:
		bits = HX_LIS3MDL_ODR_BITS_20;
		break;
	case HX_LIS3MDL_ODR_40:
		bits = HX_LIS3MDL_ODR_BITS_40;
		break;
	case HX_LIS3MDL_ODR_80:
		bits = HX_LIS3MDL_ODR_BITS_80;
		break;
	case HX_LIS3MDL_ODR_155:
		bits = HX_LIS3MDL_ODR_BITS_155;
		break;
	default:
		dev_warn(&indio_dev->dev, "Unrecognized data rate: %d",
			 odr_mhz);
		return -EINVAL;
	}
	dev_info(&indio_dev->dev, "Update bits: %02X", bits);

	return hx_lis3mdl_update_bits(indio_dev, HX_LIS3MDL_REG_CTRL_REG_1,
				      0x1E, bits);
}

static int hx_lis3mdl_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *ch, int *val,
			       int *val2, long mask)
{
	int err;
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = hx_lis3mdl_read_axis(indio_dev, ch, val);
		if (err < 0)
			goto read_error;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = mdata->gain;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = mdata->odr;
		*val2 = 1000;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}

read_error:
	return err;
}

static int hx_lis3mdl_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int val,
				int val2, long mask)
{
	switch (mask) {
	// case IIO_CHAN_INFO_SCALE:
	// 	return st_sensors_set_fullscale_by_gain(indio_dev, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		dev_info(&indio_dev->dev, "val1: %d, val2: %d", val, val2);
		return hx_lis3mdl_write_odr(indio_dev,
					    val * 1000 + val2 / 1000);
	default:
		return -EINVAL;
	}
}

static const struct iio_info hx_lis3mdl_info = {
	.attrs = &hx_lis3mdl_attribute_group,
	.read_raw = &hx_lis3mdl_read_raw,
	.write_raw = &hx_lis3mdl_write_raw
	// .debugfs_reg_access = &st_sensors_debugfs_reg_access,
};

static int hx_lis3mdl_spi_probe(struct spi_device *spi)
{
	pr_info("Hello kernel!\n");
	int err;

	struct hx_lis3mdl_data *mdata;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*mdata));
	if (!indio_dev)
		return -ENOMEM;

	mdata = iio_priv(indio_dev);

	mdata->regmap = devm_regmap_init_spi(spi, &hx_lis3mdl_regmap_config);
	if (IS_ERR(mdata->regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap (%ld)\n",
			PTR_ERR(mdata->regmap));
		return PTR_ERR(mdata->regmap);
	}

	mdata->gain = HX_LIS3MDL_SCALE_MICRO_4G;
	mdata->odr = HX_LIS3MDL_ODR_0_625;

	indio_dev->name = spi->modalias;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = (struct iio_chan_spec *)hx_lis3mdl_channels;
	indio_dev->num_channels = HX_LIS3MDL_NUM_CHAN;
	indio_dev->info = &hx_lis3mdl_info;

	err = hx_lis3mdl_sensor_init(indio_dev);
	if (err < 0)
		return err;

	return devm_iio_device_register(indio_dev->dev.parent, indio_dev);
}

static const struct spi_device_id hx_lis3mdl_id_table[] = {
	{ "hxlis3mdl" },
	{},
};
MODULE_DEVICE_TABLE(spi, hx_lis3mdl_id_table);

static struct spi_driver hx_lis3mdl_driver = {
    .driver =
        {
            .name = "lis3mdl-spi",
            .of_match_table = hx_lis3mdl_of_match,
        },
    .probe = hx_lis3mdl_spi_probe,
    .id_table = hx_lis3mdl_id_table,
};
module_spi_driver(hx_lis3mdl_driver);

MODULE_AUTHOR("Luca Erbetta <luca.erbetta105@gmail.com>");
MODULE_DESCRIPTION("STMicroelectronics LIS3MDL spi driver");
MODULE_LICENSE("GPL v2");
