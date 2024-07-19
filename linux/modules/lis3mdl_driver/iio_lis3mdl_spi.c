// SPDX-License-Identifier: GPL-2.0-only

#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>

#include "iio_lis3mdl_spi.h"

// Scale values
#define HX_LIS3MDL_SCALE_MICRO_4G 146
#define HX_LIS3MDL_SCALE_MICRO_8G 292
#define HX_LIS3MDL_SCALE_MICRO_12G 437
#define HX_LIS3MDL_SCALE_MICRO_16G 584

// Scale register bits
#define HX_LIS3MDL_SCALE_BITS_4G 0b00
#define HX_LIS3MDL_SCALE_BITS_8G 0b01
#define HX_LIS3MDL_SCALE_BITS_12G 0b10
#define HX_LIS3MDL_SCALE_BITS_16G 0b11

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
static int hx_lis3mdl_fs_list[] = { HX_LIS3MDL_SCALE_MICRO_4G,
				    HX_LIS3MDL_SCALE_MICRO_8G,
				    HX_LIS3MDL_SCALE_MICRO_12G,
				    HX_LIS3MDL_SCALE_MICRO_16G };

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
	.read_flag_mask = 0xC0
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
	dev_err(&indio_dev->dev, "Could not configure device: %d\n", err);
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
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.000%d ",
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

static int hx_lis3mdl_write_odr(struct iio_dev *indio_dev, int odr_milli_hz)
{
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);
	mdata->odr = odr_milli_hz;
	int bits;
	switch (odr_milli_hz) {
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
		dev_err(&indio_dev->dev, "Unrecognized data rate: %d\n",
			 odr_milli_hz);
		return -EINVAL;
	}

	return hx_lis3mdl_update_bits(indio_dev, HX_LIS3MDL_REG_CTRL_REG_1,
				      0x1E, bits);
}

static int hx_lis3mdl_write_scale(struct iio_dev *indio_dev, int gain)
{
	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);
	mdata->gain = gain;
	int bits;
	switch (gain) {
	case HX_LIS3MDL_SCALE_MICRO_4G:
		bits = HX_LIS3MDL_SCALE_BITS_4G;
		break;
	case HX_LIS3MDL_SCALE_MICRO_8G:
		bits = HX_LIS3MDL_SCALE_BITS_8G;
		break;
	case HX_LIS3MDL_SCALE_MICRO_12G:
		bits = HX_LIS3MDL_SCALE_BITS_12G;
		break;
	case HX_LIS3MDL_SCALE_MICRO_16G:
		bits = HX_LIS3MDL_SCALE_BITS_16G;
		break;
	default:
		dev_err(&indio_dev->dev, "Unrecognized scale: %d\n", gain);
		return -EINVAL;
	}

	return hx_lis3mdl_update_bits(indio_dev, HX_LIS3MDL_REG_CTRL_REG_2,
				      0x60, bits);
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
	case IIO_CHAN_INFO_SCALE:
		return hx_lis3mdl_write_scale(indio_dev, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
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


static int hx_lis3mdl_buffer_postenable(struct iio_dev *indio_dev)
{
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);
	sdata->enabled = true;
	unsigned int buf;
	regmap_read(sdata->regmap, HX_LIS3MDL_REG_OUT_X_L, &buf);
	return 0;
}

static int hx_lis3mdl_buffer_predisable(struct iio_dev *indio_dev)
{
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);
	sdata->enabled = false;
	return 0;
}

static int hx_lis3mdl_trig_set_state(struct iio_trigger *trig, bool state)
{
	return 0;
}

static int hx_lis3mdl_validate_device(struct iio_trigger *trig,
				      struct iio_dev *indio_dev)
{
	struct iio_dev *indio = iio_trigger_get_drvdata(trig);

	if (indio != indio_dev)
		return -EINVAL;

	return 0;
}

static irqreturn_t hx_lis3mdl_irq_handler(int irq, void *p)
{
	struct iio_trigger *trig = p;
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);

	/* Get the time stamp as close in time as possible */
	sdata->hw_timestamp = iio_get_time_ns(indio_dev);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hx_lis3mdl_irq_thread(int irq, void *p)
{
	iio_trigger_poll_nested(p);
	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops hx_lis3mdl_buffer_setup_ops = {
	.postenable = &hx_lis3mdl_buffer_postenable,
	.predisable = &hx_lis3mdl_buffer_predisable,
};

static const struct iio_trigger_ops hx_lis3mdl_trigger_ops = {
	.set_trigger_state = &hx_lis3mdl_trig_set_state,
	.validate_device = &hx_lis3mdl_validate_device,
};

int hx_lis3mdl_allocate_trigger(struct iio_dev *indio_dev,
				const struct iio_trigger_ops *trigger_ops)
{
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);
	struct device *parent = indio_dev->dev.parent;
	unsigned long irq_trig;
	int err;

	sdata->trig =
		devm_iio_trigger_alloc(parent, "%s-trigger", indio_dev->name, iio_device_id(indio_dev));
	if (sdata->trig == NULL) {
		dev_err(&indio_dev->dev, "failed to allocate iio trigger.\n");
		return -ENOMEM;
	}

	iio_trigger_set_drvdata(sdata->trig, indio_dev);
	sdata->trig->ops = trigger_ops;
	sdata->trig->dev.parent = &indio_dev->dev;

	irq_trig = irqd_get_trigger_type(irq_get_irq_data(sdata->irq));

	switch (irq_trig) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		dev_err(&indio_dev->dev,
			"falling/low specified for IRQ but hardware supports only rising/high: will request rising/high\n");
		if (irq_trig == IRQF_TRIGGER_FALLING)
			irq_trig = IRQF_TRIGGER_RISING;
		if (irq_trig == IRQF_TRIGGER_LOW)
			irq_trig = IRQF_TRIGGER_HIGH;
		break;
	case IRQF_TRIGGER_RISING:
		dev_info(&indio_dev->dev, "interrupts on the rising edge\n");
		break;
	case IRQF_TRIGGER_HIGH:
		dev_info(&indio_dev->dev, "interrupts active high level\n");
		break;
	default:
		/* This is the most preferred mode, if possible */
		dev_err(&indio_dev->dev,
			"unsupported IRQ trigger specified (%lx), enforce rising edge\n",
			irq_trig);
		irq_trig = IRQF_TRIGGER_RISING;
	}

	/* Tell the interrupt handler that we're dealing with edges */
	if (irq_trig != IRQF_TRIGGER_FALLING &&
	    irq_trig != IRQF_TRIGGER_RISING) {
		/*
		 * If we're not using edges (i.e. level interrupts) we
		 * just mask off the IRQ, handle one interrupt, then
		 * if the line is still low, we return to the
		 * interrupt handler top half again and start over.
		 */
		irq_trig |= IRQF_ONESHOT;
	}
	err = devm_request_threaded_irq(parent, sdata->irq,
					hx_lis3mdl_irq_handler,
					hx_lis3mdl_irq_thread, irq_trig,
					sdata->trig->name, sdata->trig);
	if (err) {
		dev_err(&indio_dev->dev, "failed to request trigger IRQ.\n");
		return err;
	}

	err = devm_iio_trigger_register(parent, sdata->trig);
	if (err < 0) {
		dev_err(&indio_dev->dev, "failed to register iio trigger.\n");
		return err;
	}
	indio_dev->trig = iio_trigger_get(sdata->trig);

	return 0;
}

irqreturn_t hx_lis3mdl_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct hx_lis3mdl_data *sdata = iio_priv(indio_dev);
	s64 timestamp;

	if (iio_trigger_using_own(indio_dev))
		timestamp = sdata->hw_timestamp;
	else
		timestamp = iio_get_time_ns(indio_dev);

	u8 *buf = sdata->buffer_data;
	int i;

	for_each_set_bit(i, indio_dev->active_scan_mask, 3) {
		const struct iio_chan_spec *channel = &indio_dev->channels[i];

		if (regmap_bulk_read(sdata->regmap, channel->address, buf, 2) <
		    0)
			goto hx_lis3mdl_buf_err;

		/* Advance the buffer pointer */
		buf += 2;
	}
	iio_push_to_buffers_with_timestamp(indio_dev, sdata->buffer_data,
					   timestamp);
hx_lis3mdl_buf_err:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int hx_lis3mdl_allocate_ring(struct iio_dev *indio_dev)
{
	return devm_iio_triggered_buffer_setup(indio_dev->dev.parent, indio_dev,
					       NULL,
					       &hx_lis3mdl_trigger_handler,
					       &hx_lis3mdl_buffer_setup_ops);
}

void hx_lis3mdl_dev_name_probe(struct device *dev, char *name, int len)
{
	const void *match;

	match = device_get_match_data(dev);
	if (!match)
		return;

	/* The name from the match takes precedence if present */
	strscpy(name, match, len);
}

static int hx_lis3mdl_spi_probe(struct spi_device *spi)
{
	int err;

	hx_lis3mdl_dev_name_probe(&spi->dev, spi->modalias, sizeof(spi->modalias));

	struct hx_lis3mdl_data *mdata;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*mdata));
	if (!indio_dev)
		return -ENOMEM;

	struct hx_lis3mdl_data *mdata = iio_priv(indio_dev);

	mdata->regmap = devm_regmap_init_spi(spi, &hx_lis3mdl_regmap_config);
	if (IS_ERR(mdata->regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap (%ld)\n",
			PTR_ERR(mdata->regmap));
		return PTR_ERR(mdata->regmap);
	}

	spi_set_drvdata(spi, indio_dev);
	mdata->irq = spi->irq;

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

	err = hx_lis3mdl_allocate_ring(indio_dev);
	if (err < 0)
		return err;

	dev_info(&indio_dev->dev, "IRQ: %d\n", mdata->irq);
	if (mdata->irq > 0) {
		err = hx_lis3mdl_allocate_trigger(indio_dev,
						  &hx_lis3mdl_trigger_ops);
		if (err < 0)
			return err;
	}
	dev_info(&indio_dev->dev, "Registering device!\n");
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
            .name = "hxlis3mdl",
            .of_match_table = hx_lis3mdl_of_match,
        },
    .probe = hx_lis3mdl_spi_probe,
    .id_table = hx_lis3mdl_id_table,
};
module_spi_driver(hx_lis3mdl_driver);

MODULE_AUTHOR("Luca Erbetta <luca.erbetta105@gmail.com>");
MODULE_DESCRIPTION("STMicroelectronics LIS3MDL spi driver");
MODULE_LICENSE("GPL v2");
