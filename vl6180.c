/*
 * vl6180.c - Support for STMicroelectronics VL6180 ALS, range and proximity
 * sensor
 *
 * Copyright 2018 Budi Sendjaja <sendjajab@cumminsallison.com>
 * Copyright 2017 Peter Meerwald-Stadler <pmeerw@pmeerw.net>
 * Copyright 2017 Manivannan Sadhasivam <manivannanece23@gmail.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for VL6180 (7-bit I2C slave address 0x29)
 *
 * Range: 0 to 100mm
 * ALS: < 1 Lux up to 100 kLux
 * IR: 850nm
 *
 * TODO: hardware buffer
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/errno.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

//Uncomment VL6180_I2C_DEBUG to see I2C data
//When debug is enabled and interrupt triggered too often,
//it might be due to delay in processing interrupt.
//Increase inter-measurement period to remedy that.
//#define VL6180_I2C_DEBUG 1

#define VL6180_DRV_NAME "vl6180"
#define VL6180_DEFAULT_I2C_ADDR 0x29

/* Device identification register and value */
#define VL6180_MODEL_ID             0x000
#define VL6180_MODEL_ID_VAL         0xb4
#define VL6180_MODEL_REV_MAJOR      0x001
#define VL6180_MODEL_REV_MAJOR_VAL  0x01
#define VL6180_MODEL_REV_MINOR      0x002
#define VL6180_MODEL_REV_MINOR_VAL  0x03
#define VL6180_MODULE_REV_MAJOR     0x003
#define VL6180_MODULE_REV_MAJOR_VAL 0x01
#define VL6180_MODULE_REV_MINOR     0x004
#define VL6180_MODULE_REV_MINOR_VAL 0x02
#define VL6180_DATE_HI              0x006
#define VL6180_DATE_LO              0x007
#define VL6180_TIME                 0x008

/* Configuration registers */
#define VL6180_MODE_GPIO0 	0x010
#define VL6180_MODE_GPIO1 	0x011
#define VL6180_HISTORY_CTRL 0x012
#define VL6180_INTR_CONFIG 	0x014
#define VL6180_INTR_CLEAR 	0x015
#define VL6180_OUT_OF_RESET 0x016
#define VL6180_HOLD 		0x017

/* Range setup registers */
#define VL6180_RANGE_START 				0x018
#define VL6180_THRESH_HIGH 				0x019
#define VL6180_THRESH_LOW 				0x01A
#define VL6180_INTERMEASUREMENT_PERIOD 	0x01B
#define VL6180_MAX_CONVERGENCE_TIME 	0x01C
#define VL6180_RANGE_CHECK_ENABLE       0x02D

/* Als setup registers */
#define VL6180_ALS_START 	0x038
#define VL6180_ALS_GAIN 	0x03F
#define VL6180_ALS_IT 		0x040

/* Result and value registers */
#define VL6180_RANGE_STATUS 0x04D
#define VL6180_ALS_STATUS 	0x04E
#define VL6180_INTR_STATUS 	0x04F
#define VL6180_ALS_VALUE 	0x050
#define VL6180_RANGE_VALUE 	0x062
#define VL6180_RANGE_RAW 	0x064
#define VL6180_RANGE_RATE 	0x066

#define VL6180_AVG_SAMPLE_PERIOD	     0x10A
#define VL6180_FIRMWARE_BOOTUP			 0x119
#define VL6180_FIRMWARE_BOOTUP_MASK		 0x1
#define VL6180_I2C_SLAVE_DEVICE_ADDRESS  0x212

/* bits of the RANGE_START and ALS_START register */
#define VL6180_MODE_CONT 		BIT(1) /* continuous mode */
#define VL6180_STARTSTOP 		BIT(0) /* start measurement, auto-reset */

/* bits of the INTR_STATUS and INTR_CONFIG register */
#define VL6180_ALS_READY 			BIT(5)
#define VL6180_ALS_OUT_OF_WINDOW 	BIT(4)|BIT(3)
#define VL6180_ALS_HIGH 			BIT(4)
#define VL6180_ALS_LOW	 			BIT(3)

#define VL6180_RANGE_READY 			BIT(2)
#define VL6180_RANGE_OUT_OF_WINDOW 	BIT(1)|BIT(0)
#define VL6180_RANGE_HIGH 			BIT(1)
#define VL6180_RANGE_LOW	 		BIT(0)

/* bits of the INTR_CLEAR register */
#define VL6180_CLEAR_ERROR 		BIT(2)
#define VL6180_CLEAR_ALS 		BIT(1)
#define VL6180_CLEAR_RANGE 		BIT(0)

/* bits of the HOLD register */
#define VL6180_HOLD_ON 			BIT(0)

/* default value for the ALS_IT register */
#define VL6180_ALS_IT_100 		0x63 /* 100 ms */

/* values for the ALS_GAIN register */
#define VL6180_ALS_GAIN_1 		0x46
#define VL6180_ALS_GAIN_1_25 	0x45
#define VL6180_ALS_GAIN_1_67 	0x44
#define VL6180_ALS_GAIN_2_5 	0x43
#define VL6180_ALS_GAIN_5 		0x42
#define VL6180_ALS_GAIN_10 		0x41
#define VL6180_ALS_GAIN_20 		0x40
#define VL6180_ALS_GAIN_40 		0x47

/* Intermeasurement period min/max/default value in ms*/
#define VL6180_INTERMEASUREMENT_PERIOD_MIN 		10
#define VL6180_INTERMEASUREMENT_PERIOD_MAX 		2550
#define VL6180_INTERMEASUREMENT_PERIOD_DEFAULT 	20

/* GPIO out bit position */
#define VL6180_GPIO_ENABLE 				BIT(6)
#define VL6180_GPIO_ACTIVE_POLARITY 	BIT(5)
#define VL6180_GPIO_INT_ENABLE 			BIT(4)

/* Default range threshold in millimeter */
#define VL6180_THRESH_LOW_DEFAULT  		40
#define VL6180_THRESH_HIGH_DEFAULT  	100
#define VL6180_THRESH_MIN  				0
#define VL6180_THRESH_MAX  				254
#define VL6180_RANGE_CHECK_ENABLE_VALUE 0x10

struct version_info {
	u8 model_rev_major;
	u8 model_rev_minor;
	u8 module_rev_major;
	u8 module_rev_minor;
	u8 id_date_hi;
	u8 id_date_lo;
	u16 id_time;
};

struct vl6180_data {
	struct i2c_client *client;
	struct mutex lock;
	struct version_info ver_info;
	struct regulator *power_supply;
	u16 intermeasurement_period;
	u8 range_low_threshold;
	u8 range_high_threshold;
	u8 range_value;
	/* Debug */
	u8 enableDebug;
};

enum { VL6180_ALS, VL6180_RANGE };

/**
 * struct vl6180_chan_regs - Registers for accessing channels
 * @drdy_mask:			Data ready bit in status register
 * @start_reg:			Conversion start register
 * @value_reg:			Result value register
 * @word:				Register word length
 * @continuous_mode:	single-shot(false) or continuous(true)
 */
struct vl6180_chan_regs {
	u8 drdy_mask;
	u16 start_reg, value_reg;
	bool word;
	bool continuous_mode;
};

static const struct vl6180_chan_regs vl6180_chan_regs_table[] = {
	[VL6180_ALS] = {
		.drdy_mask = VL6180_ALS_READY,
		.start_reg = VL6180_ALS_START,
		.value_reg = VL6180_ALS_VALUE,
		.word = true,
		.continuous_mode = false
	},
	[VL6180_RANGE] = {
		.drdy_mask = VL6180_RANGE_READY,
		.start_reg = VL6180_RANGE_START,
		.value_reg = VL6180_RANGE_VALUE,
		.word = false,
		.continuous_mode = true
	},
};

static int vl6180_read(struct i2c_client *client, u16 cmd, void *databuf,
		       u8 len)
{
	__be16 cmdbuf = cpu_to_be16(cmd);
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = sizeof(cmdbuf), .buf = (u8 *) &cmdbuf },
		{ .addr = client->addr, .len = len, .buf = databuf,
		  .flags = I2C_M_RD } };
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "failed reading register 0x%04x ret:%d\n",
				cmd, ret);
	}

	return ret;
}

static int vl6180_read_byte(struct i2c_client *client, u16 cmd)
{
	u8 data;
	int ret;

	ret = vl6180_read(client, cmd, &data, sizeof(data));
	if (ret < 0) {
		return ret;
	}
#ifdef VL6180_I2C_DEBUG
	printk(KERN_DEBUG "vl6180 readb  %04X %02X\n", cmd, data);
#endif
	return data;
}

static int vl6180_read_word(struct i2c_client *client, u16 cmd)
{
	__be16 data;
	int ret;

	ret = vl6180_read(client, cmd, &data, sizeof(data));
	if (ret < 0) {
		return ret;
	}
#ifdef VL6180_I2C_DEBUG
	printk(KERN_DEBUG "vl6180 readw  %04X %04X\n", cmd, data);
#endif
	return be16_to_cpu(data);
}

static int vl6180_write_byte(struct i2c_client *client, u16 cmd, u8 val)
{
	u8 buf[3];
	struct i2c_msg msgs[1] = {
		{ .addr = client->addr, .len = sizeof(buf), .buf = (u8 *) &buf } };
	int ret;

	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xff;
	buf[2] = val;
#ifdef VL6180_I2C_DEBUG
	printk(KERN_DEBUG "vl6180 writeb %04X %02X\n", cmd, val);
#endif
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "failed writing register 0x%04x ret:%d\n",
				cmd, ret);
		return ret;
	}

	return 0;
}

static int vl6180_write_word(struct i2c_client *client, u16 cmd, u16 val)
{
	__be16 buf[2];
	struct i2c_msg msgs[1] = {
		{ .addr = client->addr, .len = sizeof(buf), .buf = (u8 *) &buf } };
	int ret;

	buf[0] = cpu_to_be16(cmd);
	buf[1] = cpu_to_be16(val);
#ifdef VL6180_I2C_DEBUG
	printk(KERN_DEBUG "vl6180 writew %04X %04X\n", cmd, val);
#endif
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "failed writing register 0x%04x ret:%d\n",
				cmd, ret);
		return ret;
	}

	return 0;
}

/* HOLD is needed only to update a GROUP of parameters */
static int vl6180_hold(struct vl6180_data *data, bool hold)
{
	return vl6180_write_byte(data->client, VL6180_HOLD,
		hold ? VL6180_HOLD_ON : 0);
}

/* Refer to MicroST VL6180 AN4545 Application Note document
 * DocID026571 Rev 1, June 19, 2014
 * Chapter 9, SR03 settings page 24
 */
static int vl6180_range_static_init(struct vl6180_data *data)
{
	int ret = 0;

	/* REGISTER_TUNING_SR03_270514_CustomerView.txt */
	ret |= vl6180_write_byte(data->client, 0x0207, 0x01);
	ret |= vl6180_write_byte(data->client, 0x0208, 0x01);
	ret |= vl6180_write_byte(data->client, 0x0096, 0x00);
	ret |= vl6180_write_byte(data->client, 0x0097, 0xfd);
	ret |= vl6180_write_byte(data->client, 0x00e3, 0x00);
	ret |= vl6180_write_byte(data->client, 0x00e4, 0x04);
	ret |= vl6180_write_byte(data->client, 0x00e5, 0x02);
	ret |= vl6180_write_byte(data->client, 0x00e6, 0x01);
	ret |= vl6180_write_byte(data->client, 0x00e7, 0x03);
	ret |= vl6180_write_byte(data->client, 0x00f5, 0x02);
	ret |= vl6180_write_byte(data->client, 0x00d9, 0x05);
	ret |= vl6180_write_byte(data->client, 0x00db, 0xce);
	ret |= vl6180_write_byte(data->client, 0x00dc, 0x03);
	ret |= vl6180_write_byte(data->client, 0x00dd, 0xf8);
	ret |= vl6180_write_byte(data->client, 0x009f, 0x00);
	ret |= vl6180_write_byte(data->client, 0x00a3, 0x3c);
	ret |= vl6180_write_byte(data->client, 0x00b7, 0x00);
	ret |= vl6180_write_byte(data->client, 0x00bb, 0x3c);
	ret |= vl6180_write_byte(data->client, 0x00b2, 0x09);
	ret |= vl6180_write_byte(data->client, 0x00ca, 0x09);
	ret |= vl6180_write_byte(data->client, 0x0198, 0x01);
	ret |= vl6180_write_byte(data->client, 0x01b0, 0x17);
	ret |= vl6180_write_byte(data->client, 0x01ad, 0x00);
	ret |= vl6180_write_byte(data->client, 0x00ff, 0x05);
	ret |= vl6180_write_byte(data->client, 0x0100, 0x05);
	ret |= vl6180_write_byte(data->client, 0x0199, 0x05);
	ret |= vl6180_write_byte(data->client, 0x01a6, 0x1b);
	ret |= vl6180_write_byte(data->client, 0x01ac, 0x3e);
	ret |= vl6180_write_byte(data->client, 0x01a7, 0x1f);
	ret |= vl6180_write_byte(data->client, 0x0030, 0x00);

	/* Recommended : Public registers - See data sheet for more detail */
	//vl6180_write_byte(data->client, 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
	//vl6180_write_byte(data->client, 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
	//vl6180_write_byte(data->client, 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
	//vl6180_write_byte(data->client, 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
	//vl6180_write_byte(data->client, 0x0040, 0x63); /* Set ALS integration time to 100ms */
	//vl6180_write_byte(data->client, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

	/* Optional: Public registers - See data sheet for more detail */
	//vl6180_write_byte(data->client, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
	//vl6180_write_byte(data->client, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
	//vl6180_write_byte(data->client, 0x0014, 0x24); /* Configures interrupt on New sample ready */

	return ret;
}

/* Read a single measurement with no interrupt generation */
static int vl6180_measure(struct vl6180_data *data, int addr)
{
	struct i2c_client *client = data->client;
	int tries = 20, ret;
	u16 value;

	mutex_lock(&data->lock);

	if(vl6180_chan_regs_table[addr].continuous_mode == false)
	{
		/* Start single shot measurement */
		ret = vl6180_write_byte(client,
			vl6180_chan_regs_table[addr].start_reg, VL6180_STARTSTOP);
		if (ret < 0) {
			goto fail;
		}

		/* Only poll the interrupt status register if client has no
		 * interrupt defined
		 */
		if(!client->irq)
		{
			while (tries--) {
				ret = vl6180_read_byte(client, VL6180_INTR_STATUS);
				if (ret < 0) {
					goto fail;
				}

				if (ret & vl6180_chan_regs_table[addr].drdy_mask) {
					break;
				}
				msleep(20);
			}

			if (tries < 0) {
				ret = -EIO;
				goto fail;
			}
		}
		else
		{
			//interrupt is set to low or high threshold
			//but single-shot?
			//sleep for at least one intermeasurement period, just
			//to get a measurement
			//don't care about the interrupt at all
			msleep(data->intermeasurement_period);
		}
	}

	/* Read result value from appropriate registers */
	ret = vl6180_chan_regs_table[addr].word ?
		vl6180_read_word(client, vl6180_chan_regs_table[addr].value_reg) :
		vl6180_read_byte(client, vl6180_chan_regs_table[addr].value_reg);
	if (ret < 0) {
		goto fail;
	}
	value = ret;

	/* Clear the interrupt flag if it's single-shot */
	ret = vl6180_write_byte(client, VL6180_INTR_CLEAR,
		VL6180_CLEAR_ERROR | VL6180_CLEAR_ALS | VL6180_CLEAR_RANGE);
	if (ret < 0) {
		goto fail;
	}

	ret = value;

fail:
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_chan_spec vl6180_channels[] = {
	{
		.type = IIO_LIGHT,
		.address = VL6180_ALS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_INT_TIME) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	},
	{
		.type = IIO_DISTANCE,
		.address = VL6180_RANGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
	}
};

/*
 * Columns 3 & 4 represent the same value in decimal and hex notations.
 * Kept in order to avoid the datatype conversion while reading the
 * hardware_gain.
 */
static const int vl6180_als_gain[8][4] = {
	{ 1,	0,      70,	VL6180_ALS_GAIN_1 },
	{ 1,    250000, 69,	VL6180_ALS_GAIN_1_25 },
	{ 1,    670000, 68,	VL6180_ALS_GAIN_1_67 },
	{ 2,    500000, 67,	VL6180_ALS_GAIN_2_5 },
	{ 5,    0,      66,	VL6180_ALS_GAIN_5 },
	{ 10,   0,      65,	VL6180_ALS_GAIN_10 },
	{ 20,   0,      64,	VL6180_ALS_GAIN_20 },
	{ 40,   0,      71,	VL6180_ALS_GAIN_40 }
};

static int vl6180_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct vl6180_data *data = iio_priv(indio_dev);
	int ret, i;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = vl6180_measure(data, chan->address);
		if (ret < 0)
			return ret;
		*val = ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		ret = vl6180_read_word(data->client, VL6180_ALS_IT);
		if (ret < 0)
			return ret;
		*val = 0; /* 1 count = 1ms (0 = 1ms) */
		*val2 = (ret + 1) * 1000; /* convert to microseconds */

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			*val = 0; /* one ALS count is 0.32 Lux */
			*val2 = 320000;
			break;
		case IIO_DISTANCE:
			*val = 0; /* sensor reports mm, scale to meter */
			*val2 = 1000;
			break;
		default:
			return -EINVAL;
		}

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = vl6180_read_byte(data->client, VL6180_ALS_GAIN);
		if (ret < 0) {
			return -EINVAL;
		}

		for (i = 0; i < ARRAY_SIZE(vl6180_als_gain); i++) {
			if (ret == vl6180_als_gain[i][2]) {
				*val = vl6180_als_gain[i][0];
				*val2 = vl6180_als_gain[i][1];
			}
		}

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

/*
 * Configure measurement period, max convergence
 * time and average sample period
 */
static int vl6180_configure_timing(struct vl6180_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	if(	(data->intermeasurement_period < VL6180_INTERMEASUREMENT_PERIOD_MIN) ||
		(data->intermeasurement_period > VL6180_INTERMEASUREMENT_PERIOD_MAX) ){
		if(data->enableDebug) {
			dev_err(&data->client->dev,
					"min is %d ms max is %d ms and rounded-up per %d ms\n",
					VL6180_INTERMEASUREMENT_PERIOD_MIN,
					VL6180_INTERMEASUREMENT_PERIOD_MAX,
					VL6180_INTERMEASUREMENT_PERIOD_MIN);
		}
		return -EINVAL;
	}

	/* Continuous ranging mode only, time period is val * 10ms */
	/* Rounded up for safety measure */
	ret = vl6180_write_byte(client, VL6180_INTERMEASUREMENT_PERIOD,
			((data->intermeasurement_period+9)/10) - 1);

	if (ret < 0) {
		return ret;
	}

	/* **************************************************/
	/* DEPENDS ON INTERMEASUREMENT TIME */
	/* Can cause interrupt to hang if not set correctly */
	/* Right now, re-write these 2 below register, but  */
	/* in the future, it can change along with inter-   */
	/* measurement time                                 */
	/* **************************************************/
	/* 4 = 4ms. range is from 1-63 */
	/* TODO: Does it need to change along with intermeasurement period */
	ret = vl6180_write_byte(client, VL6180_MAX_CONVERGENCE_TIME,
							0x08);
	if (ret < 0) {
		return ret;
	}

	/* 0x18 = 24 * 64.5us = 1548us = 1.548ms + 1.3 fixed overhead = 2.848ms*/
	ret = vl6180_write_byte(client, VL6180_AVG_SAMPLE_PERIOD, 0x30);

	return ret;
}

/*
 *  sysfs enable_debug file to show debug info.
 *  1 - print out debug info
 *  0 - no debug info
 */
static ssize_t vl6180_show_enable_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", data->enableDebug);
}

/* To set sysfs enable_debug file for showing debug info */
static ssize_t vl6180_store_enable_debug(struct device *dev,
					struct device_attribute *attr, const char *buf,
					size_t count)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	u8 on;
	int ret;

	ret = kstrtou8(buf, 10, &on);
	if(ret < 0) {
		return ret;
	}

	if ((on != 0) && (on != 1)) {
		return -EINVAL;
	}

	mutex_lock(&data->lock);
	data->enableDebug = on;
	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(enable_debug, 0660 /*S_IWUSR | S_IRUGO*/,
				   vl6180_show_enable_debug, vl6180_store_enable_debug);

/*
 *  Modify sysfs status. 0 means detected object distance is
 *  less than the low threshold. 1 means higher than high threshold.
 *  It will not return 2, a place holder now
 */
static ssize_t vl6180_show_status(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	uint ret;

	if(data) {
		mutex_lock(&data->lock);

		//Value will be in status file
		if(data->range_value < data->range_low_threshold ) {
			ret = 0;
		}
		else if(data->range_value > data->range_high_threshold) {
			ret = 1;
		}
		else {
			ret = 2;
		}

		mutex_unlock(&data->lock);
	}
	else {
		return -EIO;
	}

	return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR(status, 0440, vl6180_show_status, 0);

/*
 *  sysfs intermeasurement_period file to show measurement period
 *  in millisecond.
 */
static ssize_t vl6180_show_intermeasurement_period_ms(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", data->intermeasurement_period);
}

/*
 *  sysfs intermeasurement_period file to change measurement period
 *  in millisecond.
 *  Valid value is from 10 ms to 2550 ms.
 */
static ssize_t vl6180_store_intermeasurement_period_ms(struct device *dev,
					struct device_attribute *attr, const char *buf,
					size_t count)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	/* Save current intermeasurement period */
	u16 saved_intermeasurement_period =
			data->intermeasurement_period;
	u16 intermeasurement_period;
	int ret;

	ret = kstrtou16(buf, 10, &intermeasurement_period);
	if(ret < 0) {
		return ret;
	}
	else
	{
		mutex_lock(&data->lock);
		/* put new intermeasurement into vl6180_data struct first.
		 * This is done to simplify init/probe function
		 */
		data->intermeasurement_period = intermeasurement_period;

		/* configure timing will check for the range of intermeasurement period */
		ret = vl6180_configure_timing(data);

		if (ret < 0)
		{
			data->intermeasurement_period = saved_intermeasurement_period;
			if(data->enableDebug)
				dev_err(&data->client->dev,
						"Fail to set intermeasurement period %d\n", ret);
			count = ret;
		}

		mutex_unlock(&data->lock);
	}

	return count;
}

static DEVICE_ATTR(intermeasurement_period_ms, 0660 /*S_IWUGO | S_IRUGO*/,
				   vl6180_show_intermeasurement_period_ms,
				   vl6180_store_intermeasurement_period_ms);

/* sysfs threshold_low file to show low threshold in millimeter */
static ssize_t vl6180_show_low_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", data->range_low_threshold);
}

/*
 *  sysfs threshold_low file to change low threshold in millimeter
 *  Valid value is from 0 to 255 mm.
 */
static ssize_t vl6180_store_low_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	u8 low_threshold;
	int ret;

	ret = kstrtou8(buf, 10, &low_threshold);

	if(ret < 0) {
		return ret;
	}
	else
	{
		if(low_threshold > VL6180_THRESH_MAX) {
			return -EINVAL;
		}
		else
		{
			mutex_lock(&data->lock);

			ret = vl6180_write_byte(data->client, VL6180_THRESH_LOW,
					low_threshold);
			if (ret < 0)
			{
				dev_err(&data->client->dev,
						"Fail to set low threshold %02x\n", ret);
				count = ret;
			}
			else
			{
				data->range_low_threshold = low_threshold;
			}

			mutex_unlock(&data->lock);
		}
	}
	return count;
}

static DEVICE_ATTR(threshold_low, 0660 /*S_IWUSR | S_IRUGO*/,
		vl6180_show_low_threshold, vl6180_store_low_threshold);

/* sysfs threshold_high file to show high threshold in millimeter */
static ssize_t vl6180_show_high_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", data->range_high_threshold);
}

/*
 *  sysfs threshold_high file to change high threshold in millimeter
 *  Valid value is from 0 to 255 mm.
 */
static ssize_t vl6180_store_high_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	u8 high_threshold;
	int ret;

	ret = kstrtou8(buf, 10, &high_threshold);

	if(ret < 0) {
		return ret;
	}
	else
	{
		if(high_threshold > VL6180_THRESH_MAX) {
			return -EINVAL;
		}
		else
		{
			mutex_lock(&data->lock);

			ret = vl6180_write_byte(data->client, VL6180_THRESH_HIGH,
					high_threshold);
			if (ret < 0)
			{
				dev_err(&data->client->dev,
						"Fail to set high threshold %02x\n", ret);
				count = ret;
			}
			else
			{
				data->range_high_threshold = high_threshold;
			}

			mutex_unlock(&data->lock);
		}
	}
	return count;
}

static DEVICE_ATTR(threshold_high, 0660 /*S_IWUSR | S_IRUGO*/,
				   vl6180_show_high_threshold, vl6180_store_high_threshold);

/* To set sysfs status file for info about blocked/unblocked */
static ssize_t vl6180_show_fw_version(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct vl6180_data *data = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf,
			"model-module-date-time: %02x.%02x-%02x.%02x-%02x.%02x-%04x\n",
			data->ver_info.model_rev_major,
			data->ver_info.model_rev_minor,
			data->ver_info.module_rev_major,
			data->ver_info.module_rev_minor,
			data->ver_info.id_date_hi,
			data->ver_info.id_date_lo,
			data->ver_info.id_time
			);
}
static DEVICE_ATTR(fw_version, 0440, vl6180_show_fw_version, 0);

static IIO_CONST_ATTR(als_gain_available, "1 1.25 1.67 2.5 5 10 20 40");

static struct attribute *vl6180_attributes[] = {
	&iio_const_attr_als_gain_available.dev_attr.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_intermeasurement_period_ms.attr ,
	&dev_attr_status.attr,
	&dev_attr_threshold_low.attr,
	&dev_attr_threshold_high.attr,
	&dev_attr_fw_version.attr,
	NULL
};

static const struct attribute_group vl6180_attribute_group = {
	.attrs = vl6180_attributes,
};

static int vl6180_set_als_gain(struct vl6180_data *data, int val, int val2)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(vl6180_als_gain); i++) {
		if (val == vl6180_als_gain[i][0] &&
			val2 == vl6180_als_gain[i][1]) {
				mutex_lock(&data->lock);

				ret = vl6180_write_byte(data->client, VL6180_ALS_GAIN,
					vl6180_als_gain[i][3]);

				mutex_unlock(&data->lock);
				return ret;
		}
	}

	return -EINVAL;
}

static int vl6180_set_it(struct vl6180_data *data, int val2)
{
	int ret;

	mutex_lock(&data->lock);

	ret = vl6180_write_word(data->client, VL6180_ALS_IT,
		(val2 - 500) / 1000); /* write value in ms */

	mutex_unlock(&data->lock);

	return ret;
}

static int vl6180_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct vl6180_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		if (val != 0 || val2 < 500 || val2 >= 512500) {
			return -EINVAL;
		}

		return vl6180_set_it(data, val2);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->type != IIO_LIGHT) {
			return -EINVAL;
		}

		return vl6180_set_als_gain(data, val, val2);
	default:
		return -EINVAL;
	}
}

static const struct iio_info vl6180_info = {
	.read_raw = vl6180_read_raw,
	.write_raw = vl6180_write_raw,
	.attrs = &vl6180_attribute_group,
	.driver_module = THIS_MODULE,
};

/* Called by probe function to set interrupt and create gpiochip sysfs */
static int vl6180_set_interrupt(struct vl6180_data *data)
{
	int ret;

	/* VL6180_MODE_GPIO0 */
	ret = vl6180_write_byte(data->client, VL6180_MODE_GPIO0,
							VL6180_GPIO_INT_ENABLE);
	if(ret < 0) {
		return ret;
	}

	ret = vl6180_hold(data, true);
	if (ret < 0) {
		return ret;
	}

	/* Enable ALS and Range ready interrupts */
	ret = vl6180_write_byte(data->client, VL6180_INTR_CONFIG,
			VL6180_RANGE_OUT_OF_WINDOW);
	if (ret < 0)
		goto release;

	/* Configure low threshold in mm*/
	ret = vl6180_write_byte(data->client, VL6180_THRESH_LOW,
							data->range_low_threshold);
	if (ret < 0)
		goto release;

	/* Configure HIGH threshold in mm*/
	ret = vl6180_write_byte(data->client, VL6180_THRESH_HIGH,
							data->range_high_threshold);
	if (ret < 0)
		goto release;

release:
	vl6180_hold(data, false);
	return ret;
}

static int vl6180_read_version_info(struct vl6180_data *data)
{
	int ret = 0;

	mutex_lock(&data->lock);

	ret = vl6180_read_byte(data->client, VL6180_MODEL_REV_MAJOR);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.model_rev_major = ret & 0xFF;

	ret = vl6180_read_byte(data->client, VL6180_MODEL_REV_MINOR);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.model_rev_minor = ret & 0xFF;

	ret = vl6180_read_byte(data->client, VL6180_MODULE_REV_MAJOR);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.module_rev_major = ret & 0xFF;

	ret = vl6180_read_byte(data->client, VL6180_MODULE_REV_MINOR);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.module_rev_minor = ret & 0xFF;

	ret = vl6180_read_byte(data->client, VL6180_DATE_HI);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.id_date_hi = ret & 0xFF;

	ret = vl6180_read_byte(data->client, VL6180_DATE_LO);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.id_date_lo = ret & 0xFF;

	/* Read word will swap bytes to big endian */
	ret = vl6180_read_word(data->client, VL6180_TIME);
	if(ret < 0)
		goto fail;
	else
		data->ver_info.id_time = ret & 0xFFFF;

	/* Now compare all the model and module major minor version */
	if(data->ver_info.model_rev_major != VL6180_MODEL_REV_MAJOR_VAL)
		dev_alert(&data->client->dev, "Model Rev. major mismatch %02x",ret);

	if(data->ver_info.model_rev_minor != VL6180_MODEL_REV_MINOR_VAL)
		dev_alert(&data->client->dev, "Model Rev. minor mismatch %02x",ret);

	if(data->ver_info.module_rev_major != VL6180_MODULE_REV_MAJOR_VAL)
		dev_alert(&data->client->dev, "Module Rev. major mismatch %02x",ret);

	if(data->ver_info.module_rev_minor != VL6180_MODULE_REV_MINOR_VAL)
		dev_alert(&data->client->dev, "Module Rev. minor mismatch %02x",ret);

fail:
	mutex_unlock(&data->lock);
	return ret;
}

static int vl6180_init(struct vl6180_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	ret = vl6180_read_byte(client, VL6180_MODEL_ID);
	if (ret < 0)
		return ret;

	if (ret != VL6180_MODEL_ID_VAL) {
		dev_err(&client->dev, "invalid model ID %02x\n", ret);
		return -ENODEV;
	}

	/* Read version info, reg 0x1 to 0x9, not 0x5 */
	ret = vl6180_read_version_info(data);
	if(ret < 0)
		return ret;

	/* Run the Static Init only after vl6180 dev is found */
	ret = vl6180_range_static_init(data);
	if(ret < 0)
		return ret;

	ret = vl6180_configure_timing(data);
	if(ret < 0)
		return ret;

	/* Enable ALS and Range ready interrupts */
	ret = vl6180_write_byte(client, VL6180_INTR_CONFIG,
				VL6180_ALS_READY | VL6180_RANGE_READY);
	if (ret < 0)
		return ret;

	/* ALS integration time: 100ms */
	ret = vl6180_write_word(client, VL6180_ALS_IT, VL6180_ALS_IT_100);
	if (ret < 0)
		return ret;

	/* ALS gain: 1 */
	ret = vl6180_write_byte(client, VL6180_ALS_GAIN, VL6180_ALS_GAIN_1);
	if (ret < 0)
		return ret;

	/* Disable early convergence measurement */
	ret = vl6180_write_byte(client, VL6180_RANGE_CHECK_ENABLE,
							VL6180_RANGE_CHECK_ENABLE_VALUE);
	if (ret < 0)
		return ret;

	ret = vl6180_write_byte(client, VL6180_OUT_OF_RESET, 0x00);

	return ret;
}

static irqreturn_t vl6180_irq_handler(int irq, void *private)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t vl6180_irq_thread_handler(int irq, void *private)
{
	int ret = 0 , range_status, intr_status, range_value;
	struct iio_dev *indio_dev = private;
	struct vl6180_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	mutex_lock(&data->lock);

	/* Read range value */
	range_value = vl6180_read_byte(client, VL6180_RANGE_VALUE);
	if( range_value < 0)
	{
		dev_err_ratelimited(&client->dev, "Unknown range value %02x\n",
				range_value);
		ret = range_value;
		goto clear_interrupt;
	}

	/* print debug info only if it's enabled */
	if(data->enableDebug)
	{
		intr_status = vl6180_read_byte(client, VL6180_INTR_STATUS);
		if( intr_status < 0) {
			dev_err_ratelimited(&client->dev,
					"Unknown interrupt status %02x\n", intr_status);
			ret = intr_status;
			goto clear_interrupt;
		}

		range_status = vl6180_read_byte(client, VL6180_RANGE_STATUS);
		if( range_status < 0) {
			dev_err_ratelimited(&client->dev,
					"Unknown range_status %02x\n", range_status);
			ret = range_status;
			goto clear_interrupt;
		}

		dev_info_ratelimited(&client->dev,
				"intr_sts:%02x range_status:%02x range_value:%d\n",
				intr_status,
				range_status,
				range_value);
	}

	// High or Low Threshold switching depending on the range value
	if(range_value < data->range_low_threshold)
	{
		data->range_value = range_value;

		ret = vl6180_write_byte(client, VL6180_INTR_CONFIG,
				VL6180_RANGE_HIGH);
		if (ret < 0)
			goto clear_interrupt;
	}
	else if( range_value > data->range_high_threshold)
	{
		data->range_value = range_value;

		ret = vl6180_write_byte(client, VL6180_INTR_CONFIG,
				VL6180_RANGE_LOW);
		if (ret < 0)
			goto clear_interrupt;
	}
	else
	{
		/* It should not go here, unless the interrupt is enabled
		 * while calling vl6180_measure with an object between low threshold
		 * and high threshold
		 * This can also happen if the delay between the HW interrupt and when
		 * this thread handler executes is too long, and the object's distance
		 * becomes not within the threshold anymore
		 * It is not an error case since it can happen, but print info anyways
		 */
		dev_info_ratelimited(&client->dev,
				"Range is not within threshold: %d\n", range_value);
		goto clear_interrupt;
	}

	/* update status sysfs file so that userapp know
	 * whether the sensor is blocked/unblocked */
	sysfs_notify(&indio_dev->dev.kobj, NULL, "status");

	if(data->enableDebug) {
		dev_info_ratelimited(&data->client->dev, "notify\n");
	}

clear_interrupt:
	/* Clear the interrupt */
	if(vl6180_write_byte(data->client, VL6180_INTR_CLEAR,
							VL6180_CLEAR_ERROR |
							VL6180_CLEAR_ALS |
							VL6180_CLEAR_RANGE))
	{
		dev_err_ratelimited(&client->dev, "clear interrupt error.\n");
	}

	mutex_unlock(&data->lock);

	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

/* Get intermeasurement period, low threshold and high threshold from
 * device tree. If not defined, it will be set to default value
 */
static void vl6180_get_platform_data(struct iio_dev *indio_dev)
{
	struct vl6180_data *data = iio_priv(indio_dev);
	u32 temp;

	// Using of_property_read_u32 to avoid change device tree with /bits/
	if(!of_property_read_u32(data->client->dev.of_node,
							"intermeasurement-period",
							&temp))
	{
		if(temp > VL6180_INTERMEASUREMENT_PERIOD_MAX)
			data->intermeasurement_period = VL6180_INTERMEASUREMENT_PERIOD_MAX;
		else if(temp < VL6180_INTERMEASUREMENT_PERIOD_MIN)
			data->intermeasurement_period = VL6180_INTERMEASUREMENT_PERIOD_MIN;
		else
			data->intermeasurement_period = (u16)(temp & 0xFFFF);
	}
	else
	{
		data->intermeasurement_period = VL6180_INTERMEASUREMENT_PERIOD_DEFAULT;
	}

	if(!of_property_read_u32(data->client->dev.of_node,
							"range-low-threshold",
							&temp))
	{
		if(temp > VL6180_THRESH_MAX)
			data->range_low_threshold = VL6180_THRESH_MAX;
		else
			data->range_low_threshold = (u8)(temp & 0xFF);
	}
	else
	{
		data->range_low_threshold = VL6180_THRESH_LOW_DEFAULT;
	}

	if(!of_property_read_u32(data->client->dev.of_node,
							"range-high-threshold",
							&temp))
	{
		if(temp > VL6180_THRESH_MAX)
			data->range_high_threshold = VL6180_THRESH_MAX;
		else
			data->range_high_threshold = (u8)(temp & 0xFF);
	}
	else
	{
		data->range_high_threshold = VL6180_THRESH_HIGH_DEFAULT;
	}
}

/*
 *  Set I2C address of VL6180 from the default 0x29
 */
static int vl6180_set_device_address(struct i2c_client *client,
		unsigned short new_address)
{
	/* If address used in device tree is not default 0x29
	 * then program it to the device, then update the i2c_client struct. */
	if (vl6180_write_byte(client,
			VL6180_I2C_SLAVE_DEVICE_ADDRESS,
			new_address) < 0)
	{
		dev_err(&client->dev, "%s: fail\n", __func__);
		return -EIO;
	}
	else
	{
		client->addr = new_address;
		dev_info(&client->dev, "%s:0x%02x\n", __func__, client->addr);
		return 0;
	}
}

static int vl6180_is_fw_bootup(struct i2c_client *client)
{
	return vl6180_read_byte(client, VL6180_FIRMWARE_BOOTUP);
}

static int vl6180_is_ready(struct i2c_client *client)
{
	int ret;
	u8 val;
	int retries = 10;

	while(retries--)
	{
		ret = vl6180_is_fw_bootup(client);
		if(ret & VL6180_FIRMWARE_BOOTUP_MASK) {
			break;
		}

		//At most sleep for 1 ms for sensor boot up time
		//without sleep, it takes ~11ms
		//With msleep(1), it takes ~37ms
		msleep(1);
	}

	if(retries < 0) {
		dev_err(&client->dev, "fw boot up fail:0x%02x\n", ret);
		return -EIO;
	}

	// Detect false reset condition here. This bit is always set when the
	// system comes out of reset.
	ret = vl6180_read_byte(client, VL6180_OUT_OF_RESET);
	if (ret < 0) {
		return ret;
	}

	// Check bit 0 of reg 0x16 has to be set to 1,
	// otherwise just print error right now
	if(ret & BIT(0)) {
		//Clear bit 0
		val = ret & ~BIT(0);
		ret = vl6180_write_byte(client, VL6180_OUT_OF_RESET, val);
	} else {
		dev_info(&client->dev,
				"device is not fresh out of reset:0x%02x\n", ret);
	}

	return ret;
}

static int vl6180_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct vl6180_data *data;
	struct iio_dev *indio_dev;
	int ret;
	unsigned short new_addr = VL6180_DEFAULT_I2C_ADDR;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	data->power_supply = devm_regulator_get(&client->dev, "power");
	if (IS_ERR_OR_NULL(data->power_supply)){
		return -EPROBE_DEFER;
	}

	ret = regulator_enable(data->power_supply);
	if(ret){
		return ret;
	}

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &vl6180_info;
	indio_dev->channels = vl6180_channels;
	indio_dev->num_channels = ARRAY_SIZE(vl6180_channels);
	indio_dev->name = client->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	vl6180_get_platform_data(indio_dev);

	/* Set default device address first */
	if(client->addr != VL6180_DEFAULT_I2C_ADDR)
	{
		new_addr = client->addr & 0xFF;
		client->addr = VL6180_DEFAULT_I2C_ADDR;
	}

	/* Wait for device ready */
	ret = vl6180_is_ready(client);
	if(ret < 0){
		client->addr = new_addr;
		ret = vl6180_is_ready(client);
		if(ret < 0) {
			dev_err(&client->dev,
					"device is not responding to addr 0x%02x and 0x%02x\n",
					VL6180_DEFAULT_I2C_ADDR,
					client->addr);
			return ret;
		}
	}

	/* Not checking return value, if it fails,
	 * try to continue with default address
	 * only if it's successful, new_addr will be updated to
	 * vl6180_data structure */
	vl6180_set_device_address(client, new_addr);

	ret = vl6180_init(data);
	if (ret < 0) {
		return ret;
	}

	if(client->irq)
	{
		ret = vl6180_set_interrupt(data);
		if(ret < 0) {
			dev_info(&client->dev, "gpio interrupt is not set\n");
		}

		ret = devm_request_threaded_irq(&client->dev, client->irq,
									vl6180_irq_handler,
									vl6180_irq_thread_handler,
									IRQF_TRIGGER_LOW | IRQF_ONESHOT ,
									client->name, indio_dev);
		if (ret) {
			dev_err(&client->dev, "%s: request irq %d failed\n",
					__func__, client->irq);
			return ret;
		}
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "%s: register device failed\n", __func__);
		return ret;
	}

	if(vl6180_chan_regs_table[VL6180_RANGE].continuous_mode == true)
	{
		ret = vl6180_write_byte(client, VL6180_RANGE_START,
								 VL6180_MODE_CONT | VL6180_STARTSTOP);
		if (ret < 0)
			return ret;
	}

	/* Remove file status if there is no interrupt used */
	if(!client->irq)
	{
		/* Remove status, threshold_low and threshold_high files
		 * if no interrupt is configured
		 */
		device_remove_file(&indio_dev->dev,
				&dev_attr_status);
		device_remove_file(&indio_dev->dev,
				&dev_attr_threshold_low);
		device_remove_file(&indio_dev->dev,
				&dev_attr_threshold_high);
	}

	return 0;
}

static int vl6180_remove(struct i2c_client *client)
{
	int ret;
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct vl6180_data *data = iio_priv(indio_dev);

	/* Only issue this command in continuous mode at remove */
	if(vl6180_chan_regs_table[VL6180_RANGE].continuous_mode == true)
	{
		/* Stop continuous mode */
		ret = vl6180_write_byte(client, VL6180_RANGE_START,
								 VL6180_MODE_CONT | VL6180_STARTSTOP);
		if (ret < 0)
		{
			dev_err(&client->dev, "%s: stop failed %d\n",
								__func__, ret);
			//even though it's error, just keep continuing to remove
		}
	}

	/* Write back default address */
	/* Not checking return value, if it fails, continue with remove */
	/* Write back default address */
	if(client->addr != VL6180_DEFAULT_I2C_ADDR) {
		vl6180_write_byte(client, VL6180_I2C_SLAVE_DEVICE_ADDRESS,
						VL6180_DEFAULT_I2C_ADDR);
	}

	ret = regulator_disable(data->power_supply);
	if (ret < 0)
	{
		dev_err(&client->dev, "%s: regulator disable failed %d\n",
							__func__, ret);
		return ret;
	}

	devm_iio_device_unregister(&client->dev, indio_dev);

	if(client->irq) {
		devm_free_irq(&client->dev, client->irq, indio_dev);
	}

	return 0;
}

static const struct of_device_id vl6180_of_match[] = {
	{ .compatible = "st,vl6180", },
	{ },
};
MODULE_DEVICE_TABLE(of, vl6180_of_match);

static const struct i2c_device_id vl6180_id[] = {
	{ "vl6180", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vl6180_id);

static struct i2c_driver vl6180_driver = {
	.driver = {
		.name   = VL6180_DRV_NAME,
		.of_match_table = of_match_ptr(vl6180_of_match),
	},
	.probe  = vl6180_probe,
	.remove = vl6180_remove,
	.id_table = vl6180_id,
};

module_i2c_driver(vl6180_driver);

MODULE_AUTHOR("Budi Sendjaja <sendjajab@cumminsallison.com");
MODULE_AUTHOR("Peter Meerwald-Stadler <pmeerw@pmeerw.net>");
MODULE_AUTHOR("Manivannan Sadhasivam <manivannanece23@gmail.com>");
MODULE_DESCRIPTION("STMicro VL6180 ALS, range and proximity sensor driver");
MODULE_LICENSE("GPL");
