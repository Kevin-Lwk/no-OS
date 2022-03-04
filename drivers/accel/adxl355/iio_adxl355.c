/***************************************************************************//**
 *   @file   iio_adxl355.c
 *   @brief  Implementation of IIO ADXL355 Driver.
 *   @author RBolboac (ramona.bolboaca@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "no-os/error.h"
#include "no-os/delay.h"
#include "iio.h"
#include "iio_adxl355.h"
#include "no-os/util.h"
#include "adxl355.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define ACCEL_AXIS_X (uint32_t) 1
#define ACCEL_AXIS_Y (uint32_t) 2
#define ACCEL_AXIS_Z (uint32_t) 3
#define ADXL355_NEG_OFS_MSK        GENMASK(31, 16)

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
extern int adxl355_read_device_data(struct adxl355_dev *dev,
				    uint8_t base_address,
				    uint16_t size, uint8_t *read_data);

extern int adxl355_write_device_data(struct adxl355_dev *dev,
				     uint8_t base_address,
				     uint16_t size, uint8_t *write_data);
static int adxl355_find_odr_idx(const int32_t integer, const int32_t frac);
static int32_t adxl355_find_hpf_idx(uint8_t odr_idx, char *buf, uint8_t length);
static int32_t adxl355_read_reg(struct adxl355_iio_dev *dev, uint32_t reg,
				uint32_t *readval);
static int32_t adxl355_write_reg(struct adxl355_iio_dev *dev, uint32_t reg,
				 uint32_t writeval);
static int adxl355_iio_read_raw(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_converted(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_offset(void *device, char *buf, uint32_t len,
				   const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_scale(void *device, char *buf, uint32_t len,
				  const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_calibbias(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_write_calibbias(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_hpf(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_write_hpf(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_sampling_freq(void *device, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_write_sampling_freq(void *device, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_hpf_available(void *device, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int adxl355_iio_read_samp_freq_avail(void *device, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv);

/******************************************************************************/
/************************ Variable Declarations ******************************/
/******************************************************************************/
static const int adxl355_odr_table[11][2] = {
	{4000, 0},
	{2000, 0},
	{1000, 0},
	{500, 0},
	{250, 0},
	{125, 0},
	{62, 500000},
	{31, 250000},
	{15, 625000},
	{7, 813000},
	{3, 906000},
};

static const char* adxl355_hpf_table[11][7] = {
	{"0", "9.88",       "2.48336",       "0.6218",        "0.15448",       "0.03816",       "0.00952"},
	{"0", "4.94",       "1.24168",       "0.3109",        "0.07724",       "0.01908",       "0.00476"},
	{"0", "2.47",       "0.62084",       "0.15545",       "0.03862",       "0.00954",       "0.00238"},
	{"0", "1.235",      "0.31042",       "0.077725",      "0.01931",       "0.00477",       "0.00119"},
	{"0", "0.6175",     "0.15521",       "0.0388625",     "0.009655",      "0.002385",      "0.000595"},
	{"0", "0.30875",    "0.077605",      "0.01943125",    "0.0048275",     "0.0011925",     "0.0002975"},
	{"0", "0.154375",   "0.0388025",     "0.009715625",   "0.00241375",    "0.00059625",    "0.00014875"},
	{"0", "0.0771875",  "0.01940125",    "0.0048578125",  "0.001206875",   "0.000298125",   "0.000074375"},
	{"0", "0.03859375", "0.009700625",   "0.00242890625", "0.0006034375",  "0.0001490625",  "0.0000371875"},
	{"0", "0.01929811", "0.00485062292", "0.00121453085", "0.00030173806", "0.00007453602", "0.00001859494"},
	{"0", "0.00964782", "0.00242500104", "0.0006071877",  "0.00015084972", "0.00003726324", "0.00000929628"}
};

static struct iio_attribute adxl355_iio_temp_attrs[] = {
	{
		.name = "raw",
		.show = adxl355_iio_read_raw,
		.store = NULL,
	},
	{
		.name = "converted",
		.show = adxl355_iio_read_converted,
		.store = NULL,
	},
	{
		.name = "scale",
		.show = adxl355_iio_read_scale,
		.store = NULL,
	},
	{
		.name = "offset",
		.show = adxl355_iio_read_offset,
		.store = NULL,
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute adxl355_iio_accel_attrs[] = {
	{
		.name = "raw",
		.show = adxl355_iio_read_raw,
	},
	{
		.name = "converted",
		.show = adxl355_iio_read_converted,
	},
	{
		.name   = "scale",
		.shared = IIO_SHARED_BY_TYPE,
		.show   = adxl355_iio_read_scale,
	},
	{
		.name  = "calibbias",
		.show  = adxl355_iio_read_calibbias,
		.store = adxl355_iio_write_calibbias,
	},
	{
		// TODO: if too long the writing procedure does not work
		// issue due to iiod processing, uart data is missed
		.name   = "filter_high_pass_3db_frequency",
		.shared = IIO_SHARED_BY_TYPE,
		.show   = adxl355_iio_read_hpf,
		.store  = adxl355_iio_write_hpf,
	},
	{
		.name   = "sampling_frequency",
		.shared = IIO_SHARED_BY_TYPE,
		.show   = adxl355_iio_read_sampling_freq,
		.store  = adxl355_iio_write_sampling_freq,
	},
	{
		.name   = "filter_high_pass_3db_frequency_available",
		.shared = IIO_SHARED_BY_TYPE,
		.show   = adxl355_iio_read_hpf_available,
	},
	{
		.name   = "sampling_frequency_available",
		.shared = IIO_SHARED_BY_TYPE,
		.show   = adxl355_iio_read_samp_freq_avail,
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_channel adxl355_channels[] = {
	{
		.name = "Accelleration X Axis",
		.ch_type = IIO_ACCEL,
		.indexed = true,
		.channel = 0,
		.attributes = adxl355_iio_accel_attrs,
		.address = ACCEL_AXIS_X,
		.ch_out = false,
	},
	{
		.name = "Acceleration Y Axis",
		.ch_type = IIO_ACCEL,
		.indexed = true,
		.channel = 1,
		.attributes = adxl355_iio_accel_attrs,
		.address = ACCEL_AXIS_Y,
		.ch_out = false,
	},
	{
		.name = "Acceleration Z Axis",
		.ch_type = IIO_ACCEL,
		.indexed = true,
		.channel = 2,
		.attributes = adxl355_iio_accel_attrs,
		.address = ACCEL_AXIS_Z,
		.ch_out = false,
	},
	{
		.name = "Temperature",
		.ch_type = IIO_TEMP,
		.indexed = true,
		.channel = 3,
		.attributes = adxl355_iio_temp_attrs,
		.ch_out = false,
	},
};

static struct iio_device adxl355_iio_dev = {
	.num_ch = ARRAY_SIZE(adxl355_channels),
	.channels = adxl355_channels,
	.attributes = NULL,
	.debug_attributes = NULL,
	.buffer_attributes = NULL,
	.pre_enable = NULL,
	.post_disable = NULL,
	.read_dev = NULL,
	.debug_reg_read = adxl355_read_reg,
	.debug_reg_write = adxl355_write_reg
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief Searches for match in the output data rate table
 *
 * @param integer  - Integer part of the output data rate to be searched
 * @param frac	   - Fractional part of the output data rate to be searched
 *
 * @return ret     - Table index corresponding to the encoding of the ODR
*******************************************************************************/
static int adxl355_find_odr_idx(const int32_t integer, const int32_t frac)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(adxl355_odr_table); i++) {
		if (adxl355_odr_table[i][0] == integer && adxl355_odr_table[i][1] == frac)
			return i;
	}

	return -EINVAL;
}

/***************************************************************************//**
 * @brief Searches for match in the hpf table
 *
 * @param odr_idx  - Index (encoding) for the ODR setting
 * @param buf	   - Value of the hpf to be searched
 * @param length   - Size of the buf
 *
 * @return ret     - Table index corresponding to the encoding of the hpf
*******************************************************************************/
static int32_t adxl355_find_hpf_idx(uint8_t odr_idx, char *buf, uint8_t length)
{
	for (uint8_t i = 0; i < 7; i++) {
		if (strncmp(adxl355_hpf_table[odr_idx][i], buf, length) == 0)
			return i;
	}

	return -EINVAL;
}

/***************************************************************************//**
 * @brief Wrapper for reading ADXL355 register.
 *
 * @param device  - The iio device structure.
 * @param reg	  - Address of the register to be read from.
 * @param readval - Read data.
 *
 * @return ret    - Result of the reading procedure.
*******************************************************************************/
static int32_t adxl355_read_reg(struct adxl355_iio_dev *dev, uint32_t reg,
				uint32_t *readval)
{
	return adxl355_read_device_data(dev->adxl355_dev, reg, 1, (uint8_t *)readval);
}

/***************************************************************************//**
 * @brief Wrapper for writing to ADXL355 register.
 *
 * @param device   - The iio device structure.
 * @param reg	   - Address of the register to be written to.
 * @param writeval - Data to be written.
 *
 * @return ret    - Result of the writing procedure.
*******************************************************************************/
static int32_t adxl355_write_reg(struct adxl355_iio_dev *dev, uint32_t reg,
				 uint32_t writeval)
{
	uint8_t val = writeval;
	return adxl355_write_device_data(dev->adxl355_dev, reg, 1, &val);
}

/***************************************************************************//**
 * @brief Handles the read request for raw attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_raw(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t ret;
	uint16_t temp_raw;
	uint32_t accel_x;
	uint32_t accel_y;
	uint32_t accel_z;
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		ret = adxl355_get_raw_xyz(adxl355, &accel_x, &accel_y, &accel_z);
		if (ret)
			return ret;
		switch (channel->address) {
		case ACCEL_AXIS_X:
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &accel_x);
		case ACCEL_AXIS_Y:
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &accel_y);
		case ACCEL_AXIS_Z:
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &accel_z);

		default:
			return -EINVAL;
		}

	case IIO_TEMP:
		ret = adxl355_get_raw_temp(adxl355, &temp_raw);
		if (ret)
			return ret;
		return iio_format_value(buf, len, IIO_VAL_INT, 1, &ret);

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for converted attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_converted(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t ret;
	int32_t vals[2];
	struct adxl355_frac_repr temp;
	struct adxl355_frac_repr accel_x;
	struct adxl355_frac_repr accel_y;
	struct adxl355_frac_repr accel_z;
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		ret = adxl355_get_xyz(adxl355, &accel_x, &accel_y, &accel_z);
		if (ret)
			return ret;
		switch (channel->address) {
		case ACCEL_AXIS_X:
			vals[0] = accel_x.integer;
			vals[1] = abs(accel_x.fractional);
			return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
		case ACCEL_AXIS_Y:
			vals[0] = accel_y.integer;
			vals[1] = abs(accel_y.fractional);
			return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
			vals[0] = accel_z.integer;
			vals[1] = abs(accel_z.fractional);
			return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);

		default:
			return -EINVAL;
		}
	case IIO_TEMP:
		ret = adxl355_get_temp(adxl355, &temp);
		if (ret)
			return ret;
		vals[0] = temp.integer*10000+temp.fractional/10000;
		vals[1] = 10000000;
		return iio_format_value(buf, len, IIO_VAL_FRACTIONAL, 2, vals);

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for offset attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_offset(void *device, char *buf, uint32_t len,
				   const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2];

	switch (channel->type) {
	case IIO_TEMP:
		vals[0] = -2111;
		vals[1] = 250000;
		return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for scale attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_scale(void *device, char *buf, uint32_t len,
				  const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2];

	switch (channel->type) {
	case IIO_ACCEL:
		vals[0] = 0;
		vals[1] = 38245;
		return iio_format_value(buf, len, IIO_VAL_INT_PLUS_NANO, 2, vals);
	case IIO_TEMP:
		vals[0] = -110;
		vals[1] = 497238;
		return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for calibbias attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_calibbias(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t val;

	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		switch (channel->address) {
		case ACCEL_AXIS_X:
			val = sign_extend32(adxl355->x_offset,15);
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
		case ACCEL_AXIS_Y:
			val = sign_extend32(adxl355->y_offset,15);
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
		case ACCEL_AXIS_Z:
			val = sign_extend32(adxl355->z_offset,15);
			return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);

		default:
			return -EINVAL;;
		}

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the write request for calibbias attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with the data to be written.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the writing procedure.
*******************************************************************************/
static int adxl355_iio_write_calibbias(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t ret;
	int32_t val;
	uint16_t calibbias;
	enum adxl355_op_mode op_mode;

	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		iio_parse_value(buf, IIO_VAL_INT, &val, NULL);
		// Change this value to two's complement with sign bit = BIT15
		if(val < 0)
			calibbias = BIT(15) | ~(abs(val)) + 1;
		else
			calibbias = val;

		// The device has to be in standby mode in order to be able to
		// modify the offset.
		// Obtain the current op mode
		op_mode = adxl355->op_mode;
		// Set the device in standby mode
		ret = adxl355_set_op_mode(adxl355, ADXL355_STDBY_TEMP_OFF_DRDY_OFF);
		if (ret)
			return ret;

		// Set the offset on the requested axis
		switch (channel->address) {
		case ACCEL_AXIS_X:
			ret = adxl355_set_offset(adxl355, calibbias, adxl355->y_offset,
						 adxl355->z_offset);
			break;
		case ACCEL_AXIS_Y:
			ret = adxl355_set_offset(adxl355, adxl355->x_offset, calibbias,
						 adxl355->z_offset);
			break;
		case ACCEL_AXIS_Z:
			ret = adxl355_set_offset(adxl355, adxl355->x_offset, adxl355->y_offset,
						 calibbias);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		if (ret) {
			adxl355_set_op_mode(adxl355, op_mode);
			return ret;
		}

		return adxl355_set_op_mode(adxl355, op_mode);

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for filter_low attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_hpf(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel, intptr_t priv)
{
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		// Write the value corresponding to the current ODR and HPF settings
		// TODO: check if this procedure is accepted
		strcpy(buf, adxl355_hpf_table[adxl355->odr_lpf][adxl355->hpf_corner]);
		return strlen(buf);
	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the write request for filter_low attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with the data to be written.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the writing procedure.
*******************************************************************************/
static int adxl355_iio_write_hpf(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel, intptr_t priv)
{
	int hpf_idx;
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		// Search for the given data in the HPF available data for the current ODR
		hpf_idx = adxl355_find_hpf_idx(adxl355->odr_lpf, buf,len-1);
		if (hpf_idx < 0)
			return hpf_idx;
		// Set the HPF value
		return adxl355_set_hpf_corner(adxl355, hpf_idx);

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for sampling_frequency attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_sampling_freq(void *device, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2];
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		vals[0] = adxl355_odr_table[adxl355->odr_lpf][0];
		vals[1] = adxl355_odr_table[adxl355->odr_lpf][1];
		return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the write request for sampling_frequency attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with the data to be written.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the writing procedure.
*******************************************************************************/
static int adxl355_iio_write_sampling_freq(void *device, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2];
	int odr_idx;
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &vals[0], &vals[1]);
		// Search for the given data in the ODR available data
		odr_idx = adxl355_find_odr_idx(vals[0], vals[1]);
		if (odr_idx < 0)
			return odr_idx;
		// Set the ODR value
		return adxl355_set_odr_lpf(adxl355, odr_idx);

	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for filter_high_pass_3db_frequency_available
 * 		  attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_hpf_available(void *device, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct adxl355_iio_dev *iio_adxl355 = (struct adxl355_iio_dev *)device;
	struct adxl355_dev *adxl355 = (struct adxl355_dev *)iio_adxl355->adxl355_dev;

	switch (channel->type) {
	case IIO_ACCEL:
		// Remove data existing in the buffer
		strcpy(buf, "");

		// Go through the values in HPF table for te current ODR and add them to the buffer
		for (int8_t i = 0; i < 7; i++) {
			strcat(buf, adxl355_hpf_table[adxl355->odr_lpf][i]);
			strcat(buf, " ");
		}
		return strlen(buf);
	default:
		return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Handles the read request for sampling_frequency_available attribute.
 *
 * @param device  - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel id.
 * @param priv    - Command attribute id.
 *
 * @return ret    - Result of the reading procedure.
 * 					In case of success, the size of the read data is returned.
*******************************************************************************/
static int adxl355_iio_read_samp_freq_avail(void *device, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2];
	char buffer[20];

	switch (channel->type) {
	case IIO_ACCEL:
		// Remove data existing in the buffer
		strcpy(buf, "");

		// Go through the values in ODR table and add them to the buffer
		for (int8_t i = 0; i < ARRAY_SIZE(adxl355_odr_table); i++) {
			vals[0] = adxl355_odr_table[i][0];
			vals[1] = adxl355_odr_table[i][1];
			iio_format_value(buffer, sizeof(buffer), IIO_VAL_INT_PLUS_MICRO, 2, vals);
			strcat(buf, buffer);
			strcat(buf, " ");
		}
		return strlen(buf);
	default:
		return -EINVAL;
	}
}



/***************************************************************************//**
 * @brief Initializes the ADXL355 IIO driver
 *
 * @param iio_dev    - The iio device structure.
 * @param init_param - The structure that contains the device initial
 * 		       		   parameters.
 *
 * @return ret       - Result of the initialization procedure.
*******************************************************************************/
int32_t adxl355_iio_init(struct adxl355_iio_dev **iio_dev,
			 struct adxl355_iio_init_param *init_param)
{
	int32_t ret;
	struct adxl355_iio_dev *desc;

	desc = (struct adxl355_iio_dev *)calloc(1, sizeof(*desc));
	if (!desc)
		return -ENOMEM;

	desc->iio_dev = &adxl355_iio_dev;

	// Initialize ADXL355 driver
	ret = adxl355_init(&desc->adxl355_dev, *(init_param->adxl355_initial));
	if (ret != SUCCESS)
		goto error_adlx355_init;

	// Perform soft reset
	// In some cases it not might be successfull, retrial mechanism can be implemented
	ret = adxl355_soft_reset(desc->adxl355_dev);
	if (ret != SUCCESS)
		goto error_config;

	// Set output data rate
	ret = adxl355_set_odr_lpf(desc->adxl355_dev, ADXL355_ODR_3_906HZ);
	if (ret != SUCCESS)
		goto error_config;

	// Set operation mode
	ret = adxl355_set_op_mode(desc->adxl355_dev,
				  ADXL355_MEAS_TEMP_ON_DRDY_OFF);
	if (ret != SUCCESS)
		goto error_config;

	*iio_dev = desc;

	return SUCCESS;

error_adlx355_init:
	free(desc);
	return ret;
error_config:
	adxl355_remove(desc->adxl355_dev);
	free(desc);
	return ret;
}

/***************************************************************************//**
 * @brief Free the resources allocated by adxl355_iio_init().
 *
 * @param desc - The IIO device structure.
 *
 * @return ret - Result of the remove procedure.
*******************************************************************************/
int32_t adxl355_iio_remove(struct adxl355_iio_dev *desc)
{
	int32_t ret;

	ret = adxl355_remove(desc->adxl355_dev);
	if (ret != SUCCESS)
		return ret;

	free(desc);

	return SUCCESS;
}
