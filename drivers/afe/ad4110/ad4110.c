/***************************************************************************//**
 *   @file   ad4110.c
 *   @brief  Implementation of AD4110 Driver.
 *   @author Stefan Popa (stefan.popa@analog.com)
 * 	     Andrei Porumb (andrei.porumb@analog.com)
 * 	     Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include "ad4110.h"
#include "no-os/error.h"
#include "no-os/irq.h"
#include "no-os/print_log.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/
/***************************************************************************//**
 * Compute CRC8 checksum.
 *
 * @param data      - The data buffer.
 * @param data_size - The size of the data buffer.
 *
 * @return CRC8 checksum.
*******************************************************************************/
uint8_t ad4110_compute_crc8(uint8_t *data,
			    uint8_t data_size)
{
	uint8_t i;
	uint8_t crc = 0;

	while (data_size) {
		for (i = 0x80; i != 0; i >>= 1) {
			if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
				crc <<= 1;
				crc ^= AD4110_CRC8_POLY;
			} else
				crc <<= 1;
		}
		data++;
		data_size--;
	}
	return crc;
}

/***************************************************************************//**
 * Compute XOR checksum.
 *
 * @param data      - The data buffer.
 * @param data_size - The size of the data buffer.
 *
 * @return XOR checksum.
*******************************************************************************/
uint8_t ad4110_compute_xor(uint8_t *data,
			   uint8_t data_size)
{
	uint8_t crc = 0;
	uint8_t buf[3];

	uint8_t i;

	for (i = 0; i < data_size; i++) {
		buf[i] = *data;
		crc ^= buf[i];
		data++;
	}
	return crc;
}

/***************************************************************************//**
 * SPI internal register write to device using a mask.
 *
 * @param dev      - The device structure.
 * @param reg_map  - The register map.
 *		     Accepted values: A4110_ADC
 *				      A4110_AFE
 * @param reg_addr - The register address.
 * @param data     - The register data.
 * @param mask     - The mask.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_spi_int_reg_write_msk(struct ad4110_dev *dev,
				     uint8_t reg_map,
				     uint8_t reg_addr,
				     uint32_t data,
				     uint16_t mask)
{
	int32_t ret;
	uint32_t reg_data;

	ret = ad4110_spi_int_reg_read(dev, reg_map, reg_addr, &reg_data);
	if (ret)
		return ret;
	reg_data &= ~mask;
	reg_data |= data;
	return ad4110_spi_int_reg_write(dev, reg_map, reg_addr, reg_data);
}

/***************************************************************************//**
 * Set the mode of the ADC.
 *
 * @param dev       - The device structure.
 * @param mode      - The ADC mode
 *		      Accepted values: AD4110_CONTINOUS_CONV_MODE
 *				       AD4110_SINGLE_CONV_MODE
 *				       AD4110_STANDBY_MODE
 *				       AD4110_PW_DOWN_MODE
 *				       AD4110_SYS_OFFSET_CAL
 *				       AD4110_SYS_GAIN_CAL
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_adc_mode(struct ad4110_dev *dev, enum ad4110_adc_mode mode)
{
	if(mode == AD4110_SYS_OFFSET_CAL)
		pr_info("Assuming that the applied analog input is the zero scale point. \n");
	else if(mode == AD4110_SYS_GAIN_CAL)
		pr_info("Assuming that the applied analog input is the full scale point. \n");

	return ad4110_spi_int_reg_write_msk(dev,
					    A4110_ADC,
					    AD4110_REG_ADC_MODE,
					    AD4110_ADC_MODE(mode),
					    AD4110_REG_ADC_MODE_MSK);
}

/***************************************************************************//**
 * Set the gain.
 *
 * @param dev  - The device structure.
 * @param gain - The gain value.
 * 		 Accepted values: AD4110_GAIN_0_2
 * 				  AD4110_GAIN_0_25
 * 				  AD4110_GAIN_0_3
 * 				  AD4110_GAIN_0_375
 * 				  AD4110_GAIN_0_5
 * 				  AD4110_GAIN_0_75
 * 				  AD4110_GAIN_1
 * 				  AD4110_GAIN_1_5
 * 				  AD4110_GAIN_2
 * 				  AD4110_GAIN_3
 * 				  AD4110_GAIN_4
 * 				  AD4110_GAIN_6
 * 				  AD4110_GAIN_8
 * 				  AD4110_GAIN_12
 * 				  AD4110_GAIN_16
 * 				  AD4110_GAIN_24
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_gain(struct ad4110_dev *dev, enum ad4110_gain gain)
{
	return ad4110_spi_int_reg_write_msk(dev,
					    A4110_AFE,
					    AD4110_REG_PGA_RTD_CTRL,
					    AD4110_REG_PGA_RTD_CTRL_GAIN_CH(gain),
					    AD4110_REG_PGA_RTD_CTRL_GAIN_CH(0xF));
}

/***************************************************************************//**
 * Set ADC clock.
 *
 * @param dev  - The device structure.
 * @param clk - The clock mode.
 * 		 Accepted values: AD4110_ADC_INT_CLK
 * 				  AD4110_ADC_INT_CLK_CLKIO
 * 				  AD4110_ADC_EXT_CLK
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_adc_clk(struct ad4110_dev *dev, enum ad4110_adc_clk_sel clk)
{
	return ad4110_spi_int_reg_write_msk(dev,
					    A4110_ADC,
					    AD4110_REG_ADC_MODE,
					    AD4110_REG_ADC_CLK_SEL(clk),
					    AD4110_REG_ADC_CLK_SEL(0xF));
}

/***************************************************************************//**
 * Set AFE clock.
 *
 * @param dev  - The device structure.
 * @param clk - The clock mode.
 * 		 Accepted values: AD4110_AFE_INT_CLOCK
 * 				  AD4110_AFE_ADC_CLOCKED
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_afe_clk(struct ad4110_dev *dev, enum ad4110_afe_clk_cfg clk)
{
	return ad4110_spi_int_reg_write_msk(dev,
					    A4110_AFE,
					    AD4110_REG_AFE_CLK_CTRL,
					    AD4110_REG_AFE_CLK_CTRL_CFG(clk),
					    AD4110_REG_AFE_CLK_CTRL_CFG(0xF));
}

/***************************************************************************//**
 * Set the voltage reference.
 *
 * @param dev  - The device structure.
 * @param ref - The voltage reference.
 * 		 Accepted values: AD4110_EXT_REF
 * 				  AD4110_INT_2_5V_REF
 * 				  AD4110_AVDD5_REF
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_reference(struct ad4110_dev *dev,
			     enum ad4110_voltage_reference ref)
{
	int32_t ret;

	if(ref != AD4110_INT_2_5V_REF) {
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_ADC,
						    AD4110_REG_ADC_CONFIG,
						    AD4110_REG_ADC_CONFIG_REF_SEL(ref),
						    AD4110_REG_ADC_CONFIG_REF_SEL(0xF));
	} else {
		// set REF_EN
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_ADC,
						   AD4110_REG_ADC_MODE,
						   AD4110_REG_ADC_MODE_REF_EN,
						   AD4110_REG_ADC_MODE_REF_EN);
		if (ret)
			return ret;
		// set REF_SEL to 10
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_ADC,
						    AD4110_REG_ADC_CONFIG,
						    AD4110_REG_ADC_CONFIG_REF_SEL(ref),
						    AD4110_REG_ADC_CONFIG_REF_SEL(0xF));
	}
}

/***************************************************************************//**
 * Set the operation mode.
 *
 * @param dev  - The device structure.
 * @param mode - The operation mode type.
 * 		 Accepted values: AD4110_VOLTAGE_MODE
 * 				  AD4110_CURRENT_MODE
 * 				  AD4110_CURRENT_MODE_EXT_R_SEL
 * 				  AD4110_THERMOCOUPLE
 * 				  AD4110_FLD_POWER_MODE
 * 				  AD4110_RTD_2W_MODE
 * 				  AD4110_RTD_3W_MODE
 * 				  AD4110_RTD_4W_MODE
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_set_op_mode(struct ad4110_dev *dev, enum ad4110_op_mode mode)
{
	int32_t ret;

	switch(mode) {
	case AD4110_VOLTAGE_MODE:
		// clear IMODE bit
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_DISABLE,
						    AD4110_REG_AFE_CNTRL2_IMODE_MSK);
	case AD4110_CURRENT_MODE:
		// set IMODE bit
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_ENABLE,
						    AD4110_REG_AFE_CNTRL2_IMODE_MSK);
	case AD4110_CURRENT_MODE_EXT_R_SEL:
		// set IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_ENABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// set EXT_R_SEL
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_ENABLE,
						    AD4110_REG_AFE_CNTRL2_EXT_R_SEL_MSK);
	case AD4110_THERMOCOUPLE:
		// clear IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_DISABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// enable VBIAS
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_AFE_VBIAS(AD4110_AFE_VBIAS_ON),
						    AD4110_AFE_VBIAS(AD4110_AFE_VBIAS_OFF));
	case AD4110_FLD_POWER_MODE:
		// set IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_ENABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// set EN_FLD_PWR
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_ENABLE,
						    AD4110_REG_AFE_CNTRL2_EN_FLD_PWR_MSK);

	case AD4110_RTD_4W_MODE:
		// clear IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_DISABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// clear RTD_3W4W bit
		return  ad4110_spi_int_reg_write_msk(dev,
						     A4110_AFE,
						     AD4110_REG_PGA_RTD_CTRL,
						     AD4110_DISABLE,
						     AD4110_REG_PGA_RTD_CTRL_23W_EN_MSK);
	case AD4110_RTD_3W_MODE:
		// clear IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_DISABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// set RTD_3W4W bit
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_PGA_RTD_CTRL,
						    AD4110_ENABLE,
						    AD4110_REG_PGA_RTD_CTRL_23W_EN_MSK);
	case AD4110_RTD_2W_MODE:
		// clear IMODE bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_DISABLE,
						   AD4110_REG_AFE_CNTRL2_IMODE_MSK);
		if (ret)
			return ret;

		// set RTD_3W4W bit
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_PGA_RTD_CTRL,
						   AD4110_ENABLE,
						   AD4110_REG_PGA_RTD_CTRL_23W_EN_MSK);
		if (ret)
			return ret;

		// enable VBIAS
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_AFE,
						   AD4110_REG_AFE_CNTRL2,
						   AD4110_AFE_VBIAS(AD4110_AFE_VBIAS_ON),
						   AD4110_AFE_VBIAS(AD4110_AFE_VBIAS_OFF));
		if (ret)
			return ret;

		// enable the 100 �A pull-down current source on AIN(-)
		return ad4110_spi_int_reg_write_msk(dev,
						    A4110_AFE,
						    AD4110_REG_AFE_CNTRL2,
						    AD4110_REG_AFE_CNTRL2_AINN_DN100,
						    AD4110_REG_AFE_CNTRL2_AINN_DN100);
	default:
		return FAILURE;
	}
}

/***************************************************************************//**
 * Do a SPI software reset.
 *
 * @param dev - The device structure.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_spi_do_soft_reset(struct ad4110_dev *dev)
{
	uint8_t buf [ 8 ] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	/* The AD4110 can be reset by writing a series of 64 1�s to the DIN
	 * input */
	return spi_write_and_read(dev->spi_dev, buf, 8);
}


/***************************************************************************//**
 * Get the data size of a specified register.
 *
 * @param dev      - The device structure.
 * @param reg_map  - The register map.
 *		     Accepted values: A4110_ADC
 *				      A4110_AFE
 * @param reg_addr - The register address.
 *
 * @return the data size in bytes
*******************************************************************************/
uint8_t ad4110_get_data_size(struct ad4110_dev *dev,
			     uint8_t reg_map,
			     uint8_t reg_addr)
{
	uint8_t data_size;
	data_size = 3; // default size: 1 cmd byte + 2 data bytes

	if(reg_map == A4110_ADC) {
		if(reg_addr == AD4110_REG_ADC_STATUS)
			data_size = 2;
		else if(reg_addr == AD4110_REG_DATA) {
			if(dev->data_length == AD4110_DATA_WL24)
				data_size++;

			if(dev->data_stat == AD4110_ENABLE)
				data_size++;
		} else if(reg_addr >= AD4110_ADC_OFFSET0)
			data_size = 4;
	}

	return data_size;
}

/***************************************************************************//**
 * SPI internal register write to device.
 *
 * @param dev      - The device structure.
 * @param reg_map  - The register map.
 *		     Accepted values: A4110_ADC
 *				      A4110_AFE
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_spi_int_reg_write(struct ad4110_dev *dev,
				 uint8_t reg_map,
				 uint8_t reg_addr,
				 uint32_t reg_data)
{
	uint8_t buf[5];

	uint8_t buf_size;
	uint8_t data_size = 3;

	if(reg_addr >= AD4110_ADC_OFFSET0)
		data_size = 4;

	buf[0] = (reg_map << 7) | AD4110_CMD_WR_COM_REG(reg_addr) |
		 ((dev->addr << 4) & AD4110_DEV_ADDR_MASK);
	switch(data_size) {
	case 3:
		buf[1] = (reg_data & 0xFF00) >> 8;
		buf[2] = (reg_data & 0x00FF) >> 0;
		break;

	case 4:
		buf[1] = (reg_data & 0xFF0000) >> 16;
		buf[2] = (reg_data & 0x00FF00) >> 8;
		buf[3] = (reg_data & 0x0000FF) >> 0;
		break;

	default:
		return -EINVAL;
	}

	if(((dev->afe_crc_en != AD4110_AFE_CRC_DISABLE) &&
	    (reg_map == A4110_AFE))||
	    ((dev->adc_crc_en != AD4110_ADC_CRC_DISABLE) &&
	     (reg_map == A4110_ADC))) {
		buf_size = data_size + 1;
		buf[data_size] = ad4110_compute_crc8(&buf[0], data_size);
	} else
		buf_size = data_size;
	return spi_write_and_read(dev->spi_dev, buf, buf_size);
}

/***************************************************************************//**
 * SPI internal DATA register read from device.
 *
 * @param dev      - The device structure.
 * @param reg_data - The register data.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_spi_int_data_reg_read(struct ad4110_dev *dev,
				     uint32_t *reg_data)
{
	uint8_t buf[6];

	uint8_t buf_size;
	uint8_t data_size;
	uint32_t data;
	uint8_t crc;
	int32_t ret;

	data_size = ad4110_get_data_size(dev, A4110_ADC, AD4110_REG_DATA);

	buf[0] = (A4110_ADC << 7) |
		 AD4110_CMD_READ_COM_REG(AD4110_REG_DATA) |
		 ((dev->addr << 4) & AD4110_DEV_ADDR_MASK); // cmd byte

	memset(buf + 1, 0xAA, 5); // dummy data bytes

	if(dev->adc_crc_en != AD4110_ADC_CRC_DISABLE)
		buf_size = data_size + 1; // 1 byte for crc
	else
		buf_size = data_size;

	ret = spi_write_and_read(dev->spi_dev, buf, buf_size);
	if (ret)
		return ret;

	switch (data_size) {
	case 3:
		data = (buf[1] << 8) | buf[2];
		break;
	case 4:
		data = (buf[1] << 16) | (buf[2] << 8) | buf[3];
		break;
	case 2:
		data = buf[1];
		break;
	case 5:
		// ADC Data conversion result of 24 bits + status reg append
		data = (buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4];
		break;
	default:
		return FAILURE;
	}

	if(dev->adc_crc_en == AD4110_ADC_CRC_CRC) {
		buf[0] = (A4110_ADC << 7) | AD4110_CMD_READ_COM_REG(AD4110_REG_DATA);
		crc = ad4110_compute_crc8(&buf[0], data_size);
		if (crc != buf[buf_size - 1]) {
			pr_err("%s: CRC Error.\n", __func__);
			return FAILURE;
		}
	} else if (dev->adc_crc_en == AD4110_ADC_XOR_CRC) {
		buf[0] = (A4110_ADC << 7) | AD4110_CMD_READ_COM_REG(AD4110_REG_DATA);
		crc = ad4110_compute_xor(&buf[0], data_size);
		if (crc != buf[buf_size - 1]) {
			pr_err("%s: CRC Error.\n", __func__);
			return FAILURE;
		}
	}

	*reg_data = data;

	return SUCCESS;
}

/***************************************************************************//**
 * SPI internal register read from device.
 *
 * @param dev      - The device structure.
 * @param reg_map  - The register map.
 *		     Accepted values: A4110_ADC
 *				      A4110_AFE
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_spi_int_reg_read(struct ad4110_dev *dev,
				uint8_t reg_map,
				uint8_t reg_addr,
				uint32_t *reg_data)
{
	uint8_t buf[6];

	uint8_t buf_size;
	uint8_t data_size;
	uint32_t data;
	uint8_t crc;
	int32_t ret;

	data_size = ad4110_get_data_size(dev, reg_map, reg_addr);

	buf[0] = (reg_map << 7) | AD4110_CMD_READ_COM_REG(reg_addr) |
		 ((dev->addr << 4) & AD4110_DEV_ADDR_MASK); // cmd byte

	memset(buf + 1, 0xAA, 5); // dummy data bytes

	if(((dev->afe_crc_en != AD4110_AFE_CRC_DISABLE) &&
	    (reg_map == A4110_AFE))||
	    ((dev->adc_crc_en != AD4110_ADC_CRC_DISABLE) &&
	     (reg_map == A4110_ADC)))
		buf_size = data_size + 1; // 1 byte for crc
	else
		buf_size = data_size;

	ret = spi_write_and_read(dev->spi_dev, buf, buf_size);
	if (ret)
		return ret;

	switch (data_size) {
	case 3:
		data = (buf[1] << 8) | buf[2];
		break;
	case 4:
		data = (buf[1] << 16) | (buf[2] << 8) | buf[3];
		break;
	case 2:
		data = buf[1];
		break;
	default:
		return -EINVAL;
	}

	if(((dev->afe_crc_en == AD4110_AFE_CRC) && (reg_map == A4110_AFE))||
	    ((dev->adc_crc_en == AD4110_ADC_CRC_CRC) &&
	     (reg_map == A4110_ADC))) {
		buf[0] = (reg_map << 7) | AD4110_CMD_READ_COM_REG(reg_addr);
		crc = ad4110_compute_crc8(&buf[0], data_size);
		if (crc != buf[buf_size - 1]) {
			pr_err("%s: CRC Error.\n", __func__);
			return FAILURE;
		}
	} else if ((dev->adc_crc_en == AD4110_ADC_XOR_CRC) &&
		   (reg_map == A4110_ADC)) {
		buf[0] = (reg_map << 7) | AD4110_CMD_READ_COM_REG(reg_addr);
		crc = ad4110_compute_xor(&buf[0], data_size);
		if (crc != buf[buf_size - 1]) {
			pr_err("%s: CRC Error.\n", __func__);
			return FAILURE;
		}
	}

	*reg_data = data;

	return SUCCESS;
}

/***************************************************************************//**
 * IRQ handler for ADC continuous read.
*******************************************************************************/
static void irq_adc_read(struct ad4110_callback_ctx *ctx)
{
	struct ad4110_dev *dev = ctx->dev;
	if(ctx->buffer_size > 0) {
		if(ad4110_spi_int_data_reg_read(ctx->dev, ctx->buffer))
			pr_err("DATA reg could not be read \n");
		ctx->buffer_size--;
		ctx->buffer++;
		if(irq_enable(dev->irq_desc, dev->nready_pin))
			pr_err("IRQ_enable error \n");
	} else if(irq_disable(dev->irq_desc, dev->nready_pin)) {
		pr_err("IRQ_disable error \n");
	}
}

/***************************************************************************//**
 * Initialize the device.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_setup(struct ad4110_dev **device,
		     struct ad4110_init_param init_param)
{
	struct ad4110_dev *dev;
	int32_t ret;
	dev = (struct ad4110_dev *)malloc(sizeof(*dev));
	if (!dev)
		return FAILURE;

	/* SPI */
	ret = spi_init(&dev->spi_dev, &init_param.spi_init);
	if (ret)
		goto err_dev;

	dev->afe_crc_en = AD4110_AFE_CRC_DISABLE;
	dev->adc_crc_en = AD4110_ADC_CRC_DISABLE;
	dev->addr = init_param.addr;
	dev->nready_pin = init_param.nready_pin;
	dev->irq_desc = init_param.irq_desc;
	dev->data_stat = init_param.data_stat;
	dev->data_length = init_param.data_length;
	dev->sync = init_param.sync;
	dev->op_mode = init_param.op_mode;
	dev->gain = init_param.gain;
	dev->volt_ref = init_param.volt_ref;
	dev->adc_clk = init_param.adc_clk;
	dev->afe_clk = init_param.afe_clk;

	/* Device Settings */
	ret = ad4110_spi_do_soft_reset(dev);
	if (ret)
		goto err_spi;
	mdelay(10);

	if(init_param.afe_crc_en != AD4110_AFE_CRC_DISABLE) {
		ret = ad4110_spi_int_reg_write(dev,
					       A4110_AFE,
					       AD4110_REG_AFE_CNTRL1,
					       AD4110_REG_AFE_CNTRL1_CRC_EN);
		if (ret)
			goto err_spi;
	}
	dev->afe_crc_en = init_param.afe_crc_en;

	if(init_param.adc_crc_en != AD4110_ADC_CRC_DISABLE) {
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_ADC,
						   AD4110_REG_ADC_INTERFACE,
						   AD4110_ADC_CRC_EN(init_param.adc_crc_en),
						   AD4110_REG_ADC_INTERFACE_CRC_EN_MSK);
		if (ret)
			goto err_spi;
	}
	dev->adc_crc_en = init_param.adc_crc_en;

	if(dev->data_stat == AD4110_ENABLE) {
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_ADC,
						   AD4110_REG_ADC_INTERFACE,
						   AD4110_DATA_STAT_EN,
						   AD4110_REG_ADC_INTERFACE_DS_MSK);
		if (ret)
			goto err_spi;
	}

	if(dev->data_length == AD4110_DATA_WL16) {
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_ADC,
						   AD4110_REG_ADC_INTERFACE,
						   AD4110_DATA_WL16,
						   AD4110_REG_ADC_INTERFACE_WL16_MSK);
		if (ret)
			goto err_spi;
	}

	if(dev->sync != AD4110_SYNC_EN) {
		ret = ad4110_spi_int_reg_write_msk(dev,
						   A4110_ADC,
						   AD4110_REG_ADC_GPIO_CONFIG,
						   AD4110_REG_GPIO_CONFIG_SYNC_EN(AD4110_SYNC_DIS),
						   AD4110_REG_GPIO_CONFIG_SYNC_EN(0xF));
		if (ret)
			goto err_spi;
	}


	ret = ad4110_set_op_mode(dev, dev->op_mode);
	if (ret)
		goto err_spi;

	ret = ad4110_set_gain(dev, dev->gain);
	if (ret)
		goto err_spi;

	ret = ad4110_set_reference(dev, dev->volt_ref);
	if (ret)
		goto err_spi;

	/* When AD4110_AFE_ADC_CLOCKED selected, adc_clk must be AD4110_ADC_INT_CLK_CLKIO */
	if((dev->afe_clk == AD4110_AFE_ADC_CLOCKED)
	    && (dev->adc_clk != AD4110_ADC_INT_CLK_CLKIO))
		goto err_spi;
	else {
		ret = ad4110_set_adc_clk(dev, dev->adc_clk);
		if (ret)
			goto err_spi;
	}

	ret = ad4110_set_afe_clk(dev, dev->afe_clk);
	if (ret)
		goto err_spi;

	*device = dev;

	pr_info("AD4110 successfully initialized\n");
	return SUCCESS;

err_spi:
	spi_remove(dev->spi_dev);
err_dev:
	free(dev);
	pr_err("AD4110 initialization error (%d)\n", ret);
	return ret;
}

/***************************************************************************//**
 * ADC continuous read fills buffer with buffer_size number of samples.
 *
 * @param dev          - The device structure.
 * @param buffer       - The buffer.
 * @param buffer_size  - The buffer size = number of samples.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_continuous_read(struct ad4110_dev *dev, int32_t *buffer,
			       int32_t buffer_size)
{
	int32_t ret;

	struct ad4110_callback_ctx ctx = {
		.dev = dev,
		.buffer = buffer,
		.buffer_size = buffer_size,
	};

	struct callback_desc irq_callback = {
		.callback = &irq_adc_read,
		.ctx = &ctx
	};

	ret = irq_trigger_level_set(dev->irq_desc, dev->nready_pin, IRQ_LEVEL_LOW);
	if (ret)
		return ret;
	ret = irq_register_callback(dev->irq_desc, dev->nready_pin, &irq_callback);
	if (ret)
		return ret;


	ret = ad4110_set_adc_mode(dev, AD4110_CONTINOUS_CONV_MODE);
	if (ret)
		return ret;
	// make sure adc is fully initialized before irq enabling
	mdelay(2U);
	ret = irq_enable(dev->irq_desc, dev->nready_pin);
	if (ret)
		return ret;

	while(ctx.buffer_size != 0U);

	return SUCCESS;
}

/***************************************************************************//**
 * @brief Free the resources allocated by ad4110_setup().
 *
 * @param dev - The device structure.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad4110_remove(struct ad4110_dev *dev)
{
	int32_t ret;

	ret = spi_remove(dev->spi_dev);

	free(dev);

	return ret;
}
