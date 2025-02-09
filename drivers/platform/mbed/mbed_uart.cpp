/***************************************************************************//**
 * @file  mbed_uart.cpp
 * @brief Implementation of UART Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2021-22 Analog Devices, Inc.
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
#include <mbed.h>
#include <USBCDC.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus

#include "no-os/error.h"
#include "no-os/delay.h"
#include "no-os/uart.h"
#include "mbed_uart.h"

/* Max size for USB CDC packet during transmit/receive */
#define USB_CDC_MAX_PACKET_SIZE		64

/* Max allowed length of USB serial number in characters */
#define USB_SERIAL_NUM_MAX_LENGTH	100

/* Derived USBCDC class to access protected members of USBCDC class */
class platform_usbcdc :public USBCDC
{
private:
	uint8_t usb_iserial_descriptor[(USB_SERIAL_NUM_MAX_LENGTH * 2) + 2];

public :
	/* Call parent class (USBCDC) constructor explicitly */
	platform_usbcdc(bool connect_blocking, uint16_t vendor_id, uint16_t product_id,
			const char *serial_number)
		: USBCDC(connect_blocking, vendor_id, product_id)
	{
		uint8_t usb_iserial_len;	// USB serial number length
		uint8_t i, j = 0;

		usb_iserial_len = strlen(serial_number);
		if (usb_iserial_len > USB_SERIAL_NUM_MAX_LENGTH) {
			usb_iserial_len = USB_SERIAL_NUM_MAX_LENGTH;
		}

		this->usb_iserial_descriptor[j++] = (usb_iserial_len * 2) + 2;	/* bLength */
		this->usb_iserial_descriptor[j++] = STRING_DESCRIPTOR;			/* bDescriptorType */

		/* bString iSerial */
		for (i = 0; i < usb_iserial_len; i++) {
			this->usb_iserial_descriptor[j++] = serial_number[i];
			this->usb_iserial_descriptor[j++] = 0;
		}
	}

	/* Override the virtual function that sets the USB serial number
	 * The custom (user provided) USB serial number is passed through this function */
	virtual const uint8_t *string_iserial_desc()
	{
		return this->usb_iserial_descriptor;
	}

	void change_terminal_connection(bool connect_status);
	bool data_received(uint32_t rx_size);
	bool data_transmited(void);
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Change the terminal connection status (for non-terminal) based USB interface
 * @param connect_status- new connection status
 * @note  This functions is used to change the terminal connection status of USB client
 *        interface which is different than the 'console terminal'. The console terminals acknowledge
 *        back to USB host when USB connection is opened on the console terminal and Mbed USBCDC
 *        class automatically changes the '_terminal_connected' status accordingly. However, for
 *        custom PC applications (non terminal), the terminal connection status needs to be changed
 *        manually. The '_terminal_connected' is protected member of USBCDC parent class and thus can
 *        be accessed through 'platform_usbcdc' derived class using below function.
 */
void platform_usbcdc::change_terminal_connection(bool connect_status)
{
	_terminal_connected = connect_status;
}

/**
 * @brief	Check if new USB data is received/available
 * @param	bytes[in] - Number of expected bytes to be received
 * @return	true if expected number of bytes received, else false
 */
bool platform_usbcdc::data_received(uint32_t bytes)
{
	volatile static uint32_t *rx_size = &_rx_size;

	if (*rx_size >= bytes)
		return true;
	else
		return false;
}

/**
 * @brief  Check if old USB data was transmitted
 * @return true if transmit not in progress, else false
 */
bool platform_usbcdc::data_transmited(void)
{
	volatile static bool *tx_in_progress = &_tx_in_progress;
	return !(*tx_in_progress);
}

/**
 * @brief Read data from UART device.
 * @param desc[in, out] - Instance of UART.
 * @param data[out] - Pointer to buffer containing data.
 * @param bytes_number[in] - Number of bytes to read.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t uart_read(struct uart_desc *desc, uint8_t *data,
		  uint32_t bytes_number)
{
	mbed::BufferedSerial *uart;	// pointer to BufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	uint32_t size_rd;

	if (!desc || !desc->extra || !data)
		return -EINVAL;

	if (!((struct mbed_uart_desc *)(desc->extra))->uart_port)
		return -EINVAL;

	if (((struct mbed_uart_desc *)desc->extra)->virtual_com_enable) {
		usb_cdc_dev = (platform_usbcdc *)((struct mbed_uart_desc *)(
				desc->extra))->uart_port;

		while (!usb_cdc_dev->data_received(bytes_number)) {
			/* Wait until new data is available */
		}

		/* Change terminal connection status manually */
		usb_cdc_dev->change_terminal_connection(true);

		usb_cdc_dev->receive_nb(data, bytes_number, &size_rd);
		return bytes_number;
	} else {
		uart = (BufferedSerial *)(((struct mbed_uart_desc *)(
						   desc->extra))->uart_port);
		return uart->read(data, bytes_number);
	}
}

/**
 * @brief Write data to UART device.
 * @param desc[in] - Instance of UART.
 * @param data[in, out] - Pointer to buffer containing data.
 * @param bytes_number[in] - Number of bytes to read.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t uart_write(struct uart_desc *desc, const uint8_t *data,
		   uint32_t bytes_number)
{
	mbed::BufferedSerial *uart;	// pointer to BufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	uint32_t d_sz;
	uint32_t indx = 0;
	uint32_t tx_bytes = bytes_number;

	if (!desc || !desc->extra || !data)
		return -EINVAL;

	if (!((struct mbed_uart_desc *)(desc->extra))->uart_port)
		return -EINVAL;

	if (((struct mbed_uart_desc *)desc->extra)->virtual_com_enable) {
		usb_cdc_dev = (platform_usbcdc *)((struct mbed_uart_desc *)(
				desc->extra))->uart_port;

		while (bytes_number) {
			while (!usb_cdc_dev->data_transmited()) {
				/* Wait until old data is transmitted */
			}

			/* Make sure packet size is less than max CDC packet size during data transmit */
			d_sz = (bytes_number > (USB_CDC_MAX_PACKET_SIZE - 1)) ?
			       (USB_CDC_MAX_PACKET_SIZE - 1) :
			       bytes_number;

			/* Change terminal connection status manually */
			usb_cdc_dev->change_terminal_connection(true);

			usb_cdc_dev->send_nb((uint8_t *)&data[indx], d_sz, &d_sz);

			bytes_number -= d_sz;
			indx += d_sz;
		}

		return tx_bytes;
	} else {
		uart = (BufferedSerial *)(((struct mbed_uart_desc *)(
						   desc->extra))->uart_port);
		return uart->write(data, bytes_number);
	}
}

/**
 * @brief Submit reading buffer to the UART driver.
 * Buffer is used until bytes_number bytes are read.
 * @param desc[in]- Descriptor of the UART device
 * @param data[out] - Buffer where data will be read
 * @param bytes_number[in] - Number of bytes to be read.
 * @return SUCCESS in case of success, negative error code otherwise.
 * @note Currently implemented only for UART and not for USBCDC (VCOM)
 */
int32_t uart_read_nonblocking(struct uart_desc *desc,
			      uint8_t *data,
			      uint32_t bytes_number)
{
	mbed::BufferedSerial *uart;	// pointer to BufferedSerial/UART instance

	if (!desc || !desc->extra || !data)
		return -EINVAL;

	if (!((struct mbed_uart_desc *)(desc->extra))->uart_port)
		return -EINVAL;

	uart = (BufferedSerial *)(((struct mbed_uart_desc *)(
					   desc->extra))->uart_port);

	if (uart->readable())
		return uart->read(data, bytes_number);
	else
		return 0;
}

/**
 * @brief Submit writting buffer to the UART driver.
 * Data from the buffer is sent over the UART, the function returns immediatly.
 * @param desc[in] - Descriptor of the UART device
 * @param data[in,out] - Buffer where data will be written
 * @param bytes_number[in] - Number of bytes to be written.
 * @return SUCCESS in case of success, negative error code otherwise.
 * @note Currently implemented only for UART and not for USBCDC (VCOM)
 */
int32_t uart_write_nonblocking(struct uart_desc *desc,
			       const uint8_t *data,
			       uint32_t bytes_number)
{
	mbed::BufferedSerial *uart;		// pointer to BufferedSerial/UART instance

	if (!desc || !desc->extra || !data)
		return - EINVAL;

	if (!((struct mbed_uart_desc *)(desc->extra))->uart_port)
		return -EINVAL;

	uart = (BufferedSerial *)(((struct mbed_uart_desc *)(
					   desc->extra))->uart_port);

	if (uart->writable())
		return uart->write(data, bytes_number);
	else
		return 0;
}

/**
 * @brief Set the UART communication data formats
 * @param param[in] - The structure that contains the UART parameters.
 * @param uart[in] - Mbed UART instance
 * @return SUCCESS in case of success, negative error code otherwise.
 */
static int32_t mbed_uart_set_format(struct uart_init_param *param,
				    mbed::BufferedSerial *uart)
{
	mbed::BufferedSerial::Parity Parity;
	int data_bits;
	int stop_bits;

	/* Get the data bits */
	switch (param->size) {
	case UART_CS_7:
		data_bits = 7;
		break;

	case UART_CS_8:
		data_bits = 8;
		break;

	case UART_CS_9:
		data_bits = 9;
		break;

	default:
		/* Invalid data bits. Mbed supports only 7,8 and 9 data bits */
		return -EINVAL;
	}

	/* Get the parity */
	switch (param->parity) {
	case UART_PAR_NO:
		Parity = SerialBase::None;
		break;

	case UART_PAR_EVEN:
		Parity = SerialBase::Even;
		break;

	case UART_PAR_ODD:
		Parity = SerialBase::Odd;
		break;

	default:
		/* Invalid parity. Mbed supports only none, odd and even parity */
		return -EINVAL;
	}

	/* Get the stop bits */
	switch (param->stop) {
	case UART_STOP_1_BIT:
		stop_bits = 1;
		break;

	case UART_STOP_2_BIT:
		stop_bits = 2;
		break;

	default:
		/* Invalid stop bits */
		return -EINVAL;
	}

	uart->set_format(data_bits, Parity, stop_bits);
	return SUCCESS;
}

/**
 * @brief Initialize the UART communication peripheral.
 * @param desc[in, out] - The UART descriptor.
 * @param param[in] - The structure that contains the UART parameters.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t uart_init(struct uart_desc **desc, struct uart_init_param *param)
{
	mbed::BufferedSerial *uart;	// Pointer to new BufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	struct mbed_uart_desc *mbed_uart_desc;	// Pointer to mbed uart descriptor
	struct uart_desc *uart_desc; 			// UART descriptor

	if (!desc || !param)
		return -EINVAL;

	// Create the UART description object for the device
	uart_desc = (struct uart_desc *)calloc(1, sizeof(*uart_desc));
	if (!uart_desc)
		return -ENOMEM;

	uart_desc->baud_rate = param->baud_rate;

	// Create a new mbed descriptor to store new UART instances
	mbed_uart_desc = (struct mbed_uart_desc *)calloc(1, sizeof(*mbed_uart_desc));
	if (!mbed_uart_desc)
		goto err_mbed_uart_desc;

	if (((struct mbed_uart_init_param *)param->extra)->virtual_com_enable) {
		// Create a new instance of platform_usbcdc class
		usb_cdc_dev = new platform_usbcdc(false,
						  ((struct mbed_uart_init_param *)param->extra)->vendor_id,
						  ((struct mbed_uart_init_param *)param->extra)->product_id,
						  ((struct mbed_uart_init_param *)param->extra)->serial_number);
		if (!usb_cdc_dev)
			goto err_serial_port;

		mbed_uart_desc->uart_port = (platform_usbcdc *)usb_cdc_dev;

		/* Establish connection with the USB CDC communication port */
		usb_cdc_dev->connect();
		mdelay(2000);
	} else {
		// Create and configure a new instance of BufferedSerial/UART port
		uart = new BufferedSerial(
			(PinName)(((struct mbed_uart_init_param *)param->extra)->uart_tx_pin),
			(PinName)(((struct mbed_uart_init_param *)param->extra)->uart_rx_pin),
			(int)param->baud_rate);

		if (!uart)
			goto err_serial_port;

		/* Set the UART format */
		if (mbed_uart_set_format(param, uart) != SUCCESS)
			goto err_uart_set_format;

		mbed_uart_desc->uart_port = (BufferedSerial *)uart;
	}

	mbed_uart_desc->virtual_com_enable = ((struct mbed_uart_init_param *)
					      param->extra)->virtual_com_enable;
	uart_desc->extra = mbed_uart_desc;
	*desc = uart_desc;

	return SUCCESS;

err_uart_set_format:
	delete(uart);
err_serial_port:
	free(mbed_uart_desc);
err_mbed_uart_desc:
	free(uart_desc);

	return -ENOMEM;
}

/**
 * @brief Free the resources allocated by uart_init().
 * @param desc[in] - The UART descriptor.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t uart_remove(struct uart_desc *desc)
{
	if (!desc || !desc->extra)
		return -EINVAL;

	/* Free the UART object */
	if (((struct mbed_uart_desc *)desc->extra)->virtual_com_enable) {
		if ((platform_usbcdc *)((struct mbed_uart_desc *)desc->extra)->uart_port)
			delete((platform_usbcdc *)((struct mbed_uart_desc *)
						   desc->extra)->uart_port);
	} else {
		if ((BufferedSerial *)(((struct mbed_uart_desc *)(desc->extra))->uart_port))
			delete((BufferedSerial *)(((struct mbed_uart_desc *)(
							   desc->extra))->uart_port));
	}

	/* Free the UART descriptor objects */
	free(desc->extra);
	free(desc);

	return SUCCESS;
}

#ifdef __cplusplus  // Closing extern c
}
#endif //  _cplusplus
