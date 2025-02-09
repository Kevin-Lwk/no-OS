# See No-OS/tool/scripts/src_model.mk for variable description
SRC_DIRS += $(PROJECT)/src/app
SRC_DIRS += $(NO-OS)/iio/iio_app

SRCS +=	$(NO-OS)/util/list.c \
	$(NO-OS)/util/fifo.c \
	$(NO-OS)/util/util.c

#drivers
SRCS += $(DRIVERS)/adc/adc_demo/adc_demo.c \
	$(DRIVERS)/adc/adc_demo/iio_adc_demo.c \
	$(DRIVERS)/dac/dac_demo/iio_dac_demo.c \
	$(DRIVERS)/dac/dac_demo/dac_demo.c

INCS += $(INCLUDE)/no-os/fifo.h \
	$(INCLUDE)/no-os/uart.h \
	$(INCLUDE)/no-os/list.h \
	$(INCLUDE)/no-os/util.h \
	$(INCLUDE)/no-os/error.h

INCS += $(DRIVERS)/adc/adc_demo/iio_adc_demo.h \
		$(DRIVERS)/dac/dac_demo/dac_demo.h \
		$(DRIVERS)/dac/dac_demo/iio_dac_demo.h \
		$(DRIVERS)/adc/adc_demo/adc_demo.h

ifeq ($(PLATFORM),$(filter $(PLATFORM),xilinx aducm3029))
SRCS += $(PLATFORM_DRIVERS)/delay.c \
	$(DRIVERS)/api/irq.c
endif
INCS += $(INCLUDE)/no-os/delay.h

ifeq ($(PLATFORM),$(filter $(PLATFORM),xilinx aducm3029))
# For the moment there is support only for aducm for iio with network backend
ifeq (aducm3029,$(strip $(PLATFORM)))
ifeq '$(USE_TCP_SOCKET)' 'y'
CFLAGS += -DUSE_TCP_SOCKET
endif
ENABLE_IIO_NETWORK = y
endif

ifeq (y,$(strip $(ENABLE_IIO_NETWORK)))
DISABLE_SECURE_SOCKET ?= y
SRC_DIRS += $(NO-OS)/network
SRCS	 += $(NO-OS)/util/circular_buffer.c
SRCS	 += $(PLATFORM_DRIVERS)/timer.c
INCS	 += $(INCLUDE)/no-os/timer.h \
		$(INCLUDE)/no-os/circular_buffer.h \
		$(PLATFORM_DRIVERS)/timer_extra.h \
		$(PLATFORM_DRIVERS)/rtc_extra.h
endif

SRCS += $(PLATFORM_DRIVERS)/uart.c \
		$(PLATFORM_DRIVERS)/$(PLATFORM)_irq.c

INCS += $(INCLUDE)/no-os/irq.h \
	$(INCLUDE)/no-os/rtc.h \
	$(INCLUDE)/no-os/gpio.h \
	$(PLATFORM_DRIVERS)/irq_extra.h \
	$(PLATFORM_DRIVERS)/uart_extra.h
endif

# stm32
ifeq (stm32, $(PLATFORM))
SRCS += $(PLATFORM_DRIVERS)/stm32_delay.c \
	$(PLATFORM_DRIVERS)/stm32_uart.c \
	$(PLATFORM_DRIVERS)/stm32_uart_stdio.c
INCS += $(PLATFORM_DRIVERS)/stm32_uart_stdio.h \
	$(PLATFORM_DRIVERS)/stm32_uart.h \
	$(PLATFORM_DRIVERS)/stm32_hal.h
endif

ifeq (linux,$(PLATFORM))
CFLAGS += -DENABLE_IIO_NETWORK \
			-DDISABLE_SECURE_SOCKET

LIBRARIES += iio
SRCS += $(NO-OS)/network/linux_socket/linux_socket.c 
SRCS +=	$(NO-OS)/network/tcp_socket.c
SRCS += $(PROJECT)/src/app/main.c
SRCS += $(NO-OS)/iio/iio_app/iio_app.c
SRCS += $(NO-OS)/util/circular_buffer.c

SRCS += $(DRIVERS)/platform/generic/uart.c \
		$(DRIVERS)/platform/generic/delay.c

INCS += $(NO-OS)/network/tcp_socket.h \
		$(NO-OS)/network/network_interface.h \
		$(NO-OS)/network/noos_mbedtls_config.h \
		$(NO-OS)/network/linux_socket/linux_socket.h

INCS	 += $(INCLUDE)/no-os/circular_buffer.h
INCS += $(PROJECT)/src/app/app_config.h \
		$(PROJECT)/src/app/parameters.h	

INCS += $(NO-OS)/iio/iio_app/iio_app.h 

INCS += $(INCLUDE)/no-os/gpio.h \
		$(INCLUDE)/no-os/delay.h \
		$(INCLUDE)/no-os/irq.h \
		$(INCLUDE)/no-os/trng.h

endif
