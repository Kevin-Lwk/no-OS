################################################################################
#									       #
#     Shared variables:							       #
#	- PROJECT							       #
#	- DRIVERS							       #
#	- INCLUDE							       #
#	- PLATFORM_DRIVERS						       #
#	- NO-OS								       #
#									       #
################################################################################

SRCS += $(PROJECT)/src/main.c
ifeq (y,$(strip $(TINYIIOD)))
LIBRARIES += iio
SRC_DIRS += $(NO-OS)/iio/iio_app
endif
SRCS += $(DRIVERS)/api/spi.c \
	$(DRIVERS)/api/gpio.c \
	$(DRIVERS)/frequency/hmc7044/hmc7044.c \
	$(DRIVERS)/dac/ad917x/ad9172.c \
	$(DRIVERS)/dac/ad917x/ad917x_api/ad917x_api.c \
	$(DRIVERS)/dac/ad917x/ad917x_api/ad917x_jesd_api.c \
	$(DRIVERS)/dac/ad917x/ad917x_api/ad917x_nco_api.c \
	$(DRIVERS)/dac/ad917x/ad917x_api/ad917x_reg.c
SRCS += $(DRIVERS)/axi_core/axi_dac_core/axi_dac_core.c \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.c \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.c \
	$(DRIVERS)/axi_core/jesd204/axi_adxcvr.c \
	$(DRIVERS)/axi_core/jesd204/axi_jesd204_rx.c \
	$(DRIVERS)/axi_core/jesd204/axi_jesd204_tx.c \
	$(DRIVERS)/axi_core/jesd204/xilinx_transceiver.c \
	$(NO-OS)/util/util.c
SRCS +=	$(PLATFORM_DRIVERS)/axi_io.c \
	$(PLATFORM_DRIVERS)/xilinx_spi.c \
	$(PLATFORM_DRIVERS)/xilinx_gpio.c \
	$(PLATFORM_DRIVERS)/delay.c
ifeq (y,$(strip $(TINYIIOD)))
SRCS += $(NO-OS)/util/fifo.c \
	$(NO-OS)/util/list.c \
	$(DRIVERS)/axi_core/iio_axi_dac/iio_axi_dac.c \
	$(PLATFORM_DRIVERS)/uart.c \
	$(PLATFORM_DRIVERS)/xilinx_irq.c \
	$(DRIVERS)/api/irq.c
endif
INCS += $(PROJECT)/src/parameters.h \
	$(PROJECT)/src/app_config.h
INCS += $(DRIVERS)/frequency/hmc7044/hmc7044.h \
	$(DRIVERS)/dac/ad917x/ad9172.h \
	$(DRIVERS)/dac/ad917x/ad917x_api/ad917x_reg.h \
	$(DRIVERS)/dac/ad917x/ad917x_api/AD917x.h \
	$(DRIVERS)/dac/ad917x/ad917x_api/api_def.h \
	$(DRIVERS)/dac/ad917x/ad917x_api/api_errors.h		
INCS += $(DRIVERS)/axi_core/axi_dac_core/axi_dac_core.h \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.h \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.h \
	$(DRIVERS)/axi_core/jesd204/axi_adxcvr.h \
	$(DRIVERS)/axi_core/jesd204/axi_jesd204_rx.h \
	$(DRIVERS)/axi_core/jesd204/axi_jesd204_tx.h \
	$(DRIVERS)/axi_core/jesd204/xilinx_transceiver.h
INCS +=	$(PLATFORM_DRIVERS)/spi_extra.h \
	$(PLATFORM_DRIVERS)/gpio_extra.h
INCS +=	$(INCLUDE)/no-os/axi_io.h \
	$(INCLUDE)/no-os/spi.h \
	$(INCLUDE)/no-os/gpio.h \
	$(INCLUDE)/no-os/error.h \
	$(INCLUDE)/no-os/delay.h \
	$(INCLUDE)/no-os/util.h
ifeq (y,$(strip $(TINYIIOD)))
INCS += $(INCLUDE)/no-os/fifo.h \
	$(INCLUDE)/no-os/irq.h \
	$(INCLUDE)/no-os/uart.h \
	$(INCLUDE)/no-os/list.h \
	$(PLATFORM_DRIVERS)/irq_extra.h \
	$(PLATFORM_DRIVERS)/uart_extra.h \
	$(DRIVERS)/axi_core/iio_axi_dac/iio_axi_dac.h
endif
