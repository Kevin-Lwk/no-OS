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

# Uncomment to select the profile

SRCS += $(PROJECT)/src/main.c \
	$(PROJECT)/src/cf_hdmi.c \
	$(PROJECT)/src/edid.c \
	$(PROJECT)/src/transmitter.c \
	$(PROJECT)/src/wrapper.c
SRCS += $(DRIVERS)/axi_core/axi_dmac/axi_dmac.c \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.c \
	$(DRIVERS)/api/i2c.c \
	$(DRIVERS)/api/gpio.c \
	$(DRIVERS)/api/spi.c \
	$(NO-OS)/util/util.c \
	$(NO-OS)/util/list.c
SRCS +=	$(PLATFORM_DRIVERS)/axi_io.c \
	$(PLATFORM_DRIVERS)/xilinx_spi.c \
	$(PLATFORM_DRIVERS)/xilinx_gpio.c \
	$(PLATFORM_DRIVERS)/delay.c \
	$(PLATFORM_DRIVERS)/xilinx_i2c.c \
	$(PLATFORM_DRIVERS)/irq.c \
	$(PLATFORM_DRIVERS)/timer.c
SRCS +=$(PROJECT)/TX/HAL/COMMON/tx_hal.c \
	$(PROJECT)/TX/HAL/WIRED/wrd_hal.c \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/7511_hal.c \
	$(PROJECT)/TX/LIB/tx_cec.c \
	$(PROJECT)/TX/LIB/tx_isr.c \
	$(PROJECT)/TX/LIB/tx_lib.c \
	$(PROJECT)/TX/LIB/tx_multi.c
INCS +=	$(PROJECT)/src/app_config.h \
	$(PROJECT)/src/cf_hdmi.h \
	$(PROJECT)/src/cf_hdmi_demo.h \
	$(PROJECT)/src/edid.h \
	$(PROJECT)/src/transmitter.h \
	$(PROJECT)/src/transmitter_defs.h \
	$(PROJECT)/src/wrapper.h
INCS += $(DRIVERS)/axi_core/axi_dmac/axi_dmac.h \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.h
INCS +=	$(PLATFORM_DRIVERS)/spi_extra.h \
	$(PLATFORM_DRIVERS)/gpio_extra.h \
	$(PLATFORM_DRIVERS)/i2c_extra.h \
	$(PLATFORM_DRIVERS)/irq_extra.h \
	$(PLATFORM_DRIVERS)/timer_extra.h
INCS +=	$(INCLUDE)/no-os/axi_io.h \
	$(INCLUDE)/no-os/spi.h \
	$(INCLUDE)/no-os/gpio.h \
	$(INCLUDE)/no-os/error.h \
	$(INCLUDE)/no-os/delay.h \
	$(INCLUDE)/no-os/util.h \
	$(INCLUDE)/no-os/list.h \
	$(INCLUDE)/no-os/i2c.h \
	$(INCLUDE)/no-os/irq.h \
	$(INCLUDE)/no-os/timer.h
INCS +=	$(PROJECT)/TX/tx_lib.h \
	$(PROJECT)/TX/HAL/COMMON/tx_cfg.h \
	$(PROJECT)/TX/HAL/COMMON/tx_hal.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/7511_cfg.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/7511_hal.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_cec_map_adr.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_cec_map_def.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_cec_map_fct.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_cfg.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_edid_map_adr.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_edid_map_def.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_edid_map_fct.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_lib.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_main_map_adr.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_main_map_def.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_main_map_fct.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_packet_map_adr.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_packet_map_def.h \
	$(PROJECT)/TX/HAL/WIRED/ADV7511/MACROS/ADV7511_packet_map_fct.h \
	$(PROJECT)/TX/LIB/tx_isr.h \
	$(PROJECT)/TX/LIB/tx_multi.h
