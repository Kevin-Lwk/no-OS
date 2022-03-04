ifndef MAXIM_LIBRARIES
$(error MAXIM_LIBRARIES not defined.$(ENDL))
endif

PLATFORM_RELATIVE_PATH = $1
PLATFORM_FULL_PATH = $1
CREATED_DIRECTORIES += Maxim
PROJECT_BUILD = $(BUILD_DIR)/app

ifneq "$(HEAP_SIZE)" ""
CFLAGS+=-D__HEAP_SIZE=$(HEAP_SIZE)
endif
ifneq "$(STACK_SIZE)" ""
CFLAGS+=-D_STACK_SIZE=$(STACK_SIZE)
endif

CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-gcc
GDB=arm-none-eabi-gdb
OC=arm-none-eabi-objcopy

TARGET?=max32660
TARGET_NUMBER=$(word 2,$(subst x, ,$(TARGET)))
HEX=$(basename $(BINARY)).hex
TARGET_REV=0x4131
TARGET_UC=$(addprefix MAX,$(TARGET_NUMBER))
TARGET_LC=$(addprefix max,$(TARGET_NUMBER))
TARGETCFG=$(TARGET_LC).cfg

OPENOCD_SCRIPTS=$(MAXIM_LIBRARIES)/../Tools/OpenOCD/scripts
OPENOCD_BIN=$(MAXIM_LIBRARIES)/../Tools/OpenOCD

LDFLAGS = -mcpu=cortex-m4 	\
	-Wl,--gc-sections 	\
	--specs=nosys.specs	\
	-mfloat-abi=hard 	\
	-mfpu=fpv4-sp-d16 	\
	--entry=Reset_Handler		
	
CFLAGS=-mthumb                                                                 \
        -mcpu=cortex-m4                                                         \
        -mfloat-abi=hard                                                        \
        -mfpu=fpv4-sp-d16                                                       \
        -Wa,-mimplicit-it=thumb                                                 \
        -fsingle-precision-constant                                             \
        -ffunction-sections                                                     \
        -fdata-sections                                                         \
        -MD                                                                     \
        -Wall                                                                   \
        -Wdouble-promotion                                                      \
        -Wno-format                                                      \
	-g3									\
	-c	

ASFLAGS += -x assembler-with-cpp

CFLAGS += -I$(MAXIM_LIBRARIES)/CMSIS/Include	
CFLAGS += -I$(MAXIM_LIBRARIES)/CMSIS/Device/Maxim/$(TARGET_UC)/Include	
CFLAGS += -I$(MAXIM_LIBRARIES)/PeriphDrivers/Include/$(TARGET_UC)		

CFLAGS += -DTARGET_REV=$(TARGET_REV) \
	-DTARGET=$(TARGET)		\
	-DMAXIM_PLATFORM

LSCRIPT = $(MAXIM_LIBRARIES)/CMSIS/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC).ld

$(PROJECT_TARGET):
	$(call print, Building for $(CHIPNAME))
	$(call print,Creating IDE project)
	$(MUTE) $(call mk_dir,$(BUILD_DIR)) $(HIDE)
	$(MUTE) $(call set_one_time_rule,$@)

$(PLATFORM)_sdkopen:
	$(shell python3 $(PLATFORM_TOOLS)/run_config.py $(NO-OS) $(BINARY) $(PROJECT) $(MAXIM_LIBRARIES) $(TARGET_LC))
	$(MUTE) code $(PROJECT)

$(PLATFORM)_sdkclean: clean

$(PLATFORM)_sdkbuild: build

.PHONY: $(BINARY).gdb
$(BINARY).gdb:
	@echo target remote localhost:3333 > $(BINARY).gdb	
	@echo load $(BINARY) >> $(BINARY).gdb	
	@echo file $(BINARY) >> $(BINARY).gdb
	@echo b main >> $(BINARY).gdb	
	@echo monitor reset halt >> $(BINARY).gdb	
	@echo tui enable >> $(BINARY).gdb	
	@echo c >> $(BINARY).gdb	

$(HEX): $(BINARY)
	$(MUTE) $(call print,[HEX] $(notdir $@))
	$(MUTE) $(OC) -O ihex $(BINARY) $(HEX)
	$(MUTE) $(call print,$(notdit $@) is ready)

post_build: $(HEX)

clean_hex:
	@$(call print,[Delete] $(HEX))
	-$(MUTE) $(call remove_fun,$(HEX)) $(HIDE)

clean: clean_hex

.PHONY: maxim_run
$(PLATFORM)_run: all 
	$(OPENOCD_BIN)/openocd -s $(OPENOCD_SCRIPTS) 		\
		-f interface/cmsis-dap.cfg -f target/$(TARGETCFG) \
		-c "program $(BINARY) verify reset exit"

.PHONY: debug
debug: all $(BINARY).gdb start_openocd
	$(GDB) --command=$(BINARY).gdb

.PHONY: start_openocd
ifeq ($(OS),Windows_NT)
start_openocd:
	start $(OPENOCD_BIN)/openocd -s "$(OPENOCD_SCRIPTS)" 		\
		-f interface/cmsis-dap.cfg -f target/$(TARGETCFG)
else
start_openocd:
	$(OPENOCD_BIN)/openocd -s "$(OPENOCD_SCRIPTS)" 	\
		-f interface/cmsis-dap.cfg -f target/$(TARGETCFG) -c "init" &
endif
