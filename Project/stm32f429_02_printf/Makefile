PROJECT = printf
PLATFORM_PACKAGE = Platform/STM32Cube_FW_F4/Drivers

PROJECT_PATH := $(shell dirname $(realpath $(MAKEFILE_LIST)))
ROOT_PATH := $(subst /Project,,$(shell dirname $(PROJECT_PATH)))
PLATFORM_PATH = $(ROOT_PATH)/$(PLATFORM_PACKAGE)
MIDDLEWARE_PATH = $(ROOT_PATH)/Middlewares
EXECUTABLE = $(PROJECT).elf
BIN_IMAGE = $(PROJECT).bin
HEX_IMAGE = $(PROJECT).hex

$(info ROOT Path "$(ROOT_PATH)")
$(info Project Path "$(PROJECT_PATH)")
$(info PLATFORM Path "$(PLATFORM_PATH)")


# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS = -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -O0


# Basic configurations
CFLAGS += -g -std=c99
CFLAGS += -Wall

# Optimizations
CFLAGS += -g -std=c99 -O3 -ffast-math
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000

# STM32F4xx_StdPeriph_Driver
CFLAGS += -D"assert_param(expr)=((void)0)"

# Config Linker
define get_library_path
    $(shell dirname $(shell $(CC) $(CFLAGS) -print-file-name=$(1)))
endef
LDFLAGS += -L $(call get_library_path,libc.a)
LDFLAGS += -L $(call get_library_path,libgcc.a)
LDFLAGS += -T $(PROJECT_PATH)/STM32F429ZITx_FLASH.ld


#Include file
CFLAGS += -DSTM32F429xx
CFLAGS += -I ./Inc                                                           \
          -I ../../Platform/STM32Cube_FW_F4/Drivers/STM32F4xx_HAL_Driver/Inc \
          -I ../../Platform/STM32Cube_FW_F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
          -I ../../Platform/STM32Cube_FW_F4/Drivers/BSP/STM32F429I-Discovery \
          -I ../../Platform/STM32Cube_FW_F4/Drivers/CMSIS/Include 

# STARTUP FILE
#ASSEMBLY_SOURCE = $(PROJECT_PATH)/Src/startup_stm32f429_439xx.S
ASSEMBLY_SOURCE = $(PROJECT_PATH)/Src/startup_stm32f429xx.s

#My Project
CSOURCE += $(PROJECT_PATH)/Src/main.c \
	   $(PROJECT_PATH)/Src/stm32f4xx_it.c \
	   $(PROJECT_PATH)/Src/system_stm32f4xx.c \
	   $(PROJECT_PATH)/Src/stm32f4xx_hal_msp.c
#CSOURCE +=	   $(PROJECT_PATH)/Src/syscalls.c 
	   
# Platform DIR
CSOURCE += $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
           $(PLATFORM_PATH)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c

CSOURCE += $(PLATFORM_PATH)/BSP/STM32F429I-Discovery/stm32f429i_discovery.c \


# Middleware DIR
CSOURCE += $(MIDDLEWARE_PATH)/Utilities/printf/printf.c
CFLAGS += -DPRINTF_LONG_SUPPORT
CFLAGS += -I ../../Middlewares/Utilities/printf
##### OBJECTS #####
OBJECTS = $(patsubst %.s, %.o, $(ASSEMBLY_SOURCE))
OBJECTS += $(patsubst %.c, %.o, $(CSOURCE))


all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(PROJECT).lst
	$(SIZE) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(LD) -o $@ $(OBJECTS) \
		--start-group $(LIBS) --end-group \
		$(LDFLAGS)

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

flash:
		# openocd \
		# -f interface/stlink-v2.cfg \
		# -f target/stm32f4x_stlink.cfg \
		# -c "init" \
		# -c "reset init" \
		# -c "flash probe 0" \
		# -c "flash info 0" \
		# -c "flash write_image erase $(BIN_IMAGE) 0x8000000" \
		# -c "reset run" -c shutdown || \

	st-flash write $(BIN_IMAGE) 0x8000000
#clean stage
.PHONY: clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(HEX_IMAGE)
	rm -f $(OBJECTS)
	rm -f $(PROJECT).lst
