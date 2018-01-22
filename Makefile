##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [2.24.1] date: [Thu Aug 17 11:19:16 CEST 2017]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = Internal_Robot_CTRL


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# source path
SOURCES_DIR =  \
Middlewares \
Drivers/STM32F3xx_HAL_Driver \
Middlewares/FreeRTOS \
Application \
Application/User \
Drivers \
Drivers/CMSIS \
Application/MAKEFILE 

# firmware library path
PERIFLIB_PATH = 

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c \
Src/stm32f3xx_hal_msp.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c \
Src/stm32f3xx_it.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c \
Src/stm32f3xx_hal_timebase_TIM.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_can.c \
Src/system_stm32f3xx.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c \
Src/main.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Src/freertos.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c \
Src/Trace.c \
Src/ultrasonic.c \
Src/gpio.c \
Src/usart.c \
Src/internal_sensors.c \
Src/tim.c	\
Src/ControllerAreaNetwork.c \
Src/SensorDataHandler.c
# Src/can_stm.c	\




# ASM sources
ASM_SOURCES =  \
startup_stm32f303xc.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
BINPATH = 
PREFIX = /opt/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-
CC = $(BINPATH)/$(PREFIX)gcc
AS = $(BINPATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)/$(PREFIX)objcopy
AR = $(BINPATH)/$(PREFIX)ar
SZ = $(BINPATH)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F303xC


# AS includes
AS_INCLUDES =  \
-IInc

#IDIR = /usr/include

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F3xx_HAL_Driver/Inc \
-IDrivers/STM32F3xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IDrivers/CMSIS/Device/ST/STM32F3xx/Include \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IDrivers/CMSIS/Include 
#-I/usr/include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F303VCTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***