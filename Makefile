################################################################################
# Makefile for STM32F769I-Disco
################################################################################

TARGET = TestF
DEBUG = 1
OPT = -Og

################################################################################
# Paths
################################################################################
BUILD_DIR = build

# Source paths - Top Level Only to avoid duplicates with 'find'
SOURCES_DIR = \
Core \
Drivers \
Middlewares \
USB_DEVICE \
Utilities

# Includes
INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/BSP/STM32F769I-Discovery \
-IDrivers/BSP/Components/Common \
-IDrivers/BSP/Components/otm8009a \
-IDrivers/BSP/Components/nt35510 \
-IDrivers/BSP/Components/ft6x06 \
-IDrivers/BSP/ESP8266 \
-IUtilities/Fonts \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-ITouchGFX/App \
-ITouchGFX/target/generated \
-ITouchGFX/generated/gui_generated/include \
-ITouchGFX/gui/include

# Sources
C_SOURCES = $(shell find $(SOURCES_DIR) -name '*.c' ! -name '*template.c' ! -name '*audio.c' ! -name '*sd.c' ! -name '*qspi.c' ! -name '*_lcd.c' ! -name '*_ts.c' ! -path '*Adafruit*' ! -path '*Drivers/CMSIS*' ! -path '*Utilities/Fonts*')
ASM_SOURCES = Core/Startup/startup_stm32f769nihx.s

################################################################################
# Toolchain
################################################################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(PREFIX)objcopy -O ihex
BIN = $(PREFIX)objcopy -O binary -S

################################################################################
# Flags
################################################################################
CPU = -mcpu=cortex-m7
FPU = -mfpu=fpv5-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS = 
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F769xx \
-DUSE_STM32F769I_DISCO

AS_INCLUDES = 

C_FLAGS = $(MCU) $(C_DEFS) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# TouchGFX exclusion for now (to avoid link errors if lib is missing)
# C_DEFS += -DTOUCHGFX_SUPPORT

LDSCRIPT = STM32F769NIHX_FLASH.ld
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

################################################################################
# Build
################################################################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(C_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR $(BUILD_DIR)
  
flash:
	openocd -f board/stm32f7discovery.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

.PHONY: all clean flash
