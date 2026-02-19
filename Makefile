################################################################################
# Makefile for STM32F769I-Disco
################################################################################

TARGET = TestF
DEBUG = 1
OPT = -Og
DEFS = -DSTM32F769xx -DUSE_HAL_DRIVER

################################################################################
# Paths
################################################################################
BUILD_DIR = build

# Source paths - Top Level Only to avoid duplicates with 'find'
SOURCES_DIR = \
Core \
Drivers \
Middlewares \
USB_DEVICE 

# Includes
INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
-IDrivers/CMSIS/RTOS_V2 \
-IDrivers/CMSIS/Include \
-IDrivers/BSP/STM32F769I-Discovery \
-IDrivers/BSP/Components/Common \
-IDrivers/BSP/Components/otm8009a \
-IDrivers/BSP/Components/nt35510 \
-IDrivers/BSP/Components/ft6x06 \
-IDrivers/BSP/ESP8266 \
-IDrivers/BSP/WEATHER \
-IDrivers/BSP/TEST \
-IDrivers/BSP \
-IUtilities/Fonts \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IMiddlewares/Third_Party/JPEG \
-IMiddlewares/Third_Party/JPEG/Inc \
-ITouchGFX/App \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-IUtilities/Fonts

#CPP_SOURCES += \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/hal/HAL.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/Drawable.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/Image.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/Container.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/ZoomAnimationImage.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/ListLayout.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/ModalWindow.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/SwipeContainer.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/DrawableList.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/ScrollList.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/ScrollWheel.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/ScrollWheelBase.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/ScrollWheelWithSelectionStyle.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/scrollers/ScrollBase.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/ScrollableContainer.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/SlideMenu.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/CacheableContainer.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/Container.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/Slider.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/AbstractProgressIndicator.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/AbstractDirectionProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/LineProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/ImageProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/CircleProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/BoxProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/progress_indicators/TextProgress.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/clock/DigitalClock.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/clock/AbstractClock.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/containers/clock/AnalogClock.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/AnimationTextureMapper.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/TouchArea.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/TextArea.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/Button.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/Image.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/TiledImage.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/Box.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/ButtonWithIcon.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/ToggleButton.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/RadioButton.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterARGB8888.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/CanvasWidget.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterABGR2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/Line.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainter.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB565L8Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterARGB2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterRGB565.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterBGRA2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterBWBitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/Canvas.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterRGBA2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterARGB8888Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterARGB2222Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterABGR2222Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterGRAY4.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB888L8Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterBGRA2222Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterGRAY2.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterARGB8888L8Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB565.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGBA2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterBGRA2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractShape.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterARGB2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterBW.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterABGR2222.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB888Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/Circle.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterRGB888.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterARGB8888.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB565Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterGRAY2.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterGRAY4Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/AbstractPainterBW.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterGRAY4.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGB888.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterGRAY2Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/canvas/PainterRGBA2222Bitmap.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/TextAreaWithWildcard.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/ScalableImage.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/ButtonWithLabel.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/PixelDataWidget.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/SnapshotWidget.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/BoxWithBorder.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/AbstractButton.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/RepeatButton.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/AnimatedImage.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/TextureMapper.cpp \
#Middlewares/ST/TouchGFX/framework/source/touchgfx/widgets/Keyboard.cpp \
#Middlewares/ST/TouchGFX/framework/source/platform/hal/simulator/sdl2/HALSDL2.cpp \
#Middlewares/ST/TouchGFX/framework/source/platform/hal/simulator/sdl2/OSWrappers.cpp \
#Middlewares/ST/TouchGFX/framework/source/platform/hal/simulator/sdl2/HALSDL2_icon.cpp \
#Middlewares/ST/TouchGFX/framework/source/platform/driver/touch/SDL2TouchController.cpp \
#Middlewares/ST/TouchGFX/framework/source/platform/driver/touch/ST1232TouchController.cpp

# TouchGFX disabled for minimal build
# CPP_SOURCES += \
# TouchGFX/target/generated/OSWrappers.cpp \
# TouchGFX/target/generated/STM32DMA.cpp \
# TouchGFX/target/generated/TouchGFXConfiguration.cpp \
# TouchGFX/target/generated/TouchGFXGeneratedHAL.cpp \
# TouchGFX/target/STM32TouchController.cpp \
# TouchGFX/target/TouchGFXGPIO.cpp \
# TouchGFX/target/TouchGFXHAL.cpp

#CPP_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
CPP_OBJECTS =

CPP_INCLUDES += \
-IMiddlewares/ST/TouchGFX/framework/include \
-IMiddlewares/ST/TouchGFX/framework/source \

# Sources
C_SOURCES = $(shell find $(SOURCES_DIR) -name '*.c' ! -name '*template.c' ! -name '*audio.c' ! -name '*sd.c' ! -name '*qspi.c' ! -name '*system_stm32f7xx.c' ! -name 'app_touchgfx.c' ! -path '*Adafruit*' ! -path '*Drivers/CMSIS*' ! -path '*Utilities/Fonts*' ! -name 'ltdc.c')
C_SOURCES += Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
C_SOURCES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sd.c
C_SOURCES += Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sd.c

ASM_SOURCES = Core/Startup/startup_stm32f769nihx.s

################################################################################
# Toolchain
################################################################################
PREFIX = arm-none-eabi-
CXX = arm-none-eabi-g++
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
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
-DC_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F769xx \
-DUSE_STM32F769I_DISCO_REVB03

AS_INCLUDES = 

C_FLAGS = $(MCU) $(C_DEFS) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
C_FLAGS += $(DEFS)
CFLAGS += $(DEFS)
CXXFLAGS += $(DEFS)

# TouchGFX exclusion for now (to avoid link errors if lib is missing)
# C_DEFS += -DTOUCHGFX_SUPPORT

LDSCRIPT = STM32F769XX_FLASH.ld
LIBS = -lc -lm -lnosys
LIBDIR =
# CRITICAL FIX v0.1.62: Add -u _printf_float for float formatting with newlib-nano
# Required for snprintf(%.1f) to work in ESP8266_GetWeather()
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

################################################################################
# Build
################################################################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin Binary/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# $(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
# 	$(CXX) -c $(C_FLAGS) $(CPP_INCLUDES) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(C_FLAGS) $(CPP_INCLUDES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(C_FLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(C_FLAGS) $(CPP_INCLUDES) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(CPP_OBJECTS) Makefile
	$(CC) $(OBJECTS) $(CPP_OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# $(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(CPP_OBJECTS) Makefile
#	$(CC) $(OBJECTS) $(CPP_OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

Binary/%.bin: $(BUILD_DIR)/%.bin | Binary
	cp $< $@

$(BUILD_DIR):
	mkdir $@

Binary:
	mkdir $@		

clean:
	-rm -fR $(BUILD_DIR)
  
flash:
	openocd -f board/stm32f7discovery.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

.PHONY: all clean flash TestF.hex
