#!/bin/bash
# STM32F769I-Discovery Project - Source File Backup Script
# Created: 2026-02-12
# Description: Archives all important source files with paths preserved

# Configuration
PROJECT_DIR="/Users/mich/work/Antigravity/STM32F769I-Disco"
BACKUP_DIR="${PROJECT_DIR}/Backups"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
ARCHIVE_NAME="stm32f769i_sources_${TIMESTAMP}.tar.gz"
ARCHIVE_PATH="${BACKUP_DIR}/${ARCHIVE_NAME}"

# Create backup directory if it doesn't exist
mkdir -p "${BACKUP_DIR}"

echo "======================================================"
echo "STM32F769I-Discovery Source File Backup"
echo "======================================================"
echo "Project: ${PROJECT_DIR}"
echo "Archive: ${ARCHIVE_PATH}"
echo "Timestamp: ${TIMESTAMP}"
echo ""

# List of all files to backup (with paths relative to PROJECT_DIR)
FILES=(
    # ==================== CATEGORY 0: Core Application ====================
    "Core/Src/main.c"
    "Core/Src/freertos.c"
    "Core/Src/system_stm32f7xx.c"
    "Core/Src/stm32f7xx_hal_timebase_tim.c"
    "Core/Src/stm32f7xx_hal_msp.c"
    "Core/Src/stm32f7xx_it.c"
    "Core/Src/syscalls.c"
    "Core/Src/sysmem.c"
    "Core/Inc/main.h"
    "Core/Inc/FreeRTOSConfig.h"
    "Core/Inc/stm32f7xx_hal_conf.h"
    "Core/Inc/stm32f7xx_it.h"
    "Core/Inc/RTE_Components.h"

    # ==================== CATEGORY 1: LCD/Display ====================
    "Core/Src/ltdc.c"
    "Core/Inc/ltdc.h"
    "Core/Src/dsihost.c"
    "Core/Inc/dsihost.h"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.h"
    "Drivers/BSP/Components/nt35510/nt35510.c"
    "Drivers/BSP/Components/nt35510/nt35510.h"
    "Drivers/BSP/Components/otm8009a/otm8009a.c"
    "Drivers/BSP/Components/otm8009a/otm8009a.h"
    "Drivers/BSP/Components/otm8009a/otm8009a_fixed.h"
    "Core/Src/dma2d.c"
    "Core/Inc/dma2d.h"
    "Drivers/BSP/Components/Common/lcd.h"

    # ==================== CATEGORY 2: ESP8266/WiFi ====================
    "Drivers/BSP/ESP8266/esp8266.c"
    "Drivers/BSP/ESP8266/esp8266.h"

    # ==================== CATEGORY 3: Test System ====================
    "Drivers/BSP/TEST/test_system.c"
    "Drivers/BSP/TEST/test_system.h"
    "Drivers/BSP/VERSION.h"

    # ==================== CATEGORY 4: USB/CDC ====================
    "USB_DEVICE/App/usb_device.c"
    "USB_DEVICE/App/usb_device.h"
    "USB_DEVICE/App/usbd_desc.c"
    "USB_DEVICE/App/usbd_desc.h"
    "USB_DEVICE/App/usbd_cdc_if.c"
    "USB_DEVICE/App/usbd_cdc_if.h"
    "USB_DEVICE/Target/usbd_conf.c"
    "USB_DEVICE/Target/usbd_conf.h"

    # ==================== CATEGORY 5: Weather ====================
    "Drivers/BSP/WEATHER/weather.c"
    "Drivers/BSP/WEATHER/weather.h"

    # ==================== CATEGORY 6: Peripherals ====================
    "Core/Src/i2c.c"
    "Core/Inc/i2c.h"
    "Core/Src/rtc.c"
    "Core/Inc/rtc.h"
    "Core/Src/gpio.c"
    "Core/Inc/gpio.h"
    "Core/Src/crc.c"
    "Core/Inc/crc.h"
    "Core/Src/fmc.c"
    "Core/Inc/fmc.h"

    # ==================== CATEGORY 7: SDRAM/Audio/Storage ====================
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.h"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_audio.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_audio.h"
    "Drivers/BSP/Components/wm8994/wm8994.c"
    "Drivers/BSP/Components/wm8994/wm8994.h"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.h"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sd.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sd.h"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_qspi.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_qspi.h"
    "Drivers/BSP/Components/adv7533/adv7533.c"
    "Drivers/BSP/Components/adv7533/adv7533.h"
    "Drivers/BSP/Components/Common/audio.h"
    "Drivers/BSP/Components/Common/epd.h"

    # ==================== CATEGORY 8: Touchscreen ====================
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.h"
    "Drivers/BSP/Components/ft6x06/ft6x06.c"
    "Drivers/BSP/Components/ft6x06/ft6x06.h"
    "Drivers/BSP/Components/Common/ts.h"

    # ==================== CATEGORY 9: Build/Configuration ====================
    "Makefile"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c"
    "Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.h"
    "SUMMARY.md"
    "DEBUG_GUIDE.md"
    "Drivers/BSP/Components/dp83848/dp83848.c"
    "Drivers/BSP/Components/dp83848/dp83848.h"
    "Drivers/BSP/Components/lan8742/lan8742.c"
    "Drivers/BSP/Components/lan8742/lan8742.h"
    "Drivers/BSP/Components/Common/camera.h"
    "Drivers/BSP/Components/Common/accelero.h"
    "Drivers/BSP/Components/Common/magneto.h"
    "Drivers/BSP/Components/Common/gyro.h"
    "Drivers/BSP/Components/Common/idd.h"
    "Drivers/BSP/Components/Common/io.h"
    "Drivers/BSP/Components/Common/tsensor.h"
)

# Count files
TOTAL_FILES=${#FILES[@]}
echo "Files to backup: ${TOTAL_FILES}"
echo ""

# Change to project directory
cd "${PROJECT_DIR}" || exit 1

# Create archive
echo "Creating archive..."
tar -czf "${ARCHIVE_PATH}" "${FILES[@]}" 2>&1

# Check if archive was created successfully
if [ $? -eq 0 ]; then
    ARCHIVE_SIZE=$(du -h "${ARCHIVE_PATH}" | cut -f1)
    echo ""
    echo "======================================================"
    echo "Backup completed successfully!"
    echo "======================================================"
    echo "Archive: ${ARCHIVE_PATH}"
    echo "Size: ${ARCHIVE_SIZE}"
    echo "Files: ${TOTAL_FILES}"
    echo ""
    echo "To restore, use: ./restore_sources.sh ${ARCHIVE_PATH}"
    echo "======================================================"
else
    echo ""
    echo "ERROR: Backup failed!"
    exit 1
fi
