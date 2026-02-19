#!/bin/bash
# Build and flash diagnostic firmware for STM32F769I-Discovery
# This script builds the firmware with early USB CDC initialization

set -e  # Exit on error

PROJECT_DIR="/Users/mich/work/Antigravity/STM32F769I-Disco"
cd "$PROJECT_DIR"

echo "========================================="
echo "STM32F769I-Discovery USB CDC Diagnostic"
echo "========================================="
echo ""
echo "Building firmware v0.1.113..."
echo ""

# Clean and build
make clean
make -j8

echo ""
echo "Build complete!"
echo ""
echo "========================================="
echo "Firmware Size:"
ls -lh build/TestF.bin
echo ""
echo "========================================="
echo ""
echo "IMPORTANT: Before flashing, ensure:"
echo "  1. USB cable connected to CN16 (ST-LINK)"
echo "  2. USB cable connected to CN13 (USB User FS) - THIS IS CRITICAL!"
echo ""
echo "The CN13 port is on the RIGHT side of the board,"
echo "near the Arduino headers. It's a USB-C port."
echo ""
echo "========================================="
echo ""
read -p "Press Enter to flash firmware..."
echo ""

# Flash firmware
echo "Flashing firmware..."
st-flash --reset --connect-under-reset write build/TestF.bin 0x08000000

echo ""
echo "========================================="
echo "Flashing complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Wait 3 seconds for board to initialize"
echo "  2. Check for new USB device: ls /dev/cu.usbmodem*"
echo "  3. You should see TWO devices:"
echo "     - /dev/cu.usbmodemXXXX (ST-LINK)"
echo "     - /dev/cu.usbmodemYYYY (Firmware CDC) <- NEW!"
echo ""
echo "  4. Connect to firmware CDC:"
echo "     screen /dev/cu.usbmodemYYYY 115200"
echo ""
echo "  5. You should see diagnostic messages!"
echo ""
echo "========================================="
echo ""
