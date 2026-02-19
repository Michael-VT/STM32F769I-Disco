#!/bin/bash
# Flash script for STM32F769I-Discovery
# Use this script to verify you're flashing the correct firmware

echo "======================================"
echo "STM32F769I-Discovery Flash Tool"
echo "======================================"
echo ""
echo "STEP 1: Verify binary file exists"
echo "--------------------------------------"
if [ ! -f "Binary/TestF.bin" ]; then
    echo "ERROR: Binary/TestF.bin not found!"
    echo "Run 'make' first to build the firmware."
    exit 1
fi

# Show file info
echo "File: Binary/TestF.bin"
echo "Size: $(wc -c < Binary/TestF.bin) bytes"
echo "Date: $(ls -l Binary/TestF.bin | awk '{print $6, $7, $8}')"
echo "MD5:  $(md5 -q Binary/TestF.bin 2>/dev/null || md5sum Binary/TestF.bin | cut -d' ' -f1)"
echo ""

echo "STEP 2: Expected MD5 checksum"
echo "--------------------------------------"
echo "Correct MD5: ad5a2575bf59e0e9a2666b92c759ba6d"
CURRENT_MD5=$(md5 -q Binary/TestF.bin 2>/dev/null || md5sum Binary/TestF.bin | cut -d' ' -f1)
if [ "$CURRENT_MD5" = "ad5a2575bf59e0e9a2666b92c759ba6d" ]; then
    echo "✓ Checksum MATCHES - binary is correct"
else
    echo "✗ Checksum MISMATCH - rebuild with 'make clean && make'"
fi
echo ""

echo "STEP 3: Flash commands"
echo "--------------------------------------"
echo ""
echo "Method 1: Using st-flash (recommended)"
echo "  st-flash write \"Binary/TestF.bin\" 0x08000000"
echo ""
echo "Method 2: Using STM32CubeProgrammer"
echo "  STM32_Programmer_CLI -c port=swd -w \"Binary/TestF.hex\" -rst"
echo ""
echo "Method 3: Using OpenOCD"
echo "  openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \\
    -c \"program Binary/TestF.bin verify reset exit 0x08000000\""
echo ""

echo "STEP 4: After flashing - verify it's working"
echo "--------------------------------------"
echo "Check these things:"
echo "  1. Screen should display test info (version 0.1.52)"
echo "  2. Green LED (LED1) should blink 3 times at startup, then periodically"
echo "  3. USB CDC device should appear on your computer"
echo "  4. After 5-10 seconds, blue LED on ESP8266 should blink"
echo ""
echo "If screen is blank or wrong:"
echo "  - Try power cycling the board"
echo "  - Hold the RESET button, then release"
echo "  - Check MD5 checksum above"
echo ""
echo "======================================"

# Auto-flash if st-flash is available
if command -v st-flash &> /dev/null; then
    echo ""
    echo "st-flash found! Auto-flashing..."
    echo "======================================"
    st-flash write "Binary/TestF.bin" 0x08000000
    FLASH_RESULT=$?
    echo "======================================"
    if [ $FLASH_RESULT -eq 0 ]; then
        echo "✓ Flash successful!"
        echo "Press RESET button on the board now."
    else
        echo "✗ Flash failed! Check ST-Link connection."
    fi
else
    echo "Note: st-flash not found. Install with: brew install stlink"
fi
