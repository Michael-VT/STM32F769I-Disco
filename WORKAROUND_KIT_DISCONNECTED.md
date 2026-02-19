================================================================================
              WORKAROUND: Working With/Without Kit Connected
================================================================================

Date: 2026-02-09
Project: STM32F769I-DISCO TestF Firmware
Version: v0.1.16

================================================================================
PROBLEM DESCRIPTION
================================================================================

You periodically need to disconnect the STM32F769I-DISCO kit from the computer.
This causes issues with:
1. Build system may fail when kit is not connected
2. Flashing operations fail without the kit
3. Serial port (/dev/cu.usbmodem3657395133351) disappears
4. Debugging becomes impossible

================================================================================
SOLUTION: KIT DETECTION SCRIPTS
================================================================================

Create helper scripts to detect kit presence and adapt workflow accordingly.

--------------------------------------------------------------------------------
SCRIPT 1: Check if Kit is Connected
--------------------------------------------------------------------------------
File: scripts/check_kit.sh

#!/bin/bash
# Check if STM32F769I-DISCO kit is connected

# Check for ST-LINK/V2-1
STLINK=$(lsusb | grep "ST-LINK/V2" | grep "STMicroelectronics")

# Check for virtual COM port
VCOM=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)

if [ -n "$STLINK" ] && [ -n "$VCOM" ]; then
    echo "KIT_CONNECTED"
    echo "$VCOM"
    exit 0
else
    echo "KIT_DISCONNECTED"
    exit 1
fi

--------------------------------------------------------------------------------
SCRIPT 2: Smart Build Command
--------------------------------------------------------------------------------
File: scripts/build.sh

#!/bin/bash
# Build firmware with or without kit connected

PROJECT_DIR="/Users/mich/work/Antigravity/STM32F769I-Disco"
cd "$PROJECT_DIR"

# Build firmware
echo "Building firmware..."
make clean
make -j4

if [ $? -ne 0 ]; then
    echo "Build FAILED!"
    exit 1
fi

echo "Build SUCCESS!"

# Check if kit is connected
source scripts/check_kit.sh
if [ $? -eq 0 ]; then
    echo ""
    echo "Kit is connected. Flashing firmware..."
    st-flash write build/TestF.bin 0x08000000
    if [ $? -eq 0 ]; then
        echo "Flash SUCCESS!"
        echo ""
        echo "Connect to serial port:"
        echo "  screen $VCOM 115200"
    else
        echo "Flash FAILED!"
        exit 1
    fi
else
    echo ""
    echo "Kit is NOT connected. Firmware built but NOT flashed."
    echo "Flash when kit is connected:"
    echo "  st-flash write build/TestF.bin 0x08000000"
fi

--------------------------------------------------------------------------------
SCRIPT 3: Connect to Serial Port (Safe)
--------------------------------------------------------------------------------
File: scripts/serial.sh

#!/bin/bash
# Connect to serial port, with kit detection

source scripts/check_kit.sh

if [ $? -ne 0 ]; then
    echo "Error: Kit is NOT connected!"
    echo "Please connect the STM32F769I-DISCO kit."
    exit 1
fi

echo "Connecting to $VCOM at 115200 baud..."
echo "Press Ctrl+A then D to exit screen"
sleep 1
screen "$VCOM" 115200

================================================================================
CODING PRACTICES FOR WORKING WITHOUT KIT
================================================================================

1. USE CONDITIONAL CODE FOR HARDWARE-DEPENDENT FEATURES

```c
// In main.c or freertos.c

// Define kit detection based on compile-time option or runtime check
#ifndef KIT_NOT_CONNECTED

// Code that requires kit (USB CDC, ESP8266, LCD)
void StartDisplayTask(void *argument) {
    // ... normal display task code ...
}

#else

// Stub version for development without kit
void StartDisplayTask(void *argument) {
    // Log to file or buffer instead
    for(;;) {
        osDelay(10000);
        // Store time/data in memory buffer for later retrieval
    }
}

#endif
```

2. ADD LOGGING BUFFER FOR OFFLINE DEBUGGING

```c
// Add to main.c
#define DEBUG_LOG_SIZE 4096
char debug_log[DEBUG_LOG_SIZE];
uint32_t debug_log_idx = 0;

void debug_log_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);

    // Always log to buffer
    debug_log_idx += vsnprintf(
        &debug_log[debug_log_idx],
        DEBUG_LOG_SIZE - debug_log_idx,
        format, args
    );

    // Try to send to USB CDC if available
    if (cdcTransmitMutexId != NULL) {
        CDC_Transmit_ThreadSafe((uint8_t*)debug_log, strlen(debug_log));
    }

    va_end(args);
}
```

3. MAKEFILE TARGET FOR KIT-DETECTED BUILD

```makefile
# Add to Makefile

.PHONY: check-kit build-safe build-and-flash

check-kit:
	@echo "Checking for kit connection..."
	@if ! lsusb | grep -q "ST-LINK/V2"; then \
		echo "WARNING: Kit NOT connected - building only"; \
	fi

build-safe: check-kit
	@echo "Building firmware..."
	$(MAKE) clean
	$(MAKE) -j4

build-and-flash: build-safe
	@if lsusb | grep -q "ST-LINK/V2"; then \
		echo "Flashing firmware..."; \
		st-flash write build/TestF.bin 0x08000000; \
	else \
		echo "Skipping flash - kit not connected"; \
	fi
```

================================================================================
GIT WORKFLOW FOR DISCONNECTED WORK
================================================================================

1. CREATE A GIT PRE-COMMIT HOOK

File: .git/hooks/pre-commit

#!/bin/bash
# Warn if committing without testing on kit

if ! lsusb | grep -q "ST-LINK/V2"; then
    echo ""
    echo "=========================================="
    echo "  WARNING: Kit is NOT connected!"
    echo "  You are committing UNTESTED code!"
    echo "=========================================="
    echo ""
    echo "Commit anyway? (y/N)"
    read -n 1 answer
    echo ""
    if [ "$answer" != "y" ] && [ "$answer" != "Y" ]; then
        echo "Commit aborted."
        exit 1
    fi
fi

2. BRANCH STRATEGY

- **main**: Tested on kit, working code
- **dev/***: Development without kit
- **feature/***: Specific features, may be untested

Example:
```bash
# Start new feature without kit
git checkout -b feature/display-fix

# Work on code, commit (with warnings)
git add .
git commit -m "WIP: Display height fix (UNTESTED)"

# When kit is available, merge and test
git checkout main
git merge feature/display-fix
make test-on-kit
```

================================================================================
BACKUP AND VERSION STRATEGY
================================================================================

1. BEFORE EACH CODE CHANGE STAGE:

```bash
# Update version
# VERSION.h: increment PATCH version
# v0.1.16 -> v0.1.17

# Create backup commit
git add .
git commit -m "Backup v0.1.17 before display fix"

# Create tag
git tag backup-v0.1.17-$(date +%Y%m%d-%H%M%S)
```

2. AFTER SUCCESSFUL FIX:

```bash
# Update version to MINOR if feature works
# v0.1.17 -> v0.2.0

# Create release commit
git add .
git commit -m "Release v0.2.0: Display fix working"

# Create release tag
git tag v0.2.0
```

================================================================================
SIMULATOR/EMULATOR OPTIONS (Future Work)
================================================================================

Consider these for development without kit:

1. QEMU STM32 emulation
2. Renode simulation framework
3. STM32CubeIDE simulator mode
4. Unit testing with mocks for hardware

================================================================================
SUMMARY
================================================================================

With these workarounds, you can:

1. ✅ Build code without kit connected
2. ✅ Commit changes with warnings about testing
3. ✅ Flash firmware when kit is available
4. ✅ Maintain separate branches for tested/untested code
5. ✅ Use logging buffer for offline debugging

================================================================================
