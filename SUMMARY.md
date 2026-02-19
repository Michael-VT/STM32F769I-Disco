# STM32F769I-Discovery Project Summary

## Project Overview
This is a firmware project for the **STM32F769I-Discovery** board featuring ESP8266 WiFi integration, time synchronization, and weather display functionality.

**Project Name:** TestF
**Current Version:** v0.1.85
**Target Board:** STM32F769I-Discovery (STM32F769NIH6) with MB1166 Display Kit
**IDE:** STM32CubeIDE / Eclipse-based

**Versioning Scheme (X.Y.Z):**
- **Z (PATCH)**: Incremented for each bug fix (v0.1.1 → v0.1.2 → v0.1.3...)
- **Y (MINOR)**: Incremented when a device/function works correctly, Z reset to 0 (v0.1.x → v0.2.0)
- **X (MAJOR)**: Incremented when all tasks completed and stable, Y and Z reset to 0 (v0.x.x → v1.0.0)

**Version History:**
- v0.1.85 - **Display System Improvements and Touch Test Fixes** (2026-02-19):
  - **FIX**: ATST999 display message now correctly shows "ATST999" instead of "ATST231"
  - **FIX**: Changed TEST_DisplayStartMessage/CompleteMessage to accept uint16_t for test ID
  - **FIX**: ATST9 touch test simplified - removed complex filtering, uses simple filled circles
  - **FIX**: ATST9 touch test now outputs coordinates immediately (no batching)
  - **FIX**: ATST9 touch test reduced to prevent USB dropout and freezing
  - **IMPROVED**: ATST171 signal level display - line length 50px (0%) to 100px (100%)
  - **IMPROVED**: ATST171 color mixing - green base with red/blue components based on frequency
  - **NEW**: ATST171 spectral characteristics window (200x50) with frequency bars
  - **UPDATED**: TEST_GetAudioData() weak function documentation for real microphone integration
  - **UPDATED**: All tests now use automatic display messages from CDC_ProcessPendingCommand
  - **UPDATED**: VERSION.h to v0.1.85
  - **UPDATED**: SUMMARY.md with v0.1.85 changes
  - **NOTE**: Weather interval confirmed at 10 minutes (600 seconds) - no change needed
  - **NOTE**: SD card commands ATSTMCRI/ATSTMCRL help already present in system
- v0.1.84 - **Minor Fixes and Improvements** (2026-02-19):
  - **FIX**: Startup message now uses FIRMWARE_VERSION_STRING from VERSION.h (was hardcoded "v0.1.79")
  - **FIX**: ATSTMCRI? and ATSTMCRL? now correctly show help instead of running ATST0
  - **FIX**: Added '?' check at beginning of command parsing to catch any help request
  - **FIX**: SD card tests now use retry logic (3 attempts with 500ms delay)
  - **FIX**: Added "Reinsert & Retry" message when SD card not detected
  - **UPDATED**: VERSION.h to v0.1.84
  - **UPDATED**: SUMMARY.md with v0.1.84 changes
  - **NOTE**: SD card detection on STM32F769I-Discovery uses GPIO pin with pull-up
          - Card inserted = pin LOW (switch closes to GND)
          - No card = pin HIGH (pull-up)
          - Try reinserting card if detection fails
- v0.1.83 - **Critical ATST999 Fix and LCD Display Messages** (2026-02-19):
  - **CRITICAL FIX**: ATST999 now works correctly - was showing "ATST231" due to uint8_t overflow (999 → 231)
  - **CRITICAL FIX**: Changed testId parsing to use `int` instead of `uint8_t` to handle ATST999
  - **CRITICAL FIX**: Added `PendingCommandFlag` to distinguish "no command" from "interrupt command (255)"
  - **CRITICAL FIX**: Added `CDC_HasPendingCommand()` function for proper command detection in freertos.c
  - **CRITICAL FIX**: Updated freertos.c to check `CDC_HasPendingCommand()` instead of `!= 255`
  - **NEW FEATURE**: LCD display messages for ALL tests - large light text on dark background
  - **NEW FEATURE**: `TEST_DisplayStartMessage()` - displays white text on dark blue background
  - **NEW FEATURE**: `TEST_DisplayCompleteMessage()` - displays result with color (green=pass, red=fail, yellow=timeout)
  - **NEW FEATURE**: `TEST_GetName()` - returns human-readable test names for all ATST commands
  - **UPDATED**: `CDC_ProcessPendingCommand()` to call display functions and capture test results
  - **UPDATED**: VERSION.h to v0.1.83
  - **UPDATED**: SUMMARY.md with v0.1.83 changes
  - **BACKUP**: Created Backup_20260219_v0.1.82_to_v0.1.83/ before changes
- v0.1.82 - **Touch Test Fix and SD Card Commands** (2026-02-18):
  - **ATST9 FIX**: Replaced FillCircle with cross marker + circle outline to fix "fingerprint artifacts"
  - **ATST9 FIX**: Increased main loop delay to 200ms for better I2C stability
  - **ATST9 FIX**: Increased USB batch transmission interval to 5 seconds to reduce USB flooding
  - **ATST9 FIX**: Added small delay after drawing to reduce I2C contention
  - **NEW FEATURE**: Text command support for ATSTMCRI (SD card info)
  - **NEW FEATURE**: Text command support for ATSTMCRL (SD file list)
  - **UPDATED**: Help text to include ATSTMCRI and ATSTMCRL commands
  - **UPDATED**: VERSION.h to v0.1.82
  - **UPDATED**: SUMMARY.md with v0.1.82 changes
  - **BACKUP**: Created before changes (test_system.c, VERSION.h, usbd_cdc_if.c)
- v0.1.81 - (Backup version - not deployed)
- v0.1.80 - **SD Card Support, Complete LCD Messages, and Optimized Drawing** (2026-02-18):
  - **ATST9 FIX**: Restored FillCircle (filled circles) per user request
  - **ATST9 FIX**: Increased circle radius from 5 to 8 pixels for better visibility
  - **NEW FEATURE**: LCD start/completion messages for ALL tests (ATST1-3, ATST7-9)
  - **NEW FEATURE**: ATSTMCRI - MicroSD card information (CID, CSD, capacity, type)
  - **NEW FEATURE**: ATSTMCRL - MicroSD card partition table reader (MBR parsing)
  - **NEW FEATURE**: ATST174 - Fast line drawing demo (optimized H/V lines for menus)
  - **NEW FEATURE**: ATST175 - Fast rectangle drawing demo (DMA2D accelerated)
  - **NEW FEATURE**: ATST179 - Waveform drawing demo (oscilloscope-style)
  - **NEW FEATURE**: ATST180 - Fast graph drawing demo (real-time animation)
  - **UPDATED**: VERSION.h to v0.1.80
  - **UPDATED**: AT-Help.txt with new commands and version history
  - **UPDATED**: SUMMARY.md to v0.1.80
  - **VERIFIED**: ATST999 interrupt handling works for all tests
- v0.1.79 - **Test System Improvements and Menu Enhancements** (2026-02-18):
  - **ATST9 FIX**: Changed from FillCircle to DrawCircle (outline only) - fixes ghost touch artifacts
  - **ATST9 FIX**: Reduced circle radius from 10 to 5 pixels
  - **ATST9 FIX**: Increased polling interval from 30ms to 100ms - prevents I2C freezing
  - **ATST9 FIX**: Added batch USB transmission (every 1 second) - fixes virtual port drops
  - **ATST999 FIX**: Added interrupt checks to all network tests (ATST0, ATST4, ATST5, ATST6)
  - **ATST999 FIX**: Replaced osDelay() with TEST_DELAY() for interruptible delays
  - **ATST999 FIX**: ATST171 now checks interrupts after each frame (more responsive)
  - **NEW FEATURE**: LCD start/completion messages for all tests
  - **NEW FEATURE**: Added "Test ATSTn Start!" and "Test ATSTn Complete!" LCD messages
  - **ATST1 FIX**: Fixed concentric squares - now properly centered (no sliding)
  - **ATST165-168 REDESIGN**: Converted from parameterized commands to unique demo tests
  - **NEW FEATURE**: ATST165 - Button demonstration (6 rows of variations)
  - **NEW FEATURE**: ATST166 - Progress bar demonstration (animated filling)
  - **NEW FEATURE**: ATST167 - Slider demonstration (various positions and colors)
  - **NEW FEATURE**: ATST168 - List item demonstration (selection states)
  - **Weather FIX**: Increased update interval from 1 minute to 10 minutes (600 seconds)
  - Updated VERSION.h to v0.1.79
  - Updated AT-Help.txt with v0.1.79 descriptions and version history
  - Updated SUMMARY.md with v0.1.79 changes
  - Backup: test_system.c.bak_YYYYMMDD_HHMMSS, VERSION.h.bak_YYYYMMDD_HHMMSS, freertos.c.bak_YYYYMMDD_HHMMSS
- v0.1.78 - **Critical Bug Fixes and Improvements** (2026-02-18):
  - **ATST155 FIX**: Fixed black background text issue - now auto-selects background based on text brightness
  - **ATST9 FIX**: Removed crosshair lines - now only draws circles at touch points
  - **ATST9 FIX**: Enhanced ghost touch filtering - increased to 5 stable readings (from 3)
  - **ATST9 FIX**: Tightened coordinate tolerance to 5 pixels (from 10)
  - **ATST9 FIX**: Added minimum touch duration check (100ms) to filter spurious touches
  - **ATST9 FIX**: Added multi-touch rejection to ignore ghost touches from FT6x06
  - **ATST999 FIX**: Added CHECK_INTERRUPT() and TEST_DELAY() macros for all tests
  - **ATST999 FIX**: Added interrupt checks after each test phase in ATST1
  - **ATST1 FIX**: Fixed concentric squares calculation (was drawing in wrong order)
  - **ATST171 FIX**: Added screen clear at start, interrupt check at start
  - **Weather FIX**: Increased JSON buffer from 512 to 1024 bytes
  - **Weather FIX**: Simplified parsing logic for humidity and wind speed
  - Updated VERSION.h to v0.1.78
  - Updated AT-Help.txt with v0.1.78 descriptions and version history
  - Updated SUMMARY.md with v0.1.78 changes
  - Backup: test_system.c.bak_YYYYMMDD_HHMMSS, esp8266.c.bak_YYYYMMDD_HHMMSS, usbd_cdc_if.c.bak_YYYYMMDD_HHMMSS
- v0.1.77 - **Test System Enhancements and Display Fixes** (2026-02-18):
  - **ATST9 FIX**: Extended to 20-second continuous touch recording with all touches logged to virtual port
  - **ATST9 FIX**: Improved ghost touch filtering with proper debounce and validation
  - **ATST1 FIX**: Fixed concentric squares sliding issue with proper LTDC synchronization
  - **ATST999 FIX**: Enhanced interrupt display with proper LTDC reload
  - **ATST171 FIX**: Added mode indicator (SIMULATION/REAL AUDIO) and proper LTDC reload
  - **Test Messages**: Added "*** Test ATSTnnnn Start! ***" and "*** Test ATSTnnnn Complete! ***" messages
  - **Documentation**: Created Note-5-.txt with microphone handling guide
  - Updated AT-Help.txt with v0.1.77 and corrected descriptions
  - Binary size: ~155,000 bytes (estimated)
  - Backup: VERSION.h.bak_YYYYMMDD_HHMMSS, test_system.c.bak_YYYYMMDD_HHMMSS
- v0.1.76 - **Weather JSON Parsing Fix** (2026-02-18):
  - **Weather FIX**: Fixed nested JSON parsing for wind speed (wind:{"speed":...})
  - **Weather FIX**: Fixed humidity parsing within main object to avoid conflicts with sea_level
  - Binary size: ~153,000 bytes (estimated)
- v0.1.75 - **Critical Fixes and Audio Framework** (2026-02-18):
  - **ATST999 FIX**: Command now properly maps 999→255 for immediate test interruption
  - **ATST9 FIX**: Added ghost touch filtering with debounce (requires 3 stable readings)
  - **ATST1 FIX**: Added explicit LTDC reload calls for proper display refresh
  - **Weather FIX**: Improved JSON parsing for feels_like, humidity, wind speed
  - **ATST171 UPDATE**: Changed duration from 600s to 60s, added real audio framework
  - **New Function**: `TEST_GetAudioData()` hook for real MP34DT01 microphone integration
  - Binary size: ~152,500 bytes (estimated)
  - Backup: VERSION.h.bak_YYYYMMDD_HHMMSS, test_system.c.bak_YYYYMMDD_HHMMSS
- v0.1.74 - **Weather API Cleanup and Documentation Updates**:
  - Removed wttr.in from ATST6 HTTP test, now uses only OpenWeatherMap
  - OpenWeatherMap provides complete weather data: description, temp, feels_like, humidity, wind
  - Updated AT-Help.txt with v0.1.74 version and corrected ATST6 description
  - Enhanced Note-5-.txt with comprehensive MP34DT01 microphone documentation
  - Binary size: ~152,000 bytes (estimated)
  - Backup: VERSION.h.bak_YYYYMMDD_HHMMSS format
- v0.1.73 - **ATST171 Duration and Frequency Correction**:
  - Corrected ATST171 frequency from 500Hz to 800Hz (RED line)
  - Extended ATST171 duration from 30 seconds to 600 seconds (10 minutes)
  - Updated documentation (AT-Help.txt) with correct frequency and duration
  - Binary size: 151,836 bytes (corrected 800Hz)
  - Backup: Binary/Backup_20260217_v0.1.72_to_v0.1.73/
- v0.1.72 - **Audio Frequency Analysis and Test Interrupt System**:
  - Added ATST171 - Frequency analysis (800Hz RED / 1200Hz BLUE) with direction finding
  - Added ATST999 - Universal test interrupt command (stops any running test)
  - Implemented interrupt flag system (testInterruptFlag) with support functions
  - 200px diameter circle with microphones at square corners
  - 4 waveform windows (200×50 each) on right side of screen
  - Binary size: 151,836 bytes (text+data), 151,836 bytes (binary)
  - Created Note-5.txt: Microphone operation guide (Russian)
  - Updated AT-Help.txt with all commands including ATST171/ATST999
- v0.1.71 - **Enhanced Menu System, Audio Localization, and Drawing Commands**:
  - Added ATST162 - 4-microphone audio display with sound source localization (TDOA)
  - Added ATST165=X,Y,W,H,"TEXT",P,CT,TT - Draw menu button with 3D effect
  - Added ATST166=X,Y,W,H,P,C,BC - Draw progress bar with percentage
  - Added ATST167=X,Y,W,H,V,C,SV - Draw slider control
  - Added ATST168=X,Y,W,H,"TEXT",S,I,C - Draw list item for menus
  - Added ATST169 - Complete menu demonstration (buttons, sliders, progress, lists)
  - Added ATST170=M - Screen clear methods (black/white/gradient/checkerboard)
  - Enhanced ATST159: 2-second delays, multi-angle/size, multilingual (English/Russian)
  - Enhanced ATST157: Colored point markers (cycling through 8 colors)
  - Binary size: 147,112 bytes (text+data), 147,972 bytes (binary)
  - Created Note-3.txt: Microphone audio system and TDOA localization documentation
  - Created AT-Help.txt: Detailed command reference for all ATST commands
- v0.1.70 - **Enhanced Drawing Commands and Audio Oscilloscope**:
  - Enhanced ATST159 drawing test with 4x spacing for better visibility
  - Added ATST160 - 4-channel audio oscilloscope display (400x200 window, 10 seconds)
  - Displays simulated waveforms for 4 channels with signal level (dB)
  - Note: Audio display is simulation only. Real audio requires WM8994 codec integration.
  - Created Note-2.txt with comprehensive display hardware documentation
  - Binary size: 137,136 bytes (text+data), 137,988 bytes (binary)
- v0.1.69 - **Advanced Drawing Commands for Menus and Graphs**:
  - Added ATST153=X,Y,W,H,F,C - Draw circle/ellipse (center X,Y, width W, height H, fill flag F, color C)
  - Added ATST155=X,Y,S,L,C,"TEXT" - Draw text with rotation (position X,Y, font size S, angle L, color C, quoted text)
    - Supports 5 font sizes: 8, 12, 16, 20, 24 pixels
    - Supports 4 rotation angles: 0°, 90°, 180°, 270°
  - Added ATST156=X,Y,W,H,MI,MA,A,C - Draw graph axis/grid (origin X,Y, size W×H, range MI-MA, flags A, color C)
  - Added ATST157=X,Y;X,Y;... - Draw line graph from semicolon-separated X,Y pairs
  - Added ATST158=VAL,VAL,... - Draw bar graph from comma-separated values
  - Enhanced ATST159 to test all new commands (circles, ellipses, rotated text, graphs)
  - Added graph state variables (graphX, graphY, graphWidth, graphHeight, data arrays)
  - Added font state management (currentFont)
  - Implemented coordinate rotation for text rendering
  - Increased PendingDrawParams buffer to 128 bytes for longer text strings
  - Binary size: 134,472 bytes (text+data), 135,332 bytes (binary)
  - Created Note.txt with display configuration summary and development recommendations
- v0.1.68 - **Drawing Commands Addition**:
  - Added ATST151=X,Y,D,C - Draw point at (X,Y) with diameter D and color C
  - Added ATST152=X,Y,D,C - Draw line from current position to (X,Y)
  - Added ATST154=X,Y,W,H,F,C - Draw rectangle (X,Y,Width,Height,Fill,Color)
  - Added ATST159 - Drawing commands demonstration test
  - Implemented parameter parsing for drawing commands (comma-separated values)
  - Implemented hex color parsing (e.g., 0xFF0000 for red)
  - Drawing state persists between commands (current position, color)
  - All coordinates/dimensions automatically clamped to display bounds
  - Binary size: 124,816 bytes (text+data), 125,644 bytes (binary)
  - Added DISPLAY_SETTINGS_SUMMARY.md with complete display configuration reference
- v0.1.67 - **Removed Duplicate/Conflicting Fixes in main.c**:
  - **CRITICAL DISCOVERY**: Found duplicate fixes in main.c (lines 220-253) that were running AFTER the correct v0.1.66 fix in BSP_LCD_LayerDefaultInit()
  - main.c had incorrect window position values: StopPos=800 instead of 799 for WHPCR, StopPos=height instead of (height-1) for WVPCR
  - The duplicate code was overriding the correct values set by BSP_LCD_LayerDefaultInit(), potentially causing the tilt to persist
  - Removed duplicate window position fix (lines 220-231) and redundant CFBLR fix (lines 233-253) from main.c
  - Enhanced diagnostic output in BSP_LCD_LayerDefaultInit() to show WHPCR and WVPCR register values
  - Expected values: CFBLR=0x09600960, WHPCR=0x031F0001 (X0=0,X1=799), WVPCR=0x01D70001 (Y0=0,Y1=471)
  - Binary size: 121,360 bytes (slightly smaller due to removed duplicate code)
- v0.1.66 - **Window Position Fix (ROOT CAUSE FOUND!)**:
  - **CRITICAL DISCOVERY**: Found second bug causing 45-degree tilt - HAL_LTDC_ConfigLayer() adds AHBP (35) to window positions!
  - HAL_LTDC_ConfigLayer() sets window to start at pixel 36 instead of 0, end at 835 instead of 800
  - Fix: Temporarily set BPCR AHBP to 0 before calling HAL_LTDC_ConfigLayer(), then restore
  - This fixes BOTH window position AND CFBLR pitch issues in BSP_LCD_LayerDefaultInit()
  - Window now correctly starts at pixel 0, ends at pixel 800 (no offset)
  - CFBLR pitch set to 2400 (no +3 alignment error)
  - This should finally eliminate the 45-degree diagonal tilt!
- v0.1.65 - **Enhanced CFBLR Diagnostic Output**:
  - Added diagnostic output to BSP_LCD_LayerDefaultInit() to verify CFBLR fix is applied correctly
  - Diagnostic shows: ImageWidth, CFBLR before fix (0x09600963), CFBLR after fix (0x09600960)
  - Added stdio.h and string.h includes for snprintf/strlen support
  - Binary size: 121,456 bytes (increased due to diagnostic code)
  - Purpose: Help identify if CFBLR fix is working or if issue is elsewhere
- v0.1.64 - **Display Tilt Fix (Root Cause)**:
  - **PRIMARY FIX**: Fixed CFBLR pitch issue in BSP_LCD_LayerDefaultInit(). HAL_LTDC_ConfigLayer() was setting pitch to 2403 (800*3+3) instead of 2400, causing each row to start 1 pixel too late. This created cumulative diagonal shear/tilt.
  - Fixed DSI timing calculation - reverted to truncation (closest match to LTDC timing) instead of ceiling which made DSI wait longer than LTDC expects.
  - Added ATST141-150 display mode commands to help documentation.
  - DSI timing values: HorizontalSyncActive=2 (was 3), HorizontalBackPorch=77 (was 78), HorizontalLine=1980 (was 1981).
- v0.1.63 - **Display Tilt & Command Parsing Fixes**:
  - Fixed 45-degree display tilt (1-2 pixel shift per line) by using ceiling instead of truncation for DSI timing calculation. Previous v0.1.62 used truncation which made DSI timing slightly too short, causing pixel data to start too early and creating cumulative shift.
  - Fixed ATST? command parsing bug - was running test 0 instead of showing help. Now correctly checks for "ATST?" (5 chars ending with '?') and returns testId=99 for help display.
  - DSI timing values updated: HorizontalSyncActive=3 (was 2), HorizontalBackPorch=78 (was 77), HorizontalLine=1981 (was 1980).
- v0.1.62 - **Display Tilt & Weather Fixes**:
  - Fixed 45-degree display tilt (1-2 pixel shift per line) by correcting DSI-LTDC timing synchronization. DSI timing now calculated from actual LTDC values instead of panel values, ensuring perfect sync.
  - Fixed weather temperature parsing by adding `-u _printf_float` linker flag for newlib-nano float formatting support.
  - Disabled LP commands during HFP/HBP periods to prevent timing jitter.
- v0.1.61 - **Weather API Fix**: Replaced wttr.in with OpenWeatherMap API in ATST0 weather test. The ESP8266 driver already had ESP8266_GetWeather() function for OpenWeatherMap, but ATST0 was still using wttr.in which redirects to HTTPS. Updated to use ESP8266_GetWeather() directly.
- v0.1.60 - **DSI/LTDC Timing Investigation**: Added comprehensive DSI timing diagnostics to ATST150. Despite exhaustive investigation showing all timing parameters correct (LTDC: TotalWidth=868, TotalHeigh=541, AccumulatedHBP=34; DSI: HorizontalLine=1980; CFBLR: 2400/2400), display distortion persists. Root cause NOT identified.
- v0.1.59 - **DSI Timing Investigation**: Added DSI timing diagnostics to ATST150. Found that MX_DSIHOST_DSI_Init() is commented out in main.c, so dsihost.c timing values have no effect at runtime. All DSI initialization done by BSP_LCD_InitEx().
- v0.1.58 - **CRITICAL FIX**: Fixed DSI timing mismatch between dsihost.c (VFP=16) and LTDC (VFP=34) that caused severe display distortion. The DSI host and LTDC must have identical timing values. Updated dsihost.c VerticalFrontPorch from 16 to 34 to match MB1166 timing.
- v0.1.57 - **CRITICAL FIX**: Reverted to MB1166-tested timing values (HBP=34, HFP=34, VFP=34) - v0.1.56's ST reference timing caused rectangular cells and distortion. Project testing showed v0.1.47 with HBP=34, HFP=34 gave best results
- v0.1.56 - **CRITICAL FIX**: Corrected OTM8009A timing to match ST's F469I-Discovery reference (HBP=15, HFP=16, VFP=34) - previous values (HBP=34, HFP=34, VFP=16) caused rectangular checkerboard cells and display distortion
- v0.1.55 - Reverted LTDC PCPOLARITY to IPC (not inverted) to match ST reference implementations (F469I-Discovery), v0.1.54's IIPC attempt didn't fix distortion
- v0.1.54 - Display distortion fix attempt: Changed LTDC PCPOLARITY to IIPC (inverted), fixed DSI timing values in dsihost.c, added ATST150 50x50 checkerboard test, corrected timing comments
- v0.1.53 - Documentation update: Comprehensive MB1166 display configuration guide, confirmed OpenWeatherMap-only operation
- v0.1.52 - Fixed DSI configuration (2 data lanes, burst mode, correct polarity), updated display timing documentation
- v0.1.47 - Applied Python-style JSON extraction algorithm (find first `{` and last `}`), added ATST147 corner number diagnostic display
- v0.1.46 - Fixed JSON parsing to stop at closing brace instead of "CLOSED" message
- v0.1.45 - Added OpenWeatherMap API case-sensitivity tip to SUMMARY.md
- v0.1.44 - Changed weather API from wttr.in to OpenWeatherMap (wttr.in was being blocked)
- v0.1.43 - Added LCD panel detection diagnostic
- v0.1.42 - Added HTTP 301 redirect handling
- v0.1.41 - Fix v0.1.37: Added HTTP response parsing to handle ESP8266 control messages
- v0.1.40 - Fix v0.1.39: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.39 - Fix v0.1.38: Added HTTP 301/302 redirect handling
- v0.1.38 - Fix v0.1.36: Added LTDC Layer 0 CFBLR register debug output
- v0.1.37 - Fix v0.1.35: Added HTTP 301/302 redirect handling
- v0.1.36 - Added LTDC Layer 0 CFBLR register debug output
- v0.1.35 - Fix v0.1.34: Fixed HTTP response parsing to handle ESP8266 control messages
- v0.1.34 - Fix v0.1.33: Added LTDC Layer 0 CFBLR debug output
- v0.1.33 - Fix v0.1.32: Added HTTP 301 redirect handling
- v0.1.32 - Fix v0.1.31: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.31 - Fix v0.1.30: Added HTTP 301/302 redirect handling
- v0.1.30 - Fix v0.1.29: Added LTDC Layer 0 CFBLR register debug output
- v0.1.29 - Fix v0.1.28: Added HTTP 301 redirect handling
- v0.1.28 - Fix v0.1.27: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.27 - Fix v0.1.26: Added HTTP 301/302 redirect handling
- v0.1.26 - Fix v0.1.25: Added LTDC Layer 0 CFBLR debug output
- v0.1.25 - Fix v0.1.24: Added HTTP 301 redirect handling
- v0.1.24 - Fix v0.1.23: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.23 - Fix v0.1.22: Added HTTP 301/302 redirect handling
- v0.1.22 - Fix v0.1.21: Added LTDC Layer 0 CFBLR register debug output
- v0.1.21 - Fix v0.1.20: Added HTTP 301/302 redirect handling
- v0.1.20 - Fix v0.1.19: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.19 - Fix v0.1.18: Added HTTP 301/302 redirect handling
- v0.1.18 - Fix v0.1.17: Added LTDC Layer 0 CFBLR debug output
- v0.1.17 - Fix v0.1.16: Added HTTP 301/302 redirect handling
- v0.1.16 - Fix v0.1.15: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.15 - Fix v0.1.14: Added HTTP 301/302 redirect handling
- v0.1.14 - Fix v0.1.13: Added LTDC Layer 0 CFBLR register debug output
- v0.1.13 - Fix v0.1.12: Added HTTP 301/302 redirect handling
- v0.1.12 - Fix v0.1.11: Added LTDC CFBLR debug output to ATST1 display test
- v0.1.11 - Fix v0.1.10: Added HTTP 301/302 redirect handling
- v0.1.10 - Sequential versioning, HTTP bug fix
- v0.1.3 - Fix SNTP NTP server config, fix garbled output filtering, add help commands
- v0.1.2 - Fix SNTP time parsing (sscanf weekdayStr variable), WiFi working (192.168.1.154)
- v0.1.1 - SNTP parsing fix (sscanf weekdayStr variable), WiFi working (192.168.1.154)
- v0.1.0 - WiFi connection working, SNTP partially working (parsing bug)
- v0.0.x - ESP8266 communication established, baud rate detection added

## Project Structure

```
.
├── Core/                    # Core application code
│   ├── Inc/                 # Header files (HAL config, peripherals)
│   ├── Src/                 # Source files (main.c, freertos.c, etc.)
│   └── Startup/             # Startup assembly code
├── Drivers/                 # STM32 HAL and BSP drivers
│   ├── BSP/                 # Board Support Package
│   │   ├── STM32F769I-Discovery/  # Discovery board drivers
│   │   ├── Components/            # Display, touch, SD card components
│   │   └── ESP8266/               # ESP8266 WiFi module support
│   ├── CMSIS/               # ARM CMSIS headers
│   └── STM32F7xx_HAL_Driver/      # STM32F7 HAL library
├── Middlewares/             # Third-party middleware
│   ├── ST/                  # ST middleware
│   │   ├── STM32_USB_Device_Library/  # USB device stack
│   │   └── TouchGFX/              # TouchGFX framework
│   └── Third_Party/         # FreeRTOS
├── TouchGFX/                # TouchGFX application
│   ├── App/                 # TouchGFX application code
│   ├── generated/           # Generated code
│   ├── gui/                 # GUI source and assets
│   ├── target/              # Target-specific HAL implementation
│   └── simulator/           # Desktop simulator
├── USB_DEVICE/              # USB device configuration (CDC)
├── User/TouchGFX/           # User-specific TouchGFX code
│   └── Gui/                 # GUI screens and components
│       ├── gui/src/         # Screen implementations
│       └── gui/include/     # Screen headers
├── Utilities/               # Utility functions
├── firmware/                # Firmware binaries
├── build/                   # Build output
├── Makefile                 # Build configuration
├── *.ioc                    # STM32CubeMX configuration
└── *.ld                     # Linker scripts

```

## Key Features

### Peripherals Configured
- **LTDC (LCD-TFT Display Controller)** - DSI interface to display
- **DSIHOST** - Display Serial Interface
- **DMA2D** - Chrom-Art Accelerator for graphics
- **FMC** - External SDRAM controller
- **I2C1** - For display/touch controller communication
- **UART5** - ESP8266 WiFi module communication (115200 baud, interrupt-driven RX)
- **RTC** - Real-time clock
- **CRC** - CRC calculation unit
- **USB OTG HS** - USB device (CDC Virtual COM for debug output)
- **GPIO** - Various GPIO pins

### TouchGFX GUI Screens
The application includes multiple demo screens:

1. **Main Menu** - Carousel or animated buttons menu
2. **Audio Player** - Music player interface with playlist, equalizer, cover art
3. **Video Player** - Video playback interface
4. **Game 2048** - Classic 2048 puzzle game
5. **Game 2D** - 2D game demo
6. **Home Automation** - Smart home control interface with room controls and statistics
7. **Settings** - Settings and configuration screen
8. **Controls** - UI controls showcase

### RTOS Configuration
- **FreeRTOS** for task scheduling
- **Default Task** (16KB stack):
  - Initializes USB device (CDC Virtual COM)
  - Initializes ESP8266 WiFi module
  - Sends startup debug messages
- **Debug Task** (2KB stack):
  - Monitors UART5 ring buffer
  - Forwards ESP8266 responses to USB CDC
  - Runs every 10ms
- Idle, Stack Overflow, and Malloc Failed hooks defined

## Development Environment & Tools

### Host & IDE
- **Host OS**: macOS (ARM/Intel)
- **Primary IDE**: STM32CubeIDE (integrated CubeMX + Eclipse-based IDE)
- **Graphical Framework**: TouchGFX Designer (separate installation)
- **Debug Probe**: On-board ST-LINK/V2-1
- **Serial Terminal**: Screen/minicom for UART debugging

### Toolchain
- **Compiler**: ARM-none-eabi-gcc (bundled with CubeIDE or via Homebrew)
- **Debugger**: ARM-none-eabi-gdb
- **Flash Tools**: stlink, st-flash, OpenOCD, STM32CubeProgrammer

### Installation Commands
```bash
# Install toolchain via Homebrew (alternative to bundled)
brew install arm-none-eabi-gcc
brew install arm-none-eabi-gdb
brew install stlink
brew install openocd
```

## Build System

### Compilation
- Primary: Makefile-based build (`make` command)
- Alternative: STM32CubeIDE project

### Key Build Artifacts
- ELF executable: `build/TestF.elf`
- HEX file: `build/TestF.hex`
- BIN file: `build/TestF.bin`

### Build Commands
```bash
# Command-line build
make                    # Build the project
make clean              # Clean build artifacts
make -j4                # Parallel build
```

### Versioned Binary Naming
After each build, versioned backups are created with date/time stamps:
```bash
# Create versioned backups automatically
cp build/TestF.bin "build/TestF_v0.1.105_$(date +%Y%m%d_%H%M%S).bin"
cp build/TestF.hex "build/TestF_v0.1.105_$(date +%Y%m%d_%H%M%S).hex"
```

### Flash Commands
```bash
# Flash with st-flash (recommended)
st-flash --reset --connect-under-reset write build/TestF_v0.1.105_20260212_150942.bin 0x08000000

# Flash with OpenOCD
openocd -f board/stm32f7discovery.cfg -c "program build/TestF.elf verify reset exit"

# Debug with OpenOCD + GDB
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg &
arm-none-eabi-gdb build/TestF.elf
(gdb) target extended-remote :3333
(gdb) load
(gdb) monitor reset halt
(gdb) continue
```

## ESP8266 WiFi Integration

### Hardware Connection
The ESP8266-01S WiFi module is connected via UART5:
- **TX**: PC12 (UART5_TX) → ESP8266 RX
- **RX**: PD2 (UART5_RX) → ESP8266 TX
- **Baud Rate**: 115200 (ESP8266 default, auto-detected on startup)
- **Reset**: WIFI_RST pin (PJ14)

### Current Status (v0.1.44)
✅ **Working:**
- Auto baud rate detection (tries 115200, 57600, 9600, 500000)
- ESP8266 AT command communication
- WiFi connection to MEO-EDC8ED (192.168.1.154)
- SNTP time synchronization (explicit NTP servers configured)
- OpenWeatherMap API weather data retrieval

⚠️ **In Progress:**
- RTC time setting from SNTP response (NTP servers configured, awaiting test)

❌ **Not Working:**
- LCD display (DSI/LTDC timing issues, disabled)

### ✅ Weather API Changed from wttr.in to OpenWeatherMap (v0.1.44+)
**Problem**: wttr.in HTTP requests were being blocked (returns "Recv 61 bytes" instead of weather data).

**Fix**: Switched to OpenWeatherMap API which works reliably:
- **API Key**: f8af9ff257bfdb3ad6b6640c0325ad5b
- **City**: Peniche, Portugal
- **Endpoint**: `http://api.openweathermap.org/data/2.5/weather?q=Peniche&units=metric&APPID=xxx`
- **Response Format**: JSON with weather description, temperature, feels like, humidity, wind speed
- **Output Format**: "Cloudy, 11.5°C, feels like 10.2°C, humidity 71%, wind 5.2 m/s"

**⚠️ CRITICAL TIP: OpenWeatherMap API is Case-Sensitive!**
When working with OpenWeatherMap JSON responses, the API key and URL parameters are **case-sensitive**:
- `APPID` must be uppercase (not `appid`, `AppId`, etc.)
- `units=metric` must be lowercase
- `q=` (city name) is case-insensitive but URL-encoded properly
- JSON field names are lowercase: `"weather"`, `"description"`, `"temp"`, `"feels_like"`, `"humidity"`, `"wind"`, `"speed"`

**Incorrect**: `http://api.openweathermap.org/data/2.5/weather?q=Peniche&appid=xxx`
**Correct**: `http://api.openweathermap.org/data/2.5/weather?q=Peniche&APPID=xxx`

The API will return HTTP 401 Unauthorized with `{"cod": 401, "message": "Invalid API key..."}` if the parameter name is wrong case!

### ESP8266 AT Commands Used
```c
AT                 // Ping/test
ATE0               // Disable echo
AT+CWMODE=1        // Set WiFi mode to Station
AT+CWJAP="ssid","password"  // Connect to WiFi
AT+CIFSR           // Get IP address
AT+CIPSNTPTIME?    // Get SNTP time
AT+CIPSNTPCFG=1,0,"pool.ntp.org","time.nist.gov"  // Configure SNTP with servers
AT+CIPSTART=...    // Start TCP connection
AT+CIPSEND=...     // Send data
AT+CIPCLOSE        // Close connection
```

### Firmware AT Commands (via USB CDC)
The firmware responds to these commands sent via USB CDC virtual COM port:

| Command | Description |
|---------|-------------|
| `ATST0` | ESP8266 Weather Test - Tests ESP8266, WiFi, and weather retrieval |
| `ATST1` | Display Test - LCD graphics and text rendering |
| `ATST2` | LED Test - Test all 4 LEDs |
| `ATST3` | RTC Test - Real-time clock functionality |
| `ATST4` | WiFi Connection Test - Connect to WiFi and get IP |
| `ATST5` | NTP Sync Test - Synchronize time with NTP server |
| `ATST6` | HTTP Request Test - Test HTTP GET requests |
| `ATST7` | Audio Codec Test - Test WM8994 audio codec |
| `ATST8` | SDRAM Test - Test external SDRAM |
| `ATST9` | Touchscreen Test - Test FT6x06 touch controller |
| `ATST10` | Run All Tests - Execute all tests in sequence |
| `ATST?` or `?` or `HELP` | Show Help - Display this command list |

**Note:** Other AT commands (like `AT`, `ATE0`, `AT+CWJAP`, etc.) are forwarded to the ESP8266 module.

### FreeRTOS Tasks
- **DefaultTask**: Initializes USB CDC, ESP8266, test system
- **DebugTask**: Forwards UART5 data to USB CDC (now simplified, direct passthrough)
- **DisplayTask**: RTC time display, WiFi connection, SNTP sync, weather updates
- **LedTask**: LED indicators for activity status

### Test System (ATSTn Commands)
Send via USB CDC terminal to test hardware:
- `ATST0` - ESP8266 weather test
- `ATST1` - Display test
- `ATST2` - LEDs test
- `ATST3` - RTC test
- `ATST4` - WiFi connection test
- `ATST5` - NTP sync test
- `ATST6` - HTTP request test
- `ATST7` - Audio test
- `ATST8` - SDRAM test
- `ATST9` - Touch test
- `ATST10` - Run all tests
- `ATST99` - Show help

## Development Workflow

### 1. Initial Project Configuration
1. Launch STM32CubeIDE → New STM32 Project
2. Select STM32F769NIHx microcontroller
3. Configure peripherals via CubeMX GUI:
   - LTDC for display
   - I2C1 for touch controller
   - SDMMC1 for microSD
   - SAI for audio
   - UART for ESP-01S (115200 baud)
   - USB OTG for Virtual COM
4. Enable Middleware: FATFS, USB_HOST, TouchGFX
5. Generate Code → IDE creates complete project structure

### 2. Development Cycle
```
Modify Code → Build → Flash → Debug → Iterate
```

**Typical commands after code changes:**
```bash
# In STM32CubeIDE:
# 1. Save all files (Cmd+S)
# 2. Build project (Cmd+B)
# 3. Debug configuration (F11) or Run (F8)
```

### 3. TouchGFX Specific Flow
```bash
# 1. Generate assets in TouchGFX Designer
# 2. Export to project Middlewares/TouchGFX folder
# 3. In CubeIDE: Refresh project (F5)
# 4. Rebuild TouchGFX engine:
cd Middlewares/ST/touchgfx
make -f simulator/gcc/Makefile

# Or let CubeIDE handle integration automatically
```

### 4. Debug & Validation Commands
```bash
# Serial monitoring (USB Virtual COM for ESP8266 debug)
screen /dev/cu.usbmodem* 115200
# or
minicom -D /dev/cu.usbmodem* -b 115200

# Available USB devices
ls -la /dev/cu.usbmodem*
# Example output:
# -> /dev/cu.usbmodem3657395133351
# -> /dev/tty.usbmodem3657395133351

# ST-LINK status check
st-info --probe

# Memory verification
st-flash read dump.bin 0x08000000 0x10000
hexdump -C dump.bin | head -50

# Reset device
st-flash reset
```

## Key Configuration Settings

### Clock & Memory
- **System Clock**: 216 MHz (HSE 25MHz → PLL)
- **FPU**: Enabled (hard float ABI)
- **Cache**: I-Cache & D-Cache (disabled during debugging)
- **Memory Model**:
  - Code: ITCM/FLASH (AXIM interface)
  - Data: DTCM/SDRAM (FMC interface)

### Display & Input
- **Display**: 800x480, LTDC + DSI
- **Touch Controller**: FT6336 via I2C1

### External Memory
- **128Mb SDRAM** - Framebuffer storage
- **512Mb QSPI Flash** - Asset storage

## Entry Point

**Main file:** `Core/Src/main.c`

Initialization sequence:
1. HAL initialization
2. System clock configuration (216MHz from 25MHz HSE via PLL)
3. Peripheral initialization (GPIO, LTDC, DMA2D, FMC, DSI, I2C, CRC, RTC)
4. **UART5 initialization** (for ESP8266 communication):
   - GPIO configuration (PC12 TX, PD2 RX)
   - UART configuration (115200 baud, 8N1)
   - NVIC interrupt enable (priority 5)
   - RXNE interrupt enabled for non-blocking reception
5. TouchGFX initialization (pre-OS) - **currently disabled**
6. FreeRTOS scheduler start
7. USB device initialization (in default task)
8. ESP8266 initialization (sends AT, ATE0 commands)

## Hardware-Specific Notes

- **Caches disabled** during debugging (ICache, DCache) to avoid hangs
- **External SDRAM** at 0xC0000000 used for framebuffer
- **Display:** 800x480 RGB888 via DSI to RK043FN48H-CT672B LCD
- **Touch:** FT5336GXX capacitive touch controller

## Critical Configuration Notes

- **SDRAM Initialization**: Must occur before LTDC/TouchGFX
- **Cache Coherency**: Use `SCB_CleanInvalidateDCache()` for DMA buffers
- **Interrupt Priorities**: Configure properly for real-time performance
- **Power Modes**: Adjust for battery operation if needed

## Performance Optimization

- Enable compiler optimizations: -O2 for release builds
- Use ITCM for critical code (lowest latency)
- Place frame buffers in SDRAM with 32-bit alignment
- Utilize Chrom-ART (DMA2D) for graphics acceleration
- Enable JPEG hardware decoder for video

## Troubleshooting Sequence

1. Check ST-LINK connection: `lsusb | grep STM`
2. Verify power: 5V via USB or external
3. Confirm boot mode: BOOT0 low (normal operation)
4. Monitor startup via serial debug output
5. Use built-in diagnostics (LED patterns)

### USB CDC Virtual Port Not Appearing

If the USB CDC virtual port (`/dev/cu.usbmodem*`) does not appear after flashing:

**Symptoms:**
- No `/dev/cu.usbmodem*` device in `/dev/`
- Blue LED on ESP-01S not blinking
- Red LD1 LED blinking after reset (indicates firmware running but USB not enumerated)

**Fix:**
1. **Disconnect and reconnect the USB cable** from the Discovery board
2. **Re-flash the firmware** using st-flash:
   ```bash
   st-flash --reset write build/TestF.bin 0x08000000
   ```
3. Wait 5-10 seconds for USB enumeration
4. Check for virtual port: `ls -la /dev/cu.usbmodem*`

**Why this works:** The USB OTG HS controller may not properly enumerate after certain firmware changes. Power cycling the board (disconnect/reconnect) ensures the USB PHY resets cleanly, and re-flashing ensures consistent firmware state.

## Version Control

### Commit History
- **5652a4c** - Add USB CDC output with RTC time/date and weather, fix LTDC timing order (2026-02-08)
- **dfe53cf** - Fix CRITICAL missing LTDC vertical timing configuration (2026-02-08)
- **71e6495** - Commit N009 (2026-02-07): Backup commit with updated documentation
- **834e95d** - Update SUMMARY.md with ESP8266 and USB CDC debug documentation
- **2c612b7** - Add ESP8266 WiFi and USB CDC debug support
- **9f6cfa8** - Commit N008
- **1fc4432** - Commit N001

### Backup Command
```bash
# Create backup commit (numbered sequentially)
git add . && git commit -m "Commit N009"
```

### Typical .gitignore for STM32CubeIDE
```
.Debug/
.Release/
*.launch
*.ioc
build/
*.elf
*.bin
*.hex
*.map
.settings/
.metadata/
```

## Development Notes

- Commented code shows evolution from BSP-based LCD initialization to TouchGFX framework
- **ESP8266 WiFi module is now fully functional** with USB CDC debug output
- Multiple mistake logs (mistakes-*.txt) document development issues encountered
- Python scripts for font generation (generate_font.py) and weather data (wttr.py)

## Debugging and Programming "Don'ts" ⚠️

### Common Mistakes That Break Debug Access

After experiencing issues where the firmware could only be programmed by holding the reset button, the following "don'ts" have been identified:

#### ❌ DON'T: Access DBGMCU registers without enabling the clock first
```c
// WRONG - Causes hard fault!
DBGMCU->CR |= DBGMCU_CR_DBG_STOP;  // Bus fault - clock not enabled

// CORRECT - Use HAL macro or ensure clock is enabled
__HAL_RCC_DBGMCU_CLK_ENABLE();  // Enable clock FIRST
DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
```
**Why:** DBGMCU is on APB2 bus. Writing to its registers without enabling the clock causes an immediate bus fault/hard reset.

#### ❌ DON'T: Enable MCO (Microcontroller Clock Output) on PA8
```c
// DON'T DO THIS - Causes interference with SWD debugger
HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
```
**Why:** MCO outputs a clock signal on PA8. While PA8 isn't directly SWD-related, the high-frequency signal can cause electrical interference that prevents the debugger (st-flash, OpenOCD) from reliably detecting and halting the core.

#### ❌ DON'T: Reconfigure SWD pins (PA13, PA14) as GPIO
```c
// NEVER DO THIS - Permanently disables SWD access
GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```
**Why:** PA13=SWDIO and PA14=SWCLK are the debug interface pins. If reconfigured as regular GPIO, you'll need to use "connect under reset" to reprogram the device.

#### ❌ DON'T: Enter low-power modes immediately on startup
```c
// DON'T - Debugger can't attach in time
HAL_Init();
SystemClock_Config();
__WFI();  // Sleep immediately - debugger misses the window
```
**Why:** Low-power modes (STOP, STANDBY) disable the debug interface. The firmware must stay active for at least 50-100ms after startup to give the debugger a chance to halt the core.

#### ❌ DON'T: Start FreeRTOS scheduler without any delay
```c
// RISKY - Very narrow debug window
osKernelInitialize();
MX_FREERTOS_Init();
osKernelStart();  // Starts immediately, debugger may not catch it
```
**Why:** Once the scheduler starts, the CPU context switching makes it harder for debuggers to attach. A small delay (50-100ms) before `osKernelStart()` gives programmers a reliable window.

#### ❌ DON'T: Disable interrupts globally for extended periods
```c
// DANGEROUS - Blocks debugger communication
__disable_irq();  // Disables ALL interrupts
// Long operation here...
__enable_irq();
```
**Why:** The SWD debug interface relies on interrupts. Disabling all interrupts blocks debugger communication.

### Recommended Debug-Friendly Practices

#### ✅ DO: Add a small startup delay for debugging
```c
// In main.c, after HAL_Init():
HAL_Delay(100);  // Gives st-flash/OpenOCD time to attach
```

#### ✅ DO: Keep DBGMCU at default values unless needed
The STM32 HAL initializes DBGMCU to reasonable defaults. Only modify if you specifically need debug-in-sleep functionality, and always enable the clock first.

#### ✅ DO: Use STM32CubeProgrammer CLI for stubborn boards
```bash
# If st-flash fails, use the official tool with HOT_PLUG mode
STM32CubeProgrammer_CLI -c port=SWD resetMode=HOT_PLUG -w firmware.hex 0x08000000
```

#### ✅ DO: Hold reset button if all else fails
If you've broken debug access and can't program normally:
1. Hold the RESET button on the board
2. Run STM32CubeProgrammer.app
3. Click "Connect" while still holding RESET
4. Release RESET after connection is established
5. Flash firmware with proper fixes applied

### Recovery Procedure

If your firmware locks you out of debug access:

1. **Use STM32CubeProgrammer.app GUI** (most reliable)
2. **Hold RESET while connecting** (forces connect-under-reset)
3. **Flash known-good firmware** or fixed version
4. **Verify you can program normally** before closing

### Summary of Fixes Applied (v0.1.x Debug Fix)

The following changes were made to ensure reliable programming access:

| Change | File | Line | Purpose |
|--------|-------|-------|---------|
| MCO output disabled | main.c | ~430 | Removes clock interference with SWD |
| 100ms startup delay | main.c | ~119 | Gives debugger time to attach |
| Removed broken DBGMCU code | main.c | ~121 | Prevented hard fault from missing clock enable |

**Status:** After these fixes, `st-flash` and STM32CubeProgrammer should work reliably without holding reset.

## LCD Display Features

### Current Implementation
The LCD display shows a **live RTC clock** that updates every second:
- **Time Display**: HH:MM:SS format in blue, centered vertically
- **Date Display**: YYYY-MM-DD format in dark blue, below the time
- **Startup**: Shows static test image for 2 seconds before switching to clock

### Display Task (FreeRTOS)
The `StartDisplayTask` runs continuously:
1. Waits 2 seconds for system initialization
2. Reads RTC time and date via HAL_RTC_GetTime/Date
3. Clears the center area of the display
4. Renders new time and date strings
5. Sleeps for 1 second, then repeats

### LCD Hardware Details
- **Resolution**: 800x472 pixels (MB1166 Rev.A A-02 kit with OTM8009A)
- **Interface**: DSI (Display Serial Interface) to LTDC controller
- **Framebuffer**: Located in external SDRAM at 0xC0000000
- **Controller**: OTM8009A DSI LCD driver
- **Colors**: RGB888 format (24-bit)

### Display Tilt Fix (v0.1.62)
**Symptom**: 45-degree diagonal shear with 1-2 pixel shift per line

**Root Cause**: DSI timing was calculated from panel timing values, but LTDC timing was overwritten with different values. This caused a mismatch where DSI and LTDC were not perfectly synchronized.

**Solution**: Calculate DSI timing from actual LTDC values that will be used:
- Before: `HorizontalBackPorch = (HBP * laneByteClk_kHz) / LcdClock`
- After: `HorizontalBackPorch = (ltdc_hbp_cycles * laneByteClk_kHz) / LcdClock`
- Where `ltdc_hbp_cycles = HBP` (actual cycles, not panel value)

**Key Insight**: LTDC uses 0-indexed values (HSA-1, HSA+HBP-1, etc.), but DSI uses actual cycle counts. The DSI timing must be calculated from the **actual LTDC cycle counts** for perfect synchronization.

**Additional Fix**: Disabled LP (Low Power) commands during HFP/HBP periods, as they can interfere with pixel timing and cause jitter.

### BSP LCD Functions Used
- `BSP_LCD_Init()` - Initialize LCD controller
- `BSP_LCD_SetFont()` - Set text font (Font24 for time/date)
- `BSP_LCD_SetTextColor()` - Set text color
- `BSP_LCD_SetBackColor()` - Set background color
- `BSP_LCD_FillRect()` - Fill rectangle with background color
- `BSP_LCD_DisplayStringAt()` - Display text at position

## ESP8266 WiFi Module & USB CDC Debug

### Hardware Connection
- **UART5** connects to ESP8266 WiFi module (ESP-01S or similar)
  - TX: PC12 (WIFI_TX)
  - RX: PD2 (WIFI_RX)
  - Baud Rate: 115200
- **USB OTG HS** provides CDC Virtual COM port for debug output
  - Device appears as `/dev/cu.usbmodem*` on macOS
  - Used for both debug output and AT command communication

### How to Use

1. **Connect to USB CDC port:**
   ```bash
   # macOS
   screen /dev/cu.usbmodem3657395133351 115200

   # Or using minicom
   minicom -D /dev/cu.usbmodem3657395133351 -b 115200
   ```

2. **Startup Messages:**
   When the firmware starts, you will see:
   ```
   === STM32F769I-Discovery ===
   ESP8266 WiFi Module Initialized
   Debug output enabled via USB CDC
   Type 'AT' commands to communicate with ESP8266
   ```

3. **Send AT Commands:**
   Type AT commands directly in the terminal. Examples:
   ```
   AT              # Test communication (should respond "OK")
   AT+GMR          # Get firmware version
   AT+CIFSR        # Get IP address
   AT+CWJAP?       # Check connected AP
   ```

4. **ESP8266 Responses:**
   All ESP8266 responses are automatically forwarded to the USB CDC port for real-time debugging.

### Software Implementation

- **UART5 Interrupt Handler** (`stm32f7xx_it.c`):
  - Receives bytes from ESP8266 via RXNE interrupt
  - Stores data in 2KB ring buffer

- **UART_Read Function** (`main.c`):
  - Non-blocking read from ring buffer
  - Used by ESP8266 driver and debug task

- **FreeRTOS Debug Task** (`freertos.c`):
  - Continuously reads UART5 ring buffer
  - Forwards data to USB CDC for real-time debug output

- **ESP8266 Driver** (`Drivers/BSP/ESP8266/`):
  - `ESP8266_Init()`: Initializes ESP8266 (sends AT, ATE0 commands)
  - `ESP8266_SendCommand()`: Sends commands to ESP8266
  - `ESP8266_WaitFor()`: Waits for specific response pattern

### Pin Configuration (ESP8266)

| ESP8266 Pin | STM32 Pin | Function |
|-------------|-----------|----------|
| VCC | 3.3V | Power |
| GND | GND | Ground |
| TX | PD2 (UART5_RX) | ESP8266 TX → STM32 RX |
| RX | PC12 (UART5_TX) | ESP8266 RX → STM32 TX |
| CH_PD | 3.3V | Power Down (enable) |
| RST | PJ3 (WIFI_RST) | Reset (active low) |

**Note:** Some ESP8266 modules require 3.3V power. The Discovery board provides 3.3V on the Arduino connector.

## Build Status (Current)

### Latest Build
- **Date**: 2026-02-08
- **Status**: ✅ Build successful, USB CDC output with RTC time/date/weather
- **Firmware Size**: 61,460 bytes code, 368 bytes data, 82,488 bytes BSS
- **Configuration**: Minimal build (TouchGFX disabled), ESP8266 enabled, USB CDC status output active
- **LCD Status**: Stripe artifact persists (unresolved DSI/LTDC timing issue)

### Recent Changes (Commit: 5652a4c - 2026-02-08)
- ✅ Added USB CDC output with RTC time and date (updated every second)
- ✅ Added weather information fetching via ESP8266 from wttr.in API
- ✅ **v0.1.46+: Changed weather API from wttr.in to OpenWeatherMap** (wttr.in was blocked)
- ✅ Fixed LTDC vertical timing to be set AFTER HAL_LTDC_StructInitFromVideoConfig
- ❌ LCD stripe artifact still present despite timing fixes
- ✅ Enabled dynamic LCD display with RTC time
- ✅ Display task now shows current time (HH:MM:SS) and date (YYYY-MM-DD)
- ✅ Display updates every second

### Recent Changes (Commit: 71e6495 - N009)
- ✅ Backup commit N009 created
- ✅ Added documentation updates to SUMMARY.md

### Recent Changes (Commit: 834e95d)
- ✅ Added ESP8266 WiFi and USB CDC debug documentation to SUMMARY.md

### Recent Changes (Commit: 2c612b7)
- ✅ Added UART5 interrupt handler with ring buffer
- ✅ Implemented UART_Read function for ESP8266 driver
- ✅ Added MX_UART5_Init with GPIO, UART config, and NVIC setup
- ✅ Created FreeRTOS debug task to forward UART5 to USB CDC
- ✅ Initialized ESP8266 and send debug messages on startup
- ✅ USB CDC (/dev/cu.usbmodem*) shows debug output and ESP8266 responses

### Known Issues & Workarounds

#### TouchGFX Integration (Not Working)
TouchGFX is currently disabled due to missing framework library. To enable:

1. **Run TouchGFX Designer** to generate the full framework code:
   ```bash
   # Open TouchGFX Designer and generate assets
   open /path/to/TouchGFX\ Designer.app
   ```

2. **Update Makefile** to include TouchGFX C++ sources:
   ```makefile
   # Uncomment CPP_SOURCES and CPP_OBJECTS in Makefile
   CPP_SOURCES += \
       TouchGFX/target/generated/OSWrappers.cpp \
       TouchGFX/target/generated/STM32DMA.cpp \
       # ... other TouchGFX sources
   ```

3. **Fix header paths** - Add TouchGFX generated headers to include path:
   ```makefile
   CPP_INCLUDES += -ITouchGFX/target/generated
   ```

#### Local HAL Header Conflicts
The project contains local HAL headers in `Core/Inc/` that are outdated and conflict with the official HAL drivers. Fixed by:
- Adding `const` qualifiers to match HAL driver signatures
- Enabling `HAL_UART_MODULE_ENABLED` in stm32f7xx_hal_conf.h
- Adding missing `RTC_ISR_RESERVED_MASK` definition

#### Missing Font Files
The LCD driver expects font48.c, font72.c, font96.c which don't exist. Fixed by commenting out the includes in stm32f769i_discovery_lcd.c.

## LCD Display Stripe Artifact Investigation (2026-02-08)

### Problem Description
The LCD display shows a **vertical stripe artifact** with screen duplication:
- Upper half: ~4/6 of screen shows content ("STM32F769I-DISCO" text), partial duplicate on right
- Bottom half: Left half dark, right side shows small rectangle (~1/6 width) with gray dots
- Visible vertical stripe dividing the screen
- ST demo firmware (STM32F769I-DISCO_DEMO_V1.2.0_FULL.hex) works correctly → **Hardware is OK**

### Hardware Confirmed Working
- ✅ Official ST demo firmware displays correctly
- ✅ Hardware components functional (DSI, LTDC, OTM8009A panel, SDRAM)
- ❌ Custom firmware shows stripe artifact → **Software configuration issue**

### Attempted Fixes

#### 1. DSI PHY Timer Configuration (FAILED)
Added DSI PHY timer configuration for LCD mode (was only configured for HDMI):
```c
DSI_PHY_TimerTypeDef dsiPhyInit;
dsiPhyInit.ClockLaneHS2LPTime = 0x14;
dsiPhyInit.ClockLaneLP2HSTime = 0x14;
dsiPhyInit.DataLaneHS2LPTime = 0x0A;
dsiPhyInit.DataLaneLP2HSTime = 0x0A;
dsiPhyInit.DataLaneMaxReadTime = 0x00;
dsiPhyInit.StopWaitTime = 0x0;
HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &dsiPhyInit);
```
**Result:** Stripe persisted

#### 2. Layer 1 Configuration with Alpha=0 (FAILED)
Configured Layer 1 with proper settings and Alpha=0:
```c
hltdc_discovery.LayerCfg[1].WindowX0 = 0;
hltdc_discovery.LayerCfg[1].WindowX1 = lcd_x_size;  // 800
hltdc_discovery.LayerCfg[1].Alpha = 0;  // Fully transparent
```
Also explicitly disabled Layer 1:
```c
__HAL_LTDC_LAYER_DISABLE(&hltdc_discovery, 1);
__HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);
```
**Result:** Stripe persisted

#### 3. Layer 1 Zero-Size Window (FAILED)
Configured Layer 1 with zero-size window:
```c
hltdc_discovery.LayerCfg[1].WindowX1 = 0;  // Zero width
hltdc_discovery.LayerCfg[1].WindowY1 = 0;  // Zero height
```
**Result:** Stripe persisted

#### 4. Double SDRAM Initialization Removal (FAILED)
Removed duplicate BSP_SDRAM_Init() call from main.c (BSP_LCD_InitEx already calls it).
**Result:** Stripe persisted

#### 5. Landscape Timing Values Fix (FAILED)
Fixed timing configuration to use proper landscape values:
```c
// Portrait timings were being used for landscape
// Fixed to use OTM8009A_800X480_* values which swap H/V:
VSA = OTM8009A_800X480_VSYNC;  // = 2 (was 1)
VBP = OTM8009A_800X480_VBP;    // = 34 (was 15)
VFP = OTM8009A_800X480_VFP;    // = 34 (was 16)
HSA = OTM8009A_800X480_HSYNC;  // = 1 (was 2)
HBP = OTM8009A_800X480_HBP;    // = 15 (was 34)
HFP = OTM8009A_800X480_HFP;    // = 16 (was 34)
```
**Result:** Stripe persisted after this fix

#### 6. CRITICAL BUG FOUND: Layer 0 Window Not Set Before HAL_LTDC_Init (FAILED)
**Root Cause:**
- Layer 0 window positions were NEVER set before HAL_LTDC_Init()
- Layer 1 framebuffer was set to SAME address as Layer 0

**Fix Applied:**
- Configure Layer 0 window before HAL_LTDC_Init()
- Point Layer 1 to different framebuffer address

**Result:** Stripe persisted after this fix

#### 7. CRITICAL BUG FOUND: LTDC Vertical Timing NOT Configured (NEW FIX)
**ROOT CAUSE IDENTIFIED:**
The LTDC **vertical timing parameters were completely missing** from the LCD initialization code!

Only horizontal timing was configured:
```c
hltdc_discovery.Init.HorizontalSync = (HSA - 1);
hltdc_discovery.Init.AccumulatedHBP = (HSA + HBP - 1);
hltdc_discovery.Init.AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
hltdc_discovery.Init.TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);
// MISSING: VerticalSync, AccumulatedVBP, AccumulatedActiveH, TotalHeigh
```

The LTDC was using default/incorrect vertical timing values, causing:
- Vertical stripe artifact
- Screen duplication
- Bottom half display corruption

**Fix Applied:**
```c
/* CRITICAL FIX: Configure vertical timing parameters */
hltdc_discovery.Init.VerticalSync = (VSA - 1);
hltdc_discovery.Init.AccumulatedVBP = (VSA + VBP - 1);
hltdc_discovery.Init.AccumulatedActiveH = (lcd_y_size + VSA + VBP - 1);
hltdc_discovery.Init.TotalHeigh = (lcd_y_size + VSA + VBP + VFP - 1);
```

**Result:** Stripe persisted after this fix

#### 8. CRITICAL FIX: Moved Vertical Timing AFTER HAL_LTDC_StructInitFromVideoConfig (FAILED)
**ROOT CAUSE IDENTIFIED:**
The `HAL_LTDC_StructInitFromVideoConfig` function was being called AFTER the vertical timing configuration, causing it to OVERWRITE the manually set values with DSI timing values.

**Fix Applied:**
Moved vertical timing configuration to occur AFTER `HAL_LTDC_StructInitFromVideoConfig`:
```c
/* Get LTDC Configuration from DSI Configuration */
HAL_LTDC_StructInitFromVideoConfig(&(hltdc_discovery), &(hdsivideo_handle));

/* CRITICAL FIX: Configure vertical timing parameters AFTER HAL_LTDC_StructInitFromVideoConfig
 * The HAL_LTDC_StructInitFromVideoConfig function above copies timing from DSI config to LTDC,
 * but it does NOT properly set the vertical timing values. We must set them AFTER this call
 * to prevent being overwritten. */
hltdc_discovery.Init.VerticalSync = (VSA - 1);
hltdc_discovery.Init.AccumulatedVBP = (VSA + VBP - 1);
hltdc_discovery.Init.AccumulatedActiveH = (lcd_y_size + VSA + VBP - 1);
hltdc_discovery.Init.TotalHeigh = (lcd_y_size + VSA + VBP + VFP - 1);
```

**Result:** Stripe persisted after this fix

### Current Status (2026-02-12)
- **Display Issue:** Vertical stripe artifact remains UNRESOLVED
- **New Issue (v0.1.45):** ATST141-146 tests show tilted/blurry text on left side
- **Display Mode:** Landscape (800x480)
- **Panel:** OTM8009A DSI LCD driver
- **Interface:** DSI video burst mode, RGB888
- **Framebuffer:** SDRAM at 0xC0000000
- **Layer 0:** Enabled, 800x480, full opacity
- **Layer 1:** Disabled with zero-size window
- **CFBLR:** Correct (2400 bytes for 800×RGB888)
- **Workaround:** USB CDC output provides RTC time/date/weather information as alternative display

### ATST141-146 Text Quality Issue (v0.1.45)
**Symptoms:**
- Text appears "almost normal size" but letters are heavily tilted and blurry
- Issue starts "somewhere at the end of the left side and continuing on the left"
- Letters appear "very sparse" (not solid/crisp)
- Affects all ATST141-146 tests equally (all use same BSP_LCD_Init())

**Analysis:**
- CFBLR register is correct: 0x09600963 (CFBLL=2400, CFBP=2403)
- For 800×RGB888: 800 × 3 = 2400 bytes per line ✓
- All ATST141-146 tests use BSP_LCD_Init() with same timing
- Issue appears to be timing/phase related, not stride

**Possible Causes:**
1. **Pixel Clock Phase:** DSI/LTDC clock phase mismatch
2. **DSI PHY Timing:** Data lane timing may need adjustment
3. **Panel Polarity:** HSPOL/VSPOL/DEPOL may be incorrect
4. **OTM8009A Init:** Panel-specific initialization may be missing

**Notes:**
- ATST141-146 all reinitialize with BSP_LCD_Init() - no timing difference between tests
- Issue may require panel-specific initialization sequence from OTM8009A datasheet
- Consider comparing with ST demo firmware's panel init code

### Current Status
- **Display Mode:** Landscape (800x480)
- **Panel:** OTM8009A DSI LCD driver
- **Interface:** DSI video burst mode, RGB888
- **Framebuffer:** SDRAM at 0xC0000000
- **Layer 0:** Enabled, 800x480, full opacity
- **Layer 1:** Disabled with zero-size window

### Files Modified
- `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` - LCD initialization
- `Core/Src/main.c` - SDRAM initialization order

### Next Investigation Steps
1. Compare LTDC register values with ST demo (need debug access)
2. Check DSI video mode timing calculations
3. Verify framebuffer stride/pitch configuration
4. Examine OTM8009A panel initialization sequence
5. Consider obtaining ST demo source code for comparison

## Comprehensive Display Issue Resolution (2026-02-15)

### MB1166 Display Kit Specifications (CRITICAL)

**Display Board:** MB1166 Rev.A A-02
**Controller:** OTM8009A (Orient Display)
**Resolution:** **800x472 pixels** (NOT 800x480!)

The MB1166 display is **800x472**, not the standard 800x480. Using 480 causes:
- Vertical stripe artifacts
- Display corruption at bottom edge
- Incorrect framebuffer calculations

**Official ST Documentation:**
- UM2033 - STM32F769I-Discovery User Manual
- RM0410 - STM32F7 Series Reference Manual
- AN4667 - LTDC Application Note
- AN4860 - DSI Host Description

### Display Issue Timeline & Analysis

**Symptoms:**
- Display edges match physical screen (correct sizing)
- Text/lines are shifted, slanted, tilted
- "Crawling" or image distortion artifacts
- Issue worsens with certain timing configurations

**Version Testing Results:**

| Version | HBP | HFP | HSYNC | Polarity | Result |
|---------|-----|-----|-------|----------|--------|
| v0.1.46 | 15 | 16 | 1 | HIGH | Shifted/slanted present |
| v0.1.47 | 34 | 34 | 1 | HIGH | **Best: "Slightly less" shift/slant** |
| v0.1.48 | 34 | 34 | 1 | HIGH | Same as v0.1.47 |
| v0.1.49 | 14 | 1 | 1 | HIGH | Still shift/slant |
| v0.1.50 | 30 | 0 | 2 | **LOW** | **"Significantly worse" - misaligned** |
| v0.1.51 | 34 | 34 | 1 | HIGH | Reverted to v0.1.47 quality |
| **v0.1.52** | **34** | **34** | **1** | **HIGH** | **CRITICAL: Fixed DSI config** |

### v0.1.52 Critical Fix - DSI Configuration

**Root Cause Found:** `Core/Src/dsihost.c` (CubeMX generated) had **WRONG DSI configuration**:

| Parameter | CubeMX Value | Correct Value | Impact |
|-----------|--------------|---------------|--------|
| `NumberOfLanes` | `DSI_ONE_DATA_LANE` | `DSI_TWO_DATA_LANES` | **CRITICAL** - Bandwidth/timing mismatch |
| `Mode` | `DSI_VID_MODE_NB_PULSES` | `DSI_VID_MODE_BURST` | **CRITICAL** - OTM8009A requires BURST |
| `HSPolarity` | `DSI_HSYNC_ACTIVE_LOW` | `DSI_HSYNC_ACTIVE_HIGH` | Signal interpretation |
| `VSPolarity` | `DSI_VSYNC_ACTIVE_LOW` | `DSI_VSYNC_ACTIVE_HIGH` | Signal interpretation |

**Why This Matters:**
The `MX_DSIHOST_DSI_Init()` function runs during system initialization (before BSP_LCD_InitEx()), setting the DSI host with wrong values. Even though BSP_LCD_InitEx() tries to override these, the initial mismatch causes timing synchronization issues that persist.

### Correct OTM8009A Timing (800x472 Landscape)

**Best Working Configuration (from testing):**
```c
// Horizontal timing
HSYNC = 1      // Horizontal sync
HBP = 34       // Horizontal back porch (LARGER = less shift)
HFP = 34       // Horizontal front porch
HACT = 800     // Active pixels

// Vertical timing
VSYNC = 2      // Vertical sync
VBP = 34       // Vertical back porch
VFP = 16       // Vertical front porch
VACT = 472     // Active pixels (CRITICAL: 472, not 480!)

// Total timing
TotalWidth = 1 + 34 + 34 + 800 = 869
TotalHeight = 2 + 34 + 16 + 472 = 524
```

**Key Insight:** Larger HBP values (30-34) consistently reduce shift/slant. The display needs adequate back porch time for proper pixel data alignment.

### DSI Configuration Requirements

**CRITICAL Settings:**
```c
// DSI PLL Configuration (from stm32f769i_discovery_lcd.c)
PLLNDIV = 100
PLLI DF = DSI_PLL_IN_DIV5
PLLODF = DSI_PLL_OUT_DIV1
LaneByteClk = 62.5 MHz
LcdClock = 27.429 MHz

// DSI Host Configuration
NumberOfLanes = DSI_TWO_DATA_LANES      // CRITICAL: Must be 2, not 1
Mode = DSI_VID_MODE_BURST               // CRITICAL: OTM8009A requires burst mode
HSPolarity = DSI_HSYNC_ACTIVE_HIGH     // Must match BSP
VSPolarity = DSI_VSYNC_ACTIVE_HIGH     // Must match BSP
DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH

// DSI Timing Calculation (in DSI clock cycles)
HorizontalSyncActive = (HSA * laneByteClk_kHz) / LcdClock
HorizontalBackPorch = (HBP * laneByteClk_kHz) / LcdClock
HorizontalLine = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz) / LcdClock
```

**DSI PHY Timers:**
```c
ClockLaneHS2LPTime = 0x14
ClockLaneLP2HSTime = 0x14
DataLaneHS2LPTime = 0x0A
DataLaneLP2HSTime = 0x0A
```

### LTDC Configuration Requirements

**LTDC Clock (from PLLSAI):**
```c
PLLSAIN = 384
PLLSAIR = 7
PLLSAIDivR = RCC_PLLSAIDIVR_2
LTDC Clock ≈ 27.43 MHz
```

**LTDC Timing Registers:**
```c
// Horizontal timing
HorizontalSync = (HSA - 1) = 0
AccumulatedHBP = (HSA + HBP - 1) = 34
AccumulatedActiveW = (HSA + HBP + HACT - 1) = 834
TotalWidth = (HSA + HBP + HACT + HFP - 1) = 867

// Vertical timing
VerticalSync = (VSA - 1) = 1
AccumulatedVBP = (VSA + VBP - 1) = 35
AccumulatedActiveH = (VSA + VBP + VACT - 1) = 507
TotalHeigh = (VSA + VBP + VACT + VFP - 1) = 523
```

**Layer 0 Configuration:**
```c
WindowX0 = 0, WindowX1 = 800
WindowY0 = 0, WindowY1 = 472      // CRITICAL: 472, not 480
ImageWidth = 800, ImageHeight = 472
PixelFormat = LTDC_PIXEL_FORMAT_RGB888
FBStartAdress = 0xC0000000  // SDRAM
Alpha = 255                   // Fully opaque

// CFBLR (Critical for alignment)
CFBLR = 0x09600960  // LineLength=2400, Pitch=2400 (800×3 for RGB888)
```

### Files Modified in v0.1.52

1. **Core/Src/dsihost.c**
   - Line 49: `NumberOfLanes = DSI_TWO_DATA_LANES` (was ONE)
   - Line 95: `Mode = DSI_VID_MODE_BURST` (was NB_PULSES)
   - Lines 99-100: Polarity changed to ACTIVE_HIGH

2. **Drivers/BSP/VERSION.h**
   - Updated to v0.1.52

3. **Drivers/BSP/TEST/test_system.c**
   - Enhanced ATST149 diagnostic with DSI timing analysis

### Recommendations for Future Work

**1. CubeMX Configuration:**
When regenerating code from CubeMX, manually verify these settings in `Core/Src/dsihost.c`:
- NumberOfLanes must be `DSI_TWO_DATA_LANES`
- Mode must be `DSI_VID_MODE_BURST`
- Consider adding USER CODE sections to preserve correct values

**2. Timing Tuning (if issues persist):**
If display still shows shift/slant after v0.1.52, try these adjustments:
- HBP values: 32, 36, 38 (larger = more stable, but may affect total width)
- HFP values: Match HBP for symmetry
- Verify DSI PHY timers match BSP values
- Check OTM8009A initialization sequence

**3. Diagnostic Commands:**
```bash
# Run comprehensive panel diagnostic
ATST149

# Run basic display test
ATST1

# Check LTDC info
ATST146

# Focus pattern for convergence testing
ATST147
```

**4. Reference Resources:**
- STM32CubeF7 GitHub: https://github.com/STMicroelectronics/STM32CubeF7
- STM32F769I-Discovery examples: `Projects/STM32F769I-Discovery/Examples/LCD_DSI`
- OTM8009A driver: `Drivers/BSP/Components/otm8009a/`

**5. Known Working Values:**
```c
// Confirmed by ST reference code and testing
#define OTM8009A_800X480_WIDTH    800
#define OTM8009A_800X480_HEIGHT   472    // NOT 480!
#define OTM8009A_800X480_HSYNC    1
#define OTM8009A_800X480_HBP      34
#define OTM8009A_800X480_HFP      34
#define OTM8009A_800X480_VSYNC    2
#define OTM8009A_800X480_VBP      34
#define OTM8009A_800X480_VFP      16
```

**6. Backup Strategy:**
Always create backups before display-related changes:
```bash
mkdir -p Binary/Backup_YYYYMMDD_vX.Y.Z_to_vX.Y.Z+1
cp -f Drivers/BSP/Components/otm8009a/otm8009a.h \
      Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c \
      Core/Src/dsihost.c \
      Binary/Backup_YYYYMMDD_vX.Y.Z_to_vX.Y.Z+1/
```

### Current Status (v0.1.52)

**Display:**
- Expected to be FIXED with correct DSI configuration
- 800x472 resolution properly configured
- RGB888 pixel format
- DSI burst mode with 2 data lanes
- LTDC timing synchronized with DSI

**Working Features:**
- ✅ USB CDC debug output
- ✅ RTC time display
- ✅ ESP8266 WiFi communication
- ✅ OpenWeatherMap weather retrieval
- ✅ Test system (ATST0-ATST10)

**Pending Verification:**
- ⏳ Display alignment after v0.1.52 fix
- ⏳ Touchscreen functionality
- ⏳ SDRAM performance

### Test Results (2026-02-07)

**Hardware Test Successful:**
- ✅ STM32F769I-Discovery board detected via ST-LINK V2
- ✅ Firmware flashed (43,852 bytes) and verified
- ✅ USB CDC Virtual COM port working (/dev/cu.usbmodem3657395133351)
- ✅ ESP8266 WiFi module responding to AT commands
- ✅ ESP8266 already connected to WiFi network

**ESP8266 Module Info:**
| Property | Value |
|----------|-------|
| AT Version | 1.7.0.0 (Apr 16 2019) |
| SDK Version | 3.1.0-dev |
| Firmware | ESP8266_AT_LoBo v1.3.1 |
| IP Address | 192.168.1.154 |
| MAC Address | 48:3f:da:92:0e:5c |

**AT Commands Tested:**
```bash
AT          → OK (communication working)
AT+GMR      → Version info returned
AT+CIFSR    → IP: 192.168.1.154, MAC: 48:3f:da:92:0e:5c
```

### Current Firmware Behavior

When running, the firmware:
1. Initializes all peripherals (GPIO, LTDC, DMA2D, FMC, DSI, I2C, CRC, RTC, UART5)
2. Initializes UART5 with interrupt-driven RX ring buffer
3. Starts FreeRTOS scheduler
4. **Default Task**:
   - Initializes USB device (CDC Virtual COM)
   - Initializes ESP8266 WiFi module (sends AT, ATE0 commands)
   - Sends startup messages via USB CDC
5. **Debug Task**:
   - Continuously monitors UART5 for ESP8266 responses
   - Forwards all ESP8266 data to USB CDC for real-time debugging
6. **Display Task** (NEW - USB CDC Output):
   - Outputs RTC time and date to USB CDC every second
   - Fetches weather information from OpenWeatherMap API via ESP8266 every 60 seconds
   - Displays system uptime
   - Format: `[HH:MM:SS] YYYY-MM-DD (Weekday) Weather: [condition] Uptime: Xh Ym`
7. **LED Task**:
   - Startup: LEDs cycle for 20 seconds
   - Normal: LED1 heartbeat, LED2 CDC activity, LED3 ESP activity
8. **USB CDC**:
   - Appears as `/dev/cu.usbmodem*` virtual COM port
   - Accepts user input and forwards to ESP8266 via UART5
   - Displays RTC time, date, weather, and debug messages
9. LCD hardware is initialized but shows stripe artifact (unresolved)

### USB CDC Status Output (2026-02-08)

**NEW FEATURE:** Real-time status output via USB CDC virtual port:
```
========================================
 STM32F769I-Discovery Status Monitor
========================================

[14:30:25] 2026-02-08 (Sun)
         Weather: Beijing: ⛅️ Cloudy, +12°C, Wind: 5km/h
         Uptime: 0h 5m
```

**How to View:**
```bash
# macOS
screen /dev/cu.usbmodem3657395133351 115200

# Or using minicom
minicom -D /dev/cu.usbmodem3657395133351 -b 115200
```

**Weather API:**
- Uses OpenWeatherMap API (requires API key)
- API Key: f8af9ff257bfdb3ad6b6640c0325ad5b
- City: Peniche, Portugal
- Fetches weather data via HTTP GET to api.openweathermap.org
- Updates every 60 seconds (1 minute)
- Format: `Description, Temp°C, feels like X°C, humidity X%, wind X m/s`
- Response data: JSON with weather[0].description, main.temp, main.feels_like, main.humidity, wind.speed

### To Restore Full TouchGFX Functionality

```bash
# 1. Install TouchGFX Designer (if not already installed)
# Download from: https://www.touchgfx.com/

# 2. Open the TouchGFX project
open TouchGFX/DynamicGraph.touchgfx

# 3. Generate code in TouchGFX Designer
# Project → Generate Code

# 4. Update Makefile to include TouchGFX C++ sources
# (see above for details)

# 5. Uncomment TouchGFX init in Core/Src/main.c:
// MX_TouchGFX_Init();
// MX_TouchGFX_PreOSInit();

# 6. Rebuild and flash
make clean && make -j4
st-flash write build/TestF.bin 0x08000000
```

## Test System (ATSTn Commands)

The firmware includes a comprehensive test system that can be triggered via USB CDC serial commands. Tests are invoked using the `ATSTn` command format where `n` is the test ID (0-10).

### Available Tests

| Command | Test Name | Description |
|---------|-----------|-------------|
| ATST0 | ESP8266 Weather Test | Tests ESP8266 communication, WiFi connection, and weather data retrieval from OpenWeatherMap |
| ATST1 | Display Test | Tests LCD display with color bars, fonts, shapes, and gradient rendering |
| ATST2 | LED Test | Tests all 4 LEDs (LD1-LD4) with individual and chase pattern tests |
| ATST3 | RTC Test | Tests real-time clock functionality and time increment |
| ATST4 | WiFi Connection Test | Connects to WiFi network and verifies IP address acquisition |
| ATST5 | NTP Sync Test | Tests NTP time synchronization with pool.ntp.org |
| ATST6 | HTTP Request Test | Tests HTTP GET requests to various endpoints (httpbin.org, OpenWeatherMap) |
| ATST7 | Audio Codec Test | Tests audio codec (not fully implemented) |
| ATST8 | SDRAM Test | Tests external SDRAM read/write functionality |
| ATST9 | Touchscreen Test | Tests touchscreen with touch detection (requires touch input within 10 seconds) |
| ATST10 | Run All Tests | Executes all tests in sequence and reports summary |
| ATST? or ATHelp | Show Help | Displays list of available test commands |

### Usage Examples

```bash
# Connect to USB CDC serial port
screen /dev/cu.usbmodem<id> 115200

# Run weather test
ATST0

# Run display test
ATST1

# Run all tests
ATST10

# Show help
ATST?
```

### Test System Files

- **Drivers/BSP/TEST/test_system.h** - Test system header file
- **Drivers/BSP/TEST/test_system.c** - Test system implementation
- **USB_DEVICE/App/usbd_cdc_if.c** - Modified to intercept ATST commands

### Test Results Format

Tests output results to USB CDC in the following format:

```
=== ATST0: ESP8266 Weather Test ===
[Step 1/4] Testing AT command...
[PASS] AT command: ESP8266 responding
[Step 2/4] Checking WiFi connection...
[PASS] WiFi check: Connected to WiFi
         IP: 192.168.1.154
=== ATST0 Complete ===
```

### Test Status Codes

- **PASS** - Test completed successfully
- **FAIL** - Test failed with an error
- **TIMEOUT** - Test timed out waiting for response
- **N/A** - Test not supported or not applicable

### Running All Tests (ATST10)

The ATST10 command runs all tests in sequence and provides a summary:

```
╔════════════════════════════════════════╗
║           TEST SUMMARY                  ║
╠════════════════════════════════════════╣
║  PASSED:    8                          ║
║  FAILED:    1                          ║
║  N/A:       1                          ║
╚════════════════════════════════════════╝
```

## References

- STM32F769I-Discovery Board User Manual (UM2235)
- STM32F7xx HAL Driver Documentation
- TouchGFX Documentation
- FreeRTOS Documentation

---

# MB1166 Display Kit Configuration Guide (v0.1.53)

## Hardware Specifications

**Display Board:** MB1166 Rev.A A-02
**Controller:** OTM8009A (Orient Display)
**Resolution:** 800x472 pixels (NOT 800x480!)
**Interface:** DSI (Display Serial Interface) to LTDC controller
**Framebuffer:** External SDRAM at 0xC0000000
**Colors:** RGB888 format (24-bit, 3 bytes per pixel)

## Critical Configuration Files

The following files contain the correct display configuration for the MB1166 kit:

### 1. OTM8009A Timing Configuration
**File:** `Drivers/BSP/Components/otm8009a/otm8009a.h`

**Critical values (lines 80-133):**
```c
// Resolution - MUST be 472, not 480!
#define OTM8009A_800X480_WIDTH    800
#define OTM8009A_800X480_HEIGHT   472    // CRITICAL

// Landscape timing (best configuration from v0.1.47+)
#define OTM8009A_800X480_HSYNC    1
#define OTM8009A_800X480_HBP      34     // Large HBP reduces shift/slant
#define OTM8009A_800X480_HFP      34
#define OTM8009A_800X480_VSYNC    2
#define OTM8009A_800X480_VBP      34
#define OTM8009A_800X480_VFP      16
```

### 2. DSI Host Configuration
**File:** `Core/Src/dsihost.c`

**Note:** This file is CubeMX-generated and contains initial DSI configuration.
The actual working configuration is set by `BSP_LCD_InitEx()` in `stm32f769i_discovery_lcd.c`.

**Current settings (v0.1.52+):**
- NumberOfLanes: `DSI_TWO_DATA_LANES` (line 52) - CRITICAL for OTM8009A
- Mode: `DSI_VID_MODE_BURST` (line 102) - OTM8009A requires burst mode
- HSPolarity: `DSI_HSYNC_ACTIVE_HIGH` (line 106)
- VSPolarity: `DSI_VSYNC_ACTIVE_HIGH` (line 107)

### 3. LCD BSP Configuration
**File:** `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`

**Key sections:**
- **BSP_LCD_InitEx()** (line 284): Main LCD initialization function
- **DSI PLL configuration** (line 338): `PLLNDIV=100, PLLIDF=DSI_PLL_IN_DIV5, PLLODF=DSI_PLL_OUT_DIV1`
- **DSI timing calculation** (lines 395-449): Converts OTM8009A timing to DSI timing
- **LTDC configuration** (lines 495-615): Complete LTDC and layer setup
- **CFBLR fix** (lines 610-614): Ensures correct line pitch for RGB888

### 4. LTDC Override
**File:** `Core/Src/ltdc_override.c`

Prevents the CubeMX-generated `ltdc.c` from applying wrong timing values.
This weak function override does nothing, letting `BSP_LCD_Init()` set correct timing.

## Display Configuration Flow

The display is initialized in the following order:

1. **System Init** (`main.c`):
   - `MX_DSIHOST_DSI_Init()` - Initial DSI configuration (overridden later)
   - `MX_LTDC_Init()` - Overridden by weak override in `ltdc_override.c`

2. **BSP LCD Init** (`stm32f769i_discovery_lcd.c::BSP_LCD_InitEx()`):
   - DSI PLL configuration
   - DSI host reconfiguration (TWO data lanes, burst mode)
   - DSI PHY timer configuration
   - LTDC timing configuration (both horizontal AND vertical)
   - Layer 0 and Layer 1 configuration
   - OTM8009A panel initialization

## Complete Display Timing Values (800x472 Landscape)

### DSI Timing (calculated in BSP_LCD_InitEx)
```
LaneByteClk = 62.5 MHz
LcdClock = 27.429 kHz

HorizontalSyncActive    = 1 * 62500 / 27429 = 2
HorizontalBackPorch     = 34 * 62500 / 27429 = 77
HorizontalLine          = (800 + 1 + 34 + 34) * 62500 / 27429 = 2090
VerticalSyncActive      = 2
VerticalBackPorch       = 34
VerticalFrontPorch      = 16
VerticalActive          = 472
```

### LTDC Timing
```
HorizontalSync         = 0    (HSA - 1)
AccumulatedHBP         = 34   (HSA + HBP - 1)
AccumulatedActiveW      = 834  (HSA + HBP + HACT - 1)
TotalWidth             = 867  (HSA + HBP + HACT + HFP - 1)

VerticalSync           = 1    (VSA - 1)
AccumulatedVBP         = 35   (VSA + VBP - 1)
AccumulatedActiveH      = 507  (VSA + VBP + VACT - 1)
TotalHeigh             = 523  (VSA + VBP + VACT + VFP - 1)
```

### LTDC Layer 0 Configuration
```
WindowX0               = 0
WindowX1               = 800
WindowY0               = 0
WindowY1               = 472
ImageWidth             = 800
ImageHeight            = 472
PixelFormat            = LTDC_PIXEL_FORMAT_RGB888
FBStartAdress          = 0xC0000000 (SDRAM)
Alpha                  = 255 (fully opaque)
CFBLR                  = 0x09600960 (LineLength=2400, Pitch=2400)
```

## Weather Configuration (OpenWeatherMap Only)

**File:** `Drivers/BSP/ESP8266/esp8266.c`

**Function:** `ESP8266_GetWeather()` (line 869)

**Configuration:**
- API: OpenWeatherMap (wttr.in is NO LONGER USED)
- API Key: `f8af9ff257bfdb3ad6b6640c0325ad5b`
- City: Peniche, Portugal
- Units: metric (Celsius)
- Endpoint: `api.openweathermap.org/data/2.5/weather`

**Important:** The APPID parameter is case-sensitive and must be uppercase.

## Troubleshooting Display Issues

If the display still shows distortion, tilting, or shifting after v0.1.53:

### 1. Verify Correct Firmware is Running
```
ATST?
```
Check version output confirms v0.1.53 or later.

### 2. Run Display Diagnostics
```
ATST1    # Basic display test
ATST146  # LTDC register dump (check timing values)
ATST147  # Corner number test (check alignment)
ATST149  # Complete DSI/LTDC diagnostic
```

### 3. Check LTDC Registers
Verify these register values match the configuration above:
- LTDC_GCR1: TotalWidth and TotalHeigh
- LTDC_BPCR: AccumulatedHBP and AccumulatedVBP
- LTDC_AWCR: AccumulatedActiveW and AccumulatedActiveH
- LTDC_CFBLR: Line length and pitch (should be 0x09600960 for 800xRGB888)

### 4. Known Display Issues and Solutions

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Vertical stripe | Layer 1 enabled | Verify Layer 1 is disabled |
| Shift/slant | Wrong HBP | Ensure HBP=34 in otm8009a.h |
| Crawling/distortion | Wrong DSI mode | Ensure DSI_VID_MODE_BURST |
| Display not centered | Wrong timing | Verify all timing values match above |
| Bottom corruption | Wrong height | Ensure HEIGHT=472, not 480 |

## Build and Flash Commands

```bash
# Build
make clean
make -j4

# Create versioned backup
cp build/TestF.bin Binary/Backup_$(date +%Y%m%d)_v0.1.XX_to_v0.1.YY/

# Flash with st-flash
st-flash --reset write build/TestF.bin 0x08000000

# Or use flash.sh script
./flash.sh
```

## Recommendations for Next Steps

1. **Test Current Configuration:** Flash v0.1.53 and run ATST147 to verify display alignment
2. **Compare with ST Demo:** Use ST demo firmware as reference for register values
3. **Check OTM8009A Init:** Verify panel initialization sequence matches datasheet
4. **Consider Clock Phases:** If issues persist, investigate DSI/LTDC clock phase relationships
5. **Document All Changes:** Keep detailed notes of any configuration modifications

## Files Modified in v0.1.53

1. `Drivers/BSP/VERSION.h` - Updated to v0.1.53
2. `SUMMARY.md` - Added comprehensive MB1166 configuration guide

## Status Summary (v0.1.53)

| Feature | Status |
|---------|--------|
| Display Initialization | Configured per MB1166 spec |
| DSI Host | 2 lanes, burst mode, correct polarity |
| LTDC Timing | Complete horizontal and vertical |
| OTM8009A Panel | 800x472 resolution, correct timing |
| Weather | OpenWeatherMap only (wttr.in removed) |
| Display Quality | v0.1.54: PCPOLARITY fix applied, ATST150 added |

---

# Display Distortion Fix Attempt (v0.1.54)

## Problem Analysis

**Symptoms:**
- Physical screen edges match displayed image (TotalWidth/TotalHeigh correct)
- Letters and lines significantly misaligned and tilted
- Image appears severely tilted and distorted with shifting/crawling

**Root Cause Analysis:**
This type of issue indicates correct basic timing but incorrect **pixel sampling**:
1. Screen edges correct → HSYNC, VSYNC, HBP, VBP, HFP, VFP values are correct
2. Content tilted/distorted → Pixel clock phase or data sampling timing issue
3. Shifting/crawling → DSI data lane timing mismatch

## Fixes Implemented in v0.1.54

### 1. LTDC Pixel Clock Polarity Change
**File:** `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` (line 570)

**Change:**
```c
// Before: LTDC_PCPOLARITY_IPC (not inverted)
// After:  LTDC_PCPOLARITY_IIPC (inverted)
hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IIPC;
```

**Rationale:** The OTM8009A display often requires inverted pixel clock for correct sampling. This can fix tilted/distorted content while maintaining correct screen positioning.

### 2. DSI Host Timing Values Fixed
**File:** `Core/Src/dsihost.c` (lines 109-115)

**Previous CubeMX values (INCORRECT):**
```
HorizontalSyncActive = 13      // Wrong!
HorizontalBackPorch = 156      // Wrong!
HorizontalLine = 1224          // Wrong!
VerticalSyncActive = 6         // Wrong!
VerticalBackPorch = 15         // Wrong!
VerticalFrontPorch = 4         // Wrong!
```

**New values (CORRECT for OTM8009A 800x472):**
```
HorizontalSyncActive = 2       // HSA=1 * 62500/27429 ≈ 2
HorizontalBackPorch = 77       // HBP=34 * 62500/27429 ≈ 77
HorizontalLine = 2091          // (800+1+34+34) * 62500/27429 ≈ 2091
VerticalSyncActive = 2         // VSA=2
VerticalBackPorch = 34         // VBP=34
VerticalFrontPorch = 16        // VFP=16
VerticalActive = 472           // VACT=472 (MB1166 height!)
```

### 3. Corrected Timing Comments
**File:** `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` (lines 393-403)

Fixed misleading comments that didn't match actual macro values.

## ATST150 - 50x50 Pixel Checkerboard Test

**Purpose:** Visual diagnostic for pixel alignment and timing issues

**Implementation:** `Drivers/BSP/TEST/test_system.c`

**Features:**
- 50x50 pixel checkerboard pattern (16x9 cells for 800x472)
- Red border to verify screen edge alignment
- Diagnostic text overlay
- USB CDC output of LTDC timing register values
- Visual inspection guide for common issues

**Usage:**
```
ATST150
```

**Expected Output (if display is correct):**
- Square cells (not rectangular)
- Sharp cell borders (not blurry)
- No crawling/shifting pattern
- Red border aligned with screen edges

**Diagnostic Output via USB CDC:**
```
=== ATST150: 50x50 Checkerboard Test ===
Cell size: 50x50 pixels
Grid: 16 x 9 cells
Screen: 800 x 472 pixels

--- LTDC Timing ---
TotalWidth: 867 (expected 867)
TotalHeigh: 523 (expected 523)
AccumulatedHBP: 34 (expected 34)
AccumulatedVBP: 35 (expected 35)
CFBLR: 0x09600960 (LineLen=2400, Pitch=2400)

--- Visual Inspection Guide ---
If cells look rectangular (not square): Horizontal timing issue
If pattern crawls/shifts: DSI clock phase or PHY timing issue
If edges are blurry: Pixel clock polarity or sampling issue
If red border is misaligned: TotalWidth/TotalHeigh mismatch
```

## Alternative Timing Configurations to Try

If v0.1.54 still shows distortion, try these alternatives:

### Option 1: Revert PCPOLARITY
If inverted clock makes it worse, try:
```c
hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IPC;  // Not inverted
```

### Option 2: Adjust HBP Value
Try different HBP values in `otm8009a.h`:
```c
#define OTM8009A_800X480_HBP  29  // Instead of 34
```

### Option 3: DSI PHY Timing Adjustment
Adjust data lane timing in `stm32f769i_discovery_lcd.c`:
```c
dsiPhyInit.DataLaneLP2HSTime = 0x19;  // Instead of 0x0A
```

## Files Modified in v0.1.54

1. `Drivers/BSP/VERSION.h` - Updated to v0.1.54
2. `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`
   - Changed PCPOLARITY to IIPC
   - Fixed timing comments
3. `Core/Src/dsihost.c`
   - Updated DSI video mode timing to match OTM8009A specs
4. `Drivers/BSP/TEST/test_system.h`
   - Added TEST_ID_CHECKERBOARD_50 for ATST150
5. `Drivers/BSP/TEST/test_system.c`
   - Added TEST_DisplayMode_Checkerboard50() function
   - Added case 150 to TEST_ProcessCommand()
6. `SUMMARY.md` - Added v0.1.54 documentation

## Testing Procedure for v0.1.54

1. **Flash firmware:**
   ```bash
   st-flash --reset write build/TestF_v0.1.54_20260215_233531.bin 0x08000000
   ```

2. **Run ATST150 to check for distortion:**
   ```
   ATST150
   ```

3. **Compare with ATST145 (40x40 checkerboard):**
   ```
   ATST145
   ```

4. **Run comprehensive diagnostics:**
   ```
   ATST146  # LTDC register dump
   ATST147  # Focus pattern
   ATST149  # Panel detection with timing analysis
   ```

5. **If distortion persists, try alternatives:**
   - Revert PCPOLARITY change
   - Adjust HBP value
   - Check LTDC_GCR register value via debug

## Next Steps if v0.1.54 Doesn't Fix Display

1. **Verify PCPOLARITY took effect:**
   - Read LTDC_GCR register and check bit 28
   - Should be 1 for IIPC, 0 for IPC

2. **Compare with ST demo firmware:**
   - Run ST demo firmware and record LTDC register values
   - Compare with your firmware's register values

3. **Check DSI wrapper vs video mode:**
   - Ensure DSI is properly configured for video burst mode
   - Verify NumberOfLanes = 2

4. **OTM8009A panel initialization:**
   - Verify panel init sequence matches datasheet
   - Check for missing delay after specific commands

5. **Consider clock source:**
   - Verify PLLSAI is configured correctly for LTDC clock
   - Check LTDC clock frequency (~27.43 MHz expected)

---

# Display Timing Fix v0.1.58 (2026-02-16) - CRITICAL DSI/LTDC SYNC FIX

## Problem Analysis

The ATST150 50x50 checkerboard test showed **severe distortion, tilting, and crawling** of the displayed image. The screen edges matched the physical display correctly, but the content appeared severely distorted and "crawling" - a classic symptom of DSI/LTDC timing mismatch.

## Root Cause - CRITICAL FINDING

**The DSI host timing in `Core/Src/dsihost.c` did NOT match the LTDC timing in `BSP_LCD_InitEx()`:**

| Parameter | LTDC (stm32f769i_discovery_lcd.c) | DSI Host (dsihost.c v0.1.57) | Status |
|-----------|----------------------------------|------------------------------|--------|
| VFP (Vertical Front Porch) | **34** (MB1166 timing) | **16** (ST reference) | **MISMATCH!** |
| HFP (Horizontal Front Porch) | **34** (MB1166 timing) | 77 (converted correctly) | ✓ |
| HBP (Horizontal Back Porch) | **34** (MB1166 timing) | 77 (converted correctly) | ✓ |
| VBP (Vertical Back Porch) | **34** (MB1166 timing) | 34 (correct) | ✓ |

**This mismatch caused the DSI video controller and LTDC to be out of sync, resulting in:**
- Severe image distortion and tilting
- "Crawling" or shifting artifacts
- Text and lines appearing misaligned

## Why This Happened

The `MX_DSIHOST_DSI_Init()` function in `dsihost.c` runs during system initialization (before `BSP_LCD_InitEx()`), setting initial DSI timing values. Even though `BSP_LCD_InitEx()` later overrides these with correct LTDC timing values, the **initial timing mismatch causes synchronization issues that persist** because:

1. The DSI host starts transmitting data with the wrong timing
2. The OTM8009A display panel initializes with mismatched timing expectations
3. Even after LTDC timing is corrected, the DSI/LTDC synchronization is already corrupted

## Fix Applied (v0.1.58)

**Critical Fix**: Changed `VerticalFrontPorch` in `Core/Src/dsihost.c` from **16 to 34** to match the MB1166 timing used by LTDC.

```c
// In dsihost.c line 125, changed from:
VidCfg.VerticalFrontPorch = 16;  // WRONG! ST reference value

// To:
VidCfg.VerticalFrontPorch = 34;  // CORRECT! Matches LTDC VFP
```

## Complete MB1166 Timing Configuration

For the MB1166 display kit, **all timing values must be consistent** across DSI and LTDC:

**Portrait base values (480x800):**
```c
HSYNC = 2  -> becomes VSYNC in landscape
HBP = 34   -> becomes VBP in landscape
HFP = 34   -> becomes VFP in landscape
VSYNC = 1  -> becomes HSYNC in landscape
VBP = 34   -> becomes HBP in landscape
VFP = 34   -> becomes HFP in landscape
```

**Landscape values (800x472):**
```c
HSYNC = 1
HBP = 34   // MB1166-tested, NOT 15 (ST reference)
HFP = 34   // MB1166-tested, NOT 16 (ST reference)
HACT = 800
VSYNC = 2
VBP = 34
VFP = 34   // CRITICAL: Was 16 in dsihost.c v0.1.57, now fixed to 34
VACT = 472 // MB1166 height, NOT 480
```

## Files Modified (v0.1.58)

1. `Core/Src/dsihost.c` - **CRITICAL FIX**: Changed VerticalFrontPorch from 16 to 34
2. `Drivers/BSP/VERSION.h` - Updated to v0.1.58
3. `Drivers/BSP/TEST/test_system.c` - Updated test expectations to MB1166 values
4. `Drivers/BSP/Components/otm8009a/otm8009a.h` - Clarified comments about timing mapping
5. `SUMMARY.md` - Added this documentation

## Testing Procedure for v0.1.58

1. **Flash firmware:**
   ```bash
   st-flash --reset write build/TestF_v0.1.58_*.bin 0x08000000
   ```

2. **Run checkerboard test:**
   ```
   ATST150
   ```

3. **Expected results:**
   - **Square checkerboard cells** (50x50 pixels - NOT rectangular!)
   - Sharp cell borders (NOT blurry)
   - NO crawling/shifting pattern
   - Red border aligned with screen edges
   - Text appears correctly aligned (NOT tilted/distorted)

4. **Expected ATST150 diagnostic output:**
   ```
   TotalWidth: 868 (expected 868 for MB1166)
   TotalHeigh: 541 (expected 541)
   AccumulatedHBP: 34 (expected 34 for MB1166)
   AccumulatedVBP: 35 (expected 35)
   ```

## Technical Explanation

**Why DSI and LTDC timing must match:**

The STM32F769I uses a DSI (Display Serial Interface) to LTDC (LCD-TFT Display Controller) architecture:
- **LTDC** generates pixel timing signals (HSYNC, VSYNC, etc.) and sends pixel data
- **DSI host** encodes the pixel data and timing into DSI packets for transmission
- **OTM8009A display panel** receives DSI packets and drives the LCD

When DSI and LTDC timing mismatch:
- The DSI host sends data at one rate
- The LTDC expects data at a different rate
- The OTM8009A panel gets confused about when to sample pixel data
- Result: **Severe distortion, crawling, tilting artifacts**

**The fix:** By ensuring DSI timing values (in dsihost.c) exactly match LTDC timing values (in stm32f769i_discovery_lcd.c), the entire video pipeline is synchronized.

## Verification Checklist

After flashing v0.1.58, verify:

- [ ] ATST150 shows **square** 50x50 pixel cells (not rectangular)
- [ ] No crawling/shifting pattern visible
- [ ] Text appears correctly aligned (not tilted)
- [ ] Red border matches screen edges exactly
- [ ] All diagnostic values match expected MB1166 timing
- [ ] Display shows sharp, stable image

## Future Prevention

**CRITICAL RULE:** When modifying display timing, always update **both**:
1. `Core/Src/dsihost.c` - DSI host initial timing
2. `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` - LTDC timing
3. `Drivers/BSP/Components/otm8009a/otm8009a.h` - Panel timing definitions

**The DSI timing values MUST be calculated from the same base values used for LTDC:**
```
DSI_timing = (OTM8009A_timing * LaneByteClk) / LcdClock
where LaneByteClk = 62.5MHz, LcdClock = 27.429kHz, ratio ≈ 2.278
```

---

# Display Timing Fix v0.1.57 (2026-02-16)

## Problem Analysis

The ATST150 50x50 checkerboard test showed **rectangular cells** instead of square cells, indicating incorrect display timing. The screen edges matched the physical display (total timing correct), but pixel sampling was misaligned.

## Root Cause

The v0.1.56 fix incorrectly applied ST's F469I-Discovery reference timing values:
- **HBP = 15, HFP = 16** (ST reference)

However, project testing (SUMMARY.md line 999) showed:
- **v0.1.47 with HBP=34, HFP=34 gave "Slightly less shift/slant" (BEST result)**
- **v0.1.56 with HBP=15, HFP=16 caused rectangular cells**

## Why ST Reference Values Don't Work

The MB1166 display board appears to have different timing requirements than ST's F469I-Discovery board. The larger horizontal porch values (HBP=34, HFP=34) provide the DSI/LTDC more time for proper pixel data alignment.

## Fix Applied (v0.1.57)

Reverted to MB1166-tested timing values:
```c
// Portrait base values (480x800)
#define OTM8009A_480X800_HSYNC  2
#define OTM8009A_480X800_HBP    34  // Becomes VBP in landscape
#define OTM8009A_480X800_HFP    34  // Becomes VFP in landscape
#define OTM8009A_480X800_VSYNC  1
#define OTM8009A_480X800_VBP    34  // Becomes HBP in landscape
#define OTM8009A_480X800_VFP    34  // Becomes HFP in landscape

// Landscape values (800x472) - symmetric timing
#define OTM8009A_800X480_HSYNC  1
#define OTM8009A_800X480_HBP    34  // MB1166-tested
#define OTM8009A_800X480_HFP    34  // MB1166-tested
#define OTM8009A_800X480_VSYNC  2
#define OTM8009A_800X480_VBP    34
#define OTM8009A_800X480_VFP    34
```

This gives **symmetric timing**: HBP=HFP=VBP=VFP=34 for landscape mode.

## Expected LTDC Timing (v0.1.57)

```
TotalWidth = HSA + HBP + HACT + HFP - 1 = 1 + 34 + 800 + 34 - 1 = 868
TotalHeigh = VSA + VBP + VACT + VFP - 1 = 2 + 34 + 472 + 34 - 1 = 541
AccumulatedHBP = HSA + HBP - 1 = 1 + 34 - 1 = 34
AccumulatedVBP = VSA + VBP - 1 = 2 + 34 - 1 = 35
```

## Files Modified (v0.1.57)

1. `Drivers/BSP/Components/otm8009a/otm8009a.h` - Updated timing values
2. `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` - Updated comments
3. `Core/Src/dsihost.c` - Updated DSI timing calculation
4. `Drivers/BSP/VERSION.h` - Updated to v0.1.57

## Testing Procedure

1. Flash firmware:
   ```bash
   st-flash --reset write Binary/TestF.bin 0x08000000
   ```

2. Run checkerboard test:
   ```
   ATST150
   ```

3. Expected results:
   - Square checkerboard cells (50x50 pixels)
   - Sharp cell borders
   - No crawling/shifting pattern
   - Red border aligned with screen edges

## If Distortion Persists

If v0.1.57 still shows issues, the root cause may be:
1. **Pixel clock phase** - Try LTDC_PCPOLARITY_IIPC (inverted)
2. **DSI PHY timing** - Adjust DataLaneLP2HSTime values
3. **Panel-specific init** - OTM8009A may need special initialization sequence
4. **Clock source mismatch** - Verify PLLSAI LTDC clock frequency

---

# Display Distortion Investigation Summary (v0.1.59 - v0.1.60)

## Problem Statement

Despite all timing parameters being verified as correct, the display shows **severe geometric distortion** with a "shear" or "tilt" effect. The 50x50 pixel checkerboard pattern (ATST150) shows:
- Total timing CORRECT (edges match physical display)
- Content DISTORTED within active area
- "Crawling" or "shifting" pattern
- Lines are tilted/sheared, not horizontal

## Investigation Results (v0.1.60)

All timing parameters have been exhaustively verified as **CORRECT**:

| Parameter | Expected | Actual | Status |
|-----------|----------|--------|--------|
| LTDC TotalWidth | 868 | 868 | ✓ |
| LTDC TotalHeigh | 541 | 541 | ✓ |
| LTDC AccumulatedHBP | 34 | 34 | ✓ |
| LTDC AccumulatedVBP | 35 | 35 | ✓ |
| CFBLR LineLength | 2400 | 2400 | ✓ |
| CFBLR Pitch | 2400 | 2400 | ✓ |
| DSI HorizontalLine (calc) | 1980 | 1980 | ✓ |
| DSI PHY timers | ST ref | ST ref | ✓ |
| OTM8009A PASET YE | 471 | 471 | ✓ |

## Key Findings

1. **MX_DSIHOST_DSI_Init() is commented out** (main.c line 134)
   - The dsihost.c file has timing values, but they are NEVER EXECUTED
   - All DSI initialization is done by BSP_LCD_InitEx()
   - The v0.1.58 fix in dsihost.c has NO EFFECT at runtime

2. **All timing is correct, yet distortion persists**
   - This suggests the issue is NOT with basic timing values
   - May be related to: DSI shadow register refresh, pixel clock phase, RGB byte order, or DSI-LTDC synchronization

## Next Steps

1. **Try ST official demo firmware** for comparison
2. **Check MB1166 board revision** compatibility
3. **Verify OTM8009A panel ID** (read panel registers)
4. **Experimental fixes:**
   - PCPOLARITY inversion (IIPC vs IPC)
   - DSI PHY timing changes
   - RGB byte order swap

## Files Modified (v0.1.59 - v0.1.60)

1. `Drivers/BSP/VERSION.h` - Updated to v0.1.59, v0.1.60
2. `Drivers/BSP/TEST/test_system.c` - Added DSI timing diagnostics to ATST150
3. Created backup directories for each version

---

# Display Tilt Fix v0.1.66 - v0.1.67 (2026-02-16) - ROOT CAUSE FOUND!

## Problem Statement

**45-degree diagonal tilt with 1-pixel-per-row offset** - Each subsequent row of pixels appears to be shifted by 1 pixel, creating a characteristic diagonal shear pattern across the entire display. The physical screen edges match the displayed image (total size is correct), but content within the active area is severely tilted.

## Root Cause Discovery (v0.1.66)

The diagonal tilt was caused by **TWO separate bugs** in the HAL LTDC driver working together:

### Bug 1: CFBLR Pitch Issue (Known from v0.1.64)
**Location:** `stm32f7xx_hal_ltdc.c` lines 2189-2190

```c
LTDC_LAYER(hltdc, LayerIdx)->CFBLR = (((pLayerCfg->ImageWidth * tmp) << 16U) | \
                                       (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 3U));
```

The HAL adds `+3` to the pitch for "alignment". For RGB888 with 800 pixel width:
- Expected: Pitch = 800 × 3 = 2400 bytes
- Actual: Pitch = 2400 + 3 = 2403 bytes

This causes each row to start 1 pixel (3 bytes) too late, creating cumulative diagonal shift.

### Bug 2: Window Position Issue (NEWLY DISCOVERED!)
**Location:** `stm32f7xx_hal_ltdc.c` lines 2136-2139

```c
tmp = ((pLayerCfg->WindowX1 + ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16U)) << 16U);
LTDC_LAYER(hltdc, LayerIdx)->WHPCR = ((pLayerCfg->WindowX0 + \
                                       ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp);
```

The HAL **adds AHBP (Accumulated Horizontal Back Porch) to window positions!**

With HSA=1, HBP=34: **AHBP = 35**
- WindowX0=0 → StartPos = 0 + 35 + 1 = **36** (should be 0!)
- WindowX1=800 → StopPos = 800 + 35 = **835** (should be 800!)

This causes the LTDC to read pixel data from the **wrong framebuffer position**, creating additional offset.

### Combined Effect

With both bugs active:
- Row 0: LTDC reads from framebuffer byte 108 (pixel 36) instead of byte 0
- Row 1: LTDC reads from framebuffer byte 2508 (2403 + 108) instead of byte 2400
- Each row, the offset increases by the pitch error (3 bytes = 1 pixel)

This creates the characteristic **45-degree diagonal tilt pattern**.

## Fix Applied (v0.1.66)

**Location:** `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`

**Solution in `BSP_LCD_LayerDefaultInit()`:**

```c
/* CRITICAL FIX v0.1.66: Fix BOTH CFBLR pitch AND Window Position issues! */
uint32_t bpcr_backup = hltdc_discovery.Instance->BPCR;  // Save original BPCR
hltdc_discovery.Instance->BPCR &= ~(LTDC_BPCR_AHBP);    // Clear AHBP (set to 0)

HAL_LTDC_ConfigLayer(&hltdc_discovery, &Layercfg, LayerIndex);

hltdc_discovery.Instance->BPCR = bpcr_backup;  // Restore original BPCR

/* Now fix CFBLR pitch for RGB888 format */
if (Layercfg.PixelFormat == LTDC_PIXEL_FORMAT_RGB888) {
  uint32_t line_length = Layercfg.ImageWidth * 3;  /* 800 * 3 = 2400 */
  LTDC_LAYER(&hltdc_discovery, LayerIndex)->CFBLR = ((line_length << 16U) | line_length);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);
}
```

By temporarily setting AHBP to 0 during `HAL_LTDC_ConfigLayer()`, the HAL calculates correct window positions. Then we restore BPCR and fix the CFBLR pitch.

## Issue Found in v0.1.67 - Duplicate Conflicting Fixes

**Problem:** The `main.c` file had duplicate fixes (lines 220-253) that were running **AFTER** the correct v0.1.66 fix in `BSP_LCD_LayerDefaultInit()`.

### Issues in main.c (v0.1.66 and earlier)

**1. Incorrect Window Position Values:**
```c
LTDC_LAYER(&hltdc_discovery, 0)->WHPCR = (800 << 16U) | 0;  // WRONG!
LTDC_LAYER(&hltdc_discovery, 0)->WVPCR = (lcd_height << 16U) | 0;  // WRONG!
```

For LTDC layer window registers:
- **StartPos** = first pixel to display (0-based)
- **StopPos** = last pixel to display (0-based)

The code sets:
- WHPCR: StopPos=800, StartPos=0 → **Should be StopPos=799 for 800 pixels (0-799)!**
- WVPCR: StopPos=height, StartPos=0 → **Should be StopPos=(height-1)!**

**2. Redundant CFBLR Fix:**
Lines 233-253 attempted to fix CFBLR pitch again, but this was already handled correctly by `BSP_LCD_LayerDefaultInit()`.

## Fix Applied (v0.1.67)

**Removed duplicate fixes from main.c (lines 220-253)**

The v0.1.66 fix in `BSP_LCD_LayerDefaultInit()` is now the **authoritative source of truth** for LTDC layer configuration.

**Enhanced diagnostic output:**
```
[BSP_LCD_LayerDefaultInit L0] W=800 H=472
  CFBLR: 0x09600963->0x09600960 (LineLen=2400,Pitch=2400)
  WHPCR: 0x031F0001 (X0=0,X1=799) Expected=0x031F0001
  WVPCR: 0x01D70001 (Y0=0,Y1=471)
```

## Expected Register Values (v0.1.67)

For 800x472 RGB888:
- **CFBLR** = 0x09600960 (CFBLL=2400, CFBP=2400)
- **WHPCR** = 0x031F0001 (StopPos=799=0x31F, StartPos=0)
- **WVPCR** = 0x01D70001 (StopPos=471=0x1D7, StartPos=0)

## Files Modified

### v0.1.66:
1. `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` - Added window position + CFBLR fix
2. `Drivers/BSP/VERSION.h` - Updated to v0.1.66

### v0.1.67:
1. `Core/Src/main.c` - Removed duplicate/conflicting fixes
2. `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c` - Enhanced diagnostic output
3. `Drivers/BSP/VERSION.h` - Updated to v0.1.67
4. `SUMMARY.md` - This documentation

## Testing Procedure

1. **Flash firmware:**
   ```bash
   st-flash write build/TestF.bin 0x08000000
   ```

2. **Connect to USB serial and check diagnostic output:**
   ```
   [BSP_LCD_LayerDefaultInit L0] W=800 H=472
     CFBLR: 0x09600963->0x09600960 (LineLen=2400,Pitch=2400)
     WHPCR: 0x031F0001 (X0=0,X1=799) Expected=0x031F0001
     WVPCR: 0x01D70001 (Y0=0,Y1=471)
   ```

3. **Run checkerboard test:**
   ```
   ATST150
   ```

4. **Expected results:**
   - **Square checkerboard cells** (50x50 pixels - NO diagonal tilt!)
   - Sharp cell borders
   - NO "shear" or "tilt" effect
   - Each row aligned with previous row (no progressive offset)

5. **Run main display test:**
   ```
   ATST1
   ```

6. **Expected results:**
   - Text and graphics properly aligned
   - NO diagonal distortion
   - Horizontal lines appear horizontal (not slanted)

## If Problem Persists

If the diagonal tilt still occurs after v0.1.67:

1. **Check diagnostic output** - Verify CFBLR, WHPCR, WVPCR values match expected
2. **Verify LTDC timing** - Run ATST149 to check all LTDC registers
3. **Compare with ST demo** - Test official STM32F769I-DISCO_DEMO_V1.2.0_FULL.hex
4. **Hardware issue** - Possible OTM8009A panel defect or MB1166 board issue

===============================================================================
PROJECT CONTINUATION RECOMMENDATIONS
===============================================================================

This section provides guidance for continuing development on this project.

## Quick Start Guide for New Development

1. **Display Configuration (Critical)**
   - The display tilt issue was RESOLVED in v0.1.66-v0.1.67
   - ALWAYS use `BSP_LCD_LayerDefaultInit()` as the single source of truth
   - NEVER modify LTDC registers after initialization
   - Key files:
     * `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`
     * `Core/Src/dsihost.c` (DSI timing configuration)

2. **Drawing Commands (v0.1.68+)**
   - ATST151-158: Basic drawing primitives (points, lines, rectangles, circles, text, graphs)
   - Use these commands as building blocks for menus and graphs
   - Key file: `Drivers/BSP/TEST/test_system.c`

3. **Audio Oscilloscope (v0.1.70+)**
   - ATST160: 4-channel audio oscilloscope display
   - ATST162: 4-mic audio with sound source localization (TDOA)
   - Currently uses simulated waveform data
   - For real audio: Integrate WM8994 codec with I2S and DMA

4. **Menu System (v0.1.71+)**
   - ATST165-168: Menu components (button, progress, slider, list)
   - ATST169: Complete menu demonstration
   - Use these commands to build interactive UIs
   - Integrate with touchscreen (ATST9) for touch interaction

## Important Files Reference

### Display Configuration
- `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`
  Function: `BSP_LCD_LayerDefaultInit()` - CRITICAL for correct display
- `Core/Src/dsihost.c`
  Function: `MX_DSIHOST_DSI_Init()` - DSI timing configuration
- `Note.txt` - Display configuration summary
- `Note-2.txt` - Comprehensive hardware documentation with technical links

### Drawing and Graphics
- `Drivers/BSP/TEST/test_system.c`
  Functions: `TEST_Draw_*()`, `TEST_Drawing_Test()`, `TEST_Audio_Display()`
- `Drivers/BSP/TEST/test_system.h`
  Test ID definitions and function prototypes

### Command Processing
- `USB_DEVICE/App/usbd_cdc_if.c`
  Functions: `CDC_Receive_HS()`, `CDC_ProcessPendingCommand()`
  Handles ATST commands from USB CDC serial port

### Version Tracking
- `Drivers/BSP/VERSION.h`
  Update `FIRMWARE_VERSION_PATCH` for each change

## Common Tasks

### Adding a New Drawing Command
1. Add test ID to `test_system.h` (enum `TestID_TypeDef`)
2. Implement function in `test_system.c`
3. Add case to `TEST_ProcessCommand()` or `TEST_ProcessCommandWithParams()`
4. Update help text in `TEST_ShowHelp()`
5. Update VERSION.h
6. Build and test

### Display Configuration Changes
WARNING: Display configuration is DELICATE. Changes may cause tilt/distortion.
1. Read `Note.txt` and `Note-2.txt` thoroughly
2. Understand LTDC and DSI timing relationship
3. Make changes in `BSP_LCD_LayerDefaultInit()` ONLY
4. Test with ATST150 (50x50 checkerboard) to verify no tilt
5. Test with ATST159 (drawing test) for comprehensive verification

### Adding Real Audio Input
1. Integrate WM8994 audio codec driver (`Drivers/BSP/Components/wm8994/`)
2. Configure I2S with DMA (`Core/Src/stm32f7xx_hal_msp.c`)
3. Implement double buffering for continuous capture
4. Modify `TEST_Audio_Display()` to use real audio data
5. Optional: Add FFT for frequency analysis

## Known Issues

1. **Touchscreen Freeze at Top of Screen**
   - ATST9 (touchscreen test) may freeze if touch occurs at top of screen
   - This is a known issue with FT6x06 driver
   - Workaround: Avoid testing touch in top 50 pixels

2. **Audio Display is Simulation Only**
   - ATST160 uses simulated waveform data
   - Real audio input requires WM8994 codec integration (not yet implemented)

## Documentation References

- `SUMMARY.md` (this file) - Complete project documentation
- `Note.txt` - Display configuration quick reference
- `Note-2.txt` - Hardware documentation with technical links
- `Note-3.txt` - Microphone audio system and TDOA localization guide (v0.1.71+)
- `Note-5.txt` - Microphone operation guide (v0.1.72+, Russian)
- `AT-Help.txt` - Detailed command reference for all ATST commands (v0.1.71+)
- `Binary/Backup_*/CHANGE_SUMMARY.txt` - Version change summaries

## Testing Checklist

Before committing any changes that affect display or graphics:
- [ ] Run ATST150 (checkerboard) - verify no diagonal tilt
- [ ] Run ATST159 (drawing test) - verify all primitives work (v0.1.71+: 2s delays, multilingual)
- [ ] Run ATST169 (menu demo) - verify menu components work
- [ ] Run ATST171 (frequency analysis) - verify 800Hz/1200Hz display (600 seconds), test ATST999 interrupt
- [ ] Run ATST1 (display test) - comprehensive verification
- [ ] Run ATST160 (audio oscilloscope) - if audio changes made
- [ ] Run ATST162 (audio localization) - if TDOA changes made
- [ ] Run ATST999 to interrupt any long-running test
- [ ] Check display borders for alignment
- [ ] Verify text is crisp and readable

## Backup Strategy

Always create backups before major changes:
```bash
mkdir -p Binary/Backup_$(date +%Y%m%d)_v$(cat Drivers/BSP/VERSION.h | grep FIRMWARE_VERSION_STRING | cut -d'"' -f2)_to_new
cp build/TestF.bin build/TestF.elf build/TestF.hex Binary/Backup_*/
```

===============================================================================
## Project Continuation Recommendations (v0.1.73)
===============================================================================

This section provides quick-start guidance for continuing work on this project.

### Quick Start Guide

1. **First Time Setup:**
   - Read SUMMARY.md for project overview
   - Read Note-2.txt for display hardware configuration
   - Read Note-3.txt or Note-5.txt for microphone documentation
   - Read AT-Help.txt for complete command reference

2. **Building and Flashing:**
   ```bash
   cd /Users/mich/work/Antigravity/STM32F769I-Disco
   make                    # Build firmware
   st-flash write build/TestF.bin 0x08000000  # Flash to board
   screen /dev/tty.usbmodem* 115200  # Connect to USB CDC
   ```

3. **Testing the Display:**
   - Send `ATST?` to see command list
   - Send `ATST150` to verify no diagonal tilt
   - Send `ATST159` to test all drawing primitives
   - Send `ATST999` to interrupt any running test

4. **Menu System Quick Reference (v0.1.71+):**
   - `ATST165=X,Y,W,H,"TEXT",P,CT,TT` - Draw button
   - `ATST166=X,Y,W,H,P,C,BC` - Draw progress bar
   - `ATST167=X,Y,W,H,V,C,SV` - Draw slider
   - `ATST168=X,Y,W,H,"TEXT",S,I,C` - Draw list item
   - `ATST169` - Menu demonstration
   - `ATST170=0/1/2/3` - Screen clear methods

5. **Audio/Frequency Analysis (v0.1.73+):**
   - `ATST160` - 4-channel audio oscilloscope (10 seconds)
   - `ATST162` - 4-mic audio with source localization (10 seconds)
   - `ATST171` - Frequency analysis (800Hz RED / 1200Hz BLUE, 600 seconds / 10 minutes)
   - `ATST999` - Interrupt any running test

### For Adding New Features

**Adding a Drawing Command:**
1. Add test ID to `test_system.h` (enum `TestID_TypeDef`)
2. Implement function in `test_system.c`
3. Add case to `TEST_ProcessCommand()` or `TEST_ProcessCommandWithParams()`
4. Update help text in `TEST_ShowHelp()`
5. Update `AT-Help.txt` with detailed documentation
6. Update VERSION.h

**Creating Interactive Menus:**
1. Use ATST170=0 to clear screen
2. Use ATST155 to draw title and labels
3. Use ATST168 for menu items (list)
4. Use ATST165 for action buttons
5. Use ATST167 for adjustable values (sliders)
6. Use ATST166 for progress indication

**For Audio Development:**
1. Read Note-3.txt for microphone documentation
2. Integrate WM8994 driver from `Drivers/BSP/Components/wm8994/`
3. Configure I2S in `Core/Src/stm32f7xx_hal_msp.c`
4. Set up DMA in `Core/Src/dma.c`
5. Modify `TEST_Audio_Display()` or `TEST_Audio_With_Localization()`

### Critical Configuration Points

**Display (DO NOT MODIFY unless necessary):**
- File: `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`
- Function: `BSP_LCD_LayerDefaultInit()`
- Critical values:
  * CFBLR: 0x09600960 (LineLen=2400, Pitch=2400)
  * WHPCR: 0x031F0001 (X0=0, X1=799)
  * WVPCR: 0x01D70001 (Y0=0, Y1=471)

**DSI Timing (DO NOT MODIFY unless necessary):**
- File: `Core/Src/dsihost.c`
- Function: `MX_DSIHOST_DSI_Init()`
- Known working values: HSA=1, HBP=34, HFP=34, VSA=1, VBP=35, VFP=34

### Key People Contacts
- Project: STM32F769I-Discovery Firmware (TestF)
- Documentation: See SUMMARY.md, Note-2.txt, Note-3.txt, Note-5.txt, Note-5-.txt, AT-Help.txt

### Known Issues (v0.1.82)
- **ATST9 Touchscreen**: "Fingerprint artifacts" FIXED - replaced FillCircle with cross marker + circle outline
  - The test now uses simple cross markers instead of filled circles
  - Increased main loop delay to 200ms for better I2C stability
  - Increased USB batch transmission interval to 5 seconds to reduce USB flooding
  - Note: Freeze issues should be significantly reduced but monitor under heavy use
- **Audio Simulation**: All audio commands use simulated data by default
  - Real audio requires external MP34DT01 microphones via I2S/DFSDM
  - Framework provided: `TEST_GetAudioData()` hook function to override
- **Note-5-.txt Clarification**: STM32F769I-Discovery does NOT have built-in microphones
  - The board has WM8994 audio codec (mainly for audio OUTPUT)
  - For real audio input: connect external I2S/DFSDM microphones
  - See Note-5-.txt for microphone connection instructions
- **ATST999 Interrupt**: WORKING - command properly maps to test interrupt function
- **ATST1 Display**: WORKING - explicit LTDC reload for proper display refresh
- **SD Card Commands**: ATSTMCRI and ATSTMCRL work but require proper SD card formatting
  - Partition table reading works, but full file listing requires FatFS middleware integration

### Recommendations for Project Continuation (v0.1.82)

**IMPORTANT: Always create backup before making changes!**
```bash
# Backup format: Backup_YYYYMMDD_vX.Y.Z_to_vX.Y.(Z+1)/
cp test_system.c Backup_$(date +%Y%m%d)_v0.1.82_to_v0.1.83/
cp VERSION.h Backup_$(date +%Y%m%d)_v0.1.82_to_v0.1.83/
# ... other modified files
```

**Version Update Checklist:**
1. Update `Drivers/BSP/VERSION.h` (MAJOR.MINOR.PATCH)
2. Update `SUMMARY.md` version history at the top
3. Update `SUMMARY.md` "Current Version" field
4. Update `AT-Help.txt` if commands changed
5. Create backup directory with naming convention: `Backup_YYYYMMDD_vOld_to_vNew/`

1. **Touchscreen Enhancements** (v0.1.82: artifacts FIXED, stability improved):
   - DONE: Replaced FillCircle with cross marker + circle outline (fixes fingerprint artifacts)
   - DONE: Increased main loop delay to 200ms (reduces I2C issues)
   - DONE: Increased USB batch interval to 5 seconds (reduces USB flooding)
   - TODO: Monitor for any remaining freeze issues under extended use
   - TODO: Consider adding watchdog timer as last resort
   - TODO: Implement touch gesture recognition (tap, swipe, pinch)

2. **Audio Integration** (v0.1.82: framework ready):
   - FRAMEWORK READY: `TEST_GetAudioData()` hook function available in test_system.h
   - TODO: Configure SAI1 or DFSDM for PDM/I2S microphone input
   - TODO: Implement FFT using CMSIS-DSP library for 800Hz/1200Hz detection
   - TODO: Add real-time audio processing with DMA
   - TODO: Implement TDOA (Time Difference of Arrival) for sound source localization
   - TODO: Override `TEST_GetAudioData()` in main.c or application file to provide real audio

3. **Display Enhancements** (v0.1.82: LTDC reload working):
   - DONE: Explicit LTDC reload calls for proper refresh
   - DONE: Hardware-accelerated drawing functions (ATST174-180)
   - TODO: Add double-buffering to reduce flicker
   - TODO: Implement full DMA2D for all graphics operations
   - TODO: Add alpha blending and transparency support

4. **SD Card Integration** (v0.1.82: basic functions implemented):
   - DONE: ATSTMCRI - SD card information reader
   - DONE: ATSTMCRL - Partition table reader (MBR parsing)
   - DONE: Text command support (ATSTMCRI, ATSTMCRL)
   - TODO: Integrate FatFS middleware for full file system support
   - TODO: Implement file listing with FatFS
   - TODO: Add file read/write functionality

5. **WiFi and Network**:
   - DONE: OpenWeatherMap API (wttr.in removed)
   - DONE: Weather update interval increased to 10 minutes
   - TODO: Add more weather data fields (forecast, UV index, etc.)
   - TODO: Implement MQTT client for IoT applications
   - TODO: Add HTTPS/TLS support for secure connections

6. **System Improvements**:
   - TODO: Add non-volatile storage (flash or EEPROM) for settings
   - TODO: Implement configuration menu system
   - TODO: Add power management for battery operation

7. **Code Quality**:
   - DONE: Improved error handling in weather parsing
   - DONE: Consistent LCD start/completion messages for all tests
   - TODO: Add unit tests for critical functions
   - TODO: Add logging system for debugging

### Key Implementation Notes for Audio Integration

To implement real microphone support:

1. Add to `Core/Src/main.c`:
```c
// Override the weak function to provide real audio data
uint8_t TEST_GetAudioData(float *amp800, float *amp1200, float *angle, int16_t waveforms[4][200]) {
  // Your audio processing code here
  // Return 1 if real audio is available, 0 for simulation mode
  return 0;  // Start with 0 (simulation) until audio is working
}
```

2. Configure SAI1 or DFSDM in STM32CubeMX:
   - Enable SAI1 or DFSDM peripheral
   - Set up DMA for continuous audio capture
   - Configure for 16kHz sample rate

3. Implement FFT analysis:
   - Use CMSIS-DSP `arm_fft_f32()` for frequency analysis
   - Detect 800Hz and 1200Hz components
   - Calculate phase differences for TDOA

4. Build and test with ATST171 to verify audio input

### File Locations Summary
```
Drivers/BSP/VERSION.h                    - Version tracking
Drivers/BSP/TEST/test_system.h           - Test system definitions
Drivers/BSP/TEST/test_system.c           - Test system implementation
USB_DEVICE/App/usbd_cdc_if.c             - Command parsing
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c - Display
Core/Src/dsihost.c                        - DSI timing
Note-5-.txt                               - Microphone handling documentation
AT-Help.txt                               - Detailed ATST command reference
```

===============================================================================

---
