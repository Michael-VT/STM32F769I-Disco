# 🎮 STM32F769I-Discovery Firmware

![Amber Terminal Screenshot](blob/main/screenshorts/kit_screen.jpg)
<img src="blob/main/screenshorts/kit_screen.jpg" alt="Описание" width="300"/>

<div align="center">

![STM32](https://img.shields.io/badge/STM32-STM32F769NIH6-blue?logo=stmicroelectronics)
![Platform](https://img.shields.io/badge/Platform-STM32F769I--Discovery-orange)
![Version](https://img.shields.io/badge/Version-0.1.85-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

**Advanced firmware for STM32F769I-Discovery board with ESP8266 WiFi, touchscreen, audio analysis, and comprehensive testing system**

[Features](#-features) • [Quick Start](#-quick-start) • [Configuration](#-configuration) • [Commands](#-commands) • [Hardware](#-hardware)

</div>

---

## 📖 Overview

This project is a comprehensive firmware for the **STM32F769I-Discovery** kit featuring:
- 🌐 **ESP8266 WiFi integration** with weather display and time synchronization
- 🖥️ **4-inch TFT LCD** (800×472) with capacitive touchscreen
- 🎤 **4× MP34DT01 microphones** for audio analysis
- 🎵 **WM8994 audio codec** for sound playback
- 💾 **External SDRAM** (8MB) and microSD card support
- 🔌 **USB CDC (Virtual COM Port)** for command interface
- ⚡ **FreeRTOS V2** real-time operating system

---

## ✨ Features

### 🌡️ Weather Display
- Real-time weather data from **OpenWeatherMap API**
- Temperature, feels like, humidity, wind speed
- Automatic updates every **10 minutes**

### ⏰ Time Synchronization
- **NTP** (Network Time Protocol) synchronization
- Automatic RTC updates
- Multiple NTP server support

### 🖥️ Display System
- **DSI video mode** with LTDC controller
- **DMA2D hardware acceleration** for fast graphics
- 800×472 pixel RGB888 display
- Advanced drawing primitives for menus and graphs

### 🎤 Audio Analysis
- **4× MP34DT01 MEMS microphones** for sound capture
- **TDOA** (Time Difference of Arrival) direction finding
- **FFT-based frequency analysis** (800Hz & 1200Hz)
- Real-time oscilloscope display

### 🧪 Comprehensive Test System
- **50+ ATST commands** for testing all hardware
- Touchscreen calibration and testing
- LED, RTC, SDRAM, WiFi, and HTTP tests
- Drawing primitives for custom graphics

---

## 🚀 Quick Start

### Prerequisites

- **STM32CubeIDE** or arm-none-eabi-gcc toolchain
- **ST-Link** driver for flashing
- **USB cable** for Virtual COM Port
- **WiFi network** (2.4GHz)
- **OpenWeatherMap API key** (free at openweathermap.org)

### Building the Firmware

```bash
# Clone the repository
git clone https://github.com/Michael-VT/STM32F769I-Disco.git
cd STM32F769I-Disco

# Build with Make
make clean
make -j4

# Or use STM32CubeIDE
# Open project and click Build
```

### Flashing the Firmware

1. Connect **ST-Link** to the board
2. Connect **USB** to PC (Virtual COM Port)
3. Flash `build/TestF.bin` or `build/TestF.hex`
4. Reset the board

---

## ⚙️ Configuration

### WiFi Credentials

Edit `Core/Src/freertos.c`:

```c
#define WIFI_SSID     "Your_WiFi_SSID"
#define WIFI_PASSWORD "Your_WiFi_Password"
```

### Weather Settings

Edit `Drivers/BSP/WEATHER/weather.h`:

```c
#define WEATHER_CITY  "Your_City"
#define WEATHER_APPID  "Your_OpenWeatherMap_API_Key"
```

### OpenWeatherMap API Key

1. Visit [openweathermap.org/api](https://openweathermap.org/api)
2. Sign up for free account
3. Copy your API key
4. Paste in `weather.h`

---

## 🎮 Commands (ATST System)

All commands are sent via **USB CDC (Virtual COM Port)**. Use a terminal emulator like PuTTY, CoolTerm, or Arduino Serial Monitor.

### Quick Reference

| Command | Description |
|---------|-------------|
| `ATST?` or `HELP` | Show help |
| `ATST0` | ESP8266 Weather Test |
| `ATST4` | WiFi Connection Test |
| `ATST9` | Touchscreen Test (20s) |
| `ATST10` | Run All Tests |
| `ATST999` | **Stop Current Test** |

### Basic Test Commands

```text
ATST0    - ESP8266 Weather Test
           Tests WiFi, fetches weather from OpenWeatherMap

ATST1    - Display Graphics/Text Test
           Colors, fonts, shapes, gradients

ATST2    - LED Test
           Tests all 4 LEDs (LD1-LD4)

ATST3    - RTC Time Test
           Real-time clock check

ATST4    - WiFi Connection Test
           Connects to configured network

ATST5    - NTP Time Sync Test
           Synchronizes RTC with NTP server

ATST6    - HTTP Request Test
           Tests HTTP GET to various endpoints

ATST7    - Audio Codec Test
           WM8994 audio codec check

ATST8    - SDRAM Test
           8MB external SDRAM read/write

ATST9    - Touchscreen Test (20 seconds)
           Records all touches with coordinates

ATST10   - Run All Tests
           Executes tests 0-9 in sequence
```

### Display Mode Commands

```text
ATST141  - Text Display Mode
ATST142  - Graphics Pattern Mode
ATST143  - Color Bars Mode
ATST144  - Gradient Mode
ATST145  - Checkerboard (40×40)
ATST146  - LTDC Info Display
ATST147  - Focus Pattern (convergence test)
ATST148  - Display Modes Help
ATST149  - LCD Panel Detection Diagnostic
ATST150  - Checkerboard 50×50 (timing test)
```

### Drawing Commands (Menu/Graph Support)

```text
ATST151=X,Y,D,C          - Draw point at (X,Y)
                           X=0-799, Y=0-471, D=1-50px, C=hex color

ATST152=X,Y,D,C          - Draw line from current to (X,Y)

ATST153=X,Y,W,H,F,C       - Draw circle/ellipse
                           center(X,Y), width W, height H
                           F=0(outline)/1(filled), color C

ATST154=X,Y,W,H,F,C       - Draw rectangle

ATST155=X,Y,S,L,C,"TEXT"  - Draw text
                           S=8/12/16/20/24, L=0/1/2/3(angle)

ATST156=X,Y,W,H,MI,MA,A,C - Draw graph axis/grid
                           MI=minY, MA=maxY, A=bitmask

ATST157=X,Y;X,Y;...       - Draw line graph
                           Semicolon-separated X,Y pairs

ATST158=VAL,VAL,...       - Draw bar graph
                           Comma-separated values
```

### Audio & Display Commands

```text
ATST159  - Drawing Commands Demo (2s per primitive)
ATST160  - 4-Channel Audio Oscilloscope
ATST162  - 4-Mic Audio with Sound Source Localization
ATST171  - Frequency Analysis (800Hz RED / 1200Hz BLUE)
           Direction finding, amplitude display
           4 waveform windows, spectral analysis
           Duration: 60 seconds
```

### Menu UI Commands

```text
ATST165=X,Y,W,H,"TXT",P,CT,TT  - Draw Button
ATST166=X,Y,W,H,P,C,BC         - Draw Progress Bar
ATST167=X,Y,W,H,V,C,SV          - Draw Slider
ATST168=X,Y,W,H,"TXT",S,I,C     - Draw List Item
ATST169                         - Menu Demonstration
ATST170=M                       - Screen Clear
                                 M=0(black),1(white),2(gradient),3(checkerboard)
```

### SD Card Commands

```text
ATSTMCRI - MicroSD Card Information
          Card state, capacity, CID/CSD registers

ATSTMCRL - MicroSD File/Partition List
          Lists partitions on SD card
```

### Special Commands

```text
ATST999  - 🔴 STOP CURRENT TEST
          Interrupts any running test immediately

ATVER    - Show Firmware Version

AT?      - Show Help (alias for ATST?)
```

### Color Codes

```text
0xFF0000  - Red          0x00FF00  - Green        0x0000FF  - Blue
0xFFFF00  - Yellow       0x00FFFF  - Cyan         0xFF00FF  - Magenta
0xFFFFFF  - White        0x000000  - Black        0x808080  - Gray
```

---

## 🔧 Hardware

### Board: STM32F769I-Discovery (MB1225B)

| Component | Description |
|-----------|-------------|
| **MCU** | STM32F769NIH6 (216MHz Cortex-M7, 2MB Flash, 512KB SRAM) |
| **Display** | MB1166 Kit - 4" DSI TFT (800×472 RGB888) with FT6x06 Touch |
| **Audio** | 4× MP34DT01 MEMS microphones + WM8994 codec |
| **Memory** | 8MB SDRAM + microSD slot |
| **Connectivity** | ESP8266 WiFi module (UART) |
| **USB** | USB OTG FS (Virtual COM Port + ST-Link) |

### Display Specifications

```
Resolution:     800 × 472 pixels
Interface:      DSI (Display Serial Interface)
Controller:     OTM8009A (via DSI)
Touch:          FT6x06 capacitive (I2C)
Color Format:   RGB888 (24-bit)
Backlight:      LED driver via I2C
```

---

## 📊 Display Layout

```
┌─────────────────────────────────────────────────────┐
│                    Header (Time/Date)                │
├───────────────┬─────────────────────────────────────┤
│               │                                     │
│    Main       │         Waveform / Graph            │
│    Display    │         Windows (4×)                │
│    Area        │         200×50 each                │
│               │                                     │
│               │                                     │
├───────────────┴─────────────────────────────────────┤
│                    Status / Info Bar                 │
└─────────────────────────────────────────────────────┘
```

---

## 🧪 Running Tests

### Example Test Session

```text
> Connect via USB CDC (Virtual COM Port)
> Baud rate: 115200

ATST4
*** Test ATST4 Start! ***
=== ATST4: WiFi Connection Test ===
[Step 1/4] Disconnecting...
[PASS] Disconnect: Disconnected
[Step 2/4] Setting station mode...
[PASS] Station mode: Mode set to Station
[Step 3/4] Connecting to WiFi...
[PASS] WiFi Connect: Connected successfully
[Step 4/4] Getting IP address...
[PASS] IP Address: 192.168.1.180
=== ATST4 Complete ===
*** Test ATST4 Complete! ***

ATST0
*** Test ATST0 Start! ***
=== ATST0: ESP8266 Weather Test ===
[Step 1/4] Testing AT command...
[PASS] AT command: ESP8266 responding
...
[PASS] Weather request: Clear sky, 13.4°C, feels like 12.8°C, humidity 76%
```

### Interrupting Tests

```text
# If a test is running and you want to stop it:

ATST999

*** Test ATST999 Start! ***
[ATST999] Interrupt signal sent - stopping current test...
TEST INTERRUPTED
```

---

## 🎯 Advanced Features

### Real Audio Integration (ATST171)

To enable **real microphone input** instead of simulation, override the weak function:

```c
// In your application file (e.g., main.c)
uint8_t TEST_GetAudioData(float *amp800, float *amp1200,
                          float *angle, int16_t waveforms[4][200]) {
    // 1. Read audio samples from I2S/DFSDM
    // 2. Perform FFT to detect 800Hz and 1200Hz
    // 3. Calculate TDOA for direction finding
    // 4. Fill waveforms array with data

    // Return 1 for real audio mode, 0 for simulation
    return 1;
}
```

### Custom Graphics

The drawing primitives (ATST151-158) enable:

- 📊 **Real-time graphs** - oscilloscope-style displays
- 🎛️ **Custom menus** - buttons, sliders, progress bars
- 📈 **Data visualization** - line graphs, bar charts
- 🎨 **Text rendering** - multiple fonts, rotation

---

## 📁 Project Structure

```
STM32F769I-Disco/
├── Core/                    # STM32CubeMX generated core
│   ├── Inc/                # Header files
│   └── Src/                # Source files (main.c, freertos.c)
├── Drivers/                # BSP and HAL drivers
│   ├── BSP/                # Board Support Package
│   │   ├── ESP8266/        # ESP8266 WiFi module driver
│   │   ├── WEATHER/        # Weather API client
│   │   ├── TEST/           # Test system (ATST commands)
│   │   └── STM32F769I-Discovery/
│   ├── STM32F7xx_HAL_Driver/
│   └── CMSIS/
├── Middlewares/            # FreeRTOS, USB, etc.
├── USB_DEVICE/             # USB CDC implementation
├── build/                  # Compiled binaries
│   ├── TestF.bin          # Raw binary for flashing
│   ├── TestF.hex          # Intel HEX format
│   └── TestF.elf          # ELF with debug info
├── Makefile                # Build configuration
└── README.md               # This file
```

---

## 🐛 Troubleshooting

### WiFi Not Connecting

```text
1. Check WiFi credentials in freertos.c
2. Verify ESP8266 is powered (LED should be on)
3. Try ATST4 to test connection separately
4. Check router supports 2.4GHz (ESP8266 doesn't support 5GHz)
```

### Touchscreen Not Working

```text
1. Run ATST9 to test touchscreen
2. Check FT6x06 I2C connection
3. Recalibrate if needed
4. Known issue: "ghost touches" near screen edges
```

### SD Card Not Detected

```text
1. Ensure card is fully inserted
2. Try FAT32 format (recommended)
3. Run ATSTMCRI to check card state
4. Reinsert card if detection fails (retry logic included)
```

### Display Issues

```text
1. Run ATST149 for LCD panel detection
2. Check DSI clock and timing
3. Verify LTDC configuration
4. Test with ATST1 (Display Test)
```

---

## 📈 Version History

- **v0.1.85** - Display system improvements, touch test fixes, audio analysis enhancements
- **v0.1.84** - Startup version fix, SD card improvements
- **v0.1.83** - ATST999 fix, LCD display messages
- **v0.1.82** - Touch test fixes, SD card commands
- **v0.1.80** - SD card support, optimized drawing
- See [SUMMARY.md](SUMMARY.md) for complete history

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly on real hardware
5. Submit a pull request

---

## 📝 License

This project is open source and available under the MIT License.

---

## 🔗 Links

- [STM32F769I-Discovery](https://www.st.com/en/evaluation-tools/stm32f769i-discovery.html)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [ESP8266 Datasheet](https://www.espressif.com/sites/default/files/documentation/0a-esp8266_datasheet_en.pdf)
- [OpenWeatherMap API](https://openweathermap.org/api)

---

## 📧 Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check the [SUMMARY.md](SUMMARY.md) for detailed documentation
- Review [AT-Help.txt](AT-Help.txt) for complete command reference

---

<div align="center">

**Made with ❤️ for the STM32F769I-Discovery community**

![Star](https://img.shields.io/github/stars/Michael-VT/STM32F769I-Discovery?style=social)
![Fork](https://img.shields.io/github/forks/Michael-VT/STM32F769I-Discovery?style=social)

</div>
