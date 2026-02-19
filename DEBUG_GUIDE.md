# STM32F769I-Discovery USB CDC Debugging Guide

## Current Status

You're experiencing:
- ✅ Firmware builds and flashes successfully
- ✅ LED1 blinks 3 times (firmware reaches FreeRTOS)
- ✅ Display shows test image
- ❌ No USB CDC Virtual COM port appears
- ❌ Only ST-LINK port visible (`/dev/cu.usbmodem143303`)

## Diagnostic Tools Created

### 1. USB Debug Tool (`usb_debug.py`)
Monitors USB devices and tests serial connections.

### 2. GDB Debug Script (`gdb_debug.py`)
Automated GDB debugging to check firmware state.

## Step-by-Step Debugging

### Step 1: Verify Hardware Connection

**CRITICAL:** The STM32F769I-DISCO requires **TWO USB cables**:

```
┌─────────────────────────────────────────────────────────────┐
│  Board (Top View)                                           │
│                                                             │
│    ┌──────────┐                                            │
│    │   LCD    │                                            │
│    └──────────┘                                            │
│                                                             │
│                        CN13 (USB User FS)                    │
│                   ┌──────────┐                              │
│                   │   USB-C  │ ← CONNECT HERE!              │
│                   └──────────┘                              │
│                                                             │
│    ┌──────────────────────────────────────────────────┐    │
│    │              Arduino Headers                       │    │
│    │              CN2: WiFi Module                      │    │
│    └──────────────────────────────────────────────────┘    │
│                                                             │
│                        CN16 (ST-LINK)                       │
│                   ┌──────────┐                              │
│                   │ Mini-USB │ ← Already connected         │
│                   └──────────┘                              │
└─────────────────────────────────────────────────────────────┘

CN13 Location: RIGHT side of board, near Arduino headers
CN13 Type: USB-C (or Micro-USB on older revisions)
CN13 Purpose: Firmware CDC Virtual COM
```

**Action:**
1. Find CN13 on the RIGHT side of the board
2. Connect a USB-C cable to CN13
3. The other end goes to your computer

### Step 2: Run USB Debug Tool

```bash
cd /Users/mich/work/Antigravity/STM32F769I-Disco
python3 usb_debug.py
```

Choose option 1 to monitor for device changes.

**What to expect:**
- After connecting CN13, you should see a NEW device appear
- Total should be 2 `/dev/cu.usbmodem*` devices

### Step 3: If Still No Second Device

#### Option A: Use GDB to Debug Firmware

**Terminal 1 - Start OpenOCD:**
```bash
cd /Users/mich/work/Antigravity/STM32F769I-Disco
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg
```

**Terminal 2 - Run GDB Debug:**
```bash
python3 gdb_debug.py
```

This will show:
- Program Counter (PC) location
- USB OTG register values
- Whether firmware is running

#### Option B: Manual GDB Session

```bash
# Terminal 1: OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg

# Terminal 2: GDB
arm-none-eabi-gdb build/TestF.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) break MX_USB_DEVICE_Init
(gdb) continue
# Wait for breakpoint or Ctrl+C
(gdb) info registers pc
(gdb) backtrace
```

### Step 4: Check USB OTG HS Registers

Key registers to check via GDB:

```
Address    Register        Expected Value
0x40040000 GOTGCTL         0x08000000 (Device mode)
0x40040038 GCCFG          0x01000000 (VBUS sensing disabled)
0x4004000C GRSTCTL        Check if PHY reset
0x40040050 DCFG           Device address
```

## Common Issues & Solutions

### Issue 1: Only ST-LINK Port Visible

**Possible Causes:**
1. CN13 not connected ← MOST LIKELY
2. USB cable issue
3. USB3300 PHY not powered
4. Firmware crash before USB init

**Solutions:**
1. Connect USB-C cable to CN13 (RIGHT side of board)
2. Try different USB cable
3. Check board power (LEDs should be lit)
4. Use GDB to check if MX_USB_DEVICE_Init is called

### Issue 2: Second Device Appears But No Data

**Possible Causes:**
1. FreeRTOS not starting
2. CDC_Transmit_HS not being called
3. Mutex deadlock

**Solutions:**
1. Check LED1 is blinking (FreeRTOS running)
2. Use GDB to set breakpoint at CDC_Transmit_HS
3. Check osMutex creation in freertos.c

### Issue 3: USB Device Appears Then Disappears

**Possible Causes:**
1. USB enumeration failing
2. Firmware crash after USB init
3. Power issue

**Solutions:**
1. Use USB analyzer or Console.app to see kernel logs
2. Check Console.app for USB errors
3. Try powering from external supply

## Console.app USB Debugging

On macOS, check system logs for USB events:

```bash
log show --predicate 'process == "kernel" AND eventMessage CONTAINS "USB"' --last 5m
```

Or open Console.app and filter for "USB".

## Firmware Code Flow

```
main()
  ├─ HAL_Init()
  ├─ SystemClock_Config()
  ├─ MX_GPIO_Init()
  ├─ MX_DMA2D_Init()
  ├─ MX_FMC_Init()
  ├─ BSP_SDRAM_Init()
  ├─ BSP_LCD_Init()  ← LCD initialization
  ├─ [LED blinks 3 times]
  └─ osKernelStart()
       └─ StartDefaultTask()
            ├─ MX_USB_DEVICE_Init()  ← USB CDC initialized HERE
            ├─ ESP8266_Init()
            └─ DisplayTask, etc.
```

**Key Insight:** USB CDC is initialized **AFTER** FreeRTOS starts.
If FreeRTOS fails, USB CDC will never initialize.

## What "Worked Before"

Check git history for working versions:

```bash
# Check SNTP working version
git show 1488754:Drivers/BSP/VERSION.h

# Check backup commits
git log --oneline --grep="Backup"
```

According to SUMMARY.md, these were working:
- v0.1.14: Display trace markers
- v0.2.0: SNTP working, HTTP debug output

## Next Steps

1. **Run USB debug tool:**
   ```bash
   python3 usb_debug.py
   ```

2. **Check CN13 connection** - This is the most likely issue!

3. **If CN13 is connected and still no CDC:**
   - Run GDB debug script
   - Check if MX_USB_DEVICE_Init is reached
   - Verify USB OTG registers

4. **Compare with ST Demo:**
   - Flash ST demo firmware
   - Check if it provides CDC Virtual COM
   - If yes, hardware is OK, issue is in our firmware
