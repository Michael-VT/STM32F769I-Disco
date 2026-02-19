# USB CDC Diagnostic Patch for STM32F769I-Discovery

This patch adds early USB CDC initialization and diagnostic output to help identify why the Virtual COM port is not appearing.

## Problem
- USB CDC Virtual COM port (/dev/cu.usbmodem*) not appearing
- Only ST-LINK debugger port is visible
- Need to determine if issue is hardware or software

## Solution
Move USB CDC initialization to BEFORE FreeRTOS starts, add diagnostic messages.

## Files to Modify

### 1. Core/Src/main.c

**Find these lines (around line 136-147):**
```c
MX_GPIO_Init();
// LTDC and DSI will be initialized by BSP_LCD_Init() - DO NOT call here
// MX_LTDC_Init();
MX_DMA2D_Init();  // DMA2D is used by BSP LCD for graphics operations
MX_FMC_Init();  // FMC for SDRAM - needed before LCD
// MX_DSIHOST_DSI_Init();
MX_I2C1_Init();
MX_CRC_Init();
MX_RTC_Init();
```

**Replace with:**
```c
MX_GPIO_Init();

// ============================================================================
// USB CDC DIAGNOSTIC - Initialize USB CDC EARLY (before FreeRTOS)
// ============================================================================
// This allows USB CDC to be available even if FreeRTOS fails to start
MX_USB_DEVICE_Init();

// Wait for USB to enumerate
HAL_Delay(1000);

// Try to send diagnostic message via USB CDC
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
const char *diag_msg = "\r\n=== USB CDC DIAGNOSTIC ===\r\n"
                      "USB CDC initialized BEFORE FreeRTOS\r\n"
                      "If you see this, USB CDC is working!\r\n"
                      "============================\r\n";
CDC_Transmit_HS((uint8_t*)diag_msg, strlen(diag_msg));

// Give USB time to complete transmission
HAL_Delay(500);
// ============================================================================

// LTDC and DSI will be initialized by BSP_LCD_Init() - DO NOT call here
// MX_LTDC_Init();
MX_DMA2D_Init();  // DMA2D is used by BSP LCD for graphics operations
MX_FMC_Init();  // FMC for SDRAM - needed before LCD
// MX_DSIHOST_DSI_Init();
MX_I2C1_Init();
MX_CRC_Init();
MX_RTC_Init();
```

**Find these lines (around line 352-360):**
```c
// DIAGNOSTIC: Blink LED1 3 times to show we're about to start FreeRTOS
// If you see this blinking, firmware reached scheduler start
extern void BSP_LED_On(uint8_t Led);
extern void BSP_LED_Off(uint8_t Led);
extern void BSP_LED_Toggle(uint8_t Led);
for (int i = 0; i < 6; i++) {
  BSP_LED_Toggle(LED1);
  HAL_Delay(100);
}
```

**Replace with:**
```c
// ============================================================================
// USB CDC DIAGNOSTIC - Send second diagnostic message
// ============================================================================
diag_msg = "\r\n=== PRE-FREERTOS DIAGNOSTIC ===\r\n"
           "About to start FreeRTOS scheduler...\r\n"
           "LED1 will blink 3 times, then scheduler starts\r\n"
           "================================\r\n";
CDC_Transmit_HS((uint8_t*)diag_msg, strlen(diag_msg));
HAL_Delay(500);
// ============================================================================

// DIAGNOSTIC: Blink LED1 3 times to show we're about to start FreeRTOS
// If you see this blinking, firmware reached scheduler start
extern void BSP_LED_On(uint8_t Led);
extern void BSP_LED_Off(uint8_t Led);
extern void BSP_LED_Toggle(uint8_t Led);
for (int i = 0; i < 6; i++) {
  BSP_LED_Toggle(LED1);
  HAL_Delay(100);
}
```

### 2. Core/Src/freertos.c

**Find these lines (around line 275-290):**
```c
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* FIX v0.1.101: Add delay after USB init for reliable enumeration
   * The USB3300 ULPI PHY needs time to stabilize and complete enumeration.
   * Wait additional time after MX_USB_DEVICE_Init() before continuing. */
  osDelay(1000);  /* Increased from 500ms to 1000ms for USB PHY stability */

  /* USER CODE BEGIN StartDefaultTask */

  // FIX v0.1.101: Add delay after USB init for reliable enumeration
  /* The USB3300 ULPI PHY needs time to stabilize and complete enumeration.
   * Wait additional time after MX_USB_DEVICE_Init() before continuing. */
  osDelay(1000);  /* Increased from 500ms to 1000ms for USB PHY stability */
```

**Replace with:**
```c
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  // NOTE: USB_DEVICE is already initialized in main.c before FreeRTOS!
  // MX_USB_DEVICE_Init();  // <-- COMMENTED OUT - Already done in main.c

  /* FIX v0.1.101: Add delay after USB init for reliable enumeration
   * The USB3300 ULPI PHY needs time to stabilize and complete enumeration.
   * Wait additional time after MX_USB_DEVICE_Init() before continuing. */
  osDelay(1000);  /* Increased from 500ms to 1000ms for USB PHY stability */

  /* USER CODE BEGIN StartDefaultTask */

  // ============================================================================
  // USB CDC DIAGNOSTIC - Send FreeRTOS started message
  // ============================================================================
  extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
  const char *msg = "\r\n=== FREERTOS STARTED ===\r\n"
                    "DefaultTask is now running!\r\n"
                    "USB CDC should be fully functional\r\n"
                    "========================\r\n";
  CDC_Transmit_HS((uint8_t*)msg, strlen(msg));
  osDelay(500);
  // ============================================================================

  // FIX v0.1.101: Add delay after USB init for reliable enumeration
  /* The USB3300 ULPI PHY needs time to stabilize and complete enumeration.
   * Wait additional time after MX_USB_DEVICE_Init() before continuing. */
  // osDelay(1000);  // <-- COMMENTED OUT - Already delayed above
```

## Expected Results After Applying Patch

### If USB CDC is working:
You should see these messages when connecting to /dev/cu.usbmodem*:
```
=== USB CDC DIAGNOSTIC ===
USB CDC initialized BEFORE FreeRTOS
If you see this, USB CDC is working!
============================

=== PRE-FREERTOS DIAGNOSTIC ===
About to start FreeRTOS scheduler...
LED1 will blink 3 times, then scheduler starts
================================

=== FREERTOS STARTED ===
DefaultTask is now running!
USB CDC should be fully functional
========================
```

### If USB CDC is NOT working:
- No /dev/cu.usbmodem* device appears (besides ST-LINK)
- This indicates a HARDWARE issue:
  - USB cable not connected to CN13
  - USB ULPI PHY (USB3300) not powered
  - Hardware damage

## Hardware Check

### STM32F769I-Discovery Board Layout

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  [Top Edge]                                                 │
│                                                             │
│     ┌──────────────┐                                       │
│     │    LCD       │                                       │
│     │  Display     │                                       │
│     └──────────────┘                                       │
│                                                             │
│  [Left Edge]         [Right Edge]                           │
│                              ┌─────────────────┐           │
│                              │                 │           │
│                      CN13    │    ST-LINK      │           │
│                   ◄──────────│                 │           │
│              USB User FS     │                 │           │
│               (CDC VCOM)     │                 │           │
│                              │                 │           │
│                              └─────────────────┘           │
│                                      CN16                   │
│                              ◄──────────                    │
│                              ST-LINK USB                   │
│                                                             │
│  [Bottom Edge]                                              │
│                                                             │
│     ┌─────────────────────────────────────────────────┐   │
│     │  Arduino Headers (CN3, CN4, CN5, CN6)           │   │
│     │  CN2: WiFi Module                                │   │
│     └─────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Required Connections

1. **CN16 (ST-LINK/V2-1)** - Primary USB connection
   - Purpose: Programming, debugging, ST-LINK Virtual COM
   - Cable: Mini-USB or USB-C
   - Status: ✅ Connected (shows /dev/cu.usbmodem143303)

2. **CN13 (USB User FS)** - User CDC Virtual COM
   - Purpose: Firmware USB CDC Virtual COM port
   - Cable: USB-C (on newer boards) or Micro-USB
   - Status: ❓ **CHECK THIS CONNECTION**

### How to Find CN13

CN13 is located on the **right side** of the board (near the Arduino headers):
- Look for a USB port labeled "USB USER" or "CN13"
- It's separate from the ST-LINK port
- On newer board revisions, it's a USB-C port
- On older revisions, it's a Micro-USB port

## Test Procedure

1. **Apply the patch** to Core/Src/main.c and Core/Src/freertos.c

2. **Build and flash**:
   ```bash
   make clean && make -j8
   st-flash --reset write build/TestF.bin 0x08000000
   ```

3. **Connect USB cable to CN13** (USB User FS port)

4. **Wait 2-3 seconds** for USB enumeration

5. **Check for new USB device**:
   ```bash
   ls /dev/cu.usbmodem*
   ```

6. **Expected output**:
   ```
   /dev/cu.usbmodem143303  # ST-LINK (always present)
   /dev/cu.usbmodemXXXXXX  # NEW: Firmware CDC (if working)
   ```

7. **Connect to firmware CDC**:
   ```bash
   screen /dev/cu.usbmodemXXXXXX 115200
   ```

## Troubleshooting

### No new USB device appears:

1. **Check CN13 connection**:
   - Is USB cable firmly connected to CN13?
   - Try a different USB cable
   - Try a different USB port on your computer

2. **Check board power**:
   - Is the board powered via CN16?
   - Are the LEDs lit?

3. **Try without ST-LINK USB**:
   - Disconnect CN16, power only via CN13
   - This tests if there's a power conflict

### USB device appears but no messages:

1. **Connect immediately after flashing**:
   - The diagnostic messages are sent early
   - If you connect too late, you might miss them

2. **Press RESET button**:
   - This will restart the firmware and send messages again

## Next Steps

After applying this patch:
- If you see the diagnostic messages → USB CDC is working, issue was timing-related
- If no new USB device appears → Hardware issue with CN13 connection
- If USB device appears but no messages → Software issue with CDC_Transmit_HS
