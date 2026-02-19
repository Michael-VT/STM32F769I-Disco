# Display Settings Adjustment Summary

## Project
STM32F769I-Discovery with MB1166 Rev.A A-02 Display Kit
- **Display Panel**: OTM8009A DSI LCD controller
- **Resolution**: 800 x 472 pixels (NOT 800x480!)
- **Interface**: DSI (Display Serial Interface) with 2 data lanes, burst mode

## Critical LTDC Register Settings

### CFBLR (Color Frame Buffer Line Length Register)
- **Address**: LTDC_LAYER->CFBLR
- **Format**: [31:16]=CFBLL (Line Length), [12:0]=CFBP (Pitch)
- **Correct Value**: `0x09600960`
  - CFBLL = 2400 (800 pixels × 3 bytes/pixel for RGB888)
  - CFBP = 2400 (exact pitch, no padding)
- **Bug**: HAL_LTDC_ConfigLayer() adds +3 to pitch (2400 → 2403), causing diagonal tilt
- **Fix**: Applied in BSP_LCD_LayerDefaultInit() (v0.1.66+)

### WHPCR (Window Horizontal Position Configuration Register)
- **Address**: LTDC_LAYER->WHPCR
- **Format**: [26:16]=StopPos, [10:0]=StartPos (0-based indexing!)
- **Correct Value**: `0x031F0001`
  - StopPos = 799 (for pixels 0-799, NOT 800!)
  - StartPos = 0
- **Bug**: HAL_LTDC_ConfigLayer() adds AHBP (35) to window positions
- **Fix**: Applied in BSP_LCD_LayerDefaultInit() (v0.1.66+)

### WVPCR (Window Vertical Position Configuration Register)
- **Address**: LTDC_LAYER->WVPCR
- **Format**: [26:16]=StopPos, [10:0]=StartPos (0-based indexing!)
- **Correct Value**: `0x01D70001`
  - StopPos = 471 (for pixels 0-471, NOT 472!)
  - StartPos = 0
- **Bug**: HAL_LTDC_ConfigLayer() adds AVBP to window positions
- **Fix**: Applied in BSP_LCD_LayerDefaultInit() (v0.1.66+)

## Key Display Timing Parameters

### LTDC Timing (in pixel clocks)
- **HSA** (Horizontal Sync Active): 1
- **HBP** (Horizontal Back Porch): 34
- **HFP** (Horizontal Front Porch): 34
- **VSA** (Vertical Sync Active): 1
- **VBP** (Vertical Back Porch): 35
- **VFP** (Vertical Front Porch): 34
- **TotalWidth**: 868 (800 + 1 + 34 + 34)
- **TotalHeigh**: 541 (472 + 1 + 35 + 34)

### DSI Timing (in DSI byte clock cycles)
- **HorizontalSyncActive**: 2
- **HorizontalBackPorch**: 77
- **HorizontalFrontPorch**: 34
- **HorizontalLine**: 1980
- Calculation: (HACT + HSA + HBP + HFP) × laneByteClk / LcdClock
- = (800 + 1 + 34 + 34) × 62500 / 27429 ≈ 1980

## Color Format

### RGB888 (24-bit)
- **Bytes per pixel**: 3
- **Format**: 0xBBGGRR (little-endian in memory)
- **Line length**: 800 × 3 = 2400 bytes

### Common Color Values (RGB888 format)
```c
LCD_COLOR_BLACK     0xFF000000  // Black
LCD_COLOR_WHITE     0xFFFFFFFF  // White
LCD_COLOR_RED       0xFFFF0000  // Red
LCD_COLOR_GREEN     0xFF00FF00  // Green
LCD_COLOR_BLUE      0xFF0000FF  // Blue
LCD_COLOR_YELLOW    0xFFFFFF00  // Yellow
LCD_COLOR_CYAN      0xFF00FFFF  // Cyan
LCD_COLOR_MAGENTA   0xFFFF00FF  // Magenta
LCD_COLOR_ORANGE    0xFFFFA500  // Orange
```

## Coordinate System
- **X axis**: 0 to 799 (left to right)
- **Y axis**: 0 to 471 (top to bottom)
- **Origin**: Top-left corner

## Common Issues and Solutions

### 1. Diagonal Tilt (45-degree shear)
- **Symptom**: Each row offset by 1 pixel
- **Cause**: CFBLR pitch = 2403 instead of 2400
- **Fix**: Ensure BSP_LCD_LayerDefaultInit() corrects CFBLR

### 2. Horizontal Offset
- **Symptom**: Image shifted right
- **Cause**: WHPCR includes AHBP offset
- **Fix**: Ensure BSP_LCD_LayerDefaultInit() corrects WHPCR

### 3. Vertical Offset
- **Symptom**: Image shifted down
- **Cause**: WVPCR includes AVBP offset
- **Fix**: Ensure BSP_LCD_LayerDefaultInit() corrects WVPCR

## New Drawing Commands (v0.1.68+)

### ATST151=X,Y,D,C - Draw Point
- **X**: Width coordinate (0-799)
- **Y**: Height coordinate (0-471)
- **D**: Point diameter in pixels (1-50)
- **C**: Color in hex (e.g., 0xFF0000 for red)
- **Example**: `ATST151=100,200,5,0xFF0000`

### ATST152=X,Y,D,C - Draw Line
- Draws line from current position to (X,Y)
- **X**: Width coordinate (0-799)
- **Y**: Height coordinate (0-471)
- **D**: End point diameter (1-50)
- **C**: Color in hex
- **Example**: `ATST152=400,300,3,0x00FF00`

### ATST154=X,Y,W,H,F,C - Draw Rectangle
- **X**: Top-left X coordinate (0-799)
- **Y**: Top-left Y coordinate (0-471)
- **W**: Width in pixels (1-800)
- **H**: Height in pixels (1-472)
- **F**: Fill flag (0=outline, 1=filled)
- **C**: Color in hex
- **Example**: `ATST154=100,100,200,150,1,0x0000FF`

### ATST159 - Drawing Demo
- Demonstrates all drawing commands
- **Example**: `ATST159`

## Testing Commands

### Display Tests
- **ATST1**: Full display test (colors, fonts, shapes)
- **ATST141-148**: Display mode tests
- **ATST149**: LCD panel detection diagnostic
- **ATST150**: 50x50 checkerboard (timing test)

## Firmware Version History
- **v0.1.68**: Added drawing commands (ATST151, ATST152, ATST154, ATST159)
- **v0.1.67**: Removed duplicate/conflicting fixes in main.c
- **v0.1.66**: Fixed window position bug (AHBP added by HAL)
- **v0.1.65**: Enhanced CFBLR diagnostic output

## File Locations
- **LTDC Configuration**: `Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c`
- **Test System**: `Drivers/BSP/TEST/test_system.c`
- **USB CDC Handler**: `USB_DEVICE/App/usbd_cdc_if.c`
- **Version**: `Drivers/BSP/VERSION.h`
