/**
 ******************************************************************************
 * @file    stm32f769i_discovery_lcd.c
 * @author  MCD Application Team
 * @brief   This file includes the driver for Liquid Crystal Display (LCD)
 *module mounted on STM32F769I-DISCOVERY board.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* File Info: ------------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive directly in video mode a LCD TFT using the DSI
interface. The following IPs are implied : DSI Host IP block working in
conjunction to the LTDC controller.
   - This driver is linked by construction to LCD KoD mounted on board MB1166.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
     o Select the LCD layer to be used using the BSP_LCD_SelectLayer() function.
     o Enable the LCD display using the BSP_LCD_DisplayOn() function.

  + Options
     o Configure and enable the color keying functionality using the
       BSP_LCD_SetColorKeying() function.
     o Modify in the fly the transparency and/or the frame buffer address
       using the following functions:
       - BSP_LCD_SetTransparency()
       - BSP_LCD_SetLayerAddress()

  + Display on LCD
     o Clear the whole LCD using BSP_LCD_Clear() function or only one specified
string line using the BSP_LCD_ClearStringLine() function. o Display a character
on the specified line and column using the BSP_LCD_DisplayChar() function or a
complete string line using the BSP_LCD_DisplayStringAtLine() function. o Display
a string line on the specified position (x,y in pixel) and align mode using the
BSP_LCD_DisplayStringAtLine() function. o Draw and fill a basic shapes (dot,
line, rectangle, circle, ellipse, .. bitmap) on LCD using the available set of
functions.

------------------------------------------------------------------------------*/

/* Dependencies
- stm32f769i_discovery.c
- stm32f769i_discovery_sdram.c
- stm32f7xx_hal_dsi.c
- stm32f7xx_hal_ltdc.c
- stm32f7xx_hal_ltdc_ex.c
- stm32f7xx_hal_dma2d.c
- stm32f7xx_hal_rcc_ex.c
- stm32f7xx_hal_gpio.c
- stm32f7xx_hal_cortex.c
- otm8009a.c
- adv7533.c
- fonts.h
- font24.c
- font20.c
- font16.c
- font12.c
- font8.c"
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32f769i_discovery_lcd.h"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font24.c"
// #include "../../../Utilities/Fonts/font48.c"  // Not available
// #include "../../../Utilities/Fonts/font72.c"  // Not available
#include "../../../Utilities/Fonts/font8.c"
// #include "../../../Utilities/Fonts/font96.c"  // Not available
#include "../../../Utilities/Fonts/fonts.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STM32F769I_DISCOVERY
 * @{
 */

/** @defgroup STM32F769I_DISCOVERY_LCD STM32F769I_DISCOVERY LCD
 * @{
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Private_Defines LCD Private Defines
 * @{
 */
#if defined(USE_LCD_HDMI)
#define HDMI_ASPECT_RATIO_16_9 ADV7533_ASPECT_RATIO_16_9
#define HDMI_ASPECT_RATIO_4_3 ADV7533_ASPECT_RATIO_4_3
#endif /* USE_LCD_HDMI */
#define LCD_DSI_ID 0x11
#define LCD_DSI_ID_REG 0xA8

static DSI_VidCfgTypeDef hdsivideo_handle;
/**
 * @}
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Private_TypesDefinitions LCD Private
 * TypesDefinitions
 * @{
 */
#if defined(USE_LCD_HDMI)
/**
 * @brief  DSI timming params used for different HDMI adpater
 */
typedef struct {
  uint16_t HACT;
  uint16_t HSYNC;
  uint16_t HBP;
  uint16_t HFP;
  uint16_t VACT;
  uint16_t VSYNC;
  uint16_t VBP;
  uint16_t VFP;
  uint8_t ASPECT_RATIO;
  uint8_t RGB_CODING;
} HDMI_FormatTypeDef;

/**
 * @brief  DSI packet params used for different HDMI adpater
 */
typedef struct {
  uint16_t NullPacketSize;
  uint16_t NumberOfChunks;
  uint16_t PacketSize;
} HDMI_DSIPacketTypeDef;

/**
 * @brief  LTDC PLL params used for different HDMI adpater
 */
typedef struct {
  uint16_t PLLSAIN;
  uint16_t PLLSAIR;
  uint32_t PCLK;
  uint16_t IDF;
  uint16_t NDIV;
  uint16_t ODF;
  uint16_t LaneByteClock;
  uint16_t TXEscapeCkdiv;
} HDMI_PLLConfigTypeDef;
#endif /* USE_LCD_HDMI */
/**
 * @}
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Private_Macros LCD Private Macros
 * @{
 */
#define ABS(X) ((X) > 0 ? (X) : -(X))

#define POLY_X(Z) ((int32_t)((Points + (Z))->X))
#define POLY_Y(Z) ((int32_t)((Points + (Z))->Y))
/**
 * @}
 */
/** @defgroup STM32F769I_DISCOVERY_LCD_Private_Types_Definitions Private Types
 * Definitions
 * @{
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Exported_Variables STM32F769I DISCOVERY
 * LCD Exported Variables
 * @{
 */
DMA2D_HandleTypeDef hdma2d_discovery;
LTDC_HandleTypeDef hltdc_discovery;
DSI_HandleTypeDef hdsi_discovery;
uint32_t lcd_x_size = OTM8009A_800X480_WIDTH;
uint32_t lcd_y_size = OTM8009A_800X480_HEIGHT;
LCD_Driver_t Lcd_Driver_Type = LCD_CTRL_NT35510;
/**
 * @}
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Private_Variables LCD Private Variables
 * @{
 */
#if defined(USE_LCD_HDMI)
/**
 * @brief  DSI timming used for different HDMI resolution (720x480 and 720x576)
 */
HDMI_FormatTypeDef HDMI_Format[2] = {
    /* HA   HS  HB  HF  VA   VS VB  VF  ASPECT                BPP */
    {720, 62, 60, 30, 480, 6, 19, 9, HDMI_ASPECT_RATIO_4_3,
     LCD_DSI_PIXEL_DATA_FMT_RBG888},
    {720, 64, 68, 12, 576, 5, 39, 5, HDMI_ASPECT_RATIO_16_9,
     LCD_DSI_PIXEL_DATA_FMT_RBG888}

};

/**
 * @brief  DSI packet size used for different HDMI resolution (720x480 and
 * 720x576)
 */
HDMI_DSIPacketTypeDef HDMI_DSIPacket[2] = {
    /* NP NC VP */
    {0, 1, 720},
    {0, 1, 720}};

/**
 * @brief  LTDC PLL settings used for different HDMI resolution (720x480 and
 * 720x576)
 */
HDMI_PLLConfigTypeDef HDMI_PLLConfig[4] = {
    /* N   DIV Pclk   IDF              NDIV ODF               LBClk
       TXEscapeCkdiv*/
    {325, 6, 27083, DSI_PLL_IN_DIV5, 65, DSI_PLL_OUT_DIV1, 40625, 3},
    {325, 6, 27083, DSI_PLL_IN_DIV5, 65, DSI_PLL_OUT_DIV1, 40625, 3}

};
#endif /* USE_LCD_HDMI */
/**
 * @brief  Default Active LTDC Layer in which drawing is made is LTDC Layer
 * Background
 */
static uint32_t ActiveLayer = LTDC_ACTIVE_LAYER_BACKGROUND;

/**
 * @brief  Current Drawing Layer properties variable
 */
static LCD_DrawPropTypeDef DrawProp[LTDC_MAX_LAYER_NUMBER];
/**
 * @}
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Private_FunctionPrototypes LCD Private
 * FunctionPrototypes
 * @{
 */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1,
                         uint16_t y2, uint16_t y3);
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize,
                          uint32_t ySize, uint32_t OffLine,
                          uint32_t ColorIndex);
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize,
                                     uint32_t ColorMode);
static uint16_t LCD_IO_GetID(void);
static LCD_Driver_t Driver_Type(LCD_Driver_t Lcd_type);
/**
 * @}
 */

/** @defgroup STM32F769I_DISCOVERY_LCD_Exported_Functions LCD Exported Functions
 * @{
 */

/**
 * @brief  Initializes the DSI LCD.
 * @retval LCD state
 */
uint8_t BSP_LCD_Init(void) {
  return (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE));
}

/**
 * @brief  Initializes the DSI LCD.
 * The ititialization is done as below:
 *     - DSI PLL ititialization
 *     - DSI ititialization
 *     - LTDC ititialization
 *     - OTM8009A LCD Display IC Driver ititialization
 * @param  orientation: LCD orientation, can be LCD_ORIENTATION_PORTRAIT or
 * LCD_ORIENTATION_LANDSCAPE
 * @retval LCD state
 */
uint8_t BSP_LCD_InitEx(LCD_OrientationTypeDef orientation) {
  DSI_PLLInitTypeDef dsiPllInit;
  static RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  uint32_t LcdClock = 27429; /*!< LcdClk = 27429 kHz */
  uint16_t read_id = 0;

  uint32_t laneByteClk_kHz = 0;
  uint32_t VSA;  /*!< Vertical start active time in units of lines */
  uint32_t VBP;  /*!< Vertical Back Porch time in units of lines */
  uint32_t VFP;  /*!< Vertical Front Porch time in units of lines */
  uint32_t VACT; /*!< Vertical Active time in units of lines = imageSize Y in
                    pixels to display */
  uint32_t HSA;  /*!< Horizontal start active time in units of lcdClk */
  uint32_t HBP;  /*!< Horizontal Back Porch time in units of lcdClk */
  uint32_t HFP;  /*!< Horizontal Front Porch time in units of lcdClk */
  uint32_t HACT; /*!< Horizontal Active time in units of lcdClk = imageSize X in
                    pixels to display */

  /* Toggle Hardware Reset of the DSI LCD using
   * its XRES signal (active low) */
  BSP_LCD_Reset();

  /* Check the connected monitor */
  read_id = LCD_IO_GetID();

#if defined(USE_LCD_HDMI)
  if (read_id == ADV7533_ID) {
    return BSP_LCD_HDMIInitEx(HDMI_FORMAT_720_576);
  } else if (read_id != LCD_DSI_ID) {
    return LCD_ERROR;
  }
#else
  if (read_id != LCD_DSI_ID) {
    return LCD_ERROR;
  }
#endif /* USE_LCD_HDMI */

  /* Call first MSP Initialize only in case of first initialization
   * This will set IP blocks LTDC, DSI and DMA2D
   * - out of reset
   * - clocked
   * - NVIC IRQ related to IP blocks enabled
   */
  BSP_LCD_MspInit();

  /*************************DSI
   * Initialization***********************************/

  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init
   */
  hdsi_discovery.Instance = DSI;

  HAL_DSI_DeInit(&(hdsi_discovery));

  dsiPllInit.PLLNDIV = 100;
  dsiPllInit.PLLIDF = DSI_PLL_IN_DIV5;
  dsiPllInit.PLLODF = DSI_PLL_OUT_DIV1;
  laneByteClk_kHz = 62500; /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */

  /* Set number of Lanes */
  hdsi_discovery.Init.NumberOfLanes = DSI_TWO_DATA_LANES;

  /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
  hdsi_discovery.Init.TXEscapeCkdiv = laneByteClk_kHz / 15620;

  HAL_DSI_Init(&(hdsi_discovery), &(dsiPllInit));

  /* CRITICAL FIX: Configure DSI PHY timers for LCD mode
   * These timers control DSI signal timing and must be configured
   * even for LCD mode, not just HDMI mode. Missing configuration
   * can cause display artifacts like stripes and duplication. */
  DSI_PHY_TimerTypeDef dsiPhyInit;
  dsiPhyInit.ClockLaneHS2LPTime = 0x14;
  dsiPhyInit.ClockLaneLP2HSTime = 0x14;
  dsiPhyInit.DataLaneHS2LPTime = 0x0A;
  dsiPhyInit.DataLaneLP2HSTime = 0x0A;
  dsiPhyInit.DataLaneMaxReadTime = 0x00;
  dsiPhyInit.StopWaitTime = 0x0;
  HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &dsiPhyInit);

  /* Enable the DSI module */
  HAL_DSI_Start(&(hdsi_discovery));

  /* Enable the DSI BTW for read operations */
  HAL_DSI_ConfigFlowControl(&(hdsi_discovery), DSI_FLOW_CONTROL_BTA);

  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  Lcd_Driver_Type = Driver_Type(Lcd_Driver_Type);

  /* Stop the DSI module */
  HAL_DSI_Stop(&(hdsi_discovery));

  /* Timing parameters for all Video modes
   * Set Timing parameters of LTDC depending on its chosen orientation
   */
  if (orientation == LCD_ORIENTATION_PORTRAIT) {
    lcd_x_size = OTM8009A_480X800_WIDTH;  /* 480 */
    lcd_y_size = OTM8009A_480X800_HEIGHT; /* 800 */
  } else {
    /* lcd_orientation == LCD_ORIENTATION_LANDSCAPE */
    lcd_x_size = OTM8009A_800X480_WIDTH;  /* 800 */
    lcd_y_size = OTM8009A_800X480_HEIGHT; /* 480 */
  }

  HACT = lcd_x_size;
  VACT = lcd_y_size;

  /* Timing values differ between portrait and landscape orientations
   * For landscape, use 800x480 specific timing values (OTM8009A datasheet)
   * MB1166 kit uses 800x472 resolution, NOT 800x480!
   * CRITICAL FIX v0.1.57: Use MB1166-tested timing (HBP=34, HFP=34, VFP=34)
   * Previous v0.1.56 used ST reference (HBP=15, HFP=16) which caused distortion */
  if (Lcd_Driver_Type == LCD_CTRL_OTM8009A) {
    if (orientation == LCD_ORIENTATION_LANDSCAPE) {
      /* Landscape mode: use 800x472 MB1166-tested timings
       * Symmetric timing (HBP=34, HFP=34, VBP=34, VFP=34) produces square cells */
      VSA = OTM8009A_800X480_VSYNC;  /* = 2 */
      VBP = OTM8009A_800X480_VBP;    /* = 34 */
      VFP = OTM8009A_800X480_VFP;    /* = 34 (MB1166-tested, was 16 in v0.1.56) */
      HSA = OTM8009A_800X480_HSYNC;  /* = 1 */
      HBP = OTM8009A_800X480_HBP;    /* = 34 (MB1166-tested, was 15 in v0.1.56) */
      HFP = OTM8009A_800X480_HFP;    /* = 34 (MB1166-tested, was 16 in v0.1.56) */
    } else {
      /* Portrait mode: use 480x800 timings */
      VSA = OTM8009A_480X800_VSYNC;
      VBP = OTM8009A_480X800_VBP;
      VFP = OTM8009A_480X800_VFP;
      HSA = OTM8009A_480X800_HSYNC;
      HBP = OTM8009A_480X800_HBP;
      HFP = OTM8009A_480X800_HFP;
    }
  } else {
    VSA = NT35510_480X800_VSYNC;
    VBP = NT35510_480X800_VBP;
    VFP = NT35510_480X800_VFP;
    HSA = NT35510_480X800_HSYNC;
    HBP = NT35510_480X800_HBP;
    HFP = NT35510_480X800_HFP;
  }

  hdsivideo_handle.VirtualChannelID = LCD_Driver_ID;
  hdsivideo_handle.ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG888;
  /* CRITICAL FIX v0.1.51: Revert polarity to ACTIVE_HIGH (v0.1.50 LOW was worse)
   * v0.1.50 with ACTIVE_LOW polarity made display "significantly misaligned".
   * v0.1.47 with ACTIVE_HIGH polarity showed "slightly less" shift/slant (best result).
   * The DSI host in dsihost.c uses ACTIVE_LOW, but the video handle uses ACTIVE_HIGH
   * - this appears to be the correct configuration for OTM8009A. */
  hdsivideo_handle.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  hdsivideo_handle.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  hdsivideo_handle.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  hdsivideo_handle.Mode =
      DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
  hdsivideo_handle.NullPacketSize = 0xFFF;
  hdsivideo_handle.NumberOfChunks = 0;
  hdsivideo_handle.PacketSize = HACT; /* Value depending on display orientation
                                         choice portrait/landscape */
  /* CRITICAL FIX v0.1.64: Fix 45-degree tilt (1 pixel shift per row)
   *
   * ROOT CAUSE IDENTIFIED: BSP_LCD_LayerDefaultInit() calls HAL_LTDC_ConfigLayer()
   * which sets CFBLR pitch to (WindowX1 - WindowX0) * 3 + 3 = 2403 instead of 2400.
   * This causes each row to start 3 bytes (1 pixel) too late, creating cumulative diagonal shear.
   *
   * Also fixed: DSI timing calculation uses truncation (not ceiling) for exact LTDC match.
   * Using ceiling made DSI wait longer than LTDC expects, causing additional timing mismatch.
   *
   * For HSA=1, HBP=34, HFP=34, HACT=800:
   *   LTDC: HSync=1, HBP=34, Total=869 cycles
   *   DSI timing (using truncation for exact match):
   *     HorizontalSyncActive = (1 * 62500) / 27429 = 2
   *     HorizontalBackPorch = (34 * 62500) / 27429 = 77
   *     HorizontalLine = (869 * 62500) / 27429 = 1980
   *
   * Key insight: DSI hardware handles the fractional timing difference. The LTDC and
   * DSI clocks are not integer multiples, so exact integer timing is impossible.
   * Truncation gives the closest match without introducing cumulative timing errors. */
  uint32_t ltdc_hsync_cycles = HSA;                      /* Actual HSync cycles (HSA = 1) */
  uint32_t ltdc_hbp_cycles = HBP;                        /* Actual HBP cycles (HBP = 34) */
  uint32_t ltdc_total_cycles = HACT + HSA + HBP + HFP;  /* Total line cycles (800+1+34+34 = 869) */

  /* Use direct division (truncation) for DSI timing - closest match to LTDC timing */
  hdsivideo_handle.HorizontalSyncActive = (ltdc_hsync_cycles * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalBackPorch = (ltdc_hbp_cycles * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalLine = (ltdc_total_cycles * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.VerticalSyncActive = VSA;
  hdsivideo_handle.VerticalBackPorch = VBP;
  hdsivideo_handle.VerticalFrontPorch = VFP;
  hdsivideo_handle.VerticalActive =
      VACT; /* Value depending on display orientation choice portrait/landscape
             */

  /* Enable or disable sending LP command while streaming is active in video
   * mode */
  hdsivideo_handle.LPCommandEnable =
      DSI_LP_COMMAND_ENABLE; /* Enable sending commands in mode LP (Low Power)
                              */

  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP
   * regions */
  /* Only useful when sending LP packets is allowed while streaming is active in
   * video mode */
  hdsivideo_handle.LPLargestPacketSize = 16;

  /* Largest packet size possible to transmit in LP mode in HFP region during
   * VACT period */
  /* Only useful when sending LP packets is allowed while streaming is active in
   * video mode */
  hdsivideo_handle.LPVACTLargestPacketSize = 0;

  /* CRITICAL FIX v0.1.62: Disable LP commands during horizontal timing periods
   * LP commands during HFP/HBP can interfere with pixel data timing and cause
   * line-by-line synchronization issues. Disable these to ensure clean timing.
   * LP commands during vertical periods (VSA, VBP, VFP, VACT) are still allowed. */
  /* Specify for each region of the video frame, if the transmission of command
   * in LP mode is allowed in this region */
  /* while streaming is active in video mode */
  hdsivideo_handle.LPHorizontalFrontPorchEnable =
      DSI_LP_HFP_DISABLE; /* DISABLE: LP commands during HFP cause timing jitter */
  hdsivideo_handle.LPHorizontalBackPorchEnable =
      DSI_LP_HBP_DISABLE; /* DISABLE: LP commands during HBP cause timing jitter */
  hdsivideo_handle.LPVerticalActiveEnable =
      DSI_LP_VACT_ENABLE; /* Allow sending LP commands during VACT period */
  hdsivideo_handle.LPVerticalFrontPorchEnable =
      DSI_LP_VFP_ENABLE; /* Allow sending LP commands during VFP period */
  hdsivideo_handle.LPVerticalBackPorchEnable =
      DSI_LP_VBP_ENABLE; /* Allow sending LP commands during VBP period */
  hdsivideo_handle.LPVerticalSyncActiveEnable =
      DSI_LP_VSYNC_ENABLE; /* Allow sending LP commands during VSync = VSA
                              period */

  /* Configure DSI Video mode timings with settings set above */
  HAL_DSI_ConfigVideoMode(&(hdsi_discovery), &(hdsivideo_handle));

  /*************************End DSI
   * Initialization*******************************/

  /************************LTDC
   * Initialization***********************************/

  /* CRITICAL FIX: Timing Configuration - BOTH horizontal and vertical timing must be set
   * Previously only horizontal timing was configured, leaving vertical timing at
   * default values which caused display artifacts (stripe, duplication, etc.) */
  /* Horizontal timing configuration - may be overridden by HAL_LTDC_StructInitFromVideoConfig below */
  hltdc_discovery.Init.HorizontalSync = (HSA - 1);
  hltdc_discovery.Init.AccumulatedHBP = (HSA + HBP - 1);
  hltdc_discovery.Init.AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
  hltdc_discovery.Init.TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);

  /* Vertical timing placeholder - will be set AFTER HAL_LTDC_StructInitFromVideoConfig
   * to prevent being overwritten by that function */

  /* CRITICAL FIX: Configure Layer 0 with proper window and framebuffer settings
   * Layer 0 must be fully configured BEFORE HAL_LTDC_Init() is called.
   * The window positions define where on the screen the layer's content appears.
   * For full-screen display, window spans from (0,0) to (lcd_x_size, lcd_y_size). */
  hltdc_discovery.LayerCfg[0].WindowX0 = 0;
  hltdc_discovery.LayerCfg[0].WindowX1 = lcd_x_size;  /* 800 for landscape */
  hltdc_discovery.LayerCfg[0].WindowY0 = 0;
  hltdc_discovery.LayerCfg[0].WindowY1 = lcd_y_size;  /* 480 for landscape */
  hltdc_discovery.LayerCfg[0].ImageWidth = lcd_x_size;
  hltdc_discovery.LayerCfg[0].ImageHeight = lcd_y_size;
  hltdc_discovery.LayerCfg[0].PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  hltdc_discovery.LayerCfg[0].FBStartAdress = LCD_FB_START_ADDRESS;
  hltdc_discovery.LayerCfg[0].Alpha = 255;   /* Fully opaque */
  hltdc_discovery.LayerCfg[0].Alpha0 = 0;
  hltdc_discovery.LayerCfg[0].BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  hltdc_discovery.LayerCfg[0].BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  hltdc_discovery.LayerCfg[0].Backcolor.Blue = 0;
  hltdc_discovery.LayerCfg[0].Backcolor.Green = 0;
  hltdc_discovery.LayerCfg[0].Backcolor.Red = 0;

  /* CRITICAL FIX: Configure Layer 1 to use a DIFFERENT framebuffer address
   * Layer 1 was pointing to the SAME framebuffer as Layer 0, which causes LTDC
   * hardware conflicts and display artifacts. Point Layer 1 to an unused region
   * after Layer 0's framebuffer. Layer 1 is also disabled below. */
  hltdc_discovery.LayerCfg[1].WindowX0 = 0;
  hltdc_discovery.LayerCfg[1].WindowX1 = 0;  /* Zero width window - no content visible */
  hltdc_discovery.LayerCfg[1].WindowY0 = 0;
  hltdc_discovery.LayerCfg[1].WindowY1 = 0;  /* Zero height window - no content visible */
  hltdc_discovery.LayerCfg[1].ImageWidth = lcd_x_size;
  hltdc_discovery.LayerCfg[1].ImageHeight = lcd_y_size;
  hltdc_discovery.LayerCfg[1].PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  /* Point Layer 1 to a different framebuffer region to avoid conflicts with Layer 0.
   * CRITICAL FIX v0.1.46: Use 3 bytes per pixel for RGB888 format.
   * For 800x472 landscape, this is 800 * 472 * 3 = 1,133,600 bytes.
   * This ensures Layer 1 won't interfere with Layer 0 even if accidentally enabled. */
  hltdc_discovery.LayerCfg[1].FBStartAdress = LCD_FB_START_ADDRESS + (lcd_x_size * lcd_y_size * 3);
  hltdc_discovery.LayerCfg[1].Alpha = 0;    /* Fully transparent - layer won't be visible */
  hltdc_discovery.LayerCfg[1].Alpha0 = 0;
  hltdc_discovery.LayerCfg[1].BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  hltdc_discovery.LayerCfg[1].BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  hltdc_discovery.LayerCfg[1].Backcolor.Blue = 0;
  hltdc_discovery.LayerCfg[1].Backcolor.Green = 0;
  hltdc_discovery.LayerCfg[1].Backcolor.Red = 0;

  /** LCD clock configuration
   * Note: The following values should not be changed as the PLLSAI is also used
   *      to clock the USB FS
   * PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz
   * PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 384 Mhz
   * PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 384 MHz / 7 = 54.85 MHz
   * LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 54.85 MHz / 2
   * = 27.429 MHz
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Background value */
  hltdc_discovery.Init.Backcolor.Blue = 0;
  hltdc_discovery.Init.Backcolor.Green = 0;
  hltdc_discovery.Init.Backcolor.Red = 0;
  /* CRITICAL FIX v0.1.55: Revert to ST reference PCPOLARITY setting
   * ST's F469I-Discovery and F769I-EVAL reference code use LTDC_PCPOLARITY_IPC (not inverted)
   * v0.1.54 tried IIPC (inverted) but it didn't fix the distortion.
   * IPC is the correct setting for OTM8009A in DSI configuration.
   * Reference: github/STM32F469I-Discovery/Core/Src/main.c line 482 */
  hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IPC;  /* Not inverted - ST reference setting */
  hltdc_discovery.Instance = LTDC;

  /* Get LTDC Configuration from DSI Configuration */
  HAL_LTDC_StructInitFromVideoConfig(&(hltdc_discovery), &(hdsivideo_handle));

  /* CRITICAL FIX v0.1.47: Configure BOTH horizontal and vertical timing AFTER HAL_LTDC_StructInitFromVideoConfig
   * The HAL_LTDC_StructInitFromVideoConfig function above copies timing from DSI config to LTDC,
   * but it may not properly match the exact timing values we need. We must set them AFTER this call
   * to prevent being overwritten and to ensure exact timing alignment. These values are critical
   * for eliminating display shift/slant artifacts. */
  /* Horizontal timing - must be set AFTER StructInitFromVideoConfig */
  hltdc_discovery.Init.HorizontalSync = (HSA - 1);
  hltdc_discovery.Init.AccumulatedHBP = (HSA + HBP - 1);
  hltdc_discovery.Init.AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
  hltdc_discovery.Init.TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);
  /* Vertical timing - must be set AFTER StructInitFromVideoConfig */
  hltdc_discovery.Init.VerticalSync = (VSA - 1);
  hltdc_discovery.Init.AccumulatedVBP = (VSA + VBP - 1);
  hltdc_discovery.Init.AccumulatedActiveH = (lcd_y_size + VSA + VBP - 1);
  hltdc_discovery.Init.TotalHeigh = (lcd_y_size + VSA + VBP + VFP - 1);

  /* Initialize the LTDC */
  HAL_LTDC_Init(&hltdc_discovery);

  /* CRITICAL FIX: Configure Layer 1 in hardware and disable it
   * Even though we initialized LayerCfg[1] above, HAL_LTDC_Init doesn't
   * configure the layers in hardware - we need to call HAL_LTDC_ConfigLayer.
   * Configure it with the pre-initialized settings (Alpha=0 for transparency) */
  HAL_LTDC_ConfigLayer(&hltdc_discovery, &hltdc_discovery.LayerCfg[1], 1);

  /* Explicitly disable Layer 1 to prevent vertical strip artifact */
  __HAL_LTDC_LAYER_DISABLE(&hltdc_discovery, 1);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);

  /* CRITICAL FIX: Correct CFBLR Pitch value for RGB888 format
   * The HAL_LTDC_ConfigLayer() adds +3 for alignment, but for RGB888 with 800 pixel width:
   * - LineLength = 800 * 3 = 2400 (correct, already divisible by 4)
   * - Pitch = 800 * 3 + 3 = 2403 (WRONG - causes display skew/shift!)
   * The correct pitch should be 2400, not 2403. Fix this by writing the correct value. */
  if (hltdc_discovery.LayerCfg[0].PixelFormat == LTDC_PIXEL_FORMAT_RGB888) {
    uint32_t line_length = hltdc_discovery.LayerCfg[0].ImageWidth * 3;  /* RGB888 = 3 bytes per pixel */
    /* CFBLR format: [31:16] = CFBLL (line length), [12:0] = CFBP (pitch in bytes) */
    LTDC_LAYER(&hltdc_discovery, 0)->CFBLR = ((line_length << 16U) | line_length);
    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);
  }

  /* Enable the DSI host and wrapper after the LTDC initialization
     To avoid any synchronization issue, the DSI shall be started after enabling
     the LTDC */
  HAL_DSI_Start(&hdsi_discovery);

#if !defined(DATA_IN_ExtSDRAM)
  /* Initialize the SDRAM */
  BSP_SDRAM_Init();
#endif /* DATA_IN_ExtSDRAM */

  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /************************End LTDC
   * Initialization*******************************/

  if (Lcd_Driver_Type == LCD_CTRL_NT35510) {
    /***********************NT35510
     * Initialization********************************/

    /* Initialize the NT35510 LCD Display IC Driver (TechShine LCD IC Driver)
     * depending on configuration set in 'hdsivideo_handle'.
     */
    NT35510_Init(NT35510_FORMAT_RGB888, orientation);
  }
  /***********************End NT35510
     Initialization****************************/
  else {
    /***********************OTM8009A
     * Initialization********************************/

    /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
     *  depending on configuration set in 'hdsivideo_handle'.
     */
    OTM8009A_Init(OTM8009A_FORMAT_RGB888, orientation);

    /***********************End OTM8009A
     * Initialization****************************/
  }

  /* CRITICAL FIX: Re-configure DSI video mode after panel initialization
   * to ensure proper timing synchronization between DSI and LTDC. */
  HAL_DSI_ConfigVideoMode(&(hdsi_discovery), &(hdsivideo_handle));

  /* CRITICAL FIX: Ensure Layer 1 is disabled after panel initialization
   * The panel init may have affected the LTDC layer configuration.
   * Explicitly disable Layer 1 again and reload configuration. */
  __HAL_LTDC_LAYER_DISABLE(&hltdc_discovery, 1);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);

  return LCD_OK;
}

#if defined(USE_LCD_HDMI)
/**
 * @brief  Initializes the DSI for HDMI monitor.
 * The ititialization is done as below:
 *     - DSI PLL ititialization
 *     - DSI ititialization
 *     - LTDC ititialization
 *     - DSI-HDMI ADV7533 adapter device ititialization
 * @param  format : HDMI format could be HDMI_FORMAT_720_480 or
 * HDMI_FORMAT_720_576
 * @retval LCD state
 */
uint8_t BSP_LCD_HDMIInitEx(uint8_t format) {
  /************************ADV7533
   * Initialization********************************/

  /* Initialize the ADV7533 HDMI Bridge
   *  depending on configuration set in 'hdsivideo_handle'.
   */
  adv7533ConfigTypeDef adv7533_config;

  adv7533_config.DSI_LANES = 2;
  adv7533_config.HACT = HDMI_Format[format].HACT;
  adv7533_config.HSYNC = HDMI_Format[format].HSYNC;
  adv7533_config.HBP = HDMI_Format[format].HBP;
  adv7533_config.HFP = HDMI_Format[format].HFP;
  adv7533_config.VACT = HDMI_Format[format].VACT;
  adv7533_config.VSYNC = HDMI_Format[format].VSYNC;
  adv7533_config.VBP = HDMI_Format[format].VBP;
  adv7533_config.VFP = HDMI_Format[format].VFP;

  ADV7533_Init();
  ADV7533_Configure(&adv7533_config);
  ADV7533_PowerOn();

  /************************ Update hdmi_x_size and hdmi_y_size
   * *****************/
  lcd_x_size = HDMI_Format[format].HACT;
  lcd_y_size = HDMI_Format[format].VACT;

  /***********************End ADV7533
   * Initialization****************************/
  DSI_PLLInitTypeDef dsiPllInit;
  DSI_PHY_TimerTypeDef dsiPhyInit;
  static RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Call first MSP Initialize only in case of first initialization
   * This will set IP blocks LTDC and DSI
   * - out of reset
   * - clocked
   * - NVIC IRQ related to IP blocks enabled
   */
  BSP_LCD_MspInit();

  /*************************DSI
   * Initialization***********************************/

  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init
   */
  hdsi_discovery.Instance = DSI;

  HAL_DSI_DeInit(&(hdsi_discovery));

  /* Configure the DSI PLL */
  dsiPllInit.PLLNDIV = HDMI_PLLConfig[format].NDIV;
  dsiPllInit.PLLIDF = HDMI_PLLConfig[format].IDF;
  dsiPllInit.PLLODF = HDMI_PLLConfig[format].ODF;

  /* Set number of Lanes */
  hdsi_discovery.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  /* Set the TX escape clock division ratio */
  hdsi_discovery.Init.TXEscapeCkdiv = HDMI_PLLConfig[format].TXEscapeCkdiv;
  /* Disable the automatic clock lane control (the ADV7533 must be clocked) */
  hdsi_discovery.Init.AutomaticClockLaneControl =
      DSI_AUTO_CLK_LANE_CTRL_DISABLE;

  /* Init the DSI */
  HAL_DSI_Init(&hdsi_discovery, &dsiPllInit);

  /* Configure the D-PHY Timings */
  dsiPhyInit.ClockLaneHS2LPTime = 0x14;
  dsiPhyInit.ClockLaneLP2HSTime = 0x14;
  dsiPhyInit.DataLaneHS2LPTime = 0x0A;
  dsiPhyInit.DataLaneLP2HSTime = 0x0A;
  dsiPhyInit.DataLaneMaxReadTime = 0x00;
  dsiPhyInit.StopWaitTime = 0x0;
  HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &dsiPhyInit);

  /* Virutal channel used by the ADV7533 */
  hdsivideo_handle.VirtualChannelID = HDMI_ADV7533_ID;

  /* Timing parameters for Video modes
     Set Timing parameters of DSI depending on its chosen format */
  hdsivideo_handle.ColorCoding = HDMI_Format[format].RGB_CODING;
  hdsivideo_handle.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  hdsivideo_handle.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  hdsivideo_handle.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  hdsivideo_handle.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  hdsivideo_handle.Mode = DSI_VID_MODE_NB_PULSES;
  hdsivideo_handle.NullPacketSize = HDMI_DSIPacket[format].NullPacketSize;
  hdsivideo_handle.NumberOfChunks = HDMI_DSIPacket[format].NumberOfChunks;
  hdsivideo_handle.PacketSize = HDMI_DSIPacket[format].PacketSize;
  hdsivideo_handle.HorizontalSyncActive = HDMI_Format[format].HSYNC *
                                          HDMI_PLLConfig[format].LaneByteClock /
                                          HDMI_PLLConfig[format].PCLK;
  hdsivideo_handle.HorizontalBackPorch = HDMI_Format[format].HBP *
                                         HDMI_PLLConfig[format].LaneByteClock /
                                         HDMI_PLLConfig[format].PCLK;
  hdsivideo_handle.HorizontalLine =
      (HDMI_Format[format].HACT + HDMI_Format[format].HSYNC +
       HDMI_Format[format].HBP + HDMI_Format[format].HFP) *
      HDMI_PLLConfig[format].LaneByteClock / HDMI_PLLConfig[format].PCLK;
  hdsivideo_handle.VerticalSyncActive = HDMI_Format[format].VSYNC;
  hdsivideo_handle.VerticalBackPorch = HDMI_Format[format].VBP;
  hdsivideo_handle.VerticalFrontPorch = HDMI_Format[format].VFP;
  hdsivideo_handle.VerticalActive = HDMI_Format[format].VACT;

  /* Enable or disable sending LP command while streaming is active in video
   * mode */
  hdsivideo_handle.LPCommandEnable =
      DSI_LP_COMMAND_DISABLE; /* Enable sending commands in mode LP (Low Power)
                               */

  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP
   * regions */
  /* Only useful when sending LP packets is allowed while streaming is active in
   * video mode */
  hdsivideo_handle.LPLargestPacketSize = 4;

  /* Largest packet size possible to transmit in LP mode in HFP region during
   * VACT period */
  /* Only useful when sending LP packets is allowed while streaming is active in
   * video mode */
  hdsivideo_handle.LPVACTLargestPacketSize = 4;

  /* Specify for each region, if the going in LP mode is allowed */
  /* while streaming is active in video mode                     */
  hdsivideo_handle.LPHorizontalFrontPorchEnable = DSI_LP_HFP_DISABLE;
  hdsivideo_handle.LPHorizontalBackPorchEnable = DSI_LP_HBP_DISABLE;
  hdsivideo_handle.LPVerticalActiveEnable = DSI_LP_VACT_DISABLE;
  hdsivideo_handle.LPVerticalFrontPorchEnable = DSI_LP_VFP_DISABLE;
  hdsivideo_handle.LPVerticalBackPorchEnable = DSI_LP_VBP_DISABLE;
  hdsivideo_handle.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_DISABLE;

  /* No acknoledge at the end of a frame */
  hdsivideo_handle.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;

  /* Configure DSI Video mode timings with settings set above */
  HAL_DSI_ConfigVideoMode(&hdsi_discovery, &hdsivideo_handle);

  /* Enable the DSI host and wrapper : but LTDC is not started yet at this stage
   */
  HAL_DSI_Start(&hdsi_discovery);

  /*************************End DSI
   * Initialization*******************************/

  /************************LTDC
   * Initialization***********************************/

  /* LTDC clock configuration */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = HDMI_PLLConfig[format].PLLSAIN;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = HDMI_PLLConfig[format].PLLSAIR;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Base address of LTDC registers to be set before calling De-Init */
  hltdc_discovery.Instance = LTDC;

  HAL_LTDC_DeInit(&(hltdc_discovery));

  /* Timing Configuration */
  hltdc_discovery.Init.HorizontalSync = (HDMI_Format[format].HSYNC - 1);
  hltdc_discovery.Init.AccumulatedHBP =
      (HDMI_Format[format].HSYNC + HDMI_Format[format].HBP - 1);
  hltdc_discovery.Init.AccumulatedActiveW =
      (HDMI_Format[format].HACT + HDMI_Format[format].HSYNC +
       HDMI_Format[format].HBP - 1);
  hltdc_discovery.Init.TotalWidth =
      (HDMI_Format[format].HACT + HDMI_Format[format].HSYNC +
       HDMI_Format[format].HBP + HDMI_Format[format].HFP - 1);
  hltdc_discovery.Init.VerticalSync = (HDMI_Format[format].VSYNC - 1);
  hltdc_discovery.Init.AccumulatedVBP =
      (HDMI_Format[format].VSYNC + HDMI_Format[format].VBP - 1);
  hltdc_discovery.Init.AccumulatedActiveH =
      (HDMI_Format[format].VACT + HDMI_Format[format].VSYNC +
       HDMI_Format[format].VBP - 1);
  hltdc_discovery.Init.TotalHeigh =
      (HDMI_Format[format].VACT + HDMI_Format[format].VSYNC +
       HDMI_Format[format].VBP + HDMI_Format[format].VFP - 1);

  /* background value */
  hltdc_discovery.Init.Backcolor.Blue = 0x00;
  hltdc_discovery.Init.Backcolor.Green = 0xFF;
  hltdc_discovery.Init.Backcolor.Red = 0xFF;

  /* Polarity */
  hltdc_discovery.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc_discovery.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc_discovery.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  /* Initialize & Start the LTDC */
  HAL_LTDC_Init(&hltdc_discovery);

#if !defined(DATA_IN_ExtSDRAM)
  /* Initialize the SDRAM */
  BSP_SDRAM_Init();
#endif /* DATA_IN_ExtSDRAM */

  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  /************************End LTDC
   * Initialization*******************************/

  return LCD_OK;
}
#endif /* USE_LCD_HDMI */

/**
 * @brief  BSP LCD Reset
 *         Hw reset the LCD DSI activating its XRES signal (active low for some
 * time) and desactivating it later.
 */
void BSP_LCD_Reset(void) {
  GPIO_InitTypeDef gpio_init_structure;

  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /* Configure the GPIO on PJ15 */
  gpio_init_structure.Pin = GPIO_PIN_15;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOJ, &gpio_init_structure);

  /* Activate XRES active low */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_15, GPIO_PIN_RESET);

  HAL_Delay(20); /* wait 20 ms */

  /* Desactivate XRES */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_15, GPIO_PIN_SET);

  /* Wait for 10ms after releasing XRES before sending commands */
  HAL_Delay(10);
}

/**
 * @brief  Gets the LCD X size.
 * @retval Used LCD X size
 */
uint32_t BSP_LCD_GetXSize(void) { return (lcd_x_size); }

/**
 * @brief  Gets the LCD Y size.
 * @retval Used LCD Y size
 */
uint32_t BSP_LCD_GetYSize(void) { return (lcd_y_size); }

/**
 * @brief  Set the LCD X size.
 * @param  imageWidthPixels : uint32_t image width in pixels unit
 * @retval None
 */
void BSP_LCD_SetXSize(uint32_t imageWidthPixels) {
  hltdc_discovery.LayerCfg[ActiveLayer].ImageWidth = imageWidthPixels;
}

/**
 * @brief  Set the LCD Y size.
 * @param  imageHeightPixels : uint32_t image height in lines unit
 */
void BSP_LCD_SetYSize(uint32_t imageHeightPixels) {
  hltdc_discovery.LayerCfg[ActiveLayer].ImageHeight = imageHeightPixels;
}

/**
 * @brief  Initializes the LCD layers.
 * @param  LayerIndex: Layer foreground or background
 * @param  FB_Address: Layer frame buffer
 * @retval None
 */
void BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address) {
  LCD_LayerCfgTypeDef Layercfg;

  /* Layer Init */
  Layercfg.WindowX0 = 0;
  Layercfg.WindowX1 = BSP_LCD_GetXSize();
  Layercfg.WindowY0 = 0;
  Layercfg.WindowY1 = BSP_LCD_GetYSize();
  Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  Layercfg.FBStartAdress = FB_Address;
  Layercfg.Alpha = 255;
  Layercfg.Alpha0 = 0;
  Layercfg.Backcolor.Blue = 0;
  Layercfg.Backcolor.Green = 0;
  Layercfg.Backcolor.Red = 0;
  Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  Layercfg.ImageWidth = BSP_LCD_GetXSize();
  Layercfg.ImageHeight = BSP_LCD_GetYSize();

  /* CRITICAL FIX v0.1.66: Fix BOTH CFBLR pitch AND Window Position issues!
   *
   * ISSUE 1 - CFBLR Pitch: HAL_LTDC_ConfigLayer() sets pitch to (WindowX1-WindowX0)*3+3 = 2403
   * instead of 2400 for RGB888, causing 1-pixel-per-row diagonal shift.
   *
   * ISSUE 2 - Window Position: HAL_LTDC_ConfigLayer() ADDS AHBP (35) to window positions!
   * With WindowX0=0, WindowX1=800, AHBP=35:
   *   - Window starts at pixel 36 instead of 0
   *   - Window ends at pixel 835 instead of 800
   * This causes the LTDC to read from the wrong framebuffer position.
   *
   * SOLUTION: Temporarily set BPCR AHBP to 0 during HAL_LTDC_ConfigLayer() so it
   * calculates window positions correctly, then restore BPCR. */
  uint32_t bpcr_backup = hltdc_discovery.Instance->BPCR;  // Save original BPCR
  hltdc_discovery.Instance->BPCR &= ~(LTDC_BPCR_AHBP);    // Clear AHBP (set to 0)

  HAL_LTDC_ConfigLayer(&hltdc_discovery, &Layercfg, LayerIndex);

  hltdc_discovery.Instance->BPCR = bpcr_backup;  // Restore original BPCR

  /* Now fix CFBLR pitch for RGB888 format */
  if (Layercfg.PixelFormat == LTDC_PIXEL_FORMAT_RGB888) {
    uint32_t line_length = Layercfg.ImageWidth * 3;  /* 800 * 3 = 2400 */
    uint32_t cfblr_before = LTDC_LAYER(&hltdc_discovery, LayerIndex)->CFBLR;
    uint32_t whpcr_before = LTDC_LAYER(&hltdc_discovery, LayerIndex)->WHPCR;
    uint32_t wvpcr_before = LTDC_LAYER(&hltdc_discovery, LayerIndex)->WVPCR;

    LTDC_LAYER(&hltdc_discovery, LayerIndex)->CFBLR = ((line_length << 16U) | line_length);
    uint32_t cfblr_after = LTDC_LAYER(&hltdc_discovery, LayerIndex)->CFBLR;
    uint32_t whpcr_after = LTDC_LAYER(&hltdc_discovery, LayerIndex)->WHPCR;
    uint32_t wvpcr_after = LTDC_LAYER(&hltdc_discovery, LayerIndex)->WVPCR;
    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc_discovery);

    /* Diagnostic output: verify both fixes were applied correctly
     * WHPCR format: [26:16]=StopPos, [10:0]=StartPos
     * For 800 pixels: StopPos=799 (0x31F), StartPos=0 => 0x031F0001
     * WVPCR format: [26:16]=StopPos, [10:0]=StartPos
     * For 472 pixels: StopPos=471, StartPos=0 */
    extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
    char diag_buf[256];
    uint32_t wh_start = whpcr_after & 0x1FFF;
    uint32_t wh_stop = (whpcr_after >> 16) & 0x1FFF;
    uint32_t wv_start = wvpcr_after & 0x1FFF;
    uint32_t wv_stop = (wvpcr_after >> 16) & 0x1FFF;
    snprintf(diag_buf, sizeof(diag_buf),
             "[BSP_LCD_LayerDefaultInit L%u] W=%lu H=%lu\r\n"
             "  CFBLR: 0x%08lx->0x%08lx (LineLen=%lu,Pitch=%lu)\r\n"
             "  WHPCR: 0x%08lx (X0=%lu,X1=%lu) Expected=0x031F0001\r\n"
             "  WVPCR: 0x%08lx (Y0=%lu,Y1=%lu)\r\n",
             LayerIndex, Layercfg.ImageWidth, Layercfg.ImageHeight,
             cfblr_before, cfblr_after, (cfblr_after>>16)&0x1FFF, cfblr_after&0x1FFF,
             whpcr_after, wh_start, wh_stop,
             wvpcr_after, wv_start, wv_stop);
    CDC_Transmit_HS((uint8_t *)diag_buf, strlen(diag_buf));
  }

  DrawProp[LayerIndex].BackColor = LCD_COLOR_WHITE;
  DrawProp[LayerIndex].pFont = &Font24;
  DrawProp[LayerIndex].TextColor = LCD_COLOR_BLACK;
}

/**
 * @brief  Selects the LCD Layer.
 * @param  LayerIndex: Layer foreground or background
 */
void BSP_LCD_SelectLayer(uint32_t LayerIndex) { ActiveLayer = LayerIndex; }

/**
 * @brief  Sets an LCD Layer visible
 * @param  LayerIndex: Visible Layer
 * @param  State: New state of the specified layer
 *          This parameter can be one of the following values:
 *            @arg  ENABLE
 *            @arg  DISABLE
 */
void BSP_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State) {
  if (State == ENABLE) {
    __HAL_LTDC_LAYER_ENABLE(&(hltdc_discovery), LayerIndex);
  } else {
    __HAL_LTDC_LAYER_DISABLE(&(hltdc_discovery), LayerIndex);
  }
  __HAL_LTDC_RELOAD_CONFIG(&(hltdc_discovery));
}

/**
 * @brief  Configures the transparency.
 * @param  LayerIndex: Layer foreground or background.
 * @param  Transparency: Transparency
 *           This parameter must be a number between Min_Data = 0x00 and
 * Max_Data = 0xFF
 */
void BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency) {

  HAL_LTDC_SetAlpha(&(hltdc_discovery), Transparency, LayerIndex);
}

/**
 * @brief  Sets an LCD layer frame buffer address.
 * @param  LayerIndex: Layer foreground or background
 * @param  Address: New LCD frame buffer value
 */
void BSP_LCD_SetLayerAddress(uint32_t LayerIndex, uint32_t Address) {

  HAL_LTDC_SetAddress(&(hltdc_discovery), Address, LayerIndex);
}

/**
 * @brief  Sets display window.
 * @param  LayerIndex: Layer index
 * @param  Xpos: LCD X position
 * @param  Ypos: LCD Y position
 * @param  Width: LCD window width
 * @param  Height: LCD window height
 */
void BSP_LCD_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos,
                            uint16_t Width, uint16_t Height) {
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&(hltdc_discovery), Width, Height, LayerIndex);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&(hltdc_discovery), Xpos, Ypos, LayerIndex);
}

/**
 * @brief  Configures and sets the color keying.
 * @param  LayerIndex: Layer foreground or background
 * @param  RGBValue: Color reference
 */
void BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue) {
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying(&(hltdc_discovery), RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying(&(hltdc_discovery), LayerIndex);
}

/**
 * @brief  Disables the color keying.
 * @param  LayerIndex: Layer foreground or background
 */
void BSP_LCD_ResetColorKeying(uint32_t LayerIndex) {
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying(&(hltdc_discovery), LayerIndex);
}

/**
 * @brief  Sets the LCD text color.
 * @param  Color: Text color code ARGB(8-8-8-8)
 */
void BSP_LCD_SetTextColor(uint32_t Color) {
  DrawProp[ActiveLayer].TextColor = Color;
}

/**
 * @brief  Gets the LCD text color.
 * @retval Used text color.
 */
uint32_t BSP_LCD_GetTextColor(void) { return DrawProp[ActiveLayer].TextColor; }

/**
 * @brief  Sets the LCD background color.
 * @param  Color: Layer background color code ARGB(8-8-8-8)
 */
void BSP_LCD_SetBackColor(uint32_t Color) {
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
 * @brief  Gets the LCD background color.
 * @retval Used background color
 */
uint32_t BSP_LCD_GetBackColor(void) { return DrawProp[ActiveLayer].BackColor; }

/**
 * @brief  Sets the LCD text font.
 * @param  fonts: Layer font to be used
 */
void BSP_LCD_SetFont(sFONT *fonts) { DrawProp[ActiveLayer].pFont = fonts; }

/**
 * @brief  Gets the LCD text font.
 * @retval Used layer font
 */
sFONT *BSP_LCD_GetFont(void) { return DrawProp[ActiveLayer].pFont; }

/**
 * @brief  Reads an LCD pixel.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @retval RGB pixel color
 */
uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos) {
  uint32_t ret = 0;

  if (hltdc_discovery.LayerCfg[ActiveLayer].PixelFormat ==
      LTDC_PIXEL_FORMAT_ARGB8888) {
    /* Read data value from SDRAM memory */
    ret =
        *(__IO uint32_t *)(hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress +
                           (4 * (Ypos * BSP_LCD_GetXSize() + Xpos)));
  } else if (hltdc_discovery.LayerCfg[ActiveLayer].PixelFormat ==
             LTDC_PIXEL_FORMAT_RGB888) {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t *)(hltdc_discovery.LayerCfg[ActiveLayer]
                                  .FBStartAdress +
                              (4 * (Ypos * BSP_LCD_GetXSize() + Xpos))) &
           0x00FFFFFF);
  } else if ((hltdc_discovery.LayerCfg[ActiveLayer].PixelFormat ==
              LTDC_PIXEL_FORMAT_RGB565) ||
             (hltdc_discovery.LayerCfg[ActiveLayer].PixelFormat ==
              LTDC_PIXEL_FORMAT_ARGB4444) ||
             (hltdc_discovery.LayerCfg[ActiveLayer].PixelFormat ==
              LTDC_PIXEL_FORMAT_AL88)) {
    /* Read data value from SDRAM memory */
    ret =
        *(__IO uint16_t *)(hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress +
                           (2 * (Ypos * BSP_LCD_GetXSize() + Xpos)));
  } else {
    /* Read data value from SDRAM memory */
    ret =
        *(__IO uint8_t *)(hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress +
                          (2 * (Ypos * BSP_LCD_GetXSize() + Xpos)));
  }

  return ret;
}

/**
 * @brief  Clears the whole currently active layer of LTDC.
 * @param  Color: Color of the background
 */
void BSP_LCD_Clear(uint32_t Color) {
  /* Clear the LCD */
  LL_FillBuffer(
      ActiveLayer,
      (uint32_t *)(hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress),
      BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), 0, Color);
}

/**
 * @brief  Clears the selected line in currently active layer.
 * @param  Line: Line to be cleared
 */
void BSP_LCD_ClearStringLine(uint32_t Line) {
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;

  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height),
                   BSP_LCD_GetXSize(), DrawProp[ActiveLayer].pFont->Height);

  DrawProp[ActiveLayer].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
}

/**
 * @brief  Displays one character in currently active layer.
 * @param  Xpos: Start column address
 * @param  Ypos: Line where to display the character shape.
 * @param  Ascii: Character ascii code
 *           This parameter must be a number between Min_Data = 0x20 and
 * Max_Data = 0x7E
 */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii) {
  DrawChar(
      Xpos, Ypos,
      &DrawProp[ActiveLayer]
           .pFont->table[(Ascii - ' ') * DrawProp[ActiveLayer].pFont->Height *
                         ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
}

/**
 * @brief  Displays characters in currently active layer.
 * @param  Xpos: X position (in pixel)
 * @param  Ypos: Y position (in pixel)
 * @param  Text: Pointer to string to display on LCD
 * @param  Mode: Display mode
 *          This parameter can be one of the following values:
 *            @arg  CENTER_MODE
 *            @arg  RIGHT_MODE
 *            @arg  LEFT_MODE
 */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text,
                             Text_AlignModeTypdef Mode) {
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t *ptr = Text;

  /* Get the text size */
  while (*ptr++)
    size++;

  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize() / DrawProp[ActiveLayer].pFont->Width);

  switch (Mode) {
  case CENTER_MODE: {
    refcolumn =
        Xpos + ((xsize - size) * DrawProp[ActiveLayer].pFont->Width) / 2;
    break;
  }
  case LEFT_MODE: {
    refcolumn = Xpos;
    break;
  }
  case RIGHT_MODE: {
    refcolumn = -Xpos + ((xsize - size) * DrawProp[ActiveLayer].pFont->Width);
    break;
  }
  default: {
    refcolumn = Xpos;
    break;
  }
  }

  /* Check that the Start column is located in the screen */
  if ((refcolumn < 1) || (refcolumn >= 0x8000)) {
    refcolumn = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) &
         (((BSP_LCD_GetXSize() - (i * DrawProp[ActiveLayer].pFont->Width)) &
           0xFFFF) >= DrawProp[ActiveLayer].pFont->Width)) {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp[ActiveLayer].pFont->Width;

    /* Point on the next character */
    Text++;
    i++;
  }
}

/**
 * @brief  Displays a maximum of 60 characters on the LCD.
 * @param  Line: Line where to display the character shape
 * @param  ptr: Pointer to string to display on LCD
 */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr) {
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
 * @brief  Draws an horizontal line in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Length: Line length
 */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) {
  uint32_t Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) +
             4 * (BSP_LCD_GetXSize() * Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, 0,
                DrawProp[ActiveLayer].TextColor);
}

/**
 * @brief  Draws a vertical line in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Length: Line length
 */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) {
  uint32_t Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) +
             4 * (BSP_LCD_GetXSize() * Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, Length,
                (BSP_LCD_GetXSize() - 1), DrawProp[ActiveLayer].TextColor);
}

/**
 * @brief  Draws an uni-line (between two points) in currently active layer.
 * @param  x1: Point 1 X position
 * @param  y1: Point 1 Y position
 * @param  x2: Point 2 X position
 * @param  y2: Point 2 Y position
 */
void BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, yinc1 = 0,
          yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, curpixel = 0;

  deltax = ABS(x2 - x1); /* The difference between the x's */
  deltay = ABS(y2 - y1); /* The difference between the y's */
  x = x1;                /* Start x off at the first pixel */
  y = y1;                /* Start y off at the first pixel */

  if (x2 >= x1) /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  } else /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1) /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  } else /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay) /* There is at least one x-value for every y-value */
  {
    xinc1 = 0; /* Don't change the x when numerator >= denominator */
    yinc2 = 0; /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax; /* There are more x-values than y-values */
  } else                /* There is at least one y-value for every x-value */
  {
    xinc2 = 0; /* Don't change the x for every iteration */
    yinc1 = 0; /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay; /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++) {
    BSP_LCD_DrawPixel(
        x, y, DrawProp[ActiveLayer].TextColor); /* Draw the current pixel */
    num += numadd;  /* Increase the numerator by the top of the fraction */
    if (num >= den) /* Check if numerator >= denominator */
    {
      num -= den; /* Calculate the new numerator value */
      x += xinc1; /* Change the x as appropriate */
      y += yinc1; /* Change the y as appropriate */
    }
    x += xinc2; /* Change the x as appropriate */
    y += yinc2; /* Change the y as appropriate */
  }
}

/**
 * @brief  Draws a rectangle in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Width: Rectangle width
 * @param  Height: Rectangle height
 */
void BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width,
                      uint16_t Height) {
  /* Draw horizontal lines */
  BSP_LCD_DrawHLine(Xpos, Ypos, Width);
  BSP_LCD_DrawHLine(Xpos, (Ypos + Height), Width);

  /* Draw vertical lines */
  BSP_LCD_DrawVLine(Xpos, Ypos, Height);
  BSP_LCD_DrawVLine((Xpos + Width), Ypos, Height);
}

/**
 * @brief  Draws a circle in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Radius: Circle radius
 */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius) {
  int32_t D;     /* Decision Variable */
  uint32_t CurX; /* Current X Value */
  uint32_t CurY; /* Current Y Value */

  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY) {
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos - CurY),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos - CurY),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos - CurX),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos - CurX),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos + CurY),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos + CurY),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos + CurX),
                      DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos + CurX),
                      DrawProp[ActiveLayer].TextColor);

    if (D < 0) {
      D += (CurX << 2) + 6;
    } else {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
 * @brief  Draws an poly-line (between many points) in currently active layer.
 * @param  Points: Pointer to the points array
 * @param  PointCount: Number of points
 */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount) {
  int16_t X = 0, Y = 0;

  if (PointCount < 2) {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points + PointCount - 1)->X,
                   (Points + PointCount - 1)->Y);

  while (--PointCount) {
    X = Points->X;
    Y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(X, Y, Points->X, Points->Y);
  }
}

/**
 * @brief  Draws an ellipse on LCD in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  XRadius: Ellipse X radius
 * @param  YRadius: Ellipse Y radius
 */
void BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius) {
  int x = 0, y = -YRadius, err = 2 - 2 * XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2 / rad1);

  do {
    BSP_LCD_DrawPixel((Xpos - (uint16_t)(x / K)), (Ypos + y),
                      DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos + (uint16_t)(x / K)), (Ypos + y),
                      DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos + (uint16_t)(x / K)), (Ypos - y),
                      DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos - (uint16_t)(x / K)), (Ypos - y),
                      DrawProp[ActiveLayer].TextColor);

    e2 = err;
    if (e2 <= x) {
      err += ++x * 2 + 1;
      if (-y == x && e2 <= y)
        e2 = 0;
    }
    if (e2 > y)
      err += ++y * 2 + 1;
  } while (y <= 0);
}

/**
 * @brief  Draws a bitmap picture loaded in the internal Flash (32 bpp) in
 * currently active layer.
 * @param  Xpos: Bmp X position in the LCD
 * @param  Ypos: Bmp Y position in the LCD
 * @param  pbmp: Pointer to Bmp picture address in the internal Flash
 */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp) {
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t Address;
  uint32_t InputColorMode = 0;
  uint32_t bytes_per_pixel = 3;  /* CRITICAL FIX v0.1.46: RGB888 = 3 bytes per pixel */

  /* Get bitmap data address offset */
  index = pbmp[10] + (pbmp[11] << 8) + (pbmp[12] << 16) + (pbmp[13] << 24);

  /* Read bitmap width */
  width = pbmp[18] + (pbmp[19] << 8) + (pbmp[20] << 16) + (pbmp[21] << 24);

  /* Read bitmap height */
  height = pbmp[22] + (pbmp[23] << 8) + (pbmp[24] << 16) + (pbmp[25] << 24);

  /* Read bit/pixel */
  bit_pixel = pbmp[28] + (pbmp[29] << 8);

  /* Set the address - CRITICAL FIX v0.1.46: Use 3 bytes per pixel for RGB888 */
  Address = hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress +
            (((BSP_LCD_GetXSize() * Ypos) + Xpos) * bytes_per_pixel);

  /* Get the layer pixel format */
  if ((bit_pixel / 8) == 4) {
    InputColorMode = DMA2D_INPUT_ARGB8888;
  } else if ((bit_pixel / 8) == 2) {
    InputColorMode = DMA2D_INPUT_RGB565;
  } else {
    InputColorMode = DMA2D_INPUT_RGB888;
  }

  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel / 8)));

  /* Convert picture to RGB888 pixel format - CRITICAL FIX v0.1.46 */
  for (index = 0; index < height; index++) {
    /* Pixel format conversion - use RGB888 output to match LTDC layer format */
    LL_ConvertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)Address, width,
                             InputColorMode);

    /* Increment the source and destination buffers - CRITICAL FIX: Use 3 bytes per pixel */
    Address += (BSP_LCD_GetXSize() * bytes_per_pixel);
    pbmp -= width * (bit_pixel / 8);
  }
}

/**
 * @brief  Draws a full rectangle in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Width: Rectangle width
 * @param  Height: Rectangle height
 */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width,
                      uint16_t Height) {
  uint32_t Xaddress = 0;

  /* Set the text color */
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  /* Get the rectangle start address - CRITICAL FIX v0.1.46: Use 3 bytes per pixel for RGB888 */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) +
             3 * (BSP_LCD_GetXSize() * Ypos + Xpos);

  /* Fill the rectangle */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Width, Height,
                (BSP_LCD_GetXSize() - Width), DrawProp[ActiveLayer].TextColor);
}

/**
 * @brief  Draws a full circle in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  Radius: Circle radius
 */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius) {
  int32_t D;     /* Decision Variable */
  uint32_t CurX; /* Current X Value */
  uint32_t CurY; /* Current Y Value */

  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  while (CurX <= CurY) {
    if (CurY > 0) {
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos + CurX, 2 * CurY);
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos - CurX, 2 * CurY);
    }

    if (CurX > 0) {
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos - CurY, 2 * CurX);
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos + CurY, 2 * CurX);
    }
    if (D < 0) {
      D += (CurX << 2) + 6;
    } else {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
 * @brief  Draws a full poly-line (between many points) in currently active
 * layer.
 * @param  Points: Pointer to the points array
 * @param  PointCount: Number of points
 */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount) {
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0,
          Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP = IMAGE_BOTTOM = Points->Y;

  for (counter = 1; counter < PointCount; counter++) {
    pixelX = POLY_X(counter);
    if (pixelX < IMAGE_LEFT) {
      IMAGE_LEFT = pixelX;
    }
    if (pixelX > IMAGE_RIGHT) {
      IMAGE_RIGHT = pixelX;
    }

    pixelY = POLY_Y(counter);
    if (pixelY < IMAGE_TOP) {
      IMAGE_TOP = pixelY;
    }
    if (pixelY > IMAGE_BOTTOM) {
      IMAGE_BOTTOM = pixelY;
    }
  }

  if (PointCount < 2) {
    return;
  }

  X_center = (IMAGE_LEFT + IMAGE_RIGHT) / 2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP) / 2;

  X_first = Points->X;
  Y_first = Points->Y;

  while (--PointCount) {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;

    FillTriangle(X, X2, X_center, Y, Y2, Y_center);
    FillTriangle(X, X_center, X2, Y, Y_center, Y2);
    FillTriangle(X_center, X2, X, Y_center, Y2, Y);
  }

  FillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  FillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  FillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first);
}

/**
 * @brief  Draws a full ellipse in currently active layer.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  XRadius: Ellipse X radius
 * @param  YRadius: Ellipse Y radius
 */
void BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius) {
  int x = 0, y = -YRadius, err = 2 - 2 * XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2 / rad1);

  do {
    BSP_LCD_DrawHLine((Xpos - (uint16_t)(x / K)), (Ypos + y),
                      (2 * (uint16_t)(x / K) + 1));
    BSP_LCD_DrawHLine((Xpos - (uint16_t)(x / K)), (Ypos - y),
                      (2 * (uint16_t)(x / K) + 1));

    e2 = err;
    if (e2 <= x) {
      err += ++x * 2 + 1;
      if (-y == x && e2 <= y)
        e2 = 0;
    }
    if (e2 > y)
      err += ++y * 2 + 1;
  } while (y <= 0);
}

/**
 * @brief  Switch back on the display if was switched off by previous call of
 * BSP_LCD_DisplayOff(). Exit DSI ULPM mode if was allowed and configured in Dsi
 * Configuration.
 */
void BSP_LCD_DisplayOn(void) {
#if defined(USE_LCD_HDMI)
  if (ADV7533_ID == adv7533_drv.ReadID(ADV7533_CEC_DSI_I2C_ADDR)) {
    return; /* Not supported for HDMI display */
  } else
#endif /* USE_LCD_HDMI */
  {
    /* Send Display on DCS command to display */
    HAL_DSI_ShortWrite(&(hdsi_discovery), hdsivideo_handle.VirtualChannelID,
                       DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPON, 0x00);
  }
}

/**
 * @brief  Switch Off the display.
 *         Enter DSI ULPM mode if was allowed and configured in Dsi
 * Configuration.
 */
void BSP_LCD_DisplayOff(void) {
#if defined(USE_LCD_HDMI)
  if (ADV7533_ID == adv7533_drv.ReadID(ADV7533_CEC_DSI_I2C_ADDR)) {
    return; /* Not supported for HDMI display */
  } else
#endif /* USE_LCD_HDMI */
  {
    /* Send Display off DCS Command to display */
    HAL_DSI_ShortWrite(&(hdsi_discovery), hdsivideo_handle.VirtualChannelID,
                       DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);
  }
}

/**
 * @brief  Set the brightness value
 * @param  BrightnessValue: [00: Min (black), 100 Max]
 */
void BSP_LCD_SetBrightness(uint8_t BrightnessValue) {
#if defined(USE_LCD_HDMI)
  if (ADV7533_ID == adv7533_drv.ReadID(ADV7533_CEC_DSI_I2C_ADDR)) {
    return; /* Not supported for HDMI display */
  } else
#endif /* USE_LCD_HDMI */
  {
    /* Send Display on DCS command to display */
    HAL_DSI_ShortWrite(&hdsi_discovery, LCD_Driver_ID,
                       DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_WRDISBV,
                       (uint16_t)(BrightnessValue * 255) / 100);
  }
}

/**
 * @brief  DCS or Generic short/long write command
 * @param  NbrParams: Number of parameters. It indicates the write command mode:
 *                 If inferior to 2, a long write command is performed else
 * short.
 * @param  pParams: Pointer to parameter values table.
 * @retval HAL status
 */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams) {
  if (NbrParams <= 1) {
    HAL_DSI_ShortWrite(&hdsi_discovery, LCD_Driver_ID,
                       DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
  } else {
    HAL_DSI_LongWrite(&hdsi_discovery, LCD_Driver_ID, DSI_DCS_LONG_PKT_WRITE,
                      NbrParams, pParams[NbrParams], pParams);
  }
}

/**
 * @brief  Generic read command
 * @param  LCD_ID Virtual channel ID
 * @param  Reg Register to be read
 * @param  pData pointer to a buffer to store the payload of a read back
 * operation.
 * @param  Size  Data size to be read (in byte).
 * @retval BSP status
 */
int32_t DSI_IO_ReadCmd(uint32_t Reg, uint8_t *pData, uint32_t Size) {
  int32_t ret = LCD_ERROR;

  if (HAL_DSI_Read(&hdsi_discovery, LCD_Driver_ID, pData, Size,
                   DSI_DCS_SHORT_PKT_READ, Reg, pData) == HAL_OK) {
    ret = LCD_OK;
  }

  return ret;
}

/**
 * @brief  Returns the ID of connected screen by checking the HDMI
 *        (adv7533 component) ID or LCD DSI (via TS ID) ID.
 * @retval LCD ID
 */
static uint16_t LCD_IO_GetID(void) {
#if defined(USE_LCD_HDMI)
  HDMI_IO_Init();

  HDMI_IO_Delay(120);

  if (ADV7533_ID == adv7533_drv.ReadID(ADV7533_CEC_DSI_I2C_ADDR)) {
    return ADV7533_ID;
  } else if (((HDMI_IO_Read(LCD_DSI_ADDRESS, LCD_DSI_ID_REG) == LCD_DSI_ID)) ||
             (HDMI_IO_Read(LCD_DSI_ADDRESS_A02, LCD_DSI_ID_REG) ==
              LCD_DSI_ID)) {
    return LCD_DSI_ID;
  } else {
    return 0;
  }
#else
  return LCD_DSI_ID;
#endif /* USE_LCD_HDMI */
}

/*******************************************************************************
                       LTDC, DMA2D and DSI BSP Routines
*******************************************************************************/
/**
 * @brief  De-Initializes the BSP LCD Msp
 * Application can surcharge if needed this function implementation.
 */
__weak void BSP_LCD_MspDeInit(void) {
  /** @brief Disable IRQ of LTDC IP */
  HAL_NVIC_DisableIRQ(LTDC_IRQn);

  /** @brief Disable IRQ of DMA2D IP */
  HAL_NVIC_DisableIRQ(DMA2D_IRQn);

  /** @brief Disable IRQ of DSI IP */
  HAL_NVIC_DisableIRQ(DSI_IRQn);

  /** @brief Force and let in reset state LTDC, DMA2D and DSI Host + Wrapper IPs
   */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DSI_FORCE_RESET();

  /** @brief Disable the LTDC, DMA2D and DSI Host and Wrapper clocks */
  __HAL_RCC_LTDC_CLK_DISABLE();
  __HAL_RCC_DMA2D_CLK_DISABLE();
  __HAL_RCC_DSI_CLK_DISABLE();
}

/**
 * @brief  Initialize the BSP LCD Msp.
 * Application can surcharge if needed this function implementation
 */
__weak void BSP_LCD_MspInit(void) {
  /** @brief Enable the LTDC clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /** @brief Toggle Sw reset of LTDC IP */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  /** @brief Enable the DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /** @brief Toggle Sw reset of DMA2D IP */
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();

  /** @brief Enable DSI Host and wrapper clocks */
  __HAL_RCC_DSI_CLK_ENABLE();

  /** @brief Soft Reset the DSI Host and wrapper */
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);

  /** @brief NVIC configuration for DMA2D interrupt that is now enabled */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);

  /** @brief NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}

/**
 * @brief  Draws a pixel on LCD.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  RGB_Code: Pixel color in ARGB mode (8-8-8-8)
 */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code) {
  uint32_t fb_addr = hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress;
  uint8_t *pPixel =
      (uint8_t *)(fb_addr + 3 * (Ypos * BSP_LCD_GetXSize() + Xpos));

  pPixel[0] = (uint8_t)(RGB_Code >> 16); // R
  pPixel[1] = (uint8_t)(RGB_Code >> 8);  // G
  pPixel[2] = (uint8_t)(RGB_Code);       // B
}

/**
 * @brief  Draws a character on LCD.
 * @param  Xpos: Line where to display the character shape
 * @param  Ypos: Start column address
 * @param  c: Pointer to the character data
 */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c) {
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t offset;
  uint8_t *pchar;
  uint32_t line;

  height = DrawProp[ActiveLayer].pFont->Height;
  width = DrawProp[ActiveLayer].pFont->Width;

  offset = 8 * ((width + 7) / 8) - width;

  for (i = 0; i < height; i++) {
    pchar = ((uint8_t *)c + (width + 7) / 8 * i);

    switch (((width + 7) / 8)) {

    case 1:
      line = pchar[0];
      break;

    case 2:
      line = (pchar[0] << 8) | pchar[1];
      break;

    case 3:
    default:
      line = (pchar[0] << 16) | (pchar[1] << 8) | pchar[2];
      break;
    }

    for (j = 0; j < width; j++) {
      if (line & (1 << (width - j + offset - 1))) {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].TextColor);
      } else {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].BackColor);
      }
    }
    Ypos++;
  }
}

/**
 * @brief  Fills a triangle (between 3 points).
 * @param  x1: Point 1 X position
 * @param  y1: Point 1 Y position
 * @param  x2: Point 2 X position
 * @param  y2: Point 2 Y position
 * @param  x3: Point 3 X position
 * @param  y3: Point 3 Y position
 */
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1,
                         uint16_t y2, uint16_t y3) {
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, yinc1 = 0,
          yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, curpixel = 0;

  deltax = ABS(x2 - x1); /* The difference between the x's */
  deltay = ABS(y2 - y1); /* The difference between the y's */
  x = x1;                /* Start x off at the first pixel */
  y = y1;                /* Start y off at the first pixel */

  if (x2 >= x1) /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  } else /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1) /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  } else /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay) /* There is at least one x-value for every y-value */
  {
    xinc1 = 0; /* Don't change the x when numerator >= denominator */
    yinc2 = 0; /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax; /* There are more x-values than y-values */
  } else                /* There is at least one y-value for every x-value */
  {
    xinc2 = 0; /* Don't change the x for every iteration */
    yinc1 = 0; /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay; /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++) {
    BSP_LCD_DrawLine(x, y, x3, y3);

    num += numadd;  /* Increase the numerator by the top of the fraction */
    if (num >= den) /* Check if numerator >= denominator */
    {
      num -= den; /* Calculate the new numerator value */
      x += xinc1; /* Change the x as appropriate */
      y += yinc1; /* Change the y as appropriate */
    }
    x += xinc2; /* Change the x as appropriate */
    y += yinc2; /* Change the y as appropriate */
  }
}

/**
 * @brief  Fills a buffer.
 * @param  LayerIndex: Layer index
 * @param  pDst: Pointer to destination buffer
 * @param  xSize: Buffer width
 * @param  ySize: Buffer height
 * @param  OffLine: Offset
 * @param  ColorIndex: Color index
 */
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize,
                          uint32_t ySize, uint32_t OffLine,
                          uint32_t ColorIndex) {
  /* Register to memory mode with RGB888 as color Mode to match LTDC layer format */
  /* CRITICAL FIX v0.1.46: Use DMA2D_OUTPUT_RGB888 instead of DMA2D_OUTPUT_ARGB8888
   * The LTDC layer is configured for RGB888 (3 bytes per pixel), so the DMA2D
   * must also output RGB888 format. Using ARGB8888 (4 bytes per pixel) causes
   * display shift/slant due to byte misalignment. */
  hdma2d_discovery.Init.Mode = DMA2D_R2M;
  hdma2d_discovery.Init.ColorMode = DMA2D_OUTPUT_RGB888;
  hdma2d_discovery.Init.OutputOffset = OffLine;

  hdma2d_discovery.Instance = DMA2D;

  /* DMA2D Initialization */
  if (HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK) {
    if (HAL_DMA2D_ConfigLayer(&hdma2d_discovery, LayerIndex) == HAL_OK) {
      if (HAL_DMA2D_Start(&hdma2d_discovery, ColorIndex, (uint32_t)pDst, xSize,
                          ySize) == HAL_OK) {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
      }
    }
  }
}

/**
 * @brief  Converts a line to an RGB888 pixel format.
 * @param  pSrc: Pointer to source buffer
 * @param  pDst: Output color
 * @param  xSize: Buffer width
 * @param  ColorMode: Input color mode
 */
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize,
                                     uint32_t ColorMode) {
  /* Configure the DMA2D Mode, Color Mode and output offset */
  /* CRITICAL FIX v0.1.46: Use DMA2D_OUTPUT_RGB888 to match LTDC layer format */
  hdma2d_discovery.Init.Mode = DMA2D_M2M_PFC;
  hdma2d_discovery.Init.ColorMode = DMA2D_OUTPUT_RGB888;
  hdma2d_discovery.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d_discovery.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_discovery.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_discovery.LayerCfg[1].InputColorMode = ColorMode;
  hdma2d_discovery.LayerCfg[1].InputOffset = 0;

  hdma2d_discovery.Instance = DMA2D;

  /* DMA2D Initialization */
  if (HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK) {
    if (HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1) == HAL_OK) {
      if (HAL_DMA2D_Start(&hdma2d_discovery, (uint32_t)pSrc, (uint32_t)pDst,
                          xSize, 1) == HAL_OK) {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
      }
    }
  }
}

/**
 * @brief  Check if the component ID is correct.
 * @param  Lcd_type Driver Type Control NT35510 or OTM8009A
 */
static LCD_Driver_t Driver_Type(LCD_Driver_t Lcd_type) {
  uint16_t read_id;
  /* Read the NT35510 ID */
  read_id = NT35510_ReadID();
  if (read_id == NT35510_ID) {
    Lcd_type = LCD_CTRL_NT35510;
  } else {
    /* Read the OTM8009A ID */
    read_id = OTM8009A_ReadID();
    if (read_id == OTM8009A_ID) {
      Lcd_type = LCD_CTRL_OTM8009A;
    } else {
      Lcd_type = LCD_CTRL_NONE;
    }
  }

  return Lcd_type;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
