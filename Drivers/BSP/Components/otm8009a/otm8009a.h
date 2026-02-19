/**
 ******************************************************************************
 * @file    otm8009a.h
 * @author  MCD Application Team
 * @brief   This file contains all the constants parameters for the OTM8009A
 *          which is the LCD Driver for KoD KM-040TMP-02-0621 (WVGA)
 *          DSI LCD Display.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OTM8009A_H
#define __OTM8009A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup otm8009a
 * @{
 */

/** @addtogroup OTM8009A_Exported_Variables
 * @{
 */
/* OTM8009A ID */
#define OTM8009A_ID                 0x40
#if defined( __GNUC__ ) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* GNU and ARM Compiler 6 compilers */
#ifndef __weak
#define __weak  __attribute__((weak))
#endif /* __weak */
#endif /* __GNUC__ || (__ARMCC_VERSION && (__ARMCC_VERSION >= 6010050)) */

/**
 * @brief LCD_OrientationTypeDef
 * Possible values of Display Orientation
 */
#define OTM8009A_ORIENTATION_PORTRAIT    ((uint32_t)0x00) /* Portrait orientation choice of LCD screen  */
#define OTM8009A_ORIENTATION_LANDSCAPE   ((uint32_t)0x01) /* Landscape orientation choice of LCD screen  */

/**
 * @brief  Possible values of
 * pixel data format (ie color coding) transmitted on DSI Data lane in DSI packets
 */
#define OTM8009A_FORMAT_RGB888    ((uint32_t)0x00) /* Pixel format chosen is RGB888 : 24 bpp */
#define OTM8009A_FORMAT_RGB565    ((uint32_t)0x02) /* Pixel format chosen is RGB565 : 16 bpp */

/**
 * @brief  otm8009a_480x800 Size
 */

/* Width and Height in Portrait mode */
#define  OTM8009A_480X800_WIDTH             ((uint16_t)480)     /* LCD PIXEL WIDTH   */
#define  OTM8009A_480X800_HEIGHT            ((uint16_t)800)     /* LCD PIXEL HEIGHT  */

/* Width and Height in Landscape mode */
/* CRITICAL FIX: MB1166 display is 800x472, NOT 800x480! */
/* ST UM2033 User Manual confirms 800x472-pixel display */
/* Using correct height eliminates display stripe artifacts */
#define  OTM8009A_800X480_WIDTH             ((uint16_t)800)     /* LCD PIXEL WIDTH   */
#define  OTM8009A_800X480_HEIGHT            ((uint16_t)472)     /* LCD PIXEL HEIGHT - FIXED for MB1166 */

/**
 * @brief  OTM8009A_480X800 Timing parameters for Portrait orientation mode
 * MB1166: 480x800 display (these are swapped for 800x472 landscape)
 *
 * These are the BASE timing values for portrait mode. For landscape mode,
 * these values are swapped: H<->V. See OTM8009A_800X480_* macros below.
 *
 * Base values (MB1166-tested timing):
 * - HSYNC = 2 (horizontal sync) -> becomes VSYNC in landscape
 * - HBP = 34 (horizontal back porch) -> becomes VBP in landscape
 * - HFP = 34 (horizontal front porch) -> becomes VFP in landscape
 * - VSYNC = 1 (vertical sync) -> becomes HSYNC in landscape
 * - VBP = 34 (vertical back porch) -> becomes HBP in landscape
 * - VFP = 34 (vertical front porch) -> becomes HFP in landscape
 */
#define  OTM8009A_480X800_HSYNC             ((uint16_t)2)      /* Horizontal sync -> VSYNC in landscape */
#define  OTM8009A_480X800_HBP               ((uint16_t)34)     /* Horizontal back porch -> VBP in landscape */
#define  OTM8009A_480X800_HFP               ((uint16_t)34)     /* Horizontal front porch -> VFP in landscape */
#define  OTM8009A_480X800_VSYNC             ((uint16_t)1)      /* Vertical sync -> HSYNC in landscape */
#define  OTM8009A_480X800_VBP               ((uint16_t)34)     /* Vertical back porch -> HBP in landscape */
#define  OTM8009A_480X800_VFP               ((uint16_t)34)     /* Vertical front porch -> HFP in landscape */

/**
 * @brief  OTM8009A_800X480 Timing parameters for Landscape orientation mode
 * CRITICAL FIX v0.1.58: MB1166-tested timing values for proper DSI/LTDC sync
 *
 * The landscape timing values are mapped from portrait base values:
 * - Portrait HSYNC (2) -> Landscape VSYNC
 * - Portrait HBP (34) -> Landscape VBP
 * - Portrait HFP (34) -> Landscape VFP
 * - Portrait VSYNC (1) -> Landscape HSYNC
 * - Portrait VBP (34) -> Landscape HBP
 * - Portrait VFP (34) -> Landscape HFP
 *
 * NOTE: The actual macro values here resolve to:
 * - HSYNC = 1 (from OTM8009A_480X800_VSYNC)
 * - HBP = 34 (from OTM8009A_480X800_VBP) - MB1166 tested value
 * - HFP = 34 (from OTM8009A_480X800_VFP) - MB1166 tested value
 * - VSYNC = 2 (from OTM8009A_480X800_HSYNC)
 * - VBP = 34 (from OTM8009A_480X800_HBP)
 * - VFP = 34 (from OTM8009A_480X800_HFP)
 *
 * This symmetric timing (HBP=HFP=VBP=VFP=34) produces square checkerboard cells.
 * For MB1166, VACT is 472 instead of 480.
 *
 * CRITICAL: DSI host timing values in dsihost.c MUST match these values!
 * Previous v0.1.57 had DSI VFP=16 while LTDC VFP=34, causing severe distortion.
 *
 * Expected LTDC timing:
 * - TotalWidth = HSA + HBP + HACT + HFP - 1 = 1 + 34 + 800 + 34 - 1 = 868
 * - TotalHeigh = VSA + VBP + VACT + VFP - 1 = 2 + 34 + 472 + 34 - 1 = 541
 * - AccumulatedHBP = HSA + HBP - 1 = 1 + 34 - 1 = 34
 * - AccumulatedVBP = VSA + VBP - 1 = 2 + 34 - 1 = 35
 */
#define  OTM8009A_800X480_HSYNC             OTM8009A_480X800_VSYNC  /* Horizontal sync = 1 */
#define  OTM8009A_800X480_HBP               OTM8009A_480X800_VBP    /* Horizontal back porch = 34 (MB1166) */
#define  OTM8009A_800X480_HFP               OTM8009A_480X800_VFP    /* Horizontal front porch = 34 (MB1166) */
#define  OTM8009A_800X480_VSYNC             OTM8009A_480X800_HSYNC  /* Vertical sync = 2 */
#define  OTM8009A_800X480_VBP               OTM8009A_480X800_HBP    /* Vertical back porch = 34 */
#define  OTM8009A_800X480_VFP               OTM8009A_480X800_HFP    /* Vertical front porch = 34 */


/* List of OTM8009A used commands                                  */
/* Detailed in OTM8009A Data Sheet 'DATA_SHEET_OTM8009A_V0 92.pdf' */
/* Version of 14 June 2012                                         */
#define  OTM8009A_CMD_NOP                   0x00  /* NOP command      */
#define  OTM8009A_CMD_SWRESET               0x01  /* Sw reset command */
#define  OTM8009A_CMD_RDDMADCTL             0x0B  /* Read Display MADCTR command : read memory display access ctrl */
#define  OTM8009A_CMD_RDDCOLMOD             0x0C  /* Read Display pixel format */
#define  OTM8009A_CMD_SLPIN                 0x10  /* Sleep In command */
#define  OTM8009A_CMD_SLPOUT                0x11  /* Sleep Out command */
#define  OTM8009A_CMD_PTLON                 0x12  /* Partial mode On command */

#define  OTM8009A_CMD_DISPOFF               0x28  /* Display Off command */
#define  OTM8009A_CMD_DISPON                0x29  /* Display On command */

#define  OTM8009A_CMD_CASET                 0x2A  /* Column address set command */
#define  OTM8009A_CMD_PASET                 0x2B  /* Page address set command */

#define  OTM8009A_CMD_RAMWR                 0x2C  /* Memory (GRAM) write command */
#define  OTM8009A_CMD_RAMRD                 0x2E  /* Memory (GRAM) read command */

#define  OTM8009A_CMD_PLTAR                 0x30  /* Partial area command (4 parameters) */
#define  OTM8009A_CMD_TEEOFF                 0x34  /* Tearing Effect Line Off command : command with no parameter */
#define  OTM8009A_CMD_TEEON                 0x35  /* Tearing Effect Line On command : command with 1 parameter 'TELOM' */

#define  OTM8009A_CMD_MADCTR                0x36  /* Memory Access write control command */

/* MADCTR Mode parameter values */
#define  OTM8009A_MADCTR_MODE_LANDSCAPE     0x60  /* Landscape mode for MADCTR */
#define  OTM8009A_MADCTR_MODE_PORTRAIT      0x00  /* Portrait mode for MADCTR */

/* Parameter TELOM : Tearing Effect Line Output Mode : possible values */
#define  OTM8009A_TEEON_TELOM_VBLANKING_INFO_ONLY            0x00
#define  OTM8009A_TEEON_TELOM_VBLANKING_AND_HBLANKING_INFO   0x01

#define  OTM8009A_CMD_IDMOFF                0x38  /* Idle mode Off command */
#define  OTM8009A_CMD_IDMON                 0x39  /* Idle mode On command */

#define  OTM8009A_CMD_COLMOD                0x3A  /* Interface Pixel format command */

/* Possible values of COLMOD parameter corresponding to used pixel formats */
#define  OTM8009A_COLMOD_RGB565             0x55
#define  OTM8009A_COLMOD_RGB888             0x77

#define  OTM8009A_CMD_RAMWRC                0x3C  /* Memory write continue command */
#define  OTM8009A_CMD_RAMRDC                0x3E  /* Memory read continue command */

#define  OTM8009A_CMD_WRTESCN               0x44  /* Write Tearing Effect Scan line command */
#define  OTM8009A_CMD_RDSCNL                0x45  /* Read  Tearing Effect Scan line command */

/* CABC Management : ie : Content Adaptive Backlight Control in IC OTM8009a */
#define  OTM8009A_CMD_WRDISBV               0x51  /* Write Display Brightness command          */
#define  OTM8009A_CMD_WRCTRLD               0x53  /* Write CTRL Display command                */
#define  OTM8009A_CMD_WRCABC                0x55  /* Write Content Adaptive Brightness command */
#define  OTM8009A_CMD_WRCABCMB              0x5E  /* Write CABC Minimum Brightness command     */

#define  OTM8009A_CMD_ID1                   0xDA  /* Read ID1 command      */
#define  OTM8009A_CMD_ID2                   0xDB  /* Read ID2 command      */
#define  OTM8009A_CMD_ID3                   0xDC  /* Read ID3 command      */

/**
 * @brief  OTM8009A_480X800 frequency divider
 */
#define  OTM8009A_480X800_FREQUENCY_DIVIDER  2   /* LCD Frequency divider      */

/**
 * @}
 */


/* Exported macro ------------------------------------------------------------*/

/** @defgroup OTM8009A_Exported_Macros OTM8009A Exported Macros
 * @{
 */


/**
 * @}
 */


/**
 * @}
 */


/* Exported functions --------------------------------------------------------*/


/** @addtogroup OTM8009A_Exported_Functions
 * @{
 */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams);
int32_t DSI_IO_ReadCmd(uint32_t Reg, uint8_t *pData, uint32_t Size);
uint8_t OTM8009A_Init(uint32_t ColorCoding, uint32_t orientation);
void OTM8009A_IO_Delay(uint32_t Delay);
uint16_t OTM8009A_ReadID(void);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __OTM8009A_480X800_H */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
