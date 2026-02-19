/**
 ******************************************************************************
 * @file    ltcd_override.c
 * @brief   Weak override of MX_LTDC_Init to prevent wrong LTDC timing
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Include ------------------------------------------------------------------*/
#include "../../Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h"

/**
 * @brief Weak override of MX_LTDC_Init to prevent ltdc.c from overwriting LTDC timing
 *
 * The Core/Src/ltdc.c file from STM32CubeMX contains auto-generated LTDC
 * initialization with hardcoded timing values (TotalWidth=939, TotalHeigh=496) that
 * are WRONG for MB1166 display. This weak function override does nothing,
 * preventing the wrong timing from being applied.
 */
__weak void MX_LTDC_Init(void) {
  /* Do nothing - let BSP_LCD_Init() set the correct timing */
}
