/**
 ******************************************************************************
 * @file    stm32f7xx_hal_uart.c
 * @author  MCD Application Team
 * @brief   UART HAL module driver.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#ifdef HAL_UART_MODULE_ENABLED

/** @defgroup UART UART
 * @brief HAL UART module driver
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_CR1_FIELDS                                                       \
  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE |      \
              USART_CR1_RE | USART_CR1_OVER8))

#define USART_CR3_FIELDS                                                       \
  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT))

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* static void UART_EndTxTransfer(UART_HandleTypeDef *huart); */
/* static void UART_EndRxTransfer(UART_HandleTypeDef *huart); */
/* static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma); */
/* static void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma); */
/* static void UART_DMAError(DMA_HandleTypeDef *hdma); */
/* static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma); */
/* static void UART_DMATxAbortCallback(DMA_HandleTypeDef *hdma); */
/* static void UART_DMARxAbortCallback(DMA_HandleTypeDef *hdma); */
/* static void UART_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma); */
/* static void UART_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma); */
/* static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart); */
/* static HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart); */
/* static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart); */
static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart,
                                                     uint32_t Flag,
                                                     FlagStatus Status,
                                                     uint32_t Tickstart,
                                                     uint32_t Timeout);
static void UART_SetConfig(
    UART_HandleTypeDef *huart); /* NOTE: Made static if not exported in .h */

/* Exported functions --------------------------------------------------------*/

/** @defgroup UART_Exported_Functions UART Exported Functions
 * @{
 */

/** @defgroup UART_Exported_Functions_Group1 Initialization and
 * de-initialization functions
 * @{
 */

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return HAL_ERROR;
  }

  if (huart->Init.HwFlowCtl != UART_HWCONTROL_NONE) {
    /* Check the parameters */
    assert_param(IS_UART_HW_FLOW_CONTROL(huart->Init.HwFlowCtl));
  } else {
    /* Check the parameters */
    assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
    assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));
  }

  /* Check the parameters */
  assert_param(IS_UART_INSTANCE(huart->Instance));
  assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));

  if (huart->gState == HAL_UART_STATE_RESET) {
    huart->Lock = HAL_UNLOCKED;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /* Init the UART Callback settings */
    huart->TxHalfCpltCallback = HAL_UART_TxHalfCpltCallback; /* Legacy weak */
    huart->TxCpltCallback = HAL_UART_TxCpltCallback;         /* Legacy weak */
    huart->RxHalfCpltCallback = HAL_UART_RxHalfCpltCallback; /* Legacy weak */
    huart->RxCpltCallback = HAL_UART_RxCpltCallback;         /* Legacy weak */
    huart->ErrorCallback = HAL_UART_ErrorCallback;           /* Legacy weak */
    huart->AbortCpltCallback = HAL_UART_AbortCpltCallback;   /* Legacy weak */
    huart->AbortTransmitCpltCallback =
        HAL_UART_AbortTransmitCpltCallback; /* Legacy weak */
    huart->AbortReceiveCpltCallback =
        HAL_UART_AbortReceiveCpltCallback; /* Legacy weak */

    if (huart->MspInitCallback == NULL) {
      huart->MspInitCallback = HAL_UART_MspInit;
    }
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    HAL_UART_MspInit(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = HAL_UART_STATE_BUSY;

  /* Disable the Peripheral */
  __HAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  UART_SetConfig(huart);

  /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  huart->Instance->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  huart->Instance->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

  /* Enable the Peripheral */
  __HAL_UART_ENABLE(huart);

  /* Initialize the UART state */
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->gState = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;

  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_UART_INSTANCE(huart->Instance));

  huart->gState = HAL_UART_STATE_BUSY;

  /* Disable the Peripheral */
  __HAL_UART_DISABLE(huart);

  huart->Instance->CR1 = 0x0U;
  huart->Instance->CR2 = 0x0U;
  huart->Instance->CR3 = 0x0U;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  if (huart->MspDeInitCallback == NULL) {
    huart->MspDeInitCallback = HAL_UART_MspDeInit;
  }
  huart->MspDeInitCallback(huart);
#else
  /* DeInit the low level hardware */
  HAL_UART_MspDeInit(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->gState = HAL_UART_STATE_RESET;
  huart->RxState = HAL_UART_STATE_RESET;

  return HAL_OK;
}

__weak void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_MspInit can be implemented in the user file
   */
}

__weak void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_MspDeInit can be implemented in the user file
   */
}

/**
 * @}
 */

/** @defgroup UART_Exported_Functions_Group2 IO operation functions
 * @{
 */

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout) {
  uint32_t tickstart = 0U;

  if ((huart->gState == HAL_UART_STATE_READY) ||
      (huart->gState == HAL_UART_STATE_BUSY_RX)) {
    if ((pData == NULL) || (Size == 0U)) {
      return HAL_ERROR;
    }

    __HAL_LOCK(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    while (huart->TxXferCount > 0U) {
      if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart,
                                      Timeout) != HAL_OK) {
        return HAL_TIMEOUT;
      }
      huart->Instance->TDR = (uint8_t)(*pData++ & 0xFFU);
      huart->TxXferCount--;
    }

    if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart,
                                    Timeout) != HAL_OK) {
      return HAL_TIMEOUT;
    }

    /* At end of transmission, restore the gState to Ready */
    huart->gState = HAL_UART_STATE_READY;

    __HAL_UNLOCK(huart);

    return HAL_OK;
  } else {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout) {
  uint32_t tickstart = 0U;

  if ((huart->RxState == HAL_UART_STATE_READY) ||
      (huart->RxState == HAL_UART_STATE_BUSY_TX)) {
    if ((pData == NULL) || (Size == 0U)) {
      return HAL_ERROR;
    }

    __HAL_LOCK(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    huart->RxXferSize = Size;
    huart->RxXferCount = Size;

    while (huart->RxXferCount > 0U) {
      if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart,
                                      Timeout) != HAL_OK) {
        return HAL_TIMEOUT;
      }
      *pData++ = (uint8_t)(huart->Instance->RDR & (uint8_t)0xFF);
      huart->RxXferCount--;
    }

    /* At end of Rx, restore the RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;

    __HAL_UNLOCK(huart);

    return HAL_OK;
  } else {
    return HAL_BUSY;
  }
}

/* ... Additional functions needed for full HAL support, simplified here for
 * critical paths ... */

static void UART_SetConfig(UART_HandleTypeDef *huart) {
  uint32_t tmpreg;
  uint16_t brrtemp;
  uint32_t uartdiv = 0x00000000U;
  uint32_t pclk;

  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));
  assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  assert_param(IS_UART_ONE_BIT_SAMPLE(huart->Init.OneBitSampling));

  /* Configuration of the USART OverSampling: set the OVER8 bit in the CR1
   * register */
  huart->Instance->CR1 &= ~(USART_CR1_OVER8);
  huart->Instance->CR1 |= huart->Init.OverSampling;

  /* Configuration of the USART Word Length, Parity and Mode: set the M, PCE,
   * PS, TE and RE bits */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity |
           huart->Init.Mode | huart->Init.OverSampling;
  huart->Instance->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS |
                            USART_CR1_TE | USART_CR1_RE);
  huart->Instance->CR1 |= tmpreg;

  /* Configuration of the USART Stop Bits: set the STOP[13:12] bits in the CR2
   * register */
  huart->Instance->CR2 &= ~(USART_CR2_STOP);
  huart->Instance->CR2 |= huart->Init.StopBits;

  /* Configuration of the USART Hardware Flow Control: set the RTSE and CTSE
   * bits in the CR3 register */
  huart->Instance->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT);
  huart->Instance->CR3 |= huart->Init.HwFlowCtl | huart->Init.OneBitSampling;

  /* PCLK */
  /* This part normally calculates PCLK based on instance. Simplified for this
   * board context if possible. */
  /* Getting PCLK requires accessing RCC HAL which is available. */
  /* For simplicity in this inserted code, we assume generic PCLK1/PCLK2 logic
   * or standard APB1/APB2 */
  /* Actually, to be safe, we just use SystemCoreClock or HAL_RCC_GetPCLK1Freq()
   * logic. */
  /* But let's assume we can get it via HAL_RCC_GetPCLK1Freq() if UART 5 is on
   * APB1. */

  /* NOTE: Using fixed baud calculation logic for simplicity in restoration */
  if (huart->Instance == USART1 || huart->Instance == USART6) {
    pclk = HAL_RCC_GetPCLK2Freq();
  } else {
    pclk = HAL_RCC_GetPCLK1Freq();
  }

  if (huart->Init.OverSampling == UART_OVERSAMPLING_8) {
    uartdiv = (uint16_t)(UART_DIV_SAMPLING8(pclk, huart->Init.BaudRate));
    brrtemp = uartdiv & 0xFFF0U;
    brrtemp |= (uint16_t)((uartdiv & 0x000F) >> 1U);
    huart->Instance->BRR = brrtemp;
  } else {
    uartdiv = (uint16_t)(UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate));
    huart->Instance->BRR = uartdiv;
  }
}

static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart,
                                                     uint32_t Flag,
                                                     FlagStatus Status,
                                                     uint32_t Tickstart,
                                                     uint32_t Timeout) {
  /* Wait until flag is set */
  while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status) {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY) {
      if ((Timeout == 0) || ((HAL_GetTick() - Tickstart) > Timeout)) {
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

#endif /* HAL_UART_MODULE_ENABLED */
