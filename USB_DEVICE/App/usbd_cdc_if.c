/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v1.0_Cube
 * @brief          : Usb device for Virtual Com Port.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "stm32f7xx_hal_uart.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "test_system.h"
#include "cmsis_os.h"
#include "VERSION.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t UserRxRingBuffer[APP_RX_DATA_SIZE];
volatile uint32_t UserRxRingHead = 0;
volatile uint32_t UserRxRingTail = 0;
extern UART_HandleTypeDef huart5;

// Command buffer for accumulating partial USB CDC packets
static uint8_t CMDBuffer[256];
static uint16_t CMDBufferLen = 0;

// Test command buffer for ATSTn commands
static uint8_t TestCmdBuffer[64];
static uint16_t TestCmdLen = 0;

// Deferred command processing - handle in task context, not interrupt
static volatile uint8_t PendingTestId = 255;  // Test ID (255 = interrupt command)
static volatile uint8_t PendingCommandFlag = 0;  // v0.1.83: 1 = command pending, 0 = no command

// Parameterized drawing command storage (ATST151-158)
static volatile uint8_t PendingDrawParams[128];  // Storage for drawing parameters
static volatile uint8_t PendingDrawParamsLen = 0;  // Number of bytes in params
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

// External function prototypes for TEST_Output
extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  extern UART_HandleTypeDef huart5;
  switch (cmd) {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per
   * second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits */
  /*                                        0 - 1 Stop bit */
  /*                                        1 - 1.5 Stop bits */
  /*                                        2 - 2 Stop bits */
  /* 5      | bParityType |  1   | Number | Parity */
  /*                                        0 - None */
  /*                                        1 - Odd */
  /*                                        2 - Even */
  /*                                        3 - Mark */
  /*                                        4 - Space */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16). */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:
    if (length == 7) {
      USBD_CDC_LineCodingTypeDef *lineCoding =
          (USBD_CDC_LineCodingTypeDef *)pbuf;
      huart5.Init.BaudRate = lineCoding->bitrate;
      huart5.Init.WordLength =
          (lineCoding->format == 7) ? UART_WORDLENGTH_7B : UART_WORDLENGTH_8B;
      huart5.Init.StopBits =
          (lineCoding->format == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
      huart5.Init.Parity =
          (lineCoding->paritytype == 0)
              ? UART_PARITY_NONE
              : ((lineCoding->paritytype == 1) ? UART_PARITY_ODD
                                               : UART_PARITY_EVEN);

      if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
      }
    }
    break;

  case CDC_GET_LINE_CODING: {
    USBD_CDC_LineCodingTypeDef *lineCoding = (USBD_CDC_LineCodingTypeDef *)pbuf;
    lineCoding->bitrate = huart5.Init.BaudRate;
    lineCoding->format = 0;     // Stop bits: 1
    lineCoding->paritytype = 0; // Parity: None
    lineCoding->datatype = 8;   // Data bits: 8
  } break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  extern HAL_StatusTypeDef UART5_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);

  // Process received data for ATST commands
  for (uint32_t i = 0; i < *Len; i++) {
    uint8_t ch = Buf[i];

    // Add to test command buffer
    if (TestCmdLen < sizeof(TestCmdBuffer) - 1) {
      TestCmdBuffer[TestCmdLen++] = ch;
    }

    // Check for end of line (CR or LF)
    if (ch == '\r' || ch == '\n') {
      // Null terminate (only if we have characters)
      if (TestCmdLen > 0) {
        TestCmdBuffer[TestCmdLen - 1] = '\0';
      } else {
        // Empty line, just skip
        continue;
      }

      // Check if this is an AT command (respond OK)
      if (strcmp((char *)TestCmdBuffer, "AT") == 0 ||
          strcmp((char *)TestCmdBuffer, "at") == 0) {
        // Respond with OK
        extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);
        CDC_Transmit_ThreadSafe((uint8_t *)"\r\nOK\r\n> ", 9);
        TestCmdLen = 0;
        memset(TestCmdBuffer, 0, sizeof(TestCmdBuffer));
        continue;
      }

      // Check if this is an ATVER command (respond with version)
      if (strcmp((char *)TestCmdBuffer, "ATVER") == 0 ||
          strcmp((char *)TestCmdBuffer, "atver") == 0 ||
          strcmp((char *)TestCmdBuffer, "ATVER?") == 0) {
        // Respond with version string
        extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);
        char verResponse[64];
        int verLen = snprintf(verResponse, sizeof(verResponse),
            "\r\nv%s\r\n> ", FIRMWARE_VERSION_STRING);
        CDC_Transmit_ThreadSafe((uint8_t *)verResponse, verLen);
        TestCmdLen = 0;
        memset(TestCmdBuffer, 0, sizeof(TestCmdBuffer));
        continue;
      }

      // Check if this is an ATST command or help request
      // CRITICAL FIX v0.1.63: Handle ATST? and ATST properly
      // ATST? should show help (testId=99), not run test 0
      // CRITICAL FIX v0.1.83: Use int type, -1 for "no command", 255 for interrupt
      int testId = -1;  // Default: no command (use -1, not 255!)
      char *equalsPtr = strchr((char *)TestCmdBuffer, '=');  // Check for parameters

      // v0.1.83: Check for help request (command ends with '?')
      size_t cmdLen = strlen((char *)TestCmdBuffer);
      if (cmdLen > 0 && TestCmdBuffer[cmdLen - 1] == '?') {
        // Any command ending with '?' is a help request
        testId = 99;
      }

      // Check for text-based SD card commands first (only if not help request)
      if (testId == -1 && (strcmp((char *)TestCmdBuffer, "ATSTMCRI") == 0 ||
          strcmp((char *)TestCmdBuffer, "atstmcri") == 0)) {
        // Map to test ID 172
        testId = 172;
      } else if (testId == -1 && (strcmp((char *)TestCmdBuffer, "ATSTMCRL") == 0 ||
                 strcmp((char *)TestCmdBuffer, "atstmcrl") == 0)) {
        // Map to test ID 173
        testId = 173;
      } else if (strncmp((char *)TestCmdBuffer, "ATST", 4) == 0 ||
                 strncmp((char *)TestCmdBuffer, "atst", 4) == 0) {
        // ATST command - extract test ID or check for help
        size_t cmdLen = strlen((char *)TestCmdBuffer);
        if (cmdLen == 4) {
          // Just "ATST" - treat as help request
          testId = 99;
        } else if (cmdLen == 5 && TestCmdBuffer[4] == '?') {
          // "ATST?" - help request
          testId = 99;
        } else if (cmdLen == 7 && strcmp((char *)&TestCmdBuffer[4], "999") == 0) {
          // v0.1.83: Check for ATST999 BEFORE atoi() to avoid overflow
          // atoi() returns int, but storing 999 in uint8_t truncates to 231 (0xE7)
          testId = 255;  // TEST_ID_INTERRUPT
        } else if (equalsPtr != NULL) {
          // Parameterized command like ATST151=X,Y,D,C
          // Extract test ID before the '='
          *equalsPtr = '\0';  // Temporarily null-terminate at '='
          int parsedId = atoi((char *)&TestCmdBuffer[4]);
          *equalsPtr = '=';   // Restore '='

          // Validate test ID range (must fit in uint8_t for storage)
          if (parsedId >= 0 && parsedId <= 254) {
            testId = parsedId;
          } else {
            // Invalid test ID - ignore this command
            testId = 255;
          }

          // Store parameters for later processing
          const char *params = equalsPtr + 1;
          PendingDrawParamsLen = 0;
          while (*params != '\0' && PendingDrawParamsLen < sizeof(PendingDrawParams) - 1) {
            PendingDrawParams[PendingDrawParamsLen++] = *params++;
          }
          PendingDrawParams[PendingDrawParamsLen] = '\0';
        } else {
          // ATSTn - extract number
          // v0.1.83: Use int to avoid overflow and validate range
          int parsedId = atoi((char *)&TestCmdBuffer[4]);
          if (parsedId >= 0 && parsedId <= 254) {
            testId = parsedId;
          } else {
            // Out of range - might be ATST999 which we already handled above
            // or invalid command
            testId = -1;  // No command
          }
        }
      } else if (strcmp((char *)TestCmdBuffer, "?") == 0 ||
                 strcmp((char *)TestCmdBuffer, "HELP") == 0 ||
                 strcmp((char *)TestCmdBuffer, "help") == 0 ||
                 strcmp((char *)TestCmdBuffer, "AT?") == 0) {
        // Other help commands
        testId = 99;
      }

      if (testId >= 0) {
        // Valid ATST or help command (0-254 or 255 for interrupt) - defer processing
        // Cast to uint8_t since we've validated the range
        PendingTestId = (uint8_t)testId;
        PendingCommandFlag = 1;  // v0.1.83: Set flag to indicate command is pending
      } else {
        // Not an ATST or help command - forward to ESP8266
        UART5_Transmit(TestCmdBuffer, TestCmdLen, 1000);
      }

      // Reset buffer
      TestCmdLen = 0;
      memset(TestCmdBuffer, 0, sizeof(TestCmdBuffer));
    }  // End of if (ch == '\r' || ch == '\n')
  }  // End of for loop

  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0) {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
 * @brief  Read data from the USB RX ring buffer.
 * @param  pBuf: Buffer to store read data
 * @param  Len:  Number of bytes to read
 * @retval Number of bytes actually read
 */
uint32_t VCP_Read(uint8_t *pBuf, uint32_t Len) {
  uint32_t count = 0;
  while (count < Len && UserRxRingTail != UserRxRingHead) {
    pBuf[count++] = UserRxRingBuffer[UserRxRingTail];
    UserRxRingTail = (UserRxRingTail + 1) % APP_RX_DATA_SIZE;
  }
  return count;
}

/**
 * @brief  Check if a test command is pending (to be called from task context)
 * @retval Test ID (0-10, 99, 255 for interrupt) or 255 if no pending command
 * @note   v0.1.83: Use PendingCommandFlag to distinguish "no command" from "interrupt"
 */
uint8_t CDC_GetPendingCommand(void) {
  if (!PendingCommandFlag) {
    return 255;  // No command pending
  }
  uint8_t testId = PendingTestId;
  PendingTestId = 255;  // Clear to default value
  PendingCommandFlag = 0;  // v0.1.83: Clear the flag
  return testId;
}

/**
 * @brief  Check if a test command is pending without clearing it
 * @retval 1 if command pending, 0 if no pending command
 * @note   v0.1.83: Added to properly handle ATST999 (testId=255)
 *         Uses PendingCommandFlag since PendingTestId=255 is now a valid command.
 */
uint8_t CDC_HasPendingCommand(void) {
  return PendingCommandFlag;
}

/**
 * @brief  Process a pending test command (to be called from task context)
 * @param  testId: Test ID to process
 * @retval None
 * @note   This function is called from DefaultTask, not from USB interrupt
 *         v0.1.83: Added LCD display messages for test start/completion
 */
void CDC_ProcessPendingCommand(uint8_t testId) {
  extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);
  extern TestStatus_TypeDef TEST_ProcessCommand(uint8_t testId);
  extern TestStatus_TypeDef TEST_ProcessCommandWithParams(uint8_t testId, const char *params);
  extern void TEST_DisplayStartMessage(uint16_t testId, const char *testName);
  extern void TEST_DisplayCompleteMessage(uint16_t testId, const char *testName, TestStatus_TypeDef result);
  extern const char *TEST_GetName(uint8_t testId);

  char response[128];
  int respLen;
  TestStatus_TypeDef testResult = TEST_ERROR;  // Default to error

  // Map test ID back to display ID (255 -> 999 for interrupt command)
  uint16_t displayId = (testId == 255) ? 999 : testId;

  // Get test name for display
  const char *testName = TEST_GetName(testId);

  // v0.1.83: Display start message on LCD
  TEST_DisplayStartMessage(displayId, testName);

  // Send test start message to USB
  respLen = snprintf(response, sizeof(response), "\r\n*** Test ATST%d Start! ***\r\n", displayId);
  CDC_Transmit_ThreadSafe((uint8_t *)response, respLen);

  // Small delay to ensure USB is ready
  osDelay(10);

  // Check if this is a parameterized drawing command (151-158, 170)
  if ((testId >= 151 && testId <= 158) || testId == 170) {
    if (PendingDrawParamsLen > 0 || testId == 170) {
      // Convert volatile buffer to regular string for processing
      char params[128];
      if (PendingDrawParamsLen > 0) {
        memcpy(params, (void *)PendingDrawParams, PendingDrawParamsLen + 1);
        // Run the parameterized command
        testResult = TEST_ProcessCommandWithParams(testId, params);
        // Clear parameters
        PendingDrawParamsLen = 0;
      } else {
        testResult = TEST_ProcessCommand(testId);
      }
    } else {
      testResult = TEST_ProcessCommand(testId);
    }
  } else {
    // Run the test
    testResult = TEST_ProcessCommand(testId);
  }

  // Small delay before completion message
  osDelay(10);

  // v0.1.83: Display completion message on LCD
  TEST_DisplayCompleteMessage(displayId, testName, testResult);

  // Send completion message to USB
  respLen = snprintf(response, sizeof(response), "*** Test ATST%d Complete! ***\r\n> ", displayId);
  CDC_Transmit_ThreadSafe((uint8_t *)response, respLen);
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

