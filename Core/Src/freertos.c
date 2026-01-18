/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart5;
extern uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 4096 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created. It is also called by various parts of the
  demo application. If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  ESP8266_Init();

  // Initial delay for USB enumeration
  osDelay(2000);

  // Rapid Blink to indicate alive
  for (int i = 0; i < 10; i++) {
    HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_13);
    osDelay(100);
  }
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET); // LED ON

  char msg[128];

  // Connect WiFi
  sprintf(msg, "Connecting to WiFi...\r\n");
  CDC_Transmit_HS((uint8_t *)msg, strlen(msg));

  ESP8266_SendCommand("AT+CWMODE=1");
  osDelay(500);
  ESP8266_SendCommand("AT+CWJAP=\"MEO-EDC8ED\",\"2668EB941B\"");
  osDelay(8000); // Give it time to connect

  sprintf(msg, "Starting Weather Loop...\r\n");
  CDC_Transmit_HS((uint8_t *)msg, strlen(msg));

  /* Infinite loop */
  for (;;) {
    // 1. TCP Connect
    ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"wttr.in\",80");
    if (ESP8266_WaitFor("OK", 5000) != ESP8266_OK) {
      sprintf(msg, "TCP Connection Failed\r\n");
      CDC_Transmit_HS((uint8_t *)msg, strlen(msg));
      osDelay(5000);
      continue;
    }

    // 2. Prepare HTTP Request
    const char *req_line = "GET /Peniche?format=%C+%t+%w+%h+%m+%o HTTP/1.1\r\n";
    const char *host_line = "Host: wttr.in\r\n";
    const char *ua_line = "User-Agent: curl/7.68.0\r\n";
    const char *conn_line = "Connection: close\r\n\r\n";

    // 3. Send CIPSEND
    // Calculate exact length
    uint32_t total_len = strlen(req_line) + strlen(host_line) +
                         strlen(ua_line) + strlen(conn_line);
    char sendCmd[32];
    sprintf(sendCmd, "AT+CIPSEND=%lu", total_len);
    ESP8266_SendCommand(sendCmd);

    if (ESP8266_WaitFor(">", 2000) == ESP8266_OK) {
      // 4. Send Data
      HAL_UART_Transmit(&huart5, (uint8_t *)req_line, strlen(req_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)host_line, strlen(host_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)ua_line, strlen(ua_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)conn_line, strlen(conn_line), 100);

      // 5. Read Response
      // We read continuously and forward to USB until timeout (waiting for
      // body)
      uint32_t start = HAL_GetTick();
      while (HAL_GetTick() - start < 10000) {
        uint8_t rBuf[64];
        uint32_t n = UART_Read(rBuf, sizeof(rBuf));
        if (n > 0) {
          CDC_Transmit_HS(rBuf, n);
          start = HAL_GetTick(); // extend timeout on data
        } else {
          osDelay(10);
        }
      }
      CDC_Transmit_HS((uint8_t *)"\r\n", 2);
    } else {
      sprintf(msg, "CIPSEND Failed\r\n");
      CDC_Transmit_HS((uint8_t *)msg, strlen(msg));
    }

    // Blink while waiting (10 seconds)
    for (int i = 0; i < 10; i++) {
      HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_13);
      osDelay(1000);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
