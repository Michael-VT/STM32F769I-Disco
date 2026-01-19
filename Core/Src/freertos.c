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
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h" // Added SDRAM Header
#include "stm32f769i_discovery_ts.h"
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

/* Definitions for touchTask */
osThreadId_t touchTaskHandle;
const osThreadAttr_t touchTask_attributes = {
    .name = "touchTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartTouchTask(void *argument);

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

  /* creation of touchTask */
  touchTaskHandle = osThreadNew(StartTouchTask, NULL, &touchTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE END Header_StartDefaultTask */
// CPU-based screen clearing (Bypasses DMA2D)
void Manual_LCD_Clear(uint32_t color) {
  uint32_t *fb = (uint32_t *)0xC0000000;

  CDC_Transmit_HS((uint8_t *)"  Accessing SDRAM...\r\n", 22);
  osDelay(50);

  // Test write
  fb[0] = color;
  CDC_Transmit_HS((uint8_t *)"  Test Write OK\r\n", 17);
  osDelay(50);

  for (uint32_t i = 1; i < 800 * 480; i++) {
    fb[i] = color;
    if (i % 80000 == 0) {
      CDC_Transmit_HS((uint8_t *)".", 1);
      osDelay(1); // Yield to other tasks (LED)
    }
  }
  CDC_Transmit_HS((uint8_t *)" Done\r\n", 7);
}

// Helper to remove UTF-8 and non-printable chars
void SanitizeASCII(char *str) {
  char *src = str;
  char *dst = str;
  while (*src) {
    if ((unsigned char)*src < 127 && (unsigned char)*src >= 32) {
      *dst++ = *src;
    } else if (*src == '\n' || *src == '\r') {
      // skip
    } else {
      // Replace common UTF-8 lead bytes or effectively strip
      // Simple approach: Skip all extended bytes
      // For '°' (C2 B0), we skip.
      // Ideally we could replace with ' ' but skipping is safer for now
      // *dst++ = ' ';
    }
    src++;
  }
  *dst = 0;
}

void Safe_Transmit(char *msg) {
  CDC_Transmit_HS((uint8_t *)msg, strlen(msg));
  osDelay(100); // Give VCP time to send
}

/* Initialize all configured peripherals */
extern void MX_GPIO_Init(void);
extern void MX_UART5_Init(void);
extern void MX_DMA2D_Init(void);
extern void MX_CRC_Init(void);
extern void MX_I2C1_Init(void);

/* Definition for DefaultTask */
void StartDefaultTask(void *argument) {
  // MX_GPIO_Init(); // These are typically called once in main() before RTOS
  // starts MX_UART5_Init(); MX_LTDC_Init();   // Handled by BSP_LCD_Init
  // MX_DMA2D_Init();
  // MX_CRC_Init();
  // MX_I2C1_Init();

  MX_USB_DEVICE_Init(); // MX_USB_DEVICE_Init() is usually called here but it's
                        // already in main() or properly handled.
  // Actually, let's keep it here if it's the standard CubeIDE place, but avoid
  // re-initing MX_USB_DEVICE_Init();
  extern void MX_DMA2D_Init(void);
  extern void MX_CRC_Init(void);

  osDelay(3000); // Wait for enumeration

  Safe_Transmit("=== FW v1.19 2026-01-20 00:00 ===\r\n");
  Safe_Transmit("[1] USB Ready\r\n");

  // 1. SDRAM Pattern Test (Verify health)
  Safe_Transmit("[2] SDRAM Test (1MB)...\r\n");
  BSP_SDRAM_Init();
  uint32_t *sdram_ptr = (uint32_t *)0xC0000000;
  uint8_t sdram_ok = 1;
  for (uint32_t i = 0; i < 0x40000; i++) {
    sdram_ptr[i] = i ^ 0xAAAA5555;
  }
  for (uint32_t i = 0; i < 0x40000; i++) {
    if (sdram_ptr[i] != (i ^ 0xAAAA5555)) {
      sdram_ok = 0;
      break;
    }
  }
  Safe_Transmit(sdram_ok ? "  SDRAM PASS\r\n" : "  SDRAM FAIL!\r\n");

  // 2. LCD Boot
  Safe_Transmit("[3] LCD Init...\r\n");
  if (BSP_LCD_Init() != LCD_OK) {
    Safe_Transmit("  LCD FAIL\r\n");
    for (;;)
      osDelay(1000);
  }
  Safe_Transmit("[4] LCD OK\r\n");

  Safe_Transmit("[5] Display On...\r\n");
  BSP_LCD_DisplayOn();
  osDelay(100);

  Safe_Transmit("[6] Layer Init (0xC0000000)...\r\n");
  BSP_LCD_LayerDefaultInit(0, 0xC0000000);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_SetLayerVisible(0, ENABLE);
  Safe_Transmit("[7] Layer OK\r\n");

  Safe_Transmit("[8] Set Font...\r\n");
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  Safe_Transmit("[9] Font OK\r\n");

  Safe_Transmit("[10] Set Colors...\r\n");
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  Safe_Transmit("[11] Colors OK\r\n");

  Safe_Transmit("[12] ESP Test...\r\n");
  ESP8266_Init();
  if (ESP8266_SendCommand("AT") == ESP8266_OK) {
    Safe_Transmit("  AT OK\r\n");
  } else {
    Safe_Transmit("  AT FAIL\r\n");
  }

  // 4. Hardware Stabilizers
  Safe_Transmit("[13] Hardware Polish...\r\n");
  MX_CRC_Init();   // Some BSPs need CRC for ID validation
  MX_DMA2D_Init(); // Enable hardware accelerator
  Safe_Transmit("  DMA2D/CRC OK\r\n");

  Safe_Transmit("[14] Screen Clear (DMA2D)...\r\n");
  // Precise single frame clear (800x480x4 bytes)
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(0, 0, 800, 480);
  Safe_Transmit("  Clear Done\r\n");

  Safe_Transmit("[15] LCD Warm-up...\r\n");
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)"WEATHER STATION v1.19",
                          CENTER_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"STM32F769I-DISCO READY",
                          CENTER_MODE);
  Safe_Transmit("  UI Ready\r\n");

  Safe_Transmit("[16] WiFi Connect...\r\n");
  ESP8266_SendCommand("AT+CWMODE=1");
  osDelay(500);
  ESP8266_SendCommand("AT+CWJAP=\"MEO-EDC8ED\",\"2668EB941B\"");

  // Wait for IP (Robust sync)
  Safe_Transmit("  Waiting for IP...\r\n");
  if (ESP8266_WaitFor("WIFI GOT IP", 15000) == ESP8266_OK) {
    Safe_Transmit("  WiFi Connected OK\r\n");
  } else {
    Safe_Transmit("  WiFi Timeout (Check credentials)\r\n");
  }

  Safe_Transmit("[16] Starting App Loop\r\n");

  /* Infinite loop */
  for (;;) {
    Safe_Transmit("--- Updating Weather ---\r\n");

    // UI Update: Show "FETCHING..."
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, 200, 800, 100);
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_DisplayStringAt(0, 250, (uint8_t *)"FETCHING DATA...", CENTER_MODE);

    ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"wttr.in\",80");
    if (ESP8266_WaitFor("OK", 5000) != ESP8266_OK) {
      Safe_Transmit("  TCP Connect Failed\r\n");
      osDelay(5000);
      continue;
    }

    const char *req_line = "GET /Peniche?format=%C;%t;%w;%h;%m;%o HTTP/1.1\r\n";
    const char *host_line = "Host: wttr.in\r\n";
    const char *ua_line = "User-Agent: curl/7.68.0\r\n";
    const char *conn_line = "Connection: close\r\n\r\n";
    uint32_t total_len = strlen(req_line) + strlen(host_line) +
                         strlen(ua_line) + strlen(conn_line);

    char sendCmd[32];
    sprintf(sendCmd, "AT+CIPSEND=%lu", total_len);
    ESP8266_SendCommand(sendCmd);

    if (ESP8266_WaitFor(">", 2000) == ESP8266_OK) {
      HAL_UART_Transmit(&huart5, (uint8_t *)req_line, strlen(req_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)host_line, strlen(host_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)ua_line, strlen(ua_line), 100);
      HAL_UART_Transmit(&huart5, (uint8_t *)conn_line, strlen(conn_line), 100);

      static char rx_buf[2048];
      uint32_t rx_idx = 0;
      uint32_t start = HAL_GetTick();

      while (HAL_GetTick() - start < 5000 && rx_idx < sizeof(rx_buf) - 1) {
        uint8_t ch;
        if (UART_Read(&ch, 1) > 0) {
          rx_buf[rx_idx++] = ch;
          start = HAL_GetTick();
        } else {
          osDelay(1);
        }
      }
      rx_buf[rx_idx] = 0;

      char *body = strstr(rx_buf, "\r\n\r\n");
      if (body) {
        body += 4;
        SanitizeASCII(body);
        Safe_Transmit("WEATHER: ");
        Safe_Transmit(body);
        Safe_Transmit("\r\n");

        // Parse: Condition;Temp;Wind;Humidity;...
        char condition[64] = "Unknown";
        char temp[32] = "N/A";
        char wind[32] = "N/A";
        char hum[32] = "N/A";

        char *ptr = body;
        char *next;

        // 1. Condition
        if ((next = strchr(ptr, ';')) != NULL) {
          int len = next - ptr;
          if (len > 63)
            len = 63;
          strncpy(condition, ptr, len);
          condition[len] = 0;
          ptr = next + 1;

          // 2. Temp
          if ((next = strchr(ptr, ';')) != NULL) {
            len = next - ptr;
            if (len > 31)
              len = 31;
            strncpy(temp, ptr, len);
            temp[len] = 0;
            ptr = next + 1;

            // 3. Wind
            if ((next = strchr(ptr, ';')) != NULL) {
              len = next - ptr;
              if (len > 31)
                len = 31;
              strncpy(wind, ptr, len);
              wind[len] = 0;
              ptr = next + 1;

              // 4. Humidity
              if ((next = strchr(ptr, ';')) != NULL) {
                len = next - ptr;
                if (len > 31)
                  len = 31;
                strncpy(hum, ptr, len);
                hum[len] = 0;
              }
            }
          }
        }

        // --- PRE-CLEAR ---
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_FillRect(0, 100, 800, 380);

        // --- CITY (Font48) ---
        BSP_LCD_SetFont(&Font48);
        BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
        BSP_LCD_DisplayStringAt(0, 120, (uint8_t *)"PENICHE", CENTER_MODE);

        // --- CONDITION (Font48) ---
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        BSP_LCD_DisplayStringAt(0, 180, (uint8_t *)condition, CENTER_MODE);

        // --- TEMPERATURE (Font96 / Giga) ---
        BSP_LCD_SetFont(&Font96);
        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
        char tempStr[64];
        sprintf(tempStr, "%s", temp); // Just the number + C
        BSP_LCD_DisplayStringAt(0, 260, (uint8_t *)tempStr, CENTER_MODE);

        // --- DETAILS (Font24) ---
        BSP_LCD_SetFont(&Font24);
        BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
        char details[128];
        sprintf(details, "WIN: %s | HUM: %s", wind, hum);
        BSP_LCD_DisplayStringAt(0, 380, (uint8_t *)details, CENTER_MODE);
      }
    }

    osDelay(30000); // 30s refresh
  }
  /* USER CODE END StartDefaultTask */
}

/* Definition for TouchTask */
void StartTouchTask(void *argument) {
  osDelay(5000);
  for (;;) {
    osDelay(1000);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
