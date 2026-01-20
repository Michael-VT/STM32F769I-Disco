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
extern UART_HandleTypeDef huart5;
extern uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);

/* Global System Ready flag */
volatile uint8_t systemReady = 0;
/* USER CODE END Variables */

/* Definitions for defaultTask */
// [v1.36 Data Struct]
typedef struct {
  char date[16];
  char time[16];
  char condition[64];
  char temp[16];
  char humidity[16];
  char pressure[16];
  char moon[16];
  char wind[32];
  uint8_t synced;
} WeatherData_t;

WeatherData_t latestWeather = {
    .date = "2026-01-20", .time = "00:00:00", .synced = 0};

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
// CPU-based screen clearing (v1.34: 24-bit RGB888)
void Manual_LCD_Clear(uint32_t color) {
  uint8_t *fb_24 = (uint8_t *)0xC0000000;
  uint8_t r = (uint8_t)(color >> 16);
  uint8_t g = (uint8_t)(color >> 8);
  uint8_t b = (uint8_t)(color);

  CDC_Transmit_HS((uint8_t *)"  Accessing SDRAM (24-bit)...\r\n", 31);
  osDelay(50);

  for (uint32_t i = 0; i < 800 * 480; i++) {
    fb_24[i * 3 + 0] = r;
    fb_24[i * 3 + 1] = g;
    fb_24[i * 3 + 2] = b;
    if (i % 80000 == 0) {
      CDC_Transmit_HS((uint8_t *)".", 1);
      osDelay(1);
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

void StartDefaultTask(void *argument) {
  MX_USB_DEVICE_Init();
  osDelay(3000); // v1.35: Wait for VCP enumeration

  // v1.35 UNIFIED DATA PATH
  Safe_Transmit("\r\n=== FW v1.35 BOOTING ===\r\n");

  extern DMA2D_HandleTypeDef hdma2d;
  extern LTDC_HandleTypeDef hltdc_discovery;
  extern DMA2D_HandleTypeDef hdma2d_discovery;
  extern DSI_HandleTypeDef hdsi_discovery;
  extern RTC_HandleTypeDef hrtc;
  extern void MX_DMA2D_Init(void);
  extern void MX_CRC_Init(void);

  // 1. SDRAM Init & CPU-Direct Test
  Safe_Transmit("[1] SDRAM Init...\r\n");
  BSP_SDRAM_Init();

  // Direct CPU write (v1.33: 24-bit RGB888)
  uint8_t *fb_24_init = (uint8_t *)0xC0000000;
  for (uint32_t i = 0; i < 800 * 100; i++) {
    fb_24_init[i * 3 + 0] = 0x00; // R
    fb_24_init[i * 3 + 1] = 0xFF; // G
    fb_24_init[i * 3 + 2] = 0x00; // B
  }

  Safe_Transmit("  SDRAM CPU Write OK\r\n");

  // 2. LCD Hardware Reset & Boot
  Safe_Transmit("[2] LCD Reset & Init...\r\n");
  BSP_LCD_Reset(); // Physical XRES toggle
  osDelay(500);    // Wait after reset

  if (BSP_LCD_Init() != LCD_OK) {
    Safe_Transmit("  LCD FAIL\r\n");
    for (;;)
      osDelay(1000);
  }
  osDelay(500);
  BSP_LCD_DisplayOn();
  BSP_LCD_LayerDefaultInit(0, 0xC0000000);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_SetLayerVisible(0, ENABLE);

  // Diagnostic: Check DSI Error Code
  char dsi_st[64];
  sprintf(dsi_st, "  DSI ErrCode: 0x%08lX\r\n", hdsi_discovery.ErrorCode);
  Safe_Transmit(dsi_st);

  Safe_Transmit("  LCD READY\r\n");

  // 3. Hardware Sync & Diagnostic Pattern
  Safe_Transmit("[3] DMA2D Sync...\r\n");
  MX_CRC_Init();
  MX_DMA2D_Init();
  extern DMA2D_HandleTypeDef hdma2d;
  hdma2d_discovery = hdma2d; // CRITICAL: Sync handles

  // v1.33: Manual CPU Clear (24-bit RGB888)
  uint8_t *fb_24 = (uint8_t *)0xC0000000;
  for (uint32_t i = 0; i < 800 * 480 * 3; i++)
    fb_24[i] = 0; // Black (R=0, G=0, B=0)

  // CPU-Direct Pattern: Blue stripe at top
  for (uint32_t i = 0; i < 800 * 20; i++) {
    fb_24[i * 3 + 0] = 0x00; // R
    fb_24[i * 3 + 1] = 0x00; // G
    fb_24[i * 3 + 2] = 0xFF; // B
  }

  Safe_Transmit("  Pattern OK\r\n");

  // 4. Touch Init
  Safe_Transmit("[4] Touch Init...\r\n");
  if (BSP_TS_Init(800, 480) == TS_OK) {
    Safe_Transmit("  TOUCH OK\r\n");
  } else {
    Safe_Transmit("  TOUCH FAIL\r\n");
  }

  // 5. WiFi Init
  Safe_Transmit("[5] WiFi Connect...\r\n");
  ESP8266_Init(); // Returns void
  ESP8266_SendCommand("AT+CWMODE=1");
  osDelay(500);
  // Using credentials from turn 1701 history
  ESP8266_SendCommand("AT+CWJAP=\"MEO-EDC8ED\",\"2668EB941B\"");

  Safe_Transmit("  Waiting for IP...\r\n");
  if (ESP8266_WaitFor("WIFI GOT IP", 15000) == ESP8266_OK) {
    Safe_Transmit("  WiFi Connected OK\r\n");
  } else {
    Safe_Transmit("  WiFi Join Timeout\r\n");
  }

  // FLAG SYSTEM READY - Unlocks StartTouchTask
  systemReady = 1;
  Safe_Transmit("[6] SYSTEM READY\r\n");

  // v1.35 UI Banner
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"PENICHE WEATHER v1.35",
                          CENTER_MODE);

  char condition[64] = "WAITING...";
  char temp[64] = "N/A";
  char wind[32] = "N/A";
  char hum[32] = "N/A";

  for (;;) {
    Safe_Transmit("--- Updating Weather ---\r\n");

    Safe_Transmit("WiFi: Connecting...\r\n");
    ESP8266_SendCommand("AT+CIPCLOSE");
    osDelay(100);
    ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"137.184.234.34\",80");
    if (ESP8266_WaitFor("OK", 5000) == ESP8266_OK) {
      Safe_Transmit("WiFi: TCP OK. Sending...\r\n");
      const char *req =
          "GET /Peniche?format=%C;%t;%w;%h HTTP/1.1\r\nHost: "
          "wttr.in\r\nUser-Agent: curl/7.81.0\r\nConnection: close\r\n\r\n";
      char sendCmd[32];
      sprintf(sendCmd, "AT+CIPSEND=%d", (int)strlen(req));
      ESP8266_SendCommand(sendCmd);

      if (ESP8266_WaitFor(">", 2000) == ESP8266_OK) {
        Safe_Transmit("WiFi: Sent. Reading...\r\n");
        HAL_UART_Transmit(&huart5, (uint8_t *)req, strlen(req), 100);

        static char rx_buf[1024];
        uint32_t rx_idx = 0;
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < 5000 && rx_idx < sizeof(rx_buf) - 1) {
          uint8_t ch;
          if (UART_Read(&ch, 1) > 0) {
            rx_buf[rx_idx++] = ch;
            start = HAL_GetTick();
          } else
            osDelay(1);
        }
        rx_buf[rx_idx] = 0;

        // v1.27 VERBOSE DEBUG
        Safe_Transmit("RAW RESPONSE:\r\n");
        if (rx_idx > 128) {
          char debug_chunk[129];
          memcpy(debug_chunk, rx_buf, 128);
          debug_chunk[128] = 0;
          Safe_Transmit(debug_chunk);
          Safe_Transmit("...\r\n");
        } else {
          Safe_Transmit(rx_buf);
        }

        char *body = strstr(rx_buf, "\r\n\r\n");
        if (body) {
          body += 4;
          // Skip +IPD prefix if present
          if (strncmp(body, "+IPD,", 5) == 0) {
            char *colon = strchr(body, ':');
            if (colon)
              body = colon + 1;
          }

          char *token = strtok(body, ";");
          if (token) {
            strncpy(condition, token, 63);
            SanitizeASCII(condition);
            token = strtok(NULL, ";");
            if (token) {
              strncpy(temp, token, 63);
              SanitizeASCII(temp);
              token = strtok(NULL, ";");
              if (token) {
                strncpy(wind, token, 31);
                SanitizeASCII(wind);
                token = strtok(NULL, ";");
                if (token) {
                  strncpy(hum, token, 31);
                  char *end = strchr(hum, ';');
                  if (end)
                    *end = 0;
                  SanitizeASCII(hum);
                }
              }
            }
          }
        }
      }
    }

    // UI Refresh (v1.30 Surgical update)
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, 150, 800, 330); // Leave banner/diagnostic area

    BSP_LCD_SetFont(&Font48);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); // RB Fix: Yellow -> Cyan
    BSP_LCD_DisplayStringAt(0, 180, (uint8_t *)"PENICHE", CENTER_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(0, 260, (uint8_t *)condition, CENTER_MODE);

    BSP_LCD_SetFont(&Font96);
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    BSP_LCD_DisplayStringAt(0, 320, (uint8_t *)temp, CENTER_MODE);

    // v1.33 Heartbeat Pixel (bottom-right)
    static uint32_t heart = 0;
    uint8_t *hb_ptr = (uint8_t *)0xC0000000 + (800 * 480 - 1) * 3;
    if (heart++ % 2) {
      hb_ptr[0] = 0xFF;
      hb_ptr[1] = 0xFF;
      hb_ptr[2] = 0xFF;
    } else {
      hb_ptr[0] = 0x00;
      hb_ptr[1] = 0x00;
      hb_ptr[2] = 0x00;
    }

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    char details[128];
    sprintf(details, "WIN: %s | HUM: %s", wind, hum);
    BSP_LCD_DisplayStringAt(0, 420, (uint8_t *)details, CENTER_MODE);

    char alive[64];
    sprintf(alive, "v1.35 Alive - Heap: %d | DSI Err: 0x%lx\r\n",
            (int)xPortGetFreeHeapSize(), hdsi_discovery.ErrorCode);
    Safe_Transmit(alive);
    osDelay(30000);
  }
}

/* Definition for TouchTask */
void StartTouchTask(void *argument) {
  TS_StateTypeDef TS_State;

  // Wait for main initialization to finish
  while (systemReady == 0) {
    osDelay(100);
  }

  Safe_Transmit("[TouchTask] Active\r\n");

  for (;;) {
    BSP_TS_GetState(&TS_State);
    if (TS_State.touchDetected) {
      uint16_t x = TS_State.touchX[0];
      uint16_t y = TS_State.touchY[0];

      // v1.29: Drawing disabled in TouchTask to prevent concurrency hangs
      // BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      // BSP_LCD_FillCircle(x, y, 5);

      char buf[64];
      sprintf(buf, "TOUCH: %d, %d\r\n", x, y);
      Safe_Transmit(buf);
      osDelay(100);
    }
    osDelay(50);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
