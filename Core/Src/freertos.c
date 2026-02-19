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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h" // Added SDRAM Header
#include "stm32f769i_discovery_ts.h"
#include "usbd_cdc_if.h"
#include "rtc.h"
#include "test_system.h"
#include "VERSION.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// WiFi credentials for ESP8266
#define WIFI_SSID     "MEO-EDC8ED"
#define WIFI_PASSWORD "2668EB941B"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Semaphore to signal ESP8266 initialization complete
osSemaphoreId_t esp8266InitSemId = NULL;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 8192,  // Increased from 4096 to prevent stack overflow with HTTP debug buffers
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void MX_UART5_Init(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

// External UART variables for ESP8266 debug
extern UART_HandleTypeDef huart5;
#define UART_RX_BUFFER_SIZE 2048
extern volatile uint8_t UartRxBuffer[UART_RX_BUFFER_SIZE];
extern volatile uint32_t UartRxHead;
extern volatile uint32_t UartRxTail;
extern uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);

void StartDebugTask(void *argument);
void StartDisplayTask(void *argument);
void StartLedTask(void *argument);

/* LED Control Functions */
void LED_Init(void);
void LED_On(uint8_t led);
void LED_Off(uint8_t led);
void LED_Toggle(uint8_t led);

/* LED Activity Indicators (called from other tasks) */
extern volatile uint8_t ledCdcActivity;
extern volatile uint8_t ledEspActivity;

/* Debug forwarding control - used to disable DebugTask during critical operations */
extern volatile uint8_t debugForwardingEnabled;

/* Thread-safe CDC transmit wrapper */
extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);

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

  // Blink LED1 rapidly to indicate stack overflow
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  GPIO_InitStruct.Pin = LED1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  // Rapid blink forever
  while (1) {
    HAL_GPIO_TogglePin(GPIOJ, LED1_PIN);
    HAL_Delay(50);
  }
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
  // Create mutex to protect UART5 transmission from concurrent access
  // (ESP8266_SendCommand and CDC_Receive_HS both use HAL_UART_Transmit)
  extern osMutexId_t uart5MutexId;
  const osMutexAttr_t uart5Mutex_attributes = {
    .name = "uart5Mutex"
  };
  uart5MutexId = osMutexNew(&uart5Mutex_attributes);

  // Create mutex to protect USB CDC transmission from concurrent access
  // (DisplayTask, DebugTask, DefaultTask all call CDC_Transmit_ThreadSafe)
  extern osMutexId_t cdcTransmitMutexId;
  const osMutexAttr_t cdcTransmitMutex_attributes = {
    .name = "cdcTransmitMutex"
  };
  cdcTransmitMutexId = osMutexNew(&cdcTransmitMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // Create binary semaphore to signal ESP8266 initialization complete
  const osSemaphoreAttr_t esp8266InitSem_attributes = {
    .name = "esp8266InitSem"
  };
  esp8266InitSemId = osSemaphoreNew(1, 0, &esp8266InitSem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* creation of debugTask */
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);
  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);
  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

  // Wait for USB CDC to be ready
  osDelay(500);

  // Enable UART5 RXNE interrupt now that FreeRTOS is running
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  osDelay(10);

  // Send startup message with version info
  char startupMsg[64];
  snprintf(startupMsg, sizeof(startupMsg),
           "[STARTUP] v%s %s >\r\n[BEGIN]\r\n",
           FIRMWARE_VERSION_STRING, FIRMWARE_BUILD_DATE);
  CDC_Transmit_ThreadSafe((uint8_t *)startupMsg, strlen(startupMsg));

  // Initialize test system
  TEST_Init();

  // Give ESP8266 time to boot
  osDelay(2000);

  // Initialize ESP8266
  ESP8266_Init();
  osDelay(500);  // Give ESP8266 more time to stabilize after init

  // Signal ESP8266 init complete (DisplayTask is waiting)
  if (esp8266InitSemId != NULL) {
    osSemaphoreRelease(esp8266InitSemId);
  }

  // Clear ring buffer
  extern volatile uint32_t UartRxHead;
  extern volatile uint32_t UartRxTail;
  UartRxHead = 0;
  UartRxTail = 0;

  /* Infinite loop */
  for(;;)
  {
    // Check for pending ATST commands (from USB CDC receive)
    // v0.1.83: Fixed to properly handle ATST999 (testId=255)
    extern uint8_t CDC_GetPendingCommand(void);
    extern uint8_t CDC_HasPendingCommand(void);  // New function: returns 1 if command pending
    extern void CDC_ProcessPendingCommand(uint8_t testId);

    if (CDC_HasPendingCommand()) {
      uint8_t pendingTest = CDC_GetPendingCommand();
      // Process the command in task context (not interrupt)
      // This now includes testId=255 (interrupt command)
      CDC_ProcessPendingCommand(pendingTest);
    }

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
  * @brief  Function implementing the debugTask thread.
  *         Forwards UART5 (ESP8266) data to USB CDC for debug output.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */
  // DebugTask forwards ESP8266 responses from UART5 to USB CDC
  // This helps debug communication by showing what the ESP8266 sends back
  uint8_t rxBuffer[128];
  uint8_t filteredBuffer[256];  // Buffer for filtered output (may expand escapes)
  uint32_t bytesRead;

  // Small delay to let other tasks initialize
  osDelay(500);

  /* Infinite loop - read from UART5 ring buffer and forward to USB CDC */
  for(;;)
  {
    // Only forward if enabled (can be disabled during critical operations like SNTP)
    if (debugForwardingEnabled) {
      bytesRead = UART_Read(rxBuffer, sizeof(rxBuffer) - 1);
      if (bytesRead > 0) {
        // Filter non-printable characters and forward
        uint32_t filteredIdx = 0;
        for (uint32_t i = 0; i < bytesRead && filteredIdx < sizeof(filteredBuffer) - 1; i++) {
          uint8_t ch = rxBuffer[i];
          // Only forward printable ASCII characters (32-126) and CR/LF/TAB
          if (ch >= 32 && ch <= 126) {
            filteredBuffer[filteredIdx++] = ch;
          } else if (ch == '\r' || ch == '\n' || ch == '\t') {
            filteredBuffer[filteredIdx++] = ch;
          }
          // Skip other control characters
        }
        if (filteredIdx > 0) {
          CDC_Transmit_ThreadSafe(filteredBuffer, filteredIdx);
        }
      }
    }
    // Small delay to prevent busy-waiting
    osDelay(10);
  }
  /* USER CODE END StartDebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/* LCD Display Task - Now outputs to USB CDC */
/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the displayTask thread.
 *        Outputs RTC time, date, and weather info to USB CDC virtual port.
 *        LCD display disabled due to DSI/LTDC timing issues.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  extern RTC_HandleTypeDef hrtc;
  char msgBuffer[256];
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  char weatherBuffer[128] = "";
  uint32_t weatherCounter = 0;
  const uint32_t WEATHER_UPDATE_INTERVAL = 600; // v0.1.79: Update weather every 600 seconds (10 minutes)

  // Variable to track if NTP sync was attempted
  uint8_t ntpSyncDone = 0;
  uint8_t ntpAttempts = 0;
  const uint8_t MAX_NTP_ATTEMPTS = 3;

  // Wait for ESP8266 initialization to complete (signaled by DefaultTask)
  if (esp8266InitSemId != NULL) {
    osSemaphoreAcquire(esp8266InitSemId, osWaitForever);
  }
  osDelay(500);  // Give ESP8266 more time after init semaphore

  // DEBUG: Output LTDC Layer 0 configuration for display artifact diagnosis
  extern LTDC_HandleTypeDef hltdc_discovery;
  uint32_t cfblr = LTDC_LAYER(&hltdc_discovery, 0)->CFBLR;
  uint32_t whpcr = LTDC_LAYER(&hltdc_discovery, 0)->WHPCR;
  uint32_t wvpcr = LTDC_LAYER(&hltdc_discovery, 0)->WVPCR;
  uint32_t cr = LTDC_LAYER(&hltdc_discovery, 0)->CR;
  uint32_t cfbar = LTDC_LAYER(&hltdc_discovery, 0)->CFBAR;

  char debugBuf[256];
  snprintf(debugBuf, sizeof(debugBuf),
           "\r\n=== LTDC Layer 0 Debug ===\r\n"
           "CFBLR: 0x%08lx (LineLen=%lu, Pitch=%lu)\r\n"
           "CFBAR: 0x%08lx (FrameBufferAddr)\r\n"
           "WHPCR: 0x%08lx (WindowX0=%lu, WindowX1=%lu)\r\n"
           "WVPCR: 0x%08lx (WindowY0=%lu, WindowY1=%lu)\r\n"
           "CR: 0x%08lx (LayerEnabled=%lu)\r\n",
           cfblr,
           ((cfblr >> 16) & 0x1FFF),              // CFBLL (line length in bytes)
           (cfblr & 0x1FFF),                      // CFBP (pitch in bytes)
           cfbar,                          // Frame buffer address
           whpcr,
           (whpcr & 0x1FFF),              // WHSTPOS (window X0)
           ((whpcr >> 16) & 0x1FFF),      // WHSPPOS (window X1)
           wvpcr,
           (wvpcr & 0x1FFF),              // WVSTPOS (window Y0)
           ((wvpcr >> 16) & 0x1FFF),      // WVSPPOS (window Y1)
           cr,
           (uint32_t)((cr & LTDC_LxCR_LEN) ? 1 : 0)   // LEN (layer enable)
  );
  CDC_Transmit_ThreadSafe((uint8_t *)debugBuf, strlen(debugBuf));

  // Connect to WiFi first (required before SNTP and HTTP)
  // Add retry logic - WiFi connection can fail due to timing
  ESP8266_StatusTypeDef wifiStatus = ESP8266_ERROR;
  const uint8_t MAX_WIFI_RETRIES = 3;

  for (uint8_t retry = 0; retry < MAX_WIFI_RETRIES && wifiStatus != ESP8266_OK; retry++) {
    if (retry > 0) {
      // Small delay before retry
      osDelay(500);
    }
    wifiStatus = ESP8266_Connect(WIFI_SSID, WIFI_PASSWORD);
  }

  // Initial weather fetch (only if WiFi connected)
  if (wifiStatus == ESP8266_OK) {
    // Give ESP8266 more time after WiFi connection before HTTP requests
    // The AT+CWJAP returns OK but DHCP/network setup needs more time
    osDelay(2000);

    // Use OpenWeatherMap API instead of wttr.in (wttr.in is being blocked)
    ESP8266_StatusTypeDef httpResult = ESP8266_GetWeather(weatherBuffer, sizeof(weatherBuffer) - 1);
    if (httpResult != ESP8266_OK) {
      // HTTP failed - clear buffer to show N/A
      weatherBuffer[0] = '\0';
    }
    osDelay(100);
  } else {
    // Mark WiFi as failed - will try again in main loop
    weatherBuffer[0] = '\0';
  }

  osDelay(100);

  // Check if RTC needs to be set (default year = 0 means 2000, not initialized)
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  if (sDate.Year == 0 && !ntpSyncDone) {
    // Use ESP8266's built-in SNTP functionality
    ESP8266_SendCommand("AT+CIPSNTPCFG=1,0,\"time.google.com\",\"time.cloudflare.com\"");
    osDelay(500);

    // Wait for SNTP config confirmation
    uint8_t ch;
    while (UART_Read(&ch, 1) > 0);  // Clear any pending response
    ESP8266_WaitFor("OK", 2000);

    // Step 2: Try multiple times to get SNTP time
    for (ntpAttempts = 0; ntpAttempts < MAX_NTP_ATTEMPTS && !ntpSyncDone; ntpAttempts++) {
      // Clear any pending data
      while (UART_Read(&ch, 1) > 0) osDelay(1);

      // Disable debug forwarding during SNTP collection
      debugForwardingEnabled = 0;
      osDelay(50);

      // Clear any pending data in UART buffer before sending command
      while (UART_Read(&ch, 1) > 0) osDelay(1);

      // Request SNTP time
      ESP8266_SendCommand("AT+CIPSNTPTIME?");

      // Wait for response
      uint32_t startTime = HAL_GetTick();
      char responseBuffer[256];
      uint16_t respIdx = 0;
      memset(responseBuffer, 0, sizeof(responseBuffer));

      osDelay(500);  // Wait for response to arrive

      while (HAL_GetTick() - startTime < 3000) {
        if (UART_Read(&ch, 1) > 0) {
          if (respIdx < sizeof(responseBuffer) - 1) {
            responseBuffer[respIdx++] = ch;
            responseBuffer[respIdx] = '\0';
          }

          if (respIdx > 20 && strstr(responseBuffer, "OK") != NULL &&
              strstr(responseBuffer, "+CIPSNTPTIME:") != NULL) {
            break;
          }
        }
      }

      // Re-enable debug forwarding
      debugForwardingEnabled = 1;

      // Parse SNTP time from response
      char *timeStart = strstr(responseBuffer, "+CIPSNTPTIME:");
      if (timeStart != NULL) {
        char *content = strchr(timeStart, ':');
        if (content != NULL) {
          content++; // Skip ':'

          // Parse the date/time string
          // Format: "Mon Feb 09 12:41:08 2026"
          char weekdayStr[16], monthStr[16], dayStr[8], timeStr[16], yearStr[8];
          int scanned = sscanf(content, "%3s %3s %2s %8s %4s",
                              weekdayStr, monthStr, dayStr, timeStr, yearStr);

          if (scanned >= 5) {
            // Parse month
            uint8_t month = 1;
            const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
            for (int m = 0; m < 12; m++) {
              if (strncmp(monthStr, months[m], 3) == 0) {
                month = m + 1;
                break;
              }
            }

            // Parse day, year
            uint8_t day = atoi(dayStr);
            uint16_t year = atoi(yearStr);

            // Parse time HH:MM:SS
            uint8_t hours = 0, minutes = 0, seconds = 0;
            // Use %d instead of %hhu for more reliable parsing
            int h=0, m=0, s=0;
            int timeScanned = sscanf(timeStr, "%d:%d:%d", &h, &m, &s);
            if (timeScanned == 3) {
              hours = (uint8_t)h;
              minutes = (uint8_t)m;
              seconds = (uint8_t)s;
            }

            // DEBUG: Show time parsing result
            snprintf(msgBuffer, sizeof(msgBuffer), "[SNTP] timeScanned=%d, time=%02d:%02d:%02d\r\n",
                     timeScanned, hours, minutes, seconds);
            CDC_Transmit_ThreadSafe((uint8_t *)msgBuffer, strlen(msgBuffer));

            // Validate values
            if (year >= 2000 && year <= 2100 && month >= 1 && month <= 12 &&
                day >= 1 && day <= 31 && hours < 24 && minutes < 60 && seconds < 60) {

              // Calculate weekday
              uint16_t totalDays = (year - 2000) * 365 + (year - 2000) / 4;
              const uint16_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
              uint8_t isLeap = ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0);
              for (int m = 1; m < month; m++) {
                totalDays += daysInMonth[m - 1] + (m == 2 && isLeap ? 1 : 0);
              }
              totalDays += day - 1;
              uint8_t weekday = ((totalDays + 6) % 7) + 1;

              // Set RTC
              sTime.Hours = hours;
              sTime.Minutes = minutes;
              sTime.Seconds = seconds;
              sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
              sTime.StoreOperation = RTC_STOREOPERATION_RESET;

              sDate.Year = year - 2000;
              sDate.Month = month;
              sDate.Date = day;
              sDate.WeekDay = weekday;

              HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
              HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
              ntpSyncDone = 1;
            }
          }
        }
      }

      if (!ntpSyncDone) {
        osDelay(1000);
      }
    }
  }

  /* Infinite loop - output time/date every 10 seconds */
  for(;;)
  {
    // Get RTC time and date
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Calculate uptime
    uint32_t uptimeTicks = osKernelGetTickCount();
    uint32_t uptimeTotalSecs = uptimeTicks / 1000;  // FreeRTOS ticks are ms
    uint32_t uptimeHours = uptimeTotalSecs / 3600;
    uint32_t uptimeMins = (uptimeTotalSecs % 3600) / 60;
    uint32_t uptimeSecs = uptimeTotalSecs % 60;

    // Update weather every 60 seconds (6 iterations of 10-second loop)
    weatherCounter++;
    if (weatherCounter >= WEATHER_UPDATE_INTERVAL / 10) {
      weatherCounter = 0;
      // Give ESP8266 time to be ready for HTTP (might be idle)
      osDelay(100);
      // Try to fetch weather from OpenWeatherMap - don't worry if it fails, we'll try again next time
      ESP8266_StatusTypeDef httpResult = ESP8266_GetWeather(weatherBuffer, sizeof(weatherBuffer) - 1);
      if (httpResult != ESP8266_OK) {
        // Clear buffer on failure so it shows N/A
        weatherBuffer[0] = '\0';
      }
    }

    // Format time and date string - use single snprintf for reliability
    const char *weekdays[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
    const char *weekday = (sDate.WeekDay >= 1 && sDate.WeekDay <= 7) ? weekdays[sDate.WeekDay - 1] : "???";

    snprintf(msgBuffer, sizeof(msgBuffer),
             "\r\n[%02d:%02d:%02d] %04d-%02d-%02d (%s)\r\n         Weather: %s\r\n         Uptime: %luh %lum %lus",
             sTime.Hours, sTime.Minutes, sTime.Seconds,
             2000 + sDate.Year, sDate.Month, sDate.Date, weekday,
             (strlen(weatherBuffer) > 0) ? weatherBuffer : "N/A",
             uptimeHours, uptimeMins, uptimeSecs);

    CDC_Transmit_ThreadSafe((uint8_t *)msgBuffer, strlen(msgBuffer));

    osDelay(10000); // Update every 10 seconds
  }
  /* USER CODE END StartDisplayTask */
}

/* LED Activity Indicators */
volatile uint8_t ledCdcActivity = 0;
volatile uint8_t ledEspActivity = 0;

/* Debug forwarding control - disabled for clean output */
volatile uint8_t debugForwardingEnabled = 0;

/* LED Control Functions */
// Additional LED3 and LED4 definitions (external LEDs)
#define LED3_GPIO_PORT GPIOI
#define LED3_PIN GPIO_PIN_1
#define LED4_GPIO_PORT GPIOI
#define LED4_PIN GPIO_PIN_2

void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOJ clock for onboard LEDs (LED1, LED2)
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  // Enable GPIOI clock for external LEDs (LED3, LED4)
  __HAL_RCC_GPIOI_CLK_ENABLE();

  // Configure LED1 (PJ13) and LED2 (PJ5) as outputs
  GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  // Configure LED3 (PI1) and LED4 (PI2) as outputs (external LEDs)
  GPIO_InitStruct.Pin = LED3_PIN | LED4_PIN;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  // Start with all LEDs off
  HAL_GPIO_WritePin(GPIOJ, LED1_PIN | LED2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOI, LED3_PIN | LED4_PIN, GPIO_PIN_SET);
}

void LED_On(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_WritePin(GPIOJ, LED1_PIN, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(GPIOJ, LED2_PIN, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(GPIOI, LED3_PIN, GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(GPIOI, LED4_PIN, GPIO_PIN_RESET);
      break;
  }
}

void LED_Off(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_WritePin(GPIOJ, LED1_PIN, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(GPIOJ, LED2_PIN, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(GPIOI, LED3_PIN, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(GPIOI, LED4_PIN, GPIO_PIN_SET);
      break;
  }
}

void LED_Toggle(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_TogglePin(GPIOJ, LED1_PIN);
      break;
    case 2:
      HAL_GPIO_TogglePin(GPIOJ, LED2_PIN);
      break;
    case 3:
      HAL_GPIO_TogglePin(GPIOI, LED3_PIN);
      break;
    case 4:
      HAL_GPIO_TogglePin(GPIOI, LED4_PIN);
      break;
  }
}

/* LED Task - Handles startup sequence and normal operation */
/* USER CODE BEGIN Header_StartLedTask */
/**
 * @brief Function implementing the ledTask thread.
 *        Startup: LEDs light up one after another for 20 seconds
 *        Normal: LED1 blinks (heartbeat), LED2 shows activity
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  // Initialize LEDs
  LED_Init();

  // Startup sequence: all 4 LEDs light up one after another for 20 seconds (100 * 200ms)
  uint8_t ledState = 0;  // 0=LED1, 1=LED2, 2=LED3, 3=LED4
  for (int i = 0; i < 100; i++)
  {
    LED_Off(1);
    LED_Off(2);
    LED_Off(3);
    LED_Off(4);

    LED_On(ledState + 1);

    ledState = (ledState + 1) % 4;
    osDelay(200);
  }

  // Turn off all LEDs after startup
  LED_Off(1);
  LED_Off(2);
  LED_Off(3);
  LED_Off(4);
  osDelay(500);

  // Normal operation: LED1 heartbeat blink, LED2/CDC activity, LED3/ESP activity
  uint32_t heartbeatCounter = 0;

  /* Infinite loop */
  for(;;)
  {
    // Heartbeat: LED1 blinks every second (500ms on, 500ms off)
    heartbeatCounter++;
    if (heartbeatCounter <= 5)
      LED_On(1);
    else
      LED_Off(1);

    if (heartbeatCounter >= 10)
      heartbeatCounter = 0;

    // Check for CDC activity (LD2 flashes for 200ms)
    if (ledCdcActivity)
    {
      LED_On(2);
      osDelay(200);
      LED_Off(2);
      ledCdcActivity = 0;
    }

    // Check for ESP8266 activity (LD3 flashes for 200ms)
    if (ledEspActivity)
    {
      LED_On(3);
      osDelay(200);
      LED_Off(3);
      ledEspActivity = 0;
    }

    osDelay(100);
  }
  /* USER CODE END StartLedTask */
}

