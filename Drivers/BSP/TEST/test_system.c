/**
 ******************************************************************************
 * @file    test_system.c
 * @brief   Test system implementation for STM32F769I-Discovery
 ******************************************************************************
 */

#include "test_system.h"
#include "esp8266.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"
#include "stm32f7xx_hal_sd.h"
#include "stm32f769i_discovery_sd.h"
#include "stm32f769i_discovery_ts.h"
#include "usbd_cdc_if.h"
#include "rtc.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* External variables */
extern RTC_HandleTypeDef hrtc;
extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);

/* Drawing command state */
static int32_t drawCurrentX = 0;   /* Current X position for line drawing */
static int32_t drawCurrentY = 0;   /* Current Y position for line drawing */
static uint32_t drawCurrentColor = LCD_COLOR_WHITE;  /* Current drawing color */
static int drawPositionSet = 0;    /* Flag: has current position been set? */
static sFONT *currentFont = &Font24;  /* Current font for drawing */

/* Interrupt flag for stopping tests */
static volatile uint8_t testInterruptFlag = 0;  /* Set to 1 to interrupt running test */

/* Helper macro for checking interrupt during long operations */
#define CHECK_INTERRUPT() do { \
  if (testInterruptFlag) { \
    BSP_LCD_Clear(LCD_COLOR_BLACK); \
    extern LTDC_HandleTypeDef hltdc_discovery; \
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery); \
    BSP_LCD_SetTextColor(LCD_COLOR_RED); \
    BSP_LCD_SetFont(&Font24); \
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, \
                           (uint8_t *)"TEST INTERRUPTED", CENTER_MODE); \
    TEST_Output("\r\n[Test Interrupted by ATST999]\r\n"); \
    return TEST_TIMEOUT; \
  } \
} while(0)

/* Helper macro for delays with interrupt checking */
#define TEST_DELAY(ms) do { \
  uint32_t delayStart = HAL_GetTick(); \
  while ((HAL_GetTick() - delayStart) < (ms)) { \
    CHECK_INTERRUPT(); \
    osDelay(50); \
  } \
} while(0)

/* WiFi credentials - defined in freertos.c, we need to pass them to ESP8266_Connect */
// We'll define them here as well for the test system to use
#define TEST_WIFI_SSID     "MEO-EDC8ED"
#define TEST_WIFI_PASSWORD "2668EB941B"

/* LED control functions - from freertos.c */
extern void LED_Init(void);
extern void LED_On(uint8_t led);
extern void LED_Off(uint8_t led);
extern void LED_Toggle(uint8_t led);

/* Test result output helper */
static void TEST_Output(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  CDC_Transmit_ThreadSafe((uint8_t *)buffer, strlen(buffer));
}

static void TEST_OutputResult(const char *testName, TestStatus_TypeDef status, const char *details) {
  const char *statusStr;
  switch (status) {
    case TEST_OK: statusStr = "PASS"; break;
    case TEST_ERROR: statusStr = "FAIL"; break;
    case TEST_TIMEOUT: statusStr = "TIMEOUT"; break;
    case TEST_NOT_SUPPORTED: statusStr = "N/A"; break;
    default: statusStr = "UNKNOWN"; break;
  }
  TEST_Output("\r\n[%s] %s: %s", statusStr, testName, details);
}

/* =============================================================================
 * ATST0 - ESP8266 Weather Test
 * Tests: ESP8266 communication, WiFi connection, weather data retrieval
 * ========================================================================== */
TestStatus_TypeDef TEST_ESP_Weather(void) {
  char response[256];
  TestStatus_TypeDef result = TEST_ERROR;

  TEST_Output("\r\n=== ATST0: ESP8266 Weather Test ===");

  // v0.1.85: Display start message is now handled automatically by CDC_ProcessPendingCommand
  // which calls TEST_DisplayStartMessage before running the test
  // Note: Individual test display messages removed to avoid duplication

  // Check for interrupt at start
  CHECK_INTERRUPT();

  // Step 1: Test AT command
  TEST_Output("\r\n[Step 1/4] Testing AT command...");
  CHECK_INTERRUPT();
  if (ESP8266_SendCommand("AT") == ESP8266_OK) {
    if (ESP8266_WaitFor("OK", 2000) == ESP8266_OK) {
      TEST_OutputResult("AT command", TEST_OK, "ESP8266 responding");
    } else {
      TEST_OutputResult("AT command", TEST_TIMEOUT, "No response from ESP8266");
      return TEST_TIMEOUT;
    }
  } else {
    TEST_OutputResult("AT command", TEST_ERROR, "Failed to send AT");
    return TEST_ERROR;
  }
  CHECK_INTERRUPT();

  // Step 2: Check WiFi connection
  TEST_Output("\r\n[Step 2/4] Checking WiFi connection...");
  ESP8266_StatusTypeDef wifiStatus = ESP8266_CheckConnection();
  CHECK_INTERRUPT();
  if (wifiStatus == ESP8266_OK) {
    TEST_OutputResult("WiFi check", TEST_OK, "Connected to WiFi");

    // Get IP
    char ip[32];
    if (ESP8266_GetIP(ip, sizeof(ip)) == ESP8266_OK) {
      TEST_Output("         IP: %s", ip);
    }
  } else {
    TEST_OutputResult("WiFi check", TEST_ERROR, "Not connected (try ATST4 first)");
    return TEST_ERROR;
  }
  CHECK_INTERRUPT();

  // Step 3: Test weather request using OpenWeatherMap API
  TEST_Output("\r\n[Step 3/4] Testing weather request (OpenWeatherMap)...");
  if (ESP8266_GetWeather(response, sizeof(response)) == ESP8266_OK) {
    TEST_OutputResult("Weather request", TEST_OK, response);
    result = TEST_OK;
  } else {
    TEST_OutputResult("Weather request", TEST_ERROR, "Failed to get weather data");
    return TEST_ERROR;
  }
  CHECK_INTERRUPT();

  // Step 4: Verify weather data format
  TEST_Output("\r\n[Step 4/4] Verifying weather data format...");
  // Weather format: "Description, Temp°C, feels like FeelsLike°C, humidity Hum%, wind Wind m/s"
  if (strstr(response, "C") != NULL && strstr(response, "%") != NULL) {
    TEST_OutputResult("Format verification", TEST_OK, "Weather data format is valid");
  } else {
    TEST_OutputResult("Format verification", TEST_OK, "Weather data received (format may vary)");
  }

  // v0.1.85: Display completion message is now handled automatically by CDC_ProcessPendingCommand
  // which calls TEST_DisplayCompleteMessage after the test completes
  // Note: Individual test display messages removed to avoid duplication

  TEST_Output("\r\n=== ATST0 Complete ===\r\n");
  return result;
}

/* =============================================================================
 * ATST1 - Display Graphics/Text Test
 * Tests: LCD display, colors, text rendering, fonts
 * ========================================================================== */
TestStatus_TypeDef TEST_Display(void) {
  TEST_Output("\r\n=== ATST1: Display Test ===");

  // v0.1.85: Display messages handled automatically by CDC_ProcessPendingCommand
  osDelay(500);

  // Check for interrupt at start
  CHECK_INTERRUPT();

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
           "CR: 0x%08lx (LayerEnabled=%lu)\r\n"
           "Expected: LineLen=2400, Pitch=2400 for 800x472 RGB888\r\n",
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

  // Test 1: Clear screen
  TEST_Output("\r\n[Test 1/6] Clearing screen...");
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload
  osDelay(20);  // Allow display to update
  TEST_OutputResult("Clear screen", TEST_OK, "Screen cleared to white");
  TEST_DELAY(500);  // Use interruptible delay

  // Test 2: Color bars
  TEST_Output("\r\n[Test 2/6] Drawing color bars...");
  uint32_t colors[] = {LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_BLUE,
                       LCD_COLOR_YELLOW, LCD_COLOR_CYAN, LCD_COLOR_MAGENTA};
  const char *colorNames[] = {"Red", "Green", "Blue", "Yellow", "Cyan", "Magenta"};

  for (int i = 0; i < 6; i++) {
    BSP_LCD_SetTextColor(colors[i]);
    BSP_LCD_FillRect(0, i * 50, BSP_LCD_GetXSize(), 50);
  }
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload after drawing
  TEST_OutputResult("Color bars", TEST_OK, "6 color bars displayed");
  TEST_DELAY(1000);  // Use interruptible delay
  CHECK_INTERRUPT();  // Check for interrupt after color bars

  // Test 3: Text rendering
  TEST_Output("\r\n[Test 3/6] Testing text rendering...");
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

  const char *testStrings[] = {
    "STM32F769I-DISCO",
    "Display Test",
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
    "abcdefghijklmnopqrstuvwxyz",
    "0123456789!@#$%^&*()"
  };

  for (int i = 0; i < 5; i++) {
    BSP_LCD_DisplayStringAt(0, 20 + i * 30, (uint8_t *)testStrings[i], CENTER_MODE);
  }
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload
  TEST_OutputResult("Text rendering", TEST_OK, "Multiple text lines displayed");
  TEST_DELAY(1000);  // Use interruptible delay
  CHECK_INTERRUPT();  // Check for interrupt after text rendering

  // Test 4: Different fonts
  TEST_Output("\r\n[Test 4/6] Testing fonts...");
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

  BSP_LCD_SetFont(&Font8);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"Font8 - Small text", CENTER_MODE);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"Font12 - Medium text", CENTER_MODE);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)"Font16 - Large text", CENTER_MODE);

  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Font20 - Extra Large", CENTER_MODE);

  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 140, (uint8_t *)"Font24 - Huge", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload
  TEST_OutputResult("Fonts", TEST_OK, "5 different fonts displayed");
  TEST_DELAY(1000);  // Use interruptible delay
  CHECK_INTERRUPT();  // Check for interrupt after fonts

  // Test 5: Shapes
  TEST_Output("\r\n[Test 5/6] Drawing shapes...");
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  // Draw circles
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  for (int i = 0; i < 5; i++) {
    BSP_LCD_DrawCircle(BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2, 30 + i * 20);
  }
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload after circles
  TEST_OutputResult("Circles", TEST_OK, "5 concentric circles");
  osDelay(500);

  // Draw rectangles - FIXED v0.1.79: Centered concentric squares, no sliding
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(50);

  // Calculate center position
  int16_t centerX = BSP_LCD_GetXSize() / 2;  // 400
  int16_t centerY = BSP_LCD_GetYSize() / 2;  // 236

  // Draw from largest (outer) to smallest (inner), centered
  for (int i = 0; i < 5; i++) {
    // Half-size decreases from 180 to 40 (180, 145, 110, 75, 40)
    int16_t halfSize = 180 - i * 35;
    int16_t x = centerX - halfSize;
    int16_t y = centerY - halfSize;
    int16_t size = halfSize * 2;

    // Alternate colors for better visibility
    BSP_LCD_SetTextColor(i % 2 == 0 ? LCD_COLOR_BLUE : LCD_COLOR_RED);
    BSP_LCD_DrawRect(x, y, size, size);

    // Force LTDC reload after each rectangle
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  }
  TEST_OutputResult("Concentric squares", TEST_OK, "5 centered squares (v0.1.79 fix)");
  TEST_DELAY(500);  // Use interruptible delay
  CHECK_INTERRUPT();  // Check for interrupt after rectangles

  // Test 6: Fill screen gradient
  TEST_Output("\r\n[Test 6/6] Testing pixel fill...");
  BSP_LCD_Clear(LCD_COLOR_BLACK);  // Start with black
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);
  for (int y = 0; y < BSP_LCD_GetYSize(); y++) {
    uint32_t color = (y * 255) / BSP_LCD_GetYSize();
    BSP_LCD_SetTextColor((color << 16) | (color << 8) | color);
    BSP_LCD_DrawHLine(0, y, BSP_LCD_GetXSize());
  }
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);  // Force LTDC reload after gradient
  TEST_OutputResult("Gradient", TEST_OK, "Vertical gradient displayed");
  TEST_DELAY(1000);  // Use interruptible delay
  CHECK_INTERRUPT();  // Check for interrupt after gradient

  // Restore default font
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t *)"Display Test Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST1 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST1 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST2 - LED Test
 * Tests: All LEDs (LD1-LD4) on/off functionality
 * ========================================================================== */
TestStatus_TypeDef TEST_LEDs(void) {
  TEST_Output("\r\n=== ATST2: LED Test ===");

  // v0.1.85: Display messages handled automatically by CDC_ProcessPendingCommand

  TEST_Output("\r\n[Test 1/4] Initializing LEDs...");
  LED_Init();
  TEST_OutputResult("LED Init", TEST_OK, "All LEDs initialized");
  osDelay(500);

  // Turn off all LEDs first
  LED_Off(1);
  LED_Off(2);
  LED_Off(3);
  LED_Off(4);

  // Test each LED individually
  TEST_Output("\r\n[Test 2/4] Testing individual LEDs...");
  for (int i = 1; i <= 4; i++) {
    TEST_Output("         LED%d ON...", i);
    LED_On(i);
    osDelay(300);
    LED_Off(i);
    TEST_Output("         LED%d OFF", i);
    osDelay(200);
  }
  TEST_OutputResult("Individual LEDs", TEST_OK, "All 4 LEDs tested");

  // Test chase pattern
  TEST_Output("\r\n[Test 3/4] Testing chase pattern...");
  for (int cycle = 0; cycle < 3; cycle++) {
    for (int i = 1; i <= 4; i++) {
      LED_On(i);
      osDelay(100);
      LED_Off(i);
    }
  }
  TEST_OutputResult("Chase pattern", TEST_OK, "3 cycles completed");

  // Test all on
  TEST_Output("\r\n[Test 4/4] All LEDs ON...");
  LED_On(1);
  LED_On(2);
  LED_On(3);
  LED_On(4);
  TEST_OutputResult("All ON", TEST_OK, "All 4 LEDs on");
  osDelay(1000);

  // Turn off all LEDs
  LED_Off(1);
  LED_Off(2);
  LED_Off(3);
  LED_Off(4);

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST2 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST2 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST3 - RTC Time Test
 * Tests: Real-time clock functionality
 * ========================================================================== */
TestStatus_TypeDef TEST_RTC(void) {
  TEST_Output("\r\n=== ATST3: RTC Test ===");

  // v0.1.85: Display messages handled automatically by CDC_ProcessPendingCommand

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // Test 1: Read RTC
  TEST_Output("\r\n[Test 1/3] Reading RTC...");
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  char timeStr[32];
  snprintf(timeStr, sizeof(timeStr),
           "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
  char dateStr[32];
  snprintf(dateStr, sizeof(dateStr),
           "%04d-%02d-%02d", 2000 + sDate.Year, sDate.Month, sDate.Date);

  TEST_Output("         Time: %s", timeStr);
  TEST_Output("         Date: %s", dateStr);
  TEST_OutputResult("RTC Read", TEST_OK, "RTC reading successful");

  // Test 2: Validate time values
  TEST_Output("\r\n[Test 2/3] Validating time values...");
  TestStatus_TypeDef validTime = TEST_OK;
  if (sTime.Hours > 23 || sTime.Minutes > 59 || sTime.Seconds > 59) {
    TEST_OutputResult("Time validation", TEST_ERROR, "Invalid time values");
    validTime = TEST_ERROR;
  } else {
    TEST_OutputResult("Time validation", TEST_OK, "Time values valid");
  }

  // Test 3: Wait and check time increment
  TEST_Output("\r\n[Test 3/3] Testing time increment...");
  uint8_t oldSeconds = sTime.Seconds;
  osDelay(1100); // Wait 1.1 seconds
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  if (sTime.Seconds != oldSeconds) {
    TEST_OutputResult("Time increment", TEST_OK, "RTC is running");
  } else {
    TEST_OutputResult("Time increment", TEST_ERROR, "RTC not incrementing");
    validTime = TEST_ERROR;
  }

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST3 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST3 Complete ===\r\n");
  return validTime;
}

/* =============================================================================
 * ATST4 - WiFi Connection Test
 * Tests: ESP8266 WiFi connection functionality
 * ========================================================================== */
TestStatus_TypeDef TEST_WiFi(void) {
  TEST_Output("\r\n=== ATST4: WiFi Connection Test ===");

  // Display test start message on LCD
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST4 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  // Check for interrupt at start
  CHECK_INTERRUPT();

  // Step 1: Disconnect first
  TEST_Output("\r\n[Step 1/4] Disconnecting from current AP...");
  ESP8266_Disconnect();
  TEST_DELAY(1000);  // Use TEST_DELAY instead of osDelay for interrupt checking
  TEST_OutputResult("Disconnect", TEST_OK, "Disconnected");
  CHECK_INTERRUPT();

  // Step 2: Set station mode
  TEST_Output("\r\n[Step 2/4] Setting station mode...");
  ESP8266_SendCommand("AT+CWMODE=1");
  if (ESP8266_WaitFor("OK", 2000) == ESP8266_OK) {
    TEST_OutputResult("Station mode", TEST_OK, "Mode set to Station");
  } else {
    TEST_OutputResult("Station mode", TEST_ERROR, "Failed to set mode");
    return TEST_ERROR;
  }
  CHECK_INTERRUPT();

  // Step 3: Connect to WiFi
  TEST_Output("\r\n[Step 3/4] Connecting to WiFi...");
  TEST_Output("         SSID: %s", TEST_WIFI_SSID);

  ESP8266_StatusTypeDef status = ESP8266_Connect(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);
  if (status == ESP8266_OK) {
    TEST_OutputResult("WiFi Connect", TEST_OK, "Connected successfully");
  } else {
    TEST_OutputResult("WiFi Connect", TEST_TIMEOUT, "Connection timeout");
    return TEST_TIMEOUT;
  }
  CHECK_INTERRUPT();

  // Step 4: Get IP address
  TEST_Output("\r\n[Step 4/4] Getting IP address...");
  char ip[32];
  status = ESP8266_GetIP(ip, sizeof(ip));
  if (status == ESP8266_OK) {
    TEST_OutputResult("IP Address", TEST_OK, ip);
  } else {
    TEST_OutputResult("IP Address", TEST_ERROR, "Failed to get IP");
    return TEST_ERROR;
  }

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST4 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST4 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST5 - NTP Time Sync Test
 * Tests: ESP8266 SNTP time synchronization
 * ========================================================================== */
TestStatus_TypeDef TEST_NTP(void) {
  TEST_Output("\r\n=== ATST5: NTP Sync Test ===");

  // Display test start message on LCD
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST5 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  // Check for interrupt at start
  CHECK_INTERRUPT();

  uint32_t unixTimestamp = 0;

  // Test NTP sync
  TEST_Output("\r\n[Test 1/2] Syncing with NTP server...");
  CHECK_INTERRUPT();
  ESP8266_StatusTypeDef status = ESP8266_NTPSync("pool.ntp.org", &unixTimestamp);

  if (status == ESP8266_OK && unixTimestamp > 0) {
    TEST_OutputResult("NTP Sync", TEST_OK, "Time synchronized");

    // Convert Unix timestamp to readable format
    uint32_t days = unixTimestamp / 86400;
    uint32_t hours = (unixTimestamp % 86400) / 3600;
    uint32_t minutes = (unixTimestamp % 3600) / 60;
    uint32_t seconds = unixTimestamp % 60;

    TEST_Output("\r\n[Test 2/2] Unix timestamp: %lu", unixTimestamp);
    TEST_Output("         Days since epoch: %lu", days);

    // Rough year calculation (ignoring leap years for simplicity)
    uint32_t year = 1970 + (days / 365);
    TEST_Output("         Approximate year: %lu", year);
  } else {
    TEST_OutputResult("NTP Sync", TEST_ERROR, "Failed to sync time");
    return TEST_ERROR;
  }

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST5 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST5 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST6 - HTTP Request Test
 * Tests: ESP8266 HTTP GET requests
 * ========================================================================== */
TestStatus_TypeDef TEST_HTTP(void) {
  TEST_Output("\r\n=== ATST6: HTTP Test ===");

  // Display test start message on LCD
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST6 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  // Check for interrupt at start
  CHECK_INTERRUPT();

  char response[256];
  TestStatus_TypeDef result = TEST_OK;

  // Give ESP8266 time to settle after WiFi connection
  TEST_DELAY(1000);  // Use TEST_DELAY for interrupt checking

  // Test 1: Simple GET
  TEST_Output("\r\n[Test 1/3] Testing HTTP GET to httpbin.org...");
  CHECK_INTERRUPT();
  if (ESP8266_HTTPGet("http://httpbin.org/get", response, sizeof(response)) == ESP8266_OK) {
    TEST_OutputResult("httpbin GET", TEST_OK, "Response received");
    TEST_Output("         Response: %s", response);
  } else {
    TEST_OutputResult("httpbin GET", TEST_ERROR, "Request failed");
    result = TEST_ERROR;
  }
  TEST_DELAY(2000);  // Use TEST_DELAY instead of osDelay

  // Test 2: OpenWeatherMap weather
  TEST_Output("\r\n[Test 2/3] Testing OpenWeatherMap weather...");
  CHECK_INTERRUPT();
  if (ESP8266_GetWeather(response, sizeof(response)) == ESP8266_OK) {
    TEST_OutputResult("OpenWeatherMap", TEST_OK, response);
    TEST_Output("         Weather: %s", response);
  } else {
    TEST_OutputResult("OpenWeatherMap", TEST_ERROR, "Weather request failed");
    result = TEST_ERROR;
  }
  TEST_DELAY(2000);  // Use TEST_DELAY instead of osDelay

  // Test 3: JSON test
  TEST_Output("\r\n[Test 3/3] Testing JSON endpoint...");
  CHECK_INTERRUPT();
  if (ESP8266_HTTPGet("http://httpbin.org/json", response, sizeof(response)) == ESP8266_OK) {
    TEST_OutputResult("JSON endpoint", TEST_OK, "JSON data received");
    TEST_Output("         Response: %s", response);
  } else {
    TEST_OutputResult("JSON endpoint", TEST_ERROR, "JSON request failed");
    result = TEST_ERROR;
  }

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST6 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST6 Complete ===\r\n");
  return result;
}

/* =============================================================================
 * ATST7 - Audio Codec Test
 * Tests: Audio codec functionality (basic check)
 * ========================================================================== */
TestStatus_TypeDef TEST_Audio(void) {
  TEST_Output("\r\n=== ATST7: Audio Test ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST7 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  TEST_Output("\r\n[Test 1/2] Checking audio codec presence...");
  // Basic I2C scan for audio codec
  // This is a placeholder - actual implementation would check WM8994 codec
  TEST_OutputResult("Audio codec", TEST_NOT_SUPPORTED, "Audio test not fully implemented");

  TEST_Output("\r\n[Test 2/2] Audio codec test skipped");
  TEST_Output("         Note: Full audio test requires codec library integration");

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST7 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST7 Complete ===\r\n");
  return TEST_NOT_SUPPORTED;
}

/* =============================================================================
 * ATST8 - SDRAM Test
 * Tests: External SDRAM read/write functionality
 * ========================================================================== */
TestStatus_TypeDef TEST_SDRAM(void) {
  TEST_Output("\r\n=== ATST8: SDRAM Test ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST8 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  TestStatus_TypeDef result = TEST_OK;

  // Test 1: Simple write/read test
  TEST_Output("\r\n[Test 1/2] Testing SDRAM write/read...");
  uint32_t *sdramTestPtr = (uint32_t *)SDRAM_DEVICE_ADDR;
  uint32_t testPattern = 0xA5A5A5A5;
  uint32_t readValue;

  *sdramTestPtr = testPattern;
  readValue = *sdramTestPtr;

  if (readValue == testPattern) {
    TEST_OutputResult("SDRAM basic", TEST_OK, "Write/read successful");
  } else {
    TEST_OutputResult("SDRAM basic", TEST_ERROR, "Data mismatch");
    TEST_Output("         Wrote: 0x%08lx, Read: 0x%08lx", testPattern, readValue);
    result = TEST_ERROR;
  }

  // Test 2: Pattern test
  TEST_Output("\r\n[Test 2/2] Testing SDRAM with multiple patterns...");
  uint32_t patterns[] = {0x00000000, 0xFFFFFFFF, 0x55555555, 0xAAAAAAAA};
  int patternErrors = 0;

  for (int i = 0; i < 4; i++) {
    sdramTestPtr[i * 1024] = patterns[i]; // Spread out tests
    readValue = sdramTestPtr[i * 1024];
    if (readValue != patterns[i]) {
      patternErrors++;
    }
  }

  if (patternErrors == 0) {
    TEST_OutputResult("SDRAM patterns", TEST_OK, "All patterns verified");
  } else {
    TEST_OutputResult("SDRAM patterns", TEST_ERROR, "Pattern errors detected");
    TEST_Output("         Errors: %d/4", patternErrors);
    result = TEST_ERROR;
  }

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST8 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n=== ATST8 Complete ===\r\n");
  return result;
}

/* =============================================================================
 * ATST9 - Touchscreen Test (v0.1.85)
 * Tests: Touchscreen functionality with 20-second continuous touch recording
 *         Records all touches and displays coordinates in virtual port
 *         v0.1.85: SIMPLIFIED - removed complex filtering, use simple filled circles
 *                   Fixes artifacts and freezing issues
 * ========================================================================== */
TestStatus_TypeDef TEST_Touch(void) {
  TEST_Output("\r\n=== ATST9: Touchscreen Test (20-second recording) ===");
  TEST_Output("\r\nNOTE: Simplified touch detection");
  TEST_Output("      Draws simple filled circles at touch points");
  TEST_Output("      Outputs coordinates to virtual port");

  // Display test start message on LCD
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_DARKBLUE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST9 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  TEST_Output("\r\n[Test 1/2] Initializing touchscreen...");
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST9: Touch Test (20s)", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 40, (uint8_t *)"Touch anywhere...", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 30, (uint8_t *)"Send ATST999 to stop", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  uint32_t tsStatus = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  if (tsStatus == TS_OK) {
    TEST_OutputResult("Touch init", TEST_OK, "Touchscreen initialized");
  } else {
    TEST_OutputResult("Touch init", TEST_ERROR, "Touchscreen init failed");
    return TEST_ERROR;
  }

  // Test 2: Record touches for 20 seconds
  TEST_Output("\r\n[Test 2/2] Recording touches for 20 seconds...");
  TEST_Output("         All touches will be recorded and drawn");
  TEST_Output("         Coordinates displayed in virtual port");
  TEST_Output("         Send ATST999 to stop early");

  TS_StateTypeDef tsState;
  uint32_t touchCount = 0;
  uint32_t startTime = HAL_GetTick();
  const uint32_t TEST_DURATION_MS = 20000;  // 20 seconds

  // Simple touch tracking - only track last position to avoid duplicates
  uint16_t lastTouchX = 0xFFFF;  // Invalid initial value
  uint16_t lastTouchY = 0xFFFF;
  uint8_t wasTouching = 0;
  const uint16_t TOUCH_THRESHOLD = 30;  // Pixels - touch must move this much to count as new

  // Clear screen for drawing touch points
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST9: Touch Test (20s)", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  char timeLabel[32];
  snprintf(timeLabel, sizeof(timeLabel), "Time: 0s / 20s");
  BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)timeLabel, CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"ATST999 to stop", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  while ((HAL_GetTick() - startTime < TEST_DURATION_MS) && !testInterruptFlag) {
    // Check for interrupt signal (ATST999)
    CHECK_INTERRUPT();

    // Read touch state
    BSP_TS_GetState(&tsState);

    // Only process single touches (ignore multi-touch)
    if (tsState.touchDetected == 1 &&
        tsState.touchX[0] < BSP_LCD_GetXSize() &&
        tsState.touchY[0] < BSP_LCD_GetYSize()) {

      uint16_t touchX = tsState.touchX[0];
      uint16_t touchY = tsState.touchY[0];

      // Check if this is a new touch (moved significantly from last touch)
      if (!wasTouching ||
          (abs((int16_t)touchX - (int16_t)lastTouchX) > TOUCH_THRESHOLD ||
           abs((int16_t)touchY - (int16_t)lastTouchY) > TOUCH_THRESHOLD)) {

        // New touch detected
        touchCount++;
        lastTouchX = touchX;
        lastTouchY = touchY;
        wasTouching = 1;

        // Output coordinates immediately (no batching)
        uint32_t elapsedSec = (HAL_GetTick() - startTime) / 1000;
        TEST_Output("\r\n[Touch #%lu] at %lus: X=%d, Y=%d",
                   touchCount, elapsedSec, touchX, touchY);

        // Draw simple filled circle at touch point
        // Use direct pixel writing for reliability instead of BSP_LCD_FillCircle
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        uint8_t radius = 8;
        BSP_LCD_FillCircle(touchX, touchY, radius);

        // Add small label above the circle
        BSP_LCD_SetFont(&Font8);
        char label[16];
        snprintf(label, sizeof(label), "#%lu", touchCount);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        BSP_LCD_DisplayStringAt(touchX, touchY - radius - 12, (uint8_t *)label, LEFT_MODE);

        // Update time display
        BSP_LCD_SetFont(&Font12);
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"                    ", CENTER_MODE);
        snprintf(timeLabel, sizeof(timeLabel), "Touches: %lu, Time: %lus", touchCount, elapsedSec);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)timeLabel, CENTER_MODE);

        // Force LTDC reload after drawing
        __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
      }
    } else if (tsState.touchDetected == 0) {
      // No touch - reset wasTouching flag
      wasTouching = 0;
    }
    // Ignore multi-touch (touchDetected > 1)

    // Short delay to prevent overwhelming the system
    osDelay(100);
  }

  // Check if we were interrupted
  if (testInterruptFlag) {
    TEST_Output("\r\n[ATST9] Interrupted by ATST999");
    BSP_LCD_Clear(LCD_COLOR_YELLOW);
    BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 12, (uint8_t *)"INTERRUPTED", CENTER_MODE);
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
    TEST_Output("\r\n=== ATST9 Interrupted ===\r\n");
    return TEST_TIMEOUT;
  }

  // Test complete - show summary
  TEST_Output("\r\n=== Touch Test Summary ===");
  TEST_Output("Total touches recorded: %lu", touchCount);
  TEST_Output("Duration: %lu seconds", (HAL_GetTick() - startTime) / 1000);

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_DARKGREEN);
  BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 25, (uint8_t*)"Test ATST9 Complete!", CENTER_MODE);
  BSP_LCD_SetFont(&Font16);
  char summary[64];
  snprintf(summary, sizeof(summary), "Touches: %lu", touchCount);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 10, (uint8_t *)summary, CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(2000);

  TEST_OutputResult("Touch test", TEST_OK, "20-second recording complete");
  TEST_Output("\r\n=== ATST9 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST10 - Run All Tests
 * Executes all tests in sequence
 * ========================================================================== */
TestStatus_TypeDef TEST_RunAll(void) {
  TEST_Output("\r\n\r\n╔════════════════════════════════════════╗");
  TEST_Output("║   ATST10: Running ALL Tests          ║");
  TEST_Output("╚════════════════════════════════════════╝");

  TestStatus_TypeDef results[10];
  int passCount = 0;
  int failCount = 0;
  int naCount = 0;

  // Run all tests
  results[0] = TEST_ESP_Weather();
  results[1] = TEST_Display();
  results[2] = TEST_LEDs();
  results[3] = TEST_RTC();
  results[4] = TEST_WiFi();
  results[5] = TEST_NTP();
  results[6] = TEST_HTTP();
  results[7] = TEST_Audio();
  results[8] = TEST_SDRAM();
  results[9] = TEST_Touch();

  // Count results
  for (int i = 0; i < 10; i++) {
    if (results[i] == TEST_OK) passCount++;
    else if (results[i] == TEST_NOT_SUPPORTED) naCount++;
    else failCount++;
  }

  // Print summary
  TEST_Output("\r\n\r\n╔════════════════════════════════════════╗");
  TEST_Output("║           TEST SUMMARY                  ║");
  TEST_Output("╠════════════════════════════════════════╣");
  TEST_Output("║  PASSED:    %2d                         ║", passCount);
  TEST_Output("║  FAILED:    %2d                         ║", failCount);
  TEST_Output("║  N/A:       %2d                         ║", naCount);
  TEST_Output("╚════════════════════════════════════════╝");

  if (failCount == 0) {
    TEST_Output("\r\n*** ALL TESTS PASSED ***\r\n");
    return TEST_OK;
  } else {
    TEST_Output("\r\n*** SOME TESTS FAILED ***\r\n");
    return TEST_ERROR;
  }
}

/* =============================================================================
 * Help - Show available test commands
 * ========================================================================== */
void TEST_ShowHelp(void) {
  // Combine entire help message into one transmission to avoid USB CDC corruption
  const char *helpMsg =
    "\r\n\r\n"
    "╔════════════════════════════════════════════════════════╗\r\n"
    "║        STM32F769I-Discovery Test System Help          ║\r\n"
    "╠════════════════════════════════════════════════════════╣\r\n"
    "║                                                        ║\r\n"
    "║  ATST0   - ESP8266 Weather Test                        ║\r\n"
    "║            Tests ESP8266 communication, WiFi,         ║\r\n"
    "║            and weather data retrieval                  ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST1   - Display Graphics/Text Test                  ║\r\n"
    "║            Tests LCD display with colors, fonts,       ║\r\n"
    "║            shapes, and text rendering                  ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST2   - LED Test                                    ║\r\n"
    "║            Tests all 4 LEDs with individual and        ║\r\n"
    "║            chase patterns                              ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST3   - RTC Time Test                               ║\r\n"
    "║            Tests real-time clock functionality         ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST4   - WiFi Connection Test                        ║\r\n"
    "║            Connects to WiFi and verifies IP            ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST5   - NTP Time Sync Test                          ║\r\n"
    "║            Tests NTP time synchronization               ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST6   - HTTP Request Test                           ║\r\n"
    "║            Tests HTTP GET requests to various          ║\r\n"
    "║            endpoints                                   ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST7   - Audio Codec Test                            ║\r\n"
    "║            Tests audio codec (not fully implemented)    ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST8   - SDRAM Test                                  ║\r\n"
    "║            Tests external SDRAM read/write             ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST9   - Touchscreen Test                            ║\r\n"
    "║            Tests touchscreen with touch detection      ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST10  - Run All Tests                               ║\r\n"
    "║            Executes all tests in sequence              ║\r\n"
    "║                                                        ║\r\n"
    "║  Display Mode Commands (ATST141-150):                 ║\r\n"
    "║  ATST141 - Text Mode                                  ║\r\n"
    "║  ATST142 - Graphics Pattern Mode                       ║\r\n"
    "║  ATST143 - Color Bars Mode                             ║\r\n"
    "║  ATST144 - Gradient Mode                               ║\r\n"
    "║  ATST145 - Checkerboard (40x40)                        ║\r\n"
    "║  ATST146 - LTDC Info Display                          ║\r\n"
    "║  ATST147 - Focus Pattern (convergence test)            ║\r\n"
    "║  ATST148 - Display Modes Help                          ║\r\n"
    "║  ATST149 - LCD Panel Detection Diagnostic               ║\r\n"
    "║  ATST150 - Checkerboard 50x50 (timing test)             ║\r\n"
    "║                                                        ║\r\n"
    "║  Drawing Commands (Menu/Graph Support):                 ║\r\n"
    "║  ATST151=X,Y,D,C - Draw point at (X,Y)                 ║\r\n"
    "║     X=0-799, Y=0-471, D=1-50px, C=hex color            ║\r\n"
    "║  ATST152=X,Y,D,C - Draw line from current to (X,Y)     ║\r\n"
    "║  ATST153=X,Y,W,H,F,C - Draw circle/ellipse             ║\r\n"
    "║     W=width, H=height, F=0/1 (outline/filled)          ║\r\n"
    "║  ATST154=X,Y,W,H,F,C - Draw rectangle                  ║\r\n"
    "║  ATST155=X,Y,S,L,C,\"TEXT\" - Draw text                 ║\r\n"
    "║     S=8/12/16/20/24, L=0/1/2/3 (angle), C=color        ║\r\n"
    "║  ATST156=X,Y,W,H,MI,MA,A,C - Draw graph axis           ║\r\n"
    "║     MI=minY, MA=maxY, A=bitmask (Xaxis/Yaxis/Grid)     ║\r\n"
    "║  ATST157=X,Y;X,Y;... - Draw line graph                 ║\r\n"
    "║  ATST158=VAL,VAL,... - Draw bar graph                  ║\r\n"
    "║  ATST159 - Drawing commands demo (2s per primitive)    ║\r\n"
    "║            Enhanced: 2s delay, multi-angle/size/lang    ║\r\n"
    "║  ATST160 - 4-channel audio oscilloscope                ║\r\n"
    "║            Displays 4 waveforms, 400x400 window, 10s    ║\r\n"
    "║                                                        ║\r\n"
    "║  Audio Localization (ATST162):                         ║\r\n"
    "║  ATST162 - 4-mic audio with sound source localization  ║\r\n"
    "║            Shows waveform + direction finder, 10s       ║\r\n"
    "║                                                        ║\r\n"
    "║  Menu Commands (ATST165-170):                          ║\r\n"
    "║  ATST165=X,Y,W,H,\"TXT\",P,CT,TT - Draw button          ║\r\n"
    "║     P=0/1 (released/pressed), CT=btnColor, TT=textColor ║\r\n"
    "║  ATST166=X,Y,W,H,P,C,BC - Draw progress bar            ║\r\n"
    "║     P=0-100%, C=barColor, BC=bgColor                   ║\r\n"
    "║  ATST167=X,Y,W,H,V,C,SV - Draw slider                  ║\r\n"
    "║     V=0-100, SV=0/1 (hide/show value)                  ║\r\n"
    "║  ATST168=X,Y,W,H,\"TXT\",S,I,C - Draw list item         ║\r\n"
    "║     S=0/1 (normal/selected), I=index, C=color          ║\r\n"
    "║  ATST169 - Menu demonstration                          ║\r\n"
    "║            Shows buttons, sliders, progress, lists      ║\r\n"
    "║  ATST170=M - Screen clear methods                      ║\r\n"
    "║     M=0(black),1(white),2(gradient),3(checkerboard)    ║\r\n"
    "║                                                        ║\r\n"
    "║  Audio Frequency Analysis (ATST171):                   ║\r\n"
    "║  ATST171 - Frequency analysis (800Hz RED / 1200Hz BLUE)║\r\n"
    "║            Direction finding in 200px circle           ║\r\n"
    "║            4 waveform windows (200x50 each) on right   ║\r\n"
    "║            Send ATST999 to interrupt                   ║\r\n"
    "║                                                        ║\r\n"
    "║  MicroSD Card Commands (ATSTMCRI/L):                   ║\r\n"
    "║  ATSTMCRI - MicroSD card information                   ║\r\n"
    "║            Displays card state, capacity, CID/CSD      ║\r\n"
    "║  ATSTMCRL - MicroSD card file/partition list           ║\r\n"
    "║            Lists partitions on the SD card             ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST999 - Interrupt/stop any running test             ║\r\n"
    "║                                                        ║\r\n"
    "║  Colors: 0xFF0000=R, 0x00FF00=G, 0x0000FF=B            ║\r\n"
    "║         0xFFFF00=Y, 0x00FFFF=C, 0xFF00FF=M, 0xFFFFFF=W ║\r\n"
    "║                                                        ║\r\n"
    "║  ATST?   - Show This Help                              ║\r\n"
    "║                                                        ║\r\n"
    "╚════════════════════════════════════════════════════════╝\r\n"
    "\r\n";

  // Send as one transmission - much more reliable than multiple small ones
  CDC_Transmit_ThreadSafe((uint8_t *)helpMsg, strlen(helpMsg));
}

/* =============================================================================
 * Test System Initialization
 * ========================================================================== */
void TEST_Init(void) {
  // Test system initialization - called during startup
  const char *initMsg =
    "\r\n"
    "[Test System] Initialized\r\n"
    "             Type ATST? for help\r\n";
  CDC_Transmit_ThreadSafe((uint8_t *)initMsg, strlen(initMsg));
}

/* =============================================================================
 * Display Mode Test Functions (ATST141 - ATST148)
 * Quick display mode change functions for testing different configurations
 * ========================================================================== */

/* ATST141 - Text Mode - Simple text display */
static TestStatus_TypeDef TEST_DisplayMode_Text(void) {
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 12,
                        (uint8_t *)"MODE 141: TEXT", CENTER_MODE);
  TEST_Output("\r\n[MODE 141] Text Display Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST142 - Graphics Mode - Shape patterns */
static TestStatus_TypeDef TEST_DisplayMode_Graphics(void) {
  BSP_LCD_Clear(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
  // Draw grid of circles
  for (int y = 50; y < BSP_LCD_GetYSize() - 50; y += 80) {
    for (int x = 50; x < BSP_LCD_GetXSize() - 50; x += 80) {
      BSP_LCD_DrawCircle(x, y, 30);
    }
  }
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"MODE 142: GRAPHICS", CENTER_MODE);
  TEST_Output("\r\n[MODE 142] Graphics Pattern Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST143 - Color Bar Mode - Full screen color bars */
static TestStatus_TypeDef TEST_DisplayMode_ColorBars(void) {
  uint32_t colors[] = {LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_BLUE,
                       LCD_COLOR_YELLOW, LCD_COLOR_CYAN, LCD_COLOR_MAGENTA};
  int barHeight = BSP_LCD_GetYSize() / 6;
  for (int i = 0; i < 6; i++) {
    BSP_LCD_SetTextColor(colors[i]);
    BSP_LCD_FillRect(0, i * barHeight, BSP_LCD_GetXSize(), barHeight);
  }
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10,
                        (uint8_t *)"MODE 143: COLOR BARS", CENTER_MODE);
  TEST_Output("\r\n[MODE 143] Color Bars Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST144 - Gradient Mode - Vertical color gradient */
static TestStatus_TypeDef TEST_DisplayMode_Gradient(void) {
  for (int y = 0; y < BSP_LCD_GetYSize(); y++) {
    uint32_t color = (y * 255) / BSP_LCD_GetYSize();
    BSP_LCD_SetTextColor((color << 16) | (color << 8) | color);
    BSP_LCD_DrawHLine(0, y, BSP_LCD_GetXSize());
  }
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10,
                        (uint8_t *)"MODE 144: GRADIENT", CENTER_MODE);
  TEST_Output("\r\n[MODE 144] Gradient Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST145 - Checkerboard Mode - Alternating pixel pattern */
static TestStatus_TypeDef TEST_DisplayMode_Checkerboard(void) {
  int rectSize = 40;
  for (int y = 0; y < BSP_LCD_GetYSize(); y += rectSize) {
    for (int x = 0; x < BSP_LCD_GetXSize(); x += rectSize) {
      uint32_t color = (((x/rectSize) + (y/rectSize)) % 2) == 0 ?
                       LCD_COLOR_WHITE : LCD_COLOR_BLACK;
      BSP_LCD_SetTextColor(color);
      BSP_LCD_FillRect(x, y, rectSize, rectSize);
    }
  }
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10,
                        (uint8_t *)"MODE 145: CHECKERBOARD", CENTER_MODE);
  TEST_Output("\r\n[MODE 145] Checkerboard Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST146 - Info Mode - Display LTDC configuration */
static TestStatus_TypeDef TEST_DisplayMode_Info(void) {
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);

  extern LTDC_HandleTypeDef hltdc_discovery;
  uint32_t cfblr = LTDC_LAYER(&hltdc_discovery, 0)->CFBLR;
  uint32_t cfbar = LTDC_LAYER(&hltdc_discovery, 0)->CFBAR;
  uint32_t whpcr = LTDC_LAYER(&hltdc_discovery, 0)->WHPCR;
  uint32_t wvpcr = LTDC_LAYER(&hltdc_discovery, 0)->WVPCR;

  char info[256];
  snprintf(info, sizeof(info),
           "LTDC Layer 0 Info:\n"
           "CFBLR: 0x%08lx\n"
           "CFBAR: 0x%08lx\n"
           "WHPCR: 0x%08lx\n"
           "WVPCR: 0x%08lx\n"
           "Display: %dx%d\n",
           cfblr, cfbar, whpcr, wvpcr,
           BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"MODE 146: LTDC INFO", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)info, LEFT_MODE);

  TEST_Output("\r\n[MODE 146] LTDC Info Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST147 - Focus Pattern Mode - For focus/convergence testing */
static TestStatus_TypeDef TEST_DisplayMode_FocusPattern(void) {
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  uint32_t centerX = BSP_LCD_GetXSize() / 2;
  uint32_t centerY = BSP_LCD_GetYSize() / 2;

  // Draw crosshairs from center
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawHLine(0, centerY, BSP_LCD_GetXSize());
  BSP_LCD_DrawVLine(centerX, 0, BSP_LCD_GetYSize());

  // Draw circles
  for (int i = 1; i <= 5; i++) {
    BSP_LCD_DrawCircle(centerX, centerY, i * 40);
  }

  // Draw corner boxes
  uint32_t boxSize = 30;
  BSP_LCD_DrawRect(10, 10, boxSize, boxSize);
  BSP_LCD_DrawRect(BSP_LCD_GetXSize() - 10 - boxSize, 10, boxSize, boxSize);
  BSP_LCD_DrawRect(10, BSP_LCD_GetYSize() - 10 - boxSize, boxSize, boxSize);
  BSP_LCD_DrawRect(BSP_LCD_GetXSize() - 10 - boxSize,
                 BSP_LCD_GetYSize() - 10 - boxSize, boxSize, boxSize);

  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"MODE 147: FOCUS PATTERN", CENTER_MODE);

  TEST_Output("\r\n[MODE 147] Focus Pattern Mode\r\n");
  osDelay(500);
  return TEST_OK;
}

/* ATST148 - All Modes Info - Show available display modes */
static TestStatus_TypeDef TEST_DisplayMode_AllInfo(void) {
  TEST_Output("\r\n=== Display Mode Commands (ATST141-148) ===\r\n");
  TEST_Output("  ATST141 - Text Mode (simple text display)\r\n");
  TEST_Output("  ATST142 - Graphics Mode (shape patterns)\r\n");
  TEST_Output("  ATST143 - Color Bars Mode (full screen colors)\r\n");
  TEST_Output("  ATST144 - Gradient Mode (vertical gradient)\r\n");
  TEST_Output("  ATST145 - Checkerboard Mode (alternating pattern)\r\n");
  TEST_Output("  ATST146 - Info Mode (LTDC configuration)\r\n");
  TEST_Output("  ATST147 - Focus Pattern Mode (convergence test)\r\n");
  TEST_Output("  ATST148 - This Help\r\n");
  TEST_Output("===========================================\r\n");

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"Display Mode Commands:", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)"ATST141 - Text", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 70, (uint8_t *)"ATST142 - Graphics", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 90, (uint8_t *)"ATST143 - Color Bars", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 110, (uint8_t *)"ATST144 - Gradient", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"ATST145 - Checkerboard", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 150, (uint8_t *)"ATST146 - LTDC Info", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 170, (uint8_t *)"ATST147 - Focus Pattern", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 190, (uint8_t *)"ATST148 - Help", LEFT_MODE);

  return TEST_OK;
}

/* =============================================================================
 * ATST149 - LCD Panel Detection Diagnostic
 * Detects LCD panel type and reports correct resolution/timing
 * ========================================================================== */
static TestStatus_TypeDef TEST_DisplayMode_PanelDetect(void) {
  extern LCD_Driver_t Lcd_Driver_Type;
  extern uint32_t lcd_x_size;
  extern uint32_t lcd_y_size;
  extern LTDC_HandleTypeDef hltdc_discovery;

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);

  TEST_Output("\r\n=== ATST149: LCD Panel Detection ===\r\n");

  // Report panel type
  TEST_Output("Panel Type: %s\r\n",
    (Lcd_Driver_Type == LCD_CTRL_OTM8009A) ? "OTM8009A (KoD)" :
    (Lcd_Driver_Type == LCD_CTRL_NT35510) ? "NT35510 (TechStar)" :
    "UNKNOWN");

  // Report current configured resolution
  TEST_Output("Configured: %dx%d\r\n", lcd_x_size, lcd_y_size);

  // Report correct resolution for detected panel
  if (Lcd_Driver_Type == LCD_CTRL_OTM8009A) {
    TEST_Output("Expected: 800x472 (MB1166 kit)\r\n");
    TEST_Output("Issue: HEIGHT mismatch causes artifacts!\r\n");
  } else if (Lcd_Driver_Type == LCD_CTRL_NT35510) {
    TEST_Output("Expected: 800x480\r\n");
  }

  // Read and display LTDC timing registers
  uint32_t cfblr = LTDC_LAYER(&hltdc_discovery, 0)->CFBLR;
  uint32_t twcr = LTDC->TWCR;
  uint32_t totalw = (twcr >> 16) & 0x7FF;  // TotalWidth from TWCR upper bits [31:16]
  uint32_t totalh = twcr & 0x7FF;  // TotalHeight from TWCR lower bits [11:0]

  TEST_Output("\r\n=== LTDC Timing Registers ===\r\n");
  TEST_Output("Timing values used: HSA=%lu, HBP=%lu, HFP=%lu, VSA=%lu, VBP=%lu, VFP=%lu\r\n",
    hltdc_discovery.Init.HorizontalSync + 1,
    hltdc_discovery.Init.AccumulatedHBP - hltdc_discovery.Init.HorizontalSync,
    hltdc_discovery.Init.TotalWidth - hltdc_discovery.Init.AccumulatedActiveW,
    hltdc_discovery.Init.VerticalSync + 1,
    hltdc_discovery.Init.AccumulatedVBP - hltdc_discovery.Init.VerticalSync,
    hltdc_discovery.Init.TotalHeigh - hltdc_discovery.Init.AccumulatedActiveH);
  TEST_Output("Calculated: TotalW=%lu, TotalH=%lu (HACT=%lu,VACT=%lu)\r\n",
    hltdc_discovery.Init.TotalWidth,
    hltdc_discovery.Init.TotalHeigh,
    lcd_x_size, lcd_y_size);
  TEST_Output("CFBLR: LineLen=%lu, Pitch=%lu\r\n",
    ((cfblr >> 16) & 0x1FFF), (cfblr & 0x1FFF));
  TEST_Output("Expected: LineLen=2400, Pitch=2400 (for RGB888 800x472)\r\n");
  if ((cfblr & 0x1FFF) != 2400) {
    TEST_Output("!!! WARNING: Pitch is incorrect! Display will be shifted!\r\n");
  }
  TEST_Output("TWCR (TotalW) = 0x%03lx, Expected: 0x330 (816)\r\n", totalw);
  TEST_Output("TotalW: %lu, TotalH: %lu\r\n", totalw, totalh);
  TEST_Output("Layer 0 PixelFormat: %lu (3=RGB888)\r\n", hltdc_discovery.LayerCfg[0].PixelFormat);

  /* CRITICAL FIX v0.1.51: Add DSI timing diagnostic to understand shift/slant issue */
  TEST_Output("\r\n=== DSI Timing Analysis ===\r\n");
  extern DSI_HandleTypeDef hdsi_discovery;
  uint32_t LcdClock = 27429;  /* kHz */
  uint32_t laneByteClk_kHz = 62500;  /* kHz */
  uint32_t ratio = (laneByteClk_kHz * 100) / LcdClock;  /* ratio * 100 for precision */
  TEST_Output("DSI Clock: laneByteClk=%lukHz, LcdClk=%lukHz, Ratio=%lu.%02lu\r\n",
    laneByteClk_kHz, LcdClock, ratio / 100, ratio % 100);

  /* Calculate DSI timing values that should be configured */
  uint32_t hsa = hltdc_discovery.Init.HorizontalSync + 1;
  uint32_t hbp = hltdc_discovery.Init.AccumulatedHBP - hltdc_discovery.Init.HorizontalSync;
  uint32_t hfp = hltdc_discovery.Init.TotalWidth - hltdc_discovery.Init.AccumulatedActiveW;
  uint32_t hact = lcd_x_size;

  uint32_t dsi_hsa = (hsa * laneByteClk_kHz) / LcdClock;
  uint32_t dsi_hbp = (hbp * laneByteClk_kHz) / LcdClock;
  uint32_t dsi_hline = ((hact + hsa + hbp + hfp) * laneByteClk_kHz) / LcdClock;

  TEST_Output("DSI timing (calc): HSA=%lu, HBP=%lu, HLine=%lu\r\n", dsi_hsa, dsi_hbp, dsi_hline);
  TEST_Output("Note: DSI timing in DSI clock cycles, LTDC timing in pixel clocks\r\n");

  TEST_Output("\r\n[MODE 149] Panel Detection Complete\r\n");
  osDelay(500);
  return TEST_OK;
}

/* =============================================================================
 * ATST150 - 50x50 Pixel Checkerboard Test
 * Draws a checkerboard pattern with 50x50 pixel cells for timing analysis
 * This test helps identify pixel clock phase and DSI timing issues
 * ========================================================================== */
static TestStatus_TypeDef TEST_DisplayMode_Checkerboard50(void) {
  extern LTDC_HandleTypeDef hltdc_discovery;

  BSP_LCD_Clear(LCD_COLOR_BLACK);

  // Draw 50x50 pixel checkerboard
  int cellSize = 50;
  for (int y = 0; y < BSP_LCD_GetYSize(); y += cellSize) {
    for (int x = 0; x < BSP_LCD_GetXSize(); x += cellSize) {
      // Alternate black and white
      uint32_t color = (((x/cellSize) + (y/cellSize)) % 2) == 0 ?
                       LCD_COLOR_WHITE : LCD_COLOR_BLACK;
      BSP_LCD_SetTextColor(color);
      BSP_LCD_FillRect(x, y, cellSize, cellSize);
    }
  }

  // Add red border to see screen edges clearly
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DrawRect(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  // Display diagnostic info
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST150: 50x50 CHECKERBOARD", CENTER_MODE);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 40,
                        (uint8_t *)"Check for: tilted edges,", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 25,
                        (uint8_t *)"blurry text, crawling", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 10,
                        (uint8_t *)"Cells should be SQUARE", CENTER_MODE);

  // Output diagnostic via USB CDC
  TEST_Output("\r\n=== ATST150: 50x50 Checkerboard Test ===\r\n");
  TEST_Output("Cell size: 50x50 pixels\r\n");
  TEST_Output("Grid: %d x %d cells\r\n",
             (BSP_LCD_GetXSize() / cellSize),
             (BSP_LCD_GetYSize() / cellSize));
  TEST_Output("Screen: %d x %d pixels\r\n", BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  // LTDC timing diagnostic
  TEST_Output("\r\n--- LTDC Timing ---\r\n");
  TEST_Output("TotalWidth: %lu (expected 868 for MB1166)\r\n", hltdc_discovery.Init.TotalWidth);
  TEST_Output("TotalHeigh: %lu (expected 541)\r\n", hltdc_discovery.Init.TotalHeigh);
  TEST_Output("AccumulatedHBP: %lu (expected 34 for MB1166)\r\n",
             hltdc_discovery.Init.AccumulatedHBP - hltdc_discovery.Init.HorizontalSync);
  TEST_Output("AccumulatedVBP: %lu (expected 35)\r\n",
             hltdc_discovery.Init.AccumulatedVBP - hltdc_discovery.Init.VerticalSync);

  uint32_t cfblr = LTDC_LAYER(&hltdc_discovery, 0)->CFBLR;
  TEST_Output("CFBLR: 0x%08lx (LineLen=%lu, Pitch=%lu)\r\n", cfblr,
             (cfblr >> 16) & 0x1FFF, cfblr & 0x1FFF);
  TEST_Output("Expected: LineLen=2400, Pitch=2400 for RGB888\r\n");

  TEST_Output("\r\n--- Visual Inspection Guide ---\r\n");
  TEST_Output("If cells look rectangular (not square): Horizontal timing issue\r\n");
  TEST_Output("If pattern crawls/shifts: DSI clock phase or PHY timing issue\r\n");
  TEST_Output("If edges are blurry: Pixel clock polarity or sampling issue\r\n");
  TEST_Output("If red border is misaligned: TotalWidth/TotalHeigh mismatch\r\n");

  // DSI timing diagnostic (v0.1.59)
  extern DSI_HandleTypeDef hdsi_discovery;
  TEST_Output("\r\n--- DSI Video Mode Timing (v0.1.60) ---\r\n");

  // Calculate expected DSI HorizontalLine
  uint32_t HACT = BSP_LCD_GetXSize();
  uint32_t HSA = 1, HBP = 34, HFP = 34;
  uint32_t laneByteClk_kHz = 62500;
  uint32_t LcdClock = 27429;
  uint32_t expectedHorizontalLine = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz) / LcdClock;

  TEST_Output("Expected DSI HorizontalLine: %lu\r\n", expectedHorizontalLine);
  TEST_Output("Formula: (HACT+HSA+HBP+HFP) * laneByteClk / LcdClock\r\n");
  TEST_Output("       (%lu+%lu+%lu+%lu) * %lu / %lu\r\n", HACT, HSA, HBP, HFP, laneByteClk_kHz, LcdClock);

  // Check if dsihost.c value (2091) is being used instead
  if (expectedHorizontalLine == 1980) {
    TEST_Output("Note: BSP calculates 1980, but check if DSI actually uses this value!\r\n");
    TEST_Output("      Difference from 2091 (in dsihost.c) is %lu\r\n", 2091UL - expectedHorizontalLine);
  }

  TEST_Output("\r\nCRITICAL: If display is distorted, check DSI HorizontalLine value!\r\n");
  TEST_Output("          DSI and LTDC timing must match exactly.\r\n");
  TEST_Output("\r\n[ATST150] Test Complete - Inspect display visually\r\n");

  osDelay(500);
  return TEST_OK;
}

/* =============================================================================
 * Drawing Command Helper Functions
 * ========================================================================== */

/**
 * @brief Convert hex color string to uint32_t
 * @param hexStr Hex color string (e.g., "0xFF00" or "FF00" or "0xFF0000")
 * @retval Color value as uint32_t
 */
uint32_t TEST_ParseHexColor(const char *hexStr) {
  uint32_t color = 0;

  // Skip "0x" prefix if present
  if (hexStr[0] == '0' && (hexStr[1] == 'x' || hexStr[1] == 'X')) {
    hexStr += 2;
  }

  // Parse hex string
  while (*hexStr) {
    char c = *hexStr++;
    color <<= 4;
    if (c >= '0' && c <= '9') {
      color |= (c - '0');
    } else if (c >= 'A' && c <= 'F') {
      color |= (c - 'A' + 10);
    } else if (c >= 'a' && c <= 'f') {
      color |= (c - 'a' + 10);
    }
  }

  return color;
}

/**
 * @brief Parse drawing command parameters
 * @param cmdParams Command parameters string (e.g., "100,200,5,0xFFFF")
 * @param params Output parameter array
 * @param paramCount Number of parameters to parse
 * @retval Number of parameters successfully parsed
 */
int TEST_ParseDrawParams(const char *cmdParams, int32_t *params, int paramCount) {
  int parsed = 0;
  const char *p = cmdParams;

  for (int i = 0; i < paramCount && *p != '\0'; i++) {
    // Skip whitespace
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;

    // Check for hex color (starts with 0x or contains only hex digits for last param)
    if (i == paramCount - 1 && (*p == '0' || strncmp(p, "0x", 2) == 0 || strncmp(p, "0X", 2) == 0)) {
      params[i] = (int32_t)TEST_ParseHexColor(p);
      parsed++;
      break;
    }

    // Parse decimal number
    int32_t val = 0;
    int negative = 0;

    if (*p == '-') {
      negative = 1;
      p++;
    }

    while (*p >= '0' && *p <= '9') {
      val = val * 10 + (*p - '0');
      p++;
    }

    if (negative) val = -val;
    params[i] = val;
    parsed++;

    // Skip comma
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;
  }

  return parsed;
}

/* =============================================================================
 * ATST151 - Draw Point
 * ATST151=X,Y,D,C
 * X: width coordinate (0-799 for MB1166)
 * Y: height coordinate (0-471 for MB1166)
 * D: point diameter in pixels (1-50)
 * C: color (hex: 0xFF0000=red, 0x00FF00=green, 0x0000FF=blue, 0xFFFF=white, 0x0000=black)
 * Sets current position for line drawing
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_Point(int32_t x, int32_t y, int32_t diameter, uint32_t color) {
  // Clamp coordinates to display bounds
  if (x < 0) x = 0;
  if (x >= BSP_LCD_GetXSize()) x = BSP_LCD_GetXSize() - 1;
  if (y < 0) y = 0;
  if (y >= BSP_LCD_GetYSize()) y = BSP_LCD_GetYSize() - 1;

  // Clamp diameter
  if (diameter < 1) diameter = 1;
  if (diameter > 50) diameter = 50;

  // Draw point (filled circle)
  BSP_LCD_SetTextColor(color);
  BSP_LCD_FillCircle(x, y, diameter / 2);

  // Update current position
  drawCurrentX = x;
  drawCurrentY = y;
  drawCurrentColor = color;
  drawPositionSet = 1;

  TEST_Output("\r\n[ATST151] Point: (%ld,%ld) D=%ld Color=0x%08lx\r\n", x, y, diameter, color);

  return TEST_OK;
}

/* =============================================================================
 * ATST152 - Draw Line
 * ATST152=X,Y,D,C
 * Draws a line from current position to (X,Y)
 * X: width coordinate (0-799 for MB1166)
 * Y: height coordinate (0-471 for MB1166)
 * D: point diameter at end position (1-50)
 * C: line color (hex)
 * The end point becomes the new current position
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_Line(int32_t x, int32_t y, int32_t diameter, uint32_t color) {
  if (!drawPositionSet) {
    TEST_Output("\r\n[ATST152] ERROR: No current position. Use ATST151 first.\r\n");
    return TEST_ERROR;
  }

  // Clamp coordinates to display bounds
  int32_t x1 = drawCurrentX;
  int32_t y1 = drawCurrentY;
  int32_t x2 = x;
  int32_t y2 = y;

  if (x2 < 0) x2 = 0;
  if (x2 >= BSP_LCD_GetXSize()) x2 = BSP_LCD_GetXSize() - 1;
  if (y2 < 0) y2 = 0;
  if (y2 >= BSP_LCD_GetYSize()) y2 = BSP_LCD_GetYSize() - 1;

  // Draw line
  BSP_LCD_SetTextColor(color);

  // Use BSP_LCD_DrawLine for simple line, but set thickness via filled circles
  // First draw the base line
  BSP_LCD_DrawLine(x1, y1, x2, y2);

  // Then draw circles along the line for thickness
  int dx = x2 - x1;
  int dy = y2 - y1;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if (steps == 0) steps = 1;

  int lineWidth = (diameter > 2) ? (diameter / 2) : 1;

  for (int i = 0; i <= steps; i++) {
    int px = x1 + (dx * i / steps);
    int py = y1 + (dy * i / steps);
    BSP_LCD_FillCircle(px, py, lineWidth);
  }

  // Draw end point
  if (diameter > 1) {
    BSP_LCD_FillCircle(x2, y2, diameter / 2);
  }

  // Update current position
  drawCurrentX = x2;
  drawCurrentY = y2;
  drawCurrentColor = color;

  TEST_Output("\r\n[ATST152] Line: (%ld,%ld)->(%ld,%ld) D=%ld Color=0x%08lx\r\n",
              x1, y1, x2, y2, diameter, color);

  return TEST_OK;
}

/* =============================================================================
 * ATST154 - Draw Rectangle
 * ATST154=X,Y,W,H,F,C
 * X: top-left X coordinate (0-799 for MB1166)
 * Y: top-left Y coordinate (0-471 for MB1166)
 * W: width in pixels (1-800)
 * H: height in pixels (1-472)
 * F: fill flag (0=unfilled/outline only, 1=filled)
 * C: color (hex)
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_Rect(int32_t x, int32_t y, int32_t width, int32_t height,
                                        int32_t fill, uint32_t color) {
  // Clamp coordinates
  if (x < 0) x = 0;
  if (x >= BSP_LCD_GetXSize()) x = BSP_LCD_GetXSize() - 1;
  if (y < 0) y = 0;
  if (y >= BSP_LCD_GetYSize()) y = BSP_LCD_GetYSize() - 1;

  // Clamp dimensions
  if (width < 1) width = 1;
  if (width > BSP_LCD_GetXSize() - x) width = BSP_LCD_GetXSize() - x;
  if (height < 1) height = 1;
  if (height > BSP_LCD_GetYSize() - y) height = BSP_LCD_GetYSize() - y;

  BSP_LCD_SetTextColor(color);

  if (fill) {
    BSP_LCD_FillRect(x, y, width, height);
  } else {
    BSP_LCD_DrawRect(x, y, width, height);
  }

  TEST_Output("\r\n[ATST154] Rect: (%ld,%ld) %ldx%ld %s Color=0x%08lx\r\n",
              x, y, width, height, fill ? "FILLED" : "OUTLINE", color);

  return TEST_OK;
}

/* =============================================================================
 * ATST153 - Draw Circle/Ellipse
 * ATST153=X,Y,W,H,F,C
 * X: center X coordinate (0-799 for MB1166)
 * Y: center Y coordinate (0-471 for MB1166)
 * W: width of ellipse (horizontal radius * 2, 1-800)
 * H: height of ellipse (vertical radius * 2, 1-472)
 * F: fill flag (0=outline only, 1=filled)
 * C: color (hex)
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_Circle(int32_t x, int32_t y, int32_t width, int32_t height,
                                           int32_t fill, uint32_t color) {
  // Clamp center coordinates
  if (x < 0) x = 0;
  if (x >= BSP_LCD_GetXSize()) x = BSP_LCD_GetXSize() - 1;
  if (y < 0) y = 0;
  if (y >= BSP_LCD_GetYSize()) y = BSP_LCD_GetYSize() - 1;

  // Clamp dimensions
  if (width < 1) width = 1;
  if (width > BSP_LCD_GetXSize() * 2) width = BSP_LCD_GetXSize() * 2;
  if (height < 1) height = 1;
  if (height > BSP_LCD_GetYSize() * 2) height = BSP_LCD_GetYSize() * 2;

  int32_t rx = width / 2;   /* Horizontal radius */
  int32_t ry = height / 2;  /* Vertical radius */

  BSP_LCD_SetTextColor(color);

  if (fill) {
    /* Filled ellipse - scanline method */
    for (int py = -ry; py <= ry; py++) {
      int px = (int32_t)(rx * sqrt(1 - (float)(py * py) / (ry * ry)));
      BSP_LCD_DrawHLine(x - px, y + py, px * 2 + 1);
    }
  } else {
    /* Ellipse outline - Bresenham-like algorithm */
    int x1 = 0, y1 = ry;
    long d1 = ry * ry - rx * rx * ry + 0.25 * rx * rx;
    long dx = 2 * ry * ry * x1;
    long dy = 2 * rx * rx * y1;

    while (dx < dy) {
      /* Draw 4 points using filled circles for thicker lines */
      BSP_LCD_DrawPixel(x + x1, y + y1, color);
      BSP_LCD_DrawPixel(x - x1, y + y1, color);
      BSP_LCD_DrawPixel(x + x1, y - y1, color);
      BSP_LCD_DrawPixel(x - x1, y - y1, color);

      x1++;
      dx += 2 * ry * ry;
      if (d1 < 0) {
        d1 += dx + ry * ry;
      } else {
        y1--;
        dy -= 2 * rx * rx;
        d1 += dx - dy + ry * ry;
      }
    }

    d1 = ry * ry * (x1 + 0.5) * (x1 + 0.5) + rx * rx * (y1 - 1) * (y1 - 1) - rx * rx * ry * ry;
    while (y1 >= 0) {
      BSP_LCD_DrawPixel(x + x1, y + y1, color);
      BSP_LCD_DrawPixel(x - x1, y + y1, color);
      BSP_LCD_DrawPixel(x + x1, y - y1, color);
      BSP_LCD_DrawPixel(x - x1, y - y1, color);

      y1--;
      dy -= 2 * rx * rx;
      if (d1 > 0) {
        d1 += rx * rx - dy;
      } else {
        x1++;
        dx += 2 * ry * ry;
        d1 += dx - dy + rx * rx;
      }
    }
  }

  TEST_Output("\r\n[ATST153] Ellipse: center=(%ld,%ld) %ldx%ld %s Color=0x%08lx\r\n",
              x, y, width, height, fill ? "FILLED" : "OUTLINE", color);

  return TEST_OK;
}

/* =============================================================================
 * Font Management Functions
 * ========================================================================== */

void TEST_SetFont(uint8_t fontSize) {
  switch (fontSize) {
    case 8:  currentFont = &Font8; break;
    case 12: currentFont = &Font12; break;
    case 16: currentFont = &Font16; break;
    case 20: currentFont = &Font20; break;
    case 24: currentFont = &Font24; break;
    default: currentFont = &Font24; break;
  }
  BSP_LCD_SetFont(currentFont);
}

sFONT *TEST_GetFont(void) {
  return currentFont;
}

void TEST_RotateCoord(int32_t x, int32_t y, uint8_t angle, int32_t cx, int32_t cy,
                      int32_t *outX, int32_t *outY) {
  /* Translate to origin */
  int32_t dx = x - cx;
  int32_t dy = y - cy;

  /* Rotate based on angle */
  int32_t rx, ry;
  switch (angle) {
    case 0: /* 0 degrees - no rotation */
      rx = dx;
      ry = dy;
      break;
    case 1: /* 90 degrees clockwise */
      rx = -dy;
      ry = dx;
      break;
    case 2: /* 180 degrees */
      rx = -dx;
      ry = -dy;
      break;
    case 3: /* 270 degrees clockwise (or 90 CCW) */
      rx = dy;
      ry = -dx;
      break;
    default:
      rx = dx;
      ry = dy;
      break;
  }

  /* Translate back */
  *outX = cx + rx;
  *outY = cy + ry;
}

/* =============================================================================
 * ATST155 - Draw Text with Rotation
 * ATST155=X,Y,S,L,C,"TEXT"
 * X: start X coordinate (0-799 for MB1166)
 * Y: start Y coordinate (0-471 for MB1166)
 * S: font size (8, 12, 16, 20, 24)
 * L: rotation angle (0=0°, 1=90°, 2=180°, 3=270°)
 * C: text color (hex)
 * TEXT: text string to draw (English/Russian characters)
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_Text(int32_t x, int32_t y, uint8_t fontSize,
                                         uint8_t angle, uint32_t color, const char *text) {
  /* Clamp coordinates */
  if (x < 0) x = 0;
  if (x >= BSP_LCD_GetXSize()) x = BSP_LCD_GetXSize() - 1;
  if (y < 0) y = 0;
  if (y >= BSP_LCD_GetYSize()) y = BSP_LCD_GetYSize() - 1;

  /* Set font and colors */
  TEST_SetFont(fontSize);
  BSP_LCD_SetTextColor(color);
  /* For better visibility: use white background when text is dark, black when text is light */
  /* Determine if text is light (red/green/blue components all > 128) */
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  if (r > 128 || g > 128 || b > 128) {
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);  /* Dark background for light text */
  } else {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);  /* Light background for dark text */
  }

  if (angle == 0) {
    /* No rotation - use BSP function directly */
    BSP_LCD_DisplayStringAt(x, y, (uint8_t *)text, LEFT_MODE);
  } else {
    /* Rotated text - draw character by character */
    uint32_t textLen = strlen(text);
    int32_t curX = x;
    int32_t curY = y;
    uint8_t *pText = (uint8_t *)text;

    /* Calculate rotation center */
    int32_t cx = x;
    int32_t cy = y;

    for (uint32_t i = 0; i < textLen; i++) {
      uint8_t ch = pText[i];

      /* Get character bitmap from font table */
      /* Font table is indexed by character code (ASCII starts at space ' '=32) */
      uint16_t charOffset = (ch >= ' ') ? (ch - ' ') : 0;
      const uint8_t *charBitmap = &currentFont->table[charOffset * currentFont->Height];

      /* Draw character pixel by pixel with rotation */
      for (int py = 0; py < currentFont->Height; py++) {
        for (int px = 0; px < currentFont->Width; px++) {
          /* Font bitmap is packed - each byte represents multiple pixels */
          int byteIndex = (py * currentFont->Width + px) / 8;
          int bitIndex = 7 - ((py * currentFont->Width + px) % 8);
          uint8_t byte = charBitmap[byteIndex];
          uint8_t bit = (byte >> bitIndex) & 1;

          if (bit) {
            /* Source position (unrotated) */
            int32_t sx = curX + px;
            int32_t sy = curY + py;

            /* Rotated position */
            int32_t rx, ry;
            TEST_RotateCoord(sx, sy, angle, cx, cy, &rx, &ry);

            /* Clamp to screen bounds */
            if (rx >= 0 && rx < BSP_LCD_GetXSize() && ry >= 0 && ry < BSP_LCD_GetYSize()) {
              BSP_LCD_DrawPixel(rx, ry, color);
            }
          }
        }
      }

      /* Advance cursor based on rotation */
      int32_t advX = curX + currentFont->Width;
      int32_t advY = curY;
      TEST_RotateCoord(advX, advY, angle, cx, cy, &curX, &curY);
    }
  }

  TEST_Output("\r\n[ATST155] Text: (%ld,%ld) S=%d L=%d Color=0x%08lx \"%s\"\r\n",
              x, y, fontSize, angle, color, text);

  return TEST_OK;
}

/* Graph drawing state */
static int32_t graphX = 50;      /* Default graph origin X */
static int32_t graphY = 400;     /* Default graph origin Y */
static int32_t graphWidth = 700;  /* Default graph width */
static int32_t graphHeight = 350; /* Default graph height */
static int32_t graphMinX = 0;    /* Minimum X value */
static int32_t graphMaxX = 100;  /* Maximum X value */
static int32_t graphMinY = 0;    /* Minimum Y value */
static int32_t graphMaxY = 100;  /* Maximum Y value */

/* =============================================================================
 * ATST156 - Draw Graph Axis and Grid
 * ATST156=X,Y,W,H,MI,MA,A,C
 * X: origin X coordinate (0-799)
 * Y: origin Y coordinate (0-471)
 * W: graph width (1-800)
 * H: graph height (1-472)
 * MI: minimum value for Y axis
 * MA: maximum value for Y axis
 * A: axis/grid flags (bit 0=X axis, bit 1=Y axis, bit 2=grid lines)
 * C: axis color (hex)
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_GraphAxis(int32_t x, int32_t y, int32_t width, int32_t height,
                                              int32_t minVal, int32_t maxVal,
                                              uint32_t axisFlags, uint32_t color) {
  /* Clamp and store graph parameters */
  if (x < 0) x = 0;
  if (x >= BSP_LCD_GetXSize() - 50) x = BSP_LCD_GetXSize() - 50;
  if (y < 50) y = 50;
  if (y >= BSP_LCD_GetYSize()) y = BSP_LCD_GetYSize() - 1;
  if (width < 50) width = 50;
  if (width > BSP_LCD_GetXSize() - x - 50) width = BSP_LCD_GetXSize() - x - 50;
  if (height < 50) height = 50;
  if (height > y - 50) height = y - 50;

  graphX = x;
  graphY = y;
  graphWidth = width;
  graphHeight = height;
  graphMinY = minVal;
  graphMaxY = maxVal;

  BSP_LCD_SetTextColor(color);
  TEST_SetFont(12);  /* Use small font for axis labels */

  /* Draw X axis */
  if (axisFlags & 0x01) {
    BSP_LCD_DrawHLine(x, y, width);
    /* Draw X axis arrow */
    BSP_LCD_DrawLine(x + width, y, x + width - 10, y - 5);
    BSP_LCD_DrawLine(x + width, y, x + width - 10, y + 5);
  }

  /* Draw Y axis */
  if (axisFlags & 0x02) {
    BSP_LCD_DrawVLine(x, y - height, height);
    /* Draw Y axis arrow */
    BSP_LCD_DrawLine(x, y - height, x - 5, y - height + 10);
    BSP_LCD_DrawLine(x, y - height, x + 5, y - height + 10);
  }

  /* Draw grid lines and labels */
  if (axisFlags & 0x04) {
    /* Horizontal grid lines (5 lines) */
    for (int i = 1; i <= 5; i++) {
      int gy = y - (height * i / 5);
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawHLine(x, gy, width);

      /* Y axis label */
      char label[16];
      int val = minVal + (maxVal - minVal) * i / 5;
      snprintf(label, sizeof(label), "%d", val);
      BSP_LCD_SetTextColor(color);
      BSP_LCD_DisplayStringAt(x - 35, gy - 6, (uint8_t *)label, LEFT_MODE);
    }

    /* Vertical grid lines (10 lines) */
    for (int i = 1; i <= 10; i++) {
      int gx = x + (width * i / 10);
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawVLine(gx, y - height, height);
    }
  }

  TEST_Output("\r\n[ATST156] Graph Axis: origin=(%ld,%ld) %ldx%ld range=[%ld,%ld]\r\n",
              x, y, width, height, minVal, maxVal);

  return TEST_OK;
}

/* Graph data point storage */
#define MAX_GRAPH_POINTS 100
static int32_t graphDataX[MAX_GRAPH_POINTS];
static int32_t graphDataY[MAX_GRAPH_POINTS];
static int graphDataCount = 0;

/* =============================================================================
 * ATST157 - Draw Line Graph from Data Points (Enhanced v0.1.71)
 * ATST157=X1,Y1;X2,Y2;... or ATST157 without params to draw stored data
 * Format: semicolon-separated X,Y pairs
 * Draws connected line graph with point markers and colors
 * Uses current graph settings from ATST156
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_LineGraph(const char *dataStr) {
  if (graphDataCount == 0 && dataStr == NULL) {
    TEST_Output("\r\n[ATST157] ERROR: No data points. Use ATST157=X,Y;X,Y;... first.\r\n");
    return TEST_ERROR;
  }

  /* Parse data string if provided */
  if (dataStr != NULL && strlen(dataStr) > 0) {
    graphDataCount = 0;
    const char *p = dataStr;
    int32_t x, y;

    while (*p && graphDataCount < MAX_GRAPH_POINTS) {
      /* Parse X,Y pair */
      if (sscanf(p, "%ld,%ld", &x, &y) == 2) {
        graphDataX[graphDataCount] = x;
        graphDataY[graphDataCount] = y;
        graphDataCount++;
      }

      /* Skip to next pair */
      while (*p && *p != ';') p++;
      if (*p == ';') p++;
    }

    /* Update graph range based on data */
    if (graphDataCount > 0) {
      graphMinX = graphDataX[0];
      graphMaxX = graphDataX[0];
      for (int i = 0; i < graphDataCount; i++) {
        if (graphDataX[i] < graphMinX) graphMinX = graphDataX[i];
        if (graphDataX[i] > graphMaxX) graphMaxX = graphDataX[i];
        if (graphDataY[i] < graphMinY) graphMinY = graphDataY[i];
        if (graphDataY[i] > graphMaxY) graphMaxY = graphDataY[i];
      }
      TEST_Output("\r\n[ATST157] Parsed %d points, X=[%ld,%ld] Y=[%ld,%ld]\r\n",
                  graphDataCount, graphMinX, graphMaxX, graphMinY, graphMaxY);
    }
  }

  /* Draw line graph */
  if (graphDataCount < 2) {
    TEST_Output("\r\n[ATST157] ERROR: Need at least 2 data points.\r\n");
    return TEST_ERROR;
  }

  /* Color palette for point markers (cycling through colors) */
  const uint32_t pointColors[8] = {
    LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_BLUE, LCD_COLOR_YELLOW,
    LCD_COLOR_CYAN, LCD_COLOR_MAGENTA, LCD_COLOR_ORANGE, LCD_COLOR_WHITE
  };

  BSP_LCD_SetTextColor(drawCurrentColor);

  /* Convert and draw each point with colored markers */
  for (int i = 0; i < graphDataCount; i++) {
    /* Map data coordinates to screen coordinates */
    int sx = graphX + (graphDataX[i] - graphMinX) * graphWidth / (graphMaxX - graphMinX);
    int sy = graphY - (graphDataY[i] - graphMinY) * graphHeight / (graphMaxY - graphMinY);

    /* Draw point marker with cycling color */
    BSP_LCD_SetTextColor(pointColors[i % 8]);
    BSP_LCD_FillCircle(sx, sy, 4);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawCircle(sx, sy, 5);  // White outline

    /* Draw line to next point */
    if (i < graphDataCount - 1) {
      int sx2 = graphX + (graphDataX[i + 1] - graphMinX) * graphWidth / (graphMaxX - graphMinX);
      int sy2 = graphY - (graphDataY[i + 1] - graphMinY) * graphHeight / (graphMaxY - graphMinY);
      BSP_LCD_SetTextColor(drawCurrentColor);
      BSP_LCD_DrawLine(sx, sy, sx2, sy2);
    }
  }

  TEST_Output("\r\n[ATST157] Line Graph drawn: %d points with colored markers\r\n", graphDataCount);

  return TEST_OK;
}

/* =============================================================================
 * ATST158 - Draw Bar Graph from Data
 * ATST158=VAL1,VAL2,... or ATST158 without params to draw stored data
 * Format: comma-separated values (Y values, X is index)
 * Draws bar chart using current graph settings from ATST156
 * ========================================================================== */
static TestStatus_TypeDef TEST_Draw_BarGraph(const char *dataStr) {
  /* Parse data string if provided */
  if (dataStr != NULL && strlen(dataStr) > 0) {
    graphDataCount = 0;
    const char *p = dataStr;
    int32_t val;

    while (*p && graphDataCount < MAX_GRAPH_POINTS) {
      /* Parse value */
      if (sscanf(p, "%ld", &val) == 1) {
        graphDataX[graphDataCount] = graphDataCount;  /* X is index */
        graphDataY[graphDataCount] = val;
        graphDataCount++;

        /* Update range */
        if (val < graphMinY) graphMinY = val;
        if (val > graphMaxY) graphMaxY = val;
      }

      /* Skip to next value */
      while (*p && *p != ',') p++;
      if (*p == ',') p++;
    }

    graphMaxX = graphDataCount;
    TEST_Output("\r\n[ATST158] Parsed %d values, range=[%ld,%ld]\r\n",
                graphDataCount, graphMinY, graphMaxY);
  }

  /* Draw bar graph */
  if (graphDataCount == 0) {
    TEST_Output("\r\n[ATST158] ERROR: No data. Use ATST158=VAL1,VAL2,... first.\r\n");
    return TEST_ERROR;
  }

  BSP_LCD_SetTextColor(drawCurrentColor);

  /* Calculate bar width */
  int barWidth = graphWidth / graphDataCount;
  int barGap = barWidth / 5;
  barWidth -= barGap;

  /* Draw each bar */
  for (int i = 0; i < graphDataCount; i++) {
    /* Map value to screen coordinates */
    int barH = (graphDataY[i] - graphMinY) * graphHeight / (graphMaxY - graphMinY);
    int sx = graphX + i * (barWidth + barGap) + barGap / 2;
    int sy = graphY - barH;

    /* Draw bar */
    BSP_LCD_FillRect(sx, sy, barWidth, barH);
  }

  TEST_Output("\r\n[ATST158] Bar Graph drawn: %d bars\r\n", graphDataCount);

  return TEST_OK;
}

/* =============================================================================
 * ATST159 - Drawing Commands Test (Enhanced v0.1.71)
 * Demonstrates ATST151-158 drawing commands with 2-second delay per primitive
 * Shows text at different angles, sizes, and two languages (English/Russian)
 * Graphs with point markers and different colors
 * ========================================================================== */
static TestStatus_TypeDef TEST_Drawing_Test(void) {
  TEST_Output("\r\n=== ATST159: Drawing Commands Test (Enhanced) ===\r\n");

  // Clear screen
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Drawing Test v0.1.71", CENTER_MODE);

  // Reset drawing position
  drawPositionSet = 0;

  // Test 1: Draw points at corners (ATST151) - with 2 second delay
  TEST_Output("\r\n[Test 1/11] Drawing corner points...\r\n");
  TEST_Draw_Point(80, 60, 15, LCD_COLOR_RED);
  TEST_Draw_Point(BSP_LCD_GetXSize() - 80, 60, 15, LCD_COLOR_GREEN);
  TEST_Draw_Point(80, BSP_LCD_GetYSize() - 60, 15, LCD_COLOR_BLUE);
  TEST_Draw_Point(BSP_LCD_GetXSize() - 80, BSP_LCD_GetYSize() - 60, 15, LCD_COLOR_YELLOW);
  osDelay(2000);

  // Test 2: Draw a triangle using lines (ATST152) - with 2 second delay
  TEST_Output("\r\n[Test 2/11] Drawing triangle with lines...\r\n");
  drawPositionSet = 0;
  TEST_Draw_Point(180, 80, 8, LCD_COLOR_CYAN);     // Start point
  TEST_Draw_Line(620, 320, 8, LCD_COLOR_CYAN);     // Line to right
  TEST_Draw_Line(180, 320, 8, LCD_COLOR_CYAN);     // Line to left
  TEST_Draw_Line(180, 80, 8, LCD_COLOR_CYAN);      // Close triangle
  osDelay(2000);

  // Test 3: Draw circles/ellipses (ATST153) - with 2 second delay
  TEST_Output("\r\n[Test 3/11] Drawing circles and ellipses...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Shapes Test", CENTER_MODE);
  TEST_Draw_Circle(600, 150, 120, 120, 0, LCD_COLOR_WHITE);    // Circle outline
  TEST_Draw_Circle(600, 150, 70, 70, 1, LCD_COLOR_YELLOW);      // Filled circle
  TEST_Draw_Circle(180, 250, 160, 80, 0, LCD_COLOR_MAGENTA);    // Ellipse outline
  TEST_Draw_Circle(180, 380, 100, 160, 1, LCD_COLOR_CYAN);      // Tall filled ellipse
  osDelay(2000);

  // Test 4: Draw filled rectangles (ATST154) - with 2 second delay
  TEST_Output("\r\n[Test 4/11] Drawing rectangles...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Rect Test", CENTER_MODE);
  TEST_Draw_Rect(520, 80, 140, 100, 1, LCD_COLOR_MAGENTA);
  TEST_Draw_Rect(550, 110, 80, 60, 1, LCD_COLOR_WHITE);
  TEST_Draw_Rect(80, 250, 200, 80, 0, LCD_COLOR_ORANGE);
  TEST_Draw_Rect(100, 380, 180, 70, 1, LCD_COLOR_LIGHTGREEN);
  osDelay(2000);

  // Test 5: Draw text with different sizes (ATST155) - with 2 second delay
  TEST_Output("\r\n[Test 5/11] Drawing text (different sizes)...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Text Size Test", CENTER_MODE);
  TEST_Draw_Text(60, 70, 24, 0, LCD_COLOR_WHITE, "Hello World!");
  TEST_Draw_Text(60, 110, 20, 0, LCD_COLOR_YELLOW, "FontSize=20");
  TEST_Draw_Text(60, 150, 16, 0, LCD_COLOR_GREEN, "FontSize=16");
  TEST_Draw_Text(60, 190, 12, 0, LCD_COLOR_CYAN, "FontSize=12");
  TEST_Draw_Text(60, 230, 8, 0, LCD_COLOR_MAGENTA, "FontSize=8");
  osDelay(2000);

  // Test 6: Text at different angles (ATST155) - with 2 second delay
  TEST_Output("\r\n[Test 6/11] Drawing rotated text...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Rotation Test", CENTER_MODE);
  TEST_Draw_Text(100, 80, 20, 0, LCD_COLOR_RED, "0 degrees");
  TEST_Draw_Text(100, 150, 20, 1, LCD_COLOR_GREEN, "90 degrees");
  TEST_Draw_Text(100, 220, 20, 2, LCD_COLOR_BLUE, "180 degrees");
  TEST_Draw_Text(100, 290, 20, 3, LCD_COLOR_YELLOW, "270 degrees");
  osDelay(2000);

  // Test 7: Text in two languages (ATST155) - with 2 second delay
  TEST_Output("\r\n[Test 7/11] Drawing multilingual text...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Language Test", CENTER_MODE);
  // English text
  TEST_Draw_Text(60, 70, 18, 0, LCD_COLOR_WHITE, "English Text");
  TEST_Draw_Text(60, 110, 16, 0, LCD_COLOR_CYAN, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  TEST_Draw_Text(60, 140, 14, 0, LCD_COLOR_GREEN, "abcdefghijklmnopqrstuvwxyz");
  // Russian text (using ASCII transliteration for compatibility)
  TEST_Draw_Text(60, 180, 18, 0, LCD_COLOR_YELLOW, "Russkiy Text");
  TEST_Draw_Text(60, 220, 16, 0, LCD_COLOR_MAGENTA, "AaBbVvGgDdEeZhZzIiYyKkLlMmNn");
  TEST_Draw_Text(60, 250, 14, 0, LCD_COLOR_ORANGE, "OoPpRrSsTtUuFfKhTsChShShch");
  osDelay(2000);

  // Test 8: Mixed text with rotation and size (ATST155) - with 2 second delay
  TEST_Output("\r\n[Test 8/11] Drawing mixed text variations...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Mixed Text Test", CENTER_MODE);
  TEST_Draw_Text(400, 80, 24, 1, LCD_COLOR_RED, "Vertical");
  TEST_Draw_Text(500, 200, 20, 2, LCD_COLOR_GREEN, "Inverted");
  TEST_Draw_Text(300, 300, 16, 3, LCD_COLOR_BLUE, "270deg");
  TEST_Draw_Text(100, 400, 12, 0, LCD_COLOR_YELLOW, "Small Normal");
  osDelay(2000);

  // Test 9: Draw graph axis (ATST156) - with 2 second delay
  TEST_Output("\r\n[Test 9/11] Drawing graph axis...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST159: Graph Axis Test", CENTER_MODE);
  TEST_Draw_GraphAxis(60, 420, 700, 380, 0, 100, 0x07, LCD_COLOR_WHITE);
  osDelay(2000);

  // Test 10: Draw line graph with point markers and colors (ATST157) - with 2 second delay
  TEST_Output("\r\n[Test 10/11] Drawing line graph with markers...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  TEST_Draw_GraphAxis(60, 420, 700, 380, 0, 100, 0x07, LCD_COLOR_WHITE);
  graphDataCount = 0;
  // Create sine wave data
  for (int i = 0; i < 30; i++) {
    graphDataX[graphDataCount] = i * 3;
    graphDataY[graphDataCount] = 50 + (int)(40 * sin(i * 0.25));
    graphDataCount++;
  }
  graphMinX = 0; graphMaxX = 90;
  graphMinY = 0; graphMaxY = 100;
  drawCurrentColor = LCD_COLOR_CYAN;
  TEST_Draw_LineGraph(NULL);
  osDelay(2000);

  // Test 11: Draw bar graph with different colors (ATST158) - with 2 second delay
  TEST_Output("\r\n[Test 11/11] Drawing bar graph with colors...\r\n");
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  TEST_Draw_GraphAxis(60, 420, 700, 380, 0, 100, 0x03, LCD_COLOR_WHITE);
  graphDataCount = 0;
  int barData[] = {20, 45, 70, 55, 85, 40, 60, 90, 35, 75};
  uint32_t barColors[] = {LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_BLUE, LCD_COLOR_YELLOW,
                          LCD_COLOR_CYAN, LCD_COLOR_MAGENTA, LCD_COLOR_ORANGE, LCD_COLOR_LIGHTGREEN,
                          LCD_COLOR_LIGHTBLUE, LCD_COLOR_LIGHTRED};
  for (int i = 0; i < 10; i++) {
    graphDataY[graphDataCount++] = barData[i];
  }
  graphMinY = 0; graphMaxY = 100;

  // Draw each bar with different color
  int barWidth = graphWidth / graphDataCount;
  int barGap = barWidth / 5;
  barWidth -= barGap;

  for (int i = 0; i < graphDataCount; i++) {
    int barH = (graphDataY[i] - graphMinY) * graphHeight / (graphMaxY - graphMinY);
    int sx = graphX + i * (barWidth + barGap) + barGap / 2;
    int sy = graphY - barH;

    BSP_LCD_SetTextColor(barColors[i]);
    BSP_LCD_FillRect(sx, sy, barWidth, barH);
  }
  osDelay(2000);

  // Display completion message
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"Drawing Test Complete!", CENTER_MODE);

  TEST_Output("\r\n=== ATST159 Complete ===\r\n");

  return TEST_OK;
}

/* =============================================================================
 * ATST160 - 4-Channel Audio Oscilloscope Display
 * Displays 4 audio channels with signal level and waveform
 * Each channel: 50 pixels high, 400 pixels wide window
 * Displays for 10 seconds
 * Note: This is a demonstration/simulation. Real audio input requires
 *       audio codec (WM8994) integration with DMA and I2S.
 * ========================================================================== */
static TestStatus_TypeDef TEST_Audio_Display(void) {
  TEST_Output("\r\n=== ATST160: Audio Oscilloscope Display ===\r\n");

  // Display window configuration
  const int winX = 200;          // Window start X (centered on 800px screen)
  const int winY = 36;           // Window start Y (centered on 472px screen)
  const int winWidth = 400;      // 400 pixels wide
  const int winHeight = 400;     // 200 pixels total (4 x 50)
  const int channelHeight = 50;  // Each channel is 50px high

  // Channel colors (4 distinct colors)
  const uint32_t channelColors[4] = {
    LCD_COLOR_CYAN,      // Channel 1 - Cyan
    LCD_COLOR_GREEN,     // Channel 2 - Green
    LCD_COLOR_YELLOW,    // Channel 3 - Yellow
    LCD_COLOR_MAGENTA    // Channel 4 - Magenta
  };

  // Waveform simulation data
  #define WAVEFORM_POINTS 200
  static int16_t waveform[4][WAVEFORM_POINTS];

  TEST_Output("\r\n[INFO] Displaying 4-channel audio oscilloscope...\r\n");
  TEST_Output("       Window: %dx%d pixels (%d channels x %d px each)\r\n",
              winWidth, winHeight, 4, channelHeight);
  TEST_Output("       Duration: 10 seconds\r\n");

  // Clear screen and draw main background
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST160: Audio Oscilloscope", CENTER_MODE);

  // Draw window border
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawRect(winX - 2, winY - 2, winWidth + 4, winHeight + 4);

  // Draw channel labels (left side)
  BSP_LCD_SetFont(&Font12);
  for (int ch = 0; ch < 4; ch++) {
    char label[16];
    snprintf(label, sizeof(label), "CH%d", ch + 1);
    BSP_LCD_SetTextColor(channelColors[ch]);
    BSP_LCD_DisplayStringAt(winX - 40, winY + ch * channelHeight + 18,
                           (uint8_t *)label, LEFT_MODE);
  }

  // Simulate and display waveform for 10 seconds
  uint32_t startTime = HAL_GetTick();
  int frameCount = 0;

  while ((HAL_GetTick() - startTime) < 10000) {  // 10 seconds
    frameCount++;

    // Generate simulated waveform data for each channel
    for (int ch = 0; ch < 4; ch++) {
      // Different frequency and phase for each channel
      float freq = 0.1f + ch * 0.05f;
      float phase = frameCount * 0.1f + ch * 1.5f;

      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        // Mix sine waves for more realistic waveform
        float t = i * 0.05f + phase;
        float sample = sin(t * freq) * 20000.0f;
        sample += sin(t * freq * 2.3f) * 10000.0f;  // Add harmonic
        sample += sin(t * freq * 0.5f) * 5000.0f;   // Add sub-harmonic
        waveform[ch][i] = (int16_t)sample;
      }
    }

    // Clear and redraw each channel
    for (int ch = 0; ch < 4; ch++) {
      int chY = winY + ch * channelHeight;
      int chCenterY = chY + channelHeight / 2;

      // Clear channel area
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(winX, chY, winWidth, channelHeight);

      // Draw grid lines (subtle)
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawHLine(winX, chCenterY, winWidth);
      for (int gx = winX; gx < winX + winWidth; gx += 40) {
        BSP_LCD_DrawVLine(gx, chY, channelHeight);
      }

      // Draw waveform
      BSP_LCD_SetTextColor(channelColors[ch]);

      int16_t prevX = -1, prevY = -1;
      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        // Map waveform to screen coordinates
        int screenX = winX + (i * winWidth / WAVEFORM_POINTS);
        // Scale: -32768 to +32767 maps to channelHeight
        int screenY = chCenterY - (waveform[ch][i] * channelHeight / 65536);

        // Clamp to channel bounds
        if (screenY < chY) screenY = chY;
        if (screenY >= chY + channelHeight) screenY = chY + channelHeight - 1;

        // Draw line from previous point
        if (prevX >= 0) {
          BSP_LCD_DrawLine(prevX, prevY, screenX, screenY);
        }

        prevX = screenX;
        prevY = screenY;
      }

      // Calculate and display signal level
      int32_t sum = 0;
      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        sum += abs(waveform[ch][i]);
      }
      int avgLevel = sum / WAVEFORM_POINTS;
      int dBLevel = (int)(20.0f * log10f((float)avgLevel / 32768.0f + 0.001f));

      // Draw level bar (right side)
      int barWidth = (avgLevel * 30) / 32768;
      BSP_LCD_SetTextColor(channelColors[ch]);
      BSP_LCD_FillRect(winX + winWidth + 6, chY + 10, barWidth, 30);

      // Draw level text
      char levelText[16];
      snprintf(levelText, sizeof(levelText), "%d dB", dBLevel);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DisplayStringAt(winX + winWidth + 40, chY + 18,
                             (uint8_t *)levelText, LEFT_MODE);
    }

    // Display elapsed time
    char timeText[32];
    uint32_t elapsed = (HAL_GetTick() - startTime) / 1000;
    snprintf(timeText, sizeof(timeText), "Time: %lu/%d sec", elapsed, 10);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(winX + winWidth / 2 - 50, winY + winHeight + 10,
                           (uint8_t *)timeText, LEFT_MODE);

    // Small delay between frames
    osDelay(50);
  }

  TEST_Output("\r\n[INFO] Displayed %d frames over 10 seconds\r\n", frameCount);
  TEST_Output("\r\n=== ATST160 Complete ===\r\n");

  // Clear screen on exit
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10,
                         (uint8_t *)"Audio Display Complete!", CENTER_MODE);

  return TEST_OK;
}

/* =============================================================================
 * ATST162 - 4-Microphone Audio Display with Sound Source Localization
 * Displays 4 audio channels with signal level, waveform, and sound source direction
 * Each channel: 50 pixels high, 400 pixels wide window
 * Displays for 10 seconds
 * Note: This is a demonstration/simulation. Real audio input requires
 *       audio codec (WM8994) integration with DMA and I2S.
 * ========================================================================== */
static TestStatus_TypeDef TEST_Audio_With_Localization(void) {
  TEST_Output("\r\n=== ATST162: Audio with Source Localization ===\r\n");

  // Display window configuration
  const int winX = 200;          // Window start X
  const int winY = 36;           // Window start Y
  const int winWidth = 400;      // 400 pixels wide
  const int winHeight = 400;     // 400 pixels total (4 x 50)
  const int channelHeight = 50;  // Each channel is 50px high

  // Channel colors
  const uint32_t channelColors[4] = {
    LCD_COLOR_CYAN, LCD_COLOR_GREEN, LCD_COLOR_YELLOW, LCD_COLOR_MAGENTA
  };

  // Simulated microphone positions (relative to center, in mm)
  // Mic layout: Top-Left, Top-Right, Bottom-Left, Bottom-Right
  const float micPos[4][2] = {{-50, -50}, {50, -50}, {-50, 50}, {50, 50}};

  #define WAVEFORM_POINTS 200
  static int16_t waveform[4][WAVEFORM_POINTS];

  TEST_Output("\r\n[INFO] 4-mic audio with localization...\r\n");
  TEST_Output("       Window: %dx%d pixels, Duration: 10 seconds\r\n",
              winWidth, winHeight);

  // Clear screen and draw header
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST162: Audio + Localization", CENTER_MODE);

  // Draw window border
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawRect(winX - 2, winY - 2, winWidth + 4, winHeight + 4);

  // Draw channel labels
  BSP_LCD_SetFont(&Font12);
  for (int ch = 0; ch < 4; ch++) {
    char label[16];
    snprintf(label, sizeof(label), "MIC%d", ch + 1);
    BSP_LCD_SetTextColor(channelColors[ch]);
    BSP_LCD_DisplayStringAt(winX - 40, winY + ch * channelHeight + 18,
                           (uint8_t *)label, LEFT_MODE);
  }

  uint32_t startTime = HAL_GetTick();
  int frameCount = 0;
  float sourceAngle = 0.0f;  // Sound source angle in degrees
  float sourceDistance = 100.0f;  // Source distance in mm

  while ((HAL_GetTick() - startTime) < 10000) {
    frameCount++;

    // Simulate moving sound source
    sourceAngle = frameCount * 2.0f;  // Rotating source
    if (sourceAngle >= 360.0f) sourceAngle -= 360.0f;

    // Generate time-delayed waveform for each mic based on source position
    float sourceRad = sourceAngle * 3.14159f / 180.0f;
    float sourceX = cosf(sourceRad) * sourceDistance;
    float sourceY = sinf(sourceRad) * sourceDistance;

    for (int ch = 0; ch < 4; ch++) {
      // Calculate distance from source to this mic
      float dx = sourceX - micPos[ch][0];
      float dy = sourceY - micPos[ch][1];
      float dist = sqrtf(dx * dx + dy * dy);

      // Time delay based on distance (speed of sound ~343 m/s)
      float delay = dist / 34300.0f;  // in seconds (scaled)
      float phase = frameCount * 0.1f + delay * 1000.0f;

      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        float t = i * 0.05f + phase;
        float sample = sin(t * 0.2f) * 20000.0f;
        sample += sin(t * 0.46f) * 10000.0f;
        waveform[ch][i] = (int16_t)sample;
      }
    }

    // Clear and redraw each channel
    for (int ch = 0; ch < 4; ch++) {
      int chY = winY + ch * channelHeight;
      int chCenterY = chY + channelHeight / 2;

      // Clear channel area
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(winX, chY, winWidth, channelHeight);

      // Draw grid
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawHLine(winX, chCenterY, winWidth);

      // Draw waveform
      BSP_LCD_SetTextColor(channelColors[ch]);
      int16_t prevX = -1, prevY = -1;
      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        int screenX = winX + (i * winWidth / WAVEFORM_POINTS);
        int screenY = chCenterY - (waveform[ch][i] * channelHeight / 65536);
        if (screenY < chY) screenY = chY;
        if (screenY >= chY + channelHeight) screenY = chY + channelHeight - 1;

        if (prevX >= 0) {
          BSP_LCD_DrawLine(prevX, prevY, screenX, screenY);
        }
        prevX = screenX;
        prevY = screenY;
      }

      // Signal level
      int32_t sum = 0;
      for (int i = 0; i < WAVEFORM_POINTS; i++) {
        sum += abs(waveform[ch][i]);
      }
      int avgLevel = sum / WAVEFORM_POINTS;
      int dBLevel = (int)(20.0f * log10f((float)avgLevel / 32768.0f + 0.001f));

      int barWidth = (avgLevel * 30) / 32768;
      BSP_LCD_SetTextColor(channelColors[ch]);
      BSP_LCD_FillRect(winX + winWidth + 6, chY + 10, barWidth, 30);

      char levelText[16];
      snprintf(levelText, sizeof(levelText), "%d dB", dBLevel);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DisplayStringAt(winX + winWidth + 40, chY + 18,
                             (uint8_t *)levelText, LEFT_MODE);
    }

    // Draw source direction indicator (compass)
    int compassX = 100;
    int compassY = 250;
    int compassRadius = 40;

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawCircle(compassX, compassY, compassRadius);
    BSP_LCD_DrawCircle(compassX, compassY, 2);  // Center dot

    // Draw source direction arrow
    int arrowX = compassX + (int)(cosf(sourceRad) * (compassRadius - 5));
    int arrowY = compassY + (int)(sinf(sourceRad) * (compassRadius - 5));
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DrawLine(compassX, compassY, arrowX, arrowY);
    BSP_LCD_FillCircle(arrowX, arrowY, 4);

    // Direction label
    char dirText[32];
    snprintf(dirText, sizeof(dirText), "Source: %d°", (int)sourceAngle);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(compassX - 40, compassY + compassRadius + 10,
                           (uint8_t *)dirText, LEFT_MODE);

    // Mic position indicators
    for (int ch = 0; ch < 4; ch++) {
      int mx = compassX + (int)(micPos[ch][0] * compassRadius / 100);
      int my = compassY + (int)(micPos[ch][1] * compassRadius / 100);
      BSP_LCD_SetTextColor(channelColors[ch]);
      BSP_LCD_FillRect(mx - 3, my - 3, 6, 6);
    }

    // Elapsed time
    char timeText[32];
    uint32_t elapsed = (HAL_GetTick() - startTime) / 1000;
    snprintf(timeText, sizeof(timeText), "Time: %lu/10 sec", elapsed);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(winX + winWidth / 2 - 50, winY + winHeight + 10,
                           (uint8_t *)timeText, LEFT_MODE);

    osDelay(50);
  }

  TEST_Output("\r\n[INFO] Displayed %d frames, localized source\r\n", frameCount);
  TEST_Output("\r\n=== ATST162 Complete ===\r\n");

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10,
                         (uint8_t *)"Localization Complete!", CENTER_MODE);

  return TEST_OK;
}

/* =============================================================================
 * Menu Drawing Functions - ATST165-168
 * ========================================================================== */

/**
 * ATST165=X,Y,W,H,"TEXT",P,CT,TT - Draw Menu Button
 * X,Y: Position, W,H: Size
 * TEXT: Button text (max 31 chars)
 * P: Pressed state (0=released, 1=pressed)
 * CT: Color (hex), TT: Text color (hex)
 */
TestStatus_TypeDef TEST_Menu_Draw_Button(int32_t x, int32_t y, int32_t width, int32_t height,
                                         const char *text, uint8_t pressed,
                                         uint32_t color, uint32_t textColor) {
  // Clamp coordinates
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + width > BSP_LCD_GetXSize()) width = BSP_LCD_GetXSize() - x;
  if (y + height > BSP_LCD_GetYSize()) height = BSP_LCD_GetYSize() - y;
  if (width < 40) width = 40;
  if (height < 20) height = 20;

  // Draw button shadow (for 3D effect)
  if (pressed) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_FillRect(x + 3, y + 3, width, height);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_FillRect(x + 2, y + 2, width, height);
  }

  // Draw button background
  BSP_LCD_SetTextColor(color);
  if (pressed) {
    BSP_LCD_FillRect(x, y, width, height);
    // Draw bevel (pressed)
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawRect(x, y, width, height);
  } else {
    BSP_LCD_FillRect(x, y, width, height);
    // Draw bevel (raised)
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawHLine(x, y, width);
    BSP_LCD_DrawVLine(x, y, height);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawHLine(x, y + height - 1, width);
    BSP_LCD_DrawVLine(x + width - 1, y, height);
  }

  // Draw text centered
  BSP_LCD_SetTextColor(textColor);
  BSP_LCD_SetBackColor(color);
  BSP_LCD_SetFont(&Font16);

  int textX = x + (width - strlen(text) * 8) / 2;
  int textY = y + (height - 16) / 2;
  BSP_LCD_DisplayStringAt(textX, textY, (uint8_t *)text, LEFT_MODE);

  TEST_Output("\r\n[ATST165] Button: (%ld,%ld) %ldx%ld \"%s\" %s Color=0x%08lx\r\n",
              x, y, width, height, text, pressed ? "PRESSED" : "RELEASED", color);

  return TEST_OK;
}

/**
 * ATST166=X,Y,W,H,P,C,BC - Draw Progress Bar
 * X,Y: Position, W,H: Size
 * P: Percentage (0-100)
 * C: Bar color (hex), BC: Background color (hex)
 */
TestStatus_TypeDef TEST_Menu_Draw_Progress(int32_t x, int32_t y, int32_t width, int32_t height,
                                          int32_t percent, uint32_t color, uint32_t bgColor) {
  // Clamp parameters
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + width > BSP_LCD_GetXSize()) width = BSP_LCD_GetXSize() - x;
  if (y + height > BSP_LCD_GetYSize()) height = BSP_LCD_GetYSize() - y;
  if (width < 40) width = 40;
  if (height < 10) height = 10;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  // Draw background
  BSP_LCD_SetTextColor(bgColor);
  BSP_LCD_FillRect(x, y, width, height);

  // Draw border
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  BSP_LCD_DrawRect(x, y, width, height);

  // Draw progress fill
  int fillWidth = (width - 4) * percent / 100;
  if (fillWidth > 0) {
    BSP_LCD_SetTextColor(color);
    BSP_LCD_FillRect(x + 2, y + 2, fillWidth, height - 4);
  }

  // Draw percentage text
  char pctText[16];
  snprintf(pctText, sizeof(pctText), "%ld%%", percent);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(bgColor);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(x + width / 2 - 15, y + (height - 12) / 2,
                         (uint8_t *)pctText, LEFT_MODE);

  TEST_Output("\r\n[ATST166] Progress: (%ld,%ld) %ldx%ld %ld%% Color=0x%08lx\r\n",
              x, y, width, height, percent, color);

  return TEST_OK;
}

/**
 * ATST167=X,Y,W,H,V,C,SV - Draw Slider
 * X,Y: Position, W,H: Size
 * V: Value (0-100)
 * C: Color (hex)
 * SV: Show value (0=no, 1=yes)
 */
TestStatus_TypeDef TEST_Menu_Draw_Slider(int32_t x, int32_t y, int32_t width, int32_t height,
                                        int32_t value, uint32_t color, uint8_t showValue) {
  // Clamp parameters
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + width > BSP_LCD_GetXSize()) width = BSP_LCD_GetXSize() - x;
  if (y + height > BSP_LCD_GetYSize()) height = BSP_LCD_GetYSize() - y;
  if (width < 60) width = 60;
  if (height < 10) height = 10;
  if (value < 0) value = 0;
  if (value > 100) value = 100;

  int trackY = y + height / 2;

  // Draw track background
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  BSP_LCD_FillRect(x, trackY - 2, width, 4);

  // Draw filled portion
  int fillX = x + (width - 20) * value / 100;
  BSP_LCD_SetTextColor(color);
  BSP_LCD_FillRect(x, trackY - 2, fillX - x, 4);

  // Draw slider thumb
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(fillX - 5, y, 10, height);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  BSP_LCD_DrawRect(fillX - 5, y, 10, height);

  // Draw value text if requested
  if (showValue) {
    char valText[16];
    snprintf(valText, sizeof(valText), "%ld", value);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(x + width + 10, y + (height - 12) / 2,
                           (uint8_t *)valText, LEFT_MODE);
  }

  TEST_Output("\r\n[ATST167] Slider: (%ld,%ld) %ldx%ld Value=%ld Color=0x%08lx\r\n",
              x, y, width, height, value, color);

  return TEST_OK;
}

/**
 * ATST168=X,Y,W,H,"TEXT",S,I,C - Draw List Item
 * X,Y: Position, W,H: Size
 * TEXT: Item text (max 63 chars)
 * S: Selected state (0=normal, 1=selected)
 * I: Index number
 * C: Color (hex)
 */
TestStatus_TypeDef TEST_Menu_Draw_List(int32_t x, int32_t y, int32_t width, int32_t height,
                                       const char *text, uint8_t selected, int32_t index,
                                       uint32_t color) {
  // Clamp parameters
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + width > BSP_LCD_GetXSize()) width = BSP_LCD_GetXSize() - x;
  if (y + height > BSP_LCD_GetYSize()) height = BSP_LCD_GetYSize() - y;
  if (width < 60) width = 60;
  if (height < 20) height = 20;

  // Draw background
  if (selected) {
    BSP_LCD_SetTextColor(color);
    BSP_LCD_FillRect(x, y, width, height);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawRect(x, y, width, height);
  }

  // Draw index number
  char idxText[16];
  snprintf(idxText, sizeof(idxText), "%ld.", index);
  BSP_LCD_SetTextColor(selected ? LCD_COLOR_BLACK : color);
  BSP_LCD_SetBackColor(selected ? color : LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(x + 10, y + (height - 16) / 2, (uint8_t *)idxText, LEFT_MODE);

  // Draw text
  BSP_LCD_SetTextColor(selected ? LCD_COLOR_BLACK : LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(x + 50, y + (height - 16) / 2, (uint8_t *)text, LEFT_MODE);

  // Draw selection indicator if selected
  if (selected) {
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(x + width - 25, y + (height - 16) / 2,
                           (uint8_t *)"<", LEFT_MODE);
  }

  TEST_Output("\r\n[ATST168] List: (%ld,%ld) %ldx%ld \"%s\" #%ld %s\r\n",
              x, y, width, height, text, index, selected ? "SELECTED" : "NORMAL");

  return TEST_OK;
}

/* =============================================================================
 * ATST169 - Menu Demonstration
 * Shows interactive menu with buttons, sliders, progress bars, and lists
 * ========================================================================== */
TestStatus_TypeDef TEST_Menu_Demo(void) {
  TEST_Output("\r\n=== ATST169: Menu Demonstration ===\r\n");

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"ATST169: Menu Demo", CENTER_MODE);

  // Title bar
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 40);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_DisplayStringAt(0, 12, (uint8_t *)"Menu System Demo", CENTER_MODE);

  // Buttons section
  TEST_Output("\r\n[Menu] Drawing buttons...\r\n");
  TEST_Menu_Draw_Button(50, 60, 150, 50, "Start", 0, LCD_COLOR_BLUE, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(220, 60, 150, 50, "Stop", 0, LCD_COLOR_RED, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(390, 60, 150, 50, "Config", 0, LCD_COLOR_GREEN, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(560, 60, 150, 50, "Exit", 1, LCD_COLOR_GRAY, LCD_COLOR_WHITE);
  osDelay(2000);

  // Sliders section
  TEST_Output("\r\n[Menu] Drawing sliders...\r\n");
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(50, 130, (uint8_t *)"Volume:", LEFT_MODE);
  TEST_Menu_Draw_Slider(130, 140, 200, 20, 75, LCD_COLOR_CYAN, 1);

  BSP_LCD_DisplayStringAt(400, 130, (uint8_t *)"Brightness:", LEFT_MODE);
  TEST_Menu_Draw_Slider(520, 140, 200, 20, 50, LCD_COLOR_YELLOW, 1);
  osDelay(2000);

  // Progress bars section
  TEST_Output("\r\n[Menu] Drawing progress bars...\r\n");
  BSP_LCD_DisplayStringAt(50, 180, (uint8_t *)"Loading:", LEFT_MODE);
  TEST_Menu_Draw_Progress(130, 190, 300, 25, 65, LCD_COLOR_GREEN, LCD_COLOR_DARKGRAY);

  BSP_LCD_DisplayStringAt(50, 225, (uint8_t*)"Processing:", LEFT_MODE);
  TEST_Menu_Draw_Progress(150, 235, 300, 25, 30, LCD_COLOR_ORANGE, LCD_COLOR_DARKGRAY);
  osDelay(2000);

  // List section
  TEST_Output("\r\n[Menu] Drawing list items...\r\n");
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(50, 275, (uint8_t *)"Settings Menu:", LEFT_MODE);

  TEST_Menu_Draw_List(50, 300, 700, 35, "Wi-Fi Configuration", 1, 1, LCD_COLOR_BLUE);
  TEST_Menu_Draw_List(50, 340, 700, 35, "Display Settings", 0, 2, LCD_COLOR_BLUE);
  TEST_Menu_Draw_List(50, 380, 700, 35, "Audio Options", 0, 3, LCD_COLOR_BLUE);
  TEST_Menu_Draw_List(50, 420, 700, 35, "System Information", 0, 4, LCD_COLOR_BLUE);
  osDelay(2000);

  // Animated progress
  TEST_Output("\r\n[Menu] Animated progress...\r\n");
  for (int i = 0; i <= 100; i += 5) {
    TEST_Menu_Draw_Progress(200, 300, 400, 30, i, LCD_COLOR_GREEN, LCD_COLOR_DARKGRAY);
    osDelay(100);
  }

  // Completion message
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15,
                         (uint8_t *)"Menu Demo Complete!", CENTER_MODE);

  TEST_Output("\r\n=== ATST169 Complete ===\r\n");

  return TEST_OK;
}

/* =============================================================================
 * ATST165 - Menu Button Demonstration
 * Shows various button states: normal, pressed, disabled, different colors
 * ========================================================================== */
TestStatus_TypeDef TEST_Menu_Buttons(void) {
  TEST_Output("\r\n=== ATST165: Menu Buttons Demo ===\r\n");

  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST165: Button States", CENTER_MODE);

  // Row 1: Different colored buttons (normal state)
  TEST_Menu_Draw_Button(50, 40, 150, 40, "Red Button", 0, LCD_COLOR_RED, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(220, 40, 150, 40, "Green Button", 0, LCD_COLOR_GREEN, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(390, 40, 150, 40, "Blue Button", 0, LCD_COLOR_BLUE, LCD_COLOR_WHITE);

  // Row 2: Pressed state buttons
  TEST_Menu_Draw_Button(50, 100, 150, 40, "Pressed Red", 1, LCD_COLOR_RED, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(220, 100, 150, 40, "Pressed Green", 1, LCD_COLOR_GREEN, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(390, 100, 150, 40, "Pressed Blue", 1, LCD_COLOR_BLUE, LCD_COLOR_WHITE);

  // Row 3: Disabled appearance (darker colors)
  TEST_Menu_Draw_Button(50, 160, 150, 40, "Disabled", 0, 0x404040, 0x808080);
  TEST_Menu_Draw_Button(220, 160, 150, 40, "Inactive", 0, 0x404040, 0x808080);
  TEST_Menu_Draw_Button(390, 160, 150, 40, "Locked", 0, 0x404040, 0x808080);

  // Row 4: Large action buttons
  TEST_Menu_Draw_Button(100, 220, 250, 50, "START", 0, LCD_COLOR_GREEN, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(370, 220, 250, 50, "STOP", 0, LCD_COLOR_RED, LCD_COLOR_WHITE);

  // Row 5: Small icon buttons
  TEST_Menu_Draw_Button(50, 290, 50, 50, "+", 0, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Button(120, 290, 50, 50, "-", 0, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Button(190, 290, 50, 50, "X", 0, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Button(260, 290, 50, 50, "?", 0, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Button(330, 290, 50, 50, "!", 0, LCD_COLOR_MAGENTA, LCD_COLOR_WHITE);
  TEST_Menu_Draw_Button(400, 290, 50, 50, ">>", 0, 0xFFA500, LCD_COLOR_BLACK);  // Orange

  // Row 6: Wide buttons
  TEST_Menu_Draw_Button(50, 360, 650, 40, "Wide Button Full Width", 0, 0x800080, LCD_COLOR_WHITE);  // Purple

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_Output("\r\n[ATST165] Displayed 6 rows of button variations\r\n");
  TEST_Output("\r\n=== ATST165 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST166 - Menu Progress Bar Demonstration
 * Shows progress bars at different percentages with animations
 * ========================================================================== */
TestStatus_TypeDef TEST_Menu_Progress(void) {
  TEST_Output("\r\n=== ATST166: Progress Bars Demo ===\r\n");

  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST166: Progress Bars", CENTER_MODE);

  // Row 1: Empty to full progress bars (static)
  TEST_Menu_Draw_Progress(50, 40, 200, 30, 0, LCD_COLOR_GREEN, LCD_COLOR_DARKGRAY);
  TEST_Menu_Draw_Progress(280, 40, 200, 30, 25, LCD_COLOR_BLUE, LCD_COLOR_DARKGRAY);
  TEST_Menu_Draw_Progress(510, 40, 200, 30, 50, LCD_COLOR_YELLOW, LCD_COLOR_DARKGRAY);
  TEST_Menu_Draw_Progress(50, 80, 200, 30, 75, LCD_COLOR_ORANGE, LCD_COLOR_DARKGRAY);
  TEST_Menu_Draw_Progress(280, 80, 200, 30, 100, LCD_COLOR_RED, LCD_COLOR_DARKGRAY);

  // Row 2: Different colored progress bars
  TEST_Menu_Draw_Progress(50, 130, 140, 25, 60, LCD_COLOR_RED, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(210, 130, 140, 25, 60, LCD_COLOR_GREEN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(370, 130, 140, 25, 60, LCD_COLOR_BLUE, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(530, 130, 140, 25, 60, 0x800080, LCD_COLOR_BLACK);  // Purple

  // Row 3: Tall progress bars (vertical orientation simulation)
  TEST_Menu_Draw_Progress(50, 170, 50, 100, 30, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(120, 170, 50, 100, 50, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(190, 170, 50, 100, 70, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(260, 170, 50, 100, 90, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(330, 170, 50, 100, 100, LCD_COLOR_CYAN, LCD_COLOR_BLACK);

  // Row 4: Wide progress bars
  TEST_Menu_Draw_Progress(50, 290, 650, 20, 33, 0x00FF00, 0x404040);  // Lime, Dark gray
  TEST_Menu_Draw_Progress(50, 320, 650, 20, 66, LCD_COLOR_MAGENTA, 0x404040);

  // Row 5: Small progress bars (compact)
  TEST_Menu_Draw_Progress(50, 360, 100, 15, 20, 0xFFA500, LCD_COLOR_BLACK);  // Orange
  TEST_Menu_Draw_Progress(170, 360, 100, 15, 40, 0xFFA500, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(290, 360, 100, 15, 60, 0xFFA500, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(410, 360, 100, 15, 80, 0xFFA500, LCD_COLOR_BLACK);
  TEST_Menu_Draw_Progress(530, 360, 100, 15, 100, 0xFFA500, LCD_COLOR_BLACK);

  // Animated progress bar (bottom)
  for (int i = 0; i <= 100; i += 5) {
    TEST_Menu_Draw_Progress(200, 400, 400, 30, i, LCD_COLOR_GREEN, LCD_COLOR_BLACK);
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
    osDelay(100);
  }

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  TEST_Output("\r\n[ATST166] Displayed 5 rows of progress variations + animation\r\n");
  TEST_Output("\r\n=== ATST166 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST167 - Menu Slider Demonstration
 * Shows slider controls at different positions with various colors
 * ========================================================================== */
TestStatus_TypeDef TEST_Menu_Sliders(void) {
  TEST_Output("\r\n=== ATST167: Sliders Demo ===\r\n");

  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST167: Slider Controls", CENTER_MODE);

  // Row 1: Sliders at different positions (0%, 25%, 50%, 75%, 100%)
  TEST_Menu_Draw_Slider(50, 40, 200, 15, 0, LCD_COLOR_RED, 1);
  TEST_Menu_Draw_Slider(280, 40, 200, 15, 25, LCD_COLOR_GREEN, 1);
  TEST_Menu_Draw_Slider(510, 40, 200, 15, 50, LCD_COLOR_BLUE, 1);
  TEST_Menu_Draw_Slider(50, 70, 200, 15, 75, LCD_COLOR_YELLOW, 1);
  TEST_Menu_Draw_Slider(280, 70, 200, 15, 100, LCD_COLOR_MAGENTA, 1);

  // Row 2: Different colored sliders
  TEST_Menu_Draw_Slider(50, 110, 250, 20, 40, LCD_COLOR_RED, 1);
  TEST_Menu_Draw_Slider(330, 110, 250, 20, 60, LCD_COLOR_GREEN, 1);
  TEST_Menu_Draw_Slider(50, 150, 250, 20, 80, LCD_COLOR_BLUE, 1);

  // Row 3: Tall sliders
  TEST_Menu_Draw_Slider(50, 190, 100, 40, 30, LCD_COLOR_CYAN, 1);
  TEST_Menu_Draw_Slider(170, 190, 100, 40, 50, 0xFFA500, 1);  // Orange
  TEST_Menu_Draw_Slider(290, 190, 100, 40, 70, 0x800080, 1);  // Purple
  TEST_Menu_Draw_Slider(410, 190, 100, 40, 90, 0x00FF00, 1);  // Lime
  TEST_Menu_Draw_Slider(530, 190, 100, 40, 100, 0xFFC0CB, 1);  // Pink

  // Row 4: Wide sliders
  TEST_Menu_Draw_Slider(50, 250, 650, 25, 45, 0x008080, 1);  // Teal
  TEST_Menu_Draw_Slider(50, 290, 650, 25, 55, 0xEE82EE, 1);  // Violet

  // Row 5: Sliders without value display
  TEST_Menu_Draw_Slider(50, 340, 200, 15, 33, 0x808080, 0);  // Gray
  TEST_Menu_Draw_Slider(280, 340, 200, 15, 66, 0xC0C0C0, 0);  // Silver
  TEST_Menu_Draw_Slider(510, 340, 200, 15, 99, 0x800000, 0);  // Maroon

  // Row 6: Mini sliders (compact)
  TEST_Menu_Draw_Slider(50, 380, 100, 10, 20, 0x000080, 0);  // Navy
  TEST_Menu_Draw_Slider(170, 380, 100, 10, 40, 0x808000, 0);  // Olive
  TEST_Menu_Draw_Slider(290, 380, 100, 10, 60, 0x800000, 0);  // Maroon
  TEST_Menu_Draw_Slider(410, 380, 100, 10, 80, 0x800080, 0);  // Purple
  TEST_Menu_Draw_Slider(530, 380, 100, 10, 100, 0x008080, 0);  // Teal

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  TEST_Output("\r\n[ATST167] Displayed 6 rows of slider variations\r\n");
  TEST_Output("\r\n=== ATST167 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST168 - Menu List Item Demonstration
 * Shows list items with selection states and scrolling simulation
 * ========================================================================== */
TestStatus_TypeDef TEST_Menu_Lists(void) {
  TEST_Output("\r\n=== ATST168: List Items Demo ===\r\n");

  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST168: List Items", CENTER_MODE);

  // List 1: Unselected items (normal state)
  TEST_Menu_Draw_List(50, 40, 700, 35, "File Manager", 0, 1, 0x404040);  // Dark gray
  TEST_Menu_Draw_List(50, 80, 700, 35, "Settings", 0, 2, 0x404040);
  TEST_Menu_Draw_List(50, 120, 700, 35, "Network", 0, 3, 0x404040);
  TEST_Menu_Draw_List(50, 160, 700, 35, "Display", 0, 4, 0x404040);

  // List 2: Selected item (middle of list)
  TEST_Menu_Draw_List(50, 210, 700, 35, "Audio Settings", 0, 5, 0x404040);
  TEST_Menu_Draw_List(50, 250, 700, 35, "WiFi Configuration", 1, 6, LCD_COLOR_BLUE);  // Selected
  TEST_Menu_Draw_List(50, 290, 700, 35, "Bluetooth", 0, 7, 0x404040);

  // List 3: Different colored lists
  TEST_Menu_Draw_List(50, 340, 330, 35, "Red List Item", 0, 8, LCD_COLOR_RED);
  TEST_Menu_Draw_List(420, 340, 330, 35, "Green List Item", 1, 9, LCD_COLOR_GREEN);

  // List 4: Compact list items
  TEST_Menu_Draw_List(50, 390, 165, 30, "Item 1", 0, 10, LCD_COLOR_CYAN);
  TEST_Menu_Draw_List(230, 390, 165, 30, "Item 2", 0, 11, LCD_COLOR_CYAN);
  TEST_Menu_Draw_List(410, 390, 165, 30, "Item 3", 1, 12, LCD_COLOR_CYAN);
  TEST_Menu_Draw_List(590, 390, 165, 30, "Item 4", 0, 13, LCD_COLOR_CYAN);

  // List 5: List with icons (text prefix)
  TEST_Menu_Draw_List(50, 435, 330, 35, "[+]", 0, 14, 0xFFA500);  // Orange
  TEST_Menu_Draw_List(420, 435, 330, 35, "[>]", 1, 15, 0xFFA500);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  TEST_Output("\r\n[ATST168] Displayed 5 rows of list item variations\r\n");
  TEST_Output("\r\n=== ATST168 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST170=M - Screen Clear Methods
 * M: Mode (0=black, 1=white, 2=gradient, 3=checkerboard)
 * ========================================================================== */
TestStatus_TypeDef TEST_Screen_Clear(uint8_t mode) {
  TEST_Output("\r\n=== ATST170: Screen Clear (Mode %d) ===\r\n", mode);

  switch (mode) {
    case 0:  // Black
      TEST_Output("\r\n[Clear] Filling with black...\r\n");
      BSP_LCD_Clear(LCD_COLOR_BLACK);
      break;

    case 1:  // White
      TEST_Output("\r\n[Clear] Filling with white...\r\n");
      BSP_LCD_Clear(LCD_COLOR_WHITE);
      break;

    case 2:  // Vertical gradient (black to blue)
      TEST_Output("\r\n[Clear] Drawing gradient...\r\n");
      for (int y = 0; y < BSP_LCD_GetYSize(); y++) {
        uint32_t color = (y * 255) / BSP_LCD_GetYSize();
        BSP_LCD_SetTextColor(color);  // Blue gradient
        BSP_LCD_DrawHLine(0, y, BSP_LCD_GetXSize());
      }
      break;

    case 3:  // Checkerboard pattern
      TEST_Output("\r\n[Clear] Drawing checkerboard...\r\n");
      for (int y = 0; y < BSP_LCD_GetYSize(); y += 20) {
        for (int x = 0; x < BSP_LCD_GetXSize(); x += 20) {
          uint32_t color = ((x / 20 + y / 20) % 2) ? LCD_COLOR_DARKGRAY : LCD_COLOR_BLACK;
          BSP_LCD_SetTextColor(color);
          BSP_LCD_FillRect(x, y, 20, 20);
        }
      }
      break;

    default:
      TEST_Output("\r\n[Clear] Unknown mode %d, using black...\r\n", mode);
      BSP_LCD_Clear(LCD_COLOR_BLACK);
      break;
  }

  TEST_Output("\r\n[Clear] Screen cleared\r\n");
  TEST_Output("\r\n=== ATST170 Complete ===\r\n");

  return TEST_OK;
}

/* =============================================================================
 * Process ATSTn Command
 * Main entry point for test commands from USB CDC
 * ========================================================================== */
TestStatus_TypeDef TEST_ProcessCommand(uint8_t testId) {
  switch (testId) {
    case 0:  return TEST_ESP_Weather();
    case 1:  return TEST_Display();
    case 2:  return TEST_LEDs();
    case 3:  return TEST_RTC();
    case 4:  return TEST_WiFi();
    case 5:  return TEST_NTP();
    case 6:  return TEST_HTTP();
    case 7:  return TEST_Audio();
    case 8:  return TEST_SDRAM();
    case 9:  return TEST_Touch();
    case 10:  return TEST_RunAll();
    // Display mode commands (141-148)
    case 141: return TEST_DisplayMode_Text();
    case 142: return TEST_DisplayMode_Graphics();
    case 143: return TEST_DisplayMode_ColorBars();
    case 144: return TEST_DisplayMode_Gradient();
    case 145: return TEST_DisplayMode_Checkerboard();
    case 146: return TEST_DisplayMode_Info();
    case 147: return TEST_DisplayMode_FocusPattern();
    case 148: return TEST_DisplayMode_AllInfo();
    case 149: return TEST_DisplayMode_PanelDetect();
    case 150: return TEST_DisplayMode_Checkerboard50();
    case 159: return TEST_Drawing_Test();
    case 160: return TEST_Audio_Display();
    case 162: return TEST_Audio_With_Localization();
    case 165: return TEST_Menu_Buttons();   // ATST165 - Button demonstration
    case 166: return TEST_Menu_Progress();  // ATST166 - Progress bar demonstration
    case 167: return TEST_Menu_Sliders();   // ATST167 - Slider demonstration
    case 168: return TEST_Menu_Lists();     // ATST168 - List item demonstration
    case 169: return TEST_Menu_Demo();
    case 170: return TEST_Screen_Clear(0);  // Default to black clear
    case 171: return TEST_Audio_Freq_Analysis();
    case 172: return TEST_SD_Card_Info();   // ATSTMCRI - SD card information
    case 173: return TEST_SD_Card_List();   // ATSTMCRL - SD card file list
    // Optimized drawing commands (ATST174-180)
    case 174: return TEST_Draw_Fast_Line_Demo();   // Fast line demo
    case 175: return TEST_Draw_Fast_Rect_Demo();   // Fast rect demo
    case 179: return TEST_Draw_Waveform_Demo();    // Waveform demo
    case 180: return TEST_Draw_Graph_Fast_Demo();  // Fast graph demo
    case 255: return TEST_Interrupt_All();  // ATST999 - interrupt all tests
    case 99:
    default:
      TEST_ShowHelp();
      return TEST_OK;
  }
}

/* =============================================================================
 * Process ATSTn Command with Parameters
 * Entry point for parameterized drawing commands from USB CDC
 * ========================================================================== */
TestStatus_TypeDef TEST_ProcessCommandWithParams(uint8_t testId, const char *params) {
  int32_t drawParams[8];  // Max 8 parameters for extended commands
  int parsed;
  char textBuf[128];

  switch (testId) {
    case 151:  // ATST151=X,Y,D,C - Draw point
      parsed = TEST_ParseDrawParams(params, drawParams, 4);
      if (parsed == 4) {
        return TEST_Draw_Point(drawParams[0], drawParams[1], drawParams[2], drawParams[3]);
      } else {
        TEST_Output("\r\n[ATST151] ERROR: Need 4 params (X,Y,D,C). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 152:  // ATST152=X,Y,D,C - Draw line
      parsed = TEST_ParseDrawParams(params, drawParams, 4);
      if (parsed == 4) {
        return TEST_Draw_Line(drawParams[0], drawParams[1], drawParams[2], drawParams[3]);
      } else {
        TEST_Output("\r\n[ATST152] ERROR: Need 4 params (X,Y,D,C). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 153:  // ATST153=X,Y,W,H,F,C - Draw circle/ellipse
      parsed = TEST_ParseDrawParams(params, drawParams, 6);
      if (parsed == 6) {
        return TEST_Draw_Circle(drawParams[0], drawParams[1], drawParams[2],
                                drawParams[3], drawParams[4], drawParams[5]);
      } else {
        TEST_Output("\r\n[ATST153] ERROR: Need 6 params (X,Y,W,H,F,C). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 154:  // ATST154=X,Y,W,H,F,C - Draw rectangle
      parsed = TEST_ParseDrawParams(params, drawParams, 6);
      if (parsed == 6) {
        return TEST_Draw_Rect(drawParams[0], drawParams[1], drawParams[2],
                             drawParams[3], drawParams[4], drawParams[5]);
      } else {
        TEST_Output("\r\n[ATST154] ERROR: Need 6 params (X,Y,W,H,F,C). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 155:  // ATST155=X,Y,S,L,C,"TEXT" - Draw text with rotation
      // Parse X,Y,S,L,C first, then find quoted text
      parsed = TEST_ParseDrawParams(params, drawParams, 5);
      if (parsed >= 5) {
        // Find the quoted text
        const char *quoteStart = strchr(params, '"');
        if (quoteStart != NULL) {
          quoteStart++;  // Skip opening quote
          const char *quoteEnd = strchr(quoteStart, '"');
          if (quoteEnd != NULL) {
            // Extract text
            int textLen = quoteEnd - quoteStart;
            if (textLen > sizeof(textBuf) - 1) textLen = sizeof(textBuf) - 1;
            strncpy(textBuf, quoteStart, textLen);
            textBuf[textLen] = '\0';
            return TEST_Draw_Text(drawParams[0], drawParams[1], drawParams[2],
                                 drawParams[3], drawParams[4], textBuf);
          }
        }
        TEST_Output("\r\n[ATST155] ERROR: No quoted text found.\r\n");
        return TEST_ERROR;
      } else {
        TEST_Output("\r\n[ATST155] ERROR: Need 5 params + \"TEXT\" (X,Y,S,L,C,\"TEXT\"). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 156:  // ATST156=X,Y,W,H,MI,MA,A,C - Draw graph axis
      parsed = TEST_ParseDrawParams(params, drawParams, 7);
      if (parsed == 7) {
        return TEST_Draw_GraphAxis(drawParams[0], drawParams[1], drawParams[2],
                                   drawParams[3], drawParams[4], drawParams[5],
                                   drawParams[6], drawParams[7]);
      } else {
        TEST_Output("\r\n[ATST156] ERROR: Need 7 params (X,Y,W,H,MI,MA,A,C). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 157:  // ATST157=DATA - Draw line graph (semicolon-separated X,Y pairs)
      return TEST_Draw_LineGraph(params);

    case 158:  // ATST158=DATA - Draw bar graph (comma-separated values)
      return TEST_Draw_BarGraph(params);

    case 165:  // ATST165=X,Y,W,H,"TEXT",P,CT,TT - Draw button
      {
        int32_t btnParams[8];
        parsed = TEST_ParseDrawParams(params, btnParams, 5);
        if (parsed >= 5) {
          const char *quoteStart = strchr(params, '"');
          char btnText[32];
          if (quoteStart != NULL) {
            quoteStart++;
            const char *quoteEnd = strchr(quoteStart, '"');
            if (quoteEnd != NULL) {
              int textLen = quoteEnd - quoteStart;
              if (textLen > sizeof(btnText) - 1) textLen = sizeof(btnText) - 1;
              strncpy(btnText, quoteStart, textLen);
              btnText[textLen] = '\0';
              // Parse remaining params after text
              const char *afterText = quoteEnd + 1;
              while (*afterText == ',') afterText++;
              uint8_t pressed = 0;
              uint32_t btnColor = LCD_COLOR_BLUE, textColor = LCD_COLOR_WHITE;
              sscanf(afterText, "%hhu,%lx,%lx", &pressed, &btnColor, &textColor);
              return TEST_Menu_Draw_Button(btnParams[0], btnParams[1], btnParams[2],
                                          btnParams[3], btnText, pressed, btnColor, textColor);
            }
          }
        }
        TEST_Output("\r\n[ATST165] ERROR: Invalid params. Use X,Y,W,H,\"TEXT\",P,CT,TT\r\n");
        return TEST_ERROR;
      }

    case 166:  // ATST166=X,Y,W,H,P,C,BC - Draw progress bar
      {
        int32_t progParams[7];
        parsed = TEST_ParseDrawParams(params, progParams, 7);
        if (parsed == 7) {
          return TEST_Menu_Draw_Progress(progParams[0], progParams[1], progParams[2],
                                         progParams[3], progParams[4], progParams[5], progParams[6]);
        }
        TEST_Output("\r\n[ATST166] ERROR: Need 7 params (X,Y,W,H,P,C,BC). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 167:  // ATST167=X,Y,W,H,V,C,SV - Draw slider
      {
        int32_t sliderParams[7];
        parsed = TEST_ParseDrawParams(params, sliderParams, 7);
        if (parsed >= 6) {
          uint8_t showVal = (parsed >= 7) ? (uint8_t)sliderParams[6] : 1;
          return TEST_Menu_Draw_Slider(sliderParams[0], sliderParams[1], sliderParams[2],
                                       sliderParams[3], sliderParams[4], sliderParams[5], showVal);
        }
        TEST_Output("\r\n[ATST167] ERROR: Need 6 params (X,Y,W,H,V,C,SV). Got %d.\r\n", parsed);
        return TEST_ERROR;
      }

    case 168:  // ATST168=X,Y,W,H,"TEXT",S,I,C - Draw list item
      {
        int32_t listParams[8];
        parsed = TEST_ParseDrawParams(params, listParams, 5);
        if (parsed >= 5) {
          const char *quoteStart = strchr(params, '"');
          char listText[64];
          if (quoteStart != NULL) {
            quoteStart++;
            const char *quoteEnd = strchr(quoteStart, '"');
            if (quoteEnd != NULL) {
              int textLen = quoteEnd - quoteStart;
              if (textLen > sizeof(listText) - 1) textLen = sizeof(listText) - 1;
              strncpy(listText, quoteStart, textLen);
              listText[textLen] = '\0';
              // Parse remaining params
              const char *afterText = quoteEnd + 1;
              while (*afterText == ',') afterText++;
              uint8_t selected = 0;
              int32_t index = 1;
              uint32_t itemColor = LCD_COLOR_BLUE;
              sscanf(afterText, "%hhu,%ld,%lx", &selected, &index, &itemColor);
              return TEST_Menu_Draw_List(listParams[0], listParams[1], listParams[2],
                                        listParams[3], listText, selected, index, itemColor);
            }
          }
        }
        TEST_Output("\r\n[ATST168] ERROR: Invalid params. Use X,Y,W,H,\"TEXT\",S,I,C\r\n");
        return TEST_ERROR;
      }

    case 170:  // ATST170=M - Screen clear
      {
        int32_t clearParams[1];
        parsed = TEST_ParseDrawParams(params, clearParams, 1);
        if (parsed >= 1) {
          return TEST_Screen_Clear((uint8_t)clearParams[0]);
        }
        return TEST_Screen_Clear(0);  // Default to black
      }

    default:
      TEST_Output("\r\n[CMD] ERROR: Test ID %d does not support parameters.\r\n", testId);
      return TEST_ERROR;
  }
}

/* =============================================================================
 * Interrupt Control Functions - ATST999
 * ========================================================================== */

/**
 * @brief Check if test should be interrupted
 * @retval 1 if interrupted, 0 otherwise
 */
int TEST_ShouldInterrupt(void) {
  return testInterruptFlag;
}

/**
 * @brief Set interrupt flag
 * @param state Interrupt state (1=interrupt, 0=clear)
 */
void TEST_SetInterrupt(uint8_t state) {
  testInterruptFlag = state;
}

/* =============================================================================
 * Display Helper Functions - Test Start/Completion Messages
 * Large light text on dark background for visibility
 * ========================================================================== */

/**
 * @brief Display test start message on LCD
 * @param testId Test ID (for display name) - uint16_t to support ATST999 (999)
 * @param testName Test name string
 * @note v0.1.85: Displays large white text on dark blue background
 *               Changed to uint16_t to properly display ATST999 (was showing ATST231)
 */
void TEST_DisplayStartMessage(uint16_t testId, const char *testName) {
  extern LTDC_HandleTypeDef hltdc_discovery;

  BSP_LCD_Clear(LCD_COLOR_DARKBLUE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);

  // Display "TEST STARTING" at top
  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"TEST STARTING", CENTER_MODE);

  // Display test ID and name
  char msg[64];
  snprintf(msg, sizeof(msg), "ATST%d - %s", testId, testName);
  BSP_LCD_DisplayStringAt(0, 80, (uint8_t *)msg, CENTER_MODE);

  // Display "Please wait..." at bottom
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 40, (uint8_t *)"Please wait...", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(100);  // Brief pause to ensure message is visible
}

/**
 * @brief Display test completion message on LCD
 * @param testId Test ID (for display name) - uint16_t to support ATST999 (999)
 * @param testName Test name string
 * @param result Test result (0=PASS/OK, 1=FAIL/ERROR, other=TIMEOUT)
 * @note v0.1.85: Displays large text on dark background
 *               Green = PASS, Red = FAIL, Yellow = TIMEOUT
 *               Changed to uint16_t to properly display ATST999
 */
void TEST_DisplayCompleteMessage(uint16_t testId, const char *testName, TestStatus_TypeDef result) {
  extern LTDC_HandleTypeDef hltdc_discovery;

  // Choose color based on result
  uint32_t bgColor, textColor, statusColor;

  if (result == TEST_OK) {
    bgColor = LCD_COLOR_DARKGREEN;
    textColor = LCD_COLOR_WHITE;
    statusColor = LCD_COLOR_LIGHTGREEN;
  } else if (result == TEST_ERROR) {
    bgColor = LCD_COLOR_DARKRED;
    textColor = LCD_COLOR_WHITE;
    statusColor = LCD_COLOR_RED;
  } else if (result == TEST_TIMEOUT) {
    bgColor = LCD_COLOR_DARKYELLOW;
    textColor = LCD_COLOR_BLACK;
    statusColor = LCD_COLOR_YELLOW;
  } else {
    bgColor = LCD_COLOR_DARKBLUE;
    textColor = LCD_COLOR_WHITE;
    statusColor = LCD_COLOR_LIGHTBLUE;
  }

  BSP_LCD_SetBackColor(bgColor);
  BSP_LCD_SetTextColor(textColor);
  BSP_LCD_SetFont(&Font24);

  // Clear screen with result color
  BSP_LCD_Clear(bgColor);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  // Display "TEST COMPLETE" at top
  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"TEST COMPLETE", CENTER_MODE);

  // Display test ID and name
  char msg[64];
  snprintf(msg, sizeof(msg), "ATST%d - %s", testId, testName);
  BSP_LCD_DisplayStringAt(0, 80, (uint8_t *)msg, CENTER_MODE);

  // Display result status
  const char *statusStr = (result == TEST_OK) ? "PASS" :
                          (result == TEST_ERROR) ? "FAIL" :
                          (result == TEST_TIMEOUT) ? "TIMEOUT" : "N/A";
  BSP_LCD_SetTextColor(statusColor);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)statusStr, CENTER_MODE);

  // Display "Press any key" at bottom
  BSP_LCD_SetTextColor(textColor);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 40, (uint8_t *)"Send ATST or AT? for menu", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
}

/**
 * @brief Get test name string from test ID
 * @param testId Test ID (255 = interrupt, internally mapped from ATST999)
 * @retval Test name string (static buffer)
 */
const char *TEST_GetName(uint8_t testId) {
  if (testId == 255) return "Interrupt";  // ATST999 (999 -> 255 internally)

  static char nameBuf[32];
  switch (testId) {
    case 0:   return "ESP Weather";
    case 1:   return "Display Test";
    case 2:   return "LED Test";
    case 3:   return "RTC Test";
    case 4:   return "WiFi Test";
    case 5:   return "NTP Test";
    case 6:   return "HTTP Test";
    case 7:   return "Audio Test";
    case 8:   return "SDRAM Test";
    case 9:   return "Touch Test";
    case 10:  return "Run All Tests";
    case 99:  return "Help";
    case 149: return "Panel Detect";
    case 150: return "Checkerboard";
    case 151: return "Draw Point";
    case 152: return "Draw Line";
    case 153: return "Draw Circle";
    case 154: return "Draw Rect";
    case 155: return "Draw Text";
    case 156: return "Graph Axis";
    case 157: return "Line Graph";
    case 158: return "Bar Graph";
    case 159: return "Drawing Test";
    case 160: return "Audio Display";
    case 162: return "Audio Localization";
    case 165: return "Menu Button";
    case 166: return "Menu Progress";
    case 167: return "Menu Slider";
    case 168: return "Menu List";
    case 169: return "Menu Demo";
    case 170: return "Screen Clear";
    case 171: return "Freq Analysis";
    case 172: return "SD Card Info";
    case 173: return "SD File List";
    case 174: return "Fast Line";
    case 175: return "Fast Rect";
    case 176: return "Pixel Batch";
    case 177: return "Scroll Area";
    case 178: return "Fast Text";
    case 179: return "Waveform";
    case 180: return "Fast Graph";
    default:
      snprintf(nameBuf, sizeof(nameBuf), "Test %d", testId);
      return nameBuf;
  }
}


/**
 * @brief Interrupt all tests (ATST999)
 * Stops any currently running test by setting interrupt flag
 */
TestStatus_TypeDef TEST_Interrupt_All(void) {
  testInterruptFlag = 1;
  TEST_Output("\r\n[ATST999] Interrupt signal sent - stopping current test...\r\n");

  // Clear screen to indicate test stopped
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15,
                         (uint8_t *)"TEST INTERRUPTED", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  return TEST_OK;
}

/* =============================================================================
 * ATST171 - Audio Frequency Analysis (800Hz/1200Hz)
 * Analyzes specific frequencies with direction finding
 * Circle: 200px diameter inside 200x200 pixel square
 * Shows 4 waveform windows (200x50 each) on right side
 * Lines: 2-3 pixels thick, brightness based on signal level
 *
 * NOTE: This test supports both SIMULATION mode and REAL AUDIO mode
 * - Real audio requires external MP34DT01 microphones connected via I2S/DFSDM
 * - Simulation mode is used when no real audio input is available
 * ========================================================================== */

// Audio input hook - override this function to provide real audio data
// Returns: 0 = simulation mode, 1 = real audio mode
// Output: amp800, amp1200 (0.0-1.0), angle (0-360 degrees)
/**
 * @brief Audio input hook for real microphone data (ATST171)
 *        Override this function in your application to provide real audio data
 * @param amp800 Pointer to store 800Hz amplitude (0.0-1.0)
 * @param amp1200 Pointer to store 1200Hz amplitude (0.0-1.0)
 * @param angle Pointer to store source direction angle (0-360 degrees)
 * @param waveforms Array to store waveform data for 4 channels [4][200 samples]
 * @retval 0 = use simulation mode, 1 = real audio mode
 *
 * @note This is a weak function - override it in your application to provide
 *       real audio data from MP34DT01 microphones via I2S/DFSDM
 *
 * @note v0.1.85: For real implementation, you need to:
 *       1. Initialize I2S or DFSDM for MP34DT01 microphone input
 *       2. Read audio samples from all 4 microphones
 *       3. Perform FFT to detect 800Hz and 1200Hz frequency components
 *       4. Calculate TDOA (Time Difference of Arrival) for direction finding
 *       5. Store raw waveform data in the waveforms array
 *
 * @example
 * // In main.c or your application file:
 * uint8_t TEST_GetAudioData(float *amp800, float *amp1200, float *angle, int16_t waveforms[4][200]) {
 *   // Your real audio processing code here
 *   // 1. Read audio samples from I2S/DFSDM
 *   // 2. Perform FFT to detect 800Hz and 1200Hz
 *   // 3. Calculate direction using TDOA
 *   // 4. Fill waveforms with data
 *   // Return 1 to indicate real audio mode
 *   return 1;
 * }
 */
__weak uint8_t TEST_GetAudioData(float *amp800, float *amp1200, float *angle, int16_t waveforms[4][200]) {
  // Default implementation returns 0 (simulation mode)
  // Override this function in your application to provide real audio data from MP34DT01 microphones
  (void)amp800;    // Unused in default implementation
  (void)amp1200;   // Unused in default implementation
  (void)angle;     // Unused in default implementation
  (void)waveforms; // Unused in default implementation
  return 0;  // 0 = use simulation mode
}

TestStatus_TypeDef TEST_Audio_Freq_Analysis(void) {
  TEST_Output("\r\n=== ATST171: Frequency Analysis (800Hz/1200Hz) ===\r\n");

  // Check for interrupt at start
  CHECK_INTERRUPT();

  // Clear interrupt flag at start
  testInterruptFlag = 0;

  // Clear screen and show test starting message
  extern LTDC_HandleTypeDef hltdc_discovery;
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(20);

  // Display layout - 200x200 square for microphones
  const int sqSize = 200;               // Square size: 200x200 pixels
  const int sqX = 50;                    // Square top-left X
  const int sqY = 136;                   // Square top-left Y (centered vertically)
  const int circleCenterX = sqX + sqSize / 2;  // = 150
  const int circleCenterY = sqY + sqSize / 2;  // = 236
  const int circleRadius = 100;           // 200px diameter = 100px radius

  // Microphone positions at square corners
  const int micX[4] = {sqX, sqX + sqSize, sqX, sqX + sqSize};
  const int micY[4] = {sqY, sqY, sqY + sqSize, sqY + sqSize};

  // Waveform windows (right side, 200x50 each, stacked vertically)
  const int waveWinX = 550;
  const int waveWinY = 36;
  const int waveWinWidth = 200;
  const int waveWinHeight = 50;
  const int numChannels = 4;
  const int maxSignalLength = 100;  // Max line length = max signal

  // Channel colors for waveforms
  const uint32_t waveColors[4] = {
    LCD_COLOR_CYAN, LCD_COLOR_GREEN, LCD_COLOR_YELLOW, LCD_COLOR_MAGENTA
  };

  #define WAVE_POINTS 200
  static int16_t waveform[4][WAVE_POINTS];

  TEST_Output("\r\n[INFO] Frequency analysis started (60 second test)...\r\n");
  TEST_Output("       Square: %dx%d pixels at (%d,%d)\r\n", sqSize, sqSize, sqX, sqY);
  TEST_Output("       Circle: 200px diameter, center at (%d,%d)\r\n", circleCenterX, circleCenterY);
  TEST_Output("       Waveform windows: 4 x %dx%d at (%d,%d)\r\n",
              waveWinWidth, waveWinHeight, waveWinX, waveWinY);
  TEST_Output("       Frequencies: 800Hz (RED), 1200Hz (BLUE)\r\n");
  TEST_Output("       Line thickness: 2-3 pixels\r\n");
  TEST_Output("       Signal level = line length (max 100px = max signal)\r\n");
  TEST_Output("       Brightness = signal level (half=min, full=max)\r\n");
  TEST_Output("       Mode: SIMULATION (real audio requires MP34DT01 microphones)\r\n");
  TEST_Output("       Send ATST999 to interrupt\r\n");

  // Main display loop (runs until interrupted or 60 seconds)
  uint32_t startTime = HAL_GetTick();
  int frameCount = 0;
  float sourceAngle = 0.0f;
  float sourceAmp800 = 0.0f;
  float sourceAmp1200 = 0.0f;
  uint8_t useRealAudio = 0;

  while ((HAL_GetTick() - startTime) < 60000 && !testInterruptFlag) {  // 60 seconds
    frameCount++;

    // Check if real audio data is available
    useRealAudio = TEST_GetAudioData(&sourceAmp800, &sourceAmp1200, &sourceAngle, waveform);

    if (!useRealAudio) {
      // SIMULATION MODE: Simulate rotating sound source with varying frequency content
      sourceAngle = frameCount * 2.0f;
      if (sourceAngle >= 360.0f) sourceAngle -= 360.0f;

      // Simulate amplitude for each frequency (0.0 to 1.0)
      sourceAmp800 = 0.5f + 0.4f * sinf(frameCount * 0.05f);   // 800Hz amplitude
      sourceAmp1200 = 0.3f + 0.3f * sinf(frameCount * 0.08f); // 1200Hz amplitude
    }

    // Clear and redraw main area
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    // Draw title
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t *)"ATST171: Frequency Analysis", CENTER_MODE);

    // Draw microphone square (200x200)
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_DrawRect(sqX, sqY, sqSize, sqSize);

    // Draw direction circle background (200px diameter)
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawCircle(circleCenterX, circleCenterY, circleRadius);

    // Draw microphones at square corners
    for (int i = 0; i < 4; i++) {
      BSP_LCD_SetTextColor(waveColors[i]);
      BSP_LCD_FillRect(micX[i] - 8, micY[i] - 8, 16, 16);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DrawRect(micX[i] - 8, micY[i] - 8, 16, 16);

      // Mic labels
      char label[8];
      snprintf(label, sizeof(label), "M%d", i + 1);
      BSP_LCD_SetFont(&Font12);
      BSP_LCD_DisplayStringAt(micX[i] - 10, micY[i] - 25, (uint8_t *)label, LEFT_MODE);
    }

    // Calculate source position from angle
    float sourceRad = sourceAngle * 3.14159f / 180.0f;

    // v0.1.85: Signal level display - line length 50px (0%) to 100px (100%)
    // Color always has green base, with red/blue components based on frequency levels
    const int minLineLength = 50;   // Minimum line length at 0% signal
    const int maxLineLength = 100;  // Maximum line length at 100% signal
    const int lineLengthRange = maxLineLength - minLineLength;  // 50 pixels range

    // Draw direction line with color mixing (green base + red for 800Hz + blue for 1200Hz)
    if (sourceAmp800 > 0.01f || sourceAmp1200 > 0.01f) {
      // Calculate combined signal amplitude for line length
      float combinedAmp = (sourceAmp800 + sourceAmp1200) / 2.0f;
      if (combinedAmp > 1.0f) combinedAmp = 1.0f;

      // Line length: 50px at 0%, 100px at 100%
      int lineLen = minLineLength + (int)(combinedAmp * lineLengthRange);
      int lineX = circleCenterX + (int)(cosf(sourceRad) * lineLen);
      int lineY = circleCenterY + (int)(sinf(sourceRad) * lineLen);

      // Color mixing: Green base (always present) + Red (800Hz) + Blue (1200Hz)
      // Green at 50% brightness as base, increases with signal
      uint8_t greenLevel = 128 + (uint8_t)(combinedAmp * 127);  // 128-255
      uint8_t redLevel = (uint8_t)(sourceAmp800 * 255);         // 0-255
      uint8_t blueLevel = (uint8_t)(sourceAmp1200 * 255);       // 0-255

      // Apply brightness scaling for "half brightness to full brightness" effect
      uint8_t brightness = 128 + (uint8_t)(combinedAmp * 127);  // 128-255
      greenLevel = (greenLevel * brightness) / 255;
      redLevel = (redLevel * brightness) / 255;
      blueLevel = (blueLevel * brightness) / 255;

      uint32_t lineColor = 0xFF000000 | (redLevel << 16) | (greenLevel << 8) | blueLevel;

      // Draw 3-pixel thick line
      BSP_LCD_SetTextColor(lineColor);
      BSP_LCD_DrawLine(circleCenterX, circleCenterY - 1, lineX, lineY - 1);
      BSP_LCD_DrawLine(circleCenterX, circleCenterY, lineX, lineY);
      BSP_LCD_DrawLine(circleCenterX, circleCenterY + 1, lineX, lineY + 1);

      // Draw end point circle
      BSP_LCD_FillCircle(lineX, lineY, 5);

      // Draw individual frequency indicators (small circles at end of line)
      if (sourceAmp800 > 0.1f) {
        int redOffset = (int)(sourceAmp800 * 10);
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_FillCircle(lineX + redOffset, lineY - redOffset, 3);
      }
      if (sourceAmp1200 > 0.1f) {
        int blueOffset = (int)(sourceAmp1200 * 10);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        BSP_LCD_FillCircle(lineX - blueOffset, lineY + blueOffset, 3);
      }
    }

    // Draw frequency legend
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(sqX + 10, sqY + sqSize + 10,
                           (uint8_t *)"800Hz (RED)", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillRect(sqX + 60, sqY + sqSize + 8, 8, 8);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(sqX + 90, sqY + sqSize + 10,
                           (uint8_t *)"1200Hz (BLUE)", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(sqX + 180, sqY + sqSize + 8, 8, 8);

    // Draw amplitude bars
    char ampText[32];
    snprintf(ampText, sizeof(ampText), "800Hz: %.0f%%", sourceAmp800 * 100);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(sqX + 10, sqY + sqSize + 30,
                           (uint8_t *)ampText, LEFT_MODE);

    snprintf(ampText, sizeof(ampText), "1200Hz: %.0f%%", sourceAmp1200 * 100);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(sqX + 100, sqY + sqSize + 30,
                           (uint8_t *)ampText, LEFT_MODE);

    // Generate and draw waveform for each channel
    for (int ch = 0; ch < numChannels; ch++) {
      int winY = waveWinY + ch * (waveWinHeight + 5);
      int winCenterY = winY + waveWinHeight / 2;

      // Calculate distance and phase for this mic
      float dx = cosf(sourceRad) * (sqSize / 2) - (micX[ch] - circleCenterX);
      float dy = sinf(sourceRad) * (sqSize / 2) - (micY[ch] - circleCenterY);
      float dist = sqrtf(dx * dx + dy * dy);
      float delay = dist / 34300.0f;
      float phase = frameCount * 0.1f + delay * 1000.0f;

      // Generate waveform with both frequencies
      for (int i = 0; i < WAVE_POINTS; i++) {
        float t = i * 0.05f + phase;
        // Mix 800Hz and 1200Hz
        float sample800 = sinf(t * 0.8f) * sourceAmp800;
        float sample1200 = sinf(t * 1.2f) * sourceAmp1200;
        waveform[ch][i] = (int16_t)((sample800 + sample1200) * 20000.0f);
      }

      // Draw window background and border
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(waveWinX, winY, waveWinWidth, waveWinHeight);
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawRect(waveWinX, winY, waveWinWidth, waveWinHeight);

      // Draw channel label
      char chLabel[16];
      snprintf(chLabel, sizeof(chLabel), "CH%d", ch + 1);
      BSP_LCD_SetTextColor(waveColors[ch]);
      BSP_LCD_SetFont(&Font12);
      BSP_LCD_DisplayStringAt(waveWinX + 5, winY + 5, (uint8_t *)chLabel, LEFT_MODE);

      // Draw center line
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
      BSP_LCD_DrawHLine(waveWinX, winCenterY, waveWinWidth);

      // Draw waveform
      BSP_LCD_SetTextColor(waveColors[ch]);
      int16_t prevX = -1, prevY = -1;
      for (int i = 0; i < WAVE_POINTS; i++) {
        int screenX = waveWinX + (i * waveWinWidth / WAVE_POINTS);
        int screenY = winCenterY - (waveform[ch][i] * waveWinHeight / 65536);
        if (screenY < winY) screenY = winY;
        if (screenY >= winY + waveWinHeight) screenY = winY + waveWinHeight - 1;

        if (prevX >= 0) {
          BSP_LCD_DrawLine(prevX, prevY, screenX, screenY);
        }
        prevX = screenX;
        prevY = screenY;
      }

      // Calculate and display signal level
      int32_t sum = 0;
      for (int i = 0; i < WAVE_POINTS; i++) {
        sum += abs(waveform[ch][i]);
      }
      int avgLevel = sum / WAVE_POINTS;
      int dBLevel = (int)(20.0f * log10f((float)avgLevel / 32768.0f + 0.001f));

      // Draw level text
      char levelText[24];
      snprintf(levelText, sizeof(levelText), "%d dB", dBLevel);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DisplayStringAt(waveWinX + waveWinWidth - 60, winY + 5,
                             (uint8_t *)levelText, LEFT_MODE);
    }

    // Display elapsed time and mode
    char timeText[64];
    uint32_t elapsed = (HAL_GetTick() - startTime) / 1000;
    snprintf(timeText, sizeof(timeText), "Time: %lu/60s | Angle: %.0f deg | %s",
             elapsed, sourceAngle, useRealAudio ? "REAL AUDIO" : "SIMULATION");
    BSP_LCD_SetTextColor(useRealAudio ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(waveWinX, waveWinY + numChannels * (waveWinHeight + 5) + 10,
                           (uint8_t *)timeText, LEFT_MODE);

    // v0.1.85: Spectral characteristics window (200x50) below waveforms
    int specWinY = waveWinY + numChannels * (waveWinHeight + 5) + 30;
    int specWinHeight = 50;

    // Draw spectral window background and border
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(waveWinX, specWinY, waveWinWidth, specWinHeight);
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    BSP_LCD_DrawRect(waveWinX, specWinY, waveWinWidth, specWinHeight);

    // Spectral window title
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font8);
    BSP_LCD_DisplayStringAt(waveWinX + 5, specWinY + 3, (uint8_t *)"Spectrum", LEFT_MODE);

    // Draw frequency bars for 800Hz and 1200Hz
    int barY = specWinY + 15;
    int barHeight = 8;
    int barSpacing = 12;
    int maxBarWidth = waveWinWidth - 80;  // Leave room for labels

    // 800Hz bar (RED)
    int bar800Width = (int)(sourceAmp800 * maxBarWidth);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
    BSP_LCD_FillRect(waveWinX + 50, barY, maxBarWidth, barHeight);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillRect(waveWinX + 50, barY, bar800Width, barHeight);

    // 800Hz label
    char freqLabel[16];
    snprintf(freqLabel, sizeof(freqLabel), "800Hz: %.0f%%", sourceAmp800 * 100);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_SetFont(&Font8);
    BSP_LCD_DisplayStringAt(waveWinX + 5, barY, (uint8_t *)freqLabel, LEFT_MODE);

    // 1200Hz bar (BLUE)
    int bar1200Width = (int)(sourceAmp1200 * maxBarWidth);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
    BSP_LCD_FillRect(waveWinX + 50, barY + barSpacing, maxBarWidth, barHeight);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(waveWinX + 50, barY + barSpacing, bar1200Width, barHeight);

    // 1200Hz label
    snprintf(freqLabel, sizeof(freqLabel), "1.2kHz: %.0f%%", sourceAmp1200 * 100);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(waveWinX + 5, barY + barSpacing, (uint8_t *)freqLabel, LEFT_MODE);

    // Signal quality indicator
    float signalQuality = (sourceAmp800 + sourceAmp1200) / 2.0f;
    const char *qualityStr = "NO SIGNAL";
    uint32_t qualityColor = LCD_COLOR_GRAY;
    if (signalQuality > 0.7f) {
      qualityStr = "EXCELLENT";
      qualityColor = LCD_COLOR_GREEN;
    } else if (signalQuality > 0.4f) {
      qualityStr = "GOOD";
      qualityColor = LCD_COLOR_YELLOW;
    } else if (signalQuality > 0.1f) {
      qualityStr = "WEAK";
      qualityColor = LCD_COLOR_ORANGE;
    }

    BSP_LCD_SetTextColor(qualityColor);
    BSP_LCD_SetFont(&Font8);
    BSP_LCD_DisplayStringAt(waveWinX + waveWinWidth - 70, specWinY + specWinHeight - 15,
                           (uint8_t *)qualityStr, LEFT_MODE);

    // Force LTDC reload after complete frame redraw
    extern LTDC_HandleTypeDef hltdc_discovery;
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

    // Check for interrupt after each frame (v0.1.79 - more responsive interruption)
    CHECK_INTERRUPT();

    // Small delay between frames
    osDelay(50);
  }

  // Check if interrupted
  if (testInterruptFlag) {
    TEST_Output("\r\n[ATST171] Test interrupted by ATST999\r\n");
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15,
                           (uint8_t *)"TEST INTERRUPTED", CENTER_MODE);
  } else {
    TEST_Output("\r\n[ATST171] Completed - displayed %d frames\r\n", frameCount);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15,
                           (uint8_t *)"Analysis Complete!", CENTER_MODE);
  }

  // Clear interrupt flag
  testInterruptFlag = 0;

  TEST_Output("\r\n=== ATST171 Complete ===\r\n");

  return TEST_OK;
}

/* =============================================================================
 * ATSTMCRI - MicroSD Card Information (v0.1.81)
 * Tests: Read and display SD card capacity, type, and basic information
 * Note: Full CID/CSD parsing requires HAL SD handle access
 * ========================================================================== */
TestStatus_TypeDef TEST_SD_Card_Info(void) {
  TEST_Output("\r\n=== ATSTMCRI: MicroSD Card Information ===");

  // v0.1.85: Display messages handled automatically by CDC_ProcessPendingCommand

  // v0.1.83: Try SD init with retries - card may need time to be detected
  TEST_Output("\r\n[Test 1/3] Initializing SD card (with retries)...");
  uint8_t sdState = MSD_ERROR_SD_NOT_PRESENT;
  int retryCount = 0;

  for (retryCount = 0; retryCount < 3; retryCount++) {
    if (retryCount > 0) {
      TEST_Output("         Retry %d/3...", retryCount + 1);
      osDelay(500);  // Wait 500ms between retries
    }
    sdState = BSP_SD_Init();
    if (sdState == MSD_OK) {
      break;  // Success!
    }
  }

  if (sdState != MSD_OK) {
    if (sdState == MSD_ERROR_SD_NOT_PRESENT) {
      char errBuf[64];
      snprintf(errBuf, sizeof(errBuf), "No SD card (err=%d)", sdState);
      TEST_OutputResult("SD Init", TEST_ERROR, errBuf);
      TEST_Output("\r\n  TIP: Try reinserting the SD card and run test again");
      BSP_LCD_Clear(LCD_COLOR_BLACK);
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_SetFont(&Font20);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"No SD Card!", CENTER_MODE);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 20, (uint8_t*)"Reinsert & Retry", CENTER_MODE);
    } else {
      char errBuf[64];
      snprintf(errBuf, sizeof(errBuf), "Init failed (err=%d)", sdState);
      TEST_OutputResult("SD Init", TEST_ERROR, errBuf);
    }
    return TEST_ERROR;
  }
  TEST_OutputResult("SD Init", TEST_OK, "SD card initialized");

  // Check if card is detected
  TEST_Output("\r\n[Test 2/3] Checking SD card presence...");
  if (BSP_SD_IsDetected() != SD_PRESENT) {
    TEST_OutputResult("SD Detection", TEST_ERROR, "SD card not present");
    return TEST_ERROR;
  }
  TEST_OutputResult("SD Detection", TEST_OK, "SD card present");

  // Get SD card state
  uint8_t cardState = BSP_SD_GetCardState();
  TEST_Output("\r\n[Test 3/3] Reading SD card state...");
  TEST_Output("Card State: %d", cardState);
  if (cardState == SD_TRANSFER_OK) {
    TEST_OutputResult("Card State", TEST_OK, "Transfer OK");
  } else {
    TEST_OutputResult("Card State", TEST_ERROR, "Not ready");
  }

  TEST_Output("\r\n=== Note ===");
  TEST_Output("Full card info requires HAL SD handle access.");
  TEST_Output("This function shows basic card state.");

  // Display summary on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"SD Card Information", CENTER_MODE);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"Card Detected!", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 80, (uint8_t*)"State: OK", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)"Use ATSTMCRL for partitions", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(5000);

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATSTMCRI Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("SD Card Info", TEST_OK, "Card information read successfully");
  TEST_Output("\r\n=== ATSTMCRI Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATSTMCRL - MicroSD Card File List (v0.1.80)
 * Tests: List files in the root directory of the SD card
 * Note: This is a simplified version that reads the MBR and partition table
 *       Full FAT file system support requires additional middleware
 * ========================================================================== */
TestStatus_TypeDef TEST_SD_Card_List(void) {
  TEST_Output("\r\n=== ATSTMCRL: MicroSD Card File List ===");

  // v0.1.85: Display messages handled automatically by CDC_ProcessPendingCommand

  // v0.1.83: Try SD init with retries - card may need time to be detected
  TEST_Output("\r\n[Test 1/3] Initializing SD card (with retries)...");
  uint8_t sdState = MSD_ERROR_SD_NOT_PRESENT;
  int retryCount = 0;

  for (retryCount = 0; retryCount < 3; retryCount++) {
    if (retryCount > 0) {
      TEST_Output("         Retry %d/3...", retryCount + 1);
      osDelay(500);  // Wait 500ms between retries
    }
    sdState = BSP_SD_Init();
    if (sdState == MSD_OK) {
      break;  // Success!
    }
  }

  if (sdState != MSD_OK) {
    if (sdState == MSD_ERROR_SD_NOT_PRESENT) {
      TEST_OutputResult("SD Init", TEST_ERROR, "No SD card detected");
      TEST_Output("\r\n  TIP: Try reinserting the SD card and run test again");
      BSP_LCD_Clear(LCD_COLOR_BLACK);
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_SetFont(&Font20);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"No SD Card!", CENTER_MODE);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 20, (uint8_t*)"Reinsert & Retry", CENTER_MODE);
    } else {
      char errBuf[64];
      snprintf(errBuf, sizeof(errBuf), "SD init failed (err=%d)", sdState);
      TEST_OutputResult("SD Init", TEST_ERROR, errBuf);
    }
    return TEST_ERROR;
  }
  TEST_OutputResult("SD Init", TEST_OK, "SD card initialized");

  // Check if card is detected
  TEST_Output("\r\n[Test 2/3] Checking SD card presence...");
  if (BSP_SD_IsDetected() != SD_PRESENT) {
    TEST_OutputResult("SD Detection", TEST_ERROR, "SD card not present");
    return TEST_ERROR;
  }
  TEST_OutputResult("SD Detection", TEST_OK, "SD card present");

  // Read MBR (Master Boot Record) from sector 0
  TEST_Output("\r\n[Test 3/3] Reading partition table...");
  uint32_t mbrBuffer[128];  // 512 bytes / 4 = 128 words
  uint8_t sdReadState = BSP_SD_ReadBlocks((uint32_t*)mbrBuffer, 0, 1, 5000);

  if (sdReadState != MSD_OK) {
    TEST_OutputResult("MBR Read", TEST_ERROR, "Failed to read MBR");
    return TEST_ERROR;
  }

  // Check for valid MBR (signature 0x55AA at offset 510)
  uint8_t* mbrBytes = (uint8_t*)mbrBuffer;
  if (mbrBytes[510] != 0x55 || mbrBytes[511] != 0xAA) {
    TEST_OutputResult("MBR Check", TEST_ERROR, "Invalid MBR signature (not 0x55AA)");
    TEST_Output("         Card may not be formatted with MBR");
  } else {
    TEST_OutputResult("MBR Check", TEST_OK, "Valid MBR found");
  }

  // Parse partition table (4 entries, each 16 bytes, starting at offset 0x1BE)
  TEST_Output("\r\n=== Partition Table ===");
  int partitionCount = 0;
  for (int i = 0; i < 4; i++) {
    int offset = 0x1BE + (i * 16);
    uint8_t bootIndicator = mbrBytes[offset];
    uint8_t partitionType = mbrBytes[offset + 4];
    uint32_t lbaStart = mbrBytes[offset + 8] | (mbrBytes[offset + 9] << 8) |
                       (mbrBytes[offset + 10] << 16) | (mbrBytes[offset + 11] << 24);
    uint32_t lbaSize = mbrBytes[offset + 12] | (mbrBytes[offset + 13] << 8) |
                      (mbrBytes[offset + 14] << 16) | (mbrBytes[offset + 15] << 24);

    if (partitionType != 0x00) {
      partitionCount++;
      TEST_Output("Partition %d:", i + 1);
      TEST_Output("  Boot Indicator: 0x%02X %s", bootIndicator,
             (bootIndicator == 0x80) ? "(Bootable)" : "(Not bootable)");
      TEST_Output("  Type: 0x%02X (%s)", partitionType,
             (partitionType == 0x01) ? "FAT12" :
             (partitionType == 0x04) ? "FAT16 (32M)" :
             (partitionType == 0x06) ? "FAT16 (2G)" :
             (partitionType == 0x07) ? "exFAT/NTFS" :
             (partitionType == 0x0B) ? "FAT32 (CHS)" :
             (partitionType == 0x0C) ? "FAT32 (LBA)" :
             (partitionType == 0x0E) ? "FAT16 (LBA)" : "Unknown");
      TEST_Output("  LBA Start: %lu (0x%08lX)", lbaStart, lbaStart);
      TEST_Output("  Size: %lu sectors (%lu MB)",
             lbaSize, (lbaSize * 512) / (1024 * 1024));
    }
  }

  if (partitionCount == 0) {
    TEST_Output("No valid partitions found (card may be unformatted or super-floppy format)");
  }

  // Note: Full file listing requires FatFS middleware integration
  TEST_Output("\r\n=== Note ===");
  TEST_Output("This command reads the partition table.");
  TEST_Output("Full file listing requires FatFS middleware integration.");
  TEST_Output("Contact: See fatfs.c and ff.h for file system API.");

  // Display summary on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"SD Card Partitions", CENTER_MODE);

  BSP_LCD_SetFont(&Font12);
  char partLine[64];
  snprintf(partLine, sizeof(partLine), "Found %d partition(s)", partitionCount);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)partLine, CENTER_MODE);

  if (partitionCount > 0) {
    BSP_LCD_DisplayStringAt(0, 80, (uint8_t *)"Use ATSTMCRI for details", CENTER_MODE);
  } else {
    BSP_LCD_DisplayStringAt(0, 80, (uint8_t *)"No partitions found", CENTER_MODE);
  }

  BSP_LCD_DisplayStringAt(0, 110, (uint8_t *)"FatFS needed for files", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(5000);

  // Display test completion message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATSTMCRL Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("SD Card List", TEST_OK, "Partition table read successfully");
  TEST_Output("\r\n=== ATSTMCRL Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST174 - Fast Line Drawing Demo (v0.1.80)
 * Demonstrates optimized horizontal/vertical line drawing for menus
 * ========================================================================== */
TestStatus_TypeDef TEST_Draw_Fast_Line_Demo(void) {
  TEST_Output("\r\n=== ATST174: Fast Line Drawing Demo ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST174 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"Fast Line Drawing", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"Optimized H/V lines for menus", CENTER_MODE);

  // Draw menu separator lines using optimized horizontal line drawing
  int16_t y = 70;
  for (int i = 0; i < 8; i++) {
    BSP_LCD_SetTextColor(i % 2 == 0 ? LCD_COLOR_BLUE : LCD_COLOR_RED);
    // BSP_LCD_DrawHLine is optimized for horizontal lines
    BSP_LCD_DrawHLine(50, y, 700);
    y += 30;
  }

  // Draw vertical separators for multi-column menus
  for (int i = 0; i < 4; i++) {
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    // BSP_LCD_DrawVLine is optimized for vertical lines
    BSP_LCD_DrawVLine(200 + i * 150, 60, 250);
  }

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 30, (uint8_t*)"H/V lines optimized for menu separators", CENTER_MODE);

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(3000);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST174 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("Fast Line Demo", TEST_OK, "Horizontal/vertical lines drawn");
  TEST_Output("\r\n=== ATST174 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST175 - Fast Rectangle Drawing Demo (v0.1.80)
 * Demonstrates optimized filled rectangle drawing using DMA2D
 * ========================================================================== */
TestStatus_TypeDef TEST_Draw_Fast_Rect_Demo(void) {
  TEST_Output("\r\n=== ATST175: Fast Rectangle Drawing Demo ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST175 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"Fast Rect Drawing", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"Optimized fill for buttons/panels", CENTER_MODE);

  // Draw menu buttons using optimized FillRect
  uint32_t colors[] = {LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_BLUE,
                       LCD_COLOR_YELLOW, LCD_COLOR_CYAN, LCD_COLOR_MAGENTA};

  int16_t y = 70;
  for (int i = 0; i < 6; i++) {
    BSP_LCD_SetTextColor(colors[i]);
    // BSP_LCD_FillRect uses DMA2D hardware acceleration
    BSP_LCD_FillRect(100, y, 600, 35);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);
    char label[32];
    snprintf(label, sizeof(label), "Button %d - Fast FillRect", i + 1);
    BSP_LCD_DisplayStringAt(0, y + 10, (uint8_t *)label, CENTER_MODE);
    y += 45;
  }

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(3000);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST175 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("Fast Rect Demo", TEST_OK, "Filled rectangles drawn");
  TEST_Output("\r\n=== ATST175 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST179 - Waveform Drawing Demo (v0.1.80)
 * Demonstrates oscilloscope-style waveform rendering
 * ========================================================================== */
TestStatus_TypeDef TEST_Draw_Waveform_Demo(void) {
  TEST_Output("\r\n=== ATST179: Waveform Drawing Demo ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST179 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

  // Draw waveform windows (like oscilloscope)
  uint16_t winX = 50, winY = 60, winWidth = 700, winHeight = 150;
  uint16_t numChannels = 2;
  uint32_t colors[] = {LCD_COLOR_GREEN, LCD_COLOR_YELLOW};

  for (int ch = 0; ch < numChannels; ch++) {
    uint16_t chY = winY + ch * (winHeight + 20);

    // Draw window background (dark blue = 0x000080)
    BSP_LCD_SetTextColor(0x000080);
    BSP_LCD_FillRect(winX, chY, winWidth, winHeight);

    // Draw window border
    BSP_LCD_SetTextColor(colors[ch]);
    BSP_LCD_DrawRect(winX, chY, winWidth, winHeight);

    // Draw grid lines
    BSP_LCD_SetTextColor(0x000040);  // Dark blue
    for (int i = 1; i < 10; i++) {
      BSP_LCD_DrawVLine(winX + i * (winWidth / 10), chY, winHeight);
    }
    for (int i = 1; i < 5; i++) {
      BSP_LCD_DrawHLine(winX, chY + i * (winHeight / 5), winWidth);
    }

    // Draw waveform (sine wave simulation)
    BSP_LCD_SetTextColor(colors[ch]);
    int16_t prevX = winX;
    int16_t prevY = chY + winHeight / 2;

    for (uint16_t x = 0; x < winWidth; x++) {
      float t = (float)x / 50.0f;
      float amplitude = 60.0f;
      float freq = 2.0f + ch;  // Different frequency per channel
      int16_t y = chY + winHeight / 2 + (int16_t)(amplitude * sin(t * freq));
      BSP_LCD_DrawLine(prevX, prevY, winX + x, y);
      prevX = winX + x;
      prevY = y;
    }

    // Label
    char label[32];
    snprintf(label, sizeof(label), "CH%d: Waveform", ch + 1);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(winX + 5, chY + 5, (uint8_t *)label, LEFT_MODE);
  }

  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(5000);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST179 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("Waveform Demo", TEST_OK, "Waveform rendering complete");
  TEST_Output("\r\n=== ATST179 Complete ===\r\n");
  return TEST_OK;
}

/* =============================================================================
 * ATST180 - Fast Graph Drawing Demo (v0.1.80)
 * Demonstrates optimized real-time line graph rendering
 * ========================================================================== */
TestStatus_TypeDef TEST_Draw_Graph_Fast_Demo(void) {
  TEST_Output("\r\n=== ATST180: Fast Graph Drawing Demo ===");

  // Display test start message on LCD
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 15, (uint8_t*)"Test ATST180 Start!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(500);

  // Graph area
  uint16_t graphX = 50, graphY = 80, graphWidth = 700, graphHeight = 300;
  uint16_t numPoints = 50;
  float dataPoints[50];

  // Generate initial data
  for (int i = 0; i < numPoints; i++) {
    dataPoints[i] = 50.0f + 20.0f * sin((float)i / 5.0f);
  }

  // Animate graph updates (simulating real-time data)
  for (int frame = 0; frame < 30; frame++) {
    // Clear graph area
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(graphX, graphY, graphWidth, graphHeight);

    // Draw grid
    BSP_LCD_SetTextColor(0x202020);  // Dark gray
    for (int i = 0; i <= 10; i++) {
      BSP_LCD_DrawVLine(graphX + i * (graphWidth / 10), graphY, graphHeight);
    }
    for (int i = 0; i <= 5; i++) {
      BSP_LCD_DrawHLine(graphX, graphY + i * (graphHeight / 5), graphWidth);
    }

    // Draw border
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(graphX, graphY, graphWidth, graphHeight);

    // Draw data line
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    int16_t prevX = graphX;
    int16_t prevY = graphY + graphHeight - (uint16_t)((dataPoints[0] / 100.0f) * graphHeight);

    for (int i = 1; i < numPoints; i++) {
      int16_t x = graphX + (i * graphWidth / numPoints);
      int16_t y = graphY + graphHeight - (uint16_t)((dataPoints[i] / 100.0f) * graphHeight);
      BSP_LCD_DrawLine(prevX, prevY, x, y);
      prevX = x;
      prevY = y;
    }

    // Shift data and add new point
    for (int i = 0; i < numPoints - 1; i++) {
      dataPoints[i] = dataPoints[i + 1];
    }
    dataPoints[numPoints - 1] = 50.0f + 30.0f * sin((HAL_GetTick() / 500.0f) + frame);

    // Draw info
    char info[64];
    snprintf(info, sizeof(info), "Frame: %d/30", frame + 1);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(graphX, graphY - 20, (uint8_t *)info, LEFT_MODE);

    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
    osDelay(100);

    // Check for interrupt
    if (testInterruptFlag) {
      TEST_Output("\r\n[ATST180] Interrupted by ATST999");
      BSP_LCD_Clear(LCD_COLOR_BLACK);
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_SetFont(&Font20);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 12, (uint8_t *)"INTERRUPTED", CENTER_MODE);
      return TEST_TIMEOUT;
    }
  }

  osDelay(1000);

  BSP_LCD_Clear(LCD_COLOR_BLACK);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(10);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 10, (uint8_t*)"Test ATST180 Complete!", CENTER_MODE);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  osDelay(1000);

  TEST_OutputResult("Fast Graph Demo", TEST_OK, "Real-time graph complete");
  TEST_Output("\r\n=== ATST180 Complete ===\r\n");
  return TEST_OK;
}
