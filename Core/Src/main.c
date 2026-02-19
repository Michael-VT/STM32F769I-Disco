/* USER CODE BEGIN Header */
/** Cooperate V0.0.1
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma2d.h"
#include "dsihost.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "usbd_cdc_if.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"
#include <stdio.h>
#include <string.h>
// TouchGFX - use stub declarations (implementations in app_touchgfx.c)
extern void MX_TouchGFX_Init(void);
extern void MX_TouchGFX_PreOSInit(void);
extern void MX_TouchGFX_Process(void);
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

/* USER CODE BEGIN PV */
#define UART_RX_BUFFER_SIZE 2048
volatile uint8_t UartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint32_t UartRxHead = 0;
volatile uint32_t UartRxTail = 0;
uint8_t UartRxByte;
UART_HandleTypeDef huart5;
// hrtc is defined in rtc.c
extern RTC_HandleTypeDef hrtc;
// hltdc_discovery is defined in stm32f769i_discovery_lcd.c
extern LTDC_HandleTypeDef hltdc_discovery;
// Mutex to protect UART5 transmission (shared between USB CDC and ESP8266 driver)
osMutexId_t uart5MutexId;
// Mutex to protect USB CDC transmission (shared between multiple tasks)
osMutexId_t cdcTransmitMutexId;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void MX_UART5_Init(void);
uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);
HAL_StatusTypeDef UART5_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Caches enabled for USB OTG HS ULPI PHY to work properly
  // SCB_DisableICache();
  // SCB_DisableDCache();
//  BSP_LCD_Init();
//  BSP_LCD_Clear(LCD_COLOR_WHITE);
//  BSP_LCD_DisplayStringAtLine(5, (uint8_t *)"Hello STM32F769I-DISCO");
//  MPU_Config();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // LTDC and DSI will be initialized by BSP_LCD_Init() - DO NOT call here
  // MX_LTDC_Init();
  MX_DMA2D_Init();  // DMA2D is used by BSP LCD for graphics operations
  MX_FMC_Init();  // FMC for SDRAM - needed before LCD
  // MX_DSIHOST_DSI_Init(); // NOT USED - BSP_LCD_InitEx() handles DSI initialization
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  // TouchGFX disabled - needs framework library
  // MX_TouchGFX_Init();
  // /* Call PreOsInit function */
  // MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */
  // uint32_t ts_status = TS_OK;
  // uint8_t lcd_status = LCD_OK;
  // BSP_LCD_Init();
  // ts_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  // while (ts_status != LCD_OK)
  //   ;
  // while (lcd_status != LCD_OK)
  //   ;

  // BSP_LCD_SetBrightness(100);
  // BSP_LCD_Init();
  // BSP_LCD_LayerDefaultInit(0, 0xC0000000);
  // BSP_LCD_SelectLayer(0);
  // BSP_LCD_Clear(0xFFFFFFFF);
  // BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  // BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"Disply STM32F769I-DISKO!",
  //                         CENTER_MODE);

  // BSP_SDRAM_Init();
  // BSP_LCD_Init();
  // BSP_LCD_LayerDefaultInit(0, SDRAM_DEVICE_ADDR);
  // BSP_LCD_SelectLayer(0);

  // NOTE: htim1 usage was weird in original code, commented out to avoid crash
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // ========================================
  // LCD PANEL DETECTION AND DIAGNOSTIC
  // ========================================
  // The STM32F769I-DISCO board has been manufactured with TWO different LCD panels:
  // 1. OTM8009A (KoD) - Original panels: HSYNC=2, HBP=34, HFP=34, 800x472 resolution
  // 2. NT35510 (TechShine) - Newer panels: HSYNC=120, HBP=150, HFP=150, 800x480 resolution
  // These require VERY DIFFERENT timing parameters!

  // Read panel IDs to detect which panel is installed
  uint16_t otm_id = OTM8009A_ReadID();
  uint16_t nt35510_id = NT35510_ReadID();

  // Determine panel type from ID responses
  int is_otm8009a = (otm_id == 0x40);  // OTM8009A_ID
  int is_nt35510 = (nt35510_id == 0x80);  // NT35510_ID

  const char *panel_type_str = "UNKNOWN";
  if (is_nt35510) {
    panel_type_str = "NT35510 (TechShine)";
  } else if (is_otm8009a) {
    panel_type_str = "OTM8009A (KoD)";
  } else {
    panel_type_str = "Project EQ 2026\/01\/19 10:30 v0.1.85";
  }

  char diag_buf[512];
  snprintf(diag_buf, sizeof(diag_buf),
           "\r\n========================================\r\n"
           "LCD PANEL DETECTION\r\n"
           "========================================\r\n"
           "OTM8009A ID: 0x%02X (expected: 0x40)\r\n"
           "NT35510 ID: 0x%02X (expected: 0x80)\r\n"
           "Detected Panel: %s\r\n"
           "========================================\r\n",
           otm_id,
           nt35510_id,
           panel_type_str
  );
  CDC_Transmit_HS((uint8_t *)diag_buf, strlen(diag_buf));
  HAL_Delay(500);

  // Initialize LCD with auto-detected panel
  BSP_LCD_Init();
  HAL_Delay(200);

  // Initialize only Layer 0, disable Layer 1 to prevent duplicate image
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_SetTransparency(0, 255);

  // NOTE: As of v0.1.67, BSP_LCD_LayerDefaultInit() properly fixes both:
  // 1. LTDC window positions (prevents HAL from adding AHBP offset)
  // 2. CFBLR pitch (prevents HAL from adding +3 to pitch)
  // No additional fixes needed here - doing so would overwrite the correct values!

  extern LTDC_HandleTypeDef hltdc_discovery;
  uint32_t lcd_height = BSP_LCD_GetYSize();  // 472 for OTM8009A, 480 for NT35510

  // CRITICAL FIX v0.1.45: Ensure LTDC TWCR register is correctly set
  // The TWCR register should contain TotalWidth in upper bits and TotalHeight in lower bits
  // Sometimes the LTDC timing is correct in Init struct but not written to hardware
  uint32_t expected_twcr = (hltdc_discovery.Init.TotalWidth << 16U) | hltdc_discovery.Init.TotalHeigh;
  uint32_t actual_twcr = LTDC->TWCR;
  snprintf(diag_buf, sizeof(diag_buf),
           "LTDC TWCR: Expected=0x%08lX, Actual=0x%08lX (TotalW=%lu, TotalH=%lu)\r\n",
           expected_twcr, actual_twcr, hltdc_discovery.Init.TotalWidth, hltdc_discovery.Init.TotalHeigh);
  CDC_Transmit_HS((uint8_t *)diag_buf, strlen(diag_buf));
  if (actual_twcr != expected_twcr) {
    snprintf(diag_buf, sizeof(diag_buf), "!!! TWCR MISMATCH - Fixing !!!\r\n");
    CDC_Transmit_HS((uint8_t *)diag_buf, strlen(diag_buf));
    LTDC->TWCR = expected_twcr;
    __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
  }

  // Output panel timing info
  snprintf(diag_buf, sizeof(diag_buf),
           "LCD Size: %lux%lu\r\n"
           "Panel Type: %s\r\n",
           BSP_LCD_GetXSize(), lcd_height,
           panel_type_str
  );
  CDC_Transmit_HS((uint8_t *)diag_buf, strlen(diag_buf));

  // Layer 1 is now properly disabled in BSP_LCD_Init (stm32f769i_discovery_lcd.c)

  HAL_Delay(100);

  BSP_LCD_DisplayOn();
  HAL_Delay(200);

  // Static test image - colored bars to verify LCD works
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

  // Draw title bar
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 50);

  // Draw panel type in center - CRITICAL for debugging
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 36, (uint8_t *)"STM32F769I-DISCO", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t *)panel_type_str, CENTER_MODE);

  // Draw resolution at bottom
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  char res_buf[32];
  snprintf(res_buf, sizeof(res_buf), "%lux%lu", BSP_LCD_GetXSize(), lcd_height);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 30, (uint8_t *)res_buf, CENTER_MODE);

  HAL_Delay(1000);

  // Initialize UART5 for ESP8266 communication
  // Note: RXNE interrupt will be enabled in DefaultTask after FreeRTOS starts
  // to ensure mutexes are initialized and scheduler is running
  MX_UART5_Init();

  // BSP_LCD_Clear(LCD_COLOR_WHITE);
  // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  // BSP_LCD_DisplayStringAt(0, 240, (uint8_t *)"STM32F769I-DISCO DSI WORKS!",
  //                         CENTER_MODE);

  // DEBUG: Read LTDC Layer 0 configuration registers
  extern LTDC_HandleTypeDef hltdc_discovery;
  uint32_t cfblr = LTDC_LAYER(&hltdc_discovery, 0)->CFBLR;
  uint32_t whpcr = LTDC_LAYER(&hltdc_discovery, 0)->WHPCR;
  uint32_t wvpcr = LTDC_LAYER(&hltdc_discovery, 0)->WVPCR;
  uint32_t cr = LTDC_LAYER(&hltdc_discovery, 0)->CR;

  // Also read LTDC timing registers
  uint32_t bpcr = hltdc_discovery.Instance->BPCR;
  uint32_t awcr = hltdc_discovery.Instance->AWCR;
  uint32_t twcr = hltdc_discovery.Instance->TWCR;
  uint32_t gcr = hltdc_discovery.Instance->GCR;

  char debug_buf[512];
  snprintf(debug_buf, sizeof(debug_buf),
           "\r\n=== LTDC TIMING REGISTERS ===\r\n"
           "BPCR: 0x%04lx (AHBP=%lu, AVBP=%lu)\r\n"
           "AWCR: 0x%04lx (AAW=%lu, AAH=%lu)\r\n"
           "TWCR: 0x%04lx (TOTALW=%lu, TOTALH=%lu)\r\n"
           "GCR: 0x%04lx (HSPOL=%lu, VSPOL=%lu, DEPOL=%lu, PCPOL=%lu)\r\n"
           "\r\n=== LTDC LAYER 0 ===\r\n"
           "CFBLR: 0x%08lx (LineLen=%lu, Pitch=%lu)\r\n"
           "WHPCR: 0x%08lx (WindowX0=%lu, WindowX1=%lu)\r\n"
           "WVPCR: 0x%08lx (WindowY0=%lu, WindowY1=%lu)\r\n"
           "CR: 0x%08lx (LEN=%lu)\r\n"
           "\r\n=== PANEL TYPE INFO ===\r\n"
           "OTM8009A timing: HSYNC=2, HBP=34, HFP=34, VSYNC=1, VBP=15, VFP=16\r\n"
           "NT35510 timing:  HSYNC=120, HBP=150, HFP=150, VSYNC=2, VBP=34, VFP=34\r\n"
           "Detected: %s\r\n",
           bpcr, (bpcr & 0x7FF), ((bpcr >> 16) & 0x7FF),
           awcr, (awcr & 0xFFF), ((awcr >> 16) & 0xFFF),
           twcr, (twcr & 0xFFF), ((twcr >> 16) & 0xFFF),
           gcr, (gcr & 1), ((gcr >> 1) & 1), ((gcr >> 2) & 1), ((gcr >> 3) & 1),
           cfblr,
           ((cfblr >> 16) & 0x1FFF),              // CFBLL (line length)
           (cfblr & 0x1FFF),                      // CFBP (pitch)
           whpcr,
           (whpcr & 0x1FFF),              // WHSTPOS (window X0)
           ((whpcr >> 16) & 0x1FFF),      // WHSPPOS (window X1)
           wvpcr,
           (wvpcr & 0x1FFF),              // WVSTPOS (window Y0)
           ((wvpcr >> 16) & 0x1FFF),      // WVSPPOS (window Y1)
           cr,
           (uint32_t)((cr & LTDC_LxCR_LEN) ? 1 : 0),   // LEN (layer enable)
           panel_type_str
  );
  CDC_Transmit_HS((uint8_t *)debug_buf, strlen(debug_buf));
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

// UART_Read function for ESP8266 driver - reads from ring buffer
uint32_t UART_Read(uint8_t *pBuf, uint32_t Len) {
  uint32_t count = 0;

  // First, try to read from ring buffer (filled by interrupt)
  while (count < Len && UartRxTail != UartRxHead) {
    pBuf[count++] = UartRxBuffer[UartRxTail];
    UartRxTail = (UartRxTail + 1) % UART_RX_BUFFER_SIZE;
  }

  // POLLING FALLBACK: If ring buffer is empty but RXNE flag is set,
  // read directly from RDR register (helps diagnose interrupt issues)
  if (count == 0 && __HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
    // Data is available but interrupt didn't catch it - read directly
    pBuf[count++] = (uint8_t)(huart5.Instance->RDR & 0xFF);
  }

  return count;
}

void MX_UART5_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO clocks for UART5 pins
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // Enable UART5 clock
  __HAL_RCC_UART5_CLK_ENABLE();

  // Configure UART5 TX (PC12) and RX (PD2) pins
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Configure UART5
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 500000;  // ESP8266 is running at 500K baud!
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK) {
    Error_Handler();
  }

  // Enable UART5 interrupt in NVIC
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

// MX_RTC_Init is defined in rtc.c
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  Thread-safe UART5 transmit function
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data elements (u8/u16 etc) to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  * @note   Uses mutex to protect concurrent access from ESP8266_SendCommand and CDC_Receive_HS
  */
HAL_StatusTypeDef UART5_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  HAL_StatusTypeDef status;

  // If mutex exists (FreeRTOS running), use it to protect transmission
  if (uart5MutexId != NULL)
  {
    if (osMutexAcquire(uart5MutexId, Timeout) == osOK)
    {
      status = HAL_UART_Transmit(&huart5, pData, Size, Timeout);
      osMutexRelease(uart5MutexId);
    }
    else
    {
      status = HAL_ERROR;
    }
  }
  else
  {
    // FreeRTOS not running yet, call directly
    status = HAL_UART_Transmit(&huart5, pData, Size, Timeout);
  }

  return status;
}

/**
  * @brief  Thread-safe USB CDC transmit function
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  * @note   Uses mutex to protect concurrent access from DisplayTask, DebugTask, DefaultTask
  */
uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len)
{
  uint8_t result;

  // CRITICAL FIX: Check if we're in interrupt context by attempting non-blocking mutex acquire
  // If mutex exists (FreeRTOS running), use it to protect transmission
  if (cdcTransmitMutexId != NULL)
  {
    // Try to acquire mutex with zero timeout (non-blocking)
    // This allows the function to work from interrupt context or when mutex is busy
    if (osMutexAcquire(cdcTransmitMutexId, 0) == osOK)
    {
      result = CDC_Transmit_HS(Buf, Len);
      osMutexRelease(cdcTransmitMutexId);
    }
    else
    {
      // Mutex busy or we're in interrupt context - try direct transmission
      // This is safe for USB CDC since it has internal buffering
      result = CDC_Transmit_HS(Buf, Len);
    }
  }
  else
  {
    // FreeRTOS not running yet, call directly
    result = CDC_Transmit_HS(Buf, Len);
  }

  return result;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
