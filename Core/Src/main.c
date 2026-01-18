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
#include "../../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.h"
//#include "../../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.h"
//#include "../../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.h"
//#include "../../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.h"
#include "app_touchgfx.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma2d.h"
#include "dsihost.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "ltdc.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#define UART_RX_BUFFER_SIZE 2048
volatile uint8_t UartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint32_t UartRxHead = 0;
volatile uint32_t UartRxTail = 0;
uint8_t UartRxByte;
UART_HandleTypeDef huart5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CRC_Init(void);
void MX_DMA2D_Init(void);
void MX_DSIHOST_DSI_Init(void);
void MX_FMC_Init(void);
void MX_LTDC_Init(void);
void MX_QUADSPI_Init(void);
void MX_USB_DEVICE_Init(void);
void MX_UART5_Init(void);
void MX_FREERTOS_Init(void);
void StartDefaultTask(void *argument);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART5_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_DSIHOST_DSI_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* MX_TouchGFX_Init(); */
  /* Call PreOsInit function */
  /* MX_TouchGFX_PreOSInit(); */
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

  // Start UART Reception by enabling RXNE interrupt
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);

  // BSP_LCD_Clear(LCD_COLOR_WHITE);
  // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  // BSP_LCD_DisplayStringAt(0, 240, (uint8_t *)"STM32F769I-DISCO DSI WORKS!",
  //                         CENTER_MODE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in
                           cmsis_os2.c) */
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
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    Error_Handler();
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE BEGIN 4 */

void MX_UART5_Init(void) {
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
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
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
// HAL_UART_RxCpltCallback removed as we use UART5_IRQHandler in stm32f7xx_it.c

uint32_t UART_Read(uint8_t *pBuf, uint32_t Len) {
  uint32_t count = 0;
  while (count < Len && UartRxTail != UartRxHead) {
    pBuf[count++] = UartRxBuffer[UartRxTail];
    UartRxTail = (UartRxTail + 1) % UART_RX_BUFFER_SIZE;
  }
  return count;
}
/* USER CODE END 5 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
