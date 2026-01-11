/**
 ******************************************************************************
 * @file    stm32f7xx_hal_uart.h
 * @author  MCD Application Team
 * @brief   Header file of UART HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F7xx_HAL_UART_H
#define STM32F7xx_HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal_def.h"

/** @addtogroup STM32F7xx_HAL_Driver
 * @{
 */

/** @addtogroup UART
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/** @defgroup UART_Exported_Types UART Exported Types
 * @{
 */

/**
 * @brief UART Init Structure definition
 */
typedef struct {
  uint32_t BaudRate; /*!< This member configures the UART communication baud
                        rate. The baud rate register is computed using the
                        following formula:
                          - If oversampling is 16 or in Lin mode,
                             Baud Rate Register = ((PCLKx) /
                        ((huart->Init.BaudRate)))
                          - If oversampling is 8,
                             Baud Rate Register[15:4] = ((2 * PCLKx) /
                        ((huart->Init.BaudRate)))[15:4] Baud Rate Register[3] =
                        0 Baud Rate Register[2:0] =  (((2 * PCLKx) /
                        ((huart->Init.BaudRate)))[3:0]) >> 1      */

  uint32_t WordLength; /*!< Specifies the number of data bits transmitted or
                          received in a frame. This parameter can be a value of
                          @ref UARTEx_Word_Length */

  uint32_t
      StopBits; /*!< Specifies the number of stop bits transmitted.
                     This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t Parity; /*!< Specifies the parity mode.
                        This parameter can be a value of @ref UART_Parity
                        @note When parity is enabled, the computed word length
                      is larger than the defined word length by 1 bit. */

  uint32_t
      Mode; /*!< Specifies whether the Receive or Transmit mode is enabled or
               disabled. This parameter can be a value of @ref UART_Mode */

  uint32_t HwFlowCtl; /*!< Specifies whether the hardware flow control mode is
                         enabled or disabled. This parameter can be a value of
                         @ref UART_Hardware_Flow_Control */

  uint32_t OverSampling; /*!< Specifies whether the Over sampling 8 is enabled
                            or disabled, to achieve higher speed (up to
                            f_PCLK/8). This parameter can be a value of @ref
                            UART_Over_Sampling */

  uint32_t OneBitSampling; /*!< Specifies whether a single sample or three
                              samples' majority vote is selected. Selecting the
                              single sample method increases the receiver
                              tolerance to clock deviations. This parameter can
                              be a value of @ref UART_OneBit_Sampling */
} UART_InitTypeDef;

/**
 * @brief  UART Advanced Features initialization structure definition
 */
typedef struct {
  uint32_t AdvFeatureInit; /*!< Specifies which advanced UART features is
                              initialized. Several Advanced Features may be
                              initialized at the same time .
                                This parameter can be a value of @ref
                              UART_Advanced_Features_Initialization_Type */

  uint32_t TxPinLevelInvert; /*!< Specifies whether the TX pin active level is
                                inverted. This parameter can be a value of @ref
                                UART_Tx_Inv  */

  uint32_t RxPinLevelInvert; /*!< Specifies whether the RX pin active level is
                                inverted. This parameter can be a value of @ref
                                UART_Rx_Inv  */

  uint32_t
      DataInvert; /*!< Specifies whether data are inverted (positive/direct
                     logic vs negative/inverted logic).
                       This parameter can be a value of @ref UART_Data_Inv */

  uint32_t Swap; /*!< Specifies whether TX and RX pins are swapped.
                      This parameter can be a value of @ref UART_Rx_Tx_Swap */

  uint32_t OverrunDisable; /*!< Specifies whether the reception overrun
                              detection is disabled. This parameter can be a
                              value of @ref UART_Overrun_Disable */

  uint32_t DMADisableonRxError; /*!< Specifies whether the DMA is disabled in
                                   case of reception error. This parameter can
                                   be a value of @ref
                                   UART_DMA_Disable_on_Rx_Error */

  uint32_t AutoBaudRateEnable; /*!< Specifies whether auto Baud rate detection
                                  is enabled. This parameter can be a value of
                                  @ref UART_AutoBaudRate_Enable */

  uint32_t AutoBaudRateMode; /*!< If auto Baud rate detection is enabled,
                                specifies how the reference clock is measured.
                                  This parameter can be a value of @ref
                                UART_AutoBaudRate_Mode */

  uint32_t
      MSBFirst; /*!< Specifies whether MSB is sent first on UART line.
                     This parameter can be a value of @ref UART_MSB_First */
} UART_AdvFeatureInitTypeDef;

/**
 * @brief HAL UART State definitions
 * @note  HAL UART State value is a combination of 2 different substates: gState
 * and RxState.
 *        - gState contains UART state information related to global Handle
 * management and also information related to Tx operations. gState value coding
 * follow below described bitmap : b7-b6  Error information 00 : No Error 01 :
 * (Not Used) 10 : Timeout 11 : Error b5     Peripheral initialization status 0
 * : Reset (Peripheral not initialized) 1  : Init done (Peripheral not
 * initialized. HAL UART Init function already called) b4-b3  (not used) xx :
 * Should be set to 00 b2     Intrinsic process state 0  : Ready 1  : Busy
 * (Peripheral busy with some configuration or internal operations) b1     (not
 * used) x  : Should be set to 0 b0     Tx state 0  : Ready (no Tx operation
 * ongoing) 1  : Busy (Tx operation ongoing)
 *        - RxState contains information related to Rx operations.
 *          RxState value coding follow below described bitmap :
 *          b7-b6  (not used)
 *             xx : Should be set to 00
 *          b5     Peripheral initialization status
 *             0  : Reset (Peripheral not initialized)
 *             1  : Init done (Peripheral not initialized)
 *          b4-b2  (not used)
 *             xxx : Should be set to 000
 *          b1     Rx state
 *             0  : Ready (no Rx operation ongoing)
 *             1  : Busy (Rx operation ongoing)
 *          b0     (not used)
 *             x  : Should be set to 0.
 */
typedef enum {
  HAL_UART_STATE_RESET = 0x00U,   /*!< Peripheral is not initialized
                                       Value is allowed for gState and RxState */
  HAL_UART_STATE_READY = 0x20U,   /*!< Peripheral Initialized and ready for use
                                       Value is allowed for gState and RxState */
  HAL_UART_STATE_BUSY = 0x24U,    /*!< an internal process is ongoing
                                       Value is allowed for gState only */
  HAL_UART_STATE_BUSY_TX = 0x21U, /*!< Data Transmission process is ongoing
                                       Value is allowed for gState only */
  HAL_UART_STATE_BUSY_RX = 0x22U, /*!< Data Reception process is ongoing
                                       Value is allowed for RxState only */
  HAL_UART_STATE_BUSY_TX_RX =
      0x23U, /*!< Data Transmission and Reception process is ongoing
                  Not to be used for current gState or RxState only */
  HAL_UART_STATE_TIMEOUT = 0xA0U, /*!< Timeout state
                                       Value is allowed for gState only */
  HAL_UART_STATE_ERROR = 0xE0U    /*!< Error
                                       Value is allowed for gState only */
} HAL_UART_StateTypeDef;

/**
 * @brief UART wake-up from stop mode parameters
 */
typedef struct {
  uint32_t WakeUpEvent; /*!< Specifies which event will activat the Wakeup from
                           Stop mode This parameter can be a value of @ref
                           UART_WakeUp_From_Stop_Selection */

  uint16_t AddressLength; /*!< Specifies whether the address is 4 or 7-bit long.
                               This parameter can be a value of @ref
                             UARTEx_WakeUp_Address_Length */

  uint8_t Address; /*!< UART/USART node address (7-bit long max). */
} UART_WakeUpTypeDef;

/**
 * @brief UART clock sources definition
 */
typedef enum {
  UART_CLOCKSOURCE_PCLK1 = 0x00U,    /*!< PCLK1 clock source  */
  UART_CLOCKSOURCE_PCLK2 = 0x01U,    /*!< PCLK2 clock source  */
  UART_CLOCKSOURCE_HSI = 0x02U,      /*!< HSI clock source    */
  UART_CLOCKSOURCE_SYSCLK = 0x04U,   /*!< SYSCLK clock source */
  UART_CLOCKSOURCE_LSE = 0x08U,      /*!< LSE clock source       */
  UART_CLOCKSOURCE_UNDEFINED = 0x10U /*!< Undefined clock source */
} UART_ClockSourceTypeDef;

/**
 * @brief  UART handle Structure definition
 */
typedef struct __UART_HandleTypeDef {
  USART_TypeDef *Instance; /*!< UART registers base address        */

  UART_InitTypeDef Init; /*!< UART communication parameters      */

  UART_AdvFeatureInitTypeDef
      AdvancedInit; /*!< UART Advanced Features initialization parameters */

  uint8_t *pTxBuffPtr; /*!< Pointer to UART Tx transfer Buffer */

  uint16_t TxXferSize; /*!< UART Tx Transfer size              */

  __IO uint16_t TxXferCount; /*!< UART Tx Transfer Counter           */

  uint8_t *pRxBuffPtr; /*!< Pointer to UART Rx transfer Buffer */

  uint16_t RxXferSize; /*!< UART Rx Transfer size              */

  __IO uint16_t RxXferCount; /*!< UART Rx Transfer Counter           */

  uint16_t Mask; /*!< UART Rx RDR register mask          */

  DMA_HandleTypeDef *hdmatx; /*!< UART Tx DMA Handle parameters      */

  DMA_HandleTypeDef *hdmarx; /*!< UART Rx DMA Handle parameters      */

  HAL_LockTypeDef Lock; /*!< Locking object                     */

  __IO HAL_UART_StateTypeDef
      gState; /*!< UART state information related to global Handle management
                   and also information related to Tx operations.
                   This parameter can be a value of @ref HAL_UART_StateTypeDef
               */

  __IO HAL_UART_StateTypeDef RxState; /*!< UART state information related to Rx
                                         operations. This parameter can be a
                                         value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t ErrorCode; /*!< UART Error code                    */

} UART_HandleTypeDef;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_Exported_Constants UART Exported Constants
 * @{
 */

/** @defgroup UART_Error_Code UART Error Code
 * @{
 */
#define HAL_UART_ERROR_NONE 0x00000000U /*!< No error            */
#define HAL_UART_ERROR_PE 0x00000001U   /*!< Parity error        */
#define HAL_UART_ERROR_NE 0x00000002U   /*!< Noise error         */
#define HAL_UART_ERROR_FE 0x00000004U   /*!< Frame error         */
#define HAL_UART_ERROR_ORE 0x00000008U  /*!< Overrun error       */
#define HAL_UART_ERROR_DMA 0x00000010U  /*!< DMA transfer error  */
/**
 * @}
 */

/** @defgroup UART_Stop_Bits   UART Number of Stop Bits
 * @{
 */
#define UART_STOPBITS_0_5                                                      \
  USART_CR2_STOP_0                  /*!< UART frame with 0.5 stop bit          \
                                     */
#define UART_STOPBITS_1 0x00000000U /*!< UART frame with 1 stop bit    */
#define UART_STOPBITS_1_5                                                      \
  (USART_CR2_STOP_0 | USART_CR2_STOP_1)  /*!< UART frame with 1.5 stop bits */
#define UART_STOPBITS_2 USART_CR2_STOP_1 /*!< UART frame with 2 stop bits   */
/**
 * @}
 */

/** @defgroup UART_Parity  UART Parity
 * @{
 */
#define UART_PARITY_NONE 0x00000000U                   /*!< No parity   */
#define UART_PARITY_EVEN USART_CR1_PCE                 /*!< Even parity */
#define UART_PARITY_ODD (USART_CR1_PCE | USART_CR1_PS) /*!< Odd parity  */
/**
 * @}
 */

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
 * @{
 */
#define UART_HWCONTROL_NONE 0x00000000U   /*!< No hardware control       */
#define UART_HWCONTROL_RTS USART_CR3_RTSE /*!< Request To Send           */
#define UART_HWCONTROL_CTS USART_CR3_CTSE /*!< Clear To Send             */
#define UART_HWCONTROL_RTS_CTS                                                 \
  (USART_CR3_RTSE | USART_CR3_CTSE) /*!< Request and Clear To Send */
/**
 * @}
 */

/** @defgroup UART_Mode UART Transfer Mode
 * @{
 */
#define UART_MODE_RX USART_CR1_RE                     /*!< RX mode        */
#define UART_MODE_TX USART_CR1_TE                     /*!< TX mode        */
#define UART_MODE_TX_RX (USART_CR1_TE | USART_CR1_RE) /*!< RX and TX mode */
/**
 * @}
 */

/** @defgroup UART_State  UART State
 * @{
 */
#define UART_STATE_DISABLE 0x00000000U /*!< UART disabled  */
#define UART_STATE_ENABLE USART_CR1_UE /*!< UART enabled   */
/**
 * @}
 */

/** @defgroup UART_Over_Sampling UART Over Sampling
 * @{
 */
#define UART_OVERSAMPLING_16 0x00000000U    /*!< Oversampling by 16 */
#define UART_OVERSAMPLING_8 USART_CR1_OVER8 /*!< Oversampling by 8  */
/**
 * @}
 */

/** @defgroup UART_OneBit_Sampling UART One Bit Sampling Method
 * @{
 */
#define UART_ONE_BIT_SAMPLE_DISABLE                                            \
  0x00000000U /*!< One-bit sampling disable                                    \
               */
#define UART_ONE_BIT_SAMPLE_ENABLE                                             \
  USART_CR3_ONEBIT /*!< One-bit sampling enable  */
/**
 * @}
 */

/** @defgroup UART_AutoBaud_Rate_Mode    UART Advanced Feature AutoBaud Rate
 * Mode
 * @{
 */
#define UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT                                \
  0x00000000U /*!< Auto Baud rate detection on start bit            */
#define UART_ADVFEATURE_AUTOBAUDRATE_ONFALLINGEDGE                             \
  USART_CR2_ABRMODE_0 /*!< Auto Baud rate detection on falling edge         */
#define UART_ADVFEATURE_AUTOBAUDRATE_ON0X7FFRAME                               \
  USART_CR2_ABRMODE_1 /*!< Auto Baud rate detection on 0x7F frame detection */
#define UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME                               \
  (USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1) /*!< Auto Baud rate detection on \
                                                 0x55 frame detection */
/**
 * @}
 */

/** @defgroup UART_Receiver_TimeOut UART Receiver TimeOut
 * @{
 */
#define UART_RECEIVER_TIMEOUT_DISABLE                                          \
  0x00000000U /*!< UART receiver timeout disable */
#define UART_RECEIVER_TIMEOUT_ENABLE                                           \
  USART_CR2_RTOEN /*!< UART receiver timeout enable  */
/**
 * @}
 */

/** @defgroup UART_LIN    UART Local Interconnection Network mode
 * @{
 */
#define UART_LIN_DISABLE                                                       \
  0x00000000U /*!< Local Interconnect Network disable                          \
               */
#define UART_LIN_ENABLE                                                        \
  USART_CR2_LINEN /*!< Local Interconnect Network enable  */
/**
 * @}
 */

/** @defgroup UART_LIN_Break_Detection  UART LIN Break Detection
 * @{
 */
#define UART_LINBREAKDETECTLENGTH_10B                                          \
  0x00000000U /*!< LIN 10-bit break detection length */
#define UART_LINBREAKDETECTLENGTH_11B                                          \
  USART_CR2_LBDL /*!< LIN 11-bit break detection length */
/**
 * @}
 */

/** @defgroup UART_DMA_Tx    UART DMA Tx
 * @{
 */
#define UART_DMA_TX_DISABLE 0x00000000U   /*!< UART DMA TX disabled */
#define UART_DMA_TX_ENABLE USART_CR3_DMAT /*!< UART DMA TX enabled  */
/**
 * @}
 */

/** @defgroup UART_DMA_Rx   UART DMA Rx
 * @{
 */
#define UART_DMA_RX_DISABLE 0x00000000U   /*!< UART DMA RX disabled */
#define UART_DMA_RX_ENABLE USART_CR3_DMAR /*!< UART DMA RX enabled  */
/**
 * @}
 */

/** @defgroup UART_Half_Duplex_Selection  UART Half Duplex Selection
 * @{
 */
#define UART_HALF_DUPLEX_DISABLE 0x00000000U /*!< UART half-duplex disabled */
#define UART_HALF_DUPLEX_ENABLE                                                \
  USART_CR3_HDSEL /*!< UART half-duplex enabled  */
/**
 * @}
 */

/** @defgroup UART_WakeUp_Methods   UART WakeUp Methods
 * @{
 */
#define UART_WAKEUPMETHOD_IDLELINE                                             \
  0x00000000U /*!< UART wake-up on idle line    */
#define UART_WAKEUPMETHOD_ADDRESSMARK                                          \
  USART_CR1_WAKE /*!< UART wake-up on address mark */
/**
 * @}
 */

/** @defgroup UART_Request_Parameters UART Request Parameters
 * @{
 */
#define UART_AUTOBAUD_REQUEST                                                  \
  USART_RQR_ABRRQ /*!< Auto-Baud Rate Request      */
#define UART_SENDBREAK_REQUEST                                                 \
  USART_RQR_SBKRQ /*!< Send Break Request          */
#define UART_MUTE_MODE_REQUEST                                                 \
  USART_RQR_MMRQ /*!< Mute Mode Request           */
#define UART_RXDATA_FLUSH_REQUEST                                              \
  USART_RQR_RXFRQ /*!< Receive Data flush Request  */
#define UART_TXDATA_FLUSH_REQUEST                                              \
  USART_RQR_TXFRQ /*!< Transmit data flush Request */
/**
 * @}
 */

/** @defgroup UART_Advanced_Features_Initialization_Type  UART Advanced Features
 * Initialization Type
 * @{
 */
#define UART_ADVFEATURE_NO_INIT                                                \
  0x00000000U /*!< No advanced feature initialization       */
#define UART_ADVFEATURE_TXINVERT_INIT                                          \
  0x00000001U /*!< TX pin active level inversion            */
#define UART_ADVFEATURE_RXINVERT_INIT                                          \
  0x00000002U /*!< RX pin active level inversion            */
#define UART_ADVFEATURE_DATAINVERT_INIT                                        \
  0x00000004U /*!< Binary data inversion                    */
#define UART_ADVFEATURE_SWAP_INIT                                              \
  0x00000008U /*!< Pins swap                                */
#define UART_ADVFEATURE_RXOVERRUNDISABLE_INIT                                  \
  0x00000010U /*!< RX overrun detection disabling           */
#define UART_ADVFEATURE_DMADISABLEONERROR_INIT                                 \
  0x00000020U /*!< DMA disable on Reception Error           */
#define UART_ADVFEATURE_AUTOBAUDRATE_INIT                                      \
  0x00000040U /*!< Auto Baud rate detection initialization  */
#define UART_ADVFEATURE_MSBFIRST_INIT                                          \
  0x00000080U /*!< Most significant bit sent/received first */
/**
 * @}
 */

/** @defgroup UART_Tx_Inv UART Advanced Feature TX Pin Active Level Inversion
 * @{
 */
#define UART_ADVFEATURE_TXINV_DISABLE                                          \
  0x00000000U /*!< TX pin active level inversion disable */
#define UART_ADVFEATURE_TXINV_ENABLE                                           \
  USART_CR2_TXINV /*!< TX pin active level inversion enable  */
/**
 * @}
 */

/** @defgroup UART_Rx_Inv UART Advanced Feature RX Pin Active Level Inversion
 * @{
 */
#define UART_ADVFEATURE_RXINV_DISABLE                                          \
  0x00000000U /*!< RX pin active level inversion disable */
#define UART_ADVFEATURE_RXINV_ENABLE                                           \
  USART_CR2_RXINV /*!< RX pin active level inversion enable  */
/**
 * @}
 */

/** @defgroup UART_Data_Inv  UART Advanced Feature Binary Data Inversion
 * @{
 */
#define UART_ADVFEATURE_DATAINV_DISABLE                                        \
  0x00000000U /*!< Binary data inversion disable */
#define UART_ADVFEATURE_DATAINV_ENABLE                                         \
  USART_CR2_DATAINV /*!< Binary data inversion enable  */
/**
 * @}
 */

/** @defgroup UART_Rx_Tx_Swap UART Advanced Feature RX TX Pins Swap
 * @{
 */
#define UART_ADVFEATURE_SWAP_DISABLE                                           \
  0x00000000U /*!< TX/RX pins swap disable                                     \
               */
#define UART_ADVFEATURE_SWAP_ENABLE                                            \
  USART_CR2_SWAP /*!< TX/RX pins swap enable  */
/**
 * @}
 */

/** @defgroup UART_Overrun_Disable  UART Advanced Feature Overrun Disable
 * @{
 */
#define UART_ADVFEATURE_OVERRUN_ENABLE                                         \
  0x00000000U /*!< RX overrun detection enable  */
#define UART_ADVFEATURE_OVERRUN_DISABLE                                        \
  USART_CR3_OVRDIS /*!< RX overrun detection disable */
/**
 * @}
 */

/** @defgroup UART_AutoBaudRate_Enable  UART Advanced Feature Auto BaudRate
 * Enable
 * @{
 */
#define UART_ADVFEATURE_AUTOBAUDRATE_DISABLE                                   \
  0x00000000U /*!< RX Auto Baud rate detection enable  */
#define UART_ADVFEATURE_AUTOBAUDRATE_ENABLE                                    \
  USART_CR2_ABREN /*!< RX Auto Baud rate detection disable */
/**
 * @}
 */

/** @defgroup UART_DMA_Disable_on_Rx_Error   UART Advanced Feature DMA Disable
 * On Rx Error
 * @{
 */
#define UART_ADVFEATURE_DMA_ENABLEONRXERROR                                    \
  0x00000000U /*!< DMA enable on Reception Error  */
#define UART_ADVFEATURE_DMA_DISABLEONRXERROR                                   \
  USART_CR3_DDRE /*!< DMA disable on Reception Error */
/**
 * @}
 */

/** @defgroup UART_MSB_First   UART Advanced Feature MSB First
 * @{
 */
#define UART_ADVFEATURE_MSBFIRST_DISABLE                                       \
  0x00000000U /*!< Most significant bit sent/received first disable */
#define UART_ADVFEATURE_MSBFIRST_ENABLE                                        \
  USART_CR2_MSBFIRST /*!< Most significant bit sent/received first enable  */
/**
 * @}
 */

/** @defgroup UART_Mute_Mode   UART Advanced Feature Mute Mode Enable
 * @{
 */
#define UART_ADVFEATURE_MUTEMODE_DISABLE                                       \
  0x00000000U /*!< UART mute mode disable */
#define UART_ADVFEATURE_MUTEMODE_ENABLE                                        \
  USART_CR1_MME /*!< UART mute mode enable  */
/**
 * @}
 */

/** @defgroup UART_CR2_ADDRESS_LSB_POS    UART Address-matching LSB Position In
 * CR2 Register
 * @{
 */
#define UART_CR2_ADDRESS_LSB_POS                                               \
  24U /*!< UART address-matching LSB position in CR2 register */
/**
 * @}
 */

/** @defgroup UART_DriverEnable_Polarity      UART DriverEnable Polarity
 * @{
 */
#define UART_DE_POLARITY_HIGH                                                  \
  0x00000000U /*!< Driver enable signal is active high */
#define UART_DE_POLARITY_LOW                                                   \
  USART_CR3_DEP /*!< Driver enable signal is active low  */
/**
 * @}
 */

/** @defgroup UART_CR1_DEAT_ADDRESS_LSB_POS    UART Driver Enable Assertion Time
 * LSB Position In CR1 Register
 * @{
 */
#define UART_CR1_DEAT_ADDRESS_LSB_POS                                          \
  21U /*!< UART Driver Enable assertion time LSB position in CR1 register */
/**
 * @}
 */

/** @defgroup UART_CR1_DEDT_ADDRESS_LSB_POS    UART Driver Enable DeAssertion
 * Time LSB Position In CR1 Register
 * @{
 */
#define UART_CR1_DEDT_ADDRESS_LSB_POS                                          \
  16U /*!< UART Driver Enable de-assertion time LSB position in CR1 register   \
       */
/**
 * @}
 */

/** @defgroup UART_Interruption_Mask    UART Interruptions Mask
 * @{
 */
#define UART_IT_MASK 0x001FU /*!< UART interruptions flags mask */
/**
 * @}
 */

/** @defgroup UART_TimeOut_Value    UART polling-based communications time-out
 * value
 * @{
 */
#define HAL_UART_TIMEOUT_VALUE                                                 \
  0x1FFFFFFU /*!< UART polling-based communications time-out value */
/**
 * @}
 */

/** @defgroup UART_Flags     UART Status Flags
 *        Elements values convention: 0xXXXX
 *           - 0xXXXX  : Flag mask in the ISR register
 * @{
 */
#define UART_FLAG_REACK                                                        \
  USART_ISR_REACK /*!< UART receive enable acknowledge flag      */
#define UART_FLAG_TEACK                                                        \
  USART_ISR_TEACK /*!< UART transmit enable acknowledge flag     */
#define UART_FLAG_WUF                                                          \
  USART_ISR_WUF /*!< UART wake-up from stop mode flag          */
#define UART_FLAG_RWU                                                          \
  USART_ISR_RWU /*!< UART receiver wake-up from mute mode flag */
#define UART_FLAG_SBKF                                                         \
  USART_ISR_SBKF /*!< UART send break flag                      */
#define UART_FLAG_CMF                                                          \
  USART_ISR_CMF /*!< UART character match flag                 */
#define UART_FLAG_BUSY                                                         \
  USART_ISR_BUSY /*!< UART busy flag                            */
#define UART_FLAG_ABRF                                                         \
  USART_ISR_ABRF /*!< UART auto Baud rate flag                  */
#define UART_FLAG_ABRE                                                         \
  USART_ISR_ABRE /*!< UART auto Baud rate error                 */
#define UART_FLAG_EOBF                                                         \
  USART_ISR_EOBF /*!< UART end of block flag                    */
#define UART_FLAG_RTOF                                                         \
  USART_ISR_RTOF /*!< UART receiver timeout flag                */
#define UART_FLAG_CTS                                                          \
  USART_ISR_CTS /*!< UART clear to send flag                   */
#define UART_FLAG_CTSIF                                                        \
  USART_ISR_CTSIF /*!< UART clear to send interrupt flag         */
#define UART_FLAG_LBDF                                                         \
  USART_ISR_LBDF /*!< UART LIN break detection flag             */
#define UART_FLAG_TXE                                                          \
  USART_ISR_TXE /*!< UART transmit data register empty         */
#define UART_FLAG_TC                                                           \
  USART_ISR_TC /*!< UART transmission complete                */
#define UART_FLAG_RXNE                                                         \
  USART_ISR_RXNE /*!< UART read data register not empty         */
#define UART_FLAG_IDLE                                                         \
  USART_ISR_IDLE /*!< UART idle flag                            */
#define UART_FLAG_ORE                                                          \
  USART_ISR_ORE /*!< UART overrun error                        */
#define UART_FLAG_NE                                                           \
  USART_ISR_NE /*!< UART noise error                          */
#define UART_FLAG_FE                                                           \
  USART_ISR_FE /*!< UART frame error                          */
#define UART_FLAG_PE                                                           \
  USART_ISR_PE /*!< UART parity error                         */
/**
 * @}
 */

/** @defgroup UART_Interrupt_definition   UART Interrupts Definition
 *        Elements values convention: 0000ZZZZ0XXYYYYYb
 *           - YYYYY  : Interrupt source position in the XX register (5bits)
 *           - XX  : Interrupt source register (2bits)
 *                 - 01: CR1 register
 *                 - 10: CR2 register
 *                 - 11: CR3 register
 *           - ZZZZ  : Flag position in the ISR register(4bits)
 * @{
 */
#define UART_IT_PE                                                             \
  0x0028U /*!< UART parity error interruption                 */
#define UART_IT_TXE                                                            \
  0x0727U /*!< UART transmit data register empty interruption */
#define UART_IT_TC                                                             \
  0x0626U /*!< UART transmission complete interruption        */
#define UART_IT_RXNE                                                           \
  0x0525U /*!< UART read data register not empty interruption */
#define UART_IT_IDLE                                                           \
  0x0424U /*!< UART idle interruption                         */
#define UART_IT_LBD                                                            \
  0x0846U /*!< UART LIN break detection interruption          */
#define UART_IT_CTS                                                            \
  0x096AU /*!< UART CTS interruption                          */
#define UART_IT_CM                                                             \
  0x112EU /*!< UART character match interruption              */
#define UART_IT_WUF                                                            \
  0x1476U /*!< UART wake-up from stop mode interruption       */
#define UART_IT_ERR                                                            \
  0x0060U /*!< UART error interruption                        */
#define UART_IT_ORE                                                            \
  0x0300U /*!< UART overrun error interruption                */
#define UART_IT_NE                                                             \
  0x0200U /*!< UART noise error interruption                  */
#define UART_IT_FE                                                             \
  0x0100U /*!< UART frame error interruption                  */
/**
 * @}
 */

/** @defgroup UART_IT_CLEAR_Flags  UART Interruption Clear Flags
 * @{
 */
#define UART_CLEAR_PEF USART_ICR_PECF /*!< Parity Error Clear Flag */
#define UART_CLEAR_FEF USART_ICR_FECF /*!< Framing Error Clear Flag */
#define UART_CLEAR_NEF USART_ICR_NECF /*!< Noise Error detected Clear Flag */
#define UART_CLEAR_OREF                                                        \
  USART_ICR_ORECF /*!< OverRun Error Clear Flag          */
#define UART_CLEAR_IDLEF                                                       \
  USART_ICR_IDLECF                    /*!< IDLE line detected Clear Flag     */
#define UART_CLEAR_TCF USART_ICR_TCCF /*!< Transmission Complete Clear Flag */
#define UART_CLEAR_LBDF                                                        \
  USART_ICR_LBDCF /*!< LIN Break Detection Clear Flag    */
#define UART_CLEAR_CTSF                                                        \
  USART_ICR_CTSCF /*!< CTS Interrupt Clear Flag          */
#define UART_CLEAR_RTOF                                                        \
  USART_ICR_RTOCF /*!< Receiver Time Out Clear Flag      */
#define UART_CLEAR_EOBF                                                        \
  USART_ICR_EOBCF                     /*!< End Of Block Clear Flag           */
#define UART_CLEAR_CMF USART_ICR_CMCF /*!< Character Match Clear Flag */
#define UART_CLEAR_WUF                                                         \
  USART_ICR_WUCF /*!< Wake Up from stop mode Clear Flag                        \
                  */
/**
 * @}
 */

/**
 * @}
 */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup UART_Exported_Macros UART Exported Macros
 * @{
 */

/** @brief  Reset UART handle state.
 * @param  __HANDLE__ UART handle.
 * @retval None
 */
#define __HAL_UART_RESET_HANDLE_STATE(__HANDLE__)                              \
  ((__HANDLE__)->gState = HAL_UART_STATE_RESET)

/** @brief  Flush the UART Data registers.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_FLUSH_DRREGISTER(__HANDLE__)                                \
  do {                                                                         \
    SET_BIT((__HANDLE__)->Instance->RQR, UART_RXDATA_FLUSH_REQUEST);           \
    SET_BIT((__HANDLE__)->Instance->RQR, UART_TXDATA_FLUSH_REQUEST);           \
  } while (0)

/** @brief  Clear the specified UART pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __FLAG__ specifies the flag to check.
 *          This parameter can be any combination of the following values:
 *            @arg @ref UART_CLEAR_PEF      Parity Error Clear Flag
 *            @arg @ref UART_CLEAR_FEF      Framing Error Clear Flag
 *            @arg @ref UART_CLEAR_NEF      Noise detected Clear Flag
 *            @arg @ref UART_CLEAR_OREF     Overrun Error Clear Flag
 *            @arg @ref UART_CLEAR_IDLEF    IDLE line detected Clear Flag
 *            @arg @ref UART_CLEAR_TCF      Transmission Complete Clear Flag
 *            @arg @ref UART_CLEAR_LBDF     LIN Break Detection Clear Flag
 *            @arg @ref UART_CLEAR_CTSF     CTS Interrupt Clear Flag
 *            @arg @ref UART_CLEAR_RTOF     Receiver Time Out Clear Flag
 *            @arg @ref UART_CLEAR_EOBF     End Of Block Clear Flag
 *            @arg @ref UART_CLEAR_CMF      Character Match Clear Flag
 *            @arg @ref UART_CLEAR_WUF      Wake Up from stop mode Clear Flag
 * @retval None
 */
#define __HAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__)                            \
  ((__HANDLE__)->Instance->ICR = (__FLAG__))

/** @brief  Clear the UART PE pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_CLEAR_PEFLAG(__HANDLE__)                                    \
  __HAL_UART_CLEAR_FLAG((__HANDLE__), UART_CLEAR_PEF)

/** @brief  Clear the UART FE pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_CLEAR_FEFLAG(__HANDLE__)                                    \
  __HAL_UART_CLEAR_FLAG((__HANDLE__), UART_CLEAR_FEF)

/** @brief  Clear the UART NE pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_CLEAR_NEFLAG(__HANDLE__)                                    \
  __HAL_UART_CLEAR_FLAG((__HANDLE__), UART_CLEAR_NEF)

/** @brief  Clear the UART ORE pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__)                                   \
  __HAL_UART_CLEAR_FLAG((__HANDLE__), UART_CLEAR_OREF)

/** @brief  Clear the UART IDLE pending flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_CLEAR_IDLEFLAG(__HANDLE__)                                  \
  __HAL_UART_CLEAR_FLAG((__HANDLE__), UART_CLEAR_IDLEF)

/** @brief  Check whether the specified UART flag is set or not.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __FLAG__ specifies the flag to check.
 *        This parameter can be one of the following values:
 *            @arg @ref UART_FLAG_REACK Receive enable acknowledge flag
 *            @arg @ref UART_FLAG_TEACK Transmit enable acknowledge flag
 *            @arg @ref UART_FLAG_WUF   Wake up from stop mode flag
 *            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in
 * mute mode)
 *            @arg @ref UART_FLAG_SBKF  Send Break flag
 *            @arg @ref UART_FLAG_CMF   Character match flag
 *            @arg @ref UART_FLAG_BUSY  Busy flag
 *            @arg @ref UART_FLAG_ABRF  Auto Baud rate (status) flag
 *            @arg @ref UART_FLAG_ABRE  Auto Baud rate error flag
 *            @arg @ref UART_FLAG_EOBF  End of block flag
 *            @arg @ref UART_FLAG_RTOF  Receiver timeout flag
 *            @arg @ref UART_FLAG_CTS   CTS Change flag
 *            @arg @ref UART_FLAG_LBD   LIN Break detection flag
 *            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
 *            @arg @ref UART_FLAG_TC    Transmission Complete flag
 *            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
 *            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
 *            @arg @ref UART_FLAG_ORE   OverRun Error flag
 *            @arg @ref UART_FLAG_NE    Noise Error flag
 *            @arg @ref UART_FLAG_FE    Framing Error flag
 *            @arg @ref UART_FLAG_PE    Parity Error flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__)                              \
  (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))

/** @brief  Enable the specified UART interrupt.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __INTERRUPT__ specifies the UART interrupt source to enable.
 *          This parameter can be one of the following values:
 *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
 *            @arg @ref UART_IT_CM   Character match interrupt
 *            @arg @ref UART_IT_CTS  CTS change interrupt
 *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
 *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
 *            @arg @ref UART_IT_TC   Transmission complete interrupt
 *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
 *            @arg @ref UART_IT_IDLE Idle line detection interrupt
 *            @arg @ref UART_IT_PE   Parity Error interrupt
 *            @arg @ref UART_IT_ERR  Error interrupt(Frame error, noise error,
 * overrun error)
 * @retval None
 */
#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)                        \
  (((((uint8_t)(__INTERRUPT__)) >> 5U) == 1U)                                  \
       ? ((__HANDLE__)->Instance->CR1 |=                                       \
          (1U << ((__INTERRUPT__)&UART_IT_MASK)))                              \
   : ((((uint8_t)(__INTERRUPT__)) >> 5U) == 2U)                                \
       ? ((__HANDLE__)->Instance->CR2 |=                                       \
          (1U << ((__INTERRUPT__)&UART_IT_MASK)))                              \
       : ((__HANDLE__)->Instance->CR3 |=                                       \
          (1U << ((__INTERRUPT__)&UART_IT_MASK))))

/** @brief  Disable the specified UART interrupt.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __INTERRUPT__ specifies the UART interrupt source to disable.
 *          This parameter can be one of the following values:
 *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
 *            @arg @ref UART_IT_CM   Character match interrupt
 *            @arg @ref UART_IT_CTS  CTS change interrupt
 *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
 *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
 *            @arg @ref UART_IT_TC   Transmission complete interrupt
 *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
 *            @arg @ref UART_IT_IDLE Idle line detection interrupt
 *            @arg @ref UART_IT_PE   Parity Error interrupt
 *            @arg @ref UART_IT_ERR  Error interrupt(Frame error, noise error,
 * overrun error)
 * @retval None
 */
#define __HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)                       \
  (((((uint8_t)(__INTERRUPT__)) >> 5U) == 1U)                                  \
       ? ((__HANDLE__)->Instance->CR1 &=                                       \
          ~(1U << ((__INTERRUPT__)&UART_IT_MASK)))                             \
   : ((((uint8_t)(__INTERRUPT__)) >> 5U) == 2U)                                \
       ? ((__HANDLE__)->Instance->CR2 &=                                       \
          ~(1U << ((__INTERRUPT__)&UART_IT_MASK)))                             \
       : ((__HANDLE__)->Instance->CR3 &=                                       \
          ~(1U << ((__INTERRUPT__)&UART_IT_MASK))))

/** @brief  Check whether the specified UART interrupt has occurred or not.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __IT__ specifies the UART interrupt to check.
 *          This parameter can be one of the following values:
 *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
 *            @arg @ref UART_IT_CM   Character match interrupt
 *            @arg @ref UART_IT_CTS  CTS change interrupt
 *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
 *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
 *            @arg @ref UART_IT_TC   Transmission complete interrupt
 *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
 *            @arg @ref UART_IT_IDLE Idle line detection interrupt
 *            @arg @ref UART_IT_PE   Parity Error interrupt
 *            @arg @ref UART_IT_ERR  Error interrupt(Frame error, noise error,
 * overrun error)
 * @retval The new state of __IT__ (TRUE or FALSE).
 */
#define __HAL_UART_GET_IT_SOURCE(__HANDLE__, __IT__)                           \
  (((((uint8_t)(__IT__)) >> 5U) == 1U)                                         \
       ? (((__HANDLE__)->Instance->CR1 & (1U << ((__IT__)&UART_IT_MASK))) ==   \
          (1U << ((__IT__)&UART_IT_MASK)))                                     \
   : ((((uint8_t)(__IT__)) >> 5U) == 2U)                                       \
       ? (((__HANDLE__)->Instance->CR2 & (1U << ((__IT__)&UART_IT_MASK))) ==   \
          (1U << ((__IT__)&UART_IT_MASK)))                                     \
       : (((__HANDLE__)->Instance->CR3 & (1U << ((__IT__)&UART_IT_MASK))) ==   \
          (1U << ((__IT__)&UART_IT_MASK))))

/** @brief  Check whether the specified UART interrupt flag is set or not.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __IT__ specifies the UART interrupt flag to check.
 *          This parameter can be one of the following values:
 *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
 *            @arg @ref UART_IT_CM   Character match interrupt
 *            @arg @ref UART_IT_CTS  CTS change interrupt
 *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
 *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
 *            @arg @ref UART_IT_TC   Transmission complete interrupt
 *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
 *            @arg @ref UART_IT_IDLE Idle line detection interrupt
 *            @arg @ref UART_IT_PE   Parity Error interrupt
 *            @arg @ref UART_IT_ERR  Error interrupt(Frame error, noise error,
 * overrun error)
 * @retval The new state of __IT__ (TRUE or FALSE).
 */
#define __HAL_UART_GET_IT(__HANDLE__, __IT__)                                  \
  ((__HANDLE__)->Instance->ISR & ((uint32_t)1U << ((__IT__) >> 8U)))

/** @brief  Set a specific UART request flag.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __REQ__ specifies the request flag to set.
 *          This parameter can be one of the following values:
 *            @arg @ref UART_AUTOBAUD_REQUEST Auto-Baud Rate Request
 *            @arg @ref UART_SENDBREAK_REQUEST Send Break Request
 *            @arg @ref UART_MUTE_MODE_REQUEST Mute Mode Request
 *            @arg @ref UART_RXDATA_FLUSH_REQUEST Receive Data flush Request
 *            @arg @ref UART_TXDATA_FLUSH_REQUEST Transmit data flush Request
 * @retval None
 */
#define __HAL_UART_SEND_REQ(__HANDLE__, __REQ__)                               \
  ((__HANDLE__)->Instance->RQR |= (__REQ__))

/** @brief  Enable the UART one bit sample method.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_ONE_BIT_SAMPLE_ENABLE(__HANDLE__)                           \
  ((__HANDLE__)->Instance->CR3 |= USART_CR3_ONEBIT)

/** @brief  Disable the UART one bit sample method.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_ONE_BIT_SAMPLE_DISABLE(__HANDLE__)                          \
  ((__HANDLE__)->Instance->CR3 &= (uint32_t) ~((uint32_t)USART_CR3_ONEBIT))

/** @brief  Enable UART.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_ENABLE(__HANDLE__)                                          \
  ((__HANDLE__)->Instance->CR1 |= USART_CR1_UE)

/** @brief  Disable UART.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_DISABLE(__HANDLE__)                                         \
  ((__HANDLE__)->Instance->CR1 &= ~USART_CR1_UE)

/** @brief  Enable CTS flow control.
 * @note   This macro allows to enable CTS hardware flow control for a given
 * UART instance, without need to call HAL_UART_Init() function. As involving
 * direct access to UART registers, calling this macro does not invalidate
 *         respective HAL UART handle.
 * @note   For a given UART instance, CTS hardware flow control can be enabled
 * and disabled when that UART is in any state.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_HWCONTROL_CTS_ENABLE(__HANDLE__)                            \
  do {                                                                         \
    SET_BIT((__HANDLE__)->Instance->CR3, USART_CR3_CTSE);                      \
    (__HANDLE__)->Init.HwFlowCtl |= UART_HWCONTROL_CTS;                        \
  } while (0)

/** @brief  Disable CTS flow control.
 * @note   This macro allows to disable CTS hardware flow control for a given
 * UART instance, without need to call HAL_UART_Init() function. As involving
 * direct access to UART registers, calling this macro does not invalidate
 *         respective HAL UART handle.
 * @note   For a given UART instance, CTS hardware flow control can be enabled
 * and disabled when that UART is in any state.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_HWCONTROL_CTS_DISABLE(__HANDLE__)                           \
  do {                                                                         \
    CLEAR_BIT((__HANDLE__)->Instance->CR3, USART_CR3_CTSE);                    \
    (__HANDLE__)->Init.HwFlowCtl &= ~(UART_HWCONTROL_CTS);                     \
  } while (0)

/** @brief  Enable RTS flow control.
 * @note   This macro allows to enable RTS hardware flow control for a given
 * UART instance, without need to call HAL_UART_Init() function. As involving
 * direct access to UART registers, calling this macro does not invalidate
 *         respective HAL UART handle.
 * @note   For a given UART instance, RTS hardware flow control can be enabled
 * and disabled when that UART is in any state.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_HWCONTROL_RTS_ENABLE(__HANDLE__)                            \
  do {                                                                         \
    SET_BIT((__HANDLE__)->Instance->CR3, USART_CR3_RTSE);                      \
    (__HANDLE__)->Init.HwFlowCtl |= UART_HWCONTROL_RTS;                        \
  } while (0)

/** @brief  Disable RTS flow control.
 * @note   This macro allows to disable RTS hardware flow control for a given
 * UART instance, without need to call HAL_UART_Init() function. As involving
 * direct access to UART registers, calling this macro does not invalidate
 *         respective HAL UART handle.
 * @note   For a given UART instance, RTS hardware flow control can be enabled
 * and disabled when that UART is in any state.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval None
 */
#define __HAL_UART_HWCONTROL_RTS_DISABLE(__HANDLE__)                           \
  do {                                                                         \
    CLEAR_BIT((__HANDLE__)->Instance->CR3, USART_CR3_RTSE);                    \
    (__HANDLE__)->Init.HwFlowCtl &= ~(UART_HWCONTROL_RTS);                     \
  } while (0)

/**
 * @}
 */

/* Private macros --------------------------------------------------------*/
/** @defgroup UART_Private_Macros   UART Private Macros
 * @{
 */

/** @brief  BRR division operation to set BRR register in 8-bit oversampling
 * mode.
 * @param  __PCLK__ UART clock.
 * @param  __BAUD__ Baud rate.
 * @retval Division result.
 */
#define UART_DIV_SAMPLING8(__PCLK__, __BAUD__)                                 \
  ((((__PCLK__)*2U) + ((__BAUD__) / 2U)) / (__BAUD__))

/** @brief  BRR division operation to set BRR register in 16-bit oversampling
 * mode.
 * @param  __PCLK__ UART clock.
 * @param  __BAUD__ Baud rate.
 * @retval Division result.
 */
#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__)                                \
  (((__PCLK__) + ((__BAUD__) / 2U)) / (__BAUD__))

/** @brief  Check the Baud rate range.
 * @param  __BAUDRATE__ Baud rate.
 * @retval Test result (TRUE or FALSE).
 */
#define IS_UART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) < 9000001U)

/** @brief  Check the address of the UART Handle.
 * @param  __HANDLE__ UART handle.
 * @retval Test result (TRUE or FALSE).
 */
#define IS_UART_HANDLE(__HANDLE__) ((__HANDLE__) != NULL)

/** @brief  Check the parameter of the UART start bits count.
 * @param  __NB__ UART start bits count.
 * @retval Test result (TRUE or FALSE).
 */
#define IS_UART_START_BIT(__NB__)                                              \
  ((__NB__) == UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT)

/** @brief  Check the address of the UART Handle.
 * @param  __HANDLE__ UART handle.
 * @retval Test result (TRUE or FALSE).
 */
#ifndef IS_UART_INSTANCE
#define IS_UART_INSTANCE(__HANDLE__)                                           \
  (IS_UART_HANDLE(__HANDLE__) && (((__HANDLE__)->Instance) != NULL))
#endif

/**
 * @}
 */

/* Include UART HAL Extended module */
#include "stm32f7xx_hal_uart_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_Exported_Functions UART Exported Functions
 * @{
 */

/** @addtogroup UART_Exported_Functions_Group1 Initialization and
 * de-initialization functions
 * @{
 */

/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

/**
 * @}
 */

/** @addtogroup UART_Exported_Functions_Group2 IO operation functions
 * @{
 */

/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart,
                                       uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                      uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                       uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
/* Transfer Abort functions */
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);

/**
 * @}
 */

/** @addtogroup UART_Exported_Functions_Group3 Peripheral Control functions
 * @{
 */

/* Peripheral Control functions
 * ************************************************/
HAL_StatusTypeDef HAL_UART_ReceiverTimeout_Config(UART_HandleTypeDef *huart,
                                                  uint32_t TimeoutValue);
HAL_StatusTypeDef HAL_UART_EnableReceiverTimeout(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DisableReceiverTimeout(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef
HAL_UART_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef
HAL_UART_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_UART_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_HalfDuplex_Enable(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_HalfDuplex_Disable(UART_HandleTypeDef *huart);

/**
 * @}
 */

/** @addtogroup UART_Exported_Functions_Group4 Peripheral State and Error
 * functions
 * @{
 */

/* Peripheral State and Errors functions
 * **************************************************/
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);

/**
 * @}
 */

/**
 * @}
 */

/* Private functions
 * -----------------------------------------------------------*/
/** @addtogroup UART_Private_Functions UART Private Functions
 * @{
 */
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart);
/* HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart); */
/* HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart); */
/* HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart); */
/* HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart); */
/* HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart); */
/* HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart,
 * uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout); */
void UART_Wakeup_AddressConfig(UART_HandleTypeDef *huart,
                               UART_WakeUpTypeDef WakeUpSelection);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* STM32F7xx_HAL_UART_H */
