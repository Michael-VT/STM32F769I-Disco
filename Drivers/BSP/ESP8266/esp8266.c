
#include "esp8266.h"
#include <stdio.h>
#include <string.h>

// External handle for UART (defined in main.c usually as huart6 or similar)
extern UART_HandleTypeDef huart5;
extern uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);

#define ESP_UART &huart5

void ESP8266_Init(void) {
  // Initialization logic: Reset, AT check
  ESP8266_SendCommand("AT");
  HAL_Delay(100);
  ESP8266_SendCommand("ATE0"); // Echo Off
  HAL_Delay(100);
}

ESP8266_StatusTypeDef ESP8266_SendCommand(const char *cmd) {
  // UART Transmit logic
  uint8_t buf[256];
  snprintf((char *)buf, sizeof(buf), "%s\r\n", cmd);
  HAL_UART_Transmit(ESP_UART, buf, strlen((char *)buf), 100);
  return ESP8266_OK;
}

ESP8266_StatusTypeDef ESP8266_Receive(char *buffer, uint16_t size) {
  // UART Receive logic using RingBuffer
  // This function tries to fill the buffer or timeout
  // For simplicity, we just check available bytes now
  uint32_t hasRead = UART_Read((uint8_t *)buffer, size);
  if (hasRead > 0)
    return ESP8266_OK;
  return ESP8266_TIMEOUT;
}

// Helper to wait for specific string
ESP8266_StatusTypeDef ESP8266_WaitFor(const char *pattern, uint32_t timeout) {
  uint32_t start = HAL_GetTick();
  uint8_t ch;
  uint32_t idx = 0;
  uint32_t patternLen = strlen(pattern);

  // Simple sliding window or just char matching
  // Note: this consumes data from ringbuffer!
  while (HAL_GetTick() - start < timeout) {
    if (UART_Read(&ch, 1) > 0) {
      if (ch == pattern[idx]) {
        idx++;
        if (idx == patternLen)
          return ESP8266_OK;
      } else {
        // Reset if mismatch (simple approach, fails on "AAT" for "AT" but ok
        // for distinct responses) Better: standard KMP or just stateless match
        // if unique enough
        if (ch == pattern[0])
          idx = 1;
        else
          idx = 0;
      }
    } else {
      // Allow minor sleep to let buffer fill
      // osDelay not available here if called from non-task context, so use
      // HAL_Delay(1)
      HAL_Delay(1);
    }
  }
  return ESP8266_TIMEOUT;
}
