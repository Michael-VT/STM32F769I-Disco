
#include "esp8266.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"

// External handle for UART (defined in main.c usually as huart6 or similar)
extern UART_HandleTypeDef huart5;
extern uint32_t UART_Read(uint8_t *pBuf, uint32_t Len);
extern HAL_StatusTypeDef UART5_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);

// LED activity indicators
extern volatile uint8_t ledEspActivity;

#define ESP_UART &huart5

// Forward declarations
static void ESP8266_HardwareReset(void);

/**
 * @brief Delay function that works before and after FreeRTOS starts
 * @param ms Milliseconds to delay
 * @note Uses osDelay if FreeRTOS is running, HAL_Delay otherwise
 */
static void ESP_Delay(uint32_t ms)
{
  /* Try to use osDelay if FreeRTOS scheduler is running.
   * If osDelay returns error (e.g., scheduler not started), fall back to HAL_Delay.
   * This is safe because osDelay will fail gracefully if FreeRTOS isn't running. */
  if (osDelay(ms) != osOK)
  {
    HAL_Delay(ms);
  }
}

static void ESP8266_HardwareReset(void) {
  // Configure WIFI_RST pin as output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  GPIO_InitStruct.Pin = WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

  // Pulse RESET pin low to reset ESP8266
  HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_RESET);
  ESP_Delay(100);  // Keep RESET low for 100ms (use osDelay if FreeRTOS running)
  HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_SET);
  // Wait for ESP8266 to boot up - needs sufficient time for firmware initialization
  ESP_Delay(1500);  // Increased from 500ms to 1500ms (ESP8266 needs 1-2 seconds)
}

ESP8266_StatusTypeDef ESP8266_Init(void) {
  extern UART_HandleTypeDef huart5;

  // Try different baud rates: 115200 (default), 57600, 9600
  const uint32_t baudRates[] = {115200, 57600, 9600, 500000};

  for (int br = 0; br < 4; br++) {
    // Reinitialize UART5 with new baud rate
    huart5.Init.BaudRate = baudRates[br];
    if (HAL_UART_Init(&huart5) != HAL_OK) {
      continue;
    }

    // Clear ring buffer
    extern volatile uint32_t UartRxHead;
    extern volatile uint32_t UartRxTail;
    UartRxHead = 0;
    UartRxTail = 0;

    ESP_Delay(100);

    // Send AT command
    ESP8266_SendCommand("AT");

    // Wait for "OK" response
    if (ESP8266_WaitFor("OK", 1000) == ESP8266_OK) {
      // Disable echo
      ESP_Delay(50);
      ESP8266_SendCommand("ATE0");
      ESP8266_WaitFor("OK", 500);
      return ESP8266_OK;
    }

    ESP_Delay(50);
  }

  return ESP8266_ERROR;
}

ESP8266_StatusTypeDef ESP8266_SendCommand(const char *cmd) {
  // UART Transmit logic - simple and non-blocking
  uint8_t buf[256];
  int len = snprintf((char *)buf, sizeof(buf), "%s\r\n", cmd);

  // Send the complete command using thread-safe wrapper
  // This prevents conflict with USB CDC which also uses UART5
  UART5_Transmit(buf, len, 1000);

  // Trigger LED activity indicator
  ledEspActivity = 1;

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

  while (HAL_GetTick() - start < timeout) {
    if (UART_Read(&ch, 1) > 0) {
      // Pattern matching
      if (ch == pattern[idx]) {
        idx++;
        if (idx == patternLen) {
          return ESP8266_OK;
        }
      } else {
        // Reset if mismatch
        if (ch == pattern[0])
          idx = 1;
        else
          idx = 0;
      }
    } else {
      ESP_Delay(1);
    }
  }
  return ESP8266_TIMEOUT;
}

/* WiFi Connection Functions */

/**
 * @brief Connect to WiFi Access Point
 * @param ssid WiFi network name
 * @param password WiFi password
 * @return ESP8266_OK on success, ESP8266_TIMEOUT on failure
 */
ESP8266_StatusTypeDef ESP8266_Connect(const char *ssid, const char *password) {
  char cmd[256];

  // Step 1: Set WiFi mode to Station (mode 1)
  ESP8266_SendCommand("AT+CWMODE=1");
  ESP8266_StatusTypeDef status = ESP8266_WaitFor("OK", 2000);
  if (status != ESP8266_OK) {
    return ESP8266_ERROR;
  }
  ESP_Delay(100);

  // Step 2: Connect to WiFi Access Point
  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
  ESP8266_SendCommand(cmd);

  // Wait for connection - can take up to 20 seconds
  status = ESP8266_WaitFor("OK", 20000);
  return status;
}

/**
 * @brief Disconnect from WiFi Access Point
 * @return ESP8266_OK on success
 */
ESP8266_StatusTypeDef ESP8266_Disconnect(void) {
  ESP8266_SendCommand("AT+CWJAP=\"\"");
  return ESP8266_WaitFor("OK", 5000);
}

/**
 * @brief Check if connected to WiFi Access Point
 * @return ESP8266_OK if connected, ESP8266_ERROR if not connected
 */
ESP8266_StatusTypeDef ESP8266_CheckConnection(void) {
  // Use GetIP as a reliable connection check
  // If we can get an IP address, we're connected
  char ip[32];
  ESP8266_StatusTypeDef result = ESP8266_GetIP(ip, sizeof(ip));

  // If GetIP succeeds and returns a valid IP, we're connected
  if (result == ESP8266_OK && strlen(ip) > 0) {
    return ESP8266_OK;
  }
  return ESP8266_ERROR;
}

/**
 * @brief Get current IP address
 * @param ipBuffer Buffer to store IP address string
 * @param bufferSize Size of buffer
 * @return ESP8266_OK on success, ESP8266_ERROR on failure
 */
ESP8266_StatusTypeDef ESP8266_GetIP(char *ipBuffer, uint16_t bufferSize) {
  char response[512];
  char *start, *end;
  uint32_t start_time;
  extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);

  if (ipBuffer == NULL || bufferSize < 16) {
    return ESP8266_ERROR;
  }

  // Clear the buffer first by reading any pending data
  uint8_t ch;
  while (UART_Read(&ch, 1) > 0) {
    ESP_Delay(1);
  }

  ESP8266_SendCommand("AT+CIFSR");

  // Wait for response and collect it
  start_time = HAL_GetTick();
  uint16_t idx = 0;

  while (HAL_GetTick() - start_time < 5000 && idx < sizeof(response) - 1) {
    if (UART_Read(&ch, 1) > 0) {
      response[idx++] = ch;
    } else {
      ESP_Delay(1);
    }
  }
  response[idx] = '\0';

  // Parse for STAIP: +CIFSR:STAIP,"192.168.1.154"
  // Also try: +CIFSR:STAIP,"192.168.1.xxx"
  start = strstr(response, "+CIFSR:STAIP,\"");
  if (start == NULL) {
    // Try alternate format
    start = strstr(response, "STAIP,\"");
  }
  if (start == NULL) {
    return ESP8266_ERROR;
  }

  // Find the quote after STAIP,"
  start = strchr(start, '"');
  if (start == NULL) {
    return ESP8266_ERROR;
  }
  start++; // Skip the quote

  end = strchr(start, '"');
  if (end == NULL) {
    return ESP8266_ERROR;
  }

  uint16_t ipLen = end - start;
  if (ipLen >= bufferSize) {
    ipLen = bufferSize - 1;
  }

  strncpy(ipBuffer, start, ipLen);
  ipBuffer[ipLen] = '\0';

  return ESP8266_OK;
}

/**
 * @brief Make HTTP GET request using TCP connection
 * @param url URL to request (e.g., OpenWeatherMap API)
 * @param response Buffer to store response body
 * @param maxSize Maximum size of response buffer
 * @return ESP8266_OK on success, ESP8266_TIMEOUT on timeout
 */
// Static buffers to reduce stack usage (DisplayTask has limited stack)
static char httpReqBuf[512];
static char connectBuf[256];
static char debugMsgBuf[256];

ESP8266_StatusTypeDef ESP8266_HTTPGet(const char *url, char *response, uint16_t maxSize) {
  char cmd[256];  // Reduced from 512 - only used for AT commands
  char host[64];
  char path[128];
  char *start, *end;
  uint32_t start_time;
  uint16_t idx = 0;
  uint8_t ch;
  uint16_t port = 80;
  ESP8266_StatusTypeDef result = ESP8266_ERROR;  // Default result

  // Disable debug forwarding during HTTP request to prevent race condition
  extern volatile uint8_t debugForwardingEnabled;
  uint8_t prevDebugState = debugForwardingEnabled;
  debugForwardingEnabled = 0;
  osDelay(50);  // Let DebugTask finish any pending work

  // Use goto for cleanup - ensures debug forwarding is always re-enabled
  if (response == NULL || maxSize == 0) {
    goto cleanup;
  }

  response[0] = '\0';

  // Clear any pending data
  while (UART_Read(&ch, 1) > 0) {
    ESP_Delay(1);
  }

  // Close any existing connection first to avoid stale connection issues
  ESP8266_SendCommand("AT+CIPCLOSE");

  // Wait for close to complete - wait for either "ERROR" (no connection) or "CLOSED" / "OK"
  start_time = HAL_GetTick();
  uint8_t closeComplete = 0;
  char closeBuf[32];
  uint16_t closeIdx = 0;
  closeBuf[0] = '\0';

  while (HAL_GetTick() - start_time < 2000 && !closeComplete) {
    if (UART_Read(&ch, 1) > 0) {
      // Just consume data - we're clearing the buffer
      // Look for completion indicators
      if (closeIdx < sizeof(closeBuf) - 1) {
        closeBuf[closeIdx++] = ch;
        closeBuf[closeIdx] = '\0';
        if (strstr(closeBuf, "ERROR") != NULL ||
            strstr(closeBuf, "OK") != NULL ||
            strstr(closeBuf, "CLOSED") != NULL ||
            strstr(closeBuf, "link is not valid") != NULL) {
          closeComplete = 1;
        }
      }
    } else {
      ESP_Delay(10);
    }
  }

  // Clear any remaining pending data
  while (UART_Read(&ch, 1) > 0) {
    ESP_Delay(1);
  }

  // Parse URL - extract host and path
  // Format: http://host/path or http://host:port/path
  if (strncmp(url, "http://", 7) != 0) {
    goto cleanup;
  }

  start = (char *)url + 7;
  end = strchr(start, '/');
  if (end == NULL) {
    // No path, use root
    strncpy(host, start, sizeof(host) - 1);
    host[sizeof(host) - 1] = '\0';
    strcpy(path, "/");
  } else {
    // Copy host
    uint16_t hostLen = end - start;
    if (hostLen >= sizeof(host)) hostLen = sizeof(host) - 1;
    strncpy(host, start, hostLen);
    host[hostLen] = '\0';
    strcpy(path, end);
  }

  // Check for port in host
  char *portStr = strchr(host, ':');
  if (portStr != NULL) {
    *portStr = '\0';
    port = atoi(portStr + 1);
  }

  // Start TCP connection
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d", host, port);
  ESP8266_SendCommand(cmd);

  // DEBUG: Output connection attempt
  extern uint8_t CDC_Transmit_ThreadSafe(uint8_t *Buf, uint16_t Len);
  snprintf(debugMsgBuf, sizeof(debugMsgBuf), "\r\n[HTTP] Connecting to %s:%d\r\n", host, port);
  CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));

  // Wait for connection to be established (CONNECT)
  // ESP8266 returns "CONNECT" on success, "CONNECT FAIL" or "ERROR" on failure
  start_time = HAL_GetTick();
  uint8_t connected = 0;
  memset(connectBuf, 0, sizeof(connectBuf));
  uint16_t connectIdx = 0;

  while (HAL_GetTick() - start_time < 10000 && !connected) {
    if (UART_Read(&ch, 1) > 0) {
      if (connectIdx < sizeof(connectBuf) - 1) {
        connectBuf[connectIdx++] = ch;
        connectBuf[connectIdx] = '\0';
      }
      // Check for SUCCESS first (before checking for failure)
      // ESP8266 sends "CONNECT" or "ALREADY CONNECTED" on success
      if ((strstr(connectBuf, "\nCONNECT\r\n") != NULL ||
           strstr(connectBuf, "\rCONNECT\r\n") != NULL ||
           strstr(connectBuf, "CONNECT\r\n") != NULL ||
           strstr(connectBuf, "CONNECTED") != NULL ||
           strstr(connectBuf, "ALREADY CONNECTED") != NULL) &&
          !strstr(connectBuf, "CONNECT FAIL")) {
        connected = 1;
        snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Connected!\r\n");
        CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
      }
      // Check for FAILURE (but make sure it's not just "CONNECT")
      if (strstr(connectBuf, "ERROR") != NULL ||
          (strstr(connectBuf, "CONNECT FAIL") != NULL) ||
          (strstr(connectBuf, "CLOSED") != NULL && !strstr(connectBuf, "CONNECT\r"))) {
        snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Connection failed: %.30s\r\n", connectBuf);
        CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
        result = ESP8266_ERROR;
        goto cleanup;
      }
    } else {
      ESP_Delay(10);
    }
  }

  if (!connected) {
    snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Connection timeout: %.30s\r\n", connectBuf);
    CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
    result = ESP8266_TIMEOUT;
    goto cleanup;
  }

  osDelay(100); // Small delay after connection

  // Build HTTP GET request (use static buffer)
  memset(httpReqBuf, 0, sizeof(httpReqBuf));
  snprintf(httpReqBuf, sizeof(httpReqBuf),
           "GET %s HTTP/1.1\r\n"
           "Host: %s\r\n"
           "User-Agent: STM32F769/1.0\r\n"
           "Connection: close\r\n"
           "\r\n",
           path, host);

  // Send data length
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", (int)strlen(httpReqBuf));
  ESP8266_SendCommand(cmd);

  snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Sending %d bytes...\r\n", (int)strlen(httpReqBuf));
  CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));

  // Wait for ">" prompt
  start_time = HAL_GetTick();
  uint8_t gotPrompt = 0;
  char promptBuf[64];
  uint16_t promptIdx = 0;
  promptBuf[0] = '\0';

  while (HAL_GetTick() - start_time < 2000 && !gotPrompt) {
    if (UART_Read(&ch, 1) > 0) {
      if (promptIdx < sizeof(promptBuf) - 1) {
        promptBuf[promptIdx++] = ch;
        promptBuf[promptIdx] = '\0';
      }
      if (ch == '>') {
        gotPrompt = 1;
        snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Got prompt\r\n");
        CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
      }
      if (strstr(promptBuf, "ERROR") != NULL) {
        snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] ERROR on CIPSEND: %s\r\n", promptBuf);
        CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
        result = ESP8266_ERROR;
        goto cleanup;
      }
    } else {
      ESP_Delay(1);
    }
  }

  if (!gotPrompt) {
    snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] No prompt, got: %s\r\n", promptBuf);
    CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
    result = ESP8266_TIMEOUT;
    goto cleanup;
  }

  // Send the HTTP request
  ESP8266_SendCommand(httpReqBuf);

  // Wait for response and collect data
  // Note: ESP8266 may output control messages ("busy s...", "Recv X bytes", "SEND OK")
  // before the actual HTTP response. We need to find "HTTP/1" to locate the real response.
  start_time = HAL_GetTick();
  idx = 0;
  uint8_t foundHttpStart = 0;
  uint8_t foundBody = 0;

  // Buffer to hold data until we find the HTTP response start
  char scanBuf[128];
  uint16_t scanIdx = 0;

  while (HAL_GetTick() - start_time < 10000 && idx < maxSize - 1) {
    if (UART_Read(&ch, 1) > 0) {
      if (!foundHttpStart) {
        // Look for "HTTP/1" to find the start of the actual HTTP response
        scanBuf[scanIdx++] = ch;
        if (scanIdx >= sizeof(scanBuf) - 1) {
          // Shift buffer if full
          memmove(scanBuf, scanBuf + 1, sizeof(scanBuf) - 1);
          scanIdx--;
        }
        scanBuf[scanIdx] = '\0';

        // Check for "HTTP/1" in the scan buffer
        if (strstr(scanBuf, "HTTP/1") != NULL) {
          foundHttpStart = 1;
          // Find where "HTTP/1" starts in the buffer
          char *httpStart = strstr(scanBuf, "HTTP/1");
          if (httpStart != NULL) {
            // Copy from "HTTP/1" onwards to response
            uint16_t httpLen = scanIdx - (httpStart - scanBuf);
            if (idx + httpLen < maxSize - 1) {
              memcpy(response + idx, httpStart, httpLen);
              idx += httpLen;
            }
          }
        }
      } else if (!foundBody) {
        // Look for end of headers (double CRLF = "\r\n\r\n")
        response[idx++] = ch;
        if (idx >= 4) {
          // Check for "\r\n\r\n" or "\n\n"
          if ((response[idx-4] == '\r' && response[idx-3] == '\n' &&
               response[idx-2] == '\r' && response[idx-1] == '\n') ||
              (response[idx-3] == '\n' && response[idx-2] == '\n')) {
            foundBody = 1;
            // FIX v0.1.47: Simply mark that headers are done
            // Body data will be collected below (else if branch)
          }
        }
      } else {
        // Collect body data
        // Stop at connection closed message or end of JSON (closing brace)
        if (idx > 0 && ch == '\r' && response[idx-1] == '\n') {
          // Check if next bytes are "CLOSED" or "OK"
          char peekBuf[32];
          uint16_t peekIdx = 0;
          uint8_t peekCh;
          uint32_t peekStart = HAL_GetTick();
          while (HAL_GetTick() - peekStart < 100 && peekIdx < sizeof(peekBuf) - 1) {
            if (UART_Read(&peekCh, 1) > 0) {
              peekBuf[peekIdx++] = peekCh;
            } else {
              ESP_Delay(1);
            }
          }
          peekBuf[peekIdx] = '\0';
          // FIX v0.1.45: Check for JSON completion first (closing brace)
          // OpenWeatherMap JSON ends with "}{"cod":200}" - the closing brace signals end of data
          // After that, we might get "CLOSED" or "ERROR" from ESP8266
          int jsonComplete = 0;
          for (uint16_t i = 0; i < peekIdx; i++) {
            if (peekBuf[i] == '}') {
              jsonComplete = 1;
              break;
            }
          }
          if (jsonComplete || strstr(peekBuf, "CLOSED") != NULL || strstr(peekBuf, "ERROR") != NULL) {
            break;
          }
          // Not closed, add the peeked data to response (excluding "CLOSED"/"ERROR")
          uint16_t dataLen = peekIdx;
          if (strstr(peekBuf, "CLOSED") != NULL) {
            dataLen = strstr(peekBuf, "CLOSED") - peekBuf;
          } else if (strstr(peekBuf, "ERROR") != NULL) {
            dataLen = strstr(peekBuf, "ERROR") - peekBuf;
          }
          for (uint16_t i = 0; i < dataLen && idx < maxSize - 1; i++) {
            response[idx++] = peekBuf[i];
          }
          continue;
        }
        response[idx++] = ch;
      }
    } else {
      ESP_Delay(1);
    }
  }
  response[idx] = '\0';

  // Extract body from response (after headers)
  char *bodyStart = strstr(response, "\r\n\r\n");
  if (bodyStart != NULL) {
    bodyStart += 4;  // Skip "\r\n\r\n"
    uint16_t bodyLen = idx - (bodyStart - response);
    if (bodyLen > 0) {
      memmove(response, bodyStart, bodyLen);
      response[bodyLen] = '\0';
      idx = bodyLen;
    }
  }

  // FIX v0.1.48: Improved JSON extraction
  // Find first '{' and its matching '}' to extract clean JSON
  // This handles ESP8266 "CLOSED" message that appears after JSON
  char *jsonStart = strchr(response, '{');
  if (jsonStart != NULL) {
    // Find the closing brace for this JSON (matching brace count)
    char *p = jsonStart;
    int braceCount = 1;  // Already found opening brace
    char *jsonEnd = NULL;

    while (*p && braceCount > 0) {
      if (*p == '{') braceCount++;
      else if (*p == '}') {
        braceCount--;
        if (braceCount == 0) {
          jsonEnd = p;  // Found matching closing brace
          break;
        }
      }
      p++;
    }

    if (jsonEnd != NULL && jsonEnd > jsonStart) {
      uint16_t jsonLen = jsonEnd - jsonStart + 1;  // +1 to include '}'
      if (jsonLen < idx) {
        memmove(response, jsonStart, jsonLen);
        response[jsonLen] = '\0';
        idx = jsonLen;
      }
    }
  }

  // Check for HTTP 301/302 redirect - extract Location header
  char *locationHeader = strstr(response, "Location:");
  if (locationHeader != NULL && (strstr(response, "301") != NULL || strstr(response, "302") != NULL)) {
    // Extract the redirect URL
    char *urlStart = strchr(locationHeader, ' ');
    if (urlStart != NULL) {
      urlStart++;  // Skip space
      char *urlEnd = strchr(urlStart, '\r');
      if (urlEnd == NULL) urlEnd = strchr(urlStart, '\n');
      if (urlEnd != NULL) {
        // Build new URL from redirect location
        char newUrl[128];
        uint16_t urlLen = urlEnd - urlStart;
        if (urlLen < sizeof(newUrl) - 1) {
          strncpy(newUrl, urlStart, urlLen);
          newUrl[urlLen] = '\0';

          // DEBUG: Show redirect
          snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] Following redirect to: %s\r\n", newUrl);
          CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));

          // Close current connection
          ESP8266_SendCommand("AT+CIPCLOSE");
          osDelay(100);

          // Follow redirect (only if it starts with http://, not https://)
          if (strncmp(newUrl, "http://", 7) == 0) {
            result = ESP8266_HTTPGet(newUrl, response, maxSize);
            goto cleanup;
          } else {
            // HTTPS redirect - cannot follow with ESP8266
            snprintf(debugMsgBuf, sizeof(debugMsgBuf), "[HTTP] HTTPS redirect not supported, using fallback\r\n");
            CDC_Transmit_ThreadSafe((uint8_t *)debugMsgBuf, strlen(debugMsgBuf));
            // Use a simple text fallback
            snprintf(response, maxSize, "☁️ Weather unavailable");
            result = ESP8266_OK;
            goto cleanup;
          }
        }
      }
    }
  }

  // Clean up response - remove trailing newlines and spaces
  while (idx > 0 && (response[idx-1] == '\r' || response[idx-1] == '\n' ||
                     response[idx-1] == ' ' || response[idx-1] == '\t')) {
    response[--idx] = '\0';
  }

  // Close connection
  ESP8266_SendCommand("AT+CIPCLOSE");
  osDelay(100);

  result = (idx > 0) ? ESP8266_OK : ESP8266_TIMEOUT;

cleanup:
  // Close connection if we're exiting due to error (connection might be left open)
  if (result != ESP8266_OK) {
    ESP8266_SendCommand("AT+CIPCLOSE");
    osDelay(50);
  }

  // Re-enable debug forwarding
  debugForwardingEnabled = prevDebugState;

  // If we got an error but have some data, still return error (not OK)
  if (result == ESP8266_OK && idx == 0) {
    result = ESP8266_TIMEOUT;
  }

  return result;
}

/**
 * @brief Synchronize time with NTP server
 * @param ntpServer NTP server hostname (e.g., "pool.ntp.org")
 * @param unixTimestamp Pointer to store Unix timestamp (seconds since 1970)
 * @return ESP8266_OK on success, ESP8266_TIMEOUT on timeout
 */
ESP8266_StatusTypeDef ESP8266_NTPSync(const char *ntpServer, uint32_t *unixTimestamp) {
  char cmd[256];
  uint8_t ch;
  uint32_t start_time;

  if (unixTimestamp == NULL) {
    return ESP8266_ERROR;
  }

  *unixTimestamp = 0;

  // Clear any pending data
  while (UART_Read(&ch, 1) > 0) {
    ESP_Delay(1);
  }

  // Create NTP request packet (48 bytes)
  uint8_t ntpPacket[48];
  memset(ntpPacket, 0, sizeof(ntpPacket));
  ntpPacket[0] = 0x1B;  // LI=0, VN=3, Mode=3 (client)

  // Start UDP connection to NTP server
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",123", ntpServer);
  ESP8266_SendCommand(cmd);

  // Wait for connection (or "already connected")
  start_time = HAL_GetTick();
  char connectBuf[64];
  uint16_t connectIdx = 0;
  uint8_t connected = 0;

  while (HAL_GetTick() - start_time < 5000 && !connected) {
    if (UART_Read(&ch, 1) > 0) {
      if (connectIdx < sizeof(connectBuf) - 1) {
        connectBuf[connectIdx++] = ch;
        connectBuf[connectIdx] = '\0';
      }
      if (strstr(connectBuf, "OK") != NULL || strstr(connectBuf, "CONNECTED") != NULL) {
        connected = 1;
      }
      if (strstr(connectBuf, "ERROR") != NULL) {
        return ESP8266_ERROR;
      }
    } else {
      ESP_Delay(10);
    }
  }

  if (!connected) {
    return ESP8266_TIMEOUT;
  }

  osDelay(50);

  // Send NTP packet
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", (int)sizeof(ntpPacket));
  ESP8266_SendCommand(cmd);

  // Wait for ">" prompt
  start_time = HAL_GetTick();
  uint8_t gotPrompt = 0;
  while (HAL_GetTick() - start_time < 1000 && !gotPrompt) {
    if (UART_Read(&ch, 1) > 0) {
      if (ch == '>') gotPrompt = 1;
    } else {
      ESP_Delay(1);
    }
  }

  if (!gotPrompt) {
    ESP8266_SendCommand("AT+CIPCLOSE");
    return ESP8266_TIMEOUT;
  }

  // Send the NTP packet
  UART5_Transmit(ntpPacket, sizeof(ntpPacket), 1000);

  // Wait for response (+IPD,...)
  start_time = HAL_GetTick();
  uint8_t responseBuf[256];
  uint16_t respIdx = 0;

  while (HAL_GetTick() - start_time < 3000 && respIdx < sizeof(responseBuf) - 1) {
    if (UART_Read(&ch, 1) > 0) {
      responseBuf[respIdx++] = ch;
      responseBuf[respIdx] = '\0';
      // Look for end of response (OK or CLOSED)
      if (strstr((char *)responseBuf, "OK") != NULL ||
          strstr((char *)responseBuf, "CLOSED") != NULL) {
        break;
      }
    } else {
      ESP_Delay(1);
    }
  }

  ESP8266_SendCommand("AT+CIPCLOSE");
  osDelay(50);

  // Parse NTP response - look for +IPD and extract data
  char *ipdStart = strstr((char *)responseBuf, "+IPD");
  if (ipdStart == NULL) {
    return ESP8266_TIMEOUT;
  }

  // Find the colon after +IPD
  char *colon = strchr(ipdStart, ':');
  if (colon == NULL) {
    return ESP8266_ERROR;
  }

  // Data starts after colon
  uint8_t *ntpResponse = (uint8_t *)(colon + 1);

  // Check if we have enough data (need 48 bytes)
  uint16_t available = respIdx - (ntpResponse - responseBuf);
  if (available < 48) {
    return ESP8266_ERROR;
  }

  // Extract transmit timestamp from bytes 40-43 (seconds part)
  uint32_t ntpSeconds = ((uint32_t)ntpResponse[40] << 24) |
                        ((uint32_t)ntpResponse[41] << 16) |
                        ((uint32_t)ntpResponse[42] << 8) |
                        ((uint32_t)ntpResponse[43]);

  // Validate timestamp (should be between 2000 and 2100 in NTP time)
  // NTP time 1900 epoch: year 2000 = ~3155673600, year 2100 = ~4102444800
  if (ntpSeconds < 3155673600UL || ntpSeconds > 4102444800UL) {
    return ESP8266_ERROR;
  }

  // Convert NTP (1900) to Unix (1970) - subtract 70 years (2208988800 seconds)
  *unixTimestamp = ntpSeconds - 2208988800UL;

  return ESP8266_OK;
}

/**
 * @brief Get weather information from OpenWeatherMap API
 * @param response Buffer to store formatted weather string
 * @param maxSize Maximum size of response buffer
 * @return ESP8266_OK on success, ESP8266_ERROR/ESP8266_TIMEOUT on failure
 *
 * OpenWeatherMap API response format (JSON):
 * {
 *   "weather": [{"description": "cloudy"}],
 *   "main": {"temp": 11.5, "feels_like": 10.2, "humidity": 71},
 *   "wind": {"speed": 5.2}
 * }
 *
 * Output format: "Cloudy, 11.5°C, feels like 10.2°C, humidity 71%, wind 5.2 m/s"
 */
ESP8266_StatusTypeDef ESP8266_GetWeather(char *response, uint16_t maxSize) {
  // OpenWeatherMap API configuration
  const char *OWM_HOST = "api.openweathermap.org";
  const char *OWM_API_KEY = "YOUR_OPENWEATHERMAP_API_KEY";  // Get free API key at openweathermap.org
  const char *OWM_CITY = "Your_City";  // Update with your city name
  const char *OWM_UNITS = "metric";  // Use Celsius

  // Build URL: http://api.openweathermap.org/data/2.5/weather?q=Peniche&units=metric&APPID=xxx
  char url[256];
  snprintf(url, sizeof(url),
           "http://%s/data/2.5/weather?q=%s&units=%s&APPID=%s",
           OWM_HOST, OWM_CITY, OWM_UNITS, OWM_API_KEY);

  // Buffer for raw JSON response - increased from 512 to 1024 to handle full weather data
  static char jsonBuffer[1024];

  // Get JSON response from OpenWeatherMap
  ESP8266_StatusTypeDef result = ESP8266_HTTPGet(url, jsonBuffer, sizeof(jsonBuffer) - 1);
  if (result != ESP8266_OK) {
    return result;
  }

  // Parse JSON response
  // Look for: "weather":[{"description":"..."}]
  //          "main":{"temp":...,"feels_like":...,"humidity":...}
  //          "wind":{"speed":...}

  char description[32] = "Unknown";
  float temp = 0;
  float feels_like = 0;
  int humidity = 0;
  float wind_speed = 0;

  // Extract weather description (e.g., "cloudy", "rain")
  char *descStart = strstr(jsonBuffer, "\"description\":\"");
  if (descStart != NULL) {
    descStart += 15;  // Skip "\"description\":\""
    char *descEnd = strchr(descStart, '"');
    if (descEnd != NULL) {
      uint16_t len = descEnd - descStart;
      if (len > sizeof(description) - 1) len = sizeof(description) - 1;
      strncpy(description, descStart, len);
      description[len] = '\0';
      // Capitalize first letter
      if (description[0] >= 'a' && description[0] <= 'z') {
        description[0] -= 32;
      }
    }
  }

  // Extract temperature
  char *tempStart = strstr(jsonBuffer, "\"temp\":");
  if (tempStart != NULL) {
    tempStart += 7;  // Skip "\"temp\":"
    temp = strtof(tempStart, NULL);
  }

  // Extract feels_like temperature - OpenWeatherMap API uses "feels_like"
  // Search for "\"feels_like\":" (13 characters to skip)
  char *feelsStart = strstr(jsonBuffer, "\"feels_like\":");
  if (feelsStart != NULL) {
    feelsStart += 13;  // Skip "\"feels_like\":"
    feels_like = strtof(feelsStart, NULL);
  }

  // Extract humidity - simpler approach: just search for "humidity":
  // The humidity value is in the main object, but we can search the entire buffer
  // since "humidity" appears only once in the relevant context
  char *humStart = strstr(jsonBuffer, "\"humidity\":");
  if (humStart != NULL) {
    humStart += 11;  // Skip "\"humidity\":"
    humidity = (int)strtof(humStart, NULL);
  }

  // Extract wind speed - simpler approach: search for "wind": then "speed":
  char *windObjStart = strstr(jsonBuffer, "\"wind\":");
  if (windObjStart != NULL) {
    // Now search for "speed" after the "wind" marker
    char *windSpeedStart = strstr(windObjStart, "\"speed\":");
    if (windSpeedStart != NULL) {
      windSpeedStart += 8;  // Skip "\"speed\":"
      wind_speed = strtof(windSpeedStart, NULL);
    }
  }

  // Debug: Output raw JSON buffer for diagnostics
  // Uncomment to debug parsing issues:
  // CDC_Transmit_ThreadSafe((uint8_t*)jsonBuffer, strlen(jsonBuffer));

  // Validate parsed values - if all zero except temp, parsing might have failed
  if (feels_like == 0.0f && humidity == 0 && wind_speed == 0.0f) {
    // Try alternate parsing - some OpenWeatherMap responses differ
    // Check if we got a response at all
    if (strstr(jsonBuffer, "\"temp\"") == NULL) {
      // No valid JSON response
      snprintf(response, maxSize, "No data - API error");
      return ESP8266_ERROR;
    }
  }

  // Format response string
  // Example: "Cloudy, 11.5°C, feels like 10.2°C, humidity 71%, wind 5.2 m/s"
  snprintf(response, maxSize,
           "%s, %.1f°C, feels like %.1f°C, humidity %d%%, wind %.1f m/s",
           description, temp, feels_like, humidity, wind_speed);

  return ESP8266_OK;
}
