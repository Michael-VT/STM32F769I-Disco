
#ifndef __ESP8266_H
#define __ESP8266_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Status Codes */
typedef enum {
  ESP8266_OK = 0,
  ESP8266_ERROR,
  ESP8266_TIMEOUT
} ESP8266_StatusTypeDef;

/* Function Prototypes */
ESP8266_StatusTypeDef ESP8266_Init(void);
ESP8266_StatusTypeDef ESP8266_SendCommand(const char *cmd);
ESP8266_StatusTypeDef ESP8266_Receive(char *buffer, uint16_t size);
ESP8266_StatusTypeDef ESP8266_WaitFor(const char *pattern, uint32_t timeout);

/* WiFi Functions */
ESP8266_StatusTypeDef ESP8266_Connect(const char *ssid, const char *password);
ESP8266_StatusTypeDef ESP8266_Disconnect(void);
ESP8266_StatusTypeDef ESP8266_GetIP(char *ipBuffer, uint16_t bufferSize);
ESP8266_StatusTypeDef ESP8266_HTTPGet(const char *url, char *response, uint16_t maxSize);
ESP8266_StatusTypeDef ESP8266_CheckConnection(void);

/* NTP Time Sync */
ESP8266_StatusTypeDef ESP8266_NTPSync(const char *ntpServer, uint32_t *unixTimestamp);

/* OpenWeatherMap Weather */
ESP8266_StatusTypeDef ESP8266_GetWeather(char *response, uint16_t maxSize);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H */
