
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
void ESP8266_Init(void);
ESP8266_StatusTypeDef ESP8266_SendCommand(const char *cmd);
ESP8266_StatusTypeDef ESP8266_Receive(char *buffer, uint16_t size);
ESP8266_StatusTypeDef ESP8266_WaitFor(const char *pattern, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H */
