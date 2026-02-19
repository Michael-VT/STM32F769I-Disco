/**
 ******************************************************************************
 * @file    weather.h
 * @brief   Weather module using ESP8266 and OpenWeatherMap API
 ******************************************************************************
 */

#ifndef __WEATHER_H
#define __WEATHER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp8266.h"
#include "cmsis_os2.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* OpenWeatherMap API Configuration - matches openweathermap.py */
#define WEATHER_API_HOST      "api.openweathermap.org"
#define WEATHER_API_PORT      80
#define WEATHER_API_PATH     "/data/2.5/weather"

/* WiFi credentials from openweathermap.py */
#define WIFI_SSID     "MEO-EDC8ED"
#define WIFI_PASSWORD "2668EB941B"
#define WEATHER_CITY  "Peniche"
#define WEATHER_UNITS "metric"
#define WEATHER_APPID  "f8af9ff257bfdb3ad6b6640c0325ad5b"  // Your API key from openweathermap.py

/* Weather data structure */
typedef struct {
    char description[32];    // Weather description (e.g., "Clear sky")
    float temp;             // Temperature in Celsius
    float feels_like;        // Feels like temperature
    int humidity;           // Humidity percentage
    float wind_speed;        // Wind speed in m/s
    int8_t valid;          // 1 = data valid, 0 = no data
} Weather_Data_t;

/* Weather update states */
typedef enum {
    WEATHER_STATE_IDLE = 0,
    WEATHER_STATE_FETCHING,
    WEATHER_STATE_OK,
    WEATHER_STATE_ERROR
} Weather_State_t;

/* Function Prototypes */
void WEATHER_Init(void);
void WEATHER_Update(void);
Weather_Data_t* WEATHER_GetData(void);
Weather_State_t WEATHER_GetState(void);

#ifdef __cplusplus
}
#endif
#endif /* __WEATHER_H */
