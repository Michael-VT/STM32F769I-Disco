/**
 ******************************************************************************
 * @file    weather.c
 * @brief   Weather module using ESP8266 and OpenWeatherMap API
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "weather.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
static Weather_Data_t weatherData = {0};
static Weather_State_t weatherState = WEATHER_STATE_IDLE;
static char weatherURL[256];

/**
 * @brief Initialize weather module
 */
void WEATHER_Init(void)
{
    // Build API URL with query parameters
    snprintf(weatherURL, sizeof(weatherURL),
             "GET %s?q=%s&units=%s&APPID=%s HTTP/1.1\r\nHost: %s\r\n"
             "Content-Type: application/x-www-form-urlencoded\r\n"
             "Content-Length: %u\r\nConnection: close\r\n\r\n",
             "/data/2.5/weather", WEATHER_CITY, WEATHER_UNITS, WEATHER_APPID, 0U);

    weatherState = WEATHER_STATE_IDLE;
    weatherData.valid = 0;
}

/**
 * @brief Extract JSON string value (simple version)
 */
static int json_extract(const char *json, const char *key, char *dest, int maxLen)
{
    const char *keyStart = strstr(json, key);
    if (!keyStart) return 0;

    keyStart += strlen(key);

    // Skip whitespace and colon
    while (*keyStart == ' ' || *keyStart == ':' || *keyStart == '\t' || *keyStart == '\n') keyStart++;

    // Check if value is a string (quoted) or number
    if (*keyStart == '\"') {
        keyStart++;
        const char *valueEnd = strchr(keyStart, '\"');
        if (!valueEnd) return 0;
        int len = (int)(valueEnd - keyStart);
        if (len >= maxLen) len = maxLen - 1;
        memcpy(dest, keyStart, len);
        dest[len] = '\0';
        return 1;
    } else {
        // Number value - read until comma or closing brace
        const char *valueEnd = keyStart;
        while (*valueEnd && *valueEnd != ',' && *valueEnd != '}' && *valueEnd != '\n' && *valueEnd != '\r') {
            valueEnd++;
        }
        int len = (int)(valueEnd - keyStart);
        if (len >= maxLen) len = maxLen - 1;
        memcpy(dest, keyStart, len);
        dest[len] = '\0';
        return 1;
    }
}

/**
 * @brief Parse OpenWeatherMap JSON response
 */
static int parseWeatherResponse(const char *json, Weather_Data_t *data)
{
    char tempStr[16], feelsStr[16], humidStr[8], windStr[16];
    const char *jsonStart;

    // Find JSON object start
    jsonStart = strchr(json, '{');
    if (!jsonStart) return 0;

    // Extract weather description from weather array
    // Look for "weather":[{"main":"..."}]
    const char *weatherArray = strstr(json, "\"weather\":[");
    if (weatherArray) {
        const char *mainStart = strstr(weatherArray, "\"main\":\"");
        if (mainStart) {
            mainStart += 8; // Skip "main":"
            const char *mainEnd = strchr(mainStart, '\"');
            if (mainEnd) {
                int len = (int)(mainEnd - mainStart);
                if (len > 31) len = 31;
                memcpy(data->description, mainStart, len);
                data->description[len] = '\0';
            }
        }
    }

    // Extract temperature from "main" object
    const char *mainObj = strstr(json, "\"main\":{");
    if (mainObj) {
        if (json_extract(mainObj, "\"temp\":", tempStr, sizeof(tempStr))) {
            data->temp = atof(tempStr);
        }
        if (json_extract(mainObj, "\"feels_like\":", feelsStr, sizeof(feelsStr))) {
            data->feels_like = atof(feelsStr);
        }
        if (json_extract(mainObj, "\"humidity\":", humidStr, sizeof(humidStr))) {
            data->humidity = atoi(humidStr);
        }
    }

    // Extract wind speed from "wind" object
    const char *windObj = strstr(json, "\"wind\":{");
    if (windObj) {
        if (json_extract(windObj, "\"speed\":", windStr, sizeof(windStr))) {
            data->wind_speed = atof(windStr);
            data->valid = 1;
            return 1;
        }
    }

    data->valid = 0;
    return 0;
}

/**
 * @brief Update weather from OpenWeatherMap API
 */
void WEATHER_Update(void)
{
    ESP8266_StatusTypeDef httpResult;
    char responseBuffer[512];

    weatherState = WEATHER_STATE_FETCHING;

    // Send HTTP GET request to ESP8266
    httpResult = ESP8266_HTTPGet(weatherURL, responseBuffer, sizeof(responseBuffer) - 1);

    if (httpResult == ESP8266_OK) {
        // Find JSON body (skip HTTP headers)
        const char *jsonStart = strchr(responseBuffer, '{');
        if (jsonStart) {
            if (parseWeatherResponse(jsonStart, &weatherData)) {
                weatherState = WEATHER_STATE_OK;
            } else {
                weatherState = WEATHER_STATE_ERROR;
                weatherData.valid = 0;
            }
        }
    } else {
        weatherState = WEATHER_STATE_ERROR;
        weatherData.valid = 0;
    }
}

/**
 * @brief Get weather data pointer
 */
Weather_Data_t* WEATHER_GetData(void)
{
    return &weatherData;
}

/**
 * @brief Get current weather state
 */
Weather_State_t WEATHER_GetState(void)
{
    return weatherState;
}
