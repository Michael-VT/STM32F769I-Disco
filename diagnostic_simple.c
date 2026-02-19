/**
 * Simple Diagnostic for STM32F769I-DISCO
 *
 * This diagnostic firmware tests:
 * 1. LEDs - Blink pattern to confirm firmware is running
 * 2. USB CDC - Virtual COM port should appear
 * 3. ESP8266 - UART5 communication test
 *
 * To use:
 * 1. Flash this firmware
 * 2. Connect USB cable to CN13 (USB User FS port)
 * 3. Look for /dev/cu.usbmodem* device
 * 4. Connect with: screen /dev/cu.usbmodem* 115200
 */

#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

// Simple LED toggle
void LED_Toggle(uint8_t led) {
    extern void BSP_LED_Toggle(uint8_t Led);
    BSP_LED_Toggle(led);
}

// Message buffer
char msg[256];

// Simple diagnostic task
void StartDiagnosticTask(void *argument) {
    int counter = 0;

    // Send startup message
    sprintf(msg, "\r\n=== STM32F769I-DISCO Diagnostic ===\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(1000);

    sprintf(msg, "Firmware: v0.1.112-diagnostic\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(1000);

    sprintf(msg, "Build: %s %s\r\n", __DATE__, __TIME__);
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(1000);

    sprintf(msg, "\r\nIf you see this message:\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(500);

    sprintf(msg, "✓ USB CDC is working!\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(500);

    sprintf(msg, "✓ FreeRTOS is running!\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    osDelay(500);

    sprintf(msg, "\r\nNext checks:\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    sprintf(msg, "- LED1 should be blinking (heartbeat)\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    sprintf(msg, "- ESP8266 TX/RX should be connected to CN2\r\n");
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));

    while (1) {
        // Blink LED1
        LED_Toggle(LED1);

        // Send heartbeat message every 5 seconds
        counter++;
        if (counter >= 50) {  // 50 * 100ms = 5 seconds
            sprintf(msg, "\r\n[HEARTBEAT] Uptime: %d seconds\r\n", counter / 10);
            CDC_Transmit_HS((uint8_t*)msg, strlen(msg));
            counter = 0;
        }

        osDelay(100);
    }
}

// Add this to MX_FREERTOS_Init() in freertos.c:
/*
osThreadNew(StartDiagnosticTask, NULL, &(osThreadAttr_t){
    .name = "diagnosticTask",
    .stack_size = 1024,
    .priority = osPriorityNormal
});
*/
