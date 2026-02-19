/**
 * Simple diagnostic firmware for STM32F769I-Discovery
 *
 * Features:
 * - Blink green LED (LED1) at 1Hz
 * - Flash entire screen to PURPLE to distinguish from any previous firmware
 * - Blink blue LED on ESP8266 (WIFI_RST pin toggle)
 *
 * Build: arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
 *        -nostartfiles -TSTM32F769XX_FLASH.ld diagnostic.c -o diagnostic.elf
 * Flash: st-flash write --format binary diagnostic.bin 0x08000000
 */

#include <stdint.h>

// Register definitions (simplified)
#define RCC_BASE     0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x40))
#define RCC_APB2ENR (*(volatile uint32_t*)(RCC_BASE + 0x44))

#define GPIOJ_BASE   0x40022400
#define GPIOJ_MODER  (*(volatile uint32_t*)(GPIOJ_BASE + 0x00))
#define GPIOJ_ODR    (*(volatile uint32_t*)(GPIOJ_BASE + 0x14))

#define GPIOI_BASE   0x40020000
#define GPIOI_MODER  (*(volatile uint32_t*)(GPIOI_BASE + 0x00))
#define GPIOI_ODR    (*(volatile uint32_t*)(GPIOI_BASE + 0x14))

#define LTDC_BASE    0x40016800
#define LTDC_SSR     (*(volatile uint32_t*)(LTDC_BASE + 0x54))  // Shadow Reload
#define LTDC_BPCR    (*(volatile uint32_t*)(LTDC_BASE + 0x3C))  // Back Porch
#define LTDC_AWCR    (*(volatile uint32_t*)(LTDC_BASE + 0x74))  // Auxiliary Window Control
#define LTDC_L1CR    (*(volatile uint32_t*)(LTDC_BASE + 0x84))  // Layer 1 Control
#define LTDC_SRCR    (*(volatile uint32_t*)(LTDC_BASE + 0x24))  // Shadow Reload Control

// Framebuffer in SDRAM
#define SDRAM_BASE   0xC0000000
#define LAYER1_FB    (SDRAM_BASE + 0x000000)  // Layer 1 framebuffer

// Pin definitions
#define LED1_Pin     (1 << 1)   // PI1 (Green LED)
#define WIFI_RST_Pin (1 << 12)  // PJ12 (ESP8266 Reset)

// Simple delay function
static void delay_cycles(uint32_t cycles) {
    volatile uint32_t i;
    for (i = 0; i < cycles; i++) {
        __asm__ volatile ("nop");
    }
}

// Clear framebuffer to purple (magenta) to distinguish from any previous firmware
static void clear_screen_purple(void) {
    // Purple: R=0xFF, G=0x00, B=0xFF in RGB888
    // Convert to ARGB8888: 0xFFFF00FF
    volatile uint32_t *fb = (volatile uint32_t*)LAYER1_FB;
    uint32_t purple = 0xFFFF00FF;

    // Clear enough of the screen to see the color
    // Screen is 800x480 = 384000 pixels
    for (int i = 0; i < 384000; i++) {
        fb[i] = purple;
    }

    // Trigger LTDC reload
    LTDC_SRCR = (1 << 1);  // VBR bit - Vertical Blanking Reload
    delay_cycles(100000);
}

int main(void) {
    // Enable clocks for GPIOI (LED1) and GPIOJ (WIFI_RST)
    RCC_AHB1ENR |= (1 << 8) | (1 << 9);  // Enable GPIOI, GPIOJ

    // Configure LED1 (PI1) as output
    GPIOI_MODER &= ~(3 << (1 * 2));  // Clear mode bits
    GPIOI_MODER |= (1 << (1 * 2));   // Output mode

    // Configure WIFI_RST (PJ12) as output
    GPIOJ_MODER &= ~(3 << (12 * 2));  // Clear mode bits
    GPIOJ_MODER |= (1 << (12 * 2));   // Output mode

    // Initial state: LED ON, ESP8266 out of reset
    GPIOI_ODR |= LED1_Pin;     // LED on
    GPIOJ_ODR |= WIFI_RST_Pin; // ESP8266 out of reset

    // Small delay
    delay_cycles(1000000);

    // Clear screen to purple (this will be very visible!)
    clear_screen_purple();

    // Main loop: blink LED at 1Hz and toggle ESP8266 reset
    while (1) {
        // LED ON, ESP8266 out of reset
        GPIOI_ODR |= LED1_Pin;
        GPIOJ_ODR |= WIFI_RST_Pin;
        delay_cycles(10000000);  // ~250ms at 200MHz

        // LED OFF, ESP8266 in reset
        GPIOI_ODR &= ~LED1_Pin;
        GPIOJ_ODR &= ~WIFI_RST_Pin;
        delay_cycles(10000000);  // ~250ms at 200MHz

        // LED ON, ESP8266 out of reset
        GPIOI_ODR |= LED1_Pin;
        GPIOJ_ODR |= WIFI_RST_Pin;
        delay_cycles(10000000);  // ~250ms at 200MHz

        // LED OFF, ESP8266 in reset
        GPIOI_ODR &= ~LED1_Pin;
        GPIOJ_ODR &= ~WIFI_RST_Pin;
        delay_cycles(30000000);  // ~750ms at 200MHz
    }

    return 0;
}

// Reset handler
void Reset_Handler(void);
__attribute__ ((section(".isr_vector"))) void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)SDRAM_BASE + 0x400), // Initial SP (in SDRAM)
    Reset_Handler
};

void Reset_Handler(void) {
    main();
}

void NMI_Handler(void) { while(1); }
void HardFault_Handler(void) { while(1); }
void MemManage_Handler(void) { while(1); }
void BusFault_Handler(void) { while(1); }
void UsageFault_Handler(void) { while(1); }
