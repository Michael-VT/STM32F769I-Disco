/**
 ******************************************************************************
 * @file    test_system.h
 * @brief   Test system header for STM32F769I-Discovery
 * @author  STM32F769I-Discovery Project
 * @date    2025
 *
 * @details Test system that can be triggered via ATSTn commands from USB CDC
 *          Provides comprehensive testing of all hardware and software features
 ******************************************************************************
 */

#ifndef __TEST_SYSTEM_H
#define __TEST_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Forward declaration for font structure */
typedef struct _tFont sFONT;

/* Test Status Codes */
typedef enum {
  TEST_OK = 0,
  TEST_ERROR,
  TEST_TIMEOUT,
  TEST_NOT_SUPPORTED
} TestStatus_TypeDef;

/* Test Result Structure */
typedef struct {
  const char *testName;
  TestStatus_TypeDef status;
  const char *details;
} TestResult_TypeDef;

/* Test IDs - ATSTn commands */
typedef enum {
  TEST_ID_ESP_WEATHER = 0,      // ATST0 - ESP8266 weather test
  TEST_ID_DISPLAY = 1,           // ATST1 - Display graphics/text test
  TEST_ID_LEDS = 2,              // ATST2 - LED test
  TEST_ID_RTC = 3,               // ATST3 - RTC time test
  TEST_ID_WIFI = 4,              // ATST4 - WiFi connection test
  TEST_ID_NTP = 5,               // ATST5 - NTP time sync test
  TEST_ID_HTTP = 6,              // ATST6 - HTTP request test
  TEST_ID_AUDIO = 7,             // ATST7 - Audio codec test
  TEST_ID_SDRAM = 8,             // ATST8 - SDRAM test
  TEST_ID_TOUCH = 9,             // ATST9 - Touchscreen test
  TEST_ID_ALL = 10,              // ATST10 - Run all tests
  TEST_ID_HELP = 99,             // ATST? or ATHelp - Show help
  TEST_ID_PANEL_DETECT = 149,     // ATST149 - LCD panel detection diagnostic
  TEST_ID_CHECKERBOARD_50 = 150,   // ATST150 - 50x50 pixel checkerboard test
  TEST_ID_DRAW_POINT = 151,        // ATST151 - Draw point at X,Y with diameter D, color C
  TEST_ID_DRAW_LINE = 152,         // ATST152 - Draw line from current to X,Y
  TEST_ID_DRAW_CIRCLE = 153,       // ATST153 - Draw circle/ellipse at center X,Y with width W, height H
  TEST_ID_DRAW_RECT = 154,         // ATST154 - Draw rectangle at X,Y with width W, height H
  TEST_ID_DRAW_TEXT = 155,         // ATST155 - Draw text at X,Y with font size S, angle L, color C
  TEST_ID_DRAW_GRAPH_AXIS = 156,  // ATST156 - Draw graph axis and grid
  TEST_ID_DRAW_LINE_GRAPH = 157,  // ATST157 - Draw line graph from data points
  TEST_ID_DRAW_BAR_GRAPH = 158,   // ATST158 - Draw bar graph from data
  TEST_ID_DRAWING_TEST = 159,     // ATST159 - Drawing commands test
  TEST_ID_AUDIO_DISPLAY = 160,    // ATST160 - 4-channel audio oscilloscope display
  TEST_ID_AUDIO_LOCALIZATION = 162,  // ATST162 - 4-mic audio with sound source localization
  TEST_ID_MENU_DRAW_BUTTON = 165,   // ATST165 - Draw menu button
  TEST_ID_MENU_DRAW_PROGRESS = 166, // ATST166 - Draw progress bar
  TEST_ID_MENU_DRAW_SLIDER = 167,   // ATST167 - Draw slider
  TEST_ID_MENU_DRAW_LIST = 168,     // ATST168 - Draw list item
  TEST_ID_MENU_DEMO = 169,          // ATST169 - Menu demonstration
  TEST_ID_SCREEN_CLEAR = 170,       // ATST170 - Screen clearing methods
  TEST_ID_AUDIO_FREQ_ANALYSIS = 171, // ATST171 - Frequency analysis (800Hz/1200Hz) with direction
  TEST_ID_SD_INFO = 172,            // ATSTMCRI - MicroSD card information (CID, CSD, capacity)
  TEST_ID_SD_LIST = 173,            // ATSTMCRL - MicroSD card file list (root directory)
  TEST_ID_DRAW_FAST_LINE = 174,     // ATST174 - Fast horizontal/vertical line
  TEST_ID_DRAW_FAST_RECT = 175,     // ATST175 - Fast filled rectangle (optimized)
  TEST_ID_DRAW_PIXEL_BATCH = 176,   // ATST176 - Batch pixel drawing
  TEST_ID_DRAW_SCROLL_AREA = 177,   // ATST177 - Create scrollable area
  TEST_ID_DRAW_FAST_TEXT = 178,     // ATST178 - Fast text rendering (cached)
  TEST_ID_DRAW_WAVEFORM = 179,      // ATST179 - Draw waveform (oscilloscope)
  TEST_ID_DRAW_GRAPH_FAST = 180,    // ATST180 - Fast line graph (optimized)
  TEST_ID_INTERRUPT = 255           // ATST999 - Interrupt/stop any running test
} TestID_TypeDef;

/* Function Prototypes */

/**
 * @brief Process ATSTn command from USB CDC
 * @param testId Test ID (0-10, 99, 141-150, 159, or 255 for help)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_ProcessCommand(uint8_t testId);

/**
 * @brief Process ATSTn command with parameters from USB CDC
 * @param testId Test ID (151, 152, 154 for drawing commands)
 * @param params Parameter string (e.g., "100,200,5,0xFFFF")
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_ProcessCommandWithParams(uint8_t testId, const char *params);

/**
 * @brief Run ESP8266 weather test (ATST0)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_ESP_Weather(void);

/**
 * @brief Run display graphics/text test (ATST1)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Display(void);

/**
 * @brief Run LED test (ATST2)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_LEDs(void);

/**
 * @brief Run RTC time test (ATST3)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_RTC(void);

/**
 * @brief Run WiFi connection test (ATST4)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_WiFi(void);

/**
 * @brief Run NTP time sync test (ATST5)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_NTP(void);

/**
 * @brief Run HTTP request test (ATST6)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_HTTP(void);

/**
 * @brief Run audio codec test (ATST7)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Audio(void);

/**
 * @brief Run SDRAM test (ATST8)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_SDRAM(void);

/**
 * @brief Run touchscreen test (ATST9)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Touch(void);

/**
 * @brief Run all tests (ATST10)
 * @retval TestStatus_TypeDef Test result (returns OK if all tests pass)
 */
TestStatus_TypeDef TEST_RunAll(void);

/**
 * @brief Show test help information
 */
void TEST_ShowHelp(void);

/**
 * @brief Initialize test system
 */
void TEST_Init(void);

/**
 * @brief Parse drawing command parameters (ATST151/152/154)
 * @param cmdParams Command parameters string (e.g., "100,200,5,0xFFFF")
 * @param params Output parameter array [X, Y, D, C] or [X, Y, W, H, F, C]
 * @param paramCount Number of parameters to parse (4 or 6)
 * @retval Number of parameters successfully parsed
 */
int TEST_ParseDrawParams(const char *cmdParams, int32_t *params, int paramCount);

/**
 * @brief Convert hex color string to uint32_t
 * @param hexStr Hex color string (e.g., "0xFF00" or "FF00")
 * @retval Color value as uint32_t
 */
uint32_t TEST_ParseHexColor(const char *hexStr);

/**
 * @brief Set current drawing font
 * @param fontSize Font size identifier (8=Font8, 12=Font12, 16=Font16, 20=Font20, 24=Font24)
 */
void TEST_SetFont(uint8_t fontSize);

/**
 * @brief Get current drawing font
 * @retval Pointer to current font structure
 */
sFONT *TEST_GetFont(void);

/**
 * @brief Rotate coordinates for text drawing
 * @param x Input X coordinate
 * @param y Input Y coordinate
 * @param angle Rotation angle (0=0°, 1=90°, 2=180°, 3=270°)
 * @param cx Center X for rotation
 * @param cy Center Y for rotation
 * @param outX Output X coordinate
 * @param outY Output Y coordinate
 */
void TEST_RotateCoord(int32_t x, int32_t y, uint8_t angle, int32_t cx, int32_t cy,
                      int32_t *outX, int32_t *outY);

/**
 * @brief Draw menu button (ATST165)
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Button width
 * @param height Button height
 * @param text Button text (max 31 chars)
 * @param pressed Button state (0=released, 1=pressed)
 * @param color Button color
 * @param textColor Text color
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Menu_Draw_Button(int32_t x, int32_t y, int32_t width, int32_t height,
                                         const char *text, uint8_t pressed,
                                         uint32_t color, uint32_t textColor);

/**
 * @brief Draw progress bar (ATST166)
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Progress bar width
 * @param height Progress bar height
 * @param percent Progress percentage (0-100)
 * @param color Progress bar color
 * @param bgColor Background color
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Menu_Draw_Progress(int32_t x, int32_t y, int32_t width, int32_t height,
                                           int32_t percent, uint32_t color, uint32_t bgColor);

/**
 * @brief Draw slider (ATST167)
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Slider width
 * @param height Slider height
 * @param value Slider value (0-100)
 * @param color Slider color
 * @param showValue Show value text (0=no, 1=yes)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Menu_Draw_Slider(int32_t x, int32_t y, int32_t width, int32_t height,
                                         int32_t value, uint32_t color, uint8_t showValue);

/**
 * @brief Draw list item (ATST168)
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width List item width
 * @param height List item height
 * @param text Item text (max 63 chars)
 * @param selected Item selected state (0=normal, 1=selected)
 * @param index Item index number
 * @param color Item color
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Menu_Draw_List(int32_t x, int32_t y, int32_t width, int32_t height,
                                       const char *text, uint8_t selected, int32_t index,
                                       uint32_t color);

/**
 * @brief Menu demonstration (ATST169)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Menu_Demo(void);

/**
 * @brief Screen clearing methods (ATST170)
 * @param mode Clear mode (0=black, 1=white, 2=gradient, 3=checkerboard)
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Screen_Clear(uint8_t mode);

/**
 * @brief Audio frequency analysis (ATST171)
 *        Analyzes 800Hz (red) and 1200Hz (blue) frequencies
 *        Displays direction finding, amplitude, frequency
 *        Shows 4 waveform windows on right side of screen
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Audio_Freq_Analysis(void);

/**
 * @brief MicroSD card information (ATSTMCRI)
 *        Reads and displays SD card CID, CSD, capacity, and other information
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_SD_Card_Info(void);

/**
 * @brief MicroSD card file list (ATSTMCRL)
 *        Lists files in the root directory of the SD card
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_SD_Card_List(void);

/**
 * @brief Fast line drawing demo (ATST174)
 *        Demonstrates optimized horizontal/vertical line drawing
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Draw_Fast_Line_Demo(void);

/**
 * @brief Fast rectangle drawing demo (ATST175)
 *        Demonstrates optimized filled rectangle drawing using DMA2D
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Draw_Fast_Rect_Demo(void);

/**
 * @brief Waveform drawing demo (ATST179)
 *        Demonstrates oscilloscope-style waveform rendering
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Draw_Waveform_Demo(void);

/**
 * @brief Fast graph drawing demo (ATST180)
 *        Demonstrates optimized real-time line graph rendering
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Draw_Graph_Fast_Demo(void);

/**
 * @brief Interrupt all tests (ATST999)
 *        Stops any currently running test
 * @retval TestStatus_TypeDef Test result
 */
TestStatus_TypeDef TEST_Interrupt_All(void);

/**
 * @brief Check if test should be interrupted
 * @retval 1 if interrupted, 0 otherwise
 */
int TEST_ShouldInterrupt(void);

/**
 * @brief Set interrupt flag
 * @param state Interrupt state (1=interrupt, 0=clear)
 */
void TEST_SetInterrupt(uint8_t state);

/**
 * @brief Audio input hook for real microphone data (ATST171)
 *        Override this function in your application to provide real audio data
 * @param amp800 Pointer to store 800Hz amplitude (0.0-1.0)
 * @param amp1200 Pointer to store 1200Hz amplitude (0.0-1.0)
 * @param angle Pointer to store source direction angle (0-360 degrees)
 * @param waveforms Array to store waveform data for 4 channels [4][200 samples]
 * @retval 0 = use simulation mode, 1 = real audio mode
 *
 * @note This is a weak function - override it in your application to provide
 *       real audio data from MP34DT01 microphones via I2S/DFSDM
 *
 * @example
 * // In main.c or your application file:
 * uint8_t TEST_GetAudioData(float *amp800, float *amp1200, float *angle, int16_t waveforms[4][200]) {
 *   // Your real audio processing code here
 *   // 1. Read audio samples from I2S/DFSDM
 *   // 2. Perform FFT to detect 800Hz and 1200Hz
 *   // 3. Calculate direction using TDOA
 *   // 4. Fill waveforms with data
 *   // Return 1 to indicate real audio mode
 *   return 1;
 * }
 */
uint8_t TEST_GetAudioData(float *amp800, float *amp1200, float *angle, int16_t waveforms[4][200]);

/**
 * @brief Display test start message on LCD
 * @param testId Test ID (for display name) - uint16_t to support ATST999 (999)
 * @param testName Test name string
 * @note v0.1.85: Displays large white text on dark blue background
 *               Changed to uint16_t to properly display ATST999 (was showing ATST231)
 */
void TEST_DisplayStartMessage(uint16_t testId, const char *testName);

/**
 * @brief Display test completion message on LCD
 * @param testId Test ID (for display name) - uint16_t to support ATST999 (999)
 * @param testName Test name string
 * @param result Test result (0=PASS/OK, 1=FAIL/ERROR, other=TIMEOUT)
 * @note v0.1.85: Displays large text on dark background
 *               Green = PASS, Red = FAIL, Yellow = TIMEOUT
 *               Changed to uint16_t to properly display ATST999
 */
void TEST_DisplayCompleteMessage(uint16_t testId, const char *testName, TestStatus_TypeDef result);

/**
 * @brief Get test name string from test ID
 * @param testId Test ID
 * @retval Test name string (static buffer)
 */
const char *TEST_GetName(uint8_t testId);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_SYSTEM_H */