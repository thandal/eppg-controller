#ifndef SP140_DISPLAY_H_
#define SP140_DISPLAY_H_

#include "sp140/structs.h"
#include <Adafruit_ST7735.h>

#define BLACK                 ST77XX_BLACK
#define WHITE                 ST77XX_WHITE
#define GREEN                 ST77XX_GREEN
#define YELLOW                ST77XX_YELLOW
#define RED                   ST77XX_RED
#define BLUE                  ST77XX_BLUE
#define ORANGE                ST77XX_ORANGE
#define CYAN                  ST77XX_CYAN
#define PURPLE                0x780F
#define GRAY                  0xDEFB

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

// Set up the display and show splash screen
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

// Clear screen and reset properties
void resetRotation(unsigned int orientation);

// Show data on screen
void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   float altitude, bool armed, bool cruising,
                   unsigned int armedStartMillis);

#endif  // SP140_DISPLAY_H_