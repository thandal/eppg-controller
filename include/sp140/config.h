// Copyright 2021 <Zach Whitehead>
#ifndef SP140_SHARED_CONFIG_H_
#define SP140_SHARED_CONFIG_H_

#define VERSION_MAJOR         6 
#define VERSION_MINOR         0

#define CRUISE_GRACE          1.5  // 1.5 seconds to get off throttle
#define POT_SAFE_LEVEL        0.05 * 4096  // 5% or less

#define DEFAULT_SEA_PRESSURE  1013.25  // millibar

#define ESC_DISARMED_PWM      1010
#define ESC_MIN_PWM           1030  // ESC min is 1050
#define ESC_MAX_PWM           1990  // ESC max 1950
#define ESC_BAUD_RATE         115200
// ESC packets are transmitted about every 20 ms
// 22 bytes at 115200 bps should take about 2 ms
#define ESC_DATA_V2_SIZE      22
#define ESC_TIMEOUT           2 

#define BLACK                 ST77XX_BLACK
#define WHITE                 ST77XX_WHITE
#define GREEN                 ST77XX_GREEN
#define YELLOW                ST77XX_YELLOW
#define RED                   ST77XX_RED
#define BLUE                  ST77XX_BLUE
#define ORANGE                ST77XX_ORANGE
#define CYAN                  ST77XX_CYAN
#define PURPLE                0x780F

#define ENABLE_BUZ            true    // enable buzzer

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

#ifdef M0_PIO
  #include "sp140/config-m0.h"     // device config
#else
  #include "sp140/config-rp2040.h" // device config
#endif

#endif  // SP140_SHARED_CONFIG_H_