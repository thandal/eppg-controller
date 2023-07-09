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
// We set the timeout to 2ms to be very conservative.
#define ESC_DATA_V2_SIZE      22
#define ESC_TIMEOUT           2 

#define ENABLE_BUZ            true    // enable buzzer

#ifdef M0_PIO
  #include "sp140/config-m0.h"     // device config
#else
  #include "sp140/config-rp2040.h" // device config
#endif

#endif  // SP140_SHARED_CONFIG_H_