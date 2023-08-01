#ifndef INCLUDE_SP140_CONFIG_H_
#define INCLUDE_SP140_CONFIG_H_

#define VERSION_MAJOR         6
#define VERSION_MINOR         2

#define CRUISE_GRACE          1.5  // 1.5 seconds to get off throttle
#define POT_SAFE_LEVEL        0.05 * 4096  // 5% or less

#define DEFAULT_SEA_PRESSURE  1013.25  // millibar


#define ENABLE_BUZ            true  // enable buzzer

#ifdef M0_PIO
  #include "sp140/config-m0.h"      // device config
#else
  #include "sp140/config-rp2040.h"  // device config
#endif

#endif  // INCLUDE_SP140_CONFIG_H_
