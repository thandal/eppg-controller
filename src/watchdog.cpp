#include "sp140/watchdog.h"

// Hardware-specific libraries
#ifdef M0_PIO
  #include <Adafruit_SleepyDog.h>  // watchdog
#elif RP_PIO
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#endif

void setupWatchdog() {
  #ifdef M0_PIO
    Watchdog.enable(5000);
  #elif RP_PIO
    watchdog_enable(5000, 1);
  #endif
}

void resetWatchdog() {
  #ifdef M0_PIO
    Watchdog.reset();
  #elif RP_PIO
    watchdog_update();
  #endif
}