#include "sp140/config.h"
#include "sp140/vibrate.h"

#include <Adafruit_DRV2605.h>    // haptic vibration controller

Adafruit_DRV2605 vibe;
bool vibePresent = false;

void vibrateSequence(uint8_t vibe0, uint8_t vibe1, uint8_t vibe2) {
  if (!vibePresent) return;
  int i = 0;
  vibe.setWaveform(i, vibe0);
  if (vibe1 != 0) vibe.setWaveform(++i, vibe1);
  if (vibe2 != 0) vibe.setWaveform(++i, vibe2);
  vibe.setWaveform(++i, 0);
  vibe.go();
}

void vibrateNotify() {
  vibrateSequence(15);
}

// Initialize the vibration motor and vibrate
void setupVibrate() {
  if (!ENABLE_VIB) return;
  if (!vibe.begin()) return;
  vibe.selectLibrary(1);
  vibe.setMode(DRV2605_MODE_INTTRIG);
  vibePresent = true;
  vibrateNotify();  // initial boot vibration
}
