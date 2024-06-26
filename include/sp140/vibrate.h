#ifndef INCLUDE_SP140_VIBRATE_H_
#define INCLUDE_SP140_VIBRATE_H_

#include <stdint.h>

// Set up the vibration motor and do a startup vibration.
void setupVibrate();

// Do a default notification vibration.
void vibrateNotify();

// Do a sequence of vibrations.
void vibrateSequence(uint8_t vibe0, uint8_t vibe1 = 0, uint8_t vibe2 = 0);

#endif  // INCLUDE_SP140_VIBRATE_H_
