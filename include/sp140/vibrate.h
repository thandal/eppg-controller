#ifndef SP140_VIBRATE_H_
#define SP140_VIBRATE_H_

#include <stdint.h>

// Set up the vibration motor and do a startup vibration.
void setupVibrate();

// Do a default notification vibration.
void vibrateNotify();

// Do a sequence of vibrations.
void vibrateSequence(unsigned int sequence[], int len);
void vibrateSequence(uint8_t vibe0, uint8_t vibe1 = 0, uint8_t vibe2 = 0);

#endif  // SP140_VIBRATE_H_