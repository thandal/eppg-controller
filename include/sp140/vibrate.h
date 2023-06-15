#ifndef SP140_VIBRATE_H_
#define SP140_VIBRATE_H_

// Set up the vibration motor and do a startup vibration.
void setupVibrate();

// Do a default notification vibration.
void vibrateNotify();

// Do a sequence of vibrations.
void vibrateSequence(unsigned int sequence[], int len);

#endif  // SP140_VIBRATE_H_