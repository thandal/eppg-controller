#ifndef SP140_BUZZER_H_
#define SP140_BUZZER_H_

#include <stdint.h>

// Set up the buzzer
void setupBuzzer();

// Play a single note
void buzzerNote(uint16_t freq, uint16_t duration);

// Play a sequence of notes
void buzzerSequence(uint16_t sequence[], int millis);
void buzzerSequence(uint16_t freq1, uint16_t freq2 = 0, uint16_t freq3 = 0);

#endif  // SP140_BUZZER_H_