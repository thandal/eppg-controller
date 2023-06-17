#include "sp140/config.h"
#include "sp140/structs.h"

#include <stdint.h>
#include <Arduino.h>

void buzzerNote(uint16_t freq, uint16_t millis) {
#ifdef RP_PIO
  // Non-blocking tone function that uses the rp2040's second core
  STR_NOTE note;
  note.f.freq = freq;
  note.f.duration = millis;
  rp2040.fifo.push_nb(note.data);
#else
// TODO: implement non-blocking buzzer (or use a library like ezBuzzer)
//  // Blocking tone function that delays for notes
//  tone(BUZZER_PIN, freq);
//  delay(millis);  // to distinguish the notes, delay between them
//  noTone(BUZZER_PIN);
#endif
}

void buzzerSequence(uint16_t sequence[], int len) {
  if (!ENABLE_BUZ) return;
  for (int thisNote = 0; thisNote < len; thisNote++) {
    // quarter note = 1000 / 4, eigth note = 1000/8, etc.
    int noteDuration = 125;
    buzzerNote(sequence[thisNote], noteDuration);
  }
}

void buzzerSequence(uint16_t freq1, uint16_t freq2, uint16_t freq3) {
  if (!ENABLE_BUZ) return;
  const int noteDuration = 125;
  buzzerNote(freq1, noteDuration);
  if (freq2 != 0) buzzerNote(freq2, noteDuration);
  if (freq3 != 0) buzzerNote(freq3, noteDuration);
}

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);  // Set up the buzzer
}