#include "sp140/config.h"
#include "sp140/structs.h"
#include "sp140/utilities.h"

#include "sp140/altimeter.h"
#include "sp140/device_data.h"
#include "sp140/display.h"
#include "sp140/esc_telemetry.h"
#include "sp140/vibrate.h"
#include "sp140/watchdog.h"
#include "sp140/web_usb.h"

#include <AceButton.h>             // button clicks
#include <CircularBuffer.h>        // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Servo.h>                 // to control ESC
#include <StaticThreadController.h>
#include <Thread.h>
#include <Wire.h>

using namespace ace_button;

Servo esc;  // Creating a servo class with name of esc

///uint16_t bottom_bg_color = DEFAULT_BG_COLOR;

static STR_DEVICE_DATA_140_V1 deviceData;

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button(BUTTON_TOP);
ButtonConfig* buttonConfig = button.getButtonConfig();

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
Thread escThread = Thread();
Thread counterThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &escThread, &counterThread);

CircularBuffer<int, 8> potBuffer;
bool cruising = false;
int cruisedPotVal = 0;
float throttlePWM = 0;

bool armed = false;
uint32_t armedAtMilis = 0;
unsigned int armedSecs = 0;


/// Utilities


#ifdef RP_PIO
// non-blocking tone function that uses second core
void playNote(uint16_t freq, uint16_t duration) {
    // fifo uses 32 bit messages, so package up the freq and duration
    STR_NOTE note;
    note.f.freq = freq;
    note.f.duration = duration;
    rp2040.fifo.push_nb(note.data);  // send note to second core via fifo queue
}
#else
// blocking tone function that delays for notes
void playNote(uint16_t note, uint16_t duration) {
  // quarter note = 1000 / 4, eigth note = 1000/8, etc.
  tone(BUZZER_PIN, note);
  delay(duration);  // to distinguish the notes, delay between them
  noTone(BUZZER_PIN);
}
#endif

bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ) return false;
  for (int thisNote = 0; thisNote < siz; thisNote++) {
    // quarter note = 1000 / 4, eigth note = 1000/8, etc.
    int noteDuration = 125;
    playNote(melody[thisNote], noteDuration);
  }
  return true;
}

void handleArmFail() {
  uint16_t melody[] = {820, 640};
  playMelody(melody, 2);
}


void escTelemetryThreadCallback() {
  updateEscTelemetry();
}

// Throttle easing function based on threshold/performance mode
int prevPotLvl = 0;
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    const int limitedT = last + threshold;
    // TODO: cleanup global var use
    prevPotLvl = limitedT;  // save for next time
    return limitedT;
  } else if (last - current >= threshold * 2) {  // decelerating too fast. limit
    const int limitedThrottle = last - threshold * 2;  // double the decel vs accel
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  }
  prevPotLvl = current;
  return current;
}


// Thread callback
bool throttledUp = false;
unsigned long throttledUpStartSecs = 0;
unsigned int throttledUpSecs = 0;

void counterThreadCallback() {

  // Update flight time
  if (!armed) {
    throttledUp = false;
  } else { // armed
    // Start the timer when armed and throttle is above the threshold
    if (throttlePWM > 1250 && !throttledUp) {
      throttledUp = true;
      throttledUpStartSecs = millis() / 1000.0;
    }
    if (throttledUp) {
      throttledUpSecs = millis() / 1000.0 - throttledUpStartSecs;
    } else {
      throttledUpSecs = 0;
    }
  }
}

void buttonThreadCallback() {
  button.check();
}

void setLEDs(byte state) {
  digitalWrite(LED_SW, state);
}

void ledBlinkThreadCallback() {
  setLEDs(!digitalRead(LED_SW));
}

void displayThreadCallback() {
  updateDisplay(deviceData, getEscTelemetry(), getAltitude(deviceData),
                throttledUpSecs);
}


// Returns true if the throttle/pot is below the safe threshold
bool throttleSafe() {
  pot.update();
  if (pot.getValue() < POT_SAFE_LEVEL) {
    return true;
  }
  return false;
}

unsigned long cruiseStartSecs = 0;
void setCruise() {
  // IDEA: fill a "cruise indicator" as long press activate happens
  // or gradually change color from blue to yellow with time
  if (!throttleSafe()) {  // using pot/throttle
    cruisedPotVal = pot.getValue();  // save current throttle val
    cruising = true;
    vibrateNotify();

///    // update display to show cruise
///    display.setCursor(70, 60);
///    display.setTextSize(1);
///    display.setTextColor(RED);
///    display.print(F("CRUISE"));

    uint16_t melody[] = {900, 900};
    playMelody(melody, 2);

///    bottom_bg_color = YELLOW;
///    display.fillRect(0, 93, 160, 40, bottom_bg_color);

    cruiseStartSecs = millis() / 1000.0;  // start timer
  }
}

void removeCruise(bool alert) {
  cruising = false;

  // update bottom bar
///  bottom_bg_color = DEFAULT_BG_COLOR;
///  if (armed) { bottom_bg_color = ARMED_BG_COLOR; }
///  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  // update text status
///  display.setCursor(70, 60);
///  display.setTextSize(1);
///  display.setTextColor(DEFAULT_BG_COLOR);
///  display.print(F("CRUISE"));  // overwrite in bg color to remove
///  display.setTextColor(BLACK);

  if (alert) {
    vibrateNotify();

    if (ENABLE_BUZ) {
      uint16_t melody[] = {500, 500};
      playMelody(melody, 2);
    }
  }
}

// disarm, remove cruise, alert, save updated stats
void disarmSystem() {
  throttlePWM = ESC_DISARMED_PWM;
  esc.writeMicroseconds(ESC_DISARMED_PWM);
  //Serial.println(F("disarmed"));

  // reset smoothing
  potBuffer.clear();
  prevPotLvl = 0;

  u_int16_t disarm_melody[] = { 2093, 1976, 880 };
  unsigned int disarm_vibes[] = { 100, 0 };

  armed = false;
  removeCruise(false);

  ledBlinkThread.enabled = true;
  vibrateSequence(disarm_vibes, 2);
  playMelody(disarm_melody, 3);

///  bottom_bg_color = DEFAULT_BG_COLOR;
///  display.fillRect(0, 93, 160, 40, bottom_bg_color);
///  updateDisplay();

  // update armed_time
  refreshDeviceData(&deviceData);
  deviceData.armed_time += round(armedSecs / 60);  // convert to mins
  writeDeviceData(&deviceData);
  ///delay(1000);  // TODO just disable button thread // dont allow immediate rearming
}


// Get the system ready to fly
bool armSystem() {
  armed = true;
  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  ledBlinkThread.enabled = false;
  armedAtMilis = millis();
  setGroundAltitude(getAltitude(deviceData));

  setLEDs(HIGH);
  unsigned int arm_vibes[] = {70, 33, 0};
  vibrateSequence(arm_vibes, 3);
  uint16_t arm_melody[] = {1760, 1976, 2093};
  playMelody(arm_melody, 3);

///  bottom_bg_color = ARMED_BG_COLOR;
///  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  return true;
}

// Toggle the mode: 0=CHILL, 1=SPORT
void toggleMode() {
  if (deviceData.performance_mode == 0) {
    deviceData.performance_mode = 1;
  } else {
    deviceData.performance_mode = 0;
  }
  writeDeviceData(&deviceData);
  uint16_t melody[] = {900, 1976};
  playMelody(melody, 2);
}

// The event handler for the the buttons
void handleButtonEvent(AceButton* /* btn */, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventDoubleClicked:
    if (armed) {
      disarmSystem();
    } else if (throttleSafe()) {
      armSystem();
    } else {
      handleArmFail();
    }
    break;
  case AceButton::kEventLongPressed:
    if (armed) {
      if (cruising) {
        removeCruise(true);
      } else if (throttleSafe()) {
        toggleMode();
      } else {
        setCruise();
      }
    } else {
      // show stats screen?
    }
    break;
  case AceButton::kEventLongReleased:
    break;
  }
}

// inital button setup and config
void setupButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);
  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2500);
  buttonConfig->setDoubleClickDelay(600);
}


// read throttle and send to hub
// read throttle
void handleThrottle() {
  if (!armed) return;  // safe

  armedSecs = (millis() - armedAtMilis) / 1000;  // update time while armed

  int maxPWM = ESC_MAX_PWM;
  pot.update();
  int potRaw = pot.getValue();

  if (cruising) {
    unsigned long cruisingSecs = millis() / 1000.0 - cruiseStartSecs;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_SAFE_LEVEL) {
      removeCruise(true);  // deactivate cruise
    } else {
      throttlePWM = mapd(cruisedPotVal, 0, 4095, ESC_MIN_PWM, maxPWM);
    }
  } else {
    // no need to save & smooth throttle etc when in cruise mode (& pot == 0)
    potBuffer.push(potRaw);

    int potLvl = 0;
    for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
      potLvl += potBuffer[i] / potBuffer.size();  // avg
    }

  // runs ~40x sec
  // 1000 diff in pwm from 0
  // 1000/6/40
    if (deviceData.performance_mode == 0) {  // chill mode
      potLvl = limitedThrottle(potLvl, prevPotLvl, 50);
      maxPWM = 1850;  // 85% interpolated from 1030 to 1990
    } else {
      potLvl = limitedThrottle(potLvl, prevPotLvl, 120);
      maxPWM = ESC_MAX_PWM;
    }
    // mapping val to min and max pwm
    throttlePWM = mapd(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }
  esc.writeMicroseconds(throttlePWM);  // using val as the signal to esc
}

void webUsbLineStateCallback(bool connected) {
  digitalWrite(LED_SW, connected);
  if (connected) sendWebUsbSerial(deviceData);
}


// The setup function runs once when you press reset or power the board.
void setup() {
  Serial.begin(115200);

  pinMode(LED_SW, OUTPUT);      // Set up the LED
  pinMode(BUZZER_PIN, OUTPUT);  // Set up the buzzer

  analogReadResolution(12);     // M0 family chip provides 12bit resolution
  pot.setAnalogResolution(4096);

  setupWebUsbSerial(webUsbLineStateCallback);

  ledBlinkThread.onRun(ledBlinkThreadCallback);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(displayThreadCallback);
  displayThread.setInterval(250);

  buttonThread.onRun(buttonThreadCallback);
  buttonThread.setInterval(5);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  escThread.onRun(escTelemetryThreadCallback);
  escThread.setInterval(50);

  counterThread.onRun(counterThreadCallback);
  counterThread.setInterval(250);

  setupButtons();
  setupEscTelemetry();
  setupDeviceData();
  setupWatchdog();

  refreshDeviceData(&deviceData);

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  setupAltimeter(deviceData);
  setupVibrate();

  resetWatchdog();  // Necessary? -- might be if the setupDisplay sleep is long enough to fire the watchdog!
  setupDisplay(deviceData);

  // If the button is held down at startup, toggle mode.
  if (button.isPressedRaw()) toggleMode();
}

// Main loop - everything runs in threads
void loop() {
  resetWatchdog();
  if (!armed && parseWebUsbSerial(&deviceData)) {
    writeDeviceData(&deviceData);
    resetDisplay(deviceData);
    sendWebUsbSerial(deviceData);
  }
  threads.run();
}

#ifdef RP_PIO
// Set up the second core. Nothing to do for now.
void setup1() {}

// Main loop on the second core of the RP2040.
void loop1() {
  if (rp2040.fifo.available() > 0) {
    STR_NOTE note;
    note.data = rp2040.fifo.pop();  
    tone(BUZZER_PIN, note.f.freq);
    delay(note.f.duration);
    noTone(BUZZER_PIN);
  }
}
#endif
