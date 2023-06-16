#include "sp140/config.h"
#include "sp140/structs.h"
#include "sp140/utilities.h"

#include "sp140/altimeter.h"
#include "sp140/buzzer.h"
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
Thread escTelemetryThread = Thread();
Thread counterThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &escTelemetryThread, &counterThread);

CircularBuffer<int, 8> potBuffer;
bool cruising = false;
int cruisedPotVal = 0;
float throttlePWM = 0;

bool armed = false;
uint32_t armedAtMilis = 0;
unsigned int armedSecs = 0;


/// Utilities


void handleArmFail() {
  buzzerSequence(820, 640);
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
  return pot.getValue() < POT_SAFE_LEVEL;
}

unsigned long cruiseStartSecs = 0;
void setCruise() {
  // IDEA: fill a "cruise indicator" as long press activate happens
  // or gradually change color from blue to yellow with time
  if (!throttleSafe()) {  // using pot/throttle
    cruisedPotVal = pot.getValue();  // save current throttle val
    cruising = true;
    vibrateNotify();
    buzzerSequence(900, 900);

///  // displayUpdate should handle the cruise state!
///    // update display to show cruise
///    display.setCursor(70, 60);
///    display.setTextSize(1);
///    display.setTextColor(RED);
///    display.print(F("CRUISE"));


///    bottom_bg_color = YELLOW;
///    display.fillRect(0, 93, 160, 40, bottom_bg_color);

    cruiseStartSecs = millis() / 1000.0;  // start timer
  }
}

void removeCruise(bool alert) {
  cruising = false;

///  // displayUpdate should handle the cruise state!
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
    buzzerSequence(500, 500);
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

  armed = false;
  removeCruise(false);

  ledBlinkThread.enabled = true;

  unsigned int disarm_vibes[] = { 100, 0 };
  vibrateSequence(disarm_vibes, 2);
  buzzerSequence(2093, 1976, 880);

///  // displayUpdate should handle the armed state!
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
  armedAtMilis = millis();

  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  ledBlinkThread.enabled = false;

  setLEDs(HIGH);
  vibrateSequence(70, 33);
  buzzerSequence(1760, 1976, 2093);

///  // displayUpdate should handle the armed state!
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
  buzzerSequence(900, 1976);
}

// The event handler for the the buttons
void handleButtonEvent(AceButton* /* btn */, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventDoubleClicked:
    if (armed) {
      disarmSystem();
    } else if (throttleSafe()) {
      armSystem();
      setGroundAltitude(getAltitude(deviceData));  // Assuming we arm on the ground
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


// Read throttle and send to esc
void throttleThreadCallback() {
  if (!armed) {
    esc.writeMicroseconds(ESC_DISARMED_PWM);
    return;
  }

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

  // Set up the throttle
  analogReadResolution(12);     // M0 family chip provides 12bit resolution. TODO: necessary given the next line?
  pot.setAnalogResolution(4096);

  // Set up the esc control
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  setupBuzzer();
  setupButtons();
  setupEscTelemetry();
  setupDeviceData();
  refreshDeviceData(&deviceData);
  setupAltimeter(deviceData);
  setupVibrate();

  setupWebUsbSerial(webUsbLineStateCallback);
  setupWatchdog();

  ledBlinkThread.onRun(ledBlinkThreadCallback);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(displayThreadCallback);
  displayThread.setInterval(250);

  buttonThread.onRun(buttonThreadCallback);
  buttonThread.setInterval(5);

  throttleThread.onRun(throttleThreadCallback);
  throttleThread.setInterval(22);

  escTelemetryThread.onRun(escTelemetryThreadCallback);
  escTelemetryThread.setInterval(50);

  counterThread.onRun(counterThreadCallback);
  counterThread.setInterval(250);

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
