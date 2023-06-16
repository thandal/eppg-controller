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


static STR_DEVICE_DATA_140_V1 deviceData;

AceButton button(BUTTON_TOP);
ResponsiveAnalogRead pot(THROTTLE_PIN, false);
CircularBuffer<int, 8> potBuffer;
Servo escControl;

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
Thread escTelemetryThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &escTelemetryThread);


bool armed = false;
bool cruising = false;
unsigned long armedStartMillis = 0;

// Utilities

int getAvgPot() {
  int avgPot = 0;
  for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
    avgPot += potBuffer[i];
  }
  if (potBuffer.size() > 1) avgPot /= potBuffer.size();
  return avgPot;
}

// Returns true if the throttle/pot is above the safe threshold
bool throttleActive() {
  return pot.getValue() > POT_SAFE_LEVEL;
}

void setLEDs(byte state) {
  digitalWrite(LED_SW, state);
}

// disarm, remove cruise, alert, save updated stats
void disarmSystem() {
  armed = false;
  cruising = false;
  const unsigned int armedMillis = millis() - armedStartMillis;
  escControl.writeMicroseconds(ESC_DISARMED_PWM);

  ledBlinkThread.enabled = true;
  vibrateSequence(100);
  buzzerSequence(2093, 1976, 880);

  // Update armed_minutes
  refreshDeviceData(&deviceData);
  deviceData.armed_seconds += round(armedMillis / 1000.0);
  writeDeviceData(&deviceData);
}

// Get the system ready to fly
void armSystem() {
  unsigned int currentMillis = millis();
  if (currentMillis - armedStartMillis < 1000) {  // Don't allow immediate rearming
    return;
  }
  armedStartMillis = currentMillis;

  armed = true;
  escControl.writeMicroseconds(ESC_DISARMED_PWM);

  ledBlinkThread.enabled = false;
  setLEDs(HIGH);
  vibrateSequence(70, 33);
  buzzerSequence(1760, 1976, 2093);
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
    } else if (throttleActive()) {
      buzzerSequence(820, 640); // Arm failed: Do not arm if the throttle is active.
    } else {
      armSystem();
    }
    break;
  case AceButton::kEventLongPressed:
    if (armed) {
      if (!cruising && throttleActive()) {
        cruising = true;
        vibrateNotify();
        buzzerSequence(900, 900);
      } else {
        toggleMode();
      }
    } else {
      // show stats screen?
    }
    break;
  case AceButton::kEventLongReleased:
    break;
  }
}

void setupButton() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2500);
  buttonConfig->setDoubleClickDelay(600);
}


// Thread callbacks

void escTelemetryThreadCallback() {
  updateEscTelemetry();
}

void buttonThreadCallback() {
  button.check();
}


void ledBlinkThreadCallback() {
  setLEDs(!digitalRead(LED_SW));
}

void displayThreadCallback() {
  // Assuming we arm on the ground
  const float altitude = getAltitude(deviceData);  
  if (!armed) setGroundAltitude(altitude);
  unsigned int armedSeconds = (millis() - armedStartMillis) / 1000;
  updateDisplay(
    deviceData, getEscTelemetry(), altitude,
    armed, cruising, armedSeconds);
}


// Read throttle and send to esc
void throttleThreadCallback() {
  // We want to always call pot.update() at a regular cadence.
  // This should be the only place it is called!
  pot.update(); 

  if (!armed) {
    escControl.writeMicroseconds(ESC_DISARMED_PWM);
    return;
  }

  static unsigned int cruiseStartMillis = 0;
  if (cruising) {
    if (cruiseStartMillis == 0) cruiseStartMillis = millis();
    unsigned long cruisingSecs = (millis() - cruiseStartMillis) / 1000.0;
    if (cruisingSecs >= CRUISE_GRACE && throttleActive()) {
      cruising = false;
      vibrateNotify();
      buzzerSequence(500, 500);
    }
  } else {
    cruiseStartMillis = 0;
    potBuffer.push(pot.getValue());
  }
  const int avgPot = getAvgPot();
  const int maxPWM = (deviceData.performance_mode == 0) ? 1850 : ESC_MAX_PWM;
  const int throttlePWM = map(avgPot, 0, 4095, ESC_MIN_PWM, maxPWM);
  escControl.writeMicroseconds(throttlePWM);
}

void webUsbLineStateCallback(bool connected) {
  digitalWrite(LED_SW, connected);
  if (connected) sendWebUsbSerial(deviceData);
}


// The setup function runs once when you press reset or power the board.
void setup() {
  Serial.begin(115200);  // For debug

  // Set up the throttle
  analogReadResolution(12);     // M0 family chip provides 12bit resolution. TODO: necessary given the next line?
  pot.setAnalogResolution(4096);

  // Set up the esc control
  escControl.attach(ESC_PIN);
  escControl.writeMicroseconds(ESC_DISARMED_PWM);

  pinMode(LED_SW, OUTPUT);  // Set up the LED
  setupButton();
  setupBuzzer();
  setupEscTelemetry();
  setupDeviceData();
  refreshDeviceData(&deviceData);
  setupAltimeter();
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

  resetWatchdog();  // Necessary? -- might be if the setupDisplay sleep is long enough to fire the watchdog!
  setupDisplay(deviceData);

  // If the button is held down at startup, toggle mode.
  if (button.isPressedRaw()) toggleMode();
}

// Main loop - everything runs in threads
void loop() {
  resetWatchdog();
  threads.run();
  if (!armed && parseWebUsbSerial(&deviceData)) {
    writeDeviceData(&deviceData);
    resetDisplay(deviceData);
    sendWebUsbSerial(deviceData);
  }
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
