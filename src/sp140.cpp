#include "sp140/config.h"
#include "sp140/structs.h"

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

using namespace ace_button;

AceButton button(BUTTON_TOP);
ResponsiveAnalogRead throttlePot(THROTTLE_PIN, false);
CircularBuffer<int, 8> throttlePotBuffer;
Servo escControl;

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
Thread escTelemetryThread = Thread();
Thread webUsbThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &escTelemetryThread, &webUsbThread);

bool armed = false;
bool cruising = false;
unsigned int armedStartMillis = 0;
static STR_DEVICE_DATA_140_V1 deviceData;

//
// Misc utilities
//

int getAvgPot() {
  int avgPot = 0;
  for (decltype(throttlePotBuffer)::index_t i = 0; i < throttlePotBuffer.size(); i++) {
    avgPot += throttlePotBuffer[i];
  }
  if (throttlePotBuffer.size() > 1) avgPot /= throttlePotBuffer.size();
  return avgPot;
}

// Returns true if the throttlePot is above the safe threshold
bool throttleActive() {
  return throttlePot.getValue() > POT_SAFE_LEVEL;
}

void setLEDs(byte state) {
  digitalWrite(LED_SW, state);
}

// Event handler for button presses
void handleButtonEvent(AceButton* /* btn */, uint8_t eventType, uint8_t /* st */) {
  const bool doubleClick = eventType == AceButton::kEventDoubleClicked; 
  const bool longPress = eventType == AceButton::kEventLongPressed; 

  if (doubleClick && armed) {
    // DISARM
    armed = false;
    cruising = false;

    ledBlinkThread.enabled = true;
    vibrateSequence(100);
    buzzerSequence(2093, 1976, 880);

    // Store the new total armed_minutes
    refreshDeviceData(&deviceData);
    const unsigned int armedMillis = millis() - armedStartMillis;
    deviceData.armed_seconds += round(armedMillis / 1000.0);
    writeDeviceData(&deviceData);
    return;
  }
  if (doubleClick && !armed && !throttleActive()) {
    // ARM
    // Don't allow immediate rearming
    const unsigned int currentMillis = millis();
    if (currentMillis - armedStartMillis < 2000) {  
      return;
    }

    armed = true;
    armedStartMillis = currentMillis;

    ledBlinkThread.enabled = false;
    setLEDs(HIGH);
    vibrateSequence(70, 33);
    buzzerSequence(1760, 1976, 2093);
    return;
  }
  if (longPress && armed && !cruising && throttleActive()) {
    cruising = true;
    vibrateNotify();
    buzzerSequence(900, 900);
    return;
  }
  if (longPress && !cruising && !throttleActive()) {
    // Toggle the mode: 0=CHILL, 1=SPORT
    deviceData.performance_mode = (deviceData.performance_mode == 0) ? 1 : 0;
    writeDeviceData(&deviceData);

    buzzerSequence(900, 1976);
    return;
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

//
// Thread callbacks
//

void throttleThreadCallback() {
  // We need to consistently call throttlePot.update().
  // This should be the only place it is called!
  throttlePot.update(); 

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
    throttlePotBuffer.push(throttlePot.getValue());
  }
  const int avgPot = getAvgPot();
  const int maxPWM = (deviceData.performance_mode == 0) ? 1850 : ESC_MAX_PWM;
  const int throttlePWM = map(avgPot, 0, 4095, ESC_MIN_PWM, maxPWM);
  escControl.writeMicroseconds(throttlePWM);
}

void escTelemetryThreadCallback() {
  updateEscTelemetry();
  static unsigned int lastEscStaleWarningMillis = 0;
  const unsigned int nowMillis = millis();
  if (nowMillis - getEscTelemetry().lastUpdateMillis > 2000) { // Alert if no fresh ESC telemetry
      if (nowMillis - lastEscStaleWarningMillis > 2000) {  // Alert every 2 seconds
        lastEscStaleWarningMillis = nowMillis;
        vibrateNotify();
        buzzerSequence(1000, 1000);
      }
  }
}

void buttonThreadCallback() {
  button.check();
}

void ledBlinkThreadCallback() {
  setLEDs(!digitalRead(LED_SW));
}

void displayThreadCallback() {
  // Set the ground altitude if not armed (assumes we're on the ground).
  const float altitude = getAltitude(deviceData, !armed);  
  updateDisplay(
    deviceData, getEscTelemetry(), altitude, armed, cruising, armedStartMillis);
}

void webUsbLineStateCallback(bool connected) {
  digitalWrite(LED_SW, connected);
  if (connected) sendWebUsbSerial(deviceData);
}

void webUsbThreadCallback() {
  if (!armed && parseWebUsbSerial(&deviceData)) {
    writeDeviceData(&deviceData);
    resetRotation(deviceData.screen_rotation);  // Screen orientation may have changed
    sendWebUsbSerial(deviceData);
  }
}

//
// Arduino setup/main functions
//

// The setup function runs once when you press reset or power the board.
void setup() {
  Serial.begin(115200);  // For debug

  // Set up the throttle
  analogReadResolution(12);     // M0 family chip provides 12bit resolution. TODO: necessary given the next line?
  throttlePot.setAnalogResolution(4096);

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
  setupDisplay(deviceData);
  delay(2000);  // Let the startup screen show for 2 s
  setupWatchdog();

  ledBlinkThread.onRun(ledBlinkThreadCallback);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(displayThreadCallback);
  displayThread.setInterval(250);

  buttonThread.onRun(buttonThreadCallback);
  buttonThread.setInterval(50);

  throttleThread.onRun(throttleThreadCallback);
  throttleThread.setInterval(22);

  escTelemetryThread.onRun(escTelemetryThreadCallback);
  escTelemetryThread.setInterval(15);

  webUsbThread.onRun(webUsbThreadCallback);
  webUsbThread.setInterval(50);
}

// Main loop
void loop() {
  resetWatchdog();
  threads.run();
}

#ifdef RP_PIO
// Set up the second core. Nothing to do for now.
void setup1() {}

// Main loop on the second core of the RP2040
// Play notes using delay, which doesn't block the first core.
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
