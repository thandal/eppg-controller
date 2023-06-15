#include "sp140/config.h"
#include "sp140/structs.h"
#include "sp140/utilities.h"

#include "sp140/altimeter.h"
#include "sp140/device_data.h"
#include "sp140/display.h"
#include "sp140/vibrate.h"
#include "sp140/watchdog.h"

#include <AceButton.h>           // button clicks
#include <ArduinoJson.h>
#include <CircularBuffer.h>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Servo.h>               // to control ESCs
#include <StaticThreadController.h>
#include <Thread.h>   // run tasks at different intervals
#include <Wire.h>

#ifdef USE_TINYUSB
  #include "Adafruit_TinyUSB.h"
#endif

// Hardware-specific libraries
#ifdef RP_PIO
  #include "pico/unique_id.h"
#endif

byte escData[ESC_DATA_SIZE];
byte escDataV2[ESC_DATA_V2_SIZE];
unsigned long cruisedAtMilis = 0;
unsigned long transmitted = 0;
unsigned long failed = 0;
bool cruising = false;
int prevPotLvl = 0;
int cruisedPotVal = 0;
float throttlePWM = 0;
float batteryPercent = 0;
float prevBatteryPercent = 0;
bool throttledFlag = true;
bool throttled = false;
unsigned long throttledAtMillis = 0;
unsigned int throttleSecs = 0;

//float minutes = 0;
//float seconds = 0;
//float hours = 0;  // logged flight hours

float wattHoursUsed = 0;


uint16_t _volts = 0;
uint16_t _temperatureC = 0;
int16_t _amps = 0;
uint32_t _eRPM = 0;
uint16_t _inPWM = 0;
uint16_t _outPWM = 0;

// ESC Telemetry
float watts = 0;

Servo esc;  // Creating a servo class with name of esc


///uint16_t bottom_bg_color = DEFAULT_BG_COLOR;


using namespace ace_button;


static STR_DEVICE_DATA_140_V1 deviceData;

static STR_ESC_TELEMETRY_140 telemetryData;
static telem_t raw_telemdata;


// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button(BUTTON_TOP);
ButtonConfig* buttonConfig = button.getButtonConfig();

CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
Thread telemetryThread = Thread();
Thread counterThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &telemetryThread, &counterThread);

bool armed = false;
uint32_t armedAtMilis = 0;
uint32_t cruisedAtMilisMilis = 0;
unsigned int armedSecs = 0;

#pragma message "Warning: OpenPPG software is in beta"

/// Utilities

#ifdef M0_PIO
  #define DBL_TAP_PTR ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
  #define DBL_TAP_MAGIC 0xf01669ef  // Randomly selected, adjusted to have first and last bit set
  #define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#endif

// Compute average of the ring buffer for voltage readings
float getBatteryVoltSmoothed() {
  float avg = 0.0;
  using index_t = decltype(voltageBuffer)::index_t;
  for (index_t i = 0; i < voltageBuffer.size(); i++) {
    avg += voltageBuffer[i] / voltageBuffer.size();
  }
  return avg;
}


void setLEDs(byte state) {
  digitalWrite(LED_SW, state);
}

void ledBlinkThreadCallback() {
  setLEDs(!digitalRead(LED_SW));
}

void displayThreadCallback() {
  updateDisplay(deviceData,
    getBatteryVoltSmoothed(),
    telemetryData.amps,
    watts,
    wattHoursUsed,
    getAltitude(deviceData),
    throttleSecs
  );
}


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
  if (!ENABLE_BUZ) { return false; }
  for (int thisNote = 0; thisNote < siz; thisNote++) {
    // quarter note = 1000 / 4, eigth note = 1000/8, etc.
    int noteDuration = 125;
    playNote(melody[thisNote], noteDuration);
  }
  return true;
}

void handleArmFail() {
  uint16_t melody[] = { 820, 640 };
  playMelody(melody, 2);
}

// for debugging
void printDeviceData() {
  Serial.print("version major ");
  Serial.println(deviceData.version_major);
  Serial.print("version minor ");
  Serial.println(deviceData.version_minor);
  Serial.print("armed_time ");
  Serial.println(deviceData.armed_time);
  Serial.print("crc ");
  Serial.println(deviceData.crc);
}

#ifdef M0_PIO
// get chip serial number (for SAMD21)
String chipId() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;

  char id_buf[33];
  sprintf(id_buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
  return String(id_buf);
}
#elif RP_PIO
String chipId() {
  int len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  uint8_t buff[len] = "";
  pico_get_unique_board_id_string((char *)buff, len);
  return String((char *)buff);
}
#endif // M0_PIO/RP_PIO

#ifdef M0_PIO
// reboot/reset controller
void(* resetFunc) (void) = 0;  // declare reset function @ address 0

// sets the magic pointer to trigger a reboot to the bootloader for updating
void rebootBootloader() {
  *DBL_TAP_PTR = DBL_TAP_MAGIC;

  resetFunc();
}

#elif RP_PIO

// reboot/reset controller
void rebootBootloader() {
#ifdef USE_TINYUSB
  TinyUSB_Port_EnterDFU();
#endif
}
#endif

/// Extra-Data



// ** Logic for WebUSB **
void send_usb_serial() {
#ifdef USE_TINYUSB
#ifdef M0_PIO
  const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
  DynamicJsonDocument doc(capacity);

  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;
  doc["arch"] = "SAMD21";
  doc["screen_rot"] = deviceData.screen_rotation;
  doc["armed_time"] = deviceData.armed_time;
  doc["metric_temp"] = deviceData.metric_temp;
  doc["metric_alt"] = deviceData.metric_alt;
  doc["performance_mode"] = deviceData.performance_mode;
  doc["sea_pressure"] = deviceData.sea_pressure;
  doc["device_id"] = chipId();

  char output[256];
  serializeJson(doc, output);
  usb_web.println(output);
#elif RP_PIO
  StaticJsonDocument<256> doc; // <- a little more than 256 bytes in the stack

  doc["mj_v"].set(VERSION_MAJOR);
  doc["mi_v"].set(VERSION_MINOR);
  doc["arch"].set("RP2040");
  doc["scr_rt"].set(deviceData.screen_rotation);
  doc["ar_tme"].set(deviceData.armed_time);
  doc["m_tmp"].set(deviceData.metric_temp);
  doc["m_alt"].set(deviceData.metric_alt);
  doc["prf"].set(deviceData.performance_mode);
  doc["sea_p"].set(deviceData.sea_pressure);
  //doc["id"].set(chipId()); // webusb bug prevents this extra field from being sent

  char output[256];
  serializeJson(doc, output, sizeof(output));
  usb_web.println(output);
  usb_web.flush();
  //Serial.println(chipId());
#endif // M0_PIO/RP_PIO
#endif // USE_TINYUSB
}

void line_state_callback(bool connected) {
  digitalWrite(LED_SW, connected);
  if (connected) send_usb_serial();
}


// customized for sp140
void parse_usb_serial() {
#ifdef USE_TINYUSB
  const size_t capacity = JSON_OBJECT_SIZE(12) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
///    display.fillScreen(DEFAULT_BG_COLOR);
///    display.setCursor(0, 0);
///    display.setTextSize(2);
///    display.println("BL - UF2");
    rebootBootloader();
    return;  // run only the command
  }

  if (doc["major_v"] < 5) return;

  deviceData.screen_rotation = doc["screen_rot"].as<unsigned int>();  // "3/1"
  deviceData.sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  deviceData.metric_temp = doc["metric_temp"];  // true/false
  deviceData.metric_alt = doc["metric_alt"];  // true/false
  deviceData.performance_mode = doc["performance_mode"];  // 0,1
  deviceData.batt_size = doc["batt_size"];  // 4000
  writeDeviceData(&deviceData);
  resetDisplay(deviceData);
  send_usb_serial();
#endif
}



/// Sp140-Helpers

// new v2
int CheckFlectcher16(byte byteBuffer[]) {
    int fCCRC16;
    int i;
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (i = 0; i < 18; i++) //Check only first 18 bytes, skip crc bytes
    {
        c0 = (int)(c0 + ((int)byteBuffer[i])) % 255;
        c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = ( c1 << 8 ) | c0;
    return (int)fCCRC16;
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

// new for v2 ESC telemetry
void handleSerialData(byte buffer[]) {
  // if(sizeof(buffer) != 22) {
  //     Serial.print("wrong size ");
  //     Serial.println(sizeof(buffer));
  //     return; //Ignore malformed packets
  // }

  if (buffer[20] != 255 || buffer[21] != 255) {
    Serial.println("no stop byte");

    return; //Stop byte of 65535 not recieved
  }

  //Check the fletcher checksum
  int checkFletch = CheckFlectcher16(buffer);

  // checksum
  raw_telemdata.CSUM_HI = buffer[19];
  raw_telemdata.CSUM_LO = buffer[18];

  //TODO alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_telemdata.CSUM_HI << 8) + raw_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_telemdata.V_HI = buffer[1];
  raw_telemdata.V_LO = buffer[0];

  float voltage = (raw_telemdata.V_HI << 8 | raw_telemdata.V_LO) / 100.0;
  telemetryData.volts = voltage; //Voltage

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.0; // calibration
  }

  voltageBuffer.push(telemetryData.volts);

  // Temperature
  raw_telemdata.T_HI = buffer[3];
  raw_telemdata.T_LO = buffer[2];

  float rawVal = (float)((raw_telemdata.T_HI << 8) + raw_telemdata.T_LO);

  static int SERIESRESISTOR = 10000;
  static int NOMINAL_RESISTANCE = 10000;
  static int NOMINAL_TEMPERATURE = 25;
  static int BCOEFFICIENT = 3455;

  //convert value to resistance
  float Rntc = (4096 / (float) rawVal) - 1;
  Rntc = SERIESRESISTOR / Rntc;

  // Get the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE; // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celcius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    temperature = 0;
  }

  temperature = (float) trunc(temperature * 100) / 100; // 2 decimal places
  telemetryData.temperatureC = temperature;

  // Current
  _amps = word(buffer[5], buffer[4]);
  telemetryData.amps = _amps / 12.5;

  // Serial.print("amps ");
  // Serial.print(currentAmpsInput);
  // Serial.print(" - ");

  watts = telemetryData.amps * telemetryData.volts;

  // Reserved
  raw_telemdata.R0_HI = buffer[7];
  raw_telemdata.R0_LO = buffer[6];

  // eRPM
  raw_telemdata.RPM0 = buffer[11];
  raw_telemdata.RPM1 = buffer[10];
  raw_telemdata.RPM2 = buffer[9];
  raw_telemdata.RPM3 = buffer[8];

  int poleCount = 62;
  int currentERPM = (int)((raw_telemdata.RPM0 << 24) + (raw_telemdata.RPM1 << 16) + (raw_telemdata.RPM2 << 8) + (raw_telemdata.RPM3 << 0)); //ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  telemetryData.eRPM = currentRPM;

  // Serial.print("RPM ");
  // Serial.print(currentRPM);
  // Serial.print(" - ");

  // Input Duty
  raw_telemdata.DUTYIN_HI = buffer[13];
  raw_telemdata.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((raw_telemdata.DUTYIN_HI << 8) + raw_telemdata.DUTYIN_LO) / 10);
  telemetryData.inPWM = (throttleDuty / 10); //Input throttle

  // Serial.print("throttle ");
  // Serial.print(telemetryData.inPWM);
  // Serial.print(" - ");

  // Motor Duty
  raw_telemdata.MOTORDUTY_HI = buffer[15];
  raw_telemdata.MOTORDUTY_LO = buffer[14];

  //int motorDuty = (int)(((raw_telemdata.MOTORDUTY_HI << 8) + raw_telemdata.MOTORDUTY_LO) / 10);

  // Reserved
  // raw_telemdata.R1 = buffer[17];

  /* Status Flags
  # Bit position in byte indicates flag set, 1 is set, 0 is default
  # Bit 0: Motor Started, set when motor is running as expected
  # Bit 1: Motor Saturation Event, set when saturation detected and power is reduced for desync protection
  # Bit 2: ESC Over temperature event occuring, shut down method as per configuration
  # Bit 3: ESC Overvoltage event occuring, shut down method as per configuration
  # Bit 4: ESC Undervoltage event occuring, shut down method as per configuration
  # Bit 5: Startup error detected, motor stall detected upon trying to start*/
  raw_telemdata.statusFlag = buffer[16];
  telemetryData.statusFlag = raw_telemdata.statusFlag;
  // Serial.print("status ");
  // Serial.print(raw_telemdata.statusFlag, BIN);
  // Serial.print(" - ");
  // Serial.println(" ");
}

// for debugging
void printRawEscData() {
  Serial.print(F("DATA: "));
  for (int i = 0; i < ESC_DATA_V2_SIZE; i++) {
    Serial.print(escDataV2[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}

void handleTelemetry() {
  // Flush the input
  // TODO: why?
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
  SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
  //printRawEscData();
  handleSerialData(escDataV2);
}

// throttle easing function based on threshold/performance mode
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    int limitedThrottle = last + threshold;
    // TODO: cleanup global var use
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  } else if (last - current >= threshold * 2) {  // decelerating too fast. limit
    int limitedThrottle = last - threshold * 2;  // double the decel vs accel
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  }
  prevPotLvl = current;
  return current;
}




// Thread callback
unsigned long prevPwrMillis = 0;

void counterThreadCallback() {
  // Track wattHoursUsed
  const unsigned long currentPwrMillis = millis();
  const float deltaHours = (currentPwrMillis - prevPwrMillis) / 1000.0 / 3600.0;
  prevPwrMillis = currentPwrMillis;
  if (armed) wattHoursUsed += round(watts * deltaHours);

  // Track flight time
  if (!armed) {
    throttledFlag = true;
    throttled = false;
  } else { // armed
    // start the timer when armed and throttle is above the threshold
    if (throttlePWM > 1250 && throttledFlag) {
      throttledAtMillis = millis();
      throttledFlag = false;
      throttled = true;
    }
    if (throttled) {
      throttleSecs = (millis() - throttledAtMillis) / 1000.0;
    } else {
      throttleSecs = 0;
    }
  }
}

void buttonThreadCallback() {
  button.check();
}

// Returns true if the throttle/pot is below the safe threshold
bool throttleSafe() {
  pot.update();
  if (pot.getValue() < POT_SAFE_LEVEL) {
    return true;
  }
  return false;
}

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

    cruisedAtMilis = millis();  // start timer
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

  static int maxPWM = ESC_MAX_PWM;
  pot.update();
  int potRaw = pot.getValue();

  if (cruising) {
    unsigned long cruisingSecs = (millis() - cruisedAtMilis) / 1000;

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






// The setup function runs once when you press reset or power the board.
void setup() {

  Serial.begin(115200);
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);

#ifdef USE_TINYUSB
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
#endif

  pinMode(LED_SW, OUTPUT);      // Set up the LED
  pinMode(BUZZER_PIN, OUTPUT);  // Set up the buzzer

  analogReadResolution(12);     // M0 family chip provides 12bit resolution
  pot.setAnalogResolution(4096);

  ledBlinkThread.onRun(ledBlinkThreadCallback);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(displayThreadCallback);
  displayThread.setInterval(250);

  buttonThread.onRun(buttonThreadCallback);
  buttonThread.setInterval(5);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  telemetryThread.onRun(handleTelemetry);
  telemetryThread.setInterval(50);

  counterThread.onRun(counterThreadCallback);
  counterThread.setInterval(250);

  setupButtons();
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

#ifdef USE_TINYUSB
  if (!armed && usb_web.available()) parse_usb_serial();
#endif

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
