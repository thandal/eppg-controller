// Copyright 2020 <Zach Whitehead>
// OpenPPG

#ifdef M0_PIO
  #include "../inc/sp140/m0-config.h"          // device config
#else
  #include "../inc/sp140/rp2040-config.h"         // device config
#endif

#include "../inc/sp140/structs.h"         // data structs

// Arduino "built-in"
#include <ArduinoJson.h>
#include <Servo.h>               // to control ESCs
#include <SPI.h>
#include <Thread.h>   // run tasks at different intervals
#include <Wire.h>

// Arduino "external"
#include <AceButton.h>           // button clicks
#include <Adafruit_BMP3XX.h>     // barometer
#include <Adafruit_DRV2605.h>    // haptic controller
#include <Adafruit_ST7735.h>     // screen
#include <CircularBuffer.h>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <StaticThreadController.h>
#include <TimeLib.h>  // convert time to hours mins etc

#ifdef USE_TINYUSB
  #include "Adafruit_TinyUSB.h"
#endif

#ifdef M0_PIO
  #include <Adafruit_SleepyDog.h>  // watchdog
  #include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM
#elif RP_PIO
  // rp2040 specific libraries here
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#endif

#include <Fonts/FreeSansBold12pt7b.h>

#include "../inc/sp140/globals.h"  // device config

using namespace ace_button;

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_DRV2605 vibe;

// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
ButtonConfig* buttonConfig = button_top.getButtonConfig();
#ifdef M0_PIO
  extEEPROM eep(kbits_64, 1, 64);
#endif

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
bool use_hub_v2 = true;
int page = 0;
float armAltM = 0;
uint32_t armedAtMilis = 0;
uint32_t cruisedAtMilisMilis = 0;
unsigned int armedSecs = 0;
unsigned int last_throttle = 0;

#pragma message "Warning: OpenPPG software is in beta"

/// Utilities

#define LAST_PAGE 1  // starts at 0

#ifdef M0_PIO
  #define DBL_TAP_PTR ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
  #define DBL_TAP_MAGIC 0xf01669ef  // Randomly selected, adjusted to have first and last bit set
  #define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#endif

// For CRC: Xmodem lookup table 0x1021 poly
static const uint16_t crc16table[] ={
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t crc16(uint8_t *buf, uint32_t size) {
  uint16_t crc = 0;
  for (uint32_t i = 0; i < size; i++)
    crc = (crc << 8) ^ crc16table[buf[i] ^ (crc >> 8)];
  return crc;
}


// Map double values
double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Map voltage to battery percentage, based on a
// simple set of data points from load testing.
float getBatteryPercent(float voltage) {
  float battPercent = 0;
  if (voltage > 94.8) {
    battPercent = mapd(voltage, 94.8, 99.6, 90, 100);
  } else if (voltage > 93.36) {
    battPercent = mapd(voltage, 93.36, 94.8, 80, 90);
  } else if (voltage > 91.68) {
    battPercent = mapd(voltage, 91.68, 93.36, 70, 80);
  } else if (voltage > 89.76) {
    battPercent = mapd(voltage, 89.76, 91.68, 60, 70);
  } else if (voltage > 87.6) {
    battPercent = mapd(voltage, 87.6, 89.76, 50, 60);
  } else if (voltage > 85.2) {
    battPercent = mapd(voltage, 85.2, 87.6, 40, 50);
  } else if (voltage > 82.32) {
    battPercent = mapd(voltage, 82.32, 85.2, 30, 40);
  } else if (voltage > 80.16) {
    battPercent = mapd(voltage, 80.16, 82.32, 20, 30);
  } else if (voltage > 78) {
    battPercent = mapd(voltage, 78, 80.16, 10, 20);
  } else if (voltage > 60.96) {
    battPercent = mapd(voltage, 60.96, 78, 0, 10);
  }
  return constrain(battPercent, 0, 100);
}

/**
 * For digital time display - prints leading 0
 *
 * @param digits number to be converted to a string.
 * @return string `12`, or 07 if `digits` is less than 10.
 */
String convertToDigits(byte digits) {
  String digits_string = "";
  if (digits < 10) digits_string.concat("0");
  digits_string.concat(digits);
  return digits_string;
}

/**
 * Advance to next screen page
 *
 * @return the number of next page
 */
int nextPage() {
  display.fillRect(0, 37, 160, 54, DEFAULT_BG_COLOR);

  if (page >= LAST_PAGE) {
    return page = 0;
  }
  return ++page;
}

void addVSpace() {
  display.setTextSize(1);
  display.println(" ");
}

void setLEDs(byte state) {
  // digitalWrite(LED_2, state);
  // digitalWrite(LED_3, state);
  digitalWrite(LED_SW, state);
}

// toggle LEDs
void blinkLED() {
  byte ledState = !digitalRead(LED_SW);
  setLEDs(ledState);
}

bool runVibe(unsigned int sequence[], int siz) {
  if (!vibePresent) { return false; }

  for (int thisVibe = 0; thisVibe < siz; thisVibe++) {
    vibe.setWaveform(thisVibe, sequence[thisVibe]);
  }
  vibe.go();
  return true;
}

#ifdef RP_PIO
// non-blocking tone function that uses second core
void playNote(uint16_t note, uint16_t duration) {
    STR_NOTE noteData;
    // fifo uses 32 bit messages so package up the note and duration
    uint32_t note_msg;
    noteData.duration = duration;
    noteData.freq = note;

    memcpy((uint32_t*)&note_msg, &noteData, sizeof(noteData));
    rp2040.fifo.push_nb(note_msg);  // send note to second core via fifo queue
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
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
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

// ** Logic for EEPROM **
# define EEPROM_OFFSET 0  // Address of first byte of EEPROM

// write to EEPROM
void writeDeviceData() {
  deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  #ifdef M0_PIO
    if (0 != eep.write(EEPROM_OFFSET, (uint8_t*)&deviceData, sizeof(deviceData))) {
      //Serial.println(F("error writing EEPROM"));
    }
  #elif RP_PIO
    EEPROM.put(EEPROM_OFFSET, deviceData);
    EEPROM.commit();
  #endif
}

// reset eeprom and deviceData to factory defaults
void resetDeviceData() {
  deviceData = STR_DEVICE_DATA_140_V1();
  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = 3;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;  // 1013.25 mbar
  deviceData.metric_temp = true;
  deviceData.metric_alt = true;
  deviceData.performance_mode = 0;
  deviceData.batt_size = 4000;  // 4kw
  writeDeviceData();
}

// read saved data from EEPROM
void refreshDeviceData() {
  uint8_t tempBuf[sizeof(deviceData)];
  uint16_t crc;

  #ifdef M0_PIO
    if (0 != eep.read(EEPROM_OFFSET, tempBuf, sizeof(deviceData))) {
      // Serial.println(F("error reading EEPROM"));
    }
  #elif RP_PIO
    EEPROM.get(EEPROM_OFFSET, tempBuf);
  #endif

  memcpy((uint8_t*)&deviceData, tempBuf, sizeof(deviceData));
  crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);

  if (crc != deviceData.crc) {
    resetDeviceData();
  }
}


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

bool sanitizeDeviceData() {
  bool changed = false;

  if (deviceData.screen_rotation == 1 || deviceData.screen_rotation == 3) {
  } else {
    deviceData.screen_rotation = 3;
    changed = true;
  }
  if (deviceData.sea_pressure < 0 || deviceData.sea_pressure > 10000) {
    deviceData.sea_pressure = 1013.25;
    changed = true;
  }
  if (deviceData.metric_temp != true && deviceData.metric_temp != false) {
    deviceData.metric_temp = true;
    changed = true;
  }
  if (deviceData.metric_alt != true && deviceData.metric_alt != false) {
    deviceData.metric_alt = true;
    changed = true;
  }
  if (deviceData.performance_mode < 0 || deviceData.performance_mode > 1) {
    deviceData.performance_mode = 0;
    changed = true;
  }
  if (deviceData.batt_size < 0 || deviceData.batt_size > 10000) {
    deviceData.batt_size = 4000;
    changed = true;
  }
  return changed;
}

// wipes screen and resets properties
void resetDisplay() {
  display.fillScreen(DEFAULT_BG_COLOR);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextWrap(true);
  display.setRotation(deviceData.screen_rotation);  // 1=right hand, 3=left hand
}

// customized for sp140
void parse_usb_serial() {
#ifdef USE_TINYUSB
  const size_t capacity = JSON_OBJECT_SIZE(12) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
    display.fillScreen(DEFAULT_BG_COLOR);
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("BL - UF2");
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
  sanitizeDeviceData();
  writeDeviceData();
  resetDisplay();
  send_usb_serial();
#endif
}



/// Sp140-Helpers

// track flight timer
void handleFlightTime() {
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
      throttleSecs = (millis()-throttledAtMillis) / 1000.0;
    } else {
      throttleSecs = 0;
    }
  }
}

//**************************************************************************************//
//  Helper function to print values without flashing numbers due to slow screen refresh.
//  This function only re-draws the digit that needs to be updated.
//    BUG:  If textColor is not constant across multiple uses of this function,
//          weird things happen.
//**************************************************************************************//
void dispValue(float value, float &prevVal, int maxDigits, int precision, int x, int y, int textSize, int textColor, int background){
  int numDigits = 0;
  char prevDigit[DIGIT_ARRAY_SIZE] = {};
  char digit[DIGIT_ARRAY_SIZE] = {};
  String prevValTxt = String(prevVal, precision);
  String valTxt = String(value, precision);
  prevValTxt.toCharArray(prevDigit, maxDigits+1);
  valTxt.toCharArray(digit, maxDigits+1);

  // COUNT THE NUMBER OF DIGITS THAT NEED TO BE PRINTED:
  for (int i=0; i<maxDigits; i++) {
    if (digit[i]) {
      numDigits++;
    }
  }

  display.setTextSize(textSize);
  display.setCursor(x, y);

  // PRINT LEADING SPACES TO RIGHT-ALIGN:
  display.setTextColor(background);
  for (int i=0; i<(maxDigits-numDigits); i++) {
    display.print(static_cast<char>(218));
  }
  display.setTextColor(textColor);

  // ERASE ONLY THE NESSESARY DIGITS:
  for (int i=0; i<numDigits; i++) {
    if (digit[i]!=prevDigit[i]) {
      display.setTextColor(background);
      display.print(static_cast<char>(218));
    } else {
      display.setTextColor(textColor);
      display.print(digit[i]);
    }
  }

  // BACK TO THE BEGINNING:
  display.setCursor(x, y);

  // ADVANCE THE CURSOR TO THE PROPER LOCATION:
  display.setTextColor(background);
  for (int i=0; i<(maxDigits-numDigits); i++) {
    display.print(static_cast<char>(218));
  }
  display.setTextColor(textColor);

  // PRINT THE DIGITS THAT NEED UPDATING
  for(int i=0; i<numDigits; i++){
    display.print(digit[i]);
  }

  prevVal = value;
}

// displays number of minutes and seconds (since armed)
void displayTime(int val) {
  int minutes = val / 60;  // numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

  display.print(convertToDigits(minutes));
  display.print(":");
  display.print(convertToDigits(seconds));
}

// TODO (bug) rolls over at 99mins
void displayTime(int val, int x, int y, uint16_t bg_color) {
  // displays number of minutes and seconds (since armed and throttled)
  display.setCursor(x, y);
  display.setTextSize(2);
  display.setTextColor(BLACK);
  minutes = val / 60;
  seconds = numberOfSeconds(val);
  if (minutes < 10) {
    display.setCursor(x, y);
    display.print("0");
  }
  dispValue(minutes, prevMinutes, 2, 0, x, y, 2, BLACK, bg_color);
  display.setCursor(x+24, y);
  display.print(":");
  display.setCursor(x+36, y);
  if (seconds < 10) {
    display.print("0");
  }
  dispValue(seconds, prevSeconds, 2, 0, x+36, y, 2, BLACK, bg_color);
}

// maps battery percentage to a display color
uint16_t batt2color(int percentage) {
  if (percentage >= 30) {
    return GREEN;
  } else if (percentage >= 15) {
    return YELLOW;
  }
  return RED;
}

// Start the bmp388 sensor
bool initBmp() {
  if (!bmp.begin_I2C()) { return false; }

  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);

  return true;
}

// initialize the buzzer
void initBuzz() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void vibrateNotify() {
  if (!ENABLE_VIB) return;
  vibe.setWaveform(0, 15);  // 1 through 117 (see example sketch)
  vibe.setWaveform(1, 0);
  vibe.go();
}

// initialize the vibration motor
bool initVibe() {
  if (!ENABLE_VIB) { return false; }
  if (!vibe.begin()) { return false; }
 
  vibe.selectLibrary(1);
  vibe.setMode(DRV2605_MODE_INTTRIG);
  vibrateNotify();  // initial boot vibration

  return true;
}

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

// for debugging
void printRawSentence() {
  Serial.print(F("DATA: "));
  for (int i = 0; i < ESC_DATA_V2_SIZE; i++) {
    Serial.print(escDataV2[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}

void clearModeDisplay() {
  String prevMode = (deviceData.performance_mode == 0) ? "SPORT" : "CHILL";
  display.setCursor(30, 60);
  display.setTextSize(1);
  display.setTextColor(DEFAULT_BG_COLOR);
  display.print(prevMode);
}

// on boot check for button to switch mode
void modeSwitch(bool update_display) {
  // 0=CHILL 1=SPORT 2=LUDICROUS?!
  if (deviceData.performance_mode == 0) {
    deviceData.performance_mode = 1;
  } else {
    deviceData.performance_mode = 0;
  }
  writeDeviceData();
  if (update_display) { // clear out old text
    clearModeDisplay();
  }
  uint16_t notify_melody[] = { 900, 1976 };
  playMelody(notify_melody, 2);
}

void prepareSerialRead() {  // TODO needed?
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
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

  int motorDuty = (int)(((raw_telemdata.MOTORDUTY_HI << 8) + raw_telemdata.MOTORDUTY_LO) / 10);
  int currentMotorDuty = (motorDuty / 10); //Motor duty cycle

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

void handleTelemetry() {
  prepareSerialRead();
  SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
  //printRawSentence();
  handleSerialData(escDataV2);
}


// OLD
void parseData() {
  // LSB First
  // TODO is this being called even with no ESC?

  _volts = word(escData[1], escData[0]);
  //_volts = ((unsigned int)escData[1] << 8) + escData[0];
  telemetryData.volts = _volts / 100.0;

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.5;  // calibration
  }

  if (telemetryData.volts > 1) {  // ignore empty data
    voltageBuffer.push(telemetryData.volts);
  }

  // Serial.print(F("Volts: "));
  // Serial.println(telemetryData.volts);

  // batteryPercent = mapd(telemetryData.volts, BATT_MIN_V, BATT_MAX_V, 0.0, 100.0); // flat line

  _temperatureC = word(escData[3], escData[2]);
  telemetryData.temperatureC = _temperatureC/100.0;
  // reading 17.4C = 63.32F in 84F ambient?
  // Serial.print(F("TemperatureC: "));
  // Serial.println(temperatureC);

  _amps = word(escData[5], escData[4]);
  telemetryData.amps = _amps;

  // Serial.print(F("Amps: "));
  // Serial.println(amps);

  watts = telemetryData.amps * telemetryData.volts;

  // 7 and 6 are reserved bytes

  _eRPM = escData[11];     // 0
  _eRPM << 8;
  _eRPM += escData[10];    // 0
  _eRPM << 8;
  _eRPM += escData[9];     // 30
  _eRPM << 8;
  _eRPM += escData[8];     // b4
  telemetryData.eRPM = _eRPM/6.0/2.0;

  // Serial.print(F("eRPM: "));
  // Serial.println(eRPM);

  _inPWM = word(escData[13], escData[12]);
  telemetryData.inPWM = _inPWM/100.0;

  // Serial.print(F("inPWM: "));
  // Serial.println(inPWM);

  _outPWM = word(escData[15], escData[14]);
  telemetryData.outPWM = _outPWM/100.0;

  // Serial.print(F("outPWM: "));
  // Serial.println(outPWM);

  // 17 and 16 are reserved bytes
  // 19 and 18 is checksum
  telemetryData.checksum = word(escData[19], escData[18]);

  // Serial.print(F("CHECKSUM: "));
  // Serial.print(escData[19]);
  // Serial.print(F(" + "));
  // Serial.print(escData[18]);
  // Serial.print(F(" = "));
  // Serial.println(checksum);
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

// ring buffer for voltage readings
float getBatteryVoltSmoothed() {
  float avg = 0.0;

  if (voltageBuffer.isEmpty()) { return avg; }

  using index_t = decltype(voltageBuffer)::index_t;
  for (index_t i = 0; i < voltageBuffer.size(); i++) {
    avg += voltageBuffer[i] / voltageBuffer.size();
  }
  return avg;
}






// Thread
unsigned long prevPwrMillis = 0;

void trackPower() {
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (armed) {
    wattsHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}



void checkButtons() {

  button_top.check();
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

    // update display to show cruise
    display.setCursor(70, 60);
    display.setTextSize(1);
    display.setTextColor(RED);
    display.print(F("CRUISE"));

    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);

    bottom_bg_color = YELLOW;
    display.fillRect(0, 93, 160, 40, bottom_bg_color);

    cruisedAtMilis = millis();  // start timer
  }
}

void removeCruise(bool alert) {
  cruising = false;

  // update bottom bar
  bottom_bg_color = DEFAULT_BG_COLOR;
  if (armed) { bottom_bg_color = ARMED_BG_COLOR; }
  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  // update text status
  display.setCursor(70, 60);
  display.setTextSize(1);
  display.setTextColor(DEFAULT_BG_COLOR);
  display.print(F("CRUISE"));  // overwrite in bg color to remove
  display.setTextColor(BLACK);

  if (alert) {
    vibrateNotify();

    if (ENABLE_BUZ) {
      uint16_t notify_melody[] = { 500, 500 };
      playMelody(notify_melody, 2);
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
  runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);

  bottom_bg_color = DEFAULT_BG_COLOR;
  display.fillRect(0, 93, 160, 40, bottom_bg_color);
  ///updateDisplay();

  // update armed_time
  refreshDeviceData();
  deviceData.armed_time += round(armedSecs / 60);  // convert to mins
  writeDeviceData();
  ///delay(1000);  // TODO just disable button thread // dont allow immediate rearming
}

// convert barometer data to altitude in meters
float getAltitudeM() {
  if (!bmpPresent) { return 0; }
  if (!bmp.performReading()) { return 0; }
  
  ambientTempC = bmp.temperature;
  float altitudeM = bmp.readAltitude(deviceData.sea_pressure);
  return altitudeM;
}

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = { 1760, 1976, 2093 };
  unsigned int arm_vibes[] = { 70, 33, 0 };

  armed = true;
  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  ledBlinkThread.enabled = false;
  armedAtMilis = millis();
  armAltM = getAltitudeM();

  setLEDs(HIGH);
  runVibe(arm_vibes, 3);
  playMelody(arm_melody, 3);

  bottom_bg_color = ARMED_BG_COLOR;
  display.fillRect(0, 93, 160, 40, bottom_bg_color);

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
        modeSwitch(true);
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
void initButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);

  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2500);
  buttonConfig->setDoubleClickDelay(600);
}

void displayMeta() {
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BLACK);
  display.setCursor(25, 30);
  display.println("OpenPPG");
  display.setFont();
  display.setTextSize(2);
  display.setCursor(60, 60);
  display.print("v" + String(VERSION_MAJOR) + "." + String(VERSION_MINOR));
#ifdef RP_PIO
  display.print("R");
#endif
  display.setCursor(54, 90);
  displayTime(deviceData.armed_time);
}

// inital screen setup and config
void initDisplay() {
  display.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  // display.setSPISpeed(40000000);  // 40MHz SPI speed

  pinMode(TFT_LITE, OUTPUT);
  resetDisplay();
  displayMeta();
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
  delay(2500);
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



// display altitude data on screen
void displayAlt() {
  // if no bmp, just display "ERR"
  if (!bmpPresent) {
    display.setTextSize(2);
    display.setCursor (85, 102);
    display.setTextColor(RED);
    display.print(F("AL ERR"));
    return;
  }

  float altM = 0;
  // TODO make MSL explicit?
  if (armAltM > 0 && deviceData.sea_pressure != DEFAULT_SEA_PRESSURE) {  // MSL
    altM = getAltitudeM();
  } else {  // AGL
    altM = getAltitudeM() - armAltM;
  }

  // convert to ft if not using metric
  float alt = deviceData.metric_alt ? altM : (round(altM * 3.28084));

  dispValue(alt, lastAltM, 5, 0, 70, 102, 2, BLACK, bottom_bg_color);

  display.print(deviceData.metric_alt ? F("m") : F("ft"));
  lastAltM = alt;
}


/********
 *
 * Display logic
 *
 *******/
bool screen_wiped = false;

void displayMode() {
  display.setCursor(30, 60);
  display.setTextSize(1);
  if (deviceData.performance_mode == 0) {
    display.setTextColor(BLUE);
    display.print("CHILL");
  } else {
    display.setTextColor(RED);
    display.print("SPORT");
  }
}

// display first page (voltage and current)
void displayPage0() {
  float avgVoltage = getBatteryVoltSmoothed();

  dispValue(avgVoltage, prevVolts, 5, 1, 84, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("V");

  dispValue(telemetryData.amps, prevAmps, 3, 0, 108, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("A");

  float kWatts = watts / 1000.0;
  kWatts = constrain(kWatts, 0, 50);

  dispValue(kWatts, prevKilowatts, 4, 1, 10, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kW");

  float kwh = wattsHoursUsed / 1000;
  dispValue(kwh, prevKwh, 4, 1, 10, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kWh");

  displayMode();
}

// show data on screen and handle different pages
void updateDisplay() {
  if (!screen_wiped) {
    display.fillScreen(WHITE);
    screen_wiped = true;
  }
  //Serial.print("v: ");
  //Serial.println(volts);

  displayPage0();
  //dispValue(kWatts, prevKilowatts, 4, 1, 10, /*42*/55, 2, BLACK, DEFAULT_BG_COLOR);
  //display.print("kW");

  display.setTextColor(BLACK);
  float avgVoltage = getBatteryVoltSmoothed();
  batteryPercent = getBatteryPercent(avgVoltage);  // multi-point line
  // change battery color based on charge
  int batt_width = map((int)batteryPercent, 0, 100, 0, 108);
  display.fillRect(0, 0, batt_width, 36, batt2color(batteryPercent));

  if (avgVoltage < BATT_MIN_V) {
    if (batteryFlag) {
      batteryFlag = false;
      display.fillRect(0, 0, 108, 36, DEFAULT_BG_COLOR);
    }
    display.setCursor(12, 3);
    display.setTextSize(2);
    display.setTextColor(RED);
    display.println("BATTERY");

    if ( avgVoltage < 10 ) {
      display.print(" ERROR");
    } else {
      display.print(" DEAD");
    }
  } else {
    batteryFlag = true;
    display.fillRect(map(batteryPercent, 0,100, 0,108), 0, map(batteryPercent, 0,100, 108,0), 36, DEFAULT_BG_COLOR);
  }
  // cross out battery box if battery is dead
  if (batteryPercent <= 5) {
    display.drawLine(0, 1, 106, 36, RED);
    display.drawLine(0, 0, 108, 36, RED);
    display.drawLine(1, 0, 110, 36, RED);
  }
  dispValue(batteryPercent, prevBatteryPercent, 3, 0, 108, 10, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("%");

  // battery shape end
  //display.fillRect(102, 0, 6, 9, BLACK);
  //display.fillRect(102, 27, 6, 10, BLACK);

  display.fillRect(0, 36, 160, 1, BLACK);
  display.fillRect(108, 0, 1, 36, BLACK);
  display.fillRect(0, 92, 160, 1, BLACK);

  displayAlt();

  //dispValue(ambientTempF, prevAmbTempF, 3, 0, 10, 100, 2, BLACK, DEFAULT_BG_COLOR);
  //display.print("F");

  handleFlightTime();
  displayTime(throttleSecs, 8, 102, bottom_bg_color);

  //dispPowerCycles(104,100,2);
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

  pinMode(LED_SW, OUTPUT);   // set up the internal LED2 pin

  analogReadResolution(12);     // M0 family chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  unsigned int startup_vibes[] = { 27, 27, 0 };
  initButtons();

  ledBlinkThread.onRun(blinkLED);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(updateDisplay);
  displayThread.setInterval(250);

  buttonThread.onRun(checkButtons);
  buttonThread.setInterval(5);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  telemetryThread.onRun(handleTelemetry);
  telemetryThread.setInterval(50);

  counterThread.onRun(trackPower);
  counterThread.setInterval(250);

#ifdef M0_PIO
  Watchdog.enable(5000);
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
#elif RP_PIO
  watchdog_enable(5000, 1);
  EEPROM.begin(512);
#endif

  refreshDeviceData();

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  initBuzz();
  bmpPresent = initBmp();
  getAltitudeM();  // throw away first value
  vibePresent = initVibe();

#ifdef M0_PIO
  Watchdog.reset();
#endif
  initDisplay();
  if (button_top.isPressedRaw()) {
    modeSwitch(false);
  }
}

// Main loop - everything runs in threads
void loop() {
#ifdef M0_PIO
  Watchdog.reset();
#elif RP_PIO
  watchdog_update();
#endif

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
    uint32_t noteData = rp2040.fifo.pop();  
    memcpy((uint32_t*)&note, &noteData, sizeof(note));
    tone(BUZZER_PIN, note.freq);
    delay(note.duration);
    noTone(BUZZER_PIN);
  }
}
#endif
