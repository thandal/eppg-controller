#include "sp140/config.h"
#include "sp140/structs.h"
#include "sp140/utilities.h"

#include <Adafruit_ST7735.h>
#include <Fonts/FreeSansBold12pt7b.h>

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

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

void updateStatusBar(bool armed, bool cruising) {
  unsigned int color = DEFAULT_BG_COLOR;
  if (cruising) color = YELLOW;
  else if (armed) color = ARMED_BG_COLOR;
  display.fillRect(0, 93, 160, 40, color);
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

// Map battery percentage to a display color
uint16_t batt2color(int percentage) {
  if (percentage >= 30) {
    return GREEN;
  } else if (percentage >= 15) {
    return YELLOW;
  }
  return RED;
}

// Clears screen and resets properties
void resetDisplay(const STR_DEVICE_DATA_140_V1& deviceData) {
  display.fillScreen(DEFAULT_BG_COLOR);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextWrap(true);
  display.setRotation(deviceData.screen_rotation);  // 1=right hand, 3=left hand
}

void displayBoot(const STR_DEVICE_DATA_140_V1& deviceData) {
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
  const int hours = deviceData.armed_seconds / 3600;
  const int minutes = (deviceData.armed_seconds / 60) % 60;
  const int seconds = deviceData.armed_seconds % 60;
  display.printf("%02d:%02d:%02d", hours, minutes, seconds);
}

// inital screen setup and config
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData) {
  display.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  pinMode(TFT_LITE, OUTPUT);
  resetDisplay(deviceData);
  displayBoot(deviceData);
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
  display.fillScreen(WHITE);
}

void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   float altitude, bool armed, bool cruising,
                   unsigned int sessionSeconds) {
  // "_prev" values are used with dispValue() for tidy screen drawing.
  // TODO: Is the dispValue complexity necessary???
  static float _prevVolts = 0;
  static float _prevAmps = 0;
  static float _prevKwh = 0;
  static float _prevKilowatts = 0;
  static float _prevAlt = 0;
  static float _prevBatteryPercent = 0;

  dispValue(escTelemetry.volts, _prevVolts, 5, 1, 84, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("V");

  dispValue(escTelemetry.amps, _prevAmps, 3, 0, 108, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("A");

  const float kWatts = constrain(escTelemetry.watts / 1000.0, 0, 50);
  dispValue(kWatts, _prevKilowatts, 4, 1, 10, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kW");

  const float kwh = escTelemetry.wattHours / 1000.0;
  dispValue(kwh, _prevKwh, 4, 1, 10, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kWh");

  // Display mode
  display.setCursor(30, 60);
  display.setTextSize(1);
  if (deviceData.performance_mode == 0) {
    display.setTextColor(BLUE);
    display.print("CHILL");
  } else {
    display.setTextColor(RED);
    display.print("SPORT");
  }

  // Display battery level
  display.setTextColor(BLACK); // remove?
  const float batteryPercent = getBatteryPercent(escTelemetry.volts);
  // Change battery color based on charge
  int batteryPercentWidth = map((int)batteryPercent, 0, 100, 0, 108);
  display.fillRect(0, 0, batteryPercentWidth, 36, batt2color(batteryPercent));

  static bool _batteryRedrawOnFaultFlag = true;
  if (escTelemetry.volts < BATT_MIN_V) {
    if (_batteryRedrawOnFaultFlag) {
      _batteryRedrawOnFaultFlag = false;
      display.fillRect(0, 0, 108, 36, DEFAULT_BG_COLOR);
    }
    display.setCursor(12, 3);
    display.setTextSize(2);
    display.setTextColor(RED);
    display.println("BATTERY");
    if (escTelemetry.volts < 10) {
      display.print(" ERROR");
    } else {
      display.print(" DEAD");
    }
  } else {
    _batteryRedrawOnFaultFlag = true;
    display.fillRect(batteryPercentWidth, 0, map(batteryPercent, 0, 100, 108, 0), 36, DEFAULT_BG_COLOR);
  }
  // cross out battery box if battery is dead
  if (batteryPercent <= 5) {
    display.drawLine(0, 1, 106, 36, RED);
    display.drawLine(0, 0, 108, 36, RED);
    display.drawLine(1, 0, 110, 36, RED);
  }
  dispValue(batteryPercent, _prevBatteryPercent, 3, 0, 108, 10, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("%");

  // Battery shape end
  display.fillRect(0, 36, 160, 1, BLACK);
  display.fillRect(108, 0, 1, 36, BLACK);
  display.fillRect(0, 92, 160, 1, BLACK);

  // Display altitude
  display.setTextSize(2);
  if (altitude == 0.0) {  // If no bmp, just display "ERR"
    display.setCursor (85, 102);
    display.setTextColor(RED);
    display.print(F("AL ERR"));
  } else {
    // Display in ft if not using metric
    const float alt = deviceData.metric_alt ? altitude : (round(altitude * 3.28084));
    dispValue(alt, _prevAlt, 5, 0, 70, 102, 2, BLACK, DEFAULT_BG_COLOR);
    display.print(deviceData.metric_alt ? F("m") : F("ft"));
  }

  display.setCursor(8, 102);
  display.setTextSize(2);
  const int hours = sessionSeconds / 3600;
  const int minutes = (sessionSeconds / 60) % 60;
  const int seconds = sessionSeconds % 60;
  display.printf("%02d:%02d:%02d", hours, minutes, seconds);

  static bool _prevArmed = false;
  if ((armed && !_prevArmed) || (!armed && _prevArmed)) {
    updateStatusBar(armed, cruising);
  }
  _prevArmed = armed;

  static bool _prevCruising = false;
  if (cruising && !_prevCruising) {
    updateStatusBar(armed, cruising);
    // Update text status
    display.setCursor(70, 60);
    display.setTextSize(1);
    display.setTextColor(RED);
    display.print(F("CRUISE"));
  }
  if (!cruising && _prevCruising) {
    updateStatusBar(armed, cruising);
    // Update text status
    display.setCursor(70, 60);
    display.setTextSize(1);
    display.setTextColor(DEFAULT_BG_COLOR);
    display.print(F("CRUISE"));  // overwrite in bg color to remove
    display.setTextColor(BLACK);
  }
}