#include "sp140/display.h"

#include "sp140/config.h"
#include "sp140/openppg_logo.h"
#include "sp140/structs.h"

// DEBUG WATCHDOG
#ifdef RP_PIO
  #include "hardware/watchdog.h"
  bool watchdogCausedReboot = false;
  bool watchdogEnableCausedReboot = false;
#endif

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(160, 128);

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

// Clears screen and resets properties
void resetRotation(unsigned int rotation) {
  display.setRotation(rotation);  // 1=right hand, 3=left hand
}

void displayBoot(const STR_DEVICE_DATA_140_V1& deviceData) {
  display.fillScreen(WHITE);
  display.drawXBitmap(8, 10, openppg_logo, 140, 42, BLACK);
  display.setTextSize(2);
  display.setCursor(50, 60);
  display.setTextColor(BLACK);
  display.printf("v%d.%d", VERSION_MAJOR, VERSION_MINOR);
#ifdef RP_PIO
  display.print("R");
#endif
  display.setTextColor(GRAY);
  display.setCursor(25, 80);
  display.print("[thandal]");
  // Total armed time
  display.setTextColor(BLACK);
  display.setCursor(32, 100);
  const int hours = deviceData.armed_seconds / 3600;
  const int minutes = (deviceData.armed_seconds / 60) % 60;
  const int seconds = deviceData.armed_seconds % 60;
  display.printf("%02d:%02d:%02d", hours, minutes, seconds);
}

// inital screen setup and config
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData) {
#ifdef RP_PIO
  watchdogCausedReboot = watchdog_caused_reboot();
  watchdogEnableCausedReboot = watchdog_enable_caused_reboot();
#endif
  display.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
  resetRotation(deviceData.screen_rotation);
  displayBoot(deviceData);
}

void updateDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
  ) {
  canvas.fillScreen(WHITE);
  canvas.setTextWrap(false);

  const unsigned int nowMillis = millis();

  // Display region lines
  canvas.drawFastHLine(0, 36, 160, BLACK);
  canvas.drawFastVLine(100, 0, 36, BLACK);
  canvas.drawFastHLine(0, 80, 160, BLACK);
  canvas.drawFastHLine(0, 92, 160, BLACK);

  // Display battery level and status
  canvas.setTextSize(2);
  const float batteryPercent = getBatteryPercent(escTelemetry.volts);
  //   Display battery bar
  if ((nowMillis - escTelemetry.lastUpdateMillis) > 2000) {
    canvas.setCursor(4, 3);
    canvas.setTextColor(RED);
    canvas.print("ESC DATA\n  ERROR");
  } else {
    if (batteryPercent > 0) {
      unsigned int batteryColor = RED;
      if (batteryPercent >= 30) batteryColor = GREEN;
      else if (batteryPercent >= 15) batteryColor = YELLOW;
      int batteryPercentWidth = map(static_cast<int>(batteryPercent), 0, 100, 0, 100);
      canvas.fillRect(0, 0, batteryPercentWidth, 36, batteryColor);
    } else {
      canvas.setCursor(12, 3);
      canvas.setTextColor(RED);
      canvas.println("BATTERY");
      if (escTelemetry.volts < 10) {
        canvas.print(" ERROR");
      } else {
        canvas.print(" DEAD");
      }
    }
  }
  //   Display battery percent
  canvas.setCursor(108, 10);
  canvas.setTextColor(BLACK);
  canvas.printf("%3d%%", static_cast<int>(batteryPercent));

  const float kWatts = constrain(escTelemetry.watts / 1000.0, 0, 50);
  const float volts = escTelemetry.volts;
  const float kWh = escTelemetry.wattHours / 1000.0;
  const float amps = escTelemetry.amps;

  canvas.setCursor(1, 42);
  canvas.printf("%4.1fkW  %4.1fV", kWatts, volts);
  canvas.setCursor(1, 61);
  canvas.printf("%4.1fkWh %4.1fA", kWh, amps);

  // Display modes
  canvas.setCursor(8, 83);
  canvas.setTextSize(1);
  if (deviceData.performance_mode == 0) {
      canvas.setTextColor(BLUE);
      canvas.print("CHILL");
  } else {
    canvas.setTextColor(RED);
    canvas.print("SPORT");
  }

  canvas.setCursor(46, 83);
  if (armed) {
    canvas.setTextColor(BLACK, CYAN);
    canvas.print("ARMED");
  } else {
    canvas.setTextColor(BLACK, GREEN);
    canvas.print("SAFED");
  }

  if (cruising) {
    canvas.setCursor(84, 83);
    canvas.setTextColor(BLACK, YELLOW);
    canvas.print("CRUISE");
  }

  canvas.setCursor(124, 83);
  canvas.setTextColor(BLACK);
  canvas.printf("FLAG%2d", escTelemetry.statusFlag);

  // Display statusbar
  unsigned int statusBarColor = WHITE;
  if (cruising) statusBarColor = YELLOW;
  else if (armed) statusBarColor = CYAN;
  canvas.fillRect(0, 93, 160, 40, statusBarColor);

  // Display armed time for the current session
  canvas.setTextColor(BLACK);
  canvas.setTextSize(2);
  canvas.setCursor(8, 102);
  static unsigned int _lastArmedMillis = 0;
  if (armed) _lastArmedMillis = nowMillis;
  const int sessionSeconds = (_lastArmedMillis - armedStartMillis) / 1000.0;
  canvas.printf("%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);

  // Display altitude
  canvas.setCursor(72, 102);
  canvas.setTextSize(2);
  if (altitude == __FLT_MIN__) {
    canvas.setTextColor(RED);
    canvas.print(F("ALTERR"));
  } else {
    canvas.setTextColor(BLACK);
    if (deviceData.metric_alt) {
      canvas.printf("%6.1fm", altitude);
    } else {
      canvas.printf("%5dft", static_cast<int>(round(altitude * 3.28084)));
    }
  }

  // ESC temperature
  canvas.setTextSize(1);
  canvas.setCursor(114, 28);
  canvas.printf("%0.1f≈C", escTelemetry.temperatureC);  // Note: the '≈' character shows as a degree symbol on screen 

//  // DEBUG TIMING
//  canvas.setTextSize(1);
//  canvas.setCursor(4, 118);
//  static unsigned int lastDisplayMillis = 0;
//  canvas.printf("%5d  %5d", nowMillis - escTelemetry.lastUpdateMillis, nowMillis - lastDisplayMillis);
//  lastDisplayMillis = nowMillis;
//
//  canvas.printf("  %3d %2d %2d", escTelemetry.lastReadBytes, escTelemetry.errorStopBytes, escTelemetry.errorChecksum);

//  // DEBUG WATCHDOG
//  #ifdef RP_PIO
//    canvas.setTextSize(1);
//    canvas.setCursor(4, 118);
//    canvas.printf("watchdog %d %d", watchdogCausedReboot, watchdogEnableCausedReboot);
//  #endif
//
//  // DEBUG FREE MEMORY
//  #ifdef RP_PIO
//    canvas.printf("  mem %d", rp2040.getFreeHeap());
//  #endif


  // Draw the canvas to the display.
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}
