#include "sp140/config.h"
#include "sp140/structs.h"

#include <Adafruit_ST7735.h>
#include <Fonts/FreeSansBold12pt7b.h>

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
  display.fillScreen(DEFAULT_BG_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BLACK);
  display.setCursor(20, 30);
  display.println("OpenPPG");
  display.setFont();
  display.setTextSize(2);
  display.setCursor(50, 60);
  display.print("v" + String(VERSION_MAJOR) + "." + String(VERSION_MINOR));
#ifdef RP_PIO
  display.print("R");
#endif
  // Total armed time
  display.setCursor(35, 90);
  const int hours = deviceData.armed_seconds / 3600;
  const int minutes = (deviceData.armed_seconds / 60) % 60;
  const int seconds = deviceData.armed_seconds % 60;
  display.printf("%02d:%02d:%02d", hours, minutes, seconds);
}

// inital screen setup and config
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData) {
  display.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
  resetRotation(deviceData.screen_rotation);
  displayBoot(deviceData);
}

void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   float altitude, bool armed, bool cruising,
                   unsigned int sessionSeconds) {
  canvas.fillScreen(DEFAULT_BG_COLOR);
  canvas.setTextWrap(false);

  // Display region lines
  canvas.drawFastHLine(0, 36, 160, BLACK);
  canvas.drawFastVLine(100, 0, 36, BLACK);
  canvas.drawFastHLine(0, 80, 160, BLACK);
  canvas.drawFastHLine(0, 92, 160, BLACK);

  // Display battery
  canvas.setTextColor(BLACK);
  canvas.setTextSize(2);

  // HACK
  //const float batteryPercent = getBatteryPercent(escTelemetry.volts);
  static int batteryPercent = 90;
  batteryPercent = (batteryPercent + 1) % 101;

  // Display battery bar
  if (batteryPercent > 0) {
    unsigned int batteryColor = RED;
    if (batteryPercent >= 30) batteryColor = GREEN;
    else if (batteryPercent >= 15) batteryColor = YELLOW;
    int batteryPercentWidth = map((int)batteryPercent, 0, 100, 0, 100);
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
  // Display battery percent
  canvas.setCursor(108, 10);
  canvas.setTextColor(BLACK);
  canvas.printf("%3d%%", batteryPercent);

  //const float kWatts = constrain(escTelemetry.watts / 1000.0, 0, 50);
  static float kWatts = 0;
  kWatts += 1.75;
  if (kWatts > 50) kWatts = 0.0;

  const float volts = kWatts; //escTelemetry.volts;
  const float kWh = kWatts; //escTelemetry.wattHours / 1000.0;
  const float amps = kWatts; //escTelemetry.amps;

  canvas.setCursor(1, 42);
  canvas.printf("%4.1fkW  %4.1fV", kWatts, volts);
  canvas.setCursor(1, 61);
  canvas.printf("%4.1fkWh %4.1fA", kWh, amps);

  // Display modes
  canvas.setCursor(10, 83);
  canvas.setTextSize(1);
  if (deviceData.performance_mode == 0) {
      canvas.setTextColor(BLUE);
      canvas.print("CHILL");
  } else {
    canvas.setTextColor(RED);
    canvas.print("SPORT");
  }

  if (armed) {
    canvas.setCursor(50, 83);
    canvas.setTextColor(BLACK, ARMED_BG_COLOR);
    canvas.print("ARMED");
  } else {
    canvas.setCursor(50, 83);
    canvas.setTextColor(BLACK, GREEN);
    canvas.print("SAFE");
  }

  if (cruising) {
    canvas.setCursor(90, 83);
    canvas.setTextColor(BLACK, YELLOW);
    canvas.print("CRUISE");
  }
  
  // Display statusbar
  unsigned int statusBarColor = DEFAULT_BG_COLOR;
  if (cruising) statusBarColor = YELLOW;
  else if (armed) statusBarColor = ARMED_BG_COLOR;
  canvas.fillRect(0, 93, 160, 40, statusBarColor);

  // Display armed time for the current session
  canvas.setTextColor(BLACK);
  canvas.setTextSize(2);
  canvas.setCursor(8, 102);
  const int minutes = sessionSeconds / 60;
  const int seconds = sessionSeconds % 60;
  canvas.printf("%02d:%02d", minutes, seconds);

  // Altitude
  canvas.setCursor (84, 102);
  canvas.setTextSize(2);
  if (altitude == 0.0) {
    canvas.setTextColor(RED);
    canvas.print(F("ALTERR"));
  } else {
    // Display in ft if not using metric
    canvas.setTextColor(BLACK);
    const float alt = deviceData.metric_alt ? altitude : (altitude * 3.28084);
    canvas.printf("%5.1f%c", alt, deviceData.metric_alt ? 'm' : 'f');
  }

  // Draw the canvas to the display.
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}