#include "sp140/config.h"
#include "sp140/web_usb.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_TinyUSB.h>

Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");

// Hardware-specific libraries
#ifdef RP_PIO
  #include "pico/unique_id.h"
#endif

#ifdef M0_PIO
  #define DBL_TAP_PTR ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
  #define DBL_TAP_MAGIC 0xf01669ef  // Randomly selected, adjusted to have first and last bit set
  #define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#endif

// Get chip serial number
String chipId() {
  String id = "unknown";
#ifdef M0_PIO
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;

  char buf[33];
  // NOTE: we're just using the lower 16 bits of the uint32 vals.
  snprintf(buf, sizeof(buf), "%08x%08x%08x%08x",
          (unsigned int)val1,
          (unsigned int)val2,
          (unsigned int)val3,
          (unsigned int)val4);
  id = String(buf);
#elif RP_PIO
  const int kPicoBoardIdLen = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  char buf[kPicoBoardIdLen];
  pico_get_unique_board_id_string(buf, kPicoBoardIdLen);
  id = String(buf);
#endif  // M0_PIO/RP_PIO
  return id;
}

// Reboot/reset controller
#ifdef M0_PIO
void(* resetFunc) (void) = 0;  // declare reset function @ address 0

// Sets the magic pointer to trigger a reboot to the bootloader for updating
void rebootBootloader() {
  *DBL_TAP_PTR = DBL_TAP_MAGIC;
  resetFunc();
}
#elif RP_PIO
void rebootBootloader() {
  TinyUSB_Port_EnterDFU();
}
#endif

/* Example JSON for use with https://arduinojson.org/v6/assistant:
{
  "major_v": 10,
  "minor_v": 10,
  "arch": "bananadana",
  "screen_rot": 1,
  "armed_time": 99999999,
  "metric_temp": false,
  "metric_alt": true,
  "performance_mode": 1,
  "sea_pressure": 4000,
  "device_id": "asldkfjasdfl234jldskfj341214lkj3"
}
*/

void sendWebUsbSerial(const STR_DEVICE_DATA_140_V1& deviceData) {
  DynamicJsonDocument doc(256);  // See discussion of ArduinoJson Assistant, above.

#ifdef M0_PIO
  doc["arch"] = "SAMD21";
#elif RP_PIO
  doc["arch"] = "RP2040";
#endif
  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;

  doc["screen_rot"] = deviceData.screen_rotation;
  doc["armed_time"] = static_cast<int>(deviceData.armed_seconds);
  doc["metric_temp"] = deviceData.metric_temp;
  doc["metric_alt"] = deviceData.metric_alt;
  // doc["performance_mode"] = deviceData.performance_mode;
  // doc["batt_size"] = static_cast<int>(deviceData.batt_size);
  // doc["sea_pressure"] = static_cast<float>(deviceData.sea_pressure);
  // doc["device_id"] = chipId().c_str();

  char output[256];
  Serial.println("serializeJSON");
  serializeJson(doc, output, sizeof(output));
  Serial.println(output);
  Serial.println(strlen(output));
  if (usb_web.connected()) {
    Serial.println("Sending over usb_web");
    // There appears to be an issue with usb_web println and long outputs which causes a hang!
    usb_web.println(output);  
    usb_web.flush();
  }
}

bool parseWebUsbSerial(STR_DEVICE_DATA_140_V1* deviceData) {
  if (!usb_web.available()) return false;
  DynamicJsonDocument doc(256);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
    rebootBootloader();
    return false;  // run only the command
  }

  if (doc["major_v"] < 5) return false;

  deviceData->screen_rotation = doc["screen_rot"].as<unsigned int>();  // "3/1"
  // deviceData->sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  // deviceData->metric_temp = doc["metric_temp"];  // true/false
  deviceData->metric_alt = doc["metric_alt"];  // true/false
  // deviceData->performance_mode = doc["performance_mode"];  // 0,1
  // deviceData->batt_size = doc["batt_size"];  // 4000
  return true;
}

void setupWebUsbSerial(void (*lineStateCallback)(bool connected)) {
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(lineStateCallback);
  usb_web.begin();
}
