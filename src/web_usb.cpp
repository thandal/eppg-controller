#include "sp140/config.h"
#include "sp140/web_usb.h"

#ifdef USE_TINYUSB
  #include <ArduinoJson.h>
  #include "Adafruit_TinyUSB.h"

  Adafruit_USBD_WebUSB usb_web;
  WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

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
  sprintf(buf, "%08x%08x%08x%08x",
          (unsigned int)val1,
          (unsigned int)val2,
          (unsigned int)val3,
          (unsigned int)val4);
  id = String(buf);
#elif RP_PIO
  int len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  char buf[len];
  pico_get_unique_board_id_string(buf, len);
  id = String(buf);
#endif // M0_PIO/RP_PIO
  return id;
}

// Reboot/reset controller
#ifdef M0_PIO
void(* resetFunc) (void) = 0;  // declare reset function @ address 0

// sets the magic pointer to trigger a reboot to the bootloader for updating
void rebootBootloader() {
  *DBL_TAP_PTR = DBL_TAP_MAGIC;
  resetFunc();
}
#elif RP_PIO
void rebootBootloader() {
#ifdef USE_TINYUSB
  TinyUSB_Port_EnterDFU();
#endif
}
#endif

// TODO: why are there two different doc formats?
void sendWebUsbSerial(const STR_DEVICE_DATA_140_V1& deviceData) {
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


bool parseWebUsbSerial(STR_DEVICE_DATA_140_V1* deviceData) {
#ifdef USE_TINYUSB
  if (!usb_web.available()) return false;
  const size_t capacity = JSON_OBJECT_SIZE(12) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
///    display.fillScreen(DEFAULT_BG_COLOR);
///    display.setCursor(0, 0);
///    display.setTextSize(2);
///    display.println("BL - UF2");
    rebootBootloader();
    return false;  // run only the command
  }

  if (doc["major_v"] < 5) return false;

  deviceData->screen_rotation = doc["screen_rot"].as<unsigned int>();  // "3/1"
  deviceData->sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  deviceData->metric_temp = doc["metric_temp"];  // true/false
  deviceData->metric_alt = doc["metric_alt"];  // true/false
  deviceData->performance_mode = doc["performance_mode"];  // 0,1
  deviceData->batt_size = doc["batt_size"];  // 4000
  return true;
#else
  return false;
#endif // USE_TINYUSB
}

void setupWebUsbSerial(void (*lineStateCallback) (bool connected)) {
#ifdef USE_TINYUSB
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(lineStateCallback);
#endif
}
