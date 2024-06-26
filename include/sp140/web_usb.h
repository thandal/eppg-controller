#ifndef INCLUDE_SP140_WEB_USB_H_
#define INCLUDE_SP140_WEB_USB_H_

#include "sp140/structs.h"

void setupWebUsbSerial(void (*lineStateCallback)(bool connected));

void sendWebUsbSerial(const STR_DEVICE_DATA_140_V1& deviceData);
bool parseWebUsbSerial(STR_DEVICE_DATA_140_V1* deviceData);

#endif  // INCLUDE_SP140_WEB_USB_H_
