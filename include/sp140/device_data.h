#ifndef INCLUDE_SP140_DEVICE_DATA_H_
#define INCLUDE_SP140_DEVICE_DATA_H_

#include "sp140/structs.h"

// Set up the EEPROM, etc.
void setupDeviceData();

// Write deviceData to EEPROM
void writeDeviceData(STR_DEVICE_DATA_140_V1* d);

// Reset deviceData to factory defaults and write to EEPROM
void resetDeviceData(STR_DEVICE_DATA_140_V1* d);

// Read saved data from EEPROM
void refreshDeviceData(STR_DEVICE_DATA_140_V1* d);

#endif  // INCLUDE_SP140_DEVICE_DATA_H_
