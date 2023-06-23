#ifndef SP140_ALTIMETER_H_
#define SP140_ALTIMETER_H_

#include "sp140/structs.h"

// Set up the barometer
void setupAltimeter();

// Set the ground altitude for computing Above Ground Level (AGL) altitude.
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

// Get the altitude (in meters)
float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

#endif  // SP140_ALTIMETER_H_