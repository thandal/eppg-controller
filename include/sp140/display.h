#ifndef SP140_DISPLAY_H_
#define SP140_DISPLAY_H_

#include "sp140/structs.h"

// Set up the display and show splash screen
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

// Clear screen and reset properties
void resetDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

// Show data on screen
void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData, float volts,
  float amps, float watts, float wattHours, float altitude,
  unsigned int throttleSecs);

#endif  // SP140_DISPLAY_H_