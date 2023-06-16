#ifndef SP140_DISPLAY_H_
#define SP140_DISPLAY_H_

#include "sp140/structs.h"

// Set up the display and show splash screen
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

// Clear screen and reset properties
void resetDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

// Show data on screen
void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   float altitude, bool armed, bool cruising,
                   unsigned int sessionSeconds);

#endif  // SP140_DISPLAY_H_