#ifndef INCLUDE_SP140_ESC_TELEMETRY_H_
#define INCLUDE_SP140_ESC_TELEMETRY_H_

#include "sp140/structs.h"

void setupEscTelemetry();

void updateEscTelemetry();

const STR_ESC_TELEMETRY_140& getEscTelemetry();

#endif  // INCLUDE_SP140_ESC_TELEMETRY_H_
