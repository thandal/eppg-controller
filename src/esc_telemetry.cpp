#include "sp140/config.h"
#include "sp140/esc_telemetry.h"
#include "sp140/structs.h"

#include <Arduino.h>
#include <CircularBuffer.h>        // smooth out readings

#pragma pack(push, 1)
// ESC serial telemetry packet v2: see https://docs.powerdrives.net/products/uhv/uart-telemetry-output
typedef struct  {
  // Voltage
  uint16_t rawVolts;
  // Temperature
  uint16_t rawTemperature;
  // Current
  int16_t rawAmps;
  // Reserved
  uint16_t R0;
  // eRPM
  uint32_t rawRpm;
  // Input Duty
  uint16_t dutyIn;
  // Motor Duty
  uint16_t dutyOut;
  // Status Flags
  uint8_t statusFlag;
  // Reserved
  uint8_t R1;
  // Fletcher checksum
  uint16_t checksum;
  // Stop bytes
  uint16_t stopBytes;
} STR_ESC_TELEMETRY_140_V2;
#pragma pack(pop)

static STR_ESC_TELEMETRY_140 escTelemetry;
CircularBuffer<float, 50> voltsBuffer;
uint32_t prevWattHoursMillis = 0;


#define ESC_BAUD_RATE         115200
// ESC packets (22 bytes) are transmitted about every 20 ms.
// 22 bytes at 115200 bps should take about 2 ms.
// We set the timeout to 2ms to be very conservative.
#define ESC_TIMEOUT           2

uint16_t checkFletcher16(byte buffer[], int len) {
  // See https://en.wikipedia.org/wiki/Fletcher's_checksum
  uint16_t c0 = 0;
  uint16_t c1 = 0;
  for (int i = 0; i < len; ++i) {
    c0 = (c0 + buffer[i]) % 255;
    c1 = (c1 + c0) % 255;
  }
  // Assemble the 16-bit checksum value
  return (c1 << 8) | c0;
}

void parseEscSerialData(byte buffer[]) {
  if (buffer[20] != 255 || buffer[21] != 255) {
    escTelemetry.errorStopBytes++;
    // Serial.println("ESC parse error: no stop bytes");
    return;
  }

  // Check the Fletcher checksum
  // Check only first 18 bytes, skip checksum and stop bytes
  const uint16_t computedChecksum = checkFletcher16(buffer, sizeof(STR_ESC_TELEMETRY_140_V2) - 4);
  const uint16_t checksum = word(buffer[19], buffer[18]);

  // Checksums do not match
  if (computedChecksum != checksum) {
    escTelemetry.errorChecksum++;
    // Serial.println("ESC parse error: bad checksum");
    return;
  }

  STR_ESC_TELEMETRY_140_V2 &telem = *reinterpret_cast<STR_ESC_TELEMETRY_140_V2*>(buffer);

  // Voltage
  float volts = telem.rawVolts / 100.0;
  const float kBattMinV = 60.0;   // 24 * 2.5V per cell
  const float kVoltOffset = 1.5;  // Calibration
  if (volts > kBattMinV) volts += kVoltOffset;

  voltsBuffer.push(volts);
  float avgVolts = 0.0;
  for (decltype(voltsBuffer)::index_t i = 0; i < voltsBuffer.size(); ++i) {
    avgVolts += voltsBuffer[i] / voltsBuffer.size();
  }
  escTelemetry.volts = avgVolts;

  // Current
  escTelemetry.amps = telem.rawAmps / 12.5;
  escTelemetry.watts = escTelemetry.amps * escTelemetry.volts;

  // Energy
  const uint32_t currentMillis = millis();
  const float deltaHours = (currentMillis - prevWattHoursMillis) / 1000.0 / 3600.0;
  prevWattHoursMillis = currentMillis;
  escTelemetry.wattHours += round(escTelemetry.watts * deltaHours);

  // Temperature
  const float SERIESRESISTOR = 10000.0;
  const float NOMINAL_RESISTANCE = 10000.0;
  const float NOMINAL_TEMPERATURE = 25.0;
  const float INV_T0 = 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  const float INV_B = 1.0 / 3455.0;
  // Convert value to resistance
  const float Rntc = SERIESRESISTOR / ((4096.0 / telem.rawTemperature) - 1);
  // Compute the temperature
  const float temperature = 1.0 / (log(Rntc / NOMINAL_RESISTANCE) * INV_B + INV_T0) - 273.15;
  escTelemetry.temperatureC = temperature;

  // RPM
  const int POLECOUNT = 62;
  escTelemetry.rpm = telem.rawRpm / POLECOUNT;  // Real RPM output

  // Input Duty
  escTelemetry.inPWM = telem.dutyIn / 100;  // PWM = Duty?

  // Motor Duty
  escTelemetry.outPWM = telem.dutyOut / 100;  // PWM = Duty?

  // Status
  escTelemetry.statusFlag = telem.statusFlag;

  // Update freshness
  escTelemetry.lastUpdateMillis = millis();
}

const STR_ESC_TELEMETRY_140& getEscTelemetry() {
  return escTelemetry;
}

void updateEscTelemetry() {
  // If this function is called slowly, readBytes sometimes includes *part* of the next packet.
  // But on the next read, we get the start of a new packet, rather than the rest of the packet!?
  // 1. Why doesn't the timeout cause readBytes to get the full next packet?
  // 2. Why don't we see the rest of the packet on the *next* read?
  // As a hack, we just try to parse from the front of the buffer... and it "works great"!

  byte buffer[256];
  escTelemetry.lastReadBytes = SerialESC.readBytes(buffer, sizeof(buffer));

//  // DEBUG
//  static unsigned int lastMillis = 0;
//  unsigned int nowMillis = millis();
//  Serial.printf("ESC DATA [%03d] (%03d): ", nowMillis - lastMillis, escTelemetry.lastReadBytes);
//  lastMillis = nowMillis;
//  for (int i = 0; i < escTelemetry.lastReadBytes; i++) {
//    Serial.printf("%02X ", buffer[i]);
//  }
//  Serial.println();

  if (escTelemetry.lastReadBytes >= 22) {
    parseEscSerialData(buffer);
  }
}

void setupEscTelemetry() {
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
  escTelemetry.errorStopBytes = 0;
  escTelemetry.errorChecksum = 0;
}
