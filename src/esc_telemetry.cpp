#include "sp140/config.h"
#include "sp140/esc_telemetry.h"
#include "sp140/structs.h"

#include <Arduino.h>
#include <CircularBuffer.h>        // smooth out readings

#pragma pack(push, 1)
// v2 ESC serial telemetry struct
typedef struct  {
  // Voltage
  int16_t centiVolts;
  // Temperature
  uint16_t rawTemperature;
  // Current
  uint16_t rawAmps;
  // Reserved
  uint16_t R0;
  // eRPM
  int32_t rawRpm;
  // Input Duty
  uint16_t dutyIn;
  // Motor Duty
  uint16_t dutyOut;
  // Status Flags
  uint8_t statusFlag;
} STR_ESC_TELEMETRY_140_V2;
#pragma pack(pop)

static STR_ESC_TELEMETRY_140 escTelemetry;
CircularBuffer<float, 50> voltsBuffer;
unsigned long prevWattHoursMillis = 0;

int checkFlectcher16(byte byteBuffer[], int len) {
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (int i = 0; i < len; i++) { 
        c0 = (int)(c0 + ((int)byteBuffer[i])) % 255;
        c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    const int fCCRC16 = (c1 << 8) | c0;
    return (int)fCCRC16;
}

void parseEscSerialData(byte buffer[]) {
  if (buffer[20] != 255 || buffer[21] != 255) {
    Serial.println("no stop byte");
    return; // Stop byte of 65535 not recieved
  }

  // Check the Fletcher checksum
  // Check only first 18 bytes, skip crc bytes and stop bytes
  const int checkFletch = checkFlectcher16(buffer, ESC_DATA_V2_SIZE - 4);
  const int16_t checksum = word(buffer[19], buffer[18]);

  // Checksums do not match
  if (checkFletch != checksum) {
    Serial.println("checksum error");
    return;
  }

  STR_ESC_TELEMETRY_140_V2 &telem = *(STR_ESC_TELEMETRY_140_V2*)buffer;
  
  float volts = telem.centiVolts / 100;
  if (volts > BATT_MIN_V) volts += 1.0; // calibration
  voltsBuffer.push(volts);
  float avgVolts = 0.0;
  for (decltype(voltsBuffer)::index_t i = 0; i < voltsBuffer.size(); ++i) {
    avgVolts += voltsBuffer[i] / voltsBuffer.size();
  }
  escTelemetry.volts = avgVolts;

  // Temperature
  const int SERIESRESISTOR = 10000;
  const int NOMINAL_RESISTANCE = 10000;
  const int NOMINAL_TEMPERATURE = 25;
  const int BCOEFFICIENT = 3455;
  // Convert value to resistance
  float Rntc = (4096 / (float) telem.rawTemperature) - 1;
  Rntc = SERIESRESISTOR / Rntc;
  // Compute the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE; // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celcius
  // Filter bad values
  if (temperature < 0 || temperature > 200) {
    temperature = 0;
  }
  temperature = (float) trunc(temperature * 100) / 100; // 2 decimal places
  escTelemetry.temperatureC = temperature;

  // Current
  escTelemetry.amps = telem.rawAmps / 12.5;
  escTelemetry.watts = escTelemetry.amps * escTelemetry.volts;

  // Update wattHours
  const unsigned long currentMillis= millis();
  const float deltaHours = (currentMillis - prevWattHoursMillis) / 1000.0 / 3600.0;
  prevWattHoursMillis = currentMillis;
  escTelemetry.wattHours += round(escTelemetry.watts * deltaHours);

  // RPM
  const int POLECOUNT = 62;
  escTelemetry.rpm = telem.rawRpm / POLECOUNT;  // Real RPM output 

  // Input Duty
  escTelemetry.inPWM = telem.dutyIn / 100;  // PWM = Duty?

  // Motor Duty
  escTelemetry.outPWM = telem.dutyOut / 100;  // PWM = Duty?

  // Status
  escTelemetry.statusFlag = telem.statusFlag;
}

// For debugging
void printRawEscData(byte buffer[]) {
  Serial.print(F("ESC DATA: "));
  for (int i = 0; i < ESC_DATA_V2_SIZE; i++) {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();
}

const STR_ESC_TELEMETRY_140& getEscTelemetry() {
  return escTelemetry;
}

void updateEscTelemetry() {
  // Flush the input to get to a fresh message.
  while (SerialESC.available() > 0) SerialESC.read();

  byte escDataV2[ESC_DATA_V2_SIZE];
  // TODO alert if no new data in 3 seconds
  if (SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE)) {
    //printRawEscData(escDataV2);
    parseEscSerialData(escDataV2);
  }
}

void setupEscTelemetry() {
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
}