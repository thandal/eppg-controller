#include "sp140/config.h"
#include "sp140/esc_telemetry.h"
#include "sp140/structs.h"

#include <Arduino.h>
#include <CircularBuffer.h>        // smooth out readings

// v2 ESC serial telemetry struct
typedef struct  {
  // Voltage
  int V_HI;
  int V_LO;

  // Temperature
  int T_HI;
  int T_LO;

  // Current
  int I_HI;
  int I_LO;

  // Reserved
  int R0_HI;
  int R0_LO;

  // eRPM
  int RPM0;
  int RPM1;
  int RPM2;
  int RPM3;

  // Input Duty
  int DUTYIN_HI;
  int DUTYIN_LO;

  // Motor Duty
  int MOTORDUTY_HI;
  int MOTORDUTY_LO;

  // Reserved
  int R1;

  // Status Flags
  int statusFlag;

  // checksum
  int CSUM_HI;
  int CSUM_LO;
} STR_ESC_TELEMETRY_140_V2;

static STR_ESC_TELEMETRY_140 escTelemetry;
CircularBuffer<float, 50> voltsBuffer;
unsigned long prevWattHoursMillis = 0;

int CheckFlectcher16(byte byteBuffer[]) {
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    // Check only first 18 bytes, skip crc bytes and stop bytes
    for (int i = 0; i < ESC_DATA_V2_SIZE - 4; i++) { 
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

//  // Check the fletcher checksum
//  const int checkFletch = CheckFlectcher16(buffer);
  STR_ESC_TELEMETRY_140_V2 escTelemetryV2;

  escTelemetryV2.CSUM_HI = buffer[19];
  escTelemetryV2.CSUM_LO = buffer[18];

  // TODO alert if no new data in 3 seconds
  int checksum = (int)(((escTelemetryV2.CSUM_HI << 8) + escTelemetryV2.CSUM_LO));

//  // Checksums do not match
//  if (checkFletch != checksum) {
//    Serial.println("checksum error");
//    return;
//  }

  // Voltage
  escTelemetryV2.V_HI = buffer[1];
  escTelemetryV2.V_LO = buffer[0];

  float volts = (escTelemetryV2.V_HI << 8 | escTelemetryV2.V_LO) / 100.0;
  if (volts > BATT_MIN_V) {
    volts += 1.0; // calibration
  }

  // Compute average of the ring buffer for voltage readings
  voltsBuffer.push(volts);
  float avgVolts = 0.0;
  for (decltype(voltsBuffer)::index_t i = 0; i < voltsBuffer.size(); ++i) {
    avgVolts += voltsBuffer[i] / voltsBuffer.size();
  }
  escTelemetry.volts = avgVolts;

  // Temperature
  escTelemetryV2.T_HI = buffer[3];
  escTelemetryV2.T_LO = buffer[2];

  float rawVal = (float)((escTelemetryV2.T_HI << 8) + escTelemetryV2.T_LO);

  static int SERIESRESISTOR = 10000;
  static int NOMINAL_RESISTANCE = 10000;
  static int NOMINAL_TEMPERATURE = 25;
  static int BCOEFFICIENT = 3455;

  //convert value to resistance
  float Rntc = (4096 / (float) rawVal) - 1;
  Rntc = SERIESRESISTOR / Rntc;

  // Get the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE; // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celcius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    temperature = 0;
  }

  temperature = (float) trunc(temperature * 100) / 100; // 2 decimal places
  escTelemetry.temperatureC = temperature;

  // Current
  const int16_t amps = word(buffer[5], buffer[4]);
  escTelemetry.amps = amps / 12.5;

  escTelemetry.watts = escTelemetry.amps * escTelemetry.volts;

  // Update wattHours
  const unsigned long currentMillis= millis();
  const float deltaHours = (currentMillis - prevWattHoursMillis) / 1000.0 / 3600.0;
  prevWattHoursMillis = currentMillis;
  escTelemetry.wattHours += round(escTelemetry.watts * deltaHours);

  // Reserved
  escTelemetryV2.R0_HI = buffer[7];
  escTelemetryV2.R0_LO = buffer[6];

  // eRPM
  escTelemetryV2.RPM0 = buffer[11];
  escTelemetryV2.RPM1 = buffer[10];
  escTelemetryV2.RPM2 = buffer[9];
  escTelemetryV2.RPM3 = buffer[8];

  const int poleCount = 62;
  int currentERPM = (int)((escTelemetryV2.RPM0 << 24) + (escTelemetryV2.RPM1 << 16) + (escTelemetryV2.RPM2 << 8) + (escTelemetryV2.RPM3 << 0)); //ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  escTelemetry.rpm = currentRPM;

  // Input Duty
  escTelemetryV2.DUTYIN_HI = buffer[13];
  escTelemetryV2.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((escTelemetryV2.DUTYIN_HI << 8) + escTelemetryV2.DUTYIN_LO) / 10);
  escTelemetry.inPWM = (throttleDuty / 10);  // PWM = Duty? Divide by 10?

  // Motor Duty
  escTelemetryV2.MOTORDUTY_HI = buffer[15];
  escTelemetryV2.MOTORDUTY_LO = buffer[14];
  int motorDuty = (int)(((escTelemetryV2.MOTORDUTY_HI << 8) + escTelemetryV2.MOTORDUTY_LO) / 10);
  escTelemetry.outPWM = (motorDuty / 10);  // PWM = Duty? Divide by 10?

  // Reserved
  // escTelemetryV2.R1 = buffer[17];

  escTelemetryV2.statusFlag = buffer[16];
  escTelemetry.statusFlag = escTelemetryV2.statusFlag;
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
  if (SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE)) {
    //printRawEscData(escDataV2);
    parseEscSerialData(escDataV2);
  }
}

void setupEscTelemetry() {
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
}