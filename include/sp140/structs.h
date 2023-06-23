// Copyright 2020 <Zach Whitehead>
#ifndef SP140_STRUCTS_H_
#define SP140_STRUCTS_H_

#include <stdint.h>

#pragma pack(push, 1)

// ESC telemetry
typedef struct {
  float volts;
  float temperatureC;
  float amps;
  float watts;
  float wattHours;
  float rpm;
  float inPWM;
  float outPWM;
  uint8_t statusFlag;
} STR_ESC_TELEMETRY_140;

// Device configuration data
typedef struct {
  uint8_t version_major;     // VERSION_MAJOR from config.h
  uint8_t version_minor;     // VERSION_MINOR from config.h
  uint16_t armed_seconds;    // seconds
  uint8_t screen_rotation;   // 1 = left hand, 2, 3 = right hand, 4 (90 deg)
  float sea_pressure;        // 1013.25 mbar
  bool metric_temp;          // Display temperature in C/F
  bool metric_alt;           // Display altitude in m/ft
  uint8_t performance_mode;  // 0 = CHILL, 1 = SPORT
  uint16_t batt_size;        // 4000 (4kw) or 2000 (2kw)
  uint8_t btn_mode;          // for future use
  uint8_t unused;            // for future use
  uint16_t crc;              // crc
} STR_DEVICE_DATA_140_V1;

// Note struct (passed between rp2040 cores)
typedef union {
  struct fields {
    uint16_t freq;
    uint16_t duration;
  } f;
  uint32_t data; // All packed up in a uint32.
} STR_NOTE;

#pragma pack(pop)

#endif  // SP140_STRUCTS_H_