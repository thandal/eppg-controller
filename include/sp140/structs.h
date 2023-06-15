// Copyright 2020 <Zach Whitehead>
#ifndef SP140_STRUCTS_H_
#define SP140_STRUCTS_H_

#include <stdint.h>

#pragma pack(push, 1)

// v1 ESC telemetry
typedef struct {
  float volts;
  float temperatureC;
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t statusFlag;
  uint16_t checksum;
} STR_ESC_TELEMETRY_140;

// Internal device data
typedef struct {
  uint8_t version_major;  // 5
  uint8_t version_minor;  // 1
  uint16_t armed_time;    // minutes (think Hobbs)
  uint8_t screen_rotation;  // 1,2,3,4 (90 deg)
  float sea_pressure;  // 1013.25 mbar
  bool metric_temp;    // true
  bool metric_alt;     // false
  uint8_t performance_mode;  // 0,1,2
  uint16_t batt_size;     // 4000 (4kw) or 2000 (2kw)
  uint8_t btn_mode;     // for future use
  uint8_t unused;     // for future use
  uint16_t crc;        // error check
} STR_DEVICE_DATA_140_V1;

// v2 ESC telemetry
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
} telem_t;

// Tone struct (passed between rp2040 cores)
typedef union {
  struct fields {
    uint16_t freq;
    uint16_t duration;
  } f;
  uint32_t data; // All packed up in a uint32.
} STR_NOTE;

#pragma pack(pop)

#endif  // SP140_STRUCTS_H_