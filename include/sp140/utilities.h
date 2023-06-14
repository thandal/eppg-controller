// Copyright 2019 <Zach Whitehead>
// OpenPPG

/*
#ifndef _UTLITIES_H_
#define _UTLITIES_H_

#include <Arduino.h>

uint16_t crc16(uint8_t *buf, uint32_t size);

// Map double values
double mapd(double x, double in_min, double in_max, double out_min, double out_max);

// Map voltage to battery percentage, based on a
// simple set of data points from load testing.
float getBatteryPercent(float voltage);

String convertToDigits(byte digits);
int nextPage();
void addVSpace();

void setLEDs(byte state);
void blinkLED();

bool runVibe(unsigned int sequence[], int siz);

bool playMelody(uint16_t melody[], int siz);
void handleArmFail();

// for debugging
void printDeviceData();

String chipId();
void rebootBootloader();

void displayMeta();

#endif
*/