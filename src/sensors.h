#pragma once

#include <Arduino.h>

#define ADC_Calibration_Value 22.525  // 24.20  // 34.3 The real value depends on the true resistor values for the ADC input (100K / 22 K)

void updateSensors();
void initSensors();
double ReadVoltage(byte pin);