#pragma once

#include <Arduino.h>

void initN2k(uint32_t id);
void handleN2k();
double readVoltage(byte pin);
void ListDevices(Stream &stream, bool force);

