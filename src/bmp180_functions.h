//
// Copyright (c) 2018 Gregor Christandl
//
// connect the BMP180 to the Arduino like this:
// Arduino - BMC180
// 5V ------ VCC
// GND ----- GND
// SDA ----- SDA
// SCL ----- SCL

#define I2C_Freq 100000
#define SDA_0 21
#define SCL_0 22

#include <Arduino.h>
#include <BMP180I2C.h>
#include <Wire.h>

#define I2C_ADDRESS 0x77

void bmp180_init();
double bmp180_temperature(void);
double bmp180_pressure(void);
