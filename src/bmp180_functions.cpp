// bmp180 functions

#include <Arduino.h>
#include <Wire.h>
#include <bmp180_functions.h>

// create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);

static bool hasBMP180 = false;

void bmp180Init() {
    // Initilaise the bmp180 pressure/tempertaure sensor
    // begin() initializes the interface, checks the sensor ID and reads the calibration parameters.
    if (!bmp180.begin()) {
        Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
        return;
    }

    // reset sensor to default parameters.
    bmp180.resetToDefaults();

    // enable ultra high resolution mode for pressure measurements
    bmp180.setSamplingMode(BMP180MI::MODE_UHR);

    hasBMP180 = true;
}

double bmp180Temperature() {
    double temp = 0.0;
    int count = 100;
    bool hasTemperature = false;

    if (!hasBMP180) {
        return 0.0;
    }

    // start a temperature measurement
    if (!bmp180.measureTemperature()) {
        Serial.println("could not start temperature measurement, is a measurement already running?");
        return 0.0;
    }

    // wait for the measurement to finish. proceed as soon as hasValue() returned true.
    do {
        delay(100);
    } while (!(hasTemperature = bmp180.hasValue()) && --count);

    if (hasTemperature) {
        temp = bmp180.getTemperature();
    }
    return temp;
}

double bmp180Pressure() {
    double pressure = 0.0;
    int count = 10;
    bool hasPressure = false;

    if (!hasBMP180) {
        return pressure;
    }

    // start a pressure measurement. pressure measurements depend on temperature measurement, you should only start a pressure
    // measurement immediately after a temperature measurement.
    if (!bmp180.measurePressure()) {
        Serial.println("could not start perssure measurement, is a measurement already running?");
        return pressure;
    }

    // wait for the measurement to finish. proceed as soon as hasValue() returned true.
    do {
        delay(100);
    } while (!(hasPressure = bmp180.hasValue()) && --count);
    if (hasPressure) {
        pressure = bmp180.getPressure();
    }

    return pressure;
}