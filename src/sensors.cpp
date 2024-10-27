#include <Arduino.h>
#include <sensors.h>
#include <bmp180_functions.h>
#include <oled_func.h>

// Battery voltage is connected GPIO 34 (Analog ADC1_CH6)
const int ADCpin = 34;
float voltage = 0;
float temp = 0;

void initSensors() {
    // start the i2c and if that succeeds start looking for the sensors
    if (Wire.begin()) {
        // Init the pressure and temperature sensor
        bmp180_init();

        // Init the oled display
        oled_init();
    }

}

void updateSensors() {
    static const uint32_t window = 16;   // Size of the moving average windows
 
    double AdcValue = ReadVoltage(ADCpin);
    voltage = ((voltage * (window - 1)) + (AdcValue * ADC_Calibration_Value / 4096)) / window;  // This implements a low pass filter to eliminate spike for ADC readings
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage(byte pin) {
    double reading = analogRead(pin);  // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    if (reading < 1 || reading > 4095) return 0;
    // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
    return (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
}  // Added an improved polynomial, use either, comment out as required

