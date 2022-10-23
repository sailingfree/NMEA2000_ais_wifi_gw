// bno055 functions
#include <bno055_functions.h>
#include <GwPrefs.h>

// The BNo055 gyro/compass
static BNO bno;  //create bno from the Class BNO
static bool bnoSaved = false;
static bool hasBno55 = false;
static double offset = 0.0;

uint32_t I2C_Freq = 100000;
#define SDA_0 21
#define SCL_0 22
// Initialise the bno055 device
void bno055_init(void) {
    uint8_t error;
    int address = BNO_ADDR;
    Serial.printf("Starting the bno055 setup %d %d %d\n", SDA_0, SCL_0, I2C_Freq);
    TwoWire I2C_0 = TwoWire(0);

    I2C_0.begin(SDA_0, SCL_0, I2C_Freq);

    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    I2C_0.beginTransmission(address);
    error = I2C_0.endTransmission();

    Serial.printf("i2c error return %d\n", error);

    if (error == 0) {
        Serial.printf("Found a BNo055 at address 0x%x\n", address);
    } else {
        Serial.printf("No BNo055 device found at address 0x%x\n", address);
        return;
    }

    bno.loadOffsets(100);

    bno.startBNO(200, false);  //enables high_g interrupts and puts
                               // the compass into fusion output mode
                               // NDOF. First parameter controls the
                               // threshold for the interrupt (0-255),
                               // the second one enables INT pin forwarding.
                               // the calibrartion is checked in the main loop
                               // to prevent the setup stalling
    
    // Get the compass offset. This is to compensate for the physical
    // orientation of the compass in the boat
    String val = GwGetVal(COMPASSOFF, "0");
    offset = val.toDouble();

    hasBno55 = true;
}

// Calibration values
int bnoCalib() {
    return bno.getCalibration();
}

// heading in radians
// Return NAN if the gyro is not calibrated
double bno055_heading_rads() {
    int16_t iheading;
    double fheading;
    if (!hasBno55) {
        return NAN;
    }

    // Get the heading in degrees
    iheading = bno.getHeading();

    fheading = iheading + 0.0;
    fheading = fheading * PI / 180;
  
#if 0
    bno.getCalibration();

    // Check the compass calibration first
    if (!bno.isCalibrated()) {
        bno.serialPrintCalibStat();  //print the current calibration levels via serial
        return NAN;                  // Not calibrated so do nothing
    } else if (!bnoSaved) {
        bno.saveOffsets(100);
        bnoSaved = true;
    }
#endif
    return fheading;
}

// Convert to degrees.
double bno055_heading_degs() {
    double h = bno055_heading_rads();
    if(h != NAN) {
        h *= RAD_TO_DEG;
    }
    return h;
}