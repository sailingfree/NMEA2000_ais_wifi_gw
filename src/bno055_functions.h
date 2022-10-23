// bno055 header

#include <Wire.h>
#include <BohleBots_BNO055.h>

void bno055_init(void);

// heading in radians
// NaN if no heading found
double bno055_heading_rads();
double bno055_heading_degs();
int    bnoCalib();