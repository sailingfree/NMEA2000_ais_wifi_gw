/*
Copyright (c) 2022 Peter Martin www.naiadhome.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

struct tBoatData {
    unsigned long daysSince1970;  // Days since 1970-01-01

    double trueHeading, SOG, COG, variation,
        GPSTime,  // Secs since midnight,
        latitude, longitude, altitude, HDOP, geoidalSeparation, DGPSAge,
        heading, HeadingInt, STW, AWS, MaxAws, MaxTws, AWA, TWA, AWD, TWD, TWS, TripLog, Log, RudderPosition, WaterTemperature,
        WaterDepth;

    int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
    bool MOBActivated;

   public:
    tBoatData() {
        trueHeading = 0;
        SOG = 0;
        COG = 0;
        variation = 7.0;
        GPSTime = 0;
        altitude = 0;
        HDOP = 100000;
        DGPSAge = 100000;
        daysSince1970 = 0;
        MOBActivated = false;
        SatelliteCount = 0;
        DGPSReferenceStationID = 0;
        heading = 0;
        HeadingInt = 0;
        latitude = 0;
        MaxAws = MaxTws = AWA = TWA = TWS = TWD = TripLog = RudderPosition = WaterTemperature = WaterDepth = 0;
    };
};

void displayBoat(Stream &stream);
