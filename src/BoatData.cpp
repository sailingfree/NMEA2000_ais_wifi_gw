#include <Arduino.h>
#include <BoatData.h>
#include <GwDefs.h>

// Struct to update BoatData. See BoatData.h for content
tBoatData boatData;

void displayBoat(Stream &stream) {
    stream.printf("Latitude      %f\n", boatData.latitude);
    stream.printf("Longitude     %f\n", boatData.longitude);
    stream.printf("Heading       %f\n", boatData.heading);
    stream.printf("COG           %f\n", boatData.COG);
    stream.printf("SOG           %f\n", boatData.SOG);
    stream.printf("STW           %f\n", boatData.STW);
    stream.printf("AWS           %f\n", boatData.AWS);
    stream.printf("TWS           %f\n", boatData.TWS);
    stream.printf("MaxAws        %f\n", boatData.MaxAws);
    stream.printf("MaxTws        %f\n", boatData.MaxTws);
    stream.printf("AWA           %f\n", boatData.AWA);
    stream.printf("TWA           %f\n", boatData.TWA);
    stream.printf("TWD           %f\n", boatData.TWD);
    stream.printf("TripLog       %f\n", boatData.TripLog);
    stream.printf("Log           %f\n", boatData.Log);
    stream.printf("WaterTemp     %f\n", boatData.WaterTemperature);
    stream.printf("WaterDepth    %f\n", boatData.WaterDepth);
    stream.printf("Variation     %f\n", boatData.variation);
    stream.printf("Altitude      %f\n", boatData.altitude);
    stream.printf("GPSTime       %f\n", boatData.GPSTime);
    stream.printf("DaysSince1970 %lu\n", boatData.daysSince1970);
}
