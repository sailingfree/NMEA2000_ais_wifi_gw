#include <Arduino.h>
#include <BoatData.h>
#include <GwDefs.h>

void displayBoat(Stream &stream) {
    stream.printf("Latitude      %f\n", BoatData.Latitude);
    stream.printf("Longitude     %f\n", BoatData.Longitude);
    stream.printf("Heading       %f\n", BoatData.Heading);
    stream.printf("COG           %f\n", BoatData.COG);
    stream.printf("SOG           %f\n", BoatData.SOG);
    stream.printf("STW           %f\n", BoatData.STW);
    stream.printf("AWS           %f\n", BoatData.AWS);
    stream.printf("TWS           %f\n", BoatData.TWS);
    stream.printf("MaxAws        %f\n", BoatData.MaxAws);
    stream.printf("MaxTws        %f\n", BoatData.MaxTws);
    stream.printf("AWA           %f\n", BoatData.AWA);
    stream.printf("TWA           %f\n", BoatData.TWA);
    stream.printf("TWD           %f\n", BoatData.TWD);
    stream.printf("TripLog       %f\n", BoatData.TripLog);
    stream.printf("Log           %f\n", BoatData.Log);
    stream.printf("WaterTemp     %f\n", BoatData.WaterTemperature);
    stream.printf("WaterDepth    %f\n", BoatData.WaterDepth);
    stream.printf("Variation     %f\n", BoatData.Variation);
    stream.printf("Altitude      %f\n", BoatData.Altitude);
    stream.printf("GPSTime       %f\n", BoatData.GPSTime);
    stream.printf("DaysSince1970 %lu\n", BoatData.DaysSince1970);
}
