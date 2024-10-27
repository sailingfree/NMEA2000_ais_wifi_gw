#include <Arduino.h>
#include <GwDefs.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>


void handle_json() {
    WiFiClient client = jsonServer.available();

    // Do we have a client?
    if (!client) return;

    // Read the request (we ignore the content in this example)
    while (client.available()) client.read();

    // Allocate JsonBuffer
    // Use arduinojson.org/assistant to compute the capacity.
    StaticJsonDocument<800> root;

    root["Latitude"] = boatData.latitude;
    root["Longitude"] = boatData.longitude;
    root["Heading"] = boatData.heading;
    root["COG"] = boatData.COG;
    root["SOG"] = boatData.SOG;
    root["STW"] = boatData.STW;
    root["AWS"] = boatData.AWS;
    root["TWS"] = boatData.TWS;
    root["MaxAws"] = boatData.MaxAws;
    root["MaxTws"] = boatData.MaxTws;
    root["AWA"] = boatData.AWA;
    root["TWA"] = boatData.TWA;
    root["TWD"] = boatData.TWD;
    root["TripLog"] = boatData.TripLog;
    root["Log"] = boatData.Log;
    root["WaterTemperature"] = boatData.WaterTemperature;
    root["WaterDepth"] = boatData.WaterDepth;
    root["Variation"] = boatData.variation;
    root["Altitude"] = boatData.altitude;
    root["GPSTime"] = boatData.GPSTime;
    root["DaysSince1970"] = boatData.daysSince1970;

    // Write response headers
    client.println("HTTP/1.0 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();

    // Write JSON document
    serializeJsonPretty(root, client);

    // Disconnect
    client.stop();
}

