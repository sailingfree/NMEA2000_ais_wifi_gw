// Define all common globals
#pragma once

#include <Arduino.h>
#include <BoatData.h>
#include <N2kMsg.h>
#include <WiFi.h>
#include <YDtoN2kUDP.h>
#include <NMEA2000.h>
#include <NMEA0183.h>
#include <map>

extern Stream * Console;
extern tBoatData BoatData;
extern WiFiUDP udp;
extern YDtoN2kUDP ydtoN2kUDP;
extern tNMEA2000 &NMEA2000;
extern String hostName;
extern tNMEA0183 AIS_NMEA0183;
extern uint16_t DaysSince1970;
extern double SecondsSinceMidnight;
extern std::map<String, String> Gps;
extern std::map<String, String> Sensors;
extern std::map<int, int> N2kMsgMap;


extern String WifiMode, WifiIP, WifiSSID;
extern String hostName, macAddress;

extern int NodeAddress;
extern String hostName;
extern String Model;
extern float temp;
extern float voltage;

