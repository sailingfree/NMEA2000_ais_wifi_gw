// System and network info
#pragma once

#include <Arduino.h>
#include <map>
#include <NMEA0183Messages.h>

extern std::map<String, String> Gps;
extern std::map<int, tGSV> Satellites;
extern std::map<String, String> Sensors;

extern String WifiMode, WifiIP, WifiSSID;
extern String host_name, macAddress;
extern String Model;


void getNetInfo(Stream & s);
void getSysInfo(Stream & s);
void getGps(Stream & s);
void getSatellites(Stream & s);
void getSensors(Stream & s);
int getCpuAvg(int core);
void getN2kMsgs(Stream & s);

