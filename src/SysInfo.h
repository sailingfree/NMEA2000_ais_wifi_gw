// System and network info
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

#include <Arduino.h>
#include <NMEA0183Messages.h>

#include <map>

extern std::map<String, String> mapGps;
extern std::map<int, tGSV> mapSatellites;
extern std::map<String, String> mapSensors;

extern String wifiMode, wifiIP, wifiSSID;
extern String hostName, macAddress;
extern String modelName;

void getNetInfo(Stream& s);
void getSysInfo(Stream& s);
void getGps(Stream& s);
void getSatellites(Stream& s);
void getSensors(Stream& s);
int  getCpuAvg(int core);
void getN2kMsgs(Stream& s);
void getYdMsgs(Stream& s);
