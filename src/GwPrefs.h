// preferences

#pragma once

#include <Arduino.h>

void GwPrefsInit();
String GwSetVal(const char * key, String val);
String GwGetVal(const char * key, String defval = "---");
void GwListRegs(Stream &s);
bool isGwKey(String k);
void GwPrint(Stream &s);

// WiFi mode. Can be AP CL OFF
#define WIFIMODE "wifi"

// the keys we support
// SSID for the two optional wifi networks to try
#define SSID1 "ssid1"
#define SSID2 "ssid2"
#define SSPW1 "sspw1"
#define SSPW2 "sspw2"

// The AP name and password when operating in AP mode
#define GWSSID "gwssid"
#define GWPASS "gwpass"

// Diameter of the engine pulley
#define ENGINEDIA "enginedia"

// Diameter fo the alternator pulley
#define ALTDIA "altdia"

// Number of alternator poles
#define ALTPOLES "poles"

// Pressure calibration in mbar
#define PRESSCAL "presscal"

// Compass offset value in degrees
#define COMPASSOFF "compassoff"

// Hostname advertised by the network
#define GWHOST "gwhost"

// The N2K node address we last used
#define LASTNODEADDRESS "LastNodeAddress"

// Whether to use the internal GPS
#define USEGPS "usegps"

// Whether to use the internal compass
#define SENDHEADING "useheading"