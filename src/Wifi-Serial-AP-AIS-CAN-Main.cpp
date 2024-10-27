/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Version 1.3, 04.08.2020, AK-Homberger
// Version 1.0  20.may.2021 P Martin

#include <Arduino.h>
#include <oled_func.h>
#include <AisHandler.h>
#include <GwPrefs.h>
#include <GwShell.h>
#include <Idle.h>
#include <StringStream.h>
#include <SysInfo.h>
#include <YDtoN2KUDP.h>
#include <MyOta.h>
#include <MyMdns.h>
#include <MyWifi.h>
#include <activity.h>
#include <MyN2k.h>
#include <sensors.h>
#include <GwWebServer.h>
#include <N2ktoYD.h>
#include <GwTelnet.h>
#include <GwJSON.h>

#define GWMODE "AIS "

// Global objects and variables
String hostName;
String modelName = "Naiad N2K WiFi ";

Stream *outputStream = NULL;  //&Serial;

const uint16_t serverPort = 2222;  // Define the TCP port.
                                   // This is where server sends NMea0183 data.

// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream *Console = &Serial;

int nodeAddress = 32;  // To store last NMEA2000 Node Address

// Define the network servers
// The web server is on port 80 and defined in webServer.cpp

// A JSON server to provide JSON formatted output
WiFiServer jsonServer(90);

// The telnet server for the shell.
WiFiServer telnetServer(23);

// NMEA message handler for AIS receiving and multiplexing
tNMEA0183 AIS_NMEA0183;

void debug_log(char *str) {
#if ENABLE_DEBUG_LOG == 1
    Console->println(str);
#endif
}

/////// Variables
using namespace std;

String macAddress;

// Main setup
void setup() {
    // Serial port 2 config (GPIO 16) for AIS
    const int aisBaudrate = 38400;
    const int aisSerialConfig = SERIAL_8N1;
    uint8_t chipId[6];
    uint32_t id = 0;
    int i = 0;

    // Init USB serial port
    Serial.begin(115200);
    Console = &Serial;

    // Set the on board LED off
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialise the preferences
    gwPrefsInit();

    esp_err_t fuseError = esp_efuse_mac_get_default(chipId);
    if (fuseError) {
        Serial.printf("efuse error: %d\n", fuseError);
    }

    for (i = 0; i < 6; i++) {
        if (i != 0) {
            macAddress += ":";
        }
        id += (chipId[i] << (7 * i));
        macAddress += String(chipId[i], HEX);
    }

    // Generate the hostname by appending the last two octets of the mac address to make it unique
    String hname = gwGetVal(GWHOST, "n2kgw");
    hostName = hname + String(chipId[4], HEX) + String(chipId[5], HEX);

    Serial.printf("Chipid %x %x %x %x %x %x id 0x%x MAC %s Hostname %s\n",
                  chipId[0], chipId[1], chipId[2], chipId[3], chipId[4], chipId[5],
                  id, macAddress.c_str(), hostName.c_str());

    // get CPU calibration timing
    calibrateCpu();

    initN2k(id);

    // Init AIS serial port 2
    // The AIS receiver I use is the NASA AIS Engine 3 device which outputs NMEA0183 at 38400 bps
    // https://www.nasamarine.com/product/ais-engine-3/
    Serial2.begin(aisBaudrate, aisSerialConfig);
    AIS_NMEA0183.Begin(&Serial2, 3, aisBaudrate);

    initSensors();
    
    oledWrite(0, 0, "Initialising...");

    // Init the Wifi
    initWifi(hostName);

    // Update over air (OTA)
    initOTA();

    // Init Mdns - if enabled
    initMdns(hostName);

    // Register the services
    addMdnsService("http", "tcp", 80);  // Web server
    addMdnsService("telnet", "tcp", 23);  // Telnet server of RemoteDebug, register as telnet

    // Start JSON server
    jsonServer.begin();

    // Start the telnet server
    telnetServer.begin();

    // Start Web Server
    startWebserver();

    // Init the shell
    initGwShell();
    
    // Set product information
    modelName += GWMODE;
    
    initIdle();

    delay(200);
    Serial.println("Finished setup");
}




// Main application loop.
void loop() {

    static time_t last = 0;
    static time_t last2 = 0;
    time_t now = time(NULL);

    // Process any n2k messages
    handleN2k();

    // Process any received YD messages
    YDWork();

    StringStream s;
    if (now > last + 30) {
        ListDevices(s, true);
        last = now;
    } else {
        ListDevices(s, false);
    }
    Console->print(s.data);

    // Handle any over the air updates
    handleOta();

    // Show we have some activity on the oled screen
    showActivity();
    
    // Handle any web server requests
    handleWebserver();
   
    // handle json requests
    handle_json();

    // handle the telnet session
    handleTelnet();

    // And run and shell commands
    handleShell();

    // Handle AIS messages
    handleAIS();

    // Update the sensors
    updateSensors();

}
