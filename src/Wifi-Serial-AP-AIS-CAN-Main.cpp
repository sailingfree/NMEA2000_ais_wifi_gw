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

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 (Caution!!! Pin 2 before)
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <DallasTemperature.h>
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA0183.h>
#include <NMEA0183Messages.h>
#include <NMEA0183Msg.h>
//#include <NMEA2000.h>
//#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <OneButton.h>
#include <Preferences.h>
#include <Seasmart.h>
#include <Time.h>
//#include <WebServer.h>
#include <Wire.h>
#include <oled_func.h>

#include "BoatData.h"

//#include "List.h"
#include <AisHandler.h>
#include <GwPrefs.h>
#include <GwShell.h>
#include <Idle.h>
#include <StringStream.h>
#include <SysInfo.h>

#include <nmea2000Handlers.h>

#include <list>
#include <map>

#include "N2kDataToNMEA0183.h"
#include "N2kDeviceList.h"

#include <YDtoN2KUDP.h>

#include <i2c_scanner.h>    
#include <MyOta.h>
#include <MyMdns.h>
#include <MyWifi.h>
#include <activity.h>
#include <MyN2k.h>
#include <sensors.h>
#include <GwWebServer.h>
#include <N2ktoYD.h>

// Data wire for teperature (Dallas DS18B20) is plugged into GPIO 13 on the ESP32
#define ONE_WIRE_BUS 13

#define GWMODE "AIS "

// Global objects and variables
String hostName;
String Model = "Naiad N2K WiFi ";

Stream *OutputStream = NULL;  //&Serial;

// List of n2k devices for the device scanner
tN2kDeviceList *pN2kDeviceList;

const uint16_t ServerPort = 2222;  // Define the TCP port.
                                   // This is where server sends NMea0183 data.

// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream *Console = &Serial;

// Struct to update BoatData. See BoatData.h for content
tBoatData BoatData;

int NodeAddress = 32;  // To store last NMEA2000 Node Address

// Define the network servers
// The web server is on port 80 and defined in webServer.cpp

// TCP server for serving up NMEA0183
//static const size_t MaxClients = 10;
//WiFiServer server(ServerPort, MaxClients);

// A JSON server to provide JSON formatted output
WiFiServer json(90);

// The telnet server for the shell.
WiFiServer telnet(23);

// NMEA message handler for AIS receiving and multiplexing
tNMEA0183 AIS_NMEA0183;

String other_data;

void debug_log(char *str) {
#if ENABLE_DEBUG_LOG == 1
    Console->println(str);
#endif
}

/////// Variables
using namespace std;

// Map for received n2k messages. Logs the PGN and the count
std::map<int, int> N2kMsgMap;

// Map for received NMEA0183 messages. Logs the tag and the count
std::map<String, int> NMEA0183MsgMap;

// Map for the GPS info. Logs the sentences and values
std::map<String, String> Gps;

// Map for the satellites in view.
std::map<int, tGSV> Satellites;

// Map for all external sensors and their values
std::map<String, String> Sensors;

uint8_t chipid[6];
String macAddress;



// Create UDP instance for sending YD messages
WiFiUDP     udp;

// The UDP yacht data reader
YDtoN2kUDP  ydtoN2kUDP;

// The wifi UDP socket
WiFiUDP     wifiUdp;




// Main setup
void setup() {
    // Serial port 2 config (GPIO 16) for AIS
    const int ais_baudrate = 38400;
    const int rs_config = SERIAL_8N1;
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

    // Init USB serial port
    Serial.begin(115200);
    Console = &Serial;

    // Set the on board LED off
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialise the preferences
    GwPrefsInit();

    esp_err_t fuse_error = esp_efuse_mac_get_default(chipid);
    if (fuse_error) {
        Serial.printf("efuse error: %d\n", fuse_error);
    }

    for (i = 0; i < 6; i++) {
        if (i != 0) {
            macAddress += ":";
        }
        id += (chipid[i] << (7 * i));
        macAddress += String(chipid[i], HEX);
    }

    // Generate the hostname by appending the last two octets of the mac address to make it unique
    String hname = GwGetVal(GWHOST, "n2kgw");
    hostName = hname + String(chipid[4], HEX) + String(chipid[5], HEX);

    Serial.printf("Chipid %x %x %x %x %x %x id 0x%x MAC %s Hostname %s\n",
                  chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5],
                  id, macAddress.c_str(), hostName.c_str());

    // get CPU calibration timing
    calibrateCpu();

    initN2k(id);

    // Init AIS serial port 2
    // The AIS receiver I use is the NASA AIS Engine 3 device which outputs NMEA0183 at 38400 bps
    // https://www.nasamarine.com/product/ais-engine-3/
    Serial2.begin(ais_baudrate, rs_config);
    AIS_NMEA0183.Begin(&Serial2, 3, ais_baudrate);

    initSensors();
    
    oled_write(0, 0, "Initialising...");

    // Init the Wifi
    initWifi(hostName);

    // Update over air (OTA)
    initializeOTA();

    // Init Mdns - if enabled
    initializeMdns(hostName);

    // Register the services
    addMdnsService("http", "tcp", 80);  // Web server
    addMdnsService("telnet", "tcp", 23);  // Telnet server of RemoteDebug, register as telnet

    // Start TCP server
    //server.begin();

    // Start JSON server
    json.begin();

    // Start the telnet server
    telnet.begin();

    // start the YD UDP socket
    ydtoN2kUDP.begin(4445);

    // Start Web Server
    startWebserver();

    // Init the shell
    initGwShell();

    
    // Set product information
    Model += GWMODE;

    
    IdleInit();

    delay(200);
    Serial.println("Finished setup");
}


//*****************************************************************************

static WiFiClient telnetClient;

void disconnect() {
    telnetClient.stop();
}

void handleTelnet() {
    if (telnetClient && telnetClient.connected()) {
        // Got a connected client so use it
    } else {
        // See if there is a new connection and assign the new client
        telnetClient = telnet.available();
        if (telnetClient) {
            // Set up the client
            // telnetClient.setNoDelay(true); // More faster
            telnetClient.flush();  // clear input buffer, else you get strange characters
            setShellSource(&telnetClient);
        }
    }

    if (!telnetClient) {
        setShellSource(&Serial);
        return;
    }
}

void handle_json() {
    WiFiClient client = json.available();

    // Do we have a client?
    if (!client) return;

    // Read the request (we ignore the content in this example)
    while (client.available()) client.read();

    // Allocate JsonBuffer
    // Use arduinojson.org/assistant to compute the capacity.
    StaticJsonDocument<800> root;

    root["Latitude"] = BoatData.Latitude;
    root["Longitude"] = BoatData.Longitude;
    root["Heading"] = BoatData.Heading;
    root["COG"] = BoatData.COG;
    root["SOG"] = BoatData.SOG;
    root["STW"] = BoatData.STW;
    root["AWS"] = BoatData.AWS;
    root["TWS"] = BoatData.TWS;
    root["MaxAws"] = BoatData.MaxAws;
    root["MaxTws"] = BoatData.MaxTws;
    root["AWA"] = BoatData.AWA;
    root["TWA"] = BoatData.TWA;
    root["TWD"] = BoatData.TWD;
    root["TripLog"] = BoatData.TripLog;
    root["Log"] = BoatData.Log;
    root["WaterTemperature"] = BoatData.WaterTemperature;
    root["WaterDepth"] = BoatData.WaterDepth;
    root["Variation"] = BoatData.Variation;
    root["Altitude"] = BoatData.Altitude;
    root["GPSTime"] = BoatData.GPSTime;
    root["DaysSince1970"] = BoatData.DaysSince1970;

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
    handleAis();

    // Update the sensors
    updateSensors();

}
