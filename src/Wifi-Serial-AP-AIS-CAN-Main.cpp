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
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <DallasTemperature.h>
#include <N2kMessages.h>

#include <N2kMsg.h>
#include <NMEA0183.h>
#include <NMEA0183Messages.h>
#include <NMEA0183Msg.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Seasmart.h>
#include <OneButton.h>
#include <Preferences.h>
#include <Seasmart.h>
#include <Time.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <bmp180_functions.h>
#include <bno055_functions.h>
#include <oled_func.h>

#include "BoatData.h"
#include "ESPmDNS.h"
//#include "List.h"
#include "N2kDataToNMEA0183.h"
#include "N2kDeviceList.h"
#include "NMEA0183Handlers.h"
#include <nmea2000Handlers.h>
#include <AisHandler.h>
#include <GwShell.h>
#include <map>
#include <StringStream.h>
#include <SysInfo.h>
#include <html_header.h>
#include <html_footer.h>
#include <GwPrefs.h>
#include <Idle.h>
#include <list>

template <class T>
using LinkedList = std::list<T>;


#define UDP_Forwarding 0    // Set to 1 for forwarding AIS from serial2 to UDP brodcast
#define HighTempAlarm 12    // Alarm level for fridge temperature (higher)
#define LowVoltageAlarm 11  // Alarm level for battery voltage (lower)

#define ADC_Calibration_Value  22.525 // 24.20  // 34.3 The real value depends on the true resistor values for the ADC input (100K / 22 K)

#define WLAN_CLIENT 0  // Set to 1 to enable client network. 0 to act as AP only

// Data wire for teperature (Dallas DS18B20) is plugged into GPIO 13 on the ESP32
#define ONE_WIRE_BUS 13

#define MiscSendOffset 120
#define VerySlowDataUpdatePeriod 10000  // temperature etc
#define SlowDataUpdatePeriod 1000       // Time between CAN Messages sent
#define FastDataUpdatePeriod 100       // Fast data period

#define USE_ARDUINO_OTA true
#define USE_MDNS true


// This mode is for a 0183 to n2K gateway like an Actisense NGW
// Note. this does not work fully yet.
//#define HAVE_NMEA0183 true

#if defined HAVE_NMEA0183 
////// Setup the pins for serial1 for the NMEA0183
////// These pins are behind the logic level shifter
#define RXD0 19   
#define TXD0 18
#define GWMODE "AIS & NMEA0183 GW"
#else
#define GWMODE "AIS "
#endif

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500 

// Global objects and variables
String host_name;
String Model = "Naiad N2K WiFi ";

Stream *OutputStream = NULL; //&Serial;

// List of n2k devices for the device scanner
tN2kDeviceList *pN2kDeviceList;

// Map for the wifi access points
typedef struct {
    String ssid;
    String pass;
} WiFiCreds;

static const uint16_t MaxAP = 2;
WiFiCreds wifiCreds [MaxAP];

// Wifi cofiguration Client and Access Point
String AP_password; // AP password  read from preferences
String AP_ssid;     // SSID for the AP constructed from the hostname

// Put IP address details here
const IPAddress AP_local_ip(192, 168, 15, 1);  // Static address for AP
const IPAddress AP_gateway(192, 168, 15, 1);
const IPAddress AP_subnet(255, 255, 255, 0);

int wifiType = 0;  // 0= Client 1= AP

const uint16_t ServerPort = 2222;  // Define the TCP port.
                                    // This is where server sends NMea0183 data. 

// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream * Console = & Serial;

IPAddress UnitIP;   // The address of this device. Could be client or AP

// UPD broadcast for Navionics, OpenCPN, etc.
const int YDudpPort = 4444;                   // port 4444 is for the Yacht devices interface

// Struct to update BoatData. See BoatData.h for content
tBoatData BoatData;

#ifdef HAVE_NMEA0183
// NMEA0183 for standard messages both sending and receive
tNMEA0183 STD_NMEA0183;
#endif

int NodeAddress = 32;  // To store last NMEA2000 Node Address

const size_t MaxClients = 10;
bool SendNMEA0183Conversion = true;  // Do we send NMEA2000 -> NMEA0183 conversion

// Define the network servers
// The web server on port 80
WebServer webserver(80);

// TCP server for serving up NMEA0183
WiFiServer server(ServerPort, MaxClients);

// A JSON server to provide JSON formatted output
WiFiServer json(90);

// The telnet server for the shell.
WiFiServer telnet(23);

// A list of connected TCP clients for the TCP NMEA0183 port
using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

#ifdef HAVE_NMEA0183
tN2kDataToNMEA0183 N2kDataToNMEA0183(&NMEA2000, &STD_NMEA0183);
#endif

int sendHeading;

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127489L,  // Engine dynamic
                                                  127488L,  // Engine fast dynamic
                                                  127250,   // Vessel heading
                                                  130577,   // Direction data
                                                  130310,   // Outside environmental
                                                  0};
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/  // System time
                                                 127250L,      // Heading
                                                 127258L,      // Magnetic variation
                                                 128259UL,     // Boat speed
                                                 128267UL,     // Depth
                                                 129025UL,     // Position
                                                 129026L,      // COG and SOG
                                                 129029L,      // GNSS
                                                 130306L,      // Wind
                                                 128275UL,     // Log
                                                 127245UL,     // Rudder
                                                 0};

// Battery voltage is connected GPIO 34 (Analog ADC1_CH6)
const int ADCpin = 34;
float voltage = 0;
float temp = 0;

// Serial port 2 config (GPIO 16) for AIS
const int ais_baudrate = 38400;
const int rs_config = SERIAL_8N1;

// Buffer config
#define MAX_NMEA0183_MESSAGE_SIZE 150

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

// Time

uint32_t mTimeToSec = 0;
uint32_t mTimeSeconds = 0;

// Forward declarations
void HandleNMEA2000Msg(const tN2kMsg &);
void SendNMEA0183Message(const tNMEA0183Msg &);
void GetTemperature(void *parameter);
void loadTimerFunc(TimerHandle_t xTimer);
void handleIndex();
void handleNotFound();
void clickedIt();

// Initialize the Arduino OTA
void initializeOTA() {
    // TODO: option to authentication (password)
    Console->println("OTA Started");

    // ArduinoOTA
    ArduinoOTA.onStart([]() {
                  String type;
                  if (ArduinoOTA.getCommand() == U_FLASH)
                      type = "sketch";
                  else  // U_SPIFFS
                      type = "filesystem";
                  Console->println("Start updating " + type);
              })
        .onEnd([]() {
            Console->println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Console->printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Console->printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Console->println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Console->println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Console->println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Console->println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Console->println("End Failed");
        });

    // Begin
    ArduinoOTA.begin();
}

String WifiMode = "Unknown";
String WifiSSID = "Unknown";
String WifiIP   = "Unknown";

// Connect to a wifi AP
// Try all the configured APs
bool connectWifi() {
    int wifi_retry = 0;

    Serial.printf("There are %d APs to try\n", MaxAP);

    for(int i = 0; i < MaxAP; i++) {
        Serial.printf("\nTrying %s\n", wifiCreds[i].ssid.c_str());
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiCreds[i].ssid.c_str(), wifiCreds[i].pass.c_str());
        wifi_retry = 0;

        while (WiFi.status() != WL_CONNECTED && wifi_retry < 20) {  // Check connection, try 5 seconds
            wifi_retry++;
            delay(500);
            Console->print(".");
        }
        Console->println("");
        if(WiFi.status() == WL_CONNECTED) {
            WifiMode = "Client";
            WifiSSID = wifiCreds[i].ssid;
            WifiIP = WiFi.localIP().toString();
            Console->printf("Connected to %s\n", wifiCreds[i].ssid.c_str());
            return true;
        } else {
            Console->printf("Can't connect to %s\n", wifiCreds[i].ssid.c_str());
        }
    }
    return false;
}

void disconnectWifi() {
    Console = &Serial;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WifiMode = "Not connected";
}

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

// HTML handlers
String html_start = HTML_start; //Read HTML contents
String html_end = HTML_end;
void handleRoot() {
 webserver.send(200, "text/html", html_start + html_end); //Send web page
}

void handleData() {
    String adcValue(random(100));
    webserver.send(200, "application/json", adcValue);  //Send ADC value only to client ajax request
}

void handleBoat() {
    StringStream boatData;
    boatData.printf("<pre>");
    boatData.printf("<h1>Boat Data</h1>");
    boatData.printf("<div class='info'>");
    displayBoat(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>NMEA2000 Devices</h1>");
    boatData.printf("<div class='info'>");
    ListDevices(boatData, true);
    boatData.printf("</div>");

    boatData.printf("<h1>Network</h1>");
    boatData.printf("<div class='info'>");
    getNetInfo(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>System</h1>");
    boatData.printf("<div class='info'>");
    getSysInfo(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>GPS</h1>");
    boatData.printf("<div class='info'>");
    getGps(boatData);
    getSatellites(boatData);

    boatData.printf("</div>");

    boatData.printf("<h1>Sensors</h1>");
    boatData.printf("<div class='info'>");
    getSensors(boatData);
    boatData.printf("</div>");

    boatData.printf("</pre>");
    webserver.send(200, "text/html", html_start + boatData.data.c_str() + html_end);  //Send web page
 
}

/**
 * @name: N2kToYD_Can
 */
void N2kToYD_Can(const tN2kMsg &msg, char *MsgBuf) {
    unsigned long DaysSince1970 = BoatData.DaysSince1970;
    double SecondsSinceMidnight = BoatData.GPSTime;
    int i, len;
    uint32_t canId = 0;
    char time_str[20];
    char Byte[5];
    unsigned int PF;
    time_t rawtime;
    struct tm ts;
    len = msg.DataLen;
    if (len > 134) {
        len = 134;
        Console->printf("Truncated from %d to 134\n", len);
    }

    // Set CanID

    canId = msg.Source & 0xff;
    PF = (msg.PGN >> 8) & 0xff;

    if (PF < 240) {
        canId = (canId | ((msg.Destination & 0xff) << 8));
        canId = (canId | (msg.PGN << 8));
    } else {
        canId = (canId | (msg.PGN << 8));
    }

    canId = (canId | (msg.Priority << 26));

    rawtime = (DaysSince1970 * 3600 * 24) + SecondsSinceMidnight;  // Create time from GNSS time;
    ts = *localtime(&rawtime);
    strftime(time_str, sizeof(time_str), "%T.000", &ts);  // Create time string

    snprintf(MsgBuf, 25, "%s R %0.8x", time_str, canId);  // Set time and canID

    for (i = 0; i < len; i++) {
        snprintf(Byte, 4, " %0.2x", msg.Data[i]);  // Add data fields
        strcat(MsgBuf, Byte);
    }
}

#define Max_YD_Message_Size 500
char YD_msg[Max_YD_Message_Size] = "";

// Create UDP instance
WiFiUDP udp;

// Send to Yacht device clients over udp using the cast address
void GwSendYD(const tN2kMsg &N2kMsg) {
    IPAddress udpAddress = WiFi.broadcastIP();
    udpAddress.fromString("192.168.15.255");
    N2kToYD_Can(N2kMsg, YD_msg);  // Create YD message from PGN
    udp.beginPacket(udpAddress, YDudpPort);  // Send to UDP
    udp.printf("%s\r\n", YD_msg);
    udp.endPacket();

  char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
  if ( N2kToSeasmart(N2kMsg,millis(),buf,MAX_NMEA2000_MESSAGE_SEASMART_SIZE)==0 ) return;
  
    udp.beginPacket(udpAddress, 4445);
    udp.println(buf);
    udp.endPacket();
}

// Handle some other gw messages from other gw nodes
void handle_gw_msgs(const tN2kMsg &N2kMsg) {
    ulong PGN = N2kMsg.PGN;

    if(PGN == 127508L) {
        GwSendYD(N2kMsg);
    }
}


// Main setup
void setup() {
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

    // get the MAC address
    esp_err_t fuse_error = esp_efuse_mac_get_default(chipid);
    if(fuse_error) {
        Serial.printf("efuse error: %d\n", fuse_error);
    }

    for (i = 0; i < 6; i++)  {
        if(i != 0) {
            macAddress += ":";
        }
        id += (chipid[i] << (7 * i));
        macAddress += String(chipid[i], HEX);
    }

    // Generate the hostname by appending the last two octets of the mac address to make it unique
    String hname = GwGetVal(GWHOST, "n2kgw");
    host_name = hname + String(chipid[4], HEX) + String(chipid[5], HEX);

    Serial.printf("Chipid %x %x %x %x %x %x id 0x%x MAC %s Hostname %s\n",
        chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5], 
        id, macAddress.c_str(), host_name.c_str());

      // get CPU calibration timing
    calibrateCpu();

#ifdef HAVE_NMEA0183
    // Init the serial port for the external NMEA0183 device
    // This device outputs NMEA0183 serial data at the standard 4800 bps.
    Serial1.begin(4800, SERIAL_8N1, RXD0, TXD0);
    Console->println("***Serial Txd is on pin: " + String(TXD0));
    Console->println("***Serial Rxd is on pin: " + String(RXD0));
#endif

   // Init AIS serial port 2
   // The AIS receiver I use is the NASA AIS Engine 3 device which outputs NMEA0183 at 38400 bps
   // https://www.nasamarine.com/product/ais-engine-3/
    Serial2.begin(ais_baudrate, rs_config);
    AIS_NMEA0183.Begin(&Serial2, 3, ais_baudrate);

    // start the i2c and if that succeeds start looking for the sensors
    if (Wire.begin()) {
        // Init the pressure and temperature sensor
        bmp180_init();

        // Init the BNo055 gyro
        bno055_init();

        // Init the oled display
        oled_init();
    }

    oled_write(0, 0, "Initialising...");

#ifdef HAVE_NMEA0183
   // Setup NMEA0183 ports and handlers
    InitNMEA0183Handlers(&NMEA2000, &BoatData);
    STD_NMEA0183.SetMsgHandler(HandleNMEA0183Msg);
    STD_NMEA0183.SetMessageStream(&Serial1);
    STD_NMEA0183.Open();
#endif

    // setup the WiFI map from the preferences
    wifiCreds[0].ssid = GwGetVal(SSID1);
    wifiCreds[0].pass = GwGetVal(SSPW1);
    wifiCreds[1].ssid = GwGetVal(SSID2);
    wifiCreds[1].pass = GwGetVal(SSPW2);

    // Setup params if we are to be an AP
    AP_password =       GwGetVal(GWPASS);

    if (WLAN_CLIENT == 1) {
        Console->println("Start WLAN Client");  // WiFi Mode Client
        delay(100);
        WiFi.setHostname(host_name.c_str());
        connectWifi();
    }
    
    if (WiFi.status() != WL_CONNECTED) {  // No client connection start AP
        // Init wifi connection
        Console->println("Start WLAN AP");  // WiFi Mode AP
        WiFi.mode(WIFI_AP);
        AP_ssid = host_name;
        WiFi.softAP(AP_ssid.c_str(), AP_password.c_str());
        delay(100);
        WiFi.softAPConfig(AP_local_ip, AP_gateway, AP_subnet);
        UnitIP = WiFi.softAPIP();
        Console->println("");
        Console->print("AP IP address: ");
        Console->println(UnitIP);
        wifiType = 1;
        oled_printf(0, lineh, "AP %s", UnitIP.toString().c_str());
        WifiMode = "AP";
        WifiIP = UnitIP.toString();
        WifiSSID = AP_ssid;

    } else {  // Wifi Client connection was sucessfull

        Console->println("");
        Console->println("WiFi client connected");
        Console->println("IP client address: ");
        Console->println(WiFi.localIP());
        UnitIP = WiFi.localIP();
        oled_printf(0, lineh, "Client %s", WiFi.localIP().toString().c_str());
    }

#ifdef USE_ARDUINO_OTA
    // Update over air (OTA)
    initializeOTA();
#endif

    // Register host name in mDNS
#if defined USE_MDNS

    if (MDNS.begin(host_name.c_str())) {
        Console->print("* MDNS responder started. Hostname -> ");
        Console->println(host_name);
    }

    // Register the services

#ifdef WEB_SERVER_ENABLED
    MDNS.addService("http", "tcp", 80);  // Web server
#endif

#ifndef DEBUG_DISABLED
    Console->println("Adding telnet");
    MDNS.addService("telnet", "tcp", 23);  // Telnet server of RemoteDebug, register as telnet
#endif

#endif  // MDNS

    // Start TCP server
    server.begin();

    // Start JSON server
    json.begin();

    // Start the telnet server
    telnet.begin();

    // Start Web Server
    webserver.on("/", handleRoot);
    webserver.on("/data", handleData);
    webserver.on("/boat", handleBoat);

    webserver.onNotFound(handleNotFound);

    webserver.begin();
    Console->println("HTTP server started");

    // Init the shell
    initGwShell();

    // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  
   NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);
    pN2kDeviceList = new tN2kDeviceList(&NMEA2000);

    // Set product information
    Model += GWMODE;

    NMEA2000.SetProductInformation(host_name.c_str(),               // Manufacturer's Model serial code
                                   100,                             // Manufacturer's product code
                                   Model.c_str(),  // Manufacturer's Model ID
                                   "1.0.0 (2021-06-11)",         // Manufacturer's Software version code
                                   "1.0.0 (2021-06-11)"           // Manufacturer's Model version
    );
    // Set device information
    NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                  130,  // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  25,   // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    oled_printf(0, lineh * 2, "My ID 0x%x", id);

    
    NMEA2000.SetConfigurationInformation("Naiad ",
                                         "Must be installed internally, not water or dust proof.",
                                         "Connect AIS NMEA at 34000");

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);  // Show in clear text. Leave uncommented for default Actisense format.
    
    NMEA2000.SetMsgHandler(handle_gw_msgs);

    NodeAddress = GwGetVal(LASTNODEADDRESS, "32").toInt();

    Console->printf("NodeAddress=%d\n", NodeAddress);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

    NMEA2000.ExtendTransmitMessages(TransmitMessages);
    NMEA2000.ExtendReceiveMessages(ReceiveMessages);
    NMEA2000.Open();

    IdleInit();

    delay(200);
    Serial.println("Finished setup");
}

//*****************************************************************************

void handleNotFound() {
    webserver.send(404, "text/plain", "File Not Found\n\n");
}

//*****************************************************************************
void SendBufToClients(const char *buf) {
    for (auto it = clients.begin(); it != clients.end(); it++) {
        if ((*it) != NULL && (*it)->connected()) {
            (*it)->println(buf);
        }
    }
}

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500

//*****************************************************************************
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
    if (!SendNMEA0183Conversion) return;

    char buf[MAX_NMEA0183_MESSAGE_SIZE];
    if (!NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE)) return;
    SendBufToClients(buf);
}

bool IsTimeToUpdate(unsigned long NextUpdate) {
    return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
    return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
    while (NextUpdate < millis()) NextUpdate += Period;
}

void SendN2kCompass() {
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;
    double fheading;
    int theading;

    if (IsTimeToUpdate(SlowDataUpdated)) {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        fheading = bno055_heading_rads();
        theading = fheading * 180 / PI;
        if (fheading != NAN) {
            Console->printf("Heading %f(Rads) %d(Deg)\n", fheading, theading);
            oled_printf(0, lineh * 2, "HDG %d deg %s", theading, sendHeading ? "On":"Off");
            SetN2kPGN127250(N2kMsg,
                            0,  // SID
                            fheading,
                            N2kDoubleNA,  // Deviation
                            N2kDoubleNA,  // Variation
                            N2khr_magnetic);

            NMEA2000.SendMsg(N2kMsg);

            SetN2kMagneticHeading(N2kMsg,
                                  0,
                                  fheading,
                                  N2kDoubleNA,
                                  N2kDoubleNA);

            if (sendHeading) {
                NMEA2000.SendMsg(N2kMsg);
                GwSendYD(N2kMsg);
            }

            Sensors["Heading"] = String(theading);
            Sensors["CompassCalib"] = String(bnoCalib());
        }
    }
}

void SendN2kEngineSlow() {
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated)) {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        SetN2kEngineDynamicParam(N2kMsg, 0,
                                 N2kDoubleNA,      // Oil Pressure
                                 N2kDoubleNA,      // Oil temp
                                 CToKelvin(temp),  // Coolant temp
                                 voltage,          // alternator voltage
                                 N2kDoubleNA,      // fule rate
                                 N2kDoubleNA,      // engine hours
                                 N2kDoubleNA,      // coolant pressure
                                 N2kDoubleNA,      // fuel pressure
                                 N2kInt8NA, N2kInt8NA, true);
        NMEA2000.SendMsg(N2kMsg);
        GwSendYD(N2kMsg);

        Sensors["Voltage"] = String(voltage);
    }
}

//*****************************************************************************
void AddClient(WiFiClient &client) {
    Console->println("New Client.");
    clients.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

//*****************************************************************************
void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
    Console->println("Client Disconnected.");
    (*it)->stop();
    it = clients.erase(it);
}

//*****************************************************************************
void CheckConnections() {
    WiFiClient client = server.available();  // listen for incoming clients

    if (client) AddClient(client);

    for (auto it = clients.begin(); it != clients.end(); it++) {
        if ((*it) != NULL) {
            if (!(*it)->connected()) {
                StopClient(it);
            } else {
                if ((*it)->available()) {
                    char c = (*it)->read();
                    if (c == 0x03) StopClient(it);  // Close connection by ctrl-c
                }
            }
        } else {
            it = clients.erase(it);  // Should have been erased by StopClient
        }
    }
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function

double ReadVoltage(byte pin) {
    double reading = analogRead(pin);  // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    if (reading < 1 || reading > 4095) return 0;
    // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
    return (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
}  // Added an improved polynomial, use either, comment out as required

void SendN2kEnvironment() {
    static unsigned long VerySlowDataUpdated = InitNextUpdate(VerySlowDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;
    double waterTemp = 0.0, outsideTemp = 0.0, pressure = 0.0;

    if (IsTimeToUpdate(VerySlowDataUpdated)) {
        SetNextUpdate(VerySlowDataUpdated, VerySlowDataUpdatePeriod);

        // read the sensors. The temperature should be done first
        // See hte bmp180 docs and examples
        outsideTemp = bmp180_temperature();
        pressure = bmp180_pressure();

        Console->printf("Pressure %f mbar\n", pressure / 100);
        Console->printf("Temp %f C\n", outsideTemp);

        oled_printf(0, 4 * lineh, "Temp %.1f Pres %.0f", outsideTemp, pressure / 100.0);

        SetN2kOutsideEnvironmentalParameters(N2kMsg, 0,
                                             waterTemp,
                                             outsideTemp,
                                             pressure);

        Sensors["Outside temp"] = String(outsideTemp);
        Sensors["Pressure"] = String(pressure / 100);
    }
}


static WiFiClient telnetClient;

void disconnect() {
    telnetClient.stop();
}

void handleTelnet() {

    if(telnetClient && telnetClient.connected()) {
        // Got a connected client so use it
    } else {
        // See if there is a new connection and assign the new client
        telnetClient = telnet.available();
        if(telnetClient) {
            // Set up the client
            //telnetClient.setNoDelay(true); // More faster
            telnetClient.flush(); // clear input buffer, else you get strange characters
            setShellSource(&telnetClient);
        }
    }

    if(!telnetClient) {
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
    root["BatteryVoltage"] = voltage;

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



void displayBoat(Stream & stream) {
    stream.printf("Latitude      %f\n", BoatData.Latitude);
    stream.printf("Longitude     %f\n",BoatData.Longitude);
    stream.printf("Heading       %f\n",BoatData.Heading);
    stream.printf("COG           %f\n",BoatData.COG);
    stream.printf("SOG           %f\n",BoatData.SOG);
    stream.printf("STW           %f\n",BoatData.STW);
    stream.printf("AWS           %f\n",BoatData.AWS);
    stream.printf("TWS           %f\n",BoatData.TWS);
    stream.printf("MaxAws        %f\n",BoatData.MaxAws);
    stream.printf("MaxTws        %f\n",BoatData.MaxTws);
    stream.printf("AWA           %f\n",BoatData.AWA);
    stream.printf("TWA           %f\n",BoatData.TWA);
    stream.printf("TWD           %f\n",BoatData.TWD);
    stream.printf("TripLog       %f\n",BoatData.TripLog);
    stream.printf("Log           %f\n",BoatData.Log);
    stream.printf("WaterTemp     %f\n",BoatData.WaterTemperature);
    stream.printf("WaterDepth    %f\n",BoatData.WaterDepth);
    stream.printf("Variation     %f\n",BoatData.Variation);
    stream.printf("Altitude      %f\n",BoatData.Altitude);
    stream.printf("GPSTime       %f\n",BoatData.GPSTime);
    stream.printf("DaysSince1970 %lu\n",BoatData.DaysSince1970);
    stream.printf("BatteryVolt   %f\n",voltage);

}


//*****************************************************************************
void PrintUlongList(const char *prefix, const unsigned long *List, Stream & stream) {
    uint8_t i;
    if (List != 0) {
        stream.printf(prefix);
        for (i = 0; List[i] != 0; i++) {
            if (i > 0) stream.print(", ");
            stream.printf("%lud", List[i]);
        }
        stream.println();
    }
}

//*****************************************************************************
void PrintText(const char *Text, bool AddLineFeed, Stream & stream) {
    if (Text != 0) 
        stream.print(Text);
    if (AddLineFeed) 
        stream.println();
}

//*****************************************************************************
void PrintDevice(const tNMEA2000::tDevice *pDevice, Stream & stream) {
    if (pDevice == 0) return;

    stream.printf("----------------------------------------------------------------------\n");
    stream.printf("%s\n", pDevice->GetModelID());
    stream.printf("  Source: %d\n", pDevice->GetSource());
    stream.printf("  Manufacturer code:        %d\n",pDevice->GetManufacturerCode());
    stream.printf("  Serial Code:              %s\n", pDevice->GetModelSerialCode());
    stream.printf("  Unique number:            %d\n", pDevice->GetUniqueNumber());
    stream.printf("  Software version:         %s\n", pDevice->GetSwCode());
    stream.printf("  Model version:            %s\n", pDevice->GetModelVersion());
    stream.printf("  Manufacturer Information: ");
    PrintText(pDevice->GetManufacturerInformation(), true, stream);
    stream.printf("  Installation description1: ");
    PrintText(pDevice->GetInstallationDescription1(), true, stream);
    stream.printf("  Installation description2: ");
    PrintText(pDevice->GetInstallationDescription2(), true, stream);
    PrintUlongList("  Transmit PGNs :", pDevice->GetTransmitPGNs(), stream);
    PrintUlongList("  Receive PGNs  :", pDevice->GetReceivePGNs(), stream);
    stream.printf("\n");
}

#define START_DELAY_IN_S 8
//*****************************************************************************
void ListDevices(Stream & stream, bool force = false) {
    static bool StartDelayDone = false;
    static int StartDelayCount = 0;
    static unsigned long NextStartDelay = 0;
  
    if (!StartDelayDone) {  // We let system first collect data to avoid printing all changes
        if (millis() > NextStartDelay) {
            if (StartDelayCount == 0) {
                stream.print("Reading device information from bus ");
                NextStartDelay = millis();
            }
            stream.print(".");
            NextStartDelay += 1000;
            StartDelayCount++;
            if (StartDelayCount > START_DELAY_IN_S) {
                StartDelayDone = true;
                stream.println();
            }
        }
        return;
    }
    if (!force && !pN2kDeviceList->ReadResetIsListUpdated()) return;

    stream.println();
    stream.println("**********************************************************************");
    for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
        PrintDevice(pN2kDeviceList->FindDeviceBySource(i), stream);
    }
}

// Main application loop.
void loop() {
    int wifi_retry;
    static time_t last = 0;
    static time_t last2 = 0;
    time_t now = time(NULL);;
    static int nexti = 0;
    const char spinner[] = {'|','/', '-', '\\'};

   // Get some values from the prefs
    sendHeading = GwGetVal(SENDHEADING, "0").toInt();

    // Process any n2k messages
    NMEA2000.ParseMessages();
 
#ifdef HAVE_NMEA0183
    // Read and handle the standard messages
    STD_NMEA0183.ParseMessages();
#endif

    StringStream s;
    if (now > last + 30) {
        ListDevices(s, true);
        last = now;
    } else {
        ListDevices(s, false);
    }
    Console->print(s.data);
 
    ArduinoOTA.handle();

    // every few hundred msecs

    if (millis() >= mTimeToSec) {
        // Time

        mTimeToSec = millis() + 250;

        mTimeSeconds++;

        // Blink the led

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        oled_printf(0, 0, "Up %c %s", spinner[nexti], host_name.c_str());
        nexti++;
        if(nexti >= 4) {
          nexti=0;
        }
    }
 
    // Handle any web server requests
    webserver.handleClient();

    // handle json requests
    handle_json();
 
    // handle the telnet session
    handleTelnet();
 
    // And run and shell commands
    handleShell();

    // Handle AIS messages
    handleAis();

    double AdcValue = ReadVoltage(ADCpin);
    voltage = ((voltage * 15) + (AdcValue * ADC_Calibration_Value / 4096)) / 16;  // This implements a low pass filter to eliminate spike for ADC readings

    SendN2kEnvironment();
  //  SendN2kCompass();
    SendN2kEngineSlow();
    CheckConnections(); 
    NMEA2000.ParseMessages();
 
    int SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != NodeAddress) {  // Save potentially changed Source Address to NVS memory
        NodeAddress = SourceAddress;     // Set new Node Address (to save only once)
        GwSetVal(LASTNODEADDRESS, String(SourceAddress));
        Console->printf("Address Change: New Address=%d\n", SourceAddress);
    }
 
 #if defined HAVE_NMEA0183
    N2kDataToNMEA0183.Update(&BoatData);
#endif

#if ENABLE_DEBUG_LOG == 2
    Console->print("Voltage:");
    Console->println(voltage);
    //Console->print("Temperature: ");Console->println(temp);
    Console->println("");
#endif
 
    if (wifiType == 0) {  // Check connection if working as client
        wifi_retry = 0;
        while (WiFi.status() != WL_CONNECTED && wifi_retry < 5) {  // Connection lost, 5 tries to reconnect
            wifi_retry++;
            Console->println("WiFi not connected. Try to reconnect");
            disconnectWifi();
            connectWifi();
        }
    }
}
