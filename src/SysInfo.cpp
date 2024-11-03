// System and net info
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

#include <Arduino.h>
#include <ESP.h>
#include <GwPrefs.h>
#include <NMEA0183Messages.h>
#include <SysInfo.h>
#include <esp_wifi.h>

#include "uptime_formatter.h"

// Map for received n2k messages. Logs the PGN and the count
std::map<int, int> mapN2kMsg;

// Map for the YD received messages on port 4445
std::map<int, int> mapYdMsg;

// Map for the GPS info. Logs the sentences and values
std::map<String, String> mapGps;

// Map for the satellites in view.
std::map<int, tGSV> mapSatellites;

// Map for all external sensors and their values
std::map<String, String> mapSensors;




void getNetInfo(Stream &s) {
    wifi_sta_list_t wifi_sta_list;
    tcpip_adapter_sta_list_t adapter_sta_list;

    s.println("=========== NETWORK ==========");
    s.printf("HOST NAME: %s\n", hostName.c_str());
    s.printf("MAC: %s\n", macAddress.c_str());
    s.printf("WifiMode %s\n", wifiMode.c_str());
    s.printf("WifiIP %s\n", wifiIP.c_str());
    s.printf("WifiSSID %s\n", wifiSSID.c_str());

    // Get the wifi station list and list the connected device details
    memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
    memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    if (adapter_sta_list.num) {
        s.println("----- DHCP clients ---------");
        for (int i = 0; i < adapter_sta_list.num; i++) {
            tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];

            s.printf("Station number %d \n", i);
            s.printf("MAC: ");
            for (int j = 0; j < 6; j++) {
                s.printf("%02X", station.mac[j]);
                if (i < 5) s.print(":");
            }
            s.printf("\nIP: ");
            s.println(ip4addr_ntoa((const ip4_addr_t *)&(station.ip)));
            s.println("");
        }
    }

    s.println("=========== END ==========");
}

void getSysInfo(Stream &s) {
    EspClass esp;

    uint32_t heapSize = esp.getHeapSize();      // total heap size
    uint32_t heapFree = esp.getFreeHeap();  // available heap
    uint32_t heapUsedPc = (heapSize - heapFree) * 100 / heapSize;

    uint8_t chipRev = esp.getChipRevision();
    const char *chipModel = esp.getChipModel();
    uint32_t sketchSize = esp.getSketchSize();
    uint32_t sketchFree = esp.getFreeSketchSpace();
    uint32_t flashSize = esp.getFlashChipSize();
    uint32_t flashUsedPc = (flashSize - sketchFree) * 100 / flashSize;
    uint64_t efuseMAC = esp.getEfuseMac();
    String uptime = uptime_formatter::getUptime();
    String node = gwGetVal(LASTNODEADDRESS);

    s.println("=========== SYSTEM ==========");
    s.printf("Model %s\n", modelName.c_str());
    s.printf("Node: %s\n", node.c_str());
    s.printf("Uptime: %s", uptime.c_str());
    s.printf("Heap \t%d\n", heapSize);
    s.printf("Heap Free\t%d\n", heapFree);
    s.printf("Heap used %d%%\n", heapUsedPc);
    s.printf("ChipRev \t%d\n", chipRev);
    s.printf("Model \t%s\n", chipModel);
    s.printf("Sketch \t%d\n", sketchSize);
    s.printf("Sketch Free \t%d\n", sketchFree);
    s.printf("Flash used %d%%\n", flashUsedPc);
    s.printf("Efuse \t0x%llx\n", efuseMAC);
    for (int c = 0; c < 2; c++) {
        s.printf("CPU %d load %d%%\n", c, getCpuAvg(c));
    }
    s.println("=========== SETTINGS ==========");
    GwPrint(s);
    s.println("=========== END ==========");
}

void getGps(Stream &s) {
    std::map<String, String>::iterator it = mapGps.begin();
    s.println("=========== GPS ==========");

    while (it != mapGps.end()) {
        s.printf("%s %s\n", it->first.c_str(), it->second.c_str());
        it++;
    }
    s.println("=========== END ==========");
}

void getSatellites(Stream &s) {
    time_t now = time(NULL);

    std::map<int, tGSV>::iterator it = mapSatellites.begin();
    // the map may have changed so go through it again
    it = mapSatellites.begin();
    s.println("=========== GPS Satellites==========");
    s.printf("Satellites %s\n", mapGps["GSV sats"].c_str());
    s.printf("SVID\tAZ\tELEV\tSNR\n");
    while (it != mapSatellites.end()) {
        tGSV sat = it->second;
        if (sat.Azimuth != NMEA0183DoubleNA && sat.Elevation != NMEA0183DoubleNA && sat.SNR != NMEA0183DoubleNA) {
            s.printf("%d\t%g\t%g\t%g\n", sat.SVID, sat.Azimuth, sat.Elevation, sat.SNR);
        }
        it++;
    }

    s.println("================ END ===============");
}

void getSensors(Stream &s) {
    std::map<String, String>::iterator it = mapSensors.begin();

    s.println("=========== SENSORS ==========");

    while (it != mapSensors.end()) {
        s.printf("%s %s\n", it->first.c_str(), it->second.c_str());
        it++;
    }
    s.println("=========== END ==========");
}

void getN2kMsgs(Stream &s) {
    std::map<int, int>::iterator it = mapN2kMsg.begin();

    s.println("======== N2K CAN Messages ====");

    while (it != mapN2kMsg.end()) {
        s.printf("%d %d\n", it->first, it->second);
        it++;
    }
    s.println("=========== END ==========");
}

void getYdMsgs(Stream &s) {
    std::map<int, int>::iterator it = mapYdMsg.begin();

    s.println("======== YD Messages ====");

    while (it != mapYdMsg.end()) {
        s.printf("%d %d\n", it->first, it->second);
        it++;
    }
    s.println("=========== END ==========");
}