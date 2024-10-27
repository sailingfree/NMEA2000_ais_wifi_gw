#include <Arduino.h>
#include <WebServer.h>
#include <GwWebServer.h>
#include <html_footer.h>
#include <html_header.h>
#include <BoatData.h>
#include <StringStream.h>
#include <MyN2k.h>
#include <SysInfo.h>
#include <GwDefs.h>

// The web server on port 80
WebServer webserver(80);

// HTML handlers
String html_start = HTML_start;  // Read HTML contents
String html_end = HTML_end;

//*****************************************************************************

void handleNotFound() {
    webserver.send(404, "text/plain", "File Not Found\n\n");
}

void handleRoot() {
    webserver.send(200, "text/html", html_start + html_end);  // Send web page
}

void handleData() {
    String adcValue(random(100));
    webserver.send(200, "application/json", adcValue);  // Send ADC value only to client ajax request
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
    webserver.send(200, "text/html", html_start + boatData.data.c_str() + html_end);  // Send web page
}

void startWebserver() {
    webserver.on("/", handleRoot);
    webserver.on("/data", handleData);
    webserver.on("/boat", handleBoat);

    webserver.onNotFound(handleNotFound);

    webserver.begin();
    Console->println("HTTP server started");

}

void handleWebserver() {
     webserver.handleClient();
}