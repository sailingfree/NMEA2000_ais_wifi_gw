# n2k-gateway
An ESP32 based N2K gateway for various sensors and data sources

I wanted a gateway to interface some of my older NMEA0183 equipment to my new B&G NMEA2000 kit. specificaly an AIS receiver from NASA and my trusty Horizon CP300i chart plotter. 

I also wanted to add some feeds for i-gadgets, temperature sensors, the engine RPM and future gadgets and dongles.

This was developed for use on my Hanse301 yacht to improve navigation, safety at sea and as a general interest project.

Ths relies heavily on the excellent NMEA2000 and NMEA0183 libraries from Timo Lappalainen who maintains the N2K library, the ESP32 drivers and the associated NMEA0183 libraries. 

https://github.com/ttlappalainen

It is also very heavily based on the projects from AK-Homberger https://github.com/AK-Homberger

To this I've added some extensions and additions:

- A simple command shell that can be accessed using telnet (yes, yes, I know...) to issue commands, get status of the subsystems, set/get the settings from the preferences (see below) and do some basic debugging. 

- Most of the settings and options are stored in the preferences object so they can be changed in real-time using the shell. All the WiFi details are in the settings so there are no credentials in the code. This does mean that setup requies a specific step to connect to the AP first time round to setup the WiFi details, but thats a one-off.

- A memory based Stream object that can be used as the console or for debugging output to the shell.

- Basic system information, CPU usage, memory, versions, hardware, again that can be accessed from the shell.

I've allso included temperature, pressure, GPS, compass and boat vitals from the engine, though some of these are duplicates/standby of other sensors on the boat. 

I created a general purpose PCB for the ESP32, CAN bus driver and 12V to 5V DC regulator. The PCB has a general prototype area and exposes some of the GPIOs to headers. 

All the code was developed using Visual Studio Code and PlatformIO for the ESP32. I know the code isn't that tidy, but I was interested in getting a working baseline to actually use on my boat for starters.
