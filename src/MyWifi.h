#pragma once

#include <Arduino.h>
#include <WiFiClient.h>

#define WLAN_CLIENT 1  // Set to 1 to enable client network. 0 to act as AP only

void initWifi(String & host_name);
bool connectWifi();
void disconnectWifi();
void checkWifi();