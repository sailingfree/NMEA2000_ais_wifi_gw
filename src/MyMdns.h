#pragma once

#include <Arduino.h>
#include "ESPmDNS.h"

void initializeMdns(String & host_name);
void addMdnsService(const char * service, const char * proto, uint16_t port);

