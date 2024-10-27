#include <Arduino.h>
#include <MyMdns.h>
#include "ESPmDNS.h"
#include <GwDefs.h>

// Register host name in mDNS
void initializeMdns(String & host_name) {
    if (MDNS.begin(host_name.c_str())) {
        Console->print("* MDNS responder started. Hostname -> ");
        Console->println(host_name);
    }
}

void addMdnsService(const char * service, const char * proto, uint16_t port) {
    Console->printf("Adding service %s to MDNS\n", service);
    MDNS.addService(service, proto, port);
}

