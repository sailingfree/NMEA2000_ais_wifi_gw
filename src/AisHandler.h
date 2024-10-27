// AIS handler

#pragma once

#ifndef defined_NMEA2000
#define defined_NMEA2000 1  // Dont duplicate the definition
#endif

#include <N2kMsg.h>
#include <N2kTypes.h>
#include <NMEA2000.h>

#include <N2kMessages.h>

#include <NMEA0183.h>
#include <NMEA0183Messages.h>
#include <NMEA0183Msg.h>

//#include "NMEA0183AIStoNMEA2000.h"  // Contains class, global variables and code !!!

#define MAX_NMEA0183_MESSAGE_SIZE 150  // For AIS
#define ENABLE_DEBUG_LOG 1

void handleAis();