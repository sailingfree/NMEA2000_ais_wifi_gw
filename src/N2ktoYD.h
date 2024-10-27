#pragma once

#include <Arduino.h>
#include <N2kMsg.h>


// Send to Yacht device clients over udp using the cast address
void GwSendYD(const tN2kMsg &N2kMsg);

// Handle any YD messages received 
// Read the YD data, decode the N2K messages
void handleIncomingYD(void);
