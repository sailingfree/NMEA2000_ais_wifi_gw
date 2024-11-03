#include <Arduino.h>
#include <oled_func.h>
#include <activity.h>
#include <GwDefs.h>

static uint32_t mTimeToSec = 0;
static uint32_t mTimeSeconds = 0;
static int nexti = 0;
static const char spinner[] = {'|', '/', '-', '\\'};

void showActivity() {
    // every few hundred msecs

    if (millis() >= mTimeToSec) {
        // Time

        mTimeToSec = millis() + 250;

        mTimeSeconds++;

        // Blink the led
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // Update the oled with a spinner
        oledPrintf(0, 0, "Up %c %s", spinner[nexti], hostName.c_str());
        nexti++;
        if (nexti >= 4) {
            nexti = 0;
        }

        // Show the number of received YD and CAN messages
        oledPrintf(0, OLED_LINE_3, "CAN %d YD %d", canMsgCount, ydMsgCount);
    }
}