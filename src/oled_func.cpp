// OLED helper functions

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

#include <oled_func.h>

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h

int lineh;
static bool has_oled = false;
void oled_init(void) {
    // Initialising the UI will init the display too.
    display.init();
    if (!display.connect()) {
        Serial.println("Failed to init the oled");
        has_oled = false;
        return;
    } else {
        has_oled = true;
        Serial.println("oled connect OK");
    }

    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    lineh = 13;  // from the font data. thats protected so cant query directly from OLEDDisplay.h

    display.clear();
    display.display();
}

void oled_write(int x, int y, const char* str) {
    if (!has_oled) {
        return;
    }
    if (x < 0 || y < 0 || !str) {
        return;
    }

    display.setColor(BLACK);
    display.fillRect(x, y, 128 - x, lineh);

    display.setColor(WHITE);
    display.drawString(x, y, String(str));
    display.display();
}

void oled_printf(int x, int y, const char* fmt, ...) {
    char buffer[128];
    if (!has_oled) {
        return;
    }

    va_list myargs;
    va_start(myargs, fmt);
    vsprintf(buffer, fmt, myargs);
    va_end(myargs);
    oled_write(x, y, buffer);
}