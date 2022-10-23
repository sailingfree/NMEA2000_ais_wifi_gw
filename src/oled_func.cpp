// OLED helper functions

#include <oled_func.h>

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h

int lineh;
static bool has_oled = false;
void oled_init(void) {

// Initialising the UI will init the display too.
  display.init();
  if(!display.connect()) {
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

void oled_write(int x, int y, const char * str) {
    if(!has_oled) {
        return;
    }
    if(x < 0 || y < 0 || !str) {
        return;
    }

    display.setColor(BLACK);
    display.fillRect(x, y, 128 - x, lineh);
    
    display.setColor(WHITE);
    display.drawString(x, y, String(str));
    display.display();
}

void oled_printf(int x, int y, const char * fmt, ...) {
    char buffer[128];
    if(!has_oled) {
        return;
    }

    va_list myargs;
    va_start(myargs, fmt);
    vsprintf(buffer, fmt, myargs);
    va_end(myargs);
    oled_write(x, y, buffer);
}