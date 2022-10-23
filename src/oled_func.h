// oled helper Header

#include "SSD1306.h"

extern int lineh;

void oled_init(void);
void oled_write(int x, int y, const char * str);
void oled_printf(int x, int y, const char * fmt, ...);