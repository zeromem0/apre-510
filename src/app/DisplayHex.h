#ifndef DISPLAY_HEX_H
#define DISPLAY_HEX_H

#include "Lcd.h"
#include "Theme.h"

// The hex view: 15 bytes a line, as text and as hex, 18 lines of 12 px.
#define HEX_PER_LINE   15
#define HEX_VIS_LINES  18
#define HEX_LINE_H     12

// buf[0..len) is what has been received; `line` counts from its start.
void displayHexLine(Lcd& tft, const char* buf, int len, int line, int y);
void displayHex(Lcd& tft, const char* buf, int len, int firstLine);

#endif
