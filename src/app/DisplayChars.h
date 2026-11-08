#ifndef DISPLAY_CHARS_H
#define DISPLAY_CHARS_H

#include "Lcd.h"
#include "Theme.h"

// The text view: 52 characters a line, 19 lines on the screen, 11 px each.
#define CHARS_PER_LINE   52
#define CHARS_VIS_LINES  19
#define CHARS_LINE_H     11

// buf[0..len) is what has been received; `line` counts from its start.
void displayCharsLine(Lcd& tft, const char* buf, int len, int line, int y);
void displayChars(Lcd& tft, const char* buf, int len, int firstLine);

#endif
