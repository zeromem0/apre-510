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

// Where the line rested before buf[at], and for how long.
struct CharsGap { int at; uint32_t ms; };

// The tester's text view, since v0.18: a line for each message (after LF, or
// where the line rested), wrapped at 52, the last that fit. CR and LF in a
// dim colour, the rest before a message (+30.1s) at the right. `full`:
// everything again. (v0.18 to v0.20 had a rule of positions above; taken
// off in v0.21: a line of messages is worth more.)
// y0 and rows: where the first line goes and how many (AprecID puts a few in
// the place of its picture).
void displayCharsLines(Lcd& tft, const char* buf, int len, const CharsGap* gaps, uint8_t nGaps, bool full,
                       int16_t y0 = 20, uint8_t rows = CHARS_VIS_LINES);

#endif
