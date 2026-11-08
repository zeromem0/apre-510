#ifndef DISPLAY_LARGE_H
#define DISPLAY_LARGE_H

// The large view: two rows of 16 characters with their hex under them, and
// the last 5 characters received, larger still, in boxes along the bottom.

#include "Lcd.h"
#include "Theme.h"

// buf[0..len) is what has been received; the rows show it from `start`.
void displayLarge(Lcd& tft, const char* buf, int len, int start);
// Whether the 32 characters from `start` differ from those drawn last.
bool largeBufferChanged(const char* buf, int len, int start);
// The last 5 of buf[0..len) in boxes, the box tops at y - 6, 40 high; also
// AprecID's character view (Inspector.cpp).
void displayLargeSuffix(Lcd& tft, const char* buf, int len, int y);

#endif
