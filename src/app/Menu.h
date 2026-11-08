// The start screen: a DE-9 plug, what there is to run, and which build this is.
// Up and down move the choice, OK or RUN starts it; AUTO held shows About.

#ifndef MENU_H
#define MENU_H

#include "Lcd.h"
#include <stdint.h>

namespace Menu {

#ifdef NO_USB_SENDER
// Without the USB Sender: AprecID first, where the bar starts.
const int8_t ITEM_INSP   = 0;
const int8_t ITEM_TESTER = 1;
const int8_t ITEM_SET    = 2;
#else
const int8_t ITEM_TESTER = 0;
const int8_t ITEM_INSP   = 1;
const int8_t ITEM_TTL    = 2;
const int8_t ITEM_SET    = 3;
#endif

// The check of the tester's image in flash (main.cpp): 1 whole, -1 not, 0 not
// checked (from RAM, or a file made without the length and CRC).
extern int8_t image;

void   draw(Lcd& tft);
int8_t poll(Lcd& tft);    // the item started, or -1
void   about(Lcd& tft);   // what AprecID knows; at power-up, and until a key from the menu

}
#endif
