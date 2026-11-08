// What the start screen says this firmware is, so that one build is not
// taken for another once several are about. Raise FW_VERSION with each build
// that goes to an instrument; the date is the compiler's.

#ifndef VERSION_H
#define VERSION_H

#define FW_NAME     "APRE-510"
#ifdef NO_USB_SENDER
#define FW_VERSION "0.11"       // flash_apre: no USB Sender, AprecID first in the menu
#else
#define FW_VERSION "0.05"       // flash_t232, ram_t232: as dso510-link v0.05
#endif
#define FW_DATE     __DATE__

#endif
