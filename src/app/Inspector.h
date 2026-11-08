// The inspector: what is on the probe, worked out from the signal itself,
// the other way round from a waveform generator. A probe on a pin, and the
// screen says GND, 3.3 V, RS232 at 9600 8N1, and so on.
//
// Three steps, each on what the one before found:
//   1. the range: a look at range 9, then the most sensitive range the
//      signal fits in (5 at the least: 500 mV a division);
//   2. levels and edges at 2 MS/s: lowest, highest, mean, how long the line
//      sits high, how many edges, the shortest pulse;
//   3. the detectors, in order (Inspector.cpp, DETECTORS): a steady level is
//      named after the rail or state it matches; a line with traffic is
//      matched on its levels (RS232 bipolar, TTL 0..3.3 or 0..5 V), and a
//      serial one is then read with the tester's own decoder, which learns
//      the speed and the frame, and scored by the share of frames read
//      without a framing or parity error.
//
// Only what can be told from one channel is claimed. Levels in volts rest
// on the ranges' gains, which have been measured on range 7 only.

#ifndef INSPECTOR_H
#define INSPECTOR_H

#include "Lcd.h"
#include <stdint.h>

namespace Insp {

// Hands over the next block of samples, or nullptr if there is none yet;
// `over` says a block went by unread before it.
typedef const uint16_t* (*Source)(uint16_t& n, bool& over);

// What the inspector found, for the RS232 tester when OK hands over.
struct Found {
    bool     uart;              // a serial line was read
    bool     inverted;          // rests low: RS232, or TTL upside down
    uint32_t baud;              // 0 if not known
    char     parity;            // 'N', 'E', 'O'
    uint8_t  dataBits, stopBits;
    uint32_t rateHz;            // the sample rate it was read at
};

// TESTER_TEACH: as TESTER, on the tester's view of one character;
// TESTER_HEALTH: on its bus health view (levels, bit rate, rise, fall, jitter).
enum { STAY, MENU, TESTER, TESTER_TEACH, TESTER_HEALTH };

void  enter(Lcd& tft, Source src);
// The samples, from the DMA interrupt while one is set (main.cpp), else from
// poll() through the source.
void  block(const uint16_t* s, uint16_t n, bool overrun);
uint8_t poll(Lcd& tft);   // STAY, MENU, or TESTER (open what was found in it)
void  again(Lcd& tft);    // look again, as RUN does (the page's "again")
const Found& found();
// How many results so far: each one the unit being used, for Auto off
// (hands-off, the probe moved from pad to pad, no key pressed).
uint32_t results();

}
#endif
