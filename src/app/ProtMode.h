// The protocol view, laid out the way a bench scope lays it out: the
// waveform, a bus row directly under it carrying each decoded character at
// the point in time it came from, the running stream, and what the levels
// actually measure.

#ifndef PROT_MODE_H
#define PROT_MODE_H

#include "Lcd.h"
#include <stdint.h>

#define PROT_WAVE_Y    22
#define PROT_WAVE_H    98

#define PROT_BUS_Y     124
#define PROT_BUS_H     22

#define PROT_STREAM_Y  150
#define PROT_STREAM_H  48

#define PROT_LEVEL_Y   202

namespace Prot {

void drawChrome (Lcd& tft);

// Both are drawn from the same block, in the same pass, so the characters
// line up with the bits they were read from.
void drawWave   (Lcd& tft, const uint16_t* s, uint16_t n);
void drawBus    (Lcd& tft, uint16_t n);

void drawStream (Lcd& tft, const char* buf, int len);
void drawLevels (Lcd& tft, uint32_t sampleHz, const char* buf, int len);

}
#endif
