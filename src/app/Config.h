// The settings: the theme, how the sender exports, and when the unit
// switches itself off.
//
// On the DST-210 they are kept in RAM only (Config.cpp): its flash is
// read-protected and this firmware writes nothing to it. On the DSO-152 they
// lived in a flash page over a power cycle.

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

namespace Config {

extern uint8_t format;          // Export::RAW or DUMP
extern uint8_t block;           // which of the block sizes

extern uint8_t autoOff;         // which of the auto-off times

// What the tester opens on: AprecID, the menu, or AprecID on made-up lines.
// Kept in flash with the rest (Config.cpp); the menu when nothing is kept, or
// when the start found something to say (image CRC, the clock).
enum { START_INSP, START_MENU, START_DEMO, NSTART };
extern uint8_t startup;
const char* startupName(uint8_t i);

// The inspector reads made-up lines (DemoSig.cpp) instead of the input: on
// with Startup "Demo" (and always in the demo build), so that it is said on
// the settings page, and the bar says DEMO.
extern bool demo;

// The inspector starts again by itself when the line under the probe
// changes, and beeps at each result: the probe taken from pad to pad, no
// hand off the board for RUN. On at start.
extern bool handsOff;

// The calibration output (CalOut.cpp): the 1 kHz square wave, a test line,
// or nothing. The square wave is first: a record kept by an older version
// has 0 here.
enum { CAL_1K, CAL_TTL, CAL_OFF, NCAL };
extern uint8_t calOut;
const char* calOutName(uint8_t i);

const uint8_t NBLOCK = 4;
const uint8_t NAUTOOFF = 3;
uint32_t autoOffMs();           // 0: never
const char* autoOffName(uint8_t i);
uint16_t blockSize();           // 0: everything in one
uint16_t blockSizeAt(uint8_t i);

void load();                    // at start; defaults if nothing valid is there
bool save();                    // only writes if something differs; false: it did not take

}
#endif
