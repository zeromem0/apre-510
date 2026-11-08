#include <Arduino.h>
#include "Config.h"
#include "Theme.h"
#include "Export.h"
#include "Acquire.h"            // T232_RAM

// The settings are kept in the last 2 kB page of the flash, 0x080FF800, past
// the tester (which ends below 0x080A0000) and untouched by the FNIRSI
// bootloader, which erases only the pages it writes. Read on the unit
// (tools/flash_peek.py, 2026-11-08): everything from the end of the tester to
// 0x08100000 was 0xFF, nothing of FNIRSI's there. The page is in the second
// bank, written with its own registers (FLASH_UNLOCK2 and on).
//
// One record of four words: "C510", the values, the values again inverted,
// a sum. Anything that does not check is taken as nothing kept: the defaults.
// Written only when something differs, and read back.
//
// From RAM (RAM_RUN) nothing is read or written: with the debugger attached
// FAP keeps the flash shut, and the settings start from the defaults.

namespace Config {

static const uint16_t BLOCK[NBLOCK] = { 0, 64, 256, 1024 };

// Off, 10 and 30 minutes without a key or a byte received. 10 by default: a
// unit switched on by accident in a bag should not run for hours.
static const uint8_t     AUTOOFF_MIN[NAUTOOFF]  = { 0, 10, 30 };
static const char* const AUTOOFF_NAME[NAUTOOFF] = { "off", "10 min", "30 min" };
static const char* const START_NAME[NSTART]     = { "AprecID", "Menu", "Demo" };

uint8_t format  = Export::DUMP;
uint8_t block   = 0;
uint8_t autoOff = 1;
uint8_t startup = START_MENU;
#ifdef DEMO_SIGNAL
bool    demo    = true;
#else
bool    demo    = false;
#endif
bool    handsOff = true;

uint16_t blockSizeAt(uint8_t i) { return i < NBLOCK ? BLOCK[i] : 0; }
uint16_t blockSize() { return blockSizeAt(block); }

uint32_t autoOffMs() { return autoOff < NAUTOOFF ? AUTOOFF_MIN[autoOff] * 60000ul : 0; }
const char* autoOffName(uint8_t i) { return i < NAUTOOFF ? AUTOOFF_NAME[i] : "?"; }
const char* startupName(uint8_t i) { return i < NSTART ? START_NAME[i] : "?"; }

static const uint32_t PAGE  = 0x080FF800u;
static const uint32_t MAGIC = 0x30313543u;          // "C510"

// theme, format, block, auto-off, hands-off, startup, a byte each
static void pack(uint32_t rec[4]) {
    uint32_t a = (uint32_t)Theme::idx | (uint32_t)format << 8 | (uint32_t)block << 16 | (uint32_t)autoOff << 24;
    uint32_t b = (uint32_t)(handsOff ? 1 : 0) | (uint32_t)startup << 8;
    rec[0] = MAGIC;
    rec[1] = a;
    rec[2] = b;
    rec[3] = ~(MAGIC + a + b * 3u);
}

#ifndef RAM_RUN
// The second bank's registers (AT32F403A: FLASH_UNLOCK2, STS2, CTRL2, ADDR2).
#define F_UNLOCK2 (*(volatile uint32_t*)0x40022044u)
#define F_STS2    (*(volatile uint32_t*)0x4002204Cu)
#define F_CTRL2   (*(volatile uint32_t*)0x40022050u)
#define F_ADDR2   (*(volatile uint32_t*)0x40022054u)
#define STS_OBF   0x01u                 // busy
#define STS_ERR   0x14u                 // PRGMERR, EPPERR
#define STS_ODF   0x20u                 // done
#define CTRL_FPRGM  0x01u
#define CTRL_SECERS 0x02u
#define CTRL_ERSTR  0x40u
#define CTRL_OPLK   0x80u

// From RAM: the page being written is in the bank the tester runs from.
T232_RAM __attribute__((noinline)) static bool waitDone() {
    uint32_t n = 0;
    while (F_STS2 & STS_OBF) {
        IWDG->KR = 0xAAAA;
        if (++n > 50000000u) return false;
    }
    uint32_t s = F_STS2;
    F_STS2 = STS_ERR | STS_ODF;         // cleared by writing 1
    return !(s & STS_ERR);
}

T232_RAM __attribute__((noinline)) static bool writePage(const uint32_t rec[4]) {
    bool ok = true;
    if (F_CTRL2 & CTRL_OPLK) { F_UNLOCK2 = 0x45670123u; F_UNLOCK2 = 0xCDEF89ABu; }
    if (F_CTRL2 & CTRL_OPLK) return false;
    F_STS2 = STS_ERR | STS_ODF;
    F_CTRL2 = CTRL_SECERS;
    F_ADDR2 = PAGE;
    F_CTRL2 = CTRL_SECERS | CTRL_ERSTR;
    ok = waitDone();
    F_CTRL2 = 0;
    for (uint8_t i = 0; ok && i < 4; i++) {
        F_CTRL2 = CTRL_FPRGM;
        ((volatile uint32_t*)PAGE)[i] = rec[i];
        ok = waitDone();
        F_CTRL2 = 0;
    }
    F_CTRL2 = CTRL_OPLK;
    return ok;
}
#endif

void load() {
#ifndef RAM_RUN
    const volatile uint32_t* p = (const volatile uint32_t*)PAGE;
    uint32_t a = p[1], b = p[2];
    if (p[0] != MAGIC || p[3] != ~(MAGIC + a + b * 3u)) return;         // nothing kept
    uint8_t th = a & 0xFF, fm = (a >> 8) & 0xFF, bl = (a >> 16) & 0xFF, ao = a >> 24;
    uint8_t ho = b & 0xFF, st = (b >> 8) & 0xFF;
    if (th < Theme::count())  Theme::idx = th;
    if (fm < Export::NFMT)    format = fm;
    if (bl < NBLOCK)          block = bl;
    if (ao < NAUTOOFF)        autoOff = ao;
    handsOff = ho != 0;
    if (st < NSTART)          startup = st;
#endif
#ifndef DEMO_SIGNAL
    demo = startup == START_DEMO;
#endif
}

bool save() {
#ifdef RAM_RUN
    return true;
#else
    uint32_t rec[4];
    pack(rec);
    const volatile uint32_t* p = (const volatile uint32_t*)PAGE;
    if (p[0] == rec[0] && p[1] == rec[1] && p[2] == rec[2] && p[3] == rec[3]) return true;
    __disable_irq();
    bool ok = writePage(rec);
    __enable_irq();
    for (uint8_t i = 0; i < 4; i++) if (p[i] != rec[i]) ok = false;
    return ok;
#endif
}

}
