#include <Arduino.h>
#include "Inspector.h"
#include "Acquire.h"
#include "Frontend.h"
#include "Rs232.h"
#include "Theme.h"
#include "Keys.h"
#include "DemoSig.h"
#include "Config.h"
#include "Teach.h"
#include "Beep.h"
#include "Link.h"
#include "DisplayLarge.h"
#include "DisplayChars.h"

namespace Insp {

// --- how it measures ----------------------------------------------------------
static const uint32_t MEASURE_HZ = 2000000;     // levels and edges, as the bus health view
// The tester's rates that the decoder keeps up with from the interrupt.
static const uint32_t UART_RATES[] = { 100000, 250000, 500000, 1250000 };
static const uint8_t  RANGE_MIN  = 5;           // 500 mV a division: GND and 3.3 V told apart well
static const uint8_t  RANGE_WIDE = 9;           // 10 V a division, +-50 V on the screen
static const int32_t  FIT_COUNTS = 1500;        // a signal fits a range within +-1500 of zero
static const uint32_t SETTLE_MS  = 40;          // after a range change: the relay, the offset
static const uint32_t WIDE_MS    = 60;
static const uint32_t LEVELS_MS  = 80;     // a first look; every line is then watched to WATCH_MS
static const uint32_t EDGES_MS   = 300;
// A line that talks in bursts (a master polling now and then, a scanner)
// can look quiet for a while: a quiet line is watched this long for a swing,
// and a busy one for edges, before it is judged.
static const uint32_t WATCH_MS   = 1500;
static const uint32_t EDGES_ENOUGH = 40;
static const uint32_t UART_LEARN_MS = 4000;     // to learn a speed and a frame, at most
static const uint32_t UART_MORE_MS  = 3000;     // then frames to score, at most
static const uint32_t UART_FRAMES   = 24;       // ... or this many
// Hands-off (Config::handsOff): the line watched in windows of MON_MS once
// a result is up, started again when it changes; "AUTO details" offered once
// a result has stood HINT_MS.
static const uint32_t MON_MS        = 100;
static const uint32_t OFFREST_MS    = 500;      // quiet away from the rest level so long: another line
static const uint32_t FRAMES_MS     = 1000;     // a serial result: its frames judged over so long
static const uint32_t HINT_MS       = 3000;

enum { P_SETTLE, P_WIDE, P_LEVELS, P_EDGES, P_UART, P_DONE, P_TEACH };
static volatile uint8_t g_phase = P_DONE;
static uint8_t  g_after;                        // the phase that follows P_SETTLE
static uint32_t g_t0;                           // when the phase began

static Source g_src;

// What block() gathers in the phase in force. Read and reset by poll() with
// the interrupt held off.
static uint16_t g_lo, g_hi;
static uint64_t g_sum;                  // 1.5 s at 2 MS/s is past 32 bits
static uint32_t g_n, g_high, g_edges, g_minRun;
static uint16_t g_thr, g_hyst;

// The stretches between edges, in the edge phase: the length in samples
// (15 bits, held at the top) and in bit 15 the level, 1 high. A stretch runs
// on from one block into the next unless a block went by unread in between.
static const uint16_t RUNS_MAX = 512;
static uint16_t g_runs[RUNS_MAX];
static uint16_t g_nRuns;
static uint32_t g_carry;
static bool     g_carryHigh, g_carryOk;

// Hands-off: the window being gathered on a result (block()), read and
// reset by step().
static volatile bool g_mon;
static uint16_t g_mLo, g_mHi;
static uint32_t g_mSum, g_mN;
static uint32_t g_monMs, g_offRestMs;
static uint32_t g_frMs, g_frGood, g_frBad;      // a serial result: the decoder's counts when the window began
static int32_t  g_need;                         // the swing that counts as activity, in counts
static bool     g_full;                         // watch in full this time: activity came after a quiet start
static bool     g_hint;                         // "AUTO details" shown

static void accReset() {
    g_lo = 0xFFFF; g_hi = 0;
    g_sum = g_n = g_high = g_edges = 0;
    g_minRun = 0xFFFFFFFFu;
    g_carryOk = false;
}

// --- what was found -------------------------------------------------------------
// The features of the line, as the detectors see them.
struct Feat {
    uint8_t  range;
    int32_t  loMv, hiMv, meanMv;
    bool     clipped;           // touched an end of the converter
    bool     active;            // a swing worth the name, with edges in it
    uint32_t edges;
    uint32_t edgesPerS;
    uint32_t minRunNs;          // the shortest stretch between two edges, 0 if none
    uint8_t  highPct;           // share of the time above the middle
    bool     restHigh;          // spends most of its time high
    // from the stretches between edges (timing())
    bool     periodic;          // every high and every low alike: a PWM
    uint32_t freqHz;            // ... its frequency
    uint16_t dutyPm;            // ... and high time, per mille
    bool     square;            // the few whole highs and lows that were caught are alike (freqHz, dutyPm set too)
    bool     clock;             // regular low pulses in bursts with long highs between: a clock like I2C SCL
    uint32_t clockHz;
    uint16_t perBurst;          // clocks in a burst
    uint32_t maxOtherNs;        // the longest stretch away from the rest level
    uint32_t oneBitNs;          // the shortest stretches, high and low, averaged: one bit
};

// What a detector makes of them.
struct Verdict {
    char        name[20];       // what it is
    char        info[44];       // a line about it: frequency, duty, bit rate
    const char* note;           // a word more, or ""
    bool        showLevel;      // a steady level: its value is shown big
    bool        uart;           // a serial line: read it
    bool        inverted;       // ... resting low
    bool        retry;          // ... and if nothing reads, once more the other way up
    bool        zeroCross;      // ... and, inverted, must swing through zero (RS232)
    bool        lin;            // ... and it is LIN: breaks, frames, checksums
};

static Feat    g_f;
static Verdict g_v;
static Found   g_found;
static uint32_t g_b0, g_tDecided;               // frames when the frame was settled, and when
static bool     g_decided, g_retried;
static uint8_t  g_relearns;                     // times the frame was learnt again after a bad score
static bool     g_first9600;                    // 9600 not tried yet on this line
static bool     g_at9600;                       // ... and this reading is that try
// A line at RS232 mark, with nothing on it, may still talk now and then (an
// instrument that sends a reading every half minute). It is read as a
// serial line straight away and waited on: each try at it starts counting
// from the first frame, not from when it began.
static const uint32_t IDLE_WAIT_MS = 60000;
static bool     g_idleWait;                     // this reading came from a quiet RS232 line
static bool     g_traffic;                      // ... and this try has seen something on it
static uint32_t g_waitT0;                       // ... since when it has waited
static uint8_t  g_score;                        // 0..100, frames read cleanly
static uint32_t g_scored;                       // how many frames that is of

static uint8_t  g_teach;                        // what is being explained (T_*), 0 on the result
static bool     g_mbOk;                         // a Modbus RTU frame with its CRC-16 holding was read
T232_RAM static void capTake(const uint16_t* s, uint16_t n);

static char     g_rx[53];                       // the last characters read, as text
static uint8_t  g_rxN;
static uint32_t g_drawMs;
// The last characters as they came, CR and LF kept, for the boxes that stand
// in place of the picture once RS232 or TTL is read (as on the DSO-152); MODE
// switches between them and the picture, and the choice is kept. The menu is
// OK held.
static char     g_txt[8];
static uint8_t  g_txtN;
static bool     g_txtNew;
// What MODE shows in the place of the picture on a serial line: the last
// characters in boxes, the picture, or the messages a line each (v0.19: a
// line of text read whole, where each of its fields sits).
enum { VIEW_BOXES, VIEW_WAVE, VIEW_LINES, NVIEW };
static uint8_t  g_view = VIEW_BOXES;

// The text for the lines view, kept across the readings of hands-off (each
// message of a line that talks now and then starts one); a try that did
// not hold is taken off again (g_linesKeep). Cleared on RUN and on entering.
static char     g_lines[512];
static int      g_linesN, g_linesKeep;
static CharsGap g_lgaps[10];
static uint8_t  g_lgapN;
static uint32_t g_lastRxMs;
static bool     g_linesNew;

static void linesCut(int n) {                   // back to n characters
    if (n > g_linesN) return;
    g_linesN = n;
    while (g_lgapN && g_lgaps[g_lgapN - 1].at >= n) g_lgapN--;
    g_linesNew = true;
}
static void linesClear() { g_linesN = g_linesKeep = 0; g_lgapN = 0; g_lastRxMs = 0; g_linesNew = true; }
static void linesAdd(char c) {
    if (g_linesN == (int)sizeof(g_lines)) {      // full: the older half goes
        int half = (int)sizeof(g_lines) / 2;
        memmove(g_lines, g_lines + half, (size_t)half);
        g_linesN -= half;
        g_linesKeep = g_linesKeep > half ? g_linesKeep - half : 0;
        uint8_t k = 0;
        for (uint8_t i = 0; i < g_lgapN; i++)
            if (g_lgaps[i].at - half > 0) { g_lgaps[k] = g_lgaps[i]; g_lgaps[k].at -= half; k++; }
        g_lgapN = k;
    }
    uint32_t now = millis();
    if (g_linesN && g_lastRxMs && now - g_lastRxMs >= 300) {
        if (g_lgapN == sizeof(g_lgaps) / sizeof(g_lgaps[0])) { memmove(g_lgaps, g_lgaps + 1, sizeof(g_lgaps) - sizeof(g_lgaps[0])); g_lgapN--; }
        g_lgaps[g_lgapN].at = g_linesN;
        g_lgaps[g_lgapN].ms = now - g_lastRxMs;
        g_lgapN++;
    }
    g_lastRxMs = now;
    g_lines[g_linesN++] = c;
    g_linesNew = true;
}

const Found& found() { return g_found; }

// --- the detectors ----------------------------------------------------------------
// Tried in order; the first that recognises the line names it. One wire of
// a pair is seen against GND: an RS485 or CAN wire on its own, not the
// difference between the two.
typedef bool (*Detector)(const Feat& f, Verdict& v);

static void printFreq(char* out, uint8_t size, const char* what, uint32_t hz) {
    if (hz >= 1000000)   snprintf(out, size, "%s %lu.%lu MHz", what, (unsigned long)(hz / 1000000), (unsigned long)(hz / 100000 % 10));
    else if (hz >= 1000) snprintf(out, size, "%s %lu.%lu kHz", what, (unsigned long)(hz / 1000), (unsigned long)(hz / 100 % 10));
    else                 snprintf(out, size, "%s %lu Hz", what, (unsigned long)hz);
}

static void printMv(char* out, uint8_t size, int32_t mv) {
    int32_t a = mv < 0 ? -mv : mv;
    snprintf(out, size, "%s%ld.%02ld", mv < 0 ? "-" : "", (long)(a / 1000), (long)(a % 1000 / 10));
}

// The nearest of `std` within 15 %, or 0.
static uint32_t snapTo(uint32_t v, const uint32_t* std, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        uint32_t d = v > std[i] ? v - std[i] : std[i] - v;
        if ((uint64_t)d * 100 <= (uint64_t)std[i] * 15) return std[i];
    }
    return 0;
}

struct Level { const char* name; const char* note; int16_t loMv, hiMv; };
// Steady levels, in mV; the first range that holds the mean names it.
static const Level LEVELS[] = {
    { "GND",          "0V",                -250,   250 },
    { "3.3V",         "rail, or TTL idle",  3000,  3600 },
    { "5V",           "rail, or TTL idle",  4500,  5500 },
    { "12V",          "rail",              10800, 13200 },
    { "24V",          "rail",              21600, 26400 },
    { "RS232 idle",   "mark, no traffic", -15000, -3000 },
};

static bool steadyLevel(const Feat& f, Verdict& v) {
    if (f.active) return false;
    v.showLevel = true;
    for (uint8_t i = 0; i < sizeof(LEVELS) / sizeof(LEVELS[0]); i++) {
        if (f.meanMv >= LEVELS[i].loMv && f.meanMv <= LEVELS[i].hiMv) {
            strncpy(v.name, LEVELS[i].name, sizeof(v.name) - 1);
            v.note = LEVELS[i].note;
            return true;
        }
    }
    strcpy(v.name, "DC");
    v.note = "steady";
    return true;
}

// Every high alike and every low alike, at whatever levels.
static void namePwm(const Feat& f, Verdict& v) {
    printFreq(v.name, sizeof(v.name), "PWM", f.freqHz);
    char lo[10], hi[10];
    printMv(lo, sizeof(lo), f.loMv);
    printMv(hi, sizeof(hi), f.hiMv);
    snprintf(v.info, sizeof(v.info), "duty %u.%u%%  %s/%sV", f.dutyPm / 10, f.dutyPm % 10, lo, hi);
}

static bool pwm(const Feat& f, Verdict& v) {
    if (!f.active || !f.periodic) return false;
    namePwm(f, v);
    return true;
}

// Swings through zero, by more than 3 V each way.
static bool rs232(const Feat& f, Verdict& v) {
    if (!f.active || f.loMv > -3000 || f.hiMv < 3000) return false;
    strcpy(v.name, "RS232");
    v.uart = true;
    // Rests at mark, the negative level. Taken as the standard has it, not
    // from where the line spends its time: back-to-back traffic sits at
    // space more than half the time (seen at 9600, 2026-10-28), and read
    // upside down its frames still passed. The tester turns it with OK.
    v.inverted = true;
    v.note = "";
    return true;
}

// From about 0 V up to 3.3 V or 5 V: a clock in bursts (I2C SCL), else a
// serial line resting high, read as such; failing that, said so
// (finishUart()).
static bool ttl(const Feat& f, Verdict& v) {
    if (!f.active || f.loMv < -800 || f.loMv > 800) return false;
    bool v33 = f.hiMv >= 2600 && f.hiMv < 3600;
    if (!v33 && !(f.hiMv >= 3600 && f.hiMv <= 5600)) return false;
    if (f.clock) {
        static const uint32_t I2C[] = { 100000, 400000, 1000000 };
        uint32_t std = snapTo(f.clockHz, I2C, 3);
        strcpy(v.name, "I2C SCL");
        if (std) snprintf(v.info, sizeof(v.info), "%lu kHz, %u clocks a burst", (unsigned long)(std / 1000), f.perBurst);
        else     snprintf(v.info, sizeof(v.info), "~%lu kHz, %u clocks a burst", (unsigned long)(f.clockHz / 1000), f.perBurst);
        v.note = v33 ? "3.3V logic" : "5V logic";
        return true;
    }
    strcpy(v.name, v33 ? "TTL 3.3V" : "TTL 5V");
    v.uart = true;
    v.inverted = false;                         // TTL serial rests high (as for RS232 above)
    v.retry = true;
    v.note = "";
    return true;
}

// One CAN wire: rests (recessive) at about 2.5 V; dominant bits go up to
// about 3.5 V on CAN_H, down to about 1.5 V on CAN_L. The bit time is the
// shortest stretch; bit stuffing keeps dominant stretches to five bits.
static bool can(const Feat& f, Verdict& v) {
    if (!f.active) return false;
    int32_t rest = f.restHigh ? f.hiMv : f.loMv, other = f.restHigh ? f.loMv : f.hiMv;
    int32_t d = other > rest ? other - rest : rest - other;
    if (rest < 2000 || rest > 3000 || d < 400 || d > 1800) return false;
    strcpy(v.name, other > rest ? "CAN_H" : "CAN_L");
    static const uint32_t STD[] = { 125000, 250000, 500000, 1000000 };
    uint32_t rate = f.minRunNs ? 1000000000u / f.minRunNs : 0;
    uint32_t std = snapTo(rate, STD, 4);
    uint32_t bits10 = f.minRunNs ? (uint32_t)((uint64_t)f.maxOtherNs * 10 / f.minRunNs) : 0;
    if (std) snprintf(v.info, sizeof(v.info), "%lu kbit/s", (unsigned long)(std / 1000));
    else     snprintf(v.info, sizeof(v.info), "~%lu kbit/s", (unsigned long)(rate / 1000));
    v.note = bits10 && bits10 <= 55 ? "stuffed: 5 bits at most" : "";
    return true;
}

// One RS485 wire (Profibus DP too): within about 0.3..5.5 V, away from the
// 2.5 V of a CAN wire at rest. B (+) rests high, A (-) low; read as a serial
// line, and once more upside down if nothing reads.
static bool rs485(const Feat& f, Verdict& v) {
    if (!f.active || f.loMv < 300 || f.hiMv > 5500 || f.hiMv - f.loMv < 600) return false;
    strcpy(v.name, f.restHigh ? "RS485 B" : "RS485 A");
    v.uart = true;
    v.inverted = !f.restHigh;
    v.zeroCross = false;                        // A rests low but never goes below zero
    v.retry = true;
    v.note = f.restHigh ? "B (+) wire, rests high" : "A (-) wire, rests low";
    return true;
}

// LIN: one wire at the battery (12 V, 24 V on trucks), resting recessive
// (high), pulled dominant (low, under 40 % of the battery); each frame opened
// by a break, the line dominant for 13 bits or more, which no character can
// do (9 at most). The speed is measured from one bit, to the LIN ones within
// 2 %, and given to the decoder outright: its own learning knows nothing of
// breaks, and of 10417 and 20000 on purpose.
static const uint32_t LIN_BAUD[] = { 2400, 9600, 10417, 19200, 20000 };
static uint32_t g_linBaud;

static bool lin(const Feat& f, Verdict& v) {
    if (!f.active || !f.restHigh || f.hiMv < 7000 || f.hiMv > 32000 || f.loMv > f.hiMv * 2 / 5) return false;
    if (!f.oneBitNs || f.maxOtherNs < 11u * f.oneBitNs) return false;
    uint32_t baud = (1000000000u + f.oneBitNs / 2) / f.oneBitNs;
    g_linBaud = baud;
    for (uint8_t i = 0; i < sizeof(LIN_BAUD) / sizeof(LIN_BAUD[0]); i++) {
        uint32_t d = baud > LIN_BAUD[i] ? baud - LIN_BAUD[i] : LIN_BAUD[i] - baud;
        if (d * 50 <= LIN_BAUD[i]) g_linBaud = LIN_BAUD[i];
    }
    strcpy(v.name, "LIN");
    v.uart = true;
    v.lin = true;
    v.note = f.hiMv > 18000 ? "24V bus" : "12V bus";
    return true;
}

static const Detector DETECTORS[] = { steadyLevel, pwm, rs232, ttl, lin, can, rs485 };

// --- LIN frames, put together from the decoder's queue ----------------------------
// After a break: the sync byte 0x55, the protected identifier (6 bits and two
// of parity), then, if a slave answers, 1..8 data bytes and the checksum. A
// frame is closed by the next break. The checksum is the inverted sum with
// carry of the data (LIN 1.x, classic) or of the identifier and the data
// (2.x, enhanced); whichever holds says the version. A header with nothing
// after it is a slave that did not answer.
struct LinId {
    uint8_t  data[8], n;
    bool     seen, answered, sumOk, enhanced;
    uint16_t sentHash;          // what the page was last sent of it
    uint32_t sentMs;
};
static LinId    g_linIds[64];
static uint8_t  g_linBuf[12], g_linN;
static bool     g_linOpen;                      // a break was seen, its frame not closed yet
static uint32_t g_linHeaders, g_linGood;        // headers, and those whose sync and parity held
static uint32_t g_linClassic, g_linEnhanced;    // answers whose checksum held either way

static uint8_t linPid(uint8_t id) {
    uint8_t b = id & 0x3F;
    uint8_t p0 = (uint8_t)(((b >> 0) ^ (b >> 1) ^ (b >> 2) ^ (b >> 4)) & 1);
    uint8_t p1 = (uint8_t)(~((b >> 1) ^ (b >> 3) ^ (b >> 4) ^ (b >> 5)) & 1);
    return (uint8_t)(b | (p0 << 6) | (p1 << 7));
}

static uint8_t linSum(const uint8_t* d, uint8_t n, uint16_t start) {
    uint16_t s = start;
    for (uint8_t i = 0; i < n; i++) { s += d[i]; if (s > 255) s -= 255; }
    return (uint8_t)~s;
}

static void linReset() {
    memset(g_linIds, 0, sizeof(g_linIds));
    g_linN = 0; g_linOpen = false;
    g_linHeaders = g_linGood = g_linClassic = g_linEnhanced = 0;
}

// The identifiers seen, on the screen's one line of text: "10:1234 11:-- ".
static void linSummary() {
    char* p = g_rx;
    char* end = g_rx + sizeof(g_rx) - 1;
    for (uint8_t id = 0; id < 64; id++) {
        const LinId& e = g_linIds[id];
        if (!e.seen) continue;
        if (end - p < 6 + 2 * e.n) break;
        p += sprintf(p, "%02X:", id);
        if (!e.answered) p += sprintf(p, "--");
        else for (uint8_t i = 0; i < e.n; i++) p += sprintf(p, "%02X", e.data[i]);
        if (e.answered && !e.sumOk) *p++ = '!';
        *p++ = ' ';
    }
    *p = 0;
    g_rxN = (uint8_t)(p - g_rx);
}

// To the page, when an identifier's frame changes, or once a second.
static void linSend(uint8_t id, LinId& e) {
    uint16_t h = (uint16_t)(e.answered ? 1 + e.n : 0);
    for (uint8_t i = 0; i < e.n; i++) h = (uint16_t)(h * 31 + e.data[i]);
    if (h == e.sentHash && millis() - e.sentMs < 1000) return;
    e.sentHash = h; e.sentMs = millis();
    char buf[160], hex[17];
    for (uint8_t i = 0; i < e.n; i++) sprintf(hex + 2 * i, "%02x", e.data[i]);
    hex[2 * e.n] = 0;
    Link::Json j(buf, sizeof(buf));
    j.str("t", "frame").str("proto", "lin").num("id", id).num("pid", linPid(id)).flag("response", e.answered);
    if (e.answered) j.str("data", hex).flag("classicOk", e.sumOk && !e.enhanced).flag("enhancedOk", e.enhanced);
    Link::send(j.done());
}

static void linClose() {
    if (!g_linOpen) return;
    g_linOpen = false;
    if (g_linN < 2) return;                     // a break and not even a header after it
    g_linHeaders++;
    uint8_t pid = g_linBuf[1], id = pid & 0x3F;
    if (g_linBuf[0] != 0x55 || linPid(id) != pid) return;
    g_linGood++;
    LinId& e = g_linIds[id];
    e.seen = true;
    uint8_t k = (uint8_t)(g_linN - 2);          // bytes after the identifier
    e.answered = k >= 2;                        // a data byte and the checksum at least
    e.n = 0;
    if (e.answered) {
        uint8_t nd = (uint8_t)(k - 1 > 8 ? 8 : k - 1);
        uint8_t chk = g_linBuf[2 + nd];
        bool enh = linSum(g_linBuf + 2, nd, pid) == chk, cla = linSum(g_linBuf + 2, nd, 0) == chk;
        e.n = nd;
        memcpy(e.data, g_linBuf + 2, nd);
        e.sumOk = enh || cla;
        e.enhanced = enh;
        if (enh) g_linEnhanced++;
        else if (cla) g_linClassic++;
    }
    linSend(id, e);
    linSummary();
}

static void linByte(int c) {
    if (c == Rs232::BREAK) {
        linClose();
        g_linOpen = true;
        g_linN = 0;
        return;
    }
    if (g_linOpen && g_linN < sizeof(g_linBuf)) g_linBuf[g_linN++] = (uint8_t)c;
}

static void clearVerdict(Verdict& v) {
    v.name[0] = 0; v.info[0] = 0; v.note = "";
    v.showLevel = false; v.uart = false; v.inverted = false; v.retry = false; v.zeroCross = true; v.lin = false;
}

static void classify() {
    for (uint8_t i = 0; i < sizeof(DETECTORS) / sizeof(DETECTORS[0]); i++) {
        clearVerdict(g_v);
        if (DETECTORS[i](g_f, g_v)) return;
    }
    // Nothing knew it. A swing under 1.5 V is taken for bumps on a steady
    // line (a quiet RS232 line carries some of a volt) and named by its level.
    clearVerdict(g_v);
    if (g_f.hiMv - g_f.loMv < 1500) {
        Feat q = g_f;
        q.active = false;
        steadyLevel(q, g_v);
        return;
    }
    strcpy(g_v.name, "unknown");
    g_v.note = "activity";
}

// --- the stretches between edges ----------------------------------------------------
static void sortU16(uint16_t* a, uint16_t n) {
    for (uint16_t i = 1; i < n; i++) {
        uint16_t x = a[i], j = i;
        while (j && a[j - 1] > x) { a[j] = a[j - 1]; j--; }
        a[j] = x;
    }
}

// Share, per cent, of a list within pct % (and a sample) of m.
static uint8_t within(const uint16_t* a, uint16_t n, uint16_t m, uint8_t pct) {
    if (!n) return 0;
    uint16_t tol = (uint16_t)((uint32_t)m * pct / 100);
    if (tol < 1) tol = 1;
    uint16_t k = 0;
    for (uint16_t i = 0; i < n; i++) if (a[i] + tol >= m && a[i] <= m + tol) k++;
    return (uint8_t)((uint32_t)k * 100 / n);
}

// How many of a sorted list lie in [lo, hi).
static uint16_t countIn(const uint16_t* a, uint16_t n, uint32_t lo, uint32_t hi) {
    uint16_t k = 0;
    for (uint16_t i = 0; i < n && a[i] < hi; i++) if (a[i] >= lo) k++;
    return k;
}

// The shortest stretch v, of the highs and the lows (each sorted), with at
// least 3 of them, and 1/32 of all, in [v, 1.5 v): one that comes back.
// 0 when no stretch does.
static uint16_t shortestRepeated(const uint16_t* H, uint16_t nH, const uint16_t* L, uint16_t nL) {
    uint16_t need = (uint16_t)((nH + nL) / 32);
    if (need < 3) need = 3;
    uint16_t i = 0, j = 0;
    while (i < nH || j < nL) {
        uint16_t v = (j >= nL || (i < nH && H[i] <= L[j])) ? H[i++] : L[j++];
        if (!v) continue;
        uint32_t hi = (uint32_t)v * 3 / 2 + 1;
        if (countIn(H, nH, v, hi) + countIn(L, nL, v, hi) >= need) return v;
    }
    return 0;
}

// Takes the stretches apart in place (the highs first, then the lows, each
// sorted, the level bit dropped): RAM is short in the images run from RAM.
static void timing(Feat& f) {
    uint16_t nH = 0;
    for (uint16_t i = 0; i < g_nRuns; i++) {
        if (g_runs[i] & 0x8000) {
            uint16_t x = g_runs[i];
            g_runs[i] = g_runs[nH];
            g_runs[nH++] = x;
        }
    }
    for (uint16_t i = 0; i < g_nRuns; i++) g_runs[i] &= 0x7FFF;
    uint16_t* H = g_runs;
    uint16_t* L = g_runs + nH;
    uint16_t  nL = (uint16_t)(g_nRuns - nH);
    sortU16(H, nH);
    sortU16(L, nL);
    uint16_t maxH = nH ? H[nH - 1] : 0, maxL = nL ? L[nL - 1] : 0;
    uint16_t mH = nH ? H[nH / 2] : 0, mL = nL ? L[nL / 2] : 0;
    f.maxOtherNs = (uint32_t)((uint64_t)(f.restHigh ? maxL : maxH) * 1000000000u / MEASURE_HZ);

    // Nearly every one: a clock in bursts has 94 % of its highs and lows alike
    // too (seen in the demo I2C SCL).
    f.periodic = nH >= 8 && nL >= 8 && within(H, nH, mH, 10) >= 97 && within(L, nL, mL, 10) >= 97;
    if (f.periodic) {
        f.freqHz = MEASURE_HZ / (mH + mL);
        f.dutyPm = (uint16_t)((uint32_t)mH * 1000 / (mH + mL));
    }
    // A slow square wave (the 1 kHz of a calibration output): its half period
    // is about a block long, and most stretches are cut by a block that went
    // by unread, so the eight of each are never there. The few caught whole
    // are enough to say it is one, once it has failed as a serial line
    // (finishUart()).
    f.square = f.periodic;
    if (!f.periodic && nH >= 2 && nL >= 2 && within(H, nH, mH, 10) == 100 && within(L, nL, mL, 10) == 100) {
        f.square = true;
        f.freqHz = MEASURE_HZ / (mH + mL);
        f.dutyPm = (uint16_t)((uint32_t)mH * 1000 / (mH + mL));
    }

    // The shortest stretch that comes back: a glitch on the line (a spike of
    // a sample or a few, as when the sender's port is opened again at another
    // speed) is alone or nearly, a bit of a serial line comes back many
    // times. Taken from the shortest glitch, 9600 after 115200 was read at
    // 1.25 MS/s and took over 10 s, 19200 after 9600 was "not serial: 60 %"
    // (pulse 0.5 us on the screen, 2026-11-07). The raw shortest only when
    // too few stretches came to tell.
    uint16_t rep = shortestRepeated(H, nH, L, nL);
    if (rep) f.minRunNs = (uint32_t)((uint64_t)rep * 1000000000u / MEASURE_HZ);

    // One bit: the shortest stretches, high and low, averaged. The shortest
    // alone is a sample off either way, and LIN's 19200 and 20000 are 4 %
    // apart.
    uint16_t m1 = 0xFFFF;
    if (nH) m1 = H[0];
    if (nL && L[0] < m1) m1 = L[0];
    if (rep) m1 = rep;
    f.oneBitNs = 0;
    if (m1 && m1 != 0xFFFF) {
        uint32_t sum = 0, cnt = 0;
        for (uint16_t i = 0; i < nH && (uint32_t)H[i] * 2 < (uint32_t)m1 * 3; i++) { sum += H[i]; cnt++; }
        for (uint16_t i = 0; i < nL && (uint32_t)L[i] * 2 < (uint32_t)m1 * 3; i++) { sum += L[i]; cnt++; }
        f.oneBitNs = (uint32_t)((uint64_t)sum * 1000000000u / ((uint64_t)cnt * MEASURE_HZ));
    }

    // A clock in bursts: the lows alike, the highs alike but for the long
    // ones between bursts.
    f.clock = false;
    if (!f.periodic && nL >= 16 && within(L, nL, mL, 25) >= 85) {
        uint16_t nShort = 0, gaps = 0;
        for (uint16_t i = 0; i < nH; i++) { if (H[i] < 3u * mL) nShort++; else gaps++; }
        if (gaps && nShort >= nL / 2) {
            uint16_t mHs = H[nShort / 2];               // H is sorted: the short ones come first
            f.clock = true;
            f.clockHz = MEASURE_HZ / (mL + mHs);
            f.perBurst = (uint16_t)((nL + gaps / 2) / gaps);
        }
    }
}

// --- the picture ------------------------------------------------------------------
// A block of the signal, for the screen: of the blocks at the range in
// force, the latest whose swing is at least three quarters of the widest so
// far, so that a line that talks in bursts is shown talking. Not taken while
// the screen is drawing it.
static uint16_t g_wave[Acq::BUF_LEN / 2];
static uint16_t g_waveN;
static uint32_t g_waveHz;
static uint16_t g_waveSpan;
static volatile bool g_waveNew, g_waveBusy;

T232_RAM static void keep(const uint16_t* s, uint16_t n) {
    if (g_waveBusy) return;
    uint16_t lo = 0x0FFF, hi = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = s[i] & 0x0FFF;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    uint16_t span = (uint16_t)(hi - lo);
    if (g_waveN && (uint32_t)span * 4 < (uint32_t)g_waveSpan * 3) return;
    if (span > g_waveSpan) g_waveSpan = span;
    if (n > Acq::BUF_LEN / 2) n = Acq::BUF_LEN / 2;
    memcpy(g_wave, s, n * sizeof(uint16_t));
    g_waveN = n;
    g_waveHz = Acq::sampleRate();
    g_waveNew = true;
}

static void waveReset() {
    Acq::pause();
    g_waveN = 0; g_waveSpan = 0; g_waveNew = false;
    Acq::resume();
}

// A square wave in the picture block: three whole highs and three whole lows
// at least, each kind alike within a tenth (and a sample). The stretches
// measured at 2 MS/s (timing()) do not show a slow one: at 1 kHz a half
// period is a block long, and blocks go by unread while the screen is drawn
// (the calibration output: "pulse 367 us", 2026-11-11). The block kept while
// the line is read as serial is at a rate that suits it, 10 ms of it at
// 100 kS/s. Sets freqHz and dutyPm.
static bool squareInWave(Feat& f) {
    Acq::pause();
    uint16_t n = g_waveN, lo = 0x0FFF, hi = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = g_wave[i] & 0x0FFF;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    uint32_t sum[2] = { 0, 0 }, cnt[2] = { 0, 0 }, mn[2] = { 0xFFFFFFFFu, 0xFFFFFFFFu }, mx[2] = { 0, 0 };
    if (n && hi > lo) {
        uint16_t mid = (uint16_t)((lo + hi) / 2), hy = (uint16_t)((hi - lo) / 6);
        bool high = (g_wave[0] & 0x0FFF) > mid, whole = false;
        uint32_t run = 0;
        for (uint16_t i = 1; i < n; i++) {
            uint16_t v = g_wave[i] & 0x0FFF;
            bool flip = high ? v < mid - hy : v > mid + hy;
            if (!flip) { run++; continue; }
            if (whole) {
                uint8_t k = high ? 1 : 0;
                run++;
                sum[k] += run; cnt[k]++;
                if (run < mn[k]) mn[k] = run;
                if (run > mx[k]) mx[k] = run;
            }
            whole = true;
            run = 0;
            high = !high;
        }
    }
    uint32_t hz = g_waveHz;
    Acq::resume();
    for (uint8_t k = 0; k < 2; k++)
        if (cnt[k] < 3 || mx[k] * 10u > mn[k] * 11u + 10u) return false;
    uint64_t per = (uint64_t)sum[1] * cnt[0] + (uint64_t)sum[0] * cnt[1];       // a period, times both counts
    f.freqHz = (uint32_t)((uint64_t)hz * cnt[0] * cnt[1] / per);
    f.dutyPm = (uint16_t)((uint64_t)sum[1] * cnt[0] * 1000u / per);
    return true;
}

// --- the samples ----------------------------------------------------------------
T232_RAM static void monAcc(const uint16_t* s, uint16_t n) {
    uint16_t lo = g_mLo, hi = g_mHi;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = s[i] & 0x0FFF;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    g_mLo = lo; g_mHi = hi; g_mSum += sum; g_mN += n;
}

T232_RAM void block(const uint16_t* s, uint16_t n, bool overrun) {
    uint8_t ph = g_phase;
    if (ph == P_DONE && g_mon) monAcc(s, n);
    if (ph == P_UART || (ph == P_DONE && g_found.uart)) {
        if (overrun) Rs232::noteOverrun();
        Rs232::feed(s, n, Acq::sampleRate());
        keep(s, n);
        return;
    }
    if (ph == P_TEACH) { capTake(s, n); return; }
    if (ph == P_LEVELS || ph == P_EDGES) keep(s, n);
    if (ph == P_WIDE || ph == P_LEVELS) {
        uint16_t lo = g_lo, hi = g_hi;
        uint32_t sum = 0;
        for (uint16_t i = 0; i < n; i++) {
            uint16_t v = s[i] & 0x0FFF;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
            sum += v;
        }
        g_lo = lo; g_hi = hi; g_sum += sum; g_n += n;
        return;
    }
    if (ph == P_EDGES) {
        // Edges across the middle, with hysteresis, and the stretches
        // between them. A stretch is counted only once both its edges have
        // been seen; it runs on from the block before unless that went by
        // unread.
        const uint16_t up = (uint16_t)(g_thr + g_hyst), down = (uint16_t)(g_thr - g_hyst);
        bool     cont  = g_carryOk && !overrun;
        bool     high  = cont ? g_carryHigh : (s[0] & 0x0FFF) > g_thr;
        bool     whole = cont;
        uint32_t run = cont ? g_carry : 0, minRun = g_minRun, high_n = 0, edges = 0;
        uint16_t nr = g_nRuns;
        for (uint16_t i = cont ? 0 : 1; i < n; i++) {
            uint16_t v = s[i] & 0x0FFF;
            if (v > g_thr) high_n++;
            bool flip = high ? v < down : v > up;
            if (!flip) { run++; continue; }
            if (whole) {
                if (run + 1 < minRun) minRun = run + 1;
                if (nr < RUNS_MAX) g_runs[nr++] = (uint16_t)((run + 1 > 0x7FFF ? 0x7FFF : run + 1) | (high ? 0x8000 : 0));
            }
            whole = true;
            run = 0;
            high = !high;
            edges++;
        }
        g_carry = run; g_carryHigh = high; g_carryOk = whole;   // a stretch cut at its start is not carried
        g_nRuns = nr;
        g_minRun = minRun; g_high += high_n; g_edges += edges; g_n += n;
    }
}

// --- the screen -------------------------------------------------------------------
#define VERDICT_Y  26
#define SCORE_Y    50
#define NOTE_Y     68
#define WAVE_Y     88
#define WAVE_H     88
#define DET_Y      182
#define RX_Y       206
#define FOOT_Y     228

static void bar(Lcd& tft, const char* what) {
    tft.fillRect(0, 0, 320, 20, TH.bar);
    tft.setTextSize(1);
    tft.setTextColor(TH.barText, TH.bar);
    tft.setCursor(2, 6);
    tft.print("AprecID");
    tft.setCursor(74, 6);
    tft.print(what);
    if (Config::demo) { tft.setTextColor(TH.err, TH.bar); tft.setCursor(150, 6); tft.print("DEMO"); tft.setTextColor(TH.barText, TH.bar); }
    tft.setCursor(200, 6);
    tft.print('R'); tft.print(Frontend::range());
    tft.print(Frontend::couplingDC() ? " DC  " : " AC  ");
    uint32_t hz = Acq::sampleRate();
    if (hz >= 1000000) { tft.print(hz / 1000000); tft.print('.'); tft.print((hz / 100000) % 10); tft.print(" MS/s"); }
    else               { tft.print(hz / 1000); tft.print(" kS/s"); }
}

static uint8_t teachKind();

// RS232 and TTL have the tester's bus health view, as the Fluke 125 has it
// (Health.cpp); DOWN opens it, in demo too.
static bool healthy() {
    return g_found.uart && g_found.baud && (!strncmp(g_v.name, "RS232", 5) || !strncmp(g_v.name, "TTL", 3));
}

// With the hint ("AUTO details") the line is packed closer.
static void footer(Lcd& tft) {
    tft.drawFastHLine(0, FOOT_Y - 6, 320, TH.line);
    tft.fillRect(0, FOOT_Y - 2, 320, 12, TH.bg);
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, FOOT_Y);
    const bool health = healthy();
    const char* gap = g_hint || health ? "  " : "   ";
    if (!(g_hint && health)) { tft.print("RUN again"); tft.print(gap); }
    if (g_hint) {
        tft.setTextColor(TH.text, TH.bg);
        tft.print("AUTO details");
        tft.setTextColor(TH.muted, TH.bg);
        tft.print(gap);
    }
    if (health) { tft.print((char)0x1F); tft.print(" health"); tft.print(gap); }
    if (!Config::demo) { tft.print(g_hint || health ? "OK tester" : "OK to the tester"); tft.print(gap); }
    tft.print("OK held menu");
    if (Config::demo) {                         // on the same line: the rows above are full
        tft.setTextColor(TH.err, TH.bg);
        tft.print(gap); tft.print((char)0x11); tft.print((char)0x10); tft.print(' ');
        tft.print(DemoSig::name(DemoSig::which()));
    }
}

// Hands-off, once a result is up, the next one is worked out under it, said
// only in the bar: the screen does not go blank at every pad.
static bool g_shown;

static void drawBusy(Lcd& tft, const char* what) {
    if (Config::handsOff && g_shown) { bar(tft, what); return; }
    tft.fillScreen(TH.bg);
    bar(tft, what);
    tft.setTextSize(2);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, VERDICT_Y + 8);
    tft.print(what);
    tft.print("...");
}

// A quiet RS232 line being waited on: said in full even hands-off, where the
// last result would otherwise stay up, and that was another line.
static void drawWaiting(Lcd& tft) {
    tft.fillScreen(TH.bg);
    bar(tft, "waiting for traffic");
    tft.setTextSize(3);
    tft.setTextColor(TH.text, TH.bg);
    tft.setCursor(8, VERDICT_Y + 4);
    tft.print("RS232 idle");
    tft.setTextSize(2);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, VERDICT_Y + 38);
    tft.print("waiting for traffic...");
    g_shown = false;
}

static void printTime(Lcd& tft, uint32_t ns) {
    if (ns >= 10000000) { tft.print(ns / 1000000); tft.print(" ms"); }
    else if (ns >= 10000) { tft.print(ns / 1000); tft.print(" us"); }
    else { tft.print(ns / 1000); tft.print('.'); tft.print((ns / 100) % 10); tft.print(" us"); }
}

static void drawRx(Lcd& tft) {
    tft.fillRect(0, RX_Y, 320, 12, TH.bg);
    if (!g_found.uart) return;
    tft.setTextSize(1);
    tft.setTextColor(TH.text, TH.bg);
    tft.setCursor(8, RX_Y);
    tft.print(g_rx);
}

// The kept block across the whole width, each column the lowest to the
// highest sample it covers. Scaled to the signal as a scope's auto set does:
// from its lowest to its highest sample, with zero volts always in (dotted),
// a tenth more at each end and a volt at the least, so that a steady level
// is not blown up into its noise. The range and the time across the box in
// its corners.
// RS232 or TTL read with its speed: the place of the picture can hold the
// last characters instead.
static bool readUart() {
    return g_found.uart && g_found.baud && !g_v.lin;
}
static bool showChars() {
    return g_view == VIEW_BOXES && readUart();
}
static bool showLines() {
    return g_view == VIEW_LINES && readUart();
}

// The last 5 characters in boxes, as on the DSO-152, where the picture is.
static void drawChars(Lcd& tft) {
    g_txtNew = false;
    tft.fillRect(0, WAVE_Y, 320, WAVE_H, TH.bg);
    tft.drawRect(0, WAVE_Y, 320, WAVE_H, TH.line);
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(4, WAVE_Y + 3);
    tft.print("the last characters read");
    tft.setCursor(262, WAVE_Y + 3);
    tft.print("MODE wave");
    // the boxes 40 high, in the middle of the 76 under the labels
    displayLargeSuffix(tft, g_txt, g_txtN, WAVE_Y + 12 + (WAVE_H - 13 - 40) / 2 + 6);
}

// The messages a line each, in the place of the
// picture: the last six.
static void drawLines(Lcd& tft, bool full) {
    g_linesNew = false;
    if (full) {
        tft.fillRect(0, WAVE_Y, 320, WAVE_H, TH.bg);
        tft.drawRect(0, WAVE_Y, 320, WAVE_H, TH.line);
        tft.setTextSize(1);
        tft.setTextColor(TH.muted, TH.bg);
        tft.setCursor(4, WAVE_Y + 3);
        tft.print("the messages, a line each");
        tft.setCursor(256, WAVE_Y + 3);
        tft.print("MODE boxes");
    }
    displayCharsLines(tft, g_lines, g_linesN, g_lgaps, g_lgapN, full, WAVE_Y + 13, 6);
}

static void drawWave(Lcd& tft) {
    static const char* const VDIV[] = { "10mV", "20mV", "50mV", "0.1V", "0.2V", "0.5V", "1V", "2V", "5V", "10V" };
    g_waveBusy = true;
    g_waveNew = false;
    Link::wave(g_wave, g_waveN, g_waveHz, "insp", "insp", false);    // the page has it in both views
    if (showChars()) { g_waveBusy = false; drawChars(tft); return; }
    if (showLines()) { g_waveBusy = false; drawLines(tft, false); return; }
    tft.fillRect(0, WAVE_Y, 320, WAVE_H, TH.bg);
    tft.drawRect(0, WAVE_Y, 320, WAVE_H, TH.line);
    const int16_t top = WAVE_Y + 12, h = WAVE_H - 13;     // the labels above the trace
    const int32_t zero = (int32_t)Frontend::zeroCounts();
    int32_t wLo = zero, wHi = zero;
    for (uint16_t i = 0; i < g_waveN; i++) {
        int32_t v = g_wave[i] & 0x0FFF;
        if (v < wLo) wLo = v;
        if (v > wHi) wHi = v;
    }
    int32_t m = (wHi - wLo) / 10, least = (int32_t)Frontend::countsPerVolt();
    wLo -= m; wHi += m;
    if (wHi - wLo < least) { int32_t c = (wHi + wLo) / 2; wLo = c - least / 2; wHi = c + least / 2; }
    const int32_t span = wHi - wLo;
    auto yOf = [&](int32_t v) -> int16_t {
        int32_t y = top + h - 1 - (v - wLo) * (h - 1) / span;
        return (int16_t)(y < top ? top : y > top + h - 1 ? top + h - 1 : y);
    };
    int16_t yz = yOf(zero);
    for (int16_t x = 2; x < 318; x += 4) tft.drawPixel(x, yz, TH.muted);
    if (g_waveN) {
        uint16_t n = g_waveN;
        for (int16_t x = 0; x < 318; x++) {
            uint16_t a = (uint16_t)((uint32_t)x * n / 318), b = (uint16_t)((uint32_t)(x + 1) * n / 318);
            if (b <= a) b = (uint16_t)(a + 1);
            uint16_t lo = 0x0FFF, hi = 0;
            // from the last sample of the column before: an edge that falls
            // between two columns is still drawn as a line, not left as a gap
            for (uint16_t i = a ? a - 1 : 0; i < b && i < n; i++) {
                uint16_t v = g_wave[i] & 0x0FFF;
                if (v < lo) lo = v;
                if (v > hi) hi = v;
            }
            int16_t y1 = yOf(hi), y2 = yOf(lo);
            tft.drawFastVLine((int16_t)(x + 1), y1, (int16_t)(y2 - y1 + 1), TH.trace);
        }
    }
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(4, WAVE_Y + 3);
    tft.print('R'); tft.print(Frontend::range()); tft.print(" (");
    tft.print(VDIV[Frontend::range() > 9 ? 9 : Frontend::range()]);
    tft.print("/div)");
    if (readUart()) { tft.print("   MODE text"); }
    else tft.print(", picture scaled to fit");
    if (g_waveN && g_waveHz) {
        uint32_t us = (uint32_t)((uint64_t)g_waveN * 1000000u / g_waveHz);
        tft.setCursor(268, WAVE_Y + 3);
        if (us >= 1000) { tft.print(us / 1000); tft.print('.'); tft.print((us / 100) % 10); tft.print(" ms"); }
        else            { tft.print(us); tft.print(" us"); }
    }
    g_waveBusy = false;
}

static void drawResult(Lcd& tft) {
    g_shown = true;
    tft.fillScreen(TH.bg);
    bar(tft, Config::handsOff ? "watching" : "done");

    // what it is, big
    tft.setTextSize(3);
    tft.setTextColor(TH.text, TH.bg);
    tft.setCursor(8, VERDICT_Y);
    tft.print(g_v.name);
    if (g_found.uart && g_found.baud) {
        tft.print(' ');
        tft.print(g_found.baud);
    }

    // the word more, the frame and the score, or the level
    tft.setTextSize(2);
    tft.setCursor(8, SCORE_Y);
    if (g_v.showLevel) {
        tft.setTextColor(TH.trace, TH.bg);
        Frontend::printVolts(tft, g_f.meanMv, 2, true);
        tft.setTextColor(TH.text2, TH.bg);
        tft.print("  "); tft.print(g_v.note);
    } else if (g_found.uart) {
        if (g_found.baud) {
            tft.setTextColor(TH.trace, TH.bg);
            if (g_v.lin) tft.print(g_linEnhanced ? "2.x" : g_linClassic ? "1.x" : "LIN");
            else { tft.print(g_found.dataBits); tft.print(g_found.parity); tft.print(g_found.stopBits); }
            tft.print("  ");
            tft.setTextColor(g_score >= 90 ? TH.ok : TH.err, TH.bg);
            tft.print(g_score); tft.print("% of "); tft.print(g_scored);
        } else {
            tft.setTextColor(TH.err, TH.bg);
            tft.print("no serial frames read");
        }
        if (g_v.lin && g_v.info[0]) { tft.setCursor(8, NOTE_Y); tft.setTextColor(TH.text2, TH.bg); tft.print(g_v.info); }  // the version is on the line above
        else if (g_v.note[0]) { tft.setCursor(8, NOTE_Y); tft.setTextColor(TH.text2, TH.bg); tft.print(g_v.note); }
    } else {
        if (g_v.info[0]) {
            tft.setTextColor(TH.trace, TH.bg);
            tft.print(g_v.info);
            tft.setCursor(8, NOTE_Y);
        }
        tft.setTextColor(TH.text2, TH.bg);
        tft.print(g_v.note);
    }

    if (showLines()) drawLines(tft, true);       // the screen is new: rule, frame and all
    else             drawWave(tft);

    // the features, small, under the picture
    tft.setTextSize(1);
    tft.setTextColor(TH.text2, TH.bg);
    tft.setCursor(8, DET_Y);
    tft.print("low ");   Frontend::printVolts(tft, g_f.loMv, 2, true);
    tft.print("  high "); Frontend::printVolts(tft, g_f.hiMv, 2, true);
    tft.print("  mean "); Frontend::printVolts(tft, g_f.meanMv, 2, true);
    tft.setCursor(8, DET_Y + 12);
    if (g_f.clipped) {
        tft.setTextColor(TH.err, TH.bg);
        tft.print("beyond +-50V: clipped   ");
        tft.setTextColor(TH.text2, TH.bg);
    }
    if (g_f.active) {
        tft.print("edges "); tft.print(g_f.edgesPerS); tft.print("/s  ");
        tft.print(g_f.highPct); tft.print("% high  pulse ");
        if (g_f.minRunNs) {
            printTime(tft, g_f.minRunNs);
            tft.print(" ~"); tft.print(1000000000u / g_f.minRunNs); tft.print("/s");
        } else {
            tft.print("> 0.5 ms");
        }
    }
    drawRx(tft);
    footer(tft);
}

// --- the steps --------------------------------------------------------------------
static void startPhase(uint8_t ph) {
    Acq::pause();
    accReset();
    if (ph == P_EDGES) g_nRuns = 0;         // kept after, to be read over SWD
    g_phase = ph;
    Acq::resume();
    g_t0 = millis();
}

static void settleThen(uint8_t ph) {
    g_after = ph;
    startPhase(P_SETTLE);
}

static void mbReset();

static void begin(Lcd& tft, bool full = false) {
    g_linesKeep = g_linesN;                 // what the readings before found holds
    g_mon = false;
    g_full = full;
    g_hint = false;
    g_f = Feat();
    waveReset();
    mbReset();
    g_teach = 0;
    g_found = Found();
    g_rxN = 0;
    g_rx[0] = 0;
    g_txtN = 0; g_txtNew = false;
    g_decided = false; g_retried = false; g_relearns = 0;
    g_first9600 = true; g_at9600 = false;
    g_score = 0; g_scored = 0;
    g_idleWait = false; g_traffic = false;
    Acq::pause();
    Frontend::setCouplingDC(true);
    Frontend::setRange(RANGE_WIDE);
    Acq::setSampleRate(MEASURE_HZ);
    Acq::discard();
    Acq::resume();
    drawBusy(tft, "measuring");
    settleThen(P_WIDE);
}

void enter(Lcd& tft, Source src) {
    Keys::sync();
    g_src = src;
    g_shown = false;
    linesClear();
    begin(tft);
}

static int32_t toMv(int32_t counts) {
    return (counts - (int32_t)Frontend::zeroCounts()) * 1000 / (int32_t)Frontend::countsPerVolt();
}

// The most sensitive range from RANGE_MIN up that holds what was seen.
static uint8_t pickRange(int32_t maxAbsMv) {
    for (uint8_t r = RANGE_MIN; r < RANGE_WIDE; r++)
        if ((int64_t)maxAbsMv * Frontend::countsPerVoltAt(r) / 1000 < FIT_COUNTS) return r;
    return RANGE_WIDE;
}

// A result: on the screen, and in hands-off a beep and the watch on it.
static uint32_t g_results;
uint32_t results() { return g_results; }

// The result for the page on the PC (Link.h): what the screen shows, as JSON.
// The result for the page (apre protocol 1, "insp"): its class, what the
// screen shows, and the levels.
static void linkResult() {
    char buf[400];
    Link::Json j(buf, sizeof(buf));
    const char* cls = g_v.showLevel ? "level" : g_v.lin ? "lin" : g_found.uart ? "uart"
                    : !strncmp(g_v.name, "PWM", 3) ? "pwm" : !strncmp(g_v.name, "CAN", 3) ? "can"
                    : !strncmp(g_v.name, "I2C SCL", 7) ? "clock" : "other";
    j.str("t", "insp").str("class", cls).str("name", g_v.name);
    if (g_found.uart && g_found.baud) {
        j.num("baud", (int32_t)g_found.baud).num("scorePm", g_score * 10);
        if (g_v.lin) {
            j.num("headers", (int32_t)g_linHeaders).num("recessiveMv", g_f.hiMv).num("dominantMv", g_f.loMv)
             .str("checksum", g_linEnhanced ? "enhanced" : g_linClassic ? "classic" : "none");
        } else {
            char fmt[4] = { (char)('0' + g_found.dataBits), g_found.parity, (char)('0' + g_found.stopBits), 0 };
            j.str("fmt", fmt).num("frames", (int32_t)g_scored);
            if (g_mbOk) j.flag("modbus", true);
        }
    }
    if (cls[0] == 'p') j.num("hz", (int32_t)g_f.freqHz).num("dutyPm", g_f.dutyPm);
    if (cls[0] == 'c' && cls[1] == 'l') j.num("hz", (int32_t)g_f.clockHz).num("perBurst", g_f.perBurst);
    if (cls[0] == 'c' && cls[1] == 'a') {
        static const uint32_t STD[] = { 125000, 250000, 500000, 1000000 };
        uint32_t rate = g_f.minRunNs ? 1000000000u / g_f.minRunNs : 0;
        uint32_t std = snapTo(rate, STD, 4);
        j.num("bitrate", (int32_t)(std ? std : rate))
         .num("recessiveMv", g_f.restHigh ? g_f.hiMv : g_f.loMv).num("dominantMv", g_f.restHigh ? g_f.loMv : g_f.hiMv);
    }
    if (g_v.info[0]) j.str("info", g_v.info);
    if (g_v.note && g_v.note[0]) j.str("note", g_v.note);
    j.num("loMv", g_f.loMv).num("hiMv", g_f.hiMv).num("meanMv", g_f.meanMv).num("range", g_f.range)
     .flag("active", g_f.active);
    if (g_f.clipped) j.flag("clipped", true);
    if (Config::demo) j.flag("demo", true);
    Link::result(j.done());
}

static void finish(Lcd& tft) {
    g_results++;
    if (readUart()) g_linesKeep = g_linesN;
    else            linesCut(g_linesKeep);
    g_phase = P_DONE;
    drawResult(tft);
    linkResult();
    g_drawMs = millis();
    if (!Config::handsOff) return;
    if (!g_f.active) Beep::tone(g_f.meanMv > -250 && g_f.meanMv < 250 ? 1000 : 2000, 80);   // GND, another level
    else             Beep::tone(3000, 150);                                                 // a line that moves
    Acq::pause();
    g_mLo = 0xFFFF; g_mHi = 0; g_mSum = 0; g_mN = 0;
    g_mon = true;
    Acq::resume();
    g_monMs = millis();
    g_offRestMs = 0;
    Rs232::Stats st = Rs232::stats();
    g_frMs = g_monMs; g_frGood = st.bytes; g_frBad = st.framingErrors + st.parityErrors;
}

// A serial result whose frames stopped holding: another speed or frame on the
// same levels (the levels alone do not show it). Over FRAMES_MS, a quarter of
// the frames bad, four at least; or the decoder found another speed.
static bool framesChanged() {
    if (!g_found.uart || g_v.lin) return false;
    uint32_t now = millis();
    if (now - g_frMs < FRAMES_MS) return false;
    Rs232::Stats st = Rs232::stats();
    uint32_t good = st.bytes, bad = st.framingErrors + st.parityErrors;
    uint32_t dGood = good >= g_frGood ? good - g_frGood : good;
    uint32_t dBad  = bad >= g_frBad ? bad - g_frBad : bad;
    g_frMs = now; g_frGood = good; g_frBad = bad;
    if (dBad >= 4 && dBad * 4 >= dGood + dBad) return true;
    if (st.locked && st.baud && g_found.baud) {
        uint32_t a = st.baud, b = g_found.baud, d = a > b ? a - b : b - a;
        if (d * 20 > b) return true;        // past 5 %
    }
    return false;
}

// Sets the decoder going on a line that looks serial, at a rate that gives
// eight samples a bit or more on the shortest pulse seen.
static void startUart(Lcd& tft) {
    uint32_t rate = UART_RATES[0];
    if (g_f.minRunNs) {
        uint32_t baud = 1000000000u / g_f.minRunNs;
        rate = UART_RATES[sizeof(UART_RATES) / sizeof(UART_RATES[0]) - 1];
        for (uint8_t i = 0; i < sizeof(UART_RATES) / sizeof(UART_RATES[0]); i++)
            if (UART_RATES[i] >= 8 * baud) { rate = UART_RATES[i]; break; }
    }
    // 9600 first, the speed most lines have: given outright, with
    // only the frame to learn, it is read sooner and cannot be mislearnt. Not
    // when the shortest pulse is plainly longer than its bit (104 us): a slow
    // square wave reads as clean 9600 8N1 frames.
    g_at9600 = g_first9600 && !g_v.lin && g_f.minRunNs <= 156000;
    g_first9600 = false;
    if (g_at9600) rate = UART_RATES[0];
    // a quiet line has shown no pulse to size the rate by: after 9600, the
    // fastest, which reads every speed up to 115200
    else if (g_idleWait && !g_f.minRunNs) rate = UART_RATES[sizeof(UART_RATES) / sizeof(UART_RATES[0]) - 1];
    Acq::pause();
    g_phase = P_SETTLE;
    while (Rs232::read() >= 0) {}
    Rs232::setInverted(g_v.inverted);
    Rs232::setZeroCross(g_v.zeroCross);
    if (g_at9600) Rs232::setFixedBaud(9600, true);
    else          Rs232::setFixedBaud(g_v.lin ? g_linBaud : 0);
    Rs232::setBreaks(g_v.lin);
    linesCut(g_linesKeep);                  // a try before this one in the same reading did not hold
    if (g_v.lin) linReset();
    Rs232::reset();
    // RS232 waited on from rest: its threshold is zero volts from the start,
    // so the decoder reads the block in which it learns the levels too.
    if (g_idleWait && g_v.zeroCross) Rs232::setThreshold(Rs232::zeroCounts());
    Rs232::clearCounts();
    Acq::setSampleRate(rate);
    Acq::discard();
    Acq::resume();
    g_decided = false;
    g_waveN = 0; g_waveSpan = 0;          // pictures at the rate it is read at from here on
    g_found.uart = true;
    g_found.inverted = g_v.inverted;
    g_found.rateHz = Acq::sampleRate();
    g_traffic = false;
    g_waitT0 = millis();
    if (g_idleWait) drawWaiting(tft);
    else            drawBusy(tft, "reading line");
    settleThen(P_UART);
}

// A result named by its level. On RS232 at mark, read it as a serial line
// all the same and wait for traffic (IDLE_WAIT_MS); anything else is said at
// once. Named so whether the line was quiet or not: bumps of a volt or so at
// mark make it "active" with no detector taking it, and it was then said to
// be "RS232 idle" every few seconds, a message of a line that talks every
// two seconds slipping by each time (v0.21).
static void finishSteady(Lcd& tft) {
    // or with no name at all ("unknown", "activity": bumps of over 2 V, v0.22),
    // so long as the whole window stays below -3 V: no frame of RS232 does
    // that, its start bit crosses zero.
    bool atMark = !strcmp(g_v.name, "RS232 idle") || (g_f.hiMv < -3000 && g_f.loMv > -16000);
    if (!g_v.lin && !g_v.uart && atMark) {
        strcpy(g_v.name, "RS232");
        g_v.info[0] = 0;
        g_v.note = "";
        g_v.showLevel = false;
        g_v.uart = true;
        g_v.inverted = true;
        g_v.zeroCross = true;
        g_v.retry = false;
        g_idleWait = true;
        startUart(tft);
        return;
    }
    finish(tft);
}

static void startUart(Lcd& tft);

static void finishUart(Lcd& tft) {
    Rs232::Stats  st = Rs232::stats();
    Rs232::Format fm = Rs232::format();
    if (g_v.lin) {                              // scored on its frames: headers whose sync and parity held
        uint8_t ids = 0, silent = 0;
        for (uint8_t i = 0; i < 64; i++) if (g_linIds[i].seen) { ids++; if (!g_linIds[i].answered) silent++; }
        g_scored = g_linHeaders;
        g_score = g_linHeaders ? (uint8_t)(g_linGood * 100 / g_linHeaders) : 0;
        if (g_linGood) {
            g_found.baud = g_linBaud;
            g_found.dataBits = 8; g_found.parity = 'N'; g_found.stopBits = 1;
            snprintf(g_v.info, sizeof(g_v.info), "%u IDs, %u silent", ids, silent);
            g_v.note = g_linEnhanced ? "LIN 2.x, enhanced checksum"
                     : g_linClassic  ? "LIN 1.x, classic checksum" : "no slave answered";
        } else {
            snprintf(g_v.info, sizeof(g_v.info), "no LIN frame read");
            g_found.uart = false;
        }
        finish(tft);
        return;
    }
    if (g_at9600) {                             // the try at 9600: taken only if the frames hold
        uint32_t good = g_decided ? st.bytes - g_b0 : 0, bad = st.framingErrors + st.parityErrors;
        if (!(g_decided && good + bad >= 8 && good * 10 >= (good + bad) * 9)) {
            g_rxN = 0; g_rx[0] = 0; g_txtN = 0; g_txtNew = false;
            mbReset();
            startUart(tft);                     // not 9600: worked out from the line
            return;
        }
    }
    if (!g_decided && g_v.retry && !g_retried) {   // nothing read: the other way up, once
        g_retried = true;
        g_v.inverted = !g_v.inverted;
        startUart(tft);
        return;
    }
    if (!g_decided && g_v.retry && !g_v.note[0]) g_v.note = "I2C SDA, SPI, or a GPIO?";
    if (g_mbOk) g_v.note = "Modbus RTU: CRC-16 holds";
    if (g_decided && st.locked && st.baud) {
        g_found.baud = st.baud;
        g_found.dataBits = fm.dataBits;
        g_found.parity = fm.parity;
        g_found.stopBits = fm.stopBits;
        uint32_t good = st.bytes - g_b0;
        uint32_t bad = st.framingErrors + st.parityErrors;  // counted from the moment the frame was settled
        g_scored = good + bad;
        g_score = g_scored ? (uint8_t)(good * 100 / g_scored) : 0;
    }
    // RS232 and TTL serial are the standard speeds only. Anything else that
    // reads as frames is something else: a square wave is alternating bits at
    // twice its frequency (the 1 kHz calibration output was "2000 baud 7E1
    // 100%", 2026-11-07).
    bool odd = g_found.baud && !Rs232::isStandard(g_found.baud);
    if (odd && (g_f.square || squareInWave(g_f))) {
        namePwm(g_f, g_v);
        g_v.note = "";
        g_v.uart = false;
        g_found.baud = 0;
        g_found.uart = false;
        finish(tft);
        return;
    }
    if (odd) g_score = 0;                       // learnt again like any bad reading, then not serial
    // A good line read badly once in a while: learning that starts in the
    // middle of a burst can take a wrong start bit and settle on a frame
    // that does not hold (60..76 % on clean RS232 at 19200 and 115200, about
    // one try in thirty, 2026-11-07). Learnt again, up to twice, before the
    // line is called not serial; a line that is not one fails each time.
    if (g_found.baud && g_score < 90 && g_relearns < 2) {
        g_relearns++;
        g_found.baud = 0;
        startUart(tft);
        return;
    }
    // Frames that hold only now and then are not a serial line: a data line
    // read as one finds 'frames' too (83..93 % on the demo I2C lines).
    if (g_found.baud && g_score < 90) {
        if (odd) strcpy(g_v.info, "not serial");
        else     snprintf(g_v.info, sizeof(g_v.info), "not serial: %u%% fit", g_score);
        g_found.baud = 0;
        g_found.uart = false;
        if (!g_v.note[0]) g_v.note = "I2C SDA, SPI, or a GPIO?";
    }
    finish(tft);
}

// Hands-off: one window of the line on a result, against the result. True
// if it is another line now: past an end of the converter; on a steady
// result a swing, or the level moved; on a moving one beyond its levels, or
// quiet away from where it rests.
static bool changed(bool& activity) {
    Acq::pause();
    uint16_t lo = g_mLo, hi = g_mHi;
    uint32_t sum = g_mSum, n = g_mN;
    g_mLo = 0xFFFF; g_mHi = 0; g_mSum = 0; g_mN = 0;
    Acq::resume();
    activity = false;
    if (!n) return false;
    if (lo <= 16 || hi >= 4064) return true;
    bool quiet = (int32_t)(hi - lo) < g_need;
    int32_t loMv = toMv(lo), hiMv = toMv(hi), meanMv = toMv((int32_t)(sum / n));
    // A serial result whose levels are all at mark (read from rest, bumps and
    // all: v0.22b) is taken as one read from a quiet line: its messages swing
    // far past those levels, and that is the line talking.
    bool restOnly = g_found.uart && g_f.hiMv < -3000;
    if (!g_f.active || restOnly) {
        // A serial line read from rest (its levels are those of the rest):
        // traffic on it is the line talking, not another line. Taken for
        // another (v0.19), every message of a line that talks now and then
        // started it all again, and the result was gone in a few tenths of a
        // second.
        if (!quiet) { activity = true; return !g_found.uart; }
        int32_t d = meanMv - g_f.meanMv, a = g_f.meanMv < 0 ? -g_f.meanMv : g_f.meanMv;
        int32_t tol = a / 10 > 300 ? a / 10 : 300;
        return d > tol || d < -tol;
    }
    int32_t span = g_f.hiMv - g_f.loMv, tol = span / 4 > 300 ? span / 4 : 300;
    if (loMv < g_f.loMv - tol || hiMv > g_f.hiMv + tol) return true;
    bool restHigh = g_found.uart ? !g_found.inverted : g_f.restHigh;
    int32_t rest = restHigh ? g_f.hiMv : g_f.loMv, d = meanMv - rest;
    if (quiet && (d > tol || d < -tol)) g_offRestMs += MON_MS;
    else                                g_offRestMs = 0;
    return g_offRestMs >= OFFREST_MS;
}

static void step(Lcd& tft) {
    uint32_t now = millis(), dt = now - g_t0;
    switch (g_phase) {
        case P_SETTLE:
            if (dt >= SETTLE_MS) startPhase(g_after);
            break;
        case P_WIDE:
            if (dt >= WIDE_MS && g_n) {
                Acq::pause();
                int32_t lo = toMv(g_lo), hi = toMv(g_hi);
                Acq::resume();
                int32_t a = lo < 0 ? -lo : lo, b = hi < 0 ? -hi : hi;
                uint8_t r = pickRange(a > b ? a : b);
                if (r != Frontend::range()) {
                    Acq::pause();
                    Frontend::setRange(r);
                    Acq::resume();
                    settleThen(P_LEVELS);
                } else {
                    startPhase(P_LEVELS);
                }
            }
            break;
        case P_LEVELS:
            if (dt >= LEVELS_MS && g_n) {
                Acq::pause();
                uint16_t lo = g_lo, hi = g_hi;
                uint32_t mean = (uint32_t)(g_sum / g_n);
                Acq::resume();
                // A swing worth looking at: 0.6 V (a CAN or RS485 wire on its
                // own swings about 1 V), and three steps of the 8 bits. The
                // bumps of a volt on a quiet RS232 line pass this; with no
                // detector taking them, classify() calls them steady.
                int32_t need = (int32_t)Frontend::countsPerVolt() * 6 / 10;
                if (need < 48) need = 48;
                g_need = need;
                // Hands-off, a quiet line is named at once and watched after
                // (finish()); a burst then starts it again in full.
                bool quick = Config::handsOff && !g_full && (int32_t)(hi - lo) < need;
                if (dt < WATCH_MS && !quick) break; // watched in full, for a line that talks in bursts
                g_f.range = Frontend::range();
                g_f.loMv = toMv(lo); g_f.hiMv = toMv(hi); g_f.meanMv = toMv((int32_t)mean);
                g_f.clipped = lo <= 16 || hi >= 4064;
                uint16_t span = (uint16_t)(hi - lo);
                g_thr  = (uint16_t)((lo + hi) / 2);
                g_hyst = (uint16_t)(span / 6 > 48 ? span / 6 : 48);
                g_f.active = span >= need;
                if (quick) { classify(); finishSteady(tft); break; }
                startPhase(P_EDGES);
            }
            break;
        case P_EDGES:
            if (dt >= EDGES_MS && g_n) {
                Acq::pause();
                uint32_t edges = g_edges, n = g_n, high = g_high, minRun = g_minRun;
                Acq::resume();
                if (g_f.active && edges < EDGES_ENOUGH && dt < WATCH_MS) break;
                g_f.edges = edges;
                g_f.edgesPerS = (uint32_t)((uint64_t)edges * MEASURE_HZ / n);
                g_f.highPct = (uint8_t)((uint64_t)high * 100 / n);
                g_f.restHigh = g_f.highPct > 50;
                g_f.minRunNs = minRun == 0xFFFFFFFFu ? 0 : (uint32_t)((uint64_t)minRun * 1000000000u / MEASURE_HZ);
                g_f.active = g_f.active && edges >= 4;
                Acq::pause();
                timing(g_f);
                Acq::resume();
                classify();
                if (g_v.uart) startUart(tft);
                else finishSteady(tft);
            }
            break;
        case P_UART: {
            Rs232::Format fm = Rs232::format();
            if (g_idleWait && !g_traffic) {
                Rs232::Stats ws = Rs232::stats();
                if (ws.bytes + ws.framingErrors + ws.parityErrors) {
                    g_traffic = true;           // the timing of this try counts from here
                    g_t0 = now;
                    dt = 0;
                } else if (now - g_waitT0 < IDLE_WAIT_MS) {
                    g_t0 = now;                 // nothing yet: keep waiting
                    break;
                } else {                        // a minute and nothing: as it was
                    g_idleWait = false;
                    g_found = Found();
                    classify();
                    finish(tft);
                    break;
                }
            }
            if (fm.decided && !g_decided) {
                g_decided = true;
                g_b0 = Rs232::stats().bytes;
                g_tDecided = now;
            }
            uint32_t frames = g_decided ? Rs232::stats().bytes - g_b0 : 0;
            if (g_v.lin) frames = g_linHeaders >= 20 ? UART_FRAMES : 0;    // LIN: twenty headers
            // the try at 9600 is kept short: a line at another speed should not wait on it
            uint32_t learnMs = g_at9600 ? 1500 : UART_LEARN_MS, moreMs = g_at9600 ? 1000 : UART_MORE_MS;
            if ((g_decided && (frames >= UART_FRAMES || now - g_tDecided >= moreMs))
                || (!g_decided && dt >= learnMs))
                finishUart(tft);
            break;
        }
        case P_DONE:
            if (!g_mon || g_teach) break;
            if (now - g_monMs >= MON_MS) {
                g_monMs = now;
                bool activity;
                if (changed(activity)) { begin(tft, activity); break; }
                if (framesChanged()) { begin(tft, true); break; }
            }
            // details offered where AUTO has some: the explanations, and the tester's
            // view of one character
            if (!g_hint && now - g_drawMs >= HINT_MS && (teachKind() || (g_found.uart && g_found.baud))) {
                g_hint = true;
                footer(tft);
            }
            break;
        default:
            break;
    }
}

// --- Modbus RTU, on a serial line ---------------------------------------------------
// A frame is found by its CRC-16 rather than by the silences around it,
// which the decoder's queue (read in bursts by the loop) and the demo line
// (made faster than real time) do not keep: for every place a frame could
// have started among the last 64 characters, the CRC from there on is kept
// up to date, and a frame is taken when one comes to 0 over a length that
// fits its function. A silence of 3.5 characters still starts afresh.
static uint8_t  g_mbBuf[64], g_mbN;
static uint16_t g_mbCrc[64];
static uint32_t g_mbLastMs;
static uint8_t  g_mb[64], g_mbLen;              // the last frame whose CRC held

// The frame to the page, when it changes or once a second.
static void mbSend() {
    static uint16_t sentHash;
    static uint32_t sentMs;
    uint16_t h = g_mbLen;
    for (uint8_t i = 0; i < g_mbLen; i++) h = (uint16_t)(h * 31 + g_mb[i]);
    if (h == sentHash && millis() - sentMs < 1000) return;
    sentHash = h; sentMs = millis();
    char buf[200], hex[2 * 64 + 1];
    uint8_t n = g_mbLen > 64 ? 64 : g_mbLen;
    for (uint8_t i = 0; i < n; i++) sprintf(hex + 2 * i, "%02x", g_mb[i]);
    hex[2 * n] = 0;
    Link::Json j(buf, sizeof(buf));
    j.str("t", "frame").str("proto", "modbus").str("bytes", hex).num("addr", g_mb[0]).num("fn", g_mb[1] & 0x7F);
    if (g_mb[1] & 0x80) j.num("exc", g_mb[2]);
    j.flag("crcOk", true);
    Link::send(j.done());
}

static uint16_t crcStep(uint16_t c, uint8_t b) {
    c ^= b;
    for (uint8_t k = 0; k < 8; k++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    return c;
}

// The length fits the function: a request of 8 for functions 1..6, 15, 16
// (their answers to 5, 6, 15, 16 too), an answer of 5 + its byte count for
// 1..4, a write of many of 9 + its byte count, an exception of 5.
static bool mbFits(const uint8_t* m, uint8_t n) {
    uint8_t a = m[0], f = m[1];
    if (a > 247) return false;
    if (f & 0x80) return n == 5 && a;
    if (!a && f != 5 && f != 6 && f != 15 && f != 16) return false;    // broadcast: writes only
    switch (f) {
        case 1: case 2: case 3: case 4: return n == 8 || (n >= 6 && m[2] == n - 5);
        case 5: case 6:                 return n == 8;
        case 15: case 16:               return n == 8 || (n >= 10 && m[6] == n - 9);
    }
    return false;
}

static uint32_t mbGapMs() {
    uint32_t baud = Rs232::stats().baud;
    uint32_t ms = baud ? 4u * 11u * 1000u / baud + 1 : 2;     // a little over 3.5 characters
    return ms < 2 ? 2 : ms;
}

static void mbByte(uint8_t c) {
    uint32_t now = millis();
    if (g_mbN && now - g_mbLastMs > mbGapMs()) g_mbN = 0;
    g_mbLastMs = now;
    if (g_mbN == sizeof(g_mbBuf)) {                             // the oldest start goes
        memmove(g_mbBuf, g_mbBuf + 1, g_mbN - 1);
        memmove(g_mbCrc, g_mbCrc + 1, (g_mbN - 1) * sizeof(g_mbCrc[0]));
        g_mbN--;
    }
    g_mbBuf[g_mbN] = c;
    g_mbCrc[g_mbN] = 0xFFFF;
    g_mbN++;
    for (uint8_t i = 0; i < g_mbN; i++) g_mbCrc[i] = crcStep(g_mbCrc[i], c);
    for (uint8_t i = 0; i + 4 <= g_mbN; i++) {
        if (g_mbCrc[i] == 0 && mbFits(g_mbBuf + i, (uint8_t)(g_mbN - i))) {   // over the CRC itself the CRC is 0
            g_mbLen = (uint8_t)(g_mbN - i);
            memcpy(g_mb, g_mbBuf + i, g_mbLen);
            g_mbOk = true;
            g_mbN = 0;
            mbSend();
            return;
        }
    }
}

static void mbIdle() {}

static void mbReset() { g_mbN = 0; g_mbLen = 0; g_mbOk = false; }

// --- the explanation ----------------------------------------------------------------
// One packet of what was found, drawn big and taken apart. For PWM, CAN and
// I2C SCL a block of samples is caught at a rate that suits it (TEACH_*); a
// Modbus frame comes from the decoder; RS232 and TTL go to the tester's own
// view of one character, or in the demo (which the tester does not read) a
// message is caught here and drawn with that view.
enum { T_NONE, T_PWM, T_CAN, T_SCL, T_MODBUS, T_UART };
static volatile bool g_capWant, g_capDone;
static uint32_t g_capMs, g_teachRate, g_resultRate;
static uint32_t g_spbQ8;                        // samples a bit (CAN) or a clock (SCL), x256

static uint8_t teachKind() {
    if (g_v.lin) return T_NONE;
    if (g_found.uart) return g_mbOk ? T_MODBUS : Config::demo && g_found.baud ? T_UART : T_NONE;
    if (g_v.name[0] == 'P' && g_v.name[1] == 'W') return T_PWM;
    if (g_v.name[0] == 'C' && g_v.name[1] == 'A') return T_CAN;
    if (g_v.name[0] == 'I' && g_v.name[4] == 'S') return T_SCL;
    return T_NONE;
}

// A block is taken when what it holds can be explained: for CAN a start of
// frame after a rest of eleven bits, in the first quarter of the block; for
// SCL the first clock after a rest, likewise; for a serial line the start of
// a message after a rest of twelve bits; for PWM any block. Called from
// block() while one is wanted.
T232_RAM static bool capFits(const uint16_t* s, uint16_t n) {
    if (g_teach == T_PWM) return true;
    const uint16_t up = (uint16_t)(g_thr + g_hyst), down = (uint16_t)(g_thr - g_hyst);
    // the rest level: high for SCL, and for CAN as the line was found
    bool restHigh = g_teach == T_SCL ? true : g_teach == T_UART ? !g_found.inverted : g_f.restHigh;
    uint32_t needRest = g_teach == T_CAN ? 11u * g_spbQ8 >> 8 : g_teach == T_UART ? 12u * g_spbQ8 >> 8 : 3u * g_spbQ8 >> 8;
    uint32_t rest = 0;
    for (uint16_t i = 0; i < n / 4; i++) {
        uint16_t v = s[i] & 0x0FFF;
        bool atRest = restHigh ? v > up : v < down;
        bool away   = restHigh ? v < down : v > up;
        if (atRest) rest++;
        else if (away) { if (rest >= needRest) return true; rest = 0; }
    }
    return false;
}

T232_RAM static void capTake(const uint16_t* s, uint16_t n) {
    if (!g_capWant || g_capDone || g_waveBusy || !capFits(s, n)) return;
    if (n > Acq::BUF_LEN / 2) n = Acq::BUF_LEN / 2;
    memcpy(g_wave, s, n * sizeof(uint16_t));
    g_waveN = n;
    g_waveHz = Acq::sampleRate();
    g_capDone = true;
}

// --- CAN: one frame out of the caught block --------------------------------------------
struct CanFrame {
    bool     ok;                // a whole frame was read
    const char* why;            // ... or why not
    uint16_t id; uint8_t rtr, ide, dlc; uint8_t data[8];
    uint16_t crcRx, crcCalc;
    uint8_t  stuff;             // stuff bits taken out
    bool     ack;
    // the bits as on the wire, stuff bits included: level and field
    uint8_t  nBits;
    uint8_t  bit[140], field[140];
    uint16_t sof;               // sample of the start of frame
};
enum { F_SOF, F_ID, F_CTRL, F_DLC, F_DATA, F_CRC, F_DEL, F_ACK, F_EOF, F_STUFF };
static CanFrame g_can;

static void canDecode() {
    CanFrame& c = g_can;
    memset(&c, 0, sizeof(c));
    c.why = "no start of frame";
    const uint16_t up = (uint16_t)(g_thr + g_hyst), down = (uint16_t)(g_thr - g_hyst);
    const bool restHigh = g_f.restHigh;
    const uint32_t spb = g_spbQ8;               // x256
    // the runs of each level from the first start of frame after a rest
    uint16_t i = 0, n = g_waveN;
    uint32_t rest = 0;
    for (; i < n; i++) {
        uint16_t v = g_wave[i] & 0x0FFF;
        bool atRest = restHigh ? v > up : v < down, away = restHigh ? v < down : v > up;
        if (atRest) rest++;
        else if (away) { if (rest >= (11u * spb >> 8)) break; rest = 0; }
    }
    if (i >= n) return;
    c.sof = i;
    // bits from the runs, each run a whole number of bits
    uint8_t raw[140], nRaw = 0;
    bool dom = true;                            // the line is at dominant from the start of frame
    uint16_t start = i;
    for (uint16_t k = i + 1; k <= n && nRaw < sizeof(raw); k++) {
        bool flip = false;
        if (k < n) {
            uint16_t v = g_wave[k] & 0x0FFF;
            bool atRest = restHigh ? v > up : v < down, away = restHigh ? v < down : v > up;
            flip = dom ? atRest : away;
        }
        if (!flip && k < n) continue;
        uint32_t len = k - start;
        uint32_t bits = (len * 256 + spb / 2) / spb;
        if (bits == 0) bits = 1;
        if (k == n && !dom && bits > 10) bits = 10;     // the rest after the frame, cut at the block's end
        for (uint32_t b = 0; b < bits && nRaw < sizeof(raw); b++) raw[nRaw++] = dom ? 0 : 1;
        if (!dom && bits >= 7 && nRaw > 40) break;      // end of frame: seven recessive
        dom = !dom;
        start = k;
    }
    // take the stuff bits out, field by field, up to the end of the CRC
    uint8_t bits[120], nb = 0, same = 0, last = 2;
    uint16_t need = 19;                         // SOF, ID, RTR, IDE, r0, DLC: until the DLC is known
    uint8_t r = 0;
    for (; r < nRaw && nb < need; r++) {
        if (same == 5) {                        // a stuff bit
            if (raw[r] == last) { c.why = "stuff error"; return; }
            c.bit[c.nBits] = raw[r]; c.field[c.nBits++] = F_STUFF;
            c.stuff++;
            same = 1; last = raw[r];
            continue;
        }
        bits[nb++] = raw[r];
        same = raw[r] == last ? (uint8_t)(same + 1) : 1;
        last = raw[r];
        uint8_t f = nb == 1 ? F_SOF : nb <= 12 ? F_ID : nb <= 15 ? F_CTRL : nb <= 19 ? F_DLC : 0;
        if (nb == 19) {
            uint8_t dlc = (uint8_t)(bits[15] << 3 | bits[16] << 2 | bits[17] << 1 | bits[18]);
            if (bits[13]) { c.why = "extended frame: not taken apart"; return; }
            uint8_t nd = bits[12] ? 0 : (dlc > 8 ? 8 : dlc);   // a remote frame has no data
            need = (uint16_t)(19 + 8 * nd + 15);
        }
        if (nb > 19) f = nb <= need - 15 ? F_DATA : F_CRC;
        c.bit[c.nBits] = raw[r]; c.field[c.nBits++] = f;
    }
    if (nb < need) { c.why = "frame cut short by the end of the block"; return; }
    // the rest, not stuffed: CRC delimiter, ACK, ACK delimiter, end of frame
    static const uint8_t TAIL[10] = { F_DEL, F_ACK, F_DEL, F_EOF, F_EOF, F_EOF, F_EOF, F_EOF, F_EOF, F_EOF };
    for (uint8_t t = 0; t < 10 && r < nRaw && c.nBits < sizeof(c.bit); t++, r++) {
        c.bit[c.nBits] = raw[r]; c.field[c.nBits++] = TAIL[t];
        if (t == 1) c.ack = raw[r] == 0;
    }
    // the fields
    for (uint8_t b = 1; b <= 11; b++) c.id = (uint16_t)(c.id << 1 | bits[b]);
    c.rtr = bits[12]; c.ide = bits[13];
    c.dlc = (uint8_t)(bits[15] << 3 | bits[16] << 2 | bits[17] << 1 | bits[18]);
    uint8_t nd = (uint8_t)((need - 34) / 8);
    for (uint8_t d = 0; d < nd; d++)
        for (uint8_t b = 0; b < 8; b++) c.data[d] = (uint8_t)(c.data[d] << 1 | bits[19 + d * 8 + b]);
    for (uint16_t b = need - 15; b < need; b++) c.crcRx = (uint16_t)(c.crcRx << 1 | bits[b]);
    uint16_t crc = 0;
    for (uint16_t b = 0; b < need - 15; b++) {
        uint8_t nxt = bits[b] ^ ((crc >> 14) & 1);
        crc = (uint16_t)((crc << 1) & 0x7FFF);
        if (nxt) crc ^= 0x4599;
    }
    c.crcCalc = crc;
    c.ok = true;
    c.why = "";
}

// --- the screens ----------------------------------------------------------------------
#define TEACH_Y   26
#define STRIP_Y   58
#define STRIP_H   60

static void teachBar(Lcd& tft, const char* what) {
    tft.fillScreen(TH.bg);
    bar(tft, "explained");
    tft.setTextSize(2);
    tft.setTextColor(TH.text, TH.bg);
    tft.setCursor(8, TEACH_Y);
    tft.print(what);
    tft.drawFastHLine(0, FOOT_Y - 6, 320, TH.line);
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, FOOT_Y);
    tft.print("AUTO back   RUN another   OK held menu");
}

static void line(Lcd& tft, uint8_t row, uint16_t col, const char* label) {
    tft.setTextSize(1);
    tft.setTextColor(col, TH.bg);
    tft.setCursor(8, (int16_t)(STRIP_Y + STRIP_H + 8 + row * 12));
    tft.print(label);
    tft.setTextColor(TH.text2, TH.bg);
}

// A stretch of the caught block, from sample a to b, across the screen in the
// strip, as lowest-to-highest columns; returns nothing, draws in `col`.
static void strip(Lcd& tft, uint16_t a, uint16_t b, uint16_t col) {
    tft.fillRect(0, STRIP_Y, 320, STRIP_H, TH.bg);
    if (b <= a + 1) return;
    uint16_t lo = 0x0FFF, hi = 0;
    for (uint16_t i = a; i < b; i++) { uint16_t v = g_wave[i] & 0x0FFF; if (v < lo) lo = v; if (v > hi) hi = v; }
    int32_t span = hi - lo; if (span < 64) span = 64;
    const int16_t top = STRIP_Y + 14, h = STRIP_H - 18;
    for (int16_t x = 0; x < 318; x++) {
        uint16_t s0 = (uint16_t)(a + (uint32_t)x * (b - a) / 318), s1 = (uint16_t)(a + (uint32_t)(x + 1) * (b - a) / 318);
        if (s1 <= s0) s1 = (uint16_t)(s0 + 1);
        uint16_t l = 0x0FFF, m = 0;
        for (uint16_t i = s0 > a ? s0 - 1 : s0; i < s1; i++) { uint16_t v = g_wave[i] & 0x0FFF; if (v < l) l = v; if (v > m) m = v; }   // joined to the column before
        int16_t y1 = (int16_t)(top + h - 1 - (int32_t)(m - lo) * (h - 1) / span);
        int16_t y2 = (int16_t)(top + h - 1 - (int32_t)(l - lo) * (h - 1) / span);
        tft.drawFastVLine((int16_t)(x + 1), y1, (int16_t)(y2 - y1 + 1), col);
    }
}

static void printUs(Lcd& tft, uint32_t ns) {
    if (ns >= 1000000) { tft.print(ns / 1000000); tft.print('.'); tft.print(ns / 100000 % 10); tft.print(" ms"); }
    else if (ns >= 10000) { tft.print(ns / 1000); tft.print(" us"); }
    else { tft.print(ns / 1000); tft.print('.'); tft.print(ns / 100 % 10); tft.print(" us"); }
}

static void drawPwm(Lcd& tft) {
    teachBar(tft, "PWM: two periods");
    // two periods from the first rising edge
    const uint16_t up = (uint16_t)(g_thr + g_hyst), down = (uint16_t)(g_thr - g_hyst);
    uint16_t n = g_waveN, e[6], ne = 0;
    bool high = (g_wave[0] & 0x0FFF) > g_thr;
    for (uint16_t i = 1; i < n && ne < 6; i++) {
        uint16_t v = g_wave[i] & 0x0FFF;
        if (!high && v > up)  { high = true;  e[ne++] = i; }
        else if (high && v < down) { high = false; if (ne) e[ne++] = i; }
    }
    if (ne < 5) { line(tft, 0, TH.err, "no two whole periods caught"); return; }
    // e[0] rise, e[1] fall, e[2] rise, e[3] fall, e[4] rise
    uint16_t a = e[0] > 8 ? (uint16_t)(e[0] - 8) : 0, b = (uint16_t)(e[4] + 8 < n ? e[4] + 8 : n);
    strip(tft, a, b, TH.trace);
    auto xOf = [&](uint16_t i) -> int16_t { return (int16_t)(1 + (uint32_t)(i - a) * 318 / (b - a)); };
    const uint32_t hz = g_waveHz;
    uint32_t tH = (uint32_t)((uint64_t)(e[1] - e[0]) * 1000000000u / hz);
    uint32_t tP = (uint32_t)((uint64_t)(e[2] - e[0]) * 1000000000u / hz);
    // the marks: high time over the first high, the period under the first period
    tft.drawFastHLine(xOf(e[0]), STRIP_Y + 6, (int16_t)(xOf(e[1]) - xOf(e[0])), TH.ok);
    tft.drawFastHLine(xOf(e[0]), STRIP_Y + STRIP_H - 2, (int16_t)(xOf(e[2]) - xOf(e[0])), TH.mark);
    line(tft, 0, TH.ok, "high   ");      printUs(tft, tH);
    line(tft, 1, TH.text2, "low    ");   printUs(tft, tP - tH);
    line(tft, 2, TH.mark, "period ");    printUs(tft, tP);
    tft.print("   frequency "); tft.print(1000000000u / tP); tft.print(" Hz");
    line(tft, 3, TH.text, "duty   ");    tft.print(tH * 100 / tP); tft.print('.'); tft.print(tH * 1000 / tP % 10);
    tft.print(" %: share of the period high");
    line(tft, 5, TH.muted, "Mean voltage = duty x high level: what a motor,");
    line(tft, 6, TH.muted, "a LED or an RC filter after the pin sees.");
}

static void drawScl(Lcd& tft) {
    teachBar(tft, "I2C SCL: one burst");
    const uint16_t up = (uint16_t)(g_thr + g_hyst), down = (uint16_t)(g_thr - g_hyst);
    uint16_t n = g_waveN, fall[40], nf = 0;
    bool high = true;
    uint16_t first = 0;
    for (uint16_t i = 1; i < n && nf < 40; i++) {
        uint16_t v = g_wave[i] & 0x0FFF;
        if (high && v < down) {
            if (nf && i - fall[nf - 1] > 3 * (g_spbQ8 >> 8)) break;   // the next burst
            if (!nf) first = i;
            fall[nf++] = i; high = false;
        } else if (!high && v > up) high = true;
    }
    if (nf < 9) { line(tft, 0, TH.err, "no burst of nine clocks caught"); return; }
    uint16_t a = first > 20 ? (uint16_t)(first - 20) : 0;
    uint16_t b = (uint16_t)(fall[nf - 1] + (g_spbQ8 >> 8) * 2 < n ? fall[nf - 1] + (g_spbQ8 >> 8) * 2 : n);
    strip(tft, a, b, TH.trace);
    // the clocks numbered 1..9 for each byte, the ninth (ACK) apart
    tft.setTextSize(1);
    for (uint16_t k = 0; k < nf; k++) {
        int16_t x = (int16_t)(1 + (uint32_t)(fall[k] + (g_spbQ8 >> 9) - a) * 318 / (b - a));
        uint8_t num = (uint8_t)(k % 9 + 1);
        tft.setTextColor(num == 9 ? TH.ok : ((k / 9) & 1 ? TH.mark : TH.text), TH.bg);
        tft.setCursor((int16_t)(x - 2), STRIP_Y + 2);
        tft.print(num);
    }
    uint32_t tc = (uint32_t)((uint64_t)g_spbQ8 * 1000000000u / 256u / g_waveHz);
    line(tft, 0, TH.text, "clock  ");  printUs(tft, tc); tft.print("  ("); tft.print(1000000000u / tc / 1000); tft.print(" kHz)");
    tft.print("   clocks "); tft.print(nf); tft.print(" = "); tft.print(nf / 9); tft.print(" bytes");
    line(tft, 1, TH.text2, "A byte: 8 clocks for its bits (MSB first), then");
    line(tft, 2, TH.ok,    "a 9th, ACK: the receiver pulls SDA low.");
    line(tft, 3, TH.text2, "SDA changes while SCL is low; START and STOP");
    line(tft, 4, TH.text2, "are SDA falling / rising while SCL is high.");
    line(tft, 5, TH.muted, "The bits are on SDA: one channel cannot read them.");
}

static void hexByte(Lcd& tft, uint8_t b) {
    tft.print((char)"0123456789ABCDEF"[b >> 4]); tft.print((char)"0123456789ABCDEF"[b & 15]);
}

static void drawCan(Lcd& tft) {
    canDecode();
    teachBar(tft, "CAN: one frame");
    const CanFrame& c = g_can;
    if (!c.ok) { line(tft, 0, TH.err, c.why); return; }
    // the bits as on the wire, each a cell, dominant drawn high, coloured by field
    const uint16_t COL[] = { TH.ok, TH.text, TH.text2, TH.mark, TH.trace, TH.ok2, TH.muted, TH.ok, TH.muted, TH.err };
    tft.fillRect(0, STRIP_Y, 320, STRIP_H, TH.bg);
    uint8_t nb = c.nBits;
    int16_t w = (int16_t)(316 / nb); if (w < 2) w = 2;
    int16_t x0 = (int16_t)((320 - w * nb) / 2);
    const int16_t yHi = STRIP_Y + 12, yLo = STRIP_Y + 40;
    for (uint8_t k = 0; k < nb; k++) {
        uint16_t col = COL[c.field[k]];
        int16_t x = (int16_t)(x0 + k * w), y = c.bit[k] ? yLo : yHi;     // dominant (0) up
        tft.drawFastHLine(x, y, w, col);
        if (k && c.bit[k] != c.bit[k - 1]) tft.drawFastVLine(x, yHi, yLo - yHi + 1, col);
        if (c.field[k] == F_STUFF) tft.fillRect(x, STRIP_Y + 46, w, 4, TH.err);
    }
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(2, STRIP_Y + 1); tft.print("dominant (0)");
    tft.setCursor(2, STRIP_Y + 50); tft.print("recessive (1)");
    tft.setTextColor(TH.err, TH.bg); tft.setCursor(230, STRIP_Y + 50); tft.print("stuff bits");

    char t[48];
    line(tft, 0, TH.ok, "SOF   ");      tft.print("start of frame, one dominant bit");
    line(tft, 1, TH.text, "ID    ");    snprintf(t, sizeof(t), "0x%03X (%u)  11 bits, the lower wins the bus", c.id, c.id); tft.print(t);
    line(tft, 2, TH.text2, "RTR IDE r0 "); tft.print(c.rtr); tft.print(' '); tft.print(c.ide); tft.print("  data frame, standard ID");
    line(tft, 3, TH.mark, "DLC   ");    tft.print(c.dlc); tft.print(" data bytes");
    line(tft, 4, TH.trace, "DATA  ");
    for (uint8_t d = 0; d < c.dlc && d < 8; d++) { hexByte(tft, c.data[d]); tft.print(' '); }
    line(tft, 5, TH.ok2, "CRC   ");     snprintf(t, sizeof(t), "0x%04X  ", c.crcRx); tft.print(t);
    tft.setTextColor(c.crcRx == c.crcCalc ? TH.ok : TH.err, TH.bg);
    tft.print(c.crcRx == c.crcCalc ? "matches (CRC-15)" : "DOES NOT MATCH");
    line(tft, 6, TH.ok, "ACK   ");      tft.print(c.ack ? "given by a receiver" : "none: no other node took it");
    line(tft, 7, TH.err, "stuff ");     tft.print(c.stuff); tft.print(": one of the other level after 5 alike");
}

static void drawModbus(Lcd& tft) {
    teachBar(tft, "Modbus RTU: one frame");
    const uint8_t* m = g_mb;
    uint8_t n = g_mbLen;
    // the bytes, coloured by field
    tft.fillRect(0, STRIP_Y, 320, STRIP_H, TH.bg);
    tft.setTextSize(2);
    tft.setCursor(8, STRIP_Y + 10);
    bool req = n == 8 && m[1] >= 1 && m[1] <= 6;
    for (uint8_t i = 0; i < n && i < 16; i++) {
        uint16_t col = i == 0 ? TH.ok : i == 1 ? TH.mark : i >= n - 2 ? TH.ok2 : TH.trace;
        tft.setTextColor(col, TH.bg);
        if (i == 8) tft.setCursor(8, STRIP_Y + 32);
        hexByte(tft, m[i]); tft.print(' ');
    }
    static const char* const FN[] = { "?", "Read Coils", "Read Discrete Inputs", "Read Holding Registers",
                                      "Read Input Registers", "Write Single Coil", "Write Single Register" };
    uint8_t f = m[1] & 0x7F;
    line(tft, 0, TH.ok, "address  "); tft.print(m[0]); tft.print(m[0] ? "  the slave asked, or answering" : "  broadcast");
    line(tft, 1, TH.mark, "function "); hexByte(tft, m[1]); tft.print("  ");
    if (m[1] & 0x80) tft.print("exception (error answer)");
    else if (f <= 6) tft.print(FN[f]);
    else if (f == 15) tft.print("Write Multiple Coils");
    else if (f == 16) tft.print("Write Multiple Registers");
    line(tft, 2, TH.trace, "data     ");
    if (req) {
        uint16_t a = (uint16_t)(m[2] << 8 | m[3]), v = (uint16_t)(m[4] << 8 | m[5]);
        tft.print(f <= 4 ? "from " : "at "); tft.print(a);
        tft.print(f <= 4 ? ", count " : ", value "); tft.print(v);
        tft.print("  (a request)");
    } else if (n >= 5 && f <= 4 && m[2] == n - 5) {
        tft.print(m[2]); tft.print(" bytes of values (an answer)");
    } else {
        tft.print(n - 4); tft.print(" bytes");
    }
    char t[40];
    line(tft, 3, TH.ok2, "CRC-16   "); snprintf(t, sizeof(t), "%02X %02X  ", m[n - 2], m[n - 1]); tft.print(t);
    tft.setTextColor(TH.ok, TH.bg); tft.print("matches, low byte first");
    line(tft, 5, TH.muted, "A frame: the bytes between silences of 3.5");
    line(tft, 6, TH.muted, "characters. The master asks, that slave answers.");
}

// The tester's view of one character, on the caught message; its own key
// line replaced by this one's.
static void drawUart(Lcd& tft) {
    Rs232::levelsFrom(g_wave, g_waveN);
    uint16_t n = Rs232::review(g_wave, g_waveN, 8);
    Teach::draw(tft, g_wave, n, true);
    tft.fillRect(0, FOOT_Y - 6, 320, 240 - (FOOT_Y - 6), TH.bg);
    tft.drawFastHLine(0, FOOT_Y - 6, 320, TH.line);
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, FOOT_Y);
    tft.print("AUTO back  RUN another  ");
    tft.print((char)0x11); tft.print((char)0x10);
    tft.print(" character  OK held menu");
}

static void drawTeach(Lcd& tft) {
    g_waveBusy = true;
    switch (g_teach) {
        case T_PWM:    drawPwm(tft); break;
        case T_CAN:    drawCan(tft); break;
        case T_SCL:    drawScl(tft); break;
        case T_MODBUS: drawModbus(tft); break;
        case T_UART:   Teach::select(0); drawUart(tft); break;
    }
    g_waveBusy = false;
}

// Into the explanation: a rate for the catch, and the catch asked for.
static void teachStart(Lcd& tft) {
    g_resultRate = Acq::sampleRate();
    if (g_teach == T_MODBUS) { drawTeach(tft); return; }
    uint32_t rate = g_teach == T_UART ? g_resultRate : MEASURE_HZ;    // a serial line: the rate it was read at
    if (g_teach == T_PWM && g_f.freqHz) {
        rate = g_f.freqHz * 400;                // two and a half periods in a block
        if (rate > MEASURE_HZ) rate = MEASURE_HZ;
        if (rate < 1000) rate = 1000;
    }
    if (g_teach == T_CAN) {
        uint32_t br = g_f.minRunNs ? 1000000000u / g_f.minRunNs : 250000;
        static const uint32_t STD[] = { 125000, 250000, 500000, 1000000 };
        uint32_t std = snapTo(br, STD, 4);
        if (std) br = std;
        rate = br * 8 > MEASURE_HZ ? MEASURE_HZ : br * 8;   // eight samples a bit, as far as 2 MS/s goes
    }
    Acq::pause();
    g_phase = P_SETTLE;
    Acq::setSampleRate(rate);
    Acq::discard();
    g_teachRate = Acq::sampleRate();
    if (g_teach == T_CAN) {
        uint32_t br = g_f.minRunNs ? 1000000000u / g_f.minRunNs : 250000;
        static const uint32_t STD[] = { 125000, 250000, 500000, 1000000 };
        uint32_t std = snapTo(br, STD, 4);
        if (std) br = std;
        g_spbQ8 = (uint32_t)((uint64_t)g_teachRate * 256 / br);
    } else if (g_teach == T_UART) {
        g_spbQ8 = (uint32_t)((uint64_t)g_teachRate * 256 / g_found.baud);
    } else if (g_teach == T_SCL) {
        g_spbQ8 = (uint32_t)((uint64_t)g_teachRate * 256 / (g_f.clockHz ? g_f.clockHz : 100000));
    }
    g_capDone = false;
    g_capWant = true;
    g_capMs = millis();
    g_phase = P_TEACH;
    Acq::resume();
    teachBar(tft, "catching one...");
}

static void teachLeave(Lcd& tft) {
    g_capWant = false;
    Acq::pause();
    Acq::setSampleRate(g_resultRate);
    Acq::discard();
    g_phase = P_DONE;
    g_waveN = 0; g_waveSpan = 0;
    Acq::resume();
    g_teach = T_NONE;
    drawResult(tft);
}

// What the decoder read, as text: what does not print is a dot.
static void takeRx(Lcd& tft) {
    int c;
    bool any = false;
    while ((c = Rs232::read()) >= 0) {
        if (g_v.lin) { linByte(c); any = true; continue; }        // its frames, not text
        if (c > 0xFF) continue;
        if (g_rxN == sizeof(g_rx) - 1) { memmove(g_rx, g_rx + 1, g_rxN - 1); g_rxN--; }
        g_rx[g_rxN++] = (char)((c >= 32 && c < 127) ? c : '.');
        if (g_txtN == sizeof(g_txt)) { memmove(g_txt, g_txt + 1, sizeof(g_txt) - 1); g_txtN--; }
        g_txt[g_txtN++] = (char)c;
        g_txtNew = true;
        linesAdd((char)c);
        mbByte((uint8_t)c);
        if (g_phase == P_DONE) Link::rx((uint8_t)c, "insp");      // once the result is up: before, the frame may be guessed wrong
        any = true;
    }
    if (!g_v.lin) g_rx[g_rxN] = 0;
    mbIdle();
    if (any && g_phase == P_DONE && !g_teach && millis() - g_drawMs >= 300) { drawRx(tft); g_drawMs = millis(); }
    static uint32_t charsMs;
    if (g_txtNew && g_phase == P_DONE && !g_teach && showChars() && millis() - charsMs >= 200) {
        charsMs = millis();
        drawChars(tft);
    }
    if (g_linesNew && g_phase == P_DONE && !g_teach && showLines() && millis() - charsMs >= 200) {
        charsMs = millis();
        drawLines(tft, false);
    }
}

void again(Lcd& tft) {
    if (!g_teach) begin(tft);
}

uint8_t poll(Lcd& tft) {
    if (!Acq::interruptDriven() && g_src) {
        uint32_t t = millis();
        uint16_t n;
        bool over;
        const uint16_t* s;
        while ((millis() - t) < 20 && (s = g_src(n, over)) != nullptr) block(s, n, over);
    }
    step(tft);
    if (g_phase == P_UART || g_phase == P_DONE) takeRx(tft);
    static uint32_t waveMs;
    if (g_phase == P_DONE && !g_teach && g_waveNew && millis() - waveMs >= 500) { drawWave(tft); waveMs = millis(); }

    // the explanation: drawn once caught, or said that nothing came
    static bool drawn;
    if (g_phase == P_TEACH && !drawn) {
        if (g_capDone) { drawTeach(tft); drawn = true; }
        else if (millis() - g_capMs > 3000) {
            teachBar(tft, "nothing caught in 3 s");
            drawn = true;
        }
    }
    if (Keys::hit(Keys::AUTO)) {
        if (g_teach) teachLeave(tft);
        else if (g_phase == P_DONE) {
            uint8_t k = teachKind();
            if (k == T_NONE && g_found.uart && g_found.baud && !Config::demo) return TESTER_TEACH;    // RS232, TTL: the tester's view of one character
            if (k != T_NONE) { g_teach = k; drawn = false; teachStart(tft); }
        }
    }
    if (Keys::hit(Keys::RUN)) {
        if (g_teach == T_MODBUS) drawTeach(tft);            // the latest frame
        else if (g_teach) { g_capDone = false; g_capMs = millis(); drawn = false; teachBar(tft, "catching one..."); }
        else { linesClear(); begin(tft); }
    }
    int8_t d = Config::demo ? (Keys::hit(Keys::RIGHT) ? 1 : Keys::hit(Keys::LEFT) ? -1 : 0) : 0;
    if (d && g_teach == T_UART) {                       // another character of the message
        if (g_capDone && drawn && Teach::step(d)) { g_waveBusy = true; drawUart(tft); g_waveBusy = false; }
    } else if (d) {
        DemoSig::set((uint8_t)((DemoSig::which() + DemoSig::COUNT + d) % DemoSig::COUNT));
        begin(tft);
    }
    if (Keys::hit(Keys::MODE) && g_phase == P_DONE && !g_teach && readUart()) {
        g_view = (uint8_t)((g_view + 1) % NVIEW);         // boxes, picture, lines
        if (g_view == VIEW_BOXES)      drawChars(tft);
        else if (g_view == VIEW_LINES) drawLines(tft, true);
        else                           drawWave(tft);
    }
    if (Keys::hit(Keys::DOWN) && g_phase == P_DONE && !g_teach && healthy()) return TESTER_HEALTH;
    uint8_t ok = Keys::press(Keys::OK);
    if (ok == Keys::LONG) return MENU;
    // OK: the tester, whatever is on the screen (v0.19: a line that talks now
    // and then is seldom on a serial result when a hand is free). With nothing found, on AUTO, at the fastest rate (the
    // tester comes down by itself), the way up taken from where it rests.
    if (ok == Keys::SHORT && !g_teach && !Config::demo) {
        if (!readUart()) {
            g_found.uart = false;
            g_found.baud = 0;
            g_found.inverted = g_v.uart ? g_v.inverted : g_f.meanMv < 0;
            g_found.rateHz = UART_RATES[sizeof(UART_RATES) / sizeof(UART_RATES[0]) - 1];
        }
        return TESTER;
    }
    return STAY;
}

}
