#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "Link.h"
#include "UsbFix.h"
#include "Version.h"
#include "Rs232.h"
#include "Frontend.h"
#include "Theme.h"
#include "Lcd.h"
#ifdef USBCON
extern "C" {
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUSBD_Device_CDC;
}
#endif

namespace Link {

// --- the queue to the PC ----------------------------------------------------
// Room for a few waveforms (some 1.4 kB each) besides the text.
static const uint16_t Q_SIZE = 8192;
static char     g_q[Q_SIZE];
static uint16_t g_head, g_tail;         // bytes in g_q from g_tail to g_head
static bool     g_wasOpen;
static uint32_t g_n;                    // the next message's number
static uint32_t g_lost;                 // messages that found the queue full
static char     g_last[400];            // the last result, sent again on opening
static char     g_state[200];           // the last state, likewise

static uint16_t used() { return (uint16_t)((g_head - g_tail) & (Q_SIZE - 1)); }

static void putc_(char c) { g_q[g_head] = c; g_head = (g_head + 1) & (Q_SIZE - 1); }

// A whole message or nothing, so that a full queue never leaves half a line;
// its number goes in first: {"n":12,...
static void put(const char* s) {
    if (s[0] != '{') return;
    char num[16];
    uint8_t k = (uint8_t)sprintf(num, "{\"n\":%lu,", (unsigned long)g_n);
    uint16_t n = (uint16_t)strlen(s + 1);
    if (k + n + 1 >= Q_SIZE - used()) { g_lost++; return; }
    for (uint8_t i = 0; i < k; i++) putc_(num[i]);
    for (uint16_t i = 0; i < n; i++) putc_(s[1 + i]);
    putc_('\n');
    g_n++;
}

static bool portOpen() {
#ifdef USBCON
    return usbOpen();
#else
    return false;
#endif
}

// Whether the core has nothing in flight: one packet at a time, as the USB
// Sender learnt (TtlApp.cpp), single packets of up to 63 bytes.
static bool idle() {
#ifdef USBCON
    USBD_CDC_HandleTypeDef* h = (USBD_CDC_HandleTypeDef*)hUSBD_Device_CDC.pClassData;
    return !h || (h->TxState == 0 && CDC_TransmitQueue_ReadSize(&TransmitQueue) == 0);
#else
    return true;
#endif
}

// As many packets as go out within 2 ms: one a turn of the main loop was
// too slow for the waveforms.
static void sendSome() {
#ifdef USBCON
    uint32_t t0 = micros();
    while (used() && micros() - t0 < 2000) {
        if (!idle()) continue;
        uint8_t pk[63];
        uint16_t n = 0;
        while (n < sizeof(pk) && g_tail != g_head) { pk[n++] = (uint8_t)g_q[g_tail]; g_tail = (g_tail + 1) & (Q_SIZE - 1); }
        SerialUSB.write(pk, n);
    }
#endif
}

static void esc(char*& p, char* end, const char* s) {
    for (int i = 0; s[i] && p < end - 7; i++) {
        uint8_t c = (uint8_t)s[i];
        if (c == '"' || c == '\\') { *p++ = '\\'; *p++ = (char)c; }
        else if (c >= 32 && c < 127) *p++ = (char)c;
        else { static const char H[] = "0123456789abcdef"; *p++ = '\\'; *p++ = 'u'; *p++ = '0'; *p++ = '0'; *p++ = H[c >> 4]; *p++ = H[c & 15]; }
    }
}

// --- the bytes as they are: the only text ---------------------------------------
// Lines are the page's work (CR, LF, 200 ms of quiet): bytes go in batches of
// up to 48, or after 100 ms.
static uint8_t  g_raw[48];
static uint8_t  g_rawN;
static uint32_t g_rawMs;
static const char* g_rawSrc = "tester";

static void endRaw() {
    if (!g_rawN) return;
    static const char H[] = "0123456789abcdef";
    char out[48 * 2 + 64];
    char* p = out;
    p += sprintf(p, "{\"t\":\"bytes\",\"src\":\"%s\",\"h\":\"", g_rawSrc);
    for (uint8_t i = 0; i < g_rawN; i++) { *p++ = H[g_raw[i] >> 4]; *p++ = H[g_raw[i] & 15]; }
    strcpy(p, "\"}");
    if (portOpen()) put(out);
    g_rawN = 0;
}

void rx(uint8_t c, const char* src) {
    if (src != g_rawSrc) { endRaw(); g_rawSrc = src; }
    if (!g_rawN) g_rawMs = millis();
    g_raw[g_rawN++] = c;
    if (g_rawN == sizeof(g_raw)) endRaw();
}

void send(const char* json) {
    if (portOpen()) put(json);
}

void result(const char* json) {
    strncpy(g_last, json, sizeof(g_last) - 1);
    g_last[sizeof(g_last) - 1] = 0;
    if (portOpen()) put(g_last);
}

void sendResult() {
    if (portOpen() && g_last[0]) put(g_last);
}

// --- the theme: the page wears the instrument's colours ---------------------------
// Sent on opening and whenever another theme is chosen; the colours as #rrggbb.
static uint8_t g_themeSent = 0xFF;

static void sendTheme() {
    static const char* const KEY[] = { "bg", "text", "text2", "muted", "line", "bar", "barText", "barHi",
                                       "sel", "selText", "trace", "mark", "ok", "ok2", "err" };
    const ColorTheme& t = Theme::LIST[Theme::idx];
    const uint16_t c[] = { t.bg, t.text, t.text2, t.muted, t.line, t.bar, t.barText, t.barHi,
                           t.sel, t.selText, t.trace, t.mark, t.ok, t.ok2, t.err };
    char out[400];
    char* p = out + sprintf(out, "{\"t\":\"theme\",\"name\":\"%s\"", t.name);
    for (uint8_t i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
        uint16_t v = c[i];
        unsigned r = (v >> 11) * 255 / 31, g = ((v >> 5) & 63) * 255 / 63, b = (v & 31) * 255 / 31;
        p += sprintf(p, ",\"%s\":\"#%02x%02x%02x\"", KEY[i], r, g, b);
    }
    strcpy(p, "}");
    put(out);
    g_themeSent = Theme::idx;
}

void state(const char* json) {
    if (!strcmp(json, g_state)) return;
    strncpy(g_state, json, sizeof(g_state) - 1);
    g_state[sizeof(g_state) - 1] = 0;
    if (portOpen()) put(g_state);
}

// --- waveforms ---------------------------------------------------------------
// What is sent the moment DTR comes up was lost now and then: the PC side is
// not reading yet. So the greeting waits a little.
static const uint32_t OPEN_WAIT_MS = 150;
static uint32_t g_openMs;
static char     g_msg[3072];
static uint32_t g_liveMs;

static char* b64(char* p, const uint16_t* s, uint16_t n) {
    static const char B[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (uint16_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)(s[i] >> 4) << 16;
        if (i + 1 < n) v |= (uint32_t)(s[i + 1] >> 4) << 8;
        if (i + 2 < n) v |= (uint32_t)(s[i + 2] >> 4);
        *p++ = B[(v >> 18) & 63];
        *p++ = B[(v >> 12) & 63];
        *p++ = i + 1 < n ? B[(v >> 6) & 63] : '=';
        *p++ = i + 2 < n ? B[v & 63] : '=';
    }
    return p;
}

bool wantsWave() { return portOpen() && !g_openMs && millis() - g_liveMs >= 500; }

// Sent as 8-bit samples: zero, perVolt and th in their units (the counts / 16).
void wave(const uint16_t* s, uint16_t n, uint32_t hz, const char* src, const char* kind, bool marks) {
    if (!portOpen() || !n) return;
    bool live = !strcmp(kind, "live");
    if (live && millis() - g_liveMs < 500) return;
    if (n > 1536) n = 1536;                             // 2 kB of base64 at most
    uint16_t need = (uint16_t)(n / 3 * 4 + 8 + 400 + (marks ? Rs232::markCount() * 6 + Rs232::byteCount() * 12 : 0));
    if (need >= sizeof(g_msg) || need + (live ? 2048 : 0) >= Q_SIZE - used()) { if (!live) g_lost++; return; }
    if (live) g_liveMs = millis();
    Rs232::Stats st = Rs232::stats();
    Rs232::Format f = Rs232::format();
    uint32_t pv = (uint32_t)Frontend::countsPerVolt() * 1000u / 16u;   // per volt, in thousandths
    char* p = g_msg;
    p += sprintf(p, "{\"t\":\"wave\",\"src\":\"%s\",\"kind\":\"%s\",\"hz\":%lu,\"cnt\":%u,\"zero\":%u,"
                    "\"perVolt\":%lu.%03lu,\"range\":%u,\"inv\":%s,\"th\":%u,\"baud\":%lu,\"fmt\":\"%u%c%u\",\"d\":\"",
                 src, kind, (unsigned long)hz, n, (Frontend::zeroCounts() + 8) / 16,
                 (unsigned long)(pv / 1000), (unsigned long)(pv % 1000),
                 Frontend::range(), Rs232::inverted() ? "true" : "false", (st.threshold + 8) / 16,
                 (unsigned long)st.baud, f.dataBits, f.parity, f.stopBits);
    p = b64(p, s, n);
    *p++ = '"';
    if (marks) {
        p += strlen(strcpy(p, ",\"marks\":["));
        for (uint8_t i = 0; i < Rs232::markCount(); i++) {
            if (Rs232::markAt(i) >= n) break;
            p += sprintf(p, i ? ",%u" : "%u", Rs232::markAt(i));
        }
        p += strlen(strcpy(p, "],\"chars\":["));
        for (uint8_t i = 0; i < Rs232::byteCount(); i++) {
            if (Rs232::byteAt(i) >= n) break;
            p += sprintf(p, i ? ",[%u,%u]" : "[%u,%u]", Rs232::byteAt(i), Rs232::byteVal(i));
        }
        *p++ = ']';
    }
    strcpy(p, "}");
    put(g_msg);
}

// --- a picture of the screen ------------------------------------------------------
// Asked for by the page ("screen"): sent a row at a time, as room comes free
// in the queue, each row as runs of one colour: a byte of length - 1, then the
// colour, RGB565 high byte first; in base64. Read a row at a time as it went,
// the picture came torn where the screen was redrawn meanwhile (AprecID's
// waveform, twice a second). So from flash the whole screen is read at once
// (some 30 ms) into the RAM above the 64 kB the build knows of: the AT32 has
// 224 kB with EOPB0 0xFE, which FNIRSI's start-up sees to before the tester
// runs. From RAM (an image over SWD sits there) rows are still read as they go.
static int16_t g_scrY = -1;             // the next row to send, or -1
#ifndef RAM_RUN
static uint16_t* const FRAME = (uint16_t*)0x20010000u;         // 320 x 240, to 0x20035800

static void grabScreen() {
    for (int16_t y = 0; y < 240; y++) Lcd::readRow(y, FRAME + y * 320);
}
#endif

static char* b64bytes(char* p, const uint8_t* s, uint16_t n) {
    static const char B[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (uint16_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)s[i] << 16;
        if (i + 1 < n) v |= (uint32_t)s[i + 1] << 8;
        if (i + 2 < n) v |= s[i + 2];
        *p++ = B[(v >> 18) & 63];
        *p++ = B[(v >> 12) & 63];
        *p++ = i + 1 < n ? B[(v >> 6) & 63] : '=';
        *p++ = i + 2 < n ? B[v & 63] : '=';
    }
    return p;
}

static void sendScreen() {
    static uint16_t row[320];
    static uint8_t  runs[320 * 3];
    for (uint8_t k = 0; k < 8 && g_scrY >= 0; k++) {             // a few rows a turn
        if (Q_SIZE - used() < 1500) return;                       // a row is at most ~1.3 kB
#ifdef RAM_RUN
        Lcd::readRow(g_scrY, row);
#else
        memcpy(row, FRAME + g_scrY * 320, sizeof(row));
#endif
        uint16_t n = 0;
        for (uint16_t x = 0; x < 320; ) {
            uint16_t c = row[x], len = 1;
            while (x + len < 320 && row[x + len] == c && len < 256) len++;
            runs[n++] = (uint8_t)(len - 1); runs[n++] = c >> 8; runs[n++] = c & 0xFF;
            x += len;
        }
        char* p = g_msg + sprintf(g_msg, "{\"t\":\"screen\",\"y\":%d,\"w\":320,\"h\":240,\"d\":\"", g_scrY);
        p = b64bytes(p, runs, n);
        strcpy(p, "\"}");
        put(g_msg);
        g_scrY = g_scrY == 239 ? -1 : g_scrY + 1;
    }
}

// --- commands -------------------------------------------------------------------
static Handler  g_handler;
static char     g_cmd[200];
static uint8_t  g_cmdN;

void onCommand(Handler h) { g_handler = h; }

// The value after "key": in a flat JSON object.
static const char* find(const char* line, const char* key) {
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* v = strstr(line, pat);
    if (!v) return nullptr;
    v += strlen(pat);
    while (*v == ' ') v++;
    return v;
}

bool field(const char* line, const char* key, char* out, uint8_t size) {
    const char* v = find(line, key);
    if (!v || *v != '"') return false;
    uint8_t k = 0;
    for (v++; *v && *v != '"' && k + 1 < size; v++) out[k++] = *v;
    out[k] = 0;
    return true;
}

int32_t fieldNum(const char* line, const char* key, int32_t missing) {
    const char* v = find(line, key);
    return v && (*v == '-' || (*v >= '0' && *v <= '9')) ? (int32_t)strtol(v, nullptr, 10) : missing;
}

static void command(const char* line) {
    char what[16], t[8];
    if (!field(line, "t", t, sizeof(t)) || strcmp(t, "cmd")) return;
    int32_t id = fieldNum(line, "id", 0);
    bool ok = field(line, "do", what, sizeof(what));
    if (ok && !strcmp(what, "screen")) {                        // any app: the link's own
#ifndef RAM_RUN
        grabScreen();
#endif
        g_scrY = 0;
    }
    else ok = ok && g_handler && g_handler(what, line);
    char out[64];
    sprintf(out, "{\"t\":\"ack\",\"id\":%ld,\"ok\":%s%s}", (long)id, ok ? "true" : "false", ok ? "" : ",\"err\":\"unknown\"");
    put(out);
}

static void readCommands() {
#ifdef USBCON
    int c;
    while ((c = usbRead()) >= 0) {
        if (c == '\n' || c == '\r') {
            if (g_cmdN) { g_cmd[g_cmdN] = 0; command(g_cmd); }
            g_cmdN = 0;
        } else if (g_cmdN < sizeof(g_cmd) - 1) {
            g_cmd[g_cmdN++] = (char)c;
        }
    }
#endif
}

// --- the turn of the loop ---------------------------------------------------------
static uint32_t g_tickMs;

void poll() {
    if (g_rawN && millis() - g_rawMs >= 100) endRaw();
    bool open = portOpen();
    if (open && !g_wasOpen) {                   // a page has just opened the port
        g_head = g_tail = 0;
        g_rawN = 0;
        g_cmdN = 0;
        g_openMs = millis() ? millis() : 1;
    }
    if (g_openMs && millis() - g_openMs >= OPEN_WAIT_MS) {
        g_openMs = 0;
        g_n = 0;
        g_lost = 0;
        put("{\"t\":\"hello\",\"proto\":1,\"dev\":\"DSO-510\",\"fw\":\"" FW_NAME " " FW_VERSION "\",\"bits\":8,"
            "\"caps\":[\"bytes\",\"insp\",\"frame\",\"wave\",\"state\",\"cmd\",\"theme\",\"app\",\"screen\"]}");
        if (g_last[0]) put(g_last);
        if (g_state[0]) put(g_state);
        sendTheme();
        g_tickMs = millis();
    }
    if (open && !g_openMs) {
        readCommands();
        if (Theme::idx != g_themeSent) sendTheme();
        if (millis() - g_tickMs >= 2000) {
            g_tickMs = millis();
            char out[64];
            sprintf(out, "{\"t\":\"tick\",\"ms\":%lu,\"lost\":%lu}", (unsigned long)millis(), (unsigned long)g_lost);
            put(out);
        }
    }
    if (open && !g_openMs && g_scrY >= 0) sendScreen();
    if (!open) { g_head = g_tail = 0; g_openMs = 0; g_scrY = -1; }
    g_wasOpen = open;
    if (open && !g_openMs) sendSome();
}

void flush(uint32_t ms) {
    uint32_t t0 = millis();
    while (used() && portOpen() && millis() - t0 < ms) { IWDG->KR = 0xAAAA; sendSome(); }
    g_head = g_tail = 0;
}

// --- the JSON writer ----------------------------------------------------------
Json::Json(char* buf, uint16_t size) : start(buf), p(buf), end(buf + size - 2), first(true) { *p++ = '{'; }

// The comma, the key and the colon; false if there is no room left.
static bool key(Json& j, const char* k) {
    if (j.p >= j.end - 24) return false;
    if (!j.first) *j.p++ = ',';
    j.first = false;
    *j.p++ = '"';
    esc(j.p, j.end, k);
    *j.p++ = '"'; *j.p++ = ':';
    return true;
}

Json& Json::str(const char* k, const char* v) {
    if (key(*this, k)) { *p++ = '"'; esc(p, end, v); if (p < end) *p++ = '"'; }
    return *this;
}

Json& Json::num(const char* k, int32_t v) {
    if (key(*this, k)) p += sprintf(p, "%ld", (long)v);
    return *this;
}

Json& Json::flag(const char* k, bool v) {
    if (key(*this, k)) p += strlen(strcpy(p, v ? "true" : "false"));
    return *this;
}

const char* Json::done() {
    *p++ = '}';                         // end leaves room for these two
    *p = 0;
    return start;
}

}
