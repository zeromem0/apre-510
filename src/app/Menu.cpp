#include <Arduino.h>
#include "Menu.h"
#include "Theme.h"
#include "Keys.h"
#include "Version.h"
#include "Probe.h"

namespace Menu {

int8_t image = 0;

#ifdef NO_USB_SENDER
static const char* const ITEM[] = { "AprecID", "RS232 Tester", "Settings" };
static const int8_t NITEM = 3;
#define ITEM_Y     53           // the three in the middle of where four stand
#else
static const char* const ITEM[] = { "RS232 Tester", "AprecID", "USB Sender", "Settings" };
static const int8_t NITEM = 4;
#define ITEM_Y     34
#endif
static int8_t g_sel = 0;
static bool   g_pic = false;        // About is on the screen, not the menu

#define ITEM_X     148
#define ITEM_W     160
#define ITEM_H     30
#define ITEM_STEP  38
#define FOOT_Y     196

// On the left, top to bottom, a DE-9 plug seen from the front, after the
// drawing of commons.wikimedia.org/wiki/File:DE9_Diagram.svg, turned a quarter
// right: pin 1 at the top of the wide side. Drawn from a few shapes rather than
// kept as a picture, which saves the kilobyte the mascot took.
#define PLUG_X     70           // the middle of it
#define PLUG_Y     120

static const uint16_t METAL  = 0xAD55;
static const uint16_t SHELL  = 0x7BCF;
static const uint16_t GOLD   = 0xFE60;
static const uint16_t GOLD_D = 0x9300;

// Every shape here is one of these: columns centred on y = cy, from x0
// rightwards, wL tall at the left to wR at the right, w columns in all, with
// the corners taken off as by a circle of radius r. The D of the plug is
// taller at the right; with equal heights it is a rounded rectangle, and 2r+1
// wide and tall it is a circle. Cheaper than the library's circles, which
// would cost most of what leaving out the mascot saved.
static void shape(Lcd& tft, int16_t cy, int16_t x0, int16_t wL, int16_t wR, int16_t w, int16_t r, uint16_t c) {
    for (int16_t x = 0; x < w; x++) {
        int16_t half = (int16_t)((wL * (w - 1 - x) + wR * x) / (w - 1) / 2);
        int16_t d = x < r ? r - x : (x >= w - r ? x - (w - 1 - r) : 0);
        int16_t in = 0;
        if (d > 0) {
            int16_t k = 0;
            while ((k + 1) * (k + 1) <= r * r + r - d * d) k++;
            in = r - k;
        }
        tft.fillRect(x0 + x, cy - half + in, 1, 2 * (half - in) + 1, c);
    }
}

static void circle(Lcd& tft, int16_t x, int16_t y, int16_t r, uint16_t c) {
    shape(tft, y, x - r, 2 * r + 1, 2 * r + 1, 2 * r + 1, r, c);
}

static void pin(Lcd& tft, int16_t x, int16_t y) {
    circle(tft, x, y, 5, GOLD_D);
    circle(tft, x, y, 4, GOLD);
    tft.fillRect(x - 2, y - 2, 2, 2, 0xFFF0);
}

static void drawPlug(Lcd& tft) {
    // the flange, with its two screw holes
    shape(tft, PLUG_Y, PLUG_X - 40, 174, 174, 81, 8, METAL);
    for (int8_t s = -1; s <= 1; s += 2) {
        circle(tft, PLUG_X, PLUG_Y + s * 74, 8, SHELL);
        circle(tft, PLUG_X, PLUG_Y + s * 74, 5, TH.menuBg);
    }
    shape(tft, PLUG_Y, PLUG_X - 30, 100, 123, 61, 9, SHELL);
    shape(tft, PLUG_Y, PLUG_X - 25, 91, 112, 51, 7, TH.menuBg);     // the inside, in the theme's colour, as the holes

    // pins 1 to 5 down the wide side, 6 to 9 down the narrow one
    for (int8_t i = 0; i < 5; i++) pin(tft, PLUG_X + 9, PLUG_Y - 36 + i * 18);
    for (int8_t i = 0; i < 4; i++) pin(tft, PLUG_X - 11, PLUG_Y - 27 + i * 18);
}

static void drawItem(Lcd& tft, int8_t i) {
    int16_t  y   = ITEM_Y + i * ITEM_STEP;
    bool     on  = (i == g_sel);
    uint16_t bg  = on ? TH.sel : TH.menuBg;
    tft.fillRect(ITEM_X, y, ITEM_W, ITEM_H, bg);
    tft.drawRect(ITEM_X, y, ITEM_W, ITEM_H, on ? TH.selText : TH.line);
    tft.setTextSize(2);
    tft.setTextColor(on ? TH.selText : TH.muted, bg);
    tft.setCursor(ITEM_X + 6, y + 8);
    tft.print(ITEM[i]);
}

void draw(Lcd& tft) {
    g_pic = false;
    tft.fillScreen(TH.menuBg);

    drawPlug(tft);

    for (int8_t i = 0; i < NITEM; i++) drawItem(tft, i);

    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.menuBg);
    tft.setCursor(ITEM_X + 2, FOOT_Y);
    tft.print((char)0x1E); tft.print((char)0x1F);       // up and down arrows
    tft.print(" select   OK/RUN start");

    // which build this is, so that two of them are not mistaken for each other
    tft.setTextColor(TH.trace, TH.menuBg);
    tft.setCursor(ITEM_X + 2, FOOT_Y + 16);
    // one line: "T232-DSTu v0.32 240MHz", from x 150 (the screen holds 28)
    tft.setTextWrap(false);
    tft.print(FW_NAME " v" FW_VERSION);
    tft.setTextColor(TH.muted, TH.menuBg);
    tft.print(' ');
    tft.print(SystemCoreClock / 1000000);       // 240 (72 while the clock is still coming up)
    tft.print("MHz");
    if (image) {                                // the image check: " ok" fills the line to 28
        tft.setTextColor(image > 0 ? TH.ok : TH.err, TH.menuBg);
        tft.print(image > 0 ? " ok" : " BAD");
    }
    tft.setTextWrap(true);
}

// ---- About: what AprecID knows, a tile for each, in the theme's colours ----
// Drawn from lines and text (the mock-up: docs/about/), not kept as a picture.

static uint16_t mix(uint16_t a, uint16_t b, uint8_t f) {     // a + (b - a) * f / 256, RGB565
    int16_t r = a >> 11, g = (a >> 5) & 63, l = a & 31;
    r += ((int16_t)(b >> 11) - r) * f / 256;
    g += ((int16_t)((b >> 5) & 63) - g) * f / 256;
    l += ((int16_t)(b & 31) - l) * f / 256;
    return (uint16_t)(r << 11 | g << 5 | l);
}

static void dots(Lcd& tft, int16_t x, int16_t y, int16_t w) {
    for (int16_t i = 0; i < w; i += 3) tft.drawPixel(x + i, y, TH.muted);
}

// A logic trace from (x, y): pairs of (length, height from the top), with the
// edges between them; a pair of two zeros ends it.
static void steps(Lcd& tft, int16_t x, int16_t y, const uint8_t* lv, uint16_t c) {
    int16_t prev = -1;
    for (; lv[0]; lv += 2) {
        int16_t n = lv[0], dy = lv[1];
        if (prev >= 0 && prev != dy) tft.drawFastVLine(x, y + min(prev, dy), abs(prev - dy) + 1, c);
        tft.drawFastHLine(x, y + dy, n, c);
        x += n;
        prev = dy;
    }
}

static const uint8_t T_RS232[] = { 8,19, 6,2, 6,19, 12,2, 6,19, 6,2, 8,19, 12,19, 0,0 };
static const uint8_t T_TTL[]   = { 8,3, 6,20, 6,3, 12,20, 6,3, 6,20, 20,3, 0,0 };
static const uint8_t T_485A[]  = { 8,2, 6,9, 6,2, 12,9, 6,2, 6,9, 20,2, 0,0 };
static const uint8_t T_485B[]  = { 8,20, 6,13, 6,20, 12,13, 6,20, 6,13, 20,20, 0,0 };
static const uint8_t T_CANH[]  = { 8,11, 5,3, 5,11, 10,3, 5,11, 5,3, 26,11, 0,0 };
static const uint8_t T_CANL[]  = { 8,12, 5,20, 5,12, 10,20, 5,12, 5,20, 26,12, 0,0 };
static const uint8_t T_I2C[]   = { 9,3, 3,19, 3,3, 3,19, 3,3, 3,19, 3,3, 3,19, 3,3, 3,19, 3,3,
                                   3,19, 3,3, 3,19, 3,3, 3,19, 12,3, 0,0 };
static const uint8_t T_PWM[]   = { 2,19, 4,3, 11,19, 4,3, 11,19, 4,3, 11,19, 4,3, 13,19, 0,0 };
static const uint8_t T_LIN[]   = { 4,3, 20,20, 4,3, 3,20, 3,3, 3,20, 3,3, 3,20, 3,3, 3,20, 15,3, 0,0 };

enum { I_RS232, I_TTL, I_RS485, I_MODBUS, I_CAN, I_I2C, I_PWM, I_LIN, I_3V3, I_5V, I_24V, I_GND };

// The drawing in a tile, 64 x 22 from (x, y).
static void icon(Lcd& tft, uint8_t k, int16_t x, int16_t y) {
    uint16_t c = TH.trace;
    switch (k) {
    case I_RS232: dots(tft, x, y + 11, 64); steps(tft, x, y, T_RS232, c); break;   // through zero, idles low
    case I_TTL:   dots(tft, x, y + 20, 64); steps(tft, x, y, T_TTL, c); break;     // 0 V up, idles high
    case I_RS485: steps(tft, x, y, T_485A, c); steps(tft, x, y, T_485B, TH.text2); break;
    case I_CAN:   steps(tft, x, y, T_CANH, c); steps(tft, x, y, T_CANL, TH.text2); break;
    case I_I2C:   steps(tft, x, y, T_I2C, c); break;                              // SCL: a burst of clocks
    case I_PWM:   steps(tft, x, y, T_PWM, c); break;                              // 25 %
    case I_LIN:   dots(tft, x, y + 20, 64); steps(tft, x, y, T_LIN, c); break;     // the break, then 0x55
    case I_MODBUS: {                            // a frame: address, function, data, CRC
        static const uint8_t X[] = { 0, 14, 28, 50 }, W[] = { 12, 12, 20, 14 };
        const uint16_t col[] = { c, TH.text2, TH.text, TH.ok };
        for (uint8_t i = 0; i < 4; i++) {
            tft.drawRect(x + X[i], y + 6, W[i], 11, col[i]);
            tft.fillRect(x + X[i] + 2, y + 9, W[i] - 4, 5, col[i]);
        }
        break;
    }
    case I_3V3: case I_5V: case I_24V: {        // a level over 0 V
        static const uint8_t H[] = { 6, 9, 18 };
        dots(tft, x, y + 20, 64);
        tft.drawFastHLine(x, y + 20 - H[k - I_3V3], 64, c);
        break;
    }
    case I_GND:
        tft.drawFastHLine(x, y + 4, 26, c);
        tft.drawFastVLine(x + 26, y + 4, 8, c);
        for (uint8_t i = 0; i < 3; i++) tft.drawFastHLine(x + 26 - (11 - 4 * i), y + 12 + i * 4, 23 - 8 * i, c);
        break;
    }
}

static const char* const ROW[] = { "UART", "bus", "DC" };
static const char* const TILE[3][4] = { { "RS232", "TTL", "RS485", "Modbus" },
                                        { "CAN", "I2C", "PWM", "LIN" },
                                        { "3V3", "5V", "24V", "GND" } };

void about(Lcd& tft) {
    g_pic = true;
    tft.fillScreen(TH.bg);
    tft.setTextWrap(false);
    uint16_t dim  = mix(TH.bg, TH.trace, 90);   // the title's shadow, the rule
    uint16_t band = mix(TH.bg, TH.line, 128);   // under the labels

    tft.setTextSize(4);                         // the name, with a shadow of one pixel
    tft.setTextColor(dim);
    tft.setCursor(9, 7);  tft.print("AprecID");
    tft.setTextColor(TH.trace);
    tft.setCursor(8, 6);  tft.print("AprecID");
    tft.setTextSize(1);
    tft.setTextColor(TH.text2);
    tft.setCursor(184, 8);  tft.print("touch a wire,");
    tft.setCursor(184, 18); tft.print("know the line");
    tft.setTextColor(TH.muted);
    tft.setCursor(184, 30); tft.print(FW_NAME " v" FW_VERSION);
    tft.drawFastHLine(6, 41, 308, dim);

    // a row each, its name above; tiles 76 x 46, 1 px apart
    for (uint8_t r = 0; r < 3; r++) {
        int16_t y = 46 + r * 58, ty = y + 10;
        tft.setTextSize(1);
        tft.setTextColor(TH.muted);
        tft.setCursor(6, y); tft.print(ROW[r]);
        for (uint8_t i = 0; i < 4; i++) {
            int16_t x = 6 + i * 77;
            tft.fillRect(x + 1, ty + 28, 74, 17, band);
            tft.drawRoundRect(x, ty, 76, 46, 3, TH.line);
            icon(tft, r * 4 + i, x + 6, ty + 4);
            tft.setTextSize(2);
            tft.setTextColor(TH.text);
            tft.setCursor(x + (76 - 12 * (int16_t)strlen(TILE[r][i])) / 2, ty + 30);
            tft.print(TILE[r][i]);
        }
    }
    tft.setTextSize(1);
    tft.setTextColor(TH.muted);
    tft.setCursor(6, 230);   tft.print("zeromem0.github.io/apre");
    tft.setCursor(236, 230); tft.print("any key: menu");
    tft.setTextWrap(true);
}

int8_t poll(Lcd& tft) {
    uint8_t autoKey = Keys::press(Keys::AUTO);
    if (g_pic) {
        bool any = autoKey;
        for (uint8_t k : { Keys::UP, Keys::DOWN, Keys::LEFT, Keys::RIGHT, Keys::OK, Keys::RUN, Keys::MODE })
            if (Keys::hit(k)) any = true;
        if (any) draw(tft);
        return -1;
    }
    if (autoKey == Keys::LONG) { about(tft); return -1; }

    int8_t was = g_sel;
    // round at the ends: down from the last goes to the first
    if (Keys::hit(Keys::UP))   g_sel = (int8_t)((g_sel + NITEM - 1) % NITEM);
    if (Keys::hit(Keys::DOWN)) g_sel = (int8_t)((g_sel + 1) % NITEM);
    if (g_sel != was) { drawItem(tft, was); drawItem(tft, g_sel); }

    if (Keys::hit(Keys::OK) || Keys::hit(Keys::RUN)) return g_sel;
    return -1;
}

}
