#include "DisplayLarge.h"
#include <string.h>

#define SCREEN_W   320
#define PER_ROW    16           // characters in a row
#define CELL_W     18           // ... and the width each one takes

static char lastShown[32];      // the 32 characters drawn last

// A row of 16 characters at size 3, CR and LF as D and A on green.
static void drawCharRow(Lcd& tft, const char* buf, int len, int start, int y) {
    const int x0 = (SCREEN_W - PER_ROW * CELL_W) / 2;
    tft.setTextSize(3);
    for (int i = 0; i < PER_ROW; i++) {
        int idx = start + i;
        int x = x0 + i * CELL_W;
        tft.setCursor(x, y);
        if (idx < len) {
            char c = buf[idx];
            if (c == '\r' || c == '\n') {
                tft.fillRect(x - 1, y - 1, CELL_W, 25, TH.ok);
                tft.setTextColor(TH.bg, TH.ok);
                tft.print(c == '\r' ? 'D' : 'A');
            } else {
                tft.fillRect(x - 1, y - 1, CELL_W, 25, TH.bg);
                tft.setTextColor(TH.ok, TH.bg);
                tft.print(((uint8_t)c >= 32 && (uint8_t)c < 127) ? c : '.');
            }
        } else {
            tft.fillRect(x - 1, y - 1, CELL_W, 25, TH.bg);
            tft.setTextColor(TH.ok, TH.bg);
            tft.print(' ');
        }
    }
}

// The hex of the row above, small and grey.
static void drawHexRow(Lcd& tft, const char* buf, int len, int start, int y) {
    const int x0 = (SCREEN_W - PER_ROW * CELL_W) / 2;
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    for (int i = 0; i < PER_ROW; i++) {
        int idx = start + i;
        tft.setCursor(x0 + i * CELL_W + (CELL_W - 12) / 2, y);
        if (idx < len) {
            uint8_t c = (uint8_t)buf[idx];
            if (c < 0x10) tft.print('0');
            tft.print(c, HEX);
        } else {
            tft.print("  ");
        }
    }
}

void displayLarge(Lcd& tft, const char* buf, int len, int start) {
    const int yChars1 = 40;
    const int yHex1   = yChars1 + 30;
    const int yChars2 = yHex1 + 12 + 5;
    const int yHex2   = yChars2 + 30;

    drawCharRow(tft, buf, len, start,      yChars1);
    drawHexRow (tft, buf, len, start,      yHex1);
    drawCharRow(tft, buf, len, start + 16, yChars2);
    drawHexRow (tft, buf, len, start + 16, yHex2);
    displayLargeSuffix(tft, buf, len, 180);
}

void displayLargeSuffix(Lcd& tft, const char* buf, int len, int y) {
    // The last 5 characters received, at size 4. They are taken from the end
    // of the buffer, not from the 32 above, so that they show before 32 have
    // come; the last one always sits in the right-hand box. Five boxes 60 px
    // apart, the widest (CR, LF) 56, fit in the 320.
    const int last = 5;
    const int boxW = 60, x0 = 12;
    // A size 4 letter is drawn in the top left of its 24 x 32 cell, 20 wide
    // and 28 high, leaving 4 empty on the right and below. The same 4 is left
    // on the left and above, so the space to the border is even all round;
    // the border is 2.
    const int GAP = 4, BORDER = 2, OUT = GAP + BORDER;
    tft.setTextSize(4);
    for (int i = 0; i < last; i++) {
        int idx = len - last + i;
        int x = x0 + i * boxW;
        // cleared as wide as the widest box (CR, LF)
        tft.fillRect(x - OUT, y - OUT, 48 + 2 * BORDER + GAP, 32 + GAP + 2 * BORDER, TH.bg);
        if (idx < 0) continue;                      // nothing received there yet
        char c = buf[idx];
        tft.setCursor(x, y);
        int cells = (c == '\r' || c == '\n') ? 2 : 1;
        int innerW = 24 * cells + GAP, innerH = 32 + GAP;   // GAP before, the cell brings the rest
        uint16_t fill = (cells == 2) ? TH.ok : TH.bg;
        tft.fillRect(x - OUT, y - OUT, innerW + 2 * BORDER, innerH + 2 * BORDER, TH.ok2);
        tft.fillRect(x - GAP, y - GAP, innerW, innerH, fill);
        if (cells == 2) {
            tft.setTextColor(TH.bg, TH.ok);
            tft.print(c == '\r' ? "CR" : "LF");
        } else {
            tft.setTextColor(TH.ok, TH.bg);
            tft.print(((uint8_t)c >= 32 && (uint8_t)c < 127) ? c : '.');
        }
    }

    tft.setTextColor(TH.text, TH.bg);
}

bool largeBufferChanged(const char* buf, int len, int start) {
    char now[32] = {0};
    for (int i = start, j = 0; i < len && j < 32; i++, j++) now[j] = buf[i];
    if (!memcmp(now, lastShown, 32)) return false;
    memcpy(lastShown, now, 32);
    return true;
}
