#include "DisplayChars.h"

void displayCharsLine(Lcd& tft, const char* buf, int len, int line, int y) {
    int start = line * CHARS_PER_LINE;
    int count = len - start;
    if (count > CHARS_PER_LINE) count = CHARS_PER_LINE;

    tft.fillRect(0, y, 320, CHARS_LINE_H, TH.bg);
    if (count <= 0) return;

    tft.setTextSize(1);
    tft.setTextColor(TH.text2);
    tft.setCursor(3, y);
    for (int j = 0; j < count; j++) {
        uint8_t c = (uint8_t)buf[start + j];
        tft.print((c >= 32 && c < 127) ? (char)c : '_');
    }
}

void displayChars(Lcd& tft, const char* buf, int len, int firstLine) {
    for (int i = 0; i < CHARS_VIS_LINES; i++)
        displayCharsLine(tft, buf, len, firstLine + i, 20 + i * CHARS_LINE_H);
}

// --- a line for each message --------------------------------------------------------
#define LINES_VIS CHARS_VIS_LINES

struct Row { int start, n; uint32_t gapMs; };

static void drawRow(Lcd& tft, const char* buf, const Row& r, int y) {
    tft.fillRect(1, y, 318, CHARS_LINE_H, TH.bg);
    if (r.n <= 0 && !r.gapMs) return;
    tft.setTextSize(1);
    for (int j = 0; j < r.n; j++) {
        uint8_t c = (uint8_t)buf[r.start + j];
        tft.setCursor(3 + j * 6, y);
        if (c == '\r')                { tft.setTextColor(TH.muted); tft.print((char)0x1B); }   // a left arrow
        else if (c == '\n')           { tft.setTextColor(TH.muted); tft.print((char)0x19); }   // a down arrow
        else if (c >= 32 && c < 127)  { tft.setTextColor(TH.text2); tft.print((char)c); }
        else                          { tft.setTextColor(TH.muted); tft.print('_'); }
    }
    if (r.gapMs) {                                  // the rest before it, at the right, if it fits
        char t[12];
        uint32_t ds = (r.gapMs + 50) / 100;
        if (ds >= 6000) snprintf(t, sizeof(t), "+%lum", (unsigned long)((ds + 300) / 600));
        else            snprintf(t, sizeof(t), "+%lu.%lus", (unsigned long)(ds / 10), (unsigned long)(ds % 10));
        int w = (int)strlen(t);
        if (r.n + 1 + w <= CHARS_PER_LINE) {
            tft.setTextColor(TH.muted);
            tft.setCursor(3 + (CHARS_PER_LINE - w) * 6, y);
            tft.print(t);
        }
    }
}

void displayCharsLines(Lcd& tft, const char* buf, int len, const CharsGap* gaps, uint8_t nGaps, bool full,
                       int16_t y0, uint8_t rows) {
    if (rows > LINES_VIS) rows = LINES_VIS;
    static Row shown[LINES_VIS];
    static int shownN = -1;

    // The rows, the last LINES_VIS of them kept, going through the whole text.
    Row last[LINES_VIS];
    int total = 0;
    uint8_t g = 0;
    int start = 0;
    uint32_t gapMs = 0;
    while (g < nGaps && gaps[g].at <= 0) g++;
    for (int i = 0; i <= len; i++) {
        bool atGap = g < nGaps && gaps[g].at == i;
        bool end   = i == len || atGap || i - start == CHARS_PER_LINE
                  || (i > start && buf[i - 1] == '\n');
        if (!end) continue;
        if (i > start || (i == len && gapMs)) {
            Row r = { start, i - start, gapMs };
            last[total % LINES_VIS] = r;
            total++;
            gapMs = 0;
        }
        start = i;
        if (atGap) { gapMs = gaps[g].ms; g++; }
    }

    if (full || shownN < 0) {
        for (int k = 0; k < LINES_VIS; k++) shown[k] = Row{ -1, 0, 0 };
        tft.fillRect(1, y0, 318, rows * CHARS_LINE_H, TH.bg);
    }
    int first = total > rows ? total - rows : 0;
    for (int k = 0; k < rows; k++) {
        Row r = { -1, 0, 0 };
        if (first + k < total) r = last[(first + k) % LINES_VIS];
        Row& s = shown[k];
        if (r.start == s.start && r.n == s.n && r.gapMs == s.gapMs) continue;   // as it is on the screen
        drawRow(tft, buf, r, y0 + k * CHARS_LINE_H);
        s = r;
    }
    shownN = total;
}
