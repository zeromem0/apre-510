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
