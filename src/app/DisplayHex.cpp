#include "DisplayHex.h"

void displayHexLine(Lcd& tft, const char* buf, int len, int line, int y) {
    int start = line * HEX_PER_LINE;
    int count = len - start;
    if (count > HEX_PER_LINE) count = HEX_PER_LINE;

    tft.fillRect(0, y, 320, HEX_LINE_H, TH.bg);
    if (count <= 0) return;

    tft.setTextSize(1);
    tft.setTextColor(TH.text2);

    // the characters
    tft.setCursor(3, y);
    for (int j = 0; j < count; j++) {
        uint8_t c = (uint8_t)buf[start + j];
        tft.print((c >= 32 && c < 127) ? (char)c : '_');
    }

    // and their hex
    int x = 106;
    for (int j = 0; j < count; j++) {
        uint8_t c = (uint8_t)buf[start + j];
        tft.setCursor(x, y);
        if (c < 0x10) tft.print('0');
        tft.print(c, HEX);
        x += 14;
    }
}

void displayHex(Lcd& tft, const char* buf, int len, int firstLine) {
    for (int i = 0; i < HEX_VIS_LINES; i++)
        displayHexLine(tft, buf, len, firstLine + i, 20 + i * HEX_LINE_H);
}
