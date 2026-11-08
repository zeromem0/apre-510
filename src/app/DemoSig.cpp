#include <Arduino.h>
#include "DemoSig.h"
#include "Frontend.h"

namespace DemoSig {

static const char* const NAME[COUNT] = {
    "GND", "3.3V", "5V", "RS232 9600", "TTL 115200", "PWM 1 kHz",
    "RS485 A", "RS485 B", "CAN_H 250k", "CAN_L 250k", "I2C SCL", "I2C SDA", "LIN 19200",
};

static uint8_t  g_k = RS232;
static uint32_t g_tp;                   // where in the signal's period, ns
static uint16_t g_buf[1024];
static uint16_t g_noise = 1;

const char* name(uint8_t k) { return k < COUNT ? NAME[k] : "?"; }
void        set(uint8_t k)  { g_k = k < COUNT ? k : 0; g_tp = 0; }
uint8_t     which()         { return g_k; }

// --- serial lines ---------------------------------------------------------------
static const char    TEXT232[] = "Apreciometru RS232\r\n";
static const char    TEXTTTL[] = "I (312) wifi:connected\r\n";
static const uint8_t MODBUS[]  = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B };

// The logical level at `tp` ns into a message of 8N1 characters sent from
// tp = 0 at one bit every `tb` ns; 1 (mark) before and after it.
static uint8_t uartBit(const uint8_t* m, uint8_t len, uint32_t tp, uint32_t tb) {
    uint32_t idx = tp / tb;
    if (idx >= (uint32_t)len * 10) return 1;
    uint8_t c = m[idx / 10], b = (uint8_t)(idx % 10);
    if (b == 0) return 0;                       // start
    if (b <= 8) return (c >> (b - 1)) & 1;      // data, LSB first
    return 1;                                   // stop
}

// --- CAN: one standard data frame, stuffed ----------------------------------------
static uint8_t g_can[160], g_canN;

static void canBuild() {
    uint8_t raw[100], n = 0;
    raw[n++] = 0;                                               // SOF
    for (int8_t i = 10; i >= 0; i--) raw[n++] = (0x123 >> i) & 1;   // identifier
    raw[n++] = 0; raw[n++] = 0; raw[n++] = 0;                   // RTR, IDE, r0
    for (int8_t i = 3; i >= 0; i--) raw[n++] = (2 >> i) & 1;    // DLC 2
    const uint8_t data[2] = { 0x12, 0x34 };
    for (uint8_t d = 0; d < 2; d++)
        for (int8_t i = 7; i >= 0; i--) raw[n++] = (data[d] >> i) & 1;
    uint16_t crc = 0;                                           // CRC-15, x^15+x^14+x^10+x^8+x^7+x^4+x^3+1
    for (uint8_t i = 0; i < n; i++) {
        uint8_t nxt = raw[i] ^ ((crc >> 14) & 1);
        crc = (uint16_t)((crc << 1) & 0x7FFF);
        if (nxt) crc ^= 0x4599;
    }
    for (int8_t i = 14; i >= 0; i--) raw[n++] = (crc >> i) & 1;
    // a stuff bit of the other level after five alike, from SOF to the CRC
    uint8_t last = 2, same = 0;
    g_canN = 0;
    for (uint8_t i = 0; i < n; i++) {
        g_can[g_canN++] = raw[i];
        same = raw[i] == last ? (uint8_t)(same + 1) : 1;
        last = raw[i];
        if (same == 5) { g_can[g_canN++] = (uint8_t)!last; last = (uint8_t)!last; same = 1; }
    }
    g_can[g_canN++] = 1;                                        // CRC delimiter
    g_can[g_canN++] = 0;                                        // ACK, from a receiver
    for (uint8_t i = 0; i < 8; i++) g_can[g_canN++] = 1;        // ACK delimiter, end of frame
}

// --- I2C: a write of one byte, in quarters of a bit: bit 0 SCL, bit 1 SDA ----------
static uint8_t g_i2c[96], g_i2cN;          // 4 + 2 x 9 x 4 + 4 = 80 quarters

static void i2cBuild() {
    uint8_t n = 0;
    g_i2c[n++] = 3; g_i2c[n++] = 3;                             // idle, both high
    g_i2c[n++] = 1; g_i2c[n++] = 1;                             // START: SDA falls, SCL high
    const uint8_t bytes[2] = { 0x50 << 1, 0x3C };               // address 0x50, write; then data
    for (uint8_t b = 0; b < 2; b++) {
        for (int8_t i = 8; i >= 0; i--) {
            uint8_t sda = i ? (bytes[b] >> (i - 1)) & 1 : 0;    // eight bits, MSB first, then ACK (0)
            uint8_t s = (uint8_t)(sda << 1);
            g_i2c[n++] = s; g_i2c[n++] = s;                     // SCL low, SDA set
            g_i2c[n++] = s | 1; g_i2c[n++] = s | 1;             // SCL high, SDA held
        }
    }
    g_i2c[n++] = 0; g_i2c[n++] = 1; g_i2c[n++] = 1;             // STOP: SDA rises with SCL high
    g_i2c[n++] = 3;
    g_i2cN = n;
}

// --- LIN: a frame answered, then a header left unanswered ---------------------------
// One bit a byte of g_lin, 1 recessive. A header is a break (13 dominant
// bits), a delimiter, the sync byte 0x55 and the protected identifier; the
// slave's response follows a bit later: data and checksum.
static uint8_t  g_lin[400];
static uint16_t g_linN;
static const uint32_t LIN_BIT_NS = 52083;       // 19200

static uint8_t linPid(uint8_t id) {
    uint8_t b = id & 0x3F;
    uint8_t p0 = (uint8_t)(((b >> 0) ^ (b >> 1) ^ (b >> 2) ^ (b >> 4)) & 1);
    uint8_t p1 = (uint8_t)(~((b >> 1) ^ (b >> 3) ^ (b >> 4) ^ (b >> 5)) & 1);
    return (uint8_t)(b | (p0 << 6) | (p1 << 7));
}

static void linByte(uint8_t c) {
    g_lin[g_linN++] = 0;
    for (uint8_t i = 0; i < 8; i++) g_lin[g_linN++] = (c >> i) & 1;
    g_lin[g_linN++] = 1;
}

static void linHeader(uint8_t pid) {
    for (uint8_t i = 0; i < 13; i++) g_lin[g_linN++] = 0;
    g_lin[g_linN++] = 1;
    linByte(0x55);
    linByte(pid);
}

static void linBuild() {
    g_linN = 0;
    for (uint8_t i = 0; i < 20; i++) g_lin[g_linN++] = 1;
    uint8_t pid = linPid(0x10);
    linHeader(pid);
    g_lin[g_linN++] = 1;                                        // the slave answers a bit later
    const uint8_t data[2] = { 0x12, 0x34 };
    uint16_t sum = pid;                                         // enhanced: the identifier counts too
    for (uint8_t d = 0; d < 2; d++) {
        linByte(data[d]);
        sum += data[d];
        if (sum > 255) sum -= 255;
    }
    linByte((uint8_t)~sum);
    while (g_linN < 200) g_lin[g_linN++] = 1;
    linHeader(linPid(0x11));                                    // nobody answers this one
    while (g_linN < 384) g_lin[g_linN++] = 1;                   // 384 bits: 20 ms
}

// --- one sample ---------------------------------------------------------------------
// The level in mV at g_tp, and the length of the signal's period, ns.
static int32_t level(uint32_t& period) {
    switch (g_k) {
        case GND: period = 1000000; return 0;
        case V33: period = 1000000; return 3300;
        case V5:  period = 1000000; return 5000;
        case RS232:
            period = 300000000u;
            return uartBit((const uint8_t*)TEXT232, sizeof(TEXT232) - 1, g_tp, 104167) ? -9000 : 10000;
        case TTL:
            period = 20000000u;
            return uartBit((const uint8_t*)TEXTTTL, sizeof(TEXTTTL) - 1, g_tp, 8681) ? 3300 : 0;
        case PWM:
            period = 1000000u;
            return g_tp < 250000u ? 3300 : 0;
        case RS485A:
        case RS485B: {
            period = 50000000u;
            uint8_t b = uartBit(MODBUS, sizeof(MODBUS), g_tp, 104167);
            if (g_k == RS485A) b = (uint8_t)!b;
            return b ? 3700 : 1300;
        }
        case CANH:
        case CANL: {
            period = 1000000u;
            uint32_t i = g_tp / 4000u;
            uint8_t rec = i < g_canN ? g_can[i] : 1;
            return rec ? 2500 : (g_k == CANH ? 3500 : 1500);
        }
        case SCL:
        case SDA: {
            period = 1000000u;
            uint32_t i = g_tp / 2500u;
            uint8_t q = i < g_i2cN ? g_i2c[i] : 3;
            return ((g_k == SCL ? q : q >> 1) & 1) ? 3300 : 0;
        }
        case LIN: {
            period = g_linN * LIN_BIT_NS;
            uint32_t i = g_tp / LIN_BIT_NS;
            return (i < g_linN ? g_lin[i] : 1) ? 12000 : 900;
        }
    }
    period = 1000000;
    return 0;
}

const uint16_t* block(uint16_t& n, uint32_t hz) {
    if (!g_canN) canBuild();
    if (!g_i2cN) i2cBuild();
    if (!g_linN) linBuild();
    n = sizeof(g_buf) / sizeof(g_buf[0]);
    const uint32_t dt = (1000000000u + hz / 2) / (hz ? hz : 1);
    const int32_t zero = (int32_t)Frontend::zeroCounts(), cpv = (int32_t)Frontend::countsPerVolt();
    for (uint16_t i = 0; i < n; i++) {
        uint32_t period;
        int32_t mv = level(period);
        g_noise = (uint16_t)(g_noise * 1103u + 12345u);
        int32_t c = zero + mv * cpv / 1000 + (int32_t)((g_noise >> 9) & 0x0F) - 8;
        if (c < 0) c = 0;
        if (c > 4080) c = 4080;
        g_buf[i] = (uint16_t)(c & ~15);                 // the converter's 8 bits, times 16
        g_tp += dt;
        if (g_tp >= period) g_tp -= period;
    }
    return g_buf;
}

}
