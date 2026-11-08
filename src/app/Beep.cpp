#include <Arduino.h>
#include "Beep.h"

namespace Beep {

// TMR11 is not in the STM32F103xE headers; its registers by offset, as TMR9
// in Lcd.cpp. Its clock is the core's (APB2 halved at 240 MHz, timers
// doubled; undivided at 72).
#define TMR11_REG(off) (*(volatile uint32_t *)(0x40015400u + (off)))

static uint32_t g_endMs;
static bool     g_on;

static void pinAf(bool af) {
    // PB9: the timer's output while sounding, else a plain output held low
    GPIOB->BRR = 1u << 9;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << 4)) | ((af ? 0xBu : 0x3u) << 4);
}

void tone(uint16_t hz, uint16_t ms) {
    if (!hz || !ms) return;
    RCC->APB2ENR |= (1u << 21) | RCC_APB2ENR_IOPBEN;    // TMR11 clock
    uint32_t arr = 1000000u / hz - 1;                   // counted at 1 MHz
    TMR11_REG(0x00) &= ~1u;                             // CTRL1: stop
    TMR11_REG(0x28) = SystemCoreClock / 1000000 - 1;    // PSC: 1 MHz
    TMR11_REG(0x2C) = arr;                              // ARR
    TMR11_REG(0x34) = (arr + 1) / 2;                    // CCR1: half the period
    TMR11_REG(0x18) = (TMR11_REG(0x18) & ~0x7Fu) | 0x68; // CH1 PWM mode 1, preload
    TMR11_REG(0x20) |= 1;                               // CH1 output on
    TMR11_REG(0x14) = 1;                                // EVG: load the registers
    TMR11_REG(0x00) |= 1;                               // CTRL1: count
    pinAf(true);
    g_on = true;
    g_endMs = millis() + ms;
}

void poll() {
    if (!g_on || (int32_t)(millis() - g_endMs) < 0) return;
    pinAf(false);
    TMR11_REG(0x00) &= ~1u;
    g_on = false;
}

}
