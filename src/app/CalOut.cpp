#include <Arduino.h>
#include "CalOut.h"
#include "Config.h"
#include "Pins.h"

// TMR9 is not in the STM32F103xE headers; its registers by offset (Lcd.cpp).
#define TMR9_REG(off) (*(volatile uint32_t *)(0x40014C00u + (off)))

namespace CalOut {

// The test line: the text, then the line at rest for 50 ms, one bit to each
// TIM6 update.
static const char     MSG[]     = "APRE-510 9600 8N1\r\n";
static const uint16_t MSG_BITS  = (sizeof(MSG) - 1) * 10;
static const uint16_t REST_BITS = 480;
static volatile uint16_t g_bit;

static uint32_t timer6Clock() {
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x7u;
    uint32_t pclk1 = (ppre1 & 0x4u) ? SystemCoreClock >> ((ppre1 & 0x3u) + 1u) : SystemCoreClock;
    return (ppre1 & 0x4u) ? pclk1 * 2u : pclk1;
}

void apply() {
    NVIC_DisableIRQ(TIM6_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= (1u << 19) | RCC_APB2ENR_IOPAEN;    // TMR9, port A
    TIM6->CR1  = 0;
    TIM6->DIER = 0;
    TMR9_REG(0x20) &= ~0x30u;                           // channel 2 off

    switch (Config::calOut) {
        case Config::CAL_1K:
            // PWM mode 1: high while the count is below the compare value.
            // The count is the backlight's, 0..99 at 100 kHz.
            TMR9_REG(0x38) = 50;                                    // CCR2: half
            TMR9_REG(0x18) = (TMR9_REG(0x18) & ~0xFF00u) | 0x6000u; // CH2 PWM mode 1
            TMR9_REG(0x20) |= 0x10u;                                // CH2 output on
            Pins::mode(PIN_A(3), 0xB);                              // TMR9_CH2, alternate push-pull
            break;
        case Config::CAL_TTL:
            Pins::output(PIN_A(3), true);                           // a line at rest is high
            g_bit = MSG_BITS;                                       // a rest first
            TIM6->PSC = 0;
            TIM6->ARR = (timer6Clock() + 4800u) / 9600u - 1u;       // 25 000 counts of 240 MHz
            TIM6->EGR = TIM_EGR_UG;
            TIM6->SR  = 0;
            TIM6->DIER = TIM_DIER_UIE;
            NVIC_ClearPendingIRQ(TIM6_IRQn);
            NVIC_SetPriority(TIM6_IRQn, 1);                         // above the samples' (5): they take long
            NVIC_EnableIRQ(TIM6_IRQn);
            TIM6->CR1 = TIM_CR1_CEN;
            break;
        default:
            Pins::mode(PIN_A(3), 0x4);                              // floating input, as at reset
            break;
    }
}

}

// One bit: the start bit low, eight data bits from the lowest, the stop bit
// high.
extern "C" void TIM6_IRQHandler(void) {
    TIM6->SR = 0;
    uint16_t b = CalOut::g_bit;
    bool high = true;
    if (b < CalOut::MSG_BITS) {
        uint8_t k = (uint8_t)(b % 10u);
        uint8_t c = (uint8_t)CalOut::MSG[b / 10u];
        high = k == 0 ? false : k == 9 ? true : ((c >> (k - 1)) & 1u) != 0;
    }
    GPIOA->BSRR = high ? (1u << 3) : (1u << 19);
    if (++b >= CalOut::MSG_BITS + CalOut::REST_BITS) b = 0;
    CalOut::g_bit = b;
}
