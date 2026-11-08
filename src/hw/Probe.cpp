#include "Probe.h"

extern "C" {
volatile Probe probe __attribute__((used));
// Stage at which setup() stops: the RAM loader writes it after
// loading. In .data, not .bss, so that the start-up copies it onto itself and
// keeps it.
volatile uint32_t stop_at __attribute__((used)) = 0xFFFFFFFFu;
// From flash, what a fault shows on the screen (set by the application once
// the screen is up).
void (*volatile fault_show)(void) = 0;
}

void stage(uint32_t n) {
    probe.stage = n;
    if (n == stop_at) {
        probe.magic = 0x504F5453;   // "STOP"
        for (;;) {
            IWDG->KR = 0xAAAA;
            probe.loops++;
        }
    }
}

// delay() calls yield() while it waits. Without the reload here the screen's
// 120 ms after Sleep Out was enough for a watchdog reset, and the bootloader
// switched SWD off (2026-10-26).
extern "C" void yield(void) {
    IWDG->KR = 0xAAAA;
}

// Replaces the core's endless loop: keeps the watchdog fed, so that a fault
// leaves the unit running and SWD alive, and records where it happened.
extern "C" __attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b hard_fault_c  \n");
}

// From flash (no RAM_RUN), where there is no debugger to read `probe`, the
// fault is shown on the screen and the unit stays on, the watchdog fed,
// until the power key is held for two seconds. (It used to reset, and a reset
// switches the unit off: v0.17 did that at start and nothing could be seen,
// 2026-10-26.)
extern "C" __attribute__((used)) void hard_fault_c(uint32_t *frame) {
    probe.fault[0] = SCB->CFSR;
    probe.fault[1] = SCB->HFSR;
    probe.fault[2] = SCB->BFAR;
    probe.fault[3] = SCB->MMFAR;
    probe.fault[4] = frame[6];
    probe.magic = 0x544C5546;   // "FULT"
#ifdef RAM_RUN
    for (;;) IWDG->KR = 0xAAAA;
#else
    IWDG->KR = 0xAAAA;
    void (*show)(void) = fault_show;
    fault_show = 0;                     // a fault while showing it: not again
    if (show) show();
    // The power key, PD2, active low, held: off (PD3 low), and once it has
    // been let go a reset, so that the bootloader leaves the unit off and the
    // next press starts it (main.cpp, powerOff()). Counted in loop turns: no
    // SysTick here.
    uint32_t held = 0, up = 0;
    bool off = false;
    for (;;) {
        IWDG->KR = 0xAAAA;
        bool down = !(GPIOD->IDR & (1u << 2));     // PD2: the power key
        if (!off) {
            if (down) { if (++held > 20000000u) { GPIOD->BRR = 1u << 3; off = true; } }
            else held = 0;
        } else {
            up = down ? 0 : up + 1;
            if (up > 2000000u) NVIC_SystemReset();
        }
    }
#endif
}

// From the crystal to 240 MHz, the FNIRSI firmware's own set-up, copied
// (0x0802C0xx in app.c): the 8 MHz crystal halved (CRM_MISC2 HEXTDIV 0, CFG
// PLLHEXTDIV), times 60 (PLLMULT_H 3, PLLMULT_L 11), PLLRANGE for above
// 72 MHz, AHB undivided, APB1 and APB2 halved to 120 MHz (their timers
// 240 MHz, the usual rule). The switch is made with the auto step on, as
// FNIRSI does. Register by register: the HAL knows neither the longer
// multiplier nor the auto step, and SystemCoreClockUpdate() would read the
// multiplier STM32 fashion, so the frequency is set by hand. From any clock
// (the HSI, or the 72 MHz PLL, which is switched off first).
// Each wait is bounded, the watchdog fed: a wait that never ends let the
// watchdog reset the unit, and a reset with the keys let go leaves it off.
// Returns 0, or which wait gave up: 1 no crystal, 2 back on the HSI,
// 3 the PLL stopping, 4 the PLL locking at 240, 5 the switch to it. On 4
// and 5 the core is left on the HSI (8 MHz).
static bool waitFor(volatile uint32_t& reg, uint32_t mask, uint32_t want) {
    for (uint32_t n = 0; n < 4000000u; n++) {
        if ((reg & mask) == want) return true;
        if (!(n & 0xFFFF)) IWDG->KR = 0xAAAA;
    }
    return false;
}

static void onHsi() {
    RCC->CFGR &= ~RCC_CFGR_SW;
    waitFor(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI);
    SystemCoreClock = 8000000u;
    HAL_InitTick(TICK_INT_PRIORITY);
}

uint8_t clock240() {
    if (!(RCC->CR & RCC_CR_HSERDY)) return 1;
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) {
        RCC->CFGR &= ~RCC_CFGR_SW;                          // back on the HSI
        if (!waitFor(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI)) return 2;
        RCC->CR &= ~RCC_CR_PLLON;
        if (!waitFor(RCC->CR, RCC_CR_PLLRDY, 0)) return 3;
    }
    volatile uint32_t& misc2 = *(volatile uint32_t*)0x40021054;
    misc2 &= ~0x3000u;                                      // HEXTDIV: crystal / 2
    RCC->CFGR = (RCC->CFGR & ~(0x7u << 29 | 0xFu << 18 | 0x3u << 16))
              | 1u << 31 | 3u << 29 | 0xBu << 18 | 3u << 16;
    RCC->CR |= RCC_CR_PLLON;
    if (!waitFor(RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) { onHsi(); return 4; }
    RCC->CFGR = (RCC->CFGR & ~(0xFu << 4 | 0x7u << 8 | 0x7u << 11))
              | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;
    misc2 |= 0x30u;                                         // auto step on
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    bool sw = waitFor(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);
    misc2 &= ~0x30u;
    if (!sw) { onHsi(); return 5; }
    SystemCoreClock = 240000000u;
    HAL_InitTick(TICK_INT_PRIORITY);
    probe.cfgr = RCC->CFGR;
    probe.sysclk = SystemCoreClock;
    return 0;
}

// Replaces the core's weak SystemClock_Config: 72 MHz from the crystal (x9,
// APB1 halved), or the HSI (8 MHz) if the crystal does not start. 240 MHz
// (clock240()) comes only once the menu is up (main.cpp): started from flash
// at 240 MHz straight away, the tester often did not come up (a white flash
// and the unit off), though the same image ran from RAM over SWD; 72 MHz first
// and 240 after the screen's set-up started every time (2026-10-27). Why is
// not known.
extern "C" void SystemClock_Config(void) {
    IWDG->KR = 0xAAAA;
    // The power hold high before anything else, as FNIRSI does early on. On
    // the DST-210 (PA3 there) every image that
    // set it at once kept SWD; every one that set it only after the screen's
    // ~200 ms set-up, or never, lost SWD soon after starting (2026-10-26).
    // It is the power hold (FNIRSI's power off sets it low).
    // On the DSO-510 the hold is PD3 (FNIRSI's power off at 0x0801B7FC).
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    GPIOD->BSRR = 1u << 3;
    GPIOD->CRL = (GPIOD->CRL & ~(0xFu << 12)) | (0x3u << 12);
    probe.stage = 90;
    RCC->CR |= RCC_CR_HSEON;
    // The crystal is given a good while: run from flash it is not going yet
    // (from RAM over SWD the FNIRSI firmware has started it). v0.32 waited
    // only the 200 000 turns below and then looked once, and from flash the
    // unit went off as soon as the menu was drawn, every time (2026-10-28);
    // probably the crystal was not ready by then, the start went on on the
    // HSI and later straight to 240 MHz. Up to v0.31 the reading of three
    // keys (the old start options) stood between the wait and the look.
    uint32_t n = 0;
    while (!(RCC->CR & RCC_CR_HSERDY) && ++n < 200000) {}
    for (volatile uint32_t d = 0; d < 2000; d++) {}
    while (!(RCC->CR & RCC_CR_HSERDY) && ++n < 2000000) {}

    if (!(RCC->CR & RCC_CR_HSERDY)) {
        SystemCoreClockUpdate();
    } else {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_PPRE1
                                   | 0x7u << 29))
                  | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;
        RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY)) {}
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
        probe.hse = 1;
        SystemCoreClock = 72000000u;
    }
    // The USB clock, as the FNIRSI firmware gives it (app.c, after the timers
    // in 0x0801F0B0): not from the PLL but from the internal 48 MHz HICK,
    // undivided (CRM_MISC1 bit 25), routed to the USB (CRM_MISC2 bits 8 and
    // 9), and kept on 48 MHz by the auto clock calibration (ACC, 0x40015800,
    // clock in APB2EN bit 22) against the PC's start of frame every
    // millisecond: limits 7980 and 8020 around 8000, trimming and
    // calibration on. The core's STM32 USB prescaler then plays no part.
    *(volatile uint32_t*)0x40021030 |= 1u << 25;
    *(volatile uint32_t*)0x40021054 |= 0x200u;
    *(volatile uint32_t*)0x40021054 |= 0x100u;
    RCC->APB2ENR |= 1u << 22;
    *(volatile uint32_t*)0x4001580C = 0x1F2C;
    *(volatile uint32_t*)0x40015810 = 8000;
    *(volatile uint32_t*)0x40015814 = 0x1F54;
    *(volatile uint32_t*)0x40015804 |= 2u;
    *(volatile uint32_t*)0x40015804 |= 1u;

    // cycle counter, for timing the decoder (probe.blkCyc)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_InitTick(TICK_INT_PRIORITY);
    probe.cfgr = RCC->CFGR;
    probe.sysclk = SystemCoreClock;
    // Started from flash, the FNIRSI bootloader has jumped with interrupts
    // masked (its application unmasks them only in main); SysTick needs them.
    __enable_irq();
}

struct Key { GPIO_TypeDef *port; uint8_t bit; };

// The FNIRSI key table's order on the DSO-510 (0x080349F8), confirmed on the
// unit (2026-11-07).
static const Key kKeys[9] = {
    {GPIOD, 2},  {GPIOC, 9},  {GPIOC, 7},  {GPIOC, 8},  {GPIOC, 6},
    {GPIOD, 13}, {GPIOB, 15}, {GPIOB, 14}, {GPIOB, 13},
};

const char *const kKeyNames[9] = {
    "POWER", "RUN", "UP", "DOWN", "MODE", "AUTO", "RIGHT", "OK", "LEFT",
};

void keysBegin() {
    pinMode(PD2, INPUT_PULLUP);
    pinMode(PC9, INPUT_PULLUP);
    pinMode(PC7, INPUT_PULLUP);
    pinMode(PC8, INPUT_PULLUP);
    pinMode(PC6, INPUT_PULLUP);
    pinMode(PD13, INPUT_PULLUP);
    pinMode(PB15, INPUT_PULLUP);
    pinMode(PB14, INPUT_PULLUP);
    pinMode(PB13, INPUT_PULLUP);
}

uint32_t keysRead() {
    uint32_t k = 0;
    for (uint8_t i = 0; i < 9; i++) {
        if (!(kKeys[i].port->IDR & (1u << kKeys[i].bit))) k |= 1u << i;
    }
    return k;
}
