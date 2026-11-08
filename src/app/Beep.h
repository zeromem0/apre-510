// The buzzer: PB9, TMR11 channel 1, as the FNIRSI firmware drives it (its key
// beep, about 2.2 kHz). A tone for so many milliseconds, not waited for:
// poll() ends it.

#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

namespace Beep {

void tone(uint16_t hz, uint16_t ms);
void poll();

}
#endif
