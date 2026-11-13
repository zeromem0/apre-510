// The calibration output (the 1 kHz terminal next to the BNC), PA3: FNIRSI
// drives it from TMR9 channel 2, the timer whose channel 1 is the backlight.
// Here it is that 1 kHz square wave, or a test line for the probe: a short
// text over and over, TTL 9600 8N1, or nothing (the pin left floating, as
// before v0.13). Which one is a setting (Config::calOut).

#ifndef CALOUT_H
#define CALOUT_H

namespace CalOut {

// Sets the output as the setting has it. After Lcd::backlight() (TMR9 runs
// from there) and after the clock is settled.
void apply();

}
#endif
