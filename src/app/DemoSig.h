// Made-up signals for the inspector's demo build (DEMO_SIGNAL): each a line
// as it would come off a real board or bus, sample by sample, at the sample
// rate asked for and in the converter's counts on the range in force, with a
// step of noise. For trying the detectors on the unit without the hardware,
// and for the pictures of the screens. Never in a build for use.
//
//   GND, 3.3 V, 5 V         steady levels
//   RS232                   9600 8N1, -9 / +10 V, a line of text now and then
//   TTL 3.3 V               115200 8N1, an ESP32's log line every 20 ms
//   PWM                     1 kHz, 25 %, 0 / 3.3 V
//   RS485 A, RS485 B        a Modbus RTU request (01 03 00 00 00 02 C4 0B) at
//                           9600 8N1 every 50 ms; B rests high, A low
//   CAN_H, CAN_L            250 kbit/s, one standard frame (ID 0x123, two data
//                           bytes) every millisecond, with its CRC-15 and the
//                           stuff bits; recessive 2.5 V, dominant 3.5 / 1.5 V
//   I2C SCL, I2C SDA        100 kHz: START, address 0x50 write, ACK, 0x3C,
//                           ACK, STOP, every millisecond
//   LIN                     19200, 12 V at rest, every 20 ms: the master asks
//                           for ID 0x10 and the slave answers 12 34 with the
//                           enhanced checksum; then a header for ID 0x11 that
//                           nobody answers

#ifndef DEMOSIG_H
#define DEMOSIG_H

#include <stdint.h>

namespace DemoSig {

enum { GND, V33, V5, RS232, TTL, PWM, RS485A, RS485B, CANH, CANL, SCL, SDA, LIN, COUNT };

const char* name(uint8_t k);
void        set(uint8_t k);
uint8_t     which();

// The next block of samples at `hz`, in counts on the range in force.
const uint16_t* block(uint16_t& n, uint32_t hz);

}
#endif
