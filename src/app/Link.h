// apre protocol 1 (web/protocol.html): the instrument's results, as JSON
// lines over the USB serial port, for the apre page (web/apre.html), and the
// page's commands back.
//
// Every message gets "n", counting from 0 at hello: a gap on the page is
// something lost on the way. What could not be queued (the queue full) is
// counted and said in the next tick. Nothing waits for the PC: the queue goes
// out a packet at a time as the port takes it, and is dropped while the port
// is closed. On opening: hello, then the last result, the last state and the
// theme.
//
//   hello   who and what it can do          tick    every 2 s, with "lost"
//   bytes   what was read, as it came       insp    each inspector result
//   frame   a packet taken apart            wave    a stretch of samples
//   state   what the instrument is doing    ack     the answer to a command
//   theme   the colours of the theme in force, sent again when another is chosen
//   screen  a row of what is on the screen, when the page asks ("screen")

#ifndef LINK_H
#define LINK_H

#include <stdint.h>

namespace Link {

void poll();                            // every turn of the main loop
void flush(uint32_t ms);                // before something else takes the port (USB Sender)

void rx(uint8_t c, const char* src);    // a byte read off the line: "tester", "insp"
void result(const char* json);          // an inspector result; kept, sent again on opening
void state(const char* json);           // the state; sent when it differs from the last
void send(const char* json);            // any other whole message, if a page listens
void sendResult();                      // the last result again (a page asked for it)

// A waveform, in the instrument's counts (the 8-bit samples times 16). With
// marks, those of the decoder's last review() or latchView() go with it.
// "live" ones are sent at most every 500 ms and only with room to spare.
bool wantsWave();                       // a page is listening: worth taking pictures for it
void wave(const uint16_t* s, uint16_t n, uint32_t hz, const char* src, const char* kind, bool marks);

// A command from the page: {"t":"cmd","id":7,"do":"snap",...}. The handler
// gets "do" and the whole line (for its other fields) and says whether it
// did it; Link answers with ack.
typedef bool (*Handler)(const char* what, const char* line);
void onCommand(Handler h);
// A field of a command line: a string (false if missing), or a number.
bool    field(const char* line, const char* key, char* out, uint8_t size);
int32_t fieldNum(const char* line, const char* key, int32_t missing);

// A small JSON writer.
struct Json {
    char*    start;
    char*    p;
    char*    end;
    bool     first;
    Json(char* buf, uint16_t size);
    Json& str(const char* key, const char* v);
    Json& num(const char* key, int32_t v);
    Json& flag(const char* key, bool v);
    const char* done();                 // closes the object
};

}

#endif
