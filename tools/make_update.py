#!/usr/bin/env python3
"""Puts the FNIRSI DSO-510 application and the tester together into one file
for the FNIRSI USB update (OK + power, the drive "DSO-510").

    python tools/make_update.py FNIRSI [TESTER] [--out DIR]

    FNIRSI  the FNIRSI application V2.2.1 (.zip or APP_*.bin)
    TESTER  the tester, as published (bin/APRE-510_v*.bin) or as built
            (default .pio/build/flash_apre/firmware.bin)
    DIR     where the update file goes (default update/); it gets the name of
            the FNIRSI file

The FNIRSI application is kept whole but for one word: the literal through
which its Reset_Handler jumps to __main now points at the tester's way in
(src/boot/BootStub.S). That goes on to the FNIRSI __main, whose address it
gets from here, unless RUN is held at power-on. The tester is placed where it
was linked (0x08086000), after the application, 0xFF in between.

Nothing is written to the unit. Going back: the FNIRSI file as it came, the
same way.
"""
import hashlib, os, struct, sys, zipfile, zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

APP_BASE = 0x08007000                   # where the FNIRSI bootloader puts the application
# The FNIRSI bootloader takes the flash size from the chip and writes both
# banks: the DSO-510 has 1 MB (read from the chip, 2026-11-07), and V2.2.1
# itself reaches past 0x08080000, into the second bank.
FLASH_END = 0x08100000
BANK2 = 0x08080000
# The versions this tester has been tried with on a unit, by SHA-256.
TRIED = {
    '11d036e7e463a26f596a53dc1e5ee070aacf288cbde6678710dea2dcace9adec': 'V2.2.1 (as on the unit)',
}


def fail(msg):
    raise SystemExit('make_update: ' + msg)


def read_fnirsi(path):
    """The application's bytes and its file name."""
    if path.lower().endswith('.zip'):
        with zipfile.ZipFile(path) as z:
            names = [n for n in z.namelist()
                     if os.path.basename(n).upper().startswith('APP_') and n.lower().endswith('.bin')]
            if len(names) != 1:
                fail('%s: expected one APP_*.bin in it, found %s' % (path, names or 'none'))
            return z.read(names[0]), os.path.basename(names[0])
    return open(path, 'rb').read(), os.path.basename(path)


def find_main(app):
    """The literal through which the Reset_Handler jumps to __main: in the
    first instructions of the handler, LDR r0,[pc,#n] followed by BX r0."""
    if b'T232BOOT' in app:
        fail('this is a file made with this script, not the FNIRSI one')
    sp, reset = struct.unpack_from('<II', app, 0)
    if sp & 0xFFF00000 != 0x20000000:
        fail('the file does not start with a vector table (stack 0x%08X)' % sp)
    if not (reset & 1) or not (APP_BASE <= reset < APP_BASE + len(app)):
        fail('the reset vector 0x%08X is not in the application' % reset)
    pc = (reset & ~1) - APP_BASE
    for at in range(pc, min(pc + 0x80, len(app) - 4), 2):
        ldr, bx = struct.unpack_from('<HH', app, at)
        if ldr & 0xFF00 == 0x4800 and bx == 0x4700:
            lit = ((at + 4) & ~3) + (ldr & 0xFF) * 4
            main = struct.unpack_from('<I', app, lit)[0]
            if not (main & 1) or not (APP_BASE <= main < APP_BASE + len(app)):
                fail('the jump at 0x%08X goes to 0x%08X, not into the application'
                     % (APP_BASE + at, main))
            return lit, main
    fail('no jump to __main found in the Reset_Handler at 0x%08X' % reset)


def read_tester(tester):
    """Where the tester was linked, its way in, and the offset of the word
    for the FNIRSI __main (the record t232_link in src/boot/BootStub.S)."""
    at = tester.find(b'T232BOOT')
    if at < 0 or tester.find(b'T232BOOT', at + 1) >= 0 or at % 4:
        fail('the tester has no (single) T232BOOT record: not a flash_apre build')
    link, boot, slot = struct.unpack_from('<III', tester, at + 8)
    base = link - at
    sp, reset = struct.unpack_from('<II', tester, 0)
    if sp & 0xFFF00000 != 0x20000000 or not (base <= reset < base + len(tester)):
        fail('the tester does not start with its vector table')
    if not (base <= boot < base + len(tester)):
        fail('the tester\'s way in is not inside it')
    if slot != 0xFFFFFFFF:
        fail('the tester has been patched already')
    # after the __main word, "IMG1" and the image's length and CRC-32, which
    # the tester checks at start (older builds have no such words)
    image = at + 24 if tester[at + 20:at + 24] == b'IMG1' else None
    if image is not None and struct.unpack_from('<II', tester, image) != (0xFFFFFFFF, 0xFFFFFFFF):
        fail('the tester has been patched already')
    return base, boot, at + 16, image


def build(app, tester):
    lit, main = find_main(app)
    base, boot, slot, image = read_tester(tester)
    app_end = APP_BASE + len(app)
    if app_end > base:
        need = (app_end + 0xFFF) & ~0xFFF
        fail('the FNIRSI application ends at 0x%08X, past the tester at 0x%08X: in platformio.ini '
             'set board_build.flash_offset to 0x%X and board_upload.offset_address to 0x%08X, '
             'build again (pio run) and run this again' % (app_end, base, need - 0x08000000, need))
    if base + len(tester) > FLASH_END:
        fail('the image would end at 0x%08X, past 0x%08X' % (base + len(tester), FLASH_END))

    img = bytearray(app)
    img += b'\xFF' * (base - app_end)
    img += tester
    struct.pack_into('<I', img, lit, boot | 1)
    struct.pack_into('<I', img, base - APP_BASE + slot, main)
    patched = set(range(slot, slot + 4))
    if image is not None:           # the CRC of the tester as built, those three words 0xFFFFFFFF
        struct.pack_into('<II', img, base - APP_BASE + image, len(tester), zlib.crc32(tester) & 0xFFFFFFFF)
        patched |= set(range(image, image + 8))

    # Of the application, only the literal; of the tester, only the word.
    changed = [i for i in range(len(app)) if img[i] != app[i]]
    if not changed or not set(changed) <= set(range(lit, lit + 4)):
        fail('unexpected changes in the application: %s' % changed[:8])
    t = bytes(img[base - APP_BASE:])
    changed = [i for i in range(len(tester)) if t[i] != tester[i]]
    if not set(changed) <= patched:
        fail('unexpected changes in the tester: %s' % changed[:8])
    return bytes(img), {'literal': APP_BASE + lit, 'main': main, 'boot': boot, 'base': base,
                        'app_end': app_end, 'end': base + len(tester),
                        'crc': zlib.crc32(tester) & 0xFFFFFFFF if image is not None else None}


def main(argv):
    args = list(argv)
    out_dir = os.path.join(ROOT, 'update')
    if '--out' in args:
        i = args.index('--out')
        out_dir = args[i + 1]
        del args[i:i + 2]
    if not args or args[0].startswith('-'):
        raise SystemExit(__doc__)
    app, name = read_fnirsi(args[0])
    tester_path = args[1] if len(args) > 1 else os.path.join(ROOT, '.pio', 'build', 'flash_apre', 'firmware.bin')
    tester = open(tester_path, 'rb').read()

    img, info = build(app, tester)
    sha = hashlib.sha256(app).hexdigest()
    print('FNIRSI app   0x%08X..0x%08X  %d bytes  sha256 %s' % (APP_BASE, info['app_end'], len(app), sha[:16]))
    if sha in TRIED:
        print('             %s: tried with this tester' % TRIED[sha])
    else:
        print('             NOT a version this tester has been tried with: the checks passed,')
        print('             but nobody has started it on a unit yet. Keep the FNIRSI file to go back.')
    print('jump         0x%08X: 0x%08X (__main) -> 0x%08X (tester)' % (info['literal'], info['main'], info['boot'] | 1))
    print('tester       0x%08X..0x%08X  %d bytes  (%s)' % (info['base'], info['end'], len(tester), tester_path))
    if info['crc'] is not None:
        print('             CRC-32 %08X, checked by the tester at start ("ok" in its menu)' % info['crc'])
    if info['end'] > BANK2:
        print('             past 0x%08X: the second bank, for a unit with 1 MB of flash only' % BANK2)

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, name)
    open(out, 'wb').write(img)
    print('update file  %s  %d bytes  sha256 %s' % (out, len(img), hashlib.sha256(img).hexdigest()[:16]))
    return out, img, app, sha


if __name__ == '__main__':
    main(sys.argv[1:])
