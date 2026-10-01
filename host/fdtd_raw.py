#!/usr/bin/env python3
"""
fdtd_raw.py -- dump the raw bytes the board sends, with no framing
               assumptions at all.

When a capture script produces nothing, the first question is not "why
is the protocol wrong" but "is anything coming out of the pin".  This
answers that in two seconds, and it decodes frames only as a secondary
step, so a framing mistake cannot hide the fact that bytes are or are
not arriving.

    python3 host/fdtd_raw.py /dev/ttyUSB1            # field mode
    python3 host/fdtd_raw.py /dev/ttyUSB1 --diag     # source mode

Prints: the raw hex, then the sync-byte spacing (which reveals the true
frame length regardless of what the frame is supposed to be), then the
decoded frames.
"""

import sys
import time
import collections

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial --break-system-packages")

args = [a for a in sys.argv[1:] if not a.startswith("-")]
PORT = args[0] if args else \
    "/dev/serial/by-id/usb-SIPEED_USB_Debugger_2025030317-if01-port0"
DIAG = "--diag" in sys.argv

# --long captures a full source period.  The default two-second window
# covers only about one sixty-fourth of one, which is not enough to see
# the negative half of the source LUT -- the very thing in question.
LONG = "--long" in sys.argv

SYNC = 0xA5
NBYTES = 60000 if LONG else 400
WINDOW = 8.0 if LONG else 2.0


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print(f"port {PORT}")
    ser.write(b"R")
    time.sleep(0.1)
    ser.write(b"D" if DIAG else b"F")
    time.sleep(0.1)
    ser.write(b"G")
    print("sent R, %s, G -- listening for %.0f s\n"
          % ("D" if DIAG else "F", WINDOW))

    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < WINDOW and len(buf) < NBYTES:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
    ser.write(b"S")
    ser.close()

    if not buf:
        print("NOTHING RECEIVED.")
        print("  The board is not transmitting at all.  Check that led0 is")
        print("  blinking (design alive), that this is the right port, and")
        print("  that the bitstream actually flashed.")
        return

    print(f"received {len(buf)} bytes in {time.time()-t0:.2f} s")
    print(f"  ({len(buf)/(time.time()-t0):.0f} bytes/s; one 5-byte frame per")
    print(f"   step at ~1930 steps/s would be ~9650 bytes/s)\n")

    print("first 80 bytes:")
    for i in range(0, min(80, len(buf)), 16):
        print("   " + " ".join(f"{b:02x}" for b in buf[i:i+16]))
    print()

    # Sync spacing tells you the REAL frame length, whatever it is meant
    # to be.  If this is not 5, the frame format and the host disagree.
    pos = [i for i, b in enumerate(buf) if b == SYNC]
    if len(pos) < 3:
        print(f"only {len(pos)} sync bytes (0xA5) found -- the stream is not")
        print("framed as expected.  Bytes are arriving, so the UART works;")
        print("the frame format is what disagrees.")
        return
    gaps = collections.Counter(b - a for a, b in zip(pos, pos[1:]))
    print("spacing between 0xA5 bytes:", gaps.most_common(5))
    stride = gaps.most_common(1)[0][0]
    print(f"  -> frame length looks like {stride} bytes "
          f"({'correct' if stride == 5 else 'WRONG, expected 5'})\n")

    print("decoded frames (sym, state, step):")
    prev = None
    for p in pos[:20]:
        if p + 4 >= len(buf):
            break
        sym = buf[p+1] - 256 if buf[p+1] >= 128 else buf[p+1]
        st = (buf[p+2] << 8) | buf[p+3]
        if st >= 32768:
            st -= 65536
        step = buf[p+4]
        flag = ""
        if prev is not None:
            d = (step - prev) % 256
            if d == 0:
                flag = "  <-- DUPLICATE"
            elif d > 1:
                flag = f"  <-- {d-1} DROPPED"
        prev = step
        print(f"   step {step:3d}  sym {sym:+d}  "
              f"{'src' if DIAG else 'state'} {st}{flag}")

    # High-byte census.  The payload is int16 big-endian, so a negative
    # sample MUST arrive with a high byte of 0xff.  If every frame shows
    # 0x00 there, the top half of the payload is being lost between
    # probe_state and the wire, and that alone would explain both a
    # field that never goes negative and a state that only climbs.
    his = collections.Counter(buf[p+2] for p in pos if p + 4 < len(buf))
    print("state HIGH byte census:", his.most_common(6))
    if len(his) == 1 and 0x00 in his:
        print("  -> high byte is ALWAYS 0x00 over %d frames." % sum(his.values()))
        print("     Either no negative sample occurred in this window,")
        print("     or the high half of the payload never reaches the wire.")
    elif 0xff in his:
        print("  -> 0xff present: negative payloads ARE getting through.")
    print()

    if DIAG:
        vals = set()
        for p in pos:
            if p + 4 < len(buf):
                st = (buf[p+2] << 8) | buf[p+3]
                vals.add(st - 65536 if st >= 32768 else st)
        print(f"\nsource values seen: {sorted(vals)}")
        if len(vals) <= 1:
            print("VERDICT: the source is CONSTANT -- the ROM did not initialise.")
        else:
            print("VERDICT: the source ROM is producing more than one value.")
            print("         Let the full `make diag` run to check the waveform.")


if __name__ == "__main__":
    main()
