#!/usr/bin/env python3
"""Runs ON the board. Drives the WS2812-type eye LED through SPI1 MOSI, for bring-up.

    python3 eye-test.py               cycle red, green, blue, white, off (checks colour order)
    python3 eye-test.py 255 0 0       hold one colour (R G B, 0-255) until Ctrl-C
    python3 eye-test.py --off

Each LED bit is three SPI bits at 2.4 MHz: 100 for a 0 (0.42 us high), 110 for a 1 (0.83 us
high), inside the WS2812's 0.4/0.8 us +-0.15 us windows. The frame is padded with zero bytes on
both sides so the line sits low for well over the 280 us reset time before and after.

Standard library only (no py-spidev on the image): the clock rate is set with the spidev ioctl
and the frame goes out with one write(), which the driver sends as one transfer.
"""

import argparse
import fcntl
import os
import struct
import sys
import time

DEV = "/dev/spidev-eye"
HZ = 2_400_000
SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
# 300 us of low at 2.4 MHz is 90 bytes.
PAD = bytes(96)


def encode(r, g, b, order):
    """One LED's 24 bits in wire order, three SPI bits per bit, MSB first."""
    vals = {"r": r, "g": g, "b": b}
    bits = 0
    for c in order:
        for i in range(7, -1, -1):
            bits = (bits << 3) | (0b110 if vals[c] >> i & 1 else 0b100)
    return bits.to_bytes(9, "big")


def show(fd, rgb, order):
    os.write(fd, PAD + encode(*rgb, order) + PAD)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("rgb", nargs="*", type=int)
    p.add_argument("--off", action="store_true")
    p.add_argument("--order", default="grb", help="wire colour order, grb for WS2812 (default)")
    p.add_argument("--scale", type=float, default=0.15, help="brightness cap, 0-1 (default 0.15)")
    a = p.parse_args()

    fd = os.open(DEV, os.O_WRONLY)
    fcntl.ioctl(fd, SPI_IOC_WR_MODE, struct.pack("B", 0))
    fcntl.ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", HZ))
    dim = lambda c: [round(v * a.scale) for v in c]

    if a.off:
        show(fd, (0, 0, 0), a.order)
        return
    if len(a.rgb) == 3:
        show(fd, dim(a.rgb), a.order)
        print("holding", a.rgb, "- Ctrl-C to turn off")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            show(fd, (0, 0, 0), a.order)
        return
    if a.rgb:
        sys.exit("give R G B, or nothing for the cycle")

    for name, c in [("red", (255, 0, 0)), ("green", (0, 255, 0)), ("blue", (0, 0, 255)),
                    ("white", (255, 255, 255)), ("off", (0, 0, 0))]:
        print(name, flush=True)
        show(fd, dim(c), a.order)
        time.sleep(2)


if __name__ == "__main__":
    main()
