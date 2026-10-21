#!/usr/bin/env python3
"""Trouve la frequence de reception optimale en comptant les paquets valides.

Le balayage RSSI ne resout pas mieux que la largeur du filtre. Ici on mesure
ce qui compte vraiment : combien de paquets passent le CRC a chaque
frequence. Le maximum de cette courbe est le vrai centre.
"""
import sys, time
import spidev

READ, BURST = 0x80, 0x40
SRES, SRX, SIDLE, SFRX = 0x30, 0x34, 0x36, 0x3A
RXBYTES, FIFO = 0x3B, 0x3F
XTAL = 26_000_000

CONFIG = [
    (0x00,0x2E),(0x02,0x2E),(0x03,0x07),(0x04,0x12),(0x05,0x34),(0x06,0x3D),
    (0x07,0x04),(0x08,0x05),(0x0B,0x06),(0x0C,0x00),(0x10,0xF8),(0x11,0x83),
    (0x12,0x13),(0x13,0x62),(0x14,0xF8),(0x15,0x25),(0x17,0x0F),(0x18,0x18),
    (0x19,0x16),(0x1A,0x6C),(0x1B,0x43),(0x1C,0x40),(0x1D,0x91),(0x21,0x56),
    (0x22,0x10),(0x23,0xE9),(0x24,0x2A),(0x25,0x00),(0x26,0x1F),(0x2C,0x81),
    (0x2D,0x35),(0x2E,0x09),
]

spi = spidev.SpiDev(); spi.open(0, 0); spi.max_speed_hz = 500000; spi.mode = 0
st = lambda c: spi.xfer2([c])
wr = lambda a, v: spi.xfer2([a, v])
rs = lambda a: spi.xfer2([a | READ | BURST, 0x00])[1]

st(SRES); time.sleep(0.1)
for a, v in CONFIG:
    wr(a, v)
wr(0x3E, 0xC0)

def essai(freq_mhz, duree):
    mot = int(round(freq_mhz * 1e6 * 65536 / XTAL))
    st(SIDLE)
    wr(0x0D, (mot >> 16) & 0xFF); wr(0x0E, (mot >> 8) & 0xFF); wr(0x0F, mot & 0xFF)
    st(SFRX); st(SRX)
    ok = ko = 0
    t0 = time.time()
    while time.time() - t0 < duree:
        rb = rs(RXBYTES)
        if rb & 0x80:
            st(SIDLE); st(SFRX); st(SRX); continue
        if (rb & 0x7F) == 0:
            time.sleep(0.002); continue
        n = spi.xfer2([FIFO | READ, 0x00])[1]
        if n == 0 or n > 61:
            st(SIDLE); st(SFRX); st(SRX); continue
        fin = time.time() + 0.2
        while (rs(RXBYTES) & 0x7F) < n + 2 and time.time() < fin:
            time.sleep(0.001)
        d = spi.xfer2([FIFO | READ | BURST] + [0] * (n + 2))[1:]
        if d[n + 1] & 0x80:
            ok += 1
        else:
            ko += 1
    return ok, ko

print("frequence   valides  CRC faux")
best = (None, -1)
f = 433.76
while f <= 433.845:
    ok, ko = essai(f, 6)
    barre = "#" * ok
    print(f"  {f:.3f}      {ok:3d}     {ko:3d}   {barre}")
    if ok > best[1]:
        best = (f, ok)
    f += 0.01

print(f"\nmeilleur : {best[0]:.3f} MHz avec {best[1]} paquets valides")
spi.close()
