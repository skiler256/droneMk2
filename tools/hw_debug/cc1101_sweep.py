#!/usr/bin/env python3
"""Balaie la frequence et mesure le RSSI, pour localiser une porteuse.

Sert a departager deux hypotheses quand deux radios fonctionnelles ne
s'entendent pas : soit la porteuse est bien la ou on l'attend et le
probleme est ailleurs, soit elle est decalee — typiquement parce que les
deux modules n'ont pas le meme quartz.

Un quartz de 27 MHz la ou le code suppose 26 decale la porteuse de ~17 MHz
vers le haut : 433.92 devient 450.6.

Usage : sudo python3 cc1101_sweep.py [debut_MHz] [fin_MHz] [pas_MHz]
Defaut : 420 a 465 MHz par pas de 0.2 MHz
"""
import sys
import time

import spidev

READ, BURST = 0x80, 0x40
SRES, SRX, SIDLE, SFRX = 0x30, 0x34, 0x36, 0x3A
PARTNUM, VERSION, MARCSTATE, RSSI = 0x30, 0x31, 0x35, 0x34
XTAL_HZ = 26_000_000

# Meme configuration que le pilote, sauf FREQ qui est balayee.
CONFIG = [
    (0x00, 0x2E), (0x02, 0x2E), (0x03, 0x07), (0x06, 0x3D), (0x07, 0x04),
    (0x08, 0x05), (0x0B, 0x06), (0x0C, 0x00), (0x10, 0xF8), (0x11, 0x83),
    (0x12, 0x13), (0x13, 0x22), (0x14, 0xF8), (0x15, 0x35), (0x17, 0x3F),
    (0x18, 0x18), (0x19, 0x16), (0x1A, 0x6C), (0x1B, 0x43), (0x1C, 0x40),
    (0x1D, 0x91), (0x21, 0x56), (0x22, 0x10), (0x23, 0xE9), (0x24, 0x2A),
    (0x25, 0x00), (0x26, 0x1F), (0x2C, 0x81), (0x2D, 0x35), (0x2E, 0x09),
]


def main():
    start = float(sys.argv[1]) if len(sys.argv) > 1 else 420.0
    stop = float(sys.argv[2]) if len(sys.argv) > 2 else 465.0
    step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500000
    spi.mode = 0

    def strobe(c):
        spi.xfer2([c])

    def write(a, v):
        spi.xfer2([a, v])

    def read_status(a):
        return spi.xfer2([a | READ | BURST, 0x00])[1]

    strobe(SRES)
    time.sleep(0.1)
    if read_status(PARTNUM) != 0x00 or read_status(VERSION) != 0x14:
        print("puce non reconnue, arret")
        return

    for a, v in CONFIG:
        write(a, v)

    print(f"balayage {start}-{stop} MHz par pas de {step} MHz")
    print("(mesure du plancher d'abord, puis recherche d'un pic)\n")

    results = []
    f = start
    while f <= stop + 1e-9:
        freq_word = int(round(f * 1e6 * 65536 / XTAL_HZ))
        strobe(SIDLE)
        write(0x0D, (freq_word >> 16) & 0xFF)
        write(0x0E, (freq_word >> 8) & 0xFF)
        write(0x0F, freq_word & 0xFF)
        strobe(SFRX)
        strobe(SRX)
        time.sleep(0.02)  # laisse l'AGC se stabiliser

        peak = -200.0
        for _ in range(4):
            raw = read_status(RSSI)
            dbm = (raw - 256) / 2 - 74 if raw >= 128 else raw / 2 - 74
            peak = max(peak, dbm)
            time.sleep(0.005)
        results.append((f, peak))
        f += step

    floor = sorted(r[1] for r in results)[len(results) // 2]
    print(f"plancher median : {floor:.1f} dBm\n")

    strong = [r for r in results if r[1] > floor + 12]
    if not strong:
        print("AUCUN pic detecte : rien n'emet dans la plage balayee.")
    else:
        strong.sort(key=lambda r: -r[1])
        print("pics detectes (plus de 12 dB au-dessus du plancher) :")
        for freq, dbm in strong[:12]:
            bar = "#" * max(1, int((dbm - floor) / 2))
            print(f"  {freq:7.2f} MHz  {dbm:7.1f} dBm  {bar}")
        best = strong[0]
        print(f"\nplus fort : {best[0]:.2f} MHz a {best[1]:.1f} dBm")
        ecart = best[0] - 433.92
        print(f"ecart par rapport a 433.92 MHz : {ecart:+.2f} MHz")
        if abs(ecart) > 1.0:
            implique = 26.0 * best[0] / 433.92
            print(f"  -> compatible avec un quartz de {implique:.2f} MHz "
                  f"en face")

    spi.close()


if __name__ == "__main__":
    main()
