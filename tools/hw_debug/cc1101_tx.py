#!/usr/bin/env python3
"""Emission de paquets, en verifiant que la puce atteint vraiment TX.

Le strobe STX peut etre accepte sans que l'emission ait lieu : FIFO vide,
sous-alimentation de la FIFO, ou refus. Ce script lit MARCSTATE juste apres
pour le savoir, au lieu de le supposer.
"""
import time
import spidev

READ, BURST = 0x80, 0x40
SRES, STX, SIDLE, SFTX, SFRX, SRX = 0x30, 0x35, 0x36, 0x3B, 0x3A, 0x34
PARTNUM, VERSION, MARCSTATE, TXBYTES, PKTSTATUS = 0x30, 0x31, 0x35, 0x3A, 0x38
FIFO, PATABLE = 0x3F, 0x3E

CONFIG = [
    (0x00, 0x2E),
    (0x02, 0x2E),
    (0x03, 0x07),
    (0x04, 0x12),
    (0x05, 0x34),
    (0x06, 0x3D),
    (0x07, 0x04),
    (0x08, 0x05),
    (0x0B, 0x06),
    (0x0C, 0x00),
    (0x0D, 0x10),
    (0x0E, 0xAF),
    (0x0F, 0x43),
    (0x10, 0xF8),
    (0x11, 0x83),
    (0x12, 0x13),
    (0x13, 0x62),
    (0x14, 0xF8),
    (0x15, 0x25),
    (0x17, 0x0F),
    (0x18, 0x18),
    (0x19, 0x16),
    (0x1A, 0x6C),
    (0x1B, 0x43),
    (0x1C, 0x40),
    (0x1D, 0x91),
    (0x21, 0x56),
    (0x22, 0x10),
    (0x23, 0xE9),
    (0x24, 0x2A),
    (0x25, 0x00),
    (0x26, 0x1F),
    (0x2C, 0x81),
    (0x2D, 0x35),
    (0x2E, 0x09),
]

MARC = {0x01: "IDLE", 0x0D: "RX", 0x13: "TX", 0x14: "TX_END",
        0x16: "TXFIFO_UNDERFLOW", 0x11: "RXFIFO_OVERFLOW", 0x12: "FSTXON"}

spi = spidev.SpiDev(); spi.open(0, 0); spi.max_speed_hz = 500000; spi.mode = 0
st = lambda c: spi.xfer2([c])
wr = lambda a, v: spi.xfer2([a, v])
rs = lambda a: spi.xfer2([a | READ | BURST, 0x00])[1]

st(SRES); time.sleep(0.1)
print(f"PARTNUM 0x{rs(PARTNUM):02X}  VERSION 0x{rs(VERSION):02X}")
for a, v in CONFIG:
    wr(a, v)
wr(PATABLE, 0xC0)
st(SFRX); st(SRX); time.sleep(0.2)

for i in range(12):
    msg = f"DRONE-{i:02d}".encode()
    st(SIDLE); st(SFTX)
    spi.xfer2([FIFO | BURST, len(msg)] + list(msg))
    avant = rs(TXBYTES) & 0x7F
    st(STX)

    atteint = None
    for _ in range(60):
        m = rs(MARCSTATE) & 0x1F
        if m in (0x13, 0x14):
            atteint = m
            break
        time.sleep(0.001)
    time.sleep(0.05)
    apres = rs(MARCSTATE) & 0x1F
    reste = rs(TXBYTES) & 0x7F

    print(f"{msg.decode():10s} FIFO avant {avant:2d} o  ->  "
          f"etat {MARC.get(atteint, 'JAMAIS TX') if atteint else 'JAMAIS TX':16s}"
          f" final {MARC.get(apres, hex(apres)):8s}  FIFO restant {reste}")
    time.sleep(0.4)

spi.close()
