#!/usr/bin/env python3
"""Reception brute, sans passer par le pilote C++.

Departage deux hypotheses : si les paquets arrivent ici mais pas via
bench_cc1101, le defaut est dans le pilote ; s'ils n'arrivent nulle part,
il est dans la configuration radio.
"""
import time
import spidev

READ, BURST = 0x80, 0x40
SRES, SRX, SIDLE, SFRX = 0x30, 0x34, 0x36, 0x3A
PARTNUM, VERSION, MARCSTATE, RSSI, RXBYTES, FIFO = 0x30, 0x31, 0x35, 0x34, 0x3B, 0x3F

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
    (0x0F, 0x76),
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

spi = spidev.SpiDev(); spi.open(0, 0); spi.max_speed_hz = 500000; spi.mode = 0
st = lambda c: spi.xfer2([c])
wr = lambda a, v: spi.xfer2([a, v])
rs = lambda a: spi.xfer2([a | READ | BURST, 0x00])[1]

st(SRES); time.sleep(0.1)
print(f"PARTNUM 0x{rs(PARTNUM):02X}  VERSION 0x{rs(VERSION):02X}")
for a, v in CONFIG:
    wr(a, v)
wr(0x3E, 0xC0)
st(SFRX); st(SRX); time.sleep(0.2)
print(f"MARCSTATE 0x{rs(MARCSTATE) & 0x1F:02X} (0x0D = RX)")
print("ecoute 25 s ...\n")

t0 = time.time()
ok = bad_crc = bad_len = 0
while time.time() - t0 < 25:
    rb = rs(RXBYTES)
    if rb & 0x80:
        print("debordement FIFO, purge")
        st(SIDLE); st(SFRX); st(SRX); time.sleep(0.02); continue
    if (rb & 0x7F) == 0:
        time.sleep(0.005); continue

    length = spi.xfer2([FIFO | READ, 0x00])[1]
    if length == 0 or length > 61:
        bad_len += 1
        st(SIDLE); st(SFRX); st(SRX); continue

    # La FIFO se remplit pendant que le paquet arrive : attendre qu'il soit
    # entierement la avant de lire, sinon on lit du vide a la suite des
    # donnees. Sans ligne GDO cablee, c'est a nous de le faire.
    deadline = time.time() + 0.2
    while True:
        dispo = rs(RXBYTES)
        if dispo & 0x80:            # debordement pendant l'attente
            dispo = -1
            break
        if (dispo & 0x7F) >= length + 2:
            break
        if time.time() > deadline:
            dispo = -1
            break
        time.sleep(0.001)
    if dispo == -1:
        bad_len += 1
        st(SIDLE); st(SFRX); st(SRX); continue

    data = spi.xfer2([FIFO | READ | BURST] + [0] * (length + 2))[1:]
    payload, rssi_raw, lqi_raw = data[:length], data[length], data[length + 1]

    if not (lqi_raw & 0x80):          # bit CRC_OK
        bad_crc += 1
        continue

    ok += 1
    dbm = (rssi_raw - 256) / 2 - 74 if rssi_raw >= 128 else rssi_raw / 2 - 74
    txt = ''.join(chr(b) if 32 <= b < 127 else '.' for b in payload)
    print(f"RX OK  {length:2d} o  {dbm:6.1f} dBm  lqi {lqi_raw & 0x7F:3d}  | {txt}")

print(f"\n{ok} paquet(s) valide(s), {bad_crc} CRC invalide(s), "
      f"{bad_len} longueur(s) aberrante(s)")
spi.close()
