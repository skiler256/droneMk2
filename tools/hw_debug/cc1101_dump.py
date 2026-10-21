#!/usr/bin/env python3
"""Applique la configuration radio du pilote puis relit les registres.

Repond a une seule question : ce qu'on croit ecrire est-il vraiment dans la
puce ? Affiche aussi MARCSTATE (etat reel de la radio) et RSSI (bruit de
fond, qui prouve ou non que l'etage de reception vit).

Usage : sudo python3 cc1101_dump.py [/dev/spidev0.0]
"""
import sys
import time

import spidev

READ = 0x80
BURST = 0x40

SRES, SRX, SIDLE, SFRX = 0x30, 0x34, 0x36, 0x3A
PARTNUM, VERSION, MARCSTATE, RSSI, PKTSTATUS = 0x30, 0x31, 0x35, 0x34, 0x38
PATABLE = 0x3E

# Copie conforme de src/SystemMonitoring/CC1101_Driver.cpp
CONFIG = [
    (0x00, 0x2E), (0x02, 0x2E), (0x03, 0x07), (0x06, 0x3D), (0x07, 0x04),
    (0x08, 0x05), (0x0B, 0x06), (0x0C, 0x00), (0x0D, 0x10), (0x0E, 0xB0),
    (0x0F, 0x71), (0x10, 0xCA), (0x11, 0x83), (0x12, 0x13), (0x13, 0x22),
    (0x14, 0xF8), (0x15, 0x35), (0x17, 0x3F), (0x18, 0x18), (0x19, 0x16),
    (0x1A, 0x6C), (0x1B, 0x43), (0x1C, 0x40), (0x1D, 0x91), (0x21, 0x56),
    (0x22, 0x10), (0x23, 0xE9), (0x24, 0x2A), (0x25, 0x00), (0x26, 0x1F),
    (0x2C, 0x81), (0x2D, 0x35), (0x2E, 0x09),
]
PA_POWER = 0xC0

NAMES = {
    0x07: "PKTCTRL1", 0x08: "PKTCTRL0", 0x0D: "FREQ2", 0x0E: "FREQ1",
    0x0F: "FREQ0", 0x10: "MDMCFG4", 0x11: "MDMCFG3", 0x12: "MDMCFG2",
    0x13: "MDMCFG1", 0x15: "DEVIATN", 0x17: "MCSM1", 0x18: "MCSM0",
    0x22: "FREND0",
}

MARC = {
    0x00: "SLEEP", 0x01: "IDLE", 0x0D: "RX", 0x0E: "RX_END",
    0x11: "RXFIFO_OVERFLOW", 0x13: "TX", 0x14: "TX_END",
    0x16: "TXFIFO_UNDERFLOW", 0x12: "FSTXON",
}


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/spidev0.0"
    bus, cs = (int(c) for c in dev.rsplit("spidev", 1)[1].split("."))

    spi = spidev.SpiDev()
    spi.open(bus, cs)
    spi.max_speed_hz = 500000
    spi.mode = 0

    def strobe(cmd):
        spi.xfer2([cmd])

    def write(addr, value):
        spi.xfer2([addr, value])

    def read(addr):
        return spi.xfer2([addr | READ, 0x00])[1]

    def read_status(addr):
        return spi.xfer2([addr | READ | BURST, 0x00])[1]

    strobe(SRES)
    time.sleep(0.1)

    print(f"PARTNUM 0x{read_status(PARTNUM):02X}   "
          f"VERSION 0x{read_status(VERSION):02X}")

    for addr, value in CONFIG:
        write(addr, value)
    write(PATABLE, PA_POWER)

    print("\n--- registres relus (attendu / lu) ---")
    bad = 0
    for addr, value in CONFIG:
        got = read(addr)
        flag = "" if got == value else "   <<< DIFFERENT"
        if got != value:
            bad += 1
        name = NAMES.get(addr, "")
        print(f"  0x{addr:02X} {name:9s} {value:02X} / {got:02X}{flag}")

    pa = read(PATABLE)
    print(f"  PATABLE        {PA_POWER:02X} / {pa:02X}"
          f"{'   <<< DIFFERENT' if pa != PA_POWER else ''}")
    if pa != PA_POWER:
        bad += 1

    print(f"\n{bad} registre(s) divergent(s) sur {len(CONFIG) + 1}")

    strobe(SFRX)
    strobe(SRX)
    time.sleep(0.3)

    marc = read_status(MARCSTATE) & 0x1F
    print(f"\nMARCSTATE : 0x{marc:02X}  ({MARC.get(marc, 'inconnu')})")
    print("            attendu RX (0x0D) apres le strobe SRX")

    print("\nRSSI au repos (bruit de fond) :")
    for _ in range(5):
        raw = read_status(RSSI)
        dbm = (raw - 256) / 2 - 74 if raw >= 128 else raw / 2 - 74
        print(f"  brut 0x{raw:02X}  ->  {dbm:6.1f} dBm")
        time.sleep(0.2)
    print("  un plancher plausible se situe entre -90 et -110 dBm")

    print(f"\nPKTSTATUS : 0x{read_status(PKTSTATUS):02X}")
    spi.close()


if __name__ == "__main__":
    main()
