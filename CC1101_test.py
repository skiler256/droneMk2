import spidev
import time

spi = spidev.SpiDev()
spi.open(0,0)

spi.max_speed_hz = 500000
spi.mode = 0

def strobe(cmd):
    spi.xfer2([cmd])

def read_status(addr):
    return spi.xfer2([addr | 0xC0,0x00])[1]


print("Reset...")
strobe(0x30)
time.sleep(0.1)

for i in range(5):
    print(
        "PARTNUM:",
        hex(read_status(0x30)),
        "VERSION:",
        hex(read_status(0x31))
    )
    time.sleep(0.1)

def read_reg(addr):
    return spi.xfer2([addr | 0x80, 0x00])[1]


f2 = read_reg(0x0D)
f1 = read_reg(0x0E)
f0 = read_reg(0x0F)

print(hex(f2), hex(f1), hex(f0))
spi.close()