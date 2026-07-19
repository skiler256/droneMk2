import serial
import math
import os
import time

PORT = "/dev/ttyAMA2"   # à adapter selon ton Pi
BAUD = 230400

WIDTH = 80
HEIGHT = 40

ser = serial.Serial(PORT, BAUD, timeout=1)

points = []

def clear():
    os.system("clear")

def draw():
    grid = [[" " for _ in range(WIDTH)] for _ in range(HEIGHT)]

    cx = WIDTH // 2
    cy = HEIGHT // 2

    # centre
    grid[cy][cx] = "+"

    scale = 5000  # distance max affichée en mm

    for angle, dist in points:
        if dist <= 0 or dist > scale:
            continue

        a = math.radians(angle)

        x = int(cx + math.sin(a) * dist / scale * (WIDTH//2))
        y = int(cy - math.cos(a) * dist / scale * (HEIGHT//2))

        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            grid[y][x] = "."

    print("\n".join("".join(row) for row in grid))


def read_ld06():
    global points

    buffer = bytearray()

    while True:
        data = ser.read(512)
        if data:
            buffer += data

        # recherche entête LD06
        while len(buffer) >= 47:
            if buffer[0] != 0x54:
                buffer.pop(0)
                continue

            if buffer[1] != 0x2C:
                buffer.pop(0)
                continue

            packet = buffer[:47]
            buffer = buffer[47:]

            # parsing simplifié
            start_angle = packet[4] | (packet[5] << 8)
            end_angle   = packet[42] | (packet[43] << 8)

            start_angle /= 100
            end_angle /= 100

            pts = []

            for i in range(12):
                offset = 6 + i * 3

                dist = packet[offset] | (packet[offset+1] << 8)

                angle = start_angle + (end_angle - start_angle) * i / 11

                pts.append((angle, dist))

            points.extend(pts)

            # garder seulement le dernier tour
            if len(points) > 500:
                points = points[-500:]

            clear()
            draw()


try:
    print("LD06 démarré...")
    read_ld06()

except KeyboardInterrupt:
    print("\nArrêt")

finally:
    ser.close()