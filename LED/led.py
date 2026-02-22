
import board
import neopixel
import time
import colorsys

NUM_LEDS = 300
PIN = board.D25

pixels = neopixel.NeoPixel(PIN, NUM_LEDS, brightness=0.5, auto_write=False)

def alarm_alert():
     for _ in range(5):  # number of flashes
        pixels.fill((255, 0, 0))  # RED ON
        pixels.show()
        time.sleep(0.2)

        pixels.fill((0, 0, 0))  # OFF
        pixels.show()
        time.sleep(0.2)

def disco_cycle(wait=0.01):
    for j in range(256):
        for i in range(NUM_LEDS):
            hue = ((i / NUM_LEDS) + j / 256) % 1.0
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            pixels[i] = (int(r * 255), int(g * 255), int(b * 255))
        pixels.show()
        time.sleep(wait)