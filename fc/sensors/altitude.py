# file: sonic_gpiozero_meters.py
from gpiozero import DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep

# BCM numbering: echo=24 (Pin 18), trigger=23 (Pin 16)
# max_distance in meters; HC-SR05/VMA306 is up to ~4–4.5 m in ideal conditions.
factory = LGPIOFactory()
sensor = DistanceSensor(echo=24, trigger=23, max_distance=4.5, threshold_distance=0.2, pin_factory=factory)

def main():
    try:
        while True:
            d = sensor.distance  # returns 0.0–1.0 normalized to max_distance
            abs_m = d * sensor.max_distance
            print(f"{abs_m:.3f} m")
            sleep(0.1)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    