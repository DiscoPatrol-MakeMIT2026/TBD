"""Altitude / distance helper using an ultrasonic rangefinder.

Use case:
- Measure distance (altitude) from ground using an HC-SR04-like ultrasonic
  sensor connected to a Linux SBC (Raspberry Pi) via the `lgpio` library.

This module exposes a small, testable API:
- `init_altitude()` -> returns a lgpio chip handle
- `get_altitude(h)` -> returns distance in meters or `None` on timeout
- `close_altitude(h)` -> closes the chip handle

The module also includes a small interactive demo when run as a script.
"""

import time
from typing import Optional

import lgpio

# Default GPIO configuration (BCM numbering)
"""Altitude / distance helper using an ultrasonic rangefinder.

Use case:
- Measure distance (altitude) from ground using an HC-SR04-like ultrasonic
  sensor connected to a Linux SBC (Raspberry Pi) via the `lgpio` library.

This module exposes a small, testable API:
- `init_altitude()` -> returns a lgpio chip handle
- `get_altitude(h)` -> returns distance in meters or `None` on timeout
- `close_altitude(h)` -> closes the chip handle

The module also includes a small interactive demo when run as a script.
"""

import time
from typing import Optional

import lgpio

# Default GPIO configuration (BCM numbering)
"""Altitude / distance helper using an ultrasonic rangefinder.

Use case:
- Measure distance (altitude) from ground using an HC-SR04-like ultrasonic
  sensor connected to a Linux SBC (Raspberry Pi) via the `lgpio` library.

This module exposes a small, testable API:
- `init_altitude()` -> returns a lgpio chip handle
- `get_altitude(h)` -> returns distance in meters or `None` on timeout
- `close_altitude(h)` -> closes the chip handle

The module also includes a small interactive demo when run as a script.
"""

import time
from typing import Optional

import lgpio

# Default GPIO configuration (BCM numbering)
CHIP = 0
TRIG = 23
ECHO = 24
SOUND = 343.0  # speed of sound in m/s

# Timeout for waiting for echo edges (seconds). ~30ms => ~5m range
TIMEOUT_S = 0.03


def init_altitude(chip: int = CHIP, trig: int = TRIG, echo: int = ECHO) -> int:
    """Open the gpiochip and claim TRIG as output and ECHO as input.

    Returns the lgpio chip handle which must be passed to other functions.
    """
    h = lgpio.gpiochip_open(chip)
    lgpio.gpio_claim_output(h, trig, 0)
    lgpio.gpio_claim_input(h, echo)
    # Let sensor settle
    time.sleep(0.05)
    return h


def get_altitude(h: int, trig: int = TRIG, echo: int = ECHO, timeout_s: float = TIMEOUT_S) -> Optional[float]:
    """Trigger the sensor and measure the distance.

    Returns the distance in meters, or `None` if a timeout occurred.
    This function is non-blocking beyond the configured timeouts.
    """
    # 10 µs trigger pulse
    lgpio.gpio_write(h, trig, 1)
    t_now = time.perf_counter()
    while (time.perf_counter() - t_now) < 10e-6:
        pass
    lgpio.gpio_write(h, trig, 0)

    # Wait for rising edge
    t0 = time.perf_counter()
    start = None
    while (time.perf_counter() - t0) < timeout_s:
        if lgpio.gpio_read(h, echo):
            start = time.perf_counter()
            break

    if start is None:
        return None

    # Wait for falling edge
    end = None
    while (time.perf_counter() - start) < timeout_s:
        if not lgpio.gpio_read(h, echo):
            end = time.perf_counter()
            break

    if end is None:
        return None

    pulse = end - start
    distance_m = (SOUND * pulse) / 2.0
    return distance_m


def close_altitude(h: int) -> None:
    """Close the gpiochip handle."""
    try:
        lgpio.gpiochip_close(h)
    except Exception:
        # Best-effort cleanup; ignore errors on close
        pass


if __name__ == "__main__":
    # Simple demo loop for manual testing
    h = init_altitude()
    try:
        print("Press Ctrl-C to stop; measuring distance...")
        while True:
            d = get_altitude(h)
            if d is None:
                print("timeout")
            else:
                print(f"{d:.3f} m")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("exiting")
    finally:
        close_altitude(h)
    """Close the gpiochip handle."""
    try:
        lgpio.gpiochip_close(h)
    except Exception:
        # Best-effort cleanup; ignore errors on close
        pass


if __name__ == "__main__":
    # Simple demo loop for manual testing
    h = init_altitude()
    try:
        print("Press Ctrl-C to stop; measuring distance...")
        while True:
            d = get_altitude(h)
            if d is None:
                print("timeout")
            else:
                print(f"{d:.3f} m")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("exiting")
    finally:
        close_altitude(h)
    """Close the gpiochip handle."""
    try:
        lgpio.gpiochip_close(h)
    except Exception:
        # Best-effort cleanup; ignore errors on close
        pass


if __name__ == "__main__":
    # Simple demo loop for manual testing
    h = init_altitude()
    try:
        print("Press Ctrl-C to stop; measuring distance...")
        while True:
            d = get_altitude(h)
            if d is None:
                print("timeout")
            else:
                print(f"{d:.3f} m")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("exiting")
    finally:
        close_altitude(h)