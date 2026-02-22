import smbus2
import time

bus  = smbus2.SMBus(0)
ADDR = 0x42

def send_motors(speeds: list[int]):
    """
    Send throttle to all 4 motors.
    speeds: list of 4 ints, each 0–100%
      0   → XIAO auto hard-stops that motor
      1–100 → maps to 1000–2000µs PWM
    """
    if len(speeds) != 4:
        raise ValueError(f"Expected 4 speeds, got {len(speeds)}")

    pkt = [max(0, min(100, int(s))) for s in speeds]
    bus.write_i2c_block_data(ADDR, 0, pkt)

def read_motors() -> list[int]:
    """Returns current motor % reported by XIAO."""
    raw = bus.read_i2c_block_data(ADDR, 0, 4)
    return list(raw)

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Wait for XIAO arm sequence to complete (ARM_DELAY_MS + ~5s arm sequence)
    print("Waiting for XIAO to arm...")
    time.sleep(10)
    print("Ready.")

    try:
        # Spin up
        send_motors([20, 20, 20, 20])
        time.sleep(2)

        # Mixed throttle
        send_motors([30, 25, 30, 25])
        time.sleep(2)

        # Ramp up
        for t in range(20, 51, 5):
            send_motors([t, t, t, t])
            print(f"Status: {read_motors()}")
            time.sleep(0.5)

        # Ramp down
        for t in range(50, 19, -5):
            send_motors([t, t, t, t])
            time.sleep(0.5)

        # Stop — XIAO auto hard-stops all motors
        send_motors([0, 0, 0, 0])

    except KeyboardInterrupt:
        print("\n[EMERGENCY STOP]")
        send_motors([0, 0, 0, 0])

    finally:
        bus.close()