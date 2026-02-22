#!/usr/bin/env python3
import time
import math
import os
from smbus2 import SMBus

# -------------------------
# I2C / MPU6050 Parameters
# -------------------------
# IMPORTANT: On Raspberry Pi, physical pin 3 = SDA1 and pin 5 = SCL1 for I2C-1.
# Using SMBus(1) ensures we use those exact pins.
I2C_BUS = 1
MPU_ADDR = 0x68  # Use 0x69 if AD0 is pulled high

# MPU-6050 registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
WHO_AM_I     = 0x75

# -------------------------
# Module-level state
# -------------------------
_bus = None
_inited = False
_last_time = None

# Orientation state (radians)
_roll = 0.0
_pitch = 0.0
_yaw = 0.0  # Will drift over time (no magnetometer)

# Gyro bias (deg/s)
_gx_bias = 0.0
_gy_bias = 0.0
_gz_bias = 0.0

# Complementary filter blend
_ALPHA = 0.98  # gyro weight (increase for snappier response, less accel influence)

def _read_word_2c(bus, addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg + 1)
    val = (hi << 8) | lo
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

def _check_i2c1_available():
    """
    Best-effort sanity checks that I2C-1 (pins 3/5) is available.
    Does not modify the system; just hints if I2C might be disabled.
    """
    hints = []

    # Device node should exist when I2C enabled
    if not os.path.exists("/dev/i2c-1"):
        hints.append("Missing /dev/i2c-1 (enable I2C via raspi-config).")

    # On Pi device trees, alias 'i2c1' usually exists when enabled
    if not os.path.exists("/proc/device-tree/aliases/i2c1"):
        hints.append("Device-tree alias for i2c1 not found (I2C may be disabled).")

    if hints:
        msg = " | ".join(hints)
        print(f"[WARN] I2C-1 check: {msg}")

def _mpu_init():
    global _bus, _inited, _last_time
    if _inited:
        return

    _check_i2c1_available()

    # This binds us to I2C-1 -> SDA1/SCL1 on physical pins 3 & 5.
    _bus = SMBus(I2C_BUS)

    who = _bus.read_byte_data(MPU_ADDR, WHO_AM_I)
    if who not in (0x68, 0x69):
        raise RuntimeError(f"MPU6050 not found, WHO_AM_I=0x{who:02X} (check wiring/address)")

    # Wake up and choose PLL with X gyro as clock
    _bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    _bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.05)

    # DLPF and sample rate: ~200 Hz sample
    _bus.write_byte_data(MPU_ADDR, CONFIG, 0x03)      # DLPF ~44 Hz accel, ~42 Hz gyro
    _bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 0x04)  # 1kHz/(1+4)=200 Hz

    # Ranges
    _bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)   # ±250 dps
    _bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00)  # ±2g

    # Simple gyro bias calibration (stationary)
    _calibrate_gyro(samples=300)

    _last_time = time.time()
    _inited = True

def _calibrate_gyro(samples=300):
    """Estimate gyro biases while the sensor is stationary."""
    global _gx_bias, _gy_bias, _gz_bias
    gx_sum = gy_sum = gz_sum = 0.0
    for _ in range(samples):
        gx, gy, gz = _read_gyro_dps()
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz
        time.sleep(0.002)  # ~500 Hz
    _gx_bias = gx_sum / samples
    _gy_bias = gy_sum / samples
    _gz_bias = gz_sum / samples

def _read_accel_g():
    ax = _read_word_2c(_bus, MPU_ADDR, ACCEL_XOUT_H)
    ay = _read_word_2c(_bus, MPU_ADDR, ACCEL_YOUT_H)
    az = _read_word_2c(_bus, MPU_ADDR, ACCEL_ZOUT_H)
    # ±2g -> 16384 LSB/g
    return ax / 16384.0, ay / 16384.0, az / 16384.0

def _read_gyro_dps():
    gx = _read_word_2c(_bus, MPU_ADDR, GYRO_XOUT_H)
    gy = _read_word_2c(_bus, MPU_ADDR, GYRO_YOUT_H)
    gz = _read_word_2c(_bus, MPU_ADDR, GYRO_ZOUT_H)
    # ±250 dps -> 131 LSB/(deg/s)
    return gx / 131.0, gy / 131.0, gz / 131.0

def _update_orientation():
    """
    Read sensors once and update global roll/pitch/yaw.
    - roll, pitch: complementary (gyro integrated + accel tilt)
    - yaw: integrate gyro Z only (drifts)
    """
    global _last_time, _roll, _pitch, _yaw

    now = time.time()
    if _last_time is None:
        _last_time = now
    dt = now - _last_time
    if dt <= 0:
        dt = 1e-3
    _last_time = now

    ax, ay, az = _read_accel_g()
    gx, gy, gz = _read_gyro_dps()

    # Bias-correct gyro
    gx -= _gx_bias
    gy -= _gy_bias
    gz -= _gz_bias

    # Integrate gyro (deg/s -> rad/s)
    gx_rad = math.radians(gx)
    gy_rad = math.radians(gy)
    gz_rad = math.radians(gz)

    # Predict step
    _roll  += gx_rad * dt
    _pitch += gy_rad * dt
    _yaw   += gz_rad * dt  # WILL DRIFT (no mag)

    # Accel-based tilt (radians)
    roll_acc  = math.atan2(ay, az)
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    # Complementary blend for roll/pitch
    _roll  = _ALPHA * _roll  + (1.0 - _ALPHA) * roll_acc
    _pitch = _ALPHA * _pitch + (1.0 - _ALPHA) * pitch_acc

def _ensure_init_and_update():
    if not _inited:
        _mpu_init()
    _update_orientation()

# -------------------------
# Public simple API
# -------------------------

def get_roll_deg():
    """Return roll angle in degrees (right-hand rule around X)."""
    _ensure_init_and_update()
    return math.degrees(_roll)

def get_pitch_deg():
    """Return pitch angle in degrees (right-hand rule around Y)."""
    _ensure_init_and_update()
    return math.degrees(_pitch)

def get_yaw_deg():
    """
    Return yaw angle in degrees (right-hand rule around Z).
    WARNING: Without a magnetometer, yaw is unreferenced and will drift.
    """
    _ensure_init_and_update()
    return math.degrees(_yaw)

# -------------------------
# Main: continuous terminal print (infinite loop)
# -------------------------
def main(loop_hz: float = 50.0):
    """
    Continuously print Roll, Pitch, Yaw in degrees using I2C-1 (pins 3/5).
    Press Ctrl+C to stop.
    """
    period = 1.0 / loop_hz if loop_hz > 0 else 0.0
    print(f"Using I2C bus {I2C_BUS} (pins 3=SDA1, 5=SCL1).")
    print(f"Reading roll, pitch, yaw at ~{loop_hz:.1f} Hz (Ctrl+C to stop)...")
    try:
        while True:
            r = get_roll_deg()
            p = get_pitch_deg()
            y = get_yaw_deg()
            print(f"Roll={r:7.2f}°  Pitch={p:7.2f}°  Yaw={y:7.2f}°")
            if period > 0:
                time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if _bus is not None:
            _bus.close()

if __name__ == "__main__":
    # Default: ~50 Hz loop
    main(loop_hz=50.0)