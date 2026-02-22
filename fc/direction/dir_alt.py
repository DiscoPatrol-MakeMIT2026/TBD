"""
Minidrone Hover Controller — KISS edition
Target: roll=0°, pitch=0°, yaw=0°, altitude=30 cm
"""

import math
import random

# ─── Tuning ───────────────────────────────────────────────────────────────────
DT          = 0.02          # loop period (seconds)  ~50 Hz
DURATION    = 5.0           # total sim time (seconds)

TARGET_ROLL     = 0.0       # rad
TARGET_PITCH    = 0.0       # rad
TARGET_YAW      = 0.0       # rad
TARGET_ALTITUDE = 0.30      # metres

# PID gains  (Kp, Ki, Kd)
GAINS = {
    "roll":     (4.0, 0.2, 0.8),
    "pitch":    (4.0, 0.2, 0.8),
    "yaw":      (2.0, 0.1, 0.4),
    "altitude": (3.0, 0.5, 1.0),
}

# ─── State (sensors will fill these each loop) ────────────────────────────────
roll     = 0.0    # rad  — from IMU
pitch    = 0.0    # rad  — from IMU
yaw      = 0.0    # rad  — from IMU/magnetometer
altitude = 0.0    # m    — from sonar / barometer

# PID memory
integrals  = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "altitude": 0.0}
prev_errors = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "altitude": 0.0}

# ─── Tiny helpers ─────────────────────────────────────────────────────────────

def read_sensors(drone_state):
    """Simulate IMU + sonar readings (add small noise)."""
    return (
        drone_state["roll"]     + random.gauss(0, 0.002),
        drone_state["pitch"]    + random.gauss(0, 0.002),
        drone_state["yaw"]      + random.gauss(0, 0.002),
        drone_state["altitude"] + random.gauss(0, 0.001),
    )

def pid(axis, setpoint, measurement, dt):
    """Compute PID output for one axis; clamp to [-1, 1]."""
    kp, ki, kd = GAINS[axis]
    error = setpoint - measurement
    integrals[axis]   += error * dt
    derivative         = (error - prev_errors[axis]) / dt
    prev_errors[axis]  = error
    output = kp * error + ki * integrals[axis] + kd * derivative
    return max(-1.0, min(1.0, output))

def mix_motors(thrust, roll_cmd, pitch_cmd, yaw_cmd):
    """X-quad mixing matrix → 4 motor speeds clamped to [0, 1]."""
    m1 = thrust + roll_cmd + pitch_cmd - yaw_cmd   # front-left
    m2 = thrust - roll_cmd + pitch_cmd + yaw_cmd   # front-right
    m3 = thrust - roll_cmd - pitch_cmd - yaw_cmd   # rear-right
    m4 = thrust + roll_cmd - pitch_cmd + yaw_cmd   # rear-left
    return tuple(max(0.0, min(1.0, m)) for m in (m1, m2, m3, m4))

def send_to_drone(motors):
    """In real code: write PWM values to ESCs."""
    pass   # hardware call goes here

def update_plant(drone_state, motors, dt):
    """Simple 1st-order plant simulation (replace with real drone)."""
    thrust, roll_c, pitch_c, yaw_c = (
        sum(motors) / 4,
        (motors[0] - motors[1] - motors[2] + motors[3]) / 4,
        (motors[0] + motors[1] - motors[2] - motors[3]) / 4,
        (-motors[0] + motors[1] - motors[2] + motors[3]) / 4,
    )
    a_att = dt / (0.3 + dt)
    a_alt = dt / (0.8 + dt)
    drone_state["roll"]     += a_att * (roll_c  * math.radians(30) - drone_state["roll"])
    drone_state["pitch"]    += a_att * (pitch_c * math.radians(30) - drone_state["pitch"])
    drone_state["yaw"]      += a_att * (yaw_c   * math.radians(180) - drone_state["yaw"])
    drone_state["altitude"] += a_alt * ((thrust - 0.5) * 2.0 - drone_state["altitude"])
    drone_state["altitude"]  = max(0.0, drone_state["altitude"])

# ─── Main loop ────────────────────────────────────────────────────────────────

drone_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "altitude": 0.0}

print(f"{'t':>5}  {'alt':>6}  {'roll':>7}  {'pitch':>7}  {'yaw':>7}  {'motors'}")
print("─" * 70)

t = 0.0
while t <= DURATION:

    # 1. Read sensors
    roll, pitch, yaw, altitude = read_sensors(drone_state)

    # 2. Run PID for each axis
    roll_cmd    = pid("roll",     TARGET_ROLL,     roll,     DT)
    pitch_cmd   = pid("pitch",    TARGET_PITCH,    pitch,    DT)
    yaw_cmd     = pid("yaw",      TARGET_YAW,      yaw,      DT)
    thrust_cmd  = pid("altitude", TARGET_ALTITUDE, altitude, DT)
    thrust_cmd  = max(0.0, thrust_cmd)   # thrust is always positive

    # 3. Mix motors
    motors = mix_motors(thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd)

    # 4. Send commands to drone
    send_to_drone(motors)

    # 5. Update simulated plant (remove this block on real hardware)
    update_plant(drone_state, motors, DT)

    # 6. Log every 10 ticks
    if round(t / DT) % 10 == 0:
        m = motors
        print(f"{t:5.2f}  {altitude:6.3f}m  "
              f"{math.degrees(roll):6.2f}°  "
              f"{math.degrees(pitch):6.2f}°  "
              f"{math.degrees(yaw):6.2f}°  "
              f"[{m[0]:.2f} {m[1]:.2f} {m[2]:.2f} {m[3]:.2f}]")

    t += DT