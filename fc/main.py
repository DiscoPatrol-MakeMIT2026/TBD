import time
from .sensors.imu import get_pitch_deg, get_roll_deg , get_yaw_deg
from .sensors.altitudeT import get_altitude, init_altitude, close_altitude
from .control.rc import send_motors, read_motors


# --- PID State Variables ---
roll_i = pitch_i = yaw_i = thrust_i = 0.0
prev_roll_e = prev_pitch_e = prev_yaw_e = prev_alt_e = 0.0

# --- PID constants (tune later) ---
Kp = {"roll":1.0, "pitch":1.0, "yaw":1.0, "alt":1.0}
Ki = {"roll":0.0, "pitch":0.0, "yaw":0.0, "alt":0.1}
Kd = {"roll":0.05,"pitch":0.05,"yaw":0.05,"alt":0.1}

# --- Hover target ---
ROLL_REF = 0.0
PITCH_REF = 0.0
YAW_REF = 0.0
ALT_REF = 0.30    # meters

# --- Simple PID function ---
def pid_update(target, measurement, dt, integral, prev_error, K):
    error = target - measurement
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0

    output = (
        Kp[K] * error +
        Ki[K] * integral +
        Kd[K] * derivative
    )

    return output, integral, error


# --- Motor Mixer (X configuration) ---
def mix(roll, pitch, yaw, thrust):
    m1 = thrust + roll + pitch - yaw
    m2 = thrust - roll + pitch + yaw
    m3 = thrust - roll - pitch - yaw
    m4 = thrust + roll - pitch + yaw
    return [m1, m2, m3, m4]


# ----------------------------------------------------------
# KISS MAIN LOOP
# ----------------------------------------------------------
h = init_altitude()
last = time.time()

while True:

    # Compute dt
    now = time.time()
    dt = now - last
    last = now

    # 1) ---- READ SENSORS ----
    imu_roll    = get_roll_deg()        # radians or deg
    imu_pitch   = get_pitch_deg()
    imu_yaw     = get_yaw_deg()
    altitude    = get_altitude()   # meters

    # 2) ---- UPDATE PIDs ----

    # Roll
    roll_cmd, roll_i, prev_roll_e = pid_update(
        ROLL_REF, imu_roll, dt, roll_i, prev_roll_e, "roll"
    )

    # Pitch
    pitch_cmd, pitch_i, prev_pitch_e = pid_update(
        PITCH_REF, imu_pitch, dt, pitch_i, prev_pitch_e, "pitch"
    )

    # Yaw
    yaw_cmd, yaw_i, prev_yaw_e = pid_update(
        YAW_REF, imu_yaw, dt, yaw_i, prev_yaw_e, "yaw"
    )

    # Thrust (altitude)
    thrust_cmd, thrust_i, prev_alt_e = pid_update(
        ALT_REF, altitude, dt, thrust_i, prev_alt_e, "alt"
    )

    # 3) ---- MOTOR MIX ----
    motors = mix(roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd)

    # 4) ---- SEND MOTOR COMMANDS ----
    send_motors(motors)

    # 5) Fixed loop rate: aim ~200–500Hz
    time.sleep(0.002)