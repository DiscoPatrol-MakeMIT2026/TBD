import time

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target, measurement, dt):
        error = target - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


# -----------------------------
# Sensing & Estimation
# -----------------------------
def estimate_states(sensor_data):
    """
    Converts IMU + altitude sensors into estimated:
      - roll
      - pitch
      - yaw
      - altitude
    """
    # Example placeholders:
    return {
        "roll": sensor_data["imu_roll"],
        "pitch": sensor_data["imu_pitch"],
        "yaw": sensor_data["imu_yaw"],
        "altitude": sensor_data["alt"]
    }


# -----------------------------
# Motor Mixing Algorithm (MMA)
# -----------------------------
def motor_mixing(roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd):
    """
    Standard quadcopter X-configuration mixer.
    """
    m1 = thrust_cmd + roll_cmd + pitch_cmd - yaw_cmd  # front-left
    m2 = thrust_cmd - roll_cmd + pitch_cmd + yaw_cmd  # front-right
    m3 = thrust_cmd - roll_cmd - pitch_cmd - yaw_cmd  # rear-right
    m4 = thrust_cmd + roll_cmd - pitch_cmd + yaw_cmd  # rear-left

    return [m1, m2, m3, m4]


# -----------------------------------------
# PID controllers (from the blocks)
# -----------------------------------------
roll_pid   = PID(1.0, 0.0, 0.05)
pitch_pid  = PID(1.0, 0.0, 0.05)
yaw_pid    = PID(1.0, 0.0, 0.05)
thrust_pid = PID(1.0, 0.0, 0.1)


# -----------------------------------------
# Main Control Loop
# -----------------------------------------
def control_loop(commands, sensor_data, dt):

    # 1. Estimate current states from sensors
    states = estimate_states(sensor_data)

    # 2. Independent PID loops (as in your diagram)
    roll_cmd   = roll_pid.update(commands["roll_ref"],   states["roll"],   dt)
    pitch_cmd  = pitch_pid.update(commands["pitch_ref"], states["pitch"], dt)
    yaw_cmd    = yaw_pid.update(commands["yaw_ref"],     states["yaw"],   dt)
    thrust_cmd = thrust_pid.update(commands["alt_ref"],  states["altitude"], dt)

    # 3. Motor mixing (MMA block)
    motors = motor_mixing(roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd)

    return motors


# -----------------------------------------
# Example usage
# -----------------------------------------
if __name__ == "__main__":
    commands = {
        "roll_ref": 0.0,
        "pitch_ref": 0.0,
        "yaw_ref": 0.0,
        "alt_ref": 1.5,
    }

    dt = 0.01

    while True:
        # Fake sensor data for example
        sensor = {
            "imu_roll": 0.05,
            "imu_pitch": 0.02,
            "imu_yaw": -0.01,
            "alt": 1.2,
        }

        motors = control_loop(commands, sensor, dt)
        print("Motor outputs:", motors)

        time.sleep(dt)