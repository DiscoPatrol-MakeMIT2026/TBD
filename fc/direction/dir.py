"""
Minidrone PID Flight Controller
================================
Models the control architecture from the diagram:
  - 4 independent PID loops: roll, pitch, yaw, thrust
  - Sensing & Estimation block (feedback)
  - Motor Mixing Algorithm (MMA)
  - Minidrone plant (simulated)
"""

import time
import math
from dataclasses import dataclass, field
from typing import Tuple


# ---------------------------------------------------------------------------
# PID Controller
# ---------------------------------------------------------------------------

class PID:
    """Generic discrete-time PID controller."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limits: Tuple[float, float] = (-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """Compute PID output given setpoint, measurement, and time step."""
        error = setpoint - measurement

        # Proportional
        p = self.kp * error

        # Integral (with anti-windup via clamping)
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative
        d = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = p + i + d

        # Clamp output
        output = max(self.output_min, min(self.output_max, output))

        # Conditional integration (anti-windup): back-calculate integral
        if output != p + i + d:
            self._integral -= error * dt  # undo last integration step

        return output


# ---------------------------------------------------------------------------
# Sensing & Estimation
# ---------------------------------------------------------------------------

@dataclass
class SystemState:
    """Estimated drone state from sensors."""
    roll: float = 0.0       # rad
    pitch: float = 0.0      # rad
    yaw: float = 0.0        # rad
    altitude: float = 0.0   # m


class SensingAndEstimation:
    """
    In a real system this would fuse IMU, barometer, GPS, etc.
    Here we simply pass through the plant state with optional noise.
    """

    def __init__(self, noise_std: float = 0.0):
        self.noise_std = noise_std

    def estimate(self, true_state: SystemState) -> SystemState:
        def noisy(val):
            if self.noise_std > 0:
                import random
                return val + random.gauss(0, self.noise_std)
            return val

        return SystemState(
            roll=noisy(true_state.roll),
            pitch=noisy(true_state.pitch),
            yaw=noisy(true_state.yaw),
            altitude=noisy(true_state.altitude),
        )


# ---------------------------------------------------------------------------
# Motor Mixing Algorithm (MMA)
# ---------------------------------------------------------------------------

@dataclass
class MotorCommands:
    """Normalized motor speeds [0, 1] for a quadrotor (X-config)."""
    m1: float = 0.0  # front-left  (spins CW)
    m2: float = 0.0  # front-right (spins CCW)
    m3: float = 0.0  # rear-right  (spins CW)
    m4: float = 0.0  # rear-left   (spins CCW)

    def clamp(self):
        self.m1 = max(0.0, min(1.0, self.m1))
        self.m2 = max(0.0, min(1.0, self.m2))
        self.m3 = max(0.0, min(1.0, self.m3))
        self.m4 = max(0.0, min(1.0, self.m4))
        return self

    def __repr__(self):
        return (f"Motors(m1={self.m1:.3f}, m2={self.m2:.3f}, "
                f"m3={self.m3:.3f}, m4={self.m4:.3f})")


def motor_mixing_algorithm(thrust: float, roll: float,
                            pitch: float, yaw: float) -> MotorCommands:
    """
    Standard X-quadrotor mixing matrix.

    Sign conventions (all inputs normalized to [-1, 1] except thrust [0, 1]):
      +roll  → right side down  (m1, m3 faster)
      +pitch → nose down        (m1, m2 faster)
      +yaw   → nose right (CW)  (m2, m4 faster)
    """
    m1 = thrust + roll + pitch - yaw   # front-left  CW
    m2 = thrust - roll + pitch + yaw   # front-right CCW
    m3 = thrust - roll - pitch - yaw   # rear-right  CW
    m4 = thrust + roll - pitch + yaw   # rear-left   CCW

    return MotorCommands(m1, m2, m3, m4).clamp()


# ---------------------------------------------------------------------------
# Simulated Minidrone Plant
# ---------------------------------------------------------------------------

class Minidrone:
    """
    Very simplified 1st-order linear plant for demonstration.
    In a real application this would be a full 6-DOF dynamics model.
    """

    # Time constants for each axis (seconds)
    TAU_ATTITUDE = 0.3   # roll / pitch / yaw
    TAU_ALTITUDE = 0.8   # altitude

    def __init__(self):
        self.state = SystemState()

    def step(self, motors: MotorCommands, dt: float):
        """Update plant state given motor commands."""
        # Recover virtual control inputs from motor mixing (pseudo-inverse)
        thrust = (motors.m1 + motors.m2 + motors.m3 + motors.m4) / 4.0
        roll   = (motors.m1 - motors.m2 - motors.m3 + motors.m4) / 4.0
        pitch  = (motors.m1 + motors.m2 - motors.m3 - motors.m4) / 4.0
        yaw    = (-motors.m1 + motors.m2 - motors.m3 + motors.m4) / 4.0

        # 1st-order lag:  dx/dt = (target - x) / tau
        alpha_att = dt / (self.TAU_ATTITUDE + dt)
        alpha_alt = dt / (self.TAU_ALTITUDE + dt)

        # Map roll/pitch commands [-1,1] → angle targets (±30 deg = ±0.52 rad)
        MAX_ANGLE = math.radians(30)
        self.state.roll     += alpha_att * (roll  * MAX_ANGLE - self.state.roll)
        self.state.pitch    += alpha_att * (pitch * MAX_ANGLE - self.state.pitch)
        self.state.yaw      += alpha_att * (yaw   * math.radians(180) - self.state.yaw)

        # Altitude: thrust > 0.5 → climb, < 0.5 → descend
        climb_rate = (thrust - 0.5) * 4.0   # m/s at full throttle delta
        self.state.altitude += alpha_alt * (climb_rate * 5.0 - self.state.altitude)
        self.state.altitude  = max(0.0, self.state.altitude)

        return self.state


# ---------------------------------------------------------------------------
# Flight Controller  (top-level, matches the block diagram)
# ---------------------------------------------------------------------------

@dataclass
class References:
    """Desired (commanded) values — the setpoints fed into each PID."""
    roll:     float = 0.0   # rad
    pitch:    float = 0.0   # rad
    yaw:      float = 0.0   # rad
    altitude: float = 1.0   # m


class FlightController:
    """
    Implements the closed-loop control diagram:

      References ──► [error] ──► PID ──► MMA ──► Minidrone
                        ▲                              │
                        └──── Sensing & Estimation ◄──┘
    """

    def __init__(self):
        # Four independent PID loops
        self.roll_pid   = PID(kp=4.0, ki=0.5,  kd=0.8,  output_limits=(-1.0,  1.0))
        self.pitch_pid  = PID(kp=4.0, ki=0.5,  kd=0.8,  output_limits=(-1.0,  1.0))
        self.yaw_pid    = PID(kp=2.0, ki=0.2,  kd=0.4,  output_limits=(-1.0,  1.0))
        self.thrust_pid = PID(kp=1.5, ki=0.3,  kd=0.6,  output_limits=( 0.0,  1.0))

        self.sensing = SensingAndEstimation(noise_std=0.001)
        self.drone   = Minidrone()

    def step(self, references: References, dt: float):
        """Run one control cycle."""
        # ── Sensing & Estimation ──────────────────────────────────────────
        estimated = self.sensing.estimate(self.drone.state)

        # ── Error computation & PID ───────────────────────────────────────
        roll_cmd   = self.roll_pid.compute(references.roll,     estimated.roll,     dt)
        pitch_cmd  = self.pitch_pid.compute(references.pitch,   estimated.pitch,    dt)
        yaw_cmd    = self.yaw_pid.compute(references.yaw,       estimated.yaw,      dt)
        thrust_cmd = self.thrust_pid.compute(references.altitude, estimated.altitude, dt)

        # ── Motor Mixing Algorithm ────────────────────────────────────────
        motors = motor_mixing_algorithm(thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd)

        # ── Minidrone plant ───────────────────────────────────────────────
        new_state = self.drone.step(motors, dt)

        return {
            "estimated_state": estimated,
            "roll_cmd":   roll_cmd,
            "pitch_cmd":  pitch_cmd,
            "yaw_cmd":    yaw_cmd,
            "thrust_cmd": thrust_cmd,
            "motors":     motors,
            "true_state": new_state,
        }


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def simulate(duration: float = 5.0, dt: float = 0.05):
    """Run a quick open-loop simulation and print the results."""
    controller = FlightController()

    # Command: hover at 2 m altitude, level attitude
    ref = References(roll=0.0, pitch=0.0, yaw=0.0, altitude=2.0)

    print(f"{'t':>6}  {'alt_ref':>7}  {'altitude':>8}  {'roll':>7}  "
          f"{'pitch':>7}  {'yaw':>7}  {' motors'}")
    print("-" * 80)

    t = 0.0
    while t <= duration:
        # Mid-flight disturbance: command a pitch-forward at t=2 s
        if 2.0 <= t < 3.0:
            ref.pitch = math.radians(10)
        else:
            ref.pitch = 0.0

        out = controller.step(ref, dt)
        s   = out["true_state"]
        m   = out["motors"]

        print(f"{t:6.2f}  {ref.altitude:7.2f}  {s.altitude:8.3f}  "
              f"{math.degrees(s.roll):7.2f}°"
              f"{math.degrees(s.pitch):7.2f}°"
              f"{math.degrees(s.yaw):7.2f}°  "
              f"[{m.m1:.2f} {m.m2:.2f} {m.m3:.2f} {m.m4:.2f}]")
        t += dt


if __name__ == "__main__":
    simulate()