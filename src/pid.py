"""A reusable discrete-time PID controller written from scratch (NumPy only).

Features (all optional, switched on through constructor arguments):
  * output saturation (actuator limit)            -> u_min / u_max
  * anti-windup by conditional integration         -> anti_windup=True
  * derivative on measurement (removes "kick")     -> d_on_measurement=True
  * first-order low-pass filter on the D term      -> d_filter_tau > 0
"""
from __future__ import annotations
import math


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, dt: float = 0.001,
                 u_min: float = -math.inf, u_max: float = math.inf,
                 anti_windup: bool = False, d_on_measurement: bool = False,
                 d_filter_tau: float = 0.0):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.u_min, self.u_max = u_min, u_max
        self.anti_windup = anti_windup
        self.d_on_measurement = d_on_measurement
        self.d_filter_tau = d_filter_tau
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0        # running sum of error * dt  (units: m*s)
        self.prev_error = None     # e[k-1]
        self.prev_meas = None      # y[k-1]
        self.d_state = 0.0         # filtered derivative (units: m/s)
        # stored terms, handy for plotting
        self.p_term = self.i_term = self.d_term = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """Run one controller step and return the (saturated) actuator command u[k]."""
        dt = self.dt
        error = setpoint - measurement

        # ---- D: rate of change of error (or of -measurement) ----
        if self.prev_error is None:          # first call: no history yet
            raw_rate = 0.0
        elif self.d_on_measurement:          # d(e)/dt = -d(y)/dt when the setpoint is constant
            raw_rate = -(measurement - self.prev_meas) / dt
        else:
            raw_rate = (error - self.prev_error) / dt
        if self.d_filter_tau > 0.0:          # first-order low-pass: alpha = dt/(tau+dt)
            alpha = dt / (self.d_filter_tau + dt)
            self.d_state += alpha * (raw_rate - self.d_state)
        else:
            self.d_state = raw_rate

        # ---- I: candidate integral ----
        new_integral = self.integral + error * dt

        p = self.kp * error
        i = self.ki * new_integral
        d = self.kd * self.d_state
        u_unsat = p + i + d
        u = min(max(u_unsat, self.u_min), self.u_max)

        # ---- anti-windup: only keep integrating if it does not push deeper into saturation ----
        if self.anti_windup and u != u_unsat and (error * u_unsat) > 0:
            i = self.ki * self.integral       # freeze the integral at its previous value
            u = min(max(p + i + d, self.u_min), self.u_max)
        else:
            self.integral = new_integral

        self.p_term, self.i_term, self.d_term = p, i, d
        self.prev_error, self.prev_meas = error, measurement
        return u
