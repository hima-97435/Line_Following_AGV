"""Steering controllers, line-state estimator and speed scheduler.

All steering controllers map the lateral line error e (m, + = line to the
left) to a commanded yaw rate omega (rad/s, + = turn left).
"""
import math

import numpy as np


class PID:
    """Discrete PID with filtered derivative and conditional-integration anti-windup."""

    def __init__(self, kp, ki, kd, dt, u_max, n_filter=15.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt, self.u_max, self.n = dt, u_max, n_filter
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.e_prev = None
        self.d_filt = 0.0

    def step(self, e, de=None):
        dt = self.dt
        if de is None:
            raw = 0.0 if self.e_prev is None else (e - self.e_prev) / dt
            alpha = dt * self.n / (1.0 + dt * self.n)        # first-order low-pass, cut-off N rad/s
            self.d_filt += alpha * (raw - self.d_filt)
            de = self.d_filt
        self.e_prev = e
        u_unsat = self.kp * e + self.ki * self.integral + self.kd * de
        u = float(np.clip(u_unsat, -self.u_max, self.u_max))
        # anti-windup: only integrate when not pushing further into saturation
        if self.ki > 0 and (u == u_unsat or np.sign(e) != np.sign(u_unsat)):
            self.integral += e * dt
        return u


class BangBang:
    """Three-level on/off steering (the classic hobby line follower)."""

    def __init__(self, dt, u_max, deadband):
        self.u, self.db = 0.6 * u_max, deadband

    def reset(self):
        pass

    def step(self, e, de=None):
        if abs(e) < self.db:
            return 0.0
        return math.copysign(self.u, e)


class Relay:
    """Relay with hysteresis, used for Astrom-Hagglund relay auto-tuning."""

    def __init__(self, amplitude, hysteresis):
        self.d, self.h = amplitude, hysteresis
        self.u = amplitude

    def reset(self):
        self.u = self.d

    def step(self, e, de=None):
        if e > self.h:
            self.u = self.d
        elif e < -self.h:
            self.u = -self.d
        return self.u


class FuzzyPD:
    """Zero-order Sugeno fuzzy PD controller (5 x 5 rule base)."""

    CENTERS = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])     # NB NS Z PS PB
    OUT = {"NB": -1.0, "NM": -0.6, "NS": -0.25, "Z": 0.0, "PS": 0.25, "PM": 0.6, "PB": 1.0}
    RULES = [  # rows: error NB..PB, columns: error-rate NB..PB
        ["NB", "NB", "NB", "NM", "NS"],
        ["NB", "NM", "NS", "NS", "Z"],
        ["NM", "NS", "Z", "PS", "PM"],
        ["Z", "PS", "PS", "PM", "PB"],
        ["PS", "PM", "PB", "PB", "PB"],
    ]

    def __init__(self, dt, u_max, e_scale, de_scale, n_filter=15.0):
        self.dt, self.u_max = dt, u_max
        self.e_scale, self.de_scale, self.n = e_scale, de_scale, n_filter
        self.table = np.array([[self.OUT[c] for c in row] for row in self.RULES])
        self.reset()

    def reset(self):
        self.e_prev = None
        self.d_filt = 0.0

    @classmethod
    def memberships(cls, x):
        x = float(np.clip(x, -1.0, 1.0))
        mu = np.maximum(0.0, 1.0 - np.abs(x - cls.CENTERS) / 0.5)
        return mu

    def step(self, e, de=None):
        if de is None:
            raw = 0.0 if self.e_prev is None else (e - self.e_prev) / self.dt
            alpha = self.dt * self.n / (1.0 + self.dt * self.n)
            self.d_filt += alpha * (raw - self.d_filt)
            de = self.d_filt
        self.e_prev = e
        mu_e = self.memberships(e / self.e_scale)
        mu_d = self.memberships(de / self.de_scale)
        w = np.minimum.outer(mu_e, mu_d)                  # rule firing strength (min t-norm)
        if w.sum() <= 0:
            return 0.0
        return float(self.u_max * (w * self.table).sum() / w.sum())   # weighted-average defuzzification


class LineKalman:
    """Constant-velocity Kalman filter on the line error: x = [e, de/dt]."""

    def __init__(self, dt, q, r):
        self.dt, self.q, self.r = dt, q, r
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        self.Q = q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        self.H = np.array([[1.0, 0.0]])
        self.reset()

    def reset(self, e0=0.0):
        self.x = np.array([e0, 0.0])
        self.P = np.diag([1e-3, 1e-2])
        self.initialised = False

    def step(self, z, valid=True):
        if not self.initialised and valid:
            self.x[:] = [z, 0.0]
            self.initialised = True
            return self.x.copy()
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        if valid:
            y = z - self.x[0]
            S = self.P[0, 0] + self.r
            K = self.P[:, 0] / S
            self.x = self.x + K * y
            self.P = self.P - np.outer(K, self.H @ self.P)
        return self.x.copy()


def make_controller(cp, dt):
    if cp.controller == "pid":
        return PID(cp.kp, cp.ki, cp.kd, dt, cp.omega_max, cp.d_filter_n)
    if cp.controller == "bangbang":
        return BangBang(dt, cp.omega_max, cp.bangbang_deadband)
    if cp.controller == "relay":
        return Relay(cp.relay_amp, cp.relay_hyst)
    if cp.controller == "fuzzy":
        return FuzzyPD(dt, cp.omega_max, cp.fuzzy_e_scale, cp.fuzzy_de_scale, cp.d_filter_n)
    raise ValueError(cp.controller)
