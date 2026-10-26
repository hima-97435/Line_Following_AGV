"""PID tuning: relay auto-tuning (Astrom-Hagglund) and simulation-based optimisation."""
import math

import numpy as np

from .config import SimConfig, with_overrides
from .sim import run_episode


# ------------------------------------------------------------ relay test
def relay_test(base: SimConfig, amplitude=1.5, hysteresis=0.001, duration=14.0, settle=5.0):
    """Run a relay-feedback experiment on a straight tape and estimate Ku, Tu.

    With relay amplitude d and hysteresis h the loop settles into a limit cycle
    of amplitude a and period Tu; the describing-function estimate of the
    ultimate gain is Ku = 4 d / (pi * sqrt(a^2 - h^2)).
    """
    cfg = with_overrides(base, track="straight", max_time=duration, stop_at_distance=50.0,
                         control__controller="relay", control__relay_amp=amplitude,
                         control__relay_hyst=hysteresis, control__adaptive_speed=False,
                         control__use_kalman=False)
    log, _ = run_episode(cfg)
    t, e = log["t"], log["e_meas"]
    m = t > settle
    tt, ee = t[m], e[m]
    up = np.nonzero((ee[:-1] < 0) & (ee[1:] >= 0))[0]
    # interpolate upward zero crossings for a precise period
    tz = tt[up] - ee[up] * (tt[up + 1] - tt[up]) / (ee[up + 1] - ee[up])
    Tu = float(np.mean(np.diff(tz))) if len(tz) > 2 else float("nan")
    a = float((np.percentile(ee, 99) - np.percentile(ee, 1)) / 2)
    Ku = 4 * amplitude / (math.pi * math.sqrt(max(a * a - hysteresis ** 2, 1e-12)))
    return dict(Ku=Ku, Tu=Tu, amplitude=a, relay=amplitude, hysteresis=hysteresis,
                n_cycles=int(max(len(tz) - 1, 0))), log


def classic_rules(Ku, Tu):
    """PID gains (parallel form kp, ki, kd) from ultimate gain/period."""
    rules = {
        "ZN":  (0.60 * Ku, Tu / 2.0, Tu / 8.0),     # Ziegler-Nichols classic
        "TL":  (Ku / 2.2, 2.2 * Tu, Tu / 6.3),      # Tyreus-Luyben (robust)
        "ZN-NO": (0.20 * Ku, Tu / 2.0, Tu / 3.0),   # Ziegler-Nichols "no overshoot"
    }
    out = {}
    for name, (kp, Ti, Td) in rules.items():
        out[name] = dict(kp=kp, ki=kp / Ti, kd=kp * Td)
    return out


# ------------------------------------------------------ optimisation cost
def tuning_scenarios(base: SimConfig):
    """Training scenarios: one lap of the oval + a 5 cm step on the straight."""
    return [
        with_overrides(base, track="oval", laps=1, max_time=60.0),
        with_overrides(base, track="straight", initial_offset=0.05, stop_at_distance=2.8, max_time=20.0),
    ]


def cost_from_runs(results):
    (log_o, s_o), (log_s, s_s) = results
    J = 0.0
    # AGV objective: keep the axle (load) on the path, penalise jerky steering
    if s_o.get("completed"):
        J += s_o["rms_cte"] + 0.001 * s_o["smoothness"]
    else:
        J += 1.0 + 0.5 * (1 - s_o.get("distance", 0) / 9.7)
    t, y = log_s["t"], log_s["cte"]
    if s_s.get("completed"):
        J += float(np.sum(np.abs(y)) * (t[1] - t[0]))   # IAE of the step response
    else:
        J += 1.0
    return float(J)


def evaluate_gains(base: SimConfig, params: dict):
    runs = [run_episode(with_overrides(c, **params)) for c in tuning_scenarios(base)]
    return cost_from_runs(runs), runs


def optimise(base: SimConfig, x0: dict, to_params, maxfev=120, log_space=True, step=0.5):
    """Nelder-Mead on the tuning cost.  ``to_params`` maps a vector to overrides."""
    from scipy.optimize import minimize
    keys = list(x0)
    z0 = np.array([math.log10(max(x0[k], 1e-3)) if log_space else x0[k] for k in keys])
    history = []

    def f(z):
        vals = {k: (10 ** zi if log_space else zi) for k, zi in zip(keys, z)}
        J, _ = evaluate_gains(base, to_params(vals))
        history.append(dict(vals, cost=J))
        return J

    # explicit initial simplex: half a decade per parameter (scipy's default 5 % step
    # degenerates in log space when a coordinate is log10(1) = 0)
    simplex = np.vstack([z0] + [z0 + step * np.eye(len(z0))[i] for i in range(len(z0))])
    res = minimize(f, z0, method="Nelder-Mead",
                   options=dict(maxfev=maxfev, xatol=0.01, fatol=1e-5, initial_simplex=simplex))
    best = {k: (10 ** zi if log_space else zi) for k, zi in zip(keys, res.x)}
    return best, float(res.fun), history
