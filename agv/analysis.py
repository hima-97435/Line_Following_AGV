"""Response analysis: time-domain metrics, linear plant model, frequency response.

Linearised line-following model (small heading error psi, lateral offset y):

    dy/dt   = v * psi
    dpsi/dt = omega                       (yaw rate actually achieved)
    omega   = omega_cmd / (tau s + 1) * exp(-s Td)
    e       = -(y + L * psi)              (lateral line error seen at look-ahead L)

so the plant from commanded yaw rate to measured line error is

    G(s) = -(L s + v) exp(-s Td) / (s^2 (tau s + 1))

and with the controller acting on e (omega_cmd = C(s) e) the loop transfer
function is  L(s) = C(s) (L s + v) exp(-s Td) / (s^2 (tau s + 1)).
The look-ahead adds a zero at s = -v/L, which is what makes the line follower
stabilisable with proportional control alone.
"""
import math

import numpy as np


# ------------------------------------------------------------ time-domain
def step_metrics(t, y, band_frac=0.05, min_band=0.002):
    """Metrics for regulation of an initial offset y(0) -> 0.

    rise time   : time for |y| to fall from 90 % to 10 % of |y0|
    overshoot   : largest excursion on the opposite side, % of |y0|
    settling    : last time |y| leaves the band max(band_frac*|y0|, min_band)
    ss_error    : mean |y| over the final second
    """
    t = np.asarray(t) - t[0]
    y = np.asarray(y)
    y0 = y[0] if abs(y[0]) > 1e-6 else y[np.argmax(np.abs(y))]
    sgn = math.copysign(1.0, y0)
    yn = y * sgn / abs(y0)                     # normalised, starts at +1
    try:
        t90 = t[np.argmax(yn <= 0.9)] if (yn <= 0.9).any() else np.nan
        t10 = t[np.argmax(yn <= 0.1)] if (yn <= 0.1).any() else np.nan
        rise = t10 - t90
    except ValueError:
        rise = np.nan
    overshoot = max(0.0, -yn.min()) * 100.0
    band = max(band_frac * abs(y0), min_band)
    outside = np.nonzero(np.abs(y) > band)[0]
    settling = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else (np.nan if len(outside) else 0.0)
    final = t > (t[-1] - 1.0)
    dt = np.diff(t, prepend=t[0])
    return dict(rise_time=float(rise), overshoot=float(overshoot), settling_time=float(settling),
                ss_error=float(np.mean(np.abs(y[final]))), iae=float(np.sum(np.abs(y) * dt)),
                itae=float(np.sum(t * np.abs(y) * dt)), peak_time=float(t[np.argmin(yn)]))


def fit_first_order(t, y, u_final):
    """Fit y = K u (1 - exp(-(t-Td)/tau)) to a step response by grid search."""
    t = np.asarray(t); y = np.asarray(y)
    K = np.mean(y[-max(5, len(y) // 10):]) / u_final
    best = (np.inf, None, None)
    for Td in np.arange(0.0, 0.08, 0.002):
        for tau in np.arange(0.005, 0.40, 0.0025):
            model = K * u_final * (1 - np.exp(-np.clip(t - Td, 0, None) / tau))
            err = np.mean((model - y) ** 2)
            if err < best[0]:
                best = (err, tau, Td)
    return dict(gain=float(K), tau=float(best[1]), delay=float(best[2]), mse=float(best[0]))


# --------------------------------------------------------- frequency-domain
def controller_frf(w, kp, ki, kd, n_filter):
    s = 1j * w
    return kp + ki / s + kd * s / (1 + s / n_filter)


def plant_frf(w, v, L, tau, Td):
    s = 1j * w
    return (L * s + v) * np.exp(-s * Td) / (s ** 2 * (tau * s + 1))


def unwrapped_phase(Lw, low_freq_phase_deg):
    """Unwrapped phase anchored to the known low-frequency asymptote (e.g. -180
    deg for a double integrator, -270 deg when the controller adds an integrator);
    plain np.unwrap would start at the principal value and be off by 360 deg."""
    ph = np.unwrap(np.angle(Lw))
    target = np.radians(low_freq_phase_deg)
    return ph - 2 * np.pi * np.round((ph[0] - target) / (2 * np.pi))


def loop_margins(w, Lw, low_freq_phase_deg=-180.0):
    """Gain/phase margins of a loop FRF sampled on w.

    The gain margin is taken at the -180 deg crossing above the gain crossover
    (gain-increase margin).  Loops with an integrator on top of the plant's
    double integrator are conditionally stable: their phase starts at -270 deg
    and also crosses -180 deg below the gain crossover; that crossing is
    reported separately as a gain-reduction margin.
    """
    mag = np.abs(Lw)
    ph = unwrapped_phase(Lw, low_freq_phase_deg)
    out = dict(gain_crossover=np.nan, phase_margin=np.nan, phase_crossover=np.nan, gain_margin_db=np.inf,
               lower_phase_crossover=np.nan, gain_reduction_margin_db=np.nan)
    i = np.nonzero((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    wc = np.inf
    if len(i):
        k = i[-1]
        f = (0 - np.log(mag[k])) / (np.log(mag[k + 1]) - np.log(mag[k]))
        wc = w[k] * (w[k + 1] / w[k]) ** f
        phc = ph[k] + f * (ph[k + 1] - ph[k])
        out["gain_crossover"] = float(wc)
        out["phase_margin"] = float(np.degrees(phc) + 180.0)
    j = np.nonzero(np.sign(ph[:-1] + np.pi) != np.sign(ph[1:] + np.pi))[0]
    for k in j:
        f = (-np.pi - ph[k]) / (ph[k + 1] - ph[k])
        wp = w[k] * (w[k + 1] / w[k]) ** f
        mp = np.exp(np.log(mag[k]) + f * (np.log(mag[k + 1]) - np.log(mag[k])))
        if wp > wc and not np.isfinite(out["phase_crossover"]):
            out["phase_crossover"] = float(wp)
            out["gain_margin_db"] = float(-20 * np.log10(mp))
        elif wp < wc:
            out["lower_phase_crossover"] = float(wp)
            out["gain_reduction_margin_db"] = float(20 * np.log10(mp))
    return out


def closed_loop_bandwidth(w, Lw):
    T = np.abs(Lw / (1 + Lw))
    i = np.nonzero(T < 1 / np.sqrt(2))[0]
    i = i[w[i] > w[np.argmax(T)]] if len(i) else i
    return float(w[i[0]]) if len(i) else np.nan


# ------------------------------------------------ linear time-domain model
def simulate_linear(kp, ki, kd, n_filter, v, L, tau, Td, dt_ctrl, y0, t_end=6.0, u_max=4.0,
                    psi0=0.0, dt=1e-3):
    """Simulate the linearised loop with the *same* discrete PID as the robot.

    Returns t and the lateral offset of the axle y(t) (robot-left positive).
    """
    from .controllers import PID
    pid = PID(kp, ki, kd, dt_ctrl, u_max, n_filter)
    n = int(t_end / dt)
    every = max(1, int(round(dt_ctrl / dt)))
    delay_n = int(round(Td / dt))
    buf = [0.0] * (delay_n + 1)
    y, psi, om = y0, psi0, 0.0
    u = 0.0
    ts, ys = [], []
    for k in range(n):
        if k % every == 0:
            e = -(y + L * math.sin(psi))
            u = pid.step(e)
        buf.append(u)
        u_del = buf.pop(0)
        om += dt / tau * (u_del - om)
        y += dt * v * math.sin(psi)
        psi += dt * om
        ts.append(k * dt); ys.append(y)
    return np.asarray(ts), np.asarray(ys)
