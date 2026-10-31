"""E5: frequency-domain analysis and speed dependence of the closed loop.

* Bode plot of the loop transfer function L(jw) = C(jw) G(jw) for each linear
  controller, with gain/phase margins and closed-loop bandwidth.
* Closed-loop damping vs forward speed for a fixed gain (kp tuned at 0.4 m/s)
  and for the gain-scheduled law kp = 2 v / L^2: model prediction (dominant
  poles) and PyBullet overshoot measurements.
"""
from common import *  # noqa: F401,F403
from agv.analysis import (controller_frf, plant_frf, loop_margins, closed_loop_bandwidth, step_metrics,
                          unwrapped_phase)

SPEEDS = (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8)


def dominant_damping(kp, kd, v, L, tau, Td):
    """Damping ratio of the dominant closed-loop pole pair (1st-order Pade for the delay)."""
    # char. poly: s^2 (tau s + 1)(1 + Td s / 2) + (kp + kd s)(L s + v)(1 - Td s / 2) = 0
    a = np.polymul(np.polymul([1, 0, 0], [tau, 1]), [Td / 2, 1])
    b = np.polymul(np.polymul([kd, kp], [L, v]), [-Td / 2, 1])
    poles = np.roots(np.polyadd(a, b))
    cplx = [p for p in poles if abs(p.imag) > 1e-6]
    if not cplx:
        return 1.0, float(-max(poles.real))
    p = max(cplx, key=lambda p: p.real)          # slowest complex pair
    wn = abs(p)
    return float(-p.real / wn), float(wn)


def main():
    gains = load_json("gains")
    ident = load_json("identification")
    base = SimConfig()
    v, L, N = base.control.base_speed, base.sensor.lookahead, base.control.d_filter_n
    tau, Td = ident["tau"], ident["loop_delay"]
    kp_star = 2 * v / L ** 2
    cases = {"PID-ZN": gains["PID-ZN"], "PID-TL": gains["PID-TL"], "PID-NM": gains["PID-NM"],
             "LA-GS": dict(kp=kp_star, ki=0.0, kd=0.0)}
    w = np.logspace(-1, 3, 4000)
    G = plant_frf(w, v, L, tau, Td)
    margins = {}
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    for n, g in cases.items():
        Lw = controller_frf(w, g["kp"], g["ki"], g["kd"], N) * G
        lf = -270.0 if g["ki"] > 0 else -180.0
        m = loop_margins(w, Lw, lf)
        m["bandwidth"] = closed_loop_bandwidth(w, Lw)
        margins[n] = m
        a1.semilogx(w, 20 * np.log10(np.abs(Lw)), color=COLORS[n], label=n)
        a2.semilogx(w, np.degrees(unwrapped_phase(Lw, lf)), color=COLORS[n])
        if np.isfinite(m["gain_crossover"]):
            a2.plot(m["gain_crossover"], m["phase_margin"] - 180, "o", color=COLORS[n], ms=4)
    a1.axhline(0, color="k", lw=0.8); a2.axhline(-180, color="k", lw=0.8)
    a1.set_ylabel("|L(jw)| (dB)"); a2.set_ylabel("phase (deg)"); a2.set_xlabel("frequency w (rad/s)")
    a1.set_ylim(-40, 80); a2.set_ylim(-290, -60)
    a1.set_title(f"Open-loop Bode plot, v = {v} m/s, L = {L} m, tau = {tau*1000:.0f} ms (dots: gain crossover)")
    a1.legend(ncol=4)
    savefig(fig, "bode")

    # ---- speed dependence: fixed kp (= kp* at 0.4 m/s) vs scheduled kp
    model = {"fixed": [], "scheduled": []}
    for sp in SPEEDS:
        model["fixed"].append(dominant_damping(kp_star, 0.0, sp, L, tau, Td))
        model["scheduled"].append(dominant_damping(2 * sp / L ** 2, 0.0, sp, L, tau, Td))
    cfgs, keys = [], []
    for sp in SPEEDS:
        for mode in ("fixed", "scheduled"):
            ov = dict(control__kp=kp_star) if mode == "fixed" else dict(control__gain_schedule=True)
            cfgs.append(with_overrides(SimConfig(track="straight"), control__base_speed=sp, initial_offset=0.05,
                                       stop_at_distance=min(6.2, 2.0 + 6 * sp), max_time=25, **ov))
            keys.append((sp, mode))
    res = run_many(cfgs, keep_logs=True)
    measured = {"fixed": {}, "scheduled": {}}
    for (sp, mode), (log, s) in zip(keys, res):
        measured[mode][str(sp)] = step_metrics(log["t"], log["cte"])
    zeta_from_os = lambda os_: (1.0 if os_ <= 0.01 else
                                float(-np.log(os_ / 100) / np.sqrt(np.pi ** 2 + np.log(os_ / 100) ** 2)))
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 2.9))
    for mode, c in (("fixed", COLORS["P (fixed)"]), ("scheduled", COLORS["LA-GS"])):
        lab = "fixed kp = 35.6" if mode == "fixed" else "scheduled kp = 2v/L^2"
        axs[0].plot(SPEEDS, [z for z, _ in model[mode]], "-", color=c, label=f"{lab} (model)")
        axs[0].plot(SPEEDS, [zeta_from_os(measured[mode][str(sp)]["overshoot"]) for sp in SPEEDS], "o",
                    color=c, ms=4, label=f"{lab} (PyBullet, from overshoot)")
        axs[1].plot(SPEEDS, [measured[mode][str(sp)]["settling_time"] for sp in SPEEDS], "o-", color=c, ms=4,
                    label=lab)
    axs[0].axhline(1 / np.sqrt(2), color="k", ls=":", lw=0.8)
    axs[0].set_xlabel("forward speed v (m/s)"); axs[0].set_ylabel("damping ratio zeta")
    axs[0].set_title("Dominant-pole damping"); axs[0].legend(fontsize=6.5, loc="lower left")
    axs[1].set_xlabel("forward speed v (m/s)"); axs[1].set_ylabel("5 % settling time (s)")
    axs[1].set_title("PyBullet step response (5 cm)"); axs[1].legend(fontsize=7)
    savefig(fig, "speed_damping")
    save_json("frequency", dict(margins=margins, speeds=SPEEDS, model=model, measured=measured,
                                params=dict(v=v, L=L, tau=tau, Td=Td, N=N, kp_star=kp_star)))
    for n, m in margins.items():
        print(n, {k: round(x, 3) for k, x in m.items()})


if __name__ == "__main__":
    main()
