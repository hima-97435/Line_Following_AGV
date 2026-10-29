"""E3: proportional-gain sweep - validates the look-ahead design rule kp* = 2 v / L^2.

For a robot following an arc of any radius R with its sensor L ahead of the
axle, zero steady-state axle offset requires omega = v/R = kp * L^2 / (2R),
i.e. kp* = 2 v / L^2 (independent of R).  The same gain gives damping ratio
1/sqrt(2) in the linearised loop.  We sweep kp at three speeds and compare the
measured optimum with the prediction.
"""
from common import *  # noqa: F401,F403
from agv.analysis import step_metrics

SPEEDS = (0.25, 0.40, 0.55)
KPS = (5, 8, 12, 16, 18, 20, 22, 25, 28, 30, 32, 35, 38, 40, 45, 50, 55, 60, 80, 100, 130)
STEP_KPS = (10, 20, 35.6, 60, 120)


def main():
    base = SimConfig(track="oval")
    L = base.sensor.lookahead
    cfgs = [with_overrides(base, control__kp=kp, control__base_speed=v) for v in SPEEDS for kp in KPS]
    res = run_many(cfgs)
    table = {}
    for (v, kp), (_, s) in zip([(v, kp) for v in SPEEDS for kp in KPS], res):
        table.setdefault(str(v), []).append(dict(kp=kp, status=s["status"], rms_cte=s.get("rms_cte"),
                                                 rms_e=s.get("rms_e"), max_cte=s.get("max_cte"),
                                                 smoothness=s.get("smoothness")))
    summary = {}
    for v in SPEEDS:
        rows = [r for r in table[str(v)] if r["status"] == "completed"]
        best = min(rows, key=lambda r: r["rms_cte"])
        summary[str(v)] = dict(kp_star_theory=2 * v / L ** 2, kp_best_measured=best["kp"],
                               rms_cte_best=best["rms_cte"])
    # step responses at v = 0.4 on the straight tape
    step_cfgs = [with_overrides(SimConfig(track="straight"), control__kp=kp, initial_offset=0.05,
                                stop_at_distance=3.2, max_time=20) for kp in STEP_KPS]
    step_res = run_many(step_cfgs, keep_logs=True)
    steps = {}
    for kp, (log, s) in zip(STEP_KPS, step_res):
        m = step_metrics(log["t"], log["cte"])
        steps[str(kp)] = m
    save_json("kp_sweep", dict(table=table, summary=summary, steps=steps, lookahead=L))

    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.0))
    for v, c in zip(SPEEDS, ("tab:blue", "tab:green", "tab:red")):
        rows = [r for r in table[str(v)] if r["status"] == "completed"]
        axs[0].plot([r["kp"] for r in rows], [1000 * r["rms_cte"] for r in rows], "o-", ms=3, color=c,
                    label=f"v = {v:.2f} m/s")
        axs[0].axvline(2 * v / L ** 2, color=c, ls="--", lw=0.8)
    axs[0].set_xscale("log"); axs[0].set_xlabel("proportional gain kp (rad/s per m)")
    axs[0].set_ylabel("RMS axle cross-track error (mm)")
    axs[0].set_title("Oval lap; dashed: kp* = 2v/L^2")
    axs[0].legend()
    for kp, (log, s) in zip(STEP_KPS, step_res):
        lab = f"kp = {kp:g}" + (" (= kp*)" if abs(kp - 35.6) < 0.1 else "")
        axs[1].plot(log["t"], 1000 * log["cte"], label=lab, lw=1.6 if abs(kp - 35.6) < 0.1 else 1.0)
    axs[1].set_xlim(0, 5); axs[1].set_xlabel("time (s)"); axs[1].set_ylabel("axle offset (mm)")
    axs[1].set_title("5 cm step on straight tape, v = 0.40 m/s"); axs[1].legend()
    savefig(fig, "kp_sweep")
    print(summary); print({k: (round(v["overshoot"], 1), round(v["settling_time"], 2)) for k, v in steps.items()})


if __name__ == "__main__":
    main()
