"""E9: robustness to AGV payload, floor friction and actuator lag (complex track,
v = 0.5 m/s, 3 seeds).  Gains are NOT re-tuned for the perturbed plants."""
from common import *  # noqa: F401,F403
from exp06_tracks import seeded

SWEEPS = {
    "payload": ("robot__payload_mass", (0.0, 10.0, 20.0, 40.0), "payload (kg)"),
    "friction": ("robot__floor_friction", (0.15, 0.25, 0.4, 0.9), "wheel-floor friction coefficient"),
    "motor_tau": ("robot__motor_tau", (0.03, 0.06, 0.12, 0.20), "motor time constant (s)"),
}
CTRLS = ("PID-ZN", "PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+")
SEEDS = range(3)


def main():
    ctrls = controller_set(load_json("gains"))
    jobs, cfgs = [], []
    for sweep, (key, values, _) in SWEEPS.items():
        for n in CTRLS:
            for val in values:
                for sd in SEEDS:
                    jobs.append((sweep, n, val, sd))
                    cfgs.append(seeded(with_overrides(SimConfig(track="complex", max_time=90),
                                                      control__base_speed=0.5, **{key: val}, **ctrls[n]), sd))
    res = run_many(cfgs)
    raw = {}
    for (sweep, n, val, sd), (_, s) in zip(jobs, res):
        raw.setdefault(sweep, {}).setdefault(n, {}).setdefault(str(val), []).append(s)
    summary = {}
    for sweep, (key, values, _) in SWEEPS.items():
        summary[sweep] = {}
        for n in CTRLS:
            summary[sweep][n] = {}
            for val in values:
                runs = raw[sweep][n][str(val)]
                ok = [r for r in runs if r["completed"]]
                summary[sweep][n][str(val)] = dict(success=len(ok) / len(runs),
                                                   envelope=float(np.mean([envelope_ok(r) for r in runs])),
                                                   rms_cte=mean_std([r["rms_cte"] for r in ok]),
                                                   max_cte=mean_std([r["max_cte"] for r in ok]),
                                                   energy=mean_std([r["energy"] for r in ok]),
                                                   fail_modes=[r["status"] for r in runs if not r["completed"]])
    save_json("robustness", dict(summary=summary, sweeps={k: dict(key=v[0], values=v[1]) for k, v in SWEEPS.items()}))

    fig, axs = plt.subplots(2, 3, figsize=(7.6, 4.4))
    for j, (sweep, (key, values, label)) in enumerate(SWEEPS.items()):
        for n in CTRLS:
            S = summary[sweep][n]
            axs[0, j].plot(values, [100 * S[str(v)]["envelope"] for v in values], "o-", ms=3, color=COLORS[n], label=n)
            vals_ok = [v for v in values if S[str(v)]["success"] > 0]
            axs[1, j].plot(vals_ok, [1000 * S[str(v)]["rms_cte"][0] for v in vals_ok], "o-", ms=3, color=COLORS[n])
        axs[1, j].set_xlabel(label)
        axs[0, j].set_ylim(-5, 105)
    axs[0, 0].set_ylabel("in-envelope runs (%)"); axs[1, 0].set_ylabel("RMS axle error (mm)")
    axs[0, 0].legend(fontsize=6.5, loc="lower left")
    fig.tight_layout()
    savefig(fig, "robustness")
    for sweep in summary:
        for n in CTRLS:
            print(sweep, n, [(v, summary[sweep][n][v]["success"], summary[sweep][n][v]["envelope"],
                              round(1000 * (summary[sweep][n][v]["rms_cte"][0] or 0), 1))
                             for v in summary[sweep][n]])


if __name__ == "__main__":
    main()
