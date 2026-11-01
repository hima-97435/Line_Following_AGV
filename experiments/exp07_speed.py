"""E7: speed sweep on the unseen complex track (0.3 - 1.0 m/s, 3 seeds).

Completion alone is not discriminating (the reflexive recovery brings a robot
back to the tape after an excursion), so the key metric is the in-envelope
rate: run completed with maximum axle deviation <= 50 mm.
"""
from common import *  # noqa: F401,F403
from exp06_tracks import seeded

SPEEDS = (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
SEEDS = range(3)
CTRLS = ("Bang-bang", "PID-ZN", "PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+")


def main():
    ctrls = controller_set(load_json("gains"))
    jobs, cfgs = [], []
    for n in CTRLS:
        for v in SPEEDS:
            for sd in SEEDS:
                jobs.append((n, v, sd))
                cfgs.append(seeded(with_overrides(SimConfig(track="complex", max_time=90),
                                                  control__base_speed=v, **ctrls[n]), sd))
    res = run_many(cfgs)
    raw = {}
    for (n, v, sd), (_, s) in zip(jobs, res):
        raw.setdefault(n, {}).setdefault(str(v), []).append(s)
    summary = {}
    for n in CTRLS:
        summary[n] = {}
        for v in SPEEDS:
            runs = raw[n][str(v)]
            ok = [r for r in runs if r["completed"]]
            summary[n][str(v)] = dict(success=len(ok) / len(runs),
                                      envelope=float(np.mean([envelope_ok(r) for r in runs])),
                                      rms_cte=mean_std([r["rms_cte"] for r in ok]),
                                      max_cte=mean_std([r["max_cte"] for r in ok]),
                                      time=mean_std([r["time"] for r in ok]),
                                      avg_speed=mean_std([r["avg_speed"] for r in ok]),
                                      lost_frac=mean_std([r["lost_frac"] for r in runs]),
                                      fail_modes=[r["status"] for r in runs if not r["completed"]])
    save_json("speed_sweep", dict(summary=summary, speeds=SPEEDS, seeds=list(SEEDS), envelope_m=ENVELOPE, raw=raw))

    fig, axs = plt.subplots(1, 3, figsize=(7.6, 2.7))
    for n in CTRLS:
        S = summary[n]
        axs[0].plot(SPEEDS, [100 * S[str(v)]["envelope"] for v in SPEEDS], "o-", ms=3, color=COLORS[n], label=n)
        vs = [v for v in SPEEDS if S[str(v)]["success"] > 0]
        axs[1].plot(vs, [1000 * S[str(v)]["max_cte"][0] for v in vs], "o-", ms=3, color=COLORS[n])
        axs[2].plot(vs, [S[str(v)]["time"][0] for v in vs], "o-", ms=3, color=COLORS[n])
    axs[1].axhline(1000 * ENVELOPE, color="k", ls=":", lw=0.8)
    axs[0].set_ylabel("in-envelope runs (%)"); axs[1].set_ylabel("max axle error (mm)")
    axs[2].set_ylabel("run time (s)")
    for ax in axs:
        ax.set_xlabel("commanded top speed (m/s)")
    axs[0].set_ylim(-5, 105)
    axs[0].legend(fontsize=6.5, loc="lower left")
    fig.tight_layout()
    savefig(fig, "speed_sweep")
    for n in CTRLS:
        print(n, [(v, summary[n][str(v)]["envelope"], round(1000 * summary[n][str(v)]["rms_cte"][0], 1),
                   round(1000 * summary[n][str(v)]["max_cte"][0], 1), round(summary[n][str(v)]["time"][0], 1))
                  for v in SPEEDS])


if __name__ == "__main__":
    main()
