"""E8: sensor-noise robustness and ablation of the LA-GS+ components
(complex track, v = 0.40 m/s, 5 seeds per point).

Ablation chain: LA-GS with the plain all-sensor centroid -> + peak-window
centroid (default perception) -> + sensor-range guard -> + Kalman filter ->
+ adaptive speed (= LA-GS+).  PID-ZN and PID-NM are the references.
"""
from common import *  # noqa: F401,F403
from exp06_tracks import seeded

NOISE = (0.0, 0.05, 0.10, 0.15, 0.20, 0.30)
SEEDS = range(5)


def variants(ctrls):
    la = ctrls["LA-GS"]
    return {
        "PID-ZN": ctrls["PID-ZN"],
        "PID-NM": ctrls["PID-NM"],
        "LA-GS (all-sensor centroid)": dict(la, sensor__ir_method="centroid"),
        "LA-GS": la,
        "LA-GS + guard": dict(la, control__edge_guard=True),
        "LA-GS + guard + KF": dict(la, control__edge_guard=True, control__use_kalman=True),
        "LA-GS+ (guard + KF + speed)": ctrls["LA-GS+"],
    }


STYLE = {
    "PID-ZN": (COLORS["PID-ZN"], "-"), "PID-NM": (COLORS["PID-NM"], "-"),
    "LA-GS (all-sensor centroid)": ("#98df8a", ":"), "LA-GS": (COLORS["LA-GS"], "-"),
    "LA-GS + guard": ("#bcbd22", "--"), "LA-GS + guard + KF": ("#8c564b", "--"),
    "LA-GS+ (guard + KF + speed)": (COLORS["LA-GS+"], "-"),
}


def main():
    ctrls = controller_set(load_json("gains"))
    V = variants(ctrls)
    jobs, cfgs = [], []
    for n, ov in V.items():
        for sig in NOISE:
            for sd in SEEDS:
                jobs.append((n, sig, sd))
                cfgs.append(seeded(with_overrides(SimConfig(track="complex", max_time=90),
                                                  sensor__noise_std=sig, **ov), sd))
    res = run_many(cfgs)
    raw = {}
    for (n, sig, sd), (_, s) in zip(jobs, res):
        raw.setdefault(n, {}).setdefault(str(sig), []).append(s)
    summary = {}
    for n in V:
        summary[n] = {}
        for sig in NOISE:
            runs = raw[n][str(sig)]
            ok = [r for r in runs if r["completed"]]
            summary[n][str(sig)] = dict(success=len(ok) / len(runs),
                                        envelope=float(np.mean([envelope_ok(r) for r in runs])),
                                        rms_cte=mean_std([r["rms_cte"] for r in ok]),
                                        max_cte=mean_std([r["max_cte"] for r in ok]),
                                        smoothness=mean_std([r["smoothness"] for r in ok]),
                                        sensor_rmse=mean_std([r["sensor_rmse"] for r in runs]),
                                        estimate_rmse=mean_std([r["estimate_rmse"] for r in runs]),
                                        energy=mean_std([r["energy"] for r in ok]),
                                        fail_modes=[r["status"] for r in runs if not r["completed"]])
    save_json("noise", dict(summary=summary, noise=NOISE, seeds=list(SEEDS)))

    fig, axs = plt.subplots(1, 3, figsize=(7.6, 2.9))
    x = [100 * s for s in NOISE]
    for n in V:
        S = summary[n]
        c, ls = STYLE[n]
        axs[0].plot(x, [100 * S[str(s)]["success"] for s in NOISE], ls, marker="o", ms=3, color=c, label=n)
        xs = [100 * s for s in NOISE if S[str(s)]["success"] > 0]
        axs[1].plot(xs, [1000 * S[str(s)]["rms_cte"][0] for s in NOISE if S[str(s)]["success"] > 0], ls,
                    marker="o", ms=3, color=c)
        axs[2].semilogy(xs, [S[str(s)]["smoothness"][0] for s in NOISE if S[str(s)]["success"] > 0], ls,
                        marker="o", ms=3, color=c)
    axs[0].set_ylabel("completed runs (%)"); axs[1].set_ylabel("RMS axle error (mm)")
    axs[2].set_ylabel("steering activity (rad/s^2)")
    for ax in axs:
        ax.set_xlabel("IR noise std (% of full scale)")
    axs[0].set_ylim(-5, 105)
    fig.legend(*axs[0].get_legend_handles_labels(), loc="lower center", ncol=4, fontsize=6.5,
               bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    savefig(fig, "noise")
    for n in V:
        print(f"{n:30s}", [(s, summary[n][str(s)]["success"], round(1000 * summary[n][str(s)]["rms_cte"][0], 1))
                           for s in NOISE])


if __name__ == "__main__":
    main()
