"""E6: track benchmark - every controller on the training (oval) and the unseen
validation (complex) track, 5 Monte-Carlo seeds (sensor noise, start pose)."""
from common import *  # noqa: F401,F403

SEEDS = range(5)
METRICS = ("time", "rms_cte", "max_cte", "rms_e", "smoothness", "energy", "lost_frac")


def seeded(cfg, seed):
    rng = np.random.default_rng(1000 + seed)
    return with_overrides(cfg, seed=seed, initial_offset=float(rng.uniform(-0.01, 0.01)),
                          initial_heading=float(np.radians(rng.uniform(-3, 3))))


def main():
    gains = load_json("gains")
    ctrls = controller_set(gains)
    jobs, cfgs = [], []
    for track in ("oval", "complex"):
        for n, ov in ctrls.items():
            for sd in SEEDS:
                jobs.append((track, n, sd))
                cfgs.append(seeded(with_overrides(SimConfig(track=track, max_time=90), **ov), sd))
    res = run_many(cfgs, keep_logs=True)
    table, example_logs = {}, {}
    for (track, n, sd), (log, s) in zip(jobs, res):
        table.setdefault(track, {}).setdefault(n, []).append(s)
        if sd == 0:
            example_logs[(track, n)] = log
    summary = {}
    for track, by in table.items():
        summary[track] = {}
        for n, runs in by.items():
            row = dict(success=float(np.mean([r["completed"] for r in runs])),
                       envelope=float(np.mean([envelope_ok(r) for r in runs])))
            for m in METRICS:
                row[m] = mean_std([r.get(m) for r in runs if r["completed"]])
            summary[track][n] = row
    save_json("tracks", dict(summary=summary, seeds=list(SEEDS),
                             runs={t: {n: r for n, r in by.items()} for t, by in table.items()}))

    from agv.track import Track
    T = Track("complex")
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.8), gridspec_kw=dict(width_ratios=[1.25, 1]))
    show = ("Bang-bang", "PID-ZN", "PID-NM", "Fuzzy-NM", "LA-GS+")
    for ax in axs:
        ax.plot(T.pts[:, 0], T.pts[:, 1], color="k", lw=4, alpha=0.18, label="tape")
        for n in show:
            lg = example_logs[("complex", n)]
            ax.plot(lg["x"], lg["y"], color=COLORS[n], lw=1.0, label=n)
        ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    axs[0].set_title("Complex track (unseen), v = 0.40 m/s"); axs[0].legend(loc="upper left", fontsize=7)
    axs[1].set_xlim(2.05, 2.75); axs[1].set_ylim(0.75, 1.35); axs[1].set_title("Zoom: R = 0.12 m corner")
    savefig(fig, "tracks_xy")

    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    for n in ("PID-ZN", "PID-NM", "LA-GS"):
        lg = example_logs[("complex", n)]
        ax.plot(lg["s"], 1000 * lg["cte"], color=COLORS[n], lw=0.9, label=n)
    ax2 = ax.twinx(); ax2.grid(False)
    lg = example_logs[("complex", "LA-GS")]
    ax2.fill_between(lg["s"], 0, np.abs(lg["curv"]), color="0.5", alpha=0.15, step="mid")
    ax2.set_ylabel("|track curvature| (1/m)", color="0.4")
    ax.set_xlabel("distance along track s (m)"); ax.set_ylabel("axle cross-track error (mm)")
    ax.set_title("Cross-track error along the complex track"); ax.legend(loc="upper left", ncol=3)
    savefig(fig, "tracks_error_profile")

    fig, axs = plt.subplots(1, 2, figsize=(7.4, 2.8), sharey=False)
    names = list(ctrls)
    for ax, track in zip(axs, ("oval", "complex")):
        mu = [1000 * summary[track][n]["rms_cte"][0] for n in names]
        sd = [1000 * summary[track][n]["rms_cte"][1] for n in names]
        ax.bar(range(len(names)), mu, yerr=sd, color=[COLORS[n] for n in names], capsize=2)
        ax.set_xticks(range(len(names))); ax.set_xticklabels(names, rotation=40, ha="right", fontsize=7)
        ax.set_ylabel("RMS axle error (mm)"); ax.set_title(f"{track} track (mean +- std, 5 seeds)")
    savefig(fig, "tracks_bar")
    for track in summary:
        for n, r in summary[track].items():
            print(f"{track:8s} {n:10s} succ={r['success']:.1f} t={r['time'][0]:.1f} rms={r['rms_cte'][0]*1000:.1f} "
                  f"max={r['max_cte'][0]*1000:.1f} rms_e={r['rms_e'][0]*1000:.1f} sm={r['smoothness'][0]:.2f} "
                  f"E={r['energy'][0]:.0f}")


if __name__ == "__main__":
    main()
