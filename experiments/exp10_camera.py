"""E10: camera (rendered, inverse-perspective line fit) vs IR reflectance array.

The camera evaluates the line error further ahead (L = 0.28 m).  LA-GS adapts
automatically (kp = 2 v / L^2 uses the sensor's look-ahead); the fixed-gain
PID-NM keeps the gains tuned for the IR array.
"""
from common import *  # noqa: F401,F403
from exp06_tracks import seeded
from agv.track import Track
from agv.world import World
from agv.sensors import Camera

SEEDS = range(3)
CTRLS = ("PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+")


def camera_frame_figure():
    cfg = with_overrides(SimConfig(track="complex"), sensor__kind="camera")
    track = Track("complex")
    world = World(cfg, track)
    i = int(0.115 * track.n)                    # approaching the first left-hand bend
    x, y = track.pts[i]
    world.reset_pose(x, y + 0.02, track.heading[i] + 0.05)
    world.step(10)
    cam = Camera(cfg, world, np.random.default_rng(0))
    reading = cam.read()
    img = cam.last_image
    thr = min(110.0, 0.5 * (np.percentile(img, 95) + np.percentile(img, 2)))
    world.close()
    fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.3))
    axs[0].imshow(img, cmap="gray", vmin=0, vmax=255); axs[0].set_title("rendered 80x60 camera frame")
    axs[1].imshow(img < thr, cmap="gray_r"); axs[1].set_title("thresholded tape mask")
    mask = img < thr
    fx, fy = cam.floor_x[mask], cam.floor_y[mask]
    axs[2].plot(-fy * 100, fx * 100, ".", ms=2, color="0.3", label="tape pixels (IPM)")
    xs = np.linspace(np.nanmin(fx), np.nanmax(fx), 50)
    axs[2].axhline(cfg.sensor.cam_lookahead * 100, color="tab:red", ls="--", lw=0.8,
                   label=f"look-ahead, e = {reading.error*100:.1f} cm")
    axs[2].set_xlabel("lateral (cm, right +)"); axs[2].set_ylabel("ahead of axle (cm)")
    axs[2].set_title(f"floor projection, kappa = {reading.curvature:.2f} 1/m"); axs[2].legend(fontsize=6)
    for ax in axs[:2]:
        ax.axis("off")
    savefig(fig, "camera_pipeline")
    return dict(error=reading.error, curvature=reading.curvature, heading=reading.heading)


def main():
    ctrls = controller_set(load_json("gains"))
    jobs, cfgs = [], []
    for sensor in ("ir", "camera"):
        for n in CTRLS:
            for sd in SEEDS:
                jobs.append((sensor, n, sd))
                cfgs.append(seeded(with_overrides(SimConfig(track="complex", max_time=90),
                                                  sensor__kind=sensor, **ctrls[n]), sd))
    res = run_many(cfgs, keep_logs=True)
    raw, logs = {}, {}
    for (sensor, n, sd), (log, s) in zip(jobs, res):
        raw.setdefault(sensor, {}).setdefault(n, []).append(s)
        if sd == 0:
            logs[(sensor, n)] = log
    summary = {}
    for sensor in raw:
        summary[sensor] = {}
        for n, runs in raw[sensor].items():
            ok = [r for r in runs if r["completed"]]
            summary[sensor][n] = dict(success=len(ok) / len(runs),
                                      rms_cte=mean_std([r["rms_cte"] for r in ok]),
                                      max_cte=mean_std([r["max_cte"] for r in ok]),
                                      time=mean_std([r["time"] for r in ok]),
                                      sensor_rmse=mean_std([r["sensor_rmse"] for r in runs]),
                                      sensor_ms=mean_std([r["sensor_ms"] for r in runs]),
                                      lost_frac=mean_std([r["lost_frac"] for r in runs]),
                                      fail_modes=[r["status"] for r in runs if not r["completed"]])
    frame = camera_frame_figure()
    save_json("camera_vs_ir", dict(summary=summary, frame=frame, seeds=list(SEEDS)))

    fig, ax = plt.subplots(figsize=(7.2, 2.6))
    for (sensor, n), ls in ((("ir", "LA-GS"), "-"), (("camera", "LA-GS"), "-"), (("camera", "PID-NM"), ":")):
        lg = logs[(sensor, n)]
        c = COLORS["LA-GS"] if sensor == "ir" else ("tab:purple" if n == "LA-GS" else COLORS["PID-NM"])
        ax.plot(lg["s"], 1000 * lg["cte"], ls, color=c, lw=0.9, label=f"{n}, {sensor}")
    ax.set_xlabel("distance along track s (m)"); ax.set_ylabel("axle error (mm)")
    ax.set_title("Complex track: IR array vs camera"); ax.legend(ncol=3)
    savefig(fig, "camera_vs_ir")
    for sensor in summary:
        for n, r in summary[sensor].items():
            print(sensor, n, r["success"], round(1000 * (r["rms_cte"][0] or 0), 1), round(r["sensor_ms"][0], 2),
                  r["fail_modes"])


if __name__ == "__main__":
    main()
