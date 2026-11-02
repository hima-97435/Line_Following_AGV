"""Live demonstration of the line-following AGV.

GUI (PyBullet window, real time):
    python scripts/demo.py --track complex --controller LA-GS+ --speed 0.5
    python scripts/demo.py --sensor camera --controller LA-GS
Headless (writes an animated GIF of the run, e.g. for slides):
    python scripts/demo.py --headless --gif results/figures/demo_complex.gif

Controllers: Bang-bang, PID-ZN, PID-TL, PID-NM, Fuzzy-NM, LA-GS, LA-GS+
"""
import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "experiments"))

import numpy as np  # noqa: E402

from agv.config import SimConfig, with_overrides  # noqa: E402
from agv.sim import run_episode  # noqa: E402
from agv.track import Track  # noqa: E402

DEFAULT_GAINS = {  # used only if results/data/gains.json has not been generated yet
    "PID-ZN": dict(kp=79.1, ki=501.0, kd=3.13), "PID-TL": dict(kp=60.0, ki=86.3, kd=3.01),
    "PID-NM": dict(kp=31.0, ki=1.0, kd=0.35), "Fuzzy-NM": dict(fuzzy_e_scale=0.07, fuzzy_de_scale=5.0),
}


def controller_overrides(name):
    path = os.path.join(ROOT, "results", "data", "gains.json")
    gains = json.load(open(path)) if os.path.exists(path) else DEFAULT_GAINS
    from common import controller_set
    return controller_set(gains)[name]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="complex", choices=["oval", "complex", "straight"])
    ap.add_argument("--controller", default="LA-GS+")
    ap.add_argument("--sensor", default="ir", choices=["ir", "camera"])
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--noise", type=float, default=0.02)
    ap.add_argument("--payload", type=float, default=0.0)
    ap.add_argument("--offset", type=float, default=0.0, help="initial lateral offset (m)")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--gif", default=None)
    ap.add_argument("--realtime", type=float, default=1.0, help="playback speed factor in the GUI")
    a = ap.parse_args()

    cfg = with_overrides(SimConfig(track=a.track, gui=not a.headless, max_time=150, laps=a.laps,
                                   initial_offset=a.offset),
                         control__base_speed=a.speed, sensor__kind=a.sensor, sensor__noise_std=a.noise,
                         robot__payload_mass=a.payload, **controller_overrides(a.controller))
    track = Track(a.track)
    state = dict(prev=None, t0=time.time(), text=None)

    def gui_callback(world, k, pose, reading, w_cmd, v_cmd):
        p = world.p
        if k == 0:
            pts = track.pts
            for i in range(0, len(pts) - 8, 8):    # thin reference line above the tape
                p.addUserDebugLine([*pts[i], 0.002], [*pts[i + 8], 0.002], [0.1, 0.4, 1.0], 1)
        cur = [pose[0], pose[1], 0.01]
        if state["prev"] is not None and k % 3 == 0:
            p.addUserDebugLine(state["prev"], cur, [1, 0.2, 0.1], 2, lifeTime=0)
        if k % 3 == 0:
            state["prev"] = cur
        if k % 6 == 0:
            p.resetDebugVisualizerCamera(1.2, math.degrees(pose[2]) - 90, -40, [pose[0], pose[1], 0])
            msg = (f"{a.controller} | {a.sensor} | v={v_cmd:.2f} m/s  w={w_cmd:+.2f} rad/s  "
                   f"e={reading.error*1000:+.1f} mm{'  LINE LOST' if reading.lost else ''}")
            if state["text"] is None:
                state["text"] = p.addUserDebugText(msg, [pose[0], pose[1], 0.35], [0, 0, 0], 1.2)
            else:
                state["text"] = p.addUserDebugText(msg, [pose[0], pose[1], 0.35], [0, 0, 0], 1.2,
                                                   replaceItemUniqueId=state["text"])
        # real-time pacing
        target = (k + 1) * cfg.control_dt / a.realtime
        lag = target - (time.time() - state["t0"])
        if lag > 0:
            time.sleep(lag)

    log, summary = run_episode(cfg, callback=None if a.headless else gui_callback)
    print("\nRun summary:")
    for k in ("status", "time", "avg_speed", "rms_cte", "max_cte", "rms_e", "smoothness", "energy", "lost_frac"):
        v = summary.get(k)
        print(f"  {k:12s} {v:.4f}" if isinstance(v, float) else f"  {k:12s} {v}")
    if a.gif:
        make_gif(log, track, a, summary)


def make_gif(log, track, a, summary, fps=15):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(8, 4), gridspec_kw=dict(width_ratios=[1.2, 1]))
    ax.plot(track.pts[:, 0], track.pts[:, 1], color="k", lw=5, alpha=0.2)
    path, = ax.plot([], [], color="tab:red", lw=1.2)
    body, = ax.plot([], [], color="tab:orange", lw=3)
    ax.set_aspect("equal"); ax.set_title(f"{a.controller}, {a.sensor}, v_max = {a.speed} m/s")
    err, = ax2.plot([], [], color="tab:blue")
    ax2.set_xlim(0, log["t"][-1]); m = max(5, 1000 * np.abs(log["cte"]).max() * 1.1)
    ax2.set_ylim(-m, m); ax2.set_xlabel("time (s)"); ax2.set_ylabel("axle error (mm)"); ax2.grid(alpha=0.3)
    step = max(1, int(round(1 / (fps * (log["t"][1] - log["t"][0])))))
    frames = range(0, len(log["t"]), step)

    def draw(i):
        x, y, th = log["x"][i], log["y"][i], log["yaw"][i]
        path.set_data(log["x"][:i + 1], log["y"][:i + 1])
        c, s = math.cos(th), math.sin(th)
        body.set_data([x - 0.13 * c, x + 0.17 * c], [y - 0.13 * s, y + 0.17 * s])
        err.set_data(log["t"][:i + 1], 1000 * log["cte"][:i + 1])
        return path, body, err

    FuncAnimation(fig, draw, frames=frames, blit=True).save(a.gif, writer=PillowWriter(fps=fps))
    print("GIF written to", a.gif)


if __name__ == "__main__":
    main()
