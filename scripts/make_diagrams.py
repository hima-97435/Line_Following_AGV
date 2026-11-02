"""Draw the system block diagram, control-loop flowchart and software architecture
figures used in the report (pure matplotlib, so they are reproducible)."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "experiments"))
from common import plt, savefig  # noqa: E402
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch  # noqa: E402


def box(ax, x, y, w, h, text, fc="#e8f1fb", ec="#1f4e79", fs=8, bold=False):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=fc, ec=ec, lw=1.0))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal", wrap=True)


def arrow(ax, p, q, text=None, fs=7, rad=0.0, off=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=9, lw=0.9, color="#333333",
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha="center", va="center",
                fontsize=fs, color="#333333", style="italic")


def block_diagram():
    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    ax.set_xlim(0, 10.6); ax.set_ylim(0, 5); ax.axis("off")
    fs = 7.2
    box(ax, 1.2, 4.2, 2.1, 0.85, "Mission layer\n(declarative)\ntrack, laps, v_max", fc="#fdf2e3", ec="#b26b00", fs=fs)
    box(ax, 1.2, 2.5, 2.1, 1.0, "Line sensors\nIR array (8 rays)\nor camera (80x60)", fc="#e9f6ec", ec="#2e7d32", fs=fs)
    box(ax, 3.7, 2.5, 2.0, 1.0, "Perception\ncentroid / IPM\nline fit: e, kappa", fs=fs)
    box(ax, 6.05, 2.5, 1.8, 1.0, "Kalman filter\nx = [e, de/dt]", fs=fs)
    box(ax, 6.05, 4.2, 2.3, 0.85, "Speed scheduler\nv = f(|e|, |w|, kappa)", fs=fs)
    box(ax, 9.1, 2.5, 2.4, 1.0, "Steering controller\nLA-GS / PID / fuzzy\nw = C(e_hat)", fs=fs)
    box(ax, 9.1, 0.75, 2.4, 0.95, "Reflexive supervisor\nline-loss recovery,\nderail stop", fc="#fbe9e9", ec="#b71c1c", fs=fs)
    box(ax, 5.75, 0.75, 2.7, 0.95, "Diff-drive IK + saturation\nw_L,R = (v -/+ w b/2) / r", fs=fs)
    box(ax, 1.75, 0.75, 2.9, 0.95, "PyBullet plant: motors\n(tau = 60 ms), wheels, casters,\nfriction, payload",
        fc="#eeeeee", ec="#424242", fs=fs)
    arrow(ax, (1.2, 1.23), (1.2, 2.0), "floor / tape", off=(-0.62, 0))
    arrow(ax, (2.25, 2.5), (2.7, 2.5))
    arrow(ax, (4.7, 2.5), (5.15, 2.5), "e")
    arrow(ax, (6.95, 2.5), (7.9, 2.5), "e_hat")
    arrow(ax, (4.2, 3.0), (5.2, 3.8), "kappa", off=(-0.3, 0.05))
    arrow(ax, (2.25, 4.2), (4.9, 4.2), "v_max, L")
    arrow(ax, (8.5, 3.0), (7.0, 3.8), "w", off=(0.1, -0.1))
    arrow(ax, (7.2, 4.2), (9.3, 3.0), "v  (kp = 2v / L^2)", off=(1.05, 0.25))
    arrow(ax, (9.1, 2.0), (9.1, 1.23), "w", off=(0.2, 0))
    arrow(ax, (7.9, 0.75), (7.1, 0.75), "v, w", off=(0, 0.15))
    arrow(ax, (4.4, 0.75), (3.2, 0.75), "wheel cmds", off=(0, 0.17))
    savefig(fig, "system_block_diagram")


def flowchart():
    fig, ax = plt.subplots(figsize=(4.6, 6.6))
    ax.set_xlim(0, 6.6); ax.set_ylim(0, 11.5); ax.axis("off")
    steps = [
        (10.8, "Start episode: load track, spawn AGV,\nsettle 0.25 s", "#fdf2e3"),
        (9.6, "Read sensor (8 ray casts / render frame)\n-> line error e, lost flag, curvature", "#e9f6ec"),
        (8.4, "Kalman predict / update (if enabled)", "#e8f1fb"),
        (7.2, "Line lost?", "#fff8c4"),
        (6.0, "kp = 2 v / L^2 (LA-GS);  w = C(e_hat)\nv = speed scheduler(e, w, kappa)", "#e8f1fb"),
        (4.8, "Inverse kinematics, wheel-speed saturation\n(keep turn rate, reduce v)", "#e8f1fb"),
        (3.6, "Physics: 4 x 1/240 s steps with motor lag", "#eeeeee"),
        (2.4, "Ground truth: cross-track error, progress;\nlog metrics", "#eeeeee"),
        (1.2, "End reached / derailed / timeout?", "#fff8c4"),
    ]
    for y, t, c in steps:
        box(ax, 3.0, y, 4.6, 0.8, t, fc=c, fs=7.5)
    for (y1, _, _), (y2, _, _) in zip(steps[:-1], steps[1:]):
        arrow(ax, (3.0, y1 - 0.4), (3.0, y2 + 0.4))
    box(ax, 6.0, 6.0, 0.95, 1.2, "Recovery:\nturn to\nlast side,\nslow", fc="#fbe9e9", ec="#b71c1c", fs=6.5)
    arrow(ax, (5.3, 7.2), (6.0, 6.62), "yes", off=(0.3, 0.12))
    ax.text(2.75, 6.6, "no", fontsize=7, style="italic")
    arrow(ax, (6.0, 5.38), (5.3, 4.95))
    ax.annotate("", xy=(0.55, 9.6), xytext=(0.55, 1.2),
                arrowprops=dict(arrowstyle="-|>", lw=0.9, color="#333333", connectionstyle="arc3,rad=0"))
    ax.text(0.3, 5.4, "no: next control step (60 Hz)", rotation=90, fontsize=7, va="center", style="italic")
    ax.text(3.0, 0.4, "yes -> stop, summarise metrics", ha="center", fontsize=7, style="italic")
    savefig(fig, "control_flowchart")


def software_architecture():
    fig, ax = plt.subplots(figsize=(7.4, 3.2))
    ax.set_xlim(0, 10); ax.set_ylim(0, 4.4); ax.axis("off")
    mods = [
        (1.1, 3.4, "config.py\nall parameters"), (3.1, 3.4, "track.py\ngeometry, texture,\ncoverage model"),
        (5.1, 3.4, "world.py\nPyBullet, URDF,\nmotor model"), (7.1, 3.4, "sensors.py\nIR rays, camera,\nIPM line fit"),
        (9.0, 3.4, "controllers.py\nPID, fuzzy, relay,\nKalman, scheduler"),
        (3.1, 1.9, "sim.py\nclosed loop,\nsupervisor, metrics"), (5.6, 1.9, "analysis.py\nstep metrics, FRF,\nlinear model"),
        (8.1, 1.9, "tuning.py\nrelay, ZN/TL,\nNelder-Mead"),
        (2.0, 0.55, "experiments/\nexp01 ... exp10"), (5.0, 0.55, "results/\ndata (JSON), figures"),
        (8.0, 0.55, "scripts/ demo.py,\nreport/ build_report.py"),
    ]
    for x, y, t in mods:
        box(ax, x, y, 1.75, 0.95, t, fs=7)
    for src, dst in [((3.1, 2.9), (3.1, 2.4)), ((5.1, 2.9), (3.6, 2.4)), ((7.1, 2.9), (3.9, 2.4)),
                     ((9.0, 2.9), (4.0, 2.35)), ((8.1, 1.4), (2.6, 1.0)), ((3.1, 1.4), (2.2, 1.05)),
                     ((5.6, 1.4), (2.4, 1.0)), ((2.9, 0.55), (4.1, 0.55)), ((5.9, 0.55), (7.1, 0.55))]:
        arrow(ax, src, dst)
    savefig(fig, "software_architecture")


if __name__ == "__main__":
    block_diagram(); flowchart(); software_architecture()
    print("diagrams written")
