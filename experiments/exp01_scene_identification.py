"""E1: simulation scene renders, track maps and actuator (yaw-rate) identification.

The yaw-rate response to a commanded step is fitted with a first-order lag plus
dead time.  The identified (tau, Td) feed the linear model used in E4/E5.
"""
import math

from common import *  # noqa: F401,F403
from agv.analysis import fit_first_order
from agv.track import Track, TRACK_SPECS
from agv.world import World
import pybullet as pb


def render_scene(track_name, name, eye, target, w=1000, h=620, fov=50):
    cfg = SimConfig(track=track_name)
    track = Track(track_name)
    world = World(cfg, track)
    p = world.p
    # place the robot at a characteristic point (25 % along the track)
    i = int(0.18 * track.n)
    x, y = track.pts[i]
    world.reset_pose(x, y, track.heading[i])
    view = p.computeViewMatrix(eye(x, y), target(x, y), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov, w / h, 0.02, 20)
    img = p.getCameraImage(w, h, view, proj, renderer=pb.ER_TINY_RENDERER,
                           lightDirection=[1, 1, 2], shadow=1)
    rgb = np.reshape(np.asarray(img[2], dtype=np.uint8), (h, w, 4))[..., :3]
    world.close()
    fig = plt.figure(figsize=(w / 200, h / 200))
    ax = fig.add_axes([0, 0, 1, 1]); ax.imshow(rgb); ax.axis("off")
    savefig(fig, name)


def track_maps():
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.6), gridspec_kw=dict(width_ratios=[1.3, 1]))
    info = {}
    for ax, name in zip(axs, ["oval", "complex"]):
        t = Track(name)
        sc = ax.scatter(t.pts[:, 0], t.pts[:, 1], c=np.abs(t.curvature), s=2, cmap="viridis", vmin=0, vmax=8.5)
        ax.plot(*t.pts[0], "o", color="tab:red", ms=6, label="start")
        if not t.closed:
            ax.plot(*t.pts[-1], "s", color="black", ms=5, label="end")
        ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title(f"{name} track, L = {t.length:.2f} m")
        ax.legend(loc="best")
        radii = sorted({seg[1] for seg in TRACK_SPECS[name]["segments"] if seg[0] == "A"})
        info[name] = dict(length=t.length, closed=t.closed, n_segments=len(t.segments), min_radius=min(radii),
                          radii=radii, max_curvature=float(np.abs(t.curvature).max()))
    cb = fig.colorbar(sc, ax=axs, shrink=0.8); cb.set_label("|curvature| (1/m)")
    savefig(fig, "track_maps")
    return info


def identify(payload=0.0, v=0.4, w_step=1.5):
    cfg = with_overrides(SimConfig(track="straight"), robot__payload_mass=payload)
    world = World(cfg, Track("straight"))
    r, b = cfg.robot.wheel_radius, cfg.robot.track_width
    dt = cfg.control_dt
    t_log, w_log, v_log, cmd = [], [], [], []
    for k in range(int(3.0 / dt)):
        t = k * dt
        w_c = w_step if t >= 1.0 else 0.0
        world.set_wheel_targets((v - w_c * b / 2) / r, (v + w_c * b / 2) / r)
        world.step(cfg.control_every)
        vm, wm = world.velocity()
        t_log.append(t + dt); w_log.append(wm); v_log.append(vm); cmd.append(w_c)
    world.close()
    t_log, w_log = np.asarray(t_log), np.asarray(w_log)
    m = t_log >= 1.0
    fit = fit_first_order(t_log[m] - 1.0, w_log[m], w_step)
    return dict(payload=payload, **fit, v_ss=float(np.mean(v_log[-30:]))), (t_log, w_log, np.asarray(cmd))


def main():
    render_scene("complex", "scene_overview",
                 eye=lambda x, y: [x - 1.4, y - 1.6, 1.5], target=lambda x, y: [x + 0.3, y + 0.4, 0])
    render_scene("oval", "scene_robot_closeup",
                 eye=lambda x, y: [x - 0.45, y - 0.55, 0.35], target=lambda x, y: [x + 0.1, y, 0.02], fov=45)
    info = track_maps()

    results, traces = [], {}
    for payload in (0.0, 10.0, 20.0):
        res, tr = identify(payload)
        results.append(res); traces[payload] = tr
    dt = SimConfig().control_dt
    nominal = results[0]
    ident = dict(runs=results, tau=nominal["tau"], delay=nominal["delay"],
                 loop_delay=nominal["delay"] + dt / 2,    # + half-sample zero-order hold
                 control_dt=dt, tracks=info)
    save_json("identification", ident)

    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    for payload, (t, w, c) in traces.items():
        ax.plot(t, w, label=f"PyBullet yaw rate, payload {payload:.0f} kg")
    t, w, c = traces[0.0]
    K, tau, Td = nominal["gain"], nominal["tau"], nominal["delay"]
    model = np.where(t >= 1 + Td, K * 1.5 * (1 - np.exp(-(t - 1 - Td) / tau)), 0)
    ax.plot(t, c, "k:", label="commanded yaw rate")
    ax.plot(t, model, "k--", lw=1, label=f"fit: tau = {tau*1000:.0f} ms, Td = {Td*1000:.0f} ms")
    ax.set_xlim(0.8, 1.8); ax.set_xlabel("time (s)"); ax.set_ylabel("yaw rate (rad/s)")
    ax.legend(loc="lower right")
    savefig(fig, "identification_step")
    print("identification:", {k: v for k, v in ident.items() if k != "tracks"})


if __name__ == "__main__":
    main()
