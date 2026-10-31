"""Closed-loop episode runner.

Control architecture (declarative -> procedural -> reflexive):

* declarative : mission = follow the tape to the end of the track / N laps
* procedural  : perception -> (Kalman) -> steering controller + speed scheduler
* reflexive   : line-loss recovery (turn towards last seen side) and safety
                stop on derailment, independent of the steering controller
"""
import math
import time

import numpy as np

from .config import SimConfig
from .controllers import LineKalman, SpeedScheduler, make_controller
from .sensors import make_sensor
from .track import Track
from .world import World

LOG_KEYS = ["t", "x", "y", "yaw", "v", "w", "e_meas", "e_hat", "e_true", "cte", "lost",
            "w_cmd", "v_cmd", "s", "curv", "power"]


def run_episode(cfg: SimConfig, callback=None, keep_world=False):
    rng = np.random.default_rng(cfg.seed)
    track = Track(cfg.track)
    world = World(cfg, track)
    sensor = make_sensor(cfg, world, rng)
    cp, rp = cfg.control, cfg.robot
    dt = cfg.control_dt
    ctrl = make_controller(cp, dt)
    kf = LineKalman(dt, cp.kf_q, cp.kf_r) if cp.use_kalman else None
    sched = SpeedScheduler(cp, dt)

    if cfg.stop_at_distance is not None:
        target = cfg.stop_at_distance
    elif track.closed:
        target = cfg.laps * track.length
    else:  # stop where an end-of-line marker would be, before the sensors run off the tape
        target = track.length - 0.35

    log = {k: [] for k in LOG_KEYS}
    pose, _, _ = world.pose()
    near = track.nearest(pose, hint=0)
    idx = near.idx
    travelled = 0.0
    lost_time = 0.0
    status = "timeout"
    t = 0.0
    sensor_time = 0.0
    n_steps = int(cfg.max_time / dt)
    L_eff = cfg.sensor.effective_lookahead
    v_ref = cp.base_speed

    for k in range(n_steps):
        # ---------------- perception
        t0 = time.perf_counter()
        hint = (idx + int(cfg.sensor.effective_lookahead / 0.005)) % track.n if track.closed else \
            min(idx + int(cfg.sensor.effective_lookahead / 0.005), track.n - 1)
        reading = sensor.read(hint=hint)
        sensor_time += time.perf_counter() - t0

        if kf is not None:
            e_hat, de_hat = kf.step(reading.error, valid=not reading.lost)
        else:
            e_hat, de_hat = reading.error, None

        # ---------------- control (procedural) + reflexive supervisor
        if reading.lost:
            lost_time += dt
            side = math.copysign(1.0, reading.error) if reading.error != 0 else 1.0
            w_cmd = side * 0.8 * cp.omega_max
            v_cmd = 0.5 * cp.v_min
            if lost_time > cfg.lost_timeout:
                status = "line_lost"
                break
        else:
            lost_time = 0.0
            if cp.gain_schedule:
                # zero steady-state axle offset on any arc (and zeta = 1/sqrt 2): kp = 2 v / L^2
                ctrl.kp = cp.kp_factor * 2.0 * max(v_ref, 0.05) / L_eff ** 2
            w_cmd = ctrl.step(e_hat, de_hat)
            if cp.edge_guard:
                # keep the line inside the sensor's field of view on corners tighter than
                # R_min = L^2 / (2 * half_span), where zero axle offset is unattainable
                excess = abs(e_hat) - cp.edge_frac * cfg.sensor.half_span
                if excess > 0:
                    w_cmd = float(np.clip(w_cmd + math.copysign(cp.edge_gain * excess, e_hat),
                                          -cp.omega_max, cp.omega_max))
            v_cmd = sched.step(e_hat, w_cmd, reading.curvature)
        v_ref = v_cmd

        # ---------------- differential-drive inverse kinematics + saturation
        r, b = rp.wheel_radius, rp.track_width
        wl = (v_cmd - w_cmd * b / 2) / r
        wr = (v_cmd + w_cmd * b / 2) / r
        peak = max(abs(wl), abs(wr))
        if peak > rp.wheel_max_speed:          # keep the turn rate, give up forward speed
            excess = (peak - rp.wheel_max_speed) * r
            v_sat = max(0.0, v_cmd - excess)
            wl = (v_sat - w_cmd * b / 2) / r
            wr = (v_sat + w_cmd * b / 2) / r
        world.set_wheel_targets(wl, wr)
        world.step(cfg.control_every)
        t += dt

        # ---------------- ground truth & logging
        pose, pos, orn = world.pose()
        near = track.nearest(pose, hint=idx)
        travelled += track.progress(idx, near.idx)
        idx = near.idx
        la = pose[:2] + cfg.sensor.effective_lookahead * np.array([math.cos(pose[2]), math.sin(pose[2])])
        e_true = -track.nearest(la, hint=idx).lateral
        v_meas, w_meas = world.velocity()
        js = world.p.getJointStates(world.robot, [world.jl, world.jr])
        power = sum(abs(s[1] * s[3]) for s in js)

        if k % cfg.record_every == 0:
            vals = [t, pose[0], pose[1], pose[2], v_meas, w_meas, reading.error, e_hat, e_true,
                    near.lateral, float(reading.lost), w_cmd, v_cmd, travelled, near.curvature, power]
            for key, val in zip(LOG_KEYS, vals):
                log[key].append(val)
        if callback is not None:
            callback(world, k, pose, reading, w_cmd, v_cmd)

        if abs(near.lateral) > cfg.fail_distance:
            status = "derailed"
            break
        if travelled >= target:
            status = "completed"
            break

    log = {k: np.asarray(v) for k, v in log.items()}
    summary = summarize(log, cfg, status, track, sensor_time, k + 1)
    if keep_world:
        return log, summary, world
    world.close()
    return log, summary


def summarize(log, cfg, status, track, sensor_time=0.0, steps=1):
    dt = cfg.control_dt
    out = {"status": status, "completed": status == "completed", "track": cfg.track,
           "controller": cfg.control.controller, "sensor": cfg.sensor.kind}
    if len(log["t"]) == 0:
        return out
    cte = log["cte"]
    e = log["e_true"]
    ok = log["lost"] < 0.5
    w = log["w_cmd"]
    out.update(
        time=float(log["t"][-1]),
        distance=float(log["s"][-1]),
        avg_speed=float(log["s"][-1] / max(log["t"][-1], 1e-9)),
        mean_abs_cte=float(np.mean(np.abs(cte))),
        rms_cte=float(np.sqrt(np.mean(cte ** 2))),
        max_cte=float(np.max(np.abs(cte))),
        mean_abs_e=float(np.mean(np.abs(e))),
        rms_e=float(np.sqrt(np.mean(e ** 2))),
        max_e=float(np.max(np.abs(e))),
        sensor_rmse=float(np.sqrt(np.mean((log["e_meas"][ok] - e[ok]) ** 2))) if ok.any() else float("nan"),
        estimate_rmse=float(np.sqrt(np.mean((log["e_hat"][ok] - e[ok]) ** 2))) if ok.any() else float("nan"),
        lost_frac=float(1 - ok.mean()),
        effort=float(np.mean(np.abs(w))),
        smoothness=float(np.mean(np.abs(np.diff(w))) / dt) if len(w) > 1 else 0.0,
        energy=float(np.sum(log["power"]) * dt),
        sensor_ms=1000.0 * sensor_time / max(steps, 1),
    )
    return out
