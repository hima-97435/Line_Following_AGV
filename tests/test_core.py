"""Unit tests for geometry, sensing model, controllers and estimator.

Run with:  python -m pytest -q
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agv.analysis import loop_margins, plant_frf, controller_frf, step_metrics  # noqa: E402
from agv.config import SimConfig, with_overrides  # noqa: E402
from agv.controllers import PID, FuzzyPD, LineKalman, Relay  # noqa: E402
from agv.track import Track, disk_strip_coverage, TAPE_WIDTH  # noqa: E402


def test_oval_length_and_closure():
    t = Track("oval")
    assert t.length == pytest.approx(2 * 2.5 + 2 * math.pi * 0.75, rel=1e-3)
    assert np.linalg.norm(t.pts[-1] - t.pts[0]) < 0.01


def test_nearest_signed_lateral():
    t = Track("straight")
    assert t.nearest(np.array([1.0, 0.03]), hint=0).lateral == pytest.approx(0.03, abs=1e-6)
    assert t.nearest(np.array([1.0, -0.02]), hint=0).lateral == pytest.approx(-0.02, abs=1e-6)


def test_arc_curvature_sign():
    t = Track("oval")
    assert t.curvature.max() == pytest.approx(1 / 0.75, rel=1e-6)   # left turns are positive


def test_disk_strip_coverage_limits():
    r, hw = 0.006, TAPE_WIDTH / 2
    assert disk_strip_coverage(0.0, r, hw) == pytest.approx(1.0)
    assert disk_strip_coverage(0.1, r, hw) == pytest.approx(0.0)
    assert disk_strip_coverage(hw, r, hw) == pytest.approx(0.5, abs=1e-9)   # spot centred on tape edge


def test_pid_proportional_and_saturation():
    pid = PID(10.0, 0.0, 0.0, 0.01, u_max=2.0)
    assert pid.step(0.05) == pytest.approx(0.5)
    assert pid.step(1.0) == pytest.approx(2.0)


def test_pid_anti_windup_limits_integral():
    pid = PID(1.0, 50.0, 0.0, 0.01, u_max=1.0)
    for _ in range(1000):
        pid.step(5.0)            # deep saturation
    assert pid.integral < 0.05   # conditional integration stopped the wind-up


def test_fuzzy_is_odd_and_bounded():
    f = FuzzyPD(0.01, 5.0, 0.05, 0.3)
    u_pos = f.step(0.03, de=0.0)
    f.reset()
    u_neg = f.step(-0.03, de=0.0)
    assert u_pos == pytest.approx(-u_neg)
    assert abs(FuzzyPD(0.01, 5.0, 0.05, 0.3).step(10.0, de=10.0)) <= 5.0


def test_relay_hysteresis():
    r = Relay(1.5, 0.001)
    assert r.step(0.002) == 1.5
    assert r.step(0.0005) == 1.5          # inside the band: holds
    assert r.step(-0.002) == -1.5


def test_kalman_reduces_noise():
    rng = np.random.default_rng(0)
    kf = LineKalman(1 / 60, q=4.0, r=1e-4)
    truth = 0.02 * np.sin(np.linspace(0, 6, 600))
    meas = truth + rng.normal(0, 0.01, truth.shape)
    est = np.array([kf.step(z)[0] for z in meas])
    assert np.sqrt(np.mean((est - truth) ** 2)) < 0.8 * np.sqrt(np.mean((meas - truth) ** 2))


def test_step_metrics_on_known_response():
    t = np.linspace(0, 10, 10001)
    zeta, wn = 1 / math.sqrt(2), 4.0
    wd = wn * math.sqrt(1 - zeta ** 2)
    y = np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta / math.sqrt(1 - zeta ** 2) * np.sin(wd * t))
    m = step_metrics(t, y)
    assert m["overshoot"] == pytest.approx(100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2)), rel=0.02)


def test_lookahead_gain_gives_expected_phase_margin():
    # kp* = 2v/L^2 with no actuator lag: zeta = 1/sqrt(2) -> large phase margin
    v, L = 0.4, 0.15
    w = np.logspace(-1, 3, 4000)
    Lw = controller_frf(w, 2 * v / L ** 2, 0, 0, 15) * plant_frf(w, v, L, 1e-4, 0.0)
    assert loop_margins(w, Lw)["phase_margin"] > 60


def test_with_overrides_nested():
    cfg = with_overrides(SimConfig(), control__kp=3.0, robot__payload_mass=5.0, track="complex")
    assert (cfg.control.kp, cfg.robot.payload_mass, cfg.track) == (3.0, 5.0, "complex")


@pytest.mark.slow
def test_pose_is_axle_not_centre_of_mass():
    # PyBullet reports the COM frame; World.pose() must return the axle centre
    from agv.track import Track
    from agv.world import World
    w = World(SimConfig(track="straight"), Track("straight"))
    w.reset_pose(1.0, 0.02, 0.3)
    w.step(10)
    pose, pos, _ = w.pose()
    w.close()
    assert pose[0] == pytest.approx(1.0, abs=2e-3) and pose[1] == pytest.approx(0.02, abs=2e-3)
    assert pos[2] == pytest.approx(SimConfig().robot.wheel_radius, abs=3e-3)


@pytest.mark.slow
def test_ir_array_measures_offset():
    from agv.sensors import IRArray
    from agv.track import Track
    from agv.world import World
    cfg = with_overrides(SimConfig(track="straight"), sensor__noise_std=0.0)
    w = World(cfg, Track("straight"))
    w.reset_pose(1.0, -0.02, 0.0)          # robot 2 cm right of the tape -> line appears 2 cm left
    w.step(10)
    r = IRArray(cfg, w, np.random.default_rng(0)).read(hint=200)
    w.close()
    assert not r.lost and r.error == pytest.approx(0.02, abs=0.003)


@pytest.mark.slow
def test_short_episode_runs():
    from agv.sim import run_episode
    cfg = with_overrides(SimConfig(track="straight"), stop_at_distance=0.5, max_time=5)
    _, s = run_episode(cfg)
    assert s["completed"]
