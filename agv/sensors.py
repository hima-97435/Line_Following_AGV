"""Line sensors and line-position perception.

IRArray
    Eight downward reflectance sensors on a bar ahead of the axle.  Each sensor
    casts a PyBullet ray to find its floor footprint; the reflectance is the
    area-weighted mix of floor and tape inside the IR spot, plus Gaussian noise
    and 10-bit ADC quantisation.  The line position is the darkness-weighted
    centroid of the calibrated readings.

Camera
    A forward-looking pinhole camera rendered by PyBullet (TinyRenderer).  The
    image is thresholded, dark-pixel centroids are extracted row by row and
    back-projected onto the floor (inverse perspective mapping).  A quadratic
    fitted to the floor points gives the lateral offset at the look-ahead
    distance, the line heading and a curvature preview.
"""
from dataclasses import dataclass, field
import math

import numpy as np
import pybullet as pb

from .track import TAPE_WIDTH, disk_strip_coverage

FLOOR_REFLECT = 0.85
TAPE_REFLECT = 0.08


@dataclass
class LineReading:
    error: float = 0.0          # m, lateral offset of the line at the look-ahead point (+ = left)
    lost: bool = False
    heading: float = 0.0        # rad, line direction relative to robot (camera only)
    curvature: float = 0.0      # 1/m preview curvature (camera only)
    raw: np.ndarray = field(default_factory=lambda: np.zeros(0))


def _rotation(orn):
    return np.asarray(pb.getMatrixFromQuaternion(orn)).reshape(3, 3)


class IRArray:
    def __init__(self, cfg, world, rng):
        self.cfg, self.world, self.rng = cfg, world, rng
        sp = cfg.sensor
        self.y = (np.arange(sp.n_ir) - (sp.n_ir - 1) / 2) * sp.ir_pitch   # robot-frame lateral positions
        self.local_from = np.stack([np.full(sp.n_ir, sp.lookahead), self.y,
                                    np.full(sp.n_ir, -(cfg.robot.wheel_radius - 0.012))], axis=1)
        self.local_to = self.local_from - np.array([0, 0, 0.10])
        self.levels = 2 ** sp.adc_bits - 1
        self.last_side = 0.0

    def read(self, hint=None) -> LineReading:
        sp, p = self.cfg.sensor, self.world.p
        _, pos, orn = self.world.pose()
        R, t = _rotation(orn), np.asarray(pos)
        start = self.local_from @ R.T + t
        end = self.local_to @ R.T + t
        hits = p.rayTestBatch(start.tolist(), end.tolist())
        hit_xy = np.array([h[3][:2] if h[0] >= 0 else s[:2] for h, s in zip(hits, start)])
        d = self.world.track.distance_many(hit_xy, hint)
        cover = disk_strip_coverage(d, sp.ir_spot_radius, TAPE_WIDTH / 2)
        refl = FLOOR_REFLECT * (1 - cover) + TAPE_REFLECT * cover
        refl = refl + self.rng.normal(0.0, sp.noise_std, refl.shape)
        counts = np.round(np.clip(refl, 0, 1) * self.levels)                      # ADC
        # calibrated darkness in [0, 1] with a small noise floor removed
        dark = (FLOOR_REFLECT - counts / self.levels) / (FLOOR_REFLECT - TAPE_REFLECT)
        dark = np.clip((dark - 0.08) / 0.92, 0.0, 1.0)
        if sp.ir_method == "window":
            # centroid of the darkest sensor and its two neighbours: the tape (25 mm)
            # spans at most three sensors, so the far sensors only contribute noise
            k = int(np.argmax(dark))
            lo, hi = max(0, k - 1), min(len(dark), k + 2)
            w, y = dark[lo:hi], self.y[lo:hi]
        else:
            w, y = dark, self.y
        total = w.sum()
        if total < sp.lost_threshold:
            return LineReading(error=self.last_side * self.y.max(), lost=True, raw=counts)
        e = float(np.dot(w, y) / total)
        self.last_side = math.copysign(1.0, e) if abs(e) > 1e-4 else self.last_side
        return LineReading(error=e, lost=False, raw=counts)


def make_sensor(cfg, world, rng):
    if cfg.sensor_type == "ir":
        return IRArray(cfg, world, rng)
    raise ValueError(f"Unknown sensor type {cfg.sensor_type}")
