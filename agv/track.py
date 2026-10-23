"""Track (tape line) geometry.

A track is described by a list of primitive segments (straight lines and
circular arcs).  It is sampled into a dense polyline that is used for

* ground-truth cross-track error (performance metrics),
* the IR reflectance model (how much of a sensor spot is covered by tape),
* rasterising the floor texture that PyBullet renders for the camera.
"""
from dataclasses import dataclass
import math
import numpy as np

TAPE_WIDTH = 0.025   # m, 1 inch floor-marking tape
SAMPLE_DS = 0.005    # m, polyline resolution

# ("S", length) or ("A", radius, angle_deg); positive angle = left (CCW) turn
TRACK_SPECS = {
    "straight": dict(closed=False, segments=[("S", 7.0)]),
    "oval": dict(closed=True, segments=[
        ("S", 2.5), ("A", 0.75, 180), ("S", 2.5), ("A", 0.75, 180)]),
    # unseen validation track: S-bends, tight hairpin, two near-90 deg corners
    "complex": dict(closed=False, segments=[
        ("S", 1.0), ("A", 0.50, 90), ("S", 0.3), ("A", 0.40, -90), ("S", 0.4),
        ("A", 0.12, -90),                      # tight near-90 deg corner
        ("S", 0.5), ("A", 0.30, 180),          # hairpin
        ("S", 0.9), ("A", 0.40, -60), ("A", 0.40, 60),   # chicane / S-bend
        ("S", 0.4), ("A", 0.15, 90),           # second tight corner
        ("S", 1.0), ("A", 0.50, -90), ("S", 0.5), ("A", 0.35, 90), ("S", 1.0)]),
}


@dataclass
class Nearest:
    idx: int
    lateral: float     # signed distance, positive = point lies LEFT of the track
    s: float           # arc length of nearest polyline point
    heading: float     # track heading at that point
    curvature: float


class Track:
    def __init__(self, name: str):
        if name not in TRACK_SPECS:
            raise KeyError(f"unknown track '{name}', choose from {list(TRACK_SPECS)}")
        spec = TRACK_SPECS[name]
        self.name = name
        self.closed = spec["closed"]
        self.segments = spec["segments"]
        self._build()

    # ------------------------------------------------------------------ build
    def _build(self):
        x, y, th = 0.0, 0.0, 0.0
        pts, heads, curvs = [(x, y)], [th], [0.0]
        for seg in self.segments:
            if seg[0] == "S":
                n = max(1, int(round(seg[1] / SAMPLE_DS)))
                ds = seg[1] / n
                for _ in range(n):
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                    pts.append((x, y)); heads.append(th); curvs.append(0.0)
            else:
                r, ang = seg[1], math.radians(seg[2])
                length = abs(ang) * r
                n = max(1, int(round(length / SAMPLE_DS)))
                dth = ang / n
                k = math.copysign(1.0 / r, ang)
                for _ in range(n):
                    # exact chord of a circular arc
                    chord = 2 * r * math.sin(abs(dth) / 2)
                    mid = th + dth / 2
                    x += chord * math.cos(mid)
                    y += chord * math.sin(mid)
                    th += dth
                    pts.append((x, y)); heads.append(th); curvs.append(k)
        self.pts = np.asarray(pts)
        if self.closed:  # drop the duplicated start point
            self.pts = self.pts[:-1]
            heads, curvs = heads[:-1], curvs[:-1]
        self.heading = np.asarray(heads)
        self.curvature = np.asarray(curvs)
        d = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.length = self.s[-1] + (np.linalg.norm(self.pts[0] - self.pts[-1]) if self.closed else 0.0)
        self.n = len(self.pts)
        self.normal = np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)

    # ---------------------------------------------------------------- queries
    def bounds(self, margin=0.6):
        lo = self.pts.min(axis=0) - margin
        hi = self.pts.max(axis=0) + margin
        return lo, hi

    def _window(self, hint, back=80, ahead=400):
        if hint is None:
            return np.arange(self.n)
        idx = np.arange(hint - back, hint + ahead)
        if self.closed:
            return idx % self.n
        return idx[(idx >= 0) & (idx < self.n)]

    def nearest(self, p, hint=None) -> Nearest:
        idx = self._window(hint)
        d2 = np.sum((self.pts[idx] - p[:2]) ** 2, axis=1)
        i = int(idx[np.argmin(d2)])
        lateral = float(np.dot(p[:2] - self.pts[i], self.normal[i]))
        return Nearest(i, lateral, float(self.s[i]), float(self.heading[i]), float(self.curvature[i]))

    def distance_many(self, P, hint=None):
        """Unsigned distance of many points (N,2) to the tape centre-line."""
        idx = self._window(hint, back=120, ahead=200)
        seg = self.pts[idx]
        d2 = np.sum((P[:, None, :] - seg[None, :, :]) ** 2, axis=2)
        return np.sqrt(d2.min(axis=1))

    def progress(self, i_prev, i_now):
        """Signed arc-length travelled between two polyline indices (handles wrap)."""
        ds = self.s[i_now] - self.s[i_prev]
        if self.closed:
            if ds < -self.length / 2:
                ds += self.length
            elif ds > self.length / 2:
                ds -= self.length
        return float(ds)

    # ---------------------------------------------------------------- texture
    def render_texture(self, px_per_m=250, path=None, speckle=True, seed=1):
        """Rasterise the floor (light) with the tape (dark) to an RGB image."""
        from PIL import Image, ImageDraw
        lo, hi = self.bounds()
        w = int(math.ceil((hi[0] - lo[0]) * px_per_m))
        h = int(math.ceil((hi[1] - lo[1]) * px_per_m))
        ss = 2  # supersampling for clean anti-aliased edges
        img = Image.new("L", (w * ss, h * ss), 215)
        drw = ImageDraw.Draw(img)
        pts = self.pts if not self.closed else np.vstack([self.pts, self.pts[:1]])
        # image row 0 = max y (top of the floor seen from above); PIL addresses pixel
        # centres at integer coordinates, hence the -0.5 px registration shift
        uv = [((x - lo[0]) * px_per_m * ss - 0.5, (hi[1] - y) * px_per_m * ss - 0.5) for x, y in pts]
        drw.line(uv, fill=25, width=int(round(TAPE_WIDTH * px_per_m * ss)), joint="curve")
        img = img.resize((w, h), Image.LANCZOS)
        arr = np.asarray(img, dtype=np.float32)
        if speckle:
            rng = np.random.default_rng(seed)
            arr = np.clip(arr + rng.normal(0, 6, arr.shape), 0, 255)
        rgb = np.repeat(arr[:, :, None], 3, axis=2).astype(np.uint8)
        rgb[..., 2] = np.clip(rgb[..., 2].astype(int) + 8, 0, 255)  # slightly cool floor tone
        out = Image.fromarray(rgb)
        if path is not None:
            out.save(path)
        return out, lo, hi


def disk_strip_coverage(d, r, half_w):
    """Fraction of a disk (radius r, centre at distance d from a strip's centre
    line) that lies inside the strip |x| < half_w.  Vectorised."""
    def area_below(a):
        a = np.clip(a, -r, r)
        return r * r * np.arccos(-a / r) + a * np.sqrt(np.maximum(r * r - a * a, 0.0))
    d = np.asarray(d, dtype=float)
    return (area_below(half_w - d) - area_below(-half_w - d)) / (math.pi * r * r)
