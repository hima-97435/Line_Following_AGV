"""Shared helpers for the experiment scripts (paths, parallel runs, plot style)."""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from agv.config import SimConfig, with_overrides  # noqa: E402
from agv.sim import run_episode  # noqa: E402

DATA = os.path.join(ROOT, "results", "data")
FIGS = os.path.join(ROOT, "results", "figures")
os.makedirs(DATA, exist_ok=True)
os.makedirs(FIGS, exist_ok=True)

N_WORKERS = max(1, min(14, (os.cpu_count() or 2) - 1))

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10, "axes.titlesize": 10, "axes.labelsize": 10, "legend.fontsize": 8,
    "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 110, "savefig.dpi": 200,
    "savefig.bbox": "tight",
})

COLORS = {
    "Bang-bang": "#7f7f7f", "PID-ZN": "#d62728", "PID-TL": "#ff7f0e", "PID-NM": "#1f77b4",
    "Fuzzy-NM": "#9467bd", "LA-GS": "#2ca02c", "LA-GS+": "#17becf", "P (fixed)": "#8c564b",
}


def save_json(name, obj):
    path = os.path.join(DATA, name + ".json")
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, default=_np_default)
    return path


def load_json(name):
    with open(os.path.join(DATA, name + ".json")) as f:
        return json.load(f)


def _np_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def savefig(fig, name):
    fig.savefig(os.path.join(FIGS, name + ".png"))
    plt.close(fig)


def _run(args):
    cfg, keep_log = args
    log, summary = run_episode(cfg)
    return (log if keep_log else None), summary


def run_many(cfgs, keep_logs=False):
    """Run episodes in parallel, preserving order."""
    with Pool(N_WORKERS) as pool:
        return pool.map(_run, [(c, keep_logs) for c in cfgs], chunksize=1)


def controller_set(gains):
    """Controllers compared in the study, as config overrides."""
    zn, tl, nm, fz = gains["PID-ZN"], gains["PID-TL"], gains["PID-NM"], gains["Fuzzy-NM"]
    pid = lambda g: dict(control__controller="pid", control__kp=g["kp"], control__ki=g["ki"], control__kd=g["kd"])
    return {
        "Bang-bang": dict(control__controller="bangbang"),
        "PID-ZN": pid(zn),
        "PID-TL": pid(tl),
        "PID-NM": pid(nm),
        "Fuzzy-NM": dict(control__controller="fuzzy", control__fuzzy_e_scale=fz["fuzzy_e_scale"],
                         control__fuzzy_de_scale=fz["fuzzy_de_scale"]),
        "LA-GS": dict(control__controller="pid", control__gain_schedule=True, control__ki=0.0, control__kd=0.0),
        "LA-GS+": dict(control__controller="pid", control__gain_schedule=True, control__ki=0.0, control__kd=0.0,
                       control__edge_guard=True, control__use_kalman=True, control__adaptive_speed=True),
    }


ENVELOPE = 0.05   # m: an AGV is "in envelope" if it finishes with max axle deviation <= 50 mm


def envelope_ok(summary, tol=ENVELOPE):
    return bool(summary.get("completed") and summary.get("max_cte", 1.0) <= tol)


def mean_std(values):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    if len(a) == 0:
        return float("nan"), float("nan")
    return float(a.mean()), float(a.std())


__all__ = ["SimConfig", "with_overrides", "run_episode", "run_many", "save_json", "load_json",
           "savefig", "plt", "np", "COLORS", "controller_set", "mean_std", "DATA", "FIGS", "ROOT",
           "envelope_ok", "ENVELOPE"]
