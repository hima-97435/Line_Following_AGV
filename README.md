# Line-Following AGV with PID Tuning and Response Analysis

Course project for **CSAI4014 Algorithms for Intelligent Systems and Robotics (AISR)**,
B.Tech CSE (AI & ML), Semester VII.

A differential-drive automated guided vehicle (AGV) follows floor tape in a
PyBullet physics simulation. The project compares seven steering controllers,
tunes PID gains three different ways, and analyses the closed loop in the
time and frequency domains. It also proposes a **look-ahead gain-scheduled
controller (LA-GS)** whose gain follows from the geometry of the sensor bar:

> **kp\* = 2 v / L²**: on an arc of *any* radius this is the only proportional
> gain that keeps the axle (and the load) on the tape. It also gives a
> damping ratio of exactly 1/√2 in the linearised loop.

The black-box Nelder–Mead optimiser rediscovers this gain independently, and the
PyBullet experiments confirm it (see `results/` and the report).

## Repository layout

```
agv/                  simulation package
  config.py           every parameter (robot, sensors, control, simulation)
  track.py            tape geometry, floor texture, IR spot-coverage model
  world.py            PyBullet world, generated AGV URDF, motor lag model
  sensors.py          ray-cast IR reflectance array, rendered camera + IPM line fit
  controllers.py      PID, bang-bang, fuzzy (Sugeno) PD, relay, Kalman filter, speed scheduler
  sim.py              closed-loop runner, reflexive supervisor, metrics
  analysis.py         step metrics, identification, loop FRF/margins, linear model
  tuning.py           relay auto-tuning, Ziegler-Nichols/Tyreus-Luyben, Nelder-Mead
experiments/          exp01 ... exp10 + run_all.py (reproduce every figure/table)
results/data/         JSON results of every experiment
results/figures/      all plots used in the report
scripts/demo.py       live PyBullet GUI demo / headless GIF export
scripts/make_diagrams.py   block diagram, flowchart, software architecture
report/               report generator (ReportLab) and the generated PDF
tests/                unit tests (pytest)
```

## Installation

PyBullet ships binary wheels for **Linux/macOS, Python 3.8-3.11**. On Windows,
PyPI has no wheels for current Python versions, so use **one** of these:

* **WSL2 Ubuntu (what we used, recommended):**
  ```bash
  python3.11 -m venv ~/venvs/agv          # or: uv venv --python 3.11 ~/venvs/agv
  ~/venvs/agv/bin/pip install -r requirements.txt
  ```
  WSLg shows the PyBullet GUI window directly on Windows 11.
* **Conda (native Windows):** `conda create -n agv -c conda-forge python=3.11 pybullet numpy scipy matplotlib pillow reportlab pypdf pytest`
* **Native pip on Windows:** install *Microsoft C++ Build Tools* first, then
  `pip install -r requirements.txt` (pybullet compiles from source, ~10 min).

## How to run

```bash
# unit tests
python -m pytest -q

# live demo in the PyBullet GUI (proposed controller, unseen complex track)
python scripts/demo.py --track complex --controller LA-GS+ --speed 0.5
python scripts/demo.py --track oval --controller PID-ZN          # classic tuning, for contrast
python scripts/demo.py --sensor camera --controller LA-GS        # camera-based perception
python scripts/demo.py --noise 0.2 --controller LA-GS+           # noisy floor

# headless run that writes an animated GIF (for slides)
python scripts/demo.py --headless --gif results/figures/demo_complex.gif

# reproduce every experiment, figure and JSON result (~6 min on 16 cores)
python experiments/run_all.py
python experiments/run_all.py 4 5       # only E4 and E5

# regenerate the diagrams and the report PDF
python scripts/make_diagrams.py
python report/build_report.py
```

## Experiments

| ID | Script | Question |
|----|--------|----------|
| E1 | `exp01_scene_identification.py` | Scene renders; identify the yaw-rate actuator lag (tau, dead time) |
| E2 | `exp02_autotune.py` | Relay auto-tuning (Ku, Tu) -> ZN/TL gains; Nelder–Mead PID and fuzzy tuning |
| E3 | `exp03_kp_sweep.py` | Does the optimal P gain follow kp\* = 2v/L² at three speeds? |
| E4 | `exp04_step_response.py` | Rise/settling/overshoot of every controller; linear model vs physics |
| E5 | `exp05_frequency.py` | Bode plots, gain/phase margins, damping vs speed (fixed vs scheduled gain) |
| E6 | `exp06_tracks.py` | Benchmark on the training (oval) and unseen (complex) tracks, 5 seeds |
| E7 | `exp07_speed.py` | Speed sweep 0.3–1.0 m/s, in-envelope (≤ 50 mm) rate |
| E8 | `exp08_noise.py` | Sensor-noise robustness and ablation of the LA-GS+ components |
| E9 | `exp09_robustness.py` | Payload, floor friction and motor lag, without re-tuning |
| E10 | `exp10_camera.py` | Rendered camera + inverse perspective mapping vs IR array |

## Controllers compared

| Name | Description |
|------|-------------|
| Bang-bang | three-level on/off steering (hobby-robot baseline) |
| PID-ZN | relay auto-tuned, Ziegler–Nichols rule |
| PID-TL | relay auto-tuned, Tyreus–Luyben rule |
| PID-NM | PID tuned by Nelder–Mead on the training scenarios |
| Fuzzy-NM | 5×5 Sugeno fuzzy PD, scaling factors tuned by Nelder–Mead |
| **LA-GS** | proposed: P control with kp = 2v/L², re-scheduled every step from the speed command |
| **LA-GS+** | LA-GS + sensor-range guard + Kalman line filter + adaptive speed |

## Key results (from `results/data/*.json`)

Unseen complex track, 0.40 m/s, 5 Monte-Carlo seeds, ground-truth axle error:

| Controller | RMS axle error | Max axle error | Run time | Steering activity |
|---|---|---|---|---|
| Bang-bang | 17.0 mm | 54.0 mm | 25.1 s | 22.7 rad/s² |
| PID-ZN (relay auto-tuned) | 18.9 mm | 58.7 mm | 25.1 s | 3.1 rad/s² |
| PID-TL (relay auto-tuned) | 15.8 mm | 45.6 mm | 25.3 s | 3.2 rad/s² |
| PID-NM (optimised) | 6.2 mm | 32.0 mm | 25.8 s | 1.7 rad/s² |
| Fuzzy-NM (optimised) | 5.0 mm | 32.3 mm | 25.8 s | 1.6 rad/s² |
| **LA-GS (analytical)** | **4.3 mm** | **27.3 mm** | 26.0 s | 1.8 rad/s² |
| LA-GS+ | 6.7 mm | 38.6 mm | 30.1 s | 1.5 rad/s² |

* LA-GS reduces the RMS axle error by **77 %** compared with Ziegler-Nichols tuning, for 3.6 % more run time.
* Gain sweeps at 0.25 / 0.40 / 0.55 m/s put the measured optimum within 13 % of kp\* = 2v/L².
* LA-GS stays inside a ±50 mm envelope at every speed from 0.3 to 1.0 m/s; PID-ZN leaves it at ≤ 0.5 m/s.
* LA-GS+ completed **100 %** of runs at IR noise up to 30 % of full scale (PID-NM: 0 %, plain LA-GS: 0 %).
* Stability margins at 0.4 m/s: PID-ZN PM 27°, PID-TL 49°, LA-GS PM 43° / GM 25.6 dB.

Full discussion: `report/Line_Following_AGV_Report.pdf` (Section 9).

## License

MIT (see `LICENSE`).
