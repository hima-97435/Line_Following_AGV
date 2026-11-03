# Facts and figures for the slides

All numbers come from `results/data/*.json` (regenerate with `python experiments/run_all.py`).
Figures are in `results/figures/`.

## Problem and objectives (slides 2–3)
- Tape-guided AGVs: cheap, re-routable, map-free; the steering PID is usually hand-tuned.
- Objective met: unseen-track RMS axle error −77 % vs Ziegler–Nichols (target −50 %) for +3.6 % run time (limit +20 %).
- Objective met: ≥ 90 % completion up to 20 % noise → LA-GS+ completed 100 % up to 30 %.

## Novelty (slide 5)
1. **kp\* = 2v/L²**: the only P gain with zero axle offset on any arc; ζ = 1/√2 without lag.
   Measured optimum within 13 % at 0.25/0.40/0.55 m/s (`kp_sweep.png`).
2. **Axle vs sensor error**: ZN has the smallest sensor error (4.6 mm) but the largest axle error (18.9 mm).
3. **Speed scheduling**: LA-GS in the ±50 mm envelope at every speed from 0.3 to 1.0 m/s (`speed_sweep.png`).
4. **LA-GS+**: R_min = L²/(2·w_s) = 0.23 m; window estimator + guard + KF + adaptive speed → 100 % completion at 30 % noise (`noise.png`).
5. One reproducible benchmark: 10 experiments, 346 s, every figure regenerated from code.

## Robot and environment (slide 8)
- Differential drive, r = 0.04 m, b = 0.24 m, 4 kg (+ payload), motor τ = 60 ms, 1.0 m/s top speed.
- 8-sensor IR bar at L = 0.15 m (14 mm pitch, 10-bit ADC, 2 % noise) or an 80×60 camera at L = 0.28 m.
- Tracks: oval 9.71 m (training), complex 10.95 m (unseen; R down to 0.12 m). Figures: `scene_robot_closeup.png`, `track_maps.png`.

## Algorithms (slides 9–10)
- Block diagram: `system_block_diagram.png`; flowchart: `control_flowchart.png`.
- Relay test (d = 1 rad/s): a = 9.3 mm, Tu = 0.328 s, Ku = 138 → ZN kp = 83, ki = 506, kd = 3.4; TL kp = 63, ki = 87, kd = 3.3.
- Nelder–Mead from (10, 1, 0.3) → kp = 47.3, ki = 0.49, kd = 0.078 (almost pure P).
- Plant: G(s) = −(Ls + v)/(s²(τs + 1)); look-ahead zero at −v/L.

## Results (slides 13–14)
| Controller | Complex RMS | Max | Time | PM |
|---|---|---|---|---|
| Bang-bang | 17.0 mm | 54 mm | 25.1 s | – |
| PID-ZN | 18.9 mm | 59 mm | 25.1 s | 27° |
| PID-TL | 15.8 mm | 46 mm | 25.3 s | 49° |
| PID-NM | 6.2 mm | 32 mm | 25.8 s | 43° |
| Fuzzy-NM | 5.0 mm | 32 mm | 25.8 s | – |
| **LA-GS** | **4.3 mm** | **27 mm** | 26.0 s | 43° (GM 25.6 dB) |
| LA-GS+ | 6.7 mm | 39 mm | 30.1 s | – |

- Step (5 cm): PID-NM settles in 0.65 s (2.1 % OS); LA-GS 1.20 s (7.8 % OS, due to motor lag), and the linear model predicts it (`step_response.png`).
- Payload 0–40 kg: < 0.3 mm change. Friction 0.15: only LA-GS+ stays in the envelope (`robustness.png`).
- Camera: LA-GS 16.6 mm vs PID-NM 45.3 mm (gain auto-adapts to L); IR 4.3 mm; camera 3.95 ms/step vs IR 0.41 ms.

## Honest limitations (slide 16)
- Simulation only; flat floor; no ambient-light model.
- Constant damping holds only while √2·v/L ≪ 1/τ.
- LA-GS is not the best step controller; LA-GS+ is about 15 % slower in nominal conditions.

## Live demo script (slide 12, ~6 min)
```bash
python scripts/demo.py --track complex --controller PID-ZN          # corner cutting
python scripts/demo.py --track complex --controller LA-GS           # on the tape
python scripts/demo.py --track complex --controller LA-GS+ --noise 0.2
python scripts/demo.py --sensor camera --controller LA-GS
```
Backup if the GUI fails: `results/figures/demo_complex.gif`.
