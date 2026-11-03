# Team plan: module ownership, review and commits

The initial system (code, experiments, report draft) is already in the repository,
committed from the team lead's account. The course requires that **every member
commits their own work from their own GitHub account** and can explain the
whole project in the viva. This plan gives each member a module to **own**:
review it line by line, re-run its experiments, and make a real, technical
improvement that is committed from their own account.

## 1. Module ownership (matches report Section 10)

| Member | Module | Files owned | Experiments | Report sections |
|---|---|---|---|---|
| M1 | Simulation & robot model | `agv/world.py`, `agv/track.py`, `scripts/demo.py`, `scripts/make_diagrams.py` | E1 | Abstract, 1, 2, 6, 12 |
| M2 | Sensing & perception | `agv/sensors.py`, Kalman filter in `agv/controllers.py` | E8, E10 | 3, 4, 13 |
| M3 | Control & tuning | `agv/controllers.py`, `agv/tuning.py`, LA-GS / guard in `agv/sim.py` | E2, E3 | 5, 7 |
| M4 | Analysis & validation | `agv/analysis.py`, metrics in `agv/sim.py`, `experiments/`, `tests/`, `report/` | E4–E7, E9 | 8, 9, Appendix |

## 2. Step 1: understand (everyone, days 1–2)

1. Clone the repo, set up the environment (README), run `python -m pytest -q`.
2. Run `python scripts/demo.py --track complex --controller LA-GS+` and
   `--controller PID-ZN` and watch the difference at the R = 0.12 m corner.
3. Read the report end-to-end. Each member then explains their module to the
   others in a recorded meeting (put it in the meeting log, Appendix A.2).
4. Every member must be able to answer: *Why is kp\* = 2v/L²? Why does
   integral action make the AGV cut corners? What is the relay test? What does
   the Kalman filter estimate? What does the sensor-range guard do?*

## 3. Step 2: own commits (each member, from their own account)

Each task below is a genuine extension. It changes code and adds a result,
figure or table that the member then writes about in their report section.
Keep commits small and descriptive (one logical change per commit).

### M1: Simulation & robot model
- [ ] Add a third track `warehouse` to `TRACK_SPECS` (long straights, 90° turns with
      R ≥ 0.3 m, an S-bend) and check it with the clearance test in `track.py`.
- [ ] Add clothoid (curvature-ramp) transitions as an option in the track builder and
      show their effect on the curve-entry error peaks (E6 profile figure).
- [ ] Show the payload as a visual box on the deck in the GUI; record the demo GIFs
      for the slides (`--gif`).
- Suggested commits: `Add warehouse track`, `Add clothoid transitions to track builder`,
  `Visualise payload in GUI demo`, `Record demo GIFs for slides`.

### M2: Sensing & perception
- [ ] Use the camera's curvature preview as feed-forward (w_ff = v·κ) in `sim.py`
      and measure the change in camera RMS error (E10).
- [ ] Add a lighting/reflectance variation test (tape reflectance 0.08 → 0.3, floor
      0.85 → 0.6) to the IR model and report its effect.
- [ ] Tune the Kalman `q, r` from the innovation sequence; add the plot.
- Suggested commits: `Add camera curvature feed-forward`, `Add reflectance variation
  experiment`, `Tune Kalman filter from innovation statistics`.

### M3: Control & tuning
- [ ] Add a lag-compensating derivative term to LA-GS (kd chosen from the model in
      `analysis.py`) and show the step overshoot falling from 7.8 % (E4/E5).
- [ ] Implement a pure-pursuit or LQR steering controller as an extra comparison.
- [ ] Run the relay test at several speeds and tabulate how Ku, Tu change with v.
- Suggested commits: `Add lag compensation to LA-GS`, `Add LQR steering controller`,
  `Relay test versus speed`.

### M4: Analysis & validation
- [ ] Increase the Monte-Carlo seeds (5 → 20) for E6–E8 and add 95 % confidence
      intervals to the tables.
- [ ] Add a sensitivity study of the envelope threshold (30/50/70 mm).
- [ ] Re-run `experiments/run_all.py`, regenerate the report, check the page count
      (≤ 35) and the figure and table references.
- Suggested commits: `Add confidence intervals to benchmark`, `Envelope sensitivity
  study`, `Regenerate results and report`.

## 4. Workflow

```bash
git checkout -b m3-lag-compensation         # one branch per task
# ... edit, run tests and the relevant experiment ...
python -m pytest -q
python experiments/run_all.py 4 5
git add agv/ experiments/ results/
git commit -m "Add lag compensation to LA-GS"
git push -u origin m3-lag-compensation       # open a PR, a team-mate reviews and merges
```

* Every PR is reviewed by one other member (this counts as co-learning, Section 11).
* After all PRs are merged, M4 runs `python experiments/run_all.py` and
  `python report/build_report.py` once more so every number in the report matches.

## 5. Things only the team can do

- Fill the title page (official template, logo, names, SAP IDs, group/project numbers, faculty).
- Fill the writing-allocation page counts, Section 10 names/signatures, Section 11 reflections.
- Keep the meeting log (Appendix A.2) and fill the peer-assessment sheets.
- Make the slides (max 17), record the 20-minute meeting video, upload it as **unlisted**.
- Add every member as a collaborator on the private repository and share access with the evaluator.
