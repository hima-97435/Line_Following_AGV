"""Report text: Section 9 (results and analysis), generated from results/data/*.json."""
import math

from layout import *  # noqa: F401,F403

CTRLS = ["Bang-bang", "PID-ZN", "PID-TL", "PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+"]


def mm(x, nd=1):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{1000 * x:.{nd}f}"


def num(x, nd=2):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def pct(x):
    return f"{100 * x:.0f} %"


def sec9(D, T):
    g, ident, kp = D["gains"], D["ident"], D["kp"]
    st, fq, tr = D["steps"]["metrics"], D["freq"], D["tracks"]["summary"]
    sp, nz, rb, cam = D["speed"]["summary"], D["noise"]["summary"], D["robust"]["summary"], D["camera"]["summary"]
    fig, f, t = D["fig"], T.f, T.t
    s = [H1("9. Results and Analysis"),
         P("All results below were produced by <font name='Mono' size='10'>experiments/run_all.py</font> "
           "(346 s on a 16-thread machine) and are stored as JSON in the repository. Errors are ground-truth "
           "values measured by the simulator, not sensor readings. Unless stated otherwise the speed is "
           "0.40 m/s, the IR noise 2 % of full scale, and values are mean ± standard deviation over the "
           "Monte-Carlo seeds.")]

    # ---------------------------------------------------------------- 9.1
    r = {x["relay"]: x for x in g["relay_runs"]}
    rows = [["Experiment", "Setting", "Result"]]
    for run in ident["runs"]:
        rows.append(["Yaw-rate step (E1)", f"payload {run['payload']:.0f} kg",
                     f"tau = {1000*run['tau']:.0f} ms, Td = {1000*run['delay']:.0f} ms, gain = {run['gain']:.2f}"])
    for d in (0.5, 1.0, 2.0):
        x = r[d]
        rows.append(["Relay test (E2)", f"d = {d} rad/s, h = 1 mm",
                     f"a = {1000*x['amplitude']:.1f} mm, Tu = {x['Tu']:.3f} s, Ku = {x['Ku']:.0f} "
                     f"({x['n_cycles']} cycles)"])
    s += [H2("9.1 System identification and relay test"),
          P(f"Figure {f('identification_step')} shows the open-loop yaw-rate response to a 1.5 rad/s step. A "
            f"first-order fit gives τ = {1000*ident['tau']:.0f} ms with no measurable dead time. This matches "
            f"the programmed motor lag, so the PyBullet contact dynamics add no significant extra lag, and the "
            f"payload hardly changes it (Table {t('ident')}) because the velocity-controlled motors have ample "
            f"torque. The small ripple comes from the four-point wheel/caster contact."),
          figure(fig("identification_step"), f"Figure {f('identification_step')}: Open-loop yaw-rate step "
                 "response and first-order fit", 0.5),
          P(f"The relay experiment (Figure {f('relay_limit_cycle')}) settles into a clean limit cycle. The "
            f"estimated ultimate gain depends on the relay amplitude (Ku = {r[0.5]['Ku']:.0f} to "
            f"{r[2.0]['Ku']:.0f}), which reveals nonlinearity in the loop: sensor quantisation, the IR "
            f"centroid's limited range and yaw-rate saturation. We use d = 1.0 rad/s, a compromise between "
            f"signal-to-noise ratio and small excitation."),
          figure(fig("relay_limit_cycle"), f"Figure {f('relay_limit_cycle')}: Relay auto-tuning limit cycle "
                 "on the straight tape", 0.56),
          *table(f"Table {t('ident')}: Identified actuator dynamics and relay-test results", rows,
                 [3.4 * cm, 3.8 * cm, 8.3 * cm])]

    # ---------------------------------------------------------------- 9.2
    rows = [["Controller", "Tuning method", "kp", "ki", "kd", "Training cost J"]]
    desc = {"Bang-bang": "fixed +-0.6 w_max", "PID-ZN": "relay + Ziegler-Nichols",
            "PID-TL": "relay + Tyreus-Luyben", "PID-NM": "Nelder-Mead (200 evals)",
            "Fuzzy-NM": "Nelder-Mead (100 evals)", "LA-GS": "analytical 2v/L^2", "LA-GS+": "analytical + guard"}
    for n in CTRLS:
        if n in ("PID-ZN", "PID-TL", "PID-NM"):
            kk = g[n]
            vals = [num(kk["kp"], 1), num(kk["ki"], 2), num(kk["kd"], 3)]
        elif n == "Fuzzy-NM":
            vals = [f"Se = {g[n]['fuzzy_e_scale']*1000:.0f} mm", f"Sde = {g[n]['fuzzy_de_scale']:.0f} m/s", "-"]
        elif n.startswith("LA-GS"):
            vals = [f"{g['theory_kp_star']:.1f} (at 0.4 m/s)", "0", "0"]
        else:
            vals = ["-", "-", "-"]
        rows.append([n, desc[n]] + vals + [num(g["cost"][n], 4)])
    nm = g["PID-NM"]
    s += [H2("9.2 Tuned gains"),
          P(f"Table {t('gains')} lists the gains of all controllers and their cost <i>J</i> on the training "
            f"scenarios. The relay rules produce very high gains (PID-ZN: kp = {g['PID-ZN']['kp']:.0f}, "
            f"ki = {g['PID-ZN']['ki']:.0f}) because the identified loop tolerates high gain before oscillating. "
            f"Nelder-Mead started from (10, 1, 0.3) and converged to kp = {nm['kp']:.1f}, ki = {nm['ki']:.2f}, "
            f"kd = {nm['kd']:.3f} after 200 simulation evaluations. The optimiser thus discovered on its own "
            f"that integral and derivative action are almost useless for this objective and settled on a nearly "
            f"proportional controller of the same order as kp* = {g['theory_kp_star']:.1f}. Its gain is higher "
            f"because the cost also rewards a fast 5 cm step, where the motor lag makes extra gain useful. The "
            f"fuzzy optimiser likewise drove the rate scaling to a very large value, which effectively switches "
            f"off the derivative input."),
          *table(f"Table {t('gains')}: Controller gains and training cost (oval lap + 5 cm step)", rows,
                 [2.2 * cm, 3.8 * cm, 2.8 * cm, 2.4 * cm, 1.6 * cm, 2.7 * cm]),
]

    # ---------------------------------------------------------------- 9.3
    rows = [["Speed (m/s)", "kp* = 2v/L^2 (theory)", "Best kp in sweep", "Deviation", "Best RMS axle error"]]
    for v, x in kp["summary"].items():
        dev = 100 * (x["kp_best_measured"] - x["kp_star_theory"]) / x["kp_star_theory"]
        rows.append([v, f"{x['kp_star_theory']:.1f}", f"{x['kp_best_measured']:g}", f"{dev:+.0f} %",
                     f"{mm(x['rms_cte_best'], 2)} mm"])
    ks = kp["steps"]
    s += [H2("9.3 Validation of the look-ahead gain rule"),
          P(f"Figure {f('kp_sweep')} (left) shows the RMS axle error over one oval lap for 21 proportional "
            f"gains at three speeds. Each curve has a sharp minimum, and the minimum moves with speed as "
            f"Equation (5) predicts. The measured optimum lies within 13 % of the prediction at all three speeds "
            f"(Table {t('kpsweep')}), always slightly above it, because a little extra gain shortens the "
            f"transients at the straight-to-arc junctions, which the steady-state rule ignores. Gains below "
            f"kp* let the axle drift outwards on the bends ({mm(kp['table']['0.4'][0]['rms_cte'])} mm RMS at "
            f"kp = 5); gains above it make the axle cut inwards. The right panel shows that kp* = 35.6 also gives a well-damped step "
            f"({ks['35.6']['overshoot']:.1f} % overshoot, settling in {ks['35.6']['settling_time']:.2f} s), "
            f"while kp = 10 overshoots by {ks['10']['overshoot']:.0f} %."),
          *table(f"Table {t('kpsweep')}: Predicted and measured optimal proportional gain", rows,
                 [2.4 * cm, 3.6 * cm, 3.0 * cm, 2.4 * cm, 4.1 * cm]),
          figure(fig("kp_sweep"), f"Figure {f('kp_sweep')}: Proportional-gain sweep on the oval (left; dashed "
                 "lines mark kp* = 2v/L^2) and step responses for selected gains (right)", 0.82)]

    # ---------------------------------------------------------------- 9.4
    rows = [["Controller", "Rise time (s)", "Overshoot (%)", "Settling 5 % (s)", "Steady error (mm)",
             "IAE (mm s)", "Peak |w| (rad/s)"]]
    for n in CTRLS:
        m = st[n]
        rows.append([n, num(m["rise_time"]), num(m["overshoot"], 1), num(m["settling_time"]),
                     mm(m["ss_error"], 2), mm(m["iae"], 1), num(m["peak_w"])])
    lin = D["steps"]["linear"]
    s += [H2("9.4 Time-domain step response"),
          P(f"The AGV starts 5 cm to the left of a straight tape. Figure {f('step_response')} and Table "
            f"{t('steps')} compare the responses. All PID-type controllers settle within about 1.3 s with little "
            f"overshoot. PID-ZN drops fastest at first. Bang-bang never settles: it enters a limit cycle with a "
            f"{mm(st['Bang-bang']['ss_error'], 1)} mm residual error, which is exactly the relay behaviour used "
            f"for auto-tuning. PID-ZN's integral action produces a slow tail"
            f"(settling {st['PID-ZN']['settling_time']:.2f} s) and the largest steering peak "
            f"({st['PID-ZN']['peak_w']:.1f} rad/s). PID-NM is the best step controller. LA-GS overshoots "
            f"{st['LA-GS']['overshoot']:.1f} %, more than the 4.3 % of the lag-free theory, because the 60 ms "
            f"motor lag erodes phase (Section 9.5). The step is therefore <i>not</i> where LA-GS is strongest; "
            f"its advantage is on curved paths (Section 9.6)."),
          P(f"The right panel validates the linear model of Section 5.2 against the physics simulation. With "
            f"the identified τ the model reproduces the PyBullet responses closely: for LA-GS the model "
            f"predicts {lin['LA-GS']['metrics']['overshoot']:.1f} % overshoot against "
            f"{st['LA-GS']['overshoot']:.1f} % measured, and for PID-TL {lin['PID-TL']['metrics']['overshoot']:.1f} "
            f"% against {st['PID-TL']['overshoot']:.1f} %. This justifies using the model for the "
            f"frequency-domain analysis."),
          *table(f"Table {t('steps')}: Step-response metrics (5 cm initial offset, v = 0.40 m/s)", rows,
                 [2.2 * cm, 1.9 * cm, 2.0 * cm, 2.2 * cm, 2.3 * cm, 2.0 * cm, 2.4 * cm]),
          figure(fig("step_response"), f"Figure {f('step_response')}: Step responses of all controllers "
                 "(left) and linear model versus PyBullet (right)", 0.82)]

    # ---------------------------------------------------------------- 9.5
    mg = fq["margins"]
    rows = [["Controller", "Gain crossover (rad/s)", "Phase margin (deg)", "Gain margin (dB)",
             "Gain-reduction margin (dB)", "Closed-loop bandwidth (rad/s)"]]
    for n in ("PID-ZN", "PID-TL", "PID-NM", "LA-GS"):
        m = mg[n]
        rows.append([n, num(m["gain_crossover"]), num(m["phase_margin"], 1), num(m["gain_margin_db"], 1),
                     num(m.get("gain_reduction_margin_db"), 1), num(m["bandwidth"], 1)])
    mz = fq["model"]; ms = fq["measured"]; speeds = fq["speeds"]
    zs_sched = [z for z, _ in mz["scheduled"]]
    z_fixed = [z for z, _ in mz["fixed"]]
    ts_f = [ms["fixed"][str(v)]["settling_time"] for v in speeds]
    ts_s = [ms["scheduled"][str(v)]["settling_time"] for v in speeds]
    s += [H2("9.5 Frequency-domain analysis and speed dependence"),
          P(f"Figure {f('bode')} shows the open-loop Bode plots at 0.4 m/s and Table {t('margins')} the "
            f"margins. PID-ZN has the highest crossover frequency but only {mg['PID-ZN']['phase_margin']:.0f} "
            f"degrees phase margin, in line with the known aggressiveness of the Ziegler-Nichols rule [13]. "
            f"PID-TL is better damped ({mg['PID-TL']['phase_margin']:.0f} degrees). Both have an integrator on "
            f"top of the plant's double integrator, so their phase starts at -270 degrees and the loops are "
            f"<i>conditionally stable</i>: reducing the gain by more than the gain-reduction margin would also "
            f"destabilise them. LA-GS has no integrator, a phase margin of {mg['LA-GS']['phase_margin']:.0f} "
            f"degrees and the largest gain margin ({mg['LA-GS']['gain_margin_db']:.1f} dB), so it is the most "
            f"tolerant of gain errors (for example a mis-measured look-ahead or speed)."),
          *table(f"Table {t('margins')}: Stability margins of the linear controllers (v = 0.40 m/s)", rows,
                 [2.2 * cm, 2.6 * cm, 2.3 * cm, 2.3 * cm, 3.0 * cm, 3.1 * cm]),
          figure(fig("bode"), f"Figure {f('bode')}: Open-loop Bode plots of the linear controllers", 0.55),
          P(f"Figure {f('speed_damping')} examines the speed dependence. The left panel compares the damping "
            f"of the dominant closed-loop poles predicted by the model (lines, including the motor lag) with the "
            f"damping inferred from the PyBullet overshoot (dots). A fixed gain tuned at 0.4 m/s is over-damped "
            f"at low speed (ζ = {z_fixed[0]:.2f} at 0.2 m/s) and strongly under-damped at high speed "
            f"(ζ = {z_fixed[-1]:.2f} at 0.8 m/s). With scheduling the damping stays close to 0.7 up to "
            f"0.4 m/s and then decreases ({zs_sched[-1]:.2f} in the model at 0.8 m/s), but less than with the "
            f"fixed gain in the PyBullet measurements. The reason is the motor lag: the constant ζ = 1/√2 "
            f"of Equation (6) assumes τ = 0, and the natural frequency √2<i>v</i>/<i>L</i> approaches the "
            f"motor bandwidth 1/τ as speed rises. The theory therefore holds when √2<i>v</i>/<i>L</i> "
            f"&lt;&lt; 1/τ. The practical result is in the right panel: the scheduled controller's settling time "
            f"stays between {min(ts_s[1:]):.1f} and {max(ts_s[1:]):.1f} s from 0.3 to 0.8 m/s, whereas the fixed "
            f"gain varies between {min(ts_f):.1f} and {max(ts_f):.1f} s."),
          figure(fig("speed_damping"), f"Figure {f('speed_damping')}: Closed-loop damping (left) and settling "
                 "time (right) versus speed for a fixed gain and for the scheduled gain kp = 2v/L^2", 0.74)]

    # ---------------------------------------------------------------- 9.6
    rows = [["Controller", "Oval RMS (mm)", "Oval max (mm)", "Complex RMS (mm)", "Complex max (mm)",
             "Complex sensor RMS (mm)", "Complex time (s)", "Steering act. (rad/s^2)", "Energy (J)"]]
    for n in CTRLS:
        o, c = tr["oval"][n], tr["complex"][n]
        rows.append([n, f"{mm(o['rms_cte'][0])}", mm(o["max_cte"][0]),
                     f"{mm(c['rms_cte'][0])} ± {mm(c['rms_cte'][1])}", mm(c["max_cte"][0]),
                     mm(c["rms_e"][0]), num(c["time"][0], 1), num(c["smoothness"][0], 1), num(c["energy"][0], 0)])
    la, zn = tr["complex"]["LA-GS"], tr["complex"]["PID-ZN"]
    red = 100 * (1 - la["rms_cte"][0] / zn["rms_cte"][0])
    dt_ = 100 * (la["time"][0] / zn["time"][0] - 1)
    s += [H2("9.6 Track benchmark: training and unseen tracks"),
          P(f"Table {t('tracks')} is the main comparison. Every controller completed every run on both tracks "
            f"at 0.4 m/s, so the differences lie in accuracy and smoothness. On the unseen complex track LA-GS "
            f"has the lowest RMS axle error, {mm(la['rms_cte'][0])} mm, against {mm(zn['rms_cte'][0])} mm for "
            f"PID-ZN: a {red:.0f} % reduction for only {dt_:.1f} % longer run time, which meets objective 4. "
            f"The table also shows the conflict between sensor error and axle error. PID-ZN, PID-TL and bang-"
            f"bang keep the <i>sensor</i> error smallest (4-9 mm) but have the <i>largest</i> axle error, "
            f"because forcing the sensor onto the tape pulls the axle inside every bend by L^2/2R. LA-GS "
            f"deliberately lets the sensor error grow to L^2/2R so that the axle stays on the tape. Figure "
            f"{f('tracks_xy')} shows the effect at the R = 0.12 m corner, where PID-ZN cuts the corner most, and "
            f"Figure {f('tracks_error_profile')} shows that the errors are concentrated at curvature changes."),
          P(f"Fuzzy-NM, whose optimised scaling turned it into a nonlinear proportional law, performs close to "
            f"LA-GS and was the best controller on the training oval ({mm(tr['oval']['Fuzzy-NM']['rms_cte'][0])} "
            f"mm). On the unseen track it is {100*(tr['complex']['Fuzzy-NM']['rms_cte'][0]/la['rms_cte'][0]-1):.0f} "
            f"% worse than LA-GS, a sign of slight over-fitting to the training track. LA-GS+ trades some accuracy "
            f"({mm(tr['complex']['LA-GS+']['rms_cte'][0])} mm) and {num(tr['complex']['LA-GS+']['time'][0], 1)} s "
            f"run time (adaptive speed) for the robustness shown in Sections 9.7-9.9 and the smoothest steering. "
            f"Bang-bang has by far the highest steering activity and energy use, which would mean drive-train "
            f"wear on a real vehicle."),
          *table(f"Table {t('tracks')}: Track benchmark at v = 0.40 m/s (5 seeds; all runs completed)", rows,
                 [1.9 * cm, 1.45 * cm, 1.45 * cm, 2.1 * cm, 1.6 * cm, 1.8 * cm, 1.6 * cm, 1.9 * cm, 1.4 * cm]),
          figure(fig("tracks_xy"), f"Figure {f('tracks_xy')}: Trajectories on the unseen complex track (left) "
                 "and zoom on the R = 0.12 m corner (right)", 0.66),
          figure(fig("tracks_error_profile"), f"Figure {f('tracks_error_profile')}: Axle cross-track error "
                 "along the complex track with the track curvature in the background", 0.72)]

    # ---------------------------------------------------------------- 9.7
    vlist = ["0.3", "0.5", "0.7", "1.0"]
    rows = [["Controller"] + [f"{v} m/s" for v in vlist]]
    for n in ["Bang-bang", "PID-ZN", "PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+"]:
        rows.append([n] + [f"{pct(sp[n][v]['envelope'])} / {mm(sp[n][v]['max_cte'][0], 0)}" for v in vlist])
    s += [H2("9.7 Speed sweep"),
          P(f"Every controller completed the complex track at every speed from 0.3 to 1.0 m/s, because the "
            f"reflexive supervisor brings the AGV back after an excursion. Completion is therefore not a useful "
            f"measure, and we use the <i>in-envelope rate</i>: the fraction of runs completed with a maximum axle "
            f"deviation of at most 50 mm (Figure {f('speed_sweep')}, Table {t('speed')}). LA-GS stays in the "
            f"envelope at every speed with 15-28 mm peak error, and its RMS error stays between 3.3 and 4.5 mm. "
            f"PID-ZN leaves the envelope at all speeds up to 0.5 m/s (peak errors 52-64 mm) and bang-bang up to "
            f"0.4 m/s. Interestingly, both improve at higher speed: their fixed high gain becomes relatively "
            f"smaller compared with kp* = 2v/L^2, which grows with speed. This is the same mechanism that makes "
            f"the fixed-gain PID-NM worsen away from its design speed (RMS {mm(sp['PID-NM']['0.3']['rms_cte'][0])} "
            f"mm at 0.3 m/s and {mm(sp['PID-NM']['1.0']['rms_cte'][0])} mm at 1.0 m/s against "
            f"{mm(sp['PID-NM']['0.5']['rms_cte'][0])} mm at 0.5 m/s). LA-GS+ is slower because of its adaptive "
            f"speed, and its peak error decreases with speed for the same reason."),
          *table(f"Table {t('speed')}: In-envelope rate / mean maximum axle error (mm) versus top speed "
                 "(complex track, 3 seeds)", rows, [3.0 * cm, 3.1 * cm, 3.1 * cm, 3.1 * cm, 3.2 * cm]),
          figure(fig("speed_sweep"), f"Figure {f('speed_sweep')}: Speed sweep: in-envelope rate, maximum axle "
                 "error and run time", 0.82)]

    # ---------------------------------------------------------------- 9.8
    nlist = ["0.0", "0.1", "0.15", "0.2", "0.3"]
    rows = [["Variant"] + [f"{float(x)*100:.0f} %" for x in nlist]]
    for n, row in nz.items():
        rows.append([n] + [f"{pct(row[x]['success'])} / {mm(row[x]['rms_cte'][0])}" for x in nlist])
    kf = nz["LA-GS + guard + KF"]["0.2"]
    s += [H2("9.8 Sensor noise and ablation of LA-GS+"),
          P(f"Figure {f('noise')} and Table {t('noise')} show the effect of IR noise from 0 to 30 % of full "
            f"scale and the contribution of each LA-GS+ component. High-gain PID-ZN never loses the line (it "
            f"keeps the tape centred under the bar), but its axle error stays at 19 mm and its steering activity "
            f"grows fastest with noise. Plain LA-GS with the all-sensor centroid fails from 10 % noise. The "
            f"cause is perception: noise on the sensors that do not see the tape pulls the centroid towards the "
            f"centre, so on the tight corners, where LA-GS lets the line approach the edge of the bar "
            f"(Section 5.3), the AGV under-steers and loses the line. The peak-window estimator fixes 10 % "
            f"noise. The sensor-range guard keeps the line inside the bar on corners tighter than R_min and "
            f"raises completion to 60-80 % at 15-30 % noise. The Kalman filter smooths the estimate (sensor RMSE "
            f"{mm(kf['sensor_rmse'][0])} mm to estimate RMSE {mm(kf['estimate_rmse'][0])} mm at 20 % noise) and "
            f"halves the steering activity. Adaptive speed then gives the guard more time on the tight corners. "
            f"With all four, LA-GS+ completed <b>100 % of runs at every noise level</b> with an almost constant "
            f"{mm(nz['LA-GS+ (guard + KF + speed)']['0.3']['rms_cte'][0])} mm RMS axle error, which is 2.7 times "
            f"more accurate than PID-ZN under the same noise."),
          *table(f"Table {t('noise')}: Completion rate / RMS axle error (mm) versus IR noise (complex track, "
                 "5 seeds)", rows, [4.6 * cm, 2.2 * cm, 2.2 * cm, 2.2 * cm, 2.2 * cm, 2.1 * cm]),
          figure(fig("noise"), f"Figure {f('noise')}: Noise robustness and ablation of the LA-GS+ components",
                 0.92)]

    # ---------------------------------------------------------------- 9.9
    rows = [["Controller", "Payload 40 kg", "Friction 0.15", "Friction 0.25", "Motor tau 0.12 s",
             "Motor tau 0.20 s"]]
    for n in ["PID-ZN", "PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+"]:
        cell = lambda sw, v: f"{pct(rb[sw][n][v]['envelope'])} / {mm(rb[sw][n][v]['rms_cte'][0])}"
        rows.append([n, cell("payload", "40.0"), cell("friction", "0.15"), cell("friction", "0.25"),
                     cell("motor_tau", "0.12"), cell("motor_tau", "0.2")])
    s += [H2("9.9 Robustness to payload, friction and actuator lag"),
          P(f"At 0.5 m/s and without re-tuning (Figure {f('robustness')}, Table {t('robust')}), payloads up to "
            f"40 kg (ten times the empty mass) changed the RMS error by less than 0.3 mm for every controller, "
            f"because the speed-controlled motors have enough torque. Doubling the motor time constant to "
            f"0.12 s costs at most 1 mm; at 0.20 s the proportional-type controllers degrade to 8-9 mm, as the "
            f"phase-margin analysis predicts. Low friction is the hardest case: at μ = 0.15 the wheels slip in the tight "
            f"corners and only LA-GS+, which slows down when the error or turn rate is large, stays within the "
            f"50 mm envelope ({mm(rb['friction']['LA-GS+']['0.15']['rms_cte'][0])} mm RMS). PID-ZN is outside "
            f"the envelope in almost every condition because of its corner cutting."),
          *table(f"Table {t('robust')}: In-envelope rate / RMS axle error (mm) under plant changes (complex, "
                 "0.5 m/s, 3 seeds)", rows, [2.3 * cm, 2.6 * cm, 2.6 * cm, 2.6 * cm, 2.7 * cm, 2.7 * cm]),
          figure(fig("robustness"), f"Figure {f('robustness')}: Robustness to payload, wheel-floor friction "
                 "and motor time constant", 0.62)]

    # ---------------------------------------------------------------- 9.10
    rows = [["Controller", "IR RMS (mm)", "IR max (mm)", "Camera RMS (mm)", "Camera max (mm)",
             "IR time/step (ms)", "Camera time/step (ms)"]]
    for n in ["PID-NM", "Fuzzy-NM", "LA-GS", "LA-GS+"]:
        a, b = cam["ir"][n], cam["camera"][n]
        rows.append([n, mm(a["rms_cte"][0]), mm(a["max_cte"][0], 0), mm(b["rms_cte"][0]), mm(b["max_cte"][0], 0),
                     num(a["sensor_ms"][0]), num(b["sensor_ms"][0])])
    cl, cn = cam["camera"]["LA-GS"], cam["camera"]["PID-NM"]
    s += [H2("9.10 Camera versus IR array"),
          P(f"Figure {f('camera_pipeline')} shows the camera pipeline: rendered frame, tape mask and the floor "
            f"points from inverse perspective mapping with the fitted line. The camera evaluates the error at "
            f"L = 0.28 m. Table {t('camera')} compares the two sensors on the complex track. All runs completed "
            f"with both sensors. The gain-scheduled controllers adapt automatically to the camera because "
            f"kp = 2v/L^2 uses the sensor's look-ahead (kp* = {2*0.4/0.28**2:.1f} instead of 35.6). PID-NM, "
            f"tuned for the IR bar, applies a gain about 4.6 times too high and reaches {mm(cn['rms_cte'][0])} mm "
            f"RMS error, against {mm(cl['rms_cte'][0])} mm for LA-GS. The IR array remains more accurate on this "
            f"track. The longer look-ahead raises R_min to L^2/(2w_s) = 0.33 m, so more corners lie below the "
            f"limit, and the camera costs about ten times more computation per step "
            f"({num(cl['sensor_ms'][0])} ms against {num(cam['ir']['LA-GS']['sensor_ms'][0])} ms). Its curvature "
            f"preview is not yet used for feed-forward (Section 12)."),
          figure(fig("camera_pipeline"), f"Figure {f('camera_pipeline')}: Camera perception pipeline", 0.72),
          *table(f"Table {t('camera')}: IR array versus camera on the complex track (3 seeds)", rows,
                 [2.2 * cm, 1.9 * cm, 1.9 * cm, 2.2 * cm, 2.2 * cm, 2.4 * cm, 2.7 * cm])]

    # ---------------------------------------------------------------- 9.11
    s += [H2("9.11 Discussion: what worked and what did not"),
          *bullets([
              "<b>Worked:</b> the analytical gain rule needed no tuning experiment, predicted the measured optimum "
              "within 13 % and gave the lowest axle error on the unseen track at every speed; the linear model "
              "with the identified motor lag predicted the PyBullet step responses.",
              "<b>Worked:</b> separating axle error from sensor error explained why relay tuning, which looks "
              "excellent on the sensor signal, gives the worst path accuracy; the ablation showed that noise "
              "robustness came mainly from perception and the sensor-range guard.",
              "<b>Did not work as hoped:</b> LA-GS is not the best step controller (7.8 % overshoot) and its "
              "damping falls above about 0.5 m/s, both because of the 60 ms motor lag; a small lag-compensating "
              "derivative term is the natural next step. LA-GS+ costs about 15 % run time in nominal conditions.",
              "<b>Limitations:</b> simulation only; flat floor, no ambient-light model; corners tighter than "
              "R_min remain a geometric limit of the sensor bar for any controller."])]
    return s
