"""Report text: Sections 10-13 and the appendix."""
from layout import *  # noqa: F401,F403
from reportlab.platypus import PageBreak

REPO = "https://github.com/hima-97435/Line_Following_AGV"

REFERENCES = [
    'G. Ullrich, <i>Automated Guided Vehicle Systems: A Primer with Practical Applications</i>, 2nd ed. Berlin, Germany: Springer, 2015.',
    'P. R. Wurman, R. D\'Andrea, and M. Mountz, "Coordinating hundreds of cooperative, autonomous vehicles in warehouses," <i>AI Magazine</i>, vol. 29, no. 1, pp. 9-20, 2008.',
    'M. Pakdaman, M. M. Sanaatiyan, and M. R. Ghahroudi, "A line follower robot from design to implementation: Technical issues and problems," in <i>Proc. 2nd Int. Conf. Computer and Automation Engineering (ICCAE)</i>, Singapore, 2010, pp. 5-9.',
    '<i>Industrial trucks - Safety requirements and verification - Part 4: Driverless industrial trucks and their systems</i>, ISO 3691-4:2020, International Organization for Standardization, Geneva, 2020.',
    'E. Coumans and Y. Bai, "PyBullet, a Python module for physics simulation for games, robotics and machine learning," 2016-2021. [Online]. Available: http://pybullet.org',
    'S. Russell and P. Norvig, <i>Artificial Intelligence: A Modern Approach</i>, 3rd ed. Upper Saddle River, NJ, USA: Prentice Hall, 2010.',
    'J. G. Ziegler and N. B. Nichols, "Optimum settings for automatic controllers," <i>Trans. ASME</i>, vol. 64, pp. 759-768, 1942.',
    'K. J. Astrom and T. Hagglund, "Automatic tuning of simple regulators with specifications on phase and amplitude margins," <i>Automatica</i>, vol. 20, no. 5, pp. 645-651, 1984.',
    'B. D. Tyreus and W. L. Luyben, "Tuning PI controllers for integrator/dead time processes," <i>Ind. Eng. Chem. Res.</i>, vol. 31, no. 11, pp. 2625-2628, 1992.',
    'R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-92-01, 1992.',
    'T. Takagi and M. Sugeno, "Fuzzy identification of systems and its applications to modeling and control," <i>IEEE Trans. Syst., Man, Cybern.</i>, vol. SMC-15, no. 1, pp. 116-132, 1985.',
    'R. E. Kalman, "A new approach to linear filtering and prediction problems," <i>J. Basic Eng.</i>, vol. 82, no. 1, pp. 35-45, 1960.',
    'K. J. Astrom and T. Hagglund, <i>Advanced PID Control</i>. Research Triangle Park, NC, USA: ISA, 2006.',
    'R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, <i>Introduction to Autonomous Mobile Robots</i>, 2nd ed. Cambridge, MA, USA: MIT Press, 2011.',
    'S. Thrun, W. Burgard, and D. Fox, <i>Probabilistic Robotics</i>. Cambridge, MA, USA: MIT Press, 2005.',
    'K. Ogata, <i>Modern Control Engineering</i>, 5th ed. Upper Saddle River, NJ, USA: Prentice Hall, 2010.',
    'J. A. Nelder and R. Mead, "A simplex method for function minimization," <i>Comput. J.</i>, vol. 7, no. 4, pp. 308-313, 1965.',
    'M. Quigley <i>et al.</i>, "ROS: An open-source Robot Operating System," in <i>Proc. ICRA Workshop on Open Source Software</i>, Kobe, Japan, 2009.',
]


def sec10(D, T):
    rows = [["S. No.", "Member name and SAP ID", "Technical contribution (modules / algorithms / code files / "
             "experiments)", "Report sections written + signature"],
            ["1 (M1)", "Himanshu Pokhriyal\n500121200", "Simulation and robot model: agv/world.py (PyBullet world, generated URDF, motor-lag "
             "model, COM-to-axle pose correction), agv/track.py (track geometry, floor texture), "
             "scripts/demo.py (GUI demo, GIF export); experiment E1 (scene, actuator identification); "
             f"Figures {T.f('system_block_diagram')}-{T.f('track_maps')}, {T.f('identification_step')}.",
             "Abstract, 1, 2, 6, 12"],
            ["2 (M2)", "Anshuman Singh\n500121170", "Sensing and perception: agv/sensors.py (ray-cast IR array, spot-coverage model, ADC, "
             "peak-window estimator, camera rendering, IPM calibration and line fit), Kalman filter in "
             "agv/controllers.py; experiments E8 (noise and ablation) and E10 (camera vs IR); "
             f"Figures {T.f('noise')}, {T.f('camera_pipeline')}.", "3, 4, 13"],
            ["3 (M3)", "Mradul Lakhotiya\n500125405", "Control and tuning: agv/controllers.py (PID with anti-windup, bang-bang, fuzzy PD, "
             "relay, speed scheduler), agv/tuning.py (relay auto-tuning, ZN/TL rules, Nelder-Mead), derivation "
             "and implementation of LA-GS and the sensor-range guard in agv/sim.py; experiments E2 and E3; "
             f"Figures {T.f('relay_limit_cycle')}, {T.f('kp_sweep')}.", "5, 7"],
            ["4 (M4)", "Shashwat Kashyap\n500124830", "Analysis and validation: agv/analysis.py (step metrics, identification fit, loop "
             "FRF, margins, linear model), metrics in agv/sim.py, experiments E4, E5, E6, E7, E9, "
             "experiments/run_all.py, tests/, report generator; "
             f"Figures {T.f('step_response')}-{T.f('robustness')}.", "8, 9, Appendix"]]
    s = [H1("10. Contribution"),
         P("Table %d lists the technical module owned by each member and the report sections he or she wrote. "
           "Every claim is traceable to commits in the GitHub repository and to the member's part of the video. "
           "The module split follows the package structure, so each member's commits touch his or her own "
           "files." % T.t("contrib")),
         *table(f"Table {T.t('contrib')}: Technical contribution of each member", rows,
                [1.4 * cm, 3.0 * cm, 8.1 * cm, 3.0 * cm], align_center_from=3),
         P("<b>Writing allocation.</b> Table 1 (after the Index) shows who wrote which section. Sections 10 "
           "(own row and signature) and 11 (own reflection) and the Declaration were completed by every "
           "member.")]
    return s


def sec11(D):
    s = [H1("11. Co-learning Reflection"),
         P("Each member writes 5-8 lines in his or her own words: what I learned on my own, what I learned from "
           "team-mates, and what I taught others.", "small")]
    for i in range(1, 5):
        s += [H2(f"11.{i} Member M{i}: ____________________"),
              P("<i>[To be written by the member: self-learning (e.g. a tool or concept studied independently), "
                "learning from team-mates (which module and from whom), and what he or she taught others.]</i>"),
              Spacer(1, 0.5 * cm)]
    return s


def sec12(D):
    tr = D["tracks"]["summary"]; g = D["gains"]
    la, zn = tr["complex"]["LA-GS"], tr["complex"]["PID-ZN"]
    return [H1("12. Conclusion and Future Scope"),
            H2("12.1 Main findings"),
            P(f"The look-ahead rule <i>k</i><sub>p</sub>* = 2<i>v</i>/<i>L</i><sup>2</sup> gives zero steady-state "
              f"axle offset on any arc; its measured optimum agreed within 13 % and, scheduled with speed, it kept "
              f"the AGV inside a 50 mm envelope from 0.3 to 1.0 m/s without re-tuning. Classical relay tuning (ZN, "
              f"TL) minimises the sensor error but cuts corners: {1000*zn['rms_cte'][0]:.1f} mm against "
              f"{1000*la['rms_cte'][0]:.1f} mm RMS axle error for LA-GS on the unseen track. The geometric limit "
              f"R_min = L^2/(2 w_s) explains the failures under noise, and LA-GS+ (peak-window estimator, "
              f"sensor-range guard, Kalman filter, adaptive speed) removed them up to 30 % noise. The motor lag "
              f"bounds the speed range of the constant-damping result, and the camera's longer look-ahead lowers "
              f"the gain and accuracy on tight corners."),
            H2("12.2 Industrial use"),
            P("The rule gives a commissioning engineer the steering gain of a tape-guided AGV from two numbers "
              "that are known at design time, the sensor look-ahead and the speed, instead of an iterative "
              "tuning session for each vehicle type and speed. The axle-error metric and in-envelope rate "
              "translate directly into the aisle clearance required by safety standards such as ISO 3691-4 [4]."),
            H2("12.3 Moving to a real robot"),
            *bullets([
                "Measure <i>L</i> and the sensor half-span on the real sensor bar, identify the motor lag with "
                "the same open-loop step (E1), and set <i>k</i><sub>p</sub> = 2<i>v</i>/<i>L</i><sup>2</sup>.",
                "Port agv/controllers.py to the microcontroller (the controller needs only the speed command "
                "and the line error) or run it as a ROS node [18] receiving the sensor array topic.",
                "Calibrate the IR array (white/black levels) and re-estimate the noise level for the Kalman "
                "filter from logged data.",
                "Start with the relay test at low amplitude as a safety check of the loop margins."]),
            H2("12.4 Future scope"),
            *bullets([
                "Junctions, stop markers and route selection (mission layer), and clothoid transitions between "
                "straights and arcs.",
                "Feed-forward of the camera's curvature preview (ω<sub>ff</sub> = <i>v</i>κ) to "
                "remove the transient error at curve entries.",
                "LQR or model-predictive steering on the identified model, and learning-based tuning across "
                "vehicle types.",
                "Full ROS/Gazebo integration and hardware-in-the-loop tests."])]


def sec13(D):
    s = [H1("13. References")]
    for i, ref in enumerate(REFERENCES, 1):
        s.append(Paragraph(ref, ParagraphStyleRef, bulletText=f"[{i}]"))
    return s


from reportlab.lib.styles import ParagraphStyle  # noqa: E402
ParagraphStyleRef = ParagraphStyle("ref", parent=ST["body"], fontSize=11, leading=15, leftIndent=26,
                                   bulletIndent=0, alignment=0, spaceAfter=3)


def appendix(D):
    log_rows = [["Date", "Duration", "Attendees", "Decisions / actions"]] + [["", "", "", ""] for _ in range(5)]
    peer = [["No.", "Team-mate name and SAP ID", "Contribution (1-5)", "Reliability / on time (1-5)",
             "Helped others learn (1-5)", "Quality of work (1-5)"]] + [[str(i), "", "", "", "", ""] for i in
                                                                       range(1, 5)]
    s = [H1("Appendix: Links and Logs"),
         H2("A.1 Links"),
         P(f"<b>GitHub repository (private; access shared with the evaluator):</b> {REPO}"),
         P("<b>YouTube video (unlisted):</b> https://youtu.be/R0RO__q8XGQ?si=AgLZs7mObpD58bSv"),
         P("<b>Reproducibility:</b> <font name='Mono' size='10'>python experiments/run_all.py</font> regenerates "
           "results/data/*.json and results/figures/*.png; "
           "<font name='Mono' size='10'>python report/build_report.py</font> regenerates this report."),
         H2("A.2 Meeting log")]
    s += table("Table A1: Team meeting log", log_rows, [2.4 * cm, 2.2 * cm, 4.0 * cm, 6.9 * cm],
               align_center_from=9)
    s += [H2("A.3 Peer-assessment sheet"),
          P("Each member fills one sheet for every team-mate; the individual sheets are attached after this "
            "page.", "small")]
    s += table("Table A2: Peer-assessment sheet (one per member)", peer,
               [1.1 * cm, 4.6 * cm, 2.4 * cm, 2.5 * cm, 2.5 * cm, 2.4 * cm], align_center_from=2)
    return s
