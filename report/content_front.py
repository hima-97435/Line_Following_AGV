"""Report text: title page, front matter and Sections 1-8 (theory/design/method)."""
from layout import *  # noqa: F401,F403
from reportlab.platypus import NextPageTemplate, PageBreak

REPO = "https://github.com/AmulyaJain2004/Line-Following-AGV"


def title_page(D):
    s = [NextPageTemplate("front"), Spacer(1, 1.2 * cm),
         P("[Insert the official university logo / common title-page header here]", "center"),
         Spacer(1, 1.6 * cm),
         P("A Project Report on", "center"), Spacer(1, 0.3 * cm),
         P("LINE-FOLLOWING AGV WITH PID TUNING<br/>AND RESPONSE ANALYSIS", "title"),
         Spacer(1, 1.0 * cm),
         P("Project No.: ________ &nbsp;&nbsp;&nbsp; Group No.: ________ &nbsp;&nbsp;&nbsp; Batch: B__", "center"),
         Spacer(1, 0.9 * cm),
         P("<b>Submitted by</b>", "center")]
    rows = [["Name", "SAP ID"]] + [[f"Member {i}: ______________________", "______________"] for i in range(1, 5)]
    t = Table([[Paragraph(a, ST["center"]), Paragraph(b, ST["center"])] for a, b in rows],
              colWidths=[9 * cm, 4.5 * cm])
    t.setStyle(TableStyle([("FONTNAME", (0, 0), (-1, 0), BOLD)]))
    s += [t, Spacer(1, 1.0 * cm),
          P("in partial fulfilment of the course", "center"),
          P("<b>CSAI4014 Algorithms for Intelligent Systems and Robotics</b>", "center"),
          P("B.Tech Computer Science and Engineering (AI &amp; ML), Semester VII", "center"),
          Spacer(1, 0.9 * cm),
          P("Under the guidance of", "center"), P("<b>[Faculty name]</b>", "center"),
          Spacer(1, 1.2 * cm),
          P("[School / Department, University name]", "center"),
          P("October 2026", "centerb"), PageBreak()]
    return s


def declaration(D):
    s = [H1u("Declaration"),
         P("We, the undersigned members of the group, declare that the project report entitled "
           "<i>Line-Following AGV with PID Tuning and Response Analysis</i>, submitted for the course CSAI4014 "
           "Algorithms for Intelligent Systems and Robotics, is a record of our own work. The simulation code, "
           "experiments, figures and results presented here were produced by us in the project repository "
           f"({REPO}). Wherever the work of others has been used, it has been acknowledged and cited in the "
           "References. This report has not been submitted elsewhere for any other degree or course."),
         Spacer(1, 1.2 * cm)]
    rows = [["Member", "Name", "SAP ID", "Signature"]] + [[f"M{i}", "", "", ""] for i in range(1, 5)]
    s += table("", rows, [2 * cm, 6.5 * cm, 3.5 * cm, 3.5 * cm])[1:2]
    s += [Spacer(1, 1.0 * cm), P("Date: ____________ &nbsp;&nbsp;&nbsp;&nbsp; Place: ____________"), PageBreak()]
    return s


def index_pages(D):
    return [H1u("Index", toc=False), make_toc(), PageBreak(),
            H1u("List of Figures", toc=False), make_toc("FigEntry"), Spacer(1, 0.6 * cm),
            H1u("List of Tables", toc=False), make_toc("TabEntry"), PageBreak()]


def writing_allocation(D):
    rows = [["Member", "Name and SAP ID", "Sections written by this member", "No. of pages written"],
            ["M1", "", "Abstract, 1 Introduction, 2 Problem Statement and Objectives, 6 System Design,"
                         " 12 Conclusion and Future Scope", ""],
            ["M2", "", "3 Literature and Industry Review, 4 Novelty, 13 References", ""],
            ["M3", "", "5 Syllabus Mapping and Theory, 7 Methodology and Algorithms", ""],
            ["M4", "", "8 Simulation Setup, 9 Results and Analysis, Appendix", ""],
            ["All", "Every member", "Section 10 (own row and signature), Section 11 (own reflection), "
                                    "Declaration", "-"]]
    s = [H1u("Writing Allocation")]
    s += table("Table 1: Writing allocation of the report sections", rows,
               [1.6 * cm, 4.0 * cm, 7.3 * cm, 2.6 * cm], align_center_from=3)
    s += [P("The allocation follows the technical module owned by each member (Section 10), so that every "
            "member writes about the part of the system he or she built and can defend it in the viva. "
            "Page counts are filled in after the final edit of each section.", "small"), Spacer(1, 0.6 * cm)]
    return s


def abstract(D):
    g, tr = D["gains"], D["tracks"]["summary"]
    la, zn = tr["complex"]["LA-GS"], tr["complex"]["PID-ZN"]
    best = D["kp"]["summary"]
    s = [H1u("Abstract"),
         P(f"Tape-guided automated guided vehicles (AGVs) move material in factories and warehouses, and most "
           f"of them steer with a PID loop that is tuned by trial and error. This project builds a complete "
           f"PyBullet simulation of a differential-drive AGV following floor tape. The AGV senses the tape with "
           f"a ray-cast eight-channel IR reflectance array or with a rendered camera, and seven steering "
           f"controllers are compared: bang-bang, relay-auto-tuned PID (Ziegler-Nichols and Tyreus-Luyben), "
           f"Nelder-Mead optimised PID, an optimised Sugeno fuzzy PD, and the proposed look-ahead gain-scheduled "
           f"controller (LA-GS) with its extended version LA-GS+. From the linearised kinematics we show that the "
           f"proportional gain <i>k</i><sub>p</sub>* = 2<i>v</i>/<i>L</i><sup>2</sup> keeps the axle on an arc "
           f"of any radius and, without actuator lag, gives a damping ratio of 1/√2. Gain sweeps placed the "
           f"measured optimum within 13 % of this prediction at three speeds, and black-box Nelder-Mead tuning "
           f"also converged to an almost purely proportional controller. On an unseen track LA-GS reduced the RMS axle error from {1000*zn['rms_cte'][0]:.1f} mm (Ziegler-"
           f"Nichols) to {1000*la['rms_cte'][0]:.1f} mm. LA-GS+ (sensor-range guard, Kalman filter and adaptive "
           f"speed) completed every run at sensor noise up to 30 % of full scale. Time-domain, frequency-domain, "
           f"speed, payload, friction, actuator-lag and camera-versus-IR studies are reported with reproducible "
           f"code."),
         Spacer(1, 0.4 * cm),
         P("<b>Keywords:</b> automated guided vehicle, line following, PID tuning, relay auto-tuning, gain "
           "scheduling, Kalman filter, fuzzy control, PyBullet, response analysis"), NextPageTemplate("main"),
         PageBreak()]
    return s


# ------------------------------------------------------------------ Sections
def sec1(D):
    return [H1("1. Introduction"),
            P("Automated guided vehicles carry pallets, bins and trolleys along fixed routes in manufacturing "
              "plants, hospitals, airports and warehouses. The simplest and still most widely installed guidance "
              "method is a line on the floor, either painted, made of coloured or reflective tape, or a "
              "magnetic strip, which the vehicle tracks with a short range sensor [1]. Line guidance is cheap, "
              "easy to re-route and does not need a map, which is why it remains common next to laser and "
              "natural-feature navigation. Large fleets of floor-guided drive units, such as the Kiva system "
              "used in Amazon fulfilment centres, show how far floor-referenced navigation can be scaled [2]."),
            P("At the heart of every line-following vehicle is a feedback loop: a sensor measures how far the "
              "line is from the centre of the vehicle, and a controller turns that error into a steering "
              "command. In practice the controller is almost always a PID controller, and its gains are found by "
              "trial and error on the real vehicle [3]. Poor tuning shows up as weaving on straights, cutting or "
              "overshooting corners, lost lines at higher speed and jerky steering that wears the drive train. "
              "Safety standards for driverless industrial trucks, such as ISO 3691-4, require that the vehicle "
              "stays inside its defined operating zone [4], so the deviation of the vehicle body from the "
              "path is a safety quantity and not only a quality measure."),
            P("This project studies the line-following problem the way a control engineer would. We build a "
              "physics-based PyBullet [5] simulation of a differential-drive AGV with realistic motors, wheel "
              "friction, payload and sensor noise, model the loop mathematically, and compare classical and "
              "intelligent controllers in the time and frequency domains. The work connects several units of "
              "the AISR syllabus: mobile ground robots and intelligent agents (Unit I), closed-loop control, "
              "stability, optimality and adaptation (Unit II), robotic perception and software architectures "
              "(Unit III) [6], sensors, fuzzy logic, error models and Kalman filtering (Unit IV), and the knowledge "
              "embedded in tuning rules (Unit V)."),
            H2("1.1 Why the problem matters in industry"),
            *bullets([
                "<b>Throughput:</b> an AGV that can run faster without leaving its lane moves more material per "
                "hour; the speed limit is set by the steering loop.",
                "<b>Safety and clearance:</b> aisles are only slightly wider than the vehicle and its load; the "
                "deviation of the axle (where the load sits) determines the required clearance.",
                "<b>Commissioning cost:</b> manual gain tuning must be repeated for each vehicle type, speed and "
                "payload; a design rule that gives the gain directly saves commissioning time.",
                "<b>Robustness:</b> dirty or worn tape, changing lighting and varying loads all appear to the "
                "controller as noise or plant changes."])]


def sec2(D):
    return [H1("2. Problem Statement and Objectives"),
            H2("2.1 Problem statement"),
            P("Design, implement and analyse the steering control of a differential-drive AGV that follows a "
              "25 mm wide dark tape on a light floor, using only on-board line sensing. The controller must keep "
              "the vehicle's axle, which carries the load, as close as possible to the tape over straights, arcs "
              "and tight corners, at speeds of 0.3 to 1.0 m/s, while remaining smooth and robust to sensor "
              "noise, payload, floor friction and actuator lag. The design must be supported by quantitative "
              "time-domain and frequency-domain analysis and by a comparison of tuning methods."),
            H2("2.2 Measurable objectives"),
            *numbered([
                "Build a reproducible PyBullet simulation of the AGV, tape tracks, IR array and camera, with "
                "ground-truth measurement of the axle cross-track error.",
                "Tune PID controllers by relay auto-tuning (Ziegler-Nichols and Tyreus-Luyben) and by "
                "simulation-based Nelder-Mead optimisation, and compare them with bang-bang and fuzzy control.",
                "Report rise time, overshoot and settling time for a 5 cm step, and gain and phase margins of "
                "the loop for every linear controller.",
                "Reduce the RMS axle error on an unseen validation track by at least 50 % compared with "
                "classical Ziegler-Nichols tuning without increasing lap time by more than 20 %.",
                "Complete at least 90 % of runs on the validation track with sensor noise up to 20 % of full "
                "scale, and quantify robustness to speed, payload, friction and motor lag."]),
            H2("2.3 Assumptions and limits"),
            *bullets([
                "Flat, rigid floor; the tape is continuous (no junctions or gaps) and has constant reflectance.",
                "The IR reflectance is modelled from the sensor's spot coverage of the tape plus Gaussian noise "
                "and 10-bit quantisation; ambient light and specular effects are not modelled.",
                "Motors are velocity-controlled with a first-order lag (tau = 60 ms) and a torque limit; "
                "battery dynamics and wheel wear are ignored.",
                "Results come from simulation only; transfer to hardware is discussed in Section 12."])]


def sec3(D):
    rows = [["Source", "What it offers", "Limitation for our problem"],
            ["Ziegler-Nichols rules [7]", "Gains from ultimate gain and period", "Aggressive; tuned for "
             "disturbance rejection, not path geometry"],
            ["Relay auto-tuning, Astrom-Hagglund [8]", "Ku, Tu from a safe limit-cycle experiment",
             "Inherits the rule's objective (sensor error, not axle error)"],
            ["Tyreus-Luyben rules [9]", "More robust settings for integrating plants", "Same objective; slower"],
            ["Pure pursuit, Coulter [10]", "Geometric look-ahead steering, curvature = 2y/L^2",
             "Needs a path and pose estimate, not a line sensor"],
            ["Line-follower design studies [3]", "Sensor arrays, PID implementation", "Gains found by trial "
             "and error; no frequency-domain analysis"],
            ["Fuzzy control, Takagi-Sugeno [11]", "Nonlinear rule-based steering", "Many tuning parameters"],
            ["Kalman filtering [12]", "Optimal estimation under noise", "Adds lag if poorly tuned"],
            ["ISO 3691-4 [4]", "Safety requirements for driverless trucks", "Defines zones, not controllers"],
            ["Kiva / Amazon Robotics [2]", "Large fleets of floor-referenced drive units", "Proprietary; "
             "fiducial markers, not continuous tape"]]
    return [H1("3. Literature and Industry Review"),
            P("Line and tape guidance is described in AGV textbooks as the oldest and most robust guidance "
              "principle: the vehicle measures the lateral offset of an optical or magnetic track with a sensor "
              "bar and steers to reduce it [1]. Ullrich's survey of AGV systems lists optical tape, magnetic "
              "tape and inductive wire as continuous guidance options and notes that tape systems are favoured "
              "where routes change often, because re-routing only means re-taping [1]. Floor-referenced "
              "navigation has also been scaled to fleets of hundreds of vehicles, as in the Kiva warehouse "
              "system, where drive units read floor markers and are coordinated by a central planner [2]."),
            P("For the steering loop itself, the classical tuning literature starts with Ziegler and Nichols "
              "[6], who obtained PID gains from the ultimate gain <i>K</i><sub>u</sub> and period "
              "<i>T</i><sub>u</sub> of a loop brought to the stability limit. Astrom and Hagglund [8] replaced "
              "the risky ultimate-gain experiment by relay feedback, which produces a small, controlled limit "
              "cycle from which <i>K</i><sub>u</sub> and <i>T</i><sub>u</sub> follow by describing-function "
              "analysis; this relay auto-tuner is used in many industrial controllers today [13]. Tyreus and "
              "Luyben [9] proposed more conservative rules for integrating processes, which a line follower is "
              "(the lateral position integrates the heading)."),
            P("Practical line-follower papers concentrate on sensor arrays and microcontroller implementation, "
              "and tune the PID gains by experiment [3]. In mobile robotics the closely related path-tracking "
              "problem is solved geometrically by pure pursuit [10]: the robot steers along the circular arc that "
              "reaches a look-ahead point on the path, which gives the curvature command "
              "2<i>y</i>/<i>L</i><sup>2</sup> for a lateral offset <i>y</i> at look-ahead distance <i>L</i>. "
              "Textbooks on mobile robots [14] and probabilistic robotics [15] provide the kinematic models and "
              "estimation tools (Kalman filter [12]) used for such vehicles, and fuzzy logic controllers of the "
              "Takagi-Sugeno type [11] are a common intelligent alternative to PID. Physics simulators such as "
              "PyBullet [5] make it possible to evaluate these methods reproducibly with contact dynamics, "
              "friction and rendered cameras. Table 2 summarises the sources."),
            *table("Table 2: Summary of reviewed research, standards and products", rows,
                   [4.2 * cm, 5.4 * cm, 5.9 * cm], align_center_from=9),
            H2("3.1 What is missing"),
            *bullets([
                "Line-follower tuning is evaluated on the <i>sensor</i> error. Nobody separates this from the "
                "<i>axle</i> (load) deviation that matters for clearance and safety, and the two conflict on "
                "curves.",
                "Pure pursuit gives the geometric look-ahead relation, but it is not used to derive the gain of "
                "a sensor-bar PID controller, nor to explain how the optimal gain depends on speed.",
                "Published comparisons rarely report stability margins, speed dependence and robustness to "
                "payload, friction, actuator lag and sensor noise on a common, reproducible benchmark.",
                "Classical rules (ZN, TL) and black-box optimisation are seldom compared on unseen tracks."])]


def sec4(D, T):
    g = D["gains"]; tr = D["tracks"]["summary"]; nz = D["noise"]["summary"]; kp = D["kp"]["summary"]
    la, zn = tr["complex"]["LA-GS"], tr["complex"]["PID-ZN"]
    red = 100 * (1 - la["rms_cte"][0] / zn["rms_cte"][0])
    lp = nz["LA-GS+ (guard + KF + speed)"]; zn_n = nz["PID-ZN"]
    return [H1("4. Novelty"),
            P("Compared with the textbook PID line follower and with the reviewed literature, this project "
              "contributes the following. Each point is backed by a result in Section 9."),
            *numbered([
                f"<b>Analytical look-ahead gain rule.</b> We derive that <i>k</i><sub>p</sub>* = 2<i>v</i>/"
                f"<i>L</i><sup>2</sup> is the unique proportional gain giving zero steady-state axle offset on an "
                f"arc of <i>any</i> radius, and that in the lag-free model it fixes the damping ratio at 1/√2 "
                f"at every speed. <i>Evidence:</i> the measured optimum of a 21-point gain sweep lies within 13 % "
                f"of the prediction at 0.25, 0.40 and 0.55 m/s (Table {T.t('kpsweep')}, Figure "
                f"{T.f('kp_sweep')}), and a Nelder-Mead optimiser started at (10, 1, 0.3) converged to an almost "
                f"purely proportional controller (k<sub>i</sub> = {g['PID-NM']['ki']:.2f}, k<sub>d</sub> = "
                f"{g['PID-NM']['kd']:.3f}).",
                f"<b>Axle-error versus sensor-error insight.</b> We show that integral action and high-gain "
                f"classical tuning drive the sensor error to zero but make the axle cut every corner by "
                f"<i>L</i><sup>2</sup>/2<i>R</i>. <i>Evidence:</i> on the unseen track PID-ZN has the smallest "
                f"sensor error but {1000*zn['rms_cte'][0]:.1f} mm RMS axle error, against "
                f"{1000*la['rms_cte'][0]:.1f} mm for LA-GS (a {red:.0f} % reduction, Table {T.t('tracks')}).",
                f"<b>Speed-scheduled gain with a lag-aware validity condition.</b> Re-computing "
                f"<i>k</i><sub>p</sub> from the speed command every step keeps the AGV inside a 50 mm envelope "
                f"at every speed from 0.3 to 1.0 m/s with 3.3-4.5 mm RMS error. A fixed gain is over-damped at low "
                f"speed and under-damped at high speed. We also show when the constant-damping result breaks down "
                f"(√2<i>v</i>/<i>L</i> approaching 1/τ) and confirm it in PyBullet (Figure "
                f"{T.f('speed_damping')}, Figure {T.f('speed_sweep')}).",
                f"<b>Sensor-range-aware robust extension (LA-GS+).</b> We identify the design limit "
                f"<i>R</i><sub>min</sub> = <i>L</i><sup>2</sup>/(2<i>w</i><sub>s</sub>) below which the line "
                f"leaves the sensor, and add a peak-window line estimator, a sensor-range guard, a Kalman filter "
                f"and adaptive speed. <i>Evidence:</i> at 30 % sensor noise LA-GS+ completed "
                f"{100*lp['0.3']['success']:.0f} % of runs with {1000*lp['0.3']['rms_cte'][0]:.1f} mm RMS error, "
                f"PID-ZN {100*zn_n['0.3']['success']:.0f} % with {1000*zn_n['0.3']['rms_cte'][0]:.1f} mm "
                f"(ablation in Table {T.t('noise')}).",
                "<b>Reproducible benchmark.</b> One open code base produces every figure and table: relay "
                "auto-tuning, system identification, Bode and margin analysis, linear-model validation against "
                "physics, speed, noise, payload, friction, motor-lag and camera-versus-IR studies with Monte-Carlo "
                "seeds."])]


def sec5(D):
    ident = D["ident"]
    rows = [["Unit", "Syllabus topic", "Where it is used in this project"],
            ["I", "Mobile ground robots; intelligent agents", "Differential-drive AGV model; agent "
             "perceive-decide-act loop (Section 6)"],
            ["I", "Declarative-procedural-reflexive hierarchy", "Mission layer, control layer and reflexive "
             "line-loss supervisor (Figure 2)"],
            ["II", "Open- and closed-loop control", "Open-loop identification (E1); closed-loop steering"],
            ["II", "Time- and frequency-domain analysis", "Step metrics (E4); Bode, margins, bandwidth (E5)"],
            ["II", "Stability and performance", "Phase/gain margins, dominant-pole damping, overshoot"],
            ["II", "Optimality and constraints", "Nelder-Mead tuning cost; actuator and sensor saturation"],
            ["II", "Adaptation; control actuation", "Gain scheduling kp = 2v/L^2; adaptive speed; motor model"],
            ["III", "Robotic perception; robot hardware", "IR array model, camera rendering and inverse "
             "perspective mapping"],
            ["III", "Robotic software architectures", "Layered architecture, modular Python package"],
            ["IV", "Sensors and sensing; analog and digital", "Spot-coverage reflectance model, 10-bit ADC"],
            ["IV", "Formal and fuzzy logic", "5 x 5 Sugeno fuzzy PD controller"],
            ["IV", "Probability and error models; sensor-based estimation", "Gaussian noise model; Kalman "
             "line-state filter"],
            ["V", "Knowledge in learning; explanation-based learning", "Tuning rules (ZN/TL) as encoded expert "
             "knowledge; theory explaining what the optimiser learned"]]
    s = [H1("5. Syllabus Mapping and Theory"),
         P("Table 3 maps the AISR syllabus topics to the parts of the project where they are applied. The rest "
           "of this section develops the theory used by the controllers and the analysis; the control-theoretic "
           "tools (transfer functions, Bode plots, stability margins, dominant poles) follow [16]."),
         *table("Table 3: Mapping of AISR syllabus topics to the project", rows, [1.2 * cm, 5.6 * cm, 8.7 * cm],
                align_center_from=9),
         H2("5.1 Differential-drive kinematics"),
         P("With wheel radius <i>r</i>, track width <i>b</i> and wheel speeds ω<sub>L</sub>, "
           "ω<sub>R</sub>, the forward speed and yaw rate of the AGV are"),
         P("<i>v</i> = <i>r</i>(ω<sub>R</sub> + ω<sub>L</sub>)/2, &nbsp;&nbsp; ω = "
           "<i>r</i>(ω<sub>R</sub> − ω<sub>L</sub>)/<i>b</i>, &nbsp;&nbsp; "
           "ẋ = <i>v</i> cosθ, &nbsp; ẏ = <i>v</i> sinθ, &nbsp; θ̇ = ω"
           " &nbsp;&nbsp;&nbsp; (1)", "eq"),
         P("The controller commands (<i>v</i>, ω) and the inverse kinematics ω<sub>L,R</sub> = "
           "(<i>v</i> -/+ ω<i>b</i>/2)/<i>r</i> gives the wheel references. When a wheel reference exceeds "
           "the motor limit the forward speed is reduced and the turn rate kept, so steering has priority."),
         H2("5.2 Linearised line-following model"),
         P("Let <i>y</i> be the lateral offset of the axle from the tape (positive to the left) and ψ the "
           "heading error. The IR bar sits a distance <i>L</i> ahead of the axle, so the line error it measures "
           "is <i>e</i> = −(<i>y</i> + <i>L</i> sinψ). For small angles and an actuator that follows "
           "the commanded yaw rate with lag τ and dead time <i>T</i><sub>d</sub>:"),
         P("ẏ = <i>v</i>ψ, &nbsp;&nbsp; ψ̇ = ω, &nbsp;&nbsp; "
           "ω(<i>s</i>) = e<sup>−<i>sT</i>d</sup> ω<sub>cmd</sub>(<i>s</i>)/(τ<i>s</i> + 1)"
           " &nbsp;&nbsp;&nbsp; (2)", "eq"),
         P("<i>G</i>(<i>s</i>) = <i>E</i>(<i>s</i>)/Ω<sub>cmd</sub>(<i>s</i>) = −(<i>Ls</i> + <i>v</i>)"
           " e<sup>−<i>sT</i>d</sup> / [<i>s</i><sup>2</sup>(τ<i>s</i> + 1)]"
           " &nbsp;&nbsp;&nbsp; (3)", "eq"),
         P(f"The plant is a double integrator with a zero at <i>s</i> = −<i>v</i>/<i>L</i> created by the "
           f"look-ahead. System identification in PyBullet (Section 9.1) gave τ = {1000*ident['tau']:.0f} ms "
           f"and negligible dead time; the control period of 1/60 s adds an effective delay of half a sample. "
           f"With the controller ω<sub>cmd</sub> = <i>C</i>(<i>s</i>)<i>e</i>, the loop transfer function "
           f"is <i>L</i>(<i>s</i>) = <i>C</i>(<i>s</i>)(<i>Ls</i> + <i>v</i>)e<sup>−<i>sT</i>d</sup>/"
           f"[<i>s</i><sup>2</sup>(τ<i>s</i> + 1)]."),
         H2("5.3 The look-ahead gain rule kp* = 2v/L²"),
         P("Consider the AGV driving with its axle exactly on an arc of radius <i>R</i> and heading tangent to "
           "it. The sensor point, a distance <i>L</i> ahead along the heading, lies at distance "
           "√(<i>R</i><sup>2</sup> + <i>L</i><sup>2</sup>) ≈ <i>R</i> + <i>L</i><sup>2</sup>/2<i>R</i> "
           "from the centre, so the sensor sees the tape displaced towards the inside of the curve by"),
         P("<i>e</i><sub>arc</sub> = <i>L</i><sup>2</sup>/(2<i>R</i>). &nbsp;&nbsp;&nbsp; (4)", "eq"),
         P("Staying on the arc requires the yaw rate ω = <i>v</i>/<i>R</i>. A proportional controller "
           "ω = <i>k</i><sub>p</sub><i>e</i> supplies it exactly when"),
         P("<i>k</i><sub>p</sub><i>L</i><sup>2</sup>/(2<i>R</i>) = <i>v</i>/<i>R</i> &nbsp; → &nbsp; "
           "<b><i>k</i><sub>p</sub>* = 2<i>v</i>/<i>L</i><sup>2</sup></b> &nbsp;&nbsp;&nbsp; (5)", "eq"),
         P("The radius cancels, so one gain is correct for every curve. This is the same geometry as the pure "
           "pursuit law [10], here applied to a sensor-bar controller. Two consequences follow. First, any "
           "integral action forces <i>e</i> → 0 in steady state and therefore moves the axle inwards by "
           "<i>L</i><sup>2</sup>/2<i>R</i>: the robot cuts corners. Second, ignoring the actuator lag, the closed "
           "loop with <i>C</i> = <i>k</i><sub>p</sub> has the characteristic equation "
           "<i>s</i><sup>2</sup> + <i>k</i><sub>p</sub><i>Ls</i> + <i>k</i><sub>p</sub><i>v</i> = 0, i.e."),
         P("ω<sub>n</sub> = √(<i>k</i><sub>p</sub><i>v</i>), &nbsp;&nbsp; ζ = "
           "(<i>L</i>/2)√(<i>k</i><sub>p</sub>/<i>v</i>) &nbsp; → &nbsp; with <i>k</i><sub>p</sub>*: "
           "&nbsp; ζ = 1/√2, &nbsp; ω<sub>n</sub> = √2 <i>v</i>/<i>L</i>. "
           "&nbsp;&nbsp;&nbsp; (6)", "eq"),
         P("The zero-offset gain therefore also gives the classical optimum damping ratio of 0.707 (4.3 % "
           "overshoot), independent of speed, provided <i>k</i><sub>p</sub> is re-scheduled as <i>v</i> changes "
           "and the actuator is fast compared with ω<sub>n</sub>, i.e. √2<i>v</i>/<i>L</i> &lt;&lt; "
           "1/τ (Section 9.5 quantifies the effect of the 60 ms motor lag). "
           "This is the LA-GS controller. Equation (4) also sets a limit: the steady sensor error must fit "
           "inside the sensor half-span <i>w</i><sub>s</sub>, so zero axle offset is only possible for "
           "<i>R</i> ≥ <i>R</i><sub>min</sub> = <i>L</i><sup>2</sup>/(2<i>w</i><sub>s</sub>), which is "
           "0.23 m for our IR bar (<i>L</i> = 0.15 m, <i>w</i><sub>s</sub> = 49 mm). Tighter corners require the "
           "sensor-range guard of LA-GS+."),
         H2("5.4 Discrete PID with filtered derivative and anti-windup"),
         P("All PID variants run at <i>T</i> = 1/60 s as <i>u</i><sub>k</sub> = sat(<i>k</i><sub>p</sub>"
           "<i>e</i><sub>k</sub> + <i>k</i><sub>i</sub><i>I</i><sub>k</sub> + <i>k</i><sub>d</sub><i>D</i>"
           "<sub>k</sub>), where <i>D</i><sub>k</sub> is the error rate low-pass filtered with cut-off "
           "<i>N</i> = 15 rad/s, and the integral <i>I</i><sub>k</sub> is only updated when the output is not "
           "saturated in the direction of the error (conditional-integration anti-windup)."),
         H2("5.5 Relay auto-tuning and tuning rules"),
         P("Under relay feedback of amplitude <i>d</i> with hysteresis <i>h</i> the loop oscillates with "
           "amplitude <i>a</i> and period <i>T</i><sub>u</sub>. The describing function of the relay gives the "
           "ultimate gain [8]"),
         P("<i>K</i><sub>u</sub> = 4<i>d</i> / (π√(<i>a</i><sup>2</sup> − <i>h</i><sup>2</sup>))."
           " &nbsp;&nbsp;&nbsp; (7)", "eq"),
         P("Ziegler-Nichols [7] then sets <i>k</i><sub>p</sub> = 0.6<i>K</i><sub>u</sub>, <i>T</i><sub>i</sub> = "
           "<i>T</i><sub>u</sub>/2, <i>T</i><sub>d</sub> = <i>T</i><sub>u</sub>/8, and Tyreus-Luyben [9] "
           "<i>k</i><sub>p</sub> = <i>K</i><sub>u</sub>/2.2, <i>T</i><sub>i</sub> = 2.2<i>T</i><sub>u</sub>, "
           "<i>T</i><sub>d</sub> = <i>T</i><sub>u</sub>/6.3, with <i>k</i><sub>i</sub> = <i>k</i><sub>p</sub>/"
           "<i>T</i><sub>i</sub> and <i>k</i><sub>d</sub> = <i>k</i><sub>p</sub><i>T</i><sub>d</sub>."),
         H2("5.6 Kalman line-state estimator"),
         P("The line error is modelled as a constant-velocity process <b>x</b> = [<i>e</i>, ė]<sup>T</sup> "
           "with <b>F</b> = [[1, <i>T</i>], [0, 1]], white-acceleration process noise "
           "<b>Q</b> = <i>q</i>[[<i>T</i><sup>3</sup>/3, <i>T</i><sup>2</sup>/2], [<i>T</i><sup>2</sup>/2, "
           "<i>T</i>]] and measurement <i>z</i> = <i>e</i> + <i>n</i>, <i>n</i> ~ N(0, <i>r</i>) [12], [15]. "
           "The standard predict/update cycle runs every control step; when the line is lost, only the "
           "prediction is applied. We use <i>q</i> = 4 m<sup>2</sup>s<sup>−3</sup> and <i>r</i> = "
           "4×10<sup>−5</sup> m<sup>2</sup>."),
         H2("5.7 Fuzzy PD controller"),
         P("The fuzzy controller [11] normalises the error and its filtered rate by scaling factors, fuzzifies "
           "each with five triangular sets (NB, NS, Z, PS, PB), fires a 5 × 5 rule base with the min "
           "t-norm and defuzzifies the output singletons {−1, −0.6, −0.25, 0, 0.25, 0.6, 1} "
           "× ω<sub>max</sub> by weighted average (zero-order Sugeno inference). The rule table "
           "steers harder for large errors and anticipates with the error rate.")]
    return s


def sec6(D, T):
    rows = [["Parameter", "Value", "Parameter", "Value"],
            ["Chassis (L x W x H)", "0.30 x 0.20 x 0.05 m", "Wheel radius r", "0.040 m"],
            ["Track width b", "0.240 m", "Chassis mass", "4.0 kg (+ payload)"],
            ["Max wheel speed", "25 rad/s (1.0 m/s)", "Max wheel torque", "2.5 N m"],
            ["Motor time constant", "60 ms", "Wheel-floor friction", "0.9"],
            ["Casters", "2 ball casters, frictionless", "Yaw-rate limit", "5 rad/s"],
            ["IR sensors", "8, pitch 14 mm", "IR look-ahead L", "0.15 m"],
            ["IR spot radius", "6 mm", "IR noise / ADC", "2 % FS / 10 bit"],
            ["Camera", "80 x 60 px, 70 deg FOV", "Camera pose", "0.19 m ahead, 0.18 m high, 55 deg"],
            ["Tape width", "25 mm", "Control / physics rate", "60 Hz / 240 Hz"]]
    return [H1("6. System Design"),
            H2("6.1 System architecture"),
            P("Figure 1 shows the closed-loop architecture. It follows the declarative-procedural-reflexive "
              "hierarchy of the syllabus. The <b>declarative</b> mission layer fixes the track, number of laps "
              "and top speed. The <b>procedural</b> layer runs perception, estimation, the steering controller "
              "and the speed scheduler at 60 Hz. The <b>reflexive</b> supervisor reacts immediately to "
              "exceptional events: if the line is lost it turns the AGV towards the side where the line was "
              "last seen at low speed, and it stops the run if the axle leaves a 0.2 m corridor or the line "
              "stays lost for 2 s."),
            figure(D["fig"]("system_block_diagram"), "Figure 1: Closed-loop architecture of the line-following "
                   "AGV (declarative, procedural and reflexive layers)", 0.92),
            H2("6.2 Robot and environment model"),
            P("The AGV is generated as a URDF from the parameter file so that the controller and the physics "
              "always use the same geometry (Table 4). Two driven wheels on a common axle and two frictionless "
              "ball casters support an orange chassis; an IR sensor bar is mounted 0.15 m ahead of the axle and "
              "a camera mast at the front bumper (Figure 2). The wheels are driven by PyBullet velocity motors "
              "with a torque limit, preceded by a first-order lag that represents the motor speed loop. A payload "
              "is modelled by increasing the chassis mass. The floor is an infinite collision plane carrying a "
              "texture rasterised from the track geometry, so the camera sees exactly the tape that the IR "
              "model and the ground-truth error use."),
            *table("Table 4: Robot, sensor and simulation parameters", rows, [3.7 * cm, 4.1 * cm, 3.6 * cm,
                                                                              4.1 * cm], align_center_from=9),
            figure(D["fig"]("scene_robot_closeup"), "Figure 2: The simulated AGV on the taped floor (PyBullet "
                   "TinyRenderer view): chassis, drive wheels, IR bar and camera mast", 0.52),
            P("Two tracks are used (Figure 3). The <b>oval</b> (9.71 m, straights and R = 0.75 m bends) is the "
              "<i>training</i> track on which all tuning is done. The <b>complex</b> track (10.95 m) is used "
              "only for <i>validation</i>; it contains R = 0.5 and 0.4 m bends, a hairpin (R = 0.3 m), a chicane "
              "and two near-90 degree corners with R = 0.12 m and 0.15 m, which are below the R<sub>min</sub> "
              "limit of Section 5.3. A straight 7 m tape is used for step and relay tests."),
            figure(D["fig"]("track_maps"), "Figure 3: Training (oval) and validation (complex) tracks coloured "
                   "by curvature", 0.82),
            H2("6.3 Sensors and actuators"),
            P("<b>IR reflectance array.</b> Each of the eight sensors casts a PyBullet ray to the floor to find "
              "its footprint (this accounts for chassis pitch and roll). The fraction of the 6 mm IR spot "
              "covered by the 25 mm tape is computed analytically from the distance to the tape centre-line "
              "(area of a disc inside a strip). The reflectance mixes floor (0.85) and tape (0.08), Gaussian "
              "noise is added and the value is quantised by a 10-bit ADC. After calibration to a darkness value "
              "in [0, 1] the line position is the darkness-weighted centroid of the darkest sensor and its two "
              "neighbours (peak-window centroid), which rejects noise from sensors that cannot see the tape."),
            P("<b>Camera.</b> An 80 × 60 pixel pinhole camera is rendered by PyBullet. The image is "
              "thresholded, the dark-pixel centroid of each row is back-projected onto the floor with inverse "
              "perspective mapping (IPM), and a weighted quadratic fitted to the floor points gives the line "
              "offset at <i>L</i> = 0.28 m, the heading and a curvature preview. A calibration on a straight tape "
              "showed zero bias in both driving directions after correcting half-pixel conventions."),
            H2("6.4 Software architecture"),
            P("The software is a Python package with one module per responsibility (Figure 4): parameters, "
              "track geometry, PyBullet world, sensors, controllers, the closed-loop runner, analysis and "
              "tuning. Ten experiment scripts produce JSON results and figures, and the report is generated "
              "from the same JSON files, so every number in this report is traceable to code."),
            figure(D["fig"]("software_architecture"), "Figure 4: Software architecture of the project "
                   "repository", 0.78)]


def sec7(D, T):
    pseudo = """procedure RUN_EPISODE(config)
    track <- TRACK(config.track);  world <- PYBULLET_WORLD(track, AGV_URDF(config))
    sensor, controller, kalman, scheduler <- MAKE(config)
    repeat every T = 1/60 s
        reading <- sensor.READ()                     # e, lost flag, curvature
        e_hat, de_hat <- kalman.STEP(reading)        # or raw reading if disabled
        if reading.lost then                          # reflexive layer
            w <- sign(last_e) * 0.8 * w_max ;  v <- 0.5 * v_min
        else                                          # procedural layer
            if LA-GS then controller.kp <- 2 * v_prev / L^2
            w <- controller.STEP(e_hat, de_hat)
            if guard and |e_hat| > 0.65 * w_s then w <- w + k_g * sign(e_hat) * (|e_hat| - 0.65 * w_s)
            v <- scheduler.STEP(e_hat, w, curvature)
        wheel_L, wheel_R <- INVERSE_KINEMATICS(v, w) with saturation (turn priority)
        world.STEP(4 physics sub-steps with motor lag)
        log ground-truth axle error, progress, power
    until end reached or derailed (|cte| > 0.2 m) or line lost > 2 s or timeout"""
    tune = """procedure RELAY_AUTOTUNE(d = 1.0 rad/s, h = 1 mm)
    run straight-tape episode with w = +d if e > h, -d if e < -h (else hold)
    discard first 5 s;  Tu <- mean period of upward zero crossings
    a  <- half peak-to-peak of e;  Ku <- 4d / (pi * sqrt(a^2 - h^2))
    return ZN(Ku, Tu), TL(Ku, Tu)

procedure NM_TUNE(x0)
    J(x) = RMS_axle_error(oval lap) + IAE(5 cm step) + 0.001 * steering_activity
           + penalty if a run fails                     # x = log10(kp, ki, kd)
    simplex <- x0 + 0.5 decade per parameter
    return NELDER_MEAD(J, simplex, max 200 evaluations)"""
    rows = [["ID", "Experiment", "Conditions", "Metrics"],
            ["E1", "Identification and scene", "Open-loop yaw-rate step, payload 0/10/20 kg", "tau, Td"],
            ["E2", "Auto-tuning", "Relay d = 0.5/1/2 rad/s; Nelder-Mead", "Ku, Tu, gains, cost"],
            ["E3", "Gain sweep", "P control, 21 gains x 3 speeds, oval", "RMS axle error, step"],
            ["E4", "Step response", "5 cm offset, straight, 0.4 m/s", "tr, OS, ts, IAE"],
            ["E5", "Frequency domain", "Linear model, speeds 0.2-0.8 m/s", "PM, GM, bandwidth, zeta"],
            ["E6", "Track benchmark", "Oval and complex, 5 seeds, 0.4 m/s", "RMS/max error, time, energy"],
            ["E7", "Speed sweep", "Complex, 0.3-1.0 m/s, 3 seeds", "In-envelope rate, max error"],
            ["E8", "Noise and ablation", "Noise 0-30 % FS, 5 seeds", "Completion, RMS error"],
            ["E9", "Robustness", "Payload, friction, motor lag, 3 seeds", "In-envelope rate, RMS error"],
            ["E10", "Camera vs IR", "Complex, 3 seeds", "RMS error, sensor time"]]
    return [H1("7. Methodology and Algorithms"),
            H2("7.1 Step-by-step method"),
            *numbered([
                "Model the AGV, tracks and sensors in PyBullet and validate the sensing chain (camera bias test, "
                "unit tests of geometry, coverage model, PID, fuzzy and Kalman code).",
                "Identify the actuator dynamics with an open-loop yaw-rate step (E1) and build the linear model "
                "of Section 5.2.",
                "Tune the PID controllers by relay feedback with ZN and TL rules, and by Nelder-Mead on the "
                "training scenarios only (E2); tune the fuzzy scaling factors the same way.",
                "Derive the LA-GS gain analytically and check it against a gain sweep (E3).",
                "Evaluate all controllers in the time domain (E4) and frequency domain (E5), on the training and "
                "unseen tracks (E6), and under speed, noise, payload, friction, lag and sensor changes "
                "(E7 to E10), using Monte-Carlo seeds for noise and start pose."]),
            H2("7.2 Control loop algorithm"),
            P("Figure 5 shows the flowchart of one episode and the pseudo-code below lists the main loop."),
            figure(D["fig"]("control_flowchart"), "Figure 5: Flowchart of the closed-loop simulation episode",
                   0.42, max_h=9 * cm),
            code(pseudo),
            H2("7.3 Tuning algorithms"),
            P("The relay test replaces the controller by a relay with hysteresis on the straight tape; after "
              "the transient, the limit cycle gives <i>K</i><sub>u</sub> and <i>T</i><sub>u</sub> by Equation (7). "
              "For simulation-based tuning, the cost <i>J</i> combines the RMS axle error over one oval lap, the "
              "integral absolute axle error of a 5 cm step and a small penalty on steering activity. The "
              "derivative-free Nelder-Mead simplex method [17] searches in log-space from the neutral start (<i>k</i><sub>p</sub>, <i>k</i><sub>i</sub>, "
              "<i>k</i><sub>d</sub>) = (10, 1, 0.3). An explicit initial simplex of half a decade per "
              "parameter was needed: SciPy's default 5 % step degenerates to nearly zero for a coordinate equal to "
              "log<sub>10</sub>(1) = 0."),
            code(tune),
            H2("7.4 Perception algorithm (camera)"),
            P("Each 80 × 60 frame is converted to grey with pixel noise and thresholded halfway between its "
              "2nd and 95th grey-level percentiles (at most 110). For every row, the dark pixels are mapped to the "
              "floor with a pre-computed IPM table and averaged; rows wider than three tape widths are rejected. A "
              "weighted quadratic <i>y</i> = <i>a</i> + <i>bx</i> + <i>cx</i><sup>2</sup> through the row points "
              "gives <i>e</i> = <i>y</i>(<i>L</i>), the heading and the curvature; fewer than four valid rows "
              "means the line is lost."),
            H2("7.5 Experimental design and metrics"),
            P("Table 5 lists the ten experiments. The primary metric is the ground-truth RMS cross-track error of "
              "the <b>axle</b> (where the load sits); we also report the maximum axle error, the RMS error at the "
              "sensor point, run time, steering activity (mean |Δω|/<i>T</i>), wheel energy and the "
              "in-envelope rate (run completed with maximum axle error ≤ 50 mm)."),
            *table("Table 5: Experiment plan", rows, [1.1 * cm, 3.6 * cm, 6.1 * cm, 4.7 * cm],
                   align_center_from=9)]


def sec8(D, T):
    rows = [["Item", "Setting"],
            ["Platform", "PyBullet 3.2.7 (Bullet physics), Python 3.11, NumPy, SciPy, Matplotlib"],
            ["Host", "Windows 11 + WSL2 Ubuntu (WSLg for the GUI); runs on any Linux/macOS"],
            ["Physics step / solver", "1/240 s, 50 solver iterations, gravity 9.81 m/s^2"],
            ["Control rate", "60 Hz (4 physics steps per control step)"],
            ["Renderer", "TinyRenderer (headless) / OpenGL (GUI); floor texture 250 px/m"],
            ["Monte-Carlo", "Seeds vary sensor noise, start offset (+-10 mm) and heading (+-3 deg)"],
            ["Failure criteria", "|axle error| > 0.20 m, line lost > 2 s, or timeout (90 s)"],
            ["Runtime", "About 0.8 s per lap headless; full pipeline E1-E10 about 6 min on 16 threads"]]
    cmds = """# environment (WSL2 Ubuntu / Linux / macOS)
python3.11 -m venv ~/venvs/agv && ~/venvs/agv/bin/pip install -r requirements.txt

python -m pytest -q                                   # unit tests
python scripts/demo.py --track complex --controller LA-GS+ --speed 0.5   # live GUI demo
python scripts/demo.py --track oval --controller PID-ZN                  # classic tuning
python scripts/demo.py --sensor camera --controller LA-GS                # camera perception
python experiments/run_all.py                         # all experiments -> results/
python scripts/make_diagrams.py && python report/build_report.py         # figures + this PDF"""
    return [H1("8. Simulation Setup"),
            P("The simulation runs in PyBullet [5], a Python interface to the Bullet physics engine, chosen for "
              "its contact dynamics, URDF support, ray casting and built-in camera rendering without extra "
              "services. Table 6 lists the platform settings. The world contains the textured floor and the AGV "
              "only (Figure 2 shows the AGV on the taped floor)."),
            *table("Table 6: Simulation platform and settings", rows, [4.0 * cm, 11.5 * cm], align_center_from=9),
            H2("8.1 How to run the code"),
            P("The repository README gives installation options for Windows (WSL2, conda or a source build), "
              "Linux and macOS. The commands below reproduce every result in this report; all random seeds are "
              "fixed."),
            code(cmds)]
