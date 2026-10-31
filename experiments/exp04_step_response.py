"""E4: time-domain step response of every controller + linear-model validation.

The AGV starts 5 cm to the left of a straight tape (v = 0.40 m/s).  Metrics are
computed on the ground-truth axle offset.  The linear model of agv/analysis.py
(with the identified actuator lag) is simulated with the same discrete PID and
overlaid on the PyBullet response.
"""
from common import *  # noqa: F401,F403
from agv.analysis import step_metrics, simulate_linear

OFFSET = 0.05


def main():
    gains = load_json("gains")
    ident = load_json("identification")
    ctrls = controller_set(gains)
    base = SimConfig(track="straight", initial_offset=OFFSET, stop_at_distance=3.6, max_time=20)
    names = list(ctrls)
    res = run_many([with_overrides(base, **ctrls[n]) for n in names], keep_logs=True)
    metrics, logs = {}, {}
    for n, (log, s) in zip(names, res):
        metrics[n] = dict(step_metrics(log["t"], log["cte"]), smoothness=s.get("smoothness"),
                          peak_w=float(np.max(np.abs(log["w_cmd"]))), status=s["status"])
        logs[n] = log

    # linear model predictions
    v, L = base.control.base_speed, base.sensor.lookahead
    tau, Td, dt = ident["tau"], ident["loop_delay"], base.control_dt
    kp_star = 2 * v / L ** 2
    lin_cases = {"LA-GS": (kp_star, 0.0, 0.0), "PID-NM": tuple(gains["PID-NM"][k] for k in ("kp", "ki", "kd")),
                 "PID-TL": tuple(gains["PID-TL"][k] for k in ("kp", "ki", "kd"))}
    linear = {}
    for n, (kp, ki, kd) in lin_cases.items():
        t, y = simulate_linear(kp, ki, kd, base.control.d_filter_n, v, L, tau, Td, dt, OFFSET,
                               t_end=6.0, u_max=base.control.omega_max)
        lm = step_metrics(t, y)
        linear[n] = dict(t=t[::20], y=y[::20], metrics=lm)
        metrics[n]["linear_model"] = lm
    save_json("step_response", dict(metrics=metrics, offset=OFFSET, speed=v,
                                    linear={k: dict(metrics=v_["metrics"]) for k, v_ in linear.items()}))

    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.0))
    for n in names:
        lg = logs[n]
        axs[0].plot(lg["t"], 1000 * lg["cte"], color=COLORS[n], label=n, lw=1.6 if n.startswith("LA") else 1.0)
    axs[0].axhspan(-2.5, 2.5, color="0.85", zorder=0, label="+-5 % band")
    axs[0].set_xlim(0, 6); axs[0].set_xlabel("time (s)"); axs[0].set_ylabel("axle offset (mm)")
    axs[0].set_title("PyBullet: 5 cm initial offset, v = 0.40 m/s"); axs[0].legend(ncol=2)
    for n in lin_cases:
        lg = logs[n]
        axs[1].plot(lg["t"], 1000 * lg["cte"], color=COLORS[n], label=f"{n} PyBullet")
        axs[1].plot(linear[n]["t"], 1000 * np.asarray(linear[n]["y"]), "--", color=COLORS[n], lw=1,
                    label=f"{n} linear model")
    axs[1].set_xlim(0, 4); axs[1].set_xlabel("time (s)"); axs[1].set_ylabel("axle offset (mm)")
    axs[1].set_title("Linear model vs physics simulation"); axs[1].legend()
    savefig(fig, "step_response")
    for n, m in metrics.items():
        print(f"{n:10s} tr={m['rise_time']:.2f} OS={m['overshoot']:.1f}% ts={m['settling_time']:.2f} "
              f"ss={m['ss_error']*1000:.2f}mm IAE={m['iae']*1000:.1f} mm.s peak_w={m['peak_w']:.2f}")


if __name__ == "__main__":
    main()
