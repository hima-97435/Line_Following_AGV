"""E2: PID tuning.

1. Relay auto-tuning (Astrom-Hagglund) on the straight tape -> Ku, Tu
2. Classic rules: Ziegler-Nichols (PID-ZN) and Tyreus-Luyben (PID-TL)
3. Nelder-Mead optimisation of PID gains and of the fuzzy scaling factors on
   the training scenarios (oval lap + 5 cm step), from a neutral start point.
The resulting gains are stored in results/data/gains.json for later experiments.
"""
from common import *  # noqa: F401,F403
from agv.tuning import relay_test, classic_rules, optimise, evaluate_gains


def main():
    base = SimConfig()
    relay_runs, relay_logs = [], {}
    for d in (0.5, 1.0, 2.0):
        res, log = relay_test(base, amplitude=d)
        relay_runs.append(res); relay_logs[d] = log
        print("relay", res)
    chosen = next(r for r in relay_runs if r["relay"] == 1.0)
    rules = classic_rules(chosen["Ku"], chosen["Tu"])

    # --- relay limit-cycle figure
    log = relay_logs[1.0]
    fig, ax1 = plt.subplots(figsize=(6.2, 2.8))
    m = (log["t"] > 4.0) & (log["t"] < 6.5)
    ax1.plot(log["t"][m], 1000 * log["e_meas"][m], color="tab:blue", label="measured line error e (mm)")
    ax1.set_xlabel("time (s)"); ax1.set_ylabel("e (mm)", color="tab:blue")
    ax2 = ax1.twinx()
    ax2.step(log["t"][m], log["w_cmd"][m], where="post", color="tab:red", lw=0.8, label="relay output")
    ax2.set_ylabel("yaw-rate command (rad/s)", color="tab:red"); ax2.grid(False)
    ax1.set_title(f"Relay test: d = 1.0 rad/s, a = {chosen['amplitude']*1000:.1f} mm, "
                  f"Tu = {chosen['Tu']:.3f} s, Ku = {chosen['Ku']:.0f}")
    savefig(fig, "relay_limit_cycle")

    # --- Nelder-Mead PID optimisation (neutral start, log-space search)
    pid_params = lambda g: dict(control__controller="pid", control__kp=g["kp"], control__ki=g["ki"],
                                control__kd=g["kd"])
    best_pid, J_pid, hist_pid = optimise(base, dict(kp=10.0, ki=1.0, kd=0.3), pid_params, maxfev=200)
    print("NM PID", best_pid, J_pid)

    fz_params = lambda g: dict(control__controller="fuzzy", control__fuzzy_e_scale=g["fuzzy_e_scale"],
                               control__fuzzy_de_scale=g["fuzzy_de_scale"])
    best_fz, J_fz, hist_fz = optimise(base, dict(fuzzy_e_scale=0.05, fuzzy_de_scale=0.30), fz_params, maxfev=100)
    print("NM fuzzy", best_fz, J_fz)

    gains = {
        "PID-ZN": rules["ZN"], "PID-TL": rules["TL"], "PID-ZN-NO": rules["ZN-NO"],
        "PID-NM": best_pid, "Fuzzy-NM": best_fz,
        "relay": chosen, "relay_runs": relay_runs,
        "cost": {"PID-NM": J_pid, "Fuzzy-NM": J_fz},
        "theory_kp_star": 2 * base.control.base_speed / base.sensor.lookahead ** 2,
    }
    # cost of every rule-based / analytical controller on the same training scenarios
    for name, ov in controller_set(gains).items():
        gains["cost"][name] = evaluate_gains(base, ov)[0]
    save_json("gains", gains)
    save_json("tuning_history", {"PID-NM": hist_pid, "Fuzzy-NM": hist_fz})

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.8))
    for ax, hist, title in ((axs[0], hist_pid, "PID gains (kp, ki, kd)"), (axs[1], hist_fz, "Fuzzy scaling factors")):
        c = np.array([h["cost"] for h in hist])
        ax.semilogy(c, ".", ms=3, color="0.6", label="evaluated")
        ax.semilogy(np.minimum.accumulate(c), color="tab:blue", label="best so far")
        ax.set_xlabel("simulation evaluation"); ax.set_ylabel("tuning cost J"); ax.set_title(title)
        ax.legend()
    savefig(fig, "tuning_convergence")
    print({k: v for k, v in gains.items() if k != "relay_runs"})


if __name__ == "__main__":
    main()
