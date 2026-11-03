"""Run the full experimental pipeline (E1 - E10) in dependency order.

    python experiments/run_all.py            # everything (~15-25 min on 8+ cores)
    python experiments/run_all.py 4 5        # only selected experiments
"""
import importlib
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ORDER = [
    ("1", "exp01_scene_identification"),
    ("2", "exp02_autotune"),          # writes gains.json used by E4-E10
    ("3", "exp03_kp_sweep"),
    ("4", "exp04_step_response"),
    ("5", "exp05_frequency"),
    ("6", "exp06_tracks"),
    ("7", "exp07_speed"),
    ("8", "exp08_noise"),
    ("9", "exp09_robustness"),
    ("10", "exp10_camera"),
]

if __name__ == "__main__":
    wanted = set(sys.argv[1:])
    t_all = time.time()
    for key, mod in ORDER:
        if wanted and key not in wanted:
            continue
        t0 = time.time()
        print(f"\n===== E{key}: {mod} =====", flush=True)
        importlib.import_module(mod).main()
        print(f"----- E{key} done in {time.time() - t0:.0f} s", flush=True)
    print(f"\nAll requested experiments finished in {time.time() - t_all:.0f} s")
