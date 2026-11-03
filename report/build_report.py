"""Generate the project report PDF from the experiment results.

    python report/build_report.py   ->   report/Line_Following_AGV_Report.pdf

All numbers in the text and tables are read from results/data/*.json, so the
report always matches the latest experiment run.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from layout import ReportDoc  # noqa: E402

FIG_ORDER = ["system_block_diagram", "scene_robot_closeup", "track_maps", "software_architecture",
             "control_flowchart",
             "identification_step", "relay_limit_cycle", "kp_sweep", "step_response",
             "bode", "speed_damping", "tracks_xy", "tracks_error_profile", "speed_sweep", "noise",
             "robustness", "camera_pipeline"]
TAB_ORDER = ["writing", "literature", "syllabus", "robot", "plan", "platform",
             "ident", "gains", "kpsweep", "steps", "margins", "tracks", "speed", "noise", "robust", "camera",
             "contrib"]


class Numbers:
    """Single source of figure/table numbers (order of appearance in the report)."""

    def f(self, key):
        return FIG_ORDER.index(key) + 1

    def t(self, key):
        return TAB_ORDER.index(key) + 1


def load_data():
    d = os.path.join(ROOT, "results", "data")
    j = lambda n: json.load(open(os.path.join(d, n + ".json")))
    return dict(gains=j("gains"), ident=j("identification"), kp=j("kp_sweep"), steps=j("step_response"),
                freq=j("frequency"), tracks=j("tracks"), speed=j("speed_sweep"), noise=j("noise"),
                robust=j("robustness"), camera=j("camera_vs_ir"), history=j("tuning_history"),
                fig=lambda name: os.path.join(ROOT, "results", "figures", name + ".png"))


def story(D, T):
    import content_front as F
    import content_results as R
    import content_back as B
    s = []
    s += F.title_page(D) + F.declaration(D) + F.index_pages(D) + F.writing_allocation(D) + F.abstract(D)
    s += F.sec1(D) + F.sec2(D) + F.sec3(D) + F.sec4(D, T) + F.sec5(D) + F.sec6(D, T) + F.sec7(D, T) + F.sec8(D, T)
    s += R.sec9(D, T)
    s += B.sec10(D, T) + B.sec11(D) + B.sec12(D) + B.sec13(D) + B.appendix(D)
    return s


def main():
    D, T = load_data(), Numbers()
    out = os.path.join(HERE, "Line_Following_AGV_Report.pdf")
    doc = ReportDoc(out, title="Line-Following AGV with PID Tuning and Response Analysis",
                    author="AISR project group", subject="CSAI4014 AISR course project")
    doc.multiBuild(story(D, T))
    try:
        from pypdf import PdfReader
        n = len(PdfReader(out).pages)
        print(f"wrote {out} ({n} pages)")
    except Exception:
        print("wrote", out)


if __name__ == "__main__":
    main()
