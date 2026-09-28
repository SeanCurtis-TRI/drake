#!/usr/bin/env python3
"""Campaign2 experiment 2: hero demo (dishrack teleop playback), barrier vs
volumetric hydroelastic.

WHAT IS SIMULATED
  //examples/hero_demo:convex_integrator_playback replays a recorded 100 s
  teleoperated dish-loading trajectory (Franka arm, dishrack, plates) with
  the bCENIC integrator, headless (--visualize=0).

  Two contact-representation modes:
  * "barrier"     — thin extruded barrier meshes: --margin=2e-4 --barrier=1e-4
                    (the campaign-1 hero settings).
  * "volumetric"  — --margin=0 --barrier=0 selects the volumetric
                    hydroelastic fallback (.vtk tet meshes with the
                    normalized extent field; no collision surface mesh).

PARAMETER SWEEP (18 configs)
  mode in {barrier, volumetric} x accuracy in {1e-1, 1e-2, 1e-3}
  x beta in {1, 0.1, 0.01}

HOW PARAMETERS ARE PASSED
  Plain command-line flags: --accuracy, --beta, --margin, --barrier,
  --sim_time=100, --max_step_size=0.01. lib/common.py appends
  --summary_file (always), --stats_file (heavy pass only; also the switch
  that enables collect_heavy_stats inside the driver), and --times_file
  (both passes; per-step (sim_time, wall_time) CSV used for the windowed
  real-time-ratio plots — quote RTR only from the timing pass).
  The html pass shortens to --sim_time=20 at --html_fps=4 (a full 100 s
  recording produces a >100 MB HTML file).

OUTPUTS PER RUN
  stdout.log ("JSON Statistics:" block), summary_demo.json (includes the
  integrator_stats dict and beta), times.csv, meta.json, DONE; heavy pass:
  steps.tsv.

Usage: run_hero.py [--pass heavy,timing,html] [--jobs N] [--only SUBSTR]
                   [--force] [--list] [--skip_build]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import common  # noqa: E402

EXPERIMENT = "hero"
MODES = {"barrier": (2e-4, 1e-4), "volumetric": (0.0, 0.0)}
SIM_TIME = 100.0
TIMEOUTS = {1e-1: 3600, 1e-2: 7200, 1e-3: 14400}
HTML_ID = "barrier-acc0.1-beta0.1"


def configs() -> list:
    runs = []
    for mode, (margin, barrier) in MODES.items():
        for acc in common.ACCURACIES:
            for beta in common.BETAS:
                cid = (f"{mode}-acc{common.slug(acc)}"
                       f"-beta{common.slug(beta)}")
                runs.append({
                    "experiment": EXPERIMENT,
                    "config_id": cid,
                    "kind": "hero",
                    "timeout": TIMEOUTS[acc],
                    "html": cid == HTML_ID,
                    "params": {"variant": mode, "accuracy": acc,
                               "beta": beta, "barrier": barrier,
                               "margin": margin},
                    "args": [
                        f"--accuracy={acc}", f"--beta={beta}",
                        f"--margin={margin}", f"--barrier={barrier}",
                        f"--sim_time={SIM_TIME}", "--max_step_size=0.01",
                    ],
                    # 100 s of recording is a >100 MB HTML; capture a 20 s
                    # excerpt at 4 fps for the visual sanity check instead.
                    "html_args": ["--sim_time=20", "--html_fps=4"],
                })
    return runs


if __name__ == "__main__":
    parser = common.standard_cli(argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter))
    sys.exit(common.drive(configs(), parser.parse_args()))
