#!/usr/bin/env python3
"""Campaign2 experiment 3: nut threading onto a bolt.

WHAT IS SIMULATED
  //examples/integrators:error_control_demo --example=nut_and_bolt: a nut,
  spun by a constant applied torque tau about the bolt axis, threads itself
  down a fixed bolt over 5 s of simulation. The tight helical thread
  clearance makes this the stress test for the thin-layer barrier contact
  representation, and the intended future comparison target against
  ABD/libuipc-style codimensional simulators.

  Physical/contact parameters are FIXED at the values verified to produce a
  successful threading run (the nut advances ~3 cm down the bolt;
  final z ~= -0.0298 m):
    --tau=-0.005      applied torque [N*m] (sign = threading direction)
    --barrier=2e-5    thin-layer barrier thickness [m]
    --margin=1e-5     collision margin [m]
    --E=1e8           hydroelastic modulus [Pa]
    --d=50            Hunt-Crossley-like dissipation
  SUCCESS CRITERION (checked by analyze.py from summary_demo.json's
  final_state): nut z-translation <= -0.029 m, i.e. the nut actually
  threaded all the way down rather than jamming.

PARAMETER SWEEP (9 configs)
  accuracy in {1e-1, 1e-2, 1e-3} x beta in {1, 0.1, 0.01}

HOW PARAMETERS ARE PASSED
  Plain command-line flags on the error_control_demo binary (see list
  above, plus --accuracy, --beta, --sim_time=5). lib/common.py appends
  --summary_file (always) and --stats_file (heavy pass only; this flag is
  also what enables collect_heavy_stats inside the driver). Timing pass
  omits --stats_file so the wall clock is measured un-instrumented.

OUTPUTS PER RUN
  stdout.log ("JSON Statistics:" block), summary_demo.json (final_state,
  integrator_stats), meta.json, DONE; heavy pass: steps.tsv.

TIMEOUTS: threading at accuracy 1e-3 previously took multiple hours; the
  per-accuracy timeouts below (1 h / 2 h / 6 h) reflect that.

Usage: run_nut_and_bolt.py [--pass heavy,timing,html] [--jobs N]
                           [--only SUBSTR] [--force] [--list] [--skip_build]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import common  # noqa: E402

EXPERIMENT = "nut_and_bolt"
SIM_TIME = 5.0
FIXED_ARGS = ["--example=nut_and_bolt", "--tau=-0.005", "--barrier=2e-5",
              "--margin=1e-5", "--E=1e8", "--d=50", f"--sim_time={SIM_TIME}"]
TIMEOUTS = {1e-1: 3600, 1e-2: 7200, 1e-3: 21600}
HTML_ID = "acc0.1-beta1"


def configs() -> list:
    runs = []
    for acc in common.ACCURACIES:
        for beta in common.BETAS:
            cid = f"acc{common.slug(acc)}-beta{common.slug(beta)}"
            runs.append({
                "experiment": EXPERIMENT,
                "config_id": cid,
                "kind": "demo",
                "timeout": TIMEOUTS[acc],
                "html": cid == HTML_ID,
                "params": {"variant": "nut_and_bolt", "accuracy": acc,
                           "beta": beta, "barrier": 2e-5, "margin": 1e-5},
                "args": FIXED_ARGS + [f"--accuracy={acc}", f"--beta={beta}"],
                "html_args": ["--html_fps=8"],
            })
    return runs


if __name__ == "__main__":
    parser = common.standard_cli(argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter))
    sys.exit(common.drive(configs(), parser.parse_args()))
