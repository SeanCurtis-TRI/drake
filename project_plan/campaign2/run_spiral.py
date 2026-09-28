#!/usr/bin/env python3
"""Campaign2 experiment 4: sphere rolling down a twisty (spiral) tube.

WHAT IS SIMULATED
  //examples/integrators:error_control_demo --example=sphere_and_spiral: a
  ball rolls down the inside of a helical tube mesh for 10 s. The tube is a
  thin shell, so the run only succeeds when the barrier layer is thin
  enough not to choke the tube bore — this experiment sweeps barrier/margin
  down to find that envelope while also sweeping accuracy x beta.

  SUCCESS CRITERION (checked by analyze.py from summary_demo.json's
  final_state): the ball reaches the bottom of the tube. In the verified
  reference run (margin=2e-5, barrier=2e-5, sim_time=10) the final ball
  height is z ~= 0.0345 m; we accept |z_final - 0.0345| < 5e-3. A ball
  stuck in the tube sits several cm higher.

PARAMETER SWEEP (36 configs)
  accuracy in {1e-1, 1e-2, 1e-3} x beta in {1, 0.1, 0.01}
  x (margin = barrier) in {1e-5, 2e-5, 5e-5, 1e-4}
  The margin/barrier pair is swept on the diagonal (equal values): the
  question is "how small must the layer be for the ball to make it down",
  not the full cross.

  Contact parameters otherwise fixed: --E=1e9 --d=10 (demo defaults for
  this example were verified in campaign 1; E5 data).

HOW PARAMETERS ARE PASSED
  Plain command-line flags on the error_control_demo binary:
  --example=sphere_and_spiral --sim_time=10 --E=1e9 --d=10 --accuracy
  --beta --margin --barrier. lib/common.py appends --summary_file (always)
  and --stats_file (heavy pass only, which is also the collect_heavy_stats
  switch inside the driver); the timing pass omits it.

OUTPUTS PER RUN
  stdout.log ("JSON Statistics:" block), summary_demo.json (final_state,
  integrator_stats), meta.json, DONE; heavy pass: steps.tsv.

Usage: run_spiral.py [--pass heavy,timing,html] [--jobs N] [--only SUBSTR]
                     [--force] [--list] [--skip_build]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import common  # noqa: E402

EXPERIMENT = "spiral"
SIM_TIME = 10.0
LAYERS = [1e-5, 2e-5, 5e-5, 1e-4]  # margin = barrier, swept together.
FIXED_ARGS = ["--example=sphere_and_spiral", f"--sim_time={SIM_TIME}",
              "--E=1e9", "--d=10"]
TIMEOUTS = {1e-1: 1800, 1e-2: 3600, 1e-3: 10800}
HTML_ID = "bm2e-05-acc0.1-beta0.1"


def configs() -> list:
    runs = []
    for layer in LAYERS:
        for acc in common.ACCURACIES:
            for beta in common.BETAS:
                cid = (f"bm{common.slug(layer)}-acc{common.slug(acc)}"
                       f"-beta{common.slug(beta)}")
                runs.append({
                    "experiment": EXPERIMENT,
                    "config_id": cid,
                    "kind": "demo",
                    "timeout": TIMEOUTS[acc],
                    "html": cid == HTML_ID,
                    "params": {"variant": "spiral", "accuracy": acc,
                               "beta": beta, "barrier": layer,
                               "margin": layer},
                    "args": FIXED_ARGS + [
                        f"--margin={layer}", f"--barrier={layer}",
                        f"--accuracy={acc}", f"--beta={beta}"],
                    "html_args": ["--html_fps=8"],
                })
    return runs


if __name__ == "__main__":
    parser = common.standard_cli(argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter))
    sys.exit(common.drive(configs(), parser.parse_args()))
