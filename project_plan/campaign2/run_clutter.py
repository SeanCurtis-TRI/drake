#!/usr/bin/env python3
"""Campaign2 experiment 1: clutter piles (Thingi10K meshes + primitives).

WHAT IS SIMULATED
  The //examples/multibody/clutter:clutter C++ demo drops 4 piles of 5
  objects each into a sink (`add_sink_walls: true`) and simulates 3.0 s of
  settling with the bCENIC error-controlled convex integrator.

  Two geometry variants:
  * "mesh20"  — 20 curated Thingi10K meshes (all distinct: object i of pile
    p uses mesh_files[(p*5+i) % 20]). The set was drawn with
      python3 project_plan/experiments/thingi10k_meshes.py \\
        --out project_plan/campaign2/meshes/thingi10k_lowpoly20 \\
        --seed 1 --count 20 --min-faces 100 --max-faces 300 \\
        --target-size 0.10 --min-extent 0.002
    i.e. low-poly (100..300 faces), normalized to a 10 cm bounding box,
    "wild" thin shapes kept (min extent down to 2.5 mm) but degenerate
    sub-2 mm sheets rejected. Pinned ids live in meshes/.../manifest.json.
  * "prim"    — the demo's primitive scene with `enable_boxes: true`
    (alternating spheres and boxes, same 4x5 pile layout).

PARAMETER SWEEP (36 configs per variant, 72 total)
  accuracy in {1e-1, 1e-2, 1e-3}  x  beta in {1, 0.1, 0.01}
  x  barrier in {1e-4, 1e-3}      x  margin in {1e-4, 1e-3}

HOW PARAMETERS ARE PASSED
  Everything goes through a generated config.yaml (written into the run
  directory) handed to the binary as --config=<path>:
    accuracy               -> config.simulator_config.accuracy
    beta                   -> config.icf_solver_config.beta
    collect_heavy_stats    -> config.icf_solver_config.collect_heavy_stats
                              (True on the heavy pass, False on the timing
                              pass — see lib/common.py)
    use_toi: true          -> config.icf_solver_config.use_toi
                              (CCD on; without it thin meshes tunnel the
                              barrier layer and the run aborts — the E3e
                              precedent)
    margin, barrier        -> config.scene_graph_config
                                .default_proximity_properties.{margin,barrier}
    variant scene          -> config.clutter_config.{enable_boxes,mesh_files}
  The heavy pass adds --log_file=<run_dir>/steps.tsv; the html pass keeps
  the visualization_config stanza and adds --html_file=<run_dir>/scene.html.

OUTPUTS PER RUN
  stdout.log ("JSON Statistics:" block with all cenic_* counters/timers),
  stderr.log, config.yaml, meta.json, DONE; heavy pass: steps.tsv.

Usage: run_clutter.py [--pass heavy,timing,html] [--jobs N] [--only SUBSTR]
                      [--force] [--list] [--skip_build]
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import common  # noqa: E402

EXPERIMENT = "clutter"
BARRIERS = [1e-4, 1e-3]
MARGINS = [1e-4, 1e-3]
MESH_DIR = common.CAMPAIGN / "meshes" / "thingi10k_lowpoly20"

# Wall-clock guardrails, keyed by accuracy (tight accuracy => more steps).
TIMEOUTS = {1e-1: 1800, 1e-2: 1800, 1e-3: 7200}

# Representative config captured as a static HTML for visual sanity checks:
# the mesh variant at loose accuracy (fast), nominal beta, thick barrier.
HTML_ID = "mesh20-bar0.001-mar0.0001-acc0.1-beta0.1"


def mesh_files() -> list:
    manifest = json.load(open(MESH_DIR / "manifest.json"))
    return [str(MESH_DIR / row["obj"]) for row in manifest["meshes"]]


def configs() -> list:
    meshes = mesh_files()
    assert len(meshes) == 20, f"expected 20 curated meshes, {len(meshes)}"
    runs = []
    for variant in ("mesh20", "prim"):
        scene = ({"mesh_files": meshes, "mesh_scale": 1.0}
                 if variant == "mesh20" else {"enable_boxes": True})
        for barrier in BARRIERS:
            for margin in MARGINS:
                for acc in common.ACCURACIES:
                    for beta in common.BETAS:
                        cid = (f"{variant}-bar{common.slug(barrier)}"
                               f"-mar{common.slug(margin)}"
                               f"-acc{common.slug(acc)}"
                               f"-beta{common.slug(beta)}")
                        runs.append({
                            "experiment": EXPERIMENT,
                            "config_id": cid,
                            "kind": "clutter",
                            "timeout": TIMEOUTS[acc],
                            "html": cid == HTML_ID,
                            "params": {
                                "variant": variant, "accuracy": acc,
                                "beta": beta, "barrier": barrier,
                                "margin": margin,
                            },
                            "overrides": {
                                "simulator_config": {"accuracy": acc},
                                "icf_solver_config": {"beta": beta,
                                                      "use_toi": True},
                                "scene_graph_config": {
                                    "default_proximity_properties": {
                                        "margin": margin,
                                        "barrier": barrier,
                                    }},
                                "clutter_config": scene,
                            },
                        })
    return runs


if __name__ == "__main__":
    parser = common.standard_cli(argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter))
    sys.exit(common.drive(configs(), parser.parse_args()))
