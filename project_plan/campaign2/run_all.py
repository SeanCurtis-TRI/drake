#!/usr/bin/env python3
"""Campaign2 master runner: builds the binaries once, then drives all four
experiments (clutter, hero, nut_and_bolt, spiral) through the three passes
in the measurement-correct order, and finally runs the analyzer.

PASS ORDER AND PARALLELISM
  1. heavy  — collect_heavy_stats ON; per-step statistics. Runs with
              --jobs parallel workers (default 8): heavy numbers are not
              wall-clock-sensitive.
  2. timing — collect_heavy_stats OFF; the AUTHORITATIVE wall clock and
              phase timers. ALWAYS serial (jobs=1), no exceptions: parallel
              simulations would contend for cores and corrupt the timings.
              This is the long pole of the campaign (nut & bolt at
              accuracy 1e-3 alone can take hours).
  3. html   — one representative config per experiment re-run with meshcat
              recording; static HTML snapshots land in campaign2/html/.
  4. analyze — 9 cross-experiment tables (one per accuracy x beta cell),
              per-experiment CSVs, and quantity-vs-accuracy plots.

Every run is resumable: a completed run leaves a DONE marker and is skipped
on re-invocation, so a crashed/interrupted campaign is safe to restart with
the same command line. Old campaign-1 data under project_plan/data/ is
never touched; everything new lives under project_plan/campaign2/data/.

Usage examples:
  run_all.py                          # everything: heavy, timing, html, analyze
  run_all.py --passes heavy --jobs 16 # just the heavy pass, wide
  run_all.py --experiments clutter,spiral --passes timing
  run_all.py --passes analyze         # re-generate tables/plots only
"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import common  # noqa: E402

import run_clutter    # noqa: E402
import run_hero       # noqa: E402
import run_nut_and_bolt  # noqa: E402
import run_spiral     # noqa: E402

EXPERIMENTS = {
    "clutter": run_clutter.configs,
    "hero": run_hero.configs,
    "nut_and_bolt": run_nut_and_bolt.configs,
    "spiral": run_spiral.configs,
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--experiments", default=",".join(EXPERIMENTS),
                        help="Comma-separated subset of "
                             f"{','.join(EXPERIMENTS)}.")
    parser.add_argument("--passes", default="heavy,timing,html,analyze",
                        help="Comma-separated subset of "
                             "heavy,timing,html,analyze, executed in this "
                             "canonical order regardless of how they are "
                             "listed.")
    parser.add_argument("--jobs", type=int, default=8,
                        help="Workers for the heavy/html passes; timing is "
                             "always serial.")
    parser.add_argument("--force", action="store_true",
                        help="Ignore DONE markers and re-run everything.")
    parser.add_argument("--skip_build", action="store_true")
    args = parser.parse_args()

    names = [e.strip() for e in args.experiments.split(",") if e.strip()]
    for name in names:
        if name not in EXPERIMENTS:
            sys.exit(f"unknown experiment '{name}'")
    requested = {p.strip() for p in args.passes.split(",") if p.strip()}
    unknown = requested - {"heavy", "timing", "html", "analyze"}
    if unknown:
        sys.exit(f"unknown pass(es): {sorted(unknown)}")

    if not args.skip_build and requested & {"heavy", "timing", "html"}:
        common.bazel_build()

    failures = 0
    for pass_name in ("heavy", "timing", "html"):
        if pass_name not in requested:
            continue
        for name in names:
            configs = EXPERIMENTS[name]()
            runs = ([c for c in configs if c.get("html")]
                    if pass_name == "html" else configs)
            failures += common.run_pool(runs, pass_name, args.jobs,
                                        args.force)
        if pass_name == "html":
            html_configs = [c for name in names
                            for c in EXPERIMENTS[name]() if c.get("html")]
            common.collect_html(html_configs)

    if "analyze" in requested:
        # The analyzer needs pandas/matplotlib/numpy. If the interpreter
        # running this script lacks them (the stock system python has no
        # pandas), fall back to `uv run`, which provides them ephemerally.
        try:
            import pandas  # noqa: F401
            argv = [sys.executable, str(common.CAMPAIGN / "analyze.py")]
        except ImportError:
            argv = ["uv", "run", "--no-project", "--with", "pandas",
                    "--with", "matplotlib", "--with", "numpy",
                    "python3", str(common.CAMPAIGN / "analyze.py")]
        result = subprocess.run(argv)
        failures += result.returncode != 0

    if failures:
        print(f"\n{failures} failure(s); see {common.FAILURES_LOG}")
    else:
        print("\nAll requested passes completed.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
