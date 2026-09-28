#!/usr/bin/env python3
"""Campaign driver for the thin-objects (bCENIC) paper experiments.

Runs the manifest defined in manifest.py with a worker pool, per-run
timeouts, and skip-if-done resumability. Failures never stop the campaign.

Usage (from anywhere; paths are repo-relative internally):
    python3 project_plan/experiments/run_all.py                # whole campaign
    python3 project_plan/experiments/run_all.py --tier A       # one tier
    python3 project_plan/experiments/run_all.py --experiment E4_nut_and_bolt
    python3 project_plan/experiments/run_all.py --only E4_nut_and_bolt/acc1e-2_beta1
    python3 project_plan/experiments/run_all.py --list         # show runs
    python3 project_plan/experiments/run_all.py --force ...    # ignore DONE

Per-run output layout (see project_plan/README.md):
    project_plan/data/<experiment>/<run_id>/
        meta.json    argv, params, git sha, timing, exit code
        config.yaml  clutter runs only: the exact config used
        steps.tsv    per-step CenicStepStatistics (TSV)
        times.csv    hero runs only: (sim_time, wall_time) per accepted step
        stdout.log / stderr.log
        DONE         marker written only on exit code 0
"""

import argparse
import copy
import datetime
import json
import os
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from manifest import build_manifest  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
DATA_DIR = REPO / "project_plan" / "data"
FAILURES_LOG = DATA_DIR / "failures.log"

BINARIES = {
    "demo": REPO / "bazel-bin/examples/integrators/error_control_demo",
    "clutter": REPO / "bazel-bin/examples/multibody/clutter/clutter",
    "ccd": REPO / "bazel-bin/examples/integrators/rotational_ccd_study",
    "hero": REPO / "bazel-bin/examples/hero_demo/convex_integrator_playback",
}
BUILD_TARGETS = [
    "//examples/integrators:error_control_demo",
    "//examples/multibody/clutter:clutter",
    "//examples/integrators:rotational_ccd_study",
    "//examples/hero_demo:convex_integrator_playback",
]
CLUTTER_BASE_CONFIG = REPO / "examples/multibody/clutter/config.yaml"

_print_lock = threading.Lock()


def log(msg: str) -> None:
    with _print_lock:
        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {msg}",
              flush=True)


def deep_merge(base: dict, overrides: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
            text=True, timeout=10).stdout.strip()
    except Exception:
        return "unknown"


def build_argv(run: dict, run_dir: Path) -> list:
    kind = run["kind"]
    binary = str(BINARIES[kind])
    if kind == "demo":
        return [binary] + run["args"] + [
            f"--stats_file={run_dir / 'steps.tsv'}",
            f"--summary_file={run_dir / 'summary_demo.json'}",
        ]
    if kind == "clutter":
        # Generate the per-run config: packaged defaults + overrides,
        # headless, heavy stats on.
        with open(CLUTTER_BASE_CONFIG) as f:
            cfg = yaml.safe_load(f)["config"]
        cfg.pop("visualization_config", None)
        cfg["icf_solver_config"]["collect_heavy_stats"] = True
        cfg = deep_merge(cfg, run.get("overrides", {}))
        config_path = run_dir / "config.yaml"
        with open(config_path, "w") as f:
            yaml.safe_dump({"config": cfg}, f)
        return [binary, f"--config={config_path}",
                f"--log_file={run_dir / 'steps.tsv'}"]
    if kind == "ccd":
        return [binary] + run["args"] + [
            f"--grid_output={run_dir / 'ccd_grid.tsv'}",
            f"--cost_output={run_dir / 'ccd_cost.tsv'}",
        ]
    if kind == "hero":
        # First-ever run needs network to fetch the lbm_eval model wheel;
        # it is cached under ~/.cache/drake/package_map afterwards.
        return [binary, "--visualize=0"] + run["args"] + [
            f"--stats_file={run_dir / 'steps.tsv'}",
            f"--times_file={run_dir / 'times.csv'}",
            f"--summary_file={run_dir / 'summary_demo.json'}",
        ]
    raise ValueError(f"unknown kind {kind}")


def execute(run: dict, force: bool) -> str:
    run_dir = DATA_DIR / run["experiment"] / run["run_id"]
    done_marker = run_dir / "DONE"
    if done_marker.exists() and not force:
        return "skipped"
    run_dir.mkdir(parents=True, exist_ok=True)
    done_marker.unlink(missing_ok=True)

    argv = build_argv(run, run_dir)
    meta = {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "kind": run["kind"],
        "tier": run["tier"],
        "argv": argv,
        "overrides": run.get("overrides"),
        "args": run.get("args"),
        "git_sha": git_sha(),
        "host": os.uname().nodename,
        "start_time": datetime.datetime.now().isoformat(),
        "timeout": run["timeout"],
    }

    env = dict(os.environ, MPLBACKEND="Agg")
    start = datetime.datetime.now()
    timed_out = False
    try:
        with open(run_dir / "stdout.log", "w") as out, \
                open(run_dir / "stderr.log", "w") as err:
            proc = subprocess.run(
                argv, stdout=out, stderr=err, stdin=subprocess.DEVNULL,
                cwd=run_dir, env=env, timeout=run["timeout"])
        exit_code = proc.returncode
    except subprocess.TimeoutExpired:
        exit_code = -1
        timed_out = True
    except Exception as e:  # noqa: BLE001 - record and continue
        exit_code = -2
        with open(run_dir / "stderr.log", "a") as err:
            err.write(f"\ndriver exception: {e}\n")

    meta["end_time"] = datetime.datetime.now().isoformat()
    meta["wall_seconds"] = (datetime.datetime.now() - start).total_seconds()
    meta["exit_code"] = exit_code
    meta["timed_out"] = timed_out
    with open(run_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    if exit_code == 0:
        done_marker.touch()
        return "ok"
    with _print_lock, open(FAILURES_LOG, "a") as f:
        f.write(f"{meta['end_time']} {run['experiment']}/{run['run_id']} "
                f"exit={exit_code} timed_out={timed_out}\n")
    return "timeout" if timed_out else "failed"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tier", choices=["A", "B", "C"], default=None,
                        help="Run only this tier.")
    parser.add_argument("--experiment", default=None,
                        help="Run only this experiment key.")
    parser.add_argument("--only", default=None,
                        help="Run only <experiment>/<run_id>.")
    parser.add_argument("--jobs", type=int, default=16,
                        help="Parallel workers (default 16).")
    parser.add_argument("--force", action="store_true",
                        help="Re-run even when DONE marker exists.")
    parser.add_argument("--list", action="store_true",
                        help="List selected runs and exit.")
    parser.add_argument("--skip_build", action="store_true",
                        help="Skip the initial bazel build.")
    args = parser.parse_args()

    runs = build_manifest()
    if args.tier:
        runs = [r for r in runs if r["tier"] == args.tier]
    if args.experiment:
        runs = [r for r in runs if r["experiment"] == args.experiment]
    if args.only:
        exp, _, rid = args.only.partition("/")
        runs = [r for r in runs
                if r["experiment"] == exp and (not rid or r["run_id"] == rid)]
    if not runs:
        print("No runs selected.")
        return

    if args.list:
        for r in runs:
            print(f"{r['tier']}  {r['experiment']}/{r['run_id']}")
        print(f"total: {len(runs)}")
        return

    if not args.skip_build:
        log(f"bazel build {' '.join(BUILD_TARGETS)} ...")
        subprocess.run(["bazel", "build"] + BUILD_TARGETS, cwd=REPO,
                       check=True)

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    # Tier ordering: A first, then B, then C. Within a tier keep manifest
    # order.
    tier_order = {"A": 0, "B": 1, "C": 2}
    runs.sort(key=lambda r: tier_order[r["tier"]])

    counts = {"ok": 0, "skipped": 0, "failed": 0, "timeout": 0}
    log(f"campaign: {len(runs)} runs on {args.jobs} workers")
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(execute, r, args.force): r for r in runs}
        for future in as_completed(futures):
            r = futures[future]
            status = future.result()
            counts[status] += 1
            done = sum(counts.values())
            log(f"[{done}/{len(runs)}] {status:8s} "
                f"{r['experiment']}/{r['run_id']}")
    log(f"campaign complete: {counts}")


if __name__ == "__main__":
    main()
