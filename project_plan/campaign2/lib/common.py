"""Shared run machinery for the campaign2 experiment re-run.

Forked from project_plan/experiments/run_all.py (which stays untouched so the
original campaign remains reproducible). Key differences:

- **Two-pass protocol.** Every configuration is executed twice:
  * pass "heavy": `collect_heavy_stats` ON -> per-step statistics
    (steps.tsv: condition numbers, contact counts, e0, ...). May run with
    many parallel workers.
  * pass "timing": `collect_heavy_stats` OFF -> the AUTHORITATIVE wall clock
    and phase timers. Heavy statistics (the dense rcond per Newton iteration
    and per-step logging) would otherwise contaminate the timing, which is
    exactly what happened to the original campaign data. Timing runs are
    FORCED to jobs=1 (serial) so host contention cannot skew them.
  A third pass "html" runs one representative configuration per experiment
  with --html_file to capture a static meshcat snapshot for visual checking.

- **Per-pass data roots.** data/heavy/<exp>/<config_id>/,
  data/timing/<exp>/<config_id>/, data/html/<exp>/<config_id>/ — identical
  layout below each root, so project_plan/experiments/parse_run.py's
  load_experiment(data_dir, exp) works unchanged against any root, and the
  heavy<->timing join is a config_id dict lookup.

- **collect_heavy_stats is per-pass** (the old campaign force-enabled it for
  all clutter runs).

- meta.json additionally records "pass", "config_id", and the experiment
  parameters ("params": accuracy, beta, margin, barrier, variant, ...) so the
  analyzer never has to parse run-id strings.

How parameters reach the binaries:

- kind "demo"  (bazel-bin/examples/integrators/error_control_demo):
  everything is a command-line flag (--example, --accuracy, --beta,
  --margin, --barrier, --E, --d, --tau, --use_toi, --sim_time). The heavy
  pass adds --stats_file (which is ALSO what turns collect_heavy_stats on
  inside the driver); the timing pass omits it. --summary_file always.
- kind "clutter" (bazel-bin/examples/multibody/clutter/clutter):
  everything travels in a generated config.yaml: accuracy ->
  simulator_config.accuracy, beta/use_toi/collect_heavy_stats ->
  icf_solver_config.*, margin/barrier ->
  scene_graph_config.default_proximity_properties.*, scene shape ->
  clutter_config.* (num_piles, objects_per_pile, enable_boxes, mesh_files).
  The visualization_config stanza is removed for headless passes and kept
  for the html pass (meshcat only exists when it is present).
- kind "hero"  (bazel-bin/examples/hero_demo/convex_integrator_playback):
  flags --accuracy, --beta, --margin, --barrier, --sim_time; barrier=0 &
  margin=0 selects the volumetric-hydroelastic fallback. --times_file is
  recorded in BOTH passes (a per-step Python list append; the RTR plots read
  the timing pass). --stats_file heavy-only; --summary_file always.

Resumability: a DONE marker is written on exit code 0; completed runs are
skipped unless --force. Failures are appended to data/failures.log.
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

CAMPAIGN = Path(__file__).resolve().parents[1]   # .../project_plan/campaign2
REPO = CAMPAIGN.parents[1]                        # repo root
# CAMPAIGN2_OUT_ROOT redirects all outputs (data/, html/) elsewhere; used by
# smoke tests so trial runs never pollute the real campaign data tree.
OUT_ROOT = Path(os.environ.get("CAMPAIGN2_OUT_ROOT") or CAMPAIGN)
DATA = OUT_ROOT / "data"
HTML_DIR = OUT_ROOT / "html"
FAILURES_LOG = DATA / "failures.log"

BINARIES = {
    "demo": REPO / "bazel-bin/examples/integrators/error_control_demo",
    "clutter": REPO / "bazel-bin/examples/multibody/clutter/clutter",
    "hero": REPO / "bazel-bin/examples/hero_demo/convex_integrator_playback",
}
BUILD_TARGETS = [
    "//examples/integrators:error_control_demo",
    "//examples/multibody/clutter:clutter",
    "//examples/hero_demo:convex_integrator_playback",
]
CLUTTER_BASE_CONFIG = REPO / "examples/multibody/clutter/config.yaml"

# The reduced sweep shared by all campaign2 experiments.
ACCURACIES = [1e-1, 1e-2, 1e-3]
BETAS = [1.0, 0.1, 0.01]

PASSES = ("heavy", "timing", "html")


def slug(value) -> str:
    """Formats a value as a compact id token (1e-3 -> '0.001', bool -> on)."""
    if isinstance(value, bool):
        return "on" if value else "off"
    if isinstance(value, int):
        return str(value)
    return f"{value:g}"


def deep_merge(base: dict, overrides: dict) -> dict:
    """Recursively merges `overrides` into `base` (returns `base`)."""
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
            text=True, check=True).stdout.strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def bazel_build() -> None:
    print(f"bazel build {' '.join(BUILD_TARGETS)}")
    subprocess.run(["bazel", "build"] + BUILD_TARGETS, cwd=REPO, check=True)


def build_argv(run: dict, run_dir: Path, pass_name: str) -> list:
    """Assembles the argv for one run in one pass; see the module docstring
    for the per-kind parameter plumbing."""
    kind = run["kind"]
    binary = str(BINARIES[kind])
    heavy = pass_name == "heavy"
    html = pass_name == "html"

    if kind in ("demo", "hero"):
        argv = [binary]
        if kind == "hero":
            argv.append("--visualize=0")
        argv += list(run.get("args", []))
        if html:
            argv += run.get("html_args", [])
            argv += [f"--html_file={run_dir}/scene.html"]
        argv += [f"--summary_file={run_dir}/summary_demo.json"]
        if heavy:
            argv += [f"--stats_file={run_dir}/steps.tsv"]
        if kind == "hero":
            argv += [f"--times_file={run_dir}/times.csv"]
        return argv

    assert kind == "clutter", kind
    with open(CLUTTER_BASE_CONFIG) as f:
        root = yaml.safe_load(f)
    cfg = root["config"]
    if not html:
        # Headless: meshcat is created iff this stanza exists.
        cfg.pop("visualization_config", None)
    # Heavy per-step statistics only in the heavy pass; the timing pass
    # measures the un-instrumented wall clock.
    cfg["icf_solver_config"]["collect_heavy_stats"] = heavy
    deep_merge(cfg, run.get("overrides", {}))
    config_path = run_dir / "config.yaml"
    with open(config_path, "w") as f:
        yaml.safe_dump(root, f, sort_keys=False)
    argv = [binary, f"--config={config_path}"]
    if heavy:
        argv += [f"--log_file={run_dir}/steps.tsv"]
    if html:
        argv += [f"--html_file={run_dir}/scene.html"]
    return argv


def execute(run: dict, pass_name: str, force: bool = False) -> dict:
    """Executes one run in one pass; resumable via a DONE marker."""
    run_dir = DATA / pass_name / run["experiment"] / run["config_id"]
    done = run_dir / "DONE"
    if done.exists() and not force:
        return {"run": run, "pass": pass_name, "skipped": True,
                "exit_code": 0}
    run_dir.mkdir(parents=True, exist_ok=True)
    done.unlink(missing_ok=True)

    argv = [str(a) for a in build_argv(run, run_dir, pass_name)]
    meta = {
        "campaign": "campaign2",
        "experiment": run["experiment"],
        "config_id": run["config_id"],
        "pass": pass_name,
        "kind": run["kind"],
        "params": run.get("params", {}),
        "argv": argv,
        "overrides": run.get("overrides", {}),
        "args": run.get("args", []),
        "timeout": run.get("timeout", 3600),
        "git_sha": git_sha(),
        "host": socket.gethostname(),
        "start_time": datetime.now(timezone.utc).isoformat(),
    }

    env = dict(os.environ, MPLBACKEND="Agg")
    start = time.monotonic()
    timed_out = False
    try:
        with open(run_dir / "stdout.log", "w") as out, \
             open(run_dir / "stderr.log", "w") as err:
            proc = subprocess.run(argv, cwd=run_dir, stdin=subprocess.DEVNULL,
                                  stdout=out, stderr=err, env=env,
                                  timeout=meta["timeout"])
        exit_code = proc.returncode
    except subprocess.TimeoutExpired:
        exit_code, timed_out = -1, True
    except Exception as e:  # noqa: BLE001
        exit_code = -2
        (run_dir / "stderr.log").open("a").write(f"\ndriver exception: {e}\n")

    meta.update(end_time=datetime.now(timezone.utc).isoformat(),
                wall_seconds=time.monotonic() - start,
                exit_code=exit_code, timed_out=timed_out)
    with open(run_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    if exit_code == 0:
        done.touch()
    else:
        FAILURES_LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(FAILURES_LOG, "a") as f:
            f.write(f"{pass_name}/{run['experiment']}/{run['config_id']}: "
                    f"exit={exit_code} timed_out={timed_out}\n")
    return {"run": run, "pass": pass_name, "skipped": False,
            "exit_code": exit_code}


def run_pool(runs: list, pass_name: str, jobs: int, force: bool) -> int:
    """Runs a list of run dicts in one pass. Timing is forced serial."""
    if pass_name == "timing" and jobs != 1:
        print("NOTE: timing pass is forced to --jobs=1 (serial) so host "
              "contention cannot contaminate the wall-clock measurements.")
        jobs = 1
    failures = 0
    total = len(runs)
    print(f"[{pass_name}] {total} runs at jobs={jobs}")
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(execute, run, pass_name, force): run
                   for run in runs}
        for i, future in enumerate(as_completed(futures), 1):
            result = future.result()
            run = result["run"]
            state = ("skip" if result["skipped"] else
                     "ok" if result["exit_code"] == 0 else
                     f"FAIL({result['exit_code']})")
            print(f"[{pass_name} {i}/{total}] "
                  f"{run['experiment']}/{run['config_id']}: {state}",
                  flush=True)
            if not result["skipped"] and result["exit_code"] != 0:
                failures += 1
    return failures


def standard_cli(parser):
    """Adds the CLI flags shared by every per-experiment runner."""
    parser.add_argument("--pass", dest="passes", default="heavy,timing",
                        help="Comma-separated subset of heavy,timing,html "
                             "(default: heavy,timing).")
    parser.add_argument("--jobs", type=int, default=8,
                        help="Parallel workers for the heavy/html passes; the "
                             "timing pass always runs serially.")
    parser.add_argument("--only", default="",
                        help="Substring filter on config ids.")
    parser.add_argument("--force", action="store_true",
                        help="Re-run even when a DONE marker exists.")
    parser.add_argument("--list", action="store_true",
                        help="Print the run matrix and exit.")
    parser.add_argument("--skip_build", action="store_true",
                        help="Skip the bazel build step.")
    return parser


def drive(configs: list, args) -> int:
    """Standard main() body for a per-experiment runner: filter, list or
    build+run the requested passes over `configs`."""
    if args.only:
        configs = [c for c in configs if args.only in c["config_id"]]
    passes = [p.strip() for p in args.passes.split(",") if p.strip()]
    for p in passes:
        if p not in PASSES:
            sys.exit(f"unknown pass '{p}' (choose from {PASSES})")
    if args.list:
        for c in configs:
            marks = " [html]" if c.get("html") else ""
            print(f"{c['experiment']}/{c['config_id']} "
                  f"kind={c['kind']} timeout={c.get('timeout')}s{marks}")
        print(f"{len(configs)} configs; passes: {passes}")
        return 0
    if not args.skip_build:
        bazel_build()
    failures = 0
    for p in passes:
        runs = [c for c in configs if c.get("html")] if p == "html" \
            else configs
        failures += run_pool(runs, p, args.jobs, args.force)
    if "html" in passes:
        collect_html([c for c in configs if c.get("html")])
    return failures


def collect_html(html_configs: list) -> None:
    """Copies each html-pass scene.html to campaign2/html/<experiment>.html."""
    HTML_DIR.mkdir(parents=True, exist_ok=True)
    for c in html_configs:
        src = DATA / "html" / c["experiment"] / c["config_id"] / "scene.html"
        if src.exists():
            dest = HTML_DIR / f"{c['experiment']}.html"
            dest.write_bytes(src.read_bytes())
            print(f"HTML: {dest} ({dest.stat().st_size // 1024} KiB)")
        else:
            print(f"HTML MISSING for {c['experiment']}/{c['config_id']}")
