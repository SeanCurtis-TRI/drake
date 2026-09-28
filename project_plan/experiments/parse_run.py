"""Parsing utilities for campaign run directories.

A run directory (see run_all.py) contains meta.json, stdout.log, and usually
steps.tsv. This module turns those into structured records:

    load_run(run_dir) -> dict with:
        meta:       meta.json contents
        summary:    parsed stdout (wall clock, simulator statistics, the
                    PrintSimulatorStatistics JSON block, x_final) merged with
                    summary_demo.json when present
        steps:      pandas DataFrame of per-step CenicStepStatistics
        accepted:   DataFrame of accepted steps (see accepted_steps)
        times:      DataFrame of (sim_time, wall_time) samples recorded after
                    every accepted step (hero runs; empty otherwise)

The per-step TSV columns are defined by CenicStepStatistics::to_string()
(multibody/cenic/cenic_integrator.h).
"""

import gzip
import json
import re
from pathlib import Path

import pandas as pd

STEP_COLUMNS = [
    "step_type", "time", "step_size", "num_solver_iterations",
    "total_linesearch_iterations", "max_linesearch_iterations",
    "mean_linesearch_iterations", "max_condition_number",
    "last_condition_number", "max_e0", "mean_e0",
    "total_num_constraint_pairs",
]

# "Wall clock time: 12.3" (demo) or "AdvanceTo() time [sec]: 12.3" (clutter).
_WALL_RE = re.compile(
    r"(?:Wall clock time|AdvanceTo\(\) time \[sec\]):\s*([0-9.eE+-]+)")
_XFINAL_RE = re.compile(r"x_final:\s*([0-9.eE+-]+)")
# Lines like "Number of time steps taken (simulator stats) = 446".
_STAT_LINE_RE = re.compile(r"^(.*?)\s*=\s*([0-9.eE+-]+)\s*$")


def parse_stdout(path: Path) -> dict:
    """Extracts wall clock, x_final, the printed statistics lines, and the
    JSON statistics block from a run's stdout."""
    result = {"stat_lines": {}}
    if not path.exists():
        return result
    text = path.read_text(errors="replace")

    m = _WALL_RE.search(text)
    if m:
        result["wall_clock"] = float(m.group(1))
    m = _XFINAL_RE.search(text)
    if m:
        result["x_final"] = float(m.group(1))

    # The PrintSimulatorStatistics JSON block starts after "JSON Statistics:".
    marker = text.find("JSON Statistics:")
    if marker >= 0:
        brace = text.find("{", marker)
        if brace >= 0:
            depth = 0
            for i in range(brace, len(text)):
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                    if depth == 0:
                        try:
                            result["json_stats"] = json.loads(
                                text[brace:i + 1])
                        except json.JSONDecodeError:
                            pass
                        break

    # Human-readable statistic lines.
    for line in text.splitlines():
        m = _STAT_LINE_RE.match(line.strip())
        if m and any(key in m.group(1) for key in
                     ("Number of", "step size", "step taken",
                      "time step taken")):
            try:
                result["stat_lines"][m.group(1).strip()] = float(m.group(2))
            except ValueError:
                pass
    return result


def load_steps(path: Path) -> pd.DataFrame:
    """Loads a per-step TSV, tolerating both headered (demo) and headerless
    (clutter) files."""
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=STEP_COLUMNS)
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as f:
        first = f.readline()
    has_header = first.startswith("step_type\t")
    df = pd.read_csv(path, sep="\t",
                     header=0 if has_header else None,
                     names=None if has_header else STEP_COLUMNS)
    return df


def accepted_steps(steps: pd.DataFrame) -> pd.DataFrame:
    """Returns one row per accepted integrator step.

    DoStep logs full_step / half_step_1 / half_step_2 records. A step was
    accepted by DoStep iff its half_step_2 record exists (feasibility passed);
    error control may still reject it afterwards, in which case another
    full_step at the same start time follows. We therefore keep the *last*
    half_step_2 for each start time whose successor advances time.
    """
    if steps.empty:
        return steps.copy()
    h2 = steps[steps.step_type == "half_step_2"].copy()
    if h2.empty:
        return h2
    # half_step_2 rows are logged at t = t0 + h/2 with step_size = h/2. The
    # accepted step covers [t0, t0 + h] with h = 2 * step_size.
    h2["t0"] = h2["time"] - h2["step_size"]
    h2["h"] = 2.0 * h2["step_size"]
    # Keep the last attempt for each distinct t0 (earlier ones were rejected
    # by error control and retried with smaller h).
    h2 = h2.groupby("t0", as_index=False).last()
    # A start time t0 was truly accepted iff some later record starts past t0.
    # The final record is always accepted (simulation ended there).
    h2 = h2.sort_values("t0").reset_index(drop=True)
    keep = [True] * len(h2)
    for i in range(len(h2) - 1):
        keep[i] = h2.loc[i + 1, "t0"] >= h2.loc[i, "t0"] + 0.5 * h2.loc[i, "h"] - 1e-12
    return h2[keep].reset_index(drop=True)


def load_times(path: Path) -> pd.DataFrame:
    """Loads a (sim_time, wall_time) csv written by the hero demo's monitor
    (one row per accepted step; wall_time is time.monotonic() seconds)."""
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=["sim_time", "wall_time"])
    return pd.read_csv(path)


def load_run(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    record = {"dir": run_dir, "ok": (run_dir / "DONE").exists()}
    meta_path = run_dir / "meta.json"
    record["meta"] = (json.loads(meta_path.read_text())
                      if meta_path.exists() else {})
    summary = parse_stdout(run_dir / "stdout.log")
    demo_summary_path = run_dir / "summary_demo.json"
    if demo_summary_path.exists():
        try:
            summary.update(json.loads(demo_summary_path.read_text()))
        except json.JSONDecodeError:
            pass
    record["summary"] = summary
    # Large campaigns store the per-step TSV gzipped; pandas reads either.
    steps_path = run_dir / "steps.tsv"
    if not steps_path.exists() and (run_dir / "steps.tsv.gz").exists():
        steps_path = run_dir / "steps.tsv.gz"
    record["steps"] = load_steps(steps_path)
    record["accepted"] = accepted_steps(record["steps"])
    record["times"] = load_times(run_dir / "times.csv")
    return record


def load_experiment(data_dir: Path, experiment: str) -> list:
    """Loads all run records for one experiment, sorted by run_id."""
    exp_dir = Path(data_dir) / experiment
    if not exp_dir.exists():
        return []
    return [load_run(d) for d in sorted(exp_dir.iterdir()) if d.is_dir()]
