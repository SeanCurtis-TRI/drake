#!/usr/bin/env python3
"""Campaign2 analyzer: cross-experiment tables, per-experiment CSVs, and
quantity-vs-accuracy plots.

INPUTS
  data/timing/<exp>/<config_id>/   authoritative wall clock + phase timers
                                   (collect_heavy_stats OFF)
  data/heavy/<exp>/<config_id>/    per-step statistics (heavy pass); used
                                   here only as a fallback for scene-size
                                   stats when a timing run failed.
  Parsed with project_plan/experiments/parse_run.py (load_experiment); the
  per-run statistics come from the "JSON Statistics:" stdout block (clutter)
  or summary_demo.json's integrator_stats (demo/hero) — same key set either
  way, emitted by CenicIntegrator::DoGetStatisticsSummary.

OUTPUTS
  report/REPORT.md          9 cross-experiment tables (one per acc x beta
                            cell) with every experiment/variant as a row,
                            plus a failure/timeout appendix.
  report/all_runs.csv       every config as one row (both passes' status).
  report/<exp>.csv          per-experiment slice, same columns.
  plots/clutter_<variant>_<quantity>.png   2x2 (barrier x margin) panels,
                            x = accuracy (log), one line per beta.
  plots/hero_<quantity>.png x = accuracy (log), solid lines = barrier mode,
                            dashed = volumetric, one color per beta.
  plots/hero_rtr_acc*.png   windowed mean real-time-ratio vs sim time from
                            the timing pass times.csv (window: 2 s sim
                            time), 6 lines (mode x beta).

SUCCESS CRITERIA (columns in the tables)
  nut_and_bolt: threaded iff final nut height z <= -0.029 m
                (summary_demo.json final_state[6]; the verified successful
                reference run ends at z ~= -0.0298 m).
  spiral:       ball reached the tube bottom iff |z_final - 0.0345| < 5e-3
                (reference successful run: z ~= 0.0345 m).
  clutter/hero: success = run completed (no abort/timeout).
"""

import csv
import math
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

CAMPAIGN = Path(__file__).resolve().parent
REPO = CAMPAIGN.parents[1]
sys.path.insert(0, str(REPO / "project_plan" / "experiments"))
import parse_run  # noqa: E402

# CAMPAIGN2_OUT_ROOT mirrors lib/common.py: smoke tests point it at a
# scratch directory so trial data/reports never mix with the real campaign.
OUT_ROOT = Path(os.environ.get("CAMPAIGN2_OUT_ROOT") or CAMPAIGN)
DATA = OUT_ROOT / "data"
REPORT = OUT_ROOT / "report"
PLOTS = OUT_ROOT / "plots"

EXPERIMENTS = ["clutter", "hero", "nut_and_bolt", "spiral"]
ACCURACIES = [1e-1, 1e-2, 1e-3]
BETAS = [1.0, 0.1, 0.01]

# (column key, JSON-stats key) for run-level statistics. Times in seconds.
STAT_KEYS = [
    ("steps", "integrator_num_steps_taken"),
    ("convex_solves", "cenic_num_convex_solves"),
    ("geometry_queries", "cenic_num_geometry_queries"),
    ("feasibility_calls", "cenic_num_feasibility_calls"),
    ("time_problem_build", "cenic_time_problem_build"),
    ("time_geometry", "cenic_time_geometry_queries"),
    ("time_feasibility", "cenic_time_feasibility"),
    ("time_solve", "cenic_time_solve"),
    ("surface_triangles", "cenic_scene_num_surface_triangles"),
    ("tetrahedra", "cenic_scene_num_tetrahedra"),
    ("num_velocities", "cenic_scene_num_velocities"),
]

# Quantities plotted vs accuracy (label, column, log-y?).
PLOT_QUANTITIES = [
    ("wall clock [s]", "wall_clock", True),
    ("time steps", "steps", True),
    ("convex solves", "convex_solves", True),
    ("geometry queries", "geometry_queries", True),
    ("feasibility calls", "feasibility_calls", True),
    ("problem-build time [s]", "time_problem_build", True),
    ("geometry-query time [s]", "time_geometry", True),
    ("feasibility time [s]", "time_feasibility", True),
    ("convex-solve time [s]", "time_solve", True),
]

TABLE_COLUMNS = [
    ("experiment", "experiment"), ("config", "config"),
    ("status", "status"), ("success", "success"),
    ("tris", "surface_triangles"), ("tets", "tetrahedra"),
    ("nv", "num_velocities"), ("steps", "steps"),
    ("solves", "convex_solves"), ("geom q", "geometry_queries"),
    ("feas q", "feasibility_calls"), ("wall [s]", "wall_clock"),
    ("t_build", "time_problem_build"), ("t_geom", "time_geometry"),
    ("t_feas", "time_feasibility"), ("t_solve", "time_solve"),
]

BETA_COLORS = {1.0: "tab:blue", 0.1: "tab:orange", 0.01: "tab:green"}


def stats_of(run: dict) -> dict:
    """Returns the run-level statistics dict regardless of driver kind."""
    summary = run.get("summary", {})
    return summary.get("integrator_stats") or summary.get("json_stats") or {}


def status_of(run: dict) -> str:
    meta = run.get("meta", {})
    if run.get("ok"):
        return "ok"
    if meta.get("timed_out"):
        return "timeout"
    if meta:
        return f"fail({meta.get('exit_code')})"
    return "missing"


def success_of(experiment: str, run: dict) -> str:
    """Applies the experiment-specific success criterion (see docstring)."""
    if status_of(run) != "ok":
        return "no"
    state = run.get("summary", {}).get("final_state")
    if experiment == "nut_and_bolt":
        if not state or len(state) < 7:
            return "?"
        return "yes" if state[6] <= -0.029 else "NO(jam)"
    if experiment == "spiral":
        if not state or len(state) < 7:
            return "?"
        return "yes" if abs(state[6] - 0.0345) < 5e-3 else "NO(stuck)"
    return "yes"  # clutter/hero: completion is the criterion.


def collect() -> list:
    """Loads every config as one row-dict joining timing (authoritative) and
    heavy (fallback scene stats) passes."""
    rows = []
    for exp in EXPERIMENTS:
        timing = {r["dir"].name: r
                  for r in parse_run.load_experiment(DATA / "timing", exp)}
        heavy = {r["dir"].name: r
                 for r in parse_run.load_experiment(DATA / "heavy", exp)}
        for cid in sorted(set(timing) | set(heavy)):
            t_run, h_run = timing.get(cid), heavy.get(cid)
            primary = t_run if t_run and t_run["meta"] else h_run
            params = primary["meta"].get("params", {})
            timing_status = status_of(t_run) if t_run else "missing"
            heavy_status = status_of(h_run) if h_run else "missing"
            # A config whose timing run never happened (campaign stopped
            # early) but whose heavy run succeeded is reported as
            # "heavy-only": counters, scene stats, and success are
            # timing-independent and taken from the heavy pass; wall clock
            # and phase timers are left blank rather than quoting
            # instrumentation-contaminated numbers.
            status = timing_status
            if timing_status in ("missing", "fail(None)") \
                    and heavy_status == "ok":
                status = "heavy-only"
            success_run = t_run if timing_status == "ok" else (
                h_run if heavy_status == "ok" else None)
            row = {
                "experiment": exp,
                "config": cid,
                "variant": params.get("variant", exp),
                "accuracy": params.get("accuracy"),
                "beta": params.get("beta"),
                "barrier": params.get("barrier"),
                "margin": params.get("margin"),
                "status": status,
                "heavy_status": heavy_status,
                "success": (success_of(exp, success_run)
                            if success_run else "?"),
                "wall_clock": ((t_run or {}).get("summary", {})
                               .get("wall_clock")
                               if timing_status == "ok" else None),
            }
            stats = stats_of(t_run) if timing_status == "ok" else {}
            fallback = stats_of(h_run) if h_run else {}
            timing_only = ("time_problem_build", "time_geometry",
                           "time_feasibility", "time_solve")
            for col, key in STAT_KEYS:
                value = stats.get(key)
                if value is None and col not in timing_only:
                    value = fallback.get(key)
                row[col] = value
            row["_times"] = (t_run or {}).get("times")
            rows.append(row)
    return rows


def fmt(value) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        if value == 0:
            return "0"
        if abs(value) >= 1e5 or abs(value) < 1e-3:
            return f"{value:.3g}"
        return f"{value:.4g}" if abs(value) < 1 else f"{value:,.1f}"
    if isinstance(value, int):
        return f"{value:,}"
    return str(value)


def near(a, b) -> bool:
    return a is not None and b is not None and \
        math.isclose(a, b, rel_tol=1e-9)


def write_report(rows: list) -> None:
    REPORT.mkdir(parents=True, exist_ok=True)
    lines = ["# Campaign2 cross-experiment report", "",
             "Wall clock and phase timers come from the **timing pass** "
             "(collect_heavy_stats OFF, serial execution). Counters include "
             "rejected steps. Phase times: t_build = ICF problem assembly "
             "minus geometry queries; t_geom = hydroelastic contact-surface "
             "queries inside the builder; t_feas = IsFeasibleTrajectory "
             "(CCD); t_solve = convex solves.", ""]
    for acc in ACCURACIES:
        for beta in BETAS:
            cell = [r for r in rows
                    if near(r["accuracy"], acc) and near(r["beta"], beta)]
            lines.append(f"## accuracy = {acc:g}, beta = {beta:g}")
            lines.append("")
            if not cell:
                lines += ["(no runs)", ""]
                continue
            lines.append("| " + " | ".join(h for h, _ in TABLE_COLUMNS)
                         + " |")
            lines.append("|" + "|".join("---" for _ in TABLE_COLUMNS) + "|")
            for r in sorted(cell, key=lambda r: (r["experiment"],
                                                 r["config"])):
                lines.append("| " + " | ".join(
                    fmt(r.get(key)) for _, key in TABLE_COLUMNS) + " |")
            lines.append("")
    bad = [r for r in rows
           if r["status"] not in ("ok", "heavy-only")
           or r["heavy_status"] != "ok"]
    heavy_only = [r for r in rows if r["status"] == "heavy-only"]
    lines += ["## Failures / timeouts", ""]
    if bad:
        lines.append("| experiment | config | timing pass | heavy pass |")
        lines.append("|---|---|---|---|")
        for r in bad:
            lines.append(f"| {r['experiment']} | {r['config']} | "
                         f"{r['status']} | {r['heavy_status']} |")
    else:
        lines.append("None — every attempted run completed.")
    lines.append("")
    if heavy_only:
        lines.append(
            f"Additionally, {len(heavy_only)} config(s) are `heavy-only`: "
            "the heavy pass succeeded but the serial timing pass was not "
            "run (campaign stopped early). Their counters, scene stats, "
            "and success columns are valid (timing-independent, from the "
            "heavy pass); wall clock and phase timers are omitted. "
            "Resume with `run_all.py --passes timing` to fill them in.")
        lines.append("")
    (REPORT / "REPORT.md").write_text("\n".join(lines))
    print(f"wrote {REPORT / 'REPORT.md'}")

    csv_columns = ["experiment", "config", "variant", "accuracy", "beta",
                   "barrier", "margin", "status", "heavy_status", "success",
                   "wall_clock"] + [c for c, _ in STAT_KEYS]
    with open(REPORT / "all_runs.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns,
                                extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    for exp in EXPERIMENTS:
        with open(REPORT / f"{exp}.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=csv_columns,
                                    extrasaction="ignore")
            writer.writeheader()
            writer.writerows([r for r in rows if r["experiment"] == exp])
    print(f"wrote {REPORT}/all_runs.csv and per-experiment CSVs")


def series(rows: list, column: str) -> tuple:
    """Returns (accuracies, values) for rows sorted by accuracy, dropping
    missing values."""
    pts = [(r["accuracy"], r.get(column)) for r in rows
           if r["accuracy"] is not None and r.get(column) is not None]
    pts.sort()
    return [p[0] for p in pts], [p[1] for p in pts]


def plot_clutter(rows: list) -> None:
    barriers = sorted({r["barrier"] for r in rows}, reverse=True)
    margins = sorted({r["margin"] for r in rows}, reverse=True)
    for variant in sorted({r["variant"] for r in rows}):
        vrows = [r for r in rows if r["variant"] == variant]
        for label, column, log_y in PLOT_QUANTITIES:
            fig, axes = plt.subplots(len(barriers), len(margins),
                                     figsize=(10, 8), sharex=True,
                                     sharey=True, squeeze=False)
            for i, barrier in enumerate(barriers):
                for j, margin in enumerate(margins):
                    ax = axes[i][j]
                    for beta in BETAS:
                        sel = [r for r in vrows
                               if near(r["barrier"], barrier)
                               and near(r["margin"], margin)
                               and near(r["beta"], beta)]
                        x, y = series(sel, column)
                        if x:
                            ax.plot(x, y, "o-", color=BETA_COLORS[beta],
                                    label=f"beta={beta:g}")
                    ax.set_xscale("log")
                    if log_y:
                        ax.set_yscale("log")
                    ax.set_title(f"barrier={barrier:g}, margin={margin:g}",
                                 fontsize=9)
                    ax.grid(True, which="both", alpha=0.3)
                    if i == len(barriers) - 1:
                        ax.set_xlabel("accuracy")
                    if j == 0:
                        ax.set_ylabel(label)
            axes[0][0].legend(fontsize=8)
            fig.suptitle(f"clutter/{variant}: {label} vs accuracy")
            fig.tight_layout()
            fig.savefig(PLOTS / f"clutter_{variant}_{column}.png", dpi=120)
            plt.close(fig)
    print(f"wrote clutter plots to {PLOTS}")


def plot_hero(rows: list) -> None:
    for label, column, log_y in PLOT_QUANTITIES:
        fig, ax = plt.subplots(figsize=(7, 5))
        for variant, style in (("barrier", "o-"), ("volumetric", "s--")):
            for beta in BETAS:
                sel = [r for r in rows if r["variant"] == variant
                       and near(r["beta"], beta)]
                x, y = series(sel, column)
                if x:
                    ax.plot(x, y, style, color=BETA_COLORS[beta],
                            label=f"{variant}, beta={beta:g}")
        ax.set_xscale("log")
        if log_y:
            ax.set_yscale("log")
        ax.set_xlabel("accuracy")
        ax.set_ylabel(label)
        ax.set_title(f"hero demo: {label} vs accuracy")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOTS / f"hero_{column}.png", dpi=120)
        plt.close(fig)

    # Windowed mean real-time ratio vs simulated time (timing pass only).
    window = 2.0  # seconds of simulated time per averaging window.
    for acc in ACCURACIES:
        fig, ax = plt.subplots(figsize=(7, 5))
        plotted = False
        for variant, style in (("barrier", "-"), ("volumetric", "--")):
            for beta in BETAS:
                sel = [r for r in rows if r["variant"] == variant
                       and near(r["beta"], beta)
                       and near(r["accuracy"], acc)]
                if not sel or sel[0]["_times"] is None \
                        or sel[0]["_times"].empty:
                    continue
                times = sel[0]["_times"]
                sim = times["sim_time"].to_numpy()
                wall = times["wall_time"].to_numpy()
                xs, ys = [], []
                t0 = sim[0]
                while t0 < sim[-1]:
                    t1 = t0 + window
                    i0, i1 = np.searchsorted(sim, [t0, t1])
                    i1 = min(i1, len(sim) - 1)
                    if i1 > i0 and wall[i1] > wall[i0]:
                        xs.append(0.5 * (t0 + t1))
                        ys.append((sim[i1] - sim[i0]) /
                                  (wall[i1] - wall[i0]))
                    t0 = t1
                if xs:
                    ax.plot(xs, ys, style, color=BETA_COLORS[beta],
                            label=f"{variant}, beta={beta:g}")
                    plotted = True
        if not plotted:
            plt.close(fig)
            continue
        ax.axhline(1.0, color="k", linewidth=0.8, alpha=0.5)
        ax.set_yscale("log")
        ax.set_xlabel("simulated time [s]")
        ax.set_ylabel(f"real-time ratio ({window:g}s windows)")
        ax.set_title(f"hero demo: windowed RTR, accuracy={acc:g}")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOTS / f"hero_rtr_acc{acc:g}.png", dpi=120)
        plt.close(fig)
    print(f"wrote hero plots to {PLOTS}")


def main() -> int:
    rows = collect()
    if not rows:
        print("no campaign2 data found under", DATA)
        return 1
    PLOTS.mkdir(parents=True, exist_ok=True)
    write_report(rows)
    clutter_rows = [r for r in rows if r["experiment"] == "clutter"]
    hero_rows = [r for r in rows if r["experiment"] == "hero"]
    if clutter_rows:
        plot_clutter(clutter_rows)
    if hero_rows:
        plot_hero(hero_rows)
    n_ok = sum(r["status"] == "ok" for r in rows)
    print(f"{n_ok}/{len(rows)} timing runs ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
