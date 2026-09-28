# Campaign 2 — instrumented re-run of the four headline experiments

Re-runs clutter, hero-demo, nut-and-bolt, and sphere-in-spiral-tube with
corrected measurement methodology and richer per-run statistics. All outputs
live under this directory; the original campaign under `project_plan/data/`
and `project_plan/experiments/` is untouched.

## Why a re-run

The campaign-1 wall-clock numbers were measured with `collect_heavy_stats`
enabled, which computes dense condition-number estimates inside the solve
loop — the timings were contaminated by the instrumentation. Campaign 2 uses
a strict **two-pass protocol**:

| pass   | collect_heavy_stats | parallelism | provides |
|--------|--------------------|-------------|----------|
| heavy  | ON                 | `--jobs` (default 8) | per-step stats (steps.tsv): condition numbers, contact counts, e0, dt series |
| timing | OFF                | **forced serial** | authoritative wall clock, phase timers, counters |
| html   | OFF                | serial | one representative run per experiment recorded to a static meshcat HTML for visual sanity checks (`html/<experiment>.html`) |

## Per-run statistics (every run, from the JSON Statistics stdout block /
`summary_demo.json` `integrator_stats`)

- Scene size (measured once at startup): total surface triangles
  (`cenic_scene_num_surface_triangles`), tetrahedra in the extruded barrier
  meshes (`cenic_scene_num_tetrahedra`), DOFs (`cenic_scene_num_velocities`).
- Whole-run counters, **including rejected steps**: time steps
  (`integrator_num_steps_taken`), convex solves (`cenic_num_convex_solves` =
  IcfSolver::SolveWithGuess calls), hydroelastic contact-surface queries
  (`cenic_num_geometry_queries`), CCD feasibility calls
  (`cenic_num_feasibility_calls`).
- Wall clock (timing pass) and per-phase time: problem building
  (`cenic_time_problem_build`), geometry queries
  (`cenic_time_geometry_queries`), feasibility (`cenic_time_feasibility`),
  convex solve (`cenic_time_solve`).

## Sweep

All experiments share accuracy ∈ {1e-1, 1e-2, 1e-3} × beta ∈ {1, 0.1, 0.01}.

| experiment | extra sweep | configs | script |
|---|---|---|---|
| clutter | {mesh20, primitives} × barrier {1e-4,1e-3} × margin {1e-4,1e-3} | 72 | `run_clutter.py` |
| hero | {barrier(2e-4/1e-4), volumetric(0/0)} | 18 | `run_hero.py` |
| nut_and_bolt | — (fixed threading params) | 9 | `run_nut_and_bolt.py` |
| spiral | margin=barrier ∈ {1e-5, 2e-5, 5e-5, 1e-4} | 36 | `run_spiral.py` |

135 configs → 270 heavy+timing runs + 4 html runs. Each runner script's
docstring documents the scenario, every parameter, the binary, and exactly
how each parameter reaches it.

## Thingi10K mesh set

20 low-poly (100–300 faces) "objects in the wild", normalized to a 10 cm
bounding box, thin shapes deliberately kept (min extent down to 2.5 mm),
degenerate sub-2 mm sheets rejected, every mesh validated through the
barrier-mesh inflation preflight. Reproduce with:

```
bazel build //examples/multibody/clutter:mesh_preflight
python3 project_plan/experiments/thingi10k_meshes.py \
  --out project_plan/campaign2/meshes/thingi10k_lowpoly20 \
  --seed 1 --count 20 --min-faces 100 --max-faces 300 \
  --target-size 0.10 --min-extent 0.002
```

The exact draw is pinned in `meshes/thingi10k_lowpoly20/manifest.json`
(ids: 116891, 53882, 59560, 89913, 59559, 94733, 76202, 249521, 53435,
68646, 472143, 110173, 79184, 1452674, 693468, 43780, 74458, 255657,
472084, 69128).

## Running

```
python3 project_plan/campaign2/run_all.py                  # everything
python3 project_plan/campaign2/run_all.py --passes heavy --jobs 16
python3 project_plan/campaign2/run_all.py --passes timing  # serial, long
python3 project_plan/campaign2/run_all.py --passes html,analyze
python3 project_plan/campaign2/run_clutter.py --only mesh20 --pass heavy
```

Every run directory gets a `DONE` marker on success; re-invocations skip
completed runs, so an interrupted campaign resumes with the same command.
Failures are listed in `data/failures.log` and in the report appendix.

The analyzer needs pandas/matplotlib/numpy; `run_all.py` falls back to
`uv run --with pandas ...` automatically when the invoking interpreter lacks
them. To run it directly:

```
uv run --no-project --with pandas --with matplotlib --with numpy \
  python3 project_plan/campaign2/analyze.py
```

`CAMPAIGN2_OUT_ROOT=<dir>` redirects all outputs (`data/`, `html/`,
`report/`, `plots/`) for smoke testing without touching the real tree.

Note: the hero snapshot is ~110 MB raw (StaticHtml embeds the full
dishrack/arm scene geometry), which exceeds GitHub's 100 MB file limit, so
it is tracked as `html/hero.html.gz` (~75 MB) — `gunzip -k` it to view.
The other three snapshots are 3–5 MB and tracked raw, both as
`html/<exp>.html` and as the byte-identical per-run
`data/html/<exp>/<config>/scene.html` (git stores each blob once); the
hero per-run `scene.html` is untracked for the same size reason.

## Outputs

- `data/{heavy,timing,html}/<experiment>/<config_id>/` — stdout/stderr logs,
  `meta.json` (argv, params, git sha, wall seconds, exit code), driver
  outputs (`summary_demo.json`, `steps.tsv`, `times.csv`, `config.yaml`).
- `report/REPORT.md` — 9 cross-experiment tables (one per accuracy × beta
  cell), failure appendix; `report/*.csv` — machine-readable slices.
- `plots/` — quantity-vs-accuracy plots (β lines; hero adds volumetric
  lines; hero windowed real-time-ratio curves).
- `html/<experiment>.html` — static meshcat scene+animation snapshots.
