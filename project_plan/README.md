# bCENIC Thin-Objects — Experiment Campaign

Everything needed to run, re-run, and analyze the paper experiments for the
thin-objects (barrier CENIC) method on this branch.

## Layout

```
project_plan/
  ANALYSIS.md          <- THE report: measurements, all plots, observations,
                          master summary table, code-change suggestions
  PAPER_NOTES.md       <- paper-strengthening notes (CCD/TOI alternatives,
                          SOTA comparison plan, proposed experiments)
  HERO_DEMO.md         <- standalone E7 write-up: scene, barrier-vs-volumetric
                          setup, run matrix, results, plots, how to reproduce
  experiments/
    manifest.py        declares every campaign run (E1..E7, ~135 runs)
    run_all.py         campaign driver (parallel, resumable, timeouts)
    parse_run.py       run-directory parsing library
    analyze.py         regenerates all plots + ANALYSIS.md from data/
    method_figures.py  analytic method figures (no simulation)
  data/<experiment>/<run_id>/   one directory per run:
    meta.json          argv, params, git sha, timing, exit code
    config.yaml        (clutter runs) the exact config used
    steps.tsv          per-step CenicStepStatistics
    times.csv          (hero runs) (sim_time, wall_time) per accepted step
    stdout.log / stderr.log
    summary_demo.json  (python demo runs) machine-readable run summary
    DONE               marker: run completed with exit code 0
  data/legacy_rolling_sphere/   authoritative data from earlier hand runs
  plots/<experiment>/*.png
  legacy_matlab/       original MATLAB/Octave figure scripts (reference for
                       figure formats; superseded by analyze.py)
  legacy_scripts/      original data-collection shell scripts (superseded by
                       run_all.py; they target gflags that no longer exist)
  docs/                planning/brainstorming documents
```

## How to run

```bash
# Whole campaign (builds first; ~105 runs, 16 workers, resumable):
python3 project_plan/experiments/run_all.py

# One tier / one experiment / one run:
python3 project_plan/experiments/run_all.py --tier A
python3 project_plan/experiments/run_all.py --experiment E4_nut_and_bolt
python3 project_plan/experiments/run_all.py --only E4_nut_and_bolt/acc0.01_beta1

# Hero demo (E7; ~14 runs of the full 100 s dishrack playback — the first
# ever hero run needs network access for the lbm_eval model wheel):
python3 project_plan/experiments/run_all.py --experiment E7_hero_dishrack

# Force a re-run (ignores DONE markers):
python3 project_plan/experiments/run_all.py --only E1_ball_beta/beta1_acc0.1 --force

# List what would run:
python3 project_plan/experiments/run_all.py --list

# Regenerate every plot + ANALYSIS.md from the collected data:
python3 project_plan/experiments/analyze.py
```

Runs are independent; a failure or timeout is logged to `data/failures.log`
and never stops the campaign. Re-invoking `run_all.py` skips anything with a
`DONE` marker, so interrupted campaigns resume for free.

## The binaries

| Target | Purpose |
|---|---|
| `//examples/integrators:error_control_demo` | Python demo harness: `--example` ∈ ball_on_table, clutter, plate_and_spatula, cones, teddy_and_torus, nut_and_bolt, fidget, sphere_and_spiral, spinning_plate. Flags: `--accuracy --beta --E --d --margin --barrier --resolution --tau --sim_time --use_toi --stats_file --summary_file --visualize --initial_w`. Meshcat only with `--visualize`. |
| `//examples/multibody/clutter:clutter` | C++ clutter scene, fully YAML-driven. `--config=<yaml>` selects a config (defaults to the packaged one); `--log_file` writes per-step stats; `--barrier/--accuracy` optionally override the YAML. Headless when the YAML has no `visualization_config` stanza. |
| `//examples/integrators:rotational_ccd_study` | Kinematic linear-vs-curved CCD study (no dynamics): false-negative grid over (ω, dt) + query-cost-vs-substeps ladder. |
| `//examples/hero_demo:convex_integrator_playback` | Hero demo (E7): 100 s dual-Panda dishrack playback from lbm_eval models. Flags: `--accuracy --max_step_size --sim_time --margin --barrier --stats_file --times_file --summary_file --visualize`. `--margin/--barrier` replace the ('hydroelastic', margin/barrier) properties on every collision geometry; `--barrier=0 --margin=0` selects volumetric hydro. **The first-ever run downloads the `lbm_eval` model wheel from GitHub (needs network); cached under `~/.cache/drake/package_map` afterwards.** |

Interactive examples (not part of the campaign):

- `//examples/multibody/franka:franka` — Panda + YCB box, YAML-driven.
- `examples/integrators/ball_on_table.cc` — dead code (no build target,
  uses the older ConvexIntegrator); superseded by the Python examples.
- `examples/integrators/clutter/clutter_demo.py` — expects a `models/`
  directory of SDFs that is not in the tree; kept for reference.

## Data formats

**Per-step TSV** (`steps.tsv`): columns from `CenicStepStatistics::to_string()`
(multibody/cenic/cenic_integrator.h):
`step_type time step_size num_solver_iterations total_linesearch_iterations
max_linesearch_iterations mean_linesearch_iterations max_condition_number
last_condition_number max_e0 mean_e0 total_num_constraint_pairs`.
Demo runs include a header line; clutter runs do not (parse_run.py handles
both). Rows come in full_step / half_step_1 / half_step_2 groups per DoStep
attempt; `parse_run.accepted_steps()` reconstructs accepted steps.

**JSON statistics** (in stdout, from `PrintSimulatorStatistics`): includes
the campaign-added instrumentation —
`cenic_num_feasibility_rejections_{full,half1,half2}` (CCD rejections per
check site) and `cenic_time_{model_update,solve,feasibility,linearize}`
(accumulated wall-clock runtime breakdown of DoStep).

## Known quirks

- `Mesh` collision geometry supports `barrier = 0` (volumetric hydro) the
  same way Sphere/Box do: a `.vtk` file supplies the tet mesh directly, any
  other format falls back to its convex hull, and the field is a normalized
  extent field (modulus 1.0 — the ICF builder applies E* itself, matching
  the Sphere/Box volumetric branches). That's what the E7 volumetric rows
  use; E3b's barrier=0 rows exercised the Sphere/Box branches.
- Cylinder/Capsule/Ellipsoid/HalfSpace have **no barrier path**: scenes for
  this method must use Sphere/Box/Mesh/Convex collision geometry.
- `SceneGraphConfig.default_proximity_properties.{margin,barrier}` (e.g.
  from a scenario YAML) never reach the *model*-time hydroelastic
  reification that runs during parsing — the parser registers geometries
  before defaults apply, so that first reification uses the hardcoded
  `margin=2e-4, barrier=1e-4` fallbacks in hydroelastic_internal.cc. The
  defaults DO reach the simulation: context creation builds an augmented
  model (`SceneGraph::SetDefaultParameters` → `ApplyProximityDefaults`)
  that re-reifies hydro with the backfilled values. Materializing the
  defaults at model time instead was tried and reverted — it breaks the
  documented `set_config`-after-registration workflow
  (SceneGraphTest.ApplyConfig). For per-run sweeps, the hero demo's
  `--margin/--barrier` flags sidestep all of this by replacing the
  properties per geometry (`AssignRole(kReplace)`), which always wins.
- The log-barrier patch path hard-codes dissipation d=50 s/m
  (`icf_builder.cc`, `kDefaultDissipation`) — the `--d` flag of
  error_control_demo does not reach the barrier constraints.
- error_control_demo always pre-rolls to t=0.1 s before applying `--tau`.
- The E5/E6 `--use_toi` flag correspond to
  `IcfSolverParameters::use_toi` (TOI-based step-size adjustment on CCD
  rejection; off = bisection).

## Pre-existing test drift on this branch

`bazel test //multibody/cenic/... //multibody/contact_solvers/icf/...`:
80/84 pass. The failures predate the campaign and stem from the research
branch itself (verified against the pre-campaign tree):

- `icf_builder_test` fails to *build*: the test still calls the upstream
  `IcfBuilder::UpdateModel` signature without the `beta` parameter this
  branch added.
- `icf_model_test` (2 cases) and `icf_solver_test` (8 cases) construct
  models without barrier parameters, so the barrier path hits its NaN
  guard / the hot-path `DRAKE_DEMAND` in `calc_N_x`.
- `icf_model_cpplint` flags pre-existing >80-char lines in
  `patch_constraints_pool.cc`.

These are test-maintenance debt for the branch (worth fixing before a PR
upstream), not simulation bugs — the campaign exercised the same code paths
across wide parameter ranges without tripping the guards.
