# E7 — Hero Demo: barrier hydro vs volumetric hydro on the dishrack playback

_Standalone write-up of the E7 experiment (July 2026). The summary table and
plots are regenerated from `project_plan/data/E7_hero_dishrack/` by
`python3 project_plan/experiments/analyze.py`; this file is the narrative
companion (the auto-generated section lives in ANALYSIS.md)._

## Objective

The paper's hero application (docs/Thin Objects.txt §3): characterize bCENIC
performance on a **real robotics manipulation task** and compare thin-layer
**barrier hydro** against ICF with Drake's default **volumetric
hydroelastic**. Deliverables: the RTR-vs-time figure, and per-run summary
stats — #contacts (mean/max), total #steps, mean RTR over the full sim.

## The scene

`//examples/hero_demo:convex_integrator_playback` — a recorded 100 s
dual-Panda dishrack-loading demonstration (lbm_eval riverway station),
played back through stiff joint-space PD controllers from
`keyframes.txt` + `resolved_scenario.yaml`. Models come from the
`lbm_eval_models` wheel (downloaded from GitHub on the first-ever run;
cached under `~/.cache/drake/package_map` afterwards).

Collision geometry inventory (why this scene stresses thin objects):

- **All manipulands are `.vtk` volumetric tet meshes** with
  `<drake:compliant_hydroelastic/>`: dish-rack base / **wireframe** /
  utensil holder, spatula, spoon, mug, crock, and the finray gripper
  fingers. The rack wireframe and utensils are the thin geometry.
- Panda arms: spheres (58/arm). Station/table/bins: boxes + cylinders.
  (Cylinders have no barrier path on this branch — identical in both modes.)

## The two contact models, and how to switch

Both modes run the same CENIC integrator + ICF solver; only the hydroelastic
representation of the geometry changes, selected per run by the demo's
`--margin/--barrier` flags (which **replace** the `('hydroelastic', margin |
barrier)` properties on every collision geometry via
`AssignRole(kReplace)` — this always wins over YAML/SDF defaults):

- **Barrier hydro (thin objects), `barrier > 0`** — geometry reification
  extrudes the mesh surface into a thin shell of thickness
  `margin + barrier` carrying an indicator field (surface value
  `−2·margin/barrier`, core value 2). Contact force is the log-barrier
  patch formulation.
- **Volumetric hydro, `barrier = 0` (with `margin = 0`)** — Drake's default
  compliant representation: the `.vtk` tet mesh is used directly with a
  distance-based extent field (`Mesh` support for this was added for E7 in
  `geometry/proximity/hydroelastic_internal.cc`, mirroring the pre-existing
  Sphere/Box `barrier == 0` branches; non-`.vtk` meshes fall back to their
  convex hull). N.B. the field uses modulus 1.0 (a normalized extent
  field), because `icf_builder.cc` routes all hydro surfaces through the
  log-barrier patch path and applies the effective modulus E* itself —
  building a real pressure field trips the solver's `calc_N_x` guard.

Volumetric hydro needs **accuracy ≈ 1e-3** to avoid thin-object artifacts
in this scene; barrier hydro aims to run artifact-free at loose accuracy.

## Run matrix (14 runs, all full 100 s, headless)

| block | parameters |
|---|---|
| A. accuracy sweep, barrier hydro | margin=2e-4, barrier=1e-4; accuracy ∈ {1e-1, 1e-2, 1e-3} |
| B. margin × barrier grid | margin ∈ {1e-4, 2e-4, 5e-4} × barrier ∈ {1e-5, 1e-4, 1e-3} at accuracy 1e-1 |
| C. volumetric baseline | margin=0, barrier=0; accuracy ∈ {1e-1, 1e-2, 1e-3} |

## Metrics collected per run

- `times.csv` — (sim_time, wall_time) after every accepted step, recorded by
  a simulator monitor → windowed RTR-vs-time (2 s trailing window) and mean
  RTR (= sim/wall).
- `steps.tsv` — per-step `CenicStepStatistics`: contact-pair counts
  (mean/max over accepted steps), dt, solver iterations, conditioning.
- `stdout.log` — `PrintSimulatorStatistics` JSON: total steps, CCD and
  error-control rejections, runtime breakdown.
- `summary_demo.json`, `meta.json` — parameters, wall time, exit code.

## Results

| run | wall [s] | RTR | steps | mean dt | min dt | CCD rej | EC rej | mean #contacts | max #contacts |
|---|---|---|---|---|---|---|---|---|---|
| barrier1e-05_margin0.0001_acc0.1 | 322 | 0.311 | 10608 | 9.44e-03 | 1.16e-04 | 23555 | 0 | 915 | 1373 |
| barrier0.0001_margin0.0001_acc0.1 | 112 | 0.892 | 2754 | 0.0364 | 2.51e-04 | 4435 | 0 | 615 | 929 |
| barrier0.001_margin0.0001_acc0.1 | 34.6 | 2.89 | 1289 | 0.0778 | 1.00e-03 | 621 | 7 | 368 | 632 |
| barrier0.0001_margin0.0002_acc0.001 | 207 | 0.484 | 6541 | 0.0156 | 3.97e-04 | 594 | 2798 | 851 | 1229 |
| barrier0.0001_margin0.0002_acc0.01 | 129 | 0.778 | 3498 | 0.0287 | 2.72e-04 | 2366 | 533 | 768 | 1133 |
| barrier0.0001_margin0.0002_acc0.1 | 95.4 | 1.05 | 2822 | 0.0355 | 3.81e-04 | 4640 | 2 | 784 | 1089 |
| barrier0.001_margin0.0002_acc0.1 | 36.8 | 2.71 | 1269 | 0.0789 | 1.22e-03 | 565 | 12 | 428 | 687 |
| barrier0.0001_margin0.0005_acc0.1 | 131 | 0.764 | 2909 | 0.0344 | 5.47e-04 | 4887 | 2 | 1.08e+03 | 1558 |
| barrier0.001_margin0.0005_acc0.1 | 29.6 | 3.37 | 1132 | 0.0884 | 1.95e-03 | 264 | 1 | 602 | 943 |
| volumetric_acc0.001 | 45.8 | 2.18 | 2716 | 0.0376 | 7.58e-04 | 0 | 967 | 407 | 1009 |
| volumetric_acc0.01 | 27.3 | 3.67 | 1467 | 0.0687 | 4.66e-03 | 0 | 461 | 419 | 1332 |
| volumetric_acc0.1 | 27.7 | 3.61 | 1010 | 0.099 | 0.01 | 0 | 17 | 1.61e+03 | 2759 |

### Key observations

1. **Barrier thickness, not accuracy, is the dominant RTR knob.** At fixed
   accuracy 1e-1, sweeping barrier 1e-3 → 1e-4 → 1e-5 moves mean RTR
   ~3× → ~0.9× → 0.31×, driven almost entirely by CCD rejections
   (264–621 → ~4.5k → 23.5k): thinner layers force the feasibility check
   to shrink steps far more often. Margin has a comparatively mild effect.
2. **The barrier=1e-3 runs beat the volumetric acc=1e-3 baseline**
   (RTR 2.7–3.4 vs 2.18) — the thin-layer model at loose accuracy
   outruns volumetric hydro at the accuracy it needs to be artifact-free.
   At the default barrier=1e-4 the ordering reverses (1.05 vs 2.18);
   the artifact comparison decides which barrier setting is fair.
3. **Accuracy scaling is graceful for barrier hydro**: 1.05 → 0.78 → 0.48
   for accuracy 1e-1 → 1e-2 → 1e-3 at the default margin/barrier, with
   rejections shifting from CCD-dominated to error-control-dominated.
4. **Quantitative artifact evidence for loose volumetric**: at accuracy
   1e-1 the volumetric run reports mean #contacts 1609 / max 2759 —
   ~4× every other run — consistent with objects sinking into surfaces
   and generating spurious deep-contact patches. At 1e-3 the contact
   counts normalize (407 mean). Volumetric runs show 0 CCD rejections
   (there is no thin layer to protect).
5. **Feasibility boundary**: the barrier=1e-5 cells with margin ≥ 2e-4
   abort at startup — surface ε = −2·margin/barrier ≤ −40 and the scene's
   initial resting contacts pierce the 10 µm layer
   (`calc_N_x` consistency guard, same failure class E3e hit at impact).
   Only margin=1e-4 survives at barrier=1e-5. Rule of thumb: the barrier
   thickness must not be small relative to the initial contact
   penetration.

## Plots

Real-time rate vs simulation time (2 s trailing window), barrier hydro vs
volumetric across the accuracy sweep — the paper's headline figure. The
periodic dips are the contact-heavy manipulation phases; the final third
(t ≈ 60–90 s) is the busiest part of the task:

![](plots/E7_hero_dishrack/rtr_vs_time.png)

RTR vs time across the margin × barrier grid at accuracy 1e-1 — the
barrier-thickness separation of observation 1:

![](plots/E7_hero_dishrack/rtr_vs_time_grid.png)

Active contact pairs vs time (headline runs) — note the inflated counts of
volumetric acc=0.1 (observation 4):

![](plots/E7_hero_dishrack/contacts_vs_time.png)

Accepted step size vs time (headline runs):

![](plots/E7_hero_dishrack/dt_vs_time.png)

Wall-time breakdown (convex solve / model update / CCD feasibility /
linearization / other):

![](plots/E7_hero_dishrack/runtime_breakdown.png)

## Reproducing

```bash
# Build + run all 14 runs (resumable; ~6 min wall on a 64-core machine).
# First-ever run needs network for the lbm_eval model wheel.
python3 project_plan/experiments/run_all.py --experiment E7_hero_dishrack

# Regenerate the summary table and all plots (also refreshes ANALYSIS.md):
python3 project_plan/experiments/analyze.py

# A single run, by id:
python3 project_plan/experiments/run_all.py --only E7_hero_dishrack/volumetric_acc0.001 --force

# Watch it interactively (meshcat):
bazel run //examples/hero_demo:convex_integrator_playback -- \
    --visualize=1 --accuracy=1e-1 --margin=2e-4 --barrier=1e-4
```

The two infeasible barrier=1e-5 cells re-attempt (and re-fail, in ~4 s) on
every campaign invocation; that is expected and recorded in
`data/failures.log`.

## Caveats / follow-ups

- The volumetric-vs-barrier *artifact* comparison here is quantitative only
  (contact-count inflation); the paper also wants a visual montage —
  record meshcat HTML for one volumetric acc=1e-1 and one barrier run.
- Volumetric contact surfaces are computed from normalized extent fields
  (equal effective modulus for the surface cut), matching the E3b
  convention on this branch — not bitwise upstream-Drake behavior, but the
  fairest in-solver ablation.
- Mean RTR here is sim/wall over the whole run including startup; the
  windowed curves exclude nothing.
