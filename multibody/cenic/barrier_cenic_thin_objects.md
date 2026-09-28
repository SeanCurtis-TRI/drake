# Thin Objects / Barrier CENIC — Method & Code Review

Working notes for the "Thin Objects" (a.k.a. *Barrier CENIC*) non-penetration contact method
implemented on branch `thin_objects_rebase`, starting 2025-12-08 (commit `1d1efa4f67`, "Full thin
objects implementation"). This document (a) specifies the method in enough detail to seed a paper's
method section, and (b) records a prioritized code review to guide cleanup.

> **Status:** implementation is complete for **rigid bodies only**. The CCD/feasibility path uses
> `convert_to_double`, and the ICF solve throws for non-`double` scalars.

---

## 1. Overview

Each collision geometry is wrapped in a thin **extent** layer. Extent `e` is unitless: `e = 1` at
the true surface of the object and `e = 0` at the inflated outer boundary of the layer. The layer
plays the role of a hydroelastic pressure field, except the pressure is **unbounded as `e → 1`**
(toward the true surface):

```
p(e) = E · e / (1 - e),        E = hydroelastic modulus  [Pa].
```

This produces a very stiff, barrier-like normal force with an analytic continuation into the
near-rigid limit, so the ICF optimization stays **unconstrained**. Two ingredients make it work:

1. **Barrier contact model** (`RegularizedBarrierModel`) fed through the standard discrete
   hydroelastic pipeline, but with **vertex quadrature** (area-weighted to match centroid
   quadrature at `t₀`) and analytic cost/gradient/Hessian.
2. **Linear CCD** in the `CenicIntegrator`: for the full step and each half step, mesh vertices are
   assumed to move on straight lines between `t₀` and `t₁`; a continuous collision query on those
   trajectories rejects any step that would produce penetration and shrinks `dt`. This yields
   penetration-free trajectories **up to the linear-trajectory approximation** (see §6, S1).

### Component map
| Concern | File | Key symbols |
|---|---|---|
| Barrier model math | `multibody/contact_solvers/icf/patch_constraints_pool.{h,cc}` | `RegularizedBarrierModel<T>`, `PatchConstraintsPool<T>` |
| Vertex / area-weighted quadrature | `multibody/contact_solvers/icf/icf_builder.cc` | `SetPatchConstraintsForLogBarrierContact` (~543–779); dual-area loop (~682–777) |
| Extent field generation | `geometry/proximity/hydroelastic_internal.cc`, `make_*_field.cc` | `MakeExtrudedMesh`, `barrier`/`margin` |
| Integrator + feasibility | `multibody/cenic/cenic_integrator.{h,cc}` | `DoStep`, `IsFeasibleTrajectory`, `ComputeAdjustedStepSize` |
| Feasibility driver (broad+narrow) | `geometry/proximity/feasibility_calculator.{h,cc}` | vertex-face + edge-edge over BVH candidates |
| Linear CCD narrowphase | `geometry/proximity/ccd.{h,cc}` | `point_triangle_ccd`, `edge_edge_ccd`, `cubic` |
| Root finding | `math/real_roots.cc` | `cubic_real_roots`, `cubic_real_roots_interval` (unused), `quadratic_real_roots` |
| Query plumbing | `geometry/{query_object,geometry_state,proximity_engine}.*` | `IsFeasibleTrajectory`, `FeasibilityTimeOfImpact` |
| Parameters | `multibody/contact_solvers/icf/icf_solver_parameters.h` | `beta{0.1}`, `use_toi{false}` |

---

## 2. Barrier contact model

All references `patch_constraints_pool.cc`. The model works in normal contact velocity `vₙ`
(positive = separating) with the affine map to extent `e`:

```
vₙ = v_δ · (e₀ − e),     v_δ = 2δ/dt         (:148)
```

where `e₀` is the extent at the start of the step and `δ` is the characteristic penetration depth.

- **Elastic impulse** (barrier): `n_e(e) = dt · A₀E* · e/(1−e)` (`:200`).
  Numerically stable form uses `x = 1 − e`: `n_x(x) = dt · A₀E* · (1−x)/x` (`:206`).
  Derivative `dn_e/dvₙ = −dt·A₀E* / (1−e)² / v_δ` (`:195`). Dissipation enters as `(1 − d·vₙ)`.

- **Near-rigid analytic continuation** (`UpdateTimeStep`, `:142–192`). Compare the local elastic
  stiffness `k_lin = A₀E*/(2δ)` to the near-rigid stiffness `k_nr = (m/ε)/dt²`; ratio
  `r = k_lin/k_nr`. Working in `x = 1 − e`, the transition point is `x_nr = min(1, √r)`. For
  `e + x_nr ≥ 1` (i.e. `x ≤ x_nr`, close to the true surface) the impulse/derivative switch to a
  **linear** law with slope `−m/ε` (`n_e_tilde`/`dn_e_dv_tilde`, `:211–226`). The near-rigid
  effective compliance is `m/ε = 4π²/(w·β²)` with `w ≈ 2.65/m` (spherical-body Delassus
  approximation) and `β` the tuning parameter (`:663–666`). The change of variable is what makes
  the transition robust for very small `dt` or very small contact area.

- **Analytic antiderivatives** (define the ICF cost contribution `−N`):
  elastic `calc_N_e` uses `log(1 − a − b·v)` (`:265`); stable `calc_N_x` uses `log(x)` (`:289`);
  linear regime `calc_N_linear` is a cubic polynomial in `v` (`:241`). `N_bias` is added so the
  potential is **C¹ continuous** across the transition (`:165`). For `v ≥ v̂ = min(vx, vd)` the
  impulse is clamped to zero and `N` is the constant `N_v_hat` (`CalcLogBarrierQuantities`, `:319`),
  guaranteeing no adhesion and `(1 − d·vₙ) ≥ 0`.

- **ICF assembly per pair** (`CalcLaggedLogBarrierModel`, `:927`):
  cost `μ·vt_soft·n0 − N`; impulse `γ = −μ·t̂·n0 + n·n̂`; Hessian
  `G = μ·n0/(vt_soft+vs)·M − dn_dvn·Pn` with `M = I − Pt − Pn`. Contributions are shifted to the
  body origin and accumulated in `CalcPatchQuantities`/`AccumulateGradient`/`AccumulateHessian`.

### Vertex quadrature (equivalence to centroid quadrature)
`icf_builder.cc` builds a **barycentric dual mesh**: each contact polygon is split at its centroid
into triangles, and each vertex accumulates the area `Ae` of the sub-triangles touching it plus
area-weighted normal and `δ`. Because `Σ Ae` equals the polygon area, the discrete integral
computed at the vertices equals the centroid-quadrature integral at `t₀`. Each vertex then calls
`patches.SetPairLogBarrier(..., Ae·E_star, e0, delta)`.

### Extent field and the "factor of 2"
Extruded barrier meshes (`hydroelastic_internal.cc`, sphere ~471, box ~537; commit `6bad880c51`)
set **true-surface** vertices to extent `2.0` and **inflated-boundary** vertices to
`−2·margin/barrier`. This is a scaled version of the conceptual `e=1`/`e=0` framing and interacts
with `v_δ = 2δ/dt`. **This factor of 2 should be reconciled and stated explicitly in the paper.**

---

## 3. Integrator control flow (`CenicIntegrator::DoStep`, `cenic_integrator.cc:138`)

1. Snapshot all geometry poses `X_WGs_prev` at `t₀`; solve the full ICF step → `x_next_full_`.
2. **Fixed-step mode:** accept the full step. **Error-control mode:**
   1. Set state to the full-step result and check `IsFeasibleTrajectory(prev, next)`. If infeasible,
      restore `t₀`, set `time_of_impact_`, and `return false` (`:230–237`).
   2. **Reset state to `t₀`** before the half-steps (bug fix `7732f3f999`, `:239–241`).
   3. Half-step 1 from `t₀` (reuses constraints/geometry); feasibility check vs `t₀` poses (`:267`).
   4. Half-step 2 from `t₀ + h/2` (rebuilds the model); feasibility check vs `h/2` poses (`:302`).
   5. Error estimate = `x_next_full_ − x_next_half_2_`.
3. `IsFeasibleTrajectory` (`:448`): if `use_toi`, call `FeasibilityTimeOfImpact` and set
   `time_of_impact_factor_ = toi ∈ [0,1]`; otherwise return the boolean feasibility.
4. **Step shrink on reject:** full `= toi·h·0.95`; half-1 `= toi·0.5h·0.95`;
   half-2 `= 0.5h·(1 + toi·0.95)`. `ComputeAdjustedStepSize` returns `time_of_impact_` only when
   `use_toi`, else bisects to `0.5·h` (`cenic_integrator.h:177`).

The feasibility queries flow `CenicIntegrator → QueryObject → GeometryState → ProximityEngine →
FeasibilityCalculator`.

---

## 4. Linear CCD (`ccd.cc`, `feasibility_calculator.cc`)

For each candidate primitive pair, both `point_triangle_ccd` and `edge_edge_ccd`:
1. Center all 8 endpoints at their mean and scale to unit radius (numerical conditioning).
2. Build the **coplanarity cubic** `A(t)·(B(t)×C(t)) = p₀ + p₁t + p₂t² + p₃t³` from the scalar
   triple product (`cubic`, `:294`).
3. Solve the real roots in `[0,1]` with `cubic_real_roots`.
4. For the earliest root, confirm an actual hit: `is_coplanar_point_inside_triangle` (barycentric,
   tol 1e-8) or `are_coplanar_edges_intersecting` (closest-point params in `[0,1]` **and**
   `dist < 1e-8`). Return the earliest `toi`.

Broadphase uses three `DynamicBvh` (vertex / edge / face) with moving AABBs and
`GetMovingCollisionCandidates`. The feasibility driver checks vertex-of-A vs faces-of-B,
vertex-of-B vs faces-of-A, and edge-edge — the complete set for triangle-mesh CCD.

**Root finder** (`math/real_roots.cc`): `cubic_real_roots` scales coefficients by a power of two,
handles `a==0`/`d==0` degeneracies, uses the trig method for three real roots and Cardano for one,
then one Halley/Newton polishing step. A more robust bracketing variant
`cubic_real_roots_interval` (Newton-with-bisection over derivative-sign intervals) exists but is
**not currently used**.

**Already-fixed bugs in this history** (do not re-introduce): wrong `delta_v2` reconstruction
(`b11e72942e`); degenerate fake-roots + point-in-triangle/edge tolerances (`4152cbda4b`);
scaled-vs-unscaled reconstruction (`2d36711250`); edge-BVH sized by `num_vertices` instead of
`num_edges` (`cd45c820ff`).

---

## 5. Parameters

- `IcfSolverParameters::beta{0.1}` — near-rigid transition tuning (`ε = β²/(4π²)`).
- `IcfSolverParameters::use_toi{false}` — time-of-impact step selection vs. plain bisection.

---

## 6. Code review — prioritized findings

Ordered by importance. These are the cleanup starting points; none are addressed in this document.

### Soundness (paper-critical)
- **S1 — Linear-trajectory vs. rotation gap.** Vertices are interpolated linearly between
  `X_WG0`/`X_WG1` (`feasibility_calculator.cc`), but a rotating rigid body's vertices trace arcs;
  the chord under-sweeps, so fast rotation within a step can pass through undetected. This is the
  key qualifier on the non-penetration "guarantee" and must be characterized or bounded for the
  paper (e.g. subdivide by rotation angle, or a conservative-advancement argument).

### Correctness / robustness
- **C1 — Near-zero cubic leading coefficient → missed collisions.** `cubic_real_roots`
  (`real_roots.cc:112`) demotes to quadratic only on **exact** `a == 0`. After scaling by the max
  coefficient, a leading coefficient ~1e-16 relative to others ~1 is still solved as a cubic →
  catastrophic cancellation and lost real roots ⇒ potential passthrough. The robust bracketing
  solver `cubic_real_roots_interval` (`:200`) already exists but is unused — route CCD through it,
  or add relative-magnitude degeneracy handling.
- **C2 — Global coplanarity cutoff.** `abs(f[i]) ≤ c_tol` (1e-14) for all four coefficients →
  `return false` (no collision) in both CCD functions (`ccd.cc:385,487`). A genuinely tiny /
  near-coplanar sweep is then reported feasible, missing grazing or continuous-contact collisions.
- **C3 — Wrong variable in root polishing.** `real_roots.cc:181` tests `!isfinite(r)` where `r` is
  the constant ratio `d/a`, not the loop root `x`. Harmless today (NaN arithmetic stays NaN) but
  wrong in intent. Also, polishing runs **after** the ascending sort and can nudge ordering — take
  the min (or re-sort) if strict earliest-TOI ordering is required.
- **C4 — Degenerate scale factor.** `s = 1/max(norms)` in both CCD functions is `inf` if all eight
  points coincide with their centroid; unguarded ⇒ NaN. Add a zero-extent early-out.

### Numerical stability (lower)
- **N1 — Near-singular 2×2 solves.** `is_coplanar_point_inside_triangle`,
  `are_coplanar_edges_intersecting`, and `ClosestPointEdgeToEdge` use `ldlt()` / exact `denom != 0`
  with absolute 1e-8 tolerances; nearly parallel or degenerate inputs can yield unreliable results.
  Mitigated by the scaling step but the tolerances remain absolute.
- **N2 — Magic epsilons.** `+1.0e-20` in `vd`/`vx` (`patch_constraints_pool.cc:150,411,416`) and the
  `1e-8`/`1e-14` constants throughout CCD — document/justify, ideally derive from problem scale.

### Cleanliness / dead code / efficiency (do before paper or upstream)
- **X1 — Debug artifacts in hot paths.** `UpdateTimeStep` (`:171–191`) and
  `CalcLaggedLogBarrierModel` (`:958–979`) run large `isnan` / `dn_dvn > 0` blocks with `fmt::print`
  and `throw std::logic_error("asdf")` / `runtime_error` on **every solve**. Remove or guard behind
  a debug flag.
- **X2 — Hardcoded model selection.** `CalcPatchQuantities:1098` hardcodes the log-barrier model
  ("Later add a parameter"); the Hunt-Crossley path (`CalcLaggedHuntCrossleyModel`,
  `CalcDiscreteHuntCrossley*`) is dead on this branch. Parameterize or remove.
- **X3 — Hot-path `DRAKE_DEMAND`.** `calc_N_x:295` asserts `|e0 − v/v_δ + x − 1| < 1e-14` on every
  call; downgrade to `DRAKE_ASSERT` or drop.
- **X4 — Commented-out `fmt::print` debugging** scattered in `DoStep`/`ComputeAdjustedStepSize`.
- **X5 — Confusing dead arithmetic.** `time_of_impact_ = time_of_impact_factor_ * h * 0.95` evaluates
  `inf · h · 0.95` when `use_toi` is false (factor defaults to ∞). Harmless (gated by `use_toi` in
  `ComputeAdjustedStepSize`) but confusing; make the toi vs. bisection paths explicit.
- **X6 — Per-step map copies.** `GetAllGeometryPosesInWorld()` returns a const-ref but callers copy
  into value `unordered_map`s (`X_WGs_prev`/`next`) twice per step; reuse buffers.
- **X7 — Repo hygiene.** Experiment artifacts are committed at the repo root (`rolling_sphere_data/`,
  `*.m`, `*.sh`, `VERY_IMPORTANT_CASE.txt`, `count.sh`, `doall.m`, `volumetric_vs_surface.m`, …) plus
  example assets. Segregate / `.gitignore` before any upstreaming.

## 7. Findings status (2026-08-05 CCD robustness campaign)

The campaign recorded in `project_plan/docs/CCD_ROBUSTNESS_PLAN.md` ("Implementation outcome"
section there has the details) resolved:

- **S1 — mitigated.** Rotation-adaptive conservative subdivision in `ProximityEngine`
  (`N = ceil(θ/θ_max)` slerp sub-segments over broadphase + narrowphase, default `θ_max = π/2` via
  `IcfSolverParameters::ccd_max_substep_rotation`). Documented residual limits: endpoint-pose θ
  caps at π (aliasing), and a ψ-substep chord under-sweeps radially by cos(ψ/2).
- **C1 — fixed.** Relative leading-coefficient demotion (threshold 1e-7, empirically derived) in
  `cubic_real_roots`; more importantly the CCD kernels now use the hardened bracketing solver
  `cubic_real_roots_interval` exclusively, which is structurally immune.
- **C2 — fixed.** The all-coefficients-tiny early-out samples containment at t ∈ {0,¼,½,¾,1}
  (edge-edge via robust clamped closest-point distance) instead of reporting "feasible".
- **C3 (`isfinite(r)` vs `x`), C4 (coincident-points ∞ scale) — fixed** (plus `std::abs`,
  boundary-tolerant root acceptance, polish-order, `ilogb(0)`).
- **X2 — fixed** (per-patch Hunt-Crossley vs log-barrier dispatch; this was also the root cause of
  the `BallOnTable.RollingContact` NaN: point-contact fallback pairs evaluated the barrier model on
  never-initialized slots).
- **X5, X6 — fixed** (per-step toi reset + use_toi-gated computation; participant-only zero-copy
  pose handling via the new `*ToCurrent` query variants).
- The §4 hard-coded dissipation is now sourced from
  `DefaultProximityProperties::hunt_crossley_dissipation`; the extent-field factor-of-2 is
  documented (not renormalized — not behavior-neutral) at its construction sites.

Still open from §6: X1 (debug artifacts in hot paths — the prints/throws remain, one now prints
values before the `calc_N_x` DEMAND), X3 (hot-path DEMAND), X4, X7, and the numerical-stability
items of §"Numerical stability (lower)" not covered above. The safety-critical narrowphase now has
assertive unit, adversarial, differential (vs fcl), and engine-level test suites; see
`geometry/proximity/test/ccd_*` and `geometry/test/proximity_engine_ccd_test.cc`.
