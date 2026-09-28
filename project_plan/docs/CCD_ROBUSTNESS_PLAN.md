# CCD Robustness — Critique, Improvement Tiers, and Continuation Plan

> **Portable continuation doc.** Self-contained so this task can be resumed on any machine from the
> repo alone (it travels with the branch via git). No dependency on any machine-local plan file.

> **STATUS (2026-08-05): IMPLEMENTED.** The Recommendation below has been executed — Tier 0 + Tier 1
> numerical fixes, the S1 rotation-adaptive subdivision, X5/X6 integrator hygiene, the independent
> verification layer, and the PAPER_NOTES §4 adjacent items. See "Implementation outcome (2026-08)"
> at the end of this document for what landed, where it deviates from the analysis below, and the
> remaining known limitations. Line numbers in the body of this doc refer to the pre-fix anchor
> commit `22e598e5ae` and are stale for the fixed files.

## How to resume
- **Repo / branch:** `drake-private`, branch `thin_objects_rebase_merge_upstream`.
- **Anchor commit:** `22e598e5ae` (line numbers below verified against this commit). If the CCD files
  have since changed, re-verify with the greps in the "Anchors" box.
- **Task:** expert robustness review of the linear CCD used for the Barrier CENIC non-penetration
  guarantee, plus tiered improvement options. Current status: **implemented** (see the outcome
  section at the end).
- **Scope files:** `geometry/proximity/ccd.{h,cc}`, `math/real_roots.{h,cc}`.
- **Related docs:** method/review doc `multibody/cenic/barrier_cenic_thin_objects.md` (items S1, C1,
  C2, X6 referenced here); experiment interpretation `project_plan/PAPER_NOTES.md §1,§2,§4`;
  results `project_plan/ANALYSIS.md` (E6a/E6b, line ~366 and ~444).
- **Experiment driver:** `examples/integrators/rotational_ccd_study.cc`
  (target `//examples/integrators:rotational_ccd_study`); data in
  `project_plan/data/E6a_ccd_kinematic/grid24/{ccd_grid.tsv,ccd_cost.tsv}`,
  `project_plan/data/E6b_spinning_plate/`; plots in `project_plan/plots/E6a_ccd_kinematic/`,
  `project_plan/plots/E6b_spinning_plate/`.

```
Anchors (re-run if code moved):
  grep -n "abs(f\[0\])\|const double s = 1.0 / std::max\|if (r >= 0 && r <= 1)\|constexpr double c_tol" geometry/proximity/ccd.cc
  grep -n "if (a == 0)\|cubic_real_roots_interval\|if (!isfinite(r)) continue\|std::ilogbl" math/real_roots.cc
```

---

## Context

**Purpose of the query:** guarantee no triangle passes through another within a time step. Mesh
trajectories are assumed **linear per step** (vertex positions interpolated between the step-endpoint
poses). In floating point *absolute* robustness is impossible; the goal is the most robust
implementation achievable.

**Design constraints (from the author):**
- Geometry is built with a **small minimum-separation** parameter. It is **not** passed to the CCD,
  but it means the *physics* should never present truly hard degenerate cases (exactly coplanar
  triangles sliding in-plane, sustained tangential contact). CCD sees "easy" cases.
- The query is a **global CCD run every time step** → must be *very* fast.
- Several published implementations were already tried and found no better.

**Current method:** coplanarity-cubic CCD (Provot / Bridson style). For each candidate vertex-face
and edge-edge pair, form the cubic `p(t) = A(t)·(B(t)×C(t))` (the coplanarity condition), solve real
roots in `[0,1]`, and at each root run a containment / segment-segment test. Broadphase is three
moving-AABB `DynamicBvh` (vertex/edge/face) upstream in `geometry/proximity/feasibility_calculator.cc`.

**Code-state check (as of `22e598e5ae`):** the CCD/root-finding code is **byte-for-byte unchanged**
since the review was written — `git diff --stat 43208823f9..HEAD` touches nothing in `ccd.{h,cc}`,
`real_roots.{h,cc}`, `feasibility_calculator.*`. Every finding below is current and unfixed.

---

## Part 1 — Correctness assessment

### Dominant risk: the method is *non-conservative* (can miss collisions)
A collision is reported only if (a) the root finder returns a real root, (b) it survives the
`r>=0 && r<=1` gate, and (c) the containment test passes there. Each is a floating-point
approximation, and each failure is a **false negative = missed collision = passthrough** — the one
error class this query exists to prevent. This is structural to cubic-solver CCD.

### Specific issues (file:line at commit `22e598e5ae`)

**[COR-1] Near-zero *relative* leading coefficient → lost roots (fires often).**
`cubic_real_roots` demotes to quadratic only on **exact** `a==0` (`math/real_roots.cc:112`). The
scaling at `real_roots.cc:104` divides all four coefficients by the *same* power of two, so it never
rescues a leading coefficient that is tiny *relative* to the others.
- `p₃ = α·(β×γ)` is the triple product of relative per-vertex velocity *differences*. Pure
  translation → `β=γ=0 ⇒ p₃=p₂=0` exactly (handled → linear). **Fast translation + small rotation**
  gives `p₃ ~ O(ω)` tiny while `p₁,p₂ ~ O(1)`; the Cardano/trig general branch is then
  catastrophically ill-conditioned (`R/(Q√Q) → ±1`, roots dominated by `−p/3`, huge), so genuine
  `O(1)` roots in `[0,1]` are lost → **missed collision in the fast, slightly-rotating regime.**
- Fix: relative demotion (`|a| ≤ τ·max(|b|,|c|,|d|)` → treat as quadratic, cascade), or move to the
  interval solver (Tier 1).

**[COR-2] Hard `[0,1]` acceptance with no tolerance.**
`if (r >= 0 && r <= 1)` at `ccd.cc:392` (point-triangle) and `ccd.cc:495` (edge-edge). A true impact
at `t≈0`/`t≈1` returned as `−1e−12` or `1+1e−12` is silently dropped. Conservative fix: accept on
`[−εt, 1+εt]` and clamp to `[0,1]` before the containment test.

**[COR-3] Unqualified `abs()` in `ccd.cc`.**
`ccd.cc:385–388` and `ccd.cc:487–490` call `abs(f[i])` on doubles with no `using std::abs` / no
`std::` prefix (only `<cmath>`/`<algorithm>` included). If it ever binds to C `::abs(int)`, `abs(f[i])`
truncates to int and the degeneracy guard misfires (any `|f|<1 ⇒ 0 ≤ 1e-14 ⇒ "degenerate" ⇒ return
false = no collision`). libstdc++ injects a global `double` overload so it likely works today, but it
is unportable and a latent false-negative. Use `std::abs`.

**[COR-4] Degenerate scale factor → NaN then throw.**
`s = 1/max(norms)` at `ccd.cc:359` (point-triangle) and `ccd.cc:460` (edge-edge). If all 8 centered
points coincide, `max=0 ⇒ s=inf ⇒` scaled coords NaN. The all-`≤c_tol` guard does not catch NaN
(NaN comparisons false) → `cubic_real_roots` hits `DRAKE_THROW_UNLESS(isfinite(...))` → crash. Guard
the degenerate-extent case explicitly.

**[COR-5] `real_roots.cc:181` tests the wrong variable.**
`if (!isfinite(r)) continue;` uses `r = d/a`, not the loop root `x`. Only reached in the general
branch where `r` is finite, so it never skips; NaN roots then flow through NaN arithmetic (stay NaN)
— harmless today but wrong intent. Should be `!isfinite(x)`.

**[COR-6] Single post-sort polishing step.**
One Halley/Newton iteration (`real_roots.cc:179–196`) can under-converge for clustered roots and runs
*after* the sort, so it may perturb ordering. Irrelevant to the boolean feasibility path (any hit →
infeasible); minor effect on `toi` accuracy only.

**[COR-7] Near-tangent / double roots.**
Just-touching contacts are double roots; exact doubles are handled (`A==B || arg==0`) but FP gives
`arg` slightly `>0` → single-root branch → the two near-tangent roots vanish. The min-separation
assumption makes this rare (low priority) but it is the mechanism for missing grazing contacts.

### What is correct / good
- The VF(a∈B) + VF(b∈A) + EE triple is the **complete** topological set for two moving triangles.
- `cubic()` coefficients verified correct (expansion of `A(t)·(B(t)×C(t))`, `ccd.cc:294`).
- Centering + unit-scaling is the right conditioning; the earlier scaled-vs-unscaled reconstruction
  bug is already fixed.
- Containment tolerances (`1e-8`, e.g. `is_coplanar_point_inside_triangle` at `ccd.cc:211`,
  `are_coplanar_edges_intersecting` at `ccd.cc:247`) bias toward **false positives** — the safe
  direction (extra step rejections cost speed, not safety).
- `quadratic_real_roots` uses the citardauq/`copysign` form (no cancellation) — solid.
- Exact-zero degeneracies (pure translation → linear) fall through correctly.

### Test-coverage gap (blocker for any change)
`geometry/proximity/test/ccd_test.cc` is 57 lines, one test, **no assertions** — it only `fmt::print`s
results for a single edge-edge case. The safety-critical narrowphase is effectively untested.
`math/test/real_roots_test.cc` (301 lines) is decent but omits the relative-leading-coefficient case
(COR-1). Any change must land behind a real known-answer + adversarial suite.

---

## Part 2 — Improvement tiers

### Tier 0 — Correctness patches to the current algorithm (hours; easy-case behavior unchanged)
- COR-3 `std::abs`; COR-5 `r→x`; COR-4 degenerate-scale guard; COR-2 `[−εt,1+εt]` accept + clamp.
- COR-1 relative leading-coefficient demotion in `cubic_real_roots`.
- Add an assertive known-answer + adversarial test suite (see Verification).
- Keeps the fast path; removes the most probable false-negatives. Lowest risk, high value.

### Tier 1 — Robustify root isolation, same formulation (days)
- Route CCD through the already-present but **unused** `cubic_real_roots_interval`
  (`math/real_roots.cc:200`; grep confirms zero callers). It brackets by derivative sign changes and
  runs Newton+bisection within each monotone sub-interval of `[0,1]`, so it is guaranteed to locate a
  sign-change root without Cardano ill-conditioning and naturally restricts to the interval — a direct
  structural cure for COR-1/COR-2 at the isolation step.
- Make degeneracy/containment tolerances **scale-relative**, optionally tying the containment
  inflation to a fraction of the geometry min-separation so the test is conservative by construction.
- Bias every ambiguous decision toward "collision" (reject the step): safety over speed.

### Tier 2 — Conservative formulation keyed to the min-separation (1–2 weeks) — principled long-term
The physics already supplies a minimum separation; that is the natural safety margin for a method
that *never* misses:
- **Additive CCD (ACCD)** — Li et al., *IPC* 2020. No polynomial roots; iteratively lower-bounds the
  earliest time distance can close to the target gap using per-primitive velocity bounds.
  Conservative, simple, branch-light, cache-friendly, fast in practice, returns a conservative TOI.
- **Conservative Advancement** — Mirtich; Tang et al. 2009 — using the min-separation as the margin;
  also a guaranteed-safe TOI lower bound (would upgrade `use_toi` to a certified bound on the curved
  motion).
Both eliminate the false-negative class outright (modulo the separate linear-trajectory assumption)
and remove all cubic-root numerics.

### Tier 3 — Provably robust CCD (rewrite; weeks) — the reference bar
- **Tight-Inclusion CCD (TICCD)** — Wang, Ferguson et al., *ACM TOG* 2022, plus the CCD benchmark
  (Wang et al. 2021). Interval/inclusion method with explicit FP error bounds: provably **no false
  negatives**, controllable false-positive rate, runtime competitive with inexact solvers.
- **Exact** methods — Brochu et al. 2012 (geometrically exact), Wang 2014 (root parity) — exact
  yes/no but heavier; likely overkill given the min-separation.
- On "the papers didn't seem better": inclusion/exact methods often *look* worse via more false
  positives (over-rejection → smaller `dt` → slower sim) or per-query overhead — a tuning/overhead
  trade-off, not a correctness defect. Given these constraints, Tier 2 (ACCD) most likely dominates
  Tier 3 on the speed/robustness trade-off.

---

## Part 3 — Post-experiment review (E6 campaign)

The E1–E7 campaign added `examples/integrators/rotational_ccd_study.cc` and the E6 data/plots. Read
the results carefully — they change *priority*, not the findings.

**What E6a measured.** A 24×24 (ω, dt) grid; a thin square blade spins about world-z inside the swept
annulus of a fixed obstacle. For each cell it compares the production linear query against a
"curved" oracle = **the same `FeasibilityTimeOfImpact` narrowphase subdivided into 64 rotational
substeps**. Result: **17 false negatives / 576 cells (175 colliding), all at per-step rotation
θ = ω·dt ≥ 5.6 rad** (~a full revolution/step); for θ ≤ π the linear check caught every collision.
Cost ladder: `≈5.5 + 1.9·N µs`, linear in substep count N.

**Two structural caveats on how to read it:**
1. **The oracle reuses the same kernel.** Valid for the **geometric chord-vs-arc gap (S1)** —
   subdivision reconditions the geometry — but **circular w.r.t. the numerical concerns
   (COR-1…COR-7, C2)**: a blind spot shared by every sub-query (grazing/near-coplanar, or an
   ill-conditioned-cubic miss present at all scales) is inherited by the "truth" and can never
   register as a false negative.
2. **The scene is pure rotation, no translation.** COR-1's dangerous regime (fast translation + small
   rotation) is never entered; the large-θ misses are geometric aliasing.

**Net:** E6a convincingly retires practical worry about **S1 at sane step sizes** and shows the
rotation-adaptive-subdivision mitigation is cheap. It does **not** retire COR-1…COR-7 / C2 — those
remain untested and unfixed.

**Corroborated by the campaign:**
- **X6 is real and now measured.** `project_plan/ANALYSIS.md:444`: `GetAllGeometryPosesInWorld()`
  (full map copy, up to 4×/step) dominates CCD time at low contact counts. Cheap, high-value fix.
- `project_plan/PAPER_NOTES.md §2.2` independently flags C1/C2 and proposes routing through
  `cubic_real_roots_interval` — consistent with Tier 1.
- Adjacent (non-CCD) items in `PAPER_NOTES.md §4` to track: hard-coded `kDefaultDissipation = 50.0`
  in `multibody/contact_solvers/icf/icf_builder.cc` silently overriding user `d` in the mesh-barrier
  path; the extent-field "factor of 2"; `Mesh` requiring `barrier > 0`.

---

## Recommendation (revised with experiment data)

1. **S1 soundness (data-backed, do this):** implement **rotation-adaptive conservative subdivision** —
   `N = ceil(θ/θ_max)` substeps with slerp-interpolated poses, `N=1` on the common small-θ path so the
   fast path is unchanged. E6a supplies both the justification and the cost model; already scoped in
   `PAPER_NOTES.md §2.1`.
2. **Numerical (COR-1…COR-7):** still do the Tier 0 patches — cheap insurance the experiment cannot
   substitute for. **Before claiming numerical soundness in the paper, add an *independent* oracle**
   (dense static triangle-triangle intersection sampler — easiest; or exact/rational CCD; or TICCD) and
   a **translation-dominated** + **grazing/near-coplanar** scene (P1 washer / P2 slot in
   `PAPER_NOTES.md §5` are good near-coplanar starts). The same-kernel-subdivided oracle structurally
   cannot detect numerical misses.
3. **Perf:** fix X6 (pose-map copy), now the measured CCD-time bottleneck.

Prior guidance stands: **Tier 0 now**, adopt **Tier 2 (ACCD)** as the principled long-term answer;
Tier 1 is the intermediate that keeps the current formulation; Tier 3/TICCD is the "provable"
reference but more machinery than these constraints require.

---

## Verification (for whichever tier is implemented)
- Known-answer suite for `point_triangle_ccd`/`edge_edge_ccd`: clear hit (`true`, `toi≈` known),
  clear miss, `t≈0` and `t≈1` boundary hits, **fast-translation + small-rotation** (COR-1
  regression), coincident/degenerate inputs (COR-4), parallel / near-parallel edges.
- Randomized **differential test with an INDEPENDENT oracle** (not the CCD kernel subdivided):
  dense-substepped exact triangle-triangle overlap as ground truth over many random configs; assert
  **no false negatives**. This is the gap the current E6a oracle leaves open.
- Micro-benchmark queries/sec on a representative scene to confirm the "fast every step" requirement
  is preserved.
- `real_roots`: add relative-leading-coefficient cases to `math/test/real_roots_test.cc`.

---

## Implementation outcome (2026-08)

Executed as a phased campaign (one commit per phase, each independently green; commit subjects
"CCD Phase 0" … "CCD Phase 6" plus the ICF dispatch fix). Summary, including deviations from the
analysis above:

**Tier 0 (all landed).** COR-2 (accept `[-1e-9, 1+1e-9]`, clamp), COR-3 (`std::abs`), COR-4
(coincident points ⇒ `true, toi = 0`), COR-5 (`isfinite(x)`), COR-6 (polish-then-sort over the
finite prefix, every path). COR-1 landed with a **much larger demotion threshold than proposed**:
the closed-form breakage was measured to extend to `|a|/max(|b|,|c|,|d|)` ≈ 1e-7 — not ~1e-13
(e.g. `cubic_real_roots(1e-9, 1, -5, 6)` returned `{-1e9, NaN, NaN}`, losing the genuine roots 2
and 3 entirely) — so the relative demotion threshold is **1e-7**, documented in `real_roots.h`.
Also fixed along the way: unguarded `ilogb(0)`, a heap allocation per bracket solve (std::function
small-buffer overflow), and the zero-polynomial code/comment/test contradiction.

**Tier 1 (landed; CCD no longer uses the closed form at all).** `cubic_real_roots_interval` was
hardened (guards; fixed-size interval bookkeeping; **conservative boundary acceptance**: any
sub-interval boundary — interval endpoint or derivative root — with `|f| ≤ 1e-12` on the
pre-scaled coefficients is reported as a root, which catches tangential double roots / COR-7) and
both CCD kernels route through it directly on the widened step interval. **C2 fix:** the
identically-zero-polynomial early-out now samples containment/proximity at `t ∈ {0,¼,½,¾,1}`
instead of returning "no collision"; the edge-edge containment predicate was rebuilt on clamped
closest-point distance (Ericson) because the old normal-equations solve was singular exactly for
the parallel-overlap case the fallback must catch. Residual gap (documented): instantaneous
in-plane transversal crossings between samples; the principled fix remains Tier 2 ACCD.

**S1 (landed, subdivision-only by decision).** Rotation-adaptive conservative subdivision inside
`ProximityEngine` (broadphase + narrowphase per sub-segment; slerp-interpolated poses in
engine-owned scratch; toi composed `(k + toi_local)/N`), default `θ_max = π/2` via
`IcfSolverParameters::ccd_max_substep_rotation`. **Two documented limitations:** (1) pose-derived
θ ∈ [0, π] — true per-step rotations beyond π alias to 2π−θ and are under-subdivided; no
endpoint-pose method can see them (an ω-based step guard was scoped but deliberately not built).
(2) a ψ-radian sub-chord under-sweeps rotating geometry radially by cos(ψ/2) — π/2 substeps give
≈0.71·r reach — so θ_max must be tightened for obstacles requiring detection depth below
(1−cos(ψ/2))·r (bound: `θ_max ≤ sqrt(8·δ/r)` for guaranteed depth δ at radius r).

**X5/X6 (landed).** The 4 full-pose-map copies per step are gone: new
`{IsFeasibleTrajectory,FeasibilityTimeOfImpact}ToCurrent` query variants use the current geometry
state as the trajectory end (zero copy), and the trajectory-start snapshot is restricted to CCD
participants (`GetCcdParticipantGeometryIds`). Fixed-step mode skips pose work entirely.
`time_of_impact_*` reset per step. The `FeasibilityCalculator` is now a plain double class (the
per-primitive-pair conversions are gone), which also fixed a latent AutoDiffXd dangling-pointer
bug in the BVH moving-frame setup.

**Verification (the "blocker" is closed).** `ccd_test.cc` is a real known-answer + adversarial
suite; `real_roots_test.cc` covers the interval solver (previously zero tests) and the COR-1
regime; `ccd_differential_test.cc` checks constructed-truth queries (wide-margin hits/misses,
importance-biased into the COR-1 / boundary-time / extreme-scale / dyadic families) and runs a
differential comparison against **fcl's independent interval-Newton CCD** (`intersect_VF/_EE`,
linkable from `@fcl_internal//:fcl`), with strict dense-time adjudication of disagreements. Soak
at 100k trials/cell: **2.4M constructed queries, zero false negatives / positives; 2M fcl
comparisons, zero confirmed missed collisions** (19 unconfirmable fcl-side extra hits in 2M; fcl
itself fails an entire extreme-scale family that our pre-normalized kernels pass).
`geometry/benchmarking:ccd_benchmark` pins the kernel cost: ~190–200 ns/query (5.0–5.6 M
queries/s), zero steady-state allocations. Engine-level: `proximity_engine_ccd_test.cc`
proves the subdivision catches a spinning-blade collision that the N=1 chord check provably
misses, at the correct composed toi.

**Adjacent items.** `kDefaultDissipation` now reads
`DefaultProximityProperties::hunt_crossley_dissipation` (same 50.0 default, now user-controllable)
— PAPER_NOTES §4 item closed. The extent-field factor-of-2 is precisely documented at all three
construction sites (it is **not** behavior-neutral to renormalize; reconcile in the paper text).
The "Mesh requires barrier > 0" §4 item was already resolved by the `.vtk` volumetric fallback.
P1 (washer stack) and P2 (sheet-through-slot) exist as `examples/integrators:{washer_stack,
sheet_through_slot}` with first data under `project_plan/data/P{1,2}_*`.

**Also fixed (blocking bug found during verification):** the ICF pool hard-coded the log-barrier
cost for *every* patch; point-contact fallback pairs (populated with Hunt-Crossley data) evaluated
the barrier on all-zero slots ⇒ the long-standing `BallOnTable.RollingContact` NaN. Per-patch
dispatch fixed it; `cenic_barrier_thin_objects_test` is now 10/10.

## Change log
- Written from the session that produced `multibody/cenic/barrier_cenic_thin_objects.md`; ported here
  (repo-local, portable) so the task can be resumed on another machine. Line numbers verified at
  commit `22e598e5ae`. Status: analysis only — no code changes made.
- 2026-08-05: Implemented (see "Implementation outcome" above).
