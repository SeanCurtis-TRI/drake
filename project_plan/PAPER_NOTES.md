# Paper-Strengthening Notes — bCENIC Thin Objects

_Companion to ANALYSIS.md. These are forward-looking notes on making the
paper stronger: CCD/TOI algorithm alternatives, what a state-of-the-art
comparison would require, weaknesses to preempt, and proposed additional
experiments. No third-party software was run for this round._

## 1. The linear-CCD approximation, quantified (E6a)

The production feasibility check assumes every mesh vertex moves in a
straight line between step endpoints. The kinematic study
(`examples/integrators:rotational_ccd_study`) measured, on a 24×24 (ω, dt)
grid with a 64-substep subdivided ground truth:

- **576 cells, 175 with a true collision; the linear check caught 158 and
  missed 17 (≈10% of colliding cells).**
- Every false negative occurred at per-step rotation **θ = ω·dt ≥ 5.6 rad**
  (approaching/exceeding a full revolution) — the aliasing regime where the
  endpoints straddle the obstacle. For θ ≤ ~π the linear check caught every
  collision in this scene.
- Mean linear query cost: **≈3–6 µs** per pair-set query; an N-substep
  subdivided query costs ≈ 5.5 + 1.9·N µs (measured ladder, linear in N).

**Implications for the paper.**
1. The safe-operating envelope is statable: reject (or subdivide) any step
   whose fastest body rotates more than a bounded angle θ_max (data
   supports θ_max ≈ π/2 with wide margin in this scene; an adversarial
   scene could tighten it — see §4).
2. Curved CCD by conservative subdivision is *cheap*: even N=64 costs
   ~127 µs per query, and the campaign's runtime breakdown shows CCD
   feasibility is a minor share of total runtime (see ANALYSIS.md
   "Code-change suggestions" — mean share across runs is small compared to
   the convex solve). A rotation-adaptive N (N = ceil(θ/θ_max)) would make
   the check sound w.r.t. rotation at negligible cost for the common
   small-θ case: **N=1 whenever θ ≤ θ_max, so the common path is unchanged.**
3. Error control already suppresses large θ in practice (accuracy rejects
   wild steps before CCD sees them) — worth stating, but not relying on:
   fixed-step mode and loose accuracies do reach the unsafe regime (E6b's
   ω=128 rad/s runs show heavy CCD rejection activity).

## 2. CCD / TOI algorithm alternatives

> **STATUS (2026-08):** items 1 and 2 below are implemented (plus the Tier-0
> numerical fixes, X5/X6, the §4 dissipation item, and P1/P2 from §5); see
> `docs/CCD_ROBUSTNESS_PLAN.md` → "Implementation outcome" for details and
> the documented residual limitations (θ > π pose aliasing; cos(ψ/2)
> chord under-sweep per substep).

Ranked by implementation effort against expected payoff:

1. **Rotation-adaptive conservative subdivision** (lowest effort, uses the
   existing linear kernel unchanged). Split [t, t+h] into N = ceil(θ/θ_max)
   substeps with interpolated poses (slerp on rotation). Cost model measured
   in E6a. Sound up to the subdivision bound; no new narrowphase math.
2. **Bracketed root-finding on the coplanarity cubic.** `math/real_roots.h`
   already declares `cubic_real_roots_interval` (bracketing variant) but
   nothing uses it. The design doc (barrier_cenic_thin_objects.md, C1/C2)
   flags two robustness holes in the current closed-form path: the leading
   coefficient is only demoted on exact `a == 0`, and a global coplanarity
   early-out (`c_tol = 1e-14`) can skip grazing contacts. Routing the
   narrowphase through the bracketing solver (with interval arithmetic on
   the endpoints) addresses both without changing the linear-trajectory
   model itself.
3. **Conservative advancement (Mirtich-style)** on the screw motion:
   advance each pair by a guaranteed-safe fraction using bounds on relative
   velocity and rotation. More invasive (needs distance queries per
   advance), but yields a true TOI for the *curved* motion — would upgrade
   `use_toi` from "linear TOI × 0.95 heuristic" to a certified bound.
4. **ACCD (additive CCD, Li et al. 2021)** as used by IPC-class methods:
   robust for degenerate/parallel cases, well documented, and reviewers
   will ask how the method compares. Worth implementing only if reviewers
   push on soundness; the subdivision route (1) plus (2) covers the same
   ground more cheaply in this framework.
5. **TOI usage note (E5/E6b data).** With the barrier layer, TOI-based step
   adjustment is an *optimization*, not a correctness requirement — the
   feasibility gate already guarantees non-penetration. E5 measures that
   `use_toi` does not collapse dt for rolling/sliding (parallel) motion;
   quote those numbers when defending the TOI heuristic.

## 3. What a compelling SOTA comparison needs (future round)

| Baseline | What it shows | Practical notes |
|---|---|---|
| **Volumetric hydroelastic + CENIC** (in-repo: barrier=0) | The thin-layer model's cost/benefit inside the *same* solver | Already collected as E3b (barrier=0 rows); the fairest ablation and immune to "different-stack" criticism. The volumetric fallback for `Mesh` now exists (.vtk tet meshes, extent field), and E7 (hero demo) uses it for the barrier-vs-volumetric RTR comparison. |
| **Discrete-time SAP/ICF (Drake)** | Error control vs fixed-step industry default | Same plant/geometry; run `--integrator=discrete` in error_control_demo. Cheap to add — recommend including in the next data round. |
| **IPC / Codimensional IPC** | The academic gold standard for non-penetration on codim geometry | Different stack (FEM, elastic bodies); compare on *rigid* scenes (nut&bolt, spinning plate). Expect IPC slower but exact; the pitch is "convex, no line-search failures, robotics-speed". Use their public repos + our meshes; measure wall time, penetration, energy behavior. |
| **ABD (Affine Body Dynamics)** | Rigid-body IPC-family baseline; the planning doc's Demo 4 target | Nut & bolt is the canonical scene. Need mesh export + parameter matching protocol (friction, restitution none, same initial state, same tolerance-to-what metric — success rate + wall time). |
| **MuJoCo (elliptic friction, penalty)** | Robotics-mainstream reference | Fast but penetrating; report penetration depth vs speed honestly. MJCF scenes already exist for two examples (ball_on_table.xml, clutter.xml). |

Comparison-protocol notes: fix a *task-level success metric* per scene (nut
travel distance, pile settle energy, plate rest state), report
wall-clock-to-success at matched accuracy rather than per-step cost, and
report penetration statistics (bCENIC: zero by construction — that row of
the table is the headline).

## 4. Weaknesses to preempt in the paper

- **S1 (rotation gap)** — addressed quantitatively by E6a; propose
  rotation-adaptive subdivision (§2.1) as the mitigation, with its measured
  cost. Consider one adversarial scene (long thin rod, obstacle near the
  rotation axis) where θ_max must be smaller, to show the bound is
  scene-dependent but boundable via body-frame geometry radius.
- **The "factor of 2" in the extent field** (surface vertices set to
  e = 2.0, `surface_epsilon = −2·margin/barrier`): reconcile the code with
  the paper's e ∈ [0, 1] formulation before submission — reviewers reading
  the artifact will find it (barrier_cenic_thin_objects.md flags this).
- **Rigid-only, double-only**: state clearly; codim IPC handles deformables.
- **Mesh requires barrier > 0** (`hydroelastic_internal.cc` DRAKE_DEMANDs
  it): either implement the volumetric fallback for closed meshes or state
  the restriction.
- **Dissipation hard-coded** in the log-barrier patch path
  (`kDefaultDissipation = 50.0` in icf_builder.cc): make it a proximity
  property before the artifact release; it silently overrides user d values
  in the mesh-barrier path.
- **Friction is regularized** (vt_soft): quantify creep on an inclined-rest
  scene (proposed experiment P3 below) so the static/dynamic friction claim
  has a measured error bar.

## 5. Proposed additional experiments (beyond this round)

- **P1 — Washer stack** (planning-doc candidate): N thin washers stacked
  coaxially, near-coplanar contact everywhere; the conditioning-brutal case
  for the barycentric dual quadrature. Meshes are trivial solids of
  revolution; success = stable stack + conditioning trend vs N.
- **P2 — Thin sheet through a slot**: sheet pulled through a slot with
  clearance ≈ 2·(margin+barrier); stresses tangential sliding of two
  barrier layers. Measures max pressure and CCD activity during insertion.
- **P3 — Inclined-plane friction creep**: thin plate resting on a ramp just
  below the static-friction angle; measure drift velocity vs vt_soft
  regularization → the quantitative static-friction claim.
- **P4 — Adversarial rotation scene** for E6a: thin rod, obstacle near the
  rotation axis (small radius → large angular false-negative window);
  produces the paper's "why subdivision bound depends on geometry" figure.
- **P5 — Energy audit of the barrier layer**: drop test with restitution 0;
  measure energy injected/dissipated by the barrier vs δ and β (reviewers
  from the IPC community will ask about the barrier's work).
- **P6 — Discrete vs error-controlled on identical scenes** (cheap,
  in-repo): `--integrator=discrete` at several fixed dt vs cenic at several
  accuracies; the wall-clock-vs-quality Pareto plot that motivates error
  control (legacy volumetric_vs_surface.m did exactly this by hand —
  regenerate with the driver).

## 6. Figure inventory status vs the planning docs

Covered this round: Demo 1 (E1 + method figures), Demo 3 (E3a–d), Demo 4
(E4), Demo 5 (E6a/E6b), rolling-sphere/TOI (E5), thin-objects-in-the-wild
capability runs (E2), method-section figures (continuation, stiffness cap,
N/n/dn). Not covered: Demo 2's *curated* cluttered-bin hero scene (needs
scene authoring/art pass — current E2 scenes are the physics stand-ins),
barycentric-dual-vs-naive quadrature comparison (needs a naive-quadrature
code path to compare against; currently only the dual exists), and all
third-party baselines (§3).
