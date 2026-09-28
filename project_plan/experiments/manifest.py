"""Experiment manifest for the thin-objects (bCENIC) paper campaign.

Each run is a dict:
    experiment: experiment key, becomes data/<experiment>/
    run_id: short parameter slug, becomes data/<experiment>/<run_id>/
    kind: "demo" (error_control_demo.py), "clutter" (clutter.cc),
          "ccd" (rotational_ccd_study.cc),
          "hero" (hero_demo/convex_integrator_playback.py)
    args: extra command-line args (demo/ccd) — the driver adds output paths
    overrides: nested dict merged into config.yaml (clutter only)
    tier: "A" | "B" | "C" — run order (A first; a usable report exists after A)
    timeout: per-run timeout in seconds

See project_plan/README.md for how to (re)run.
"""


import json
from pathlib import Path

_WILD_MESH_DIR = Path(__file__).resolve().parents[1] / "data" / \
    "thingi10k_meshes"


def _wild_mesh_files() -> list:
    """Absolute paths of the curated Thingi10K OBJs (E3e), or [] when the
    mesh set has not been generated yet (see thingi10k_meshes.py)."""
    manifest = _WILD_MESH_DIR / "manifest.json"
    if not manifest.exists():
        return []
    meshes = json.loads(manifest.read_text())["meshes"]
    return [str(_WILD_MESH_DIR / m["obj"]) for m in meshes]


def _slug(value) -> str:
    """Formats a float as a compact slug token, e.g. 0.01 -> '1e-2'."""
    if isinstance(value, bool):
        return "on" if value else "off"
    if isinstance(value, int):
        return str(value)
    return f"{value:g}"


def _tier_for_accuracy(acc: float) -> str:
    if acc >= 1e-1:
        return "A"
    if acc >= 1e-2:
        return "B"
    return "C"


def build_manifest() -> list:
    runs = []

    # ------------------------------------------------------------------
    # E1: Barrier & conditioning microbenchmark (planning-doc Demo 1).
    # Single sphere on a table; sweep the beta regularization and accuracy.
    for beta in [0.25, 0.5, 1.0, 2.0, 4.0]:
        for acc in [1e-1, 1e-2, 1e-3]:
            runs.append(dict(
                experiment="E1_ball_beta",
                run_id=f"beta{_slug(beta)}_acc{_slug(acc)}",
                kind="demo",
                args=["--example=ball_on_table", "--sim_time=3",
                      f"--beta={beta}", f"--accuracy={acc}"],
                tier=_tier_for_accuracy(acc),
                timeout=15 * 60,
            ))

    # ------------------------------------------------------------------
    # E2: Thin objects in the wild (Demo 2 capability runs).
    e2_cases = [
        ("plate_and_spatula", []),
        ("cones", []),
        ("teddy_and_torus", []),
        ("sphere_and_spiral", ["--sim_time=10"]),
    ]
    for example, extra in e2_cases:
        for acc in [1e-1, 1e-2]:
            runs.append(dict(
                experiment="E2_wild",
                run_id=f"{example}_acc{_slug(acc)}",
                kind="demo",
                args=[f"--example={example}", f"--accuracy={acc}"] + extra,
                tier=_tier_for_accuracy(acc),
                timeout=45 * 60,
            ))
    runs.append(dict(
        experiment="E2_wild",
        run_id="plate_and_spatula_acc1e-3",
        kind="demo",
        args=["--example=plate_and_spatula", "--accuracy=1e-3"],
        tier="C",
        timeout=45 * 60,
    ))

    # ------------------------------------------------------------------
    # E3a: Clutter beta characterization (Demo 3), 4 piles x 5 = 20 objects.
    for beta in [0.1, 0.25, 0.5, 1.0, 2.0, 4.0]:
        for acc in [1e-1, 1e-2, 1e-3]:
            runs.append(dict(
                experiment="E3a_clutter_beta",
                run_id=f"beta{_slug(beta)}_acc{_slug(acc)}",
                kind="clutter",
                overrides={
                    "simulator_config": {"accuracy": acc},
                    "icf_solver_config": {"beta": beta},
                },
                tier=_tier_for_accuracy(acc),
                timeout=60 * 60,
            ))

    # E3b: Clutter barrier sweep (barrier=0 recovers volumetric hydro for
    # the sphere/box primitives).
    for barrier in [0.0, 1e-5, 1e-4, 5e-4, 1e-3]:
        for acc in [1e-1, 1e-2, 1e-3]:
            runs.append(dict(
                experiment="E3b_clutter_barrier",
                run_id=f"barrier{_slug(barrier)}_acc{_slug(acc)}",
                kind="clutter",
                overrides={
                    "simulator_config": {"accuracy": acc},
                    "scene_graph_config": {
                        "default_proximity_properties": {"barrier": barrier},
                    },
                },
                tier=_tier_for_accuracy(acc),
                timeout=60 * 60,
            ))

    # E3c: Clutter stiffness x margin at fixed accuracy 1e-2, beta 1.
    for E in [1e5, 1e7, 1e9, 1e11]:
        for margin in [1e-4, 5e-4, 1e-3]:
            runs.append(dict(
                experiment="E3c_clutter_E_margin",
                run_id=f"E{_slug(E)}_margin{_slug(margin)}",
                kind="clutter",
                overrides={
                    "simulator_config": {"accuracy": 1e-2},
                    "icf_solver_config": {"beta": 1.0},
                    "scene_graph_config": {
                        "default_proximity_properties": {
                            "hydroelastic_modulus": E,
                            "margin": margin,
                        },
                    },
                },
                tier="B",
                timeout=30 * 60,
            ))

    # E3d: Clutter scaling (40 and 100 objects), plus box scenes.
    for opp, acc in [(10, 1e-1), (10, 1e-2), (25, 1e-1), (25, 1e-2)]:
        runs.append(dict(
            experiment="E3d_clutter_scaling",
            run_id=f"opp{opp}_acc{_slug(acc)}",
            kind="clutter",
            overrides={
                "simulator_config": {"accuracy": acc},
                "clutter_config": {"objects_per_pile": opp},
            },
            tier="C",
            timeout=120 * 60,
        ))
    for acc in [1e-1, 1e-2]:
        runs.append(dict(
            experiment="E3d_clutter_scaling",
            run_id=f"boxes_opp10_acc{_slug(acc)}",
            kind="clutter",
            overrides={
                "simulator_config": {"accuracy": acc},
                "clutter_config": {
                    "objects_per_pile": 10,
                    "enable_boxes": True,
                },
            },
            tier="C",
            timeout=120 * 60,
        ))

    # ------------------------------------------------------------------
    # E3e: Clutter of "objects in the wild" (Thingi10K meshes). Same
    # numerical characterization as E3a but with 10 real meshes (4 piles x 3
    # objects = 12 bodies; the 10 curated meshes cycle so two appear twice).
    # x axis of the report plots is accuracy, one line per beta.
    #
    # 4x3 rather than 2x5: with 5-object stacks the top mesh falls ~0.6 m
    # and the impact chaotically pierces the barrier layer in some grid
    # cells even with barrier=1e-3 + TOI (extent field e0 -> 2). The lower
    # stacks (~0.38 m max fall) carry the impact in every screened cell.
    #
    # N.B. Unlike E3a, these runs use use_toi=true and barrier=1e-3:
    # with the E3a defaults (use_toi=false, barrier=1e-4) the pile impact
    # at t~0.25-0.33s drives contact points through the thin extruded
    # layer (extent field e0 -> 2, the rigid-core/log-barrier singularity)
    # and the runs abort with dt collapsed below the 1e-14 minimum. The
    # thicker 1e-3 layer (project-plan design range delta ~ 0.1-1 mm)
    # plus the TOI step limiter carries the impact for beta >= 0.01;
    # beta=0.001 still hits Hessian factorization failures at impact
    # (conditioning ~1/beta^2), which is reported as data.
    wild_meshes = _wild_mesh_files()
    if wild_meshes:
        for beta in [0.001, 0.01, 0.1, 1.0]:
            for acc in [1e-1, 1e-2, 1e-3, 1e-4]:
                timeout = {1e-3: 4 * 60 * 60, 1e-4: 8 * 60 * 60}.get(
                    acc, 2 * 60 * 60)
                runs.append(dict(
                    experiment="E3e_clutter_wild",
                    run_id=f"beta{_slug(beta)}_acc{_slug(acc)}",
                    kind="clutter",
                    overrides={
                        "simulator_config": {"accuracy": acc},
                        "icf_solver_config": {"beta": beta,
                                              "use_toi": True},
                        "scene_graph_config": {
                            "default_proximity_properties": {
                                "barrier": 1e-3,
                            },
                        },
                        "clutter_config": {
                            "mesh_files": wild_meshes,
                            "num_piles": 4,
                            "objects_per_pile": 3,
                        },
                    },
                    tier=_tier_for_accuracy(acc),
                    timeout=timeout,
                ))

    # ------------------------------------------------------------------
    # E4: Nut & bolt (Demo 4). Known-good parameters from the example's
    # header comment.
    for acc in [1e-1, 1e-2, 1e-3]:
        for beta in [0.5, 1.0, 2.0]:
            runs.append(dict(
                experiment="E4_nut_and_bolt",
                run_id=f"acc{_slug(acc)}_beta{_slug(beta)}",
                kind="demo",
                args=["--example=nut_and_bolt", "--tau=-0.005",
                      "--barrier=2e-5", "--margin=1e-5", "--E=1e8", "--d=50",
                      f"--accuracy={acc}", f"--beta={beta}"],
                tier=_tier_for_accuracy(acc),
                timeout=60 * 60,
            ))

    # ------------------------------------------------------------------
    # E5: TOI vs parallel motion (rolling sphere / sphere down spiral).
    # The interesting claim: TOI-based step adjustment must not collapse dt
    # for motion parallel to the contact surface.
    for example, extra in [("ball_on_table", []),
                           ("sphere_and_spiral", ["--sim_time=10"])]:
        for use_toi in [False, True]:
            for acc in [1e-1, 1e-2, 1e-3]:
                toi_args = ["--use_toi"] if use_toi else []
                runs.append(dict(
                    experiment="E5_toi",
                    run_id=(f"{example}_toi{_slug(use_toi)}"
                            f"_acc{_slug(acc)}"),
                    kind="demo",
                    args=[f"--example={example}", f"--accuracy={acc}"]
                         + extra + toi_args,
                    tier=_tier_for_accuracy(acc),
                    timeout=60 * 60,
                ))

    # ------------------------------------------------------------------
    # E6a: Rotational CCD kinematic study (Demo 5; linear-CCD rotation gap).
    runs.append(dict(
        experiment="E6a_ccd_kinematic",
        run_id="grid24",
        kind="ccd",
        args=["--grid_size=24", "--truth_substeps=64"],
        tier="A",
        timeout=30 * 60,
    ))

    # E6b: Rotational CCD dynamic companion: spinning thin plate falling
    # onto a table under error control.
    for omega in [2.0, 8.0, 32.0, 128.0]:
        for use_toi in [False, True]:
            toi_args = ["--use_toi"] if use_toi else []
            runs.append(dict(
                experiment="E6b_spinning_plate",
                run_id=f"w{_slug(omega)}_toi{_slug(use_toi)}",
                kind="demo",
                args=["--example=spinning_plate", f"--initial_w={omega}",
                      "--accuracy=1e-2"] + toi_args,
                tier="B",
                timeout=30 * 60,
            ))

    # ------------------------------------------------------------------
    # E7: Hero demo (dishrack playback) — barrier hydro vs volumetric hydro.
    # Full 100 s dual-Panda dishrack-loading playback. barrier=0 + margin=0
    # recovers Drake's default volumetric hydroelastic (the .vtk tet meshes
    # with distance-based extent fields); barrier > 0 is the thin-layer
    # barrier hydro this paper is about. Volumetric needs accuracy 1e-3 to
    # be artifact-free with the thin objects (rack wireframe, utensils).
    #
    # N.B. The first-ever hero run needs network access to download the
    # lbm_eval model wheel (cached under ~/.cache/drake afterwards).
    def _hero_run(run_id, margin, barrier, acc):
        return dict(
            experiment="E7_hero_dishrack",
            run_id=run_id,
            kind="hero",
            args=[f"--accuracy={acc}", f"--margin={margin}",
                  f"--barrier={barrier}", "--sim_time=100"],
            tier=_tier_for_accuracy(acc),
            timeout={1e-1: 4, 1e-2: 8}.get(acc, 12) * 60 * 60,
        )

    # Barrier hydro: accuracy sweep at the default margin/barrier.
    for acc in [1e-1, 1e-2, 1e-3]:
        runs.append(_hero_run(
            f"barrier{_slug(1e-4)}_margin{_slug(2e-4)}_acc{_slug(acc)}",
            2e-4, 1e-4, acc))
    # Barrier hydro: margin x barrier grid at accuracy 1e-1 (skipping the
    # margin=2e-4/barrier=1e-4 cell already covered by the sweep above).
    for margin in [1e-4, 2e-4, 5e-4]:
        for barrier in [1e-5, 1e-4, 1e-3]:
            if (margin, barrier) == (2e-4, 1e-4):
                continue
            runs.append(_hero_run(
                f"barrier{_slug(barrier)}_margin{_slug(margin)}"
                f"_acc{_slug(1e-1)}",
                margin, barrier, 1e-1))
    # Volumetric hydro baseline: 1e-3 is the artifact-free headline; the
    # looser rows quantify the cost/artifact tradeoff.
    for acc in [1e-1, 1e-2, 1e-3]:
        runs.append(_hero_run(
            f"volumetric_acc{_slug(acc)}", 0.0, 0.0, acc))

    return runs


if __name__ == "__main__":
    manifest = build_manifest()
    from collections import Counter
    by_exp = Counter(r["experiment"] for r in manifest)
    by_tier = Counter(r["tier"] for r in manifest)
    for exp, count in sorted(by_exp.items()):
        print(f"{exp:26s} {count}")
    print(f"{'TOTAL':26s} {len(manifest)}   tiers: {dict(by_tier)}")
