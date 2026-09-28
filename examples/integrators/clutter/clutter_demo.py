import argparse
import glob
import os
import time

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ApplySimulatorConfig,
    ApplyVisualizationConfig,
    BodyIndex,
    DiagramBuilder,
    DiscreteContactApproximation,
    Parser,
    PrintSimulatorStatistics,
    RigidTransform,
    RollPitchYaw,
    SceneGraphConfig,
    Simulator,
    SimulatorConfig,
    StartMeshcat,
    VisualizationConfig,
)
from pydrake.multibody.contact_solvers import IcfSolverParameters

##
#
# Drop a collection of mesh bodies into a box container.
# Bodies are loaded from all .sdf files found in a models directory and
# arranged into 4 piles above the container before being dropped.
#
##

# 4 pile positions (x, y) inside the 0.8m x 0.8m container.
_PILE_XY = [(-0.15, -0.15), (0.15, -0.15), (-0.15, 0.15), (0.15, 0.15)]

# Vertical spacing between consecutive bodies within the same pile.
_BODY_SPACING_Z = 0.25


def create_scene(
    models_dir: str,
    time_step: float,
    meshcat,
    E: float,
    d: float,
    margin: float,
    barrier: float,
    resolution: float,
    drop_height: float,
    visualize: bool = True,
):
    """Build the Drake diagram, load all models, and return (diagram, plant, context)
    with free bodies placed into 4 piles at the requested drop height."""

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant)

    # Load the fixed container.
    container_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "container.sdf")
    parser.AddModels(container_path)

    # Discover and load every .sdf in the models directory.
    sdf_files = sorted(glob.glob(os.path.join(models_dir, "*.sdf")))
    if not sdf_files:
        print(f"Warning: no .sdf files found in {models_dir}")
    for sdf_path in sdf_files:
        print(f"  Loading {os.path.basename(sdf_path)}")
        parser.AddModels(sdf_path)

    if time_step > 0:
        plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)
    plant.Finalize()

    sg_config = SceneGraphConfig()
    sg_config.default_proximity_properties.compliance_type = "compliant"
    sg_config.default_proximity_properties.hydroelastic_modulus = E
    sg_config.default_proximity_properties.hunt_crossley_dissipation = d
    sg_config.default_proximity_properties.margin = margin
    sg_config.default_proximity_properties.barrier = barrier
    sg_config.default_proximity_properties.resolution_hint = resolution
    sg_config.default_proximity_properties.dynamic_friction = 0.5
    sg_config.default_proximity_properties.static_friction = 0.5
    scene_graph.set_config(sg_config)

    if visualize:
        vis_config = VisualizationConfig()
        vis_config.publish_period = 0.1
        ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

    # Collect all free bodies (excludes world and any welded links).
    free_bodies = [
        plant.get_body(BodyIndex(i))
        for i in range(plant.num_bodies())
        if plant.get_body(BodyIndex(i)).is_floating_base_body()
    ]
    print(f"Placing {len(free_bodies)} free body/bodies into 4 piles at drop_height={drop_height} m")

    # Distribute bodies round-robin across the 4 piles, stacking vertically.
    pile_counts = [0] * 4
    for i, body in enumerate(free_bodies):
        pile_idx = i % 4
        px, py = _PILE_XY[pile_idx]
        pz = drop_height + pile_counts[pile_idx] * _BODY_SPACING_Z
        pile_counts[pile_idx] += 1

        X_WB = RigidTransform(
            RollPitchYaw(
                np.random.uniform(0, 2 * np.pi),
                np.random.uniform(0, 2 * np.pi),
                np.random.uniform(0, 2 * np.pi),
            ),
            [px, py, pz],
        )
        plant.SetFreeBodyPose(plant_context, body, X_WB)

    return diagram, plant, context


def run_simulation(
    models_dir: str,
    integrator: str,
    accuracy: float,
    max_step_size: float,
    meshcat,
    E: float,
    d: float,
    margin: float,
    barrier: float,
    resolution: float,
    beta: float,
    drop_height: float,
    sim_time: float,
    seed: int,
    stats_file: str = None,
    visualize: bool = True,
):
    np.random.seed(seed)

    config = SimulatorConfig()
    if integrator != "discrete":
        config.integration_scheme = integrator
    config.max_step_size = max_step_size
    config.accuracy = accuracy
    config.target_realtime_rate = 0
    config.use_error_control = True

    time_step = max_step_size if integrator == "discrete" else 0.0

    diagram, plant, context = create_scene(
        models_dir=models_dir,
        time_step=time_step,
        meshcat=meshcat,
        E=E,
        d=d,
        margin=margin,
        barrier=barrier,
        resolution=resolution,
        drop_height=drop_height,
        visualize=visualize,
    )

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)

    if integrator == "cenic":
        ci = simulator.get_mutable_integrator()
        params = IcfSolverParameters()
        params.beta = beta
        params.collect_heavy_stats = stats_file is not None
        ci.SetSolverParameters(params)

    simulator.Initialize()

    if visualize:
        input("Waiting for meshcat... [ENTER] to continue")

    meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"\nWall clock time: {wall_time:.2f}s\n")
    PrintSimulatorStatistics(simulator)

    if stats_file is not None and integrator == "cenic":
        stats = simulator.get_mutable_integrator().get_step_statistics()
        with open(stats_file, "w") as f:
            f.write("step_type\ttime\tstep_size\tnum_solver_iterations\t"
                    "total_linesearch_iterations\tmax_linesearch_iterations\t"
                    "mean_linesearch_iterations\tmax_condition_number\t"
                    "last_condition_number\tmax_e0\tmean_e0\t"
                    "total_num_constraint_pairs\n")
            for stat in stats:
                f.write(stat.to_string() + "\n")
        print(f"Step statistics written to {stats_file}")

    return wall_time


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--models_dir",
        type=str,
        default=os.path.join(script_dir, "models"),
        help="Directory containing .sdf (and associated mesh) files to load.",
    )
    parser.add_argument(
        "--drop_height",
        type=float,
        default=0.5,
        help="Height (m) above the container floor at which to start placing bodies. "
             "Increase this to give bodies more room before they stack.",
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="cenic",
        help="Integrator: implicit_euler, runge_kutta3, cenic, discrete.",
    )
    parser.add_argument("--accuracy",      type=float, default=1e-1)
    parser.add_argument("--max_step_size", type=float, default=0.1)
    parser.add_argument("--sim_time",      type=float, default=5.0)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--E",          type=float, default=1e9,   help="Hydroelastic modulus [Pa].")
    parser.add_argument("--margin",     type=float, default=2e-5,  help="Margin [m].")
    parser.add_argument("--barrier",    type=float, default=2e-5,  help="Barrier [m].")
    parser.add_argument("--d",          type=float, default=10,    help="Hunt-Crossley dissipation [s/m].")
    parser.add_argument("--resolution", type=float, default=0.005, help="Resolution hint [m].")
    parser.add_argument("--beta",       type=float, default=0.1,   help="Beta for cenic integrator.")
    parser.add_argument("--seed",       type=int,   default=42,    help="Random seed for pile layout.")
    parser.add_argument(
        "--stats_file",
        type=str,
        default=None,
        help=(
            "If provided, collect heavy cenic step statistics and write them "
            "to this file in tab-separated format. Only active when "
            "--integrator=cenic."
        ),
    )

    args = parser.parse_args()

    meshcat = StartMeshcat()

    run_simulation(
        models_dir=args.models_dir,
        integrator=args.integrator,
        accuracy=args.accuracy,
        max_step_size=args.max_step_size,
        meshcat=meshcat,
        E=args.E,
        d=args.d,
        margin=args.margin,
        barrier=args.barrier,
        resolution=args.resolution,
        beta=args.beta,
        drop_height=args.drop_height,
        sim_time=args.sim_time,
        seed=args.seed,
        stats_file=args.stats_file,
        visualize=args.visualize,
    )
