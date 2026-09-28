import argparse
from pydrake.all import *
from pydrake.multibody.contact_solvers import IcfSolverParameters
import builtins

import time
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

##
#
# Compare different integration schemes on a few toy examples.
#
##


@dataclass
class SimulationExample:
    """A little container for setting up different examples."""

    name: str
    url: str
    use_hydroelastic: bool
    initial_state: np.array
    sim_time: float

def fidget():
    name = "fidget"
    url = "package://drake/examples/integrators/fidget/fidget.sdf"
    use_hydroelastic = True
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    sim_time = 5.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def spinning_plate(initial_w: float = 10.0):
    """A thin (codimensional) square plate spinning about an in-plane axis a
    few centimeters above a table. Stresses the linear-CCD assumption: with a
    large angular velocity, straight-line vertex trajectories under-sweep the
    true screw motion near grazing contact."""
    name = "Spinning plate"
    url = "package://drake/examples/integrators/spinning_plate.sdf"
    use_hydroelastic = True
    # Floating base: [qw qx qy qz, x y z, wx wy wz, vx vy vz].
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
         initial_w, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    sim_time = 2.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def ball_on_table():
    """A sphere is dropped on a table with some initial horizontal velocity."""
    name = "Ball on table"
    url = "package://drake/examples/integrators/ball_on_table.xml"
    use_hydroelastic = True
    # initial_state = np.array(
    #     [1.0, 0.0, 0.0, 0.0, 0.05, 0.0,  0.5,
    #      1.0, 0.0, 0.0, 0.0, 0.0, 0.05,  1.0,
    #      1.0, 0.0, 0.0, 0.0, 0.05, 0.05, 1.5,
    #      1.0, 0.0, 0.0, 0.0, -0.05, 0.0, 2.0,
    #      1.0, 0.0, 0.0, 0.0, 0.0, -0.05, 2.5,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
    # )
    # Initial state that produces impulse (at h = 4.281817161535433e-05)
    # initial_state = np.array(
    #     [1.0, 0.0, 0.0, 0.0, 0.3, 0.0,  0.2003100155452781,
    #      0.0, 0.0, 0.0, 0.0, 0.0, -5.826597059588274]
    # )
    # initial_state = np.array(
    #     [1.0, 0.0, 0.2,  0.0, 0.0, 0.0, 1,
    #      1.0, 0.0, -0.2, 0.01, 0.01, 0.0, 1.5,
    #      1.0, 0.2, 0.0,  0.02, 0.02, 0.0, 2,
    #      1.0, -0.2, 0.0, 0.03, 0.03, 0.0, 2.5,
    #      1.0, 0.0, 0.2,  0.04, 0.04, 0.0, 3,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # )
    # initial_state = np.array(
    #     [1.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.5,
    #      1.0, 0.2, 0.0, 0.05, 0.05, 0.0, 1,
    #      1.0, 0.0, -0.2, 0.1, 0.1, 0.0, 1.5,
    #      1.0, 0.0, 0.2, 0.15, 0.15, 0.0, 2,
    #      1.0, 0.0, 0.0, 0.2, 0.2, 0.0, 2.5,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #      0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # )
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2 + 2e-4,
         0.0, 5.0, 0.0, 0.5, 0.0, 0.0]
    )
    # # Generate non-overlapping random positions for 10 balls of radius 0.1
    # num_balls = 10
    # radius = 0.1
    # positions = []
    # while len(positions) < num_balls:
    #     candidate = np.array([np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0), np.random.uniform(0.5, 2.0)])
    #     if builtins.all(np.linalg.norm(candidate - p, ord=2) > 2 * radius for p in positions):
    #         positions.append(candidate)
    # # Each ball: [qw, qx, qy, qz, x, y, z] for all balls, then [vx, vy, vz, wx, wy, wz] for all balls
    # q_list = []
    # v_list = []
    # for pos in positions:
    #     # Quaternion for no rotation: [1, 0, 0, 0], position: [x, y, z]
    #     q_list.extend([1, 0, 0, 0, pos[0], pos[1], pos[2]])
    #     # Zero velocity: [vx, vy, vz, wx, wy, wz]
    #     v_list.extend([0, 0, 0, 0, 0, 0])
    # initial_state = np.array(q_list + v_list)
    sim_time = 10.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def clutter():
    """Several spheres fall into a box."""
    name = "Clutter"
    url = "package://drake/examples/integrators/clutter.xml"
    use_hydroelastic = True
    initial_state = np.array(
        [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
    )
    sim_time = 3.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

# This case works:
#  bazel run //examples/integrators:error_control_demo -- --example=nut_and_bolt --visualize --tau=-0.005 --barrier=2e-5 --margin=1e-5 --E=1e8 --d=50
def nut_and_bolt():
    """Nut fastening to bolt under gravity (no friction)."""
    name = "Nut and Bolt"
    url = "package://drake/examples/integrators/nut_and_bolt.sdf"
    use_hydroelastic = True
    initial_state = np.array(
        [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
    )
    sim_time = 5.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def plate_and_spatula():
    """A spatula is dropped onto a plate."""
    name = "Plate and spatula"
    url = "package://drake/examples/integrators/plate_and_spatula.sdf"
    use_hydroelastic = True
    initial_state = np.array([
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02,   # plate pos
        1.0, 0.0, 0.0, 0.0, -0.05, 0.0, 0.3,  # spatula pos
        1.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.5,   # plate pos
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # plate vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # spatula vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # spatula vel
    ])
    sim_time = 2.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def cones():
    """Cones"""
    name = "Cones"
    url = "package://drake/examples/integrators/cones.sdf"
    use_hydroelastic = True
    initial_state = np.array([
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,   # cone0 pos
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2,   # cone1 pos
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3,   # cone2 pos
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4,   # cone3 pos
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,   # cone4 pos
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # cone0 vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # cone1 vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # cone2 vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # cone3 vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # cone4 vel
    ])
    sim_time = 3.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def sphere_and_spiral():
    """A sphere dropped onto a fixed spiral mesh."""
    name = "Sphere and spiral"
    url = "package://drake/examples/integrators/sphere_and_spiral.sdf"
    use_hydroelastic = True
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    sim_time = 20.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )

def teddy_and_torus():
    """A teddy bear is dropped onto a torus."""
    name = "Teddy and torus"
    url = "package://drake/examples/integrators/teddy_and_torus.sdf"
    use_hydroelastic = True
    initial_state = np.array([
        1.0, 1.0, 0.0, 0.0, -0.2, 0.0, 0.1,   # teddy pos
        1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1,   # teddy pos
        1.0, 1.0, 0.0, 0.0, 0.2, 0.0, 0.1,   # teddy pos
        1.0, 0.0, 0.0, 0.0, -0.1, 0.0, 0.4,  # torus pos
        1.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.4,  # torus pos
        1.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.4,  # torus pos
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # teddy vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # teddy vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # teddy vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # torus vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # torus vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # torus vel
    ])
    sim_time = 3.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def create_scene(
    url: str,
    time_step: float,
    meshcat: Meshcat,
    E: float,
    d: float,
    margin: float,
    barrier: float,
    resolution: float,
    tau: float,
    hydroelastic: bool = False,
    visualize: bool = True,
):
    """
    Set up a drake system dyagram

    Args:
        xml: mjcf robot description.
        time_step: dt for MultibodyPlant.
        hydroelastic: whether to use hydroelastic contact.
        meshcat: meshcat instance for visualization.
        visualize: whether to show the visualization

    Returns:
        The system diagram, the MbP within that diagram, and the logger used to
        keep track of time steps.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )

    parser = Parser(plant)
    parser.AddModels(url=url)
    if time_step > 0:
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kLagged
        )
    plant.Finalize()

    if hydroelastic:
        sg_config = SceneGraphConfig()
        sg_config.default_proximity_properties.compliance_type = "compliant"
        sg_config.default_proximity_properties.hydroelastic_modulus = E
        sg_config.default_proximity_properties.hunt_crossley_dissipation = d
        sg_config.default_proximity_properties.margin = margin
        sg_config.default_proximity_properties.barrier = barrier
        sg_config.default_proximity_properties.resolution_hint = resolution
        sg_config.default_proximity_properties.dynamic_friction = 0.0
        sg_config.default_proximity_properties.static_friction = 0.0
        scene_graph.set_config(sg_config)

    if meshcat is not None:
        # Visualization (interactive --visualize, or headless recording for
        # --html_file). The long publish period in headless mode avoids
        # perturbing error control; the recording's forced publishes come
        # from StartRecording's frame schedule.
        vis_config = VisualizationConfig()
        vis_config.publish_period = 0.1 if visualize else 1e9
        ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    return diagram, plant


def run_simulation(
    example: SimulationExample,
    integrator: str,
    accuracy: float,
    max_step_size: float,
    meshcat: Meshcat,
    E: float,
    d: float,
    margin: float,
    barrier: float,
    resolution: float,
    beta: float,
    tau: float,
    stats_file: str = None,
    visualize: bool = True,
    use_toi: bool = False,
    summary_file: str = None,
    html_file: str = None,
    html_fps: float = 16.0,
):
    """
    Run a short simulation, and report the time-steps used throughout.

    Args:
        example: container defining the scenario to simulate.
        integrator: which integration strategy to use ("implicit_euler",
            "runge_kutta3", "cenic", "discrete").
        accuracy: the desired accuracy (ignored for "discrete").
        max_step_size: the maximum (and initial) timestep dt.
        meshcat: meshcat instance for visualization.
        visualize: whether to show stuff on meshcat (slow).

    Returns:
        Timesteps (dt) throughout the simulation, and the wall-clock time.
    """
    url = example.url
    use_hydroelastic = example.use_hydroelastic
    initial_state = example.initial_state
    sim_time = example.sim_time

    # We can use a more standard simulation setup and rely on a logger to
    # tell use the time step information. Note that in this case enabling
    # visualization messes with the time step report though.

    # Configure Drake's built-in error controlled integration
    config = SimulatorConfig()
    if integrator != "discrete":
        config.integration_scheme = integrator
    config.max_step_size = max_step_size
    config.accuracy = accuracy
    config.target_realtime_rate = 0
    config.use_error_control = True

    # Set up the system diagram and initial condition
    if integrator == "discrete":
        time_step = max_step_size
    else:
        time_step = 0.0
    diagram, plant = create_scene(
        url, time_step, meshcat, E, d, margin, barrier, resolution, tau, use_hydroelastic, visualize
    )
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

    plant.SetPositionsAndVelocities(plant_context, initial_state)

    # data_file = open('/home/joemasterjohn/tri/drake/data.txt', 'w')
    # area_file = open('/home/joemasterjohn/tri/drake/areas.txt', 'w')

    # data_file.write(f"t\th\tmin_e\tmax_e\tmean_e\tarea\tforce\t{' '.join(plant.GetStateNames())}\n")
    # prev_time = 0.0
    # areas = []
    # timesteps = []

    # def monitor(context):
    #     nonlocal prev_time
    #     nonlocal areas
    #     nonlocal timesteps
    #     sim_time = context.get_time()
    #     h = sim_time - prev_time
    #     timesteps.append(h)
    #     prev_time = sim_time

    #     plant_context = plant.GetMyContextFromRoot(context)

    #     # State vector
    #     state = plant.GetPositionsAndVelocities(plant_context)

    #     v = state[12]

    #     # Contact surfaces via QueryObject
    #     query_object = plant.get_geometry_query_input_port().Eval(plant_context)
    #     contact_surfaces = query_object.ComputeContactSurfaces(HydroelasticContactRepresentation.kPolygon)

    #     # Expect only one contact surface
    #     if len(contact_surfaces) > 0:
    #         s  = contact_surfaces[0]
    #         min_e = float('inf')
    #         max_e = -float('inf')
    #         mean_e = 0
    #         total_area = 0
    #         for face in range(s.num_faces()):
    #             e = s.poly_e_MN().EvaluateCartesian(face, s.centroid(face))
    #             A = s.area(face)
    #             min_e = min(e, min_e)
    #             max_e = max(e, max_e)
    #             mean_e += e*A
    #             total_area += A
    #             areas.append(A)

    #         mean_e /= total_area

    #         data_file.write(f"{sim_time}\t{h}\t{min_e}\t{max_e}\t{mean_e}\t{total_area}\t{' '.join(map(str, state))}\n")
    #     else:
    #         data_file.write(f"{sim_time}\t{h}\t0\t0\t0\t0\t{' '.join(map(str, state))}\n")

    #     print(f"MONITOR t: {context.get_time()}")

    #     return EventStatus.Succeeded()

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    # simulator.set_monitor(monitor)

    if integrator == "cenic":
        ci = simulator.get_mutable_integrator()
        # We can also set some solver parameters for the integrator here
        params = IcfSolverParameters()
        params.beta = beta
        params.use_toi = use_toi
        params.collect_heavy_stats = stats_file is not None
        ci.SetSolverParameters(params)

    simulator.Initialize()

    # print(f"Running the {example.name} example with {integrator} integrator.")
    if visualize:
        input("Waiting for meshcat... [ENTER] to continue")

    # Simulate
    if meshcat is not None:
        if html_file is not None:
            # StaticHtml() embeds every recorded frame; keep the rate modest.
            meshcat.StartRecording(frames_per_second=html_fps)
        else:
            meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(0.1)
        # Add external torque if tau is non-zero
    if tau != 0.0:
        # Find the first floating body (not the world body)
        floating_body = None
        for body_index in [BodyIndex(x) for x in range(0, plant.num_bodies())]:
            body = plant.get_body(body_index)
            if not body == plant.world_body() and body.is_floating_base_body():
                floating_body = body
                break

        if floating_body is not None:
            # Create an ExternallyAppliedSpatialForce with constant torque about z-axis
            external_force = ExternallyAppliedSpatialForce()
            external_force.body_index = floating_body.index()
            external_force.p_BoBq_B = [0, 0, 0]  # Applied at body origin
            external_force.F_Bq_W = SpatialForce(tau=[0, 0, tau], f=[0, 0, 0])

            plant.get_applied_spatial_force_input_port().FixValue(plant_context, [external_force])

    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time
    if meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()
    if html_file is not None and meshcat is not None:
        with open(html_file, "w") as f:
            f.write(meshcat.StaticHtml())
        print(f"Static HTML written to {html_file}")

    # area_file.write('\n'.join(map(str, areas)))

    print(f"\nWall clock time: {wall_time}\n")
    PrintSimulatorStatistics(simulator)

    x_final = plant.GetPositions(plant_context)[4]
    print(f"x_final: {x_final}\n")

    if summary_file is not None:
        import json
        summary = {
            "example": example.name,
            "integrator": integrator,
            "accuracy": accuracy,
            "max_step_size": max_step_size,
            "E": E,
            "d": d,
            "margin": margin,
            "barrier": barrier,
            "resolution": resolution,
            "beta": beta,
            "tau": tau,
            "use_toi": use_toi,
            "sim_time": sim_time,
            "wall_clock": wall_time,
            "x_final": float(x_final),
            "num_steps_taken": simulator.get_num_steps_taken(),
            "final_state":
                plant.GetPositionsAndVelocities(plant_context).tolist(),
        }
        integrator_obj = simulator.get_integrator()
        summary["integrator_stats"] = {
            name: value
            for (name, value) in integrator_obj.GetStatisticsSummary()
        } if hasattr(integrator_obj, "GetStatisticsSummary") else {}
        with open(summary_file, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"Summary written to {summary_file}")

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

    #return np.asarray(timesteps), wall_time
    return [], wall_time


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--example",
        type=str,
        default="clutter",
        help=(
            "Which example to run. One of: ball_on_table, clutter, "
            "plate_and_spatula, cones, teddy_and_torus, nut_and_bolt, "
            "fidget, sphere_and_spiral, spinning_plate"
        ),
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="cenic",
        help=(
            "Integrator to use, e.g., implicit_euler, runge_kutta3, cenic, "
            "discrete. Default: cenic."
        ),
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=1e-1,
        help="Integrator accuracy (ignored for discrete).",
    )
    parser.add_argument(
        "--max_step_size",
        type=float,
        default=0.1,
        help=(
            "Maximum time step size (or fixed step size for discrete "
            "integrator)."
        ),
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help=(
            "Whether to make plots of the step size over time. Default: "
            "False."
        ),
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help=(
            "Whether to visualize with Meshcat. Default: "
            "False."
        ),
    )
    parser.add_argument(
        "--E",
        type=float,
        default=1e9,
        help="Default hydroelastic modulus [Pa].",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=2e-5,
        help="Default margin [m].",
    )
    parser.add_argument(
        "--barrier",
        type=float,
        default=2e-5,
        help="Default barrier [m].",
    )
    parser.add_argument(
        "--d",
        type=float,
        default=10,
        help="Default H&C dissipation [s/m].",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.005,
        help="Default resolution hint [m].",
    )
    parser.add_argument(
        "--beta",
        type=float,
        default=0.1,
        help="Beta parameter for cenic integrator.",
    )
    parser.add_argument(
        "--tau",
        type=float,
        default=0.0,
        help="External torque about z-axis applied to floating body [N*m].",
    )
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
    parser.add_argument(
        "--summary_file",
        type=str,
        default=None,
        help=(
            "If provided, write a machine-readable JSON summary of the run "
            "(parameters, wall clock, x_final, integrator statistics) to "
            "this file."
        ),
    )
    parser.add_argument(
        "--sim_time",
        type=float,
        default=None,
        help="Override the example's default simulation duration [s].",
    )
    parser.add_argument(
        "--use_toi",
        action="store_true",
        help=(
            "Use time-of-impact estimation for step-size adjustment when the "
            "barrier/CCD feasibility check rejects a step (cenic only)."
        ),
    )
    parser.add_argument(
        "--initial_w",
        type=float,
        default=10.0,
        help=(
            "Initial angular velocity magnitude [rad/s] for the "
            "spinning_plate example."
        ),
    )

    parser.add_argument(
        "--html_file",
        type=str,
        default=None,
        help=(
            "If provided, record the simulation with meshcat and write a "
            "standalone static HTML snapshot (scene + animation) to this "
            "path. Works headless (no --visualize needed; no prompts)."
        ),
    )
    parser.add_argument(
        "--html_fps",
        type=float,
        default=16.0,
        help="Recording frame rate for --html_file (file size grows w/ fps).",
    )
    args = parser.parse_args()

    # Set up the example system
    if args.example == "ball_on_table":
        example = ball_on_table()
    elif args.example == "clutter":
        example = clutter()
    elif args.example == "plate_and_spatula":
        example = plate_and_spatula()
    elif args.example == "cones":
        example = cones()
    elif args.example == "teddy_and_torus":
        example = teddy_and_torus()
    elif args.example == "nut_and_bolt":
        example = nut_and_bolt()
    elif args.example == "fidget":
        example = fidget()
    elif args.example == "sphere_and_spiral":
        example = sphere_and_spiral()
    elif args.example == "spinning_plate":
        example = spinning_plate(args.initial_w)
    else:
        raise ValueError(f"Unknown example {args.example}")

    if args.sim_time is not None:
        example.sim_time = args.sim_time

    # Start a meshcat server when visualization was requested, or when a
    # static-HTML recording is wanted (headless; no interactive prompts).
    meshcat = (
        StartMeshcat() if (args.visualize or args.html_file is not None)
        else None
    )

    time_steps, _ = run_simulation(
        example,
        args.integrator,
        args.accuracy,
        max_step_size=args.max_step_size,
        meshcat=meshcat,
        E=args.E,
        d=args.d,
        margin=args.margin,
        barrier=args.barrier,
        resolution=args.resolution,
        beta=args.beta,
        tau=args.tau,
        stats_file=args.stats_file,
        visualize=args.visualize,
        use_toi=args.use_toi,
        summary_file=args.summary_file,
        html_file=args.html_file,
        html_fps=args.html_fps,
    )

    if args.plot:
        times = np.cumsum(time_steps)
        plt.title(
            (
                f"{example.name} | {args.integrator} integrator | "
                f"accuracy = {args.accuracy}"
            )
        )
        plt.plot(times, time_steps, "o")
        plt.ylim(1e-10, 1e0)
        plt.yscale("log")
        plt.xlabel("time (s)")
        plt.ylabel("step size (s)")
        plt.show()
