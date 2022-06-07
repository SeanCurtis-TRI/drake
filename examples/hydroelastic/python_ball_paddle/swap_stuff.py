"""
This is an example for using hydroelastic contact model through pydrake.
It reads two simple SDFormat files of a compliant hydroelastic ball and
a compliant hydroelastic paddle.
The ball is dropped on an edge of the paddle and bounces off.
"""
import argparse
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (
    AddContactMaterial,
    AddCompliantHydroelasticProperties,
    DrakeVisualizer,
    GeometryInstance,
    Box,
    MakePhongIllustrationProperties,
    ProximityProperties,
)
from pydrake.math import RigidTransform
from pydrake.math import RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.plant import ConnectContactResultsToDrakeVisualizer
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.analysis import ApplySimulatorConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.analysis import SimulatorConfig
from pydrake.systems.analysis import PrintSimulatorStatistics
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import VectorLogSink

NEVER_SWAP = 0
SWAP_IN_MODEL = 1
SWAP_IN_CONTEXT = 2

swap_geometries = NEVER_SWAP

def make_ball_paddle(contact_model, contact_surface_representation,
                     time_step):
    multibody_plant_config = \
        MultibodyPlantConfig(
            time_step=time_step,
            contact_model=contact_model,
            contact_surface_representation=contact_surface_representation)
    # We pose the paddle, so that its top surface is on World's X-Y plane.
    # Intuitively we push it down 1 cm because the box is 2 cm thick.
    p_WPaddle_fixed = RigidTransform(RollPitchYaw(0, 0, 0),
                                     np.array([0, 0, -0.01]))
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(multibody_plant_config, builder)

    parser = Parser(plant)
    paddle_sdf_file_name = \
        FindResourceOrThrow("drake/examples/hydroelastic/python_ball_paddle"
                            "/paddle.sdf")
    paddle = parser.AddModelFromFile(paddle_sdf_file_name, model_name="paddle")
    plant.WeldFrames(
        frame_on_parent_P=plant.world_frame(),
        frame_on_child_C=plant.GetFrameByName("paddle", paddle),
        X_PC=p_WPaddle_fixed
    )

    if (swap_geometries == SWAP_IN_MODEL):
        print("Swapping paddle in the model!")
        # Create alternate geometry
        paddle_body = plant.GetBodyByName("paddle")
        paddle_geos = plant.GetCollisionGeometriesForBody(paddle_body)
        print(paddle_geos)
        old_geo = paddle_geos[0]
        geo_frame = scene_graph.model_inspector().GetFrameId(old_geo)
        # make new geometry
        geom = GeometryInstance(RigidTransform(np.array([0, 0, -0.01])),
                                Box(0.4, 0.4, 0.02), "alt_paddle"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 0, 1, 1])
        )
        prox_props = ProximityProperties()
        AddContactMaterial(dissipation=2.7,
                           point_stiffness=3.9,
                           friction=CoulombFriction(0.25, 0.125),
                           properties=prox_props)
        AddCompliantHydroelasticProperties(resolution_hint=100,
                                           hydroelastic_modulus=2.5e6,
                                           properties=prox_props)
        geom.set_proximity_properties(prox_props)
        new_geo = scene_graph.RegisterGeometry(plant.get_source_id(),
                                               geo_frame, geom)

        plant.SwapCollisionGeometries(paddle_body, old_geo, new_geo)
        print(plant.GetCollisionGeometriesForBody(paddle_body))
        scene_graph.RemoveGeometry(plant.get_source_id(), old_geo)

    ball_sdf_file_name = \
        FindResourceOrThrow("drake/examples/hydroelastic/python_ball_paddle"
                            "/ball.sdf")
    parser.AddModelFromFile(ball_sdf_file_name)
    # TODO(DamrongGuoy): Let users override hydroelastic modulus, dissipation,
    #  and resolution hint from the two SDF files above.

    plant.Finalize()

    DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)
    # TODO: If we change collision geometries in the *context*, display of
    # contact forces *fails*. The reason is that it creates an internal table
    # mapping all of the collision geometries that *MBP* knows about at the
    # time it gets created. This makes it *very* fragile when changing the
    # geometries in SceneGraph.
    #
    # What *should* happen is that it should build a table of body->frame ids
    # upon construction (relying on Mbp's immutability property). But it should
    # have a cache entry that keys on scene graph version changes to build
    # geometry id-related names. So, every time version number changes, this
    # rebuilds the table. See DrakeVisualizer for how it keys on version.
    ConnectContactResultsToDrakeVisualizer(builder=builder, plant=plant,
                                           scene_graph=scene_graph)

    nx = plant.num_positions() + plant.num_velocities()
    state_logger = builder.AddSystem(VectorLogSink(nx))
    builder.Connect(plant.get_state_output_port(),
                    state_logger.get_input_port())

    diagram = builder.Build()
    return diagram, plant, scene_graph, state_logger


def simulate_diagram(diagram, ball_paddle_plant, ball_paddle_sg, state_logger,
                     ball_init_position, ball_init_velocity,
                     simulation_time, target_realtime_rate):
    q_init_val = np.array([
        1, 0, 0, 0, ball_init_position[0], ball_init_position[1],
        ball_init_position[2]
    ])
    v_init_val = np.hstack((np.zeros(3), ball_init_velocity))
    qv_init_val = np.concatenate((q_init_val, v_init_val))

    simulator_config = SimulatorConfig(
                           target_realtime_rate=target_realtime_rate,
                           publish_every_time_step=True)
    simulator = Simulator(diagram)
    ApplySimulatorConfig(simulator, simulator_config)

    context = simulator.get_context()

    plant_context = diagram.GetSubsystemContext(ball_paddle_plant, context)

    if (swap_geometries == SWAP_IN_CONTEXT):
        print("Swapping paddle in the context!")
        plant = ball_paddle_plant
        scene_graph = ball_paddle_sg
        sg_context = diagram.GetSubsystemContext(ball_paddle_sg, context)
        # Create alternate geometry
        paddle_body = plant.GetBodyByName("paddle")
        paddle_geos = plant.GetCollisionGeometriesForBody(paddle_body)
        print(paddle_geos)
        old_geo = paddle_geos[0]
        geo_frame = scene_graph.model_inspector().GetFrameId(old_geo)
        # make new geometry
        geom = GeometryInstance(RigidTransform(np.array([0, 0, -0.01])),
                                Box(0.4, 0.4, 0.02), "alt_paddle"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 1, 0, 1])
        )
        prox_props = ProximityProperties()
        AddContactMaterial(dissipation=2.7,
                        point_stiffness=3.9,
                        friction=CoulombFriction(0.25, 0.125),
                        properties=prox_props)
        AddCompliantHydroelasticProperties(resolution_hint=100,
                                           hydroelastic_modulus=2.5e6,
                                           properties=prox_props)
        geom.set_proximity_properties(prox_props)
        new_geo = scene_graph.RegisterGeometry(sg_context,
                                               plant.get_source_id(),
                                               geo_frame, geom)

        plant.SwapCollisionGeometries(paddle_body, old_geo, new_geo)
        print(plant.GetCollisionGeometriesForBody(paddle_body))
        scene_graph.RemoveGeometry(sg_context, plant.get_source_id(), old_geo)
    
    ball_paddle_plant.SetPositionsAndVelocities(plant_context,
                                                qv_init_val)
    simulator.get_mutable_context().SetTime(0)
    state_log = state_logger.FindMutableLog(simulator.get_mutable_context())
    state_log.Clear()
    simulator.Initialize()
    simulator.AdvanceTo(boundary_time=simulation_time)
    PrintSimulatorStatistics(simulator)
    return state_log.sample_times(), state_log.data()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simulation_time", type=float, default=0.5,
        help="Desired duration of the simulation in seconds. "
             "Default 0.5.")
    parser.add_argument(
        "--contact_model", type=str, default="hydroelastic_with_fallback",
        help="Contact model. Options are: 'point', 'hydroelastic', "
             "'hydroelastic_with_fallback'. "
             "Default 'hydroelastic_with_fallback'")
    parser.add_argument(
        "--contact_surface_representation", type=str, default="polygon",
        help="Contact-surface representation for hydroelastics. "
             "Options are: 'triangle' or 'polygon'. Default 'polygon'.")
    parser.add_argument(
        "--time_step", type=float, default=0.001,
        help="The fixed time step period (in seconds) of discrete updates "
             "for the multibody plant modeled as a discrete system. "
             "If zero, we will use an integrator for a continuous system. "
             "Non-negative. Default 0.001.")
    parser.add_argument(
        "--ball_initial_position", nargs=3, metavar=('x', 'y', 'z'),
        default=[0, 0, 0.1],
        help="Ball's initial position: x, y, z (in meters) in World frame. "
             "Default: 0 0 0.1")
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Target realtime rate. Default 1.0.")
    parser.add_argument("--swap", type=str, default="none",
                        choices=['none', 'context', 'model'])
    args = parser.parse_args()

    # global swap_geometries
    if (args.swap == 'none'):
        swap_geometries = NEVER_SWAP
    elif (args.swap == 'context'):
        swap_geometries = SWAP_IN_CONTEXT
    elif (args.swap == 'model'):
        swap_geometries = SWAP_IN_MODEL

    diagram, ball_paddle_plant, scene_graph, state_logger = make_ball_paddle(
        args.contact_model, args.contact_surface_representation,
        args.time_step)
    time_samples, state_samples = simulate_diagram(
        diagram, ball_paddle_plant, scene_graph, state_logger,
        np.array(args.ball_initial_position),
        np.array([0., 0., 0.]),
        args.simulation_time, args.target_realtime_rate)
    print("\nFinal state variables:")
    print(state_samples[:, -1])
