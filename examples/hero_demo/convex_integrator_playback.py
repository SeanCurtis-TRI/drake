"""
Play back a demonstration using the Convex Integrator.

Track joint targets with a stiff PD controller so we can avoid going through the
(discrete-time) DiffIK.
"""

import argparse
import time

import numpy as np
import yaml

from pydrake.common import FindResourceOrThrow
from pydrake.common.yaml import yaml_dump, yaml_load_file, yaml_load_typed
from pydrake.geometry import (
    ProximityProperties,
    Role,
    RoleAssign,
    SceneGraphConfig,
    StartMeshcat,
)

# Importing CenicIntegrator registers its pybind type so that
# Simulator.get_mutable_integrator() downcasts to it (for
# SetSolverParameters / get_step_statistics).
from pydrake.multibody.cenic import CenicIntegrator  # noqa: F401
from pydrake.multibody.contact_solvers import IcfSolverParameters
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import PdControllerGains
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    PrintSimulatorStatistics,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import (
    DiagramBuilder,
    EventStatus,
    LeafSystem,
)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

# Usage:
#   bazel build //intuitive/visuomotor:...
#   ./run //intuitive/visuomotor:convex_integrator_playback

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--visualize", type=int, default=1)
arg_parser.add_argument("--accuracy", type=float, default=1e-1)
arg_parser.add_argument("--max_step_size", type=float, default=0.1)
arg_parser.add_argument("--sim_time", type=float, default=100.0)
arg_parser.add_argument(
    "--margin",
    type=float,
    default=None,
    help=(
        "If provided, override the ('hydroelastic', 'margin') proximity "
        "property [m] on every collision geometry."
    ),
)
arg_parser.add_argument(
    "--barrier",
    type=float,
    default=None,
    help=(
        "If provided, override the ('hydroelastic', 'barrier') proximity "
        "property [m] on every collision geometry. Use --barrier=0 "
        "(with --margin=0) for Drake's default volumetric hydroelastic; "
        "barrier > 0 selects the thin-layer barrier hydro representation."
    ),
)
arg_parser.add_argument(
    "--times_file",
    type=str,
    default=None,
    help=(
        "If provided, record (sim_time, wall_time) after every accepted "
        "step and write them to this file in csv format."
    ),
)
arg_parser.add_argument(
    "--summary_file",
    type=str,
    default=None,
    help="If provided, write a machine-readable run summary to this file.",
)
arg_parser.add_argument(
    "--stats_file",
    type=str,
    default=None,
    help=(
        "If provided, collect heavy cenic step statistics and write them "
        "to this file in tab-separated format."
    ),
)
arg_parser.add_argument(
    "--beta",
    type=float,
    default=None,
    help=(
        "ICF log-barrier regularization parameter beta. If omitted, the "
        "IcfSolverParameters default is used."
    ),
)
arg_parser.add_argument(
    "--html_file",
    type=str,
    default=None,
    help=(
        "If provided, record the simulation with meshcat and write a "
        "standalone static HTML snapshot (scene + animation) to this path. "
        "Works headless (no --visualize needed; no interactive prompts)."
    ),
)
arg_parser.add_argument(
    "--html_fps",
    type=float,
    default=8.0,
    help=(
        "Recording frame rate used for --html_file. Kept low by default: "
        "StaticHtml() embeds every frame, so file size grows with fps x "
        "sim_time."
    ),
)
args = arg_parser.parse_args()


class JointTargetSource(LeafSystem):
    """
    This simple leaf system sends out joint targets to be tracked by a PID
    controller, as recorded in a keyframes.txt file.
    """

    def __init__(self):
        super().__init__()

        # Parse joint targets from the keyframes file
        keyframes_file = FindResourceOrThrow(
            "drake/examples/hero_demo/keyframes.txt"
        )
        with open(keyframes_file, "r") as f:
            lines = f.readlines()

        times = []
        left_arm_targets = []
        right_arm_targets = []
        left_gripper_targets = []
        right_gripper_targets = []
        for line in lines:
            if line.startswith("time: "):
                time_str = line.split(":")[1].strip()
                times.append(float(time_str))
            elif line.startswith("left::panda: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 7
                left_arm_targets.append(target)
            elif line.startswith("right::panda: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 7
                right_arm_targets.append(target)
            elif line.startswith("left::panda_hand: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 2
                left_gripper_targets.append(target)
            elif line.startswith("right::panda_hand: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 2
                right_gripper_targets.append(target)

        self.left_arm_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(left_arm_targets).T
            )
        )
        self.right_arm_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(right_arm_targets).T
            )
        )
        self.left_gripper_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(left_gripper_targets).T
            )
        )
        self.right_gripper_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(right_gripper_targets).T
            )
        )

        self.DeclareVectorOutputPort("left_arm", 14, self.CalcLeftArmTarget)
        self.DeclareVectorOutputPort("right_arm", 14, self.CalcRightArmTarget)
        self.DeclareVectorOutputPort(
            "left_gripper", 4, self.CalcLeftGripperTarget
        )
        self.DeclareVectorOutputPort(
            "right_gripper", 4, self.CalcRightGripperTarget
        )

    def CalcLeftArmTarget(self, context, output):
        q_nom = self.left_arm_spline.value(context.get_time()).flatten()
        v_nom = np.zeros(7)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcRightArmTarget(self, context, output):
        q_nom = self.right_arm_spline.value(context.get_time()).flatten()
        v_nom = np.zeros(7)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcLeftGripperTarget(self, context, output):
        q_nom = self.left_gripper_spline.value(context.get_time()).flatten()
        if q_nom[1] < 0.03:
            # When the gripper is closed, squeeze it closed tightly
            q_nom = np.array([0.01, -0.01])
        v_nom = np.zeros(2)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcRightGripperTarget(self, context, output):
        q_nom = self.right_gripper_spline.value(context.get_time()).flatten()
        if q_nom[1] < 0.03:
            # When the gripper is closed, squeeze it closed tightly
            q_nom = np.array([0.01, -0.01])
        v_nom = np.zeros(2)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)


# Load model directives from the scenario file saved with the recording
model_directives_file = FindResourceOrThrow(
    "drake/examples/hero_demo/resolved_scenario.yaml"
)
data = yaml_load_file(model_directives_file)
directives_data = data["directives"]
directives_string = yaml_dump({"directives": directives_data})

# Parse plant and scene graph configs from the scenario file.
# Override time_step to 0.0 since CENIC requires continuous-time integration.
plant_config = yaml_load_typed(
    schema=MultibodyPlantConfig, data=yaml.dump(data["plant_config"])
)
plant_config.time_step = 0.0
scene_graph_config = yaml_load_typed(
    schema=SceneGraphConfig, data=yaml.dump(data["scene_graph_config"])
)

# Set up the system diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlant(
    plant_config, scene_graph_config, builder
)
parser = Parser(builder)

remote_params = PackageMap.RemoteParams(
    urls=[
        "https://github.com/ToyotaResearchInstitute/lbm_eval/releases/"
        "download/1.1.0/lbm_eval_models-1.1.0-py3-none-any.whl"
    ],
    sha256="97d61eb617d2d409d7c5873824ff79d26f6dd1a5532e428d4d320fac15c2957d",
    archive_type="zip",
    strip_prefix="lbm_eval_models",
)
parser.package_map().AddRemote(
    package_name="lbm_eval_models", params=remote_params
)
parser.AddModelsFromString(
    file_contents=directives_string, file_type="dmd.yaml"
)

# Remove implicit PD actuation
for idx in plant.GetJointActuatorIndices():
    actuator = plant.get_joint_actuator(idx)
    if actuator.has_controller():
        actuator.set_controller_gains(PdControllerGains(p=0.0, d=0.0))

plant.Finalize()

# Override the hydroelastic margin/barrier proximity properties on every
# collision geometry. The scenario's scene_graph_config defaults only backfill
# properties that are missing, so replacing the properties here guarantees the
# requested values reach the hydroelastic reification. barrier == 0 selects
# Drake's default volumetric hydroelastic representation; barrier > 0 selects
# the thin-layer barrier hydro representation.
if args.margin is not None or args.barrier is not None:
    source_id = plant.get_source_id()
    inspector = scene_graph.model_inspector()
    for geometry_id in inspector.GetAllGeometryIds(Role.kProximity):
        props = ProximityProperties(
            inspector.GetProximityProperties(geometry_id)
        )
        if args.margin is not None:
            props.UpdateProperty("hydroelastic", "margin", args.margin)
        if args.barrier is not None:
            props.UpdateProperty("hydroelastic", "barrier", args.barrier)
        scene_graph.AssignRole(
            source_id, geometry_id, props, RoleAssign.kReplace
        )

# Connect to meshcat for visualization (also needed headlessly when
# recording a static HTML snapshot via --html_file).
if args.visualize or args.html_file is not None:
    meshcat = StartMeshcat()
    vis_config = VisualizationConfig()
    vis_config.publish_period = 1e9  # very long to avoid extra publishes
    vis_config.publish_contacts = False
    vis_config.publish_inertia = False
    vis_config.publish_proximity = False
    ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    # Configure meshcat parameters for nicer visualization
    # with open("intuitive/sim/meshcat_params.yaml", "r") as f:
    #    meshcat_params = yaml.safe_load(f)
    # for p in meshcat_params["initial_properties"]:
    #    meshcat.SetProperty(p["path"], p["property"], p["value"])
    # meshcat.SetProperty("/Axes", "visible", False)
    # meshcat.SetCameraPose([0.7, -0.3, 0.7], [0.0, 0.0, 0.2])

# Connect stiff joint-level PID controllers to the robot
Kp_arm = 1e4 * np.ones(7)
Ki_arm = 0.0 * np.ones(7)
Kd_arm = 1e3 * np.ones(7)

Px_gripper = np.eye(4)
Py_gripper = np.array([[0.5, -0.5]])
Kp_gripper = 5e3 * np.ones(2)
Ki_gripper = 0.0 * np.ones(2)
Kd_gripper = 1e3 * np.ones(2)

joint_target_source = builder.AddSystem(JointTargetSource())

left_arm_ctrl = builder.AddSystem(PidController(Kp_arm, Ki_arm, Kd_arm))
right_arm_ctrl = builder.AddSystem(PidController(Kp_arm, Ki_arm, Kd_arm))
left_gripper_ctrl = builder.AddSystem(
    PidController(Px_gripper, Py_gripper, Kp_gripper, Ki_gripper, Kd_gripper)
)
right_gripper_ctrl = builder.AddSystem(
    PidController(Px_gripper, Py_gripper, Kp_gripper, Ki_gripper, Kd_gripper)
)

left_arm = plant.GetModelInstanceByName("left::panda")
right_arm = plant.GetModelInstanceByName("right::panda")
left_gripper = plant.GetModelInstanceByName("left::panda_hand")
right_gripper = plant.GetModelInstanceByName("right::panda_hand")

builder.Connect(
    joint_target_source.GetOutputPort("left_arm"),
    left_arm_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(left_arm),
    left_arm_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    left_arm_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(left_arm),
)

builder.Connect(
    joint_target_source.GetOutputPort("right_arm"),
    right_arm_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(right_arm),
    right_arm_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    right_arm_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(right_arm),
)

builder.Connect(
    joint_target_source.GetOutputPort("left_gripper"),
    left_gripper_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(left_gripper),
    left_gripper_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    left_gripper_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(left_gripper),
)

builder.Connect(
    joint_target_source.GetOutputPort("right_gripper"),
    right_gripper_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(right_gripper),
    right_gripper_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    right_gripper_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(right_gripper),
)

# Set initial conditions from the recording
initial_positions = data["initial_position"]
for model in initial_positions.keys():
    model_instance = plant.GetModelInstanceByName(model)
    for joint_name in initial_positions[model].keys():
        joint = plant.GetJointByName(joint_name, model_instance)
        joint.set_default_positions(initial_positions[model][joint_name])

# Compile the system diagram
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Set up the simulator
config = SimulatorConfig()
if plant.time_step() == 0.0:
    config.integration_scheme = "cenic"
config.accuracy = args.accuracy
config.max_step_size = args.max_step_size
config.target_realtime_rate = 0.0
config.use_error_control = True

simulator = Simulator(diagram, context)
ApplySimulatorConfig(config, simulator)
integrator = simulator.get_mutable_integrator()

if config.integration_scheme == "cenic":
    # N.B. always construct the parameters (previously they were only set
    # when --stats_file was given, so flags like --beta would silently no-op).
    params = IcfSolverParameters()
    if args.beta is not None:
        params.beta = args.beta
    # Heavy per-step statistics (condition numbers etc.) are collected only
    # when a stats file is requested; timing-oriented runs leave this off so
    # the instrumentation cannot perturb the measured wall clock.
    params.collect_heavy_stats = args.stats_file is not None
    integrator.SetSolverParameters(params)

# Record (sim_time, wall_time) after every accepted step; this is the data
# source for real-time-rate (RTR) vs time plots.
recorded_times = []
if args.times_file is not None:

    def record_times(root_context):
        recorded_times.append((root_context.get_time(), time.monotonic()))
        return EventStatus.Succeeded()

    simulator.set_monitor(record_times)

simulator.Initialize()

if args.visualize:
    input("Waiting for meshcat... press [ENTER] to continue")

# Run the simulation
recording = args.visualize or args.html_file is not None
if recording:
    if args.html_file is not None:
        meshcat.StartRecording(frames_per_second=args.html_fps)
    else:
        meshcat.StartRecording()
start_time = time.time()
simulator.AdvanceTo(args.sim_time)
wall_time = time.time() - start_time
if recording:
    meshcat.StopRecording()
    meshcat.PublishRecording()
if args.html_file is not None:
    with open(args.html_file, "w") as f:
        f.write(meshcat.StaticHtml())
    print(f"Static HTML written to {args.html_file}")

# Save recorded times
if args.times_file is not None:
    with open(args.times_file, "w") as f:
        f.write("sim_time,wall_time\n")
        for sim_t, wall_t in recorded_times:
            f.write(f"{sim_t},{wall_t}\n")
    print(f"Timing logs written to {args.times_file}")

print("")
print(f"Wall clock time: {wall_time:.4f}")
print(f"Sim time  : {context.get_time():.4f} seconds")
print("")

PrintSimulatorStatistics(simulator)

if args.summary_file is not None:
    import json

    summary = {
        "example": "hero_dishrack",
        "integrator": config.integration_scheme,
        "accuracy": args.accuracy,
        "max_step_size": args.max_step_size,
        "margin": args.margin,
        "barrier": args.barrier,
        "sim_time": context.get_time(),
        "wall_clock": wall_time,
        "beta": args.beta,
        "num_steps_taken": simulator.get_num_steps_taken(),
        "integrator_stats": {
            name: value
            for (name, value) in integrator.GetStatisticsSummary()
        },
    }
    with open(args.summary_file, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Summary written to {args.summary_file}")

if args.stats_file is not None and config.integration_scheme == "cenic":
    stats = integrator.get_step_statistics()
    with open(args.stats_file, "w") as f:
        f.write(
            "step_type\ttime\tstep_size\tnum_solver_iterations\t"
            "total_linesearch_iterations\tmax_linesearch_iterations\t"
            "mean_linesearch_iterations\tmax_condition_number\t"
            "last_condition_number\tmax_e0\tmean_e0\t"
            "total_num_constraint_pairs\n"
        )
        for stat in stats:
            f.write(stat.to_string() + "\n")
    print(f"Step statistics written to {args.stats_file}")

if args.visualize:
    input("\nWaiting for meshcat... press [ENTER] to quit")
