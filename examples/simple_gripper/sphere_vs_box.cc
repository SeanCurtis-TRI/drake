#include <memory>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a"\n" << a << std::endl;

namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::Box;
using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
using drake::multibody::RigidBody;
using drake::multibody::WeldJoint;
using drake::multibody::PrismaticJoint;
using drake::lcm::DrakeLcm;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::ContactResultsToLcmSystem;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::PrismaticJoint;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::RungeKutta2Integrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;
using drake::systems::Sine;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(target_realtime_rate, 0.1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

// we'll just run four time steps.
DEFINE_double(simulation_time, 0.004,
              "Desired duration of the simulation. [s].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");
DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");
DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates of period 'max_time_step'."
    "If 'false', the plant is modeled as a continuous system.");

// Contact parameters
DEFINE_double(penetration_allowance, 1.0e-4,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-3,
              "The maximum slipping speed allowed during stiction. [m/s]");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  DRAKE_DEMAND(FLAGS_max_time_step > 0);

  MultibodyPlant<double>& plant =
      FLAGS_time_stepping ?
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step) :
      *builder.AddSystem<MultibodyPlant>();

  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // Build model.
  const double radius = 0.0015;
  RigidTransform<double> X_WSphere(RollPitchYaw<double>(
      Vector3d(-3.04739982050051, -1.57079632679489, -3.13972548667908)).ToRotationMatrix(),
                                   {0.184010155136233, 0.0278488077094353, 0.0100348279363227});

  const Vector3d box_size(0.115, 0.01, 0.0025);
  RigidTransform<double> X_WBox(RollPitchYaw<double>(
      Vector3d(-3.13860212217648, -1.53862510693394, -1.57380202520141)).ToRotationMatrix(),
                                   {0.183005780540596, 0.0314497444837003, -0.0394187438155612});

  PRINT_VARn(X_WBox.GetAsIsometry3().matrix());
  PRINT_VARn(X_WSphere.GetAsIsometry3().matrix());

  X_WSphere = X_WBox.inverse() * X_WSphere;
  X_WBox = RigidTransform<double>::Identity();
  PRINT_VARn(X_WBox.GetAsIsometry3().matrix());
  PRINT_VARn(X_WSphere.GetAsIsometry3().matrix());

  std::cout << "Sphere position: " << X_WSphere.translation().transpose() << "\n";

  double box_mass = 0.094;
  SpatialInertia<double> Mbox_B =
      SpatialInertia<double>::MakeFromCentralInertia(
          box_mass, Vector3d::Zero(),
          RotationalInertia<double>(0.000156, 0.000156, 0.00015));

  double sphere_mass = 0.05;
  SpatialInertia<double> Msphere_S =
      SpatialInertia<double>::MakeFromCentralInertia(
          sphere_mass, Vector3d::Zero(),
          RotationalInertia<double>(0.16, 0.16, 0.16));


  const RigidBody<double>& box = plant.AddRigidBody("Box", Mbox_B);
  plant.AddJoint<WeldJoint>("WeldBox", plant.world_body(), {}, box, {}, X_WBox.GetAsIsometry3());
  const geometry::VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 0.5));
  plant.RegisterVisualGeometry(
      box, Isometry3d::Identity(), Box(box_size(0),box_size(1),box_size(2)), red, &scene_graph);
  plant.RegisterCollisionGeometry(
      box, Isometry3d::Identity(), Box(box_size(0),box_size(1),box_size(2)), multibody::parsing::default_friction(), &scene_graph);

  const RigidBody<double>& sphere = plant.AddRigidBody("Sphere", Msphere_S);
  //plant.AddJoint<WeldJoint>("WeldSphere", plant.world_body(), {}, sphere, {}, X_WSphere.GetAsIsometry3());
  const geometry::VisualMaterial magenta(Vector4<double>(1.0, 0.0, 1.0, 1.0));
  plant.RegisterVisualGeometry(
      sphere, Isometry3d::Identity(), Sphere(radius), magenta, &scene_graph);
  plant.RegisterCollisionGeometry(
      sphere, Isometry3d::Identity(), Sphere(radius), multibody::parsing::default_friction(), &scene_graph);

  // Now the model is complete.
  plant.Finalize(&scene_graph);

  PRINT_VAR(plant.num_velocities());
  PRINT_VAR(plant.num_positions());

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_max_time_step > 0 ? FLAGS_max_time_step :
      plant.get_contact_penalty_method_time_scale() / 30;

  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", max_time_step);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  //publisher.set_publish_period(0.0001);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Publish contact results for visualization.
  const auto& contact_results_to_lcm =
      *builder.AddSystem<ContactResultsToLcmSystem>(plant);
  const auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_results_to_lcm.get_input_port(0));
  builder.Connect(contact_results_to_lcm.get_output_port(0),
                  contact_results_publisher.get_input_port());

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& diagram_context = simulator.get_mutable_context();

  diagram->SetDefaultContext(&diagram_context);
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &diagram_context);

  (void) plant_context;

  plant.model().SetFreeBodyPoseOrThrow(sphere, X_WSphere.GetAsIsometry3(), &plant_context);

  // Set up simulator.
  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<RungeKutta2Integrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  // publish some debug info
  fmt::print("Plant num positions: {}\n", plant.num_positions());
  fmt::print("Plant num velocities: {}\n", plant.num_velocities());

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  // Print some stats for variable time step integrators.
  fmt::print("Integrator stats:\n");
  fmt::print("Number of time steps taken = {:d}\n",
             integrator->get_num_steps_taken());
  if (!integrator->get_fixed_step_mode()) {
    fmt::print("Initial time step taken = {:10.6g} s\n",
               integrator->get_actual_initial_step_size_taken());
    fmt::print("Largest time step taken = {:10.6g} s\n",
               integrator->get_largest_step_size_taken());
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
               integrator->get_smallest_adapted_step_size_taken());
  }

  PRINT_VAR(simulator.get_num_discrete_updates());
  PRINT_VAR(simulator.get_num_steps_taken());

  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Demo used to exercise MultibodyPlant's contact modeling in a gripping "
      "scenario. SceneGraph is used for both visualization and contact "
      "handling. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::simple_gripper::do_main();
}
