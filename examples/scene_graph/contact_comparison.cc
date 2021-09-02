/** @file

 This program provides a basis for exploring different aspects of Drake's
 contact models.

 The scenario consists of two unit boxes. The experiment can be run in four
 modes:

   - point


 Visualization configuration

   - build drake_visualizer
     bazel build //tools:drake_visualizer
   - run drake_visualizer
     ./bazel-bin/tools/drake_visualizer
   - In the menu, select:
     Plugins -> Contacts -> Configure Force Vector For Point Contacts
     - set "Vector scaling mode" to "Scaled"
     - set "Global scale" to 1
   - In the menu, select:
     Plugins -> Contacts -> Configure Hydroelastic Contact Visualization
      - Set "Vector scalaing mode" to "Scaled".
      - set "Global scale of all vectors" to 1.
      - Set "Render moment vector" to off.
      - Set "Maximum pressure" to 7.
   - In the menu, select
     Plugins -> Camera -> Configure Camera
      - Set "Set Orthographic" to on.
      - Set "Orient to axis" to +Y
      - Set camera position to (-0.45, -10, 0.45)
      - Set Ortho. scale to 0.5.
      - Press "Set Camera Position" .
      - Press "Read Focal Point"
      - Set focal point Y to 0.0
      - Press "Set Focal Point"
 */
#include <chrono>
#include <thread>
#include <utility>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace contact_comparison {

using Eigen::Vector3d;
using geometry::DrakeVisualizer;
using geometry::ProximityProperties;
using geometry::Rgba;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::ContactResultsToLcmSystem;
using multibody::CoulombFriction;
using multibody::SpatialInertia;
using std::move;
using systems::Diagram;
using systems::DiagramBuilder;

DEFINE_double(duration, 4.0, "Amount of time to run the experiment series.");
DEFINE_int32(fps, 30, "The number of samples per second");
/* This default delay of 20 ms seems to be enough to allow the contact messages
 to be processed, drawn, and captured. */
DEFINE_double(delay, 0.02,
              "The amount of time we delay between Publish calls.");
DEFINE_string(experiment, "point",
              "Select the experiment to run. Should be one of [point, "
              "equal_soft, rigid_soft, varying_soft]");

/* Computes the position of the moving box for the given time. The moving box
 center moves along a circular arc. This assumes knowledge that the two boxes
 are unit boxes and that the stationary box is centered on the origin. */
Vector3d CalcCircleCenter(double t, double duration) {
  const double kMaxDepth = 0.25;
  const double kRadius = 0.3;
  const Vector3d kCircleCenter =
      Vector3d{-1, 0, 1}.normalized() * (std::sqrt(2) + kRadius - kMaxDepth);
  const double kMinTheta = -std::acos((-1 - kCircleCenter.x()) / kRadius) - 0.1;
  const double kMaxTheta = -std::asin((kCircleCenter.z() - 1) / kRadius) + 0.1;
  const double kRange = kMaxTheta - kMinTheta;

  const double min_t = 0;

  const double theta = (t - min_t) / duration * kRange + kMinTheta;

  return kCircleCenter +
         kRadius * Vector3d(std::cos(theta), 0, std ::sin(theta));
}

/* Support the experiment for redistributing elasticity across two soft bodies.
 It starts out as (essentially) (0, ∞) passes through (1e4, 1e4) at the halfway
 point and ends up as (∞, 0). */
std::pair<double, double> CalcElasticity(double t, double duration) {
  const double kMinElasticity = 1;
  const double kMaxElasticity = 40;
  const double kRange = kMaxElasticity - kMinElasticity;

  const double weight = t / duration;

  double elasticity_A = kMinElasticity + kRange * weight;
  double elasticity_B = kMinElasticity + kRange * (1 - weight);

  return std::make_pair(elasticity_A, elasticity_B);
}
enum Experiment { kPoint, kRigidSoft, kEqualSoft, kVaryingSoft };

Experiment ClassifyExperiment(std::string_view description) {
  if (description == "point") return kPoint;
  else if (description == "rigid_soft")
    return kRigidSoft;
  else if (description == "equal_soft") {
    return kEqualSoft;
  } else if (description == "varying_soft") {
    return kVaryingSoft;
  } else {
    throw std::runtime_error(
        fmt::format("Unrecognized experiment request: '{}'. Should be one of "
                    "[point, equal_soft, rigid_soft, varying_soft]",
                    description));
  }
}

int do_main() {
  const Experiment experiment = ClassifyExperiment(FLAGS_experiment);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  const geometry::SourceId source_id = *plant.get_source_id();

  /* The goal is for the force magnitudes of both hydroelastic and point contact
   to have the same magnitude as hydro contact. This decreases the force
   magnitude to be on the same scale (see instructions above). */
  plant.set_penetration_allowance(6.0);
  if (experiment != kPoint) {
    plant.set_contact_model(multibody::ContactModel::kHydroelastic);
  }

  /* Add moving box. */
  const auto& body_A = plant.AddRigidBody(
      "Box_A",
      SpatialInertia<double>(1.0, Vector3d::Zero(),
                             multibody::UnitInertia<double>::SolidCube(1)));
  plant.WeldFrames(plant.world_frame(), {plant.GetFrameByName("Box_A")});
  ProximityProperties prop_A;
  const double elasticity = experiment == kEqualSoft ? 20.0 : 10.0;
  geometry::AddContactMaterial(elasticity, 0.0, CoulombFriction<double>(),
                               &prop_A);
  prop_A.AddProperty("phong", "diffuse", Rgba(1.0, 0.25, 0.25, 0.25));
  if (experiment == kRigidSoft) {
    geometry::AddRigidHydroelasticProperties(1.0, &prop_A);
  } else {
    geometry::AddSoftHydroelasticProperties(1.0, &prop_A);
  }
  plant.RegisterCollisionGeometry(body_A, {}, geometry::Box(1, 1.02, 1),
                                  "collision_A", move(prop_A));
  const geometry::GeometryId geo_A =
      plant.GetCollisionGeometriesForBody(body_A)[0];

  /* Add stationary box. */
  const auto& body_B = plant.AddRigidBody(
      "Box_B",
      SpatialInertia<double>(1.0, Vector3d::Zero(),
                             multibody::UnitInertia<double>::SolidCube(1)));
  ProximityProperties prop_B;
  geometry::AddContactMaterial(elasticity, 0.0, CoulombFriction<double>(),
                               &prop_B);
  prop_B.AddProperty("phong", "diffuse", Rgba(0.25, 0.25, 1.0, 0.25));
  geometry::AddSoftHydroelasticProperties(1.0, &prop_B);
  plant.RegisterCollisionGeometry(body_B, {}, geometry::Box(1, 0.5, 1),
                                  "collision_B", move(prop_B));
  const geometry::GeometryId geo_B =
      plant.GetCollisionGeometriesForBody(body_B)[0];

  plant.Finalize();

  lcm::DrakeLcm lcm;

  geometry::DrakeVisualizerParams params;
  params.role = geometry::Role::kProximity;
  const auto& viz = DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph,
                                                          &lcm, params);

  const auto& contact_viz = ConnectContactResultsToDrakeVisualizer(
      &builder, plant, scene_graph, &lcm);

  const auto diagram = builder.Build();
  const auto diagram_context = diagram->CreateDefaultContext();
  diagram_context->DisableCaching();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());
  auto& sg_context =
      scene_graph.GetMyMutableContextFromRoot(diagram_context.get());

  const int kSampleCount =
      static_cast<int>(FLAGS_fps * FLAGS_duration + 0.5) + 1;
  const double kDt = 1.0 / (FLAGS_fps);

  const auto& inspector = scene_graph.model_inspector();
  if (experiment == kVaryingSoft) {
    math::RigidTransformd X_WB(CalcCircleCenter(0.5, 1.0));
    plant.SetFreeBodyPose(&plant_context, body_B, X_WB);
  }

  for (int s = 0; s < kSampleCount; ++s) {
    const double t = s * kDt;
    diagram_context->SetTime(t);
    if (experiment == kVaryingSoft) {
      auto [elasticity_A, elasticity_B] = CalcElasticity(t, FLAGS_duration);
      ProximityProperties new_prop_A(*inspector.GetProximityProperties(geo_A));
      new_prop_A.UpdateProperty("material", "elastic_modulus", elasticity_A);
      scene_graph.AssignRole(&sg_context, source_id, geo_A, move(new_prop_A),
                             geometry::RoleAssign::kReplace);
      ProximityProperties new_prop_B(*inspector.GetProximityProperties(geo_B));
      new_prop_B.UpdateProperty("material", "elastic_modulus", elasticity_B);
      scene_graph.AssignRole(&sg_context, source_id, geo_B, move(new_prop_B),
                             geometry::RoleAssign::kReplace);
    } else {
      math::RigidTransformd X_WB(CalcCircleCenter(t, FLAGS_duration));
      plant.SetFreeBodyPose(&plant_context, body_B, X_WB);
    }
    diagram->Publish(*diagram_context);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>((kDt + FLAGS_delay) * 1000)));
  }

  unused(viz, contact_viz, plant_context);
  return 0;
}

}  // namespace contact_comparison
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::contact_comparison::do_main();
}
