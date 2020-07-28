/** @file

 Related to convex_collision_cost.cc. The big difference is that rather than
 run a parameterized test based on varying mesh complexity, this simply loads
 the refrigerator used by fleet learning and runs queries on it.
 */
#include <string>

#include <gflags/gflags.h>

#include "drake/common/profiler.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

DEFINE_bool(has_contact, true,
            "If true, the box will be in contact with the fridge. If false "
            "the box will be near the fridge, but not in contact");
DEFINE_bool(as_convex, true,
            "If true, the collision geometry is convex meshes. If false, they "
            "are boxes");

namespace drake {
namespace geometry {
namespace profiling {
namespace convex {

using Eigen::Vector3d;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::Context;
using systems::DiagramBuilder;
using systems::lcm::LcmPublisherSystem;

int do_main() {
  DiagramBuilder<double> builder;
  auto& mbp = *builder.AddSystem<MultibodyPlant>(0.0);
  auto& sg = *builder.AddSystem<SceneGraph>();
  // Parse refrigerator.
  Parser parser(&mbp, &sg);
  const std::string fridge_path =
      "/home/seancurtis/Downloads/fl_convex/Fridge/";
  const std::string fridge =
      fridge_path + (FLAGS_as_convex ? "Fridge.urdf" : "FridgeBox.urdf");
  parser.AddAllModelsFromFile(fridge);
  mbp.Finalize();

  builder.Connect(sg.get_query_output_port(),
                  mbp.get_geometry_query_input_port());
  builder.Connect(mbp.get_geometry_poses_output_port(),
                  sg.get_source_pose_port(mbp.get_source_id().value()));

  // Visualize (optionally)
  ConnectDrakeVisualizer(&builder, sg, nullptr, Role::kProximity);

#if 0
  const RigidTransformd X_WS{};
  const RigidTransformd X_SF{Vector3d(0, 0, 0.8)};
#else
  const RigidTransformd X_WS{RotationMatrixd::MakeZRotation(M_PI / 7)};
  const RigidTransformd X_SF{RotationMatrixd::MakeZRotation( M_PI) *
                             RotationMatrixd::MakeXRotation(M_PI / 2),
                             Vector3d(0, 0, 0.8)};
#endif

  const geometry::SourceId s_id = sg.RegisterSource("main_program");
  const double p_SBx = FLAGS_has_contact ? 0.9 : 0.95;
  const RigidTransformd X_SB{Vector3d(p_SBx, -0.2, 0.45)};
  auto geo = std::make_unique<geometry::GeometryInstance>(
      X_WS * X_SB, std::make_unique<geometry::Box>(1.0, 1.0, 1.0), "counter");
  geometry::ProximityProperties props;
  const geometry::GeometryId box_id = geo->id();
  geo->set_proximity_properties(std::move(props));
  sg.RegisterAnchoredGeometry(s_id, std::move(geo));
  DRAKE_DEMAND(sg.model_inspector().GetProximityProperties(box_id));

  // Build diagram
  auto diagram = builder.Build();
  // Allocate context
  auto context = diagram->AllocateContext();
  context->DisableCaching();
  diagram->SetDefaultContext(context.get());

  // Pose the fridge. Simply by making sure the rotation matrix *isn't* the
  // identity, we'll guarantee overlaping bounding boxes.
  auto& mbp_context = mbp.GetMyMutableContextFromRoot(context.get());
  const auto& fridge_body = mbp.GetBodyByName("body");
  mbp.SetFreeBodyPose(&mbp_context, fridge_body, X_WS * X_SF);

  // Initialize things.
  auto init_events = diagram->AllocateCompositeEventCollection();
  diagram->GetInitializationEvents(*context, init_events.get());
  if (init_events->HasEvents()) {
    diagram->Publish(*context, init_events->get_publish_events());
  }
  diagram->Publish(*context);

  // Perform a bunch of queries.
  const auto& sg_context = diagram->GetSubsystemContext(sg, *context);
  //  Possibly collect data based on mesh complexity?
  const auto& qo =
      sg.get_query_output_port().Eval<QueryObject<double>>(sg_context);
  int contact_count = -1;
  for (int i = 0; i < 50; ++i) {
    const auto& results = qo.ComputePointPairPenetration();
    if (contact_count < 0) {
      contact_count = static_cast<int>(results.size());
      std::cout << i << " has " << contact_count << " contacts\n";
    } else if (contact_count != static_cast<int>(results.size())) {
      std::cout << i << " has " << contact_count << " contacts\n";
    }
  }

  std::cout << TableOfAverages() << "\n";

  return 0;
}
}  // namespace convex
}  // namespace profiling
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::profiling::convex::do_main();
}
