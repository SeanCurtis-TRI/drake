/** @file

 Related to convex_collision_cost.cc. The big difference is that rather than
 run a parameterized test based on varying mesh complexity, this simply loads
 the refrigerator used by fleet learning and runs queries on it.
 */
#include <string>

#include "drake/common/profiler.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
// #include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

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
  const std::string fridge =
      "/home/seancurtis/Downloads/fl_convex/Fridge/Fridge.urdf";
  parser.AddAllModelsFromFile(fridge);
  mbp.Finalize();

  builder.Connect(sg.get_query_output_port(),
                  mbp.get_geometry_query_input_port());
  builder.Connect(mbp.get_geometry_poses_output_port(),
                  sg.get_source_pose_port(mbp.get_source_id().value()));

  // Visualize (optionally)
  ConnectDrakeVisualizer(&builder, sg, nullptr, Role::kProximity);

  const RigidTransformd X_WS{RotationMatrixd::MakeZRotation(M_PI / 7)};

  const geometry::SourceId s_id = sg.RegisterSource("main_program");
  const RigidTransformd X_SB{Vector3d(0.9, -0.2, 0.45)};
  auto geo = std::make_unique<geometry::GeometryInstance>(
      X_WS * X_SB, std::make_unique<geometry::Box>(1.0, 1.0, 1.0), "counter");
  geometry::ProximityProperties props;
  const geometry::GeometryId box_id = geo->id();
  geo->set_proximity_properties(std::move(props));
  std::cout << "Adding anchored box\n";
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
  const RigidTransformd X_SF{RotationMatrixd::MakeZRotation( M_PI) *
                             RotationMatrixd::MakeXRotation(M_PI / 2),
                             Vector3d(0, 0, 0.8)};
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
  //   gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::profiling::convex::do_main();
}
