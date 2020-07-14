/** @file

 Related to convex_collision_cost.cc. The big difference is that rather than
 run a parameterized test based on varying mesh complexity, this simply loads
 the refrigerator used by fleet learning and runs queries on it.
 */
#include <string>

#include "drake/common/profiler.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
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

using lcm::DrakeLcm;
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
  // Build diagram
  auto diagram = builder.Build();
  // Allocate context
  auto context = diagram->AllocateContext();
  diagram->SetDefaultContext(context.get());

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
  for (int i = 0; i < 10; ++i) {
    const auto& results = qo.ComputePointPairPenetration();
    std::cout << i << " has " << results.size() << " contacts\n";
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
