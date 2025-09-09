#include <drake/common/find_resource.h>
#include <drake/multibody/parsing/parser.h>

namespace {
constexpr char kModel[] = "drake/examples/multibody/so/TwoLinkRobot.sdf";

void PrintCollisionCandidates(
    const drake::geometry::QueryObject<double>& query_object) {
  const auto &inspector = query_object.inspector();
  const auto collision_candidates = inspector.GetCollisionCandidates();
  fprintf(stdout, "Collision Candidates:\n");
  for (const auto &cc : collision_candidates) {
    fprintf(stdout, "  %s vs %s\n", inspector.GetName(cc.first).c_str(),
            inspector.GetName(cc.second).c_str());
  }
}
}  // namespace

int main(int argc, char **argv) {
  bool add_half_space = false;
  if (argc > 1 && std::string(argv[1]) == "--add-half-space") {
    add_half_space = true;
  }

  constexpr double kSimTimeStep = 1e-2;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph<double>(&builder,
                                                            kSimTimeStep);

  drake::multibody::Parser model_parser(&plant, &scene_graph);
  model_parser.SetAutoRenaming(true);

  const std::string model_path = drake::FindResourceOrThrow(kModel);

  auto models = model_parser.AddModels(model_path);
  if (models.size() < 1) {
    fprintf(stderr, "Failed to load model '%s'\n", kModel);
    return 1;
  }
  auto robot = models[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link", robot));

  // load exactly same model to cause collision
  auto models2 = model_parser.AddModels(model_path);
  if (models2.size() < 1) {
    fprintf(stderr, "Failed to load model2 '%s'\n", kModel);
    return 1;
  }
  auto robot2 = models2[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link", robot2),
                   drake::math::RigidTransformd(Eigen::Vector3d(0, 0, 0)));

  if (add_half_space) {
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        drake::geometry::HalfSpace::MakePose({-1, 0, 0}, {10, 0, 0}),
        drake::geometry::HalfSpace(), "halfspace",
        drake::geometry::ProximityProperties());
  }

  // finalize
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();


  // check collision
  const auto &scene_graph_context =
      diagram->GetSubsystemContext(scene_graph, *diagram_context);
  const auto &query_object =
      scene_graph.get_query_output_port()
          .Eval<drake::geometry::QueryObject<double>>(scene_graph_context);
  // debug
  PrintCollisionCandidates(query_object);
  if (query_object.HasCollisions()) {
    printf("Collision\n");
  } else {
    printf("No Collision\n");
  }
  return 0;
}
