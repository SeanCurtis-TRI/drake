#include <iostream>
#include <limits>

#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_string(url, "", "URL of the model to load (e.g. package://pkg/model.sdf).");

namespace drake {
namespace {
int do_main() {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlantConfig config{
      .time_step = 0.01,
      .contact_model = "point",
  };
  auto [plant, scene_graph] = multibody::AddMultibodyPlant(config, &builder);
  multibody::Parser parser(&builder);
  parser.package_map().AddPackageXml(
      "/home/seancurtis/troubleshooting/npfaff/reuse_objs/scene_174/"
      "package.xml");
  parser.AddModelsFromUrl(FLAGS_url);
  plant.Finalize();

  fmt::print("Model contains {} total proximity geometries.\n",
             scene_graph.model_inspector().NumGeometriesWithRole(
                 geometry::Role::kProximity));

  fmt::print("Hit Enter to continue...");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return 0;
}
}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::do_main();
}