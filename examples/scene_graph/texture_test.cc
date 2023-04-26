/* Add a bunch of different objs to scene graph and examine how they display
 in visualization and rendering. */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/examples/scene_graph/solar_system.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_string(dir, "",
              "If none provided, files are written to a temporary directory. "
              "Otherwise in the given dir.");

namespace drake {
namespace geometry {
namespace test {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace fs = std::filesystem;

constexpr char obj_format[] = R"""(
{}

v -1.0 -1.0 -1.0
v -1.0 1.0 -1.0
v 1.0 -1.0 -1.0
v 1.0 1.0 -1.0
v -1.0 -1.0 1.0
v -1.0 1.0 1.0
v 1.0 -1.0 1.0
v 1.0 1.0 1.0
vn 1 0 0
vn -1 0 0
vn 0 1 0
vn 0 -1 0
vn 0 0 1
vn 0 0 -1
vt 0.0 0.0
vt 0.0 1.0
vt 1.0 0.0
vt 1.0 1.0
{}
f 1/1/6 2/2/6 4/4/6 3/3/6
f 1/1/4 3/2/4 7/4/4 5/3/4
f 1/1/2 5/2/2 6/4/2 2/3/2
f 8/1/5 6/2/5 5/4/5 7/3/5
f 8/1/1 7/2/1 3/4/1 4/3/1
f 8/1/3 4/2/3 2/4/3 6/3/3
)""";

constexpr char mtl_contents[] = R"""(
newmtl mat
Kd 1 1 1
map_Kd 4_color.png
)""";

constexpr char mtl_name[] = "test.mtl";

constexpr char use_mtl[] = "mtllib test.mtl";
constexpr char comment_mtl[] = "# mtllib test.mtl";
constexpr char no_mtl[] = "";

class Populater {
 public:
  Populater(std::string_view possible_dir) {
    dir_ = possible_dir.empty() ? temp_directory() : possible_dir;
    std::cout << "Temp directory: " << dir_ << "\n";
    {
      const fs::path mtl_path = dir_ / mtl_name;
      WriteFile(mtl_path, mtl_contents);
    }
    // Texture referenced in mtl file.
    {
      const std::string tex_source =
          FindResourceOrThrow("drake/examples/scene_graph/4_color.png");
      const fs::path tex_dest = dir_ / "4_color.png";
      fs::remove(tex_dest);
      fs::copy(tex_source, tex_dest);
    }
    // Override texture.
    {
      const std::string tex_source =
          FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
      const fs::path tex_dest = dir_ / "diag_gradient.png";
      fs::remove(tex_dest);
      fs::copy(tex_source, tex_dest);
      alt_texture_ = tex_dest.string();
    }
  }

  void FillSceneGraph(SceneGraph<double>* scene_graph) {
    constexpr double kDelta = 0.25;
    source_id_ = scene_graph->RegisterSource("populater");
    constexpr double offset_0 = -2.5;
    double offset = offset_0;

    std::vector<Config> configs{
        // With usemtl.
        {.name = "no_mtl_no_phong",
         .p_FG = Vector3d((offset = offset_0) * kDelta, -kDelta, 0),
         .mtllib = no_mtl,
         .mat = "mat"},
        {.name = "no_mtl_phong",
         .p_FG = Vector3d(++offset * kDelta, -kDelta, 0),
         .mtllib = no_mtl,
         .mat = "mat",
         .rgba = Rgba(0.2, 0.9, 0.4)},
        {.name = "comment_mtl_no_phong",
         .p_FG = Vector3d(++offset * kDelta, -kDelta, 0),
         .mtllib = comment_mtl,
         .mat = "mat"},
        {.name = "comment_mtl_phong",
         .p_FG = Vector3d(++offset * kDelta, -kDelta, 0),
         .mtllib = comment_mtl,
         .mat = "mat",
         .rgba = Rgba(0.2, 0.9, 0.4)},
        {.name = "mtl_no_phong",
         .p_FG = Vector3d(++offset * kDelta, -kDelta, 0),
         .mtllib = use_mtl,
         .mat = "mat"},
        {.name = "mtl_phong",
         .p_FG = Vector3d(++offset * kDelta, -kDelta, 0),
         .mtllib = use_mtl,
         .mat = "mat",
         .rgba = Rgba(0.2, 0.9, 0.4)},
        // No usemtl.
        {.name = "no_mtl_no_phong_map",
         .p_FG = Vector3d((offset = offset_0) * kDelta, 0, 0),
         .mtllib = no_mtl,
         .mat = ""},
        {.name = "no_mtl_phong_map",
         .p_FG = Vector3d(++offset * kDelta, 0, 0),
         .mtllib = no_mtl,
         .mat = "",
         .rgba = std::nullopt,
         .texture = (dir_ / "diag_gradient.png").string()},
        {.name = "comment_mtl_no_phong_map",
         .p_FG = Vector3d(++offset * kDelta, 0, 0),
         .mtllib = comment_mtl,
         .mat = ""},
        {.name = "comment_mtl_phong_map",
         .p_FG = Vector3d(++offset * kDelta, 0, 0),
         .mtllib = comment_mtl,
         .mat = "",
         .rgba = std::nullopt,
         .texture = (dir_ / "diag_gradient.png").string()},
        {.name = "mtl_no_phong_map",
         .p_FG = Vector3d(++offset * kDelta, 0, 0),
         .mtllib = use_mtl,
         .mat = ""},
        {.name = "mtl_phong_map",
         .p_FG = Vector3d(++offset * kDelta, 0, 0),
         .mtllib = use_mtl,
         .mat = "",
         .texture = (dir_ / "diag_gradient.png").string()},
        // Usemtl with bad material.
        {.name = "no_mtl_no_phong_map_bad_usemtl",
         .p_FG = Vector3d((offset = offset_0) * kDelta, kDelta, 0),
         .mtllib = no_mtl,
         .mat = "bad"},
        {.name = "no_mtl_phong_map_bad_usemtl",
         .p_FG = Vector3d(++offset * kDelta, kDelta, 0),
         .mtllib = no_mtl,
         .mat = "bad",
         .rgba = std::nullopt,
         .texture = (dir_ / "diag_gradient.png").string()},
        {.name = "comment_mtl_no_phong_map_bad_usemtl",
         .p_FG = Vector3d(++offset * kDelta, kDelta, 0),
         .mtllib = comment_mtl,
         .mat = "bad"},
        {.name = "comment_mtl_phong_map_bad_usemtl",
         .p_FG = Vector3d(++offset * kDelta, kDelta, 0),
         .mtllib = comment_mtl,
         .mat = "bad",
         .rgba = std::nullopt,
         .texture = (dir_ / "diag_gradient.png").string()},
        {.name = "mtl_no_phong_map_bad_usemtl",
         .p_FG = Vector3d(++offset * kDelta, kDelta, 0),
         .mtllib = use_mtl,
         .mat = "bad"},
        {.name = "mtl_phong_map_bad_usemtl",
         .p_FG = Vector3d(++offset * kDelta, kDelta, 0),
         .mtllib = use_mtl,
         .mat = "bad",
         .texture = (dir_ / "diag_gradient.png").string()},
    };
    for (const auto& config : configs) {
      AddMesh(scene_graph,
              WriteObj(config.mtllib, config.mat,
                       fmt::format("{}.obj", config.name)),
              RigidTransformd(config.p_FG), config.name, config.rgba,
              config.texture);
    }
  }

 private:
  static void WriteFile(const fs::path& path, const std::string_view contents) {
    fs::remove(path);
    std::ofstream file(path);
    DRAKE_DEMAND(file.is_open());
    file << contents;
  }

  struct Config {
    std::string_view name;
    Vector3d p_FG;
    std::string_view mtllib{};
    std::string_view mat;
    std::optional<Rgba> rgba;
    std::string_view texture;
  };

  void AddMesh(SceneGraph<double>* scene_graph, const fs::path obj_path,
               const RigidTransformd& X_FG, std::string_view name,
               std::optional<Rgba> color, std::string_view texture = "") {
    const GeometryId id = scene_graph->RegisterAnchoredGeometry(
        source_id_, std::make_unique<GeometryInstance>(
                        X_FG, Mesh(obj_path.string(), 0.1), std::string(name)));
    IllustrationProperties props;
    if (color.has_value()) {
      props.AddProperty("phong", "diffuse", *color);
    }
    if (!texture.empty()) {
      props.AddProperty("phong", "diffuse_map", texture);
    }
    scene_graph->AssignRole(source_id_, id, props);
  }

  fs::path WriteObj(const std::string_view mtllib,
                    const std::string_view mat_name,
                    const std::string_view obj_name) {
    const std::string obj_contents =
        fmt::format(obj_format, mtllib,
                    mat_name.empty() ? "" : fmt::format("usemtl {}", mat_name));
    const fs::path obj_path = dir_ / obj_name;
    WriteFile(obj_path, obj_contents);
    return obj_path;
  }

  fs::path dir_;
  SourceId source_id_;
  std::string alt_texture_;
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto* scene_graph = builder.AddSystem<SceneGraph<double>>();

  Populater(FLAGS_dir).FillSceneGraph(scene_graph);

  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  auto meshcat = std::make_shared<Meshcat>();
  MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph, meshcat, {});

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.005);
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();
  simulator.AdvanceTo(0.1);

  sleep(15);

  return 0;
}
}  // namespace test
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::test::do_main();
}
