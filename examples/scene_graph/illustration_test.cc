#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 0.01,
              "Desired duration of the simulation in seconds.");
DEFINE_string(temp_dir, "",
              "The directory to the temporary obj files into. If none given "
              "temporary directory will be created.");
DEFINE_bool(cleanup, false,
            "If true, the temporary directory will be emptied at conclusion");

namespace drake {
namespace examples {
namespace illustration_test {
namespace {

using geometry::DrakeVisualizerd;
using geometry::DrakeVisualizerParams;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::Meshcat;
using geometry::MeshcatParams;
using geometry::MeshcatVisualizer;
using geometry::MeshcatVisualizerParams;
using geometry::Rgba;
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using math::RigidTransformd;

namespace fs = std::filesystem;

struct TempDir {
  fs::path path;
  bool cleanup;
  bool remove_dir;
};

std::string ReadContents(fs::path path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void WriteFile(fs::path file_path, std::string_view data) {
  std::ofstream f(file_path);
  f << data;
  // We'll assume it worked.
}

class SphereMaker {
 public:
  SphereMaker(double radius, SceneGraph<double>* scene_graph, fs::path temp_dir)
      : radius_(radius),
        scene_graph_(scene_graph),
        temp_dir_(std::move(temp_dir)),
        s_id_(scene_graph->RegisterSource("sphere_maker")) {
    DRAKE_DEMAND(scene_graph_ != nullptr);
    props_.AddProperty("phong", "diffuse", Rgba(1, 0.5, 0));
    pos_ = ReadContents(
        FindResourceOrThrow("drake/examples/scene_graph/vertex_positions.txt"));
    norm_ = ReadContents(
        FindResourceOrThrow("drake/examples/scene_graph/vertex_normals.txt"));
    uvs_ = ReadContents(
        FindResourceOrThrow("drake/examples/scene_graph/vertex_uvs.txt"));
    faces_ = ReadContents(
        FindResourceOrThrow("drake/examples/scene_graph/faces.txt"));

    fs::copy(FindResourceOrThrow("drake/examples/scene_graph/purple.png"),
             temp_dir_ / "purple.png");
    WriteFile(temp_dir_ / "default.mtl", R"""(
newmtl material
)""");
    WriteFile(temp_dir_ / "simple.mtl", R"""(
newmtl material
Kd 0.25 0.5 1.0
)""");
    WriteFile(temp_dir_ / "textured.mtl", R"""(
newmtl material
map_Kd purple.png
)""");
    WriteFile(temp_dir_ / "specular.mtl", R"""(
newmtl material
Ks 1 1 1
Ns 1.000
map_Ks purple.png
)""");
  }

  void AddSphere(const Vector3<double>& p_WS,
                 const std::string& name, const std::string& mesh_data) {
    const fs::path mesh_path = temp_dir_ / name;
    fmt::print("bpy.ops.import_scene.obj(filepath=\"{}\")\n",
               mesh_path.string());
    fmt::print("bpy.ops.transform.resize(value=({0}, {0}, {0}))\n", radius_);
    fmt::print("bpy.ops.transform.translate(value=({}, {}, {}))\n", p_WS.x(),
               p_WS.y(), p_WS.z());
    WriteFile(mesh_path, mesh_data);
    const GeometryId g_id = scene_graph_->RegisterAnchoredGeometry(
        s_id_,
        std::make_unique<GeometryInstance>(
            RigidTransformd(p_WS), Mesh(mesh_path.string(), radius_), name));
    scene_graph_->AssignRole(s_id_, g_id, props_);
  }

  std::string MakeSphere(std::string_view mtl_lib, std::string_view material) {
    std::stringstream ss;
    if (!mtl_lib.empty()) {
      ss << "mtllib " << mtl_lib << "\n";
    }
    ss << pos_ << "\n";
    ss << norm_ << "\n";
    ss << uvs_ << "\n";
    if (!material.empty()) {
      ss << "usemtl " << material << "\n";
    }
    ss << faces_;
    return ss.str();
  }

 private:
  const double radius_;
  SceneGraph<double>* scene_graph_;
  fs::path temp_dir_;
  const SourceId s_id_;
  IllustrationProperties props_;
  std::string pos_;
  std::string norm_;
  std::string uvs_;
  std::string faces_;
};

TempDir GetTempDir(fs::path temp_dir) {
  bool program_made = false;
  if (temp_dir.empty() || !fs::exists(temp_dir) ||
      !fs::is_directory(temp_dir)) {
    temp_dir = fs::path(temp_directory());
    program_made = true;
  }
  fmt::print("\nUsing directory: {}\n\n", temp_dir.string());
  return {
      .path = temp_dir, .cleanup = FLAGS_cleanup, .remove_dir = program_made};
}

class PositionArray {
 public:
  PositionArray(int rows, int cols, double distance)
      : rows_(rows), cols_(cols), dist_(distance) {
    init_x_ = -(cols_ - 1) * distance * 0.5;
    init_z_ = -(cols_ - 1) * distance * 0.5;
    // init_y is implicitly zero.
  }

  Vector3<double> next() {
    const int layer_size = cols_ * rows_;
    const int curr_layer = curr_ / layer_size;
    const int layer_index = curr_ % layer_size;
    const double x = init_x_ + (layer_index % cols_) * dist_;
    const double z = init_z_ + (layer_index / cols_) * dist_;
    const double y = curr_layer * dist_;
    ++curr_;
    return {x, y, z};
  }

 private:
  const int rows_;
  const int cols_;
  const double dist_;
  double init_x_;
  double init_z_;
  int curr_{0};
};

/* Simply creates an array of *anchored* spheres with varying properties. */
void AddGeometry(SceneGraph<double>* scene_graph, const TempDir& temp_dir) {
  constexpr double kRadius = 0.3;
  SphereMaker maker(kRadius, scene_graph, temp_dir.path);
  PositionArray pos(3, 3, kRadius * 2.5);
  maker.AddSphere(pos.next(), "no_material.obj", maker.MakeSphere("", ""));
  maker.AddSphere(pos.next(), "bad_mtl.obj", maker.MakeSphere("not.mtl", ""));
  maker.AddSphere(pos.next(), "valid_mtl_no_mat.obj",
                  maker.MakeSphere("default.mtl", ""));
  maker.AddSphere(pos.next(), "valid_mtl_default_mat.obj",
                  maker.MakeSphere("default.mtl", "material"));
  maker.AddSphere(pos.next(), "valid_mtl_diffuse_mat.obj",
                  maker.MakeSphere("simple.mtl", "material"));
  maker.AddSphere(pos.next(), "valid_mtl_textured_diffuse_mat.obj",
                  maker.MakeSphere("textured.mtl", "material"));
  maker.AddSphere(pos.next(), "valid_mtl_textured_specular_mat.obj",
                  maker.MakeSphere("specular.mtl", "material"));
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");

  const TempDir temp_dir = GetTempDir(FLAGS_temp_dir);
  AddGeometry(scene_graph, temp_dir);

  DrakeVisualizerParams drake_vis_params{
      .default_color = {Rgba{0.0, 0.0, 0.5}}};
  DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, nullptr,
                                 drake_vis_params);
  MeshcatParams meshcat_params{.host = "localhost", .port = 8001};
  auto meshcat = std::make_shared<Meshcat>(meshcat_params);
  MeshcatVisualizerParams meshcat_vis_params{.default_color =
                                                 Rgba{0.0, 0.5, 0}};
  MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph, meshcat);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  fmt::print("Hit enter to exit: ");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  if (temp_dir.cleanup) {
    fmt::print("\nCleaning up temporary files\n");
    if (temp_dir.remove_dir) {
      fmt::print("    Removing temp directory\n");
      fs::remove_all(temp_dir.path);
    } else {
      for (auto const& dir_entry : fs::directory_iterator{temp_dir.path}) {
        fs::remove(dir_entry.path());
      }
    }
  }
  return 0;
}

}  // namespace
}  // namespace illustration_test
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::illustration_test::do_main();
}
