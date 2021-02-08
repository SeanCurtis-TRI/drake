#include <iostream>
#include <memory>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/slicing/mesh_slice.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace slicing {

using Eigen::Vector3d;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::Rgba;
using geometry::internal::PosedHalfSpace;
using math::RigidTransformd;
using std::make_unique;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto& scene_graph = *builder.AddSystem<geometry::SceneGraph<double>>();
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  IllustrationProperties diffuse;
  diffuse.AddProperty("phong", "diffuse", Rgba{1.0, 0.7, 0.7, 1.0});
  IllustrationProperties plane_diffuse;
  plane_diffuse.AddProperty("phong", "diffuse", Rgba{0.7, 0.7, 1.0, 0.25});

  const FrameId world_id = scene_graph.model_inspector().world_frame_id();
  const FrameId frame_id = world_id;
  // Add initial box.
  const geometry::SourceId source_id = scene_graph.RegisterSource("Main");
  const std::string box_path =
      FindResourceOrThrow("drake/examples/slicing/box.obj");
  GeometryId id = scene_graph.RegisterGeometry(
      source_id, frame_id,
      make_unique<GeometryInstance>(
          RigidTransformd{}, make_unique<Mesh>(box_path, 1.0), "original"));
  scene_graph.AssignRole(source_id, id, diffuse);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& sg_context = scene_graph.GetMyMutableContextFromRoot(context.get());
  diagram->Publish(*context);

  std::string cut_prefix = "/home/seancurtis/temp/slicing/box";
  int i = 0;
  // With each cut, I'll cut the last resulting mesh in the next cut.
  GeometryId to_cut = id;
  // The pose of the geometry to cut (source geometry) in the world.
  RigidTransformd X_WS;

  GeometryId plane_id;  // default to invalid.

  const std::vector<PosedHalfSpace<double>> cut_planes{
      {Vector3d{0, 0, 1}, Vector3d{0, 0, 0}, true},
      {Vector3d{1, 2, 3}.normalized(), Vector3d{0, 0, -0.1}, true},
      {Vector3d{1, 0, 0}, Vector3d{0.25, 0, 0}, true}};
  for (const auto& hs_W : cut_planes) {
    std::string junk;
    std::cerr << "\n\nType something + enter to cut: ";
    std::cin >> junk;

    const RigidTransformd X_WS_original = X_WS;
    const std::string cut_prefix_original = cut_prefix;

    const Vector3d n_S = X_WS.rotation().inverse() * hs_W.normal();
    const Vector3d p_SP =
        X_WS.inverse() * hs_W.boundary_plane().point_on_plane();
    auto cuts = SliceMesh(to_cut, 1.0, p_SP, n_S, scene_graph, sg_context);
    if (cuts.size() > 0) {
      scene_graph.RemoveGeometry(&sg_context, source_id, to_cut);
      for (size_t c = 0; c < cuts.size(); ++c) {
        const auto& mesh_cut = cuts[c];
        cut_prefix = fmt::format("{}{}", cut_prefix_original, c);
        const std::string obj_name = cut_prefix + ".obj";
        WriteSurfaceMeshToObj(mesh_cut.mesh, obj_name);
        to_cut = scene_graph.RegisterGeometry(
            &sg_context, source_id, frame_id,
            make_unique<GeometryInstance>(X_WS_original * mesh_cut.X_SourceCut,
                                          make_unique<Mesh>(obj_name, 1.0),
                                          fmt::format("original{}", ++i)));
        scene_graph.AssignRole(&sg_context, source_id, to_cut, diffuse);
        X_WS = X_WS_original * mesh_cut.X_SourceCut;
      }
    }
    if (plane_id.is_valid()) {
      scene_graph.RemoveGeometry(&sg_context, source_id, plane_id);
    }
    plane_id = scene_graph.RegisterGeometry(
        &sg_context, source_id, world_id,
        make_unique<GeometryInstance>(
            geometry::HalfSpace::MakePose(
                hs_W.normal(), hs_W.boundary_plane().point_on_plane()),
            make_unique<geometry::HalfSpace>(), "cutting plane"));

    scene_graph.AssignRole(&sg_context, source_id, plane_id, plane_diffuse);

    diagram->Publish(*context);
  }

  return 0;
}
}  // namespace slicing
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::slicing::do_main();
}
