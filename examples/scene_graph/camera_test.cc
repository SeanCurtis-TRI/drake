/* @file Consumes an obj file and renders a single image. */

#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"

DEFINE_string(obj, "", "Path to the obj to render.");
DEFINE_string(out_dir, "",
              "Path to the directory to write to; if empty, will write to the "
              "current directory. The final image is path/to/camera_test.png");
DEFINE_double(dir_x, 0, "X-component of Cz_W");
DEFINE_double(dir_y, 1, "Y-component of Cz_W");
DEFINE_double(dir_z, 0, "Z-component of Cz_W");
DEFINE_double(dolly, 0, "The amount to dolly *towards* the object.");

namespace drake {
namespace examples {
namespace {

namespace fs = std::filesystem;

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::Mesh;
using geometry::PerceptionProperties;
using geometry::ReadObjToTriangleSurfaceMesh;
using geometry::RenderEngineVtkParams;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::TriangleSurfaceMesh;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderLabel;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using systems::sensors::PixelType;
using systems::sensors::RgbdSensor;
using systems::sensors::SaveToPng;

std::string MakeImageName() {
  if (FLAGS_out_dir.empty()) {
    return "./camera_test.png";
  }

  std::string dir_path_s;

  fs::path dir_path(FLAGS_out_dir);
  if (fs::exists(dir_path)) {
    if (fs::is_directory(dir_path)) {
      if (::access(dir_path.string().c_str(), W_OK) == 0) {
        dir_path_s = dir_path.string();
      } else {
        std::cerr << "No access to the requested output directory: "
                  << dir_path << ";";
      }
    } else {
      std::cerr << "Requested output directory is not a directory: "
                << dir_path << ";";
    }
  } else {
    std::cerr << "The requested output directory does not exist: " << dir_path
              << ";";
  }

  if (dir_path_s.empty()) {
    std::cerr << " Using current directory.\n";
    return "./camera_test.png";
  }

  // TODO(SeanCurtis-TRI): It would be good to confirm I can actually write to
  // this file; i.e., there's not currently a file there that I'm going to
  // overwrite.
  return (dir_path / "camera_test.png").string();
}

/* Given a vector direction that the camera is looking (Cz_W) and a mesh, the
 camera is positioned and oriented so that it is looking in the given direction
 with the bounding box of the mesh maximally filled in frame.

 The calculations is subject to gimbal lock; Cz_W should not be too close to Wz
 and should have non-zero magnitude. */
RigidTransformd MakeCameraPose(const TriangleSurfaceMesh<double>& mesh_W,
                               const Vector3d& Cz_W_in,
                               const ColorRenderCamera& camera) {
  // 0. Compute camera orientation.
  const Vector3d Cz_W = Cz_W_in.normalized();
  DRAKE_THROW_UNLESS(std::abs(Cz_W.dot(Vector3d::UnitZ())) < 0.99);
  const Vector3d Cx_W = -Vector3d::UnitZ().cross(Cz_W).normalized();
  const Vector3d Cy_W = Cz_W.cross(Cx_W).normalized();
  const RotationMatrixd R_WC =
      RotationMatrixd::MakeFromOrthonormalColumns(Cx_W, Cy_W, Cz_W);
  const RotationMatrixd R_CW = R_WC.inverse();

  // 1. Compute mesh bounding box in the camera frame.
  Vector3d p_WBmin_C =
      Vector3d::Constant(std::numeric_limits<double>::infinity());
  Vector3d p_WBmax_C = -p_WBmin_C;
  for (const auto& p_WV_W : mesh_W.vertices()) {
    const Vector3d p_WV_C = R_CW * p_WV_W;
    p_WBmin_C = p_WBmin_C.cwiseMin(p_WV_C);
    p_WBmax_C = p_WBmax_C.cwiseMax(p_WV_C);
  }
  std::cout << "Mesh bounding box (in camera):\n"
            << "  min " << p_WBmin_C.transpose() << "\n"
            << "  max " << p_WBmax_C.transpose() << "\n";

  // The center of the box. This will be positioned at the image center.
  const Vector3d p_WBo_C = (p_WBmin_C + p_WBmax_C) / 2;
  std::cout << "Mesh bounding box center (in camera): " << p_WBo_C.transpose() << "\n";

  // Note: in C, the coefficients of max_pt are not all >= than those in min_pt.
  // They are merely the measure of two corners.
  const Vector3d p_BoBmin_C = p_WBmin_C - p_WBo_C;
  const Vector3d p_BoBmax_C = p_WBmax_C - p_WBo_C;
  const Vector3d box_size_C = p_BoBmax_C - p_BoBmin_C;

  // Now determine whether we're fitting to the height or width of the image.
  const double box_width = box_size_C.x();
  const double box_height = box_size_C.y();
  const double box_aspect_ratio = box_height / box_width;
  const double camera_aspect_ratio =
      static_cast<double>(camera.core().intrinsics().height()) /
      camera.core().intrinsics().width();

  std::cout << "Aspect ratios:\n" << "  box: " << box_aspect_ratio << "\n  camera: " << camera_aspect_ratio << "\n";
  // Assume we'll fit the width.
  double box_measure{box_width};
  double fov{camera.core().intrinsics().fov_x()};
  if (box_aspect_ratio > camera_aspect_ratio) {
    // Box is taller and skinnier than image; fit vertically as the limiting
    // factor.
    box_measure = box_height;
    fov = camera.core().intrinsics().fov_y();
    std::cout << "Fitting vertical dimension\n";
  } else {
    std::cout << "Fitting horizontal dimension\n";
  }
  const double focal_distance =
      0.5 * box_measure / std::tan(fov / 2) + box_measure / 2;
  // How far does it have to be so that the *whole* bounding box lies in front
  // of the near clipping plane.
  const double min_distance =
      box_size_C.z() / 2 + camera.core().clipping().near();
  const double camera_distance =
      std::max(focal_distance, min_distance) - FLAGS_dolly;
  std::cout << "Distance:\n"
            << "  focal_distance;  " << focal_distance << "\n"
            << "  min_distance;    " << min_distance << "\n"
            << "  dolly:           " << FLAGS_dolly << "\n"
            << "  camera_distance; " << camera_distance << "\n";

  const Vector3d p_BoCo_C{0, 0, -camera_distance};
  const Vector3d p_WCo_C = p_WBo_C + p_BoCo_C;
  const Vector3d p_WCo_W = R_WC * p_WCo_C;

  // Am I truly centered?
  const Vector3d p_WBo_W = R_WC * p_WBo_C;
  const Vector3d p_CoBo_W = p_WBo_W - p_WCo_W;
  DRAKE_THROW_UNLESS(std::abs(p_CoBo_W.norm() - Cz_W.dot(p_CoBo_W)) < 1e-14);

  return RigidTransformd(R_WC, p_WCo_W);
}

int do_main() {
  if (FLAGS_obj.empty()) {
    std::cerr << "No obj specified!\n";
    return 1;
  }

  std::cerr << "obj: '" << FLAGS_obj << "'\n";

  bool error = false;
  TriangleSurfaceMesh<double> obj_mesh = ReadObjToTriangleSurfaceMesh(
      FLAGS_obj, 1.0, [&error](std::string_view msg) {
        error = true;
        std::cerr << msg << "\n";
      });
  if (error) {
    std::cerr << "Failed to parse obj\n";
    return 1;
  }

  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  const std::string render_name("renderer");
  scene_graph->AddRenderer(render_name,
                           MakeRenderEngineVtk(RenderEngineVtkParams()));

  const SourceId source_id = scene_graph->RegisterSource("photographer");
  const GeometryId geo_id = scene_graph->RegisterAnchoredGeometry(
      source_id,
      make_unique<GeometryInstance>(RigidTransformd{},
                                    make_unique<Mesh>(FLAGS_obj, 1), "obj"));
  PerceptionProperties properties;
  properties.AddProperty("phong", "diffuse", Vector4d{0.8, 0.8, 0.8, 1.0});
  properties.AddProperty("label", "id", RenderLabel(geo_id.get_value()));
  scene_graph->AssignRole(source_id, geo_id, properties);

  // Create the camera.
  const ColorRenderCamera color_camera{
      {render_name, {640, 480, M_PI_4}, {0.1, 10.0}, {}}, false};
  const DepthRenderCamera depth_camera{color_camera.core(), {0.1, 2.0}};

  const Vector3d Bz_W = Vector3d(FLAGS_dir_x, FLAGS_dir_y, FLAGS_dir_z).normalized();
  const RigidTransformd X_WC = MakeCameraPose(obj_mesh, Bz_W, color_camera);
  std::cerr << X_WC.GetAsMatrix34() << "\n";

  auto camera = builder.AddSystem<RgbdSensor>(scene_graph->world_frame_id(),
                                              X_WC, color_camera, depth_camera);
  builder.Connect(scene_graph->get_query_output_port(),
                  camera->query_object_input_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const auto& camera_context = camera->GetMyContextFromRoot(*context);
  const auto& image_port = camera->color_image_output_port();
  auto image = image_port.Eval<systems::sensors::ImageRgba8U>(camera_context);
  const std::string image_name = MakeImageName();
  std::cout << "Saving rendering to " << image_name << "\n";
  SaveToPng(image, image_name);

  diagram->ForcedPublish(*context);

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
