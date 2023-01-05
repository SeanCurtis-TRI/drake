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
DEFINE_int32(width, 640, "Width of output image");
DEFINE_int32(height, 480, "Height of output image");

namespace drake {
namespace examples {
namespace {

namespace fs = std::filesystem;

using Eigen::AngleAxisd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
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
        std::cerr << "No access to the requested output directory: " << dir_path
                  << ";";
      }
    } else {
      std::cerr << "Requested output directory is not a directory: " << dir_path
                << ";";
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

const Matrix2d MakeRotation(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  return (Matrix2d() << c, -s, s, c).finished();
}

Vector3d Find2DFit(const TriangleSurfaceMesh<double>& mesh_W,
                   const RotationMatrixd& R_CW, int axis, double half_fov) {
  // First define frustum normals
  const Matrix2d R = MakeRotation(half_fov + M_PI_2);
  const Vector2d Cy(0, 1);
  // "left" and "right" normals for the frustum.
  const Vector2d n_C[] = {R.transpose() * Cy, R * Cy};

  // Now find extreme vertices: farthest in the n1, n2, and -Cy directions.
  const double kInf = std::numeric_limits<double>::infinity();
  double max_projection[] = {-kInf, -kInf};
  Vector2d max_vertex[] = {Vector2d(), Vector2d()};
  // Also get the vertex deepest into the camera.
  Vector2d most_forward_vertex;
  double most_forward = kInf;
  int v = 0;
  for (const auto& p_WV_W : mesh_W.vertices()) {
    ++v;
    const Vector3d p_WV_C = R_CW * p_WV_W;
    const Vector2d p_WVp_C(p_WV_C(axis), p_WV_C.z());
    const double y = Cy.dot(p_WVp_C);
    if (y < most_forward) {
      most_forward = y;
      most_forward_vertex = p_WVp_C;
    }
    for (int i = 0; i < 2; ++i) {
      double proj = n_C[i].dot(p_WVp_C);
      if (proj > max_projection[i]) {
        max_projection[i] = proj;
        max_vertex[i] = p_WVp_C;
      }
    }
  }

  // Now find the camera body origin p_WCo_C.
  const Vector2d d0_C(n_C[0].y(), -n_C[0].x());
  const double denom = n_C[1].dot(d0_C);
  // The lines should *not* be parallel except for infinite focal lengths.
  DRAKE_DEMAND(std::abs(denom) > 1e-5);
  const double num = n_C[1].dot(max_vertex[1] - max_vertex[0]);
  const double t = num / denom;
  const Vector2d p_WCp_C = max_vertex[0] + t * d0_C;
  Vector3d p_WCo_C(0, 0, p_WCp_C.y());
  p_WCo_C(axis) = p_WCp_C.x();
  return p_WCo_C;
}

RigidTransformd MakeCameraPose(const TriangleSurfaceMesh<double>& mesh_W,
                               const Vector3d& Cz_W_in,
                               const ColorRenderCamera& camera) {
  /*
   The ideal framing will place the camera frustum as shown:

              ○⋅⋅⋅⋅  \
              ⋅░░░░⋅⋅⋅○\   / n̂₁
              ⋅░░░░░░░░⋅ \/
               ⋅░░░░░░░░⋅  \
               ⋅░░░░░░░░⋅  θ \
           ┄┄┄┄⋅░░░░░░░░░⋅┄ d̂←> O
               ⋅░░░░░░░░░⋅   /
                ⋅░░░░░░⋅⋅⋅○/
                ⋅░░░⋅⋅⋅  /\
                ○⋅⋅⋅   /   \ n̂₂
                     /

    We're given a mesh with vertices visualized as `○`. We're likewise given the
    camera direction d̂ and half angle θ. We want to define the camera origin
    relative to mesh such that we have a tight fit to the frustum.

    Here's how we'll do it:

      - To be a tight fit, there has to be at least *one* vertex that lies on
        the upper frustum boundary (and another on the lower). We'll call them
        v₁ and v₂.
      - We can define the normals of the frustum boundaries as n̂₁ and n̂₂.
      - It should be clear that v₁ and v₂ are the vertices most in the n̂₁ and n̂₂
        directions, respectively.
      - Given those points and we can define the two frustum boundary "lines".
      - The origin O is where the lines intersect. It defines the distance
        along d̂ *and* the offset perpendicular to d̂: (d, o)

    For a 3D frustum, where we have two axes, we can compute two sets of (d, o)
    tuples. One in the x direction (dx, ox) and another in the y direction
    (dy, oy). O = [ox, oy, max(dx, dy)]. */

  // 0. Compute camera orientation.
  const Vector3d Cz_W = Cz_W_in.normalized();
  DRAKE_THROW_UNLESS(std::abs(Cz_W.dot(Vector3d::UnitZ())) < 0.99);
  const Vector3d Cx_W = -Vector3d::UnitZ().cross(Cz_W).normalized();
  const Vector3d Cy_W = Cz_W.cross(Cx_W).normalized();
  const RotationMatrixd R_WC =
      RotationMatrixd::MakeFromOrthonormalColumns(Cx_W, Cy_W, Cz_W);
  const RotationMatrixd R_CW = R_WC.inverse();

  const double half_fov_x = camera.core().intrinsics().fov_x() / 2;
  const Vector3d Co_from_x_C = Find2DFit(mesh_W, R_CW, 0, half_fov_x);
  const double half_fov_y = camera.core().intrinsics().fov_y() / 2;
  const Vector3d Co_from_y_C = Find2DFit(mesh_W, R_CW, 1, half_fov_y);

  const Vector3d p_WCo_C(Co_from_x_C.x(), Co_from_y_C.y(),
                         std::min(Co_from_x_C.z(), Co_from_y_C.z()));
  const Vector3d p_WCo_W = R_WC * p_WCo_C;
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
      {render_name, {FLAGS_width, FLAGS_height, M_PI_4}, {0.001, 10.0}, {}},
      false};
  const DepthRenderCamera depth_camera{color_camera.core(), {0.1, 2.0}};

  const Vector3d Bz_W =
      Vector3d(FLAGS_dir_x, FLAGS_dir_y, FLAGS_dir_z).normalized();
  const RigidTransformd X_WC = MakeCameraPose(obj_mesh, Bz_W, color_camera);

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
