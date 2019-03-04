#include <iostream>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/profiler.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/render/gl/render_engine_gl.h"
#include "drake/geometry/dev/render/render_engine_vtk.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {

using common::TimerIndex;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::sensors::ImageDepth32F;

DepthCameraProperties get_camera() {
  const int kWidth = 200;
  const int kHeight = 200;
  return DepthCameraProperties{kWidth,         kHeight, M_PI_4,
                               Fidelity::kLow, 0.5,     9.};
}

Isometry3d X_WC() {
  return Isometry3d(Translation3d(0, 0, 3) *
                    AngleAxisd(M_PI, Vector3d::UnitY()) *
                    AngleAxisd(-M_PI_2, Vector3d::UnitZ()));
}

template <typename Engine>
void Render(Engine* engine, const char* name, TimerIndex timer) {
  engine->AddFlatTerrain();
  engine->UpdateViewpoint(X_WC());

  Isometry3d X_WCylinder =
      RigidTransformd(RollPitchYawd{1., 0.25, 0}, Vector3d{0.5, -0.75, 0.5})
          .GetAsIsometry3();
  RenderIndex cyl_index = engine->RegisterVisual(
      Cylinder(0.5, 0.1), PerceptionProperties(), X_WCylinder);
  engine->UpdateVisualPose(X_WCylinder, cyl_index);

  Isometry3d X_WSphere =
      RigidTransformd(RollPitchYawd{0.5, 0, 0}, Vector3d{0.2, 0, 0.5})
          .GetAsIsometry3();
  RenderIndex sphere_index =
      engine->RegisterVisual(Sphere(0.5), PerceptionProperties(), X_WSphere);
  engine->UpdateVisualPose(X_WSphere, sphere_index);

  Isometry3d X_WBox =
      RigidTransformd(RollPitchYawd{M_PI_4, M_PI_4, 0}, Vector3d{0.2, 0.7, 0.5})
          .GetAsIsometry3();
  RenderIndex box_index =
      engine->RegisterVisual(Box(1, 0.5, 0.2), PerceptionProperties(), X_WBox);
  engine->UpdateVisualPose(X_WBox, box_index);

  Isometry3d X_WMesh = RigidTransformd(RollPitchYawd{M_PI_4, -M_PI_4, 0},
                                       Vector3d{-0.15, -0.5, 0.5})
                           .GetAsIsometry3();
  RenderIndex mesh_index = engine->RegisterVisual(
      Mesh(FindResourceOrThrow("drake/geometry/dev/render/gl/polyhedron.obj"),
           0.5),
      PerceptionProperties(), X_WMesh);
  engine->UpdateVisualPose(X_WMesh, mesh_index);

  DepthCameraProperties camera = get_camera();
  ImageDepth32F depth_image(camera.width, camera.height);
  auto engine_clone = engine->Clone();
  for (int i = 0; i < 100; ++i) {
    startTimer(timer);
    engine_clone->RenderDepthImage(camera, &depth_image);
    lapTimer(timer);
  }
  SaveToTiff(depth_image,
             fmt::format("/home/seancurtis/temp/{}_depth.tiff", name));
}

int DoMain() {
  std::cout << "Exercise!\n";

  TimerIndex vtk_timer = addTimer("VTK");
  RenderEngineVtk vtk_engine;
  Render(&vtk_engine, "vtk", vtk_timer);

  //  RenderEngineDepthGl gl_engine;
  //  Render(&gl_engine, "gl");

  TimerIndex inv_timer = addTimer("inv");
  RenderEngineGl inv_engine;
  Render(&inv_engine, "inv", inv_timer);

  std::cout << TableOfAverages() << "\n";

  return 0;
}

}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::dev::render::gl::DoMain(); }
