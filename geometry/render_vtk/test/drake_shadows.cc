#include <fmt/format.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
namespace {

/* Note: This is supposed to recreate what is in shadows.cc. The various vectors
 in that file assume a y-up world. They've been adapted here to comport with
 Drake's z-up world. */

int do_main() {
  const RenderEngineVtkParams params{
      .default_clear_color = {0.752941, 0.752941, 0.752941},
      .lights =
          {
              {.type = "spot",
               .color = Rgba{1.0, 1.0, 0.9843}, /* "HighNoonSun" */
               .position = {0, -0.2, 1.0},
               .frame = "world",
               .intensity = 2.3,
               .direction = Eigen::Vector3d{0, 0.2, -1}.normalized(),
               .cone_angle = 40},
              {.type = "directional",
               .color = Rgba{1, 0.835294, 0.666667}, /* "100W Tungsten" */
               .position = {1.0, -1.0, 1.0},
               .frame = "world",
               .intensity = 2.8,
               .direction = Eigen::Vector3d{-1, 1, -1}.normalized(),
               .cone_angle = 30},
          },
      .environment_map =
          EnvironmentMap{
              .skybox = false,
              .texture =
                  EquirectangularMap{
                      .path = FindResourceOrThrow(
                          "drake/geometry/test/env_256_six_color_room.hdr")}},
      .exposure = 0.25,
      .cast_shadows = true,
  };
  auto engine = MakeRenderEngineVtk(params);

  const int kImageSize = 640;
  const render::ColorRenderCamera camera{
      {"unused",
       {kImageSize, kImageSize, 30.0 / 180.0 * M_PI} /* w, h, fov_y */,
       {0.071025, 5.80216} /* clipping */,
       {}},
      /* show */ true};

  PerceptionProperties sphere_mat;
  sphere_mat.AddProperty("phong", "diffuse", Rgba(1, 1, 1));
  sphere_mat.AddProperty("label", "id", render::RenderLabel(13));
  const double kRadius = 0.5;  // VTK's default radius.
  engine->RegisterVisual(GeometryId::get_new_id(), Sphere(kRadius), sphere_mat,
                         {},
                         false);

  PerceptionProperties ground_mat;
  ground_mat.AddProperty("phong", "diffuse", Rgba(1, 1, 1));
  ground_mat.AddProperty("label", "id", render::RenderLabel(13));
  engine->RegisterVisual(
      GeometryId::get_new_id(), Box(2, 2, 0.1), ground_mat,
      math::RigidTransformd{Eigen::Vector3d{0, 0, -(0.05 + kRadius)}}, false);

  const Eigen::Vector3d p_WT(0, 0, -0.05);
  const Eigen::Vector3d up_W = Eigen::Vector3d::UnitZ();

  const Eigen::Vector3d Cz_W = Eigen::Vector3d(0.19245, 0.96225, -0.19245).normalized();
  const Eigen::Vector3d Cx_W = up_W.cross(Cz_W).normalized();
  const Eigen::Vector3d Cy_W = Cz_W.cross(Cx_W).normalized();

  const Eigen::Vector3d p_WC = p_WT - Cz_W * 2.6055;
  // As documented on CameraInfo, the camera looks in the Cz direction but Cy
  // points *down* in the image. So, why don't I simply flip the image?
  const math::RigidTransformd X_WC(
      math::RotationMatrixd::MakeFromOrthonormalColumns(-Cx_W, -Cy_W, Cz_W),
      p_WC);

  engine->UpdateViewpoint(X_WC);
  systems::sensors::ImageRgba8U image(kImageSize, kImageSize);
  fmt::print("Rendering.\n");
  engine->RenderColorImage(camera, &image);

  common::MaybePauseForUser();

  return 0;
}

}  // namespace
}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake

int main() {
  return drake::geometry::render_vtk::internal::do_main();
}
