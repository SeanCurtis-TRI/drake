#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_writer.h"

/* Camera posing. The camera's pose in the world frame is uniquely defined by
 its origin's position, a vector towards a *target* point and an up vector
 (all quantities expressed in the world frame).
 The default values for camera position and target are estimated from a
 reference image provided from fleet learning.  */
DEFINE_double(cam_x, -0.25, "X-position (m) of camera origin");
DEFINE_double(cam_y, -1.65, "Y-position (m) of camera origin");
DEFINE_double(cam_z, 1.5, "Z-position (m) of camera origin");

DEFINE_double(tgt_x, -0.25, "X-position (m) of camera's target point");
DEFINE_double(tgt_y, 0.5, "Y-position (m) of camera's target point");
DEFINE_double(tgt_z, 0.25, "Z-position (m) of camera's target point");

DEFINE_double(up_x, 0, "X-position of camera's up vector");
DEFINE_double(up_y, 0, "Y-position of camera's up vector");
DEFINE_double(up_z, 1, "Z-position of camera's up vector");

/* Camera intrinsics
 The values for width, height, and fov_y are drawn from fmk0000_sim_launch.json
 from mobile_manipulation code.  */
DEFINE_double(fov_y, 80.0 * M_PI / 180, "Vertical field of view (in radians)");
DEFINE_double(far, 100, "The distance (in meters) to the far clipping plane");
DEFINE_bool(show_window, false, "If true, render window will be shown");

/* Render engine selection. */
DEFINE_int32(render_samples, 10, "Number of renderings to make per image size");

/* File paths. */
DEFINE_string(fl_root, "/home/seancurtis/code/mobile_manipulation",
              "The path to the mobile_manipulation repo");
DEFINE_string(img_dir, "", "If given, a directory to save images to");

namespace drake {
namespace examples {
namespace fl {
namespace {

using Eigen::Vector3d;
using geometry::render::MakeRenderEngineGl;
using geometry::render::MakeRenderEngineVtk;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::pair;
using std::unique_ptr;
using std::vector;
using systems::sensors::ImageRgba8U;

/* Creates a camera pose for a camera whose origin is at point Co, it is aimed
 to look at point T (in the center of the image), with the camera biased towards
 the given `up` direction. All quantities are measured and expressed in the
 frame W (generally assumed to be the world frame). */
RigidTransformd CreateCameraPose(const Vector3d& p_WCo, const Vector3d& p_WT,
                                 const Vector3d& up_W) {
  // We need to position and orient the camera. We have the camera body frame
  // B (see rgbd_sensor.h) and the camera frame C (see camera_info.h).
  // By default X_BC = I in the RgbdSensor. So, to aim the camera, Cz = Bz
  // should point from the camera position to the origin. Cy points *down* the
  // image, so we need to align it in the -up direction. So,  we compute the
  // basis using camera Y-ish in the Cy ≈ -up direction to compute Cx, and
  // then use Cx an and Cz to compute Cy.
  const Vector3d p_CoT_W = p_WT - p_WCo;
  const double dist = p_CoT_W.norm();
  if (dist < 1e-2) {
    throw std::runtime_error(
        "The target point must be at least 1 cm away from the camera origin!");
  }
  const Vector3d Cz_W = p_CoT_W / dist;
  const Vector3d Cx_W = -up_W.cross(Cz_W).normalized();
  const Vector3d Cy_W = Cz_W.cross(Cx_W).normalized();
  const RotationMatrixd R_WC =
      RotationMatrixd::MakeFromOrthonormalColumns(Cx_W, Cy_W, Cz_W);
  return {R_WC, p_WCo};
}

enum class RenderType { VTK, GL };

/* Makes a RenderEngine implementaiton based on the requested render type. */
unique_ptr<geometry::render::RenderEngine> MakeRenderer(RenderType r_type) {
  switch (r_type) {
    case RenderType::VTK: {
      geometry::render::RenderEngineVtkParams params;
      params.default_clear_color = Vector3d{0, 0, 0};
      return MakeRenderEngineVtk(params);
    }
    case RenderType::GL: {
      geometry::render::RenderEngineGlParams params;
      params.default_clear_color = geometry::Rgba{0, 0, 0, 1};
      return MakeRenderEngineGl(params);
    }
  }
}

std::string AsString(RenderType r_type) {
  switch (r_type) {
    case RenderType::VTK:
      return "vtk";
    case RenderType::GL:
      return "gl";
  }
}

int do_main() {
  for (const auto render_type : {RenderType::VTK, RenderType::GL}) {
    systems::DiagramBuilder<double> builder;
    auto& plant = *builder.AddSystem<multibody::MultibodyPlant<double>>(1.0);
    auto& scene_graph = *builder.AddSystem<geometry::SceneGraph<double>>();
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    const std::string renderer_name{"renderer"};
    scene_graph.AddRenderer(renderer_name, MakeRenderer(render_type));
    multibody::Parser parser(&plant, &scene_graph);
    double load_time{};
    {
      std::cout << "Loading assets\n";
      auto start = std::chrono::high_resolution_clock::now();
      parser.AddModelFromFile(FLAGS_fl_root + "/config/urdfs/mock_kitchen.sdf");
      /* TODO(SeanCurtis-TRI) The kitchen scenario seems to include kitchen
       walls; I'm omitting them because, for the default camera location, they
       occlude the view of the kitchen. However, the reference image seems to
       include *some* backdrop outside the window. Find out what it is. It
       doesn't appear to be:
       /home/seancurtis/code/mobile_manipulation/config/urdfs/mock_kitchen_walls.sdf
       We need to see if that contributes to performance.  */
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      load_time = elapsed.count();
    }
    plant.Finalize();
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(*plant.get_source_id()));
    geometry::DrakeVisualizerd::AddToBuilder(
        &builder, scene_graph, nullptr,
        {1 / 60.0, geometry::Role::kIllustration, {0.9, 0.8, 0.8, 1.0}});

    /* Add a visual indicator of where I think the camera should go and how it
     is oriented.  */
    const geometry::SourceId s_id = scene_graph.RegisterSource("main");
    const Vector3d p_WC{FLAGS_cam_x, FLAGS_cam_y, FLAGS_cam_z};
    const Vector3d p_WT{FLAGS_tgt_x, FLAGS_tgt_y, FLAGS_tgt_z};
    const Vector3d up_W{FLAGS_up_x, FLAGS_up_y, FLAGS_up_z};
    const RigidTransformd X_WC = CreateCameraPose(p_WC, p_WT, up_W);
    // Create a rectangular box as a display of camera location and orientation.
    auto cam = std::make_unique<geometry::GeometryInstance>(
        X_WC, std::make_unique<geometry::Box>(0.25, 0.125, 0.0625), "camera");
    cam->set_illustration_properties(geometry::IllustrationProperties());
    scene_graph.RegisterAnchoredGeometry(s_id, std::move(cam));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();
    auto& sg_context = scene_graph.GetMyMutableContextFromRoot(context.get());
    const auto& query_object =
        scene_graph.get_query_output_port().Eval<geometry::QueryObject<double>>(
            sg_context);
    const auto& inspector = query_object.inspector();

    /* Defer writing any report out until after the diagram has been finished.
     We want to keep the report separate from any warnings written to the
     console. */
    std::cout << "\n\nResults for " << AsString(render_type) << " renderer\n";
    if (render_type == RenderType::GL && FLAGS_show_window) {
      std::cout << "  The live window will display the image upside down; this "
                   "is *not* a problem with the rendering!\n";
    }
    std::cout << "\nLoad time: " << load_time << " ms\n";
    std::cout << "  Total geometries: " << inspector.num_geometries() << "\n";
    std::cout << "    Proximity geometries:    "
              << inspector.NumGeometriesWithRole(geometry::Role::kProximity)
              << "\n";
    std::cout << "    Perception geometries:   "
              << inspector.NumGeometriesWithRole(geometry::Role::kPerception)
              << "\n";
    std::cout << "    Illustration geometries: "
              << inspector.NumGeometriesWithRole(geometry::Role::kIllustration)
              << "\n";
    diagram->Publish(*context);

    std::cout << "\nRendering " << FLAGS_render_samples << " times per size\n";
    const std::string prefix_fmt = "{}, {}";
    std::cout << fmt::format(prefix_fmt, "width", "height")
              << ", Samples (ms) -----> \n";
    for (const auto& size : vector<pair<int, int>>{
             {10, 8}, {160, 128}, {640, 512}, {1280, 1024}, {2560, 2048}}) {
      /* Render the scene. */
      const int w = size.first;
      const int h = size.second;
      ImageRgba8U image(w, h);
      geometry::render::ColorRenderCamera color_camera{
          {renderer_name,
           /* These explicit intrinsics are taken from fmk000_sim_launch.json.
            {2560, 2048, 1220.3556, 1220.3556, 1280, 1024}
            they should be equivalent to the intrinsics created by the
            (w, h, fov_y) generated for (w, h) = (2560, 2048) and the default
            fov_y value.  */
           {w, h, FLAGS_fov_y},
           {0.1, FLAGS_far},
           RigidTransformd{}},
          FLAGS_show_window};
      std::cout << fmt::format(prefix_fmt, w, h);
      {
        for (int i = 0; i < FLAGS_render_samples; ++i) {
          // Jiggle the camera along the x-axis.
          const RigidTransformd X_RW(Vector3d{0.01 - ((i % 2) * 0.02), 0, 0});
          auto start = std::chrono::high_resolution_clock::now();
          query_object.RenderColorImage(
              color_camera, scene_graph.world_frame_id(), X_RW * X_WC, &image);
          auto end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double, std::milli> elapsed = end - start;
          std::cout << ", " << elapsed.count();
        }
        std::cout << "\n";
      }
      if (!FLAGS_img_dir.empty()) {
        systems::sensors::SaveToPng(
            image, fmt::format("{}/fl_color_image_{}_{}_{}.png", FLAGS_img_dir,
                               AsString(render_type), w, h));
      }
    }
  }
  if (FLAGS_show_window) {
    std::cout << "Hit enter to continue: ";
    std::cin.get();
  }
  return 0;
}

}  // namespace
}  // namespace fl
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::fl::do_main();
}
