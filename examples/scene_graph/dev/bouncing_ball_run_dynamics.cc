#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/profiler.h"
#include "drake/examples/scene_graph/dev/bouncing_ball_plant.h"
#include "drake/geometry/dev/geometry_visualization.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_int32(ball_count, 2, "Number of bouncing balls to include");
DEFINE_int32(depth_count, 2, "The number of depth images to render");
DEFINE_bool(fast_depth, false, "Sets the depth camera to use the fast depth");
DEFINE_double(render_fps, 10, "Frames per simulation second to render");

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {
namespace {

using geometry::GeometryInstance;
template <typename T>
using ProximitySceneGraph = geometry::SceneGraph<T>;
template <typename T>
using RenderSceneGraph = geometry::dev::SceneGraph<T>;
using geometry::dev::ConnectDrakeVisualizer;
using geometry::dev::render::DepthCameraProperties;
using geometry::dev::render::Fidelity;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::IllustrationProperties;
using geometry::ProximityProperties;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::InputPort;
using systems::sensors::dev::RgbdCamera;
using systems::sensors::PixelType;
using std::make_unique;

int do_main() {
  drake::common::TimerIndex main_timer = addTimer("Main");
  startTimer(main_timer);
  systems::DiagramBuilder<double> builder;
  auto proximity_scene_graph = builder.AddSystem<ProximitySceneGraph<double>>();
  proximity_scene_graph->set_name("proximity_scene_graph");

  std::cout << "Adding " << FLAGS_ball_count << " balls\n";
  SourceId ball_source_id = proximity_scene_graph->RegisterSource("balls");
  auto bouncing_balls = builder.AddSystem<BouncingBallPlant>(
      ball_source_id, proximity_scene_graph, FLAGS_ball_count);

  SourceId global_source = proximity_scene_graph->RegisterSource("anchored");
  // Add a "ground" halfspace. Define the pose of the half space (H) in the
  // world from its normal (Hz_W) and a point on the plane (p_WH). In this case,
  // X_WH will be the identity.
  Vector3<double> Hz_W(0, 0, 1);
  Vector3<double> p_WH(0, 0, 0);
  GeometryId ground_id = proximity_scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WH),
                                    make_unique<HalfSpace>(), "ground"));
  proximity_scene_graph->AssignRole(global_source, ground_id,
                                    ProximityProperties());
  proximity_scene_graph->AssignRole(global_source, ground_id,
                                    IllustrationProperties());

  builder.Connect(bouncing_balls->get_geometry_pose_output_port(),
                  proximity_scene_graph->get_source_pose_port(ball_source_id));
  builder.Connect(proximity_scene_graph->get_query_output_port(),
                  bouncing_balls->get_geometry_query_input_port());

    // Add rendering scene graph -- use it for both illustration and perception.
    auto render_scene_graph =
        builder.AddSystem<RenderSceneGraph<double>>(*proximity_scene_graph);
    render_scene_graph->set_name("render_scene_graph");
    builder.Connect(bouncing_balls->get_geometry_pose_output_port(),
                    render_scene_graph->get_source_pose_port(ball_source_id));

    DrakeLcm lcm;
    ConnectDrakeVisualizer(&builder, *render_scene_graph, &lcm);

  if (FLAGS_depth_count > 0) {
    // Publishing images to drake visualizer
    auto image_to_lcm_image_array =
        builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
    image_to_lcm_image_array->set_name("converter");

    const double lcm_period = 1. / FLAGS_render_fps;
    auto image_array_lcm_publisher =
        builder.template AddSystem(systems::lcm::LcmPublisherSystem::Make<
            robotlocomotion::image_array_t>(
            "DRAKE_RGBD_CAMERA_IMAGES", &lcm, lcm_period));
    image_array_lcm_publisher->set_name("publisher");
    builder.Connect(
        image_to_lcm_image_array->image_array_t_msg_output_port(),
        image_array_lcm_publisher->get_input_port());

    std::vector<RgbdCamera*> cameras;
    const Fidelity fidelity =
          FLAGS_fast_depth ? Fidelity::kFastDepth : Fidelity::kLow;
    DepthCameraProperties
          camera_properties(320, 240, M_PI_4, fidelity, 0.1, 2.0);

    // Set them up in a circular array of cameras. At "zero" degrees, the camera
    // is located at (0, -1, 0) and is looking in the (0, 1, 0) direction (with
    // a slight downward angle). To achieve that, the camera has to yaw 90
    // degrees. All other cameras are simply rotations around that.
    const double radius = 1.5;
    for (int i = 0; i < FLAGS_depth_count; ++i) {
      const double theta = M_PI * 2 * i / FLAGS_depth_count;
      const double x = radius * cos(theta - M_PI / 2);
      const double y = radius * sin(theta - M_PI / 2);
      Vector3<double> p_WC(x, y, 0.75);
      Vector3<double> rpy_WC(0, 0.4, M_PI_2 + theta);
      const std::string name = "depth" + std::to_string(i);
      std::cout << "Adding camera: " << name << "\n";
      auto camera = builder.AddSystem<RgbdCamera>(name, p_WC, rpy_WC,
                                                  camera_properties, false);
      const auto& port =
          image_to_lcm_image_array->DeclareImageInputPort<PixelType::kDepth32F>(
              name);
      builder.Connect(camera->depth_image_output_port(), port);
      builder.Connect(render_scene_graph->get_query_output_port(),
                      camera->query_object_input_port());

      cameras.push_back(camera);
    }
  }

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_context().EnableCaching();

  for (int i = 0; i < FLAGS_ball_count; ++i) {
    const double theta = 2 * i * M_PI / FLAGS_ball_count;
    const double z = 0.3 + sin(theta) * 0.05;
    systems::Context<double>& ball_context =
        diagram->GetMutableSubsystemContext(*bouncing_balls,
                                            &simulator.get_mutable_context());
    bouncing_balls->set_z(&ball_context, i, z);
    bouncing_balls->set_zdot(&ball_context, i, 0.);
  }

  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  common::TimerIndex timer = addTimer("Full simulation");
  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  startTimer(timer);
  simulator.StepTo(FLAGS_simulation_time);
  lapTimer(timer);
  lapTimer(main_timer);

  std::cout << TableOfAverages() << "\n";

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::bouncing_ball::do_main();
}
