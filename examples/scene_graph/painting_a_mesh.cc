/** @file
 A simple example that exercises the mesh painter system. In this case, the
 texture the mesh painter controls is simply output to drake visualizer.

 In this case, the canvas object is a single quad and the painter is a cylinder
 that intersects the quad and moves in a circular path (drawing a circle on the
 plane).
 */
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/mesh_painter_system.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"

DEFINE_double(
    sim_rate, 1.0,
    "The desired simulation rate in seconds, defaults to real time (1 s)");
DEFINE_double(
    texture_fps, 30.0,
    "The frequency in Hz at which the texture is painted, defaults to 30 Hz");
DEFINE_double(
    duration, 2 * M_PI,
    "The duration to run the simulation in seconds; defaults to 6.28 s");
DEFINE_string(
    painter_shape, "cylinder",
    "The shape to use as the painter object; should be 'cylinder' or 'box'");

namespace drake {
namespace examples {
namespace scene_graph {
namespace mesh_painter {

using Eigen::Vector4d;
using geometry::Box;
using geometry::ConnectDrakeVisualizer;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::MeshPainterSystem;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::SourceId;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;
using systems::DiagramBuilder;
using systems::LeafSystem;
using systems::sensors::PixelType;
using systems::Simulator;

/** Moves a cylinder in a circular path around the world origin (intersecting
 the z = 0 plane in the world frame with its long axis parallel with Wz).

 @system{MovingRod,, @output_port{geometry_pose} }

 This system's output is strictly a function of time and has no state.
 */
class MovingRod final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingRod)

  explicit MovingRod(SceneGraph<double>* scene_graph) {
    // Add geometry for a ball that moves based on sinusoidal derivatives.
    source_id_ = scene_graph->RegisterSource("moving_rod");
    frame_id_ =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("moving_frame"));
    unique_ptr<Shape> shape;
    if (FLAGS_painter_shape == "cylinder") {
      shape = make_unique<Cylinder>(kRodRadius, kRodLength);
    } else if (FLAGS_painter_shape == "box") {
      shape = make_unique<Box>(kRodRadius * 2, kRodRadius * 2, kRodLength);
    } else {
      throw std::runtime_error("The '--painter_shape' argument must be either "
                               "'box' or 'cylinder' (case matters)");
    }
    geometry_id_ = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(RigidTransformd(), move(shape), "rod"));

    // Note: No proximity role is required because the mesh painter system
    // will handle all of that itself.

    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.8, 0.1, 0.1, 1));
    scene_graph->AssignRole(source_id_, geometry_id_, illus_props);

    geometry_pose_port_ =
        this->DeclareAbstractOutputPort("geometry_pose",
                                        &MovingRod::CalcFramePoseOutput)
            .get_index();
  }

  SourceId source_id() const { return source_id_; }

  const systems::OutputPort<double>& geometry_pose_output_port() const {
    return systems::System<double>::get_output_port(geometry_pose_port_);
  }

  GeometryId geometry_id() const { return geometry_id_; }

 private:
  void CalcFramePoseOutput(
      const Context<double>& context, FramePoseVector<double>* poses) const {
    RigidTransformd pose;
    const double p_WF_x = std::cos(context.get_time());
    const double p_WF_y = std::sin(context.get_time());
    pose.set_translation({p_WF_x, p_WF_y, p_WF_z});
    *poses = {{frame_id_, pose}};
  }

  SourceId source_id_;
  FrameId frame_id_;
  GeometryId geometry_id_;
  static constexpr double kRodRadius{0.1};
  static constexpr double kRodLength{1.0};
  // The z-component of the painter frame in the world frame.
  static constexpr double p_WF_z{kRodLength * 0.4};

  int geometry_pose_port_{-1};
};

int do_main() {
  DiagramBuilder<double> builder;

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();

  // Add the moving rod -- the painter object.
  auto& moving_rod = *builder.AddSystem<MovingRod>(&scene_graph);
  builder.Connect(moving_rod.geometry_pose_output_port(),
                  scene_graph.get_source_pose_port(moving_rod.source_id()));

  // Add the canvas object -- a single quad.

  auto file_name =
      FindResourceOrThrow("drake/examples/scene_graph/textured_quad.obj");
  const double scale = 3;
  SourceId source_id = scene_graph.RegisterSource("world");
  GeometryId ground_id = scene_graph.RegisterAnchoredGeometry(
      source_id,
      make_unique<GeometryInstance>(
          RigidTransformd{}, make_unique<Mesh>(file_name, scale), "canvas"));
  // Note: the visualizer will not see the updated texture.
  IllustrationProperties illustration_box;
  illustration_box.AddProperty("phong", "diffuse",
                               Vector4d{0.5, 0.5, 0.45, 1.0});
  scene_graph.AssignRole(source_id, ground_id, illustration_box);

  // Now visualize.
  DrakeLcm lcm;

  // Visualize geometry.
  ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  // Create and visualize painter system.
  auto& painter_system = *builder.AddSystem<MeshPainterSystem>(
      ground_id, moving_rod.geometry_id(), scene_graph, 512, 512);
  builder.Connect(scene_graph.get_query_output_port(),
                  painter_system.geometry_query_input_port());

  // TODO(SeanCurtis-TRI): Visualize the image.
  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
  const auto& texture_port =
      image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
          "texture");

  image_to_lcm_image_array->set_name("converter");
  auto image_array_lcm_publisher =
      builder.template AddSystem(systems::lcm::LcmPublisherSystem::Make<
          robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", &lcm,
          1. / FLAGS_texture_fps /* publish period */));
  image_array_lcm_publisher->set_name("publisher");

  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port());

  builder.Connect(painter_system.texture_output_port(), texture_port);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(FLAGS_sim_rate);
  simulator.Initialize();
  simulator.AdvanceTo(2 * M_PI);  // One period of the circular motion.

  return 0;
}

}  // namespace mesh_painter
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::mesh_painter::do_main();
}
