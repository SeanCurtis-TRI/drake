#include <memory>

#include <utility>

#include "drake/common/call_matlab.h"
#include "drake/examples/geometry_world/free_ball_plant.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/shapes.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace geometry_world {
namespace {

using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::HalfSpace;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::InputPortDescriptor;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta3Integrator;
using std::make_unique;
using std::vector;

void AddAnchored(GeometrySystem<double>* geometry_system, bool force_plane) {
  // Funnel-like half-spaces
  SourceId global_source = geometry_system->RegisterSource("anchored");
  if (force_plane) {
    Vector3<double> normal_G(0.0, 0, 1);
    Vector3<double> point_G(0, 0, 0);
    geometry_system->RegisterAnchoredGeometry(
        global_source,
        make_unique<GeometryInstance<double>>(
            Isometry3<double>::Identity(),
            make_unique<HalfSpace>(normal_G.normalized(), point_G)));
  } else {
    Vector3<double> normal_G(0.25, 0, 1);
    Vector3<double> point_G(0, 0, 0);
    geometry_system->RegisterAnchoredGeometry(
        global_source,
        make_unique<GeometryInstance<double>>(
            Isometry3<double>::Identity(),
            make_unique<HalfSpace>(normal_G.normalized(), point_G)));
    auto plane_rotation =
        AngleAxis<double>(2 * 3.141597 / 3, Vector3<double>::UnitZ()).matrix();
    normal_G = plane_rotation * normal_G;
    geometry_system->RegisterAnchoredGeometry(
        global_source,
        make_unique<GeometryInstance<double>>(
            Isometry3<double>::Identity(),
            make_unique<HalfSpace>(normal_G.normalized(), point_G)));
    normal_G = plane_rotation * normal_G;
    geometry_system->RegisterAnchoredGeometry(
        global_source,
        make_unique<GeometryInstance<double>>(
            Isometry3<double>::Identity(),
            make_unique<HalfSpace>(normal_G.normalized(), point_G)));
  }
}

// Maps from a hue value in the range [0, 360) to an RGB value (with 100%
// alpha).
Eigen::Vector4d GetColorFromHue(double hue) {
  Eigen::Vector4d color(1, 1, 1, 1);
  const double s = 1.0;
  const double v = 1.0;
  // Taken from:
  //  https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
  // We assume S = V = 1 (i.e., full color saturation and brightness).
  double hh = hue / 60.0;
  int64_t i = static_cast<int64_t>(hh);
  double ff = hh - i;
  double p = v * (1.0 - s);
  double q = v * (1.0 - (s * ff));
  double t = v * (1.0 - (s * (1.0 - ff)));
  switch (i) {
    case 0:
      color.head<3>() << v, t, p;
      break;
    case 1:
      color.head<3>() << q, v, p;
      break;
    case 2:
      color.head<3>() << p, v, t;
      break;
    case 3:
      color.head<3>() << p, q, v;
      break;
    case 4:
      color.head<3>() << t,  p, v;
      break;
    case 5:
    default:
      color.head<3>() << v, p, q;
      break;
  }

  return color;
}

int do_main() {
  systems::DiagramBuilder<double> builder;


  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  AddAnchored(geometry_system, false /* force plane */);

  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1/60.0);

  vector<FreeBallPlant<double>*> ball_systems;
  int kCount = 10;
  const float kHueOffset = 360.0 / kCount;
  for (int i = 0; i < kCount; ++i) {
    double hue = kHueOffset * i;
    std::string sys_name = "ball" + std::to_string(i);
    SourceId ball_source_id = geometry_system->RegisterSource(sys_name);
    auto bouncing_ball = builder.AddSystem<FreeBallPlant>(
        ball_source_id, geometry_system,
        GetColorFromHue(hue), Vector3<double>(0.25, 0.3, 0.25));
    ball_systems.push_back(bouncing_ball);
    bouncing_ball->set_name(sys_name);
    builder.Connect(bouncing_ball->get_geometry_id_output_port(),
                    geometry_system->get_source_frame_id_port(ball_source_id));
    builder.Connect(bouncing_ball->get_geometry_pose_output_port(),
                    geometry_system->get_source_pose_port(ball_source_id));
  }

  builder.Connect(*geometry_system, *converter);
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // Set initial state
  auto init_ball = [&](FreeBallPlant<double>* system,
                       Vector3<double> pos, Vector3<double> vel) {
    systems::Context<double>* ball_context =
        diagram->GetMutableSubsystemContext(
            simulator.get_mutable_context(), system);
    system->set_pos(ball_context, pos);
    system->set_vel(ball_context, vel);
  };

#if 0
  // Simply place them in a vertical line
  for (int i = 0; i < kCount; ++i) {
    auto bouncing_ball = ball_systems[i];
    Vector3<double> pos(0, 0, 0.3 + (i * 0.15));
    init_ball(bouncing_ball, pos, Vector3<double>(0, 0, 0));
  }
#else
  // Initial state of bouncing balls. Position them in a circle. This assumes
  // ball diameter of 1.0
  // Position them in a circle with one ball's space between them. This implies
  // a circumference of 2 * kCount * diameter.
  const double kBallDiameter = 0.1;
  const double circle_radius = (2 * kCount * kBallDiameter) / (2 * 3.141597);
  Vector3<double> pos_0(circle_radius, 0, 0.5);
  const double kRotation = 2 * 3.141597 / kCount;
  auto rotation =
      AngleAxis<double>(kRotation, Vector3<double>::UnitZ()).matrix();

  for (int i = 0; i < kCount; ++i) {
    auto bouncing_ball = ball_systems[i];
    pos_0 = rotation * pos_0;
    init_ball(bouncing_ball, pos_0, Vector3<double>(0, 0, 0));
  }
#endif

//  auto context = simulator.get_mutable_context();
//  simulator.reset_integrator<RungeKutta3Integrator<double>>(*diagram, context);
//  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-3);
//  simulator.get_mutable_integrator()->set_target_accuracy(1e-5);
  simulator.get_mutable_integrator()->set_maximum_step_size(0.00005);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.StepTo(10);

  return 0;
}

}  // namespace
}  // namespace geometry_world
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::geometry_world::do_main();
}
