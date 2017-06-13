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
using systems::rendering::PoseBundleToDrawMessage;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
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
  int kCount = 8;
  for (int i = 0; i < kCount; ++i) {
    std::string sys_name = "ball" + std::to_string(i);
    SourceId ball_source_id = geometry_system->RegisterSource(sys_name);
    auto bouncing_ball = builder.AddSystem<FreeBallPlant>(
        ball_source_id, geometry_system, Vector3<double>(0.25, 0.3, 0.25));
    ball_systems.push_back(bouncing_ball);
    bouncing_ball->set_name(sys_name);
    builder.Connect(bouncing_ball->get_geometry_id_output_port(),
                    geometry_system->get_source_frame_id_port(ball_source_id));
    builder.Connect(bouncing_ball->get_geometry_pose_output_port(),
                    geometry_system->get_source_pose_port(ball_source_id));
  }

  builder.Connect(*geometry_system, *converter);
  builder.Connect(*converter, *publisher);

  // Log the state.
  // TODO(SeanCurtis-TRI): Encode state size in FreeBallPlant.
//  const int state_size = 6;
//  auto x_logger = builder.AddSystem<systems::SignalLogger<double>>(state_size);
//  x_logger->set_name("x_logger");
//  builder.Connect(bouncing_ball->get_state_output_port(),
//                  x_logger->get_input_port(0));

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
    Vector3<double> pos(0, 0, 0.3 + (i * 0.3));
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
  auto rotation = AngleAxis<double>(kRotation, Vector3<double>::UnitZ()).matrix();

  for (int i = 0; i < kCount; ++i) {
    auto bouncing_ball = ball_systems[i];
    pos_0 = rotation * pos_0;
    init_ball(bouncing_ball, pos_0, Vector3<double>(0, 0, 0));
  }
#endif


  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.StepTo(13);

//  const int nsteps = x_logger->sample_times().rows();
//  MatrixX<double> all_data(nsteps, 2);
//  all_data << x_logger->sample_times(), x_logger->data();
//  std::ofstream file("bouncing_ball.dat");
//  file << all_data;
//  file.close();

  using common::CallMatlab;
#if 0
  // Plot the results (launch lcm_call_matlab_client to see the plots).
  CallMatlab("figure", 1);
  CallMatlab("plot",
             x_logger->sample_times(), x_logger->data().row(0),
             x_logger->sample_times(), x_logger->data().row(1));
  CallMatlab("legend", "z", "zdot");
  CallMatlab("axis", "tight");
#endif

//  std::stringstream cmd;
//  cmd << "time = [" << x_logger->sample_times() << "];";
//  CallMatlab("eval", cmd.str());
//
//  cmd.str("");
//  cmd << "z = [" << x_logger->data().row(0).transpose() << "];";
//  CallMatlab("eval", cmd.str());
//
//  cmd.str("");
//  cmd << "zdot = [" << x_logger->data().row(1).transpose() << "];";
//  CallMatlab("eval", cmd.str());

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::geometry_world::do_main();
}
