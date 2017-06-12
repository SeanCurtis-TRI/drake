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

#define USE_TWO_BALLS

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  SourceId ball_source_id = geometry_system->RegisterSource("ball1");
  auto bouncing_ball = builder.AddSystem<FreeBallPlant>(
      ball_source_id, geometry_system, Vector3<double>(0.25, 0.3, 0.25));
  bouncing_ball->set_name("BouncingBall1");

#ifdef USE_TWO_BALLS
  SourceId ball_source_id2 = geometry_system->RegisterSource("ball2");
  auto bouncing_ball2 = builder.AddSystem<FreeBallPlant>(
      ball_source_id2, geometry_system, Vector3<double>(-0.25, 0.3, -0.25));
  bouncing_ball->set_name("BouncingBall2");
#endif
  SourceId global_source = geometry_system->RegisterSource("anchored");
  Vector3<double> normal_G(0, 0, 1);
  Vector3<double> point_G(0, 0, 0);
  geometry_system->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance<double>>(
          Isometry3<double>::Identity(),
          make_unique<HalfSpace>(normal_G, point_G)));

  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1/60.0);

  builder.Connect(bouncing_ball->get_geometry_id_output_port(),
                  geometry_system->get_source_frame_id_port(ball_source_id));
  builder.Connect(bouncing_ball->get_geometry_pose_output_port(),
                  geometry_system->get_source_pose_port(ball_source_id));

#ifdef USE_TWO_BALLS
  builder.Connect(bouncing_ball2->get_geometry_id_output_port(),
                  geometry_system->get_source_frame_id_port(ball_source_id2));
  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  geometry_system->get_source_pose_port(ball_source_id2));
#endif
  builder.Connect(*geometry_system, *converter);
  builder.Connect(*converter, *publisher);

  // Log the state.
  // TODO(SeanCurtis-TRI): Encode state size in FreeBallPlant.
  const int state_size = 6;
  auto x_logger = builder.AddSystem<systems::SignalLogger<double>>(state_size);
  x_logger->set_name("x_logger");
  builder.Connect(bouncing_ball->get_state_output_port(),
                  x_logger->get_input_port(0));

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto init_ball = [&](FreeBallPlant<double>* system,
                       Vector3<double> pos, Vector3<double> vel) {
    systems::Context<double>* ball_context =
        diagram->GetMutableSubsystemContext(
            simulator.get_mutable_context(), system);
    system->set_pos(ball_context, pos);
    system->set_vel(ball_context, vel);
  };
  init_ball(bouncing_ball, Vector3<double>(0.25, 0.25, 0.5),
            Vector3<double>(0, 0, 0));
#ifdef USE_TWO_BALLS
  init_ball(bouncing_ball2, Vector3<double>(-0.25, -0.25, 0.5),
            Vector3<double>(0, 0, 0.1));
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

  std::stringstream cmd;
  cmd << "time = [" << x_logger->sample_times() << "];";
  CallMatlab("eval", cmd.str());

  cmd.str("");
  cmd << "z = [" << x_logger->data().row(0).transpose() << "];";
  CallMatlab("eval", cmd.str());

  cmd.str("");
  cmd << "zdot = [" << x_logger->data().row(1).transpose() << "];";
  CallMatlab("eval", cmd.str());

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::geometry_world::do_main();
}
