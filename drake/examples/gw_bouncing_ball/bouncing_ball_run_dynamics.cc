#include <memory>

#include "drake/common/call_matlab.h"
#include "drake/examples/gw_bouncing_ball/bouncing_ball_plant.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
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
namespace bouncing_ball {
namespace {

using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::InputPortDescriptor;
using systems::rendering::PoseBundleToDrawMessage;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  SourceId ball_source_id = geometry_system->AddSourceInput("ball");

  auto bouncing_ball = builder.AddSystem<BouncingBallPlant>(ball_source_id,
                                                            geometry_system);
  bouncing_ball->set_name("BouncingBall");

  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
  std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1/60.0);


  builder.Connect(bouncing_ball->get_geometry_output_port(),
                  geometry_system->get_port_for_source_id(ball_source_id));
  builder.Connect(*geometry_system, *converter);
  builder.Connect(*converter, *publisher);

  // Log the state.
  auto x_logger = builder.AddSystem<systems::SignalLogger<double>>(
      BouncingBallVectorIndices::kNumCoordinates);
  x_logger->set_name("x_logger");
  builder.Connect(bouncing_ball->get_state_output_port(),
                  x_logger->get_input_port(0));

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* pendulum_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), bouncing_ball);
  bouncing_ball->set_z(pendulum_context, 0.3);
  bouncing_ball->set_zdot(pendulum_context, 0.);

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
  return drake::examples::bouncing_ball::do_main();
}
