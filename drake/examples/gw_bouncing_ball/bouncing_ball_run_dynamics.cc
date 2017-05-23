#include <memory>

#include "drake/common/call_matlab.h"
#include "drake/examples/gw_bouncing_ball/bouncing_ball_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace bouncing_ball {
namespace {

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto bouncing_ball = builder.AddSystem<BouncingBallPlant>();
  bouncing_ball->set_name("BouncingBall");

  // Log the state.
  auto x_logger = builder.AddSystem<systems::SignalLogger<double>>(
      BouncingBallVectorIndices::kNumCoordinates);
  x_logger->set_name("x_logger");
  builder.Connect(bouncing_ball->get_output_port(),
                  x_logger->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* pendulum_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), bouncing_ball);
  bouncing_ball->set_z(pendulum_context, 0.3);
  bouncing_ball->set_zdot(pendulum_context, 0.);

  simulator.Initialize();
  simulator.StepTo(10);

//  const int nsteps = x_logger->sample_times().rows();
//  MatrixX<double> all_data(nsteps, 2);
//  all_data << x_logger->sample_times(), x_logger->data();
//  std::ofstream file("bouncing_ball.dat");
//  file << all_data;
//  file.close();

  // Plot the results (launch lcm_call_matlab_client to see the plots).
  using common::CallMatlab;
  CallMatlab("figure", 1);
  CallMatlab("plot",
             x_logger->sample_times(), x_logger->data().row(0),
             x_logger->sample_times(), x_logger->data().row(1));
  CallMatlab("legend", "z", "zdot");
  CallMatlab("axis", "tight");

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::bouncing_ball::do_main();
}
