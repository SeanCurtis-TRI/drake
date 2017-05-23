#include <memory>

#include "drake/examples/gw_bouncing_ball/bouncing_ball_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace bouncing_ball {
namespace {

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto bouncing_ball = builder.AddSystem<BouncingBallPlant>();
  bouncing_ball->set_name("BouncingBall");
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* pendulum_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), bouncing_ball);
  bouncing_ball->set_z(pendulum_context, 0.3);
  bouncing_ball->set_zdot(pendulum_context, 0.);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::bouncing_ball::do_main();
}
