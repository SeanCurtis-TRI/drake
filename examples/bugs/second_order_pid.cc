#include <gflags/gflags.h>

#include "drake/common/symbolic.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/primitives/symbolic_vector_system.h"

namespace drake {
namespace examples {
namespace pid {
namespace {

int DoMain() {
  using Eigen::Vector2d;
  using symbolic::Expression;
  using symbolic::Variable;
  using systems::controllers::PidController;
  using systems::Context;
  using systems::DiagramContext;
  using systems::Simulator;

  const double k = 1.0;
  const double s = 1.0;
  const double m = 1.0;

  Variable x{"x"};
  Variable xd{"xd"};
  Vector2<Variable> state{x, xd};
  Variable f{"f"};
  Vector2<Expression> dynamics{xd, (-k * x - s * xd + f) / m};
  Vector2<Expression> output{x, xd};

  systems::DiagramBuilder<double> builder;
  auto& plant = *builder.AddSystem(systems::SymbolicVectorSystemBuilder()
                                       .state(state)
                                       .input(f)
                                       .dynamics(dynamics)
                                       .output(output)
                                       .Build());

  const double kp = 1.0;
  const double ki = 1.0;
  const double kd = 1.0;
  auto& controller = *builder.AddSystem<PidController>(
      VectorX<double>::Constant(1, kp), VectorX<double>::Constant(1, ki),
      VectorX<double>::Constant(1, kd));

  builder.Connect(plant.get_output_port(),
                  controller.get_input_port_estimated_state());
  builder.Connect(controller.get_output_port_control(),
                  plant.get_input_port());

  builder.ExportInput(controller.get_input_port_desired_state());

  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  Context<double>& context = simulator.get_mutable_context();
  Context<double>& system_context =
      diagram->GetMutableSubsystemContext(plant, &context);
  system_context.get_mutable_continuous_state_vector().SetFromVector(
      Vector2d{0.0, 0.0});
  context.FixInputPort(0, Vector2d{10, 0});

  simulator.AdvanceTo(0.01);

  return 0;
}

}  // namespace
}  // namespace pid
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pid::DoMain();
}