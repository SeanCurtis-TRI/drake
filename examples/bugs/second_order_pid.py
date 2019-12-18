from pydrake.symbolic import Variable
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SymbolicVectorSystem

x, xd = Variable('x'), Variable('xdot')
f = Variable('f')
state = [x, xd]
k = 1
s = 1
m = 1
dynamics = [
    xd,
    (-k*x - s*xd + f) / m,
    ]
mass_spring_damper = SymbolicVectorSystem(
    state=state,
    dynamics=dynamics,
    input=[f],
    output=[x, xd]
)

builder = DiagramBuilder()
# It works when dropping in PendulumPlant here
plant = builder.AddSystem(mass_spring_damper)
# It also works when leaving out the controller (exporting plant input)
controller = builder.AddSystem(PidController(kp=[1], ki=[1], kd=[1]))

builder.Connect(plant.get_output_port(0), controller.get_input_port_estimated_state())
builder.Connect(controller.get_output_port_control(), plant.get_input_port(0))

builder.ExportInput(controller.get_input_port_desired_state())

diagram = builder.Build()

simulator = Simulator(diagram)
context = simulator.get_mutable_context()
system_context = diagram.GetMutableSubsystemContext(plant, context)
system_context.get_mutable_continuous_state_vector().SetFromVector([0, 0])
context.FixInputPort(0, [10, 0])

simulator.AdvanceTo(0.01)
