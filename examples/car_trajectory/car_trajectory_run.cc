/** @file
 A tutorial example of visualizing a (block) car driving through obstacles.
 */

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram_context.h"

namespace drake {
namespace examples {
namespace car_trajectory {

using lcm::DrakeLcm;
using multibody::collision::PointPair;
using multibody::joints::kFixed;
using std::make_unique;
using systems::Context;
using systems::DiagramContext;
using systems::DiagramBuilder;
using systems::DiagramContext;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

// Add the given geometry as static geometry in the world frame.
void AddStaticGeometryToWorld(
    const DrakeShapes::Geometry& geometry,
    const Isometry3<double>& X_WG,
    const Eigen::Vector4d& vis_color,
    RigidBodyTreed* tree) {
  RigidBody<double>& world = tree->world();

  world.AddVisualElement(DrakeShapes::VisualElement(geometry, X_WG, vis_color));
  tree->addCollisionElement(
      drake::multibody::collision::Element(geometry, X_WG, &world), world,
      "terrain");
  // TODO(SeanCurtis-TRI): Figure out if I need to compile this.
//  tree->compile();
}

// Populates the world with the static meshes.
void PopulateStaticWorld(RigidBodyTreed* tree) {
  // A simple grid of uniform 10x10x2 boxes with a fixed gap.
  const double size = 4;
  const double half_height = 1;
  DrakeShapes::Box box(Eigen::Vector3d(size, size, half_height * 2));
  const int x_count = 10;
  const int y_count = 10;
  const double gap = 10;
  for (int i = 0; i < x_count; ++i) {
    double x = i * (size + gap);
    for (int j = 0; j < y_count; ++j) {
      double y = j * (size + gap);
      Isometry3<double> X_WG = Isometry3<double>::Identity();
      X_WG.translation() << x, y, half_height;
      AddStaticGeometryToWorld(box, X_WG, Eigen::Vector4d{0.8, 0.8, 0.8, 0.5},
                               tree);
    }
  }
}

// Adds a 3-dof car: [x, y, α, ẋ, ẏ, α̇]  (Using α instead of θ because the dot
// displays better....)
void AddCar(RigidBodyTreed* tree) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/car_trajectory/box_car.urdf"),
      kFixed, nullptr /* weld to frame */, tree);
}

// Sets the car configuration in the given context.
void SetCarState(double x, double y, double theta,
                 Context<double>* plant_context) {
  ContinuousState<double>& state =
      plant_context->get_mutable_state().get_mutable_continuous_state();
  DRAKE_DEMAND(state.num_q() == 3);
  VectorBase<double>& qs = state.get_mutable_generalized_position();
  qs[0] = x;
  qs[1] = y;
  qs[2] = theta;
}

int main() {
  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  auto& tree = *tree_ptr;

  PopulateStaticWorld(tree_ptr.get());
  AddCar(tree_ptr.get());
  tree_ptr->compile();

  // Create the plant.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));

  // Create visualization.

  // LCM communication.

  DrakeLcm lcm;

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  builder.Connect(plant.get_output_port(0), publisher->get_input_port(0));

  auto diagram = builder.Build();

  auto full_context = diagram->CreateDefaultContext();
  publisher->PublishLoadRobot();

  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &*full_context);

  // Update the car configuration, the visualization, and perform collision.
  auto test_state = [&diagram, &full_context, &plant, &plant_context, &tree](
      double x, double y, double theta) {
    SetCarState(x, y, theta, &plant_context);
    diagram->Publish(*full_context);
    auto state_vec = plant.GetStateVector(plant_context);
    const int nq = tree.get_num_positions();
    const int nv = tree.get_num_velocities();
    VectorX<double> q = state_vec.topRows(nq);
    VectorX<double> v = state_vec.bottomRows(nv);
    auto kinsol = tree.doKinematics(q, v);
    std::vector<PointPair<double>> contacts =
        tree.ComputeMaximumDepthCollisionPoints(kinsol, false, false);
    bool colliding = contacts.size() > 0;
    std::cout << "Configuration: " << x << ", " << y << ", " << theta;
    if (colliding) {
      std::cout << " - COLLIDES!\n";
    } else {
      std::cout << " - collision free\n";
    }
    return colliding;
  };

  bool colliding = test_state(0, 0, 0);
  colliding = test_state(4, 0, M_PI_2);

  return 0;
}

}  // namespace car_trajectory
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::car_trajectory::main();
}
