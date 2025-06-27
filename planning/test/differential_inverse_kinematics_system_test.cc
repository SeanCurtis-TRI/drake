#include "operational_space_control/differential_inverse_kinematics_system.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/planning/dof_mask.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/framework/bus_value.h"

namespace anzu {
namespace operational_space_control {
namespace {

using drake::CompareMatrices;
using drake::Value;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialVelocity;
using drake::planning::CollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::planning::DofMask;
using drake::planning::JointLimits;
using drake::planning::RobotClearance;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::planning::SceneGraphCollisionChecker;
using drake::solvers::Binding;
using drake::solvers::EvaluatorBase;
using drake::systems::BusValue;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using DiffIk = DifferentialInverseKinematicsSystem;
using CallbackDetails = DiffIk::CallbackDetails;
using Ingredient = DiffIk::Ingredient;
using Recipe = DiffIk::Recipe;

// TODO(sean.curtis): rebase this when 16952 merges.

/* Assorted construction parameters. */
static constexpr std::string_view kRobotName{"robot"};
static constexpr double kTimeStep{0.25};
static constexpr double kK_VX{2};
static constexpr double kQpWeight{2.5};

/* Parameters for a single test evaluation of Diff Ik. */
struct Sample {
  /* The desired translational velocity of the ball. */
  Vector3d vd_WB;
  /* The expected translational velocity of the ball. */
  Vector3d v_WB_expected;
  /* Axis masking on cartesian velocities means that Diff IK may produce
  unexpected velocities for the masked elements. This mask communicates which
  translational velocity elements matter to the test (1) or should be ignored
  (0). */
  Vector3d v_WB_mask;
  /* Description to use for feedback; this will be printed upon failure. */
  std::string description;
  /* The values of q₀ and q₁ for the query. By default, it has initial values
  for the non-redundant robot. */
  VectorXd q_active = (VectorXd(2) << 0, 0).finished();
};

/* Testing strategy.

We're _not_ explicitly testing:

  - DiffIk::Ingredient
    - It is an abstract interface and has nothing to test.
  - DiffIk::Recipe
    - The only public method (AddToProgram()) is implicitly tested over and
      over in each invocation of the constructor.

The system has two possible inputs which control the commanded velocity: desired
position and velocity. We'll have a single test vis a vis the position input --
confirming how it infers a desired velocity -- but mostly test against the
velocity input.

The reason for this is that the position input is merely used to construct a
desired velocity. In the remaining tests, by explicitly specifying the desired
velocity, we can limit our reasoning about velocity space and not worry about
the mechanism for inferring velocity from position. */
class DifferentialInverseKinematicsTest : public ::testing::Test {
 protected:
  /* Constructs the differential inverse kinematic system with the given recipe.

  @param recipe          The optimization recipe.
  @param has_redundancy  If `true`, the robot will have 3-dofs (with one
                         redundant -- see MakeRobotDiagram()). */
  static DiffIk MakeDiffIk(std::unique_ptr<Recipe> recipe,
                           bool has_redundancy = false) {
    std::shared_ptr<CollisionChecker> collision_checker =
        MakeChecker(has_redundancy);
    const DofMask robot_dofs = DofMask::MakeFromModel(
        collision_checker->plant(), std::string(kRobotName));
    return DiffIk(std::move(recipe), "world", collision_checker, robot_dofs,
                  kTimeStep, kK_VX, MakeSpatialLimits());
  }

  /* Convenience function for adding the least-squares cost. */
  static void AddLeastSquaresCost(
      Recipe* recipe,
      const drake::string_unordered_map<drake::Vector6d>& masks = {}) {
    auto ingredient = std::make_shared<DiffIk::LeastSquaresCost>();
    ingredient->cartesian_axis_masks = masks;
    recipe->AddIngredient(std::move(ingredient));
  }

  /* Create the collision checker for the DiffIK dut. */
  static std::shared_ptr<CollisionChecker> MakeChecker(bool has_redundancy) {
    const int dof_count = has_redundancy ? 3 : 2;
    std::shared_ptr<RobotDiagram<double>> robot = MakeRobotDiagram(dof_count);
    const ModelInstanceIndex robot_index =
        robot->plant().GetModelInstanceByName(kRobotName);
    CollisionCheckerParams params{
        .model = std::move(robot),
        .robot_model_instances = {robot_index},
        .edge_step_size = 0.1,  // Arbitrary irrelevant value.
        .env_collision_padding = 0,
    };
    return std::make_shared<SceneGraphCollisionChecker>(std::move(params));
  }

  /* Build a model with a *very* simple robot and an obstacle.

    - The robot is simply a zero-radius sphere (a point) whose origin moves on
      the Wz = 0 plane.
    - The obstacle is simply a unit sphere at the origin. It _does_ have a
      degree of freedom so that we can distinguish between the robot qs and
      environment qs, but we will keep the obstacle located at the origin.

  The robot can be configured to have two or three dofs.
    - Two dofs: q0 controls movement on the Wx axis and q1 the Wy axis.
    - Three dofs: we add a third _redundant_ dof: q2 is a linear combination
      (A = (Wx + Wy)/√2).

  Some notes on the intention behind this design:

    1. The mapping from generalized position and velocity to cartesian position
       and velocity is simple.
       - For the 2-dof robot: the position of the sphere p_WB is simply
         (q0, q1, 0) and its velocity v_WB is simply (q̇₀, q̇₁, 0).
       - The 3-dof version adds a redundant term: p_WB = q₀·W_x + q₁·W_y + q₂·A.
    2. The Diff IK system does not make any special allowances for the
       interpretation of the robot dofs. So, we can use this simple mapping
       between generalized state and cartesian kinematics without fear of
       under testing the system.
    3. The 3-dof robot, with its built in redundancy, serves as the basis for
       evaluating the "joint centering" constraint. It will only be introduced
       for *that* test -- as it is overly cumbersome in all other tests. */
  static std::shared_ptr<RobotDiagram<double>> MakeRobotDiagram(int dof_count) {
    DRAKE_DEMAND(dof_count == 2 || dof_count == 3);

    RobotDiagramBuilder<double> builder;
    MultibodyPlant<double>& plant = builder.plant();

    // Add the obstacle.
    const RigidBody<double>& obstacle =
        plant.AddRigidBody("obstacle", SpatialInertia<double>::MakeUnitary());
    plant.RegisterCollisionGeometry(obstacle, {}, Sphere(1.0), "obstacle",
                                    CoulombFriction<double>());
    plant.AddJoint<PrismaticJoint>("Wz", plant.world_body(), std::nullopt,
                                   obstacle, std::nullopt, Vector3d::UnitZ());

    // Add the articulated robot.
    const ModelInstanceIndex robot_index =
        plant.AddModelInstance(std::string(kRobotName));
    const RigidBody<double>& body0 = plant.AddRigidBody(
        "body0", robot_index, SpatialInertia<double>::Zero());
    const RigidBody<double>& ball = plant.AddRigidBody(
        "ball", robot_index, SpatialInertia<double>::MakeUnitary());
    plant.RegisterCollisionGeometry(ball, {}, Sphere(0.0), "ball",
                                    CoulombFriction<double>());
    plant.AddJoint<PrismaticJoint>("Wx", plant.world_body(), std::nullopt,
                                   body0, std::nullopt, Vector3d::UnitX());
    if (dof_count == 3) {
      const RigidBody<double>& body1 = plant.AddRigidBody(
          "body1", robot_index, SpatialInertia<double>::Zero());
      plant.AddJoint<PrismaticJoint>("Wy", body0, std::nullopt, body1,
                                     std::nullopt, Vector3d::UnitY());
      plant.AddJoint<PrismaticJoint>(
          "A", body1, std::nullopt, ball, std::nullopt,
          (Vector3d::UnitX() + Vector3d::UnitY()).normalized());
    } else {
      plant.AddJoint<PrismaticJoint>("Wy", body0, std::nullopt, ball,
                                     std::nullopt, Vector3d::UnitY());
    }

    plant.Finalize();

    return builder.Build();
  }

  /* Given a plant containing the "ball" rigid body and commanded velocities for
  the active dofs, computes the spatial velocity of "ball". */
  static SpatialVelocity<double> GetBallSpatialVelocity(
      const MultibodyPlant<double>& plant, VectorXd v_active) {
    auto context = plant.CreateDefaultContext();
    Eigen::VectorXd v_full(plant.num_velocities());
    auto active_dof = DofMask::MakeFromModel(plant, std::string(kRobotName));
    active_dof.SetInArray(v_active, &v_full);
    plant.SetVelocities(context.get(), v_full);
    const RigidBody<double>& body = plant.GetRigidBodyByName("ball");
    return plant.EvalBodySpatialVelocityInWorld(*context, body);
  }

  /* When the input is a position vector, this spatial velocity (all
  non-negative values), defines the bounds for the inferred desired velocity. */
  static SpatialVelocity<double> MakeSpatialLimits() {
    return SpatialVelocity<double>(Vector3d(2, 3, 4), Vector3d(5, 6, 7));
  }

  /* Evaluates the given `dut` against the set of samples; returns the commanded
  velocities. */
  static std::vector<VectorXd> EvaluateSamples(
      const DiffIk& dut, const std::vector<Sample>& samples) {
    std::vector<VectorXd> results;
    auto context = dut.CreateDefaultContext();

    VectorXd q(dut.plant().num_positions());
    q.setZero();

    BusValue desired_velocities;
    for (const auto& sample : samples) {
      SCOPED_TRACE(sample.description);
      // The reference pose will *always* be the zero pose. Only relevant for
      // joint centering.
      q.setZero();
      dut.get_input_port_nominal_posture().FixValue(context.get(), q);
      // The current pose is as specified by the
      dut.active_dof().SetInArray(sample.q_active, &q);
      dut.get_input_port_position().FixValue(context.get(), q);
      // The robot can't exhibit angular velocity at all or translational
      // velocity in the Wz direction.
      const SpatialVelocity<double> Vd_WB(Vector3d{0, 0, 0}, sample.vd_WB);

      desired_velocities.Set("robot::ball", Value{Vd_WB});
      dut.get_input_port_desired_cartesian_velocities().FixValue(
          context.get(), desired_velocities);

      const VectorXd commanded_velocities =
          dut.get_output_port_commanded_velocity().Eval(*context);
      results.push_back(commanded_velocities);

      const SpatialVelocity<double> V_WB_full =
          GetBallSpatialVelocity(dut.plant(), commanded_velocities);
      const SpatialVelocity<double> V_WB_masked(
          V_WB_full.rotational(),
          V_WB_full.translational().cwiseProduct(sample.v_WB_mask));
      const SpatialVelocity<double> V_WB_expected(
          Vector3d(0, 0, 0),
          sample.v_WB_expected.cwiseProduct(sample.v_WB_mask));

      EXPECT_TRUE(CompareMatrices(V_WB_masked.get_coeffs(),
                                  V_WB_expected.get_coeffs(), 1e-14));
    }
    return results;
  }
};

TEST_F(DifferentialInverseKinematicsTest, Structure) {
  /* No costs or constraints; we're just checking structure. */
  DiffIk dut = MakeDiffIk(std::make_unique<Recipe>());

  // Port check.
  EXPECT_EQ(&dut.GetInputPort("position"), &dut.get_input_port_position());
  EXPECT_EQ(&dut.GetInputPort("nominal_posture"),
            &dut.get_input_port_nominal_posture());
  EXPECT_EQ(&dut.GetInputPort("desired_cartesian_velocities"),
            &dut.get_input_port_desired_cartesian_velocities());
  EXPECT_EQ(&dut.GetInputPort("desired_cartesian_poses"),
            &dut.get_input_port_desired_cartesian_poses());
  EXPECT_EQ(&dut.GetOutputPort("commanded_velocity"),
            &dut.get_output_port_commanded_velocity());

  // plant() aliases into the collision checker's plant.
  EXPECT_EQ(&dut.plant(), &dut.collision_checker().plant());

  // Construction parameters:
  EXPECT_EQ(dut.active_dof(),
            DofMask::MakeFromModel(dut.collision_checker().plant(),
                                   std::string(kRobotName)));
  EXPECT_EQ(dut.time_step(), kTimeStep);
  EXPECT_EQ(dut.task_frame().name(), "world");
}

/* A simple ingredient whose sole purpose is to examine the contents of details
and compare it with what we'd expect to see. */
class TestIngredient final : public Ingredient {
 public:
  TestIngredient() = default;
  ~TestIngredient() final = default;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final {
    // program?
    if (details->v_next.size() != dut_->active_dof().count()) {
      throw std::runtime_error("Detail has wrong number of decision variables");
    }
    if (&details->collision_checker != &dut_->collision_checker()) {
      throw std::runtime_error("Detail collision checker doesn't match system");
    }
    if (details->active_dof != dut_->active_dof()) {
      throw std::runtime_error("Detail active dof doesn't match system");
    }
    if (details->time_step != dut_->time_step()) {
      throw std::runtime_error("Detail time step doesn't match system");
    }
    // TODO(sean.curtis): Extend the test to confirm the details that arise from
    // the context: plant_context, nominal_posture, X_TGlist, VdTGlist, JvTGs.
    return {};
  }

  void set_dut(const DiffIk* dut) {
    DRAKE_DEMAND(dut != nullptr);
    dut_ = dut;
  }

 private:
  const DiffIk* dut_{};
};

/* DiffIkSystem is responsible for collecting up the "details" of the program
for the recipe/ingredient. We'll confirm that the right values get passed. */
TEST_F(DifferentialInverseKinematicsTest, ImplementRecipe) {
  auto recipe = std::make_unique<Recipe>();
  // We need a quadratic cost function in order to evaluate the output.
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<TestIngredient>();
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));
  ingredient->set_dut(&dut);

  // To trigger TestIngredient::AddToProgram(), we have to evaluate the output
  // port; so create a context, connect dummy values, and evaluate.
  auto context = dut.CreateDefaultContext();
  VectorXd q(dut.plant().num_positions());
  q.setZero();
  dut.get_input_port_position().FixValue(context.get(), q);
  dut.get_input_port_nominal_posture().FixValue(context.get(), q);
  BusValue desired_vel;
  desired_vel.Set("robot::ball", Value{SpatialVelocity<double>(
                                     Vector3d::Zero(), Vector3d::Zero())});
  dut.get_input_port_desired_cartesian_velocities().FixValue(context.get(),
                                                             desired_vel);

  // If CallbackDetails don't match expectations, this throws.
  EXPECT_NO_THROW(dut.get_output_port_commanded_velocity().Eval(*context));
}

/* Test the correctness of the LeastSquares use of its axis mask.

We don't test LeastSquaresCost more generally. This cost is applied in all
constraint tests. Its properties are affirmed again and again based on the
projection of the desired velocity onto the convex hull of the feasible velocity
region. */
TEST_F(DifferentialInverseKinematicsTest, LeastSquaresAxisMask) {
  {
    /* Mask out everything except for vx, vy and the answer shouldn't change.
    The "masked" rows were already zero. */
    auto recipe = std::make_unique<Recipe>();
    auto ingredient = std::make_shared<DiffIk::LeastSquaresCost>();
    drake::string_unordered_map<drake::Vector6d> masks;
    masks["robot::ball"] = drake::Vector6d(0, 0, 0, 1, 1, 0);
    recipe->AddIngredient(ingredient);
    ingredient->cartesian_axis_masks = masks;
    DiffIk dut = MakeDiffIk(std::move(recipe));
    std::vector<Sample> samples{
        {.vd_WB = Vector3d{1, -2, 0},
         .v_WB_expected = Vector3d{1, -2, 0},
         .description = "Only vx and vy included"},
    };
    EvaluateSamples(dut, samples);
  }

  {
    /* Also mask out vy; the solution ignores vd_y. */
    auto recipe = std::make_unique<Recipe>();
    auto ingredient = std::make_shared<DiffIk::LeastSquaresCost>();
    drake::string_unordered_map<drake::Vector6d> masks;
    // Omitting a critical cartesian velocity element means we won't get it.
    masks["robot::ball"] = drake::Vector6d(0, 0, 0, 1, 0, 0);
    recipe->AddIngredient(ingredient);
    ingredient->cartesian_axis_masks = masks;
    DiffIk dut = MakeDiffIk(std::move(recipe));
    std::vector<Sample> samples{
        {.vd_WB = Vector3d{1, -2, 0},
         .v_WB_expected = Vector3d{1, 0, 0},
         .v_WB_mask = Vector3d(1, 0, 1),
         .description = "Only vx included"},
    };
    EvaluateSamples(dut, samples);
  }
}

/* We'll use the least-squares cost without constraints to confirm the value of
the desired velocity inferred from position; in the absence of constraints, the
least squares cost will always report the desired velocity as the solution. */
TEST_F(DifferentialInverseKinematicsTest, VelocityFromPosition) {
  auto recipe = std::make_unique<Recipe>();
  AddLeastSquaresCost(recipe.get());
  DiffIk dut = MakeDiffIk(std::move(recipe));

  auto context = dut.CreateDefaultContext();

  VectorXd q(dut.plant().num_positions());
  q.setZero();
  dut.get_input_port_position().FixValue(context.get(), q);
  dut.get_input_port_nominal_posture().FixValue(context.get(), q);

  BusValue desired_positions;
  const Vector3d pd_WB(2.5, -3.5, 0);
  desired_positions.Set("robot::ball", Value{RigidTransformd(pd_WB)});

  dut.get_input_port_desired_cartesian_poses().FixValue(context.get(),
                                                        desired_positions);

  const VectorXd commanded_velocities =
      dut.get_output_port_commanded_velocity().Eval(*context);

  const SpatialVelocity<double> V_WB =
      GetBallSpatialVelocity(dut.plant(), commanded_velocities);

  // The unconstrained velocity -- ignoring Vd_TG_limits.
  const Vector3d v_WB_candidate = pd_WB * (kK_VX / kTimeStep);
  const Vector3d vd_WB_limits = MakeSpatialLimits().translational();
  auto my_clamp = [](double value, double limit) {
    return std::clamp(value, -limit, limit);
  };
  // The expected velocity, constrained by the Vd_TG_limits.
  const Vector3d v_WB_expected(my_clamp(v_WB_candidate[0], vd_WB_limits[0]),
                               my_clamp(v_WB_candidate[1], vd_WB_limits[1]), 0);
  const SpatialVelocity<double> V_WB_expected(Vector3d(0, 0, 0), v_WB_expected);

  EXPECT_TRUE(
      CompareMatrices(V_WB.get_coeffs(), V_WB_expected.get_coeffs(), 1e-14));
}

/* The joint centering cost resolves scenarios where the optimal solution isn't
unique. The cost isn't _required_ -- an optimal solution will be produced
regardless.

The redundant robot with 3-dofs has a constant, configuration-independent Jv_TG.
The transform from q_active to the null space is simply P = [1/2, 1/2, -1/√2].
We also know that the so-called nominal pose is the zero vector (see
EvaluateSamples()). Therefore, for a given Vd_WB, we can pick a _specific_ pose
tq hat will shift v_next away from Osqp's default value.  See below for details.
*/
TEST_F(DifferentialInverseKinematicsTest, JointCenteringCost) {
  // The desired velocity. Note: the v_active that produces this with the
  // *smallest* projection into the nullspace is v₁ = (1.5, -0.5, 1/√2);
  // P⋅v₁ = 0. Curiously, that is what Osqp returns as a solution.
  const Vector3d vd_WB(2.0, 0, 0);

  // We can force the solution to be the solution v₂ = [2, 0, 0]. We can
  // achieve this by making sure that:
  //
  //     P⋅v₂ = P(qₙ − q)
  //     P⋅v₂ = P(−q)         - qₙ is hard-coded as the zero vector.
  //     P⋅v₂ = −P⋅q
  //        1 = −P⋅q
  //       -1 = P⋅q
  //
  // So, we simply need to pick a q such that P⋅q = -1. We arbitrarily pick
  // q = [-1, -1, 0] as non-trivial but simple to verify.
  const Vector3d q(-1, -1, 0);

  // We'll collect two velocity commands -- one without joint centering and one
  // with. We expect the two commands to be significantly different *and* that
  // the centered_command is, essentially, [2, 0, 0].

  std::vector<VectorXd> commands;
  for (bool add_centering : {false, true}) {
    auto recipe = std::make_unique<Recipe>();
    AddLeastSquaresCost(recipe.get());
    if (add_centering) {
      auto ingredient = std::make_shared<DiffIk::JointCenteringCost>();
      recipe->AddIngredient(ingredient);
    }
    DiffIk dut = MakeDiffIk(std::move(recipe), true);

    std::vector<Sample> samples{
        {.vd_WB = vd_WB,
         .v_WB_expected = vd_WB,
         .description = fmt::format("Overactuated robot {} joint centering",
                                    add_centering ? "with" : "without"),
         .q_active = q}};

    commands.push_back(EvaluateSamples(dut, samples)[0]);
  }
  const VectorXd& uncentered_command = commands[0];
  const VectorXd& centered_command = commands[1];

  // They don't match, even with a *large* tolerance.
  EXPECT_FALSE(CompareMatrices(uncentered_command, centered_command, 1e-1));

  // The centered command is our target velocity.
  Vector3d expected_command(2, 0, 0);
  EXPECT_TRUE(CompareMatrices(centered_command, expected_command, 1e-15));
}

/* We'll repeat the previous joint centering test, but, this time, we'll mask
out one of the two cartesian velocity elements.

By masking out Vy, we change the null space of the Jacobian and its
corresponding P:

        |   0    1  0 |
    P = | -1/√2  0  1 |

To control the output, we want -P⋅v = P⋅q.

If I want the commanded velocity to be [1.5, 0, 0]ᵀ:

    P⋅v = P⋅[1.5, 0, 0]ᵀ = [0, -1.5/√2]ᵀ
So,
   [0, 1.5/√2]ᵀ = P⋅q  -->  q = [-0.5, 0, 1/√2]ᵀ
*/
TEST_F(DifferentialInverseKinematicsTest, JointCenteringCostAxisMask) {
  const Vector3d vd_WB(1.5, 2.0, 0);
  const Vector3d expected_command(1.5, 0, 0);
  // q picked such that we get the expected command.
  const Vector3d q(-0.5, 0, 1 / std::sqrt(2));

  drake::string_unordered_map<drake::Vector6d> masks;
  // Masking out vy.
  masks["robot::ball"] = drake::Vector6d(0, 0, 0, 1, 0, 0);
  const Vector3d vd_WB_masked(1.5, 0, 0);

  std::vector<VectorXd> commands;
  for (bool add_centering : {false, true}) {
    auto recipe = std::make_unique<Recipe>();
    AddLeastSquaresCost(recipe.get(), masks);
    if (add_centering) {
      auto ingredient = std::make_shared<DiffIk::JointCenteringCost>();
      ingredient->cartesian_axis_masks = masks;
      recipe->AddIngredient(ingredient);
    }
    DiffIk dut = MakeDiffIk(std::move(recipe), true);

    std::vector<Sample> samples{
        {.vd_WB = vd_WB,
         .v_WB_expected = vd_WB_masked,
         .v_WB_mask = Vector3d(1, 0, 1),
         .description = fmt::format("Overactuated robot {} joint centering",
                                    add_centering ? "with" : "without"),
         .q_active = q}};

    commands.push_back(EvaluateSamples(dut, samples)[0]);
  }
  const VectorXd& uncentered_command = commands[0];
  const VectorXd& centered_command = commands[1];

  EXPECT_FALSE(CompareMatrices(uncentered_command, centered_command, 1e-1));

  EXPECT_TRUE(CompareMatrices(centered_command, expected_command, 1e-15));
}

/* We're starting the ball at p_WB = (0, 0, 0). The cartesian position limit
constraint limits the velocity by placing a box around the current position and
disallowing velocities that would cause the position to leave the box in *one*
time step.

We'll create a rectangular box around the position and show that as long as
p_WB + Vd_WB * Δt lies within the box, then the velocity is accepted, otherwise
it is projected.

         vd₀     vd₁          vd₀ and vd₁ lie outside the constraint and get
          ●       ●           projected onto the box.
          ┆       ┆
     2 ┼──○───────○────┐      vd₂ lies within the box and is the optimal
       │               │      solution.
     0 ┼  ·       ●    │
       │ p_WB     vd₂  │
    -2 ┼──┼────────────┤
      -1  0            5
*/
TEST_F(DifferentialInverseKinematicsTest, CartesianPositionLimitConstraint) {
  auto recipe = std::make_unique<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient =
      std::make_shared<DiffIk::CartesianPositionLimitConstraint>();
  ingredient->cartesian_bounds_lower = Vector3d(-1, -2, 0);
  ingredient->cartesian_bounds_upper = Vector3d(5, 2, 0);
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  std::vector<Sample> samples{
      {.vd_WB = Vector3d{0, 3 / kTimeStep, 0},
       .v_WB_expected = Vector3d{0, 2 / kTimeStep, 0},
       .description = "vd0 - velocity clipped; direction unchanged"},
      {.vd_WB = Vector3d{3 / kTimeStep, 3 / kTimeStep, 0},
       .v_WB_expected = Vector3d{3 / kTimeStep, 2 / kTimeStep, 0},
       .description = "vd1 - velocity projected; direction change"},
      {.vd_WB = Vector3d{3 / kTimeStep, 0, 0},
       .v_WB_expected = Vector3d{3 / kTimeStep, 0, 0},
       .description = "vd2 - velocity already feasible"},
  };

  SCOPED_TRACE("CartesianPositionLimitConstraint");
  EvaluateSamples(dut, samples);
}

/* The Cartesian velocity constraint limits places a box in _velocity_ space.
If the desired velocity is not already in the box, it gets projected onto it.
The box is mirrored around the zero velocity along each axis.

                    vd₁
                      ●         vd₀ and vd₁ are outside the feasible region of
                      ┆         velocities and get projected onto it.
          1 ┼─────────○────┐
   vd₀ ●┄┄┄┄○              │    vd₂ is already feasible.
            │              │
            │       vd₂ ●  │
         -1 ┼──────────────┤
            -2             2
*/
TEST_F(DifferentialInverseKinematicsTest, CartesianVelocityLimitConstraint) {
  auto recipe = std::make_unique<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient =
      std::make_shared<DiffIk::CartesianVelocityLimitConstraint>();
  ingredient->spatial_velocity_limit = VectorXd(6);
  ingredient->spatial_velocity_limit << 0, 0, 0, 2, 1, 0;
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  std::vector<Sample> samples{
      {.vd_WB = Vector3d{-2.5, 0.5, 0},
       .v_WB_expected = Vector3d{-2, 0.5, 0},
       .description = "vd0 - velocity clipped on the x-axis"},
      {.vd_WB = Vector3d{1, 3, 0},
       .v_WB_expected = Vector3d{1, 1, 0},
       .description = "vd1 - velocity clipped on the y-axis"},
      {.vd_WB = Vector3d{1.5, -0.5, 0},
       .v_WB_expected = Vector3d{1.5, -0.5, 0},
       .description = "vd2 - velocity already feasible"},
  };

  SCOPED_TRACE("CartesianVelocityLimitConstraint");
  EvaluateSamples(dut, samples);
}

/* CollisionConstraint constrains the next velocity by representing an obstacle
with a half space constraint in velocity space. The normal of the half space
boundary aligns with the gradient of the signed distance function. The plane's
distance to the zero-velocity origin is the speed necessary to travel the
signed distance in a single time step (ϕ/Δt).

We'll create a scenario using the spherical obstacle located at the origin and
compute velocities for the robot point to work its way past.

    Cartesian Space                     Velocity Space

                  Wy
                  ┆                                          vy
                **┆**                           vd₀░░┃        ┆
              *  ·┆·  *  ├── ϕ ─┤                ●┄┄┄○        ┆
            * ·   ┆   · *                       ░░░░░┃ vd₁    ┆
           * ·    ┆    · *                      ░░░░░┃  ●     ┆
    ┄┄┄┄┄┄┄*┄·┄┄┄┄┼┄┄┄┄·┄*┄┄┄┄┄┄⊙┄ Wx           ┄┄┄┄┄┃┄┄┄┄┄┄┄┄┼┄┄┄┄  vx
           *  ·   │   ·  *                      ░░░░░┃        ┆
             *   ·┆·   *                        ░░░░░┃        ┆
                **┆**                           ░░░░░├─ ϕ/Δt ─┤
            -1    │    1

    · - obstacle boundary                ░┃ - velocity-space proxy for obstacle
    * - obstacle with safety offset      vd₀ is infeasible and gets projected
    ⊙ - "ball" object                    vd₁ is feasible and is taken as
    ϕ - distance from ball to safety         solution
        boundary
*/
TEST_F(DifferentialInverseKinematicsTest, CollisionConstraint) {
  const double safety_dist = 0.25;
  // Obstacle geometry is a sphere with radius 1.
  const double obstacle_safety_radius = 1.0 + safety_dist;
  const double phi = 1.0;
  // We'll position the ball to achieve the target ϕ.
  const double p_WB_x = phi + obstacle_safety_radius;
  // p_WB_y = 0 --> ∇ϕ = [1, 0, 0].
  const VectorXd q_active_init = (VectorXd(2) << p_WB_x, 0).finished();

  for (const double influence : {0.1, 10.0}) {
    auto recipe = std::make_unique<Recipe>();
    AddLeastSquaresCost(recipe.get());
    auto ingredient = std::make_shared<DiffIk::CollisionConstraint>();
    ingredient->safety_distance = safety_dist;
    ingredient->influence_distance = influence;
    recipe->AddIngredient(ingredient);
    DiffIk dut = MakeDiffIk(std::move(recipe));

    std::vector<Sample> samples{
        {.vd_WB = Vector3d{-phi * 0.9 / kTimeStep, 0.5, 0},
         .v_WB_expected = Vector3d{-phi * 0.9 / kTimeStep, 0.5, 0},
         .description = "vd0 - already feasible",
         .q_active = q_active_init}};
    // When influence distance < ϕ, the obstacle adds no constraint.
    if (influence < phi) {
      samples.push_back({.vd_WB = Vector3d{-phi * 2 / kTimeStep, 1, 0},
                         .v_WB_expected = Vector3d{-phi * 2 / kTimeStep, 1, 0},
                         .description = "vd1 - influence ignores obstacle",
                         .q_active = q_active_init});
    } else {
      samples.push_back(
          {.vd_WB = Vector3d{-phi * 2 / kTimeStep, 1, 0},
           .v_WB_expected = Vector3d{-phi / kTimeStep, 1, 0},
           .description = "vd1 - velocity clipped by the half space",
           .q_active = q_active_init});
    }

    SCOPED_TRACE(fmt::format("CollisionConstraint: influence = {}", influence));
    EvaluateSamples(dut, samples);
  }

  // We can also use the SelectDataForCollisionConstraintFunction callback to
  // exclude the obstacle.
  {
    auto select_nothing = [](const DofMask&, const RobotClearance&, VectorXd*,
                             Eigen::MatrixXd*) {};
    // When influence distance > ϕ, the constraint is not added.
    auto recipe = std::make_unique<Recipe>();
    AddLeastSquaresCost(recipe.get());
    auto ingredient = std::make_shared<DiffIk::CollisionConstraint>();
    ingredient->safety_distance = safety_dist;
    ingredient->influence_distance = 100;
    ingredient->select_data_for_collision_constraint = select_nothing;
    recipe->AddIngredient(ingredient);
    DiffIk dut = MakeDiffIk(std::move(recipe));

    std::vector<Sample> samples{
        {.vd_WB = Vector3d{-phi * 2 / kTimeStep, 1, 0},
         .v_WB_expected = Vector3d{-phi * 2 / kTimeStep, 1, 0},
         .description = "vd1 - obstacle filtered",
         .q_active = q_active_init},
    };

    SCOPED_TRACE("CollisionConstraint - obstacle filtered");
    EvaluateSamples(dut, samples);
  }
}

/* The constraint on joint velocities is conceptually simple, but has some
implementations that make it tricky. Rather than simply applying a bounding box
in the joint velocity space based on the declared joint velocity limits, the
constraint modifies the domain of the box based on where the robot configuration
is relative to the declared joint position limits. The closer to a position
boundary `q` is the tighter the velocity bound towards that boundary is. So,
we'll have to test with various q to see the effect of the constraint.

Note: that the the box isn't simply scaled; the boundaries on one side are
moved. Limits in the direction opposite the near boundary are unchanged.

We'll bound the robot to move in within a region.

            Cartesian space

            Wy
             ┆
    2  ┏━━━━━┿━━━━━━━━━━━━━━━┓
       ┃ ┌───┼─────────────┐ ┃            ┃ - q limits boundary
       ┃ │   ┆         q_b │٭┃q_c         │ - padded boundary; outside this
       ┃ │   ┆┌┉┉┉┉┉┉┉┐  ٭ │ ┃  ٭ q_d         boundary, velocity @ 0%.
       ┃ │   ┆┊ q_a ٭ ┊    │ ┃            ┊ - padded boundary; inside this
 ┄┄┄┄┄┄╂┄┼┄┄┄┼└┉┉┉┉┉┉┉┘┄┄┄┄┼┄╂┄┄ Wx           boundary, velocity limit @ 100%.
       ┃ │   ┆             │ ┃            ٭ - configuration at which we'll
       ┃ └───┼─────────────┘ ┃                evaluate diff ik. Each one is in a
   -1  ┗━━━━━┿━━━━━━━━━━━━━━━┛                region of interest.
      -1     ┆               3

            Velocity space

           Vy
            ┆
     2 ┌────┼─────────┐
       │    ┆         │
       │    ┆         │
       │    ┆         │
       │    ┆         │
    ┄┄┄┼┄┄┄┄┼┄┄┄┄┄┄┄┄┄┼┄┄┄┄ Vx      │ - limits on v
       │    ┆         │
  -0.5 └────┼─────────┘
      -1    ┆        1.5

Ideally, we would configure the robot so that q is nearest each position
boundary in turn. We're only sampling a portion of these, relying on the
structure of the code to pick up the per-axis symmetry we need for free. */
TEST_F(DifferentialInverseKinematicsTest, JointVelocityLimitConstraint) {
  // If q is less than 0.25 units away from the q-limit boundary, the velocity
  // limit is zero.
  const double kMargin = 0.25;
  // If q is at least 0.5 units away from the q-limit boundary, the full
  // velocity limit will be used.
  const double kInfluence = 0.5;
  // clang-format off
  // x- and y- quantities map to q1 and q0, respectively.
  const JointLimits joint_limits(
      Vector2d(-1, -1),      /* q lower */
      Vector2d(3, 2),        /* q upper */
      Vector2d(-0.5, -1),    /* v lower */
      Vector2d(1.5, 2),      /* v upper */
      Vector2d(0, 0),        /* a lower */
      Vector2d(0, 0));       /* a upper */
  // clang-format on
  // Velocity values at the boundaries of specified limits.
  const Vector3d& min_vel = joint_limits.velocity_lower();
  const Vector3d& max_vel = joint_limits.velocity_upper();
  const Vector3d vel_offset(0.1, 0.1, 0);

  const auto& q_lower = joint_limits.position_lower();
  const auto& q_upper = joint_limits.position_upper();
  const Vector2d q_center = (q_lower + q_upper) / 2;
  // At q_a, the full joint limits should apply.
  const Vector2d q_a(q_upper[0] - kMargin - kInfluence - 0.1, q_center[1]);
  // At q_b, the velocity limit in the +v0 (+Wx) direction is cut in half.
  const double decay_width = kInfluence - kMargin;
  const Vector2d q_b(q_upper[0] - kMargin - decay_width / 2, q_center[1]);
  // At q_c, the velocity limit in the +v0 (+Wx) direction is zero.
  const Vector2d q_c(q_upper[0] - 0.5 * kMargin, q_center[1]);
  // q_d lies outside the position limits; the velocity limit in the +V0
  // direction is zero.
  const Vector2d q_d(q_upper[0] + 1, q_center[1]);

  auto recipe = std::make_unique<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<DiffIk::JointVelocityLimitConstraint>();
  ingredient->joint_limits = joint_limits;
  ingredient->min_margin = kMargin;
  ingredient->influence_margin = kInfluence;
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  // Note: in the description below, an "outward Vd" is towards the near
  // boundary. Similarly, an "inward" Vd points away from the near boundary.
  std::vector<Sample> samples{
      {.vd_WB = max_vel + vel_offset,
       .v_WB_expected = max_vel,
       .description = "At q_a, clamp outward Vd to standard joint limits",
       .q_active = q_a},
      {.vd_WB = max_vel - vel_offset,
       .v_WB_expected = max_vel - vel_offset,
       .description = "At q_a, feasible Vd is still feasible",
       .q_active = q_a},
      {.vd_WB = min_vel - vel_offset,
       .v_WB_expected = min_vel,
       .description = "At q_a, clamp inward Vd to standard joint limits",
       .q_active = q_a},
      {.vd_WB = max_vel,
       .v_WB_expected = max_vel.cwiseProduct(Vector3d(0.5, 1, 0)),
       .description = "At q_b, clamp outward Vd to half standard joint limits",
       .q_active = q_b},
      {.vd_WB = max_vel * 0.49,
       .v_WB_expected = max_vel * 0.49,
       .description = "At q_b, inward Vd already feasible",
       .q_active = q_b},
      {.vd_WB = min_vel - vel_offset,
       .v_WB_expected = min_vel,
       .description = "At q_b, clamp inward Vd to standard joint limits",
       .q_active = q_b},
      {.vd_WB = max_vel,
       .v_WB_expected = max_vel.cwiseProduct(Vector3d(0, 1, 0)),
       .description = "At q_c, clamp outward Vd to zero",
       .q_active = q_c},
      {.vd_WB = min_vel,
       .v_WB_expected = min_vel,
       .description = "At q_c, inward Vd already feasible",
       .q_active = q_c},
      {.vd_WB = max_vel,
       .v_WB_expected = max_vel.cwiseProduct(Vector3d(0, 1, 0)),
       .description = "At q_d, clamp outward Vd to zero",
       .q_active = q_d},
      {.vd_WB = min_vel,
       .v_WB_expected = min_vel,
       .description = "At q_d, inward Vd already feasible",
       .q_active = q_d},
  };

  SCOPED_TRACE("JointVelocityLimitConstraint");
  EvaluateSamples(dut, samples);
}

}  // namespace
}  // namespace operational_space_control
}  // namespace anzu
