#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/linear_system.h"

using drake::math::RigidTransformd;
using drake::systems::Context;
using drake::test::LimitMalloc;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static VectorX<double> CalcGeneralizedAccelerations(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalForwardDynamics(context).get_vdot();
  }
};

namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

// Fixture to perform forward dynamics tests on a model of a KUKA Iiwa arm. The
// base is free.
class KukaIiwaModelForwardDynamicsTests : public test::KukaIiwaModelTests {
 protected:
  // Given the state of the joints in q and v, this method calculates the
  // forward dynamics for the floating KUKA iiwa robot using the articulated
  // body algorithm. The pose and spatial velocity of the base are arbitrary.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] v robot's joint velocities (generalized velocities).
  // @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaArticulatedBodyAlgorithm(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    DRAKE_DEMAND(vdot != nullptr);
    // Update joint positions and velocities.
    VectorX<double> x(q.size() + v.size());
    x << q, v;
    SetState(x);
    *vdot =
        MultibodyPlantTester::CalcGeneralizedAccelerations(*plant_, *context_);
  }

  // This method calculates the forward dynamics for the 7-DOF KUKA iiwa robot
  // by explicitly solving for the inverse of the mass matrix.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] v robot's joint velocities (generalized velocities).
  // @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaMassMatrixSolve(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    DRAKE_DEMAND(vdot != nullptr);
    // Update joint positions and velocities.
    VectorX<double> x(q.size() + v.size());
    x << q, v;
    SetState(x);

    // Compute force element contributions.
    MultibodyForces<double> forces(*plant_);
    plant_->CalcForceElementsContribution(*context_, &forces);

    // Construct M, the mass matrix.
    const int nv = plant_->num_velocities();
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);

    // Compute tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W via inverse
    // dynamics.
    const VectorX<double> zero_vdot = VectorX<double>::Zero(nv);
    const VectorX<double> tau_id =
        plant_->CalcInverseDynamics(*context_, zero_vdot, forces);

    // Solve for vdot.
    *vdot = M.llt().solve(-tau_id);
  }

  // Verify the solution obtained using the ABA against a reference solution
  // computed by explicitly taking the inverse of the mass matrix.
  void CompareForwardDynamics(const Eigen::Ref<const VectorX<double>>& q,
                              const Eigen::Ref<const VectorX<double>>& v) {
    // Compute forward dynamics using articulated body algorithm.
    VectorX<double> vdot(plant_->num_velocities());
    CalcForwardDynamicsViaArticulatedBodyAlgorithm(q, v, &vdot);

    // Compute forward dynamics using mass matrix.
    VectorX<double> vdot_expected(plant_->num_velocities());
    CalcForwardDynamicsViaMassMatrixSolve(q, v, &vdot_expected);

    // We estimate the difference between vdot and vdot_expected to be in the
    // order of machine epsilon times the condition number "kappa" of the mass
    // matrix.
    const int nv = plant_->num_velocities();
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
    const double kappa = 1.0 / M.llt().rcond();

    // Compare expected results against actual vdot.
    const double kRelativeTolerance = kappa * kEpsilon;
    EXPECT_TRUE(CompareMatrices(vdot, vdot_expected, kRelativeTolerance,
                                MatrixCompareType::relative));
  }
};

// This test is used to verify the correctness of the articulated body algorithm
// for solving forward dynamics. The output from the articulated body algorithm
// is compared against the output from solving using the mass matrix. We verify
// the computation for an arbitrary set of robot states.
TEST_F(KukaIiwaModelForwardDynamicsTests, ForwardDynamicsTest) {
  // Joint angles and velocities.
  VectorX<double> q(kNumJoints), qdot(kNumJoints);
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Test 1: Static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  CompareForwardDynamics(q, qdot);

  // Test 2: Another static configuration.
  q << q30, -q45, q60, -q30, q45, -q60, q30;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  CompareForwardDynamics(q, qdot);

  // Test 3: Non-static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  CompareForwardDynamics(q, qdot);

  // Test 4: Another non-static configuration.
  q << -q45, q60, -q30, q45, -q60, q30, -q45;
  qdot << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  CompareForwardDynamics(q, qdot);

  // Test 5: Another non-static configuration.
  q << q30, q45, q60, -q30, -q45, -q60, 0;
  qdot << 0.3, -0.1, 0.4, -0.1, 0.5, -0.9, 0.2;
  CompareForwardDynamics(q, qdot);
}

// For complex articulated systems such as a humanoid robot, round-off errors
// might accumulate leading to (close to, by machine epsilon) unphysical ABIs in
// the Articulated Body Algorithm. See related issue #12640.
// This test verifies this does not trigger a spurious exception.
GTEST_TEST(MultibodyPlantForwardDynamics, AtlasRobot) {
  MultibodyPlant<double> plant(0.0);
  const std::string model_path =
      FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");
  Parser parser(&plant);
  auto atlas_instance = parser.AddModelFromFile(model_path);
  plant.Finalize();

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  for (JointIndex joint_index(0); joint_index < plant.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant.get_joint(joint_index);
    // This model only has weld and revolute joints. Weld joints have zero DOFs.
    if (joint.num_velocities() != 0) {
      const RevoluteJoint<double>& revolute_joint =
          dynamic_cast<const RevoluteJoint<double>&>(joint);
      // Arbitrary non-zero angle.
      revolute_joint.set_angle(context.get(), 0.5 * joint_index);
    }
  }
  const int num_actuators = plant.num_actuators();
  plant.get_actuation_input_port(atlas_instance)
      .FixValue(context.get(), VectorX<double>::Zero(num_actuators));
  auto derivatives = plant.AllocateTimeDerivatives();
  {
    // CalcTimeDerivatives should not be allocating, but for now we have a few
    // remaining fixes before it's down to zero:
    //  2 temps in MbTS::CalcArticulatedBodyForceCache (F_B_W_, tau_).
    //  1 temp  in MbP::AssembleActuationInput (actuation_input).
    //  2 temps in MbTS::DoCalcTimeDerivatives (xdot, qdot).
    LimitMalloc guard({ .max_num_allocations = 5 });
    EXPECT_NO_THROW(plant.CalcTimeDerivatives(*context, derivatives.get()));
  }

  // Verify that the implicit dynamics match the continuous ones.
  Eigen::VectorXd residual = plant.AllocateImplicitTimeDerivativesResidual();
  plant.CalcImplicitTimeDerivativesResidual(*context, *derivatives, &residual);
  // Note the slightly looser tolerance of 4e-13 which was required for this
  // test.
  EXPECT_TRUE(CompareMatrices(
      residual, Eigen::VectorXd::Zero(plant.num_multibody_states()), 4e-13));
}

// Verifies we can do forward dynamics on a model with a zero-sized state.
GTEST_TEST(WeldedBoxesTest, ForwardDynamicsViaArticulatedBodyAlgorithm) {
  // Problem parameters.
  const double kCubeSize = 1.5;  // Size of the box, in meters.
  const double kBoxMass = 2.0;   // Mass of each box, in Kg.
  // We use discrete_update_period = 0 to set a continuous model that uses the
  // Articulated Body Algorithm (ABA) to evaluate forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  // Set a model with two boxes anchored to the world via weld joints.
  const Vector3d p_BoBcm_B = Vector3d::Zero();
  const UnitInertia<double> G_BBcm =
      UnitInertia<double>::SolidBox(kCubeSize, kCubeSize, kCubeSize);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(kBoxMass, p_BoBcm_B,
                                                     G_BBcm);
  // Create two rigid bodies.
  const auto& boxA = plant.AddRigidBody("boxA", M_BBo_B);
  const auto& boxB = plant.AddRigidBody("boxB", M_BBo_B);

  // Desired transformation for the boxes in the world.
  const RigidTransformd X_WA(Vector3d::Zero());
  const RigidTransformd X_WB(Vector3d(kCubeSize, 0, 0));
  const RigidTransformd X_AB = X_WA.inverse() * X_WB;

  // Pin boxA to the world and boxB to boxA with weld joints.
  plant.WeldFrames(plant.world_body().body_frame(), boxA.body_frame(), X_WA);
  plant.WeldFrames(boxA.body_frame(), boxB.body_frame(), X_AB);

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  // Evaluate forward dynamics.
  const VectorXd vdot =
      MultibodyPlantTester::CalcGeneralizedAccelerations(plant, *context);
  EXPECT_EQ(vdot.size(), 0);
}

std::unique_ptr<systems::LinearSystem<double>> MakeLinearizedCartPole(
    double time_step) {
  const std::string sdf_file = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");

  MultibodyPlant<double> plant(time_step);
  Parser(&plant).AddModelFromFile(sdf_file);
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  plant.get_actuation_input_port().FixValue(context.get(), 0.);
  plant.SetPositionsAndVelocities(context.get(),
                                  Eigen::Vector4d{0, M_PI, 0, 0});

  return systems::Linearize(plant, *context,
                            plant.get_actuation_input_port().get_index(),
                            systems::OutputPortSelection::kNoOutput);
}

// This test revealed a bug (#17037) in MultibodyPlant<AutoDiffXd>.
GTEST_TEST(MultibodyPlantTest, CartPoleLinearization) {
  const double kTimeStep = 0.1;
  auto ct_linearization = MakeLinearizedCartPole(0.0);
  auto dt_linearization = MakeLinearizedCartPole(kTimeStep);

  // v_next = v0 + time_step * (A * x + B * u)
  // q_next = q0 + time_step * v_next
  Eigen::Matrix4d A_expected = Eigen::Matrix4d::Identity();
  A_expected.bottomRows<2>() +=
      kTimeStep * ct_linearization->A().bottomRows<2>();
  A_expected.topRows<2>() += kTimeStep * A_expected.bottomRows<2>();
  Eigen::Vector4d B_expected;
  B_expected.bottomRows<2>() =
      kTimeStep * ct_linearization->B().bottomRows<2>();
  B_expected.topRows<2>() = kTimeStep * B_expected.bottomRows<2>();

  EXPECT_TRUE(CompareMatrices(dt_linearization->A(), A_expected, 1e-16));
  EXPECT_TRUE(CompareMatrices(dt_linearization->B(), B_expected, 1e-16));
}
// TODO(amcastro-tri): Include test with non-zero actuation and external forces.

// Helper function to create a uniform-density cube B and add it to a plant.
// @param[in] plant MultibodyPlant to which body B is added.
// @param[in] body_name name of the body that is being added to the plant.
// @param[in] mass mass of the body that is being added to the plant.
// @param[in] length length, width, and depth of the cube-shaped body.
// @param[in] skip_validity_check setting which is `true` to skip the validity
//  check on the new body B's spatial inertia, which ensures an exception is not
//  thrown when setting body B's spatial inertia (which would otherwise occur if
//  mass or link_length is NaN). Avoiding this early exception allows for a
//  later exception to be thrown in a subsequent function and tested below.
// @note The position vector from Bcm (B's center of mass which is at the cube's
// geometric center) to Bo (B's origin) is p_BcmBo = (-length/2, 0, 0).
const RigidBody<double>& AddCubicalLink(
    MultibodyPlant<double>* plant,
    const std::string& body_name,
    const double mass,
    const double length,
    const bool skip_validity_check = false) {
  DRAKE_DEMAND(plant != nullptr);
  const Vector3<double> p_BoBcm_B(length / 2, 0, 0);
  const UnitInertia<double> G_BBcm_B = UnitInertia<double>::SolidCube(length);
  const UnitInertia<double> G_BBo_B =
      G_BBcm_B.ShiftFromCenterOfMass(-p_BoBcm_B);
  const SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B,
                                       skip_validity_check);
  return plant->AddRigidBody(body_name, M_BBo_B);
}

/* Adds a joint between the `inboard` and `outboard` bodies in the given
 `plant`. The joint type must be one of: "Revolute", "Prismatic", or "Planar".
 */
void AddJoint(const std::string& joint_type, const RigidBody<double>& inboard,
              const RigidBody<double>& outboard,
              MultibodyPlant<double>* plant) {
  // TODO: We're doing forward *dynamics*; gravity points in the -Wz direction.
  // If I introduce a revoluteZ joint; then gravity doesn't really play a role.
  // For example, for a fixed mass and configuration, changing revolution from
  // being around the Z axis to the X axis will lead to a change in behavior.
  // Are we properly accounting for that -- my documentation isn't currently.
  if (joint_type == "Revolute") {
    plant->AddJoint<RevoluteJoint>(
        fmt::format("revoluteZ_{}_{}", inboard.name(), outboard.name()),
        inboard, std::nullopt, outboard, std::nullopt,
        Vector3<double>::UnitZ());
  } else if (joint_type == "Prismatic") {
    plant->AddJoint<PrismaticJoint>(
        fmt::format("prismaticX_{}_{}", inboard.name(), outboard.name()),
        inboard, std::nullopt, outboard, std::nullopt,
        Vector3<double>::UnitX());
  } else if (joint_type == "Planar") {
    plant->AddJoint<PlanarJoint>(
        fmt::format("planar_{}_{}", inboard.name(), outboard.name()), inboard,
        std::nullopt, outboard, std::nullopt, Vector3<double>::Zero());
  } else {
    DRAKE_UNREACHABLE();
  }
}

/* Configuration of a single hinge matrix test instance on a *distal* body. */
struct HingeMatrixConfig {
  std::string description;  // Must be a valid test name: CamelCase.
  std::vector<double> masses;
  bool expect_throw{};
  // Must be one of the strings supported by AddJoint() (see above).
  std::string joint_type;
};

/* Defines the test name suffix for a particular test parameter. */
std::ostream& operator<<(std::ostream& out, const HingeMatrixConfig& c) {
  out << c.joint_type << c.description;
  return out;
}

/* Concerns about this test:
  1. We're joining bodies at their centers of mass. What affect does that have
     on revolute joints.
  2. How do we worry about joint dofs and gravity?
 */

/* Test fixture for confirming the efficacy of detecting bad mass properties.
 Tests for this fixture serve two purposes:

   1. Confirm that there are bad models/configurations that do get caught by
      the detection mechanism. Essentially, serve as an existence proof.
      At the same time we'll try confirm that "slightly corrected" versions of
      the same bad data doesn't throw.
   2. Enumerate several surprising cases which produce false negatives (we
      expected an error, but didn't get one). They can serve as a basis for
      further analysis. */
class HingeInertiaMatrixTest
    : public ::testing::TestWithParam<HingeMatrixConfig> {
 public:
  /* We want the plant's forward dynamics to use the Articulated Body Algorithm;
   initializing it as a *continuous* plant (time_step = 0) guarantees that. */
  HingeInertiaMatrixTest() : plant_(0.0) {}

 protected:
  MultibodyPlant<double> plant_;
};

/* Fixture for tests that consider bad mass for *distal* bodies. */
class DistalHingeMatrixTest : public HingeInertiaMatrixTest {};

/* Tests problems with the mass on *distal* bodies. For a distal body with a
 1-dof joint, we have no "near singular" values. We test a number of masses
 (zero, close to zero, sufficiently non-zero) to confirm the expected behavior.
 The non-zero values are chosen empirically as producing reliable outcomes
 across all supported platforms but otherwise have no meaning. */
std::vector<HingeMatrixConfig> MakeDistalTests() {
  // TODO(mitiguy): Is the "no near-singular value" thing due to it being distal
  // or being 1-dof?

  std::vector<HingeMatrixConfig> tests;
  // We get the same behavior across all joint types.
  for (const char* joint_type : {"Prismatic", "Revolute", "Planar"}) {
    tests.push_back({
        .description = "ZeroMass",
        .masses = {0.0},
        .expect_throw = true,
        .joint_type = joint_type,
    });
    tests.push_back({
        .description = "MicroMass",
        .masses = {1e-140},
        .expect_throw = false,  // Micro values still valid.
        .joint_type = joint_type,
    });
    tests.push_back({
        .description = "SmallMass",
        .masses = {1e-4},
        .expect_throw = false,  // Evidence of non-zero non-throw.
        .joint_type = joint_type,
    });
  }
  return tests;
}

INSTANTIATE_TEST_SUITE_P(DistalBodies,
                         DistalHingeMatrixTest,
                         testing::ValuesIn(MakeDistalTests()));

/* Simply connects a body with configured mass and joint to the world. Attempts
 to evaluate forward dynamics and confirms whether an exception is thrown or
 not according to expectations. */
TEST_P(DistalHingeMatrixTest, DistalBody) {
  const HingeMatrixConfig& config = GetParam();

  // Set up the plant with body A connected to the world with the requested
  // joint type.
  const RigidBody<double>& body_A = AddCubicalLink(
      &plant_, "bodyA", config.masses[0], 3 /* arbitrary length */);
  AddJoint(config.joint_type, plant_.world_body(), body_A, &plant_);
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  if (config.expect_throw) {
    DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context),
                                "An internal mass matrix associated.+");
  } else {
    DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context));
  }
}

/* Fixture for tests that consider bad mass for a chain of bodies. */
class InboardHingeMatrixTest : public HingeInertiaMatrixTest {};

/* Tests problems with the mass on *distal* bodies. For a distal body with a
 1-dof joint, we have no "near singular" values. We test a number of masses
 (zero, close to zero, sufficiently non-zero) to confirm the expected behavior.
 The non-zero values are chosen empirically as producing reliable outcomes
 across all supported platforms but otherwise have no meaning. */
std::vector<HingeMatrixConfig> MakeInboardTests() {
  std::vector<HingeMatrixConfig> tests;

  // We get the same behavior across all joint types -- more or less. Exceptions
  // are noted.
  for (const std::string joint_type : {"Prismatic", "Revolute", "Planar"}) {
    tests.push_back({
        .description = "ZeroUnitB",
        .masses = {0.0, 1.0},
        .expect_throw = true,
        .joint_type = joint_type,
    });
    // N.B. Masses larger than 1e99 (e.g., 1e100) will cease to throw. The
    // *math* says it should, but the numerics diverge at this point.
    tests.push_back({
        .description = "ZeroAMassiveB",
        .masses = {0, 1e99},
        .expect_throw = true,
        .joint_type = joint_type,
    });
    tests.push_back({
        .description = "SmallAMassiveB",
        .masses = {1e-7, 1e99},
        .expect_throw = false,  // TODO(mitiguy): Huge ratio; why no throw?
        .joint_type = joint_type,
    });
    if (joint_type != "Revolute") {
      // Note: The revolute joint exhibits inconsistent behavior. Is it that the
      // revolute joint is wrong, or that the translation is wrong?
      tests.push_back({
          .description = "SmallALargeB",
          .masses = {1e-7, 1e9},  // Why is this ratio interesting?
          .expect_throw = true,   // TODO(mitiguy): Smaller ratio throws!
          .joint_type = joint_type,
      });
    }
  }

  // The weird, one-off exceptions to the pattern above.
  tests.push_back({
      .description = "SmallALargeB",
      .masses = {1e-7, 1e9},  // Why is this ratio interesting?
      .expect_throw = false,  // TODO(mitiguy: Why is this one odd?
      .joint_type = "Revolute",
  });
  return tests;
}

INSTANTIATE_TEST_SUITE_P(InboardBodies,
                         InboardHingeMatrixTest,
                         testing::ValuesIn(MakeInboardTests()));

TEST_P(InboardHingeMatrixTest, InboardBody) {
  const HingeMatrixConfig& config = GetParam();

  // Set up the plant with bodies A and B connected to the world in a chain with
  // the indicated joint.
  const RigidBody<double>& body_A =
      AddCubicalLink(&plant_, "bodyA", config.masses[0], 1 /* length */);
  const RigidBody<double>& body_B =
      AddCubicalLink(&plant_, "bodyB", config.masses[1], 1 /* length */);
  AddJoint(config.joint_type, plant_.world_body(), body_A, &plant_);
  AddJoint(config.joint_type, body_A, body_B, &plant_);
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  if (config.expect_throw) {
    DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context),
                                "An internal mass matrix associated.+");
  } else {
    DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context));
  }
}

// ----------------------------------------------------------------------------
// There are 5 tests below whose purpose is to improving MultibodyPlant feedback
// for invalid mass/inertia properties. The tests below uses forward dynamics in
// continuous mode and test whether an exception is thrown for invalid
// mass/inertia properties that are detected in the Articulated Body Algorithm.
//
//
// The first two tests use a single body that is connected to world by a one
// degree-of-freedom joint. The 1ˢᵗ test uses a prismatic (translational) joint
// whereas the 2ⁿᵈ test uses a revolute joint. These first two tests ensure an
// exception is thrown for a single body that is either translating or rotating
// and is at the end of a topological chain.
//
// Tests 3 and 4 use multiple bodies with multiple degrees of freedom (albeit
// with each joint being a single degree of freedom). Test 3 uses prismatic
// joints whereas test 4 uses revolute joints. Tests 3 and 4 ensure an exception
// can be regardless of *whether or not the problematic joint is the last in a
// topological chain of bodies*.
//
// Also, since the expected tolerances for rotating
// bodies are several orders of magnitude smaller than translating bodies, both
// translational and rotational joints are tested. *BUT YOU USE THE SAME
// VALUES!*
//
// Test 5 uses a single rigid body that connects to the world with a 6
// degree-of-freedom "free-joint" and hence involves a 6 x 6 articulated body
// hinge inertia matrix (whereas tests 1, 2, 3, 4 only involve a 1 x 1 matrix).
// ----------------------------------------------------------------------------

#if 0

// Perform a forward dynamic analysis for a planar triple pendulum consisting of
// rigid bodies A, B, C, each which has an inboard revolute-pin axis that is
// parallel to the world's Z-direction. Verify an exception is thrown if body
// C's mass and inertia are zero or bodies A and B's mass and inertia are zero.
GTEST_TEST(TestHingeInertiaMatrix, ThrowErrorForZeroInertiaRotating3Bodies) {
  // Create a plant with constructor argument = 0 to signal use of a continuous
  // model (and hence the Articulated Body Algorithm for forward dynamics).
  MultibodyPlant<double> plant(0.0);

  // TODO(SeanCurtis-TRI): Why play with gravity when we can use pins on
  // different pin joints? At this point, it seems like you're playing with
  // people's minds but without any real value for the test.

  // World X is vertically downward and world Y is horizontally-right.
  plant.mutable_gravity_field().set_gravity_vector(Vector3d(9.8, 0, 0));

  // Create the bodies in the triple pendulum.
  // Reminder: Cubical link B has p_BoBcm_B = [length/2, 0, 0].
  const double mA = 1, mB = 1, mC = 0;  // Mass of links A, B, C.
  const double length = 0.2;      // Length of uniform-density links A, B, C.
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&plant, "bodyB", mB, length);
  const RigidBody<double>& body_C = AddCubicalLink(&plant, "bodyC", mC, length);

  // Add body A to world W  with a Z-revolute joint.
  const RigidBody<double>& world_body = plant.world_body();
  const RevoluteJoint<double>& WA_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("WA_revolute_jointZ",
      world_body, std::nullopt, body_A, std::nullopt, Vector3<double>::UnitZ());

  // Add body B to body A with a Z-revolute joint. To do this, create a
  // "fixed" frame Af at the X-distal end of link A that connects to B.
  const Vector3d p_AoAfo_A(length, 0, 0);  // Position vector from Ao to Afo.
  const math::RigidTransformd X_AAf(p_AoAfo_A);  // Rigid transform from A to Af
  const RevoluteJoint<double>& AB_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("AB_revolute_jointZ",
      body_A, X_AAf, body_B, std::nullopt, Vector3<double>::UnitZ());

  // Add body C to body B with a Z-revolute joint. To do this, create a
  // "fixed" frame Bf at the X-distal end of link B that connects to C.
  const Vector3d p_BoBfo_B(length, 0, 0);  // Position vector from Bo to Bfo.
  const math::RigidTransformd X_BBf(p_BoBfo_B);  // Rigid transform from B to Bf
  const RevoluteJoint<double>& BC_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("BC_revolute_jointZ",
      body_B, X_BBf, body_C, std::nullopt, Vector3<double>::UnitZ());

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();
  // TODO: `ptr` is not style guide compliant; `raw` would be acceptable.
  // Sor just call context.get() in all three cases.
  systems::Context<double>* context_ptr = context.get();
  WA_revolute_jointZ.set_angle(context_ptr, 0);
  AB_revolute_jointZ.set_angle(context_ptr, 0);
  BC_revolute_jointZ.set_angle(context_ptr, 0);

  // Verify proper assertion is thrown if mA = mB = 1, mc = 0 since articulated
  // body hinge inertia matrix = [0] which is not positive definite.
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "An internal mass matrix .+ body bodyB to body bodyC "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if mA = 1, mB = 0, mC = 1.
  body_A.SetMass(context_ptr, 1);
  body_B.SetMass(context_ptr, 0);
  body_C.SetMass(context_ptr, 1);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))

  // TODO: These two tests are really the most interesting; in this case the
  // throw condition is dependent on the mass *and* configuration.

  // Verify proper assertion is thrown if mA = mB = 0, mC = 1 since articulated
  // body hinge inertia matrix ≈ [-1.90126e-17] which is not positive-definite.
  // TODO(Mitiguy) Improve robustness of test. If mC = 2, no assertion is thrown
  //  since the articulated body hinge inertia matrix ≈ [1.582e-17].
  //  The tests herein were chosen because they worked -- based on computation
  //  in computer hardware available from CI (Continuous Integration) testing.
  body_A.SetMass(context_ptr, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if the initial revolute angles for WA and BC
  // are each 0 degrees and AB's initial revolute angle is far-enough from zero.
  AB_revolute_jointZ.set_angle(context_ptr,  0.1 * M_PI/180);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))
}

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-mass, zero-inertia free body (both translates and rotates).
GTEST_TEST(TestHingeInertiaMatrix, ThrowErrorForZeroMassInertiaFreeBody) {
  // Create a plant with constructor argument = 0 to signal use of a continuous
  // model (and hence the Articulated Body Algorithm for forward dynamics).
  MultibodyPlant<double> plant(0.0);

  const double mA = 0;  // Mass of link A.
  const double length = 0.3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();

  // Verify assertion is thrown if mA = 0 since articulated body hinge inertia
  // matrix is 6 x 6 zero matrix (albeit with NaN in upper-triangular part).
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if mA = 1E-4 since articulated body hinge
  // inertia matrix is positive definite (and far from singular).
  body_A.SetMass(context.get(), 1E-4);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))
}
#endif
}  // namespace
}  // namespace multibody
}  // namespace drake
