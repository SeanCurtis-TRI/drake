#include "drake/multibody/tree/body_node.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/planar_mobilizer.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

class MultibodyElementTester {
  public:
  MultibodyElementTester() = delete;
  static void set_index(Body<double>* element, BodyIndex index) {
    element->set_parent_tree(nullptr, index);
  }
};

namespace internal {

using std::move;
using Eigen::Vector3d;
using drake::systems::Context;

// Friend to give us access to private members.
class BodyNodeTester {
 public:
  BodyNodeTester() = delete;

  static void CallFactor(const BodyNode<double>& node,
                         const MatrixUpTo6<double>& D_B) {
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<double>> llt_D_B;
    node.CalcArticulatedBodyHingeInertiaMatrixFactorization(D_B, &llt_D_B);
  }
};

namespace {

// TODO(SeanCurtis-TRI): Consider moving this into common test utilities so
// other tests can create mocked bodies. Possibly template it again.

// Minimal definition of a body that we can use to construct a BodyNode.
class DummyBody : public Body<double> {
 public:
  DummyBody(std::string name, BodyIndex index)
      : Body(move(name), ModelInstanceIndex(0)) {
    MultibodyElementTester::set_index(this, index);
  }
  int get_num_flexible_positions() const override { return 0; }
  int get_num_flexible_velocities() const override { return 0; }
  double default_mass() const override { return 0; }
  RotationalInertia<double> default_rotational_inertia() const override {
    return {};
  }
  const double& get_mass(const Context<double>&) const override {
    return mass_;
  }
  const Vector3d CalcCenterOfMassInBodyFrame(
      const Context<double>&) const override {
    return Vector3d::Zero();
  }
  Vector3d CalcCenterOfMassTranslationalVelocityInWorld(
      const Context<double>&) const override {
    return Vector3d::Zero();
  }
  SpatialInertia<double> CalcSpatialInertiaInBodyFrame(
      const Context<double>&) const override {
    return SpatialInertia<double>::MakeUnitary();
  };
  std::unique_ptr<Body<double>> DoCloneToScalar(
      const MultibodyTree<double>&) const override {
    return nullptr;
  }
  std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>&) const override {
    return nullptr;
  }
  std::unique_ptr<Body<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>&) const override {
    return nullptr;
  }

 protected:
  double mass_{0};
};

// This simply confirms that the message keys on the mobilizer properties to
// provide the properly classified message contents (rotation and/or
// translation). This is the only place we test the specific wording of the
// exception message. In all other test sites, a small indication should be
// sufficient.
GTEST_TEST(BodyNodeTest, FactorHingeMatrixErrorMessages) {
  using Tester = BodyNodeTester;
  using Matrix = MatrixUpTo6<double>;

  // Hinge matrices with a single scalar.
  Matrix scalar_mat(1, 1);
  scalar_mat(0, 0) = -1;  // This should *definitely* fail.

  const DummyBody parent("parent", world_index());
  const DummyBody child("child", BodyIndex(1));
  const BodyNode<double> parent_node(nullptr, &parent, nullptr);

  {
    // Rotation only.
    const RevoluteMobilizer<double> mobilizer(
        parent.body_frame(), child.body_frame(), Vector3d{0, 0, 1});
    const BodyNode<double> body_node(&parent_node, &child, &mobilizer);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Tester::CallFactor(body_node, scalar_mat),
        "An internal mass matrix associated with the joint that connects body "
        "parent to body child is not positive-definite. Since the joint allows "
        "rotation, ensure body child \\(combined with other outboard bodies\\) "
        "has reasonable non-zero moments of inertia about the joint rotation "
        "axes.");
  }

  {
    // Translation only.
    const PrismaticMobilizer<double> mobilizer(
        parent.body_frame(), child.body_frame(), Vector3d{0, 0, 1});
    const BodyNode<double> body_node(&parent_node, &child, &mobilizer);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Tester::CallFactor(body_node, scalar_mat),
        "An internal mass matrix associated with the joint that connects body "
        "parent to body child is not positive-definite. Since the joint allows "
        "translation, ensure body child \\(combined with other outboard "
        "bodies\\) has a reasonable non-zero mass.");
  }

  {
    // Rotation and translation.
    const PlanarMobilizer<double> mobilizer(parent.body_frame(),
                                            child.body_frame());
    const BodyNode<double> body_node(&parent_node, &child, &mobilizer);
    // Here we're looking for indication that the both rotation and translation
    // messages are included (with appropriate whitespace between them).
    DRAKE_EXPECT_THROWS_MESSAGE(
        Tester::CallFactor(body_node, scalar_mat),
        "An internal .+ allows rotation.+ axes. Since the .+ allows "
        "translation.+");
  }
}

// This test seeks to characterize the conditions under which
// BodyNode<T>::CalcArticulatedBodyHingeInertiaMatrixFactorization()
// throws. By passing hand-crafted hinge matrices, we can precisely
// characterize the test. This allows us to make assertions about the
// quality of the test -- we rely on integration tests in
// multibody_plant_forward_dynamics_test.cc to assess if it plays well
// with the hinge matrices produced by MultibodyPlant.
GTEST_TEST(BodyNodeTest, FactorHingeMatrixThrows) {
  using Tester = BodyNodeTester;
  using Matrix = MatrixUpTo6<double>;

  const DummyBody world("world", world_index());
  const DummyBody body("child", BodyIndex(1));
  const RevoluteMobilizer<double> mobilizer(
      world.body_frame(), body.body_frame(), Vector3d{0, 0, 1});
  const BodyNode<double> world_node(nullptr, &world, nullptr);
  const BodyNode<double> body_node(&world_node, &body, &mobilizer);

  // Hinge matrices with a single scalar.
  Matrix scalar_mat(1, 1);

  // Valid hinge matrix values - none of these should throw.
  for (const double value : {1.0e-12, 1.0e-1, 1.0, 1.0e10, 1.0e100}) {
    scalar_mat(0, 0) = value;
    EXPECT_NO_THROW(Tester::CallFactor(body_node, scalar_mat))
        << "For expected good value: " << value;
  }

  // Values that should throw.
  for (const double value : {0.0, -1.0e-15, -1.0, -1.0e10}) {
    scalar_mat(0, 0) = value;
    EXPECT_THROW(Tester::CallFactor(body_node, scalar_mat), std::exception)
        << "For expected bad value: " << value;
  }
}

// TODO: Test for matrices that aren't 1x1.

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
