#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

// TODO(SeanCurtis-TRI): Use this to determine if there is any disagreement in
// the MaximumDepthCollision and CollisionExists. In other words, non-empty
// results should imply true in the former (and 0 and false for the latter).

namespace drake {
namespace examples {

using std::make_unique;
using std::unique_ptr;

using Eigen::VectorXd;

using multibody::collision::PointPair;
using multibody::joints::kFixed;
using parsers::urdf::AddModelInstanceFromUrdfFile;

DEFINE_int32(sample_count, 10, "The number of samples");

unique_ptr<RigidBodyTreed> MakeTree() {
  auto tree = make_unique<RigidBodyTreed>();

  AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(
          "drake/manipulation/models/"
          "iiwa_description/urdf/iiwa14_primitive_collision.urdf"),
      kFixed, nullptr /* weld to frame */, tree.get());
  return tree;
}

void single_test(int s, const VectorXd& q, const VectorXd& v, RigidBodyTreed* tree) {
  std::cout << s << ": " << q.transpose() << "\n";
  KinematicsCache<double> cache = tree->doKinematics(q, v);
  bool is_colliding = tree->CollisionsExist(cache, false);
  std::vector<PointPair<double>> pairs =
      tree->ComputeMaximumDepthCollisionPoints(cache, false);
  std::cout << "\t" << is_colliding << " | " << pairs.size() << " collisions\n";
  for (const auto& pair : pairs) {
    std::cout << "\t";
    std::cout << "\t" << pair.elementA->get_body()->get_name();
    std::cout << "\t" << pair.elementB->get_body()->get_name();
    std::cout << "\n";
  }
}

int main() {
  std::cout << FLAGS_sample_count << " samples\n";


  unique_ptr<RigidBodyTreed> tree = MakeTree();
  const int q_size = tree->get_num_positions();
  const int v_size = tree->get_num_velocities();
  VectorXd q = VectorXd::Zero(q_size);
  VectorXd v = VectorXd::Zero(v_size);

  // One generator per state element, clamped to the joint limits.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<std::uniform_real_distribution<double>> generators(q_size);
  for (int i = 0; i < tree->get_num_bodies(); ++i) {
    const RigidBody<double>& body = tree->get_body(i);
    if (body.has_joint()) {
      int index = body.get_position_start_index();
      const DrakeJoint& joint = body.getJoint();
      const Eigen::VectorXd& min_values = joint.getJointLimitMin();
      const Eigen::VectorXd& max_values = joint.getJointLimitMax();
      for (int j = 0; j < joint.get_num_positions(); ++j) {
        generators[index + j] = std::uniform_real_distribution<double>(
            min_values[j], max_values[j]);
      }
    }
  }
  std::cout << std::setprecision(17);

#if 1
  q << -0.71792274150046431, 0.6442051121730179, 2.5490024308609276, 0.27072069579295333, -1.295611995556293, -1.181454566638185, 1.0034856318580636;
  single_test(0, q, v, tree.get());
#else
  for (int s = 0; s < FLAGS_sample_count; ++s) {
    for (int i = 0; i < q_size; ++i) {
      q(i) = generators[i](gen);
    }
    std::cout << s << ": " << q.transpose() << "\n";
    KinematicsCache<double> cache = tree->doKinematics(q, v);
    bool is_colliding = tree->CollisionsExist(cache, false);
    std::vector<PointPair<double>> pairs =
        tree->ComputeMaximumDepthCollisionPoints(cache, false);
    std::cout << "\t" << is_colliding << " | " << pairs.size() << " collisions\n";
    for (const auto& pair : pairs) {
      std::cout << "\t";
      std::cout << "\t" << pair.elementA->get_body()->get_name();
      std::cout << "\t" << pair.elementB->get_body()->get_name();
      std::cout << "\n";
    }
  }
#endif
  return 0;
}

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::main();
}