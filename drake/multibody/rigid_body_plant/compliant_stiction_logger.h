#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/collision/element.h"
#include "drake/systems/framework/leaf_system.h"

// Forward declaration
template <typename T> class RigidBodyTree;

namespace drake {
namespace systems {

// Information for computing the slipping behavior at a contact point. Contains:
//    1. The ids of the two collision elements in the colliion (a and b).
//    2. The contact normal (pointing into element a), measured and
//        expressed in the world frame.
//    3. The contact point, in the world frame.
//    4. The relative velocity of the elements' two bodies *at* the contact
//      point, measured and expressed in the world frame.
struct SlipData {
  SlipData(DrakeCollision::ElementId a, DrakeCollision::ElementId b,
           const Vector3<double>& n, const Vector3<double>& p,
           const Vector3<double>& v)
      : element_a(a), element_b(b), normal_W(n), p_WC(p), v_AB_W(v) {}
  DrakeCollision::ElementId element_a;
  DrakeCollision::ElementId element_b;
  Vector3<double> normal_W;
  Vector3<double> p_WC;
  Vector3<double> v_AB_W;
};

// This class is a utility class for measuring slip velocity at contact points.
//  It performs work upon calling Publish. During Publish it:
//    1. Pulls the contact results from the RigidBodyTree.
//    2. For each contact point and normal:
//      Computes the relative velocity of *coincident* points in the frames of
//      the two bodies involved in the collision.
//    3. Stores per-time-step information of:
//      a. The geometry ids involved in the collision
//      b. The contact point and normal
//      c. The relative velocity.
//  At the close of simulation, the aggregated data can be accessed and analyzed
//  reporting statistics about the relative velocities of the contact points.
class SlipDetector : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlipDetector)

  // Takes the tree that it operates on -- it should be connected to that tree's
  // plant. It also takes an *optional* sample period. If none is provided
  // (or a non-positive value is provided), it will publish at each step.
  // Otherwise, it publishes at the given period.
  SlipDetector(const RigidBodyTree<double>* tree, double period=-1.0);

  void DoPublish(
      const systems::Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override;

  const InputPortDescriptor<double>& contact_input_port() const {
    return get_input_port(contact_input_port_);
  }

  const InputPortDescriptor<double>& state_input_port() const {
    return get_input_port(tree_state_input_port_);
  }

  const std::multimap<double, SlipData>& get_slip_data() const {
    return slip_data_;
  };

 private:
  const RigidBodyTree<double>* tree_{nullptr};
  int tree_state_input_port_{-1};
  int contact_input_port_{-1};
  mutable std::multimap<double, SlipData> slip_data_;
};

}  // namespace systems
}  // namespace drake
