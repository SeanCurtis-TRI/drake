#include "drake/multibody/rigid_body_plant/compliant_stiction_logger.h"

#include "drake/multibody/rigid_body_plant/contact_info.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

using systems::LeafSystem;

SlipDetector::SlipDetector(const RigidBodyTree<double> *tree, double period)
    : LeafSystem<double>(), tree_(tree) {
  // Get the RigidBodyTree's state vector.
  const int vector_size =
      tree->get_num_positions() + tree->get_num_velocities();
  tree_state_input_port_ =
      DeclareInputPort(systems::kVectorValued, vector_size).get_index();
  // The contact input port.
  contact_input_port_ = this->DeclareAbstractInputPort().get_index();

  if (period > 0) {
    LeafSystem<double>::DeclarePeriodicPublish(period);
  } else {
    PublishEvent<double> event(systems::Event<double>::TriggerType::kPerStep);
    this->DeclarePerStepEvent(event);
  }
}

void SlipDetector::DoPublish(
    const systems::Context<double>& context,
    const std::vector<const PublishEvent<double>*>&) const {
  // 1. Pull the inputs (contact and tree state).
  const BasicVector<double>* input_vector =
      EvalVectorInput(context, tree_state_input_port_);
  VectorX<double> q =
      input_vector->get_value().head(tree_->get_num_positions());
  VectorX<double> v =
      input_vector->get_value().tail(tree_->get_num_velocities());
  auto kinsol = tree_->doKinematics(q, v);
  const ContactResults<double>& contacts =
      this->EvalAbstractInput(context, contact_input_port_)
          ->GetValue<ContactResults<double>>();

  // 2. Calculate and store the relative velocity at each contact point.
  const double kTime = context.get_time();
  for (int i = 0; i < contacts.get_num_contacts(); ++i) {
    const ContactInfo<double>& info = contacts.get_contact_info(i);
    const auto& contact_force = info.get_resultant_force();

    // Compute relative velocity of the contact
    const Vector3<double>& p_WC = contact_force.get_application_point();
    const auto& body_a =
        tree_->FindBody(info.get_element_id_1())->get_body_index();
    const auto& body_b =
        tree_->FindBody(info.get_element_id_2())->get_body_index();
    // The contact point in A's frame.
    const auto X_AW = kinsol.get_element(body_a)
        .transform_to_world.inverse(Eigen::Isometry);
    const Vector3<double> p_AAc = X_AW * p_WC;
    // The contact point in B's frame.
    const auto X_BW = kinsol.get_element(body_b)
        .transform_to_world.inverse(Eigen::Isometry);
    const Vector3<double> p_BBc = X_BW * p_WC;

    const auto JA =
        tree_->transformPointsJacobian(kinsol, p_AAc, body_a, 0, false);
    const auto JB =
        tree_->transformPointsJacobian(kinsol, p_BBc, body_b, 0, false);
    const auto J = JA - JB;
    // The *relative* velocity of the contact point in A relative to that in
    // B, expressed in the world frame.
    const auto v_Contact_BcAc_W = J * kinsol.getV();

    // Compute the actual value!!
    slip_data_.emplace(
        std::make_pair(kTime,
                       SlipData(info.get_element_id_1(),
                                info.get_element_id_2(),
                                contact_force.get_normal(),
                                p_WC,
                                v_Contact_BcAc_W)));
  }
}
}  // namespace systems
}  // namespace drake
