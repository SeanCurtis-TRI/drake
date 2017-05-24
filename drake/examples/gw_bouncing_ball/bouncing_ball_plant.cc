#include "drake/examples/gw_bouncing_ball/bouncing_ball_plant.h"

#include <algorithm>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/spatial_pose.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

using geometry::Contact;
using geometry::FrameKinematicsSet;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::HalfSpace;
using geometry::SourceId;
using geometry::SpatialPose;
using geometry::Sphere;
using systems::Value;
using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        GeometrySystem<T>* geometry_system)
    : source_id_(source_id), geometry_system_(geometry_system) {
  geometry_port_ = this->DeclareAbstractOutputPort(
      Value<FrameKinematicsSet<T>>(
          geometry_system->MakeDefaultFrameKinematicsSet(source_id))).
      get_index();
  state_port_ =
      this->DeclareVectorOutputPort(BouncingBallVector<T>()).get_index();
  this->DeclareContinuousState(
      BouncingBallVector<T>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");

  ball_frame_id_ = geometry_system->RegisterFrame(
      source_id, GeometryFrame<T>("ball_frame", Isometry3<T>::Identity() /*X_PF = X_WF*/));
  ball_id_ = geometry_system->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance<T>>(Isometry3<double>::Identity(), /*X_FG*/
                                       make_unique<Sphere>(diameter_ / 2.0)));
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::OutputPortDescriptor<T>&
BouncingBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
BouncingBallPlant<T>::get_geometry_output_port() const {
  return systems::System<T>::get_output_port(geometry_port_);
}

template <typename T>
void BouncingBallPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // 1) Output for the plant's state.
  get_mutable_state_output(output)->set_value(get_state(context).get_value());

  // 2) Output for GeometrySystem's input.
  FrameKinematicsSet<T> fks =
      geometry_system_->GetFrameKinematicsSet(context, source_id_);
  const BouncingBallVector<T>& state = get_state(context);
  fks.ReportPose(ball_frame_id_,
                 SpatialPose<T>(Quaternion<T>::Identity(),
                                Vector3<T>(0, 0, state.z())));
  output->GetMutableData(geometry_port_)
      ->template GetMutableValue<FrameKinematicsSet<T>>() = fks;
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>* derivative_vector = get_mutable_state(derivatives);

  std::vector<Contact<T>> contacts;
  geometry_system_->ComputeContact(context, &contacts);
  T fC = 0; // the contact force
  if (contacts.size() > 0) {
    if (contacts.size() != 1) throw std::logic_error(
          "Bouncing ball should always have at most one contact.");

    const T& x = contacts[0].depth;  // Penetration depth, > 0 at penetration.
    const T& xdot = -state.zdot();  // Penetration rate, > 0 during penetration.

//    PRINT_VAR(contacts[0].depth);
//    PRINT_VAR(state.zdot());

    fC = k_ * x * (1.0 + d_ * xdot);
  }
  derivative_vector->set_z(state.zdot());
  const T fN = max(0.0, fC);

  derivative_vector->set_zdot((- m_ * g_ + fN) / m_);
}

// BouncingBallPlant has no constructor arguments, so there's no work to do
// here.
//template <typename T>
//BouncingBallPlant<AutoDiffXd>* BouncingBallPlant<T>::DoToAutoDiffXd() const {
//  return new BouncingBallPlant<AutoDiffXd>();
//}
//
//template <typename T>
//BouncingBallPlant<symbolic::Expression>*
//BouncingBallPlant<T>::DoToSymbolic() const {
//  return new BouncingBallPlant<symbolic::Expression>();
//}

template class BouncingBallPlant<double>;
//template class BouncingBallPlant<AutoDiffXd>;
//template class BouncingBallPlant<symbolic::Expression>;

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
