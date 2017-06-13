#include "drake/examples/geometry_world/free_ball_plant.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace examples {
namespace geometry_world {

using geometry::Contact;
using geometry::FrameIdVector;
using geometry::FramePoseSet;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::HalfSpace;
using geometry::SourceId;
using geometry::Sphere;
using systems::Value;
using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

template <typename T>
FreeBallPlant<T>::FreeBallPlant(SourceId source_id,
                                GeometrySystem<T>* geometry_system,
                                const Vector3<T>& init_position)
    : source_id_(source_id), geometry_system_(geometry_system),
      init_position_(init_position) {
  state_port_ =
      this->DeclareVectorOutputPort(MyState(6)).get_index();
  // 3 translational degrees of freedom and velocities.
  this->DeclareContinuousState(MyState(6), 3, 3, 0);

  ball_frame_id_ = geometry_system->RegisterFrame(
      source_id, GeometryFrame<T>("ball_frame",
                                  Isometry3<T>::Identity() /*X_PF = X_WF*/));
  ball_id_ = geometry_system->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance<T>>(Isometry3<double>::Identity(), /*X_FG*/
                                       make_unique<Sphere>(diameter_ / 2.0)));
  FrameIdVector ids(source_id);
  ids.AddFrameId(ball_frame_id_);
  geometry_id_port_ = this->DeclareAbstractOutputPort(
      Value<FrameIdVector>(ids)).get_index();
  FramePoseSet<T> poses(source_id);
  poses.AddValue(Isometry3<T>::Identity());
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
      Value<FramePoseSet<T>>(poses)).get_index();
}

template <typename T>
FreeBallPlant<T>::~FreeBallPlant() {}

template <typename T>
const systems::OutputPortDescriptor<T>&
FreeBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
FreeBallPlant<T>::get_geometry_id_output_port() const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
FreeBallPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
void FreeBallPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                        systems::SystemOutput<T>* output) const {
  // 1) Output for the plant's state.
  get_mutable_state_output(output)->set_value(get_state(context).get_value());

  // 2) Output for GeometrySystem's input.
  const MyState& state = get_state(context);
  Isometry3<T> pose = Isometry3<T>::Identity();
  pose.translation() <<
      state.GetAtIndex(0), state.GetAtIndex(1), state.GetAtIndex(2);
//  std::cout << "Ball: " << ball_id_ << " at " << pose.translation().transpose() << ", vel: ";
//  std::cout << state.GetAtIndex(0) << ", " <<  state.GetAtIndex(1)<< ", " <<  state.GetAtIndex(2) << "\n";
  FramePoseSet<T> poses(source_id_);
  poses.AddValue(pose);
  output->GetMutableData(geometry_pose_port_)
      ->template GetMutableValue<FramePoseSet<T>>() = poses;
}

// Compute the actual physics.
template <typename T>
void FreeBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const MyState& state = get_state(context);
  MyState* derivative_vector = get_mutable_state(derivatives);

  std::vector<Contact<T>> contacts;
  geometry_system_->ComputeContact(context, &contacts);
  Vector3<T> fC(0, 0, 0);
  // Hacky dissipative drag force. Apply a friction force counter to the ball's
  // velocity so that the ball eventually comes to rest.
  Vector3<T> vel = state.get_value().template tail<3>();
  fC += -vel * 0.1;

  for (const auto& contact : contacts) {
    if (contact.id_A == ball_id_ || contact.id_B == ball_id_) {
      const T& x = contact.depth;  // depth > 0 --> penetration.
      // TODO(SeanCurtis-TRI): Replace this with proper rate of change.
      const T& xdot = 0;  // rate > 0 --> increasing penetration.
      const Vector3<T> N = contact.id_A == ball_id_ ? -contact.nhat_AcBc_W : contact.nhat_AcBc_W;
//    PRINT_VAR(contacts[0].depth);
//    PRINT_VAR(state.zdot());
      fC += (k_ * x * (1.0 + d_ * xdot)) * N;
    }
  }

  derivative_vector->get_mutable_value().template head<3>() =
      state.get_value().template tail<3>();
  Vector3<T> a_WB = (m_ * g_ + fC) / m_;
  derivative_vector->get_mutable_value().template tail<3>() = a_WB;
}

// BouncingBallPlant has no constructor arguments, so there's no work to do
// here.
// template <typename T>
// BouncingBallPlant<AutoDiffXd>* BouncingBallPlant<T>::DoToAutoDiffXd() const {
//   return new BouncingBallPlant<AutoDiffXd>();
// }
//
// template <typename T>
// BouncingBallPlant<symbolic::Expression>*
// BouncingBallPlant<T>::DoToSymbolic() const {
//   return new BouncingBallPlant<symbolic::Expression>();
// }

template class FreeBallPlant<double>;
// template class BouncingBallPlant<AutoDiffXd>;
// template class BouncingBallPlant<symbolic::Expression>;

}  // namespace geometry_world
}  // namespace examples
}  // namespace drake
