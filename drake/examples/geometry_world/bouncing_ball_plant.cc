#include "drake/examples/geometry_world/bouncing_ball_plant.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shapes/shapes.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

using geometry::PenetrationAsPointPair;
using geometry::FrameIdVector;
using geometry::FramePoseSet;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::HalfSpace;
using geometry::SourceId;
using geometry::Sphere;
using systems::Context;
using systems::Value;
using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        GeometrySystem<T>* geometry_system,
                                        const Vector2<T>& init_position)
    : source_id_(source_id), geometry_system_(geometry_system),
      init_position_(init_position) {
  geometry_query_port_ = this->DeclareAbstractInputPort().get_index();
  state_port_ =
      this->DeclareVectorOutputPort(BouncingBallVector<T>(),
                                    &BouncingBallPlant::CopyStateToOutput)
          .get_index();
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(&BouncingBallPlant::AllocateFrameIdOutput,
                                      &BouncingBallPlant::CalcFrameIdOutput)
      .get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &BouncingBallPlant::AllocateFramePoseOutput,
          &BouncingBallPlant::CalcFramePoseOutput).get_index();

    this->DeclareContinuousState(
      BouncingBallVector<T>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");

  ball_frame_id_ = geometry_system->RegisterFrame(
      source_id, GeometryFrame<T>("ball_frame",
                                  Isometry3<T>::Identity() /*X_PF = X_WF*/));
  ball_id_ = geometry_system->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance<T>>(Isometry3<double>::Identity(), /*X_FG*/
                                       make_unique<Sphere>(diameter_ / 2.0)));
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::InputPortDescriptor<T>&
BouncingBallPlant<T>::get_geometry_query_input_port() const {
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_geometry_id_output_port() const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

// Updates the state output port.
template <typename T>
void BouncingBallPlant<T>::CopyStateToOutput(
    const Context<T>& context,
    BouncingBallVector<T>* state_output_vector) const {
  state_output_vector->set_value(get_state(context).get_value());
}

template <typename T>
FramePoseSet<T> BouncingBallPlant<T>::AllocateFramePoseOutput(
    const Context<T>& context) const {
  FramePoseSet<T> poses(source_id_);
  poses.AddValue(Isometry3<T>::Identity());
  return poses;
}

template <typename T>
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseSet<T>* pose_set) const {
  const BouncingBallVector<T>& state = get_state(context);
  Isometry3<T> pose = Isometry3<T>::Identity();
  pose.translation() << init_position_(0), init_position_(1), state.z();
  FramePoseSet<T> poses(source_id_);
  poses.AddValue(pose);
  *pose_set = poses;
}

template <typename T>
FrameIdVector BouncingBallPlant<T>::AllocateFrameIdOutput(
    const MyContext& context) const {
  FrameIdVector ids(source_id_);
  ids.AddFrameId(ball_frame_id_);
  return ids;
}

template <typename T>
void BouncingBallPlant<T>::CalcFrameIdOutput(const MyContext &context,
                                              FrameIdVector *) const {
  // TODO(SeanCurtis-TRI): Only take action if the topology has changed; this
  // system never changes the topology.
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>* derivative_vector = get_mutable_state(derivatives);

  const geometry::QueryHandle<T>& query_handle =
      this->template EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryHandle<T>>();

  std::vector<PenetrationAsPointPair<T>> penetrations =
      geometry_system_->ComputePenetration(query_handle);
  T fC = 0;  // the contact force
  if (penetrations.size() > 0) {
    for (const auto& penetration : penetrations) {
      if (penetration.id_A == ball_id_ || penetration.id_B == ball_id_) {
        const T
            &x = penetration.depth;  // Penetration depth, > 0 at penetration.
        const T
            &xdot = -state.zdot();   // Penetration rate, > 0 implies increasing
                                     // penetration.

//    PRINT_VAR(contacts[0].depth);
//    PRINT_VAR(state.zdot());

        fC = k_ * x * (1.0 + d_ * xdot);
      }
    }
  }
  derivative_vector->set_z(state.zdot());
  const T fN = max(0.0, fC);

  derivative_vector->set_zdot((- m_ * g_ + fN) / m_);
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

template class BouncingBallPlant<double>;
// template class BouncingBallPlant<AutoDiffXd>;
// template class BouncingBallPlant<symbolic::Expression>;

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
