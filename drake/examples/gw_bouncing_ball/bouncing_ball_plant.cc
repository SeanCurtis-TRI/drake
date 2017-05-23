#include "drake/examples/gw_bouncing_ball/bouncing_ball_plant.h"

#include <algorithm>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant() {
  this->DeclareVectorOutputPort(BouncingBallVector<T>());
  this->DeclareContinuousState(
      BouncingBallVector<T>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::OutputPortDescriptor<T>&
BouncingBallPlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void BouncingBallPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  get_mutable_output(output)->set_value(get_state(context).get_value());
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>* derivative_vector = get_mutable_state(derivatives);

  derivative_vector->set_z(state.zdot());

  const T& x = -state.z();  // Penetration depth, > 0 at penetration.
  const T& xdot = -state.zdot();  // Penetration rate, > 0 during penetration.

  const T fN = max(0.0, k_ * x * (1.0 - d_ * xdot));

  derivative_vector->set_zdot((- m_ * g_ + fN));
}

// BouncingBallPlant has no constructor arguments, so there's no work to do
// here.
template <typename T>
BouncingBallPlant<AutoDiffXd>* BouncingBallPlant<T>::DoToAutoDiffXd() const {
  return new BouncingBallPlant<AutoDiffXd>();
}

template <typename T>
BouncingBallPlant<symbolic::Expression>*
BouncingBallPlant<T>::DoToSymbolic() const {
  return new BouncingBallPlant<symbolic::Expression>();
}

template class BouncingBallPlant<double>;
template class BouncingBallPlant<AutoDiffXd>;
template class BouncingBallPlant<symbolic::Expression>;

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
