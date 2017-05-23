#pragma once

#include <memory>

#include "drake/common/symbolic_formula.h"
#include "drake/examples/gw_bouncing_ball/gen/bouncing_ball_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

/// A model of a bouncing ball with Hunt-Crossley compliant contact model.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class BouncingBallPlant : public systems::LeafSystem<T> {
 public:
  BouncingBallPlant();
  ~BouncingBallPlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /// Returns the port to output state.
  const systems::OutputPortDescriptor<T>& get_output_port() const;

  void set_z(MyContext* context, const T& z) const {
    get_mutable_state(context)->set_z(z);
  }

  void set_zdot(MyContext* context, const T& zdot) const {
    get_mutable_state(context)->set_zdot(zdot);
  }

  /// BouncingBall mass in kg
  T m() const { return m_; }

  /// Stiffness constant.
  T k() const {return k_; }

  /// Hunt-Crossley's dissipation factor.
  T d() const {return d_; }

  /// Gravity in m/s^2
  T g() const { return g_; }

  explicit BouncingBallPlant(const BouncingBallPlant& other) = delete;
  BouncingBallPlant& operator=(const BouncingBallPlant& other) = delete;
  explicit BouncingBallPlant(BouncingBallPlant&& other) = delete;
  BouncingBallPlant& operator=(BouncingBallPlant&& other) = delete;

 protected:
  // System<T> override.
  BouncingBallPlant<AutoDiffXd>* DoToAutoDiffXd() const override;
  BouncingBallPlant<symbolic::Expression>* DoToSymbolic() const override;

 private:
  void DoCalcOutput(const MyContext& context, MyOutput* output) const override;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const BouncingBallVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const BouncingBallVector<T>&>(cstate.get_vector());
  }

  static BouncingBallVector<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<BouncingBallVector<T>*>(cstate->get_mutable_vector());
  }

  static BouncingBallVector<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<BouncingBallVector<T>*>(
        output->GetMutableVectorData(0));
  }

  static const BouncingBallVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static BouncingBallVector<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  const double diameter_{0.05};  // Ball diameter, just for visualization.
  const double m_{0.1};   // kg
  const double g_{9.81};  // m/s^2
  // Stiffness constant [N/m]. Estimated so that under its onw weight the ball
  // penetrates the plane by 1 mm.
  const double k_{m_* g_ / 0.001};
  // Hunt-Crossley's dissipation factor.
  const double d_{0.0};  // [s/m]
};

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
