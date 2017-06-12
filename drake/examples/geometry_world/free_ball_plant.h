#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_system.h"

namespace drake {
namespace examples {
namespace geometry_world {

template <typename T>
class FreeBallPlant : public systems::LeafSystem<T> {
 public:
  FreeBallPlant(geometry::SourceId source_id,
                geometry::GeometrySystem<T>* geometry_system,
                const Vector3<T>& init_position);
  ~FreeBallPlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;
  using MyState = systems::BasicVector<T>;

  /// Returns the port to output state.
  const systems::OutputPortDescriptor<T>& get_state_output_port() const;
  const systems::OutputPortDescriptor<T>& get_geometry_id_output_port() const;
  const systems::OutputPortDescriptor<T>& get_geometry_pose_output_port() const;

  void set_pos(MyContext* context, const Vector3<T>& pos) const {
    MyState* state = get_mutable_state(context);
    state->get_mutable_value().template head<3>() = pos;
  }

  void set_vel(MyContext* context, const Vector3<T>& vel) const {
    MyState* state = get_mutable_state(context);
    state->get_mutable_value().template tail<3>() = vel;
  }

  /// BouncingBall mass in kg
  T m() const { return m_; }

  /// Stiffness constant.
  T k() const {return k_; }

  /// Hunt-Crossley's dissipation factor.
  T d() const {return d_; }

  /// Gravity in m/s^2
  const Vector3<T>& g() const { return g_; }

  explicit FreeBallPlant(const FreeBallPlant&) = delete;
  FreeBallPlant& operator=(const FreeBallPlant&) = delete;
  explicit FreeBallPlant(FreeBallPlant&&) = delete;
  FreeBallPlant& operator=(FreeBallPlant&&) = delete;

 protected:
  // System<T> override.
//  BouncingBallPlant<AutoDiffXd>* DoToAutoDiffXd() const override;
//  BouncingBallPlant<symbolic::Expression>* DoToSymbolic() const override;

 private:
  void DoCalcOutput(const MyContext& context, MyOutput* output) const override;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const MyState& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const MyState&>(cstate.get_vector());
  }

  static MyState* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<MyState*>(cstate->get_mutable_vector());
  }

  MyState* get_mutable_state_output(MyOutput *output) const {
    return dynamic_cast<MyState*>(
        output->GetMutableVectorData(state_port_));
  }

  static const MyState& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static MyState* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  geometry::SourceId source_id_;
  const geometry::GeometrySystem<T>* geometry_system_;
  const Vector3<T> init_position_;
  geometry::FrameId ball_frame_id_;
  geometry::GeometryId ball_id_;

  int geometry_id_port_;
  int geometry_pose_port_;
  int state_port_;

  const double diameter_{0.1};  // Ball diameter, just for visualization.
  const double m_{0.1};   // kg
  const Vector3<double> g_{0, 0, -9.81};  // m/s^2
  // Stiffness constant [N/m]. Estimated so that under its onw weight the ball
  // penetrates the plane by 1 mm.
  const double k_{m_* 9.81 / 0.001};
  // Hunt-Crossley's dissipation factor.
  const double d_{0.0};  // [s/m]
};

}  // namespace geometry_world
}  // namespace examples
}  // namespace drake