#include "drake/examples/scene_graph/dev/bouncing_ball_plant.h"

#include <algorithm>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using std::make_unique;
using systems::BasicVector;
using systems::Context;
using systems::Value;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        SceneGraph<T>* scene_graph,
                                        int ball_count)
    : ball_count_(ball_count), source_id_(source_id) {
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(ball_count > 0);

  geometry_query_port_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName, systems::Value<geometry::QueryObject<T>>{})
           .get_index();
  const int num_states = ball_count * 2;
  state_port_ =
      this->DeclareVectorOutputPort(BasicVector<T>(num_states),
                                    &BouncingBallPlant::CopyStateToOutput)
          .get_index();

  this->DeclareContinuousState(BasicVector<T>(num_states),
                               ball_count /* num_q */, ball_count /* num_v */,
                               0 /* num_z */);

  // Lay the balls out in a dense grid so they stay close to the origin and
  // in camera. (We don't want rendering to get cheap simply because the
  // geometry lies outside the camera frustum.)
  const int side_length = static_cast<int>(std::sqrt(ball_count) + 0.5) + 1;
  const double stride = diameter_ * 1.1;
  const double origin_x = -(side_length - 1) * stride / 2;
  const double origin_y = origin_x;

  for (int i = 0; i < ball_count; ++i) {
    std::string name = "ball" + std::to_string(i);
    FrameId frame_id = scene_graph->RegisterFrame(
        source_id, GeometryFrame(name, Isometry3<double>::Identity()));
    GeometryId ball_id = scene_graph->RegisterGeometry(
        source_id, frame_id,
        make_unique<GeometryInstance>(Isometry3<double>::Identity(), /*X_FG*/
                                      make_unique<Sphere>(diameter_ / 2.0),
                                      "ball"));
    // Use the default material.
    scene_graph->AssignRole(source_id, ball_id, IllustrationProperties());
    scene_graph->AssignRole(source_id, ball_id, ProximityProperties());
    const int row = i / side_length;
    const int col = i % side_length;
    p_WB_.push_back(Vector2<double>{origin_x + row * stride,
                                    origin_y + col * stride});
    frame_ids_.push_back(frame_id);
    frame_indexes_.insert({frame_id, i});
  }

  // Allocate the output port now that the frame has been registered.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          FramePoseVector<double>(source_id_, frame_ids_),
          &BouncingBallPlant::CalcFramePoseOutput)
      .get_index();
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::InputPort<T>&
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
BouncingBallPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

// Updates the state output port.
template <typename T>
void BouncingBallPlant<T>::CopyStateToOutput(
    const Context<T>& context,
    BasicVector<T>* state_output_vector) const {
  state_output_vector->set_value(get_state(context).get_value());
}

template <typename T>
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(poses->source_id() == source_id_);
  DRAKE_DEMAND(poses->size() == ball_count_);

  poses->clear();

  for (size_t i = 0; i < frame_ids_.size(); ++i) {
    FrameId frame_id = frame_ids_[i];
    Isometry3<T> pose = Isometry3<T>::Identity();
    const BasicVector<T>& state = get_state(context);
    pose.translation() << p_WB_[i].x(), p_WB_[i].y(), state[i];
    poses->set_value(frame_id, pose);
  }
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BasicVector<T>& state = get_state(context);
  BasicVector<T>& derivative_vector = get_mutable_state(derivatives);

  const geometry::QueryObject<T>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<T>>();
  const auto& inspector = query_object.inspector();

  // x_dot = z_dot
  const int frame_count = static_cast<int>(frame_ids_.size());
  for (int i = 0; i < frame_count; ++i) {
    derivative_vector[i] = state[frame_count + i];
    // Without contact, every ball is accelerating _down_.
    derivative_vector[i + frame_count] = -g_;
  }

  std::vector<PenetrationAsPointPair<T>> penetrations =
      query_object.ComputePointPairPenetration();
  T fC = 0;  // the contact force
  if (penetrations.size() > 0) {
    for (const auto& penetration : penetrations) {
      FrameId frame_id = inspector.GetFrameId(penetration.id_A);
      if (frame_indexes_.count(frame_id) == 0) {
        // frame is world frame.
        frame_id = inspector.GetFrameId(penetration.id_B);
        DRAKE_DEMAND(frame_indexes_.count(frame_id) > 0);
      }
      const int frame_index = frame_indexes_.at(frame_id);
      const int frame_v_index = frame_index + frame_count;
      // Penetration depth, > 0 during penetration.
      const T& x = penetration.depth;
      // Penetration rate, > 0 implies increasing penetration.
      const T& xdot = -state[frame_v_index];

      fC = k_ * x * (1.0 + d_ * xdot);
      const T fN = max(0.0, fC);
      derivative_vector[frame_v_index] = (-m_ * g_ + fN) / m_;
    }
  }
}

template class BouncingBallPlant<double>;

}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
