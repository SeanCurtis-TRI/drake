#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/rendering/frame_velocity.h"

namespace drake {
namespace systems {
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Subsume this functionality into
// GeometryWorld/GeometrySystem when they become available.

// TODO(david-german-tri): Consider renaming this to FrameKinematicsBundle,
// since it contains both poses and velocities.

/// PoseBundle is a container for a set of poses, represented by an Isometry3,
/// and corresponding velocities, represented by a FrameVelocity. The poses and
/// velocities are expressed in the world frame: X_WFi, V_WFi. Each pose has a
/// name and a model instance ID.  If two poses in the bundle have the same
/// model instance ID, they must not have the same name.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double and AutoDiffXd are supported.
template <typename T>
class PoseBundle {
 public:
  explicit PoseBundle(int num_poses);
  ~PoseBundle();

  int get_num_poses() const;
  const Isometry3<T>& get_pose(int index) const;
  void set_pose(int index, const Isometry3<T>& pose);

  const FrameVelocity<T>& get_velocity(int index) const;
  void set_velocity(int index, const FrameVelocity<T>& velocity);

  const std::string& get_name(int index) const;
  void set_name(int index, const std::string& name);

  int get_model_instance_id(int index) const;
  void set_model_instance_id(int index, int id);

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseBundle)

  template <typename T1 = T,
      typename std::enable_if<
          std::is_same<T1, double>::value, void*>::type *& = nullptr>
  std::unique_ptr<PoseBundle<AutoDiffXd>> ToAutoDiffXd() const {
    // TODO(sherm1): Consider changing this to overload a default implementation
    // in AbstractValue, as discussed in
    // https://github.com/RobotLocomotion/drake/issues/5454

    auto bundle = std::make_unique<PoseBundle<AutoDiffXd>>(get_num_poses());
    for (int pose_index = 0; pose_index < get_num_poses(); pose_index++) {
      Isometry3<AutoDiffXd> pose(get_pose(pose_index));
      bundle->set_pose(pose_index, pose);
      FrameVelocity<AutoDiffXd> velocity;
      velocity.set_velocity(multibody::SpatialVelocity<AutoDiffXd>(
          get_velocity(pose_index).get_velocity().get_coeffs()));
      bundle->set_velocity(pose_index, velocity);
      bundle->set_name(pose_index, get_name(pose_index));
      bundle->set_model_instance_id(pose_index,
                                    get_model_instance_id(pose_index));
    }
    return bundle;
  }

 private:
  std::vector<Isometry3<T>> poses_;
  std::vector<FrameVelocity<T>> velocities_;
  std::vector<std::string> names_;
  std::vector<int> ids_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
