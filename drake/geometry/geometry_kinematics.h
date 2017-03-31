#pragma once

#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/**
 This class serves as the basis for connecting frames, moved according to
 arbitrary mechanisms, with geometry used for collision and visualization that
 can be queried by any system in the diagram.

 For geometry queries, we only need the kinematics of the frame: position,
 velocity, and acceleration in the world frame.
 */
class GeometryKinematics {
 public:
 private:
  // TODO(SeanCurtis-TRI): How to represent pose, pose', and pose''?

  // TODO(SeanCurtis-TRI): Guarantee uniqueness.
  // The name of the frame; must be *globally* unique.
  std::string name_;

  // TODO(SeanCurtis-TRI): Should this be a void pointer, or should I do some
  // CRTP so that I can stash typed artifacts and get them back out typed?
  // This arbitrary user data allows drivers of the frame to associate
  // information with results from the result of geometric queries.
  void* user_data_;
};
}  // namespace geometry
}  // namespace drake
