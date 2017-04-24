#pragma once

#include "drake/geometry/frame_kinematics_set.h"

namespace drake {
namespace geometry {
/** @cond */

// This serves as a _mock_ GeometryWorld. GeometryWorld serves as a factory of
// the FrameKinematicsSet class. It is the _only_ class that can generate them.
// The GeometryState class is responsible for validating the state of a
// FrameKinematicsSet. To test that function, I need access to an instance of
// the FrameKinematicsSet. This mock allows me to create such an instance
// in a very lightweight manner by exploiting a declared friend relationship.
template <typename T>
class GeometryWorld {
 public:
  static FrameKinematicsSet<T> MakeFKS(SourceId s_id) {
    return FrameKinematicsSet<T>(s_id);
  }
};

/** @endcode */
}  // namespace geometry
}  // namespace drake
