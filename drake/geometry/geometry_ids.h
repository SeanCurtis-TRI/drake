#pragma once

#include "drake/geometry/type_safe_int_id.h"

namespace drake {
namespace geometry {

/// Type used to identify geometry channels by index in GeometryWorld.
using ChannelId = TypeSafeIntId<class ChannelTag>;

/// Type used to identify a frame kinematics instance by index in
/// GeometryWorld
using FrameId = TypeSafeIntId<class KinematicsTag>;

/// Type used to identify a geometry instance by index in GeometryWorld.
using GeometryId = TypeSafeIntId<class GeometryTag>;

}  // namespace geometry
}  // namespace drake
