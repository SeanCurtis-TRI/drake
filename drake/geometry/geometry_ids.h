#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace geometry {

/// Type used to identify geometry channels by index in GeometryWorld.
using ChannelId = Identifier<class ChannelTag>;

/// Type used to identify a frame kinematics instance by index in
/// GeometryWorld
using FrameId = Identifier<class KinematicsTag>;

/// Type used to identify a geometry instance by index in GeometryWorld.
using GeometryId = Identifier<class GeometryTag>;

}  // namespace geometry
}  // namespace drake
