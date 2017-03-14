#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/// Type used to identify geometry bundles by index in GeometryWorld.
using BundleIndex = TypeSafeIndex<class BundleTag>;

/// Type used to identify a geometry kinematics instance by index in
/// GeometryWorld
using KinematicsIndex = TypeSafeIndex<class KinematicsTag>;

}  // namespace geometry
}  // namespace drake
