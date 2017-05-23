#include "drake/geometry/geometry_system.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

using GSystem = GeometrySystem<double>;

GTEST_TEST(GeometrySystemTest, Construct) {
  GSystem system;
}
}  // namespace
}  // namespace geometry
}  // namespace drake
