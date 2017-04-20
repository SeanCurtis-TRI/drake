#include "drake/geometry/geometry_instance.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace geometry {
namespace {

using GeomInstance = GeometryInstance<double>;
using Pose = Isometry3<double>;

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  EXPECT_TRUE(is_copyable_unique_ptr_compatible<GeomInstance>::value);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
