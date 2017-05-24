#include "drake/geometry/geometry_system.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

using GSystem = GeometrySystem<double>;

// Tests the addition of input ports. Only valid source ids map to input ports
// and the input ports are ordered in the declaration order.
GTEST_TEST(GeometrySystemTest, TestInputPorts) {
  GSystem system;
  SourceId src1 = system.AddSourceInput("name1");
  SourceId src2 = system.AddSourceInput("name2");
  EXPECT_NO_THROW(system.get_port_for_source_id(src1));
  EXPECT_EQ(system.get_port_for_source_id(src1).get_index(), 0);
  EXPECT_EQ(system.get_port_for_source_id(src2).get_index(), 1);
  EXPECT_THROW(system.get_port_for_source_id(SourceId::get_new_id()),
               std::logic_error);
}

GTEST_TEST(GeometrySystemTest, Construct) {
  GSystem system;
}
}  // namespace
}  // namespace geometry
}  // namespace drake
