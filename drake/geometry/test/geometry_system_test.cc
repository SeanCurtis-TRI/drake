#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/shapes.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace geometry {
namespace {

using systems::OutputPortDescriptor;
using GSystem = GeometrySystem<double>;
using std::make_unique;
using std::unique_ptr;

// Utility methods to create a sphere of the given radius.
std::unique_ptr <Shape> make_sphere(double radius) {
  return std::unique_ptr<Shape>(new Sphere(radius));
}

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
  // TODO: This isn't actually a *TEST*. It doesn't validate anything. I was
  // using this to exercise code.
  using std::to_string;
  GSystem system;
  SourceId s_id = system.AddSourceInput("first_source");
  std::unordered_set<FrameId> frames;
  std::unordered_set<GeometryId> geometries;
  const double kRadius = 0.25;
  for (int f = 0; f < 3; ++f) {
    FrameId f_id = system.RegisterFrame(
        s_id, GeometryFrame<double>("frame_" + to_string(f),
                                    Isometry3<double>::Identity()));
    frames.emplace(f_id);
    for (int g = 0; g < 3; ++g) {
      GeometryId g_id = system.RegisterGeometry(
          s_id, f_id,
          make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                                make_sphere(kRadius)));
      geometries.emplace(g_id);
    }
  }
  OutputPortDescriptor<double> descriptor(&system, 0, systems::kAbstractValued, 0);
  auto value = system.AllocateOutputAbstract(descriptor);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
