#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
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
std::unique_ptr<Shape> make_sphere(double radius) {
  return std::unique_ptr<Shape>(new Sphere(radius));
}

std::unique_ptr<Shape> make_plane(const Vector3<double>& n, const Vector3<double>& p) {
  return std::unique_ptr<Shape>(new HalfSpace(n, p));
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

GTEST_TEST(MickeyMouse, LoadTest) {
  GSystem system;

  // Single frame with a single sphere
  SourceId s_id = system.AddSourceInput("first_source");
  const double kRadius = 0.25;
  FrameId f_id = system.RegisterFrame(
      s_id, GeometryFrame<double>("some_frame",
                                  Isometry3<double>::Identity()));
  GeometryId head_id = system.RegisterGeometry(
        s_id, f_id,
        make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                              make_sphere(kRadius)));
  auto offset = Vector3<double>(1, 0, 1).normalized() * (kRadius * 1.25);
  auto ear_pose = Isometry3<double>::Identity();
  ear_pose.translation() = offset;
  system.RegisterGeometry(
      s_id, head_id,
      make_unique<GeometryInstance<double>>(ear_pose, make_sphere(kRadius/2)));

  ear_pose.translation() << -offset(0), offset(1), offset(2);
  system.RegisterGeometry(
      s_id, head_id,
      make_unique<GeometryInstance<double>>(ear_pose, make_sphere(kRadius/2)));

  system.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance<double>>(
                Isometry3<double>::Identity(),
                make_plane(Vector3<double>(0, 0, 1),
                           Vector3<double>(0, 0, 0))));
  DispatchLoadMessage(system);
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
