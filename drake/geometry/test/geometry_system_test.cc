#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {
namespace {

using GSystem = GeometrySystem<double>;
using std::make_unique;
using std::unique_ptr;


GTEST_TEST(GeometrySystemTest, Constructor) {
  GeometrySystem<double> double_system;
//  GeometrySystem<Expression> expression_system;
}
#if 0
// Utility methods to create a sphere of the given radius.
std::unique_ptr<Shape> make_sphere(double radius) {
  return std::unique_ptr<Shape>(new Sphere(radius));
}

std::unique_ptr<Shape> make_plane(const Vector3<double>& n,
                                  const Vector3<double>& p) {
  return std::unique_ptr<Shape>(new HalfSpace(n, p));
}
// Tests the addition of input ports. Only valid source ids map to input ports
// and the input ports are ordered in the declaration order.
GTEST_TEST(GeometrySystemTest, TestInputPorts) {
  GSystem system;
  SourceId src1 = system.RegisterSource("name1");
  SourceId src2 = system.RegisterSource("name2");
  EXPECT_NO_THROW(system.get_source_frame_id_port(src1));
  EXPECT_EQ(system.get_source_frame_id_port(src1).get_index(), 0);
  EXPECT_EQ(system.get_source_frame_id_port(src2).get_index(), 1);
  EXPECT_THROW(system.get_source_frame_id_port(SourceId::get_new_id()),
               std::logic_error);
}

// This is *not* a real test. This is basically me quickly exercising the
// underlying code.
GTEST_TEST(MickeyMouse, LoadTest) {
  GSystem system;

  // Single frame with a single sphere
  SourceId s_id = system.RegisterSource("first_source");
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
#endif
}  // namespace
}  // namespace geometry
}  // namespace drake
