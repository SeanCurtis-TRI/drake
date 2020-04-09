#include "drake/geometry/render_query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/geometry/render/camera_properties.h"

namespace drake {
namespace geometry {

using math::RigidTransformd;
using render::DepthCameraProperties;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;

// Friend class to RenderQueryObjectTest -- left in `drake::geometry` to match
// the friend declaration. The name gives it access to the private/protected
// access of RenderQueryObject's *parent* class, QueryObject.
class QueryObjectTest : public ::testing::Test {
 protected:
  template <typename T>
  static ::testing::AssertionResult is_default(
      const RenderQueryObject<T>& object) {
    if (object.scene_graph_ != nullptr || object.context_ != nullptr ||
        object.state_ != nullptr) {
      return ::testing::AssertionFailure()
          << "A default query object should have all null fields. Has "
             "scene_graph: "
          << object.scene_graph_ << ", context: " << object.context_
          << ", state: " << object.state_.get();
    }
    return ::testing::AssertionSuccess();
  }

  template <typename T>
  static void ThrowIfNotCallable(const QueryObject<T>& query_object) {
    query_object.ThrowIfNotCallable();
  }

  // TODO(SeanCurtis-TRI): In addition to confirming that default throws, it
  // would probably be good to confirm that the *right* method is getting call
  // when the query object is well configured.
};

// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (SceneGraph) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  RenderQueryObject<double> default_object;

  EXPECT_TRUE(is_default(default_object));

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(           \
      expression, std::runtime_error,    \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(ThrowIfNotCallable(default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.

  // Rendering API throws if default.
  const RigidTransformd X_PC;
  const FrameId parent_frame = FrameId::get_new_id();
  DepthCameraProperties camera{2, 2, 1.5, "renderer", 0.1, 10.0};
  ImageRgba8U color_image(2, 2);
  ImageDepth32F depth_image(2, 2);
  ImageLabel16I label_image(2, 2);
  EXPECT_DEFAULT_ERROR(default_object.RenderColorImage(
      camera, parent_frame, X_PC, false, &color_image));
  EXPECT_DEFAULT_ERROR(default_object.RenderDepthImage(camera, parent_frame,
                                                       X_PC, &depth_image));
  EXPECT_DEFAULT_ERROR(default_object.RenderLabelImage(
      camera, parent_frame, X_PC, false, &label_image));
}
#undef EXPECT_DEFAULT_ERROR

}  // namespace geometry
}  // namespace drake
