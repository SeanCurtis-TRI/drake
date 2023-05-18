#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ClippingRange;
using render::ColorRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using render_gl::internal::OpenGlContext;
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;

#if 0
// This doesn't currently work because the opengl context can't be copied!
// The solution is to *clone* OpenGlContexts via public API and use that in the
// render engine.
GTEST_TEST(OpenGlContextTest, ThreadSafe) {
  const OpenGlContext source;
  std::vector<OpenGlContext> contexts;
  contexts.push_back(source);
  contexts.push_back(source);

  std::atomic<int> num_errors{0};
  auto work = [&num_errors, &contexts](int i) {
    try {
      contexts.at(i).MakeCurrent();
    } catch (std::exception& e) {
      log()->error("Worker {} exception: {}", i, e.what());
      ++num_errors;
    }
  };

  std::vector<std::thread> threads;
  threads.push_back(std::thread(work, 0));
  threads.push_back(std::thread(work, 1));

  for (auto& thread : threads) {
    thread.join();
  }

  ASSERT_EQ(num_errors, 0);
}
#endif

// This confirms that when a RenderEngine gets cloned, the clones have OpenGl
// contexts that are sufficiently independent to allow rendering in independent
// threads, but still share the same OpenGl artifacts on the GPU.
//
// The ability to render images without failure indicates the independence of
// the contexts. The fact that each rendered image has red in the middle is
// proof that the cloned contexts have access to the registered geometry.
GTEST_TEST(RenderEngineGlTest, ClonesAreThreadsafe) {
  std::unique_ptr<RenderEngine> source_engine = MakeRenderEngineGl();

  // Add a single object to the source engine; it should be renderable by each
  // cloned engine.
  PerceptionProperties material;
  material.AddProperty("label", "id", render::RenderLabel::kDontCare);
  material.AddProperty("phong", "diffuse", Rgba(1, 0, 0));
  source_engine->RegisterVisual(GeometryId::get_new_id(), Box(1, 1, 1),
                                material, math::RigidTransformd(),
                                false /* needs update */);

  // Pose of camera body in the world: Looking straight down from 3m above the
  // ground.
  const RigidTransformd X_WC(
      RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                      AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
      {0, 0, 3.0});
  source_engine->UpdateViewpoint(X_WC);

  // Create the clones.
  std::vector<std::unique_ptr<RenderEngine>> renderers;
  for (int i = 0; i < 2; ++i) {
    renderers.push_back(source_engine->Clone());
    renderers.back()->UpdateViewpoint(X_WC);
  }

  // Create some renderings.
  const ColorRenderCamera camera(RenderCameraCore(
      "color", CameraInfo(640, 480, 2.0), ClippingRange(0.25, 10.0), {}));

  // Render from the main thread with the source engine; this should bind its
  // OpenGl context to the main thread. If any of the clones shared contexts
  // with it, they should be unable to bind their contexts in *other* threads.
  ImageRgba8U source_image(camera.core().intrinsics().width(),
                           camera.core().intrinsics().height());
  ASSERT_NO_THROW(source_engine->RenderColorImage(camera, &source_image));

  // Simply look to see if we have an image with the material color in the
  // center. If present, the box should guarantee it.
  auto check_image =
      [w = camera.core().intrinsics().width(),
       h = camera.core().intrinsics().height()](const ImageRgba8U& image) {
        const int x = w / 2;
        const int y = h / 2;
        EXPECT_NEAR(image.at(x, y)[0], 255, 1);
        EXPECT_NEAR(image.at(x, y)[1], 0, 1);
        EXPECT_NEAR(image.at(x, y)[2], 0, 1);
      };

  check_image(source_image);

  // The multi-threaded work function; render and check the image.
  std::atomic<int>
      num_errors{0};
  auto work = [&num_errors, &renderers, &check_image, &camera](int i) {
    RenderEngine& renderer = *renderers.at(i);
    ImageRgba8U image(camera.core().intrinsics().width(),
                      camera.core().intrinsics().height());
    const int old_errors = num_errors;
    try {
      renderer.RenderColorImage(camera, &image);
    } catch (std::exception& e) {
      log()->error("Worker {} exception: {}", i, e.what());
      ++num_errors;
    }
    if (old_errors == num_errors) {
      // No rendering error occurred; test the images.
      SCOPED_TRACE(fmt::format("Worker {}", i));
      check_image(image);
    }
  };

  std::vector<std::thread> threads;
  for (int i = 0; i < 2; ++i) {
    threads.push_back(std::thread(work, i));
  }

  for (auto& thread : threads) {
    thread.join();
  }

  ASSERT_EQ(num_errors, 0);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
