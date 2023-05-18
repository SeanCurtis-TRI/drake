#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"
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

// Tests two properties regarding OpenGlContexts and threads:
// - A context bound in one thread cannot be bound in another (see `source`
//   below).
// - Cloned contexts created in one thread can be bound in other threads, even
//   even if the original context is bound in different thread.
GTEST_TEST(OpenGlContextTest, ThreadSafety) {
  std::vector<OpenGlContext> contexts;
  contexts.reserve(3);
  contexts.emplace_back(false);
  const OpenGlContext& source = contexts.back();
  // This is calling the *copy constructor* of OpenGlContext to emplace back.
  contexts.emplace_back(source);
  contexts.emplace_back(source);

  // We can bind this in the main thread, no problem.
  ASSERT_NO_THROW(source.MakeCurrent());
  ASSERT_TRUE(source.IsCurrent());

  std::atomic<int> num_errors{0};
  auto work = [&num_errors, &contexts](int i) {
    try {
      const OpenGlContext& context = contexts.at(i);
      context.MakeCurrent();
      if (i == 0) {
        // Successfully binding contexts[0] (aka source) *should've" been an
        // error.
        throw std::runtime_error("Worker 0 should not be able to bind the "
                                 "source context in a different thread.");
      }
      if (!context.IsCurrent()) {
        // MakeCurrent() didn't really work; also an error.
        throw std::runtime_error("OpenGlContext is not the current thread.");
      }
    } catch (std::exception& e) {
      // The single exception we expect (0 can't make context current) is 
      // ignored. Everything else is an error.
      using std::string_view;
      if (i != 0 ||
          e.what() != string_view("Error making an OpenGL context current")) {
        log()->error("Worker {} exception: {}", i, e.what());
        ++num_errors;
      }
    }
  };

  std::vector<std::thread> threads;
  threads.push_back(std::thread(work, 0)); // Erroneously rebind `source`.
  threads.push_back(std::thread(work, 1));
  threads.push_back(std::thread(work, 2));

  for (auto& thread : threads) {
    thread.join();
  }

  ASSERT_EQ(num_errors, 0);
}

// TODO: Move this into opengl test.
// A test on the semantics of OpenGL objects:
// - Independently created contexts don't share objects.
// - A cloned context shares objects with the source.
// - Destruction of a context which shares OpenGL objects does not destroy the
//   objects.
GTEST_TEST(OpenGlContextTest, OpenGlObjectSharing) {
  const OpenGlContext source;
  source.MakeCurrent();

  // Create a buffer in source's context.
  GLuint buffer;
  glCreateFramebuffers(1, &buffer);
  ASSERT_TRUE(glIsFramebuffer(buffer));
  // Reject the null hypothesis -- bad handle is not a buffer.
  ASSERT_FALSE(glIsFramebuffer(buffer + 1));

  {
    const OpenGlContext clone(source);
    clone.MakeCurrent();
    ASSERT_TRUE(clone.IsCurrent());
    // It's still a frame buffer with the clone the current context.
    ASSERT_TRUE(glIsFramebuffer(buffer));
    // We'll destroy this context as we leave. Making sure the context isn't
    // bound when the destructor is called will lead to its immediate
    // destruction.
    OpenGlContext::ClearCurrent();
  }

  // Even after the destruction of the sharing context, the buffer still exists
  // in the original.
  source.MakeCurrent();
  ASSERT_TRUE(glIsFramebuffer(buffer));

  // However, a newly created OpenGlContext does *not* have the frame buffer.
  const OpenGlContext independent;
  independent.MakeCurrent();
  ASSERT_FALSE(glIsFramebuffer(buffer));
}


// RenderEngineGl has to handle its OpenGlContext correctly to benefit from
// its thread-independence and OpenGL object sharing.
//
// This test confirms that clones of RenderEngineGl properly clone contexts. We
// assume that creating RenderEngineGl instances from scratch must create
// OpenGlContexts from scratch as well, so we don't explcitly test it.
//
// The independence is tested similarly to the OpenGlContext above; one instance
// is bound in the main thread. Its clones should be independently functional
// in its own threads.
//
// Furthermore, the images each produce should include the same textured box --
// proof that OpenGl objects (texture and mesh data) are shared across all
// engines.
GTEST_TEST(RenderEngineGlTest, ThreadSafety) {
  std::unique_ptr<RenderEngine> source_engine = MakeRenderEngineGl();

  // Add a single textured, object to the source engine; it should be renderable
  // by each cloned engine.
  PerceptionProperties material;
  material.AddProperty("label", "id", render::RenderLabel::kDontCare);
  material.AddProperty("phong", "diffuse_map", FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"));
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
      "color", CameraInfo(640, 480, 2.0), ClippingRange(0.25, 10.0), {}), true);

  // Render from the main thread with the source engine; this should bind its
  // OpenGl context to the main thread. If any of the clones shared contexts
  // with it, they should be unable to bind their contexts in *other* threads.
  ImageRgba8U source_image(camera.core().intrinsics().width(),
                           camera.core().intrinsics().height());
  ASSERT_NO_THROW(source_engine->RenderColorImage(camera, &source_image));
  sleep(3);

  // Simply look to see if we have an image with the material color in the
  // center. If present, the box should guarantee it.
  auto check_image =
      [w = camera.core().intrinsics().width(),
       h = camera.core().intrinsics().height()](const ImageRgba8U& image) {
        const int x = w / 2;
        const int y = h / 2;
        EXPECT_NEAR(image.at(x, y)[0], 4, 1);
        EXPECT_NEAR(image.at(x, y)[1], 241, 1);
        EXPECT_NEAR(image.at(x, y)[2], 33, 1);
      };

  check_image(source_image);

  // The multi-threaded work function; render and check the image.
  std::atomic<int>
      num_errors{0};
  auto work = [&num_errors, &renderers, &check_image, &camera](int i) {
    RenderEngine& renderer = *renderers.at(i);
    ImageRgba8U image(camera.core().intrinsics().width(),
                      camera.core().intrinsics().height());
    bool rendered = true;
    try {
      renderer.RenderColorImage(camera, &image);
      sleep(3);
    } catch (std::exception& e) {
      log()->error("Worker {} exception: {}", i, e.what());
      rendered = false;
    }
    if (rendered) {
      // No rendering error occurred; test the images.
      SCOPED_TRACE(fmt::format("Worker {}", i));
      check_image(image);
    } else {
      ++num_errors;
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
