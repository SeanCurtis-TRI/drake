#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace {
enum AsyncMode {
  /* No asynchrony; run serially. */
  kNone,
  /* Use std::async(policy=std::launch::async). */
  kTask,
  /* Use std::thread. */
  kThread,
};
std::string_view to_string(AsyncMode mode) {
  switch (mode) {
    case kNone:
      return "kNone";
    case kTask:
      return "kTask";
    case kThread:
      return "kThread";
  }
  DRAKE_UNREACHABLE();
}
}  // namespace
DRAKE_FORMATTER_AS(, , AsyncMode, x, to_string(x))

namespace fs = std::filesystem;

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
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;

struct Params {
  AsyncMode async_mode{kNone};
  int num_workers{1};
  int num_repeats{1};

  friend std::ostream& operator<<(std::ostream& os, const Params& params) {
    os << "(" << params.async_mode << ", " << params.num_workers << ", "
       << params.num_repeats << ")";
    return os;
  }
};

class ThreadTest : public testing::TestWithParam<Params> {};

TEST_P(ThreadTest, Run) {
  const auto& [async_mode, num_workers, num_repeats] = GetParam();
  DRAKE_DEMAND(num_workers > 0);
  DRAKE_DEMAND(num_repeats > 0);

  // Looking straight down from 3m above the ground.
  const RigidTransformd X_WC(
      RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                      AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
      {0, 0, 3.0});
  const ColorRenderCamera camera_params(RenderCameraCore(
      "color", CameraInfo(640, 480, 2.0), ClippingRange(0.25, 10.0), X_WC));

  // Prepare the cameras, one per thread.
  std::vector<std::unique_ptr<RenderEngine>> renderers;
  for (int i = 0; i < num_workers; ++i) {
    renderers.push_back(MakeRenderEngineGl());
  }

  // Add model(s).
  PerceptionProperties material;
  material.AddProperty("label", "id", render::RenderLabel::kDontCare);
  const GeometryInstance instance(
      RigidTransformd(),
      Mesh(FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj"), 1),
      "box");
  for (auto& engine : renderers) {
    if (engine == nullptr) continue;
    engine->UpdateViewpoint(X_WC);
    engine->RegisterVisual(instance.id(), instance.shape(), material,
                           instance.pose(), false /* needs update*/);
  }

  // The worker functor for the i'th camera.
  MatrixX<ImageRgba8U> images(num_workers, num_repeats);
  std::atomic<int> num_errors{0};
  auto work = [&images, &num_errors, num_repeats = num_repeats, &renderers,
               &camera_params](int i) {
    RenderEngine* renderer = renderers.at(i).get();
    for (int j = 0; j < num_repeats; ++j) {
      ImageRgba8U& image = images(i, j);
      image = ImageRgba8U(camera_params.core().intrinsics().width(),
                          camera_params.core().intrinsics().height());
      try {
        renderer->RenderColorImage(camera_params, &image);
      } catch (std::exception& e) {
        drake::log()->error("Worker {} exception: {}", i, e.what());
        ++num_errors;
      }
    }
  };
  // Render on multiple threads concurrently.
  switch (async_mode) {
    case kNone:
      drake::log()->info("Runnning {} workers serially", num_workers);
      break;
    case kTask:
      drake::log()->info("Launching {} std::async workers", num_workers);
      break;
    case kThread:
      drake::log()->info("Launching {} std::thread workers", num_workers);
      break;
  }
  std::vector<std::future<void>> futures;
  std::vector<std::thread> threads;
  for (int i = num_workers - 1; i >= 0; --i) {
    switch (async_mode) {
      case kNone:
        work(i);
        break;
      case kTask:
        futures.push_back(std::async(std::launch::async, work, i));
        break;
      case kThread:
        threads.push_back(std::thread(work, i));
        break;
    }
  }

  log()->info("Starting parallel work!");
  // Wait for all workers to finish.
  for (auto& thread : threads) {
    thread.join();
  }
  for (auto& future : futures) {
    future.get();
  }

  // Save the first image for offline inspection.
  // See bazel-testlogs/geometry/render_gl/thread_test/test.outputs/output.zip.
  const std::string filename = fmt::format("image-{}.png", async_mode);
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    const fs::path img_path = fs::path(dir) / filename;
    log()->info("Writing {}", img_path);
    SaveToPng(images(0, 0), img_path);
  }

  // Check for errors.
  EXPECT_EQ(num_errors, 0);

  // Check that all output images are identical.
      const ImageRgba8U& image_00 = images(0, 0);
  for (int i = 0; i < num_workers; ++i) {
    SCOPED_TRACE(i);
    for (int j = 0; j < num_repeats; ++j) {
      SCOPED_TRACE(j);
      const ImageRgba8U& image_ij = images(i, j);
      ASSERT_EQ(image_ij, image_00);
    }
  }
}

constexpr Params kParams[] = {
    // {.async_mode = kNone, .num_workers = 1, .num_repeats = 2},
    {.async_mode = kTask, .num_workers = 3, .num_repeats = 2},
    // {.async_mode = kThread, .num_workers = 4, .num_repeats = 2},
};

INSTANTIATE_TEST_SUITE_P(All, ThreadTest, testing::ValuesIn(kParams));

}  // namespace
}  // namespace geometry
}  // namespace drake
