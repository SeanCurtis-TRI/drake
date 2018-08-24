#include "drake/geometry/render/render_engine_vtk.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/image.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;

const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = true;

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
const double kColorPixelTolerance = 1.001;
const double kDepthTolerance = 1.1e-4;

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, kWidth) and [0, kHeight) respectively.
struct ScreenCoord {
  int x{};
  int y{};
};

void CompareColor(const uint8_t* pixel,
                  const ColorI& color,
                  int) {
  ASSERT_NEAR(pixel[0], color.r, kColorPixelTolerance);
  ASSERT_NEAR(pixel[1], color.g, kColorPixelTolerance);
  ASSERT_NEAR(pixel[2], color.b, kColorPixelTolerance);
  // TODO(SeanCurtis-TRI): Kill alpha from the argument list as well.
//  ASSERT_EQ(pixel[3], alpha);
}

// This suite tests RenderEngineVtk.
class RgbdRendererTest : public ::testing::Test {
 public:
  RgbdRendererTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
      // Looking straight down from 3m above the ground.
        X_WR_(Eigen::Translation3d(0, 0, kDefaultDistance) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())) {}

 protected:
  void Render(RenderEngineVtk* renderer = nullptr) {
    if (!renderer) renderer = renderer_.get();
    renderer->RenderColorImage(camera_, &color_, kShowWindow);
    renderer->RenderDepthImage(camera_, &depth_, kShowWindow);
    renderer->RenderLabelImage(camera_, &label_, kShowWindow);
  }

  void VerifyUniformColor(const ColorI& pixel, int alpha) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        CompareColor(color_.at(x, y), pixel, alpha);
      }
    }
  }

  void VerifyUniformLabel(int16_t label) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_EQ(label_.at(x, y)[0], label);
      }
    }
  }

  void VerifyUniformDepth(float depth) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_NEAR(depth_.at(x, y)[0], depth, kDepthTolerance);
      }
    }
  }

  // Verifies the chosen pixels, i.e. `kOutliers`, belong to the terrain.
  void VerifyOutliers() {
    const auto& kTerrain = renderer_->get_flat_terrain_color();
    for (const auto& screen_coord : kOutliers) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      // Color
      CompareColor(color_.at(x, y), kTerrain, 255u);
      // Depth
      ASSERT_NEAR(depth_.at(x, y)[0], kDefaultDistance, kDepthTolerance);
      // Label
      ASSERT_EQ(label_.at(x, y)[0], RenderLabel::terrain_label());
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const Eigen::Isometry3d& X_WR, bool add_terrain = false) {
    renderer_ = std::make_unique<RenderEngineVtk>();
    renderer_->UpdateViewpoint(X_WR);

    if (add_terrain) renderer_->AddFlatTerrain();
  }

  // Populates the given renderer with the sphere required for
  // PerformSphereTest().
  void PopulateSphereTest(RenderEngineVtk* renderer) {
    Sphere sphere{0.5};
    render_label_ = RenderLabel::new_label();
    RenderMaterial material(render_label_);
    RenderIndex geometry_index = renderer->RegisterVisual(sphere, material);
    Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
    renderer->UpdateVisualPose(X_WV, geometry_index);
  }

  // Perform the sphere test on the given renderer. To pass, the renderer will
  // have to have had PopulateSphereTest() invoked on it already.
  void PerformSphereTest(RenderEngineVtk* renderer) {
    Render(renderer);

    VerifyOutliers();

    // Verifies inside the sphere.
    const int x = kInlier.x;
    const int y = kInlier.y;
    // Color
    CompareColor(color_.at(x, y), kDefaultVisualColor, 255u);
    // Depth
    ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
    // Label
    ASSERT_EQ(label_.at(x, y)[0], static_cast<int>(render_label_));
  }

  // This is the integer version of the GeometryMaterial default diffuse color.
  const ColorI kDefaultVisualColor = {229u, 229u, 229u};
  const float kDefaultDistance{3.f};
  // `kInlier` is chosen to point to a pixel representing an object in the
  // test scene.
  const ScreenCoord kInlier = {320, 240};
  // The outliers are chosen to point to pixels representing the ground,
  // not objects in the test scene.
  const std::array<ScreenCoord, 4> kOutliers{
      {{10, 10}, {10, 470}, {630, 10}, {630, 470}}};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY, kZNear, kZFar};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  RenderLabel render_label_;
  Isometry3d X_WR_;

  std::unique_ptr<RenderEngineVtk> renderer_;
};

TEST_F(RgbdRendererTest, NoBodyTest) {
  SetUp(Isometry3d::Identity());
  Render();

  VerifyUniformColor(renderer_->get_sky_color(), 0u);
  VerifyUniformLabel(RenderLabel::empty_label());
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      EXPECT_TRUE(std::isnan(depth_.at(x, y)[0]));
    }
  }
}

TEST_F(RgbdRendererTest, TerrainTest) {
  SetUp(X_WR_, true);

  const auto& kTerrain = renderer_->get_flat_terrain_color();
  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 5.f}})) {
    X_WR_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    VerifyUniformColor(kTerrain, 255u);
    VerifyUniformLabel(RenderLabel::terrain_label());
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WR_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(RenderLabel::terrain_label());
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  X_WR_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(RenderLabel::terrain_label());
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      ASSERT_EQ(InvalidDepth::kTooFar, depth_.at(x, y)[0]);
    }
  }
}

TEST_F(RgbdRendererTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  Isometry3d X_WR = Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  SetUp(X_WR, true);

  // Returns y in [0, kHeight / 2], index of horizon location in image
  // coordinate system under two assumptions: 1) the ground plane is not clipped
  // by `kClippingPlaneFar`, 2) camera is located above the ground.
  auto CalcHorizon = [](double z) {
    const double kTerrainSize = 50.;
    const double kFocalLength = kHeight * 0.5 / std::tan(0.5 * kFovY);
    return 0.5 * kHeight + z / kTerrainSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  const std::array<double, 3> Zs{{2., 1., 0.5}};
  for (const auto& z : Zs) {
    X_WR.translation().z() = z;
    renderer_->UpdateViewpoint(X_WR);
    Render();

    const auto& kTerrain = renderer_->get_flat_terrain_color();
    int actual_horizon{0};
    for (int y = 0; y < kHeight; ++y) {
      // Looking for the boundary between the sky and the ground.
      if ((static_cast<uint8_t>(kTerrain.r == color_.at(0, y)[0])) &&
          (static_cast<uint8_t>(kTerrain.g == color_.at(0, y)[1])) &&
          (static_cast<uint8_t>(kTerrain.b == color_.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

TEST_F(RgbdRendererTest, SphereTest) {
  SetUp(X_WR_, true);

  PopulateSphereTest(renderer_.get());
  PerformSphereTest(renderer_.get());
}

TEST_F(RgbdRendererTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  RenderLabel render_label = RenderLabel::new_label();
  RenderMaterial material(render_label);
  RenderIndex geometry_index = renderer_->RegisterVisual(cylinder, material);
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.6)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 1.8f, kDepthTolerance);
  // Label

  ASSERT_EQ(label_.at(x, y)[0], render_label);
}

TEST_F(RgbdRendererTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  RenderLabel render_label = RenderLabel::new_label();
  RenderMaterial material(render_label);
  RenderIndex geometry_index = renderer_->RegisterVisual(box, material);
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);
  Render();

  VerifyOutliers();

  // Verifies inside the box.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], render_label);
}

TEST_F(RgbdRendererTest, MeshTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  RenderLabel render_label = RenderLabel::new_label();
  RenderMaterial material(render_label);
  RenderIndex geometry_index = renderer_->RegisterVisual(mesh, material);
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  // TODO(SeanCurtis-TRI): When the texture is supported, change it to be the
  // texture color.
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 2., kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], render_label);
}

// All of the clone tests use the PerformSphereTest to confirm that the clone
// is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test.
TEST_F(RgbdRendererTest, SimpleClone) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  auto clone = renderer_->Clone();
  PerformSphereTest(clone.get());
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RgbdRendererTest, ClonePersistence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  auto clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed and replaced.
  renderer_ = std::move(clone);
  PerformSphereTest(renderer_.get());
}

// Tests that the cloned renderer still works, even when the original has values
// changed.
TEST_F(RgbdRendererTest, CloneIndependence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  auto clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  Isometry3d X_WT_new{Translation3d{0, 0, 10}};
  // This assumes that the terrain is zero-indexed.
  renderer_->UpdateVisualPose(X_WT_new, RenderIndex(0));
  PerformSphereTest(clone.get());
}

// TODO(kunimatsu-tri) Move DepthImageToPointCloudConversionTest here from
// rgbd_camera_test.

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
