#include "drake/geometry/dev/render/gl/render_engine_gl.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using std::make_unique;
using std::unique_ptr;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::InvalidDepth;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

// NOTE: The depth tolerance is this large mostly due to the combination of
// several factors:
//   - the sphere test (sphere against terrain)
//   - The even-valued window dimensions
//   - the tests against various camera properties
// The explanation is as follows. The distance to the sphere is only exactly
// 2 at the point of the sphere directly underneath the camera (the sphere's
// "peak"). However, with an even-valued window dimension, we never really
// sample that point. We sample the center of pixels all evenly arrayed around
// that point. So, that introduces some error. As the image gets *smaller* the
// pixels get bigger and so the distance away from the peak center increases,
// which, in turn, increase the measured distance for the fragment. This
// tolerance accounts for the test case where one image has pixels that are *4X*
// larger (in area) than the default image size.
const double kDepthTolerance = 2.5e-4;

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, image_width) and [0, image_height) respectively.
struct ScreenCoord {
  ScreenCoord(int x_in, int y_in) : x(x_in), y(y_in) {}
  int x{};
  int y{};
};

std::ostream& operator<<(std::ostream& out, const ScreenCoord& c) {
  out << "(" << c.x << ", " << c.y << ")";
  return out;
}

class RenderEngineGlTest : public ::testing::Test {
 public:
  RenderEngineGlTest()
      : depth_(kWidth, kHeight),
      // Looking straight down from 3m above the ground.
        X_WR_(Eigen::Translation3d(0, 0, kDefaultDistance) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())) {}

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineGl* renderer = nullptr,
              const DepthCameraProperties* camera_in = nullptr,
              ImageDepth32F* depth_in = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageDepth32F* depth = depth_in ? depth_in : &depth_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(camera, depth));
  }

  // Confirms that all pixels in the member depth image have the same value.
  void VerifyUniformDepth(float depth) {
    if (depth == std::numeric_limits<float>::infinity()) {
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
          ASSERT_EQ(depth_.at(x, y)[0], depth);
        }
      }
    } else {
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
          ASSERT_NEAR(depth_.at(x, y)[0], depth, kDepthTolerance);
        }
      }
    }
  }

  // Compute the set of outliers for a given set of camera properties.
  static std::vector<ScreenCoord> GetOutliers(const CameraProperties& camera) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, camera.height - kInset - 1},
        {camera.width - kInset - 1, camera.height - kInset - 1},
        {camera.width - kInset - 1, kInset}};
  }

  // Compute the inlier for the given set of camera properties.
  static ScreenCoord GetInlier(const CameraProperties& camera) {
    return ScreenCoord(camera.width / 2, camera.height / 2);
  }

  // Tests that the depth value in the given `image` at the given `coord` is
  // the expected depth to within a tolerance. Handles the special case where
  // the expected distance is infinity.
  static ::testing::AssertionResult IsExpectedDepth(const ImageDepth32F& image,
                                                    const ScreenCoord& coord,
                                                    float expected_depth,
                                                    float tolerance) {
    const float actual_depth = image.at(coord.x, coord.y)[0];
    if (expected_depth == std::numeric_limits<float>::infinity()) {
      if (actual_depth == expected_depth) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
            << "Expected depth at " << coord << " to be "
            << "infinity. Found: " << actual_depth;
      }
    } else {
      float delta = std::abs(expected_depth - actual_depth);
      if (delta <= tolerance) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
            << "Expected depth at " << coord << " to be "
            << expected_depth << ". Found " << actual_depth << ". Difference "
            << delta << "is greater than tolerance " << tolerance;
      }
    }
  }

  // Verifies the "outlier" pixels for the given camera belong to the terrain.
  // If images are provided, the given images will be tested, otherwise the
  // member images will be tested.
  void VerifyOutliers(const RenderEngineGl& renderer,
                      const DepthCameraProperties& camera,
                      const char* name,
                      ImageDepth32F* depth_in = nullptr) {
    ImageDepth32F& depth = depth_in ? *depth_in : depth_;

    for (const auto& screen_coord : GetOutliers(camera)) {
      // Depth
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))<< name;
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const Eigen::Isometry3d& X_WR, bool add_terrain = false) {
    renderer_ = make_unique<RenderEngineGl>();
    renderer_->UpdateViewpoint(X_WR);

    if (add_terrain) renderer_->AddFlatTerrain();
  }

  // Creates a simple perception properties set for fixed, known results.
  PerceptionProperties simple_material() const {
    PerceptionProperties material;
    material.AddGroup("phong");
    Vector4d color(kDefaultVisualColor.r / 255., kDefaultVisualColor.g / 255.,
                   kDefaultVisualColor.b / 255., 1.);
    material.AddProperty("phong", "diffuse", color);
    material.AddGroup("label");
    // NOTE: Any render label is sufficient; we aren't  testing them for this
    // depth-only renderer.
    material.AddProperty("label", "id", RenderLabel::empty_label());
    return material;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineGl* renderer) {
    Sphere sphere{0.5};
    RenderIndex geometry_index = renderer->RegisterVisual(
        sphere, PerceptionProperties(), Isometry3d::Identity());
    Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
    renderer->UpdateVisualPose(X_WV, geometry_index);
  }

  // Performs the work to test the rendering with a sphere centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant sphere and camera configuration (e.g., PopulateSphereTest()).
  // If force_hidden is true, then the render windows will be suppressed
  // regardless of any other settings.
  void PerformCenterShapeTest(RenderEngineGl* renderer,
                              const char* name,
                              const DepthCameraProperties* camera = nullptr) {
    const DepthCameraProperties& cam = camera ? *camera : camera_;
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageDepth32F depth(cam.width, cam.height);
    Render(renderer, &cam, &depth);

    VerifyOutliers(*renderer, cam, name, &depth);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(cam);
    EXPECT_TRUE(IsExpectedDepth(depth, inlier, expected_object_depth_,
                                kDepthTolerance)) << name;
  }

  // Provide a default visual color for this tests -- it is intended to be
  // different from the default color of the VTK render engine.
  const ColorI kDefaultVisualColor = {229u, 229u, 229u};
  const float kDefaultDistance{3.f};

  // Values to be used with the "centered shape" tests.
  // The amount inset from the edge of the images to *still* expect terrain
  // values.
  static constexpr int kInset{10};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY, Fidelity::kLow,
                                         kZNear, kZFar};
  ImageDepth32F depth_;
  Isometry3d X_WR_;
  unique_ptr<RenderEngineGl> renderer_;
};

// Tests an empty image -- confirms that it clears to the "empty" color -- no
// use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, NoBodyTest) {
  SetUp(Isometry3d::Identity());
  Render();

  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, TerrainTest) {
  SetUp(X_WR_, true);

  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WR_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WR_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  X_WR_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      ASSERT_EQ(InvalidDepth::kTooFar, depth_.at(x, y)[0]);
    }
  }
}

// Performs the shape centered in the image with a box.
TEST_F(RenderEngineGlTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  RenderIndex geometry_index =
      renderer_->RegisterVisual(box, simple_material(), Isometry3d::Identity());
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Box test");
}

// Performs the shape centered in the image with a sphere.
TEST_F(RenderEngineGlTest, SphereTest) {
  SetUp(X_WR_, true);

  PopulateSphereTest(renderer_.get());

  PerformCenterShapeTest(renderer_.get(), "Sphere test");
}

// Performs the shape centered in the image with a cylinder.
TEST_F(RenderEngineGlTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  RenderIndex geometry_index = renderer_->RegisterVisual(
      cylinder, simple_material(), Isometry3d::Identity());
  // Position the top of the cylinder to be 1 m above the terrain.
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.4)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Cylinder test");
}

// Performs the shape centered in the image with a mesh (which happens to be a
// box). This simultaneously confirms that if a diffuse_map is specified but it
// doesn't refer to a file that can be read, that the appearance defaults to
// the diffuse rgba value.
TEST_F(RenderEngineGlTest, MeshTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture.
  material.AddProperty("phong", "diffuse_map", "bad_path");
  RenderIndex geometry_index = renderer_->RegisterVisual(
      mesh, material, Isometry3d::Identity());
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// Performs the shape centered in the image with a convex mesh (which happens to
// be a  box). This simultaneously confirms that if a diffuse_map is specified
// but it doesn't refer to a file that can be read, that the appearance defaults
// to the diffuse rgba value.
TEST_F(RenderEngineGlTest, ConvexTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Convex convex(filename);
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture.
  material.AddProperty("phong", "diffuse_map", "bad_path");
  RenderIndex geometry_index = renderer_->RegisterVisual(
      convex, material, Isometry3d::Identity());
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// This confirms that geometries are correctly removed from the render engine.
// We add two new geometries (testing the rendering after each addition).
// By removing the first of the added geometries, we can confirm that the
// remaining geometries are re-ordered appropriately. Then by removing the,
// second we should restore the original default image.
//
// The default image is based on a sphere sitting on a plane at z = 0 with the
// camera located above the sphere's center and aimed at that center.
// THe default sphere is drawn with `●`, the first added sphere with `x`, and
// the second with `o`. The height of the top of each sphere and its depth in
// the camera's depth sensors are indicated as zᵢ and dᵢ, i ∈ {0, 1, 2},
// respectively.
//
//             /|\       <---- camera_z = 3
//              v
//
//
//
//
//            ooooo       <---- z₂ = 4r = 2, d₂ = 1
//          oo     oo
//         o         o
//        o           o
//        o           o
//        o   xxxxx   o   <---- z₁ = 3r = 1.5, d₁ = 1.5
//         oxx     xxo
//         xoo     oox
//        x   ooooo   x
//        x   ●●●●●   x   <---- z₀ = 2r = 1, d₀ = 2
//        x ●●     ●● x
//         ●         ●
//        ● xx     xx ●
// z      ●   xxxxx   ●
// ^      ●           ●
// |       ●         ●
// |        ●●     ●●
// |__________●●●●●____________
//
TEST_F(RenderEngineGlTest, RemoveVisual) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());
  float default_depth = expected_object_depth_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](double z) {
    const double kRadius = 0.5;
    Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    PerceptionProperties material;
    RenderIndex index =
        renderer_->RegisterVisual(sphere, material, Isometry3d::Identity());
    Isometry3d X_WV{Eigen::Translation3d(0, 0, z)};
    renderer_->UpdateVisualPose(X_WV, index);
    return std::make_tuple(index, depth);
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](float depth) {
    expected_object_depth_ = depth;
  };

  // Add another sphere of a different color in front of the default sphere
  float depth1{};
  RenderIndex index1{};
  std::tie(index1, depth1) = add_sphere(0.75);
  set_expectations(depth1);
  PerformCenterShapeTest(renderer_.get(), "First sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  float depth2{};
  RenderIndex index2{};
  std::tie(index2, depth2) = add_sphere(1.0);
  set_expectations(depth2);
  PerformCenterShapeTest(renderer_.get(), "Second sphere added in remove test");

  // Remove the first sphere added:
  //  1. index2 should be returned as the index of the shape that got moved.
  //  2. The test should pass without changing expectations.
  optional<RenderIndex> moved = renderer_->RemoveVisual(index1);
  EXPECT_TRUE(moved);
  EXPECT_EQ(*moved, index2);
  PerformCenterShapeTest(renderer_.get(), "First added sphere removed");

  // Remove the second added sphere (now indexed by index1):
  //  1. There should be no returned index.
  //  2. The rendering should match the default sphere test results.
  // Confirm restoration to original image.
  moved = nullopt;
  moved = renderer_->RemoveVisual(index1);
  EXPECT_FALSE(moved);
  set_expectations(default_depth);
  PerformCenterShapeTest(renderer_.get(),
                         "Default image restored by removing extra geometries");
}


// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineGlTest, SimpleClone) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineGl*>(clone.get()), nullptr);
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()),
                         "Simple clone");
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineGlTest, ClonePersistence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed.
  renderer_.reset();
  ASSERT_EQ(nullptr, renderer_);
  PerformCenterShapeTest(static_cast<RenderEngineGl*>(clone.get()),
                         "Clone persistence");
}

// Tests that the cloned renderer still works, even when the original has values
// RenderEngineGlTest
TEST_F(RenderEngineGlTest, CloneIndependence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  Isometry3d X_WT_new{Translation3d{0, 0, 10}};
  // This assumes that the terrain is zero-indexed.
  renderer_->UpdateVisualPose(X_WT_new, RenderIndex(0));
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()),
                         "Clone independence");
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineGlTest, DifferentCameras) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  // Baseline -- confirm that all of the defaults in this test still produce
  // the expected outcome.
  PerformCenterShapeTest(renderer_.get(), "Camera change - baseline", &camera_);

  // Test changes in sensor sizes.
  {
    // Now run it again with a camera with a smaller sensor (quarter area).
    DepthCameraProperties small_camera{camera_};
    small_camera.width /= 2;
    small_camera.height /= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - small camera",
                           &small_camera);

    // Now run it again with a camera with a bigger sensor (4X area).
    DepthCameraProperties big_camera{camera_};
    big_camera.width *= 2;
    big_camera.height *= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - big camera",
                           &big_camera);
  }

  // Test changes in fov (larger and smaller).
  {
    DepthCameraProperties narrow_fov(camera_);
    narrow_fov.fov_y /= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - narrow fov",
                           &narrow_fov);

    DepthCameraProperties wide_fov(camera_);
    wide_fov.fov_y *= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - wide fov",
                           &wide_fov);
  }

  // Test changes to depth range.
  {
    DepthCameraProperties clipping_far_plane(camera_);
    clipping_far_plane.z_far = expected_outlier_depth_ - 0.1;
    const float old_outlier_depth = expected_outlier_depth_;
    expected_outlier_depth_ = std::numeric_limits<float>::infinity();
    PerformCenterShapeTest(renderer_.get(),
                           "Camera change - z far clips terrain",
                           &clipping_far_plane);
    // NOTE: Need to restored expected outlier depth for next test.
    expected_outlier_depth_ = old_outlier_depth;

    DepthCameraProperties clipping_near_plane(camera_);
    clipping_near_plane.z_near = expected_object_depth_ + 0.1;
    expected_object_depth_ = 0;
    PerformCenterShapeTest(renderer_.get(),
                           "Camera change - z near clips mesh",
                           &clipping_near_plane);
  }
}

// Tests that registered geometry without specific values renders without error.
// TODO(SeanCurtis-TRI): When the ability to set defaults is exposed through a
// public API, actually test for the *default values*. Until then, error-free
// rendering is sufficient.
TEST_F(RenderEngineGlTest, DefaultProperties) {
  SetUp(X_WR_, false  /* no terrain */);

  // Sets up a box.
  Box box(1, 1, 1);
  RenderIndex geometry_index = renderer_->RegisterVisual(
      box, PerceptionProperties(), Isometry3d::Identity());
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  EXPECT_NO_THROW(Render());
}

// This is a test to make sure that independent renders are truly independent.
// So, we're going to do a bunch of interleaved operations on the two renderers
// to make sure that each is still correct.
TEST_F(RenderEngineGlTest, MultipleRenderers) {
  RenderEngineGl engine1;
  RenderEngineGl engine2;

  engine1.UpdateViewpoint(X_WR_);
  engine1.AddFlatTerrain();
  PopulateSphereTest(&engine1);
  PerformCenterShapeTest(&engine1, "engine1 - initial render");

  Render(&engine2);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());

  engine2.UpdateViewpoint(X_WR_);
  engine2.AddFlatTerrain();
  Box box(1, 1, 1);
  RenderIndex geometry_index =
      engine2.RegisterVisual(box, simple_material(), Isometry3d::Identity());
  Isometry3d X_WV{Eigen::Translation3d(0.2, 0.2, 0.5)};
  engine2.UpdateVisualPose(X_WV, geometry_index);
  PerformCenterShapeTest(&engine2, "engine2 - offset box test");

  PerformCenterShapeTest(&engine1, "engine1 - second render");
}

}  // namespace
}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
