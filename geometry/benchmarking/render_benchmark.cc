/* See render_benchmark_doxygen.h for discussion of this benchmark.  */
#include <unistd.h>

#include <filesystem>
#include <iostream>

#include <benchmark/benchmark.h>
#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/profiler.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRange;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::SaveToPng;
using systems::sensors::SaveToTiff;

DEFINE_string(save_image_path, "",
              "Enables saving rendered images in the given location");
DEFINE_bool(show_window, false, "Whether to display the rendered images");

// Default sphere array sizes.
const double kZSpherePosition = -4.;

// Default camera properties.
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

/* The render engines generally supported by this benchmark; not all
 renderers are supported by all operating systems.  */
enum class EngineType { Vtk, Gl };

/* Creates a render engine of the given type with the given background color. */
template <EngineType engine_type>
std::unique_ptr<RenderEngine> MakeEngine(
    const Vector3d& bg_rgb, const std::vector<LightParameter>& lights = {}) {
  // Offset the light from its default position (coincident with the camera)
  // so that shadows can be seen in the render.
  if constexpr (engine_type == EngineType::Vtk) {
    const RenderEngineVtkParams params{.default_clear_color = bg_rgb,
                                       .lights = lights};
    return MakeRenderEngineVtk(params);
  }
  if constexpr (engine_type == EngineType::Gl) {
    const Rgba bg(bg_rgb[0], bg_rgb[1], bg_rgb[2]);
    const RenderEngineGlParams params{.default_clear_color = bg,
                                      .lights = lights};
    return MakeRenderEngineGl(params);
  }
}

class RenderBenchmark : public benchmark::Fixture {
 public:
  struct ArgProfiler {
    std::string name;
    benchmark::IterationCount iterations;
    std::string table;
  };

  RenderBenchmark() {
    default_material_.AddProperty("phong", "diffuse", sphere_rgba_);
    default_material_.AddProperty("label", "id", RenderLabel::kDontCare);
  }

  ~RenderBenchmark() override {
    for (const auto& result : profilers_) {
      std::cerr << result.name << "\n" << result.table << "\n";
    }
  }

  void SetUp(::benchmark::State&) { depth_cameras_.clear(); }

  std::vector<LightParameter> CreateLights(int num_lights) const {
    const double kTotalIntensity = 1.0;
    const double kPerLightIntensity = kTotalIntensity / num_lights;
    /* Five positions distributed uniformly around a circle. The ordering jumps
    around to give them an interesting orientation. */
    const std::vector<Vector3d> light_positions{{0.5, 0.5, 0},
                                                {-0.11062, -0.6984, 0},
                                                {-0.32102, 0.63004, 0},
                                                {0.63004, -0.32102, 0},
                                                {-0.6984, -0.11062, 0}};
    std::vector<LightParameter> lights;
    for (int i = 0; i < num_lights; ++i) {
      lights.push_back(LightParameter{.type = "point",
                                      .position = light_positions[i] * 10,
                                      .intensity = kPerLightIntensity});
    }
    return lights;
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void ColorImage(::benchmark::State& state, const std::string& name) {
    auto [sphere_count, camera_count, width, height, num_lights, sphere_type] =
        ReadState(state);
    auto renderer = MakeEngine<engine_type>(bg_rgb_, CreateLights(num_lights));
    SetupScene(sphere_count, sphere_type, camera_count, width, height,
               renderer.get());
    ImageRgba8U color_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[0].core(),
                                        FLAGS_show_window);
      renderer->RenderColorImage(color_cam, &color_image);
    }

    reset();
    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderColorImage(color_cam, &color_image);
      }
    }
    const std::string profile_name =
        fmt::format("{}/{}/{}/{}/{}/{}", state.name(), sphere_count,
                    camera_count, width, height, num_lights);
    if (profilers_.size() == 0) {
      profilers_.push_back({.name = profile_name,
                            .iterations = state.iterations(),
                            .table = TableOfAverages()});
    } else {
      if (profilers_.back().name != profile_name) {
        profilers_.push_back({.name = profile_name,
                              .iterations = state.iterations(),
                              .table = TableOfAverages()});
      } else if (profilers_.back().iterations < state.iterations()) {
        profilers_.back().iterations += state.iterations();
        profilers_.back().table = TableOfAverages();
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "png");
      SaveToPng(color_image, path_name);
    }
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void DepthImage(::benchmark::State& state, const std::string& name) {
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    auto [sphere_count, camera_count, width, height, nil, sphere_type] =
        ReadState(state);
    SetupScene(sphere_count, sphere_type, camera_count, width, height,
               renderer.get());
    ImageDepth32F depth_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      renderer->RenderDepthImage(depth_cameras_[0], &depth_image);
    }

    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        renderer->RenderDepthImage(depth_cameras_[i], &depth_image);
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "tiff");
      SaveToTiff(depth_image, path_name);
    }
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void LabelImage(::benchmark::State& state, const std::string& name) {
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    auto [sphere_count, camera_count, width, height, nil, sphere_type] =
        ReadState(state);
    SetupScene(sphere_count, sphere_type, camera_count, width, height,
               renderer.get());
    ImageLabel16I label_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[0].core(),
                                        FLAGS_show_window);
      renderer->RenderLabelImage(color_cam, &label_image);
    }

    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderLabelImage(color_cam, &label_image);
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "png");
      SaveToPng(label_image, path_name);
    }
  }

  /* Parse arguments from the benchmark state.
   @return A tuple representing the sphere count, camera count, width, height,
           light_cont, sphere_type. */
  static std::tuple<int, int, int, int, int, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1), state.range(2),
                           state.range(3), state.range(4), state.range(5));
  }

  /* Helper function for generating the image path name based on the benchmark
   arguments and file format. The benchmark state is assumed to have 4 arguments
   representing the sphere count, camera count, width, and height.  */
  static std::string image_path_name(const std::string& test_name,
                                     const benchmark::State& state,
                                     const std::string& format) {
    DRAKE_DEMAND(!FLAGS_save_image_path.empty());
    std::filesystem::path save_path = FLAGS_save_image_path;
    const auto [sphere_count, camera_count, width, height, num_lights,
                sphere_type] = ReadState(state);
    const char* sphere_labels[] = {"prim", "obj", "gltf"};
    return save_path.append(fmt::format(
        "{}_{}_{}_{}_{}_{}_{}.{}", test_name, sphere_count, camera_count, width,
        height, num_lights, sphere_labels[sphere_type], format));
  }

  /* Computes a compact array of spheres which will remain in view. */
  void AddSphereArray(int sphere_count, int sphere_type,
                      const RenderCameraCore& core, RenderEngine* engine) {
    /* We assume the camera is located at (0, 0, c.z) pointing in the -Wz
     direction. We further assume that the camera's "up" direction points in the
     +Wy direction. All spheres will be placed on a plane at z = s.z. Given the
     camera field of view and w/h aspect ratio, we can determine the visible
     rectangle at s.z.

         c.z      s.z         Simple geometry.
          ┆        ┆          Right triangle with height d = s.z - c.z and angle
          ┆        ╱          θ = fov_y / 2.
          ┆      ╱ ┆          hₛ/2 = d * tan(θ)
          ┆    ╱   ┆ hₛ/2       hₛ = 2d * tan(θ)
          ┆  ╱     ┆          wₛ = hₛ * w / h
          ┆╱_θ_____┆______    (w, h) is the size of the image sensor giving us
           ╲ θ     ┆          the camera's aspect ratio.
             ╲     ┆
               ╲   ┆
                 ╲ ┆
                   ╲
                   ┆
      This gives us the measure of the rectangle we need to fit all spheres
      into. We want all spheres to be visible so that it affects the rendering
      cost. */
    const double aspect_ratio = core.intrinsics().width() /
                                static_cast<double>(core.intrinsics().height());
    const double theta_2 = core.intrinsics().fov_y() / 2.0;
    const double d = -kZSpherePosition;  // c.z = 0.
    const double h = 2 * d * std::tan(theta_2);
    /* Given the measure of the rectangle, we need to place the spheres: this
     includes determining radius and position. We'll place the N spheres in a
     rectangular grid with R rows and C columns. It must be the case that
     RC ≥ N. With the constraint that we want C/R "as close" to w/h as possible.
     We define "as close as possible" as the maximum C/R ≤ w/h. For notation
     convenience we'll define α = w/h.

       RC ≥ N  --> C/R ≥ N/R²  (C, R, and N are all positive).
       C/R ≥ N/R² and C/R ≤ α --> N/R² ≤ C/R ≤ α
       N/R² ≤ α
       N/α ≤ R²
       √(N/α) ≤ R

     We'll use the formula above to find our initial guess for the number of
     rows. We'll increment row to the last value that satisfies C/R ≤ w/h. */
    const double N = static_cast<double>(sphere_count);
    int rows = static_cast<int>(std::max(std::sqrt(N / aspect_ratio), 1.0));
    int cols = static_cast<int>(std::ceil(N / rows));
    while (static_cast<double>(cols) / rows > aspect_ratio) {
      ++rows;
      cols = static_cast<int>(std::ceil(N / rows));
    }

    /* Because we've required C/R ≤ w/h, we know we'll always be fitting the
     number of rows to the height of the image. The radius value we want
     satisfies R * 2*radius = h. radius = h / 2R. */
    const double distance = h / (2 * rows);
    /* We make the actual radius *slightly* smaller so there's some space
     between the spheres. */
    const double radius = distance * 0.95;
    const Vector3d scale(radius, radius, radius);
    Sphere primitive(radius);
    const std::string root = "/home/seancurtis/code/drake/geometry/benchmarking/";
    Mesh obj(root + "color_texture_sphere.obj", scale);
    Mesh gltf(root + "multi_texture_sphere.gltf", scale);
    std::array<Shape*, 3> spheres = {&primitive, &obj, &gltf};
    auto add_sphere = [this, engine, &spheres,
                       sphere_type](const Vector3d& p_WS) {
      GeometryId geometry_id = GeometryId::get_new_id();
      PerceptionProperties material;
      if (sphere_type == 0) {
        material = default_material_;
      }
      engine->RegisterVisual(geometry_id, *spheres[sphere_type], material,
                             RigidTransformd::Identity(),
                             true /* needs update */);
      poses_.insert({geometry_id, RigidTransformd{p_WS}});
    };

    int count = 0;
    double y = -(rows - 1) * distance;
    for (int r = 0; r < rows; ++r) {
      double x = -(cols - 1) * distance;
      for (int c = 0; c < cols; ++c) {
        add_sphere(Vector3d{x, y, kZSpherePosition});
        ++count;
        if (count >= sphere_count) break;
        x += 2 * distance;
      }
      if (count >= sphere_count) break;
      y += 2 * distance;
    }
  }

  /*
   @param sphere_type  0: primitive, 1: obj, 2: glTF. */
  void SetupScene(const int sphere_count, int sphere_type,
                  const int camera_count, const int width, const int height,
                  RenderEngine* engine) {
    // Set up the camera so that Cz = -Wz, Cx = Wx, and Cy = -Wy. The camera
    // will look down the Wz axis and have the image U direction aligned with
    // the Wx direction.
    const Vector3d Cx_W{1, 0, 0};
    const Vector3d Cy_W{0, -1, 0};
    const Vector3d Cz_W{0, 0, -1};
    RigidTransformd X_WC{
        RotationMatrixd::MakeFromOrthonormalColumns(Cx_W, Cy_W, Cz_W)};
    engine->UpdateViewpoint(X_WC);

    // Add the cameras.
    for (int i = 0; i < camera_count; ++i) {
      depth_cameras_.emplace_back(RenderCameraCore{"unused" + std::to_string(i),
                                                   {width, height, kFovY},
                                                   {0.01, 100.0},
                                                   {}},
                                  DepthRange{kZNear, kZFar});
    }
    AddSphereArray(sphere_count, sphere_type, depth_cameras_[0].core(), engine);
  }

  std::vector<DepthRenderCamera> depth_cameras_;
  PerceptionProperties default_material_;
  const Vector3d bg_rgb_{200 / 255., 0, 250 / 255.};
  const Rgba sphere_rgba_{0, 0.8, 0.5, 1};
  std::unordered_map<GeometryId, RigidTransformd> poses_;
  std::vector<ArgProfiler> profilers_;
};

/* Inorder to reuse the same fixture and RendererFoo methods, we alias the
fixture, so that we can use --benchmark_filter to select on the set of
arguments by filtering in the prefix of the fixture name. */
using LightingBenchmark = RenderBenchmark;
using ReadbackBenchmark = RenderBenchmark;
using TextureBenchmark = RenderBenchmark;

/* These macros serve the purpose of allowing compact and *consistent*
 declarations of benchmarks. The goal is to create a benchmark for each
 renderer type (e.g., Vtk, Gl) combined with each image type (Color, Depth, and
 Label). Each benchmark instance should be executed using the same parameters.

 These macros guarantee that a benchmark is declared, dispatches the right
 benchmark harness and is executed with a common set of parameters.

 The macro is invoked as follows:

   MAKE_BENCHMARK(Foo, ImageType)

 such that there must be a `EngineType::Foo` enum and ImageType must be one of
 (Color, Depth, or Label). Capitalization matters.

 N.B. The macro STR converts a single macro parameter into a string and we use
 it to make a string out of the concatenation of two macro parameters (i.e., we
 get FooColor out of the parameters Foo and Color).

 The parameters are 5-tuples of: sphere count, camera count, image width, image
 height, and light count. */
#define STR(s) #s

// The boilerplate for defining a benchmark. The result can immediately have
// configuration APIs invoked.
#define DEFINE_BENCHMARK(Fixture, Renderer, ImageT)                     \
  BENCHMARK_DEFINE_F(Fixture, Renderer##ImageT)                         \
  (benchmark::State & state) {                                          \
    ImageT##Image<EngineType::Renderer>(state, STR(Renderer##ImageT));  \
  }                                                                     \
  BENCHMARK_REGISTER_F(Fixture, Renderer##ImageT)

#define MAKE_BENCHMARK(Renderer, ImageT)                                 \
  DEFINE_BENCHMARK(LightingBenchmark, Renderer, ImageT)                  \
      ->Unit(benchmark::kMillisecond)                                    \
      ->ArgsProduct({{1}, {1}, {640}, {480}, {1, 2, 3, 4, 5}, {0}})      \
      ->ArgsProduct({{1}, {1}, {1280}, {960}, {1, 2, 3, 4, 5}, {0}})     \
      ->ArgsProduct({{1}, {1}, {2560}, {1920}, {1, 2, 3, 4, 5}, {0}})    \
      ->ArgsProduct({{960}, {1}, {2560}, {1920}, {1, 2, 3, 4, 5}, {0}}); \
                                                                         \
  DEFINE_BENCHMARK(ReadbackBenchmark, Renderer, ImageT)                  \
      ->Unit(benchmark::kMillisecond)                                    \
      ->ArgsProduct({{1, 960}, {10}, {640}, {480}, {1}, {0}})            \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {640}, {480}, {1}, {0}})    \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {320}, {240}, {1}, {0}})    \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {1280}, {960}, {1}, {0}})   \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {2560}, {1920}, {1}, {0}}); \
                                                                         \
  DEFINE_BENCHMARK(TextureBenchmark, Renderer, ImageT)                   \
      ->Unit(benchmark::kMillisecond)                                    \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {640}, {480}, {1}, {1}})    \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {1280}, {960}, {1}, {1}})   \
      ->ArgsProduct(                                                     \
          {{1, 60, 120, 240, 480, 960}, {1}, {2560}, {1920}, {1}, {1}});

MAKE_BENCHMARK(Vtk, Color);
// MAKE_BENCHMARK(Vtk, Depth);
// MAKE_BENCHMARK(Vtk, Label);

#ifndef __APPLE__
MAKE_BENCHMARK(Gl, Color);
// MAKE_BENCHMARK(Gl, Depth);
// MAKE_BENCHMARK(Gl, Label);
#endif

}  // namespace
}  // namespace geometry
}  // namespace drake
