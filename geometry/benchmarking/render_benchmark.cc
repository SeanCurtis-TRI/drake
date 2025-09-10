/* See render_benchmark_doxygen.h for discussion of this benchmark.  */
#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <string>

#include <benchmark/benchmark.h>
#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/profiler.h"
#include "drake/common/string_map.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/systems/sensors/image_io.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

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
using systems::sensors::ImageIo;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

DEFINE_string(save_image_path, "",
              "Enables saving rendered images in the given location");
DEFINE_bool(show_window, false, "Whether to display the rendered images");
DEFINE_bool(force_duplication, false,
            "If true, force GL to duplicate resources");

// Default sphere array sizes.
const double kZSpherePosition = -4.;

// Default camera properties.
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

/* The render engines generally supported by this benchmark; not all
 renderers are supported by all operating systems.  */
enum class EngineType { Vtk, Gl };

/* The aggregate of all parameters used across all fixtures. Some fixtures may
 choose to set arbitrary defaults to some of these and only parameterize a
 subset.

 However, all fixtures ultimately report with respect to a common parameter
 space so that the profiling data is formatted uniformly to facilitate analysis.
 */
struct FixtureParameters {
  std::string benchmark_name;
  int sphere_count = 1;
  int camera_count = 1;
  int width = 640;
  int height = 480;
  int num_lights = 1;
  // 0: primitive, 1: obj, 2: single-texture gltf, 3: multi-texture gltf.
  int sphere_type = 0;
};

/* Creates a render engine of the given type with the given background color and
 light set. */
template <EngineType engine_type>
std::unique_ptr<RenderEngine> MakeEngine(
    const Vector3d& bg_rgb, const std::vector<LightParameter>& lights = {}) {
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

/* Used to specify how to format benchmark state as a "name", either for a file
 or for display purposes. */
struct NameFormat {
  std::string delimiter = "/";
  std::string_view alt_name;
};

/* The basic fixture. It is responsible for initializing a render engine and
 running the requested renderings against that engine. */
class RenderBenchmarkBase : public benchmark::Fixture {
 public:
  RenderBenchmarkBase() {
    default_material_.AddProperty("phong", "diffuse", sphere_rgba_);
    default_material_.AddProperty("label", "id", RenderLabel::kDontCare);
  }

  ~RenderBenchmarkBase() override {
    for (const auto& result : profilers_) {
      std::cerr << result.name << "\n" << result.table << "\n";
    }
  }

  void SetUp(::benchmark::State&) { depth_cameras_.clear(); }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void ColorImage(::benchmark::State& state, const std::string& name) {
    const FixtureParameters params = GetParametersFromState(state);
    auto renderer = MakeEngine<engine_type>(bg_rgb_, CreateLights(params));
    SetupScene(params, renderer.get());
    ImageRgba8U color_image(params.width, params.height);

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
      for (int i = 0; i < params.camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderColorImage(color_cam, &color_image);
      }
    }
    FinishIteration(state, params, name, color_image, "png");
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void DepthImage(::benchmark::State& state, const std::string& name) {
    const FixtureParameters params = GetParametersFromState(state);
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    SetupScene(params, renderer.get());
    ImageDepth32F depth_image(params.width, params.height);

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
      for (int i = 0; i < params.camera_count; ++i) {
        renderer->RenderDepthImage(depth_cameras_[i], &depth_image);
      }
    }
    FinishIteration(state, params, name, depth_image, "tiff");
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void LabelImage(::benchmark::State& state, const std::string& name) {
    const FixtureParameters params = GetParametersFromState(state);
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    SetupScene(params, renderer.get());
    ImageLabel16I label_image(params.width, params.height);

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
      for (int i = 0; i < params.camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderLabelImage(color_cam, &label_image);
      }
    }
    FinishIteration(state, params, name, label_image, "png");
  }

 protected:
  /* Convert benchmark state into common fixture parameters. Child fixtures can
   override this so they can specify a subset of the parameters in their
   declarations.
   The default implementation assumes all fixture parameters are contained in
   the state (in a very specific order). */
  virtual FixtureParameters DoGetParametersFromState(
      const ::benchmark::State& state) {
    return FixtureParameters{
        .sphere_count = static_cast<int>(state.range(0)),
        .camera_count = static_cast<int>(state.range(1)),
        .width = static_cast<int>(state.range(2)),
        .height = static_cast<int>(state.range(3)),
        .num_lights = static_cast<int>(state.range(4)),
        .sphere_type = static_cast<int>(state.range(5)),
    };
  }

 private:
  /* Create a set of lights from the fixture parameters. */
  std::vector<LightParameter> CreateLights(
      const FixtureParameters& params) const {
    const double kTotalIntensity = 1.0;
    const double kPerLightIntensity = kTotalIntensity / params.num_lights;
    /* Five positions distributed uniformly around a circle. The positions are
     *not* ordered around the circle, but jump from position to position so that
     two lights in sequence are not adjacent. */
    const std::vector<Vector3d> light_positions{{0.5, 0.5, 0},
                                                {-0.11062, -0.6984, 0},
                                                {-0.32102, 0.63004, 0},
                                                {0.63004, -0.32102, 0},
                                                {-0.6984, -0.11062, 0}};
    std::vector<LightParameter> lights;
    for (int i = 0; i < params.num_lights; ++i) {
      lights.push_back(LightParameter{.type = "point",
                                      .position = light_positions[i] * 10,
                                      .intensity = kPerLightIntensity});
    }
    return lights;
  }

  FixtureParameters GetParametersFromState(const ::benchmark::State& state) {
    auto params = DoGetParametersFromState(state);
    params.benchmark_name = state.name();
    return params;
  }

  struct ArgProfiler {
    std::string name;
    benchmark::IterationCount iterations;
    std::string table;
  };

  // Returns a string of the form Name/#/#/#.../# based on the state name and
  // state argument values.
  static std::string GetBenchmarkName(const FixtureParameters& params,
                                      NameFormat format = {}) {
    std::string result = format.alt_name.empty() ? params.benchmark_name
                                                 : std::string(format.alt_name);

    result += fmt::format("{}{}", format.delimiter, params.sphere_count);
    result += fmt::format("{}{}", format.delimiter, params.camera_count);
    result += fmt::format("{}{}", format.delimiter, params.width);
    result += fmt::format("{}{}", format.delimiter, params.height);
    result += fmt::format("{}{}", format.delimiter, params.num_lights);
    result += fmt::format("{}{}", format.delimiter, params.sphere_type);

    return result;
  }

  /* Helper function for generating the image path name based on the benchmark
   arguments and file format. The benchmark state is assumed to have 4 arguments
   representing the sphere count, camera count, width, and height.  */
  static std::string image_path_name(std::string_view test_name,
                                     const FixtureParameters& params,
                                     std::string_view format) {
    DRAKE_DEMAND(!FLAGS_save_image_path.empty());
    fs::path save_path = FLAGS_save_image_path;
    const std::string name = fmt::format(
        "{}.{}",
        GetBenchmarkName(params,
                         NameFormat{.delimiter = "_", .alt_name = test_name}),
        format);
    return save_path / name;
  }

  /* Reads the obj located at `path` and creates an in-memory version with
   unique mesh and texture names. */
  Mesh ForceObjDuplication(const fs::path& path, int index, double scale) {
    // RenderEngineGl uses geometry resource management to prevent redundantly
    // loading the same data over and over. That management is based on
    // computing the sha of the file contents. So, to create apparently
    // different files, we need apparently different contents.
    //
    //  For both the obj and the .png file, we can simply append some bytes
    //  (commented out for the .obj). The png file will simply ignore extra
    //  bytes beyond its final chunk marker.

    const auto source_png =
        MemoryFile::Make(path.parent_path() / "base_color.png");
    const auto source_obj = MemoryFile::Make(path);

    string_map<FileSource> supporting_files{
        {"color_texture_sphere.mtl",
         MemoryFile(
             fmt::format(
                 "newmtl diffuse_map_only\nNs 250\nKa 1 1 1\nKs 0.5 0.5 0.5\n"
                 "Ke 0 0 0\nNi 1.5\nd 1\nillum 2\nmap_Kd color_{}.png\n",
                 index),
             ".mtl", "color_texture_sphere.mtl")},
        {fmt::format("color_{}.png", index),
         MemoryFile(source_png.contents() + fmt::to_string(index), ".png",
                    fmt::format("color_{}.png", index))},
    };
    std::string new_contents =
        source_obj.contents() + fmt::format("\n// {}\n", index);
    return Mesh(InMemoryMesh(MemoryFile(std::move(new_contents), ".obj",
                                        fmt::format("sphere_{}.obj", index)),
                             supporting_files),
                scale);
  }

  Mesh ForceGltfDuplication(const fs::path& path, int index, double scale) {
    throw std::runtime_error(
        fmt::format("Not duplicating gltf for {} yet.", index));
    return Mesh(path, scale);
  }

  /* @pre FLAGS_force_duplication == false. */
  GeometryInstance MakeSphereInstance(int sphere_type, int index,
                                      double radius) {
    DRAKE_DEMAND(sphere_type >= 0 && sphere_type <= 3);
    if (FLAGS_force_duplication && sphere_type == 3) {
      throw std::runtime_error(
          "Don't pass the --force_duplication flag for benchmark fixtures that "
          "use sphere_type > 2.");
    }

    auto make_instance = [index](const auto& shape,
                                 PerceptionProperties material = {}) {
      GeometryInstance instance(RigidTransformd::Identity(), shape,
                                fmt::format("sphere_{}", index));
      instance.set_perception_properties(material);
      return instance;
    };

    if (sphere_type == 0) {
      return make_instance(Sphere(radius), default_material_);
    } else {
      const fs::path test_path =
          FindResourceOrThrow("drake/geometry/benchmarking/omr.png");
      const fs::path mesh_path = test_path.parent_path();
      if (FLAGS_force_duplication) {
        // When we force duplication, we need to trick the resource management
        // into thinking that every mesh and texture is unique. We can do this
        // simply by renaming the files.
        if (sphere_type == 1) {
          return make_instance(ForceObjDuplication(
              mesh_path / "color_texture_sphere.obj", index, radius));
        } else if (sphere_type == 2) {
          return make_instance(ForceGltfDuplication(
              mesh_path / "color_texture_sphere.gltf", index, radius));
        }
        DRAKE_UNREACHABLE();
      } else {
        // If we're not forcing duplication, we can simply name the mesh file.
        if (sphere_type == 1) {
          return make_instance(
              Mesh(mesh_path / "color_texture_sphere.obj", radius));
        } else if (sphere_type == 2) {
          return make_instance(
              Mesh(mesh_path / "color_texture_sphere.gltf", radius));
        } else {
          return make_instance(
              Mesh(mesh_path / "multi_texture_sphere.gltf", radius));
        }
      }
    }
  }

  /* Computes a compact array of spheres which will remain in view. */
  void AddSphereArray(const FixtureParameters& params,
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
    const double N = static_cast<double>(params.sphere_count);
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

    int count = 0;
    double y = -(rows - 1) * distance;
    for (int r = 0; r < rows; ++r) {
      double x = -(cols - 1) * distance;
      for (int c = 0; c < cols; ++c) {
        const GeometryInstance instance =
            MakeSphereInstance(params.sphere_type, count, radius);
        engine->RegisterVisual(
            instance.id(), instance.shape(), *instance.perception_properties(),
            RigidTransformd::Identity(), true /* needs update */);
        poses_[instance.id()] =
            RigidTransformd(Vector3d(x, y, kZSpherePosition));
        ++count;
        if (count >= params.sphere_count) break;
        x += 2 * distance;
      }
      if (count >= params.sphere_count) break;
      y += 2 * distance;
    }
  }

  /*
   @param sphere_type  0: primitive, 1: obj, 2: glTF. */
  void SetupScene(const FixtureParameters& params, RenderEngine* engine) {
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
    for (int i = 0; i < params.camera_count; ++i) {
      depth_cameras_.emplace_back(
          RenderCameraCore{"unused" + std::to_string(i),
                           {params.width, params.height, kFovY},
                           {0.01, 100.0},
                           {}},
          DepthRange{kZNear, kZFar});
    }
    AddSphereArray(params, depth_cameras_[0].core(), engine);
  }

  /* At the conclusion of an iteration, optionall write out profile logs and
  images as specified. */
  template <typename ImageType>
  void FinishIteration(const ::benchmark::State& state,
                       const FixtureParameters& params, std::string_view name,
                       const ImageType& image, std::string_view extension) {
    // Note: benchmark will tentatively run multiple passes for a single
    // benchmark, probing to find an appropriate number of iterations. This
    // function aggregates those redundant attempts, preferring the result with
    // the most iterations.
    const std::string profile_name = GetBenchmarkName(params);
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
      const std::string path_name = image_path_name(name, params, extension);
      ImageIo().Save(image, path_name);
    }
  }

  const Rgba sphere_rgba_{0, 0.8, 0.5, 1};
  PerceptionProperties default_material_;
  std::vector<DepthRenderCamera> depth_cameras_;
  std::vector<ArgProfiler> profilers_;
  const Vector3d bg_rgb_{200 / 255., 0, 250 / 255.};
  std::unordered_map<GeometryId, RigidTransformd> poses_;
};

/* The benchmark sets the benchmark state as (using default or all others):
  (sphere_count, camera_count, width, height).
*/
class ReadbackBenchmark : public RenderBenchmarkBase {
 public:
  FixtureParameters DoGetParametersFromState(
      const ::benchmark::State& state) override {
    return FixtureParameters{
        .sphere_count = static_cast<int>(state.range(0)),
        .camera_count = static_cast<int>(state.range(1)),
        .width = static_cast<int>(state.range(2)),
        .height = static_cast<int>(state.range(3)),
    };
  }
};

/* The benchmark sets the benchmark state as (using default or all others):
  (sphere_count, width, height, num_lights).
*/
class LightingBenchmark : public RenderBenchmarkBase {
 public:
  FixtureParameters DoGetParametersFromState(
      const ::benchmark::State& state) override {
    return FixtureParameters{
        .sphere_count = static_cast<int>(state.range(0)),
        .width = static_cast<int>(state.range(1)),
        .height = static_cast<int>(state.range(2)),
        .num_lights = static_cast<int>(state.range(3)),
    };
  }
};

/* The benchmark sets the benchmark state as (using default on all others):
 (sphere_count, width, height, sphere_type). */
class TextureBenchmark : public RenderBenchmarkBase {
 public:
  FixtureParameters DoGetParametersFromState(
      const ::benchmark::State& state) override {
    return FixtureParameters{
        .sphere_count = static_cast<int>(state.range(0)),
        .width = static_cast<int>(state.range(1)),
        .height = static_cast<int>(state.range(2)),
        .sphere_type = static_cast<int>(state.range(3)),
    };
  }
};

/* The benchmark sets the benchmark state as (using default on all others):
 (sphere_count, width, height, sphere_type). */
class PbrVsPhongBenchmark : public RenderBenchmarkBase {
 public:
  FixtureParameters DoGetParametersFromState(
      const ::benchmark::State& state) override {
    return FixtureParameters{
        .sphere_count = static_cast<int>(state.range(0)),
        .width = static_cast<int>(state.range(1)),
        .height = static_cast<int>(state.range(2)),
        .sphere_type = static_cast<int>(state.range(3)),
    };
  }
};

/* These macros serve the purpose of allowing compact and *consistent*
 declarations of benchmarks. The goal is to create a benchmark for each
 renderer type (e.g., Vtk, Gl) combined with each image type (Color, Depth, and
 Label). Each benchmark instance should be executed using the same parameters.

 These macros guarantee that a benchmark is declared, dispatches the right
 benchmark harness and is executed with a common set of parameters.

 The macro is invoked as follows:

   MAKE_ALL_BENCHMARKS(Foo, ImageType)

 such that there must be a `EngineType::Foo` enum and ImageType must be one of
 (Color, Depth, or Label). Capitalization matters.

 N.B. The macro STR converts a single macro parameter into a string and we use
 it to make a string out of the concatenation of two macro parameters (i.e., we
 get FooColor out of the parameters Foo and Color).

 The benchmark arguments can vary from fixture to fixture. See the fixture
 details to see what the interpretation of the argument values is. */
#define STR(s) #s

// The boilerplate for defining a benchmark. The result can immediately have
// configuration APIs invoked.
#define DEFINE_BENCHMARK(Fixture, Renderer, ImageT)                    \
  BENCHMARK_DEFINE_F(Fixture, Renderer##ImageT)                        \
  (benchmark::State & state) {                                         \
    ImageT##Image<EngineType::Renderer>(state, STR(Renderer##ImageT)); \
  }                                                                    \
  BENCHMARK_REGISTER_F(Fixture, Renderer##ImageT)->Unit(benchmark::kMillisecond)

// TODO(SeanCurtis-TRI): Not all benchmarks make sense for all image types. It
// would be nice if we simply didn't declare tests for those image types.

// Each benchmark has its own state encoding -- see the named benchmark fixture
// for the interpretation of each parameter group.
#define MAKE_ALL_BENCHMARKS(Renderer, ImageT)                             \
  DEFINE_BENCHMARK(LightingBenchmark, Renderer, ImageT)                   \
      ->ArgsProduct({{1}, {640}, {480}, {1, 2, 3, 4, 5}})                 \
      ->ArgsProduct({{1}, {1280}, {960}, {1, 2, 3, 4, 5}})                \
      ->ArgsProduct({{1}, {2560}, {1920}, {1, 2, 3, 4, 5}})               \
      ->ArgsProduct({{960}, {2560}, {1920}, {1, 2, 3, 4, 5}});            \
                                                                          \
  DEFINE_BENCHMARK(ReadbackBenchmark, Renderer, ImageT)                   \
      ->ArgsProduct({{1, 960}, {10}, {640}, {480}})                       \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {1}, {640}, {480}})     \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {1}, {320}, {240}})     \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {1}, {1280}, {960}})    \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {1}, {2560}, {1920}});  \
                                                                          \
  DEFINE_BENCHMARK(TextureBenchmark, Renderer, ImageT)                    \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {640}, {480}, {0, 1}})  \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {1280}, {960}, {0, 1}}) \
      ->ArgsProduct({{1, 60, 120, 240, 480, 960}, {2560}, {1920}, {0, 1}});

// TODO(SeanCurtis-TRI): This compares the obj with a single texture with a glTF
// with *a lot* of textures. So, I'm not just measuring PBR shader complexity,
// but also model complexity. I need to simplify the glTF for this comparison
// and defer the model complexity to a different benchmark.
#define MAKE_VTK_ONLY_BENCHMARKS                                     \
  DEFINE_BENCHMARK(PbrVsPhongBenchmark, Vtk, Color)                  \
      ->ArgsProduct({{1, 10, 20, 40, 80}, {640}, {480}, {1, 2, 3}})  \
      ->ArgsProduct({{1, 10, 20, 40, 80}, {1280}, {960}, {1, 2, 3}}) \
      ->ArgsProduct({{1, 10, 20, 40, 80}, {2560}, {1920}, {1, 2, 3}});

MAKE_ALL_BENCHMARKS(Vtk, Color);
MAKE_VTK_ONLY_BENCHMARKS;
// MAKE_ALL_BENCHMARKS(Vtk, Depth);
// MAKE_ALL_BENCHMARKS(Vtk, Label);

#ifndef __APPLE__
MAKE_ALL_BENCHMARKS(Gl, Color);
// MAKE_ALL_BENCHMARKS(Gl, Depth);
// MAKE_ALL_BENCHMARKS(Gl, Label);
#endif

}  // namespace
}  // namespace geometry
}  // namespace drake
