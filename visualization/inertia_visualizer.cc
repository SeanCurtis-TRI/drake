#include "drake/visualization/inertia_visualizer.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>

namespace drake {
namespace visualization {

using Eigen::Vector3d;
using geometry::Ellipsoid;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Rgba;
using geometry::SceneGraph;
using math::RigidTransform;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;

namespace {
// Maps a value x in [minimum, maximum] to a color in the plasma colormap.
Rgba MapPlasmaColor(double minimum, double maximum, double x) {
  // Clamp and normalize x to [0, 1]
  double t = (maximum > minimum) ? (x - minimum) / (maximum - minimum) : 0.0;
  t = std::min(std::max(t, 0.0), 1.0);
  // Plasma colormap approximation (from matplotlib's plasma)
  // See: https://github.com/BIDS/colormap/blob/master/colormaps.py
  double r = std::clamp(2.0 * t - 1.5 * t * t, 0.0, 1.0);
  double g = std::clamp(0.5 * t + 0.5 * t * t, 0.0, 1.0);
  double b = std::clamp(1.0 - t + 0.5 * t * t, 0.0, 1.0);
  // These are not exact, but provide a reasonable plasma-like gradient.
  return Rgba{r, g, b, 1.0};
}

IllustrationProperties MakeProperties(const Rgba& rgba = Rgba(0, 0, 1)) {
  IllustrationProperties props;
  props.AddProperty("meshcat", "accepting", "inertia");
  // We set inherent opacity (alpha) to 1. This means if InertiaVisualizer is
  // used in a viewer without a slider control for opacity, the inertia
  // geometry will be permanently visible (but does allow us to use
  props.AddProperty("phong", "diffuse", rgba);
  return props;
}

// Iterates through the bodies in a consistent way, skipping those welded to the
// world body. For all other bodies, the functor is invoked.
template <typename T>
void ProcessBodies(const MultibodyPlant<T>& plant,
                   std::function<void(const RigidBody<T>&)> f) {
  const std::vector<const RigidBody<T>*> world_bodies =
      plant.GetBodiesWeldedTo(plant.world_body());
  const int num_bodies = plant.num_bodies();
  for (multibody::BodyIndex i{0}; i < num_bodies; ++i) {
    bool welded = false;
    for (const auto* world_body : world_bodies) {
      if (world_body->index() == i) {
        welded = true;
        break;
      }
    }
    if (welded) {
      continue;
    }
    f(plant.get_body(i));
  }
}

}  // namespace

template <typename T>
InertiaVisualizer<T>::InertiaVisualizer(
    const MultibodyPlant<T>& plant, SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource("inertia_visualizer");

  double min_mass = std::numeric_limits<double>::infinity();
  double max_mass = -std::numeric_limits<double>::infinity();
  auto plant_context = plant.CreateDefaultContext();
  auto get_mass = [&plant, &plant_context](const RigidBody<T>& body) {
    const SpatialInertia<T> M_BBo_B =
        body.CalcSpatialInertiaInBodyFrame(*plant_context);
    return ExtractDoubleOrThrow(M_BBo_B.get_mass());
  };
  ProcessBodies<T>(plant,
                [&min_mass, &max_mass, &get_mass](const RigidBody<T>& body) {
                  const double mass = get_mass(body);
                  if (mass < min_mass) min_mass = mass;
                  if (mass > max_mass) max_mass = mass;
                });

  auto make_item = [this, &plant, &scene_graph, min_mass, max_mass,
                    &get_mass](const RigidBody<T>& body) {
    // Add a Bcm geometry frame.
    Item item;
    item.body = body.index();
    item.Bo_frame = plant.GetBodyFrameIdIfExists(body.index()).value();
    item.Bcm_frame = scene_graph->RegisterFrame(
        source_id_, SceneGraph<T>::world_frame_id(),
        GeometryFrame{fmt::format(
            "InertiaVisualizer::{}::{}",
            plant.GetModelInstanceName(body.model_instance()), body.name())});

    // Add an illustration ellipsoid to be placed at Bcm.
    // This ellipsoid will be replaced by a properly-sized geometry in the
    // subsequent call to UpdateItems() so its specifics don't matter.
    auto ellipsoid = std::make_unique<Ellipsoid>(0.001, 0.001, 0.001);
    auto geom = std::make_unique<GeometryInstance>(
        RigidTransform<double>(), std::move(ellipsoid),
        fmt::format("$inertia({})", body.index()));
    geom->set_illustration_properties(MakeProperties(
        MapPlasmaColor(min_mass, max_mass, get_mass(body))));
    item.geometry = scene_graph->RegisterGeometry(source_id_, item.Bcm_frame,
                                                  std::move(geom));
    items_.push_back(std::move(item));
  };
  ProcessBodies<T>(plant, make_item);

  // Update the geometry information to reflect the initial inertia values
  // from the plant.
  UpdateItems(plant, *plant_context, scene_graph);

  this->DeclareAbstractInputPort("plant_geometry_pose",
                                 Value<FramePoseVector<T>>());
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &InertiaVisualizer<T>::CalcFramePoseOutput);
}

template <typename T>
template <typename U>
InertiaVisualizer<T>::InertiaVisualizer(const InertiaVisualizer<U>& other)
    : source_id_{other.source_id_}, items_{other.items_} {}

template <typename T>
const InertiaVisualizer<T>& InertiaVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  auto result =
      builder->template AddSystem<InertiaVisualizer<T>>(plant, scene_graph);
  result->set_name("inertia_visualizer");
  builder->Connect(plant.get_geometry_pose_output_port(),
                   result->get_input_port());
  builder->Connect(result->get_output_port(),
                   scene_graph->get_source_pose_port(result->source_id_));
  return *result;
}

template <typename T>
InertiaVisualizer<T>::~InertiaVisualizer() = default;

// TODO(trowell-tri): This method also needs to run whenever mass or inertia
// changes in the plant.
template <typename T>
void InertiaVisualizer<T>::UpdateItems(
    const MultibodyPlant<T>& plant, const Context<T>& plant_context,
    SceneGraph<T>* scene_graph) {
  for (auto& item : items_) {
    const RigidBody<T>& body = plant.get_body(item.body);

    auto [ellipsoid, X_BBcm] =
        internal::CalculateInertiaGeometry(body, plant_context);
    item.X_BBcm = X_BBcm;
    scene_graph->ChangeShape(source_id_, item.geometry, ellipsoid);
  }
}

template <typename T>
void InertiaVisualizer<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  const auto& plant_poses =
      this->get_input_port().template Eval<FramePoseVector<T>>(context);

  poses->clear();
  for (const auto& item : items_) {
    const RigidTransform<T>& X_WBo = plant_poses.value(item.Bo_frame);
    const RigidTransform<T> X_WBcm = X_WBo * item.X_BBcm.template cast<T>();
    poses->set_value(item.Bcm_frame, X_WBcm);
  }
}

namespace internal {

// TODO(jwnimmer-tri) Add a system Parameter to configure whether to display
// ellipsoids or boxes as the shape, and whether or not to rescale the shape
// to match the body's mass.
template <typename T>
std::pair<Ellipsoid, RigidTransform<double>> CalculateInertiaGeometry(
    const RigidBody<T>& body, const Context<T>& plant_context) {
  // Interrogate the plant context for the spatial inertia of body B about its
  // origin Bo, expressed in body frame B.
  const SpatialInertia<T> M_BBo_B =
      body.CalcSpatialInertiaInBodyFrame(plant_context);

  RigidTransform<double> X_BE;
  Vector3<double> radii;
  std::tie(radii, X_BE) =
      M_BBo_B.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid();
  // Everything must be visualized; so we need a 3D shape from what may possible
  // be a 2D, 1D, or 0D inertia. So, we'll enforce that all measures are a
  // minimum size.
  radii = radii.array().max(0.001);

  // For a massless body, use a very tiny sphere (though expressed as an
  // ellipsoid for consistency).
  const double mass = ExtractDoubleOrThrow(M_BBo_B.get_mass());
  if (mass == 0) {
    return std::make_pair(
        Ellipsoid(radii),
        RigidTransform<double>(ExtractDoubleOrThrow(M_BBo_B.get_com())));
  }

  // An ellipsoid that has one of its diameters >100x greater than another one
  // does not render well on the screen, so we'll make sure no dimension of this
  // unit inertia's ellipsoid is smaller than 1/100 of its maximum dimension.
  radii = radii.array().max(1e-2 * radii.maxCoeff());
  return std::make_pair(Ellipsoid(radii), X_BE);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&CalculateInertiaGeometry<T>));

}  // namespace internal
}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer);
