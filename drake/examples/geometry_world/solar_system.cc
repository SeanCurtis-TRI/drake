#include "drake/examples/geometry_world/solar_system.h"

#include <memory>
#include <vector>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/math/axis_angle.h"

namespace drake {
namespace examples {
namespace solar_system {

using Eigen::Vector4d;
using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::VisualMaterial;
using geometry::Sphere;
using systems::Context;
using systems::BasicVector;
using std::make_unique;

template <typename T>
SolarSystem<T>::SolarSystem(GeometrySystem<T>* geometry_system) {
  source_id_ = geometry_system->RegisterSource("solar_system");
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(&SolarSystem::AllocateFrameIdOutput,
                                      &SolarSystem::CalcFrameIdOutput)
          .get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(&SolarSystem::AllocateFramePoseOutput,
                                      &SolarSystem::CalcFramePoseOutput)
          .get_index();

  VectorX<T> initial_state;
  initial_state.resize(kBodyCount * 2);
  // clang-format off
  initial_state << 0,               // earth initial position
                   M_PI / 2,        // moon initial position
                   -M_PI / 2,       // mars initial position
                   0,               // phobos initial position
                   2 * M_PI / 5,    // earth revolution lasts 5 seconds.
                   2 * M_PI,        // moon revolution lasts 1 second.
                   2 * M_PI / 6,    // mars revolution lasts 6 seconds.
                   2 * M_PI * 0.9;  // phobos revolution lasts 0.9 seconds.
  // clang-format on
  this->DeclareContinuousState(BasicVector<T>(initial_state),
                               kBodyCount /* num_q */, kBodyCount /* num_v */,
                               0 /* num_z */);

  AllocateGeometry(geometry_system);
}

template <typename T>
SolarSystem<T>::~SolarSystem() {}

template <typename T>
const systems::OutputPort<T>& SolarSystem<T>::get_geometry_id_output_port()
    const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const systems::OutputPort<T>& SolarSystem<T>::get_geometry_pose_output_port()
    const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
void SolarSystem<T>::AllocateGeometry(GeometrySystem<T>* geometry_system) {
  body_ids_.reserve(kBodyCount);
  initial_poses_.reserve(kBodyCount);
  axes_.reserve(kBodyCount);

  // Define the axes of rotation
  // Allocate the sun.
  geometry_system->RegisterAnchoredGeometry(
      source_id_, std::make_unique<GeometryInstance>(
                      Isometry3<double>::Identity(), make_unique<Sphere>(1.f),
                      VisualMaterial(Vector4d(1, 1, 0, 1))));

  // Allocate the "celestial bodies": two planets orbiting on different planes,
  // each with a moon.
  // Earth
  FrameId planet_id = geometry_system->RegisterFrame(
      source_id_, GeometryFrame("earth", Isometry3<double>::Identity()));
  body_ids_.push_back(planet_id);
  initial_poses_.push_back(Isometry3<double>::Identity());
  axes_.push_back(Vector3<double>::UnitZ());

  const double kEarthOrbitRadius = 3.0;
  Isometry3<double> earth_pose = Isometry3<double>::Identity();
  earth_pose.translation() << kEarthOrbitRadius, 0, 0;
  geometry_system->RegisterGeometry(
      source_id_, planet_id,
      std::make_unique<GeometryInstance>(earth_pose, make_unique<Sphere>(0.25f),
                                         VisualMaterial(Vector4d(0, 0, 1, 1))));
  // Earth Moon
  FrameId moon_id = geometry_system->RegisterFrame(
      source_id_, planet_id, GeometryFrame("moon", earth_pose));
  body_ids_.push_back(moon_id);
  initial_poses_.push_back(earth_pose);
  Vector3<double> plane_normal;
  plane_normal << 1, 1, 1;
  axes_.push_back(plane_normal.normalized());

  const double kMoonOrbitRadius = 0.35;
  Isometry3<double> moon_pose = Isometry3<double>::Identity();
  // Pick a position at kMoonOrbitRadius distance from the earth's origin on
  // the plane _perpendicular_ to the moon's normal (axes_.back()).
  Vector3<double> moon_position(-1, 0.5, 0.5);
  moon_pose.translation() = moon_position.normalized() * kMoonOrbitRadius;
  geometry_system->RegisterGeometry(
      source_id_, moon_id, std::make_unique<GeometryInstance>(
                               moon_pose, make_unique<Sphere>(0.075f),
                               VisualMaterial(Vector4d(0.5, 0.5, 0.35, 1))));

  // Mars
  planet_id = geometry_system->RegisterFrame(
      source_id_, GeometryFrame("mars", Isometry3<double>::Identity()));
  body_ids_.push_back(planet_id);
  initial_poses_.push_back(Isometry3<double>::Identity());
  plane_normal << .1, .1, 1;
  axes_.push_back(plane_normal.normalized());

  const double kMarsOrbitRadius = 5.0;
  Isometry3<double> mars_pose = Isometry3<double>::Identity();
  // Pick a position at kMarsOrbitRadius distance from the sun's origin on
  // the plane _perpendicular_ to mars's normal (axes_.back()).
  Vector3<double> mars_position(1, 1, -.2);
  mars_pose.translation() = mars_position.normalized() * kMarsOrbitRadius;
  geometry_system->RegisterGeometry(
      source_id_, planet_id, std::make_unique<GeometryInstance>(
                                 mars_pose, make_unique<Sphere>(0.24f),
                                 VisualMaterial(Vector4d(0.9, 0.1, 0, 1))));
  // Mars moon - have it rotate in the opposite direction
  Isometry3<T> phobos_rotation_pose = Isometry3<double>::Identity();
  phobos_rotation_pose.linear() =
      AngleAxis<T>(M_PI / 2, Vector3<T>::UnitX()).matrix();
  moon_id = geometry_system->RegisterFrame(
      source_id_, planet_id, GeometryFrame("phobos", phobos_rotation_pose));
  body_ids_.push_back(moon_id);
  initial_poses_.push_back(mars_pose);
  plane_normal << 0, 0, 1;
  axes_.push_back(plane_normal.normalized());

  const double kPhobosOrbitRadius = 0.34;
  Isometry3<double> phobos_pose = Isometry3<double>::Identity();
  phobos_pose.translation() << kPhobosOrbitRadius, 0, 0;
  geometry_system->RegisterGeometry(
      source_id_, moon_id, std::make_unique<GeometryInstance>(
                               phobos_pose, make_unique<Sphere>(0.06f),
                               VisualMaterial(Vector4d(0.65, 0.6, 0.8, 1))));

  DRAKE_DEMAND(static_cast<int>(body_ids_.size()) == kBodyCount);
}

template <typename T>
FramePoseVector<T> SolarSystem<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  return FramePoseVector<T>(source_id_, initial_poses_);
}

template <typename T>
void SolarSystem<T>::CalcFramePoseOutput(const Context<T>& context,
                                         FramePoseVector<T>* poses) const {
  const BasicVector<T>& state = get_state(context);
  DRAKE_ASSERT(poses->vector().size() == body_ids_.size());
  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  for (size_t i = 0; i < body_ids_.size(); ++i) {
    // Frames only revolve around their origin; it is only necessary to set the
    // rotation value.
    T rot{state[i]};
    pose_data[i].linear() = AngleAxis<T>(rot, axes_[i]).matrix();
  }
}

template <typename T>
FrameIdVector SolarSystem<T>::AllocateFrameIdOutput(const MyContext&) const {
  FrameIdVector ids(source_id_);
  ids.AddFrameIds(body_ids_);
  return ids;
}

template <typename T>
void SolarSystem<T>::CalcFrameIdOutput(const MyContext&, FrameIdVector*) const {
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
void SolarSystem<T>::DoCalcTimeDerivatives(
    const MyContext& context, MyContinuousState* derivatives) const {
  const BasicVector<T>& state = get_state(context);
  BasicVector<T>* derivative_vector = get_mutable_state(derivatives);
  derivative_vector->SetZero();
  derivative_vector->get_mutable_value().head(kBodyCount) =
      state.get_value().tail(kBodyCount);
}

template class SolarSystem<double>;

}  // namespace solar_system
}  // namespace examples
}  // namespace drake
