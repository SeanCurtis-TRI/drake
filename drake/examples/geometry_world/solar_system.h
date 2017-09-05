#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace systems {
class SystemSymbolicInspector;
}  // namespace systems

namespace examples {
namespace solar_system {

/** A model of an orrey -- a simple mechanical model of the solar system.

 The orrey contains one sun and four orbiting bodies: two planets (Earth and
 Mars) each with one moon. The orrey is articulated by placing the _frame_ for
 each body at its parent's origin, and then displacing the geometry from that
 origin to its orbital distance. Then each orbiting frame has a single degree of
 freedom: its angular position around its axis of rotation.

 - The sun is stationary -- an anchored geometry.
 - Earth orbits on the xy-plane. Its moon revolves around the earth on an
 different arbitrary plane (illustrating transform compositions).
 - Mars orbits the sun at a farther distance on a plane that is tilted off of
 the xy-plane. Its moon (Phobos) orbits around Mars on a plane parallel to
 Mars's orbital plane.

 This system illustrates the following features:

 1. Registering anchored geometry.
 2. Registering frames as children of other frames.
 3. Creating a fixed FrameIdVector output.
 4. Updating the context-dependent FramePoseVector output.

 @tparam T The vector element type, which must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double */
template <typename T>
class SolarSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolarSystem)

  explicit SolarSystem(geometry::GeometrySystem<T>* geometry_system);
  ~SolarSystem() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& get_geometry_id_output_port() const;
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  // The default leaf system zeros out all of the state as "default" behavior.
  // This subverts that (as a quick test) to make sure that's the source of my
  // problems. The long term solution is to make sure SetDefaultState uses the
  // model values if they exist (and zeros otherwise).
  // TODO(SeanCurtis-TRI): Kill this override when LeafSystem::SetDefaultState()
  // pulls from the models instead of simply zeroing things out.
  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>*) const override {}

 protected:
  // No inputs implies no feedthrough; this makes it explicit.
  bool DoHasDirectFeedthrough(const systems::SystemSymbolicInspector*, int,
                              int) const override {
    return false;
  }

 private:
  // Allocate all of the geometry.
  void AllocateGeometry(geometry::GeometrySystem<T>* geometry_system);

  // Allocate the frame pose set output port value.
  geometry::FramePoseVector<T> AllocateFramePoseOutput(
      const MyContext& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           geometry::FramePoseVector<T>* poses) const;

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(const MyContext& context) const;
  // Calculate the id output.
  void CalcFrameIdOutput(const MyContext& context,
                         geometry::FrameIdVector* id_set) const;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const systems::BasicVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
  }

  static systems::BasicVector<T>* get_mutable_state(MyContinuousState* cstate) {
    return dynamic_cast<systems::BasicVector<T>*>(cstate->get_mutable_vector());
  }

  static const systems::BasicVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  // Interaction with geometry world
  geometry::SourceId source_id_{};

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};

  // Solar system specification
  const int kBodyCount = 4;
  std::vector<geometry::FrameId> body_ids_;
  std::vector<Vector3<double>> axes_;
  std::vector<Isometry3<double>> initial_poses_;
};

}  // namespace solar_system
}  // namespace examples
}  // namespace drake
