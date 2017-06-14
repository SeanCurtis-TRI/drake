#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/geometry_world.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {

namespace systems { template <typename T> class DiagramContext; }
namespace geometry {

template <typename T> class GeometryContext;

/** The system wrapper for GeometryWorld. It serves as the nexus for all
 geometry in the system. Upstream System instances that introduce geometry
 into the world are responsible for registering that geometry with GeometryWorld
 and provide updated, per-frame kinematics on their outputs. Geometric queries
 are performed directly on the %GeometrySystem.

 Ultimately, this class serves as an interface for placing GeometryWorld into
 the drake system architecture. However, the nature of GeometryWorld precludes
 a perfect fit.

 Inputs

 %GeometrySystem will have one input for each unique geometry source.

 Outputs
 Single abstract output port containing an instance of GeometryQuery. Downstream
 systems can connect to this port. The value provided can be used to perform
 geometric queries on the current state of the underlying GeometryWorld.

 //TODO(SeanCurtis-TRI): Should I also have an LCM message-compatible output?

 Working with GeometrySystem
 - Extracting GeometryContext for changing topology

 @tparam T
 @see GeometryWorld
 */
template <typename T>
class GeometrySystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometrySystem)

  GeometrySystem();
  ~GeometrySystem() override;

  /** Registers a new source to the geometry system (see GeometryWorld for the
   discussion of "geometry source"). The caller must save the returned SourceId;
   it is the token by which all other operations on the geometry world are
   conducted.

   This source id can be used to register arbitrary _anchored_ geometry. But if
   dynamic geometry is registered (via RegisterGeometry/RegisterFrame), then
   the context-dependent pose values must be provided on an input port.
   See get_port_for_source_id().
   @param name          The optional name of the source. If none is provided
                        (or the empty string) it will be defined by
                        GeometryState's logic.
   @throws  std::logic_error if a context has been allocated for this system.
   @see GeometryState::RegisterNewSource() */
  SourceId RegisterSource(const std::string &name = "");

  /** Given a valid source identifier, returns an input frame id port associated
   with that `id`. This port is used to broadcast frame id order to interpret
   the poses and velocities coming in on other ports.
   @throws  std::logic_error if the source_id is _not_ recognized, or if the
   context has already been allocated.. */
  const systems::InputPortDescriptor<T>& get_source_frame_id_port(SourceId id);

  /** Given a valid source identifier, returns an input _pose_ port associated
   with that id. This port is used to communicate _pose_ data for registered
   frames.
   @throws  std::logic_error if the source_id is _not_ recognized, or if the
   context has already been allocated.. */
  const systems::InputPortDescriptor<T>& get_source_pose_port(SourceId id);

  /** Given a valid source identifier, returns an input _velocity_ port
   associated with that id. This port is used to communicate _velocity_ data for
   registered frames.
   @throws  std::logic_error if the source_id is _not_ recognized, or if the
   context has already been allocated.. */
  const systems::InputPortDescriptor<T>& get_source_velocity_port(SourceId id);

  /** @name             Topology Manipulation
   Topology manipulation consists of changing the data contained in
   GeometryWorld. This includes registering a new geometry source, adding or
   removing frames, and adding or removing geometries.

   The topology can be manipulated at one of two phases:
     - Initialization
     - Discrete updates

   The initialization phase begins with the instantiation of a %GeometrySystem
   and ends when a context is allocated by the %GeometrySystem instance. This is
   the only phase when geometry sources can be registered with GeometryWorld.
   Once a source is registered, it can register frames and geometries. Any
   frames and geometries registered during this phase become part of the
   _default_ context state for %GeometrySystem and calls to
   CreateDefaultContext() will produce identical contexts.

   The discrete update phase happens during the simulation. When geometry
   sources need to modify the topology (introduce or removing frames and
   geometries) in response to some internal state, they should request a
   discrete update event and invoke the appropriate commands to change the
   geometry data for the source.

   The two interfaces are distinguished by their parameter list. They generally,
   have the same parameter lists, but the methods to use during discrete
   updates take an additional context argument. This is the context of the
   geometry source and it _must_ be a sibling to the %GeometrySystem (in that
   they are both contained by the same diagram). The initialization methods
   do not take a context, but invoking them _after_ %GeometrySystem has
   allocated a context will result in an exception. */
  //@{

  /** Initialization registration of a new frame on this channel, receiving the
   unique id for the new frame.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   @param frame         The definition of the frame to add.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or if a context has been allocated. */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame<T>& frame);

  /** Discrete update registration of a new frame on this channel, receiving the
   unique id for the new frame.
   @param context       The context of the _caller_. The caller must be a
                        sibling system of GeometrySystem.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   @param frame         The definition of the frame to add.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. the context does not belong to a sibling system.
   */
  FrameId RegisterFrame(systems::Context<T>* sibling_context,
                        SourceId source_id,
                        const GeometryFrame<T>& frame);

  /** Initialization registration of a new frame for the given source as a child
   of a previously registered frame. The id of the new frame is returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source, or
                             3. a context has been allocated. */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame<T>& frame);

  /** Discrete update registration of a new frame for the given source as a
   child of a previously registered frame. The id of the new frame is returned.
   @param context      The context of the _caller_. The caller must be a
                       sibling system of GeometrySystem.
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source, or
                             3. the context does not belong to a sibling system.
   */
  FrameId RegisterFrame(systems::Context<T>* sibling_context,
                        SourceId source_id,
                        FrameId parent_id, const GeometryFrame<T>& frame);

  /** Initialization registration of  a `geometry` instance as "hanging" from
   the specified frame at the given pose relative to the frame. The geometry is
   _rigidly_ affixed to the parent frame.
   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` doesn't belong to the source, or
                             3. the `geometry` is equal to `nullptr`, or
                             4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Discrete update registration of a `geometry` instance as "hanging" from
   the specified frame at the given pose relative to the frame. The geometry is
   _rigidly_ affixed to the parent frame.
   @param context     The context of the _caller_. The caller must be a
                      sibling system of GeometrySystem.
   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` doesn't belong to the source, or
                             3. the `geometry` is equal to `nullptr`, or
                             4. the context does not belong to a sibling system.
   */
  GeometryId RegisterGeometry(systems::Context<T>* sibling_context,
                              SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Initialization registration of a `geometry` instance as "hanging" from the
   specified geometry's frame `F`, with the given pose relative to that frame.
   The geometry is _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry is registered.

   @param source_id    The identifier for the geometry source registering the
                       geometry.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to an active
                            source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. the `geometry` is equal to `nullptr`, or
                            4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Discrete update registration of a `geometry` instance as "hanging" from
   the specified geometry's frame `F`, with the given pose relative to that
   frame. The geometry is _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry is registered.

   @param context      The context of the _caller_. The caller must be a
                       sibling system of GeometrySystem.
   @param source_id    The identifier for the geometry source registering the
                       geometry.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to an active
                            source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. the `geometry` is equal to `nullptr`, or
                            4. the context does not belong to a sibling system.
   */
  GeometryId RegisterGeometry(systems::Context<T>* sibling_context,
                              SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Initialization registration of  the given geometry to the world as
   anchored geometry.
   @param source_id     The identifier for the geometry source registering the
                        geometry.
   @param geometry      The geometry to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or a context has been allocated. */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** Discrete update registration of the given geometry to the world as
   anchored geometry.
   @param context       The context of the _caller_. The caller must be a
                        sibling system of GeometrySystem.
   @param source_id     The identifier for the geometry source registering the
                        geometry.
   @param geometry      The geometry to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. the context does not belong to a sibling system.
   */
  GeometryId RegisterAnchoredGeometry(
      systems::Context<T>* sibling_context,
      SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** Initialization clearing of all the registered frames and geometries from
   this source, but leaves the source active for future registration of frames
   and geometries.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or if a context has been allocated. */
  void ClearSource(SourceId source_id);

  /** Discrete update clearing of all the registered frames and geometries from
   this source, but leaves the source active for future registration of frames
   and geometries.
   @param context     The context of the _caller_. The caller must be a
                      sibling system of GeometrySystem.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. the context does not belong to a sibling system.
   */
  void ClearSource(systems::Context<T>* sibling_context, SourceId source_id);

  /** Initialization removal of the given frame from the the indicated source's
   frames. All registered geometries connected to this frame will also be
   removed from the world.
   @param source_id   The identifier for the owner geometry source.
   @param frame_id    The identifier of the frame to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `frame_id` doesn't belong to the source, or
                            3. a context has been allocated. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Discrete update removal of the given frame from the the indicated source's
   frames. All registered geometries connected to this frame will also be
   removed from the world.
   @param context     The context of the _caller_. The caller must be a
                      sibling system of GeometrySystem.
   @param source_id   The identifier for the owner geometry source.
   @param frame_id    The identifier of the frame to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `frame_id` doesn't belong to the source, or
                            3. the context does not belong to a sibling system.
   */
  void RemoveFrame(systems::Context<T>* sibling_context, SourceId source_id,
                   FrameId frame_id);

  /** Initialization removal of the given geometry from the the indicated
   source's geometries. All registered geometries connected to this geometry
   will also be removed from the world.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. a context has been allocated. */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  /** Discrete update removal of the given geometry from the the indicated
   source's geometries. All registered geometries connected to this geometry
   will also be removed from the world.
   @param context     The context of the _caller_. The caller must be a
                      sibling system of GeometrySystem.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. the context does not belong to a sibling system.
   */
  void RemoveGeometry(systems::Context<T>* sibling_context, SourceId source_id,
                      GeometryId geometry_id);
  //@}

  /** @name     Geometry Queries
   These perform queries on the state of the geometry world including:
   proximity queries, contact queries, ray-casting queries, and look ups on
   geometry resources.

   These operations require a context. The caller can provide their own context
   provided they are a sibling system to the GeometrySystem interface (i.e.,
   they are contained in the same diagram).

   The details of these queries are fully specified in the documentation for
   GeometryWorld.
   */

  //@{

  /** Report the name for the given source id.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::get_source_name() for details. */
  const std::string& get_source_name(const systems::Context<T>& sibling_context,
                                     SourceId id) const;

  /** Reports if the given source id is registered.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::SourceIsRegistered() for details. */
  bool SourceIsRegistered(const systems::Context<T>& sibling_context,
                          SourceId id) const;

  /** Reports the frame to which this geometry is registered.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::GetFrameId() for details. */
  FrameId GetFrameId(const systems::Context<T>& sibling_context,
                     GeometryId geometry_id) const;

  /** Determines contacts across all geometries in GeometryWorld.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::ComputeContact() for details. */
  bool ComputeContact(const systems::Context<T>& sibling_context,
                      std::vector<Contact<T>>* contacts) const;

  // TODO(SeanCurtis-TRI): Flesh this out with the full set of queries.

  //@}

 private:
  // Aggregates the input poses into the output PoseBundle, in the order
  // the input ports were added. Aborts if any inputs have an unexpected
  // dimension.
  void CalcPoseBundle(const systems::Context<T>& context,
                      systems::rendering::PoseBundle<T>* output) const;

  // Constructs a PoseBundle of length equal to the concatenation of all inputs.
  // This is the method used by the allocator for the output port.
  systems::rendering::PoseBundle<T> MakePoseBundle(
      const systems::Context<T>& context) const;

  // Allow the load dispatch to peek into GeometrySystem.
  friend void DispatchLoadMessage(const GeometrySystem<T>&);

  // Updates the state of geometry world from the inputs. It is *declared* const
  // so it can be invoked in the const query methods.  But it has mutable
  // behavior. Part of a horrible hack.
  const GeometryContext<T>& UpdateFromInputs(
      const systems::Context<T>& sibling_context) const;

  // Override of construction to account for
  //    - instantiating a GeometryContext instance (as opposed to LeafContext),
  //    - modifying the state to prevent additional sources being added. */
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  // Given a sibling context, extracts a mutable instance of the geometry
  // context.
  GeometryContext<T>& ExtractMutableContextViaSiblingContext(
      const systems::Context<T>& sibling_context);

  // Given a const sibling context, extracts a const instance of the geometry
  // context.
  const GeometryContext<T>& ExtractContextViaSiblingContext(
      const systems::Context<T>& sibling_context) const;

  // Helper method for throwing an exception if a context has *ever* been
  // allocated by this system.
  void ThrowIfContextAllocated() const;

  // The underlying representation of the world's geometry.
  GeometryWorld<T> geometry_world_;

  // A raw pointer to the default geometry state (which serves as the model for
  // allocating contexts for this system. It will only be non-null between
  // construction and context allocation. It serves a key role in enforcing the
  // property that source ids can only be added prior to context allocation.
  // This is mutable so that it can be cleared in the const method
  // AllocateContext().
  GeometryState<T>* initial_state_;
  mutable bool context_allocated_{false};

  // Enumeration of the type of port to extract.
  enum PortType {
    ID,
    POSE,
    VELOCITY
  };

  // For the given source id, reports the id of the existing port of port_type
  // (creating it as necessary).
  const systems::InputPortDescriptor<T>& get_port_for_source_id(
      SourceId id, PortType port_type);

  // A struct that stores the port indices for a given source.
  // TODO(SeanCurtis-TRI): Consider making these Index values. This would
  // require relying on the default value.
  struct SourcePorts {
    int id_port{-1};
    int pose_port{-1};
    int velocity_port{-1};
  };

  // A mapping from added source identifier to the port indices associated with
  // that id.
  std::unordered_map<SourceId, SourcePorts> input_source_ids_;
};

}  // namespace geometry
}  // namespace drake
