#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/geometry_world.h"

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

  /** Adds an input port to accept frame kinematics data from an upstream
   geometry source (see GeometryWorld for definition of "geometry source").
   @throws  std::logic_error if a context has been allocated for this system. */
  const systems::InputPortDescriptor<T>& AddSourceInput(
      const std::string& name = "");

  /** Reports the source id associated with the previously requested source
   input port. If the `port_descriptor` does not match a previously added source
   input, an exception is thrown. */
  SourceId get_port_source_id(
      const systems::InputPortDescriptor<T>& port_descriptor) const;

  /** Reports the source id associated with the previously requested source
   input port index. If the `port_index` does not match a previously added
   source input, an exception is thrown. */
  SourceId get_port_source_id(int port_index) const;

  /** Updates the state of all geometry in its geometry world by pulling pose
   information from input ports, providing an updated GeometryQuery on the
   output. */
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

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

  /** Registers a new geometry source to GeometryWorld, receiving the unique
  identifier for this new source.
  @param name          The optional name of the source. If none is provided
                       it will be named Source## where the number is the
                       value of the returned SourceId.
  @throws std::logic_error if the name duplicates a previously registered
                           source name or if a context has been allocated. */
  SourceId RegisterNewSource(const std::string& name = "");

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
  FrameId RegisterFrame(systems::Context<T>* context, SourceId source_id,
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
  FrameId RegisterFrame(systems::Context<T>* context, SourceId source_id,
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
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
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
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
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
      systems::Context<T>* context,
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
  void ClearSource(systems::Context<T>* context, SourceId source_id);

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
  void RemoveFrame(systems::Context<T>* context, SourceId source_id,
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
  void RemoveGeometry(systems::Context<T>* context, SourceId source_id,
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
  const std::string& get_source_name(const systems::Context<T>& context,
                                     SourceId id) const;

  /** Reports if the given source id is registered.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::SourceIsRegistered() for details. */
  bool SourceIsRegistered(const systems::Context<T>& context,
                          SourceId id) const;

  /** Creates a frame kinematics set for the given source id.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::GetFrameKinematicsSet() for details. */
  FrameKinematicsSet<T> GetFrameKinematicsSet(
      const systems::Context<T>& context, SourceId source_id) const;

  /** Reports the frame to which this geometry is registered.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::GetFrameId() for details. */
  FrameId GetFrameId(const systems::Context<T>& context,
                     GeometryId geometry_id) const;

  /** Determines contacts across all geometries in GeometryWorld.
   @param   context   The context of a sibling system to `this` %GeometrySystem.
   See GeometryWorld::ComputeContact() for details. */
  bool ComputeContact(const systems::Context<T>& context,
                      std::vector<Contact<T>>* contacts) const;

  // TODO(SeanCurtis-TRI): Flesh this out with the full set of queries.

  //@}

 private:
  // Override of construction to account for
  //    - instantiating a GeometryContext instance (as opposed to LeafContext),
  //    - modifying the state to prevent additional sources being added. */
  std::unique_ptr<systems::Context<T>> MakeContext() const override;

  // Given a sibling context, extracts a mutable instance of the geometry state.
  GeometryState<T>* ExtractMutableStateViaSiblingContext(
      systems::Context<T>* context);

  // Given a const sibling context, extracts a const instance of the geometry
  // state.
  const GeometryState<T>& ExtractStateViaSiblingContext(
      const systems::Context<T>& context) const;

  // Given a mutable *sibling* context, extracts a mutable GeometryContext for
  // this system from the parent diagram context. Throws an exception if such a
  // context cannot be found.
  GeometryContext<T>* get_mutable_context(const systems::Context<T>* context);

  // Given a *sibling* context, extracts a GeometryContext for this system from
  // the parent diagram context. Throws an exception if such a context cannot be
  // found.
  const GeometryContext<T>& get_context(
      const systems::Context<T>& context) const;

  // Extracts a mutable geometry state from the given GeometryContext.
  // Ultimately, this should be a private utility method, but it is made public
  // to facilitate testing.
  GeometryState<T>* get_mutable_state(GeometryContext<T>* context);

  // Extracts a read-only geometry state from the given GeometryContext.
  // Ultimately, this should be a private utility method, but it is made public
  // to facilitate testing.
  const GeometryState<T>& get_state(const GeometryContext<T>& context) const;

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
  mutable GeometryState<T>* initial_state_;

  // A mapping from added source input ports and the source ids assigned to
  // them. It relies on the fact that the index of an input port will match the
  // position in input_source_ids_;
  std::vector<SourceId> input_source_ids_;
};

}  // namespace geometry
}  // namespace drake
