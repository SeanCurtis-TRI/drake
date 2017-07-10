#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/query_handle.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/geometry_world.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryContext;

/** GeometrySystem serves as a system-level wrapper for GeometryWorld. It serves
 as the nexus for all geometry (and geometry-based operations) in the system.
 Through GeometrySystem, other systems that introduce geometry can _register_
 that geometry as part of a common global domain, including it in geometric
 queries (e.g., cars controlled by one LeafSystem can be observed by a different
 sensor system). GeometrySystem provides the interface for registering the
 geometry, updating its position based on the current context, and performing
 geometric queries.

 Only registered "geometry sources" can introduce geometry into %GeometrySystem.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose values for all of the geometries the
 source owns, via a port connection on %GeometrySystem.

 The basic workflow for interacting with %GeometrySystem is:
   - Register as a geometry source, acquiring a unique SourceId.
   - Register geometry (anchored and dynamic) with the system.
   - Connect source's geometry output ports to the corresponding %GeometrySystem
     input ports.
     - Implement appropriate `Calc*` methods on the geometry output ports to
       update geometry pose values.

 @section geom_sys_inputs Inputs
 @cond
 In future versions, this will *also* include velocity and (possibly)
 acceleration ports.
 // TODO(SeanCurtis-TRI): Modify this to reflect the number of actual port
 // types.
 @endcond

 For each registered geometry source, there are _two_ input ports: id and pose.
 Failing to connect to those ports or providing "bad" values on
 those ports will cause runtime errors to be thrown. The two ports work in
 tandem. Through these ports, the upstream source system communicates the
 poses of all of the _frames_ it has registered with %GeometrySystem (see
 RegisterFrame() for more details).

 __identifier port__: An abstract-valued port containing an instance of
 FrameIdVector. It should contain the FrameId of each frame registered by the
 upstream source exactly once. The _order_ of the ids is how the values in the
 pose port will be interpreted.

 __pose port__: An abstract-valued port containing an instance of FramePoseSet.
 There should be one pose value for each id in the the identifier port value.
 The iᵗʰ pose belongs to the iᵗʰ id.

 @section geom_sys_outputs Outputs

 %GeometrySystem has two output ports:.

 __query port__: An abstract-valued port containing an instance of QueryHandle.
 It provides a "ticket" for downstream LeafSystem instances to perform geometric
 queries on the %GeometrySystem. To perform geometric queries, downstream
 LeafSystem instances must have a pointer to the %GeometrySystem, and connect
 an input port to this output port. The `const QueryHandle` reference returned
 by evaluating the input port is provided as an argument to query methods
 defined on the %GeometrySystem pointer.

 __lcm visualization port__: An abstract-valued port containing an instance of
 PoseBundle. This is a convenience port designed to feed LCM update messages to
 director for the purpose of visualizing the state world's geometry. Additional
 uses of this port are strongly discouraged; instead, use an appropriate
 geometric query to obtain the state of the world's geometry.

 @section geom_sys_workflow Working with GeometrySystem

 LeafSystem instances can relate to GeometrySystem in one of two ways: as a
 consumer that _performs_ queries, or as a producer that introduces geometry
 into the shared world and defines its context-dependent kinematics values.
 It is reasonable for systems to perform either role singly, or both.

 __Consumer__

 Consumers perform geometric queries upon the world geometry. %GeometrySystem
 _serves_ those queries. As indicated above, in order for a LeafSystem to act
 as a consumer, it must:
   1. define a QueryHandle-valued input port and connect it to %GeometrySystem's
   corresponding output port, and
   2. have a reference to the connected %GeometrySystem instance.

 With those two requirements satisfied, a LeafSystem can perform geometry
 queries by:
   1. evaluating the QueryHandle input port, and
   2. passing the returned handle into the appropriate query method on
   GeometrySystem (e.g., GeometrySystem::ComputeContact()) and then process
   the results.

 __Producer__

 All producers introduce geometry into the shared geometric world. This is
 called _registering_ geometry. Depending on what exactly has been registered,
 a producer may also have to _update kinematics_. Producers themselves must be
 registered with %GeometrySystem as producers (a.k.a. _geometry sources_). They
 do this by acquiring a SourceId (via GeometrySystem::RegisterSource()). The
 SourceId serves as a unique handle through which the producer's identity is
 validated and its ownership of its registered geometry is maintained.

 _Registering Geometry_

 %GeometrySystem cannot know what belongs in the shared world. It must
 be informed of what the world contains by other systems. Defining the geometry
 in the world and informing the %GeometrySystem is called _registering_ the
 geometry. Geometry can be registered as _anchored_ or _dynamic_. The
 source that registers the geometry _owns_ the geometry; operations that change
 the geometry, or its frames, requires the SourceId used to register it.

 Dynamic geometry can move; more specifically, its kinematics (e.g., pose)
 depends on a system's Context. Particularly, dynamic geometry is
 _fixed_ to a _frame_ whose kinematics values depend on a context. As the frame
 moves, the geometries fixed to it move with it. Therefore, to register dynamic
 geometry a frame must be registered first. These registered frames serve as the
 basis for repositioning geometry in the shared world. The geometry source is
 responsible for providing up-to-date kinematics values for those registered
 frames upon request (via an appropriate output port on the source LeafSystem
 connecting to the appropriate input port on %GeometrySystem). The work flow is
 as follows:
   1. Source LeafSystem acquires a SourceId (GeometrySource::RegisterSource()).
   2. Source registers a frame (GeometrySource::RegisterFrame()).
     - A frame always has a "parent" frame. It can implicitly be the world
     frame, _or_ another frame registered by the source.
   3. Register one or more geometries to a frame
   (GeometrySource::RegisterGeometry()).
     - The registered geometry is posed relative to the frame to which it is
     fixed.
     - The geometry can also be posed relative to another registered geometry.
     It will be affixed to _that_ geometry's frame.

 Anchored geometry is _independent_ of the context (i.e., it doesn't move).
 Anchored geometries are always affixed to the immobile world frame. As such,
 registering a frame is _not_ required for registering anchored geometry
 (see GeometrySource::RegisterAnchoredGeometry()). However, the source still
 owns the anchored geometry.

 _Updating Kinematics_

 Registering _dynamic_ geometry implies a contract between the geometry source
 and %GeometrySystem. The geometry source must do the following:
   - It must provide, populate, and connect two output ports: the "id" port and
   the "pose" port.
   - The id port must contain _all_ the frame ids returned as a result of frame
   registration.
   - The pose port must contain one pose per registered frame; the pose is
   expressed relative to the registered frame's _parent_ frame.

 Failure to meet these requirements will lead to a run-time error.

 @cond
 // TODO(SeanCurtis-TRI): Future work which will require add'l documentation:
 //   - velocity kinematics.
 //   - Finalizing API for topology changes at discrete events.
 @endcond

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 @see GeometryWorld
 */
template <typename T>
class GeometrySystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometrySystem)

  GeometrySystem();
  ~GeometrySystem() override;

  /** @name       Port management
   Access to GeometrySystem's input/output ports. This topic includes
   registration of geometry sources because the input ports are mapped to
   registered geometry sources.

   Registration of a source doesn't automatically create all of the input ports
   for the source. Input ports are allocated on a per-request basis. However,
   a source that registers frames and geometries _must_ connect outputs to
   the inputs associated with that source. Failure to do so will be treated as
   a runtime error during the evaluation of %GeometrySystem. %GeometrySystem
   will detect that frames have been registered but no values have been provided
   via the appropriate input port.
   */
  //@{

  /** Registers a new source to the geometry system (see GeometryWorld for the
   discussion of "geometry source"). The caller must save the returned SourceId;
   it is the token by which all other operations on the geometry world are
   conducted.

   This source id can be used to register arbitrary _anchored_ geometry. But if
   dynamic geometry is registered (via RegisterGeometry/RegisterFrame), then
   the context-dependent pose values must be provided on an input port.
   See get_source_frame_id_port() and get_source_pose_port().
   @param name          The optional name of the source. If none is provided
                        (or the empty string) a unique name will be defined by
                        GeometrySystem's logic.
   @throws  std::logic_error if a context has already been allocated for this
                             %GeometrySystem.
   @see GeometryState::RegisterNewSource() */
  SourceId RegisterSource(const std::string &name = "");

  /** Given a valid source `id`, returns frame id-valued input port which
   belongs to that `id`. This port's value is an ordered list of frame ids; it
   is used to provide an interpretation on the pose values provide don the
   pose port.
   @throws  std::logic_error if the source_id is _not_ recognized, or if the
   context has already been allocated.. */
  const systems::InputPortDescriptor<T>& get_source_frame_id_port(SourceId id);

  /** Given a valid source `id`, returns an input _pose_ port associated
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

  /** Returns the output port which produces the PoseBundle for LCM
   communication to drake visualizer. */
  const systems::OutputPort<T>& get_pose_bundle_output_port() const {
    return systems::System<T>::get_output_port(bundle_port_index_);
  }

  /** Returns the output port which produces the QueryHandle for performing
   geometric queries. */
  const systems::OutputPort<T>& get_query_output_port() const {
    return systems::System<T>::get_output_port(query_port_index_);
  }

  //@}

  /** @name             Topology Manipulation
   Topology manipulation consists of changing the data contained in
   GeometryWorld. This includes registering a new geometry source, adding or
   removing frames, and adding or removing geometries.

   Currently, the topology can only be manipulated at initialization.
   Eventually, the API will expand to include modifications of the topology
   during discrete updates.

   The initialization phase begins with the instantiation of a %GeometrySystem
   and ends when a context is allocated by the %GeometrySystem instance. This is
   the only phase when geometry sources can be registered with GeometryWorld.
   Once a source is registered, it can register frames and geometries. Any
   frames and geometries registered during this phase become part of the
   _default_ context state for %GeometrySystem and calls to
   CreateDefaultContext() will produce identical contexts.

   Every geometry must be associated with a frame. It is rigidly affixed to the
   frame. As the frame moves, so does the geometry. Colloquially, this is
   referred to as "hanging" a geometry on a frame.
   */
  //@{

  /** Registers a new frame F on for this source. The frame hangs on the world
   frame (W). Its pose is defined relative to the world frame (i.e, `X_WF`).
   Returns the corresponding unique frame id.
   @param source_id     The id for the source registering the frame.
   @param frame         The definition of the frame to add.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or if a context has been allocated. */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame<T>& frame);

  /** Registers a new frame F for this source. The frame hangs on another
   previously registered frame P (indicated by `parent_id`). The pose of the new
   frame is defined relative to the parent frame (i.e., `X_PF`).  Returns the
   corresponding unique frame id.
   @param source_id    The id for the source registering the frame.
   @param parent_id    The id of the parent frame P.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source,
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source, or
                             3. a context has been allocated. */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame<T>& frame);

  /** Registers a new geometry G for this source. The geometry hangs on a
   previously registered frame F (indicated by `frame_id`). The pose of the
   geometry is defined in a fixed position relative to F (i.e., `X_FG`).
   Returns the corresponding unique geometry id.
   @param source_id   The id for the source registering the geometry.
   @param frame_id    The id for the frame F to hang the geometry on.
   @param geometry    The geometry G to affix to frame F.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source,
                             2. the `frame_id` doesn't belong to the source,
                             3. the `geometry` is equal to `nullptr`,
                             4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Registers a new geometry G for this source. The pose of the geometry is
   defined in a fixed position relative to another, previously registered
   geometry P (indicated by `geometry_id`). The pose of the new geometry is
   defined in a fixed position relative to P (i.e., `X_PG`). By induction, this
   geometry is rigidly affixed to the frame that P is affixed to. Returns the
   corresponding unique geometry id.

   @param source_id    The id for the source registering the geometry.
   @param geometry_id  The id for the parent geometry P.
   @param geometry     The geometry G to add.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to an active
                            source,
                            2. the `geometry_id` doesn't belong to the source,
                            3. the `geometry` is equal to `nullptr`, or
                            4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /** Registers a new _anchored_ geometry G for this source. The geometry hangs
   from the world frame (W). Its pose is defined in that frame (i.e., `X_WG`).
   Returns the corresponding unique geometry id.
   @param source_id     The id for the source registering the frame.
   @param geometry      The anchored geometry G to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or a context has been allocated. */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** Clears of all the registered frames and geometries from this source, but
   the source is still registered, allowing future registration of frames
   and geometries.
   @param source_id   The id of the source whose regisetered elements will be
                      cleared.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source or if a context has been allocated. */
  void ClearSource(SourceId source_id);

  /** Removes the given frame F (indicated by `frame_id`) from the the given
   source's registered frames. All registered geometries connected to this frame
   will also be removed.
   @param source_id   The id for the owner geometry source.
   @param frame_id    The id of the frame to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source,
                            2. the `frame_id` doesn't belong to the source, or
                            3. a context has been allocated. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Removes the given geometry G (indicated by `geometry_id`) from the the
   given source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source,
                            2. the `geometry_id` doesn't belong to the source,
                               or
                            3. a context has been allocated. */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  //@}

  /** @name     Geometry Queries
   These methods perform queries on the state of the geometry world including:
   proximity queries, contact queries, ray-casting queries, and look ups on
   geometry resources.

   These operations require a QueryHandle instance. The caller must acquire one
   from the %GeometrySystem by connecting to the output port that provides
   GeometryQuery instances.

   The details of these queries are more fully specified in the documentation
   for GeometryWorld. */

  //@{

  /** Report the name for the given source id.
   @param   handle   The QueryHandle produced by evaluating the connected
                     input port on the querying LeafSystem.
   See GeometryWorld::get_source_name() for details. */
  const std::string& get_source_name(const QueryHandle<T>& handle,
                                     SourceId id) const;

  /** Reports if the given source id is registered.
   @param   handle   The QueryHandle produced by evaluating the connected
                     input port on the querying LeafSystem.
   See GeometryWorld::SourceIsRegistered() for details. */
  bool SourceIsRegistered(const QueryHandle<T>& handle,
                          SourceId id) const;

  /** Reports the frame to which this geometry is registered.
   @param   handle   The QueryHandle produced by evaluating the connected
                     input port on the querying LeafSystem.
   See GeometryWorld::GetFrameId() for details. */
  FrameId GetFrameId(const QueryHandle<T>& handle,
                     GeometryId geometry_id) const;

  /** Determines contacts across all geometries in GeometryWorld.
   @param   handle   The QueryHandle produced by evaluating the connected
                     input port on the querying LeafSystem.
   @param   contacts A vector to be populated with computed contact info. The
                     size of `contacts` remains unchanged if no contacts were
                     found.
   See GeometryWorld::ComputeContact() for details. */
  bool ComputeContact(const QueryHandle<T>& handle,
                      std::vector<Contact<T>>* contacts) const;

  // TODO(SeanCurtis-TRI): Flesh this out with the full set of queries.

  //@}

 protected:
  /** GeometrySystem *has* direct feedthrough. All inputs ultimately affect the
   "pose bundle" output port. However, the query output port is unaffected.
   This implementation encodes that relationship in reporting direct
   feedthrough. */
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port,
                              int output_port) const override;

 private:
  // Friend class to facilitate testing.
  friend class GeometrySystemTester;

  // Constructs a QueryHandle for OutputPort allocation.
  QueryHandle<T> MakeQueryHandle(const systems::Context<T>& context) const;

  // Sets the context into the output port value so downstream consumers can
  // perform queries.
  void CalcQueryHandle(const systems::Context<T>& context,
                      QueryHandle<T>* output) const;

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

  // Updates the state of geometry world from *all* the inputs.
  const GeometryContext<T>& FullPoseUpdate(const QueryHandle<T>& handle) const;

  // Override of construction to account for
  //    - instantiating a GeometryContext instance (as opposed to LeafContext),
  //    - modifying the state to prevent additional sources being added. */
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

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
  // TODO(SeanCurtis-TRI): Consider making these TypeSafeIndex values.
  struct SourcePorts {
    int id_port{-1};
    int pose_port{-1};
    int velocity_port{-1};
  };

  // A mapping from added source identifier to the port indices associated with
  // that id.
  std::unordered_map<SourceId, SourcePorts> input_source_ids_;

  // The index of the output port with the PoseBundle abstract value.
  int bundle_port_index_;

  // The index of the output port with the QueryHandle abstract value.
  int query_port_index_;
};

}  // namespace geometry
}  // namespace drake
