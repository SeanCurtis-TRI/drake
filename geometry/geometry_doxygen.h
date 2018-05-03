/** @defgroup geometry_world   Geometry World

 */

/** @defgroup gw_geometry_queries Geometry Queries
 @ingroup geometry_world
 */
// This is a blind copy-and-paste from the deleted geometry_world.h

/** GeometryWorld serves as the common geometric space where disparate
 subsystems can perform geometric queries on geometries introduced into the
 simulator by other subsystems. The geometries are associated with "parent"
 frames; as
 the frame moves, the geometry moves with it in a rigid manner. These parent
 frames are moved by the subsystem that introduced the geometry. GeometryWorld
 performs geometric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, contact/collision detection, etc.)  The results
 provided by these queries depend on the kinematics of the parent frames.
 Prior to each query, the current frame kinematics are reported to GeometryWorld
 to produce a coherent view of the geometric world.

 As geometry is introduced into the simulator, it must be _registered_ with
 GeometryWorld so that it can be included in the queries.

 GeometryWorld is ignorant of the interpretation of the geometry and of what
 mechanisms cause it to move. This knowledge is owned by a "geometry source".
 A geometry source registers the frames that will be moving, and the geometries
 that are moved by them. All data registered with GeometryWorld is associated
 with a source identifier. Conceptually, a single geometry source registers
 geometries and then provides subsequent kinematics values to move the geometry.
 In practice, the responsibilities can be split so long as the _source
 identifier_ is shared across the split. It is important to note, that all
 manipulation of registered data is keyed on the registering source id. The
 balance of this discussion assumes a monolithic geometry source responsible for
 registration _and_ producing kinematics values.

 Geometry sources register frames with GeometryWorld, declaring the frames that
 the source owns and is responsible for computing kinematics values (pose,
 velocity, and acceleration). The geometry source can also "hang" geometry on
 those registered frames in a _rigid_ relationship.  As the frame moves,
 the associated geometry moves with it.  The associated geometry can consist of
 a single shape, or a hierarchy of shapes (such that the whole hierarchy is
 treated as a single rigid union of shapes).

 @section geom_world_workflow Workflow for Geometry Sources

 For a class that serves as a geometry source, the expected work flow is as
 follows in two phases: registration and evaluation.

 @subsection geom_world_declare_workflow Registration:

 @code
 // Assuming the following variables are defined.
 GeometryState state;
 GeometryWorld* geometry_world;

 // Register as a geometry source. It is important to save the unique SourceId
 // into a class member, e.g., source_id_.
 source_id_ = geometry_world->RegisterNewSource(&state, "source_name");

 // Declare moving frames and the hanging geometry. The geometry source is
 // responsible for saving the FrameId instances for use later.
 FrameId f0 = geometry_world->RegisterFrame(&state, source_id_);
 // Instantiate the geometry to hang on the frame and define its pose.
 unique_ptr<GeometryInstance> instance = ...;
 GeometryId g0 = geometry_world->RegisterGeometry(&state, source_id_, f0,
                                                  move(instance));
 FrameId f1 = geometry_world->RegisterFrame(&state, source_id_);
 instance.reset(...);  // Create a new GeometryInstance.
 GeometryId g1 = geometry_world->RegisterGeometry(&state, source_id_, f1,
                                                  move(instance));
 // continue registering frames and geometries.
 @endcode

 @subsection geom_world_value_workflow For setting values during the simulation.

 @code{.cpp}
 // Assume the following variables are defined.
 GeometryWorld* geometry_world;
 SourceId source_id_;

 // Define ordered sets of frame ids and their corresponding kinematics values.
 FrameIdVVector ids(source_id_);
 ids.AddFrames(frames_);

 FramePoseVector poses(source_id_);
 FrameVelocityVector velocities(source_id_);

 foreach (FrameId frame_id : frames_) {
    // Compute pose (Isometry3) and SpatialVelocity.
    Isometry3 X_WF = ...;
    poses.AddValue(X_WF);
    SpatialVelocity V_WF = ...;
    velocities.AddValue(V_WF);
 }
 geometry_world->SetFramePoses(ids, poses);
 geometry_world->SetFrameVelocities(ids, velocities);
 @endcode

 @subsection geom_world_usage_notes Notes on workflow

 These code snippets show a general workflow as an order of operations, but
 should not be taken as literal suggestions. It merely underscores several
 principles:
   - A geometry source must register itself before doing anything else.
   - The SourceId returned is very important and should be saved as a member of
     the class. All operations on GeometryWorld depend on that unique id.
   - In order to register a geometry, the frame it hangs on must be registered
     first.
   - The geometry source is responsible for creating and defining the
     GeometryInstance. GeometryWorld merely takes ownership when passed over.

  What these examples _don't_ cover:
    - The example shows saving FrameId values in local variables (e.g., `f0`).
      In practice, these values would be saved into a mapping structure that
      will associate the FrameId with the source of that frame's values. How
      that mapping is done is up to the class serving as geometry source.
    - It provides no details on _how_ to instantiate a GeometryInstance (see
      that class's documentation for details).
    - It doesn't give an example of building a _hierarchy_ of geometry
      instances. Each GeometryInstance includes a frame, so new geometry can be
      "hung" from previously registered geometries.
    - There are methods for _removing_ registered frames and geometries which
      are not illustrated here.

  Finally, the outlined work flow assumes execution independent of the System
  architecture. The workflow changes slightly when running in a Diagram. For
  details on the change, see SceneGraph.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported. */