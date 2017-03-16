#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_indexes.h"

namespace drake {
namespace geometry {

// Forward declarations.
class GeometryWorld;
class GeometryInstance;   // NOTE: This does not exist yet.

/**
 This class serves as the connection between GeometryWorld and upstream entities
 that are responsible for moving frames that move geometry.

 The upstream entity owns its set of moving frames. It knows what geometry
 "hangs" on any particular frame. It is responsible for passing ownership of
 the geometry to GeometryWorld. It is also responsible for maintaining its set
 of moving frames
 <!-- TODO(SeanCurtis-TRI): Include this when adding the editing methods.
 (including adding or removing frames from the set)
  -->
 and setting the frame's kinematics values for a particular context.

 This class serves as a channel between the upstream entity and GeometryWorld
 through which the upstream entity can fulfill its responsibilities as owner:
    - declaring the existence of frames that will move geometry,
    - declaring what geometry hangs on a declared frame,
    <!-- TODO(SeanCurtis-TRI): Include this when adding the editing methods.
    - removing frames from its geometry set,
    -->
    - populating frame kinematics in a provided object.
 */
class GeometryChannel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryChannel)

  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.
   */
  FrameId DeclareFrame();

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.

   If the `id` is not a valid frame identifier declared through this channel, an
   exception will be thrown.

   @param id          The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @param X_FG        the transform X_FG which transforms the geometry
                      from its canonical frame `G` to the parent frame `F`,
                      i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId DeclareGeometry(FrameId id,
                             unique_ptr<GeometryInstance> geometry,
                             const Isometry3& X_FG);

  /**
   Declares a `geometry` instance as "hanging" from the specified geometry's
   frame `F`, with the given pose relative to that frame. The geometry is
   _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry was attached.

   If the `id` is not a valid geometry identifier declared through this channel,
   an exception will be thrown.

   @param id          The id for the geometry to hang the declared geometry on.
   @param geometry    The geometry to hang.
   @param X_FG        the transform X_FG which transforms the geometry
                      from its canonical frame `G` to the parent frame `F`,
                      i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId DeclareGeometry(GeometryId id,
                             unique_ptr<GeometryInstance> geometry,
                             const Isometry3& X_FG);

  /**
   Requests a frame kinematics set for this channel. The set will have *no*
   kinematics values for any of the set's frames. The caller should invoke this
   once (per unique context), set the kinematics values for *all* of the frames,
   and then provide this object for the GeometryWorld.

   In the System architecture, this will serve as the value for the
   AbstractValue output port.
   */
  FrameKinematicsSet& GetCleanFrameSet();

  // TODO(SeanCurtis-TRI): Functionality to look at in the future
  //  - Remove geometry (or sub-trees of geometry).
  //  - Remove frame (and its geometry)
  //  - Anchor the frame
  //  - Unanchor the frame

  // Provides access to the private constructor.
  friend class GeometryWorld;

 private:
  // Private constructor; GeometryWorld instantiates them upon request and
  // they cannot be created any other way.
  GeometryChannel(FrameSetId index, GeometryWorld* geometry_world);

  // The index associated with this channel.
  ChannelId index_;

  // A handle to the geometry_world_ to coordinate the channel's operations.
  GeometryWorld* geometry_world_;

  // The frames declared in this channel.
  GeometryFrameSet frame_set_{};
};
}  // namespace geometry
}  // namespace drake
