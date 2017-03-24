#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

// Forward declarations.
template <typename T> class GeometryWorld;
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

 The upstream frame owner should request a channel from GeometryWorld. The
 upstream frame owner should persist this channel. It is through this channel
 that the upstream owner can guarantee consistency with GeometryWorld (see
 UpdateFrameKinematicsSet()). It is the owner's responsibility to _close_ the
 channel before destroying it. Failure to close before destroying is considered
 to be an error.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryChannel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryChannel)

  /** Destructor; detects if it has been closed properly. */
  ~GeometryChannel();

  /** Reports the channels unique identifier. */
  ChannelId get_id() const { return id_; }

  /**
   Closes the channel.  This informs GeometryWorld that the upstream frame
   owner is *done* moving geometry. The frames and geometry declared through
   this channel will be _removed_ from GeometryWorld.

   @param context       A mutable context.
   */
  void Close(drake::systems::Context<T>* context);

  /**
   @name Declaring Frames and Geometry

   These methods serve as the mechanism for the upstream frame owner to inform
   GeometryWorld about the frames it intends to move and the geometry that
   "hangs" on those frames.
   @{
   */

  // TODO(SeanCurtis-TRI): Add metadata. E.g., name, some kind of payload, etc.
  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.
   @param context  A mutable context.
   */
  FrameId DeclareFrame(drake::systems::Context<T>* context);

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.

   If the `id` is not a valid frame identifier declared through this channel, an
   exception will be thrown.

   @param context     A mutable context.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @param X_FG        the transform X_FG which transforms the geometry
                      from its canonical frame `G` to the parent frame `F`,
                      i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId DeclareGeometry(drake::systems::Context<T>* context,
                             FrameId frame_id,
                             std::unique_ptr<GeometryInstance> geometry,
                             const Isometry3<T>& X_FG);

  /**
   Declares a `geometry` instance as "hanging" from the specified geometry's
   frame `F`, with the given pose relative to that frame. The geometry is
   _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry was attached.

   If the `id` is not a valid geometry identifier declared through this channel,
   an exception will be thrown.

   @param context      A mutable context.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @param X_FG         the transform X_FG which transforms the geometry
                       from its canonical frame `G` to the parent frame `F`,
                       i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId DeclareGeometry(drake::systems::Context<T>* context,
                             GeometryId geometry_id,
                             std::unique_ptr<GeometryInstance> geometry,
                             const Isometry3<T>& X_FG);

  /** @} */

  /** @name Writing Frame Kinematics Values
   These methods enable the upstream frame owner to communicate particular
   _values_ for the declared frames. They:

      - provide access to a convenience class (FrameKinematicsSet) and
      - a method for keeping a persisted instance of FrameKinematicsSet in sync
      with the current context.
   @{
   */

  /**
   Requests a frame kinematics set for this channel. This can be invoked once
   and the instance can be persisted in the upstream frame owner. However, it
   must be kept current (see UpdateFrameKinematicsSet()).

   It is recommended that this be invoked after receiving a requested channel
   and persisted by the upstream frame owner. Disposing of the frame and
   requesting a new frame kinematics set would be _valid_ but inefficient.
   */
  FrameKinematicsSet<T> GetFrameKinematicsSet();

  /**
   The upstream moving frame owner should invoke this method prior to setting
   frame kinematics data. This should be done *each* time guaranteeing the
   set reflects the state in the context.

   @param context       The context to update the frame kinematics set against.
   @param frame_set     The frame set to update.
   */
  void UpdateFrameKinematicsSet(const drake::systems::Context<T>& context,
                                FrameKinematicsSet<T>* frame_set);

  /** @} */

  // TODO(SeanCurtis-TRI): Functionality to look at in the future
  //  - Remove geometry (or sub-trees of geometry).
  //  - Remove frame (and its geometry)
  //  - Anchor the frame
  //  - Unanchor the frame

  // Provides access to the private constructor.
  friend class GeometryWorld<T>;

 private:
  // Private constructor; GeometryWorld instantiates them upon request and
  // they cannot be created any other way.
  GeometryChannel(ChannelId index);

  // The index associated with this channel.
  ChannelId id_;

  // Tracks whether this channel was closed before destroyed.
  bool is_open_;
};
}  // namespace geometry
}  // namespace drake
