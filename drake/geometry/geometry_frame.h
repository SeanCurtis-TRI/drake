#pragma once

#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** This simple struct carries the definition of a frame used by GeometryWorld.
 To register moving frames with GeometryWorld (see
 GeometryWorld::RegisterFrame), a geometry source (see
 GeometryWorld::RegisterNewSource) instantiates a frame and passes ownership
 over to GeometryWorld.

 A frame is defined by three pieces of information:
    - the name, which must be unique within a single geometry source,
    - the "frame group", an integer identifier that can be used to group frames
      together within a geometry source, and
    - the initial pose of the frame (measured and expressed in its parent
      frame). The parent is defined at registration. This is only the _initial_
      pose; registered frames are expected to move with time.

 @internal The "frame group" is intended as a generic synonym for the model
 instance id defined by the RigidBodyTree and used again in automotive to
 serve as unique car identifiers.

 @see GeometryWorld

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct GeometryFrame {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryFrame)

  /** Constructor.
   @param frame_name        The name of the frame.
   @param initial_pose      The pose of the frame, measured and expressed in the
                            _intended_ parent frame.
   @param frame_group_id    The optional frame group identifier. If unspecified,
                            defaults to the common, 0 group. */
  GeometryFrame(const std::string& frame_name, const Isometry3<T>& initial_pose,
                int frame_group_id = 0)
      : name(frame_name), pose(initial_pose), frame_group(frame_group_id) {}

  /** The name of the frame. Must be unique across frames from the same
   geometry source. */
  std::string name;

  /** The initial pose of the frame, measured and expressed in the parent frame.
   */
  Isometry3<T> pose;

  // TODO(SeanCurtis-TRI): Consider whether this should be an Identifier or
  // TypeSafeIndex type.
  /** The frame group to which this frame belongs. */
  int frame_group;
};
}  // namespace geometry
}  // namespace drake
