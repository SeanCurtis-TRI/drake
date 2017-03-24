#include "drake/geometry/geometry_channel.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

using drake::systems::Context;
using std::unique_ptr;

template <typename T>
GeometryChannel<T>::GeometryChannel(ChannelId index) : id_(index),
                                                       is_open_(true) {
}

template <typename T>
GeometryChannel<T>::~GeometryChannel() {
  DRAKE_DEMAND(!is_open_);
}

template <typename T>
void GeometryChannel<T>::Close(Context<T>* context) {
  // TODO(SeanCurtis-TRI): Modify the context to remove this geometry's data.
  is_open_ = false;
}

template <typename T>
FrameId GeometryChannel<T>::DeclareFrame(Context<T>* context) {
  // TODO(SeanCurtis-TRI): Do the work!
  static int NEXT_FRAME = 1;
  return FrameId(NEXT_FRAME++);
}

template <typename T>
GeometryId GeometryChannel<T>::DeclareGeometry(Context<T>* context,
                                            FrameId id,
                                            unique_ptr<GeometryInstance> geometry,
                                            const Isometry3<T>& X_FG) {
  // TODO(SeanCurtis-TRI): Do the work!
  static int NEXT_GEOMETRY = 1;
  return GeometryId(NEXT_GEOMETRY++);
}

template <typename T>
GeometryId GeometryChannel<T>::DeclareGeometry(Context<T>* context,
                                            GeometryId id,
                                            unique_ptr<GeometryInstance> geometry,
                                            const Isometry3<T>& X_FG) {
  // TODO(SeanCurtis-TRI): Do the work!
  return DeclareGeometry(context, FrameId(0), geometry, X_FG);
}

template <typename T>
FrameKinematicsSet<T> GeometryChannel<T>::GetFrameKinematicsSet() {
  FrameKinematicsSet<T> set(id_);
  // TODO(SeanCurtis-TRI): Populate this from the reference data.
  return set;
}

template <typename T>
void GeometryChannel<T>::UpdateFrameKinematicsSet(const Context<T>& context,
                                                  FrameKinematicsSet<T>* frame_set) {
  // TODO(SeanCurtis-TRI): Do this work.
}
}  // namespace geometry
}  // namespace drake
