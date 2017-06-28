#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

// forward declarations
template <typename T> class GeometrySystem;
template <typename T> class GeometryContext;

/** The %QueryHandle serves as a mechanism to allow LeafSystem instances to
 perform geometry queries on GeometrySystem. One of the GeometrySystem output
 ports is abstract-valued on the %QueryHandle.

 From the perspective of every class _except_ GeometrySystem, this class should
 simply be considered a ticket of sorts; acquired _from_ a GeometrySystem
 instance, it is provided in calls to query methods to validate the call.

 To perform geometry queries on GeometrySystem:
   - a LeafSystem must have a %QueryHandle-valued input port and connect it to
   the corresponding output port on GeometrySystem,
   - the querying LeafSystem can evaluate the input port, retrieving a `const
 QueryHandle*` in return, and, finally,
   - the acquired handle is passed into query methods invoked on a pointer to
 the GeometrySystem.

 @internal There are no public methods. By design, GeometrySystem is the only
 entity that can read and write the single data member. Every other entity
 simply gets the value and passes it around.
 @endinternal

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 @sa GeometrySystem */
template <typename T>
class QueryHandle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(QueryHandle)

 private:
  // Allow GeometrySystem unique access to getting the context back out.
  friend class GeometrySystem<T>;

  // Only the GeometrySystem<T> can create this class.
  QueryHandle(const GeometryContext<T>* context) : context_(context) {}

  // The context associated with the current handle.
  const GeometryContext<T>* context_;
};

}  // namespace geometry
}  // namespace drake
