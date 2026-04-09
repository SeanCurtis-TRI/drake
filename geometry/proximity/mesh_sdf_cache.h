#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "drake/common/drake_export.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance DRAKE_NO_EXPORT {

/* Two-level cache for deferred, shared MeshDistanceBoundary computation.

 MeshDistanceBoundary construction (BVH + FeatureNormalSet) is deferred from
 geometry registration time to the first query that needs each
 (source file, scale) combination. Results are cached and shared across:
  - All geometries that reference the same source file at the same scale.
  - All MeshSdfCache instances that share the same underlying table (i.e.,
    instances that were copy-assigned from the same original).

 Cache structure:
  - A shared table maps file_key → (unit_mesh, scale_key →
    (ref_count, MeshDistanceBoundary)). The MeshDistanceBoundary is computed
    lazily on the first GetOrCompute() call for each (file_key, scale_key)
    pair. Reference counting tracks how many geometry registrations — across
    all instances sharing the table — use each (file, scale) entry; entries
    are evicted when their count reaches zero.
  - A per-instance index maps geometry_id → (file_key, scale_key).

 Thread safety: during the query phase — after all geometry has been
 registered and the set of registered geometries is stable — concurrent
 calls to GetOrCompute() and contains() across any instances sharing the
 same underlying table are safe. GetOrCompute() uses an internal
 shared_mutex to serialise the first computation for each (source, scale)
 pair while allowing subsequent reads to proceed in parallel.

 During the mutation phase — construction, destruction, copy assignment,
 Register(), and Remove() — these operations acquire an exclusive lock on
 the shared table and so are safe with respect to concurrent GetOrCompute()
 calls on *other* instances. However, they must not be called concurrently
 with any other method on the *same* instance, since the per-instance index
 (geometry_to_key_) is not protected by any lock. */
class MeshSdfCache {
 public:
  // Only copy assignment is supported, to facilitate ProximityEngine copying
  // while sharing the underlying boundary data across copies.
  MeshSdfCache(const MeshSdfCache&) = delete;
  MeshSdfCache(MeshSdfCache&&) = delete;
  MeshSdfCache& operator=(const MeshSdfCache& other);
  MeshSdfCache& operator=(MeshSdfCache&&) = delete;

  MeshSdfCache();
  ~MeshSdfCache();

  /* Returns true if geometry `id` has been registered. */
  bool contains(GeometryId id) const { return geometry_to_key_.contains(id); }

  /* Registers a Mesh geometry. If this is the first registration for `mesh`'s
   source across all instances sharing this table, the unit-scale (1,1,1)
   surface mesh is loaded and stored; otherwise the stored unit mesh is
   reused. Unsupported extensions (.vtk and .obj are supported) are silently
   ignored; the geometry will not be queryable via GetOrCompute(). */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. If this is the first registration for
   `convex`'s source across all instances sharing this table, the
   triangulated convex hull at unit scale is stored; otherwise the stored
   unit mesh is reused. */
  void Register(GeometryId id, const Convex& convex);

  /* Removes a geometry. Decrements the reference count for its (source,
   scale) entry in the shared table; evicts the scale entry if the count
   reaches zero, and the file entry if all its scale entries are gone. */
  void Remove(GeometryId id);

  /* Returns the MeshDistanceBoundary for `id`, computing and caching it on
   the first call for each (source, scale) pair.
   @pre contains(id) is true. */
  const MeshDistanceBoundary& GetOrCompute(GeometryId id) const;

  /* Registers `id` with a pre-built surface mesh, bypassing file I/O.
   Intended for use in unit tests only. */
  void RegisterMeshForTesting(GeometryId id,
                              TriangleSurfaceMesh<double> unit_mesh);

 private:
  friend class MeshSdfCacheTester;

  /* Identifies a specific (source file, scale) entry in the shared table.
   Used as the value type of the per-instance geometry index.  */
  struct Key {
    std::string file_key;
    std::array<double, 3> scale_key;
  };

  using ScaleKey = std::array<double, 3>;

  /* Lazy boundary entry for one (file, scale) pair.  */
  struct ScaleEntry {
    /* Number of geometry registrations (across all MeshSdfCache instances
     sharing the same table) that reference this (file, scale) pair.  */
    int ref_count{0};
    /* Computed lazily on the first GetOrCompute() call for this pair.  */
    std::optional<MeshDistanceBoundary> boundary;
  };

  /* All data for one source file, independent of scale.  */
  struct FileEntry {
    /* Boundary surface mesh at unit scale (1, 1, 1).  */
    TriangleSurfaceMesh<double> unit_mesh;
    /* Per-scale ref counts and lazily-computed boundaries. std::map is used
     (not unordered_map) because std::map insertions do not invalidate
     references to existing elements, which is required by GetOrCompute().  */
    std::map<ScaleKey, ScaleEntry> scales;
  };

  /* Shared table owned via shared_ptr. All MeshSdfCache instances derived
   from the same original (via copy assignment) share the same instance.  */
  struct SharedTable {
    /* Guards all access to file_entries during lazy computation and during
     mutation (Register, Remove, copy assignment, destruction).  */
    mutable std::shared_mutex mutex;
    std::unordered_map<std::string, FileEntry> file_entries;
  };

  /* Decrements the ref count for the given key, evicting entries as needed.
   Must be called under an exclusive lock on table_->mutex. */
  void DecrementRef(const Key& key);

  /* Common bookkeeping for Register(). Acquires an exclusive lock, calls
   make_unit_mesh() to obtain the unit-scale surface mesh if and only if this
   is the first registration for file_key, then increments the ref count and
   records the key. If make_unit_mesh() returns nullopt the geometry is
   silently skipped (used to handle unsupported file extensions). */
  void RegisterImpl(GeometryId id, const std::string& file_key,
                    const ScaleKey& scale_key,
                    std::function<std::optional<TriangleSurfaceMesh<double>>()>
                        make_unit_mesh);

  /* Shared across all MeshSdfCache instances derived from the same original.
   Never null. */
  std::shared_ptr<SharedTable> table_;

  /* Per-instance index: geometry_id → (file_key, scale_key). */
  std::unordered_map<GeometryId, Key> geometry_to_key_;
};

// clang-format off
}  // namespace point_distance
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
