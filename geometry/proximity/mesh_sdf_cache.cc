#include "drake/geometry/proximity/mesh_sdf_cache.h"

#include <mutex>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {

using Eigen::Vector3d;

MeshSdfCache::MeshSdfCache() : table_(std::make_shared<SharedTable>()) {}

MeshSdfCache::MeshSdfCache(const MeshSdfCache& other) : table_(other.table_) {
  // Hold the lock across both the key copy and the increment so that no
  // concurrent Remove() on `other` can evict an entry between the two steps.
  std::unique_lock lock(table_->mutex);
  geometry_to_key_ = other.geometry_to_key_;
  IncrementAllRefsUnsafe();
}

MeshSdfCache& MeshSdfCache::operator=(const MeshSdfCache& other) {
  if (this == &other) return *this;

  Flush();
  // We should never overwrite a populated table; doing so would mess up
  // reference counts.
  DRAKE_DEMAND(table_->file_entries.empty());

  // Adopt other's shared table, then hold its lock across both the key copy
  // and the increment so that no concurrent Remove() on `other` can evict an
  // entry between the two steps.
  table_ = other.table_;

  std::unique_lock lock(table_->mutex);
  geometry_to_key_ = other.geometry_to_key_;
  IncrementAllRefsUnsafe();

  return *this;
}

MeshSdfCache::MeshSdfCache(MeshSdfCache&& other)
    : table_(std::move(other.table_)),
      geometry_to_key_(std::move(other.geometry_to_key_)) {
  // Leave the moved-from in a valid empty state so its destructor is safe.
  other.table_ = std::make_shared<SharedTable>();
}

MeshSdfCache& MeshSdfCache::operator=(MeshSdfCache&& other) {
  if (this == &other) return *this;
  // Flush our old content before taking other's.
  Flush();
  table_ = std::move(other.table_);
  geometry_to_key_ = std::move(other.geometry_to_key_);
  // Leave the moved-from in a valid empty state so its destructor is safe.
  other.table_ = std::make_shared<SharedTable>();
  return *this;
}

MeshSdfCache::~MeshSdfCache() {
  // When this *single* cache gets deleted, it must remove all of its references
  // into the shared table.
  Flush();
}

void MeshSdfCache::Register(GeometryId id, const Mesh& mesh) {
  const Vector3d scale = mesh.scale3();
  RegisterImpl(id, mesh.source().GetCacheKey(/* is_convex= */ false),
               ScaleKey{scale[0], scale[1], scale[2]},
               [&mesh]() -> std::optional<TriangleSurfaceMesh<double>> {
                 if (mesh.extension() == ".vtk") {
                   return ConvertVolumeToSurfaceMesh(
                       ReadVtkToVolumeMesh(mesh.source(), Vector3d::Ones()));
                 } else if (mesh.extension() == ".obj") {
                   return ReadObjToTriangleSurfaceMesh(mesh.source(),
                                                       Vector3d::Ones());
                 }
                 // We don't yet have general proximity support for other mesh
                 // types (e.g., glTF).
                 return std::nullopt;
               });
}

void MeshSdfCache::Register(GeometryId id, const Convex& convex) {
  const Vector3d scale = convex.scale3();
  RegisterImpl(
      id, convex.source().GetCacheKey(/* is_convex= */ true),
      ScaleKey{scale[0], scale[1], scale[2]},
      [&convex, &scale]() -> std::optional<TriangleSurfaceMesh<double>> {
        TriangleSurfaceMesh<double> scaled_tri =
            MakeTriangleFromPolygonMesh(convex.GetConvexHull());
        const Vector3d inv_scale(1.0 / scale[0], 1.0 / scale[1],
                                 1.0 / scale[2]);
        return scaled_tri.CreateScaledMesh(inv_scale);
      });
}

void MeshSdfCache::Remove(GeometryId id) {
  auto geo_it = geometry_to_key_.find(id);
  if (geo_it == geometry_to_key_.end()) return;

  const Key key = geo_it->second;
  geometry_to_key_.erase(geo_it);

  std::unique_lock lock(table_->mutex);
  DecrementSingleRefUnsafe(key);
}

const MeshDistanceBoundary& MeshSdfCache::GetOrCompute(GeometryId id) const {
  const Key& key = geometry_to_key_.at(id);

  // Fast path: boundary already computed.
  {
    std::shared_lock read_lock(table_->mutex);
    const FileEntry& file_entry = table_->file_entries.at(key.file_key);
    const ScaleEntry& scale_entry = file_entry.scales.at(key.scale_key);
    if (scale_entry.boundary.has_value()) return *scale_entry.boundary;
  }

  // Slow path: compute under exclusive lock, then re-check (double-checked
  // locking — safe because std::map insertions do not invalidate references).
  std::unique_lock write_lock(table_->mutex);
  FileEntry& file_entry = table_->file_entries.at(key.file_key);
  ScaleEntry& scale_entry = file_entry.scales.at(key.scale_key);
  if (scale_entry.boundary.has_value()) return *scale_entry.boundary;

  const ScaleKey& sk = key.scale_key;
  TriangleSurfaceMesh<double> scaled_mesh =
      file_entry.unit_mesh.CreateScaledMesh(Vector3d(sk[0], sk[1], sk[2]));
  scale_entry.boundary.emplace(std::move(scaled_mesh));
  return *scale_entry.boundary;
}

void MeshSdfCache::DecrementSingleRefUnsafe(const Key& key) {
  auto file_it = table_->file_entries.find(key.file_key);
  if (file_it == table_->file_entries.end()) return;
  FileEntry& file_entry = file_it->second;

  auto scale_it = file_entry.scales.find(key.scale_key);
  if (scale_it == file_entry.scales.end()) return;

  if (--scale_it->second.ref_count == 0) {
    file_entry.scales.erase(scale_it);
  }
  if (file_entry.scales.empty()) {
    table_->file_entries.erase(file_it);
  }
}

void MeshSdfCache::DecrementAllRefs() {
  std::unique_lock lock(table_->mutex);
  for (const auto& [id, key] : geometry_to_key_) {
    DecrementSingleRefUnsafe(key);
  }
}

void MeshSdfCache::IncrementAllRefsUnsafe() {
  for (const auto& [id, key] : geometry_to_key_) {
    ++(table_->file_entries.at(key.file_key)
           .scales.at(key.scale_key)
           .ref_count);
  }
}

void MeshSdfCache::RegisterImpl(
    GeometryId id, const std::string& file_key, const ScaleKey& scale_key,
    std::function<std::optional<TriangleSurfaceMesh<double>>()>
        make_unit_mesh) {
  std::unique_lock lock(table_->mutex);
  if (!table_->file_entries.contains(file_key)) {
    auto unit_mesh = make_unit_mesh();
    if (!unit_mesh.has_value()) return;
    table_->file_entries.emplace(file_key,
                                 FileEntry{std::move(*unit_mesh), {}});
  }
  table_->file_entries.at(file_key).scales[scale_key].ref_count++;
  geometry_to_key_[id] = Key{file_key, scale_key};
}

void MeshSdfCache::Flush() {
  DecrementAllRefs();
  geometry_to_key_.clear();
}

// clang-format off
}  // namespace point_distance
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
