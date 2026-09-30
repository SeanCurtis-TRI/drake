#include "drake/geometry/proximity/proximity_utilities.h"

#include <set>
#include <unordered_set>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"

namespace drake {
namespace geometry {
namespace internal {

std::string GetGeometryName(const coal::CollisionObject& object) {
  // Note: coal::GEOM_CONVEX is an alias for coal::GEOM_CONVEX32, so it cannot
  // appear as a case of its own.
  switch (object.collisionGeometry()->getNodeType()) {
    case coal::BV_UNKNOWN:
    case coal::BV_AABB:
    case coal::BV_OBB:
    case coal::BV_RSS:
    case coal::BV_kIOS:
    case coal::BV_OBBRSS:
    case coal::BV_KDOP16:
    case coal::BV_KDOP18:
    case coal::BV_KDOP24:
    case coal::HF_AABB:
    case coal::HF_OBBRSS:
      return "Unsupported";
    case coal::GEOM_BOX:
      return "Box";
    case coal::GEOM_SPHERE:
      return "Sphere";
    case coal::GEOM_ELLIPSOID:
      return "Ellipsoid";
    case coal::GEOM_CAPSULE:
      return "Capsule";
    case coal::GEOM_CONE:
      return "Cone";
    case coal::GEOM_CYLINDER:
      return "Cylinder";
    case coal::GEOM_CONVEX16:
    case coal::GEOM_CONVEX32:
      return "Convex";
    case coal::GEOM_PLANE:
      return "Plane";
    case coal::GEOM_HALFSPACE:
      return "Halfspace";
    case coal::GEOM_TRIANGLE:
      return "Mesh";
    case coal::GEOM_OCTREE:
      return "Octtree";
    case coal::NODE_COUNT:
      return "Unsupported";
  }
  DRAKE_UNREACHABLE();
}

std::vector<coal::Triangle32> MakeCoalTriangles(
    const std::vector<int>& face_data) {
  std::vector<coal::Triangle32> triangles;
  int i = 0;
  while (i < ssize(face_data)) {
    const int count = face_data[i];
    DRAKE_DEMAND(count >= 3);
    const int first = face_data[i + 1];
    for (int t = 1; t < count - 1; ++t) {
      triangles.emplace_back(first, face_data[i + 1 + t], face_data[i + 2 + t]);
    }
    i += count + 1;
  }
  return triangles;
}

int CountEdges(const VolumeMesh<double>& mesh) {
  std::unordered_set<SortedPair<int>> edges;

  for (auto& t : mesh.tetrahedra()) {
    // 6 edges of a tetrahedron
    edges.emplace(t.vertex(0), t.vertex(1));
    edges.emplace(t.vertex(1), t.vertex(2));
    edges.emplace(t.vertex(0), t.vertex(2));
    edges.emplace(t.vertex(0), t.vertex(3));
    edges.emplace(t.vertex(1), t.vertex(3));
    edges.emplace(t.vertex(2), t.vertex(3));
  }
  return edges.size();
}

int CountFaces(const VolumeMesh<double>& mesh) {
  std::set<SortedTriplet<int>> faces;

  for (const auto& t : mesh.tetrahedra()) {
    // 4 faces of a tetrahedron, all facing in
    faces.emplace(t.vertex(0), t.vertex(1), t.vertex(2));
    faces.emplace(t.vertex(1), t.vertex(0), t.vertex(3));
    faces.emplace(t.vertex(2), t.vertex(1), t.vertex(3));
    faces.emplace(t.vertex(0), t.vertex(2), t.vertex(3));
  }

  return faces.size();
}

int ComputeEulerCharacteristic(const VolumeMesh<double>& mesh) {
  const int k0 = mesh.vertices().size();
  const int k1 = CountEdges(mesh);
  const int k2 = CountFaces(mesh);
  const int k3 = mesh.tetrahedra().size();

  return k0 - k1 + k2 - k3;
}

double CalcDistanceToSurface(const Capsule& capsule, const Vector3d& p_CP) {
  const double half_length = capsule.length() / 2;
  const double z = std::clamp(p_CP.z(), -half_length, half_length);
  const Vector3d p_CQ(0, 0, z);
  return (p_CQ - p_CP).norm() - capsule.radius();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
