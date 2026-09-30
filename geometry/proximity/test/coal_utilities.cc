#include "drake/geometry/proximity/test/coal_utilities.h"

#include <memory>
#include <utility>
#include <vector>

#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

std::shared_ptr<CoalConvex> MakeConvex(const Convex& convex) {
  const PolygonSurfaceMesh<double>& hull = convex.GetConvexHull();
  auto vertices = std::make_shared<std::vector<Vector3d>>();
  vertices->reserve(hull.num_vertices());
  for (int i = 0; i < hull.num_vertices(); ++i) {
    vertices->push_back(hull.vertex(i));
  }
  auto faces = std::make_shared<std::vector<coal::Triangle32>>(
      MakeCoalTriangles(hull.face_data()));
  const int num_vertices = ssize(*vertices);
  const int num_faces = ssize(*faces);
  return std::make_shared<CoalConvex>(std::move(vertices), num_vertices,
                                      std::move(faces), num_faces);
}

}  // namespace

std::unique_ptr<coal::CollisionObject> MakeCoalObject(
    const Shape& shape, GeometryId id, bool is_dynamic,
    const math::RigidTransformd& X_WG) {
  struct CoalObjectMaker final : public ShapeReifier {
    using ShapeReifier::ImplementGeometry;
    void ImplementGeometry(const Box& box, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          std::make_shared<coal::Box>(box.size());
    }
    void ImplementGeometry(const Capsule& capsule, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          std::make_shared<coal::Capsule>(capsule.radius(), capsule.length());
    }
    void ImplementGeometry(const Convex& convex, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          MakeConvex(convex);
    }
    void ImplementGeometry(const Cylinder& cylinder, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          std::make_shared<coal::Cylinder>(cylinder.radius(),
                                           cylinder.length());
    }
    void ImplementGeometry(const HalfSpace&, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          std::make_shared<coal::Halfspace>(0, 0, 1, 0);
    }
    void ImplementGeometry(const Sphere& sphere, void* data) override {
      *static_cast<std::shared_ptr<coal::CollisionGeometry>*>(data) =
          std::make_shared<coal::Sphere>(sphere.radius());
    }
  };
  CoalObjectMaker maker;
  std::shared_ptr<coal::CollisionGeometry> coal_geometry;
  shape.Reify(&maker, &coal_geometry);
  auto object = std::make_unique<coal::CollisionObject>(coal_geometry);
  EncodedData(id, is_dynamic).write_to(object.get());
  object->setTransform(ToCoalTransform(X_WG));
  return object;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
