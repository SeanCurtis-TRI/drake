#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

#include <array>
#include <memory>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace multibody {
namespace fem {
using geometry::Box;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using geometry::VolumeVertex;
using geometry::VolumeVertexIndex;

namespace {

/* Given the basis for a deformable mesh (tets, vertices, and a distance
 field defined over those vertices), modifies the mesh so that the resulting
 mesh has _no_ internal tet faces whose vertices all lie on the mesh boundary.

 An "internal" face is any face that is adjacent to _two_ tets; the face lies on
 the interior of the mesh. It may be that vertices that define a face all lie
 on the mesh boundary. In such a case, any field that is defined on the mesh
 vertices *must* evalute to the boundary value across the domain of that
 interior face. This is a lie -- the inside of the mesh should never "appear" to
 be on the boundary.

 This algorithm modifies the tetrahedron topology by refining such faces,
 replacing an ill-defined face with three faces that, by definition, have at
 least one vertex on the interior of the mesh. The adjacent tetrahedra are
 likewise refined to support the new vertex/faces. This will increase the
 resolution of the mesh. */
#if 1  // Set this to zero to disable the refinement.
template <typename T>
void EliminateBoundarySpanningFaces(
    std::vector<VolumeElement>* tets_in,
    std::vector<VolumeVertex<T>>* vertices_in, std::vector<T>* distance_in,
    const std::function<T(const Vector3<T>&)>& calc_distance) {
  using FaceId = geometry::internal::SortedTriplet<int>;
  std::unordered_map<FaceId, std::vector<int>> adjacent_tets;

  std::vector<VolumeElement>& tets = *tets_in;
  std::vector<VolumeVertex<T>>& vertices = *vertices_in;
  std::vector<T>& distance = *distance_in;
  std::unordered_set<FaceId> zero_faces;

  const int original_tet_count = static_cast<int>(tets.size());
  const int original_vert_count = static_cast<int>(vertices.size());

  /* Enumerate the four tet faces in tet-local index values. */
  std::array<std::tuple<int, int, int>, 4> tet_faces{
      std::make_tuple(0, 1, 2), std::make_tuple(1, 3, 2),
      std::make_tuple(0, 3, 1), std::make_tuple(2, 3, 0)};

  /* This is an arbitrary tolerance. Maybe it could be zero if the original
   distances could provide perfect zeros for boundary vertices. Alternatively,
   it may have to be scaled up as the mesh gets larger. */
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  using std::abs;

  for (int t = 0; t < static_cast<int>(tets.size()); ++t) {
    const auto& tet = tets[t];
    for (const auto& [v0, v1, v2] : tet_faces) {
      const VolumeVertexIndex V0 = tet.vertex(v0);
      const VolumeVertexIndex V1 = tet.vertex(v1);
      const VolumeVertexIndex V2 = tet.vertex(v2);
      const FaceId f_id(V0, V1, V2);
      /* If f_id is not already in `adjacent_tets`, this adds it with an empty
       vector of tet indices. */
      adjacent_tets[f_id].push_back(t);
      if (abs(distance[V0]) <= kTolerance && abs(distance[V1]) <= kTolerance &&
          abs(distance[v2]) <= kTolerance) {
        zero_faces.insert(f_id);
      }
    }
  }

  int refined_face_count = 0;
  for (const auto& f_id : zero_faces) {
    /* Faces with a single adjacent tet are on the boundary and don't need
     changing. */
    if (adjacent_tets[f_id].size() == 1) continue;
    ++refined_face_count;

    /* The interior face needs to be split around its centroid C. */
    const Vector3<T> p_MC =
        (vertices[f_id.first()].r_MV() + vertices[f_id.second()].r_MV() +
         vertices[f_id.third()].r_MV()) /
        3;
    VolumeVertexIndex c_index(vertices.size());
    vertices.emplace_back(p_MC);
    distance.emplace_back(calc_distance(p_MC));

    for (const int t_index : adjacent_tets[f_id]) {
      /* We're intentionally copying the original tet, because we'll be
       replacing it within the vector of tets. */
      const VolumeElement tet = tets[t_index];
      /* We don't know which is the first face that *isn't* f_id. That face will
       produce a tet that overwrites the tet at t_index. So, we use this
       bool so we don't have to care whether it is face 0 or 1. */
      bool old_is_overwritten = false;
      for (int f = 0; f < 4; ++f) {
        const auto& [v0, v1, v2] = tet_faces[f];
        const VolumeVertexIndex V0 = tet.vertex(v0);
        const VolumeVertexIndex V1 = tet.vertex(v1);
        const VolumeVertexIndex V2 = tet.vertex(v2);
        const FaceId curr_id(V0, V1, V2);
        /* No work to do relative to the face I'm refining. */
        if (curr_id == f_id) continue;
        if (old_is_overwritten) {
          const int new_t_index = static_cast<int>(tets.size());
          tets.emplace_back(V0, V1, V2, c_index);
          auto& adjacent = adjacent_tets[curr_id];
          /* When we originally populated this map, all of the *original* faces
           of the mesh were included with at least one adjacent tet. If we look
           up a face that has no listed adjacency, it is one of the new faces
           we've added. It can only be adjacent to new tets, so we don't have to
           do any work in updating the adjacency data. */
          if (adjacent.size() == 0) continue;
          if (adjacent[0] == t_index) {
            adjacent[0] = new_t_index;
          } else {
            adjacent[1] = new_t_index;
          }
        } else {
          tets[t_index] = VolumeElement(V0, V1, V2, c_index);
          old_is_overwritten = true;
        }
      }
    }
  }
  const int tet_delta = static_cast<int>(tets.size()) - original_tet_count;
  const int vert_delta =
      static_cast<int>(vertices.size()) - original_vert_count;
  /* It must be the case that for every ill-defined internal face we processed
   that we added four tetrahedra and one vertex. */
  DRAKE_DEMAND(tet_delta == refined_face_count * 4);
  DRAKE_DEMAND(vert_delta == refined_face_count);
}
#else

template <typename T>
void EliminateBoundarySpanningFaces(
    std::vector<VolumeElement>*,
    std::vector<VolumeVertex<T>>*, std::vector<T>*,
    const std::function<T(const Vector3<T>&)>&) {}

#endif

}  // namespace

/* Generates connectivity for the tetrahedral elements of the mesh by splitting
 each cube into five tetrahedra.
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @return
     A sequence of tetrahedral elements that share unique vertices. */
std::vector<VolumeElement> GenerateDiamondCubicElements(
    const Vector3<int>& num_vertices) {
  std::vector<VolumeElement> elements;
  const Vector3<int> num_cells = num_vertices - Vector3<int>(1, 1, 1);
  /* The number of vertices in each direction. */
  for (int i = 0; i < num_cells(0); ++i) {
    for (int j = 0; j < num_cells(1); ++j) {
      for (int k = 0; k < num_cells(2); ++k) {
        /* The following picture shows where vertex vₛ (for `v[s]` below)
         locates in the rectangular cell.  The I-, J-, K-axes show the
         direction of increasing i, j, k indices.

                       v₁     v₃
                       ●------●
                      /|     /|
                     / |  v₇/ |
                 v₅ ●------●  |
                    |  |   |  |
                    |  ●---|--● v₂
                    | /v₀  | /
                    |/     |/
            +K   v₄ ●------● v₆
             |
             |
             o------+J
            /
           /
         +I                                                        */
        VolumeVertexIndex v[8];
        int s = 0;
        for (int l = 0; l < 2; ++l) {
          for (int m = 0; m < 2; ++m) {
            for (int n = 0; n < 2; ++n) {
              v[s++] =
                  VolumeVertexIndex(geometry::internal::CalcSequentialIndex(
                      i + l, j + m, k + n, num_vertices));
            }
          }
        }

        /* The following table subdivides the rectangular cell into five
         tetrahedra. Refer to the picture above to determine which four vertices
         form a tetrahedron. Refer to splitting No. 13 in:
         http://www.baumanneduard.ch/Splitting%20a%20cube%20in%20tetrahedras2.htm
         The splitting alternates depending on the positions of the cube to
         ensure that the mesh is conforming. */
        if ((i + j + k) % 2 == 1) {
          elements.emplace_back(v[6], v[4], v[7], v[2]);
          elements.emplace_back(v[7], v[2], v[1], v[3]);
          elements.emplace_back(v[1], v[4], v[7], v[5]);
          elements.emplace_back(v[2], v[4], v[1], v[0]);
          elements.emplace_back(v[7], v[4], v[1], v[2]);
        } else {
          elements.emplace_back(v[0], v[6], v[5], v[4]);
          elements.emplace_back(v[3], v[6], v[0], v[2]);
          elements.emplace_back(v[5], v[6], v[3], v[7]);
          elements.emplace_back(v[3], v[0], v[5], v[1]);
          elements.emplace_back(v[0], v[6], v[3], v[5]);
        }
      }
    }
  }
  return elements;
}

template <typename T>
internal::ReferenceDeformableGeometry<T> MakeDiamondCubicBoxDeformableGeometry(
    const Box& box, double resolution_hint,
    const math::RigidTransform<T>& X_WB) {
  DRAKE_DEMAND(resolution_hint > 0.);
  /* Number of vertices in x-, y-, and z- directions.  In each direction,
   there is one more vertices than cells. */
  const Vector3<int> num_vertices{
      1 + static_cast<int>(ceil(box.width() / resolution_hint)),
      1 + static_cast<int>(ceil(box.depth() / resolution_hint)),
      1 + static_cast<int>(ceil(box.height() / resolution_hint))};

  /* Initially generate vertices in box's frame B. */
  std::vector<VolumeVertex<T>> vertices =
      geometry::internal::GenerateVertices<T>(box, num_vertices);

  // TODO(xuchenhan-tri): This is an expedient but expensive way to calculate
  //  the signed distance of the vertices. When moving out of dev/, come up with
  //  a better solution.
  /* Generate the vertex distances to the shape. */
  geometry::SceneGraph<T> scene_graph;
  const auto& source_id = scene_graph.RegisterSource();
  const auto& geometry_id = scene_graph.RegisterGeometry(
      source_id, scene_graph.world_frame_id(),
      std::make_unique<geometry::GeometryInstance>(math::RigidTransformd(),
                                                   box.Clone(), "box"));
  scene_graph.AssignRole(source_id, geometry_id,
                         geometry::ProximityProperties());
  const auto& context = scene_graph.CreateDefaultContext();
  const auto& query_object =
      scene_graph.get_query_output_port()
          .template Eval<geometry::QueryObject<T>>(*context);
  std::vector<T> signed_distances;
  for (const VolumeVertex<T>& vertex : vertices) {
    const auto& d = query_object.ComputeSignedDistanceToPoint(vertex.r_MV());
    DRAKE_DEMAND(d.size() == 1);
    signed_distances.emplace_back(d[0].distance);
  }

  std::vector<VolumeElement> elements =
      GenerateDiamondCubicElements(num_vertices);

  auto calc_distance = [&query_object](const Vector3<T>& p_BQ) {
    const auto& d = query_object.ComputeSignedDistanceToPoint(p_BQ);
    DRAKE_DEMAND(d.size() == 1);
    return d[0].distance;
  };
  EliminateBoundarySpanningFaces<T>(&elements, &vertices, &signed_distances,
                                    calc_distance);

  // Now that we've corrected the mesh in the box frame, let's transform the
  // vertices.
  for (VolumeVertex<T>& vertex : vertices) {
    // Transform to World frame.
    vertex = VolumeVertex<T>(X_WB * vertex.r_MV());
  }

  auto mesh =
      std::make_unique<VolumeMesh<T>>(std::move(elements), std::move(vertices));
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "Approximated signed distance", std::move(signed_distances), mesh.get(),
      false);
  return {std::move(mesh), std::move(mesh_field)};
}

template <typename T>
VolumeMesh<T> MakeOctahedronVolumeMesh() {
  // Eight tetrahedra in the octahedron mesh. Order the vertices to follow the
  // convention in geometry::VolumeMesh. See geometry::VolumeMesh for more
  // details.
  const int element_data[8][4] = {
      // The top four tetrahedrons share the top vertex v5.
      {0, 1, 2, 5},
      {0, 2, 3, 5},
      {0, 3, 4, 5},
      {0, 4, 1, 5},
      // The bottom four tetrahedrons share the bottom vertex v6.
      {0, 2, 1, 6},
      {0, 3, 2, 6},
      {0, 4, 3, 6},
      {0, 1, 4, 6}};
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  // clang-format off
  const Vector3<T> vertex_data[7] = {
      { 0,  0,  0},
      { 1,  0,  0},
      { 0,  1,  0},
      {-1,  0,  0},
      { 0, -1,  0},
      { 0,  0,  1},
      { 0,  0, -1}};
  // clang-format on
  std::vector<VolumeVertex<T>> vertices;
  for (const auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

template <typename T>
internal::ReferenceDeformableGeometry<T> MakeOctahedronDeformableGeometry() {
  auto mesh = std::make_unique<VolumeMesh<T>>(MakeOctahedronVolumeMesh<T>());
  /* The distance to surface of the octahedron is zero for all vertices except
   for v0 that has signed distance to the surface of -1/√3. */
  std::vector<T> signed_distances(7, 0.0);
  signed_distances[0] = -1.0 / std::sqrt(3);
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "Approximated signed distance", std::move(signed_distances), mesh.get(),
      false);
  return {std::move(mesh), std::move(mesh_field)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeDiamondCubicBoxDeformableGeometry<T>, &MakeOctahedronVolumeMesh<T>,
     &MakeOctahedronDeformableGeometry<T>))

}  // namespace fem
}  // namespace multibody
}  // namespace drake
