#include "drake/examples/slicing/mesh_slice.h"

#include <fstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace examples {
namespace slicing {

using geometry::Box;
using geometry::Capsule;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::QueryObject;
using geometry::ReadObjToSurfaceMesh;
using geometry::SceneGraph;
using geometry::ShapeReifier;
using geometry::Sphere;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using geometry::internal::PosedHalfSpace;
using math::RigidTransform;
using multibody::RotationalInertia;
using std::move;
using std::pair;
using std::string;
using std::unordered_map;
using std::vector;
using systems::Context;
using VIndex = SurfaceVertexIndex;

namespace internal {

/** Reifier for identifying Mesh and Convex types and extracting their
 meaningful parameters. */
class MeshIdentifier final : public ShapeReifier {
 public:
  const std::string& filename() const { return file_name_; }
  double scale() const { return scale_; }

 private:
  /** @name  Implementation of ShapeReifier interface  */
  //@{

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere&, void*) final {
    ThrowForBadShape("Sphere");
  }
  void ImplementGeometry(const Cylinder&, void*) final {
    ThrowForBadShape("Cylinder");
  }
  void ImplementGeometry(const HalfSpace&, void*) final {
    ThrowForBadShape("HalfSpace");
  }
  void ImplementGeometry(const Box&, void*) final { ThrowForBadShape("Box"); }
  void ImplementGeometry(const Capsule&, void*) final {
    ThrowForBadShape("Capsule");
  }
  void ImplementGeometry(const Ellipsoid&, void*) final {
    ThrowForBadShape("Ellipsoid");
  }
  void ImplementGeometry(const Mesh& mesh, void*) final {
    file_name_ = mesh.filename();
    scale_ = mesh.scale();
  }
  void ImplementGeometry(const Convex& convex, void*) final {
    file_name_ = convex.filename();
    scale_ = convex.scale();
  }

  //@}

  void ThrowForBadShape(const std::string& name) {
    throw std::runtime_error(
        fmt::format("Expected Mesh or Convex, got {}", name));
  }

  std::string file_name_;
  double scale_{1.0};
};

/* This struct tracks the meshes being constructed from the mesh/plane
 intersection. During the cutting process, it accumulates a portion of the mesh
 located on one side of the plane. It contains:

   - The current definition of the mesh (vertices and faces).
     - the faces are defined with indices into this structs vertex list.
   - A map representing mesh edges that border the open mesh hole (i.e., the
     edges that lie on the plane). This is used to facilitate "closing" the
     resultant mesh. Each edge is encoded as a vertex pair (u, v) such that
     open_edges[u] contains v and the ordering as (u, v) is a
     *counter-clockwise* ordering when looking *through* the plane, into the
     mesh.
   - A map from source mesh vertex indices to this building mesh's indices.
   - A map from edges in the source mesh that intersect the plane to the vertex
     in the building mesh located at the intersection point.  */
template <typename T>
struct BuildingMesh {
  vector<SurfaceVertex<T>> vertices;
  vector<SurfaceFace> faces;
  unordered_map<VIndex, VIndex> open_edges;
  unordered_map<VIndex, VIndex> vertices_source_to_new;
  unordered_map<SortedPair<VIndex>, VIndex> edges_to_vertices;
};

/* Creates the final cut mesh (including mass properties and transforms). This
 assumes that the mesh implied by the given `vertices_S` and `faces` is a
 *closed* mesh (the results will largely be meaningless without that).

 The resulting MeshCut will have its mesh with vertices defined in frame G and
 the mesh's center of mass is coincident with Go.

 @param vertices_S    The cut mesh's vertices measured and expressed in the
                      source geometry's frame S.
 @param faces         The triangles of the cut mesh.
 @returns The fully specified cut mesh. */
template <typename T>
MeshCut<T> ComputeCutWithMass(const vector<SurfaceVertex<T>>& vertices_S,
                              const vector<SurfaceFace>& faces,
                              double density) {
  /* This is a custom implementation of the dyadic product targeted to computing
   the intertia tensor. Instead of a 3x3 matrix, we're recording just the upper
   triangle because of the symmetry of the inertia tensor. */
  auto dyad_prod = [](const Vector3<T>& a, const Vector3<T>& b) {
    Vector6<T> product;
    // clang-format off
    product << a[0] * b[0], a[1] * b[1], a[2] * b[2],
               a[0] * b[1], a[0] * b[2], a[1] * b[2];
    // clang-format on
    return product;
  };

  /* Accumulators for the various mass properties.  */
  /* The accumulated inertia as: ixx, iyy, izz, ixy, ixz, iyz. During
   accumulation they will be off by scale factors (60 for the first three and
   120 by the last three). */

  Vector6<T> I_GSo_S_values = Vector6<T>::Zero();
  /* The total volume will be too large by a factor of six when we're done
   accumulating.  */

  T total_volume{0};
  /* The center of mass will need to be scaled by a factor of 1/24 when we're
   done accumulating. 1/6 for the deferred volume calculation and 1/4 for the
   deferred centroid scaling.  */
  Vector3<T> p_SoGcm = Vector3<T>::Zero();

  for (const auto& tri : faces) {
    /* Notation and calculation of tetrahedral mass properties taken from
     Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation. */
    const auto& p = vertices_S[tri.vertex(0)].r_MV();
    const auto& q = vertices_S[tri.vertex(1)].r_MV();
    const auto& r = vertices_S[tri.vertex(2)].r_MV();

    /* This is the signed volume *times six*.  */
    const T six_volume = (p).cross(q).dot(r);
    total_volume += six_volume;
    /* True center of mass would divide by 4 (the invisible origin vertex);
     move the division from here to the end to reduce work.  */
    const Vector3<T> p_SoTcm = (p + q + r);
    p_SoGcm += six_volume * p_SoTcm;
    /* The *full* calculation for a single test would scale the tensor by
     mass/10. mass = density * volume. --> density * volume / 10. For every
     tet, density / 10 is constant. So, we'll factor it out and apply it to
     the accumulated result. So, here, we'll simply multiply by the
     tet-dependent quantity volume. Technically, we have 6 * volume so we'll
     apply the scale factor density / 60 at the end to correct it.  */
    const Vector3<T> p_sum = p + (q + r) / 2;
    const Vector3<T> q_sum = q + (p + r) / 2;
    const Vector3<T> r_sum = r + (p + q) / 2;
    const T d = p.dot(p_sum) + q.dot(q_sum) + r.dot(r_sum);
    Vector6<T> I_TSo_S;
    I_TSo_S << d, d, d, 0, 0, 0;
    I_TSo_S -= dyad_prod(p, p_sum) + dyad_prod(q, q_sum) + dyad_prod(r, r_sum);
    I_TSo_S *= six_volume;

    I_GSo_S_values += I_TSo_S;
  }
  /* Done accumulating. Properly normalize and shift. */
  const T mass = density * total_volume / 6;
  p_SoGcm /= (4 * total_volume);
  /* Correct based the magnitude based on comment above. */
  I_GSo_S_values *= density / 60;
  RotationalInertia<T> I_GSo_S(I_GSo_S_values[0], I_GSo_S_values[1],
                               I_GSo_S_values[2], I_GSo_S_values[3],
                               I_GSo_S_values[4], I_GSo_S_values[5]);
  RotationalInertia<T> I_GGcm_S = I_GSo_S.ShiftToCenterOfMass(mass, p_SoGcm);
  // The basis between S and G is the same, but we translate it so G can have
  // its center of mass at its origin.
  const RigidTransform<T> X_SG{p_SoGcm};
  // TODO(SeanCurtis-TRI) Define X_SG and vertices_G.
  vector<SurfaceVertex<T>> vertices_G;
  vertices_G.reserve(vertices_S.size());
  for (const auto v_S : vertices_S) {
    vertices_G.emplace_back(v_S.r_MV() - p_SoGcm);
  }
  // TODO(SeanCurtis-TRI) As performance optimizations:
  //   1. Eliminate the copy of the faces.
  //   2. Instead of copying vertex positions, mutate in place.
  return MeshCut<T>{
      SurfaceMesh<T>(vector<SurfaceFace>(faces), move(vertices_G)), X_SG, mass,
      I_GGcm_S};
}

template <typename T>
int sgn(const T& x) {
  if (x > 0) {
    return 1;
  } else {
    if (x < 0) return -1;
    return 0;
  }
}

/* An edge needs to be cut. Utility for cutting the edge and returning the
 index of the resultant cut vertices. It is *plural* because we need to
 introduce an independent vertex for both the positive and negative meshes.

@returns (pos_index, neg_index) -- the indices of the vertices that split the
         edge (a, b) for the positive and negative meshes, respectively.
 @pre s_a and s_b must not have the same sign (positive, negative, or zero).
 */
template <typename T>
pair<SurfaceVertexIndex, SurfaceVertexIndex> GetVertexAddIfNeeded(
    SurfaceVertexIndex a, SurfaceVertexIndex b, const T& s_a, const T& s_b,
    const std::vector<SurfaceVertex<double>>& vertices_G,
    BuildingMesh<T>* positive_mesh_G, BuildingMesh<T>* negative_mesh_G) {
  DRAKE_DEMAND(sgn(s_a) != sgn(s_b));

  SortedPair<SurfaceVertexIndex> edge_a_b(a, b);
  /* Any edge that needs to be split needs to be split for both meshes at the
   same time (or has already been). So, we test *one* cache for presence relying
   on the other to match.
   This will no longer be true when the half space no longer partitions the
   mesh into two disjoint pieces -- i.e., when thickness comes into play. I
   might be better off making this map from edge to vertex index *pair*! */
  auto edge_to_v_iter_P = positive_mesh_G->edges_to_vertices.find(edge_a_b);
  auto edge_to_v_iter_N = negative_mesh_G->edges_to_vertices.find(edge_a_b);
  if (edge_to_v_iter_P == positive_mesh_G->edges_to_vertices.end()) {
    using std::abs;
    const T t = abs(s_a) / (abs(s_a) + abs(s_b));
    // We know that the magnitude of the denominator should always be at
    // least as large as the magnitude of the numerator (implying that it should
    // never be greater than unity). Barring an (unlikely) machine epsilon
    // remainder on division, the assertion below should hold.
    DRAKE_DEMAND(t >= 0 && t <= 1);
    bool inserted;
    std::tie(edge_to_v_iter_P, inserted) =
        positive_mesh_G->edges_to_vertices.insert(
            {edge_a_b, SurfaceVertexIndex(positive_mesh_G->vertices.size())});
    DRAKE_DEMAND(inserted);
    std::tie(edge_to_v_iter_N, inserted) =
        negative_mesh_G->edges_to_vertices.insert(
            {edge_a_b, SurfaceVertexIndex(negative_mesh_G->vertices.size())});
    DRAKE_DEMAND(inserted);
    /* When I add thickness, it won't be this point, and it won't be the *same*
     point for both positive and negative meshes. */
    const Vector3<T> p_GV = (vertices_G[a].r_MV() +
                             t * (vertices_G[b].r_MV() - vertices_G[a].r_MV()));
    positive_mesh_G->vertices.emplace_back(p_GV);
    negative_mesh_G->vertices.emplace_back(p_GV);
  }

  return {edge_to_v_iter_P->second, edge_to_v_iter_N->second};
}

/* Utility routine for getting the vertex index from the
 `vertices_to_newly_created_vertices` hashtable. Given a vertex `index` from the
 input mesh, returns the corresponding vertex index in `new_vertices_G`. The
 method creates the vertex in `new_vertices_G` if it hasn't already been added.
 */
template <typename T>
SurfaceVertexIndex GetVertexAddIfNeeded(
    const vector<SurfaceVertex<double>>& vertices_G, SurfaceVertexIndex index,
    BuildingMesh<T>* mesh_G) {
  auto v_to_new_v_iter = mesh_G->vertices_source_to_new.find(index);
  if (v_to_new_v_iter == mesh_G->vertices_source_to_new.end()) {
    bool inserted{false};
    std::tie(v_to_new_v_iter, inserted) = mesh_G->vertices_source_to_new.insert(
        {index, SurfaceVertexIndex(mesh_G->vertices.size())});
    DRAKE_DEMAND(inserted);
    mesh_G->vertices.emplace_back(vertices_G[index].r_MV());
  }
  return v_to_new_v_iter->second;
}

/* Using the data stored in mesh's `open_edges`, constructs triangles to close
 the open hole produced by plane intersection.

 The current implementation makes some naive assumptions:

   - the intersection of the original mesh with the plane forms a *convex*
     polygon. This will be true as long as the original mesh was convex.
   - The surface doesn't have duplicate, coincident vertices. This will likewise
     be satisifed as long as the input mesh has no duplicate vertices (i.e., the
     source mesh is watertight). */
template <typename T>
void CloseMesh(BuildingMesh<T>* mesh) {
  /* TODO(SeanCurtis-TRI) Remove the assumption of convexity.  */
  /* We *want* to create a triangle fan from a single vertex. However, if I
   have multiple, co-linear edges and I select a vertex in the middle, it will
   lead to the construction of one or more zero-area triangles. The way to avoid
   that is to introduce a "centroid" and triangulate around that. */
  const VIndex anchor_index(mesh->vertices.size());
  Vector3<T> centroid = Vector3<T>::Zero();
  for (const auto& [u, v] : mesh->open_edges) {
    centroid += mesh->vertices[u].r_MV();
    mesh->faces.emplace_back(anchor_index, u, v);
  }
  mesh->vertices.emplace_back(centroid / mesh->open_edges.size());
}

template <typename T>
vector<MeshCut<T>> SliceMesh(const SurfaceMesh<double>& mesh_G,
                             const PosedHalfSpace<T>& hs_G, double density,
                             double thickness) {
  DRAKE_DEMAND(density > 0);

  /* The meshes that get built from the positive and negative sides of the
   half space's boundary.  */
  BuildingMesh<T> positive_mesh_data_G;
  BuildingMesh<T> negative_mesh_data_G;

  const vector<SurfaceVertex<double>>& vertices_G = mesh_G.vertices();
  for (const SurfaceFace& triangle : mesh_G.faces()) {
    /* TODO(SeanCurtis-TRI) This classification only works for thickness = 0.
     To make thickness work I simply classifying relative to the center and
     counting positives is insufficient. I really have the following cases (for
     t = thickness / 2):

        | >= t | -t < s < t | < -t | Implication                               |
        | +--+ | +--------+ | +--+ | +---------------------------------------+ |
        |   3  |     0      |  0   | Whole tri into 1.
        |   2  |     1      |  0   | Cut 2 edges; quad into 1.
        |   2  |     0      |  1   | Cut 4 edges; quad into 1, tri into 0.
        |   1  |     2      |  0   | Cut 2 edges; tri into 1.
        |   1  |     1      |  1   | Cut 4 edges; tri into 1, tri into 0.
        |   1  |     0      |  2   | Cut 4 edges; tri goes into 1, quad into 0.
        |   0  |     3      |  0   | Whole tri thrown out.
        |   0  |     2      |  1   | Cut 2 edges, tri into 0.
        |   0  |     1      |  2   | Cut 2 edges, quad into 0.
        |   0  |     0      |  3   | Whole tri in 0.

     There are three cases where we're cutting four edges. In two, we're really
     cutting two edges twice. In the third case (1|1|1), we're cutting two edges
     once and one edge twice.

     There is a great deal of symmetry here. For example, 2|1|0 and 0|1|2 are
     the same case, just with meshes 0 and 2 reversed. The introduction of new
     vertices should be formulated such that it can be handled. With that in
     mind we actually have the following completely unique cases:

      - 3|0|0, 0|0|3 - triangle belongs wholly in one mesh.
      - 0|3|0 - triangle gets thrown out.
      - 2|1|0, 0|1|2 - quad goes into *one* mesh.
      - 1|2|0, 0|2|1 - tri goes into one mesh.
      - 1|0|2, 2|0|1 - tri into one, quad into other.
      - 1|1|1 - tris into both.

     Do all of this after getting thickness = 0 working. */
    // Compute the signed distance of each triangle vertex from the half space.
    T s[3];
    int num_positive = 0;
    for (int i = 0; i < 3; ++i) {
      s[i] = hs_G.CalcSignedDistance(vertices_G[triangle.vertex(i)].r_MV());
      if (s[i] > 0) ++num_positive;
    }

    if (num_positive == 3) {
      // The triangle lies entirely on the "positive" side.
      const VIndex v0_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(0),
                                                 &positive_mesh_data_G);
      const VIndex v1_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(1),
                                                 &positive_mesh_data_G);
      const VIndex v2_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(2),
                                                 &positive_mesh_data_G);
      positive_mesh_data_G.faces.emplace_back(v0_new, v1_new, v2_new);
    } else if (num_positive == 0) {
      // The triangle lies entirely on the "negative" side.
      const VIndex v0_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(0),
                                                 &negative_mesh_data_G);
      const VIndex v1_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(1),
                                                 &negative_mesh_data_G);
      const VIndex v2_new = GetVertexAddIfNeeded(vertices_G, triangle.vertex(2),
                                                 &negative_mesh_data_G);
      negative_mesh_data_G.faces.emplace_back(v0_new, v1_new, v2_new);
    } else if (num_positive == 1) {
      // One vertex on the positive side. We clip two edges; one triangle goes
      // into mesh 1, a quadrilateral (cut into two triangles) goes into
      // mesh 0.
      for (int i0 = 0; i0 < 3; ++i0) {
        if (s[i0] >= 0) {
          // Identify the single positive vertex.
          const int i1 = (i0 + 1) % 3;
          const int i2 = (i0 + 2) % 3;
          const VIndex v0 = triangle.vertex(i0);
          const VIndex v1 = triangle.vertex(i1);
          const VIndex v2 = triangle.vertex(i2);

          // Get the vertices that result from splitting edges 01 and 02.
          const auto [e01_P, e01_N] = GetVertexAddIfNeeded(
              v0, v1, s[i0], s[i1], vertices_G, &positive_mesh_data_G,
              &negative_mesh_data_G);
          const auto [e02_P, e02_N] = GetVertexAddIfNeeded(
              v0, v2, s[i0], s[i2], vertices_G, &positive_mesh_data_G,
              &negative_mesh_data_G);
          const VIndex v0_P =
              GetVertexAddIfNeeded(vertices_G, v0, &positive_mesh_data_G);
          const VIndex v1_N =
              GetVertexAddIfNeeded(vertices_G, v1, &negative_mesh_data_G);
          const VIndex v2_N =
              GetVertexAddIfNeeded(vertices_G, v2, &negative_mesh_data_G);

          /* We add one triangle to the positive side and two to the negative
           side.

                       v0
                      ╱╲           // add (v0, e01, e02)
               e01_P ╱  ╲ e02_P
              ______╱____╲___
              _______________
             e01_N ╱      ╲ e02_N
                  ╱________╲       // split quad into (v1, e02, e01) and
                v1          v2     // (v1, v2, e02)

           We also need to add edges (e01_P, e02_P) and (e01_N, e02_N) to
           their respective building meshes' set of "open edges". They
           positive and negative sides both introduce identical edges, but
           they are *wound* in opposite directions (e.g., one would be (u, v)
           the other would be (v, u). The challenge is figuring out which
           mesh gets which ordering.

           This is actually quite simple. The input triangle already has its
           vertices ordered to be counter-clockwise while looking down the
           triangle normal. The edges are split at points which preserve the
           ordering. So the edge would be ordered as (e01, e02) *on the
           triangle*. However, the edges should be wound from the perspective
           of the missing adjacent triangle, so we reverse the order. It's that
           simple. */
          positive_mesh_data_G.faces.emplace_back(v0_P, e01_P, e02_P);
          positive_mesh_data_G.open_edges[e02_P] = e01_P;
          negative_mesh_data_G.faces.emplace_back(v1_N, e02_N, e01_N);
          negative_mesh_data_G.faces.emplace_back(v1_N, v2_N, e02_N);
          negative_mesh_data_G.open_edges[e01_N] = e02_N;
        }
      }
    } else {
      // Num positive == 2. It's num_positive == 1 flipped upside down.
      for (int i0 = 0; i0 < 3; ++i0) {
        if (s[i0] < 0) {
          // Identify the single negative vertex.
          const int i1 = (i0 + 1) % 3;
          const int i2 = (i0 + 2) % 3;
          const VIndex v0 = triangle.vertex(i0);
          const VIndex v1 = triangle.vertex(i1);
          const VIndex v2 = triangle.vertex(i2);

          // Get the vertices that result from splitting edges 01 and 02.
          const auto [e01_P, e01_N] = GetVertexAddIfNeeded(
              v0, v1, s[i0], s[i1], vertices_G, &positive_mesh_data_G,
              &negative_mesh_data_G);
          const auto [e02_P, e02_N] = GetVertexAddIfNeeded(
              v0, v2, s[i0], s[i2], vertices_G, &positive_mesh_data_G,
              &negative_mesh_data_G);
          const VIndex v0_N =
              GetVertexAddIfNeeded(vertices_G, v0, &negative_mesh_data_G);
          const VIndex v1_P =
              GetVertexAddIfNeeded(vertices_G, v1, &positive_mesh_data_G);
          const VIndex v2_P =
              GetVertexAddIfNeeded(vertices_G, v2, &positive_mesh_data_G);

          /* We add one triangle to the negative side and two to the positive
           side.

                v2 ________ v1
                   ╲      ╱         // split quad into (v1, v2, e02) and
              e02_P_╲____╱__e01_P   // (v1, e02, e01)
                  __________
               e02_N ╲  ╱ e01_N    // add(v0, e01, e02)
                      ╲╱
                      v0
           The logic for logging the "open edges" is the reverse of that
           outlined in the previous case.  */
          negative_mesh_data_G.faces.emplace_back(v0_N, e01_N, e02_N);
          negative_mesh_data_G.open_edges[e02_N] = e01_N;
          positive_mesh_data_G.faces.emplace_back(v1_P, e02_P, e01_P);
          positive_mesh_data_G.faces.emplace_back(v1_P, v2_P, e02_P);
          positive_mesh_data_G.open_edges[e01_P] = e02_P;
        }
      }
    }
  }

  /* TODO(SeanCurtis-TRI) This seems like it will quickly create ugly complexity
   in the meshes without providing distinctive feature (lots and lots of
   co-linear edges). A step where I "dissolve" all of the edges which provide
   no value would be good.
     1. For a vertex a, it is found in edges: (a, b), (a, c), ..., (a, z).
     2. Determine necessary conditions
        - There exists a pair of edges ai, aj that are colinear.
        - All the faces on each side of those edges are planar w.r.t. each
          other.
     3. Dissolve the vertex
        - build two edge loops based on the loops on each side of the edge.
          (Both loops will include i & j).
        - Delete all faces which reference a.
        - Add triangle fans for each loop.
   To make this work:
    - I need a quick look from vertex -> adjacent edges and vertex -> faces.
    - I need a better dynamic data structure if I'm going to try to do this
      on the fly.  */

  vector<MeshCut<T>> cut_pieces;

  CloseMesh(&positive_mesh_data_G);
  if (positive_mesh_data_G.faces.size() > 0) {
    cut_pieces.push_back(internal::ComputeCutWithMass(
        positive_mesh_data_G.vertices, positive_mesh_data_G.faces, density));
  }

  CloseMesh(&negative_mesh_data_G);
  if (negative_mesh_data_G.faces.size() > 0) {
    cut_pieces.push_back(internal::ComputeCutWithMass(
        negative_mesh_data_G.vertices, negative_mesh_data_G.faces, density));
  }
  return cut_pieces;
}

/* Writes the given surface `mesh` to the  given stream `stream`. */
template <typename T>
void WriteSurfaceMeshToObj(const SurfaceMesh<T>& mesh, std::ostream* stream) {
  DRAKE_DEMAND(stream != nullptr);
  std::ostream& s = *stream;
  s << "# Written by "
       "drake::examples::slicing::internal::WriteSurfaceMeshToObj\n";
  for (const auto& v : mesh.vertices()) {
    const Eigen::Vector3d p_MV = v.r_MV().template cast<double>();
    s << "v " << p_MV[0] << " " << p_MV[1] << " " << p_MV[2] << "\n";
  }
  for (const auto& tri : mesh.faces()) {
    s << "f " << (tri.vertex(0) + 1) << " " << (tri.vertex(1) + 1) << " "
      << (tri.vertex(2) + 1) << "\n";
  }
}

}  // namespace internal

template <typename T>
void WriteSurfaceMeshToObj(const geometry::SurfaceMesh<T>& mesh,
                           const std::string& file_name) {
  std::ofstream file(file_name);
  if (file.fail()) {
    throw std::runtime_error(fmt::format(
        "WriteSurfaceMeshToObj: Cannot create file: {}.", file_name));
  }
  internal::WriteSurfaceMeshToObj(mesh, &file);
}

template <typename T>
vector<MeshCut<T>> SliceMesh(GeometryId mesh_id, double density,
                             const Vector3<T>& p_GP, const Vector3<T> n_G,
                             const SceneGraph<T>& scene_graph,
                             const Context<T>& sg_context, double thickness) {
  DRAKE_DEMAND(n_G.norm() > 1e-10);
  // 1. Identify the geometry with geometry id.
  //    Throw if not Mesh/Convex.
  const auto& inspector = scene_graph.get_query_output_port()
                              .template Eval<QueryObject<T>>(sg_context)
                              .inspector();
  internal::MeshIdentifier identifier;
  // This throws if mesh_id isn't a Mesh or Convex.
  inspector.GetShape(mesh_id).Reify(&identifier);

  // 2. Load the mesh.
  SurfaceMesh<double> mesh_G =
      ReadObjToSurfaceMesh(identifier.filename(), identifier.scale());

  // 3. Slice the mesh (creating up to two meshes).
  const PosedHalfSpace<T> hs_G{n_G, p_GP, false /* needs_normalization */};
  return internal::SliceMesh(mesh_G, hs_G, density, thickness);
}

template void WriteSurfaceMeshToObj(
    const geometry::SurfaceMesh<double>& mesh,
    const std::string& file_name);

template vector<MeshCut<double>> SliceMesh(
    GeometryId mesh_id, double density, const Vector3<double>& p_GP,
    const Vector3<double> n_G, const SceneGraph<double>& scene_graph,
    const Context<double>& sg_context, double thickness);

namespace internal {
template MeshCut<double> ComputeCutWithMass(
    const std::vector<geometry::SurfaceVertex<double>>& vertices_S,
    const std::vector<geometry::SurfaceFace>& faces, double density);
}  // namespace internal

}  // namespace slicing
}  // namespace examples
}  // namespace drake
