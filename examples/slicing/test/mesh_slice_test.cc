#include "drake/examples/slicing/mesh_slice.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"

namespace drake {
namespace examples {
namespace slicing {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using geometry::ReadObjToSurfaceMesh;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::RotationalInertia;
using std::pair;
using std::string;
using std::vector;

/* Transforms a mesh with vertices measured and expressed in frame A as a mesh
 in frame B. In other words, vertex i in mesh B is vertex i in mesh A multiplied
 by X_BA. */
SurfaceMesh<double> TransformMesh(const SurfaceMesh<double>& mesh_A,
                                  const RigidTransformd& X_BA) {
  vector<SurfaceFace> faces(mesh_A.faces());
  vector<SurfaceVertex<double>> vertices_B;
  for (const auto& v_A : mesh_A.vertices()) {
    vertices_B.emplace_back(X_BA * v_A.r_MV());
  }
  return SurfaceMesh<double>(move(faces), move(vertices_B));
}

/* Confirms that the mass properties computed for a cube (under various
  transformations) matches the closed-form solution. Specifically, we test
    1. That the cut mesh has been defined in the frame B, such that the mesh's
       center of mass is at Bo.
    2. The mass measure is correct.
    3. The inertia tensor is correct.
    4. The pose of the cut mesh frame B relative to the source mesh frame S is
       as expected.
   All of these quantities are assessed under arbitrary configurations of the
   source mesh. */
GTEST_TEST(ComputeCutWithMass, CubeMassProperties) {
  const double density = 1.5;  // kg/m^3.
  const double side = 2.0;     // m. Should match definition in box.obj.
  const double mass = side * side * side * density;
  const double ixx = mass * side * side / 6;
  const RotationalInertia<double> I_BBcm_B_expected(ixx, ixx, ixx);

  const string box_path = FindResourceOrThrow("drake/examples/slicing/box.obj");
  SurfaceMesh<double> mesh_B = ReadObjToSurfaceMesh(box_path);

  /* Transforms from the box's canonical frame to various test frames. */
  vector<pair<RigidTransformd, string>> X_TBs{
      {RigidTransformd(), "Identity"},
      {RigidTransformd{Vector3d{1.25, 2.75, -3.5}}, "Translation only"},
      {RigidTransformd{RotationMatrixd{
           AngleAxisd(3 * M_PI / 7, Vector3d{1, 2, 3}.normalized())}},
       "Rotation only"}};

  constexpr double kEps = 10 * std::numeric_limits<double>::epsilon();

  for (const auto& [X_TB, description] : X_TBs) {
    const SurfaceMesh<double> mesh_T = TransformMesh(mesh_B, X_TB);
    const RotationalInertia<double> I_BBcm_T_expected =
        I_BBcm_B_expected.ReExpress(X_TB.rotation());
    MeshCut<double> cut = internal::ComputeCutWithMass(mesh_T.vertices(),
                                                       mesh_T.faces(), density);
    EXPECT_NEAR(cut.mass, mass, kEps) << "\nFor " << description;
    EXPECT_TRUE(cut.inertia.IsNearlyEqualTo(I_BBcm_T_expected, kEps))
        << "\nFor " << description << "\n"
        << "  Actual: " << cut.inertia.CopyToFullMatrix3() << "  Expected:\n"
        << I_BBcm_T_expected.CopyToFullMatrix3();
    EXPECT_TRUE(CompareMatrices(cut.X_SourceCut.translation(),
                                X_TB.translation(), kEps))
        << "\nFor " << description;
    EXPECT_TRUE(CompareMatrices(cut.X_SourceCut.rotation().matrix(),
                                RotationMatrixd().matrix(), kEps))
        << "\nFor " << description;

    // In the particular case of the box, the center of mass is its mean vertex
    // position.
    Vector3d p_BBcm(0, 0, 0);
    for (const auto& v_B : cut.mesh.vertices()) {
      p_BBcm += v_B.r_MV();
    }
    p_BBcm /= cut.mesh.num_vertices();
    EXPECT_TRUE(CompareMatrices(p_BBcm, Vector3d::Zero(), kEps))
        << "\nFor " << description;
  }
}

}  // namespace
}  // namespace slicing
}  // namespace examples
}  // namespace drake
