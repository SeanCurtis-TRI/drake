#include "drake/geometry/proximity/distance_box_sphere.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace signed_distance {
namespace {

using Eigen::Vector3d;

::testing::AssertionResult IsExpectedFace(const NearestFeature& feature,
                                          int axis, double sign) {
  if (!feature.is_face() || feature.is_vertex() || feature.is_edge()) {
    return ::testing::AssertionFailure()
           << "feature did not report as a face; "
              "it reported, instead as: "
           << (feature.is_vertex() ? "vertex"
                                   : (feature.is_edge() ? "edge" : "unknown"));
  }

  // Because the nearest feature is a face, the selector doesn't matter.
  // We just need to confirm that the face_axis reports the
  // expected axis, and that the selector for that axis has the right
  // sign.

  if (feature.face_axis() != axis || feature.selector()(axis) != sign) {
    return ::testing::AssertionFailure()
           << "Feature reported as face with unexpected face data: "
           << " expected (axis, sign): (" << axis << ", " << sign << "), got ("
           << feature.face_axis() << ", " << feature.selector()(axis) << ")";
  }
  return ::testing::AssertionSuccess();
}

// Confirms, for various query points, that the reported NearestFeature is
// correct.
GTEST_TEST(BoxSphereDistance, FindNearestFeature_Face) {
  // Half-extent of an axis-aligned box.
  Vector3d h{1.5, 0.5, 3.5};

  // For each face, facing in each axis direction, test a number of query points
  // that should all produce a face as the nearest feature.

  for (const int axis : {0, 1, 2}) {
    for (double sign : {-1, 1}) {
      const Vector3d grad_B = sign * Vector3d::Unit(axis);
      const Vector3d p_BN_B = grad_B.cwiseProduct(h);
      // Case: point is *outside*.
      {
        const double kDistance = 0.5;

        // Query in center of the face.
        Vector3d p_BQ_B = p_BN_B + kDistance * grad_B;
        {
          const NearestFeature feature = FindNearestFeature(h, p_BQ_B);
          EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
          EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BQ_B), p_BN_B));
        }

        // Query at each corner of the face.
        for (const double u : {-1, 1}) {
          for (const double v : {-1, 1}) {
            Vector3d corner{0, 0, 0};
            corner((axis + 1) % 3) = u;
            corner((axis + 2) % 3) = v;
            const Vector3d p_BC_B = p_BQ_B + corner.cwiseProduct(h);
            const NearestFeature feature = FindNearestFeature(h, p_BC_B);
            EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BC_B),
                                        p_BN_B + corner.cwiseProduct(h)));
          }
        }
      }

      // Case: point is *on* the boundary.
      {
        // Query in center of the face.
        {
          const NearestFeature feature = FindNearestFeature(h, p_BN_B);
          EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
          EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BN_B), p_BN_B));
        }

        const double eps = DistanceToPointRelativeTolerance(h(axis));
        const double almost_one = 1 - 4 * eps;
        // Query at *almost* each corner of the face. Actually being *on* the
        // corner is considered to be a vertex/edge feature.
        for (const double u : {-almost_one, almost_one}) {
          for (const double v : {-almost_one, almost_one}) {
            Vector3d corner{0, 0, 0};
            corner((axis + 1) % 3) = u;
            corner((axis + 2) % 3) = v;
            const Vector3d p_BC_B = p_BN_B + corner.cwiseProduct(h);
            const NearestFeature feature = FindNearestFeature(h, p_BC_B);
            EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BC_B), p_BC_B));
          }
        }
      }

      // Case: point is *inside* the box.
      {
        const double distance = -0.1 * h.minCoeff();
        // Query in center of the face.
        Vector3d p_BQ_B = p_BN_B + distance * grad_B;
        {
          const NearestFeature feature = FindNearestFeature(h, p_BQ_B);
          EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
          EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BQ_B), p_BN_B));
        }

        // Query at *almost* each corner of the face. Actually being *on* the
        // corner is considered to be a vertex feature.
        for (const double u : {-1, 1}) {
          for (const double v : {-1, 1}) {
            Vector3d corner{0, 0, 0};
            corner((axis + 1) % 3) = u;
            corner((axis + 2) % 3) = v;
            // Come in from the corner *more* than the distance I'm penetrating
            // the target face; avoid accidentally being closer to another face.
            const Vector3d p_NC_B =
                corner.cwiseProduct(h) + corner * (2 * distance);
            const Vector3d p_BC_B = p_BQ_B + p_NC_B;
            const NearestFeature feature = FindNearestFeature(h, p_BC_B);
            EXPECT_TRUE(IsExpectedFace(feature, axis, sign));
            EXPECT_TRUE(
                CompareMatrices(feature.p_BN_B(h, p_BC_B), p_BN_B + p_NC_B));
          }
        }
      }
    }
  }
}

// @param axis    The axis along which the query point was reported as inside.
//                The selector should be zero on this axis.
// @param expected_selector  The expected selector from the feature. It should
//                           be the case that expected_selector(axis) is zero
//                           and 1 or -1 everywhere else.
::testing::AssertionResult IsExpectedEdge(const NearestFeature& feature,
                                          int axis,
                                          const Vector3d& expected_selector) {
  if (feature.is_face() || feature.is_vertex() || !feature.is_edge()) {
    return ::testing::AssertionFailure()
        << "feature did not report as an edge; "
           "it reported, instead as: "
        << (feature.is_vertex() ? "vertex"
                                : (feature.is_face() ? "face" : "unknown"));
  }

  if (feature.inside_axis() != axis) {
    return ::testing::AssertionFailure()
           << "Feature reported as edge with unexpected inside axis: "
           << " expected axis: " << axis << ", got " << feature.inside_axis();
  }

  // Because the nearest feature is an edge, we want the selector to match
  // exactly.
  if (((expected_selector - feature.selector()).array() == 0).count() < 3) {
    return ::testing::AssertionFailure()
           << "Selector for edge does not match expected. Expected: "
           << expected_selector.transpose()
           << " got: " << feature.selector().transpose();
  }
  return ::testing::AssertionSuccess();
}

// Confirms, for various query points, that the reported NearestFeature is
// correct.
GTEST_TEST(BoxSphereDistance, FindNearestFeature_Edge) {
  // Half-extent of an axis-aligned box.
  Vector3d h{1.5, 0.5, 3.5};

  for (const int axis : {0, 1, 2}) {
    const double eps = DistanceToPointRelativeTolerance(h(axis));
    // Avoid the ends of the edge; that will get classified as a vertex.
    const double almost_one = 1 - 4 * eps;
    for (double sign_u : {-1, 1}) {
      for (double sign_v : {-1, 1}) {
        Vector3d edge_selector{0, 0, 0};
        edge_selector((axis + 1) % 3) = sign_u;
        edge_selector((axis + 2) % 3) = sign_v;

        const Vector3d p_BN_B = edge_selector.cwiseProduct(h);

        // Case: point is *outside*.
        {
          const double kDistance = 0.5;

          // Query in center of the edge.
          Vector3d p_BQ_B = p_BN_B + kDistance * edge_selector;
          {
            const NearestFeature feature = FindNearestFeature(h, p_BQ_B);
            EXPECT_TRUE(IsExpectedEdge(feature, axis, edge_selector));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BQ_B), p_BN_B));
          }

          // Query at each the ends of the edge.
          for (const double u : {-almost_one, almost_one}) {
            Vector3d end = Vector3d::Unit(axis) * u;
            const Vector3d p_BE_B = p_BQ_B + end.cwiseProduct(h);
            const NearestFeature feature = FindNearestFeature(h, p_BE_B);
            EXPECT_TRUE(IsExpectedEdge(feature, axis, edge_selector));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BE_B),
                                        p_BN_B + end.cwiseProduct(h)));
          }
        }

        // Case: point is *on* the boundary.
        {
          // Query in center of the edge.
          {
            const NearestFeature feature = FindNearestFeature(h, p_BN_B);
            EXPECT_TRUE(IsExpectedEdge(feature, axis, edge_selector));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BN_B), p_BN_B));
          }

          // Query at each the ends of the edge.
          for (const double u : {-almost_one, almost_one}) {
            Vector3d end = Vector3d::Unit(axis) * u;
            const Vector3d p_BE_B = p_BN_B + end.cwiseProduct(h);
            const NearestFeature feature = FindNearestFeature(h, p_BE_B);
            EXPECT_TRUE(IsExpectedEdge(feature, axis, edge_selector));
            EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BE_B), p_BE_B));
          }
        }

        // Not testing inside; the feature will always be a face when inside.
      }
    }
  }
}

// @param axis    The axis along which the query point was reported as inside.
//                The selector should be zero on this axis.
// @param expected_selector  The expected selector from the feature. It should
//                           be the case that expected_selector(axis) is zero
//                           and 1 or -1 everywhere else.
::testing::AssertionResult IsExpectedVertex(const NearestFeature& feature,
                                            const Vector3d& expected_selector) {
  if (feature.is_face() || !feature.is_vertex() || feature.is_edge()) {
    return ::testing::AssertionFailure()
        << "feature did not report as a vertex; "
           "it reported, instead as: "
        << (feature.is_edge() ? "edge"
                                : (feature.is_face() ? "face" : "unknown"));
  }

  // Because the nearest feature is a vertex, we want the selector to match
  // exactly.
  if (((expected_selector - feature.selector()).array() == 0).count() < 3) {
    return ::testing::AssertionFailure()
        << "Selector for vertex does not match expected. Expected: "
        << expected_selector.transpose()
        << " got: " << feature.selector().transpose();
  }
  return ::testing::AssertionSuccess();
}

// Confirms, for various query points, that the reported NearestFeature is
// correct.
GTEST_TEST(BoxSphereDistance, FindNearestFeature_Vertex) {
  // Half-extent of an axis-aligned box.
  Vector3d h{1.5, 0.5, 3.5};

  for (double x : {-1, 1}) {
    for (double y : {-1, 1}) {
      for (double z : {-1, 1}) {
        const Vector3d edge_selector{x, y, z};
        // position of corner.
        Vector3d p_BC_B = edge_selector.cwiseProduct(h);

        // Query point is the vertex.
        {
          const NearestFeature feature = FindNearestFeature(h, p_BC_B);
          EXPECT_TRUE(IsExpectedVertex(feature, edge_selector));
          EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BC_B), p_BC_B));
        }

        // Case: point is *outside*.
        {
          Vector3d p_BQ_B = edge_selector.cwiseProduct(h) + 0.5 * edge_selector;
          const NearestFeature feature = FindNearestFeature(h, p_BQ_B);
          EXPECT_TRUE(IsExpectedVertex(feature, edge_selector));
          EXPECT_TRUE(CompareMatrices(feature.p_BN_B(h, p_BQ_B), p_BC_B));
        }

        // Not testing inside; the feature will always be a face when inside.
      }
    }
  }
}

// Test the operation that computes the witness from a feature.
GTEST_TEST(BoxSphereDistance, WitnessFromFeature) {
  // No need to test witness.p_BN_B because it simply invokes the
  // NearestFeature::p_BN_B() method. We just need to confirm grad_B and
  // p_NQ_B.
}

}  // namespace
}  // namespace signed_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
