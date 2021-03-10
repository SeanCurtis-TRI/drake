#include "drake/math/small_matrix_fast_methods.h"

#include <cstdio>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

using std::cout; using std::endl;

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {

namespace  {
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Matrix34d = Eigen::Matrix<double, 3, 4>;

GTEST_TEST(TestSmallMatrixFastMethods, UsingAVX) {
#ifdef __APPLE__
  EXPECT_TRUE(IsUsingPortableCompositionMethods());
#else
  EXPECT_FALSE(IsUsingPortableCompositionMethods());
#endif
}

GTEST_TEST(TestSmallMatrixFastMethods, TestRotationCompositions) {
  Matrix3d M, N;
  M << 1, 5, 9,
       2, 6, 10,
       3, 7, 11;
  N << 13, 17, 21,
       14, 18, 22,
       15, 19, 23;
  const Matrix3d MN_expected = M * N;
  const Matrix3d MtN_expected = M.transpose() * N;

  Matrix3d MN, MtN;
  ComposeRR(M.data(), N.data(), MN.data());
  ComposeRinvR(M.data(), N.data(), MtN.data());

  // Should be a perfect match with integer elements.
  EXPECT_TRUE(CompareMatrices(MN, MN_expected, 0));
  EXPECT_TRUE(CompareMatrices(MtN, MtN_expected, 0));
}

GTEST_TEST(TestSmallMatrixFastMethods, TestTransformCompositions) {
  Matrix34d M, N;
  M << 1, 4, 7, 10,
       2, 5, 8, 11,
       3, 6, 9, 12;
  N << 13, 16, 19, 22,
       14, 17, 20, 23,
       15, 18, 21, 24;
  // Append 0001 row so we can use straight matrix operations to get the
  // result.
  Matrix4d M0, N0;
  M0.topRows(3) = M;
  M0.row(3) << 0, 0, 0, 1;
  N0.topRows(3) = N;
  N0.row(3) << 0, 0, 0, 1;

  const Matrix3d Rt = M.leftCols(3).transpose();  // RotationMatrix inverse
  Matrix4d M0inv;
  M0inv.block<3, 3>(0, 0) = Rt;
  M0inv.block<3, 1>(0, 3) = -Rt * M.col(3);
  M0inv.row(3) << 0, 0, 0, 1;

  const Matrix34d MN_expected = (M0*N0).topRows(3);
  const Matrix34d MinvN_expected = (M0inv*N0).topRows(3);

  Matrix34d MN, MinvN;
  ComposeXX(M.data(), N.data(), MN.data());
  ComposeXinvX(M.data(), N.data(), MinvN.data());

  // Should be a perfect match with integer elements.
  EXPECT_TRUE(CompareMatrices(MN, MN_expected, 0));
  EXPECT_TRUE(CompareMatrices(MinvN, MinvN_expected, 0));
}
}  // namespace

}  // namespace math
}  // namespace drake
