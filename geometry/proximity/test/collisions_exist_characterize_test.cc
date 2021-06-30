#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/collisions_exist_callback.h"
#include "drake/geometry/proximity/test/characterization_utilities.h"

/* @file This provides the test that supports the values contained in the
 table documented for QueryObject::HasCollisions() in query_object.h. */

// TODO(SeanCurtis-TRI) When we've fully implemented our own
//  PenetrationAsPointPair algorithm in Drake, we can express HasCollisions()
//  in terms of that function, and can document its level of service likewise
//  in terms of that function. We will no longer need this independent test.
//  But that also requires a query mode that doesn't require calculated
//  return values (witness points, etc.) and the ability to bail out early.

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {

using math::RigidTransformd;
using std::unique_ptr;
using std::vector;

/* Specification of whether a geometry pair *should* be separated or
 penetrating. */
enum SeparationState { kSeparated, kPenetrating };

/* This represents a single cell of the table -- an instance of invoking the
 query. It explicitly calls out the two shapes, the true configuration (e.g.,
 separated or penetrating), and the exected threshold at which the query stops
 lying. For example,

     QueryInstance(kBox, kBox, kSeparated, 1e-4);

 Indicates, for a (box, box) pair, the deepest observed penetration that was
 still considered separated is 1e-4.
 
 N.B. This struct shadows drake::geometry::internal::QueryInstance defined in
 characterization_utilities.h. It supplants it in this test's context. */
struct QueryInstance {
  QueryInstance(GeometryType shape1_in, GeometryType shape2_in,
                SeparationState state_in, double error_in)
      : shape1(shape1_in),
        shape2(shape2_in),
        state(state_in),
        error(error_in) {}

  const GeometryType shape1{};
  const GeometryType shape2{};
  const SeparationState state{};
  const double error{};
};

std::ostream& operator<<(std::ostream& out, const QueryInstance& query) {
  out << query.shape1 << "vs" << query.shape2
      << (query.state == kSeparated ? "Separated" : "Penetrating");
  return out;
}

/* Provides a human readable name for the parameters in the testing output.
 
 N.B. This function shadows drake::geometry::internal::QueryInstanceName defined
 in characterization_utilities.h. It supplants it in this tests's context. */
std::string QueryInstanceName(
    const testing::TestParamInfo<QueryInstance>& info) {
  return fmt::format("{}", info.param);
}

/* Characterizes the QueryObject::HasCollisions() method. In this case, the
 method returns `true` or `false` based on the detection of *any* intersection.
 So, the "error" in this case consists of a false classification. Separated
 geometries could report collision, and colliding geometries could report as
 separated.

 If we examine the range of the signed distance function, ϕ, on a line we see:

         ────────────────────────┼──────────────────────
                                 0 <-- ϕ == 0 --> touching

 We can then consider what value HasCollisions() returns based on the value of
 ϕ. In an ideal world, it would simply be:

        Ideal separation:
            ϕ ≤ 0 --> HasCollisions() returns true.
            ϕ > 0 --> HasCollisions() returns false.

         (          true         ](      false          )
         ────────────────────────┼──────────────────────
                                 0

 Note that ϕ == 0, touching, is considered a collision based on arbitrary
 definition.

 The goal of this test is to *quantify* how far off of the ideal our code is.
 The text below uses separated geometries as the basis for discussion; the
 penetrating case is analogous, but reversed.

 For separated geometry, we report an expected error ε. This is the magnitude of
 penetration distance which incorrectly reported as separation. If the code is
 behaving in an ideal manner, ε = 0. A non-zero value for ε has the following
 effect:

        Flawed separation (for some ε ≥ 0):
            ϕ ≤ -ε --> HasCollisions() returns true.
            ϕ > -ε --> HasCollisions() returns false.

         (          true      ](         false          )
         ──────────────────────╂─┼──────────────────────
                               ε 0

 In this case, signed distances as *deep* as -ε have been observed to report
 separation.

 As with CharacterizeResultTest, this test works by creating pairs of geometries
 in a sequence of curated relative poses with *known* signed distance between
 them. It is unique in that rather than simply assessing the error in a fixed
 set of poses, it needs to search the space of relative poses to find
 misclassifications. It doesn't exhaustively search the space of relative poses.
 It makes use of the curated relative poses and *coarsely* searches over their
 separation distance (relative to the relative pose's separating plane).

 An expected error of ε means that of all relative poses considered, this was
 the *largest* deviation from ideal observed. */
class CharacterizeHasCollisionsTest
    : public CharacterizeResultTest<double>,
      public testing::WithParamInterface<QueryInstance> {
 public:
  /* Default constructor required by gtest. We aren't using the signed distance
   callback mechanism for CharacterizeResultTest, so we pass it a nullptr for
   the callback argument. */
  CharacterizeHasCollisionsTest() : 
    CharacterizeResultTest<double>(nullptr /* callback */) {}

 protected:
  /* Run the test for a single query.

   The algorithm works as follows:

   - Initialize test_distance to the *expected* threshhold.
   - For a set of poses (X_WAs) and a set of relative configurations in A
     - Set the distance (separating or penetration based on expectation) to the
       test_distance.
     - Evaluate the HasCollisions callback.
     - While the result is misclassified:
       - Increase the magnitude of the test_distance by a "small" amount.
       - Evaluate the HasCollisions callback.
     - While the result is correctly classified and test_distance >(=) zero:
       - Decrease the magnitude of the test_distance by a "small" amount.
       - Evaluate the HasCollisions callback.
   - The resulting test_distance is the actual threshold.
     - Test against the expected threshold and report errors as appropriate.
     - While the result is incorrect:
       - Bump the distance a "small" amount.
       - Evaluate the callback.
       - set observation -> worst_observation (if new observation is larger).
   - The worst observed distance should match the expectation; pass/fail based
     on that criterion.
  */
  void RunCharacterization(const QueryInstance& query) {
    std::cerr << query << "\n";
    const unique_ptr<Shape> shape_A =
        this->MakeShape(query.shape1, false /* use_alt */);
    fcl::CollisionObjectd object_A = MakeFclShape(*shape_A).object();
    this->EncodeData(&object_A);

    const  unique_ptr<Shape> shape_B =
        this->MakeShape(query.shape2, query.shape1 == query.shape2);
    fcl::CollisionObjectd object_B = MakeFclShape(*shape_B).object();
    this->EncodeData(&object_B);

    const bool expect_has_collision =
        query.state == SeparationState::kPenetrating;
    double max_observed_magnitude = 0.0;
    std::string max_observed_description("undefined");
    /* The sign to apply to the distance. */
    const double sign = expect_has_collision ? -1 : 1;
    /* The shapes are documented to be on the order of 2e-1 m with 2e-3 m
     penetration. So, we'll search for error in one part in one hundred. */
    const double kDeltaMag = 1e-5;
    const double delta = sign * kDeltaMag;
    /* The results are asymmetric. Zero belongs to correct penetrating
     classification. Epsilon belongs to searated classification. */
    const double smallest_error =
        expect_has_collision ? 0.0 : std::numeric_limits<double>::epsilon();
    std::cerr << "  expect_has_collisions: " << expect_has_collision << "\n";

    /* The *magnitude* of the maximum observed error. The actual value will be
     negative for penetration or positive for separation. So, a value of zero
     means there is no error in the observation. */
    double test_distance = sign * query.error;
    const vector<ContactConfiguration<double>> configs =
        this->MakeContactConfigurations(*shape_A, *shape_B);
    for (const RigidTransformd& X_WA : this->X_WAs()) {
      std::cerr << "  X_WA\n" << X_WA.GetAsMatrix34() << "\n";
      object_A.setTransform((X_WA).GetAsIsometry3());
      for (const auto& config : configs) {
        std::cerr << "    config: " << config.description << "\n";
        const RigidTransformd T_A =
            RigidTransformd(test_distance * config.separating_dir_A);
        object_B.setTransform((X_WA * T_A * config.X_AB).GetAsIsometry3());

        CallbackData data{&this->collision_filter_};
        Callback(&object_A, &object_B, &data);

        /* Try growing the threshold to see if it should be larger. */
        while (data.collisions_exist != expect_has_collision) {
          std::cerr << "    growing\n";
          data.collisions_exist = expect_has_collision;
          test_distance += delta;

          const RigidTransformd T_A2 =
              RigidTransformd(test_distance * config.separating_dir_A);
          object_B.setTransform((X_WA * T_A2 * config.X_AB).GetAsIsometry3());
          Callback(&object_A, &object_B, &data);
        }

        /* By the time we get here, we know that the last invocation of
         HasCollisions did *not* misclassify the configuration. Shrink it back
         to misclassification (if such exists). */
        while (data.collisions_exist >= smallest_error &&
               data.collisions_exist == expect_has_collision) {
          std::cerr << "    shrinking\n";
          data.collisions_exist = !expect_has_collision;
          test_distance -= delta;

          const RigidTransformd T_A2 =
              RigidTransformd(test_distance * config.separating_dir_A);
          object_B.setTransform((X_WA * T_A2 * config.X_AB).GetAsIsometry3());
          Callback(&object_A, &object_B, &data);
        }

        const double test_magnitude = std::abs(test_distance);
        if (test_magnitude > max_observed_magnitude) {
          max_observed_description = config.description;
          max_observed_magnitude = test_magnitude;
        }
      }
    }
    EXPECT_LE(max_observed_magnitude, query.error + kDeltaMag)
        << "Expected error is too big!"
        << "\n  " << max_observed_description
        << "\n  Expected error at: " << query.error
        << "\n  Observed error at: " << max_observed_magnitude << "\n  For "
        << (expect_has_collision ? "penetrating" : "separated") << "objects";
    
    EXPECT_GE(max_observed_magnitude, query.error - kDeltaMag)
        << "Expected error is too small!"
        << "\n  " << max_observed_description
        << "\n  Expected error at: " << query.error
        << "\n  Observed error at: " << max_observed_magnitude << "\n  For "
        << (expect_has_collision ? "penetrating" : "separated") << "objects";
  }
};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    HasCollisionSeparated, CharacterizeHasCollisionsTest,
    testing::Values(
        // Separated

        // QueryInstance(kBox, kBox, kSeparated, 0.0),
        // QueryInstance(kBox, kCapsule, kSeparated, 0.0),
        // QueryInstance(kBox, kConvex, kSeparated, 0.0),
        // QueryInstance(kBox, kCylinder, kSeparated, 0.0),
        // QueryInstance(kBox, kEllipsoid, kSeparated, 0.0),
        // QueryInstance(kBox, kHalfSpace, kSeparated, 0.0),
        // QueryInstance(kBox, kSphere, kSeparated, 0.0),

        // QueryInstance(kCapsule, kCapsule, kSeparated, 0.0),
        // QueryInstance(kCapsule, kConvex, kSeparated, 0.0),
        // QueryInstance(kCapsule, kCylinder, kSeparated, 0.0),
        // QueryInstance(kCapsule, kEllipsoid, kSeparated, 0.0),
        // QueryInstance(kCapsule, kHalfSpace, kSeparated, 0.0),
        // QueryInstance(kCapsule, kSphere, kSeparated, 0.0),

        // QueryInstance(kConvex, kConvex, kSeparated, 0.0),
        // QueryInstance(kConvex, kCylinder, kSeparated, 0.0),
        // QueryInstance(kConvex, kEllipsoid, kSeparated, 0.0),
        // QueryInstance(kConvex, kHalfSpace, kSeparated, 0.0),
        // QueryInstance(kConvex, kSphere, kSeparated, 0.0),

        // QueryInstance(kCylinder, kCylinder, kSeparated, 0.0),
        // QueryInstance(kCylinder, kEllipsoid, kSeparated, 0.0),
        // QueryInstance(kCylinder, kHalfSpace, kSeparated, 0.0),
        // QueryInstance(kCylinder, kSphere, kSeparated, 0.0),

        // QueryInstance(kEllipsoid, kEllipsoid, kSeparated, 0.0),
        // QueryInstance(kEllipsoid, kHalfSpace, kSeparated, 0.0),
        // QueryInstance(kEllipsoid, kSphere, kSeparated, 0.0),

        QueryInstance(kHalfSpace, kHalfSpace, kSeparated, 0.0)
        // QueryInstance(kHalfSpace, kSphere, kSeparated, 0.0),

        // QueryInstance(kSphere, kSphere, kSeparated, 0.0),

        // Penetrating

        // QueryInstance(kBox, kBox, kPenetrating, 0.0),
        // QueryInstance(kBox, kCapsule, kPenetrating, 0.0),
        // QueryInstance(kBox, kConvex, kPenetrating, 0.0),
        // QueryInstance(kBox, kCylinder, kPenetrating, 0.0),
        // QueryInstance(kBox, kEllipsoid, kPenetrating, 0.0),
        // QueryInstance(kBox, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kBox, kSphere, kPenetrating, 0.0),

        // QueryInstance(kCapsule, kCapsule, kPenetrating, 0.0),
        // QueryInstance(kCapsule, kConvex, kPenetrating, 0.0),
        // QueryInstance(kCapsule, kCylinder, kPenetrating, 0.0),
        // QueryInstance(kCapsule, kEllipsoid, kPenetrating, 0.0),
        // QueryInstance(kCapsule, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kCapsule, kSphere, kPenetrating, 0.0),

        // QueryInstance(kConvex, kConvex, kPenetrating, 0.0),
        // QueryInstance(kConvex, kCylinder, kPenetrating, 0.0),
        // QueryInstance(kConvex, kEllipsoid, kPenetrating, 0.0),
        // QueryInstance(kConvex, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kConvex, kSphere, kPenetrating, 0.0),

        // QueryInstance(kCylinder, kCylinder, kPenetrating, 0.0),
        // QueryInstance(kCylinder, kEllipsoid, kPenetrating, 0.0),
        // QueryInstance(kCylinder, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kCylinder, kSphere, kPenetrating, 0.0),

        // QueryInstance(kEllipsoid, kEllipsoid, kPenetrating, 0.0),
        // QueryInstance(kEllipsoid, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kEllipsoid, kSphere, kPenetrating, 0.0),

        // QueryInstance(kHalfSpace, kHalfSpace, kPenetrating, 0.0),
        // QueryInstance(kHalfSpace, kSphere, kPenetrating, 0.0),

        // QueryInstance(kSphere, kSphere, kPenetrating, 0.0)
        ),
    QueryInstanceName);
// clang-format on

TEST_P(CharacterizeHasCollisionsTest, Characterize) {
  this->RunCharacterization(GetParam());
}


}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
