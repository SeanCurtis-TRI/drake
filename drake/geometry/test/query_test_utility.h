#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_engine_stub.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {

// Mock class to get access to the internal components of GeometryState.
template <typename T>
class GeometryStateTester {
 public:
  GeometryStateTester() : state_(nullptr) {}
  void set_state(GeometryState<T>* state) { state_ = state; }

  GeometryEngine<T>* get_engine() {
    return state_->geometry_engine_.get_mutable();
  }

  const std::vector<GeometryId>& get_index_to_id_map() {
    return state_->geometry_index_id_map_;
  }

 private:
  GeometryState<T>* state_;
};

namespace test {

// Convenience struct for associating data with pairs of indices. It can serve
// as a key in a set or map as long as the set/map is declared with this as
// the class as its own Hash class. E.g.,
//  unordered_map<IndexPair, float, IndexPair> pair_to_scalar_;
struct IdPair {
  GeometryId id1;
  GeometryId id2;
  IdPair() {}
  IdPair(GeometryId i1, GeometryId i2) : id1(i1), id2(i2) {}
  bool operator==(const IdPair& other) const {
    return id1 == other.id1 && id2 == other.id2;
  }
  // The pair serves is its own hash function.
  size_t operator()(const IdPair& p) const noexcept {
    size_t key = static_cast<size_t>(p.id1.get_value());
    return key << 32 | static_cast<size_t>(p.id2.get_value());
  }
};

class GeometryQueryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    state_tester_.set_state(&state_);
  }

  // Utility methods to create a sphere of the given radius.
  static std::unique_ptr <Shape> make_sphere(double radius) {
    return std::unique_ptr<Shape>(new Sphere(radius));
  }

  // Initialize the engine with four spheres. One at the origin and one each
  // at <1, 0, 0>, <0, 1, 0>, and <0, 0, 1>. The sphere at <1, 0, 0> is rotated
  // 90 degrees around the x-axis.
  void SetUpAxisSpheres();

  // A collection of operations to assert correctness of results. Confirms that
  // the nearest pair data is consistent when considering the first *n* spheres.
  void ExpectNearestPairs(
      const std::vector<NearestPair<double>> &results,
      const std::vector<IdPair> &pairs);

  // Source id for the frames/geometries.
  SourceId source_id_;
  // The state of the geometry world.
  GeometryState<double> state_;
  // The poses of the axis spheres.
  std::vector<Isometry3<double>> poses_;
  // The class that provides access to the GeometryState internals.
  GeometryStateTester<double> state_tester_;
  // Geometry indices for the axis-sphere configuration.
  static const double kRadius;
  static const int kSphereCount;

  // The expected results. A mapping from an IdPair to the nearest pair values
  // that should be generated.
  std::unordered_map<IdPair, NearestPair<double>, IdPair> expected_nearest_;
};

}  // namespace test
}  // namespace geometry
}  // namespace drake
