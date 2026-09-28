#include <vector>

#include <benchmark/benchmark.h>

#include "drake/common/random.h"
#include "drake/geometry/proximity/ccd.h"
#include "drake/geometry/proximity/test/ccd_test_configurations.h"

// Measures the CCD narrowphase kernels (point_triangle_ccd / edge_edge_ccd)
// over batches drawn from the same generator families as the differential
// test — mixed hits and misses, per adversarial family. This is the
// benchmark that guards the "global CCD runs every time step, so the
// narrowphase must be very fast" requirement while the kernels are hardened.
//
// See geometry/proximity/test/ccd_test_configurations.h for the families.

namespace drake {
namespace geometry {
namespace internal {
namespace {

constexpr int kBatchSize = 1024;

std::vector<CcdQueryCase> MakeBatch(CcdQueryFamily family, bool vertex_face) {
  RandomGenerator generator(1234);
  std::vector<CcdQueryCase> batch;
  batch.reserve(kBatchSize);
  for (int i = 0; i < kBatchSize; ++i) {
    // Even entries hit, odd entries miss — a branchy 50/50 mix.
    if (vertex_face) {
      batch.push_back((i % 2 == 0) ? MakeVertexFaceHit(family, &generator)
                                   : MakeVertexFaceMiss(family, &generator));
    } else {
      batch.push_back((i % 2 == 0) ? MakeEdgeEdgeHit(family, &generator)
                                   : MakeEdgeEdgeMiss(family, &generator));
    }
  }
  return batch;
}

void RunPointTriangleBatch(benchmark::State& state,  // NOLINT
                           CcdQueryFamily family) {
  const std::vector<CcdQueryCase> batch = MakeBatch(family, true);
  for (auto _ : state) {
    for (const auto& c : batch) {
      double toi;
      benchmark::DoNotOptimize(point_triangle_ccd(c.x[0], c.x[1], c.x[2],
                                                  c.x[3], c.x[4], c.x[5],
                                                  c.x[6], c.x[7], &toi));
    }
  }
  state.SetItemsProcessed(state.iterations() * kBatchSize);
}

void RunEdgeEdgeBatch(benchmark::State& state,  // NOLINT
                      CcdQueryFamily family) {
  const std::vector<CcdQueryCase> batch = MakeBatch(family, false);
  for (auto _ : state) {
    for (const auto& c : batch) {
      double toi;
      benchmark::DoNotOptimize(edge_edge_ccd(c.x[0], c.x[1], c.x[2], c.x[3],
                                             c.x[4], c.x[5], c.x[6], c.x[7],
                                             &toi));
    }
  }
  state.SetItemsProcessed(state.iterations() * kBatchSize);
}

void PointTriangleClean(benchmark::State& state) {  // NOLINT
  RunPointTriangleBatch(state, CcdQueryFamily::kClean);
}
void PointTriangleAdversarial(benchmark::State& state) {  // NOLINT
  RunPointTriangleBatch(state, CcdQueryFamily::kFastTranslationTinyRotation);
}
void PointTriangleBoundaryTime(benchmark::State& state) {  // NOLINT
  RunPointTriangleBatch(state, CcdQueryFamily::kBoundaryTime);
}
void EdgeEdgeClean(benchmark::State& state) {  // NOLINT
  RunEdgeEdgeBatch(state, CcdQueryFamily::kClean);
}
void EdgeEdgeAdversarial(benchmark::State& state) {  // NOLINT
  RunEdgeEdgeBatch(state, CcdQueryFamily::kFastTranslationTinyRotation);
}
void EdgeEdgeBoundaryTime(benchmark::State& state) {  // NOLINT
  RunEdgeEdgeBatch(state, CcdQueryFamily::kBoundaryTime);
}

BENCHMARK(PointTriangleClean)->Unit(benchmark::kMicrosecond);
BENCHMARK(PointTriangleAdversarial)->Unit(benchmark::kMicrosecond);
BENCHMARK(PointTriangleBoundaryTime)->Unit(benchmark::kMicrosecond);
BENCHMARK(EdgeEdgeClean)->Unit(benchmark::kMicrosecond);
BENCHMARK(EdgeEdgeAdversarial)->Unit(benchmark::kMicrosecond);
BENCHMARK(EdgeEdgeBoundaryTime)->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
