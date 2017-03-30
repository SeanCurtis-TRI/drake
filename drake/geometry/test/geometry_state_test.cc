#include "drake/geometry/geometry_state.h"

#include <regex>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace  {

using std::regex;
using std::regex_match;

// Helper method; verifies exception messages against regular expressions.
// This provides test failure messages that are easiser to grok.
void ExpectErrorMessage(const char* err_msg, const char* reg_exp) {
  auto matcher = [](const char* s, const char* re) {
    return regex_match(s, regex(re)); };
  EXPECT_PRED2(matcher, err_msg, reg_exp);
}

// Confirms that requested source are considered "active". This simultaneously
// tests source registration and "activeness" query.
GTEST_TEST(GeometryStateTest, SourceRequest) {
  GeometryState<double> state;
  SourceId s1 = state.RequestSourceId();
  EXPECT_TRUE(state.SourceIsActive(s1));
  SourceId s2 = SourceId::get_new_id();
  EXPECT_FALSE(state.SourceIsActive(s2));
}

// Tests the request of frames. This simultaneously tests the request
// functionality as well as that the frame is properly registered with the
// source.
GTEST_TEST(GeometryStateTest, FrameRequestValidSource) {
  GeometryState<double> state;
  SourceId s1 = state.RequestSourceId();
  FrameId f1 = state.RequestFrameIdForSource(s1);
  // Source from frame should be the requesting source.
  SourceId parent = state.GetSourceId(f1);
  EXPECT_EQ(s1, parent);
}

// Requesting a frame for an inactive source throws an exception.
GTEST_TEST(GeometryStateTest, FrameRequestBadSource) {
  GeometryState<double> state;
  SourceId closed = SourceId::get_new_id();
  try {
    state.RequestFrameIdForSource(closed);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err ) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry source \\d+ is not active.");
  }
}

// Tests attempts to declare geometry on frames.
GTEST_TEST(GeometryStateTest, GeometryRequestGoodSource) {
  GeometryState<double> state;bazel
  SourceId s1 = state.RequestSourceId();
  FrameId f1 = state.RequestFrameIdForSource(s1);

  // Tests "positive" behavior. Good parameters lead to good results.
  GeometryId g_id = state.RequestGeometryIdForFrame(s1, f1);
  FrameId parent_frame = state.GetFrameId(g_id);
  EXPECT_EQ(parent_frame, f1);
  SourceId parent_source = state.GetSourceId(g_id);
  EXPECT_EQ(parent_source, s1);
}

// Tests attempts to declare geometry on frames with an inactive source.
GTEST_TEST(GeometryStateTest, GeometryRequestBadSource) {
  GeometryState<double> state;
  FrameId f1 = FrameId::get_new_id();

  SourceId closed = SourceId::get_new_id();
  try {
    state.RequestGeometryIdForFrame(closed, f1);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry source \\d+ is not active.");
  }
}

// Frame doesn't belong to an inactive source.
GTEST_TEST(GeometryStateTest, GeometryRequestBadFrame) {
  GeometryState<double> state;
  SourceId s1 = state.RequestSourceId();

  FrameId invalid_frame = FrameId::get_new_id();
  try {
    state.RequestGeometryIdForFrame(s1, invalid_frame);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(), "Referenced frame \\d+ for source \\d+\\."
        " But the frame doesn't belong to the source.");
  }
}

// Tests the SourceId from FrameId query with bad arguments. The positive test
// has been implicitly conducted above (see test FrameRequest).
GTEST_TEST(GeometryStateTest, SourceFromFrame) {
  GeometryState<double> state;
  FrameId frame_id = FrameId::get_new_id();
  try {
    state.GetSourceId(frame_id);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced frame \\d+ has not been declared.");
  }
}

// Tests the FrameId from GeometryId query with bad arguments. The positive test
// has been implicitly tested above (see test GeometryRequest).
GTEST_TEST(GeometryStateTest, SourceFromGeometry) {
  GeometryState<double> state;
  GeometryId g_id = GeometryId::get_new_id();
  try {
    state.GetSourceId(g_id);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
  }
}

// This confirms that removing a source clears the data associated with the
// source.
GTEST_TEST(GeometryStateTest, RemoveSourceClearsData) {
  GeometryState<double> state;
  SourceId source_id = state.RequestSourceId();

  // Creates k frames with n geometries each.
  const int frame_count = 2;
  const int geometry_count = 3;
  FrameId frames[frame_count];
  GeometryId geometries[frame_count * geometry_count];
  for (int f = 0; f < frame_count; ++f) {
    frames[f] = state.RequestFrameIdForSource(source_id);
    int geometry_position = f * geometry_count;
    for (int g = 0; g < geometry_count; ++g) {
      geometries[geometry_position++] =
          state.RequestGeometryIdForFrame(source_id, frames[f]);
    }
  }
  // Confirms that the same source is reachable from all geometries.
  for (int i = 0; i < frame_count * geometry_count; ++i) {
    EXPECT_EQ(state.GetSourceId(geometries[i]), source_id);
  }

  state.RemoveSource(source_id);
  EXPECT_FALSE(state.SourceIsActive(source_id));
  // confirm frames have been closed
  for (int f = 0; f < frame_count; ++f) {
    try {
      state.GetSourceId(frames[f]);
      GTEST_FAIL();
    } catch (const std::runtime_error& err) {
      ExpectErrorMessage(err.what(),
                         "Referenced frame \\d+ has not been declared.");
    }
  }
  // confirm geometries have been closed
  for (int g = 0; g < frame_count * geometry_count; ++g) {
    try {
      state.GetSourceId(geometries[g]);
      GTEST_FAIL();
    } catch (const std::runtime_error& err) {
      ExpectErrorMessage(err.what(),
                         "Referenced geometry \\d+ does not belong to a known "
                         "frame.");
    }
  }
}
}  // namespace
}  // namespace geometry
}  // namespace drake
