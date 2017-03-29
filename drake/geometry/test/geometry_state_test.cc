#include "drake/geometry/geometry_state.h"

#include <regex>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_channel.h"

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

// Confirms that requested channels are considered "open". This simultaneously
// tests channel request and channel "openness" query.
GTEST_TEST(GeometryStateTest, ChannelRequest) {
  GeometryState<double> state;
  ChannelId c1 = state.RequestChannelId();
  EXPECT_TRUE(state.ChannelIsOpen(c1));
  ChannelId c2 = ChannelId::get_new_id();
  EXPECT_FALSE(state.ChannelIsOpen(c2));
}

// Tests the request of frames. This simultaneously tests the request
// functionality as well as that the frame is properly registered with the
// channel.
GTEST_TEST(GeometryStateTest, FrameRequest) {
  GeometryState<double> state;
  ChannelId c1 = state.RequestChannelId();
  FrameId f1 = state.RequestFrameIdForChannel(c1);
  // Channel from frame should be the requesting channel.
  ChannelId parent = state.GetChannelId(f1);
  EXPECT_EQ(c1, parent);

  // Requesting a frame for a closed channel throws an exception
  ChannelId closed = ChannelId::get_new_id();
  try {
    state.RequestFrameIdForChannel(closed);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err ) {
    ExpectErrorMessage(err.what(), "Referenced channel \\d+ is not open.");
  }
}

// Tests attempts to declare geometry on frames.
GTEST_TEST(GeometryStateTest, GeometryRequest) {
  GeometryState<double> state;
  ChannelId c1 = state.RequestChannelId();
  FrameId f1 = state.RequestFrameIdForChannel(c1);

  // Tests "positive" behavior. Good parameters lead to good results.
  GeometryId g_id = state.RequestGeometryIdForFrame(c1, f1);
  FrameId parent_frame = state.GetFrameId(g_id);
  EXPECT_EQ(parent_frame, f1);
  ChannelId parent_channel = state.GetChannelId(g_id);
  EXPECT_EQ(parent_channel, c1);

  // Tests bad behavior, inconsistent parameters produce exceptions.

  // Case 1: Closed channel.
  ChannelId closed = ChannelId::get_new_id();
  try {
    g_id = state.RequestGeometryIdForFrame(closed, f1);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(), "Referenced channel \\d+ is not open.");
  }

  // Case 2: Frame doesn't belong to an open channel.
  FrameId invalid_frame = FrameId::get_new_id();
  try {
    g_id = state.RequestGeometryIdForFrame(c1, invalid_frame);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(), "Referenced frame \\d+ for channel \\d+\\."
    " But the frame doesn't belong to the channel.");
  }
}

// Tests the ChannelId from FrameId query with bad arguments. The positive test
// has been implicitly conducted above (see test FrameRequest).
GTEST_TEST(GeometryStateTest, ChannelFromFrame) {
  GeometryState<double> state;
  FrameId frame_id = FrameId::get_new_id();
  try {
    state.GetChannelId(frame_id);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced frame \\d+ has not been declared.");
  }
}

// Tests the FrameId from GeometryId query with bad arguments. The positive test
// has been implicitly tested above (see test GeometryRequest).
GTEST_TEST(GeometryStateTest, ChannelFromGeometry) {
  GeometryState<double> state;
  GeometryId g_id = GeometryId::get_new_id();
  try {
    state.GetChannelId(g_id);
    GTEST_FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
  }
}

// This confirms that closing a channel clears the data associated with the
// channel.
GTEST_TEST(GeometryStateTest, ClosingChannelClearsData) {
  GeometryState<double> state;
  ChannelId channel_id = state.RequestChannelId();

  // Creates k frames with n geometries each.
  const int frame_count = 2;
  const int geometry_count = 3;
  FrameId frames[frame_count];
  GeometryId geometries[frame_count * geometry_count];
  for (int f = 0; f < frame_count; ++f) {
    frames[f] = state.RequestFrameIdForChannel(channel_id);
    int geometry_position = f * geometry_count;
    for (int g = 0; g < geometry_count; ++g) {
      geometries[geometry_position++] =
          state.RequestGeometryIdForFrame(channel_id, frames[f]);
    }
  }
  // Confirms that the same channel is reachable from all geometries.
  for (int i = 0; i < frame_count * geometry_count; ++i) {
    EXPECT_EQ(state.GetChannelId(geometries[i]), channel_id);
  }

  state.CloseChannel(channel_id);
  EXPECT_FALSE(state.ChannelIsOpen(channel_id));
  // confirm frames have been closed
  for (int f = 0; f < frame_count; ++f) {
    try {
      state.GetChannelId(frames[f]);
      GTEST_FAIL();
    } catch (const std::runtime_error& err) {
      ExpectErrorMessage(err.what(),
                         "Referenced frame \\d+ has not been declared.");
    }
  }
  // confirm geometries have been closed
  for (int g = 0; g < frame_count * geometry_count; ++g) {
    try {
      state.GetChannelId(geometries[g]);
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
