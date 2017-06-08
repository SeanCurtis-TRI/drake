#include "drake/geometry/frame_id_vector.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/test/expect_error_message.h"

namespace drake {
namespace geometry {
namespace {

using std::vector;

// Simply tests successful construction.
GTEST_TEST(FrameIdVector, ConstructorSuccess) {
  SourceId source_id = SourceId::get_new_id();

  // Case: Empty set.
  FrameIdVector ids1(source_id);
  EXPECT_EQ(ids1.get_source_id(), source_id);
  EXPECT_EQ(ids1.size(), 0);

  // Case: Copy from vector.
  vector<FrameId> frames{FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id()};
  const int frame_count = static_cast<int>(frames.size());
  FrameIdVector ids2(source_id, frames);
  EXPECT_EQ(ids2.get_source_id(), source_id);
  EXPECT_EQ(ids2.size(), frame_count);
  // Confirms successful copy.
  for (int i = 0; i < frame_count; ++i) {
    EXPECT_EQ(ids2.get_frame_id(i), frames[i]);
  }

  // Case: Move from vector.
  for (int i = 0; i < frame_count; ++i) {
    frames[i] = FrameId::get_new_id();
  }
  FrameIdVector ids3(source_id, move(frames));
  EXPECT_EQ(ids3.get_source_id(), source_id);
  EXPECT_EQ(ids3.size(), frame_count);
  EXPECT_EQ(frames.size(), 0);
  for (int i = 0; i < frame_count; ++i) {
    EXPECT_NE(ids2.get_frame_id(i), ids3.get_frame_id(i));
  }
}

// Tests the range iterators.
GTEST_TEST(FrameIdVector, RangeIteration) {
  vector<FrameId> frames{FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id()};
  FrameIdVector ids(SourceId::get_new_id(), frames);
  int i = 0;
  for (auto id : ids) {
    EXPECT_EQ(id, frames[i++]);
  }
}

// Tests conditions where input vector of ids contain duplicates.
GTEST_TEST(FrameIdVector, ConstructorWithDuplicates) {
  SourceId source_id = SourceId::get_new_id();
  vector<FrameId> frames{FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id()};
  frames.push_back(frames[0]);

  // Case: Construct by copying frames.
  EXPECT_ERROR_MESSAGE_IF_ARMED(FrameIdVector(source_id, frames),
                                std::logic_error,
                                "The frame id vector contains duplicate frame "
                                "ids, including, at least, \\d+.");

  // Case: Construct by moving frames.
  EXPECT_ERROR_MESSAGE_IF_ARMED(FrameIdVector(source_id, move(frames)),
                                std::logic_error,
                                "The frame id vector contains duplicate frame "
                                "ids, including, at least, \\d+.");
}

// Tests the functionality for adding single frames to the set.
GTEST_TEST(FrameIdVector, AddingFramesSingle) {
  FrameIdVector ids(SourceId::get_new_id());
  // Do *not* re-order these tests; the logic depends on the sequence.
  // Case: Add single to empty.
  FrameId f0 = FrameId::get_new_id();
  int report_count = -1;
  EXPECT_NO_THROW(report_count = ids.AddFrameId(f0));
  EXPECT_EQ(report_count, 1);
  EXPECT_EQ(ids.size(), 1);
  EXPECT_EQ(ids.get_frame_id(0), f0);

  // Case: Add single to non-empty (unique).
  FrameId f1 = FrameId::get_new_id();
  EXPECT_NO_THROW(report_count = ids.AddFrameId(f1));
  EXPECT_EQ(report_count, 2);
  EXPECT_EQ(ids.size(), 2);
  EXPECT_EQ(ids.get_frame_id(1), f1);

  // Case: Add single to non-empty (not unique).
  EXPECT_ERROR_MESSAGE_IF_ARMED(ids.AddFrameId(f0),
                                std::logic_error,
                                "Id vector already contains frame id: \\d+.");
}

// Tests the functionality for adding multiple frames to the set.
GTEST_TEST(FrameIdVector, AddingFramesMultiple) {
  vector<FrameId> unique1{FrameId::get_new_id(),
                          FrameId::get_new_id(),
                          FrameId::get_new_id(),
                          FrameId::get_new_id()};
  vector<FrameId> unique2{FrameId::get_new_id(),
                          FrameId::get_new_id(),
                          FrameId::get_new_id(),
                          FrameId::get_new_id()};
  vector<FrameId> duplicate{unique1[0]};
  FrameIdVector ids(SourceId::get_new_id());
  // Do *not* re-order these tests; the logic depends on the sequence.

  // Case: Add multiple to empty (all unique).
  int report_count = -1;
  EXPECT_NO_THROW(report_count = ids.AddFrameIds(unique1));
  EXPECT_EQ(report_count, static_cast<int>(unique1.size()));
  EXPECT_EQ(ids.size(), report_count);
  for (int i = 0; i < static_cast<int>(unique1.size()); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), unique1[i]);
  }

  // Case: Add multiple to non-empty (unique result).
  EXPECT_NO_THROW(report_count = ids.AddFrameIds(unique2));
  EXPECT_EQ(report_count, static_cast<int>(unique1.size() + unique2.size()));
  EXPECT_EQ(ids.size(), report_count);
  int i = 0;
  for (; i < static_cast<int>(unique1.size()); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), unique1[i]);
  }
  for (int j = 0; i < static_cast<int>(unique1.size() + unique2.size());
       ++i, ++j) {
    EXPECT_EQ(ids.get_frame_id(i), unique2[j]);
  }

  // Case: Add multiple to non-empty (non-unique result).
  EXPECT_ERROR_MESSAGE_IF_ARMED(ids.AddFrameIds(duplicate),
                                std::logic_error,
                                "The frame id vector contains duplicate frame "
                                "ids, including, at least, \\d+.");
}

// Tests the functionality that tries to report the index of the desired frame.
GTEST_TEST(FrameIdVector, FrameLookup) {
  vector<FrameId> frames{FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id()};
  FrameIdVector ids(SourceId::get_new_id(), frames);
  int index = -1;
  int expected_index = 2;
  EXPECT_NO_THROW(index = ids.GetIndex(frames[expected_index]));
  EXPECT_EQ(index, expected_index);

  EXPECT_ERROR_MESSAGE(ids.GetIndex(FrameId::get_new_id()),
                       std::logic_error,
                       "The given frame id is not in the set: \\d+.");
}

// Tests frame removal from the set.
GTEST_TEST(FrameIdVector, RemoveFrames) {
  vector<FrameId> frames{FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id(),
                         FrameId::get_new_id()};
  FrameIdVector ids(SourceId::get_new_id(), frames);
  const int target_index = 1;
  FrameId removed = frames[target_index];
  frames.erase(frames.begin() + target_index);

  // Case remove frame via valid frame identifier.
  int report_index = -1;
  EXPECT_NO_THROW(report_index = ids.RemoveFrameId(removed));
  EXPECT_EQ(report_index, target_index);
  EXPECT_EQ(ids.size(), static_cast<int>(frames.size()));
  for (int i = 0; i < ids.size(); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), frames[i]);
  }

  // Case: remove invalid frame id.
  EXPECT_ERROR_MESSAGE(ids.RemoveFrameId(FrameId::get_new_id()),
                       std::logic_error,
                       "Cannot remove frame identifier \\d+. "
                       "It does not belong to the set.");

  frames.erase(frames.begin());
  EXPECT_NO_THROW(ids.RemoveFrameIdByIndex(0));
  for (int i = 0; i < ids.size(); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), frames[i]);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
