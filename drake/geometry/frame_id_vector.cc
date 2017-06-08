#include "drake/geometry/frame_id_vector.h"

#include <string>
#include <utility>

namespace drake {
namespace geometry {

using std::move;

FrameIdVector::FrameIdVector(SourceId source_id) : source_id_(source_id) {}

FrameIdVector::FrameIdVector(SourceId source_id,
                             const std::vector<FrameId>& ids) :
    source_id_(source_id), frame_ids_(ids) {
  DRAKE_ASSERT_VOID(ThrowIfDuplicatesExist());
}

FrameIdVector::FrameIdVector(SourceId source_id, std::vector<FrameId>&& ids) :
    source_id_(source_id), frame_ids_(move(ids)) {
  DRAKE_ASSERT_VOID(ThrowIfDuplicatesExist());
}

int FrameIdVector::GetIndex(FrameId frame_id) const {
  for (size_t i = 0; i < frame_ids_.size(); ++i) {
    if (frame_ids_[i] == frame_id) return static_cast<int>(i);
  }
  using std::to_string;
  throw std::logic_error("The given frame id is not in the set: " +
                         to_string(frame_id) + ".");
}

int FrameIdVector::AddFrameId(FrameId frame_id) {
  DRAKE_ASSERT_VOID(ThrowIfContains(frame_id));
  frame_ids_.push_back(frame_id);
  return static_cast<int>(frame_ids_.size());
}

int FrameIdVector::AddFrameIds(const std::vector<FrameId>& ids) {
  frame_ids_.insert(frame_ids_.end(), ids.begin(), ids.end());
  DRAKE_ASSERT_VOID(ThrowIfDuplicatesExist());
  return static_cast<int>(frame_ids_.size());
}

int FrameIdVector::RemoveFrameId(FrameId frame_id) {
  for (auto itr = frame_ids_.begin(); itr != frame_ids_.end(); ++itr) {
    if (*itr == frame_id) {
      int i = static_cast<int>(itr - frame_ids_.begin());
      frame_ids_.erase(itr);
      return i;
    }
  }
  using std::to_string;
  throw std::logic_error("Cannot remove frame identifier " +
                         to_string(frame_id) +
                         ". It does not belong to the set.");
}

void FrameIdVector::RemoveFrameIdByIndex(int index) {
  frame_ids_.erase(frame_ids_.begin() + index);
}

void FrameIdVector::ThrowIfDuplicatesExist() {
  const int count = size();
  for (int i = 0; i < count - 1; ++i) {
    FrameId f_i = frame_ids_[i];
    for (int j = i + 1; j < count; ++j) {
      FrameId f_j = frame_ids_[j];
      if (f_i == f_j) {
        throw std::logic_error("The frame id vector contains duplicate frame"
                                   " ids, including, at least, " +
            to_string(f_i) + ".");
      }
    }
  }
}

void FrameIdVector::ThrowIfContains(FrameId frame_id) {
  for (FrameId test_id : frame_ids_) {
    if (test_id == frame_id) {
      using std::to_string;
      throw std::logic_error("Id vector already contains frame id: " +
          to_string(frame_id) + ".");
    }
  }
}

}  // namespace geometry
}  // namespace drake
