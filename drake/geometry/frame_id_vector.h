#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Extend documentation to give usage examples.
//  Usage notes:
//    0. *Set* semantics. It is represented by an ordered structure for
//       performance reasons. However, the frame ids in the set must be unique.
//      a. Debug build provides data validation and correctness (to the extent
//         that it can.)
//    1. Expectation of constructing once and updating as you go.

/** Represents an _ordered_ set of frame identifiers. Instances of this class
 work in conjunction with instances of FramePoseVector and FrameVelocityVector
 to communicate frame kinematics to GeometryWorld and GeometrySystem. Taken in
 aggregate, the represent a "struct-of-arrays" paradigm. The iᵗʰ value in the
 %FrameIdVector represents the frame identifier whose position is specified by
 the iᵗʰ value in the corresponding FramePoseVector (and analogously for the
 FrameVelocityVector).

 The order of the frame ids can be arbitrary. However, the geometry source is
 required to include *all* frame ids that have been registered with its source
 identifier. Omitting a registered frame id is considered an error.

 @internal In future iterations, this will be relaxed to allow only
 communicating kinematics for frames that have _changed_. But in the initial
 version, this requirement is in place for pedagogical purposes to help users
 use GeometryWorld/GeometrySystem correctly. */
class FrameIdVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameIdVector)

  typedef std::vector<FrameId>::const_iterator Iterator;

  /** Empty constructor.
   @param source_id   The id for the geometry source reporting frame kinematics.
   */
  explicit FrameIdVector(SourceId source_id);

  /** Constructor which initializes the frame ids by _copying_ the given set.
   In Debug builds, the input ids will be tested for duplicates; an exception is
   thrown if duplicates are found.
   @param source_id   The id for the geometry source reporting frame kinematics.
   @param ids         The vector of ids which are copied into this vector.
   @throws std::logic_error (in Debug) if any of the ids are duplicated. */
  FrameIdVector(SourceId source_id, const std::vector<FrameId>& ids);

  /** Constructor which initializes the frame ids by _moving_ them from the
   given set. In Debug builds, the input ids will be tested for duplicates; an
   exception is thrown if duplicates are found.
   @param source_id   The id for the geometry source reporting frame kinematics.
   @param ids         The vector of ids which are moved into this vector.
   @throws std::logic_error (in Debug) if any of the ids are duplicated. */
  FrameIdVector(SourceId source_id, std::vector<FrameId>&& ids);

  /** Reports the source id for this data. */
  SourceId get_source_id() const { return source_id_; }

  /** Report the number of ids stored in the vector. */
  int size() const { return static_cast<int>(frame_ids_.size()); }

  /** Returns the iᵗʰ frame id. */
  FrameId get_frame_id(int i) const { return frame_ids_.at(i); }

  /** Returns the index of the given frame id.
   @throws std::logic_error if the frame id is not in the set. */
  int GetIndex(FrameId frame_id) const;

  /** Appends the given frame identifier to the set. In Debug build, the added
   identifier is confirmed to be unique and an exception is thrown if not.
   * @param frame_id    The frame identifier to add.
   * @return The total number of frame identifiers in the set.  */
  int AddFrameId(FrameId frame_id);

  /** Appends the given set of frame `ids` to the set. In Debug build, the
   resultant set is tested for duplicate frame ids.
   @param ids  The ordered set of frame ids to append to the current set.
   @throws std::logic_error (in Debug) if there are duplicate ids in the
                            resultant set. */
  int AddFrameIds(const std::vector<FrameId>& ids);

  /** Removes the given frame identifier from the set, reporting its former
   position in the vector. This is an O(N) operation on the average because the
   subsequent frames will be moved to maintain a compact representation.
   @param frame_id  The frame identifier to remove.
   @return The position of the frame id was previously occupying.
   @throws std::logic_error if the given identifier is not present in the set.
   */
  int RemoveFrameId(FrameId frame_id);

  /** Simply removes the frame id at position 'index'. */
  void RemoveFrameIdByIndex(int index);

  /** @name  Support for range-based loop iteration */
  //@{

  Iterator begin() const { return frame_ids_.cbegin(); }
  Iterator end() const { return frame_ids_.cend(); }

  // @}

 private:
  // Utility method for testing if the full set of frame identifiers are unique.
  // While this is O(N^2), it is only invoked
  void ThrowIfDuplicatesExist();

  // Throws an exception if the given frame_id is already in frame_ids_.
  void ThrowIfContains(FrameId frame_id);

  // The id of the reporting geometry source.
  SourceId source_id_;
  // The ordered set of moving frame identifiers.
  std::vector<FrameId> frame_ids_;
};
}  // namespace geometry
}  // namespace drake
