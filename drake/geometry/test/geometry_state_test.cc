#include "drake/geometry/geometry_state.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/shapes.h"
#include "drake/geometry/test/expect_error_message.h"
#include "drake/geometry/test/geometry_world_stub.h"

namespace drake {
namespace geometry {
// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
  using State = GeometryState<T>;

 public:
  void set_state(const State* state) { state_ = state; }

  FrameId get_world_frame() {
    return state_->kWorldFrame;
  }

  const SourceFrameIdMap& get_source_frame_id_map() {
    return state_->source_frame_id_map_;
  }

  const SourceFrameIdMap& get_source_root_frame_map() {
    return state_->source_root_frame_map_;
  }

  const FrameIdFrameMap& get_frames() {
    return state_->frames_;
  }

  const GeometryIdGeometryMap& get_geometries() {
    return state_->geometries_;
  }

  const std::vector<GeometryId>& get_geometry_index_id_map() {
    return state_->geometry_index_id_map_;
  }

  const std::vector<Isometry3<T>>& get_geometry_frame_poses() {
    return state_->X_FG_;
  }

  const std::vector<Isometry3<T>>& get_geometry_world_poses() {
    return state_->X_WG_;
  }

  const std::vector<Isometry3<T>>& get_frame_parent_poses() {
    return state_->X_PF_;
  }

 private:
  const State* state_;
};

namespace {

using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    frame_ = make_unique<GeometryFrame<double>>("ref_frame",
                                                Isometry3<double>::Identity());
    instance_pose_.translation() << 10, 20, 30;
    instance_ = make_unique<GeometryInstance<double>>(
        instance_pose_, unique_ptr<Shape>(new Sphere(1.0)));
    gs_tester_.set_state(&geometry_state_);
  }

  // Utility method for adding a source to the state.
  SourceId NewSource() {
    SourceId s_id = SourceId::get_new_id();
    geometry_state_.RegisterNewSource(s_id);
    return s_id;
  }

  // This method sets up a dummy tree to facilitate testing. It creates a single
  // source with a fixed configuration of frames with a pre-determined number of
  // geometries per frame.
  //
  //  Creates the following tree:
  //                                        s_id
  //                                        /  \
  //                                       f0   f1
  //                                      / |    |________
  //                                    g0  g1   |   |   |
  //                                             f2  g2  g3
  //                                            / \
  //                                           g4 g5
  //
  // Frame configuration
  //  f0 is @ <1, 2, 3>, with a 90-degree rotation around x.
  //  f1 is @ <10, 20, 30>, with a 90-degree rotation around y.
  //  f2 is @ <-10, -20, -30>, with a -90-degree rotation around y
  // Geometry configuration
  //  gi is at position <i + 1, 0, 0>, with a rotation of iπ/2 radians around
  //    the x-axis.
  // The relationship between f1 and f2 is such that for g2 & g3, the pose
  // relative to the parent frame f2 is the same as to the world, e.g.,
  // X_PG = X_WG.
  SourceId SetUpSingleSourceTree() {
    using std::to_string;

    source_id_ = NewSource();

    // Create f0.
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << 1, 2, 3;
    pose.linear() << 1, 0, 0, 0, 0, 1, 0, -1, 0;  // 90° around x-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame<double>("f0", pose)));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f1.
    pose.translation() << 10, 20, 30;
    pose.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;  // 90° around y-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame<double>("f1", pose)));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f2.
    pose.translation() << -10, -20, -30;
    pose.linear() << 0, 0, 1, 0, 1, 0, -1, 0, 0;  // -90° around y-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, frames_[1], GeometryFrame<double>("f2", pose)));
    X_WF_.push_back(X_WF_[1] * pose);
    X_PF_.push_back(pose);

    // Add geometries to each frame.
    geometries_.resize(kFrameCount * kGeometryCount);
    int g_count = 0;
    const Vector3<double> x_axis(1, 0, 0);
    for (auto frame_id : frames_) {
      for (int i = 0; i < kGeometryCount; ++i) {
        pose.translation() << g_count + 1, 0, 0;
        pose.linear() =
            AngleAxis<double>(g_count * M_PI / 2.0, x_axis).matrix();
        geometries_[g_count] = geometry_state_.RegisterGeometry(
            source_id_, frame_id,
            make_unique<GeometryInstance<double>>(
                pose, std::unique_ptr<Shape>(new Sphere(1))));
        X_FG_.push_back(pose);
        ++g_count;
      }
    }
    return source_id_;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const { return kFrameCount; }
  int single_tree_geometry_count() const {
    return kFrameCount * kGeometryCount;
  }

  // This method confirms that the stored dummy identifiers don't map to any
  // active source identifier. This should only be invoked for scenarios where
  // there is *only* the single source.
  void AssertSingleTreeCleared() {
    // confirm frames have been closed
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(frames_[f]),
                           std::logic_error,
                           "Referenced frame \\d+ has not been registered.");
    }
    // confirm geometries have been closed
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(geometries_[g]),
                           std::logic_error,
                           "Referenced geometry \\d+ does not belong to a known"
                           " frame.");
    }
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_frames().size(), 0);
    EXPECT_EQ(gs_tester_.get_geometries().size(), 0);
    // TODO(SeanCurtis-TRI): After poses are removed/re-ordered confirm that the
    // poses have been removed.
  }

  // Utility function for facilitating tests; confirms that the identified
  // frame doesn't belong to the identified source. This explicitly tests the
  // underlying state data structures.
  void ExpectSourceDoesNotHaveFrame(SourceId source_id, FrameId frame_id) {
    // Frame is not in the source-to-set-of-frame-ids mapping.
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(source_id).find(frame_id),
              gs_tester_.get_source_frame_id_map().at(source_id).end());
    // Frame is not in the source-to-set-of-root-ids mapping.
    EXPECT_EQ(
        gs_tester_.get_source_root_frame_map().at(source_id).find(frame_id),
        gs_tester_.get_source_root_frame_map().at(source_id).end());
    // Frame not in frames
    EXPECT_EQ(gs_tester_.get_frames().find(frame_id),
              gs_tester_.get_frames().end());
    // TODO(SeanCurtis-TRI): After removing and reordering frame and geometry
    // poses, check for a reduction in pose counts.
  }

  // Members owned by the test class.
  unique_ptr<GeometryFrame<double>> frame_;
  unique_ptr<GeometryInstance<double>> instance_;
  Isometry3<double> instance_pose_{Isometry3<double>::Identity()};
  GeometryState<double> geometry_state_;
  GeometryStateTester<double> gs_tester_;

  // Values for setting up and testing the dummy tree.
  enum Counts {
    kFrameCount = 3,
    kGeometryCount = 2    // Geometries *per frame*.
  };
  // The frame ids created in the dummy tree instantiation.
  std::vector<FrameId> frames_;
  // The geometry ids created in the dummy tree instantiation.
  std::vector<GeometryId> geometries_;
  // THe id of the single-source tree.
  SourceId source_id_;

  // The poses of the frames in the world frame.
  std::vector<Isometry3<double>> X_WF_;
  // The poses of the frames in the parent's frame.
  std::vector<Isometry3<double>> X_PF_;
  // The poses of the geometries in the parent frame.
  std::vector<Isometry3<double>> X_FG_;
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  EXPECT_EQ(geometry_state_.get_num_sources(), 0);
  EXPECT_EQ(geometry_state_.get_num_frames(), 0);
  EXPECT_EQ(geometry_state_.get_num_geometries(), 0);
}

// Tests the geometry statistics values. It uses the single-source tree to
// create a state with interesting metrics. Also confirms the "is active"-ness
// of known valid sources and known invalid sources.
TEST_F(GeometryStateTest, GeometryStatistics) {
  SourceId dummy_source = SetUpSingleSourceTree();
  EXPECT_TRUE(geometry_state_.source_is_active(dummy_source));
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.get_num_geometries(), single_tree_geometry_count());
  SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.source_is_active(false_id));
}

// Confirms that the actions of initializing the single-source tree leave the
// geometry state in the expected configuration.
TEST_F(GeometryStateTest, ValidateSingleSourceTree) {
  SourceId s_id = SetUpSingleSourceTree();

  // The source has *direct* access to all registered frames.
  {
    const auto& s_f_id_map = gs_tester_.get_source_frame_id_map();
    EXPECT_EQ(s_f_id_map.size(), 1);
    EXPECT_NE(s_f_id_map.find(s_id), s_f_id_map.end());
    const auto &f_id_set = s_f_id_map.at(s_id);
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_NE(f_id_set.find(frames_[f]), f_id_set.end());
    }
  }

  // The root map *only* includes the root frames. Frames 0 & 1 *should* be
  // included; frame 2 should *not*.
  {
    const auto& s_root_map = gs_tester_.get_source_root_frame_map();
    EXPECT_EQ(s_root_map.size(), 1);
    EXPECT_NE(s_root_map.find(s_id), s_root_map.end());
    const auto &root_id_set = s_root_map.at(s_id);
    EXPECT_NE(root_id_set.find(frames_[0]), root_id_set.end());
    EXPECT_NE(root_id_set.find(frames_[1]), root_id_set.end());
    EXPECT_EQ(root_id_set.find(frames_[2]), root_id_set.end());
  }

  // The internal frames are what and where they should be.
  {
    using std::to_string;
    const auto& internal_frames = gs_tester_.get_frames();
    EXPECT_EQ(internal_frames.size(), kFrameCount);
    auto test_frame = [internal_frames, this, s_id](int i, FrameId parent_id,
                                                    int num_child_frames) {
      const auto& frame = internal_frames.at(frames_[i]);
      EXPECT_EQ(frame.get_source_id(), s_id);
      EXPECT_EQ(frame.get_id(), frames_[i]);
      EXPECT_EQ(frame.get_name(), "f" + to_string(i));
      EXPECT_EQ(frame.get_frame_group(), 0);  // Defaults to zero.
      EXPECT_EQ(frame.get_pose_index(), i);   // ith frame added.
      EXPECT_EQ(frame.get_parent_frame_id(), parent_id);
      EXPECT_EQ(frame.get_child_frames().size(), num_child_frames);
      const auto& child_geometries = frame.get_child_geometries();
      EXPECT_EQ(child_geometries.size(), 2);
      EXPECT_NE(child_geometries.find(geometries_[i * 2]),
                                      child_geometries.end());
      EXPECT_NE(child_geometries.find(geometries_[i * 2 + 1]),
                                      child_geometries.end());
      const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
      EXPECT_TRUE(
          CompareMatrices(frame_in_parent[frame.get_pose_index()].matrix(),
                          X_PF_[i].matrix()));
    };
    test_frame(0, gs_tester_.get_world_frame(), 0);
    test_frame(1, gs_tester_.get_world_frame(), 1);
    test_frame(2, frames_[1], 0);
  }

  // The internal geometries are what and where they should be.
  {
    const auto& internal_geometries = gs_tester_.get_geometries();
    EXPECT_EQ(internal_geometries.size(), kFrameCount * kGeometryCount);
    auto test_geometry = [internal_geometries, this, s_id](int i) {
      const auto& geometry = internal_geometries.at(geometries_[i]);
      EXPECT_EQ(geometry.get_frame_id(), frames_[i / 2]);
      EXPECT_EQ(geometry.get_id(), geometries_[i]);
      // TODO(SeanCurtis-TRI): Update this when names are being used.
      EXPECT_EQ(geometry.get_name(), "no_name");
      EXPECT_EQ(geometry.get_engine_index(), i);
      EXPECT_EQ(geometry.get_child_geometries().size(), 0);
      EXPECT_FALSE(geometry.get_parent());
      const auto& geo_in_frame = gs_tester_.get_geometry_frame_poses();
      EXPECT_TRUE(
          CompareMatrices(geo_in_frame[geometry.get_engine_index()].matrix(),
                          X_FG_[i].matrix()));
      EXPECT_EQ(
          gs_tester_.get_geometry_index_id_map()[geometry.get_engine_index()],
          geometry.get_id());
    };
    for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
      test_geometry(i);
    }
  }
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  SourceId s_id = SourceId::get_new_id();  // This is not an active source.
  ASSERT_ERROR_MESSAGE(geometry_state_.RegisterFrame(s_id, *frame_.get()),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");
}

// Tests that a frame added to a valid source can be used to acquire that source
// and appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  SourceId s_id = NewSource();
  FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_.get());
  EXPECT_EQ(geometry_state_.GetSourceId(fid), s_id);
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), 1);
  // The frame *is* a root frame; so both sets should be the same size.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size(),
            gs_tester_.get_source_root_frame_map().at(s_id).size());
}

// Tests that a frame added to a valid source which already has frames is
// correctly appended.
TEST_F(GeometryStateTest, AddFrameToSourceWithFrames) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_);
  EXPECT_EQ(geometry_state_.GetSourceId(fid), s_id);
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), kFrameCount + 1);
  // The added frame *is* a root frame. The single-source tree has *one*
  // non-root frame. This "off-by-one" property should be preserved.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size() - 1,
            gs_tester_.get_source_root_frame_map().at(s_id).size());
}

// Tests that a frame added to a new source doesn't modify previously existing
// sources.
TEST_F(GeometryStateTest, AddFrameToNewSourceWithFrames) {
  SourceId s_id = SetUpSingleSourceTree();
  SourceId new_s_id = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(new_s_id);
  FrameId fid = geometry_state_.RegisterFrame(new_s_id, *frame_.get());
  // Confirm addition.
  EXPECT_EQ(geometry_state_.GetSourceId(fid), new_s_id);
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(new_s_id);
    EXPECT_NE(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), 1);
  }
  // Confirm original source is unchanged.
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
    EXPECT_EQ(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), kFrameCount);
  }
}

// Tests the functionality of adding a frame to another frame.
TEST_F(GeometryStateTest, AddFrameOnFrame) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  EXPECT_EQ(geometry_state_.GetSourceId(fid), s_id);

  // Test parent-child relationship wiring.
  //  Difference between total and root frames. I now have *two* non-root
  // frames.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size() - 2,
            gs_tester_.get_source_root_frame_map().at(s_id).size());
  //  Frame for this id has frame_[0] as parent.
  const auto& frame = gs_tester_.get_frames().at(fid);
  EXPECT_TRUE(frame.has_parent(frames_[0]));
  //  Parent frame has this as child.
  const auto& parent = gs_tester_.get_frames().at(frames_[0]);
  EXPECT_TRUE(parent.has_child(fid));
}

// Tests the valid removal of an existing frame (and its attached geometry).
TEST_F(GeometryStateTest, RemoveFrame) {
  SourceId s_id = SetUpSingleSourceTree();
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveFrame(s_id, frames_[0]);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount - 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            (kFrameCount -1)* kGeometryCount);

  ExpectSourceDoesNotHaveFrame(s_id, frames_[0]);
}

// Tests the removal of a frame that has other frames hanging on it.
TEST_F(GeometryStateTest, RemoveFrameTree) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  geometry_state_.RemoveFrame(s_id, frames_[0]);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount - 1);

  ExpectSourceDoesNotHaveFrame(s_id, frames_[0]);
  ExpectSourceDoesNotHaveFrame(s_id, fid);
}

// Tests the removal of a frame whose parent is *not* the world frame.
TEST_F(GeometryStateTest, RemoveFrameLeaf) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_.get());
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  EXPECT_TRUE(gs_tester_.get_frames().at(frames_[0]).has_child(fid));
  geometry_state_.RemoveFrame(s_id, fid);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount);
  ExpectSourceDoesNotHaveFrame(s_id, fid);
  EXPECT_FALSE(gs_tester_.get_frames().at(frames_[0]).has_child(fid));
}

// Tests the response to invalid invocations of RemoveFrame.
TEST_F(GeometryStateTest, RemoveFrameInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Valid source, invalid frame.
  EXPECT_ERROR_MESSAGE(geometry_state_.RemoveFrame(s_id, FrameId::get_new_id()),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");

  // Case: Invalid source, valid frame.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(SourceId::get_new_id(), frames_[0]),
      std::logic_error,
      "Trying to remove frame \\d+ from source \\d+.+the frame doesn't "
      "belong.+");

  // Case: Valid source and frame, but frame does _not_ belong to source.
  SourceId s_id2 = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(s_id2);
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_.get());
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(s_id, frame_id),
      std::logic_error,
      "Trying to remove frame \\d+ from source \\d+.+the frame doesn't "
      "belong.+");
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::GetSourceId(GeometryId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryGoodSource) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  GeometryId g_id = geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(instance_));
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.GetSourceId(g_id), s_id);
  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  CompareMatrices(X_FG.matrix(), instance_pose_.matrix());

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.has_frame(f_id));
  EXPECT_FALSE(geometry.get_parent());
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::GetSourceId(GeometryId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  SourceId s_id = SourceId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");
}

// Tests registration of geometry on valid source and non-existant frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  SourceId s_id = NewSource();

  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced frame \\d+ for source \\d+\\."
                           " But the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  unique_ptr<GeometryInstance<double>> null_geometry;
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(null_geometry)),
                       std::logic_error,
                       "Registering null geometry to frame \\d+, on source "
                       "\\d+.");
}

// Tests the logic for hanging a geometry on another geometry. This confirms
// topology and pose values.
TEST_F(GeometryStateTest, RegisterGeometryonValidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  Isometry3<double> expected_pose = Isometry3<double>::Identity();
  double x = 3;
  double y = 2;
  double z = 1;
  pose.translation() << x, y, z;
  const int parent_index = 0;
  const GeometryId parent_id = geometries_[parent_index];
  const FrameId frame_id = geometry_state_.GetFrameId(parent_id);
  auto instance = make_unique<GeometryInstance<double>>(
      pose, unique_ptr<Shape>(new Sphere(1)));
  GeometryId g_id =
      geometry_state_.RegisterGeometryWithParent(s_id,
                                                 parent_id,
                                                 move(instance));
  // This relies on the gᵗʰ geometry having position [ g+1 0 0 ]ᵀ.
  expected_pose.translation() << (parent_index + 1) + x, y, z;
  EXPECT_EQ(frame_id,
            geometry_state_.GetFrameId(g_id));
  // TODO(SeanCurtis-TRI): Test that the pose of the registered geometry,
  // collapsed down to the common frame, *is* the expected pose.
  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), expected_pose.matrix(),
                  1e-14, MatrixCompareType::absolute));
  Isometry3<double> X_PG = geometry_state_.GetPoseInParent(g_id);
  EXPECT_TRUE(CompareMatrices(X_PG.matrix(), pose.matrix(),
                  1e-14, MatrixCompareType::absolute));

  EXPECT_TRUE(gs_tester_.get_frames().at(frame_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_EQ(geometry.get_frame_id(), frame_id);
  EXPECT_TRUE(geometry.has_parent(parent_id));
  EXPECT_TRUE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));
}

// Tests the response to the erroneous action of trying to hang a new geometry
// on a non-existant geometry id.
TEST_F(GeometryStateTest, RegisterGeometryonInvalidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance<double>>(
      pose, unique_ptr<Shape>(new Sphere(1)));
  GeometryId junk_id = GeometryId::get_new_id();
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, junk_id, move(instance)),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// Tests the response to passing a null pointer as a GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometryonGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  unique_ptr<GeometryInstance<double>> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, geometries_[0],
                                                 move(instance)),
      std::logic_error,
      "Registering null geometry to geometry \\d+, on source \\d+.");
}

// Tests the RemoveGeometry functionality.
TEST_F(GeometryStateTest, RemoveGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry to remove, and the frame to which it belongs.
  GeometryId g_id = geometries_[0];
  FrameId f_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveGeometry(s_id, g_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);

  EXPECT_FALSE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().find(g_id),
            gs_tester_.get_geometries().end());
  // Based on the logic of the geometry engine stub, the last geometry id should
  // now be the first.
  GeometryId last_geometry_id = geometries_[geometries_.size() - 1];
  const auto& last_geometry =
      gs_tester_.get_geometries().at(last_geometry_id);
  EXPECT_EQ(last_geometry.get_engine_index(), 0);
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map()[0], last_geometry_id);
}

// Tests the RemoveGeometry functionality in which the geometry removed has
// geometry children.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveParent) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry to remove, and the frame to which it belongs.
  GeometryId root_id = geometries_[0];
  FrameId f_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(root_id), f_id);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, root_id,
      make_unique<GeometryInstance<double>>(
          Isometry3<double>::Identity(),
          unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(gs_tester_.get_geometries().at(g_id).get_engine_index(),
            geometries_.size());

  geometry_state_.RemoveGeometry(s_id, root_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);

  const auto& frame = gs_tester_.get_frames().at(f_id);
  EXPECT_FALSE(frame.has_child(root_id));
  EXPECT_FALSE(frame.has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().find(root_id),
            gs_tester_.get_geometries().end());
  EXPECT_EQ(gs_tester_.get_geometries().find(g_id),
            gs_tester_.get_geometries().end());
  // Based on the logic of the geometry engine stub, the last geometry id should
  // now be the first. The deleted child geometry was already last.
  GeometryId last_geometry_id = geometries_[geometries_.size() - 1];
  const auto& last_geometry =
      gs_tester_.get_geometries().at(last_geometry_id);
  EXPECT_EQ(last_geometry.get_engine_index(), 0);
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map()[0], last_geometry_id);
}

// Tests the RemoveGeometry functionality in which the geometry is a child of
// another geometry.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveChild) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry parent and frame to which it belongs.
  GeometryId parent_id = geometries_[0];
  FrameId frame_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, parent_id,
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                            unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), frame_id);

  geometry_state_.RemoveGeometry(s_id, g_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount);
  EXPECT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);

  EXPECT_FALSE(gs_tester_.get_frames().at(frame_id).has_child(g_id));
  EXPECT_TRUE(gs_tester_.get_frames().at(frame_id).has_child(parent_id));
  EXPECT_FALSE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));

  // The geometry we deleted is the *last*; the engine indices of all other
  // geometries should be unchanged.
  for (size_t i = 0; i < geometries_.size(); ++i) {
    EXPECT_EQ(gs_tester_.get_geometry_index_id_map().at(i),
              geometries_[i]);
  }
}

// Tests the response to invalid misuse of RemoveGeometry.
TEST_F(GeometryStateTest, RemoveGeometryInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Invalid source id, valid geometry id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(SourceId::get_new_id(),
                                     geometries_[0]),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
      "belong.+");

  // Case: Invalid geometry id, valid source id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");

  // Case: Valid geometry and source, but geometry belongs to different source.
  SourceId s_id2 = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(s_id2);
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  GeometryId g_id = geometry_state_.RegisterGeometry(
      s_id2, frame_id,
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                            unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, g_id),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
          "belong.+");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance<double>>(
      pose, unique_ptr<Shape>(new Sphere(1)));
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::runtime_error,
      "Not implemented yet!");
}

// Tests the response of attempting to register a null pointer GeometryInstance
// as anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredNullGeometry) {
  unique_ptr<GeometryInstance<double>> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::runtime_error,
      "Not implemented yet!");
}

// Confirms the behavior for requesting geometry poses with a bad geometry
// identifier. The basic behavior is tested implicitly in other tests because
// they rely on them to validate state.
TEST_F(GeometryStateTest, GetPoseForBadGeometryId) {
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetPoseInFrame(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetPoseInParent(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// This confirms the failure state of calling GeometryState::GetSourceId with a
// bad frame/geometry identifier.
TEST_F(GeometryStateTest, GetSourceIdFromBadId) {
  EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(FrameId::get_new_id()),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");
  EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(GeometryId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryStateTest, GetFrameIdFromBadId) {
  EXPECT_ERROR_MESSAGE(geometry_state_.GetFrameId(GeometryId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
}

// This tests that clearing a source eliminates all of its geometry and frames,
// leaving the source active.
TEST_F(GeometryStateTest, ClearSourceData) {
  EXPECT_ERROR_MESSAGE(geometry_state_.ClearSource(SourceId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");

  SourceId s_id = SetUpSingleSourceTree();
  geometry_state_.ClearSource(s_id);
  EXPECT_TRUE(geometry_state_.source_is_active(s_id));
  AssertSingleTreeCleared();
}

// Tests the functionality for acquiring the parent geometry for a given
// geometry.
TEST_F(GeometryStateTest, GetParentGeometry) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Attempt to query non-existant geometry id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.FindParentGeometry(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");

  // Case: Query geometry id directly registered to the frame. The optional must
  // be false.
  auto frame_result = geometry_state_.FindParentGeometry(geometries_[0]);
  EXPECT_FALSE(frame_result);

  // Case: Query geometry registered to another geometry.
  auto instance = make_unique<GeometryInstance<double>>(
      Isometry3<double>::Identity(), unique_ptr<Shape>(new Sphere(1)));
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(s_id,
                                                               geometries_[0],
                                                               move(instance));
  auto geo_result = geometry_state_.FindParentGeometry(g_id);
  EXPECT_TRUE(geo_result);
  EXPECT_EQ(*geo_result, geometries_[0]);
}

// Tests the validation of frame kinematics data provided.
TEST_F(GeometryStateTest, ValidateKinematicsData) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameKinematicsSet<double> fks = GeometryWorld<double>::MakeFKS(s_id);
  // Create one pose per frame.
  std::vector<SpatialPose<double>> poses;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
  }
  // Case: Valid data.
  fks.ReportPoses(frames_, poses);
  EXPECT_NO_THROW(geometry_state_.ValidateKinematicsSet(fks));

  // Case: Strictly missing required frames.
  fks.Clear();
  fks.ReportPose(frames_[0], poses[0]);
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Disagreement in expected number of frames \\(\\d+\\) "
                           "and the given number of frames \\(\\d+\\).");

  // Case: Strictly adding frames that don't belong.
  fks.Clear();
  fks.ReportPoses(frames_, poses);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Disagreement in expected number of frames \\(\\d+\\) "
                           "and the given number of frames \\(\\d+\\).");

  // Case: Correct number; required frame swapped with invalid frame.
  fks.Clear();
  std::vector<FrameId> frames_subset(++frames_.begin(), frames_.end());
  std::vector<SpatialPose<double>> pose_subset(++poses.begin(), poses.end());
  fks.ReportPoses(frames_subset, pose_subset);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Frame id provided in kinematics data \\(\\d+\\) "
                           "does not belong to the source \\(\\d+\\)."
                           " At least one required frame id is also missing.");
}

// Tests the GeometryState::SetFrameKinematics() method. This doesn't test
// invalid kinematics sets (as that has been tested already). It simply confirms
// that for valid values, the geometries are posed as expected in the world
// frame. This only tests pose (not velocity or acceleration). These tests use
// simple transforms because it isn't evaluating the matrix multiplication, only
// that the right matrix multiplications are performed based on the hierarchy
// of constructs.
TEST_F(GeometryStateTest, SetKinematicsData) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameKinematicsSet<double> fks = GeometryWorld<double>::MakeFKS(s_id);

  // Create a vector of poses (initially set to the identity pose).
  SpatialPose<double> identity(Quaternion<double>(1, 0, 0, 0),
                               Vector3<double>(0, 0, 0));
  std::vector<SpatialPose<double>> frame_poses;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_poses.push_back(identity);
  }

  // Case 1: Set all frames to identity poses. The world pose of all the
  // geometry should be that of the geometry in its frame.
  fks.ReportPoses(frames_, frame_poses);
  geometry_state_.SetFrameKinematics(fks);
  const auto& world_poses = gs_tester_.get_geometry_world_poses();
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                                X_FG_[i].matrix().block<3, 4>(0, 0)));
  }

  // Case 2: Move the two *root* frames 1 unit in the +y direction. The f2 will
  // stay at the identity.
  // The final geometry poses should all be offset by 1 unit in the y.
  SpatialPose<double> offset(Quaternion<double>(1, 0, 0, 0),
                             Vector3<double>(0, 1, 0));
  Isometry3<double> M_offset = offset.get_isometry();
  fks.Clear();
  fks.ReportPose(frames_[0], offset);
  fks.ReportPose(frames_[1], offset);
  fks.ReportPose(frames_[2], identity);
  geometry_state_.SetFrameKinematics(fks);
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (M_offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }

  // Case 3: All frames get set to move up one unit. This will leave geometries
  // 0, 1, 2, & 3 moved up 1, and geometries 4 & 5 moved up two.
  fks.Clear();
  fks.ReportPose(frames_[0], offset);
  fks.ReportPose(frames_[1], offset);
  fks.ReportPose(frames_[2], offset);
  geometry_state_.SetFrameKinematics(fks);
  for (int i = 0; i < (kFrameCount - 1) * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (M_offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }
  for (int i = (kFrameCount - 1) * kGeometryCount;
       i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(
        world_poses[i].matrix().block<3, 4>(0, 0),
        (M_offset * M_offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
