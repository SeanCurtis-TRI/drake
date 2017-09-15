#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_system.h"

// TODO:

namespace drake {
namespace geometry {

/** Index type for use with GeometryTree and GeometryTreeNode. */
using GeometryTreeNodeIndex = TypeSafeIndex<class GeometryTreeNodeTag>;

/// Definition of the node of a GeometryTree.
class GeometryTreeNode {
 public:
  GeometryTreeNode(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
      : index_(index), parent_(parent) {}

  const GeometryTreeNode* parent() const { return parent_; }

  virtual void RegisterSelf(GeometrySystem* system) const = 0;
  virtual bool is_frame() const { return false; }

 private:
  // A pointer to the parent node -- should be null for a root node.
  GeometryTreeNodeIndex index_{};
  const GeometryTreeNode* parent_{nullptr};
  std::vector<GeometryTreeNode*> children_;
};

namespace internal {

template <class Payload>
class GeometryTreeNodeImpl : public GeometryTreeNode {
 public:
  GeometryTreeNodeImpl(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
      : GeometryTreeNode(index, parent) {}

 private:
  // The data associated with this geometry tree node.
  Payload payload_;
};

class InstanceNode : public GeometryTreeNodeImpl<GeometryInstance> {
 public:
  InstanceNode(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
  : GeometryTreeNodeImpl(index, parent) {}
};
class FrameNode : public GeometryTreeNodeImpl<GeometryFrame> {
 public:
  FrameNode(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
  : GeometryTreeNodeImpl(index, parent) {}
};
class GeometryIdNode : public GeometryTreeNodeImpl<GeometryId> {
 public:
  GeometryIdNode(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
  : GeometryTreeNodeImpl(index, parent) {}
};
class FrameIddNode : public GeometryTreeNodeImpl<FrameId> {
 public:
  FrameIddNode(GeometryTreeNodeIndex index, GeometryTreeNode* parent)
  : GeometryTreeNodeImpl(index, parent) {}
};

}  // namespace internal

// Represents a forest of geometry trees.
class GeometryForest {
 public:

  GeometryTreeNodeIndex AddRoot(const GeometryFrame& frame);
  GeometryTreeNodeIndex AddRoot(const GeometryInstance& geometry);

  GeometryTreeNodeIndex AddNode(GeometryTreeNodeIndex parent_index,
                                const GeometryFrame& frame);
  GeometryTreeNodeIndex AddNode(GeometryTreeNodeIndex parent_index,
                                const GeometryInstance& instance);

  const GeometryTreeNode& tree_node(GeometryTreeNodeIndex id) const {
    DRAKE_ASSERT(id < static_cast<int>(nodes_.size()));
    return *nodes_.at(id);
  }

 private:
  std::vector<GeometryTreeNode*> roots_;
  std::vector<std::unique_ptr<GeometryTreeNode>> nodes_;
};

}  // namespace geometry
}  // namespace drake
