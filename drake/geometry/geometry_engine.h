#pragma once

namespace drake {
namespace geometry {

/**
 A geometry engine is the entity that implements the geometric queries. This
 serves as the interface to specific geometry library implementations (e.g.,
 Bullet and FCL.)  Historically, this replaces the DrakeCollision::Model
 class.
 */
public DRAKE_EXPORT GeometryEngine {
 public:
  virtual ~GeometryEngine() {}
};
}  // namespace geometry
}  // namespace drake
