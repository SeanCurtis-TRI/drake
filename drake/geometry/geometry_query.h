#pragma once

namespace drake {
namespace geometry {

/**
 A class containing the geometric state of the world for a single context.

 This class serves as the output of the GeometryWorld and GeometrySystem.
 A valid GeometryQuery instance will support geometric queries on the geometry
 on the world predicated on a given Context.  Queries include:

    - Distance between geometries
    - Collisions between geometry
    - Ray-geometry intersection
 */
class GeometryQuery {
  // TODO(SeanCurtis-TRI): Determine how to limit the scope of these queries.
  //    e.g., limit it to just those elements belonging to a single input,
  //      limit it to a subset of elements in a single input,
  //      limit it to a subset of elements which span multiple inputs.
  // TODO(SeanCurtis-TRI): Determine the return value format. Currently uses
  //      PointPair.  However, the struct is overloaded so a value can have
  //      different interpretations based on how it is used.  Remove this
  //      ambiguity.

  // NOTE: This is just a taxonomy of the *types* of queries; the interfaces
  // are not remotely complete.  Each of these fundamental queries may have
  // different variants depending on the scope.

  /**
   Computes the pair-wise nearest points for all elements in the world.

   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints() const;

  /**
   Computes the contact across all elements in the world.  Only reports results
   for elements in *contact*; if two elements are separated, there will be no
   result for this pair.
   */
  bool ComputeContact() const;

};
}  // namespace geometry
}  // namespace drake
