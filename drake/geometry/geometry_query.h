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

   This method is affected by collision filtering; element pairs that have
   been filtered will not produce contacts, even if the elements are
   penetrating.
   */
  bool ComputeContact() const;

  /**
   Computes the pair-wise nearest points for the explicitly indicated pairs of
   bodies.

   *@param[in]  pairs       A "list" of
   */

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      closestPointsPairwise -- for an explicit set of pairs
  //      collisionDetectFromPoints -- distance from points to nearest object
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //      collidingPoints -- which points are within distance d of a body
  //      collidingPointsCHeckOnly -- simply query if any point is "near" a body
  //      collisionRaycast - cast one or more rays and report hit distance
  //
  /**

   */

};
}  // namespace geometry
}  // namespace drake
