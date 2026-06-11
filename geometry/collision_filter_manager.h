#pragma once

#include <functional>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/proximity/collision_filter.h"

namespace drake {
namespace geometry {

// Forward declarations.
template <typename T>
class GeometryState;

// Disable formatter to preserve doxygen tables.
// clang-format off
/** Class for configuring "collision filters"; collision filters limit the scope
 of various proximity queries.

 The sole source of %CollisionFilterManager instances is SceneGraph. See
 @ref scene_graph_collision_filter_manager "SceneGraph's documentation" for
 details on acquiring an instance.

 A SceneGraph instance contains the set of geometries with assigned proximity
 properties `G = D ⋃ A = {g₀, g₁, ..., gₙ}`, where D is the set of dynamic
 geometries and A is the set of anchored geometries (by definition `D ⋂ A = ∅`).
 Many proximity queries operate on pairs of geometries (e.g., (gᵢ, gⱼ)). So, in
 principle, the potential domain of those queries (the "candidate pairs") are
 the set of geometry pairs `G × G`. In practice, we exclude pairs from the set.

 The %CollisionFilterManager excludes many pairs intrinsically, and provides
 mechanisms for the users to exclude even more pairs.

 Intrinsic Exclusions

 - `Iₚ = {(g, g)}, ∀ g ∈ G` is the set of all pairs consisting of a geometry
   with itself; there is no meaningful proximity query on such a pair.
 - `A × A` represents all pairs consisting only of anchored geometries; an
   anchored geometry is never tested against another anchored geometry.
 - `F = {(gᵢ, gⱼ)} ∀ i, j`, such that `gᵢ, gⱼ ∈ D` and `frame(gᵢ) == frame(gⱼ)`;
   the pairs where both geometries are rigidly affixed to the same frame.
 - Intrinsic exclusions are immutable: they cannot be restored to the candidate
   set by the user.

 User-Defined Exclusions

 - Using CollisionFilterDeclaration, the user can declare other pairs to be
   excluded (i.e., "filtered"). Subsequent declarations can restore those pairs
   to the candidate set ("allowing" the pair).
 - Declarations of user-defined exclusions and allowances can only be made on
   known geometries. Declarations on unknown geometries will be rejected with an
   error.

 Geometry Activity

 In addition to excluding specific geometry pairs, a geometry can be declared
 *inactive* (see CollisionFilterDeclaration::Deactivate()). An inactive geometry
 is essentially excluded from G. It still exists as a known geometry. So,
 adding or removing filters on that geometry is still valid. However, those
 declarations will have no apparent effect until the geometry is reactivated.
 An inactive geometry will simply never appear in the candidate set C.

 We can define `N` as the set of inactive geometries and `Gₐ = G - N` as the set
 of active geometries. 
 
 Candidate Geometry Pairs
 
 Therefore, the set of geometry pair candidates for proximity queries is defined
 as:

  `C = (Gₐ × Gₐ) - (A × A) - F - I`.

 A %CollisionFilterManager is a view into geometry data (either that owned by
 a SceneGraph instance or a SceneGraph context). The manager instance can be
 copied or moved and the resulting instance is a view into the same data. For
 both the original and the copy (or just the target when moving the manager),
 the source data must stay alive for at least as long as the manager instance.

 @warning Generally, the effect of applying a declaration is based on the state
 of SceneGraph's geometry data at the time of application. The geometry's active
 status is the one exception (see below). More concretely:
 - For a particular FrameId in a GeometrySet instance, only those geometries
   attached to the identified frame with the proximity role assigned at the
   time of the call will be included in the filter. If geometries are
   subsequently added or assigned the proximity role, they will not be
   retroactively added to the user-declared filter.
 - In general, adding collision geometries and assigning proximity roles should
   happen prior to collision filter configuration.
 - Declaring a set of geometries active/inactive is the exception. These add or
   remove the geometries from the set Gₐ. We are not adding/deleting filtered
   pairs at the time of declaration, but simply changing the active status of
   the geometries. A geometry's active status doesn't change just because other
   geometries get added/removed.

 <h3>Transient vs Persistent changes</h3>

 Collision filtering is all about defining which geometry pairs are in the set
 C. The simplest way to modify the set C is through *persistent* modifications
 to a "base configuration" (via calls to Apply()). However, declarations can
 also be applied in a *transient* manner. Transient declarations form a history.
 The current definition of the set C is the result of applying the history
 of collision filter declarations to the persistent base configuration in order.
 That history can be modified:

   - New transient declarations can be appended.
   - Arbitrary transient declarations can be removed from the sequence.
   - The current configuration (based on a history with an arbitrary number
     of transient declarations) can be "flattened" into the persistent base
     (with no history).

 The table below illustrates a sequence of operations and their effect on C. We
 define C's initial configuration as `C = {P₁, P₂, ..., Pₙ}` (for `n` geometry
 pairs). Each action is described as either persistent or transient.

  | Line |  C                  |  Action |
  | :--: | :------------------ | :-------------------------------------------------- |
  |  1   | `{P₁, P₂, ..., Pₙ}` | Initial condition of the persistent base            |
  |  2   | `{P₂, ..., Pₙ}`     | Remove P₁ from the persistent base                  |
  |  3   | `{P₂, ..., Pₙ₋₁}`   | Remove Pₙ from the persistent base                  |
  |  4   | `{P₂}`              | Remove all pairs except P₂ from the persistent base |
  |  5   | `{P₂, P₄, P₅}`      | Transient declaration #1 puts P₄ and P₅ into C      |
  |  6   | `{P₂, P₃, P₄, P₅}`  | Transient declaration #2 puts P₃ and P₄ into C      |
  |  7   | `{P₂, P₃, P₄}`      | Remove declaration #1.                              |
  |  8   | `{P₂, P₃, P₄}`      | Configuration flattened; #2 no longer exists        |
  __Table 1__: An example sequence of operations on collision filters.

 Notes:
   - lines 2 - 4 represent a series of *persistent* operations, filtering pairs
     of geometry (aka removing them from C by calling Apply()).
   - line 5: the first transient filter declaration which is assigned the id
     value #1 (via a call to ApplyTransient()).
   - line 6: Adds a new transient filter declaration to the sequence (assigned
     id #2). Note, that it redundantly declares that P₄ is a member of C just
     as declaration #1 did. This redundancy is fine. In fact, it may be very
     important (see below).
   - line 7: We remove declaration #1. Although #1 added both P₄ and P₅ to C,
     the result of removing this declaration from the sequence is that P₅ is no
     longer a member of C but P₄ *is*. This is because #2's declaration that P₄
     *is* in C preserves its membership.
   - line 8: the current configuration is "flattened" into the persistent base.
     All history is thrown out and any filter identifiers are rendered invalid.

 This example workflow above illustrates some key ideas in using transient
 declarations.

   - __The persistent configuration can only be modified when there is no
     transient history.__ Applying a filter declaration should have the effect
     of realizing that declaration. If a pair is *declared* to be in C, the
     result of the declaration is that the pair is in C (assuming that the pair
     *can* be in C). If we allowed modifying the persistent configuration with
     an active transient history, there might be no discernible change in the
     resultant configuration state because a subsequent transient declaration
     may supplant it. This would lead to inscrutable bugs. Therefore, it's
     simply not allowed.
   - __When defining a transient declaration, the declaration should include all
     critical pairs *explicitly*.__ This includes those pairs that should and
     should not be in C. Any pair *not* explicitly accounted for should be one
     whose filter status is immaterial.
     It *might* seems desirable (from an optimization perspective) to examine
     the *current* configuration of C and apply the minimum change to put it
     into a desired state (i.e., if a pair I need filtered is already filtered,
     I would omit it from the declaration). This approach is fraught with peril.
     If the current configuration is the result of previous transient
     declarations, the removal of any of those declarations could invalidate the
     attempted difference calculation and my declaration would no longer be
     sufficient to guarantee the set C required.
   - __Anyone at any time can add to the history.__ Some code can modify C to
     suit its needs. However, it can also make subsequent calls into code that
     also modifies C with no guarantees that the called code will undo those
     changes. Code that modifies collision filters configuration should document
     that it does so. It should also have a clear protocol for cleaning up its
     own changes. Finally, code that applies declarations should not take for
     granted that its transient declarations are necessarily the last
     declarations in the history.

 <h4>Making transient filter declarations</h4>

 There is a custom API for applying a filter declaration as a *transient*
 declaration. It returns the id for the transient API (used to remove the
 declaration from the history sequence).

 Attempting to change the persistent configuration when there are active
 transient declarations in the history will throw an exception.   */
// clang-format on
class CollisionFilterManager {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterManager);

  /** Applies the given `declaration` to the geometry state managed by `this`
   instance.

   The process of *applying* the collision filter data also validates it. The
   following polices are implemented during application:

     - Referencing an invalid id (FrameId or GeometryId): throws.
     - Declaring a filtered pair that is already filtered: no discernible
       change.
     - Attempts to "allow" collision between a pair that is strictly excluded
       (e.g., between two anchored geometries) will be ignored.

   @throws std::exception if the `declaration` references invalid ids or there
                          is an active history. */
  void Apply(const CollisionFilterDeclaration& declaration) {
    /* Modifications made via this API are by the user. Only the internals can
     declare filters to be "invariant". */
    internal::CollisionFilter::ActiveStatusChange active_status_change;
    filter_->Apply(declaration, extract_ids_, false /* is_invariant */,
                   &active_status_change);
    NotifyActiveStatusChange(active_status_change);
  }

  // TODO(SeanCurtis-TRI) SceneGraphInspector includes the method
  //  CollisionFiltered. It reports whether two geometries are filtered. It
  //  would be logical to include that here with *two* caveats.
  //  1. The SceneGraphInspector relies on GeometryState to provide detailed
  //     error message in case of failure (e.g., id is valid, but has no
  //     proximity) role. This class lacks that intelligence.
  //  2. The SceneGraphInspector can be acquired via an input port (from a
  //     QueryObject instance). CollisionFilterManager cannot. However, the
  //     QueryObject class can be extended to return a manager just like an
  //     inspector.
  //  If those can be resolved, it would make more sense to make the
  //  CollisionFilterManager the one-stop shop for all things collision filter
  //  rather than having it split over two classes.

  /** Applies the declaration as the newest *transient* modification to the
   collision filter configuration. The declaration must be considered "valid",
   as defined for Apply(). */
  FilterId ApplyTransient(const CollisionFilterDeclaration& declaration) {
    internal::CollisionFilter::ActiveStatusChange active_status_change;
    const FilterId result = filter_->ApplyTransient(declaration, extract_ids_,
                                                    &active_status_change);
    NotifyActiveStatusChange(active_status_change);
    return result;
  }

  /** Attempts to remove the transient declaration from the history for the
   declaration associated with the given `filter_id`.
   @param filter_id   The id of the filter declaration to remove.
   @returns `true` iff `is_active(filter_id)` returns `true` before calling this
            method (i.e., `filter_id` refers to an existent filter that has
            successfully been removed). */
  bool RemoveDeclaration(FilterId filter_id) {
    internal::CollisionFilter::ActiveStatusChange active_status_change;
    const bool result =
        filter_->RemoveDeclaration(filter_id, &active_status_change);
    NotifyActiveStatusChange(active_status_change);
    return result;
  }

  /** Reports if there are any active transient filter declarations. */
  bool has_transient_history() const {
    return filter_->has_transient_history();
  }

  /** Reports if the transient collision filter declaration indicated by the
   given `filter_id` is part of the history. */
  bool IsActive(FilterId filter_id) const {
    return filter_->IsActive(filter_id);
  }

 private:
  /* Only GeometryState can construct a collision filter manager. */
  template <typename>
  friend class GeometryState;

  /* The callback type used to push net changes of the filter's active-status
   (inactive) set to the owning geometry data. GeometryState binds this to its
   ProximityEngine (see ProximityEngine::ApplyActiveStatusChange()), which
   exploits the inactive set to cull inactive dynamic geometries from its
   broadphase structures. This is internal plumbing for that pure optimization;
   it has no observable effect on query results. Like `filter`, the bound
   target is a view into the geometry data this manager fronts and shares its
   lifetime contract. */
  using ActiveStatusChangeCallback =
      std::function<void(const internal::CollisionFilter::ActiveStatusChange&)>;

  /* Constructs the manager for a `filter` with a callback for resolving a
   GeometrySet into a set of GeometryIds and a callback for active-status
   changes.
   @pre filter is non-null.
   @pre active_status_change_callback is non-empty. */
  explicit CollisionFilterManager(
      internal::CollisionFilter* filter,
      internal::CollisionFilter::ExtractIds extract_ids,
      ActiveStatusChangeCallback active_status_change_callback);

  /* Pushes a non-empty active-status change to the callback. */
  void NotifyActiveStatusChange(
      const internal::CollisionFilter::ActiveStatusChange& change) {
    if (!change.empty()) {
      active_status_change_callback_(change);
    }
  }

  internal::CollisionFilter* filter_{};
  internal::CollisionFilter::ExtractIds extract_ids_;
  ActiveStatusChangeCallback active_status_change_callback_;
};
}  // namespace geometry
}  // namespace drake
