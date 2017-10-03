#pragma once

#include "drake/common/drake_optional.h"
#include "drake/multibody/rigid_body_plant/compliant_material_parameters.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {

/**
 * Adds a box-shaped terrain to @p tree with default "size" and depth. This
 * directly modifies the existing world rigid body within @p tree and thus does
 * not need to return a `model_instance_id` value.
 *
 * Two opposite corners of the resulting axis-aligned box are:
 * `(size / 2, size / 2, 0)` and
 * `(-size / 2, -size / 2, -depth)`.
 *
 * @param[in] tree The RigidBodyTreed to which to add the terrain.
 *
 * @param[in] contact_material The optional compliant contact material -- if
 * not provided, uses the hard-coded default values.
 */
void AddFlatTerrainToWorld(
    RigidBodyTreed* tree,
    const optional<systems::CompliantMaterialParameters>& contact_material);

/**
 * Adds a box-shaped terrain to @p tree. This directly modifies the existing
 * world rigid body within @p tree and thus does not need to return a
 * `model_instance_id` value.
 *
 * Two opposite corners of the resulting axis-aligned box are:
 * `(box_size / 2, box_size / 2, 0)` and
 * `(-box_size / 2, -box_size / 2, -box_depth)`.
 *
 * @param[in] tree The RigidBodyTreed to which to add the terrain.
 *
 * @param[in] contact_material The optional compliant contact material -- if
 * not provided, uses the hard-coded default values.
 *
 * @param[in] box_size The length and width of the terrain aligned with the
 * world's X and Y axes.
 *
 * @param[in] box_depth The depth of the terrain aligned with the world's Z
 * axis. Note that regardless of how deep the terrain is, the top surface of the
 * terrain will be at Z = 0.
 */
void AddFlatTerrainToWorld(
    RigidBodyTreed* tree, double box_size, double box_depth,
    const optional<systems::CompliantMaterialParameters>& contact_material);

}  // namespace multibody
}  // namespace drake
