#pragma once

#include "drake/multibody/rigid_body_plant/compliant_material_parameters.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {

/** @name Adding a ground plane to a RigidBodyTree.
 *
 * These functions provide a simple interface for adding a ground plane to a
 * rigid body tree. The ground plane is actually a box whose top surface is
 * placed flush with the z = 0 plane.
 *
 * Fully specified, the box is parameterized with:
 *   - size - the size of the box along the x- and y-axes (forming a square)
 *   - depth - the size of the box along the z-axis
 *   - contact_material - the compliant contact material properties for the
 *     ground plane.
 *
 * Two opposite corners of the resulting axis-aligned box are:
 * `(size / 2, size / 2, 0)` and
 * `(-size / 2, -size / 2, -depth)`.
 *
 * The various overloads allow the ground plane's parameters to default to
 * hard-coded values.
 */
//@{

/**
 * Adds terrain which uses default values for all parameters.
 */
void AddFlatTerrainToWorld(RigidBodyTreed* tree);

/**
 * Adds terrain which uses default size values and specified contact material.
 */
void AddFlatTerrainToWorld(
    RigidBodyTreed* tree,
    const systems::CompliantMaterialParameters& contact_material);

/**
 * Adds terrain which uses given size values and default contact material.
 */
void AddFlatTerrainToWorld(
    RigidBodyTreed* tree, double box_size, double box_depth);

/**
 * Adds fully-specified terrain.
 */
void AddFlatTerrainToWorld(
    RigidBodyTreed* tree, double box_size, double box_depth,
    const systems::CompliantMaterialParameters& contact_material);

}  // namespace multibody
}  // namespace drake
