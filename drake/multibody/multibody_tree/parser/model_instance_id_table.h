#pragma once

#include <map>
#include <string>

#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parser {

/**
 Defines a data type that maps model _names_ to their BodyIndex values within
 the MultibodyTree. The model names are defined in a single URDF or SDF file
 and are thus guaranteed to be unique within an instance of this data type.
 The BodyIndex values are determined by the MultibodyTree. The values are unique
 among all model instances within the tree and can be used to access Body
 references via MultibodyTree::get_body(). This data type is used to inform
 applications of the BodyIndex values that were assigned to model instances as
 they were added to a `RigidBodyTree` while parsing a URDF or SDF description.

 The model names within this data type are specified by the URDF and SDF.
 They are _not_ the same as "model instance names" since multiple instances
 of the same model may be added to the same MultibodyTree. Model instance
 names can be decided by the application based on the information contained
 within this data type. It is recommended, but not required, that
 applications separately create mappings from model instance IDs to
 meaningful model instance names. This is because an instance ID, as an
 integer, does not convey much information about the model instance. */
typedef std::map<std::string, BodyIndex> ModelInstanceIdTable;

/**
 * Adds the model instances in `source_table` to `dest_table`. Throws a
 * `std::runtime_error` if there is a collision in the model names.
 */
void AddModelInstancesToTable(
    const ModelInstanceIdTable& source_table, ModelInstanceIdTable* dest_table);

  }  // namespace parser
  }  // namespace multibody
  }  // namespace drake
