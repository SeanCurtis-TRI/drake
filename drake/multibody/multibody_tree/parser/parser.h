#pragma once

#include <string>

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/parser/model_instance_id_table.h"
#include "drake/multibody/parsers/package_map.h"

namespace drake {
namespace multibody {
namespace parser {

// TODO(SeanCurtis-TRI): Enable the "floating_base_type" parameter when we have
// floating mobilizers.
/** Adds the model(s) defined within an SDF or URDF file to a multi-body tree.
 One instance of each model is added. The models in an SDF are assumed to be
 described in the world frame.

 This method can only be used with models that either

 1. do not use `package://` to reference modeling resources like mesh files, or
 2. only reference packages that are defined up the directory tree relative to
 `filename`. Files that contain `package://` references that do not meet these
 requirements should instead use
 AddModelInstancesFromFileToWorldSearchingInRosPackages().

 @param[in] filename            The name of the SDF/URDF file containing the
                                model(s) to be added.
 @param[in] floating_base_type  The type of joint that connects each otherwise
                                unparented link to the existing multibody tree.
 @param[out] tree               The rigid body tree to which to add the model.
 @return A table mapping the names of the models whose instances were just
 added to the MultibodyTree to their BodyIndex values, which are unique within
 the MultibodyTree. */
ModelInstanceIdTable
AddModelInstancesFromFileToWorld(
    const std::string& filename,
//    FloatingBaseType floating_base_type,
    MultibodyTree<double>* tree);

// TODO(SeanCurtis-TRI): Documentation.
ModelInstanceIdTable
AddModelInstancesFromFileToWorldSearchingInRosPackages(
    const std::string& filename, const parsers::PackageMap& package_map,
//    const drake::multibody::joints::FloatingBaseType floating_base_type,
    MultibodyTree<double>* tree);

ModelInstanceIdTable AddModelInstancesFromSdfFile(
    const std::string& filename,
//    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    MultibodyTree<double>* tree);

}  // namespace parser
}  // namespace multibody
}  // namespace drake
