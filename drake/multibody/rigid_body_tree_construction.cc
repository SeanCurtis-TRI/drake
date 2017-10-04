#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {
namespace multibody {

using systems::CompliantMaterialParameters;

void AddFlatTerrainToWorld(RigidBodyTreed* tree) {
  AddFlatTerrainToWorld(tree, CompliantMaterialParameters());
}

void AddFlatTerrainToWorld(
    RigidBodyTreed* tree,
    const CompliantMaterialParameters& contact_material) {
  AddFlatTerrainToWorld(tree, 1000, 10, contact_material);
}

void AddFlatTerrainToWorld(
    RigidBodyTreed* tree, double box_size, double box_depth) {
  AddFlatTerrainToWorld(tree, box_size, box_depth,
                        CompliantMaterialParameters());
}

void AddFlatTerrainToWorld(
    RigidBodyTreed* tree, double box_size, double box_depth,
    const CompliantMaterialParameters& contact_material) {
  DrakeShapes::Box geom(Eigen::Vector3d(box_size, box_size, box_depth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -box_depth / 2;  // Top of the box is at z = 0.
  RigidBody<double>& world = tree->world();

  // Defines a color called "desert sand" according to htmlcsscolor.com.
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;

  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  drake::multibody::collision::Element element(geom, T_element_to_link, &world);
  element.set_compliant_parameters(contact_material);
  tree->addCollisionElement( element, world, "terrain");
  tree->compile();
}

}  // namespace multibody
}  // namespace drake
