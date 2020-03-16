#include "drake/multibody/parsing/parser.h"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {

using internal::AddModelFromSdfFile;
using internal::AddModelFromUrdfFile;
using internal::AddModelsFromSdfFile;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

Parser::Parser(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)
    : plant_(plant), scene_graph_(scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
}

namespace {
enum class FileType { kSdf, kUrdf };
FileType DetermineFileType(const std::string& file_name) {
  const std::string ext = filesystem::path(file_name).extension().string();
  if ((ext == ".urdf") || (ext == ".URDF")) {
    return FileType::kUrdf;
  }
  if ((ext == ".sdf") || (ext == ".SDF")) {
    return FileType::kSdf;
  }
  throw std::runtime_error(fmt::format(
      "The file type '{}' is not supported for '{}'",
      ext, file_name));
}

FileType DetermineFileTypeFromContents(const std::string& xml_text) {
  XMLDocument xml_doc;
  xml_doc.Parse(xml_text.c_str(), xml_text.size());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(
        fmt::format("Failed to parse XML from given string:\n{}\n\nError: {}",
                    xml_text, xml_doc.ErrorName()));
  }
  XMLElement* first_child = xml_doc.FirstChildElement();
  if (first_child == nullptr) {
    throw std::runtime_error(fmt::format(
        "Can't parse model from XML string; the XML is empty: {}", xml_text));
  }
  const std::string name(first_child->Name());
  if (name == "robot") {
    return FileType::kUrdf;
  } else if (name == "sdf") {
    return FileType::kSdf;
  } else {
    throw std::runtime_error(fmt::format(
        "The string couldn't be identified as either URDF or SDF by its root "
        "tag ('<robot>' or '<sdf>', respectively). Found '<{}>' in the XML\n{}",
        name, xml_text));
  }
}

}  // namespace

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromFile(
    const std::string& file_name) {
  // Always search for a package.xml file, starting the crawl upward from
  // the file's path.
  package_map_.PopulateUpstreamToDrake(file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelsFromSdfFile(file_name, package_map_, plant_,
        scene_graph_);
  } else {
    return {AddModelFromUrdfFile(
        file_name, {}, package_map_, plant_, scene_graph_)};
  }
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  // Always search for a package.xml file, starting the crawl upward from
  // the file's path.
  package_map_.PopulateUpstreamToDrake(file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelFromSdfFile(file_name, model_name, package_map_,
        plant_, scene_graph_);
  } else {
    return AddModelFromUrdfFile(
        file_name, model_name, package_map_, plant_, scene_graph_);
  }
}

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromString(
    const std::string& xml_text, const std::string& root_directory) {
  package_map_.PopulateUpstreamToDrakeFromFolder(root_directory);
  const FileType type = DetermineFileTypeFromContents(xml_text);
  if (type == FileType::kSdf) {
    return AddModelsFromSdfString(xml_text, package_map_, root_directory,
                                  plant_, scene_graph_);
  } else {
    return {AddModelFromUrdfString(xml_text, {}, package_map_, root_directory,
                                   plant_, scene_graph_)};
  }
}

ModelInstanceIndex Parser::AddModelFromString(const std::string& xml_text,
                                      const std::string& model_name,
                                      const std::string& root_directory) {
  package_map_.PopulateUpstreamToDrakeFromFolder(root_directory);
  const FileType type = DetermineFileTypeFromContents(xml_text);
  if (type == FileType::kSdf) {
    return AddModelFromSdfString(xml_text, model_name, package_map_,
                                 root_directory, plant_, scene_graph_);
  } else {
    return AddModelFromUrdfString(xml_text, model_name, package_map_,
                                  root_directory, plant_, scene_graph_);
  }
}

}  // namespace multibody
}  // namespace drake
