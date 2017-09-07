#include "drake/multibody/multibody_tree/parser/model_instance_id_table.h"

namespace drake {
namespace multibody {
namespace parser {

void AddModelInstancesToTable(const ModelInstanceIdTable& source_table,
                              ModelInstanceIdTable* dest_table) {
  for (const auto& model_entry : source_table) {
    const std::string& model_name = model_entry.first;
    if (dest_table->count(model_name) > 0) {
      throw std::runtime_error("AddModelInstancesToTable: Collision occured "
                                   "with model name\"" + model_name + "\".");
    }
    (*dest_table)[model_name] = model_entry.second;
  }
}

}  // namespace parser
}  // namespace multibody
}  // namespace drake
