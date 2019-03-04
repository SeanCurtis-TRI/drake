#pragma once

namespace drake {
namespace geometry {
namespace dev {
namespace render {

/** Specification of render fidelity to use for generating renderings of the
 SceneGraph. Each fidelity will consume an instance of PerceptionProperties.
 Each fidelity level has independent requirements for what properties should be
 set. See the list below for details:

 <!-- TODO(SeanCurtis-TRI): Figure out context copying so that copies of the
   renderer are fully independent.  ->
 - fast depth - an optimized camera that supports only depth rendering. Note:
   this renderer is not thread-safe. All instances of this render share
   underlying mechanisms (for now). So, even with multiple renderers, operations
   on the renderer should be serialized by the caller (i.e., associate a mutex
   with calls to QueryObject::RenderDepthImage()).
 - low fidelity - see @ref render_engine_vtk_properties "Low-fidelity geometry perception properties"
 - medium fidelity -- not supported yet.
 - high fidelity -- not supported yet.
 */
enum class Fidelity {
  kFastDepth,
  kLow,
  kMedium,
  kHigh
};

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
