#include "drake/geometry/render/internal_gltf_parser.h"

#include <functional>
#include <fstream>
#include <sstream>

#include <tiny_gltf.h>

#include "drake/common/drake_export.h"
#include "drake/common/text_logging.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::map;
using std::pair;
using std::string;
using std::vector;
namespace fs = std::filesystem;

/* The size of the component type in bytes. */
int ComponentSize(int component_type) {
  switch (component_type) {
    case TINYGLTF_COMPONENT_TYPE_BYTE:
      return 1;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
      return 1;
    case TINYGLTF_COMPONENT_TYPE_SHORT:
      return 2;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
      return 2;
    case TINYGLTF_COMPONENT_TYPE_INT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_FLOAT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_DOUBLE:
      return 8;
    default:
      throw std::runtime_error(
          fmt::format("{} (invalid component type)", component_type));
  }
}

/* Reports the dimension of the requested element. */
int ElementDimension(int element_type) {
  switch (element_type) {
    case TINYGLTF_TYPE_VEC2:
      return 2;
    case TINYGLTF_TYPE_VEC3:
      return 3;
    case TINYGLTF_TYPE_VEC4:
      return 4;
    case TINYGLTF_TYPE_MAT2:
      return 4;
    case TINYGLTF_TYPE_MAT3:
      return 9;
    case TINYGLTF_TYPE_MAT4:
      return 16;
    case TINYGLTF_TYPE_SCALAR:
      return 1;
    default:
      throw std::runtime_error(
          fmt::format("{} (invalid element type)", element_type));
  }
}

template <typename Element>
static void ThrowIfInvalidIndex(int index, const std::vector<Element>& array,
                                std::string_view container_name,
                                std::string_view element_name,
                                std::string_view elaboration,
                                const std::string& file_name) {
  if (index < 0 || index >= ssize(array)) {
    throw std::runtime_error(
        fmt::format("A {} uses an invalid {} index{} in {}.", container_name,
                    element_name, elaboration, file_name));
  }
}


/* Provide a uniform interface to reading data from a glTF buffer. This will
 enable us to extract vertex positions, normals, texture coordinates, and
 triangle definitions (indices). More particularly, these quantities typically
 need to be transformed, e.g., vertex positions and normals need to be expressed
 in Drake's z-up frame.
 
 The buffer being read from can consists of different scalar types (referred to
 as "component types" in glTF.). */
class BufferView {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BufferView);

  /* Constructs the view on a primitive's attribute. The `model` parameter is
   aliased by this instance and must outlive it. */
  BufferView(const tinygltf::Primitive& primitive,
               const std::string& attribute, const tinygltf::Model& model,
               const std::string& file_name) {
    const auto iter = primitive.attributes.find(attribute);
    if (iter != primitive.attributes.end()) {
      const int accessor_index = iter->second;
      ThrowIfInvalidIndex(
          accessor_index, model.accessors, "mesh primitive", "accessor",
          fmt::format(" for the {} attribute", attribute), file_name);
      const tinygltf::Accessor& accessor = model.accessors.at(accessor_index);
      // TODO: Configure to dispatch vertex_count Vec3<double> or vec2<double>.
      Initialize(accessor,
                 fmt::format("mesh primitive's {} attribute", attribute), model,
                 file_name);
    } else if (attribute == "TEXCOORD_0") {
      /* We've requested TEXCOORD_0, but it's not defined. So, we'll configure
       the view to return 2 * V zeros (two 0-valued texture coordinates per
       vertex). */
      const auto pos_iter = primitive.attributes.find("POSITION");
      DRAKE_DEMAND(pos_iter != primitive.attributes.end());
      const tinygltf::Accessor& accessor = model.accessors.at(pos_iter->second);
      element_count_ = accessor.count;
      /* This is the default value, but we want to underscore here that it
       *must* be zero to get the "auto-zeros" behavior. */
      buffer_ = nullptr;
      /* These last two values don't matter, but we'll initialize them to be
       conceptually consistent with the expected behavior. */
      components_per_element_ = 2;
      component_type_ = TINYGLTF_COMPONENT_TYPE_DOUBLE;
    }
  }

  /* Constructs the reader for triangle indices. */
  BufferView(int tri_accessor_index, const tinygltf::Model& model,
               const std::string& file_name) {
    ThrowIfInvalidIndex(tri_accessor_index, model.accessors, "mesh primitive",
                        "accessor", " for its indices", file_name);
    Initialize(model.accessors.at(tri_accessor_index),
               "mesh primitive's indices", model, file_name);
    /* Triangle indices report as 1-dimensional elements: scalars indicating
     indices. We're going to reshape it here into a Tx3 block of indices. */
    DRAKE_DEMAND(components_per_element_ == 1);
    DRAKE_DEMAND(element_count_ % 3 == 0);
    components_per_element_ = 3;
    element_count_ /= 3;
  }

  /* Reports the number of elements. */
  int count() const { return element_count_; }

  /* Reports if the view has defined values. If it is undefined, SetAttribute()
   will return all zeros. */
  bool is_defined() const { return buffer_ == nullptr; }

  /* Populates a Vxd matrix of Target from these attributes (if compatible).
   V is the number of elements and d is the dimension of the element.

   @throws if this view is incompatible for reasons such as a) wrong element
              dimension, b) the type cannot be implicitly converted to a Target,
              etc. */
  template <typename Target, int dimension>
  Eigen::Matrix<Target, Eigen::Dynamic, dimension, Eigen::RowMajor>
  GetValues() const {
    DRAKE_DEMAND(components_per_element_ == dimension);

    // Simply output zeros.
    if (buffer_ == nullptr) {
      return Eigen::Matrix<Target, Eigen::Dynamic, dimension,
                           Eigen::RowMajor>::Zero(element_count_, dimension);
    }

    switch (component_type_) {
      case TINYGLTF_COMPONENT_TYPE_BYTE:
        return GetValuesFromType<char, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
        return GetValuesFromType<unsigned char, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_SHORT:
        return GetValuesFromType<short, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        return GetValuesFromType<unsigned short, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_INT:
        return GetValuesFromType<int, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        return GetValuesFromType<unsigned int, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_FLOAT:
        return GetValuesFromType<float, Target, dimension>();
      case TINYGLTF_COMPONENT_TYPE_DOUBLE:
        return GetValuesFromType<double, Target, dimension>();
      default:
        throw std::runtime_error(
            fmt::format("{} (invalid component type)", component_type_));
    }
  }

 private:

  /* Initializes the buffer reader for the given accessor. */
  void Initialize(const tinygltf::Accessor& accessor,
                  std::string_view accessor_owner, const tinygltf::Model& model,
                  const std::string& file_name) {
    element_count_ = accessor.count;
    component_type_ = accessor.componentType;
    components_per_element_ = ElementDimension(accessor.type);

    const int bufferView_index = accessor.bufferView;
    ThrowIfInvalidIndex(bufferView_index, model.bufferViews, accessor_owner,
                        "bufferView", "", file_name);
    const tinygltf::BufferView& buffer_view =
        model.bufferViews.at(bufferView_index);
    const int buffer_index = buffer_view.buffer;
    ThrowIfInvalidIndex(buffer_index, model.buffers, accessor_owner, "buffer",
                        "", file_name);
    const tinygltf::Buffer& buffer = model.buffers.at(buffer_index);
    byte_stride_ = buffer_view.byteStride;
    DRAKE_DEMAND(byte_stride_ == 0 ||
                 byte_stride_ >=
                     components_per_element_ * ComponentSize(component_type_));
    buffer_ = buffer.data.data() + accessor.byteOffset + buffer_view.byteOffset;
  }

  /* Takes the Nx3 matrix of component type Source in the view's buffer and
   converts it to an Nx3 matrix of Target. */
  template <typename Source, typename Target, int dimension>
  Eigen::Matrix<Target, Eigen::Dynamic, dimension, Eigen::RowMajor>
  GetValuesFromType() const {
    /* The data is tightly packed, as signalled by a 0-valued byte stride. */
    if (byte_stride_ == 0) {
      auto from_buffer =
          Eigen::Map<const Eigen::Matrix<Source, Eigen::Dynamic, dimension,
                                         Eigen::RowMajor>>(
              reinterpret_cast<const Source*>(buffer_), element_count_,
              dimension);
      return from_buffer.template cast<Target>();
    }

    /* We have interleaved data and need to select it out carefully. */
    Eigen::Matrix<Source, Eigen::Dynamic, dimension, Eigen::RowMajor>
        from_buffer(element_count_, dimension);
    for (int e = 0; e < element_count_; ++e) {
      from_buffer.row(e) = Eigen::Map<const Eigen::Vector<Source, dimension>>(
          reinterpret_cast<const Source*>(buffer_ + e * byte_stride_));
    }
    return from_buffer.template cast<Target>();
  }

  /* The underlying buffer data aliased from the `model` parameter provided to
   the constructor. If null, accessors should simply return zeros (used for
   models without texture coordinates). */
  const unsigned char* buffer_{nullptr};

  /* The number of elements in the view. The definition of "element" depends
   on the data the view is looking at; it could be a triple of ints (a triangle)
   or a vector normal (triple of doubles). */
  int element_count_{};

  /* The dimensionality of this view's elements (i.e., 1 for scalars, 2 for
   Vector2, etc.) See `component_type_` below. */
  int components_per_element_{};

  /* The type of the component stored in the buffer (byte, int, float, etc.).
   The name "component type" is taken from glTF. */
  int component_type_{};

  /* As documented by glTF, this is the "The stride, in bytes, between vertex
   attributes." If zero, the bytes are tightly packed. We interpret that to mean
   that, regardless of the *size* of the component (byte, short, int, etc.)
   and the dimension of the element, if the first element is found at address
   p, the next element is at address p + byte_stride_. Implicit in this is the
   assumption that byte_stride_ >= sizeof(element).
  
  (https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#_bufferview_bytestride)*/
  int byte_stride_{0};
};

/* Simply returns the indices of all nodes that have no parents (are root
 nodes). It searches *all* the nodes, unconstrained by what may or may not be
 indicated by the model's scenes. */
vector<int> FindAllRootNodes(const tinygltf::Model& model) {
  vector<bool> has_parent(model.nodes.size(), false);
  for (const auto& node : model.nodes) {
    for (int child_index : node.children) {
      has_parent[child_index] = true;
    }
  }
  vector<int> roots;
  for (int n = 0; n < ssize(has_parent); ++n) {
    if (!has_parent[n]) {
      roots.push_back(n);
    }
  }
  return roots;
}

/* Identifies the source scene from the glTF file and returns the indices of
 that scene's root nodes. If no default scene can be identified, then
 all root nodes in the file are returned. */
vector<int> FindTargetRootNodes(
    const tinygltf::Model& model, const fs::path& path,
    const drake::internal::DiagnosticPolicy& policy) {
  /* The root nodes of all the hierarchies that will be instantiated (by
   index). */
  vector<int> root_indices;
  if (model.scenes.size() > 0) {
    if (model.defaultScene >= ssize(model.scenes)) {
      policy.Error(fmt::format(
          "Error parsing a glTF file; it defines {} scenes but has an "
          "invalid value for the \"scene\" property: {}. '{}'. No geometry "
          "will be added.",
          model.scenes.size(), model.defaultScene, path.string()));
      return root_indices;
    }
    // TODO(SeanCurtis-TRI): I need to decide if this *deserves* a warning. It
    // is not clear if blindly picking the zeroth scene is consistent with the
    // glTF spec, which states (EMPHASIS mine):
    //
    //   When scene is undefined, client implementations MAY delay rendering
    //   until a particular scene is requested.
    //
    // See: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#scenes
    if (model.defaultScene < 0 && ssize(model.scenes) > 1) {
      policy.Warning(
          fmt::format("Parsing a glTF file with multiple scene and no explicit "
                      "default scene; using the zeroth scene: '{}'.",
                      path.string()));
    }
    // tinygltf initializes defaultScene to -1 to indicate an undefined value
    const int scene_index = std::max(model.defaultScene, 0);
    /* TODO: Can I trust that these are actually root nodes? Will tinygltf
     catch them if there's an error? */
    root_indices = model.scenes[scene_index].nodes;
    if (root_indices.empty()) {
      policy.Error(fmt::format(
          "Error parsing a glTF file; the 0th scene had no root nodes. '{}'.",
          path.string()));
    }
  } else {
    if (model.nodes.size() == 0) {
      policy.Error(fmt::format(
          "Error parsing a glTF file; it has no scenes and no nodes. '{}'.",
          path.string()));
    }
    root_indices = FindAllRootNodes(model);
    if (root_indices.empty() && model.nodes.size() > 0) {
      policy.Error(
          fmt::format("Error parsing a glTF file; none of its {} nodes are "
                      "root nodes. '{}'.",
                      model.nodes.size(), path.string()));
    }
  }
  return root_indices;
}

/* Creates a RenderMaterial for the glTF material indicated by the given
 index. Currently, we produce the most rudimentary of phong materials; we
 extract the diffuse color (aka baseColorFactor) and (possibly) diffuse color
 map. */
RenderMaterial MakeGltfMaterial(const tinygltf::Model& model,
                                const fs::path& path, int mat_index) {
  RenderMaterial material;
  const tinygltf::Material gltf_mat = model.materials.at(mat_index);
  const tinygltf::PbrMetallicRoughness& gltf_pbr =
      gltf_mat.pbrMetallicRoughness;
  const vector<double>& gltf_rgba = gltf_pbr.baseColorFactor;
  material.diffuse =
      Rgba(gltf_rgba[0], gltf_rgba[1], gltf_rgba[2], gltf_rgba[3]);
  const bool valid_uv_set = gltf_pbr.baseColorTexture.texCoord == 0;
  if (!valid_uv_set) {
    // TODO(SeanCurtis-TRI) Would this be better as a one-time warning?
    log()->debug(
        "Drake's support for glTF files only includes zero-indexed texture "
        "coordinates. The material '{}' specifies texture coordinate set {}. "
        "These texture coordinates will be ignored. '{}'.",
        gltf_mat.name.empty() ? string("<unnamed>") : gltf_mat.name,
        gltf_pbr.baseColorTexture.texCoord, path.string());
  }
  const int diffuse_index = valid_uv_set ? gltf_pbr.baseColorTexture.index : -1;
  if (diffuse_index >= 0) {
    const tinygltf::Image& image = model.images.at(diffuse_index);
    // If the image *format* is unsupported, that will get reported downstream
    // (in the TextureLibrary).
    if (image.uri.starts_with("embedded:")) {
      material.diffuse_map = image.uri;
    } else {
      material.diffuse_map =
          (path.parent_path() / image.uri).lexically_normal();
    }
  }
  // TODO(SeanCurtis-TRI): One-time warning messages for any of the other
  // textures that we're currently not supporting.
  return material;
}

/* A glTF primitive and its transformed pose in the file (the pose may not be
 a rigid transform; it can include scale). A primitive's pose is the same as
 its node's (N) pose (as only nodes get posed). This flattens the node
 hierarchy but as we are rendering it as a rigid assembly, we lose nothing. */
struct PosedPrimitive {
  Eigen::Matrix4d T_FN;
  const tinygltf::Primitive* primitive{};
};

void AccumulateMeshData(const tinygltf::Model& model, const fs::path& path,
                        const PosedPrimitive& prim, RenderMesh* render_mesh) {
  if (prim.primitive->mode != TINYGLTF_MODE_TRIANGLES) {
    log()->debug(fmt::format(
        "Drake's native support of glTF files requires that all primitives "
        "use the triangle mode ({}). At least one primitive has an "
        "unsupported  mode: {}. The primitive will be ignored. {}.",
        TINYGLTF_MODE_TRIANGLES, prim.primitive->mode, path.string()));
    return;
  }

  /* Triangle data. */
  const BufferView tri_data(prim.primitive->indices, model, path.string());

  /* Vertex positions. */
  const BufferView pos_data(*prim.primitive, "POSITION", model, path.string());
  DRAKE_DEMAND(pos_data.count() > 0);

  /* Vertex normals. */
  const BufferView norm_data(*prim.primitive, "NORMAL", model, path.string());
  if (norm_data.count() == 0) {
    /* In parsing other mesh file formats (e.g., .obj), we throw when normals
     are missing. We'll continue that behavior here. */
    throw std::runtime_error(fmt::format(
        "Drake's native support of glTF files requires that all primitives "
        "define normals. At least one primitive is missing normals. {}",
        path.string()));
  }

  /* Texture coordinates. */
  const BufferView uv_data(*prim.primitive, "TEXCOORD_0", model, path.string());
  if (uv_data.is_defined()) {
    /* The model provided texture coordinates for *this* primitive, update the
     RenderMesh accordingly. */
    if (render_mesh->uv_state == UvState::kNone) {
      /* If the render mesh hasn't accumulated any geometry yet, the current
       uv state is meaningless and we can simply set to full. Otherwise,
       "none" means previous primitives lacked UVs, so it becomes partial. */
      if (render_mesh->positions.rows() == 0) { 
        render_mesh->uv_state = UvState::kFull;
      } else {
        render_mesh->uv_state = UvState::kPartial;
      }
    }
  } else {
    /* This primitive doesn't have texture coordinates; demote the RenderMesh
     if necessary. */
    if (render_mesh->uv_state == UvState::kFull) {
      render_mesh->uv_state = UvState::kPartial;
    }
  }

  /* All vertex attributes line up. */
  const int add_v_count = pos_data.count();
  DRAKE_DEMAND(add_v_count == norm_data.count());
  DRAKE_DEMAND(add_v_count == uv_data.count());

  /* Extract and transform the triangle data from the buffer. The vertex indices
   may need to be offset based on the concatenation operation. */
  auto tris = tri_data.GetValues<unsigned int, 3>();
  const int prev_v_count = render_mesh->positions.rows();
  if (prev_v_count > 0) {
    /* The presence of vertex data requires index remapping. */
    const auto offset = Vector3<unsigned int>::Constant(
        static_cast<unsigned int>(prev_v_count));
    for (int r = 0; r < tris.rows(); ++r) {
      tris.row(r) += offset;
    }
  }

  /* Vertex attribute data must likewise be transformed to eliminate the
   transforms internal to the glTF file and then re-express in Drake's z-up
   frame. */
  auto pos = pos_data.GetValues<double, 3>();
  auto norm = norm_data.GetValues<double, 3>();
  auto uvs = uv_data.GetValues<double, 2>();

  Eigen::Matrix4d X_DF;
  // clang-format off
  X_DF << 1, 0,  0, 0,
          0, 0, -1, 0,
          0, 1,  0, 0,
          0, 0,  0, 1;
  // clang-format on
  Eigen::Matrix4d T_DN = X_DF * prim.T_FN;
  for (int v = 0; v < add_v_count; ++v) {
    const Vector4d p_FV(pos.row(v)[0], pos.row(v)[1], pos.row(v)[2], 1.0);
    pos.row(v) = (T_DN * p_FV).head<3>();
    /* TODO: Normals need to be scaled by the *inverse* scale. For non-identity
     scale, I'll need to extract the scale (and the orientation of that
     scale vector to apply it to the normals -- and then re-normalize). */
    norm.row(v) = T_DN.block<3, 3>(0, 0) * norm.row(v).transpose();
  }

  /* Now concatenate the new data onto the old. */
  render_mesh->positions.resize(prev_v_count + add_v_count, 3);
  render_mesh->positions.block(prev_v_count, 0, add_v_count, 3) = pos;

  render_mesh->normals.resize(prev_v_count + add_v_count, 3);
  render_mesh->normals.block(prev_v_count, 0, add_v_count, 3) = norm;

  render_mesh->uvs.resize(prev_v_count + add_v_count, 2);
  render_mesh->uvs.block(prev_v_count, 0, add_v_count, 2) = uvs;

  const int old_tri_count = render_mesh->indices.rows();
  render_mesh->indices.resize(old_tri_count + tris.rows(), 3);
  render_mesh->indices.block(old_tri_count, 0, tris.rows(), 3) = tris;
}

/* Creates a transform from the given `nodes` data. */
Matrix4d EigenMatrixFromNode(const tinygltf::Node& node) {
  Matrix4d T;
  if (node.matrix.size() == 16) {
    // For glTF, transform matrix is a *column-major* matrix.
    int i = -1;
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) {
        T(r, c) = node.matrix.at(++i);
      }
    }
  } else {
    T = Matrix4d::Identity();
    if (node.translation.size() > 0) {
      DRAKE_DEMAND(node.translation.size() == 3);
      const Vector3d p(node.translation.at(0), node.translation.at(1),
                       node.translation.at(2));
      T.block<3, 1>(0, 3) = p.transpose();
    }
    if (node.rotation.size() > 0) {
      DRAKE_DEMAND(node.rotation.size() == 4);
      const Quaternion<double> quat(node.rotation[3], node.rotation[0],
                                    node.rotation[1], node.rotation[2]);
      T.block<3, 3>(0, 0) = math::RotationMatrixd(quat).matrix();
    }
    if (node.scale.size() > 0) {
      DRAKE_DEMAND(node.scale.size() == 3);
      for (int i = 0; i < 3; ++i) {
        T.block<3, 1>(0, i) *= node.scale.at(i);
      }
    }
  }
  return T;
}

/* Recursively traverses the node hierarchy rooted at the node with the given
 index. For each *mesh* node, it adds the mesh posed mesh primitives to the
 given material-to-primitive map.

 @param node_index   The index of the node N at the root of the sub tree.
 @param T_FP         The transform for N's parent node P relative to the file
                     frame F.
 @param primitives   A map from material index to the primitives that use it.
 */
void FindPrimitivesInTree(const tinygltf::Model& model, int node_index,
                          const Matrix4d& T_FP,
                          map<int, vector<PosedPrimitive>>* primitives) {
  const tinygltf::Node& node = model.nodes.at(node_index);

  const Matrix4d T_PN = EigenMatrixFromNode(node);
  const Matrix4d T_FN = T_FP * T_PN;

  if (node.mesh >= 0) {
    const tinygltf::Mesh& mesh = model.meshes.at(node.mesh);
    for (const tinygltf::Primitive& prim : mesh.primitives) {
      (*primitives)[prim.material].emplace_back(T_FN, &prim);
    }
  }
  for (int child_index : node.children) {
    FindPrimitivesInTree(model, child_index, T_FN, primitives);
  }
}

/* Creates a map from material index to all mesh primitives in the hierarchy
 forest whose trees are rooted at the nodes with the given root indices. */
map<int, vector<PosedPrimitive>> FindPrimitivesInForest(
    const tinygltf::Model& model, const vector<int>& root_indices) {
  map<int, vector<PosedPrimitive>> primitives_by_material;
  const Matrix4d I = Matrix4d::Identity();
  for (int node_index : root_indices) {
    FindPrimitivesInTree(model, node_index, I, &primitives_by_material);
  }
  return primitives_by_material;
}

/* Given the indices of the root nodes, builds the render data based on the
 hierarchies rooted at those nodes. */
vector<RenderMesh>
MakeRenderMeshesFromNodes(const tinygltf::Model& model, const fs::path& path,
                          const drake::internal::DiagnosticPolicy& policy,
                          const vector<int> root_nodes,
                          const GeometryProperties& properties,
                          const Rgba& default_diffuse) {
  vector<RenderMesh> meshes;

  map<int, vector<PosedPrimitive>> primitives_by_material =
      FindPrimitivesInForest(model, root_nodes);
  for (const auto& [mat_index, primitives] : primitives_by_material) {
    RenderMesh render_mesh;

    // TODO(SeanCurtis-TRI): Perhaps it would be better to partition the
    // primitives in those that *do* have texture coordinates and those that
    // don't. Then I could create two variants of the material, one with
    // textures for the meshes with UVs, and one without for those that don't.

    int uv_count = 0;
    for (const PosedPrimitive& posed_prim : primitives) {
      if (posed_prim.primitive->attributes.count("TEXCOORD_0") > 0) {
        uv_count += 1;
      }
    }
    /* Create the material for set of triangles. */
    render_mesh.uv_state = uv_count == 0                   ? UvState::kNone
                           : uv_count == ssize(primitives) ? UvState::kFull
                                                           : UvState::kPartial;
    // Build the material.
    if (mat_index < 0 || mat_index >= ssize(model.materials) ||
        render_mesh.uv_state != UvState::kFull) {
      if (mat_index >= ssize(model.materials)) {
        policy.Warning(fmt::format(
            "A glTF file specifies a material with a bad index ({}) in {}. "
            "Using the fallback material.",
            mat_index, path));
      }
      // Warnings for uv_state are issued by MakeMeshFallbackMaterial().
      render_mesh.material = *MaybeMakeMeshFallbackMaterial(
          properties, path, default_diffuse, policy, render_mesh.uv_state);
    } else {
      render_mesh.material = MakeGltfMaterial(model, path, mat_index);
    }

    // TODO(SeanCurtis-TRI): It would be better to enumerate the buffers spanned
    // by these primitives. Currently, each time a buffer is referenced, it
    // gets copied into the RenderMesh again. Instead, the vertex data should
    // get copied once, and the referenced multiple times. Or, better yet, if
    // we have multiple instances of a single buffer (material and all), it
    // should be instanced in the render engine as well.
    for (const PosedPrimitive& posed_prim : primitives) {
      AccumulateMeshData(model, path, posed_prim, &render_mesh);
    }
    meshes.push_back(std::move(render_mesh));
  }
  return meshes;
}

}  // namespace

std::pair<std::vector<RenderMesh>, std::map<std::string, MemoryImageFile>>
GetRenderMeshesFromGltf(fs::path gltf_path,
                        const GeometryProperties& properties,
                        const Rgba& default_diffuse,
                        const drake::internal::DiagnosticPolicy& policy) {
  std::ifstream f(gltf_path);
  if (!f.is_open()) {
    policy.Error(fmt::format("Unable to read '{}'", gltf_path.string()));
    return {{}, {}};
  }

  std::stringstream ss;
  ss << f.rdbuf();
  return GetRenderMeshesFromGltfFromString(ss.str(), properties,
                                           default_diffuse, policy, gltf_path);
}

std::pair<std::vector<RenderMesh>, std::map<std::string, MemoryImageFile>>
GetRenderMeshesFromGltfFromString(std::string_view gltf_contents,
                        const GeometryProperties& properties,
                        const Rgba& default_diffuse,
                        const drake::internal::DiagnosticPolicy& policy,
                        fs::path gltf_path) {
  tinygltf::TinyGLTF loader;

  map<string, MemoryImageFile> images;
  // Callback for every *embedded* image. *Not* called for external images.
  auto load_image_cb =
      [&images, &gltf_path](
          tinygltf::Image* image, const int image_index, std::string* /*err*/,
          std::string* /*warn*/, int /*req_width*/, int /*req_height*/,
          const unsigned char* bytes, int size, void* /*user_data*/) -> bool {
    // We'll create a uri for this in-memory image that can be referenced
    // in the texture library.
    const string image_key =
        fmt::format("embedded:{}?image={}", gltf_path.string(), image_index);
    image->uri = image_key;
    DRAKE_DEMAND(!image->mimeType.empty());
    vector<unsigned char> data(bytes, bytes + size);
    images.insert(
        {image_key, MemoryImageFile{image->mimeType, std::move(data)}});
    return true;
  };
  loader.SetImageLoader(load_image_cb, nullptr);

  string error;
  string warn;

  tinygltf::Model model;
  const bool valid_parse = loader.LoadASCIIFromString(
      &model, &error, &warn, gltf_contents.data(), gltf_contents.size(),
      gltf_path.parent_path().string());

  if (!valid_parse) {
    throw std::runtime_error(fmt::format("Failed parsing the glTF file: {}: {}",
                                         gltf_path.string(), error));
  }

  /* We better not get any errors if we have a valid parse. */
  DRAKE_DEMAND(error.empty());
  if (!warn.empty()) {
    policy.Warning(warn);
  }

  /* The root nodes of all the hierarchies that will be instantiated (by
   index). */
  vector<int> root_indices = FindTargetRootNodes(model, gltf_path, policy);

  /* Walk the forest of node trees rooted in the target root nodes. Create a
   RenderMesh for each unique material applied to a glTF primitive in the
   forest. */
  return {MakeRenderMeshesFromNodes(model, gltf_path, policy, root_indices,
                                    properties, default_diffuse),
          std::move(images)};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
