#include "drake/geometry/render/internal_gltf_parser.h"

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
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::map;
using std::pair;
using std::string;
using std::vector;

// TODO(SeanCurtis-TRI): rather than having tinygltf read all of the image
// data from disk, we should defer that to Drake. That'll save us load time to
// just those images we explicitly care about. This would be done by setting
// the TINYGLTF_NO_EXTERNAL_IMAGE compiler flag and then handling all of the
// image loading on our end. Note: this is *only* external images. Any images
// stored as data:// uris will get decoded by tinygltf. We'd need to handle the
// case where a material references an unsupported image type (like .ktx2).

// We'll force tingltf to skip all of the ktx2 files when loading images.
bool DontLoadKtx2Data(tinygltf::Image* image, const int image_idx,
                      std::string* err, std::string* warn, int req_width,
                      int req_height, const unsigned char* bytes, int size,
                      void* user_data) {
  if (image->uri.ends_with(".ktx2")) {
    // The return value indicates whether there was an error; skipping is *not*
    // an error.
    return true;
  }
  return tinygltf::LoadImageData(image, image_idx, err, warn, req_width,
                                 req_height, bytes, size, user_data);
}

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

/* Convenience class for reading entries from a glTF buffer *for meshes*.*/
class BufferReader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BufferReader);

  /* Creates a zero-filled dummy reader that can write out zeros for
   `count` elements each consisting of `byte_size` number of bytes. */
  BufferReader(int count, int byte_size) {
    const int total_bytes = count * byte_size;
    owned_buffer_ = vector<unsigned char>(total_bytes, 0);
    buffer_ = owned_buffer_.data();
    end_ = buffer_ + total_bytes;
    element_count_ = count;
    byte_size_ = byte_size;
    byte_stride_ = 0;
  }

  /* Constructs the reader for triangle indices. */
  BufferReader(int tri_accessor_index, const tinygltf::Model& model,
               const std::string& file_name) {
    ThrowIfInvalidIndex(tri_accessor_index, model.accessors, "mesh primitive",
                        "accessor", " for its indices", file_name);
    Initialize(model.accessors.at(tri_accessor_index),
               "mesh primitive's indices", model, file_name);
  }

  /* Constructs the reader on the model for the indexed accessor. The `model`
   parameter is aliased by this instance and must outlive it. */
  BufferReader(const tinygltf::Primitive& primitive,
               const std::string& attribute, const tinygltf::Model& model,
               const std::string& file_name) {
    const auto iter = primitive.attributes.find(attribute);
    if (iter != primitive.attributes.end()) {
      const int accessor_index = iter->second;
      ThrowIfInvalidIndex(
          accessor_index, model.accessors, "mesh primitive", "accessor",
          fmt::format(" for the {} attribute", attribute), file_name);
      const tinygltf::Accessor& accessor = model.accessors.at(accessor_index);
      Initialize(accessor,
                 fmt::format("mesh primitive's {} attribute", attribute), model,
                 file_name);
    }
  }

  /* Returns the number of elements this reader can produce. */
  int count() const { return element_count_; }

  /* Writes a contiguous block of data from the ith element in the buffer to
   the given pointer. */
  void WriteElement(int i, void* data) {
    DRAKE_DEMAND(i >= 0 && i < element_count_);
    std::memcpy(data, buffer_ + (i * byte_stride_), byte_size_);
  }

 private:
  /* Initializes the buffer reader for the given accessor. */
  void Initialize(const tinygltf::Accessor& accessor,
                  std::string_view accessor_owner, const tinygltf::Model& model,
                  const std::string& file_name) {
    element_count_ = accessor.count;
    const int component_count = ComponentSize(accessor.componentType);
    /* Tinygltf swaps the glTF strings with encoded ints in accessor.type. The
     number of components in the accessor type are encoded in the lowest
     *five* bits (because the largest possible value is 16). */
    const int component_size = (accessor.type & 0x1f);
    byte_size_ = component_count * component_size;

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
    buffer_ = buffer.data.data() + accessor.byteOffset + buffer_view.byteOffset;
  }

  const unsigned char* buffer_{};
  const unsigned char* end_{};
  /* The number of elements. */
  int element_count_{};
  /* The size of a single element (e.g., 1-byte for scalar byte, 12 bytes for
   Vec3 of floats). */
  int byte_size_{};
  /* The distance (in bytes) between elements in the buffer. */
  int byte_stride_{};
  /* For the dummy reader, a vector of zeros. */
  std::vector<unsigned char> owned_buffer_{};
};

}  // namespace

class DRAKE_NO_EXPORT GltfParser::Impl {
 public:
  Impl(std::filesystem::path gltf_path, const DiagnosticPolicy* policy)
      : path_(std::move(gltf_path)), policy_(*policy) {
    DRAKE_DEMAND(policy != nullptr);
    tinygltf::TinyGLTF loader;
    loader.SetImageLoader(&DontLoadKtx2Data, nullptr);
    string error;
    string warn;

    const bool valid_parse =
        loader.LoadASCIIFromFile(&model_, &error, &warn, path_.string());

    if (!valid_parse) {
      throw std::runtime_error(fmt::format(
          "Failed parsing the glTF file: {}: {}", path_.string(), error));
    }

    /* We better not get any errors if we have a valid parse. */
    DRAKE_DEMAND(error.empty());
    if (!warn.empty()) {
      policy_.Warning(warn);
    }
  }

  pair<vector<RenderMesh>, map<string, RenderTexture>> ExtractRenderData(
      const GeometryProperties& properties, const Rgba& default_diffuse) {
    /* The root nodes of all the hierarchies that will be instantiated (by
     index). */
    vector<int> root_indices = FindTargetRootNodes();

    /* Walk the forest of node trees rooted in the target root nodes. Create a
     RenderMesh for each unique material applied to a glTF primitive in the
     forest. */
    return MakeRenderMeshesFromNodes(root_indices, properties, default_diffuse);
  }

 private:
  /* Identifies the source scene from the glTF file and returns the indices of
   that scene's root nodes. If no default scene can be identified, then
   all root nodes in the file are returned. */
  vector<int> FindTargetRootNodes() {
    /* The root nodes of all the hierarchies that will be instantiated (by
     index). */
    vector<int> root_indices;
    if (model_.scenes.size() > 0) {
      if (model_.defaultScene >= ssize(model_.scenes)) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it defines {} scenes but has an "
            "invalid value for the \"scene\" property: {}. '{}'. No geometry "
            "will be added.",
            model_.scenes.size(), model_.defaultScene, path_.string()));
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
      if (model_.defaultScene < 0 && ssize(model_.scenes) > 1) {
        policy_.Warning(fmt::format(
            "Parsing a glTF file with multiple scene and no explicit "
            "default scene; using the zeroth scene: '{}'.",
            path_.string()));
      }
      // tinygltf initializes defaultScene to -1 to indicate an undefined value
      const int scene_index = std::max(model_.defaultScene, 0);
      /* TODO: Can I trust that these are actually root nodes? Will tinygltf
       catch them if there's an error? */
      root_indices = model_.scenes[scene_index].nodes;
    } else {
      if (model_.nodes.size() == 0) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it has no scenes and no nodes. '{}'.",
            path_.string()));
      }
      root_indices = FindAllRootNodes();
      if (root_indices.empty() && model_.nodes.size() > 0) {
        policy_.Error(
            fmt::format("Error parsing a glTF file; none of its {} nodes are "
                        "root nodes. '{}'.",
                        model_.nodes.size(), path_.string()));
      }
    }
    return root_indices;
  }

  /* Simply returns the indices of all nodes that have no parents (are root
   nodes). It searches *all* the nodes, unconstrained by what may or may not be
   indicated by the model's scenes. */
  vector<int> FindAllRootNodes() {
    vector<bool> has_parent(model_.nodes.size(), false);
    for (const auto& node : model_.nodes) {
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

  /* Given the indices of the root nodes, builds the render data based on the
   hierarchies rooted at those nodes. */
  pair<vector<RenderMesh>, map<string, RenderTexture>>
  MakeRenderMeshesFromNodes(const vector<int> root_nodes,
                            const GeometryProperties& properties,
                            const Rgba& default_diffuse) {
    vector<RenderMesh> meshes;
    map<string, RenderTexture> image_cache;

    map<int, vector<PosedPrimitive>> primitives_by_material =
        FindPrimitivesInForest(root_nodes);
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
      render_mesh.uv_state = uv_count == 0 ? UvState::kNone
                             : uv_count == ssize(primitives)
                                 ? UvState::kFull
                                 : UvState::kPartial;
      // Build the material.
      if (mat_index < 0 || mat_index >= ssize(model_.materials) ||
          render_mesh.uv_state != UvState::kFull) {
        if (mat_index >= ssize(model_.materials)) {
          policy_.Warning(fmt::format(
              "A glTF file specifies a material with a bad index ({}) in {}. "
              "Using the fallback material.",
              mat_index, path_));
        }
        // Warnings for uv_state are issued by MakeMeshFallbackMaterial().
        render_mesh.material = *MaybeMakeMeshFallbackMaterial(
            properties, path_, default_diffuse, policy_, render_mesh.uv_state);
      } else {
        render_mesh.material = MakeGltfMaterial(mat_index, &image_cache);
      }

      for (const PosedPrimitive& posed_prim : primitives) {
        AccumulateMeshData(posed_prim, &render_mesh);
      }
      meshes.push_back(std::move(render_mesh));
    }
    return {std::move(meshes), std::move(image_cache)};
  }

  /* Creates a RenderMaterial for the glTF material indicated by the given
   index. Currently, we produce the most rudimentary of phong materials; we
   extract the diffuse color (aka baseColorFactor) and (possibly) diffuse color
   map. The diffuse map is added to the image_cache and the material's
   diffuse_map value contains the key for the image in the cache. */
  RenderMaterial MakeGltfMaterial(int mat_index,
                                  map<string, RenderTexture>* image_cache) {
    RenderMaterial material;
    const tinygltf::Material gltf_mat = model_.materials.at(mat_index);
    const tinygltf::PbrMetallicRoughness& gltf_pbr =
        gltf_mat.pbrMetallicRoughness;
    const vector<double>& gltf_rgba = gltf_pbr.baseColorFactor;
    material.diffuse =
        Rgba(gltf_rgba[0], gltf_rgba[1], gltf_rgba[2], gltf_rgba[3]);
    const int diffuse_index = gltf_pbr.baseColorTexture.index;
    if (diffuse_index >= 0) {
      if (gltf_pbr.baseColorTexture.texCoord != 0) {
        // TODO(SeanCurtis-TRI) Would this be better as a one-time warning?
        log()->debug(
            "Drake's native support of glTF files only includes zero-indexed "
            "texture coordinates. The material '{}' specifies texture "
            "coordinates {} in {}.",
            gltf_mat.name.empty() ? string("<unnamed>") : gltf_mat.name,
            gltf_pbr.baseColorTexture.texCoord, path_.string());
      }
      const tinygltf::Image& image = model_.images.at(diffuse_index);
      if (image.uri.starts_with("data:")) {
        // An embedded texture needs to be included in the image cache.
        const string image_key =
            fmt::format("{}?image={}", path_.string(), diffuse_index);
        material.diffuse_map = image_key;
        if (image_cache->count(image_key) == 0) {
          // The image is not "as is" encoded; it has been decoded into pixels.
          DRAKE_DEMAND(image.as_is == false);
          RenderTexture render_image{.width = image.width,
                                     .height = image.height,
                                     .channels = image.component,
                                     .bits = image.bits,
                                     .pixel_type = image.pixel_type};
          render_image.pixel_data.reserve(image.image.size());
          std::copy(image.image.begin(), image.image.end(),
                    std::back_inserter(render_image.pixel_data));
          (*image_cache)[image_key] = std::move(render_image);
        }
      } else {
        // This implicitly ignores the extension used ktx textures (if named).
        material.diffuse_map =
            (path_.parent_path() / image.uri).lexically_normal();
      }
    }
    // TODO(SeanCurtis-TRI): Debug messages for any of the other textures that
    // we're currently not supporting.
    return material;
  }

  /* A glTF primitive and its transformed pose in the file (the pose may not be
   a rigid transform; it can include scale). A primitive's pose is the same as
   its node's (N) pose (as only nodes get posed). This flattens the node
   hierarchy but as we are rendering it as a rigid assembly, we lose nothing. */
  struct DRAKE_NO_EXPORT PosedPrimitive {
    Eigen::Matrix4d T_FN;
    const tinygltf::Primitive* primitive{};
  };

  /* Recursively traverses the node hierarchy rooted at the node with the given
   index. For each *mesh* node, it adds the mesh posed mesh primitives to the
   given material-to-primitive map.

   @param node_index   The index of the node N at the root of the sub tree.
   @param T_FP         The transform for N's parent node P relative to the file
                       frame F.
   @param primitives   A map from material index to the primitives that use it.
   */
  void FindPrimitivesInTree(int node_index, const Matrix4d& T_FP,
                            map<int, vector<PosedPrimitive>>* primitives) {
    const tinygltf::Node& node = model_.nodes.at(node_index);

    const Matrix4d T_PN = EigenMatrixFromNode(node);
    const Matrix4d T_FN = T_FP * T_PN;

    if (node.mesh >= 0) {
      const tinygltf::Mesh& mesh = model_.meshes.at(node.mesh);
      for (const tinygltf::Primitive& prim : mesh.primitives) {
        (*primitives)[prim.material].emplace_back(T_FN, &prim);
      }
    }
    for (int child_index : node.children) {
      FindPrimitivesInTree(child_index, T_FN, primitives);
    }
  }

  /* Creates a map from material index to all mesh primitives in the hierarchy
   forest whose trees are rooted at the nodes with the given root indices. */
  map<int, vector<PosedPrimitive>> FindPrimitivesInForest(
      const vector<int>& root_indices) {
    map<int, vector<PosedPrimitive>> primitives_by_material;
    const Matrix4d I = Matrix4d::Identity();
    for (int node_index : root_indices) {
      FindPrimitivesInTree(node_index, I, &primitives_by_material);
    }
    return primitives_by_material;
  }

  void AccumulateMeshData(const PosedPrimitive& prim, RenderMesh* render_mesh) {
    if (prim.primitive->mode != TINYGLTF_MODE_TRIANGLES) {
      log()->debug(fmt::format(
          "Drake's native support of glTF files requires that all primitives "
          "use the triangle mode ({}). At least one primitive has an "
          "unsupported  mode: {}. The primitive will be ignored. {}.",
          TINYGLTF_MODE_TRIANGLES, prim.primitive->mode, path_.string()));
      return;
    }

    BufferReader tri_data(prim.primitive->indices, model_, path_.string());
    BufferReader pos_data(*prim.primitive, "POSITION", model_, path_.string());
    DRAKE_DEMAND(pos_data.count() > 0);
    // No normals is a failure.
    BufferReader norm_data(*prim.primitive, "NORMAL", model_, path_.string());
    if (norm_data.count() == 0) {
      // The current practice for parsing an OBJ without normals is to simply
      // throw, so we'll treat glTF the same.
      throw std::runtime_error(fmt::format(
          "Drake's native support of glTF files requires that all primitives "
          "define normals. At least one primitive is missing normals. {}",
          path_.string()));
    }

    // When parsing an OBJ, if there are no texture coordinates, we set the
    // primitives to all zeros and mark the uv state appropriately. To maintain
    // compatibility, we'll do the same here.
    BufferReader uv_data(*prim.primitive, "TEXCOORD_0", model_, path_.string());
    if (uv_data.count() == 0) {
      // Replace the buffer reader with a zero-valued reader. A single texture
      // coordinate is 2 floats, for a size of 8 bytes.
      uv_data = BufferReader(pos_data.count(), 8);
      if (render_mesh->uv_state == UvState::kFull) {
        render_mesh->uv_state = UvState::kPartial;
      }
    } else {
      if (render_mesh->uv_state == UvState::kNone) {
        // If the render mesh hasn't accumulated any geometry yet, the current
        // uv state is meaningless and we can simply set to full. Otherwise,
        // "none" means previous primitives lacked UVs, so it becomes partial.
        if (render_mesh->positions.rows() == 0) {
          render_mesh->uv_state = UvState::kFull;
        } else {
          render_mesh->uv_state = UvState::kPartial;
        }
      }
    }

    DRAKE_DEMAND(pos_data.count() == norm_data.count());
    DRAKE_DEMAND(pos_data.count() == uv_data.count());

    /* The map from indices in the glTF file to indices in the RenderMesh.  */
    map<int, int> gltf_vertex_to_new_vertex;
    /* Accumulators for vertex positions, normals, and uvs. We can't write */
    vector<Vector3d> positions(pos_data.count());
    vector<Vector3d> normals(pos_data.count());
    vector<Vector2d> uvs(pos_data.count());

    /* Note: tri_data should have 3*N entries; each entry is an index into the
     vertices. */
    DRAKE_DEMAND(tri_data.count() % 3 == 0);
    const int tri_count = tri_data.count() / 3;
    const int tri_offset = render_mesh->indices.rows();
    render_mesh->indices.resize(tri_offset + tri_count, 3);

    /* The number of pre-existing vertices in the render_mesh. */
    const int v_offset = render_mesh->positions.size();
    int local_index = 0;
    for (int i = 0; i < tri_data.count(); ++i) {
      int v_index = 0;
      // N.B. If the glTF has stored indices as shorts, we're assuming
      // appropriate endianness, such that the two bytes written to this four
      // byte int will report the right value.
      tri_data.WriteElement(i, &v_index);
      if (gltf_vertex_to_new_vertex.count(v_index) > 0) {
        continue;
      }
      const int new_index = v_offset + local_index;
      gltf_vertex_to_new_vertex[v_index] = new_index;
      pos_data.WriteElement(i, positions[local_index].data());
      norm_data.WriteElement(i, normals[local_index].data());
      uv_data.WriteElement(i, uvs[local_index].data());
      ++local_index;

      const int tri_index = i / 3;
      const int tri_vert_index = i % 3;
      render_mesh->indices(tri_index, tri_vert_index) = new_index;
    }

    // Now I need to concatenate the vectors I've made to render mesh's
    // data.
    //  1. Resize each of the arrays by the number of vertices I have.
    //  2. Write.
    render_mesh->positions.resize(v_offset + ssize(positions), 3);
    render_mesh->normals.resize(v_offset + ssize(positions), 3);
    render_mesh->uvs.resize(v_offset + ssize(positions), 2);
    for (int v = 0; v < ssize(positions); ++v) {
      render_mesh->positions.row(v + v_offset) = positions[v];
      render_mesh->normals.row(v + v_offset) = normals[v];
      render_mesh->uvs.row(v + v_offset) = uvs[v];
    }
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

  // The path to the glTF being parsed.
  std::filesystem::path path_;

  const drake::internal::DiagnosticPolicy& policy_;

  // The parsed glTF file.
  tinygltf::Model model_;
};

GltfParser::GltfParser(std::filesystem::path gltf_path,
                       const DiagnosticPolicy* policy)
    : impl_(new GltfParser::Impl(std::move(gltf_path), policy)) {}

GltfParser::~GltfParser() {
  delete impl_;
}

pair<vector<RenderMesh>, map<string, RenderTexture>>
GltfParser::ExtractRenderData(const GeometryProperties& properties,
                              const Rgba& default_diffuse) {
  return impl_->ExtractRenderData(properties, default_diffuse);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
