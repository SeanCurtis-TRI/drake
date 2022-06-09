#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometryVersion;
using geometry::MeshFieldLinear;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Sphere;
using geometry::SurfaceTriangle;
using geometry::TriangleSurfaceMesh;
using math::RigidTransform;
using multibody::internal::FullBodyName;
using std::function;
using std::make_unique;
using std::move;
using std::optional;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using symbolic::Expression;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {

/* Friend class to expose the internals of ContactResultsToLcmSystem. */
class ContactResultsToLcmTester {
 public:
  ContactResultsToLcmTester() = delete;

  /* Provide access to the hidden constructor that allows control over the
   naming method -- so we can directly test that it works. */
  template <typename T>
  static unique_ptr<ContactResultsToLcmSystem<T>> Make() {
    return unique_ptr<ContactResultsToLcmSystem<T>>(
        new ContactResultsToLcmSystem<T>());
  }

  /* Returns a reference to the geometry id --> full body name data stored in
   the system's cache entry. */
  template <typename T>
  static const unordered_map<GeometryId, FullBodyName>&
  get_geometry_id_to_body_map(const Context<T>& context,
                              const ContactResultsToLcmSystem<T>& system) {
    return system.EvalHydroGeometryNames(context);
  }

  /* Returns the serial number for the geometry id --> full body name cache
   entry. */
  template <typename T>
  static int64_t geometry_id_to_body_serial_number(
      const Context<T>& context, const ContactResultsToLcmSystem<T>& system) {
    return system.get_cache_entry(system.hydro_name_data_cache_index_)
        .get_cache_entry_value(context)
        .serial_number();
  }

  /* Returns a reference to the body index --> body name data stored in
   the system's cache entry. */
  template <typename T>
  static const vector<string>& get_body_names(
      const Context<T>& context, const ContactResultsToLcmSystem<T>& system) {
    return system.EvalPointBodyNames(context);
  }

  /* Returns the serial number for the body index --> body name cache
   entry. */
  template <typename T>
  static int64_t body_names_serial_number(
      const Context<T>& context, const ContactResultsToLcmSystem<T>& system) {
    return system.get_cache_entry(system.point_body_name_cache_index_)
        .get_cache_entry_value(context)
        .serial_number();
  }
};

namespace internal {

/* For the purpose of this test, enable writing FullBodyName to string so that
 failure messages include human readable output (rather than bytestrings). */
std::ostream& operator<<(std::ostream& out, const FullBodyName& name) {
  out << "Model: '" << name.model << "', Body = '" << name.body << "', Geo: '"
      << name.geometry << "', is_unique: " << name.body_name_is_unique
      << ", geometry count: " << name.geometry_count;
  return out;
}

}  // namespace internal

namespace {

/* The fixed number of faces in the test mesh for hydroelastic contact. */
constexpr int kNumFaces = 2;
constexpr int kNumPointPerTri = 3;

/* Creates an arbitrary contact surface representing contact between the
 geometries identified by `id_M` and `id_N`. The surface data (vertex positions,
 pressure field) are largely garbage. The value aren't used in math, only to
 be copied and compared. The underlying mesh's vertex positions can be offset
 by the given `offset` value so that we can distinguish between two "different"
 meshes. */
template <typename T>
ContactSurface<T> MakeContactSurface(GeometryId id_M, GeometryId id_N,
                                     const Vector3<T>& offset) {
  /* Create the surface mesh first. It is simply two triangles (make sure we're
   looping through elements). The position of the vertices is offset by offset.
   The position of the vertices is irrelevant -- the mesh is just a collection
   of doubles that get copied. */
  vector<SurfaceTriangle> faces;
  vector<Vector3<T>> vertices;
  vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5) + offset);
  faces.emplace_back(0, 1, 2);
  faces.emplace_back(2, 3, 0);
  auto mesh = make_unique<TriangleSurfaceMesh<T>>(move(faces), move(vertices));

  /* Create the "e" field values (i.e., "hydroelastic pressure") - simply
   increasing values at each vertex. */
  vector<T> e_MN(mesh->num_vertices());
  std::iota(e_MN.begin(), e_MN.end(), 1);

  TriangleSurfaceMesh<T>* mesh_pointer = mesh.get();
  EXPECT_EQ(mesh->num_triangles(), kNumFaces);
  return ContactSurface<T>(
      id_M, id_N, move(mesh),
      make_unique<MeshFieldLinear<T, TriangleSurfaceMesh<T>>>(move(e_MN),
                                                              mesh_pointer));
}

/* Makes quadrature data sufficient for the ContactSurface generated by
 MakeContactSurface(). */
template <typename T>
vector<HydroelasticQuadraturePointData<T>> MakeQuadratureData(
    const Vector3<T>& offset) {
  /* We'll pick *three* quadrature points per triangle, to make sure we get
   proper loop iteration. For simplicitly, we'll explicitly define the first
   quadrature point and then express the others in terms of the first. */
  constexpr int kNumPoints = kNumFaces * kNumPointPerTri;
  vector<HydroelasticQuadraturePointData<T>> quadrature_point_data(kNumPoints);

  quadrature_point_data[0].p_WQ = Vector3<double>(1, 3, 5) + offset;
  quadrature_point_data[0].vt_BqAq_W = Vector3<double>(7, 11, 13) + offset;
  quadrature_point_data[0].traction_Aq_W = Vector3<double>(17, 19, 23) + offset;
  quadrature_point_data[0].face_index = 0;
  for (int i = 1; i < kNumPoints; ++i) {
    quadrature_point_data[i].p_WQ = quadrature_point_data[i - 1].p_WQ + offset;
    quadrature_point_data[i].vt_BqAq_W =
        quadrature_point_data[i - 1].vt_BqAq_W + offset;
    quadrature_point_data[i].traction_Aq_W =
        quadrature_point_data[i - 1].traction_Aq_W + offset;
    quadrature_point_data[i].face_index = i / kNumPointPerTri;
  }
  return quadrature_point_data;
}

/* Retrieves a set of fixed geometry ids -- always the same N values for a given
 N. */
template <int N>
const std::array<GeometryId, N>& GetGeometryIds() {
  static bool initialized = false;
  static std::array<GeometryId, N> ids;
  if (!initialized) {
    initialized = true;
    for (int i = 0; i < N; ++i) {
      ids[i] = GeometryId::get_new_id();
    }
  }
  return ids;
}

// TODO(2022-10-01) Kill this struct when deprecation is done.
// Foregoing scoped enum; favoring compactness for this short-lived struct.
enum TestMode {
  kFullyConnected,            // The new API with both ports connected.
  kDeprecatedNoSceneGraph,    // Deprecated constructor; no SceneGraph
                              // connection.
  kDeprecatedWithSceneGraph,  // Deprecated constructor; with SceneGraph
                              // connection.
};

/* A description of the canonical plant-scene graph diagram with contact result
 visualization. It includes:
   - The root Diagram and pointers to the scene graph, multibody plant, and
     contact visualization systems.
   - Two maps which have been pre-populated based on the bodies and geometries
     added to plant and scene_graph. It is what the cached name tables in
     contact_lcm should look like.
   - A quick look up between a body index and the collision geometries
     registered to that body. */
template <typename T>
struct TestSetup {
  unique_ptr<Diagram<T>> diagram;
  SceneGraph<T>* scene_graph{};
  MultibodyPlant<T>* plant{};
  ContactResultsToLcmSystem<T>* contact_lcm{};
  unordered_map<GeometryId, FullBodyName> expected_geo_body_map;
  vector<string> expected_body_names;
  vector<vector<GeometryId>> body_collision_geometries;
};

/* ContactResultsToLcmSystem basically does two things:

  - Compute cache entries that store a bunch of data about the population of
    collision geometry for the plant reporting contact.
  - Calculates its output value (lcm message), combining input ContactResults
    with the cached tables.

 The testing strategy exploits this functional decomposition directly. First we
 test various code paths for creating the cached tables and confirm that the
 contents are what we expect.

 Subsequently, we assume we know the table contents and pass in known
 ContactResults instances and confirm the resulting message is as expected.

 ContactResults contains various vectors of data (e.g., vertex positions,
 quadrature point data, etc.) The test *assumes* that the order of the
 data in the input is the order of the data on the output. It simplifies the
 test and only fails if the serialization perturbs the order. */
template <typename T>
class ContactResultsToLcmTest : public ::testing::Test {
 protected:
  /* Adds a body with the given `name` to the given `plant`-`scene_graph` pair
   as part of the given model instance.

   The constructor of ContactResultsToLcmSystem populates two tables. This
   method aids in building two corresponding tables which contain the expected
   contents. These can be compared directly with the tables in the dut after
   construction.

   The first table (`body_names`) maps body index to a mangled body name.

   The second table (`id_to_body`) maps collision geometry id to a collection of
   related collection of body name data. The provided FullBodyName, `ref_name`,
   should have the body_name_is_unique and geometry_count fields set.
   This method adds ref_name.geometry_count collision geometries to the
   new body. It inserts one entry into `id_to_body` by using the `ref_name` and
   setting the `model` name, `body` name, and `geometry` name. The geometry
   name is an arbitrary string.

   @param body_name        The name of the body to add.
   @param model_index      The model instance to which this body will be added.
   @param ref_name         A reference FullBodyName which has already defined
                           .geometry_count and .body_name_is_unique.
   @param plant            The plant to add the body to.
   @param scene_graph      The scene graph to register the geometries to.
   @param body_names       The ContactResultsToLcmSystem table that maps body
                           index to body name.
   @param id_to_body       The ContactResultsToLcmSystem table that maps
                           GeometryId to FullBodyName.
   @param body_geometries  The ith body maps to a vector of the geometry ids
                           registered to that body.
   @pre `model_index` is a valid model instance index. */
  void AddBody(const std::string& body_name, ModelInstanceIndex model_index,
               FullBodyName ref_name, MultibodyPlant<T>* plant,
               SceneGraph<T>* scene_graph, vector<string>* body_names,
               unordered_map<GeometryId, FullBodyName>* id_to_body,
               vector<vector<GeometryId>>* body_geometries) const {
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    const auto& body =
        plant->AddRigidBody(body_name, model_index,
                            SpatialInertia<double>{1.0, {0, 0, 0}, {1, 1, 1}});
    /* The expected format based on knowledge of the ContactResultToLcmSystem's
     implementation. */
    body_names->push_back(fmt::format("{}({})", body_name, model_index));
    DRAKE_DEMAND(body_geometries->size() == body.index());
    body_geometries->push_back({});

    /* We will *simulate* registering a collision geometry with MbP. Rather than
     instantiating SceneGraph<T> (which we can't even do for symbolic), we'll
     use friend access to shove a GeometryId into MbP's table of known,
     per-body collision geometries. That is sufficient for
     ContactResultsToLcmSystem to add entries to its tables. */
    ref_name.model = plant->GetModelInstanceName(model_index);
    ref_name.body = body.name();
    ProximityProperties props;
    geometry::AddContactMaterial(1e-4, 1e8, CoulombFriction<double>{0.1, 0.1},
                                 &props);
    for (int g = 0; g < ref_name.geometry_count; ++g) {
      const GeometryId g_id = plant->RegisterCollisionGeometry(
          body, {}, Sphere(0.25), fmt::format("TestGeometry({})", g), props);
      // MultibodyPlant *may* mangle the name of the geometry such that it is
      // different from name passed to RegisterCollisionGeometry. Rather than
      // worrying about what that mangling is, we'll simply retrieve the name
      // from scene_graph.
      ref_name.geometry = scene_graph->model_inspector().GetName(g_id);
      id_to_body->insert({g_id, ref_name});
      body_geometries->at(body.index()).push_back(g_id);
    }
  }

  /* Builds a "canonical" populated plant-scene graph diagram. This provides a
   common, initial configuration for all tests. Each invocation creates a new
   diagram in an "identical" initial state. Multiple invocations will only
   differ in GeometryId values, but there is a bijection between the GeometryIds
   across invocations. */
  TestSetup<T> CreateSetup(TestMode test_mode = kFullyConnected) const {
    TestSetup<T> setup;
    // World body has index 0 and should be included by default.
    setup.expected_body_names.push_back("WorldBody(0)");
    setup.body_collision_geometries.push_back({});

    DiagramBuilder<T> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

    /* Bodies 1 and 2 go to the same model instance. Body 3 goes to its own.
    Body 1 gets two geometries (to test the geometry_count field). Bodies 1
    and 3 have the same name (to test body_name_is_unique_field).  */
    const ModelInstanceIndex model12 = plant.AddModelInstance("JustForBody12");

    FullBodyName ref_name;

    ref_name.body_name_is_unique = false;
    ref_name.geometry_count = 2;
    this->AddBody("dupe_name", model12, ref_name, &plant, &scene_graph,
                  &setup.expected_body_names, &setup.expected_geo_body_map,
                  &setup.body_collision_geometries);

    ref_name.body_name_is_unique = true;
    ref_name.geometry_count = 1;
    this->AddBody("body2", model12, ref_name, &plant, &scene_graph,
                  &setup.expected_body_names, &setup.expected_geo_body_map,
                  &setup.body_collision_geometries);

    const ModelInstanceIndex model3 = plant.AddModelInstance("JustForBody3");
    ref_name.body_name_is_unique = false;
    this->AddBody("dupe_name", model3, ref_name, &plant, &scene_graph,
                  &setup.expected_body_names, &setup.expected_geo_body_map,
                  &setup.body_collision_geometries);

    plant.Finalize();

    // TODO(2020-10-01): When done deprecating, remove the test_mode parameter.
    // The final code should use the kFullyConnected branch.
    ContactResultsToLcmSystem<T>* contact_lcm{};
    if (test_mode == kFullyConnected) {
      contact_lcm = builder.AddSystem(ContactResultsToLcmTester::Make<T>());
      builder.Connect(scene_graph.get_query_output_port(),
                      contact_lcm->get_query_object_input_port());
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      contact_lcm = builder.AddSystem(unique_ptr<ContactResultsToLcmSystem<T>>(
          new ContactResultsToLcmSystem<T>(plant)));
#pragma GCC diagnostic pop
      if (test_mode != kDeprecatedNoSceneGraph) {
        builder.Connect(scene_graph.get_query_output_port(),
                        contact_lcm->get_query_object_input_port());
      }
    }
    builder.Connect(plant.get_contact_results_output_port(),
                    contact_lcm->get_contact_result_input_port());
    contact_lcm->set_name("contact_lcm");

    setup.plant = &plant;
    setup.scene_graph = &scene_graph;
    setup.contact_lcm = contact_lcm;
    setup.diagram = builder.Build();

    return setup;
  }

  /* Adds fake point-pair contact results to the given set of contact `results`.

   The contact is in terms of the bodies and geometries recorded in `setup`.

   @param setup     The data for the test configuration; it includes the indices
                    of the bodies in the MultibodyPlant included in the test and
                    the geometry ids associated with each body.
   @param results   New point contact results are written to this output. */
  template <typename U = T>
  static void AddFakePointPairContact(const TestSetup<T>& setup,
                                      ContactResults<U>* results) {
    DRAKE_DEMAND(results != nullptr);

    ContactResults<U> temp_results;
    temp_results.set_plant(setup.plant);

    /* CreateSetup() creates three bodies with indices 1, 2, & 3. To create
     point pair contact, it is sufficient to exercise those. Each has at least
     one geometry assocaited with it; we'll simply use its first. */
    DRAKE_DEMAND(setup.body_collision_geometries.size() >= 4);
    const BodyIndex b1{1};
    const GeometryId g1 = setup.body_collision_geometries[b1][0];
    const BodyIndex b2{2};
    const GeometryId g2 = setup.body_collision_geometries[b2][0];
    const BodyIndex b3{3};
    const GeometryId g3 = setup.body_collision_geometries[b3][0];

    /* Other than valid body indices and geometry ids, all numerical values are
     arbitrary garbage -- signal that we can use to determine the right value
     ended up in the write field. */
    PointPairContactInfo<U> pair1(
        b1, b2, Vector3<U>{1.1, 2.2, 3.3}, Vector3<U>{4.4, 5.5, 6.6}, 7.7, 8.8,
        PenetrationAsPointPair<U>{g1, g2, Vector3<U>{11.1, 22.2, 33.3},
                                  Vector3<U>{44.4, 55.5, 66.6},
                                  Vector3<U>{77.7, 88.8, 99.9}, 101.1});
    temp_results.AddContactInfo(pair1);

    PointPairContactInfo<U> pair2(
        b2, b3, Vector3<U>{1.2, 2.3, 3.4}, Vector3<U>{4.5, 5.6, 6.7}, 7.8, 8.9,
        PenetrationAsPointPair<U>{g2, g3, Vector3<U>{11.2, 22.3, 33.4},
                                  Vector3<U>{44.5, 55.6, 66.7},
                                  Vector3<U>{77.8, 88.9, 99.1}, 101.2});
    temp_results.AddContactInfo(pair2);

    /* By default, `AddContactInfo()` stores pointers to contact info stored
     elsewhere. However, when it gets copied, it converts external aliases to
     internally owned copies. By doing this copy assignement here, we cause
     the provided `results` instance to own its contact info. */
    *results = temp_results;
  }

  /* Adds fake hydro contact results to the given set of contact `results`.

   The contact is in terms of the bodies and geometries recorded in `setup`.

   @param setup     The data for the test configuration; it includes the indices
                    of the bodies in the MultibodyPlant included in the test and
                    the geometry ids associated with each body.
   @param results   New hydro contact results are written to this output. */
  template <typename U = T>
  static void AddFakeHydroContact(const TestSetup<T>& setup,
                                  ContactResults<U>* results) {
    DRAKE_DEMAND(results != nullptr);

    ContactResults<U> temp_results;
    temp_results.set_plant(setup.plant);


    /* CreateSetup() creates three bodies with indices 1, 2, & 3. To create
     point pair contact, it is sufficient to exercise those. Each has at least
     one geometry assocaited with it; we'll simply use its first. */
    DRAKE_DEMAND(setup.body_collision_geometries.size() >= 4);
    const BodyIndex b1{1};
    const GeometryId g1 = setup.body_collision_geometries[b1][0];
    const BodyIndex b2{2};
    const GeometryId g2 = setup.body_collision_geometries[b2][0];
    const BodyIndex b3{3};
    const GeometryId g3 = setup.body_collision_geometries[b3][0];

    /* Other than valid body indices and geometry ids, all numerical values are
     arbitrary garbage -- signal that we can use to determine the right value
     ended up in the write field. */
    ContactSurface<U> surface1(
        MakeContactSurface<U>(g1, g2, Vector3<U>{1, 2, 3}));
    HydroelasticContactInfo<U> pair1(
        &surface1,
        SpatialForce<U>(Vector3<U>(1.1, 2.2, 3.3), Vector3<U>(4.4, 5.5, 6.6)),
        MakeQuadratureData<U>(Vector3<U>{1, 2, 3}));
    temp_results.AddContactInfo(&pair1);

    ContactSurface<U> surface2(
        MakeContactSurface<U>(g2, g3, Vector3<U>{-3, -1, 2}));
    static const never_destroyed<HydroelasticContactInfo<U>> pair2(
        &surface2,
        SpatialForce<U>(Vector3<U>(1.2, 2.3, 3.4), Vector3<U>(4.5, 5.6, 6.7)),
        MakeQuadratureData<U>(Vector3<U>{-3, -1, -2}));
    temp_results.AddContactInfo(&pair2.access());

    /* By default, `AddContactInfo()` stores pointers to contact info stored
     elsewhere. However, when it gets copied, it converts external aliases to
     internally owned copies. By doing this copy assignement here, we cause
     the provided `results` instance to own its contact info. */
    *results = temp_results;
  }
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(ContactResultsToLcmTest, ScalarTypes);

/* Confirms the details of the tables. This is where we examine all the fields
 and match it against specific expectations. This test is *not* particularly
 concerned with the correctness of *when* the tables get computed; it simply
 assumes that the tables are available when its message output port is
 evaluated.

 We'll construct the a plant-scene graph pair with known contents, instantiate
 an instance of ContactResultsToLcmSystem on them, and then access the cache
 entries directly (using friend access), confirming they've been populated. */
TYPED_TEST(ContactResultsToLcmTest, CacheEntryContents) {
  using T = TypeParam;

  TestSetup<T> setup = this->CreateSetup();

  unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
  Context<T>& lcm_context =
      setup.contact_lcm->GetMyMutableContextFromRoot(context.get());
  // Empty results.
  ContactResults<T> results;
  results.set_plant(setup.plant);
  setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                              results);

  /* Evalute cache entries and examine the tables. */
  const auto& body_names = ContactResultsToLcmTester::get_body_names(
      lcm_context, *setup.contact_lcm);
  const auto& id_to_body_map =
      ContactResultsToLcmTester::get_geometry_id_to_body_map(
          lcm_context, *setup.contact_lcm);
  EXPECT_EQ(body_names, setup.expected_body_names);
  EXPECT_EQ(id_to_body_map, setup.expected_geo_body_map);
}

/* Confirms the circumstances in which the geo-->body table definition changes
 (or not).
  - Initial evaluation of output triggers change.
  - A change to the contact results does *not* cause a table change.
  - Change to scene graph that isn't Proximity based does not trigger a table
    change.
  - Change to proximity *does* trigger a table change.

 The body name table onlychanges upon initial evaluation of output.

 When the cache entry Eval method performs a calculation, it increments the
 cache entry value's serial number. We'll use changes in the serial number as
 evidence of invocations of calculation (or not). */
TYPED_TEST(ContactResultsToLcmTest, CacheEntryRecompute) {
  using T = TypeParam;

  TestSetup<T> setup = this->CreateSetup();
  unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
  Context<T>& lcm_context =
      setup.contact_lcm->GetMyMutableContextFromRoot(context.get());
  Context<T>& sg_context =
      setup.scene_graph->GetMyMutableContextFromRoot(context.get());

  int64_t body_name_number =
      ContactResultsToLcmTester::body_names_serial_number(lcm_context,
                                                          *setup.contact_lcm);
  int64_t geo_body_map_number =
      ContactResultsToLcmTester::geometry_id_to_body_serial_number(
          lcm_context, *setup.contact_lcm);

  // Empty results.
  ContactResults<T> results;
  results.set_plant(setup.plant);
  setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                              results);

  // Convenience function for evaluating the lcm output port and acquiring the
  // serial numbers for the "body names" and "geometry --> full body name" cache
  // entries.
  auto get_table_numbers = [&setup, &lcm_context]() {
    setup.contact_lcm->get_lcm_message_output_port()
        .template Eval<lcmt_contact_results_for_viz>(lcm_context);
    return std::pair<int64_t, int64_t>{
        ContactResultsToLcmTester::body_names_serial_number(lcm_context,
                                                            *setup.contact_lcm),
        ContactResultsToLcmTester::geometry_id_to_body_serial_number(
            lcm_context, *setup.contact_lcm)};
  };

  // Initial evaluation, even on empty results, triggers a table change.
  {
    auto [new_body_name_number, new_geo_body_map_number] = get_table_numbers();
    EXPECT_NE(body_name_number, new_body_name_number);
    EXPECT_NE(geo_body_map_number, new_geo_body_map_number);
    body_name_number = new_body_name_number;
    geo_body_map_number = new_geo_body_map_number;
  }

  // A change to contact results input does not trigger a table change.
  {
    this->AddFakePointPairContact(setup, &results);
    auto [new_body_name_number, new_geo_body_map_number] = get_table_numbers();
    EXPECT_EQ(body_name_number, new_body_name_number);
    EXPECT_EQ(geo_body_map_number, new_geo_body_map_number);
  }

  SceneGraph<T>& sg = *setup.scene_graph;
  const SceneGraphInspector<T>& inspector =
      sg.get_query_output_port()
          .template Eval<QueryObject<T>>(sg_context)
          .inspector();

  // Change to scene graph that doesn't change proximity version doesn't
  // trigger a table change.
  {
    // Modify scene graph's context so that illustration version changes, but
    // proximity remains fixed. (We're omitting perception because it only
    // changes if we have a RenderEngine added.)
    const GeometryVersion old_version = inspector.geometry_version();

    auto instance = make_unique<GeometryInstance>(
        RigidTransform<double>{}, make_unique<Sphere>(0.5), "visual");
    instance->set_illustration_properties({});
    sg.RegisterGeometry(&sg_context, *setup.plant->get_source_id(),
                        sg.world_frame_id(), move(instance));

    const GeometryVersion new_version = inspector.geometry_version();
    ASSERT_FALSE(new_version.IsSameAs(old_version, Role::kIllustration));
    ASSERT_TRUE(new_version.IsSameAs(old_version, Role::kProximity));

    auto [new_body_name_number, new_geo_body_map_number] = get_table_numbers();
    EXPECT_EQ(body_name_number, new_body_name_number);
    EXPECT_EQ(geo_body_map_number, new_geo_body_map_number);
  }

  // Changing scene graph proximity version *does* trigger a table change. Body
  // name table remains unchanged.
  {
    const GeometryVersion old_version = inspector.geometry_version();

    ProximityProperties props;
    geometry::AddContactMaterial(1e-4, 1e8, CoulombFriction<double>{0.1, 0.1},
                                 &props);
    auto instance = make_unique<GeometryInstance>(
        RigidTransform<double>{}, make_unique<Sphere>(0.5), "new_collision");
    instance->set_proximity_properties(props);
    sg.RegisterGeometry(&sg_context, *setup.plant->get_source_id(),
                        sg.world_frame_id(), move(instance));

    const GeometryVersion new_version = inspector.geometry_version();
    ASSERT_FALSE(new_version.IsSameAs(old_version, Role::kProximity));

    auto [new_body_name_number, new_geo_body_map_number] = get_table_numbers();
    EXPECT_EQ(body_name_number, new_body_name_number);
    EXPECT_NE(geo_body_map_number, new_geo_body_map_number);
  }
}

// TODO(2022-10-01) Delete this test outright.
/* There is a deprecated constructor. Historically, it has been possible to
 directly instantiate ContactResulsToLcmSystem and connect it to MbP. This test
 shows that things still "work" in that context. The cached tables are identical
 except the geometry names in the geometry --> full body name tables now have
 the form  Id(#). We stipulate that there are no other differences. */
TYPED_TEST(ContactResultsToLcmTest, DeprecatedConstruction) {
  using T = TypeParam;

  // Deprecated constructor without connection to SceneGraph.
  {
    TestSetup<T> setup = this->CreateSetup(kDeprecatedNoSceneGraph);

    unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
    Context<T>& lcm_context =
        setup.contact_lcm->GetMyMutableContextFromRoot(context.get());
    // Empty results.
    ContactResults<T> results;
    results.set_plant(setup.plant);
    setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                                results);

    // Body names are the same.
    const auto& body_names = ContactResultsToLcmTester::get_body_names(
        lcm_context, *setup.contact_lcm);
    EXPECT_EQ(body_names, setup.expected_body_names);

    // The geometry names in the geo --> body full name map will have changed.
    for (auto& [geometry_id, full_name] : setup.expected_geo_body_map) {
      setup.expected_geo_body_map[geometry_id].geometry =
          fmt::format("Id({})", geometry_id);
    }
    const auto& id_to_body_map =
        ContactResultsToLcmTester::get_geometry_id_to_body_map(
            lcm_context, *setup.contact_lcm);
    EXPECT_EQ(id_to_body_map, setup.expected_geo_body_map);
  }

  // Deprecated constructor with connection to SceneGraph.
  {
    TestSetup<T> setup = this->CreateSetup(kDeprecatedWithSceneGraph);

    unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
    Context<T>& lcm_context =
        setup.contact_lcm->GetMyMutableContextFromRoot(context.get());
    // Empty results.
    ContactResults<T> results;
    results.set_plant(setup.plant);
    setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                                results);

    // Body names are the same.
    const auto& body_names = ContactResultsToLcmTester::get_body_names(
        lcm_context, *setup.contact_lcm);
    EXPECT_EQ(body_names, setup.expected_body_names);

    // Even with the deprecated constructor, if it has been connected to
    // SceneGraph, the names come out as expected.
    const auto& id_to_body_map =
        ContactResultsToLcmTester::get_geometry_id_to_body_map(
            lcm_context, *setup.contact_lcm);
    EXPECT_EQ(id_to_body_map, setup.expected_geo_body_map);
  }
}

/* Given the correct cached tables (as tested above), confirm message contents.
 */
TYPED_TEST(ContactResultsToLcmTest, EmptyContactResults) {
  using T = TypeParam;

  TestSetup<T> setup = this->CreateSetup();
  unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
  const double kTime = 1.5;
  // Message time is an integer number of microseconds.
  const int64_t kTimeMicroSec = static_cast<int64_t>(kTime * 1e6);
  context->SetTime(kTime);
  Context<T>& lcm_context =
      setup.contact_lcm->GetMyMutableContextFromRoot(context.get());

  ContactResults<T> results_empty;
  results_empty.set_plant(setup.plant);

  setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                              results_empty);

  const auto& message =
      setup.contact_lcm->get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(lcm_context);

  EXPECT_EQ(message.timestamp, kTimeMicroSec);
  EXPECT_EQ(message.num_point_pair_contacts, 0);
  EXPECT_EQ(message.point_pair_contact_info.size(), 0);
  EXPECT_EQ(message.num_hydroelastic_contacts, 0);
  EXPECT_EQ(message.hydroelastic_contacts.size(), 0);
}

/* Tests the case where ContactResults contains *only* point-pair data. This
 test bears primary responsibility to make sure that point pair data is
 serialized correctly.

 Translation of point pair contact results to lcm message is straightforward.
 There are *three* things that this system does in the process:

   - Extracts double values from T-Valued quantities.
   - Looks up body names based on *body indices*.
   - Writes various Vector3 quantities (forces, positions, etc. -- all Vector3-
     valued data, but with different semantics). We'll want to make sure that
     each Vector3-valued quantity gets mapped to the right part of the message,
     because type checking isn't going to do it for us.

 As such, we don't need much for this test. We'll do two point-pair contacts
 with unique values to confirm successful iteration and copying. */
TYPED_TEST(ContactResultsToLcmTest, PointPairContactOnly) {
    using T = TypeParam;

  TestSetup<T> setup = this->CreateSetup();
  unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
  const double kTime = 1.5;
  // Message time is an integer number of microseconds.
  const int64_t kTimeMicroSec = static_cast<int64_t>(kTime * 1e6);
  context->SetTime(kTime);
  Context<T>& lcm_context =
      setup.contact_lcm->GetMyMutableContextFromRoot(context.get());

  ContactResults<T> results_point;
  this->AddFakePointPairContact(setup, &results_point);
  setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                              results_point);

  const auto& message =
      setup.contact_lcm->get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(lcm_context);

  ASSERT_EQ(message.num_point_pair_contacts, 2);
  ASSERT_EQ(message.point_pair_contact_info.size(), 2);
  ASSERT_EQ(message.num_hydroelastic_contacts, 0);
  ASSERT_EQ(message.hydroelastic_contacts.size(), 0);

  /* Test the message for a point pair contact against the input point pair
  data. */
  const auto& body_names = ContactResultsToLcmTester::get_body_names(
      lcm_context, *setup.contact_lcm);
  for (int c = 0; c < 2; ++c) {
    SCOPED_TRACE(fmt::format("Penetration point pair {} out of 2", (c + 1)));

    const auto& pair_message = message.point_pair_contact_info[c];
    const auto& pair_data = results_point.point_pair_contact_info(c);

    EXPECT_EQ(pair_message.timestamp, kTimeMicroSec);
    EXPECT_EQ(pair_message.body1_name, body_names[pair_data.bodyA_index()]);
    EXPECT_EQ(pair_message.body2_name, body_names[pair_data.bodyB_index()]);
    // clang-format off
      EXPECT_TRUE(CompareMatrices(
          Vector3<double>(pair_message.contact_point),
          ExtractDoubleOrThrow(pair_data.contact_point())));
      EXPECT_TRUE(CompareMatrices(
          Vector3<double>(pair_message.contact_force),
          ExtractDoubleOrThrow(pair_data.contact_force())));
      EXPECT_TRUE(CompareMatrices(
          Vector3<double>(pair_message.normal),
          ExtractDoubleOrThrow(pair_data.point_pair().nhat_BA_W)));
    // clang-format on
    /* None of the rest of the pair data makes it to the message. */
  }
}

/* Tests the case where ContactResults contains *only* hydroelastic data. This
 test bears primary responsibility to make sure that hydroelastic data is
 serialized correctly.

 Translation of hydroelastic contact results to lcm message is straightforward.
 There are *three* things that this system does in the process:

   - Extracts double values from T-Valued quantities.
   - Looks up body names based on geometry ids.
   - Writes various Vector3 quantities (forces, positions, etc. -- all Vector3-
     valued data, but with different semantics). We'll want to make sure that
     each Vector3-valued quantity gets mapped to the right part of the message,
     because type checking isn't going to do it for us.

 As such, we don't need much for this test. We'll do two hydroelastic contacts
 with unique values to confirm successful iteration and copying. */
TYPED_TEST(ContactResultsToLcmTest, HydroContactOnly) {
  using T = TypeParam;

  TestSetup<T> setup = this->CreateSetup();
  unique_ptr<Context<T>> context = setup.diagram->CreateDefaultContext();
  Context<T>& lcm_context =
      setup.contact_lcm->GetMyMutableContextFromRoot(context.get());

  ContactResults<T> results_hydro;
  this->AddFakeHydroContact(setup, &results_hydro);
  setup.contact_lcm->get_contact_result_input_port().FixValue(&lcm_context,
                                                              results_hydro);

  const auto& message =
      setup.contact_lcm->get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(lcm_context);

  ASSERT_EQ(message.num_point_pair_contacts, 0);
  ASSERT_EQ(message.point_pair_contact_info.size(), 0);
  ASSERT_EQ(message.num_hydroelastic_contacts, 2);
  ASSERT_EQ(message.hydroelastic_contacts.size(), 2);

  /* Test the message for hydro contact against the input hydro data. */
  const auto& geo_to_body_map =
      ContactResultsToLcmTester::get_geometry_id_to_body_map(
          lcm_context, *setup.contact_lcm);
  for (int c = 0; c < 2; ++c) {
    SCOPED_TRACE(fmt::format("Hydro contact {} out of 2", (c + 1)));

    const auto& pair_message = message.hydroelastic_contacts[c];
    const auto& pair_data = results_hydro.hydroelastic_contact_info(c);
    const auto& surface = pair_data.contact_surface();
    const auto& mesh = surface.tri_mesh_W();
    const auto& field = surface.tri_e_MN();

    const auto& name1 = geo_to_body_map.at(surface.id_M());
    EXPECT_EQ(pair_message.body1_name, name1.body);
    EXPECT_EQ(pair_message.model1_name, name1.model);
    EXPECT_EQ(pair_message.geometry1_name, name1.geometry);
    EXPECT_EQ(pair_message.body1_unique, name1.body_name_is_unique);
    EXPECT_EQ(pair_message.collision_count1, name1.geometry_count);

    const auto& name2 = geo_to_body_map.at(surface.id_N());
    EXPECT_EQ(pair_message.body2_name, name2.body);
    EXPECT_EQ(pair_message.model2_name, name2.model);
    EXPECT_EQ(pair_message.geometry2_name, name2.geometry);
    EXPECT_EQ(pair_message.body2_unique, name2.body_name_is_unique);
    EXPECT_EQ(pair_message.collision_count2, name2.geometry_count);

    /* Mesh aggregate results: centroid, force, moment. */
    // clang-format off
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.centroid_W),
        ExtractDoubleOrThrow(mesh.centroid())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.force_C_W),
        ExtractDoubleOrThrow(pair_data.F_Ac_W().translational())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.moment_C_W),
        ExtractDoubleOrThrow(pair_data.F_Ac_W().rotational())));
    // clang-format on

    /* Compare meshes and pressure fields. */

    /* Confirm vertices (count and values) and per-vertex pressure values. */
    EXPECT_EQ(static_cast<int>(pair_message.pressure.size()),
              mesh.num_vertices());
    EXPECT_EQ(pair_message.num_vertices, mesh.num_vertices());
    EXPECT_EQ(static_cast<int>(pair_message.p_WV.size()), mesh.num_vertices());
    for (int v = 0; v < mesh.num_vertices(); ++v) {
      const auto& point_WV = pair_message.p_WV[v];
      const Vector3<double> p_WV_message(point_WV.x, point_WV.y, point_WV.z);
      EXPECT_TRUE(CompareMatrices(p_WV_message, mesh.vertex(v)));
      EXPECT_EQ(pair_message.pressure[v], field.EvaluateAtVertex(v));
    }

    /* Confirm faces. Each triangle produces a sequence that looks like:
    // [3, i0, i1, i2] in the face data. Confirm size and contents. */
    ASSERT_EQ(pair_message.poly_data_int_count, mesh.num_triangles() * 4);
    ASSERT_EQ(pair_message.poly_data.size(), mesh.num_triangles() * 4);

    int index = -1;
    for (int f = 0; f < mesh.num_triangles(); ++f) {
      const auto& tri = mesh.element(f);
      ASSERT_EQ(pair_message.poly_data[++index], 3);
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(0));
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(1));
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(2));
    }

    /* Compare quadrature data. */
    const auto& data_quads = pair_data.quadrature_point_data();
    const auto& message_quads = pair_message.quadrature_point_data;
    EXPECT_EQ(pair_message.num_quadrature_points, data_quads.size());
    EXPECT_EQ(message_quads.size(), data_quads.size());
    EXPECT_EQ(pair_message.num_quadrature_points,
              mesh.num_triangles() * kNumPointPerTri);
    for (int q = 0; q < static_cast<int>(data_quads.size()); ++q) {
      // clang-format off
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].p_WQ),
        ExtractDoubleOrThrow(data_quads[q].p_WQ)));
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].vt_BqAq_W),
        ExtractDoubleOrThrow(data_quads[q].vt_BqAq_W)));
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].traction_Aq_W),
        ExtractDoubleOrThrow(data_quads[q].traction_Aq_W)));
      // clang-format on
    }
  }
}

/* We've shown that we can construct ContactResultsToLcmSystem for T = double
 and AutoDiffXd that they produce the same cache entry values. Now we show that
 the same holds true as a product of scalar conversion. */
TYPED_TEST(ContactResultsToLcmTest, NumericalScalarConversion) {
  using T = TypeParam;
  // We can only convert double -> AutoDiffXd.
  if constexpr (std::is_same_v<T, double>) {
    using U = AutoDiffXd;

    TestSetup<T> setup_t = this->CreateSetup();
    unique_ptr<Diagram<U>> diagram =
        systems::System<T>::template ToScalarType<U>(*setup_t.diagram);
    const auto& plant = dynamic_cast<const MultibodyPlant<U>&>(
        diagram->GetSubsystemByName("plant"));
    const auto& contact_lcm = dynamic_cast<const ContactResultsToLcmSystem<U>&>(
        diagram->GetSubsystemByName("contact_lcm"));

    // Show that the tables created for a T-valued system are reproduced by the
    // U-valued system.
    unique_ptr<Context<U>> context = diagram->CreateDefaultContext();
    Context<U>& lcm_context =
        contact_lcm.GetMyMutableContextFromRoot(context.get());
    // Empty results.
    ContactResults<U> results;
    results.set_plant(&plant);
    contact_lcm.get_contact_result_input_port().FixValue(&lcm_context, results);

    /* Evalute cache entries and examine the tables. */
    const auto& body_names =
        ContactResultsToLcmTester::get_body_names(lcm_context, contact_lcm);
    const auto& id_to_body_map =
        ContactResultsToLcmTester::get_geometry_id_to_body_map(lcm_context,
                                                               contact_lcm);
    EXPECT_EQ(body_names, setup_t.expected_body_names);
    EXPECT_EQ(id_to_body_map, setup_t.expected_geo_body_map);
  }
}

/* This merely illustrates that a double-valued ContactResultsToLcmSystem can
 be scalar converted to symbolic::Expression, but evaluating its outport throws
 the expected message. */
GTEST_TEST(ContactResultsToLcmTestScalar, SymbolicExpression) {
  unique_ptr<ContactResultsToLcmSystem<double>> dut_double =
      ContactResultsToLcmTester::Make<double>();
  unique_ptr<ContactResultsToLcmSystem<Expression>> dut_sym =
      systems::System<double>::ToSymbolic(*dut_double);
  EXPECT_NE(dut_sym, nullptr);
  // The dut system is not connected to other systems, but that isn't a problem.
  // The error due to scalar type will precede any possible error attributable
  // to unconnected ports.
  auto context = dut_sym->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut_sym->get_lcm_message_output_port().Eval<lcmt_contact_results_for_viz>(
          *context),
      ".+cannot be evaluated when T = symbolic.+");
}

/* There are four overloads of ConnectContactResultsToDrakeVisualizer(). They
 differ along two axes:

   - Does the ContactResultsToLcmSystem instance connect directly to the plant
     or to some arbitrary passed OutputPort?
   - Does ContactResultsToLcmSystem use default geometry names or does it get
     geometry names from a given SceneGraph instance.

 The test fixture is built to facilitate tests structured along those axes. Each
 test comprises four parts:

   1. Construct a diagram (see ConfigureDiagram()).
   2. Invoke a particular overload (found in each TEST_F).
   3. Confirm the returned value is of expected type with documented properties
      (see ExpectValidPublisher()).
   4. Confirm geometry names are as expected (see ExpectGeometryNameSemantics().
 */
class ConnectVisualizerTest : public ::testing::Test {
 protected:
  void AddPlantAndSceneGraphAndOneLink(DiagramBuilder<double>* builder) {
    auto system_pair = AddMultibodyPlantSceneGraph(builder, 0.0);
    plant_ = &system_pair.plant;
    scene_graph_ = &system_pair.scene_graph;

    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    const auto& body = plant_->AddRigidBody("link",
        SpatialInertia<double>::MakeTestCube());
    plant_->RegisterCollisionGeometry(body, {}, Sphere(1.0), kGeoName,
                                      CoulombFriction<double>{});
    plant_->Finalize();
  }

  void ConfigureDiagram(bool is_nested) {
    if (is_nested) {
      /* We'll treat the MBP-SG pair as a nested diagram so we can test all
       overloads of the DUT. This means, exporting the contact port out of the
       diagram. */
      DiagramBuilder<double> inner_builder;
      AddPlantAndSceneGraphAndOneLink(&inner_builder);
      inner_builder.ExportOutput(plant_->get_contact_results_output_port(),
                                 "contact_results");
      inner_builder.ExportOutput(scene_graph_->get_query_output_port(),
                                 "query");
      auto diagram = builder_.AddSystem(inner_builder.Build());
      contact_results_port_ = &diagram->GetOutputPort("contact_results");
      query_object_port_ = &diagram->GetOutputPort("query");
    } else {
      AddPlantAndSceneGraphAndOneLink(&builder_);
    }
  }

  /* Confirms that the publisher pointer is non-null and has been configured
   with the given publication period.

   When no publication period is given, we check that the default value from the
   cc file made it through.  We don't specifically care that this is 64 Hz, just
   that it's some sensible default.  If the cc file changes, we should update
   the value here as well.
  */
  void ExpectValidPublisher(systems::lcm::LcmPublisherSystem* publisher,
                            double expected_publish_period = 1.0 / 64) {
    /* Confirm that we get a non-null result. */
    ASSERT_NE(publisher, nullptr);

    /* Check that the publishing event was set as documented. */
    EXPECT_EQ(publisher->get_publish_period(), expected_publish_period);
  }

  /* Confirms that the names for geometries stored in the
   ContactResultsToLcmSystem are as expepcted (default or scene_graph as
   indicated). */
  void ExpectGeometryNameSemantics(bool expect_default_names) {
    /* Grab the contact results to lcm system and confirm the namer has
     correctly handled. The name should either be the tester constant (kGeoName)
     or Id(\d+). */
    ContactResultsToLcmSystem<double>* contact = nullptr;
    for (auto* system : builder_.GetMutableSystems()) {
      contact = dynamic_cast<ContactResultsToLcmSystem<double>*>(system);
      if (contact != nullptr) break;
    }
    DRAKE_DEMAND(contact != nullptr);
    auto diagram = builder_.Build();
    auto diagram_context = diagram->CreateDefaultContext();
    const auto& contact_context =
        contact->GetMyContextFromRoot(*diagram_context);
    const auto& id_to_body_map =
        ContactResultsToLcmTester::get_geometry_id_to_body_map(contact_context,
                                                               *contact);
    ASSERT_EQ(id_to_body_map.size(), 1);
    const auto& [id, name] = *id_to_body_map.begin();
    if (expect_default_names) {
      EXPECT_EQ(name.geometry, fmt::format("Id({})", id));
    } else {
      EXPECT_EQ(name.geometry, kGeoName);
    }
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  const systems::OutputPort<double>* contact_results_port_{};
  const systems::OutputPort<double>* query_object_port_{};
  static constexpr char kGeoName[] = "test_sphere";
};

TEST_F(ConnectVisualizerTest, ConnectToPlantSceneGraphNames) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher =
      ConnectContactResultsToDrakeVisualizer(&builder_, *plant_, *scene_graph_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

TEST_F(ConnectVisualizerTest, ConnectToPortSceneGraphNames) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *contact_results_port_, *query_object_port_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

TEST_F(ConnectVisualizerTest, ConnectToPlantSceneGraphNamesWithPeriod) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *scene_graph_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

TEST_F(ConnectVisualizerTest, ConnectToPortSceneGraphNamesWithPeriod) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *contact_results_port_, *query_object_port_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
