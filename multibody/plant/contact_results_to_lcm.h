#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace multibody {
namespace internal {

/* Stores the "full" name of a body *in contact*: its model instance name, body
 name, and geometry name. This assumes that `model` name is guaranteed to be
 unique within MBP. So, for two bodies which may be identically named (body
 name), they *must* differ by model name. For a body that has multiple collision
 geometries, we rely on the fact that every collision geometry for a single
 frame must be uniquely named.

 This includes further information to allow visualizers to make streamlining
 decisions. It reports if the body name is unique across the entire plant
 (body_name_is_unique) and the number of collision geometries associated
 with the body.

 If the body name is unique, the visualizer can display only the body
 name without fear of introducing ambiguity. Furthermore, with the number
 collision geometries per body, the visualizer can draw conclusions about
 the possible number of contact patches between bodies and streamline
 accordingly. */
struct FullBodyName {
  std::string model;
  std::string body;
  std::string geometry;
  bool body_name_is_unique;
  int geometry_count;
};

/* A table of per-GeometryId FullBodyName entries and the GeometryVersion the
 table is associated with. */
struct GeometryNameTable {
  geometry::GeometryVersion version;
  std::unordered_map<geometry::GeometryId, FullBodyName> names;
};

/* Facilitate unit testing. See ContactResultsToLcmSystem::Equals(). */
bool operator==(const FullBodyName& n1, const FullBodyName& n2);

}  // namespace internal

/** A System that encodes ContactResults into a lcmt_contact_results_for_viz
 message. It has a single input port with type ContactResults<T> and a single
 output port with lcmt_contact_results_for_viz.

 Although this class can be instantiated on all default scalars, its
 functionality will be limited for `T` = symbolic::Expression. If there are any
 symbolic::Variable instances in the expression, attempting to evaluate the
 output port will throw an exception. The support is sufficient that a
 systems::Diagram with a %ContactResultsToLcmSystem can be scalar converted to
 symbolic::Expression without error, but not necessarily evaluated.

 <h3>Constructing instances</h3>

 %ContactResultsToLcmSystem is meaningless unless included in a diagram with
 appropriate MultibodyPlant and SceneGraph instances which provide the full
 contact data. As such, the only mechanism for instantiating an instance of
 *this* system is via using one of the overloaded
 @ref contact_result_vis_creation "ConnectContactResultsToDrakeVisualizer()"
 functions to add contact visualization to a diagram.

 <h3>How contacts are described in visualization</h3>

 In the visualizer, each contact between two bodies is uniquely characterized
 by two triples of names: (model instance name, body name, geometry name).
 These triples help distinguish contacts which might otherwise be ambiguous
 (e.g., contact with two bodies, both called "box" but part of different model
 instances).

 %ContactResultsToLcmSystem gets the model instance and body names from an
 instance of MultibodyPlant, but *geometry* names are not available from the
 plant. By default, %ContactResultsToLcmSystem will *generate* a unique name
 based on a geometry's unique id (e.g., "Id(7)"). For many applications
 (those cases where each body has only a single collision geometry), this is
 perfectly acceptable. However, in cases where a body has multiple collision
 geometries, those default names may not be helpful when viewing the visualized
 results. Instead, %ContactResultsToLcmSystem can use the names associated with
 the id in a geometry::SceneGraph instance. The only method for doing this is
 via the @ref contact_result_vis_creation
 "ConnectContactResultsToDrakeVisualizer()" functions and requires the diagram
 to be instantiated as double valued. If a diagram with a different scalar
 type is required, it should subsequently be scalar converted.

 <h3>Scalar support</h3>

 As noted before, instances of this system can only be created via a call to
 @ref contact_result_vis_creation "ConnectContactResultsToDrakeVisualizer()".
 Those functions only produce `double`-valued instances. A diagram containing
 %ContactResultsToLcmSystem can be scalar converted to both AutoDiffXd and
 symbolic::Expression. The output port of the AutoDiffXd-valued instance can
 be successfully evaluated. Attempting to do so for the
 symbolic::Expression-valued instance will throw.

 @system
 name: ContactResultsToLcmSystem
 input_ports:
 - u0
 - query_object
 output_ports:
 - y0
 @endsystem

 @tparam_default_scalar
 @ingroup visualization */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** Constructs an instance with *default* geometry names (e.g., "Id(7)").

   @param plant   The MultibodyPlant that the ContactResults are generated from.
   @pre The `plant` parameter (or a fully equivalent plant) connects to `this`
        system's input port.
   @pre The `plant` parameter is finalized. */
  DRAKE_DEPRECATED("2022-10-01",
                   "Please use ConnectContactResultsToDrakeVisualizer().")
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>& plant);

  /** Scalar-converting copy constructor. */
  template <typename U>
  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>& other)
      : ContactResultsToLcmSystem<T>() {}

  const systems::InputPort<T>& get_contact_result_input_port() const;
  const systems::InputPort<T>& get_query_object_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  /* We don't want people arbitrarily constructing this instance; they should
   use the ConnectContactResultsToDrakeVisualizer() functions instead. */
  ContactResultsToLcmSystem();

  friend class ContactResultsToLcmTester;

  // The connection function gets friend access so it can call the default
  // constructor.
  friend systems::lcm::LcmPublisherSystem*
  ConnectContactResultsToDrakeVisualizer(systems::DiagramBuilder<double>*,
                                         const systems::OutputPort<double>&,
                                         const systems::OutputPort<double>&,
                                         lcm::DrakeLcmInterface*,
                                         const std::optional<double>);

  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  // Computes the full geometry name table based on the current geometry
  // configuration as defined in the given `context`.
  void CalcHydroGeometryNames(const systems::Context<T>& context,
                              internal::GeometryNameTable* name_table) const;

  const std::unordered_map<geometry::GeometryId, internal::FullBodyName>&
  EvalHydroGeometryNames(const systems::Context<T>& context) const;

  // Caches the names of each MultibodyPlant body. The vector encodes an
  // implicit mapping between body i and its name; the ith entry in vector is
  // the name of body i.
  void CalcPointBodyNames(const systems::Context<T>& context,
                          std::vector<std::string>* body_names) const;

  const std::vector<std::string>& EvalPointBodyNames(
      const systems::Context<T>& context) const;

  // Named indices for the i/o ports.
  systems::InputPortIndex contact_result_input_port_index_;
  systems::InputPortIndex query_object_input_port_index_;
  systems::OutputPortIndex message_output_port_index_;

  /* ContactResultsToLcmSystem stores a "model" of the set of proximity
   geometries appropriate for the current configuration of MultibodyPlant and
   SceneGraph. Currently, MultibodyPlant's configuration cannot change after
   finalization but SceneGraph does allow for modification of the geometries
   associated with a body. We need to detect changes to SceneGraph's proximity
   version and update the model accordingly.

   Ideally, there would be an input port that is GeometryVersion valued.
   SceneGraph would provide it. Every time GeometryVersion changes, we'd get
   a signal. The cache entry would simply depend on that input signal. So, when
   the version changes, we'd automatically recompute the table.

   We don't currently have that. (And won't for the forseeable future.) So,
   we're going to simulate the effect. And as along as we're doing that, we can
   tailor it a bit more tightly. GeometryVersion can change in perception,
   illustration, or proximity ways. We only care about proximity.

   We create a cache entry that contains the table of data per geometry and
   the version from which the table was created. When we process a contact
   result, we need to assess whether or not the version of SceneGraph's data
   has changed. Connected to a QueryObject, we can easily query the current
   version. If it's different from our current value (retrieved by evaluating
   the cache entry), we mark the entry as dirty and then re-evaluate it.

   This approach is parallelizaable, because the table is always associated in
   the context with the SceneGraph version. It's parallelizable. */

  // TODO(SeanCurtis-TRI): There is some incoherence in how body names are
  //  stored based on contact type (point vs hydro). Hydro uses a cache entry
  //  mapping geometry id to FullBodyName (with associated version/mutex
  //  overhead). Point makes exclusive use of body_names_. They should be
  //  reconciled.

  /* The index of the cache entry that stores the name data for fully qualified
   body/geometry names used by hydroelastic contact results. */
  systems::CacheIndex hydro_name_data_cache_index_{};

  /* The index of the cache entry that stores the names of all MultibodyPlant
   body names used by point contact results. */
  systems::CacheIndex point_body_name_cache_index_{};
};

/** @name Visualizing contact results
 @anchor contact_result_vis_creation

 These functions extend a Diagram with the required components to publish
 contact results (as reported by MultibodyPlant) to a visualizer (either meldis
 or drake_visualizer). We recommend using these functions instead of assembling
 the requisite components by hand.

 These must be called _during_ Diagram building. Each function makes
 modifications to the diagram being constructed by `builder` including the
 following changes:

 - adds systems multibody::ContactResultsToLcmSystem and LcmPublisherSystem to
   the Diagram and connects the draw message output to the publisher input,
 - connects a ContactResults<double>-valued output port to the
   ContactResultsToLcmSystem system,
 - connects a QueryObject<double>-valued output port to the
   ContactResultsToLcmSystem system, and
 - sets the publishing rate based on publish_period.

 The two overloads differ in the following way:

  - One overload takes a pair of OutputPort instances and one does not. This
    determines what is connected to the ContactResultsToLcmSystem input ports.
    The overload that specifies two OutputPort instances will attempt to connect
    those ports. The one that doesn't will connect the given plant's contact
    results output port and scene_graph's query object output port to the
    corresponding input ports on the ContactResultsToLcmSystem system.

 The parameters have the following semantics:

 @param builder                The diagram builder being used to construct the
                               Diagram. Systems will be added to this builder.
 @param plant                  (System overload) The System in `builder`
                               containing the plant whose contact results are to
                               be visualized.
 @param scene_graph            (System overload) The SceneGraph that will
                               determine how the geometry names will appear in
                               the lcm message.
 @param contact_results_port   The optional port that will be connected to the
                               ContactResultsToLcmSystem (as documented above).
 @param contact_results_port   The optional port that will be connected to the
                               ContactResultsToLcmSystem (as documented above).
 @param publish_period         An optional period to pass along to the
                               LcmPublisherSystem constructor; when null, a
                               reasonable default period will be used.
 @param lcm                    An optional lcm interface through which lcm
                               messages will be dispatched. Will be allocated
                               internally if none is supplied. If one is given,
                               it must remain alive at least as long as the
                               diagram built from `builder`.

 @returns (for all overloads) the LcmPublisherSystem (in case callers, e.g.,
          need to change the default publishing rate).

 @pre `plant` is contained within the supplied `builder`.
 @pre `scene_graph` is contained with the supplied `builder`.
 @pre `contact_results_port` and `query_object_port` (if given) belong to
      systems that are immediate children of `builder`. */
//@{

/** System-connecting overload.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

/** OutputPort-connecting overload.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& contact_results_port,
    const systems::OutputPort<double>& query_object_port,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

//@}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
