#pragma once

#include <optional>

#include "drake/common/value.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {
namespace profiling {

/** A system that evaluates contact between geometries in SceneGraph. The system
 can be instantiated in one of two modes: simple and visualization.

   - Simple: the system declares a periodic publish event. When triggered, the
     event evaluates the contact. The output port will always output an empty
     message.
   - Visualization: the system declares an output port of type
     lcmt_contact_results_for_viz. It depends on a downstream lcm publisher
     to pull on this output port to cause contact evaluation. The data in this
     message will be "filled out" to include any data that isn't available
     from only geometric sources (e.g., body names and forces).

In both modes, statistics will be collected about the results of the contact
queries.

The system is configured upon construction to determine the type of contact
query to evaluate: contact surface or point contact.

@system{ContactResultMaker,
  @intput_port{query_object},
  @output_port{contact_result}
}
*/
class ContactResultMaker final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultMaker)

  /** The type of contact evaluated: contact surface or point contact.  */
  enum ContactQueryType {
      kContactSurfaces,
      kPointContact
  };

  /** Constructs a visualization %ContactResultMaker instance with the specified
   contact `query_type`. Defaults to contact surface queries.  */
  explicit ContactResultMaker(ContactQueryType query_type = kContactSurfaces);

  /** Constructs a "simple" (non-visualizing_ %ContactResultsMaker instance with
   the specified contact `query_type`. Defaults to contact surface queries.

   @param collision_period  The time elapsed between evaluations of collision
                            queries. Passing a period value <= 0 is equivalent
                            to calling the visualization construtor.  */
  ContactResultMaker(double collision_period,
                     ContactQueryType query_type = kContactSurfaces);

  const systems::InputPort<double>& get_geometry_query_port() const {
    return *geometry_query_input_port_;
  }

  const systems::OutputPort<double>& get_contact_result_output_port() const {
    return *contact_result_output_port_;
  }

  struct HydroelasticStats {
    int evaluations{0};
    int total_contacts{0};
  };
  struct PointPairStats {
    int evaluations{0};
    int total_contacts{0};
  };

  const HydroelasticStats& hydroelastic_stats() const { return hydro_stats_; }
  const PointPairStats& point_pair_stats() const { return point_pair_stats_; }

 private:
  void CalcContactResults(const systems::Context<double>& context,
                          lcmt_contact_results_for_viz* results) const;
  void CalcContactResultsDummy(const systems::Context<double>& context,
                               lcmt_contact_results_for_viz* reults) const;
  systems::EventStatus QueryInPublish(
      const systems::Context<double>& context) const;

  systems::InputPort<double>* geometry_query_input_port_{};
  systems::OutputPort<double>* contact_result_output_port_{};
  ContactQueryType query_type_{kContactSurfaces};

  mutable HydroelasticStats hydro_stats_;
  mutable PointPairStats point_pair_stats_;
};
}  // namespace profiling
}  // namespace geometry
}  // namespace drake
