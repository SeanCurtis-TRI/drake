#include "drake/geometry/profiling/contact_result_maker.h"

#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"

namespace drake {
namespace geometry {
namespace profiling {

using Eigen::Vector3d;
using systems::EventStatus;

ContactResultMaker::ContactResultMaker(ContactQueryType query_type)
    : ContactResultMaker(-1, query_type) {}

ContactResultMaker::ContactResultMaker(double collision_period,
                                       ContactQueryType query_type)
    : query_type_(query_type) {
  geometry_query_input_port_ = &this->DeclareAbstractInputPort(
      "query_object", Value<QueryObject<double>>());
  if (collision_period > 0.0) {
    contact_result_output_port_ = &this->DeclareAbstractOutputPort(
        "contact_result", &ContactResultMaker::CalcContactResultsDummy);
    DeclarePeriodicPublishEvent(collision_period, 0.0,
                                &ContactResultMaker::QueryInPublish);
  } else {
    contact_result_output_port_ = &this->DeclareAbstractOutputPort(
        "contact_result", &ContactResultMaker::CalcContactResults);
  }
}

void ContactResultMaker::CalcContactResults(
    const systems::Context<double>& context,
    lcmt_contact_results_for_viz* results) const {
  const auto& query_object =
      get_geometry_query_port().Eval<QueryObject<double>>(context);
  auto& msg = *results;
  msg.timestamp = context.get_time() * 1e6;  // express in microseconds.

  auto write_double3 = [](const Vector3d& src, double* dest) {
    dest[0] = src(0);
    dest[1] = src(1);
    dest[2] = src(2);
  };

  if (query_type_ == kContactSurfaces) {
    std::vector<ContactSurface<double>> contacts =
        query_object.ComputeContactSurfaces();
    const int num_contacts = static_cast<int>(contacts.size());

    msg.num_point_pair_contacts = 0;
    msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
    msg.num_hydroelastic_contacts = num_contacts;
    msg.hydroelastic_contacts.resize(num_contacts);

    ++hydro_stats_.evaluations;
    hydro_stats_.total_contacts += num_contacts;

    for (int i = 0; i < num_contacts; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
          msg.hydroelastic_contacts[i];

      surface_msg.body1_name = "Id_" + to_string(contacts[i].id_M());
      surface_msg.body2_name = "Id_" + to_string(contacts[i].id_N());

      const SurfaceMesh<double>& mesh_W = contacts[i].mesh_W();
      surface_msg.num_triangles = mesh_W.num_faces();
      surface_msg.triangles.resize(surface_msg.num_triangles);

      // Loop through each contact triangle on the contact surface.
      for (SurfaceFaceIndex j(0); j < surface_msg.num_triangles; ++j) {
        lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
            surface_msg.triangles[j];

        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const SurfaceVertex<double>& vA = mesh_W.vertex(face.vertex(0));
        const SurfaceVertex<double>& vB = mesh_W.vertex(face.vertex(1));
        const SurfaceVertex<double>& vC = mesh_W.vertex(face.vertex(2));

        write_double3(vA.r_MV(), tri_msg.p_WA);
        write_double3(vB.r_MV(), tri_msg.p_WB);
        write_double3(vC.r_MV(), tri_msg.p_WC);

        tri_msg.pressure_A = contacts[i].EvaluateE_MN(face.vertex(0));
        tri_msg.pressure_B = contacts[i].EvaluateE_MN(face.vertex(1));
        tri_msg.pressure_C = contacts[i].EvaluateE_MN(face.vertex(2));
      }
    }
  } else if (query_type_ == kPointContact) {
    std::vector<PenetrationAsPointPair<double>> contacts =
        query_object.ComputePointPairPenetration();
    const int num_contacts = static_cast<int>(contacts.size());
    msg.num_point_pair_contacts = num_contacts;
    msg.point_pair_contact_info.resize(num_contacts);
    msg.num_hydroelastic_contacts = 0;
    msg.hydroelastic_contacts.resize(0);

    point_pair_stats_.total_contacts += num_contacts;
    ++point_pair_stats_.evaluations;

    for (int i = 0; i < num_contacts; ++i) {
      lcmt_point_pair_contact_info_for_viz& pair_msg =
          msg.point_pair_contact_info[i];
      const PenetrationAsPointPair<double>& contact = contacts[i];
      pair_msg.body1_name = "Id_" + to_string(contact.id_A);
      pair_msg.body2_name = "Id_" + to_string(contact.id_B);
      const Vector3d p_WC = (contact.p_WCa + contact.p_WCb) * 0.5;
      write_double3(p_WC, pair_msg.contact_point);
      // Note: This force is fake; we're just reporting a force of unit
      // magnitude in the normal direction. We have to report *something*.
      write_double3(contact.nhat_BA_W, pair_msg.contact_force);
      write_double3(contact.nhat_BA_W, pair_msg.normal);
    }
  }
}

void ContactResultMaker::CalcContactResultsDummy(
    const systems::Context<double>& context,
    lcmt_contact_results_for_viz* results) const {
  auto& msg = *results;
  msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
  msg.num_point_pair_contacts = 0;
  msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
  msg.num_hydroelastic_contacts = 0;
  msg.hydroelastic_contacts.resize(msg.num_hydroelastic_contacts);
}

EventStatus ContactResultMaker::QueryInPublish(
    const systems::Context<double>& context) const {
  const auto& query_object =
      get_geometry_query_port().Eval<QueryObject<double>>(context);
  if (query_type_ == kContactSurfaces) {
    std::vector<ContactSurface<double>> contacts =
        query_object.ComputeContactSurfaces();
    hydro_stats_.total_contacts += static_cast<int>(contacts.size());
    ++hydro_stats_.evaluations;
  } else if (query_type_ == kPointContact) {
    std::vector<PenetrationAsPointPair<double>> contacts =
        query_object.ComputePointPairPenetration();
    point_pair_stats_.total_contacts += static_cast<int>(contacts.size());
    ++point_pair_stats_.evaluations;
  } else {
    throw std::logic_error("Unsupported contact mode");
  }
  return EventStatus::Succeeded();
}

}  // namespace profiling
}  // namespace geometry
}  // namespace drake
