// A simple utility function for evaluating signed distance.
#include <memory>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"

DEFINE_double(scale, 1.0, "Scale of the obj file");
DEFINE_bool(alt_poses, false, "Uses the alternate set of poses");

namespace drake {
namespace geometry {
namespace probing {
namespace {

using math::RigidTransformd;
using std::make_unique;

int do_main() {
  const ProximityProperties props;

  fmt::print("Using first set of poses: {}\n", !FLAGS_alt_poses);
  fmt::print("Using scale: {}\n", FLAGS_scale);
  fmt::print("\n");
  constexpr const char* line_format = "{:>15} {:>8}   {:<20}\n";
  fmt::print(line_format, "Mesh", "Convex", "Distance (m)");
  for (int obj_index = 0; obj_index <= 15; ++obj_index) {
    for (bool is_convex : {true, false}) {
      SceneGraph<double> sg;
      const SourceId s_id = sg.RegisterSource("tester");

      // Register the geometries. We'll register them such that X_FG = I so we
      // can simply pose the *frames* (X_WF) to determine X_WG.

      const FrameId f1_id = sg.RegisterFrame(s_id, GeometryFrame("frame1"));
      // This is the sphere listed in #18704.
      const GeometryId g1_id = sg.RegisterGeometry(
          s_id, f1_id,
          make_unique<GeometryInstance>(
              RigidTransformd(), make_unique<Sphere>(0.01200000000000000025),
              "shape1"));
      sg.AssignRole(s_id, g1_id, props);

      const FrameId f2_id = sg.RegisterFrame(s_id, GeometryFrame("frame2"));
      GeometryId g2_id;
      const std::string file_name = FindResourceOrThrow(fmt::format(
          "drake/examples/scene_graph/meshes/mesh_col_{}.obj", obj_index));
      if (is_convex) {
        g2_id = sg.RegisterGeometry(
            s_id, f2_id,
            make_unique<GeometryInstance>(
                RigidTransformd(), make_unique<Convex>(file_name, FLAGS_scale),
                "shape2"));
      } else {
        g2_id = sg.RegisterGeometry(
            s_id, f2_id,
            make_unique<GeometryInstance>(
                RigidTransformd(), make_unique<Mesh>(file_name, FLAGS_scale),
                "shape2"));
      }
      sg.AssignRole(s_id, g2_id, props);

      // Allocate context.
      auto context = sg.CreateDefaultContext();

      // Fix poses.
      Eigen::Matrix<double, 3, 4> f1_pose;
      Eigen::Matrix<double, 3, 4> f2_pose;
      // clang-format off
      if (FLAGS_alt_poses) {
        f1_pose <<
            -0.44891714095597207157, 0.86632127192498886714, 0.21899966750245522529, 0.69134078467990878192,
            0.44092752233991133748, 0.0015914144089439888948, 0.89754130124655895351, -0.047286579335901468557,
            0.77721060247471085436, 0.4994846556306300478, -0.38269930518678518805, 0.11697927955152286061;
        f2_pose <<
            0.96713841202357886395, 0.16339883066460536565, -0.19479248991157169235, 0.69861093786591421662
            -0.17593352094622538573, 0.98319336270688029167, -0.048766871302155401224, -0.0076967924383244887276,
            0.18355023344026075161, 0.081434843074549706499, 0.97963139911777497026, 0.11141130288498900247;
      } else {
        f1_pose <<
            -0.40370167831914760548, -0.019479333109567995019, -0.9146832842595900015, 0.74747104350671045303,
            -0.4289595844695910376, 0.88710005183204809764, 0.17043231187557600359, 0.025509175267072643767,
            0.80809568110093199511, 0.46116597188125679763, -0.36647962639802561524, 0.045779118115638992026;
        f2_pose <<
            0.98447767053732548881, 0.11941117744570119152, -0.12862615175162719905, 0.70727978567772809004,
            -0.11946133329759119068, 0.99281162514245557826, 0.0073530149434261633107, 0.019925905554638502493,
            0.12857957092852342895, 0.0081269725622573462526, 0.99166589447091968434, 0.044593263795921309067;
      }
      // clang-format on
      RigidTransformd X_WF1(f1_pose);
      RigidTransformd X_WF2(f2_pose);
      FramePoseVector<double> poses;
      poses.set_value(f1_id, X_WF1);
      poses.set_value(f2_id, X_WF2);

      const auto& frame_port = sg.get_source_pose_port(s_id);
      frame_port.FixValue(context.get(), poses);
      const auto& query_object =
          sg.get_query_output_port().Eval<QueryObject<double>>(*context);
      const auto& result =
          query_object.ComputeSignedDistancePairClosestPoints(g1_id, g2_id);
      (void)result;

      fmt::print(line_format, fmt::format("mesh_col_{}.obj", obj_index),
                 is_convex, result.distance);
    }
  }

  return 0;
}
}  // namespace
}  // namespace probing
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::probing::do_main();
}
