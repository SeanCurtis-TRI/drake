# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "frame_kinematics",
    srcs = ["frame_id_vector.cc"],
    hdrs = [
        "frame_id_vector.h",
        "frame_kinematics_vector.h",
    ],
    deps = [
        ":geometry_ids",
        "//drake/common",
        "//drake/multibody/multibody_tree/math:spatial_velocity",
    ],
)

drake_cc_library(
    name = "geometry_context",
    srcs = ["geometry_context.cc"],
    hdrs = ["geometry_context.h"],
    deps = [
        ":geometry_state",
        "//drake/systems/framework:leaf_context",
    ],
)

drake_cc_library(
    name = "geometry_engine",
    srcs = ["geometry_engine_stub.cc"],
    hdrs = [
        "geometry_engine.h",
        "geometry_engine_stub.h",
    ],
    deps = [
        ":geometry_ids",
        ":geometry_index",
        ":geometry_instance",
        ":geometry_query_inputs",
        ":shape_specification",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
        "//drake/geometry/query_results:nearest_pair",
        "//drake/geometry/query_results:penetration_as_point_pair",
        "//drake/geometry/query_results:point_proximity",
    ],
)

drake_cc_library(
    name = "geometry_frame",
    srcs = [],
    hdrs = ["geometry_frame.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "geometry_ids",
    srcs = [],
    hdrs = ["geometry_ids.h"],
    deps = [":identifier"],
)

drake_cc_library(
    name = "geometry_index",
    srcs = [],
    hdrs = ["geometry_index.h"],
    deps = ["//drake/common:type_safe_index"],
)

drake_cc_library(
    name = "geometry_instance",
    srcs = ["geometry_instance.cc"],
    hdrs = ["geometry_instance.h"],
    deps = [
        ":geometry_material",
        ":shape_specification",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
    ],
)

drake_cc_library(
    name = "geometry_material",
    srcs = ["geometry_material.cc"],
    hdrs = ["geometry_material.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "geometry_query_inputs",
    srcs = [],
    hdrs = ["geometry_query_inputs.h"],
    deps = [
        ":geometry_ids",
        ":geometry_index",
    ],
)

drake_cc_library(
    name = "geometry_state",
    srcs = ["geometry_state.cc"],
    hdrs = ["geometry_state.h"],
    deps = [
        ":frame_kinematics",
        ":geometry_engine",
        ":geometry_frame",
        ":geometry_ids",
        ":geometry_index",
        ":geometry_instance",
        ":internal_frame",
        ":internal_geometry",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "geometry_system",
    srcs = ["geometry_system.cc"],
    hdrs = [
        "geometry_system.h",
        "query_handle.h",
    ],
    deps = [
        ":geometry_context",
        ":geometry_state",
        ":geometry_world",
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/rendering:pose_bundle",
    ],
)

drake_cc_library(
    name = "geometry_visualization",
    srcs = ["geometry_visualization.cc"],
    hdrs = ["geometry_visualization.h"],
    deps = [
        ":geometry_state",
        ":geometry_system",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_library(
    name = "geometry_world",
    srcs = ["geometry_world.cc"],
    hdrs = ["geometry_world.h"],
    deps = [
        ":frame_kinematics",
        ":geometry_context",
        ":geometry_frame",
        ":geometry_instance",
        ":geometry_query_inputs",
        ":geometry_state",
        "//drake/common",
        "//drake/geometry/query_results:penetration_as_point_pair",
    ],
)

drake_cc_library(
    name = "internal_frame",
    srcs = ["internal_frame.cc"],
    hdrs = ["internal_frame.h"],
    deps = [
        ":geometry_ids",
        ":geometry_index",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "internal_geometry",
    srcs = [],
    hdrs = ["internal_geometry.h"],
    deps = [
        ":geometry_ids",
        ":geometry_index",
        ":geometry_material",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "shape_specification",
    srcs = [],
    hdrs = ["shape_specification.h"],
    deps = ["//drake/common"],
)

# -----------------------------------------------------

drake_cc_library(
    name = "expect_error_message",
    srcs = [],
    hdrs = ["test/expect_error_message.h"],
)

drake_cc_library(
    name = "query_test_utility",
    testonly = 1,
    srcs = ["test/query_test_utility.cc"],
    hdrs = ["test/query_test_utility.h"],
    deps = [
        ":geometry_context",
        ":geometry_engine",
        ":geometry_state",
        ":geometry_world",
        ":internal_geometry",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:without_main",
    ],
)

drake_cc_googletest(
    name = "frame_id_vector_test",
    deps = [
        ":expect_error_message",
        ":frame_kinematics",
    ],
)

drake_cc_googletest(
    name = "frame_kinematics_vector_test",
    deps = [
        ":frame_kinematics",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "geometry_engine_stub_test",
    deps = [
        ":geometry_engine",
        ":query_test_utility",
    ],
)

drake_cc_googletest(
    name = "geometry_instance_test",
    deps = [
        ":geometry_instance",
        "//drake/common:copyable_unique_ptr",
    ],
)

drake_cc_googletest(
    name = "geometry_state_test",
    deps = [
        ":expect_error_message",
        ":geometry_state",
        ":geometry_world",
        ":shape_specification",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "geometry_system_test",
    deps = [
        ":expect_error_message",
        ":geometry_system",
        ":geometry_visualization",
    ],
)

drake_cc_googletest(
    name = "geometry_world_test",
    deps = [
        ":expect_error_message",
        ":geometry_instance",
        ":geometry_world",
        ":query_test_utility",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

add_lint_tests()
