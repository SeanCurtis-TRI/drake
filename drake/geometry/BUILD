# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "frame_kinematics_set",
    srcs = ["frame_kinematics_set.cc"],
    hdrs = ["frame_kinematics_set.h"],
    deps = [
        ":geometry_ids",
        ":spatial_pose",
        "//drake/multibody/multibody_tree/math:spatial_velocity",
    ],
)

drake_cc_library(
    name = "geometry_context",
    srcs = ["geometry_context.cc"],
    hdrs = ["geometry_context.h"],
    deps = ["//drake/systems/framework:leaf_context"],
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
        ":geometry_query_results",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
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
        ":shapes",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
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
    name = "geometry_query_results",
    srcs = [],
    hdrs = ["geometry_query_results.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "geometry_state",
    srcs = ["geometry_state.cc"],
    hdrs = ["geometry_state.h"],
    deps = [
        ":frame_kinematics_set",
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
    hdrs = ["geometry_system.h"],
    deps = [
        ":geometry_context",
        ":geometry_instance",
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
        ":frame_kinematics_set",
        ":geometry_context",
        ":geometry_frame",
        ":geometry_ids",
        ":geometry_instance",
        ":geometry_query_inputs",
        ":geometry_query_results",
        ":geometry_state",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "internal_frame",
    srcs = [],
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
    name = "shapes",
    srcs = ["shapes.cc"],
    hdrs = ["shapes.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "spatial_pose",
    srcs = [],
    hdrs = ["spatial_pose.h"],
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
        "@gtest//:main",
        ":geometry_engine",
        ":geometry_state",
        ":internal_geometry",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "frame_kinematics_set_test",
    deps = [
        ":expect_error_message",
        ":frame_kinematics_set",
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
        ":shapes",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "geometry_system_test",
    deps = [
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

drake_cc_googletest(
    name = "spatial_pose_test",
    deps = [
        ":spatial_pose",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
