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
    name = "geometry_world",
    srcs = ["geometry_world.cc"],
    hdrs = ["geometry_world.h"],
    deps = [
        ":frame_kinematics_set",
        ":geometry_context",
        ":geometry_ids",
        ":geometry_instance",
        ":geometry_state",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "geometry_state",
    srcs = ["geometry_state.cc"],
    hdrs = ["geometry_state.h"],
    deps = [
        ":geometry_engine",
        ":geometry_ids",
        ":geometry_index",
        ":geometry_instance",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "frame_kinematics_set",
    srcs = ["frame_kinematics_set.cc"],
    hdrs = ["frame_kinematics_set.h"],
    deps = [
        ":geometry_ids",
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
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
    ],
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
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_googletest(
    name = "frame_kinematics_set_test",
    deps = [
        ":frame_kinematics_set",
        ":geometry_world",
    ],
)

drake_cc_googletest(
    name = "geometry_world_test",
    deps = [
        ":geometry_instance",
        ":geometry_world",
        "//drake/common",
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
    name = "identifier_test",
    deps = [":identifier"],
)

cpplint()
