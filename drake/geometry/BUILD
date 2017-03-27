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
    srcs = [
        "geometry_channel.cc",
        "geometry_state.cc",
        "geometry_world.cc",
    ],
    hdrs = [
        "geometry_channel.h",
        "geometry_engine.h",
        "geometry_instance.h",
        "geometry_state.h",
        "geometry_world.h",
    ],
    deps = [
        ":frame_kinematics_set",
        ":geometry_ids",
        "//drake/common",
        "//drake/systems/framework:context",
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
    name = "geometry_ids",
    srcs = [],
    hdrs = ["geometry_ids.h"],
    deps = [":identifier"],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_googletest(
    name = "geometry_state_test",
    deps = [":geometry_world"]
)

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

cpplint()
