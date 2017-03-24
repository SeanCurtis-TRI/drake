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
        "frame_kinematics_set.cc",
        "geometry_channel.cc",
        "geometry_state.cc",
    ],
    hdrs = [
        "frame_kinematics_set.h",
        "geometry_channel.h",
        "geometry_engine.h",
        "geometry_ids.h",
        "geometry_instance.h",
        "geometry_state.h",
        "geometry_world.h",
    ],
    deps = [
        ":identifier",
        "//drake/common",
        "//drake/multibody/multibody_tree/math:spatial_velocity",
        "//drake/systems/framework:context",
    ],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

cpplint()
