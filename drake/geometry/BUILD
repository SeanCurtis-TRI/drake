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
    srcs = [
        "frame_kinematics_set.cc",
    ],
    hdrs = [
        "frame_kinematics_set.h",
    ],
    deps = [
        ":geometry_ids",
        "//drake/common",
        "//drake/multibody/multibody_tree/math:spatial_velocity",
    ],
)

drake_cc_library(
    name = "geometry_channel",
    srcs = ["geometry_channel.cc"],
    hdrs = ["geometry_channel.h"],
    deps = [
        ":frame_kinematics_set",
        ":geometry_ids",
        ":geometry_instance",
        ":geometry_world",
        "//drake/common",
        "//drake/systems/framework:context",
    ],
)

drake_cc_library(
    name = "geometry_ids",
    srcs = [],
    hdrs = ["geometry_ids.h"],
    deps = [
        ":type_safe_int_id",
    ],
)

drake_cc_library(
    name = "geometry_instance",
    srcs = [],
    hdrs = ["geometry_instance.h"],
    deps = [],
)

drake_cc_library(
    name = "geometry_world",
    srcs = [
    ],
    hdrs = [
        "geometry_engine.h",
        "geometry_world.h",
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
