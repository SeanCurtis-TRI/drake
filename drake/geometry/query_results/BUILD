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
    name = "penetration_as_point_pair",
    srcs = [],
    hdrs = ["penetration_as_point_pair.h"],
    deps = [
        "//drake/common",
        "//drake/geometry:geometry_ids",
    ],
)

drake_cc_library(
    name = "nearest_pair",
    srcs = [],
    hdrs = ["nearest_pair.h"],
    deps = [
        "//drake/common",
        "//drake/geometry:geometry_ids",
    ],
)

drake_cc_library(
    name = "point_proximity",
    srcs = [],
    hdrs = ["point_proximity.h"],
    deps = [
        "//drake/common",
        "//drake/geometry:geometry_ids",
    ],
)

cpplint()
