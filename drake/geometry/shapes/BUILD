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
    name = "half_space",
    srcs = ["half_space.cc"],
    hdrs = ["half_space.h"],
    deps = [
        ":shape",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "sphere",
    srcs = [],
    hdrs = ["sphere.h"],
    deps = [
        ":shape",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "shape",
    srcs = [],
    hdrs = ["shape.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "all_shapes",
    srcs = [],
    hdrs = ["shapes.h"],
    deps = [
        ":half_space",
        ":sphere",
    ],
)

add_lint_tests()
