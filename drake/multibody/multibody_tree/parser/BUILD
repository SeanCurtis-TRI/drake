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
    name = "multibody_tree_parser",
    srcs = [
        "model_instance_id_table.cc",
        "parser.cc",
    ],
    hdrs = [
        "model_instance_id_table.h",
        "parser.h",
    ],
    deps = [
        "//drake/multibody/multibody_tree",
        "@sdformat",
    ],
)

add_lint_tests()
