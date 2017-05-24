# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library", "drake_cc_binary")

drake_cc_library(
    name = "bouncing_ball_vector",
    srcs = ["gen/bouncing_ball_vector.cc"],
    hdrs = ["gen/bouncing_ball_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "bouncing_ball_plant",
    srcs = ["bouncing_ball_plant.cc"],
    hdrs = ["bouncing_ball_plant.h"],
    deps = [
        ":bouncing_ball_vector",
        "//drake/geometry:geometry_ids",
        "//drake/geometry:geometry_system",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "bouncing_ball_run_dynamics",
    testonly = 1,
    srcs = ["bouncing_ball_run_dynamics.cc"],
    add_test_rule = 1,
    deps = [
        ":bouncing_ball_plant",
        "//drake/common:call_matlab",
        "//drake/geometry:geometry_system",
        "//drake/geometry:geometry_visualization",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
    ],
)

# === test/ ===

#drake_cc_googletest(
#    name = "bouncing_ball_plant_test",
#    deps = [
#        ":bouncing_ball_plant",
#        "//drake/common:autodiff",
#    ],
#)

cpplint()
