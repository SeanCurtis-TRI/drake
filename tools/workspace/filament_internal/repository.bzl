load("@drake//tools/workspace:github.bzl", "github_archive")

def filament_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/filament",
        commit = "v1.69.3",
        sha256 = "a65696e7acc9f5c272aae5a730853febef63f328a863021fd3bb2caf2594fa5b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
