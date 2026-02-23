load("@drake//tools/workspace:github.bzl", "github_archive")

def filament_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/filament",
        commit = "v1.69.3",
        sha256 = "8b5e1a8d68bb4bc84e076cdeaf0e07a4e2cb4f28b97a825ea9b5036b1088dd9f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
