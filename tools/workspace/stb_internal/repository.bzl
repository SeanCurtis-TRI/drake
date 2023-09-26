load("@drake//tools/workspace:github.bzl", "github_archive")

def stb_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nothings/stb",
        commit = "5736b15f7ea0ffb08dd38af21067c314d6a3aae9",
        sha256 = "d00921d49b06af62aa6bfb97c1b136bec661dd11dd4eecbcb0da1f6da7cedb4c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
