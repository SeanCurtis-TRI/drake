load("//tools/workspace:github.bzl", "github_archive")

# Patches dist/main.min.js to add a traverse() fallback in set_property_chain
# so that glTF-loaded meshes (where <object> is a THREE.Group) can have deep
# properties like material.map.offset set via set_property.  The file is a
# single minified line so a unified-diff patch is impractical; the script is
# base64-encoded to avoid shell quoting hazards.
_DIST_JS_PATCH_CMD = (
    "python3 -c \"import base64; exec(base64.b64decode(b'" +
    "cGF0aCA9ICdkaXN0L21haW4ubWluLmpzJwp0eHQgPSBvcGVuKHBhdGgpLnJlYWQoKQpv" +
    "bGQxID0gJyJvYmplY3QiPT10eXBlb2YgSVtBXSl7ST1JW0FdO2NvbnRpbnVlfScKbmV3" +
    "MSA9ICcib2JqZWN0Ij09dHlwZW9mIElbQV0mJm51bGwhPT1JW0FdKXtJPUlbQV07Y29u" +
    "dGludWV9Jwphc3NlcnQgb2xkMSBpbiB0eHQsICdQYXR0ZXJuIG9sZDEgbm90IGZvdW5k" +
    "IGluIGRpc3QvbWFpbi5taW4uanMnCnR4dCA9IHR4dC5yZXBsYWNlKG9sZDEsIG5ldzEs" +
    "IDEpCm9sZDIgPSAnaWYobnVsbCE9PWl8fEIgaW4gSXx8KGk9YFxcXCcke1F9XFxcJyBo" +
    "YXMgbm8gcHJvcGVydHkgXFxcJyR7Qn1cXFwnYCksbnVsbD09aSlJW0JdPXQ7ZWxzZXtj" +
    "b25zdCBnPSIvIitlLmpvaW4oIi8iKSxCPUpTT04uc3RyaW5naWZ5KHQpO2NvbnNvbGUu" +
    "ZXJyb3IoYEVycm9yIGluIHNldF9wcm9wZXJ0eSgiJHtnfSIsICIke0F9IiwgJHtCfSlc" +
    "XFxcbiR7aX0uIFRoZSB2YWx1ZSB3aWxsIG5vdCBiZSBzZXQuYCl9fScKbmV3MiA9ICdp" +
    "ZihudWxsIT09aXx8QiBpbiBJfHwoaT1gXFxcJyR7UX1cXFwnIGhhcyBubyBwcm9wZXJ0" +
    "eSBcXFwnJHtCfVxcXCdgKSxudWxsPT1pKUlbQl09dDtlbHNle2xldCBhcHBsaWVkPWZh" +
    "bHNlO3RoaXMub2JqZWN0LnRyYXZlcnNlKChjKT0+e2lmKGM9PT10aGlzLm9iamVjdCly" +
    "ZXR1cm47bGV0IGN1cj1jLG9rPXRydWU7Zm9yKGNvbnN0IHAgb2YgZyl7aWYocCBpbiBj" +
    "dXImJiJvYmplY3QiPT10eXBlb2YgY3VyW3BdJiZudWxsIT09Y3VyW3BdKWN1cj1jdXJb" +
    "cF07ZWxzZXtvaz1mYWxzZTticmVha319aWYob2smJkIgaW4gY3VyKXtjdXJbQl09dDth" +
    "cHBsaWVkPXRydWV9fSk7aWYoIWFwcGxpZWQpe2NvbnN0IGY9Ii8iK2Uuam9pbigiLyIp" +
    "LHY9SlNPTi5zdHJpbmdpZnkodCk7Y29uc29sZS5lcnJvcihgRXJyb3IgaW4gc2V0X3By" +
    "b3BlcnR5KCIke2Z9IiwgIiR7QX0iLCAke3Z9KVxcXFxuJHtpfS4gVGhlIHZhbHVlIHdp" +
    "bGwgbm90IGJlIHNldC5gKX19fScKYXNzZXJ0IG9sZDIgaW4gdHh0LCAnUGF0dGVybiBv" +
    "bGQyIG5vdCBmb3VuZCBpbiBkaXN0L21haW4ubWluLmpzJwp0eHQgPSB0eHQucmVwbGFj" +
    "ZShvbGQyLCBuZXcyLCAxKQpvcGVuKHBhdGgsICd3Jykud3JpdGUodHh0KQo=" +
    "').decode())\""
)

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "meshcat-dev/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "f7b8588f73aa14bffe52f662fb0ba04f4f1d4f22",
        sha256 = "79dfc209d1c7330fcf6bf323074a8d01cb5627c97d989ccb34b7e8aba165f36e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [":patches/set_property_chain_traverse.patch"],
        patch_args = ["-p0"],
        patch_cmds = [_DIST_JS_PATCH_CMD],
        mirrors = mirrors,
    )
