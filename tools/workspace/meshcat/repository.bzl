load("//tools/workspace:github.bzl", "github_archive")

# Patches dist/main.min.js to add a traverse() fallback in set_property_chain
# so that glTF-loaded meshes (where <object> is a THREE.Group) can have deep
# properties like material.map.offset set via set_property.  The file is a
# single minified line so a unified-diff patch is impractical; the script is
# base64-encoded to avoid shell quoting hazards.
_DIST_JS_PATCH_CMD = (
    "python3 -c \"import base64; exec(base64.b64decode(b'" +
    "cGF0aCA9ICdkaXN0L21haW4ubWluLmpzJwp0eHQgPSBvcGVuKHBhdGgpLnJlYWQoKQpv" +
    "bGQgPSAnaWYobnVsbCE9PWl8fEIgaW4gSXx8KGk9YFwnJHtRfVwnIGhhcyBubyBwcm9w" +
    "ZXJ0eSBcJyR7Qn1cJ2ApLG51bGw9PWkpSVtCXT10O2Vsc2V7Y29uc3QgZz0iLyIrZS5q" +
    "b2luKCIvIiksQj1KU09OLnN0cmluZ2lmeSh0KTtjb25zb2xlLmVycm9yKGBFcnJvciBp" +
    "biBzZXRfcHJvcGVydHkoIiR7Z30iLCAiJHtBfSIsICR7Qn0pXFxuJHtpfS4gVGhlIHZh" +
    "bHVlIHdpbGwgbm90IGJlIHNldC5gKX19JwpuZXcgPSAnaWYobnVsbCE9PWl8fEIgaW4g" +
    "SXx8KGk9YFwnJHtRfVwnIGhhcyBubyBwcm9wZXJ0eSBcJyR7Qn1cJ2ApLG51bGw9PWkp" +
    "SVtCXT10O2Vsc2V7bGV0IGFwcGxpZWQ9ZmFsc2U7dGhpcy5vYmplY3QudHJhdmVyc2Uo" +
    "KGMpPT57aWYoYz09PXRoaXMub2JqZWN0KXJldHVybjtsZXQgY3VyPWMsb2s9dHJ1ZTtm" +
    "b3IoY29uc3QgcCBvZiBnKXtpZihwIGluIGN1ciYmIm9iamVjdCI9PXR5cGVvZiBjdXJb" +
    "cF0pY3VyPWN1cltwXTtlbHNle29rPWZhbHNlO2JyZWFrfX1pZihvayYmQiBpbiBjdXIp" +
    "e2N1cltCXT10O2FwcGxpZWQ9dHJ1ZX19KTtpZighYXBwbGllZCl7Y29uc3QgZj0iLyIr" +
    "ZS5qb2luKCIvIiksdj1KU09OLnN0cmluZ2lmeSh0KTtjb25zb2xlLmVycm9yKGBFcnJv" +
    "ciBpbiBzZXRfcHJvcGVydHkoIiR7Zn0iLCAiJHtBfSIsICR7dn0pXFxuJHtpfS4gVGhl" +
    "IHZhbHVlIHdpbGwgbm90IGJlIHNldC5gKX19fScKYXNzZXJ0IG9sZCBpbiB0eHQsICdQ" +
    "YXR0ZXJuIG5vdCBmb3VuZCBpbiBkaXN0L21haW4ubWluLmpzJwpvcGVuKHBhdGgsICd3" +
    "Jykud3JpdGUodHh0LnJlcGxhY2Uob2xkLCBuZXcsIDEpKQo=" +
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
