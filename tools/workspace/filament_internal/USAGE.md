# Example: Using Filament in a Drake BUILD file

To use Filament headers in your Drake code, add a dependency on the appropriate
Filament target. For example:

```python
cc_library(
    name = "my_renderer",
    srcs = ["my_renderer.cc"],
    hdrs = ["my_renderer.h"],
    deps = [
        "@filament_internal//:filament_headers",
        "@filament_internal//:math",
        # other dependencies...
    ],
)
```

## Available Targets

- `@filament_internal//:filament_headers` - Core Filament rendering API
- `@filament_internal//:math` - Filament math library (vectors, matrices)
- `@filament_internal//:utils_headers` - Utility library
- `@filament_internal//:filamat_headers` - Material builder
- `@filament_internal//:gltfio_headers` - glTF 2.0 loader
- `@filament_internal//:geometry_headers` - Geometry utilities
- `@filament_internal//:image_headers` - Image processing
- `@filament_internal//:viewer_headers` - Viewer library

## Important Note

The current integration provides header-only access to Filament's API. To link
against Filament's actual implementation libraries, additional work is required:

1. **Prebuilt binaries**: Download from https://github.com/google/filament/releases
2. **CMake integration**: Use rules_foreign_cc to build Filament with CMake
3. **Manual Bazel build**: Port Filament's CMake build to Bazel (significant effort)

For most use cases, option #2 (rules_foreign_cc) is recommended.
