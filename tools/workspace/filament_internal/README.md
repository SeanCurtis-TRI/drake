# Filament

This directory contains the Bazel infrastructure to integrate Google's Filament
rendering engine as a dependency in Drake.

## Overview

Filament is a real-time physically based rendering (PBR) engine designed for 
Android, iOS, Linux, macOS, Windows, and WebGL2. It provides a modern, 
efficient rendering solution with support for:

- Multiple rendering backends (Vulkan, Metal, OpenGL/ES, WebGL)
- Physically-based materials and lighting
- glTF 2.0 loading
- Image-based lighting
- Advanced post-processing effects

## Integration Notes

Filament uses CMake as its build system. This Bazel integration provides
header-only targets that expose Filament's public API. To use Filament's
actual libraries, you would need to either:

1. Use prebuilt binaries from Filament's releases
2. Integrate via rules_foreign_cc to build with CMake
3. Manually port the CMake build to Bazel

The current integration provides access to headers for:
- `@filament_internal//:filament_headers` - Core Filament API
- `@filament_internal//:math` - Math utilities
- `@filament_internal//:utils_headers` - Utility library headers
- `@filament_internal//:filamat_headers` - Material builder API
- `@filament_internal//:gltfio_headers` - glTF loading API
- `@filament_internal//:geometry_headers` - Geometry utilities
- `@filament_internal//:image_headers` - Image utilities
- `@filament_internal//:viewer_headers` - Viewer library

## Upstream

- Repository: https://github.com/google/filament
- Version: v1.69.3 (Released: February 19, 2026)
- License: Apache-2.0
- Documentation: https://google.github.io/filament/

## Drake Modifications

None. This is a vanilla checkout of the upstream repository with a custom
BUILD file to expose headers.
