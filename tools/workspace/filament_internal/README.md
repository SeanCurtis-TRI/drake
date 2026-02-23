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

Filament uses CMake as its build system. This Bazel integration builds Filament
from source using a genrule that invokes CMake, then creates cc_library targets
that link against the built static libraries.

The integration provides full linking support for:
- `@filament_internal//:filament` - Core rendering engine with all dependencies
- `@filament_internal//:backend` - Rendering backend (OpenGL/Vulkan/Metal)
- `@filament_internal//:math` - Math utilities  
- `@filament_internal//:utils` - Utility library
- `@filament_internal//:filabridge` - Bridge library
- `@filament_internal//:filaflat` - Material serialization
- `@filament_internal//:gltfio` - glTF loading API
- `@filament_internal//:geometry` - Geometry utilities
- `@filament_internal//:image` - Image utilities
- `@filament_internal//:viewer` - High-level viewer library
- `@filament_internal//:filament_headers` - Headers only (no linking)

### Build Configuration

Filament is built with:
- CMake in Release mode
- Position Independent Code (PIC) enabled for Bazel compatibility
- OpenGL backend enabled
- Metal and Vulkan backends disabled (can be enabled if needed)
- Static linking for all libraries
- Sample applications skipped

### System Requirements

To build Filament, your system needs:
- CMake 3.22.1 or newer
- Ninja build system
- C++20 compiler (clang recommended)
- OpenGL development libraries (Linux: libGL, libEGL; macOS: built-in)

On Ubuntu/Debian:
```bash
sudo apt-get install cmake ninja-build clang libglu1-mesa-dev libxi-dev
```

On macOS:
```bash
brew install cmake ninja
```

## Upstream

- Repository: https://github.com/google/filament
- Version: v1.69.3 (Released: February 19, 2026)
- License: Apache-2.0
- Documentation: https://google.github.io/filament/

## Drake Modifications

The package.BUILD.bazel file is a Drake-specific addition that:
1. Builds Filament from source using CMake via genrule
2. Creates Bazel cc_library targets for each Filament component
3. Configures linkopts for platform-specific system libraries (OpenGL, pthread, etc.)
4. Enables static linking and position-independent code for Bazel compatibility

No source code modifications were made to Filament itself.
