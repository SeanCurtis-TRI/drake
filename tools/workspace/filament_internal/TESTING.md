# Testing Filament Integration

This document provides instructions for testing the Filament integration in Drake.

## Prerequisites

Before building Filament, ensure you have:

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install cmake ninja-build clang libglu1-mesa-dev \
    libxi-dev libxcomposite-dev libxxf86vm-dev
```

### macOS
```bash
brew install cmake ninja
```

## Building Filament

The Filament libraries are built automatically when needed as a dependency. However, you can also build them explicitly:

```bash
cd /path/to/drake
bazel build @filament_internal//:filament
```

This will:
1. Download Filament v1.69.3 from GitHub
2. Configure with CMake
3. Build all required libraries
4. Make them available as Bazel targets

**Note:** The first build takes 5-10 minutes depending on your system.

## Testing the Integration

### 1. Verify headers are accessible

Create a simple test file `test_filament_headers.cc`:

```cpp
#include <filament/Engine.h>
#include <filament/View.h>

int main() {
    // This will compile but won't link (no implementation)
    return 0;
}
```

Build with headers only:
```bash
bazel build //your/package:test --config=your_config
```

### 2. Test full linking

Create a complete test `test_filament_link.cc`:

```cpp
#include <filament/Engine.h>
#include <iostream>

int main() {
    filament::Engine* engine = filament::Engine::create();
    if (engine) {
        std::cout << "Filament engine created successfully!" << std::endl;
        filament::Engine::destroy(&engine);
        return 0;
    }
    std::cerr << "Failed to create Filament engine" << std::endl;
    return 1;
}
```

BUILD file:
```python
cc_binary(
    name = "test_filament_link",
    srcs = ["test_filament_link.cc"],
    deps = ["@filament_internal//:filament"],
)
```

Build and run:
```bash
bazel run //your/package:test_filament_link
```

## Troubleshooting

### CMake not found
```
ERROR: ninja build system not found
```
**Solution:** Install CMake and Ninja (see Prerequisites)

### OpenGL libraries not found (Linux)
```
ERROR: cannot find -lGL
```
**Solution:** Install OpenGL development libraries:
```bash
sudo apt-get install libgl1-mesa-dev libegl1-mesa-dev
```

### Build takes too long
The initial build compiles all of Filament, which takes time. Subsequent builds use Bazel's cache and are much faster.

### Linking errors
If you see undefined symbols, ensure you're depending on the correct target:
- Use `@filament_internal//:filament` for the full engine
- Use `@filament_internal//:gltfio` for glTF loading (includes filament)
- Don't mix `_headers` targets with full library targets

## Performance Notes

- The genrule is tagged with `tags = ["manual"]` to avoid unnecessary builds
- Libraries are only built when explicitly needed
- Bazel caches the build outputs across different projects
- Static linking is used for better performance and portability
