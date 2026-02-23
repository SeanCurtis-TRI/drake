# Example: Using Filament in a Drake BUILD file

To use Filament in your Drake code with full linking support, add a dependency 
on the appropriate Filament target. For example:

```python
cc_library(
    name = "my_renderer",
    srcs = ["my_renderer.cc"],
    hdrs = ["my_renderer.h"],
    deps = [
        "@filament_internal//:filament",
        "@filament_internal//:gltfio",
        # other dependencies...
    ],
)
```

## Available Targets

### Full Libraries (with linking support)
- `@filament_internal//:filament` - Core Filament rendering engine (includes all dependencies)
- `@filament_internal//:backend` - Rendering backend (OpenGL/Vulkan/Metal)
- `@filament_internal//:math` - Math library (vectors, matrices)
- `@filament_internal//:utils` - Utility library
- `@filament_internal//:filabridge` - Bridge library between Filament components
- `@filament_internal//:filaflat` - Material serialization library
- `@filament_internal//:geometry` - Geometry utilities
- `@filament_internal//:image` - Image processing
- `@filament_internal//:gltfio` - glTF 2.0 loader
- `@filament_internal//:viewer` - Viewer library (high-level glTF viewer)

### Header-Only Target
- `@filament_internal//:filament_headers` - Headers only (for compilation without linking)

## Build Configuration

The Filament libraries are built from source using CMake via a Bazel genrule.
The build is configured with:
- Release mode
- Position Independent Code (PIC) enabled
- OpenGL support enabled
- Vulkan and Metal support disabled (can be enabled if needed)
- Sample apps disabled
- Static linking

## Platform Support

Filament is built for:
- Linux (with OpenGL support)
- macOS (with OpenGL support)
- Windows (experimental)

## Example Usage

```cpp
#include <filament/Engine.h>
#include <filament/View.h>
#include <filament/Scene.h>
#include <filament/Renderer.h>

using namespace filament;

int main() {
    Engine* engine = Engine::create();
    
    // Create rendering resources
    Renderer* renderer = engine->createRenderer();
    View* view = engine->createView();
    Scene* scene = engine->createScene();
    
    // Setup and render...
    
    // Cleanup
    engine->destroy(&renderer);
    engine->destroy(&view);
    engine->destroy(&scene);
    Engine::destroy(&engine);
    
    return 0;
}
```

## Dependencies

When using `@filament_internal//:filament`, you automatically get all required
dependencies. However, you may need system libraries:
- **Linux**: OpenGL, EGL, pthread, dl
- **macOS**: OpenGL framework, Cocoa framework, QuartzCore framework

These are automatically linked via linkopts in the BUILD file.
