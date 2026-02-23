#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Camera.h>
#include <filament/SwapChain.h>
#include <filament/Fence.h>
#include <filament/RenderableManager.h>
#include <filament/Viewport.h>
#include <filament/LightManager.h>
#include <filament/TransformManager.h>
#include <utils/EntityManager.h>
#include <backend/PixelBufferDescriptor.h>
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/TextureProvider.h>
#include <math/vec3.h>
#include <math/mat4.h>

namespace gltfio = filament::gltfio;

// Simple function to save RGBA data as PPM (since we can't use Drake's I/O due to ABI issues)
void SaveImageAsPPM(const std::string& filename, 
                    const uint8_t* rgba_data, uint32_t width, uint32_t height) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }
    
    // Write PPM header (P6 format for binary RGB)
    file << "P6\n" << width << " " << height << "\n255\n";
    
    // Write RGB data (strip alpha channel)
    for (uint32_t i = 0; i < width * height; ++i) {
        file.write(reinterpret_cast<const char*>(&rgba_data[i * 4]), 3);
    }
    
    file.close();
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_gltf_file> <output_image_path>" << std::endl;
        return 1;
    }

    std::filesystem::path gltf_path = argv[1];
    std::filesystem::path output_path = argv[2];
    std::cout << "Loading glTF file: " << gltf_path << std::endl;
    std::cout << "Output will be saved to: " << output_path << std::endl;
    
    // Image dimensions
    const uint32_t width = 1024;
    const uint32_t height = 768;

    // Create Filament engine
    filament::Engine* engine = filament::Engine::create(filament::Engine::Backend::OPENGL);
    if (!engine) {
        std::cerr << "Failed to create Filament engine" << std::endl;
        return 1;
    }
    std::cout << "Created Filament engine" << std::endl;

    // Create renderer, scene, view, and camera
    filament::Renderer* renderer = engine->createRenderer();
    filament::Scene* scene = engine->createScene();
    filament::View* view = engine->createView();
    
    // Create swap chain for offscreen rendering
    filament::SwapChain* swap_chain = engine->createSwapChain(width, height);
    
    // Create camera entity
    utils::Entity camera_entity = utils::EntityManager::get().create();
    filament::Camera* camera = engine->createCamera(camera_entity);
    
    // Configure camera
    camera->setProjection(45.0, double(width) / height, 0.1, 100.0,
                          filament::Camera::Fov::VERTICAL);
    camera->lookAt({5.0, 5.0, 5.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    
    // Configure view
    view->setScene(scene);
    view->setCamera(camera);
    view->setViewport({0, 0, width, height});
    
    // Add lighting to the scene
    utils::Entity light_entity = utils::EntityManager::get().create();
    filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
        .color({1.0f, 1.0f, 1.0f})
        .intensity(100000.0f)
        .direction({0.0f, -1.0f, -1.0f})
        .castShadows(true)
        .build(*engine, light_entity);
    scene->addEntity(light_entity);
    
    std::cout << "Created renderer, scene, view, camera, and lighting" << std::endl;

    // Create material provider for glTF assets
    std::cout << "Creating MaterialProvider..." << std::endl;
    gltfio::MaterialProvider* material_provider = 
        gltfio::createJitShaderProvider(engine, false, {});
    if (!material_provider) {
        std::cerr << "Failed to create material provider" << std::endl;
        engine->destroy(renderer);
        engine->destroy(scene);
        engine->destroy(view);
        engine->destroyCameraComponent(camera_entity);
        utils::EntityManager::get().destroy(camera_entity);
        filament::Engine::destroy(&engine);
        return 1;
    }
    std::cout << "MaterialProvider created" << std::endl;

    // Create glTF asset loader with proper configuration
    std::cout << "Creating AssetLoader..." << std::endl;
    gltfio::AssetConfiguration asset_config;
    asset_config.engine = engine;
    asset_config.materials = material_provider;
    
    gltfio::AssetLoader* asset_loader = gltfio::AssetLoader::create(asset_config);
    if (!asset_loader) {
        std::cerr << "Failed to create glTF asset loader" << std::endl;
        engine->destroy(renderer);
        engine->destroy(scene);
        engine->destroy(view);
        engine->destroyCameraComponent(camera_entity);
        utils::EntityManager::get().destroy(camera_entity);
        filament::Engine::destroy(&engine);
        return 1;
    }
    std::cout << "AssetLoader created successfully" << std::endl;

    // Load the glTF asset - read file into memory first
    std::cout << "Reading glTF file - " << gltf_path << " ..." << std::endl;
    std::ifstream file(gltf_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open glTF file: " << gltf_path << std::endl;
        gltfio::AssetLoader::destroy(&asset_loader);
        engine->destroy(renderer);
        engine->destroy(scene);
        engine->destroy(view);
        engine->destroyCameraComponent(camera_entity);
        utils::EntityManager::get().destroy(camera_entity);
        filament::Engine::destroy(&engine);
        return 1;
    }
    
    std::vector<uint8_t> file_buffer((std::istreambuf_iterator<char>(file)), {});
    file.close();
    std::cout << "File read, buffer size: " << file_buffer.size() << " bytes" << std::endl;
    
    std::cout << "Creating asset from buffer..." << std::endl;
    gltfio::FilamentAsset* asset = asset_loader->createAsset(
        file_buffer.data(), file_buffer.size());
    if (!asset) {
        std::cerr << "Failed to load glTF file: " << gltf_path << std::endl;
        gltfio::AssetLoader::destroy(&asset_loader);
        engine->destroy(renderer);
        engine->destroy(scene);
        engine->destroy(view);
        engine->destroyCameraComponent(camera_entity);
        utils::EntityManager::get().destroy(camera_entity);
        filament::Engine::destroy(&engine);
        return 1;
    }

    std::cout << "Successfully loaded glTF asset" << std::endl;
    std::cout << "  Entities: " << asset->getEntityCount() << std::endl;

    // Create resource loader for textures and buffers
    std::cout << "Creating ResourceLoader..." << std::endl;
    gltfio::ResourceLoader* resource_loader = 
        new gltfio::ResourceLoader({.engine = engine,
                                    .gltfPath = gltf_path.c_str()});
    
    // Create and add texture provider for PNG/JPEG support
    std::cout << "Adding texture providers..." << std::endl;
    gltfio::TextureProvider* stb_provider = gltfio::createStbProvider(engine);
    resource_loader->addTextureProvider("image/png", stb_provider);
    resource_loader->addTextureProvider("image/jpeg", stb_provider);
    gltfio::TextureProvider* ktx2_provider = gltfio::createKtx2Provider(engine);
    resource_loader->addTextureProvider("image/ktx2", ktx2_provider);
    std::cout << "ResourceLoader created with texture providers for PNG, JPEG, and KTX2" << std::endl;
    
    // Load resources (textures, buffers, etc.) - use async loading but wait for completion
    std::cout << "Loading resources asynchronously..." << std::endl;
    resource_loader->asyncBeginLoad(asset);
    
    // Poll until resources are loaded
    auto start_resource_load = std::chrono::steady_clock::now();
    float progress = 0.0f;
    while (progress < 1.0f) {
        resource_loader->asyncUpdateLoad();
        progress = resource_loader->asyncGetLoadProgress();
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_resource_load).count();
        if (elapsed > 30) {
            std::cerr << "Warning: Resource loading timeout after 30 seconds (progress: " 
                      << (progress * 100.0f) << "%)" << std::endl;
            break;
        }
        
        if (progress < 1.0f) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::cout << "Resources loaded (progress: " << (progress * 100.0f) << "%)" << std::endl;
    
    // Check if asset is ready to render
    if (progress < 1.0f) {
        std::cerr << "Warning: Asset resources not fully loaded. Rendering may fail or produce incorrect results." << std::endl;
        std::cerr << "Consider converting textures to a supported format." << std::endl;
    }
    
    // Release asset resources that are no longer needed
    asset->releaseSourceData();

    // Add asset entities to the scene
    std::cout << "Adding entities to scene..." << std::endl;
    const utils::Entity* entities = asset->getEntities();
    size_t entity_count = asset->getEntityCount();
    
    // Check which entities are actually renderable
    auto& renderableManager = engine->getRenderableManager();
    size_t renderable_count = 0;
    for (size_t i = 0; i < entity_count; ++i) {
        if (renderableManager.hasComponent(entities[i])) {
            renderable_count++;
        }
    }
    std::cout << "Found " << renderable_count << " renderable entities out of " 
              << entity_count << " total entities" << std::endl;
    
    if (renderable_count == 0) {
        std::cerr << "ERROR: No renderable entities found in asset. Cannot render." << std::endl;
        // Continue with cleanup
    } else {
        scene->addEntities(entities, entity_count);
        scene->addEntities(entities, entity_count);
    
        std::cout << "Added asset to scene" << std::endl;
        
        // Render the scene multiple times to ensure everything is initialized
        std::cout << "Rendering scene..." << std::endl;
        
        // Render a few frames to let things settle
        for (int frame = 0; frame < 3; ++frame) {
            if (renderer->beginFrame(swap_chain)) {
                renderer->render(view);
                renderer->endFrame();
            }
            engine->flushAndWait();
        }
        
        // Ensure rendering is complete
        engine->flushAndWait();
        
        // Read back the rendered image
        std::cout << "Reading back rendered image..." << std::endl;
        std::vector<uint8_t> pixel_data(width * height * 4);
        
        filament::backend::PixelBufferDescriptor pixel_buffer(
            pixel_data.data(),
            pixel_data.size(),
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE);
        
        if (renderer->beginFrame(swap_chain)) {
            // TODO: This should be done immediately on the heels of render; no
            // need to call beginFrame/endFrame again.
            renderer->readPixels(0, 0, width, height, std::move(pixel_buffer));
            renderer->endFrame();
        } else {
            std::cout << "Unable to read pixels - renderer could not begin frame" << std::endl;
        }
        
        
        // Wait for readPixels to complete
        engine->flushAndWait();
        
        // Save the image
        std::cout << "Saving image to: " << output_path << std::endl;
        
        // Flip image vertically (OpenGL origin is bottom-left)
        std::vector<uint8_t> flipped_data(width * height * 4);
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                uint32_t src_idx = ((height - 1 - y) * width + x) * 4;
                uint32_t dst_idx = (y * width + x) * 4;
                flipped_data[dst_idx + 0] = pixel_data[src_idx + 0];
                flipped_data[dst_idx + 1] = pixel_data[src_idx + 1];
                flipped_data[dst_idx + 2] = pixel_data[src_idx + 2];
                flipped_data[dst_idx + 3] = pixel_data[src_idx + 3];
            }
        }
        
        // Save as PPM format (simple format, no library needed)
        SaveImageAsPPM(output_path, flipped_data.data(), width, height);
        std::cout << "Image saved successfully!" << std::endl;
        std::cout << "Note: Image saved as PPM format. Use .ppm extension or convert with imagemagick." << std::endl;
    }
    
    std::cout << "glTF test completed successfully!" << std::endl;

    // Cleanup - order matters!
    std::cout << "Starting cleanup..." << std::endl;
    
    // Remove light from scene
    scene->remove(light_entity);
    engine->destroy(light_entity);
    
    // Destroy swap chain
    std::cout << "Destroying swap chain..." << std::endl;
    engine->destroy(swap_chain);
    
    // Destroy gltfio objects in reverse creation order
    std::cout << "Destroying texture provider..." << std::endl;
    delete stb_provider;
    
    std::cout << "Destroying resource loader..." << std::endl;
    delete resource_loader;
    
    std::cout << "Destroying asset..." << std::endl;
    asset_loader->destroyAsset(asset);
    
    std::cout << "Destroying asset loader..." << std::endl;
    gltfio::AssetLoader::destroy(&asset_loader);
    
    std::cout << "Destroying material provider..." << std::endl;
    delete material_provider;
    material_provider = nullptr;
    
    // Destroy camera before engine
    std::cout << "Destroying camera component..." << std::endl;
    engine->destroyCameraComponent(camera_entity);
    std::cout << "Destroying camera entity..." << std::endl;
    utils::EntityManager::get().destroy(camera_entity);
    
    // Destroy engine last
    std::cout << "About to destroy engine..." << std::endl;
    auto* engine_ptr = engine;
    filament::Engine::destroy(&engine_ptr);
    std::cout << "Engine destroyed" << std::endl;
    
    std::cout << "Cleanup complete" << std::endl;
    
    return 0;
}
