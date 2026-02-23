#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Camera.h>
#include <filament/SwapChain.h>
#include <utils/EntityManager.h>
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>

namespace gltfio = filament::gltfio;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_gltf_file>" << std::endl;
        return 1;
    }

    std::string gltf_path = argv[1];
    std::cout << "Loading glTF file: " << gltf_path << std::endl;

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
    
    // Create camera entity
    utils::Entity camera_entity = utils::EntityManager::get().create();
    filament::Camera* camera = engine->createCamera(camera_entity);
    
    // Configure view
    view->setScene(scene);
    view->setCamera(camera);
    
    std::cout << "Created renderer, scene, view, and camera" << std::endl;

    // Create glTF asset loader
    gltfio::AssetLoader* asset_loader = gltfio::AssetLoader::create({engine});
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

    // Load the glTF asset - read file into memory first
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
    
    std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(file)), {});
    file.close();
    
    gltfio::FilamentAsset* asset = asset_loader->createAsset(
        buffer.data(), buffer.size());
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
    gltfio::ResourceLoader* resource_loader = 
        new gltfio::ResourceLoader({engine});
    
    // Load resources (textures, buffers, etc.)
    resource_loader->asyncBeginLoad(asset);
    
    // Wait for resources to load
    float progress = 0.0f;
    while (progress < 1.0f) {
        progress = resource_loader->asyncGetLoadProgress();
    }
    resource_loader->asyncUpdateLoad();
    
    std::cout << "Resources loaded" << std::endl;

    // Add asset entities to the scene
    scene->addEntities(asset->getEntities(), asset->getEntityCount());
    
    std::cout << "Added asset to scene" << std::endl;
    std::cout << "glTF test completed successfully!" << std::endl;

    // Cleanup
    asset_loader->destroyAsset(asset);
    delete resource_loader;
    gltfio::AssetLoader::destroy(&asset_loader);
    
    engine->destroy(renderer);
    engine->destroy(scene);
    engine->destroy(view);
    engine->destroyCameraComponent(camera_entity);
    utils::EntityManager::get().destroy(camera_entity);
    
    filament::Engine::destroy(&engine);
    
    std::cout << "Cleanup complete" << std::endl;
    
    return 0;
}
