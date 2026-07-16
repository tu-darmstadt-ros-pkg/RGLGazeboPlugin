// Copyright 2026 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include <gz/common/Image.hh>
#include <gz/common/Material.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/sim/Util.hh>

#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>

#include "RGLServerPluginManager.hh"

namespace rgl
{

namespace
{

std::string ResolveTexturePath(const std::string& uri, const std::string& filePath)
{
    if (uri.empty()) {
        return "";
    }
    std::string fullPath = gz::sim::asFullPath(uri, filePath);
    if (std::filesystem::exists(fullPath)) {
        return fullPath;
    }
    if (std::filesystem::exists(uri)) {
        return uri;
    }
    return "";
}

}  // namespace

rgl_texture_t RGLServerPluginManager::GetColorTextureFromFile(const std::string& texturePath)
{
    const std::string cacheKey = "file:" + texturePath;
    if (auto it = colorTextureCache.find(cacheKey); it != colorTextureCache.end()) {
        return it->second;
    }

    gz::common::Image image(texturePath);
    if (!image.Valid() || image.Width() == 0 || image.Height() == 0) {
        gzwarn << "Failed to load texture image '" << texturePath << "' for RGL color texture.\n";
        return nullptr;
    }

    std::vector<unsigned char> rgbaData = image.RGBAData();
    if (rgbaData.size() != static_cast<std::size_t>(image.Width()) * image.Height() * 4) {
        gzwarn << "Unexpected RGBA data size for texture image '" << texturePath << "'.\n";
        return nullptr;
    }

    rgl_texture_t texture = nullptr;
    if (!CheckRGL(rgl_texture_create_rgba8888(&texture, rgbaData.data(),
                                              static_cast<int32_t>(image.Width()),
                                              static_cast<int32_t>(image.Height())))) {
        gzwarn << "Failed to create RGL color texture from image '" << texturePath << "'.\n";
        return nullptr;
    }

    colorTextureCache.insert({cacheKey, texture});
    return texture;
}

rgl_texture_t RGLServerPluginManager::GetColorTextureFromColor(const gz::math::Color& color)
{
    const uint8_t rgba[4] = {
            static_cast<uint8_t>(255.0f * std::clamp(color.R(), 0.0f, 1.0f)),
            static_cast<uint8_t>(255.0f * std::clamp(color.G(), 0.0f, 1.0f)),
            static_cast<uint8_t>(255.0f * std::clamp(color.B(), 0.0f, 1.0f)),
            static_cast<uint8_t>(255.0f * std::clamp(color.A(), 0.0f, 1.0f))};

    char cacheKey[32];
    snprintf(cacheKey, sizeof(cacheKey), "rgba:%02x%02x%02x%02x", rgba[0], rgba[1], rgba[2], rgba[3]);
    if (auto it = colorTextureCache.find(cacheKey); it != colorTextureCache.end()) {
        return it->second;
    }

    rgl_texture_t texture = nullptr;
    if (!CheckRGL(rgl_texture_create_rgba8888(&texture, rgba, 1, 1))) {
        gzwarn << "Failed to create 1x1 RGL color texture.\n";
        return nullptr;
    }

    colorTextureCache.insert({cacheKey, texture});
    return texture;
}

rgl_texture_t RGLServerPluginManager::GetColorTexture(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm,
        const sdf::Geometry& geometry)
{
    // 1. SDF <material> on the visual has the highest priority (same as in rendering).
    if (auto* materialComponent = ecm.Component<gz::sim::components::Material>(entity)) {
        const sdf::Material& material = materialComponent->Data();

        // PBR albedo map, checking both workflows.
        if (const sdf::Pbr* pbr = material.PbrMaterial()) {
            for (auto workflowType : {sdf::PbrWorkflowType::METAL, sdf::PbrWorkflowType::SPECULAR}) {
                const sdf::PbrWorkflow* workflow = pbr->Workflow(workflowType);
                if (workflow == nullptr) {
                    continue;
                }
                std::string texturePath = ResolveTexturePath(workflow->AlbedoMap(), material.FilePath());
                if (!texturePath.empty()) {
                    if (auto texture = GetColorTextureFromFile(texturePath)) {
                        return texture;
                    }
                }
            }
        }

        return GetColorTextureFromColor(material.Diffuse());
    }

    // 2. For mesh geometries without an SDF material override: material embedded in the mesh file.
    if (geometry.Type() == sdf::GeometryType::MESH) {
        std::string meshPath = gz::sim::asFullPath(
                geometry.MeshShape()->Uri(),
                geometry.MeshShape()->FilePath());
        const gz::common::Mesh* mesh = meshManager->Load(meshPath);
        if (mesh == nullptr) {
            return nullptr;
        }

        // RGL supports a single texture per entity; use the material of the first submesh
        // that has one. Meshes with multiple different materials are not fully supported.
        if (mesh->MaterialCount() > 1) {
            gzwarn << "Mesh '" << meshPath << "' has multiple materials. RGL supports one color texture "
                   << "per entity; using the material of the first textured submesh for the whole mesh.\n";
        }

        gz::common::MaterialPtr fallbackMaterial = nullptr;
        for (unsigned int subMeshIdx = 0; subMeshIdx < mesh->SubMeshCount(); ++subMeshIdx) {
            auto subMesh = mesh->SubMeshByIndex(subMeshIdx).lock();
            if (!subMesh || !subMesh->GetMaterialIndex().has_value()) {
                continue;
            }
            auto material = mesh->MaterialByIndex(subMesh->GetMaterialIndex().value());
            if (material == nullptr) {
                continue;
            }
            fallbackMaterial = fallbackMaterial ? fallbackMaterial : material;
            std::string texturePath = ResolveTexturePath(material->TextureImage(), meshPath);
            if (!texturePath.empty()) {
                if (auto texture = GetColorTextureFromFile(texturePath)) {
                    return texture;
                }
            }
        }
        if (fallbackMaterial != nullptr) {
            return GetColorTextureFromColor(fallbackMaterial->Diffuse());
        }
    }

    return nullptr;
}

}  // namespace rgl
