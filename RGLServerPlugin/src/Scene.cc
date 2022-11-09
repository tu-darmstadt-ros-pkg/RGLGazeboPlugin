// Copyright 2022 Robotec.AI
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

#include "RGLServerPluginManager.hh"

#include <ignition/gazebo/components/SystemPluginInfo.hh>

#define RGL_INSTANCE "rgl::RGLServerPluginInstance"

using namespace rgl;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantFunctionResult"

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::CheckNewLidarsCb(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm) {
    auto data = ecm.ComponentData<ignition::gazebo::components::SystemPluginInfo>(entity);
    if (data == std::nullopt) {
        return true;
    }
    auto plugins = data->plugins();
    for (const auto& plugin : plugins) {
        if (plugin.name() == RGL_INSTANCE) {
            gazebo_lidars.insert(entity);
            for (auto descendant: ecm.Descendants(entity)) {
                lidar_ignore.insert(descendant);
            }
        }
    }
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::CheckRemovedLidarsCb(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm) {
    if (!gazebo_lidars.contains(entity)) {
        return true;
    }
    for (auto descendant: ecm.Descendants(entity)) {
        lidar_ignore.erase(descendant);
    }
    gazebo_lidars.erase(entity);
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::LoadEntityToRGLCb(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry* geometry) {
    if (lidar_ignore.contains(entity)) {
        return true;
    }
    if (entities_in_rgl.contains(entity)) {
        return true;
    }
    rgl_mesh_t new_mesh;
    if (!LoadMeshToRGL(&new_mesh, geometry->Data())) {
        return true;
    }
    rgl_entity_t new_rgl_entity;
    RGL_CHECK(rgl_entity_create(&new_rgl_entity, nullptr, new_mesh));
    entities_in_rgl.insert(std::make_pair(entity, std::make_pair(new_rgl_entity, new_mesh)));
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::RemoveEntityFromRGLCb(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry*) {
    if (lidar_ignore.contains(entity)) {
        lidar_ignore.erase(entity);
        return true;
    }
    if (!entities_in_rgl.contains(entity)) {
        return true;
    }
    RGL_CHECK(rgl_entity_destroy(entities_in_rgl.at(entity).first));
    RGL_CHECK(rgl_mesh_destroy(entities_in_rgl.at(entity).second));
    entities_in_rgl.erase(entity);
    return true;
}
#pragma clang diagnostic pop

void RGLServerPluginManager::UpdateRGLEntityPose(const ignition::gazebo::EntityComponentManager& ecm) {
    for (auto entity: entities_in_rgl) {
        auto rgl_matrix = GetRglMatrix(entity.first, ecm);
        RGL_CHECK(rgl_entity_set_pose(entity.second.first, &rgl_matrix));
    }
}
