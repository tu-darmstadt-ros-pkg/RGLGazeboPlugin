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

#include <limits>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/sim/Util.hh>

#include "RGLServerPluginManager.hh"


namespace rgl
{

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantFunctionResult"

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::RegisterNewLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    // Plugin must be inside CustomSensor
    if (!ecm.EntityHasComponentType(entity, gz::sim::components::CustomSensor::typeId))
    {
        return true;
    }

    // Looking for plugin
    auto pluginData = ecm.ComponentData<gz::sim::components::SystemPluginInfo>(entity);
    if (pluginData == std::nullopt) {
        return true;
    }
    auto plugins = pluginData->plugins();
    for (const auto& plugin : plugins) {
        if (plugin.name() == RGL_INSTANCE) {
            lidarEntities.insert(entity);
            lidarsWithoutHousing.insert(entity);
            // Without a readable rate the transforms are updated every step.
            raytraceSchedules[entity] = {RaytraceInterval(entity, ecm).value_or(std::chrono::steady_clock::duration::zero()),
                                         RaytracePhase(entity, ecm)};
            if (doIgnoreEntitiesInLidarLink) {
                for (auto descendant: ecm.Descendants(entity)) {
                    entitiesToIgnore.insert(descendant);
                }
            }
            if (!colorTexturesEnabled && PluginRequestsColor(plugin.innerxml())) {
                gzmsg << "A lidar with publish_color enabled was registered; "
                      << "loading color textures for scene entities.\n";
                colorTexturesEnabled = true;
                AssignColorTexturesToLoadedEntities(ecm);
            }
        }
    }

    // RGL lidar plugin not found
    if (!lidarEntities.contains(entity)) {
        return true;
    }

    if (doIgnoreEntitiesInLidarLink) {
        // Ignore all entities in link associated with RGL lidar
        // Link could contain visual representation of the lidar
        for (auto entityInParentLink : GetEntitiesInParentLink(entity, ecm)) {
            entitiesToIgnore.insert(entityInParentLink);
        }
    }

    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::UnregisterLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    if (!lidarEntities.contains(entity)) {
        return true;
    }
    for (auto entityInParentLink : GetEntitiesInParentLink(entity, ecm)) {
        entitiesToIgnore.erase(entityInParentLink);
    }
    lidarEntities.erase(entity);
    lidarsWithoutHousing.erase(entity);
    raytraceSchedules.erase(entity);
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::LoadEntityToRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry* geometry,
        const gz::sim::EntityComponentManager& ecm)
{
    if (entitiesToIgnore.contains(entity)) {
        return true;
    }
    if (entitiesInRgl.contains(entity)) {
        gzwarn << "Trying to add same entity (" << entity << ") to rgl multiple times!\n";
        return true;
    }
    rgl_mesh_t rglMesh;
    gz::math::AxisAlignedBox bounds;
    if (!LoadMeshToRGL(&rglMesh, bounds, geometry->Data())) {
        gzerr << "Failed to load mesh to RGL from entity (" << entity << "). Skipping...\n";
        return true;
    }
    rgl_entity_t rglEntity;
    if (!CheckRGL(rgl_entity_create(&rglEntity, nullptr, rglMesh))) {
        gzerr << "Failed to load entity (" << entity << ") to RGL. Skipping...\n";
        return true;
    }
    // Hits report the model the visual belongs to (RGL_FIELD_ENTITY_ID_I32).
    if (!CheckRGL(rgl_entity_set_id(rglEntity, static_cast<int32_t>(gz::sim::topLevelModel(entity, ecm))))) {
        gzwarn << "Failed to set the model id of entity (" << entity << ") in RGL.\n";
    }
    // Assign a color texture from the visual's material so lidars can output colored point clouds
    // (RGL_FIELD_COLOR_RGBA_U32). Not fatal on failure; such points are colored white.
    if (colorTexturesEnabled) {
        if (rgl_texture_t colorTexture = GetColorTexture(entity, ecm, geometry->Data())) {
            if (!CheckRGL(rgl_entity_set_color_texture(rglEntity, colorTexture))) {
                gzwarn << "Failed to set color texture for entity (" << entity << ").\n";
            }
        }
    }
    entitiesInRgl.insert({entity, {rglEntity, rglMesh}});
    entityBounds.insert({entity, bounds});
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::RemoveEntityFromRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry*)
{
    if (entitiesToIgnore.contains(entity)) {
        entitiesToIgnore.erase(entity);
        return true;
    }
    if (!entitiesInRgl.contains(entity)) {
        return true;
    }
    if (!CheckRGL(rgl_entity_destroy(entitiesInRgl.at(entity).first))) {
        gzerr << "Failed to remove entity (" << entity << ") from RGL.\n";
    }
    if (!CheckRGL(rgl_mesh_destroy(entitiesInRgl.at(entity).second))) {
        gzerr << "Failed to remove mesh from entity (" << entity << ") in RGL.\n";
    }
    entitiesInRgl.erase(entity);
    entityBounds.erase(entity);
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::SetLaserRetroCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::LaserRetro* laser_retro)
{
    if (entitiesToIgnore.contains(entity)) {
        return true;
    }

    if (!entitiesInRgl.contains(entity)) {
        gzerr << "Trying to set Laser Retro for entity (" << entity << ") not loaded to RGL!\n";
        return true;
    }

    if (!CheckRGL(rgl_entity_set_laser_retro(entitiesInRgl.at(entity).first, laser_retro->Data()))) {
        gzerr << "Failed to set Laser Retro for entity (" << entity << ").\n";
    }
    return true;
}
#pragma clang diagnostic pop

void RGLServerPluginManager::UpdateRGLEntityTransforms(const gz::sim::EntityComponentManager& ecm)
{
    for (auto entity: entitiesInRgl) {
        rgl_mat3x4f rglMatrix = FindWorldPoseInRglMatrix(entity.first, ecm);
        if (!CheckRGL(rgl_entity_set_transform(entity.second.first, &rglMatrix))) {
            gzerr << "Failed to update transform for entity (" << entity.first << ").\n";
        }
    }
}

bool RGLServerPluginManager::RaytraceDueNextStep(const gz::sim::UpdateInfo& info)
{
    const auto next = info.simTime + info.dt;
    bool due = false;
    for (auto& [lidar, schedule] : raytraceSchedules) {
        if (next >= schedule.last + schedule.interval) {
            schedule.last = next;
            due = true;
        }
    }
    return due;
}

void RGLServerPluginManager::IgnoreLidarHousings(const gz::sim::EntityComponentManager& ecm)
{
    for (auto lidar = lidarsWithoutHousing.begin(); lidar != lidarsWithoutHousing.end();) {
        const gz::math::Vector3d origin = FindWorldPose(*lidar, ecm).Pos();
        // A housing is part of the sensor's own model; a visual of the world
        // around it may enclose the origin too, and must stay visible.
        const gz::sim::Entity ownModel = gz::sim::topLevelModel(*lidar, ecm);
        gz::sim::Entity housing = gz::sim::kNullEntity;
        double housingVolume = std::numeric_limits<double>::infinity();
        for (const auto& [entity, bounds] : entityBounds) {
            if (gz::sim::topLevelModel(entity, ecm) != ownModel) {
                continue;
            }
            const auto pose = FindWorldPose(entity, ecm);
            const auto local = pose.Rot().Inverse().RotateVector(origin - pose.Pos());
            if (bounds.Contains(local) && bounds.Volume() < housingVolume) {
                housing = entity;
                housingVolume = bounds.Volume();
            }
        }
        if (housing == gz::sim::kNullEntity) {
            ++lidar;
            continue;
        }
        if (!CheckRGL(rgl_entity_set_ignored_by_sensor(entitiesInRgl.at(housing).first,
                                                       static_cast<int32_t>(*lidar)))) {
            gzerr << "Failed to let lidar (" << *lidar << ") ignore its housing (" << housing << ").\n";
        } else {
            const auto name = ecm.Component<gz::sim::components::Name>(housing);
            gzmsg << "Lidar (" << *lidar << ") ignores its housing '"
                  << (name ? name->Data() : std::to_string(housing)) << "'.\n";
        }
        lidar = lidarsWithoutHousing.erase(lidar);
    }
}

std::unordered_set<gz::sim::Entity> RGLServerPluginManager::GetEntitiesInParentLink(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    auto parentEntity = ecm.ParentEntity(entity);
    if (parentEntity == gz::sim::kNullEntity ||
        !ecm.EntityHasComponentType(parentEntity, gz::sim::components::Link::typeId)) {
        return {};
    }
    return ecm.Descendants(parentEntity);
}

}  // namespace rgl
