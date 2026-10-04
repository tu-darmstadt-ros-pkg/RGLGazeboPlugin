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

#include <algorithm>
#include <exception>
#include <regex>
#include <vector>

#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/SystemPluginInfo.hh>

#include "Utils.hh"
#include "gz/math/Matrix4.hh"

#define WORLD_ENTITY_ID 1

namespace rgl
{

bool CheckRGL(rgl_status_t status)
{
    if (status == RGL_SUCCESS) {
        return true;
    }

    const char* msg;
    rgl_get_last_error_string(&msg);
    gzerr << msg << "\n";
    return false;
}

gz::math::Pose3<double> FindWorldPose(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm)
{
    auto localPose = ecm.Component<gz::sim::components::Pose>(entity);
    if (nullptr == localPose) {
        gzmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
        return gz::math::Pose3d::Zero;
    }
    auto worldPose = localPose->Data();

    gz::sim::Entity thisEntity = entity;
    gz::sim::Entity parent;

    while ((parent = ecm.ParentEntity(thisEntity)) != WORLD_ENTITY_ID) {
        auto parentPose = ecm.Component<gz::sim::components::Pose>(parent);
        if (nullptr == parentPose) {
            gzmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
            return gz::math::Pose3d::Zero;
        }
        worldPose = parentPose->Data() * worldPose;
        thisEntity = parent;
    }

    return worldPose;
}

rgl_mat3x4f FindWorldPoseInRglMatrix(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm)
{
    return IgnPose3dToRglMatrix(FindWorldPose(entity, ecm));
}

rgl_mat3x4f IgnPose3dToRglMatrix(
        const gz::math::Pose3<double>& pose)
{

    auto ignMatrix = gz::math::Matrix4<double>(pose);
    rgl_mat3x4f rglMatrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rglMatrix.value[i][j] = static_cast<float>(ignMatrix(i, j));
        }
    }
    return rglMatrix;
}

std::optional<std::chrono::steady_clock::duration> RaytraceInterval(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    if (!ecm.EntityHasComponentType(entity, gz::sim::components::CustomSensor::typeId)) {
        return std::nullopt;
    }
    const auto pluginData = ecm.ComponentData<gz::sim::components::SystemPluginInfo>(entity);
    if (!pluginData) {
        return std::nullopt;
    }
    static const std::regex updateRateRegex("<update_rate>\\s*([^<\\s]+)\\s*</update_rate>");
    for (const auto& plugin : pluginData->plugins()) {
        std::smatch match;
        const std::string xml = plugin.innerxml();
        if (plugin.name() != RGL_INSTANCE || !std::regex_search(xml, match, updateRateRegex)) {
            continue;
        }
        try {
            const float updateRateHz = std::stof(match[1].str());
            return std::chrono::microseconds(static_cast<int64_t>(1e6 / updateRateHz));
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::chrono::steady_clock::duration RaytracePhase(
        gz::sim::Entity sensor,
        const gz::sim::EntityComponentManager& ecm)
{
    const auto interval = RaytraceInterval(sensor, ecm);
    if (!interval) {
        return std::chrono::steady_clock::duration::zero();
    }
    const gz::sim::Entity model = gz::sim::topLevelModel(sensor, ecm);
    std::vector<gz::sim::Entity> peers;
    ecm.Each<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity& entity, const gz::sim::components::CustomSensor*) {
            if (gz::sim::topLevelModel(entity, ecm) == model && RaytraceInterval(entity, ecm) == interval) {
                peers.push_back(entity);
            }
            return true;
        });
    std::sort(peers.begin(), peers.end());
    const auto rank = std::find(peers.begin(), peers.end(), sensor) - peers.begin();
    return *interval * rank / static_cast<int64_t>(std::max<std::size_t>(peers.size(), 1));
}

bool RaytraceDue(std::chrono::steady_clock::duration interval,
                 std::chrono::steady_clock::duration phase,
                 std::chrono::steady_clock::duration simTime,
                 std::chrono::steady_clock::duration dt)
{
    // The number of raytrace times up to t.
    const auto count = [&](std::chrono::steady_clock::duration t) {
        const auto sinceFirst = t - phase;
        const auto n = sinceFirst / interval;
        return sinceFirst % interval < std::chrono::steady_clock::duration::zero() ? n - 1 : n;
    };
    return count(simTime) > count(simTime - dt);
}

void ValidateRGLVersion()
{
    int32_t outMajor, outMinor, outPatch;
    if (!CheckRGL(rgl_get_version_info(&outMajor, &outMinor, &outPatch))) {
        throw std::runtime_error("Failed to get RGL library version.");
    }

    if (outMajor != RGL_VERSION_MAJOR || outMinor != RGL_VERSION_MINOR || outPatch != RGL_VERSION_PATCH) {
        std::ostringstream oss;
        oss << "RGL library version: " << outMajor << "." << outMinor << "." << outPatch
            << " does not match RGL core.h version: " << RGL_VERSION_MAJOR << "." << RGL_VERSION_MINOR
            << "." << RGL_VERSION_PATCH << ".\n";
        throw std::runtime_error(oss.str());
    }
}

}  // namespace rgl
