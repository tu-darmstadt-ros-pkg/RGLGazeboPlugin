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

#pragma once

#include "rgl/api/core.h"

#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

#include <chrono>
#include <optional>

#define RGL_INSTANCE "rgl::RGLServerPluginInstance"

namespace rgl
{

// Checks RGL native call output status.
// Returns true when success.
// When API call returns error status, it prints error string and returns false.
bool CheckRGL(rgl_status_t status);

gz::math::Pose3<double> FindWorldPose(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm);

rgl_mat3x4f FindWorldPoseInRglMatrix(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm);

rgl_mat3x4f IgnPose3dToRglMatrix(const gz::math::Pose3<double>& pose);

// The raytrace interval of an RGL sensor (a custom sensor running RGL_INSTANCE),
// from its <update_rate>, as the instance computes it; nullopt for any other entity.
std::optional<std::chrono::steady_clock::duration> RaytraceInterval(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm);

// Where in its interval a sensor raytraces. The RGL sensors of one model with
// the same interval are spread evenly over it, in entity order, so they never
// raytrace, and publish, in the same step.
std::chrono::steady_clock::duration RaytracePhase(
        gz::sim::Entity sensor,
        const gz::sim::EntityComponentManager& ecm);

// Whether a sensor raytraces in the step from simTime - dt to simTime: its
// raytraces lie at phase + n * interval, each taken by the step that reaches
// it. The schedule depends on sim time only, not on when the sensor appeared
// or whether the simulation was paused.
bool RaytraceDue(std::chrono::steady_clock::duration interval,
                 std::chrono::steady_clock::duration phase,
                 std::chrono::steady_clock::duration simTime,
                 std::chrono::steady_clock::duration dt);


// Throws exception when version of RGL library mismatch
void ValidateRGLVersion();

}  // namespace rgl
