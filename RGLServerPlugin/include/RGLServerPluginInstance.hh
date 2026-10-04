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

#include <filesystem>

#include "rgl/api/core.h"
#include "LidarPatternLoader.hh"

#include <gz/common/MeshManager.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/details/laserscan.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <condition_variable>
#include <mutex>
#include <random>
#include <stop_token>
#include <thread>


namespace rgl
{

class RGLServerPluginInstance :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    RGLServerPluginInstance() = default;

    ~RGLServerPluginInstance() override = default;

    // only called once, when plugin is being loaded
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    // called every time before physics update runs (can change entities)
    void PreUpdate(
            const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm) override;

    // called every time after physics runs (can't change entities)
    void PostUpdate(
            const gz::sim::UpdateInfo& info,
            const gz::sim::EntityComponentManager& ecm) override;

private:
    bool LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf);
    void CreateLidar(gz::sim::Entity entity,
                     gz::sim::EntityComponentManager& ecm);

    void UpdateLidarPose(const gz::sim::EntityComponentManager& ecm);
    void UpdateAlternatingLidarPattern();

    bool ShouldRayTrace(const gz::sim::UpdateInfo& info);
    void RayTrace(std::chrono::steady_clock::duration sim_time);
    void FetchAndPublishRaytraceResults();

    bool FetchLaserScanResult();
    bool FetchPointCloudResult(rgl_node_t formatNode);

    // Noise only in the sensor frame, where a point's direction is its ray.
    gz::msgs::PointCloudPacked CreatePointCloudMsg(std::chrono::steady_clock::duration sim_time, const std::string& frame, bool sensorFrame);
    gz::msgs::LaserScan CreateLaserScanMsg(std::chrono::steady_clock::duration sim_time, const std::string& frame);
    // pattern_camera: one ray per pixel. The simulation thread only copies the
    // per-ray results; a worker thread builds and publishes the images.
    struct CameraFrame
    {
        std::chrono::steady_clock::duration simTime{0};
        std::vector<float> distances{};
        std::vector<int32_t> entityIds{};
        std::vector<uint32_t> colors{};  // 0xAARRGGBB
    };
    bool FetchCameraFrame(std::chrono::steady_clock::duration sim_time);
    void PublishCameraFrames(std::stop_token stop);
    // Each pixel's distance as measured: depth within the range, with noise, NaN
    // where nothing is measured or, with filter_own_model, the own model is seen.
    std::vector<float> MeasureDistances(const CameraFrame& frame);
    gz::msgs::Image CreateDepthImageMsg(std::chrono::steady_clock::duration sim_time, const std::vector<float>& distances);
    gz::msgs::Image CreateColorImageMsg(const CameraFrame& frame);
    gz::msgs::PointCloudPacked CreateCameraPointCloudMsg(const CameraFrame& frame, const std::vector<float>& distances);
    gz::msgs::CameraInfo CreateCameraInfoMsg(std::chrono::steady_clock::duration sim_time);

    void DestroyLidar();

    std::string topicName;
    std::string frameId;
    rgl_vec2f lidarMinMaxRange;
    gz::math::Angle scanHMin;
    gz::math::Angle scanHMax;
    int scanHSamples;
    std::vector<rgl_mat3x4f> lidarPattern;
    std::size_t alternatingPatternIndex = 0;

    struct ResultPointCloud
    {
        std::vector<char> data{};
        int32_t hitPointCount{};
        // Extended with RGL_FIELD_COLOR_RGBA_U32 (+ sizeof(uint32_t)) when publish_color is enabled
        std::size_t pointSize{sizeof(rgl_vec3f) + sizeof(float)};  // Based on rglFields
        std::vector<rgl_field_t> rglFields = {
                RGL_FIELD_XYZ_VEC3_F32,
                RGL_FIELD_LASER_RETRO_F32
        };
    } resultPointCloud{};

    struct ResultLaserScan
    {
        std::vector<float> distances{};
        std::vector<float> intensities{};
        // No hitPointCount needed because LaserScan message contains non-hits also
        inline static const std::vector<rgl_field_t> rglFields = {
            RGL_FIELD_DISTANCE_F32,
            RGL_FIELD_LASER_RETRO_F32
        };
    } resultLaserScan{};

    // Byte offsets in an RGL point: the published fields come first, then the
    // entity id (filter_own_model).
    std::size_t publishedPointSize = 0;
    std::size_t entityIdOffset = 0;

    bool updateOnPausedSim = false;
    bool publishLaserScan = false;
    // pattern_camera: publish a depth image and its camera info instead of a
    // point cloud, and optionally a color image and an organized point cloud.
    bool publishDepthImage = false;
    CameraModel camera;
    std::string cameraInfoTopicName;
    std::string colorTopicName;
    std::string cameraPointsTopicName;
    // Whether the camera's rays return colors, for the color image or the cloud.
    bool cameraColor = false;
    bool publishColor = false;
    // Drop points on the model the sensor belongs to, as a robot's self-filter does.
    bool filterOwnModel = false;
    // Add a per-point "timestamp" field (float64, ns of simulation time).
    bool publishTimestamps = false;

    // <noise>: distance noise along each ray with stddev a + b * d^2, and
    // angular noise of the ray direction (point clouds only).
    struct Noise
    {
        float distanceStddev = 0.0f;           // a, m
        float distanceStddevQuadratic = 0.0f;  // b, 1/m
        float angularStddev = 0.0f;            // rad
        float Stddev(float distance) const { return distanceStddev + distanceStddevQuadratic * distance * distance; }
        bool Active() const { return distanceStddev > 0.0f || distanceStddevQuadratic > 0.0f || angularStddev > 0.0f; }
    } noise;
    // One generator per thread that applies noise: clouds on the simulation
    // thread, camera frames on the publishing thread.
    std::mt19937 cloudNoiseGenerator{std::random_device{}()};
    std::mt19937 cameraNoiseGenerator{std::random_device{}()};
    void AddCloudNoise(float* xyz);
    // RGL entity id of the sensor's model; the manager sets each entity's id to its model.
    int32_t ownModelId = -1;

    gz::sim::Entity thisLidarEntity;
    gz::transport::Node::Publisher pointCloudPublisher;
    gz::transport::Node::Publisher laserScanPublisher;
    gz::transport::Node::Publisher depthImagePublisher;
    gz::transport::Node::Publisher cameraInfoPublisher;
    gz::transport::Node::Publisher colorImagePublisher;
    gz::transport::Node::Publisher cameraPointsPublisher;
    gz::transport::Node::Publisher pointCloudWorldPublisher;
    gz::transport::Node gazeboNode;

    std::vector<rgl_node_t> rglNodesUseRays;
    rgl_node_t rglNodeLidarPose = nullptr;
    rgl_node_t rglNodeSetRange = nullptr;
    rgl_node_t rglNodeRaytrace = nullptr;
    rgl_node_t rglNodeCompact = nullptr;
    rgl_node_t rglNodeYieldLaserScan = nullptr;
    rgl_node_t rglNodeFormatPointCloudSensor = nullptr;
    rgl_node_t rglNodeFormatPointCloudWorld = nullptr;
    rgl_node_t rglNodeToLidarFrame = nullptr;
    rgl_node_t rglNodeYieldCamera = nullptr;

    // Each pixel's ray direction in the optical frame (z along the view, x
    // right, y down): a distance along the ray times it is the pixel's point,
    // and its z is the depth per metre of distance.
    std::vector<gz::math::Vector3f> opticalDirections;
    CameraFrame pendingCameraFrame;
    bool cameraFramePending = false;
    std::mutex cameraMutex;
    std::condition_variable_any cameraCondition;

    std::chrono::steady_clock::duration raytraceIntervalTime;
    std::chrono::steady_clock::duration raytracePhase{0};

    // rgl_graph_run is asynchronous (GPU); the result is fetched
    // publishDelaySteps later, or before the next raytrace if that comes
    // first, so the simulation goes on while the GPU works.
    int publishDelaySteps = 0;
    int stepsSinceRaytrace = 0;
    bool raytracePending = false;
    std::chrono::steady_clock::duration pendingRaytraceTime{0};

    bool isLidarInitialized = false;

    int onPausedSimUpdateCounter = 0;
    const int onPausedSimRaytraceInterval = 100;

    std::size_t lidarPatternSampleSize = 0;

    const std::string worldFrameId = "world";
    const std::string worldTopicPostfix = "/world";

    // Last, so it is stopped and joined before the members it uses go.
    std::jthread cameraPublisherThread;
};

}  // namespace rgl
