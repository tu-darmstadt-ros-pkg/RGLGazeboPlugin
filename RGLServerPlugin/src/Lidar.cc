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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>

#include <gz/sim/Util.hh>

#include "RGLServerPluginInstance.hh"
#include "Utils.hh"

#define PARAM_UPDATE_RATE_ID "update_rate"
#define PARAM_RANGE_ID "range"
#define PARAM_RANGE_MIN_ID "min"
#define PARAM_RANGE_MAX_ID "max"
#define PARAM_TOPIC_ID "topic"
#define PARAM_FRAME_ID "frame"
#define PARAM_UPDATE_ON_PAUSED_SIM_ID "update_on_paused_sim"
#define PARAM_PUBLISH_DELAY_STEPS_ID "publish_delay_steps"
#define PARAM_PUBLISH_COLOR_ID "publish_color"
#define PARAM_FILTER_OWN_MODEL_ID "filter_own_model"
#define PARAM_PUBLISH_TIMESTAMPS_ID "publish_timestamps"
#define PARAM_CAMERA_INFO_TOPIC_ID "camera_info_topic"
#define PARAM_COLOR_TOPIC_ID "color_topic"
#define PARAM_POINTS_TOPIC_ID "points_topic"
#define PARAM_NOISE_ID "noise"

namespace rgl
{

bool RGLServerPluginInstance::LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf)
{
    // Required parameters
    if (!sdf->HasElement(PARAM_UPDATE_RATE_ID)) {
        gzerr << "No '" << PARAM_UPDATE_RATE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_RANGE_ID) ||
        !sdf->FindElement(PARAM_RANGE_ID)->HasElement(PARAM_RANGE_MIN_ID) ||
        !sdf->FindElement(PARAM_RANGE_ID)->HasElement(PARAM_RANGE_MAX_ID)) {
        gzerr << "Range parameter is not defined correctly. Disabling plugin. Should be: \n"
               << "<" PARAM_RANGE_ID << ">\n"
               << "  <" << PARAM_RANGE_MIN_ID << ">" << "</" << PARAM_RANGE_MIN_ID << ">\n"
               << "  <" << PARAM_RANGE_MAX_ID << ">" << "</" << PARAM_RANGE_MAX_ID << ">\n"
               << "</" PARAM_RANGE_ID << ">\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_TOPIC_ID)) {
        gzerr << "No '" << PARAM_TOPIC_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_FRAME_ID)) {
        gzerr << "No '" << PARAM_FRAME_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    // Optional parameters
    if (!sdf->HasElement(PARAM_UPDATE_ON_PAUSED_SIM_ID)) {
        gzwarn << "No '" << PARAM_UPDATE_ON_PAUSED_SIM_ID << "' parameter specified for the RGL lidar. "
                << "Using default value: " << updateOnPausedSim << "\n";
    } else {
        updateOnPausedSim = sdf->Get<bool>(PARAM_UPDATE_ON_PAUSED_SIM_ID);
    }

    if (sdf->HasElement(PARAM_PUBLISH_DELAY_STEPS_ID)) {
        publishDelaySteps = sdf->Get<int>(PARAM_PUBLISH_DELAY_STEPS_ID);
        if (publishDelaySteps < 0) {
            gzerr << "'" << PARAM_PUBLISH_DELAY_STEPS_ID << "' must not be negative. Disabling plugin.\n";
            return false;
        }
    }

    // Load configuration
    float updateRateHz = sdf->Get<float>(PARAM_UPDATE_RATE_ID);
    raytraceIntervalTime = std::chrono::microseconds(static_cast<int64_t>(1e6 / updateRateHz));
    lidarMinMaxRange.value[0] = sdf->FindElement(PARAM_RANGE_ID)->Get<float>(PARAM_RANGE_MIN_ID);
    lidarMinMaxRange.value[1] = sdf->FindElement(PARAM_RANGE_ID)->Get<float>(PARAM_RANGE_MAX_ID);
    topicName = sdf->Get<std::string>(PARAM_TOPIC_ID);
    frameId = sdf->Get<std::string>(PARAM_FRAME_ID);

    if (!LidarPatternLoader::Load(sdf, lidarPattern, lidarPatternSampleSize)) {
        return false;
    }

    if ((lidarPattern.size() % lidarPatternSampleSize) != 0) {
        gzerr << "Total pattern size (" << lidarPattern.size() << ") must be a multiple of the sample size (" << lidarPatternSampleSize << "). Disabling plugin.\n";
        return false;
    }

    // Optional colored point cloud (RGB sampled from entity materials/textures)
    if (sdf->HasElement(PARAM_PUBLISH_COLOR_ID)) {
        publishColor = sdf->Get<bool>(PARAM_PUBLISH_COLOR_ID);
    }
    if (publishColor) {
        resultPointCloud.rglFields.push_back(RGL_FIELD_COLOR_RGBA_U32);
        resultPointCloud.pointSize += sizeof(uint32_t);
    }
    publishedPointSize = resultPointCloud.pointSize;

    if (sdf->HasElement(PARAM_FILTER_OWN_MODEL_ID)) {
        filterOwnModel = sdf->Get<bool>(PARAM_FILTER_OWN_MODEL_ID);
    }
    // The entity id follows the published fields; it is read for filtering and not published.
    entityIdOffset = resultPointCloud.pointSize;
    if (filterOwnModel) {
        resultPointCloud.rglFields.push_back(RGL_FIELD_ENTITY_ID_I32);
        resultPointCloud.pointSize += sizeof(int32_t);
    }

    if (sdf->HasElement("pattern_camera")) {
        if (!sdf->HasElement(PARAM_CAMERA_INFO_TOPIC_ID)) {
            gzerr << "No '" << PARAM_CAMERA_INFO_TOPIC_ID << "' parameter specified for the RGL camera. Disabling plugin.\n";
            return false;
        }
        if (!LidarPatternLoader::LoadCameraModel(sdf->FindElement("pattern_camera"), camera)) {
            return false;
        }
        publishDepthImage = true;
        cameraInfoTopicName = sdf->Get<std::string>(PARAM_CAMERA_INFO_TOPIC_ID);
        if (sdf->HasElement(PARAM_COLOR_TOPIC_ID)) {
            colorTopicName = sdf->Get<std::string>(PARAM_COLOR_TOPIC_ID);
        }
        if (sdf->HasElement(PARAM_POINTS_TOPIC_ID)) {
            cameraPointsTopicName = sdf->Get<std::string>(PARAM_POINTS_TOPIC_ID);
        }
        cameraColor = !colorTopicName.empty() || (!cameraPointsTopicName.empty() && publishColor);
    }

    if (sdf->HasElement(PARAM_PUBLISH_TIMESTAMPS_ID)) {
        publishTimestamps = sdf->Get<bool>(PARAM_PUBLISH_TIMESTAMPS_ID);
    }

    if (sdf->HasElement(PARAM_NOISE_ID)) {
        const auto noiseSdf = sdf->FindElement(PARAM_NOISE_ID);
        noise.distanceStddev = noiseSdf->Get<float>("distance_stddev", 0.0f).first;
        noise.distanceStddevQuadratic = noiseSdf->Get<float>("distance_stddev_quadratic", 0.0f).first;
        noise.angularStddev = noiseSdf->Get<float>("angular_stddev", 0.0f).first;
    }

    // Check for 2d pattern and get LaserScan parameters
    if (sdf->HasElement("pattern_lidar2d")) {
        gzmsg << "Lidar is 2D, switching to publish LaserScan messages";
        publishLaserScan = true;
        scanHMin = sdf->FindElement("pattern_lidar2d")->FindElement("horizontal")->Get<float>("min_angle");
        scanHMax = sdf->FindElement("pattern_lidar2d")->FindElement("horizontal")->Get<float>("max_angle");
        scanHSamples = lidarPatternSampleSize;
    }

    return true;
}

void RGLServerPluginInstance::CreateLidar(gz::sim::Entity entity,
                                          gz::sim::EntityComponentManager& ecm)
{
    thisLidarEntity = entity;
    ownModelId = static_cast<int32_t>(gz::sim::topLevelModel(entity, ecm));
    // The model's sensors all exist when its systems are configured.
    raytracePhase = RaytracePhase(entity, ecm);

    rgl_mat3x4f identity = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0
    };

    // Resize result data containers with the maximum possible point count (number of lasers).
    // This improves performance in runtime because no additional allocations are needed.
    resultPointCloud.data.resize(resultPointCloud.pointSize * lidarPatternSampleSize);
    if (publishLaserScan)
    {
        resultLaserScan.distances.resize(lidarPatternSampleSize);
        resultLaserScan.intensities.resize(lidarPatternSampleSize);
    }

    for (std::size_t i = 0; i < lidarPattern.size(); i += lidarPatternSampleSize)
    {
      rglNodesUseRays.emplace_back();
      if (!CheckRGL(rgl_node_rays_from_mat3x4f(&rglNodesUseRays.back(),
                                               lidarPattern.data() + i,
                                               lidarPatternSampleSize))) {
          gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
          return;
      }
    }

    // A camera's rays reach as far as the scene goes, for its color image; its
    // range applies to the measured distances.
    const rgl_vec2f rayRange = publishDepthImage ? rgl_vec2f{0.0f, std::numeric_limits<float>::max()} : lidarMinMaxRange;
    if (!CheckRGL(rgl_node_rays_set_range(&rglNodeSetRange, &rayRange, 1)) ||
        !CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &identity)) ||
        !CheckRGL(rgl_node_raytrace(&rglNodeRaytrace, nullptr)) ||
        !CheckRGL(rgl_node_points_compact_by_field(&rglNodeCompact, RGL_FIELD_IS_HIT_I32)) ||
        !CheckRGL(rgl_node_points_yield(&rglNodeYieldLaserScan, resultLaserScan.rglFields.data(), resultLaserScan.rglFields.size())) ||
        !CheckRGL(rgl_node_points_format(&rglNodeFormatPointCloudSensor, resultPointCloud.rglFields.data(), resultPointCloud.rglFields.size())) ||
        !CheckRGL(rgl_node_points_format(&rglNodeFormatPointCloudWorld, resultPointCloud.rglFields.data(), resultPointCloud.rglFields.size())) ||
        !CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &identity))) {

        gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    // The manager lets this id pass through the lidar's housing.
    if (!CheckRGL(rgl_node_raytrace_configure_id(rglNodeRaytrace, static_cast<int32_t>(entity)))) {
        gzerr << "Failed to set the RGL sensor id when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (!CheckRGL(rgl_graph_node_add_child(rglNodesUseRays.front(), rglNodeSetRange)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeSetRange, rglNodeLidarPose)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeLidarPose, rglNodeRaytrace)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeCompact)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeFormatPointCloudWorld))) {

        gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (publishLaserScan) {
        if(!CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeYieldLaserScan)) ||
           // Optimization: rglNodeYieldLaserScan should be prioritized because it will be requested first
           !CheckRGL(rgl_graph_node_set_priority(rglNodeYieldLaserScan, 1))) {
            gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        }
        gzmsg << "Start publishing LaserScan messages on topic '" << topicName << "'\n";
        laserScanPublisher = gazeboNode.Advertise<gz::msgs::LaserScan>(topicName);
    } else if (publishDepthImage) {
        // Every ray's distance in pixel order, NaN where nothing was hit.
        std::vector<rgl_field_t> fields = {RGL_FIELD_DISTANCE_F32};
        if (filterOwnModel) {
            fields.push_back(RGL_FIELD_ENTITY_ID_I32);
        }
        if (cameraColor) {
            fields.push_back(RGL_FIELD_COLOR_RGBA_U32);
        }
        const float nan = std::numeric_limits<float>::quiet_NaN();
        if (!CheckRGL(rgl_node_points_yield(&rglNodeYieldCamera, fields.data(), fields.size())) ||
            !CheckRGL(rgl_node_raytrace_configure_non_hits(rglNodeRaytrace, nan, nan)) ||
            !CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeYieldCamera)) ||
            !CheckRGL(rgl_graph_node_set_priority(rglNodeYieldCamera, 1))) {
            gzerr << "Failed to connect RGL nodes when initializing camera. Disabling plugin.\n";
            return;
        }
        // A ray's direction is the third column of its rotation, in the
        // sensor frame (x along the view, y left, z up).
        opticalDirections.resize(lidarPatternSampleSize);
        for (std::size_t i = 0; i < lidarPatternSampleSize; ++i) {
            const auto& ray = lidarPattern[i].value;
            opticalDirections[i].Set(-ray[1][2], -ray[2][2], ray[0][2]);
        }
        gzmsg << "Start publishing depth Image messages on topic '" << topicName
              << "' and CameraInfo on '" << cameraInfoTopicName << "'\n";
        depthImagePublisher = gazeboNode.Advertise<gz::msgs::Image>(topicName);
        cameraInfoPublisher = gazeboNode.Advertise<gz::msgs::CameraInfo>(cameraInfoTopicName);
        if (!colorTopicName.empty()) {
            gzmsg << "Start publishing color Image messages on topic '" << colorTopicName << "'\n";
            colorImagePublisher = gazeboNode.Advertise<gz::msgs::Image>(colorTopicName);
        }
        if (!cameraPointsTopicName.empty()) {
            gzmsg << "Start publishing PointCloudPacked messages on topic '" << cameraPointsTopicName << "'\n";
            cameraPointsPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(cameraPointsTopicName);
        }
        cameraPublisherThread = std::jthread([this](std::stop_token stop) { PublishCameraFrames(stop); });
    } else {  // publish PointCloud
        if(!CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeToLidarFrame)) ||
           !CheckRGL(rgl_graph_node_add_child(rglNodeToLidarFrame, rglNodeFormatPointCloudSensor)) ||
           // Optimization: rglNodeFormatPointCloudSensor should be prioritized because it will be requested first
           !CheckRGL(rgl_graph_node_set_priority(rglNodeFormatPointCloudSensor, 1))) {
            gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        }
        gzmsg << "Start publishing PointCloudPacked messages on topic '" << topicName << "'\n";
        pointCloudPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName);
    }
    pointCloudWorldPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName + worldTopicPostfix);

    isLidarInitialized = true;
}

void RGLServerPluginInstance::UpdateLidarPose(const gz::sim::EntityComponentManager& ecm)
{
    gz::math::Pose3<double> ignLidarToWorld = FindWorldPose(thisLidarEntity, ecm);
    gz::math::Pose3<double> ignWorldToLidar = ignLidarToWorld.Inverse();
    rgl_mat3x4f rglLidarToWorld = IgnPose3dToRglMatrix(ignLidarToWorld);
    rgl_mat3x4f rglWorldToLidar = IgnPose3dToRglMatrix(ignWorldToLidar);
    CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &rglLidarToWorld));
    CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &rglWorldToLidar));
}

void RGLServerPluginInstance::UpdateAlternatingLidarPattern()
{
    // remove old child
    if(!CheckRGL(rgl_graph_node_remove_child(rglNodesUseRays[alternatingPatternIndex], rglNodeSetRange)))
    {
        gzerr << "Failed to update alternating lidar pattern, not able to remove child.\n";
        return;
    }

    alternatingPatternIndex = (alternatingPatternIndex + 1) % rglNodesUseRays.size();

    // add new child
    if(!CheckRGL(rgl_graph_node_add_child(rglNodesUseRays[alternatingPatternIndex], rglNodeSetRange)))
    {
        gzerr << "Failed to update alternating lidar pattern, not able to add new child.\n";
        return;
    }
}

bool RGLServerPluginInstance::ShouldRayTrace(const gz::sim::UpdateInfo& info)
{
    if (!isLidarInitialized) {
        return false;
    }

    if (info.paused) {
        ++onPausedSimUpdateCounter;
        if (!updateOnPausedSim) {
            return false;
        }
        if (onPausedSimUpdateCounter < onPausedSimRaytraceInterval) {
            return false;
        }
        onPausedSimUpdateCounter = 0;
        return true;
    }

    // Simulation running; the manager follows the same schedule.
    return RaytraceDue(raytraceIntervalTime, raytracePhase, info.simTime, info.dt);
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration simTime)
{
    if (rglNodesUseRays.size() > 1) {
        UpdateAlternatingLidarPattern();
    }

    if (!CheckRGL(rgl_graph_run(rglNodeRaytrace))) {
        gzerr << "Failed to perform raytrace.\n";
        return;
    }
    // rgl_graph_run only queues GPU work; the result is fetched later.
    raytracePending = true;
    stepsSinceRaytrace = 0;
    pendingRaytraceTime = simTime;
}

void RGLServerPluginInstance::FetchAndPublishRaytraceResults()
{
    if (!raytracePending) {
        return;
    }
    raytracePending = false;
    const auto simTime = pendingRaytraceTime;

    if (publishLaserScan) {
        if (!FetchLaserScanResult()) {
            gzerr << "Failed to fetch LaserScan result data from RGL lidar.\n";
            return;
        }
        auto msg = CreateLaserScanMsg(simTime, frameId);
        laserScanPublisher.Publish(msg);
    } else if (publishDepthImage) {
        if (!FetchCameraFrame(simTime)) {
            gzerr << "Failed to fetch result data from RGL camera.\n";
            return;
        }
    } else {  // publish PointCloud
        if (!FetchPointCloudResult(rglNodeFormatPointCloudSensor)) {
            gzerr << "Failed to fetch PointCloud result data (sensor frame) from RGL lidar.\n";
            return;
        }
        pointCloudPublisher.Publish(CreatePointCloudMsg(simTime, frameId, true));
    }

    if (pointCloudWorldPublisher.HasConnections()) {
        if (!FetchPointCloudResult(rglNodeFormatPointCloudWorld)) {
            gzerr << "Failed to fetch PointCloud result data (world frame) from RGL lidar.\n";
            return;
        }
        auto msg = CreatePointCloudMsg(simTime, worldFrameId, false);
        pointCloudWorldPublisher.Publish(msg);
    }
}

bool RGLServerPluginInstance::FetchLaserScanResult()
{
    if (!CheckRGL(rgl_graph_get_result_data(rglNodeYieldLaserScan, RGL_FIELD_DISTANCE_F32, resultLaserScan.distances.data())) ||
        !CheckRGL(rgl_graph_get_result_data(rglNodeYieldLaserScan, RGL_FIELD_LASER_RETRO_F32, resultLaserScan.intensities.data()))) {
        return false;
    }
    return true;
}

bool RGLServerPluginInstance::FetchPointCloudResult(rgl_node_t formatNode)
{
    int32_t rglPointSize = -1;
    if (!CheckRGL(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &resultPointCloud.hitPointCount, &rglPointSize))) {
        return false;
    }
    assert(rglPointSize == resultPointCloud.pointSize);
    if (!CheckRGL(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, resultPointCloud.data.data()))) {
        return false;
    }
    return true;
}

gz::msgs::LaserScan RGLServerPluginInstance::CreateLaserScanMsg(std::chrono::steady_clock::duration simTime, const std::string& frame)
{
    auto pointCount = resultLaserScan.distances.size();
    gz::msgs::LaserScan outMsg;
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    auto _frame = outMsg.mutable_header()->add_data();
    _frame->set_key("frame_id");
    _frame->add_value(frame);

    outMsg.set_frame(frame);
    outMsg.set_count(pointCount);

    outMsg.set_range_min(lidarMinMaxRange.value[0]);
    outMsg.set_range_max(lidarMinMaxRange.value[1]);

    gz::math::Angle hStep((scanHMax-scanHMin)/scanHSamples);

    outMsg.set_angle_min(scanHMin.Radian());
    outMsg.set_angle_max(scanHMax.Radian());
    outMsg.set_angle_step(hStep.Radian());

    for (int i = 0; i < pointCount; ++i) {
        outMsg.add_ranges(resultLaserScan.distances[i]);
        outMsg.add_intensities(resultLaserScan.intensities[i]);
    }

    return outMsg;
}

gz::msgs::PointCloudPacked RGLServerPluginInstance::CreatePointCloudMsg(std::chrono::steady_clock::duration simTime, const std::string& frame, bool sensorFrame)
{
    gz::msgs::PointCloudPacked outMsg;
    std::vector<std::pair<std::string, gz::msgs::PointCloudPacked::Field::DataType>> msgFields = {
            {"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
            {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32}};
    if (publishColor) {
        // RGL_FIELD_COLOR_RGBA_U32 is packed as 0xAARRGGBB, which matches the memory layout
        // of the float-typed "rgb" field convention used by PCL, RViz and ros_gz_bridge.
        msgFields.emplace_back("rgb", gz::msgs::PointCloudPacked::Field::FLOAT32);
    }
    if (publishTimestamps) {
        msgFields.emplace_back("timestamp", gz::msgs::PointCloudPacked::Field::FLOAT64);
    }
    gz::msgs::InitPointCloudPacked(outMsg, frame, false, msgFields);
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);

    // All rays are traced at simTime.
    const std::size_t rglStep = resultPointCloud.pointSize;
    const std::size_t copySize = publishedPointSize;
    const std::size_t msgStep = outMsg.point_step();
    const double timestamp = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(simTime).count());

    outMsg.mutable_data()->resize(resultPointCloud.hitPointCount * msgStep);
    char* out = outMsg.mutable_data()->data();
    std::size_t pointCount = 0;
    for (int32_t i = 0; i < resultPointCloud.hitPointCount; ++i) {
        const char* point = resultPointCloud.data.data() + i * rglStep;
        if (filterOwnModel) {
            int32_t entityId;
            memcpy(&entityId, point + entityIdOffset, sizeof(entityId));
            if (entityId == ownModelId) {
                continue;
            }
        }
        memcpy(out + pointCount * msgStep, point, copySize);
        if (sensorFrame && noise.Active()) {
            AddCloudNoise(reinterpret_cast<float*>(out + pointCount * msgStep));
        }
        if (publishTimestamps) {
            memcpy(out + pointCount * msgStep + copySize, &timestamp, sizeof(timestamp));
        }
        ++pointCount;
    }
    outMsg.mutable_data()->resize(pointCount * msgStep);
    outMsg.set_height(1);
    outMsg.set_width(pointCount);
    outMsg.set_row_step(pointCount * msgStep);
    return outMsg;
}

void RGLServerPluginInstance::AddCloudNoise(float* xyz)
{
    std::normal_distribution<float> standard;
    const gz::math::Vector3f point(xyz[0], xyz[1], xyz[2]);
    const float distance = point.Length();
    if (distance <= 0.0f) {
        return;
    }
    gz::math::Vector3f direction = point / distance;
    if (noise.angularStddev > 0.0f) {
        // Two directions across the ray, tilted by small normal angles.
        const gz::math::Vector3f helper = std::abs(direction.Z()) < 0.9f ? gz::math::Vector3f::UnitZ : gz::math::Vector3f::UnitX;
        const gz::math::Vector3f across = direction.Cross(helper).Normalized();
        const gz::math::Vector3f up = direction.Cross(across);
        direction += noise.angularStddev * (standard(cloudNoiseGenerator) * across + standard(cloudNoiseGenerator) * up);
        direction.Normalize();
    }
    const float noisy = distance + noise.Stddev(distance) * standard(cloudNoiseGenerator);
    xyz[0] = noisy * direction.X();
    xyz[1] = noisy * direction.Y();
    xyz[2] = noisy * direction.Z();
}

bool RGLServerPluginInstance::FetchCameraFrame(std::chrono::steady_clock::duration simTime)
{
    std::unique_lock lock(cameraMutex);
    // The worker takes a frame long before the next is due; this rarely waits.
    cameraCondition.wait(lock, [this] { return !cameraFramePending; });
    CameraFrame& frame = pendingCameraFrame;
    frame.simTime = simTime;
    frame.distances.resize(lidarPatternSampleSize);
    if (!CheckRGL(rgl_graph_get_result_data(rglNodeYieldCamera, RGL_FIELD_DISTANCE_F32, frame.distances.data()))) {
        return false;
    }
    if (filterOwnModel) {
        frame.entityIds.resize(lidarPatternSampleSize);
        if (!CheckRGL(rgl_graph_get_result_data(rglNodeYieldCamera, RGL_FIELD_ENTITY_ID_I32, frame.entityIds.data()))) {
            return false;
        }
    }
    if (cameraColor) {
        frame.colors.resize(lidarPatternSampleSize);
        if (!CheckRGL(rgl_graph_get_result_data(rglNodeYieldCamera, RGL_FIELD_COLOR_RGBA_U32, frame.colors.data()))) {
            return false;
        }
    }
    cameraFramePending = true;
    lock.unlock();
    cameraCondition.notify_all();
    return true;
}

void RGLServerPluginInstance::PublishCameraFrames(std::stop_token stop)
{
    CameraFrame frame;
    while (true) {
        {
            std::unique_lock lock(cameraMutex);
            if (!cameraCondition.wait(lock, stop, [this] { return cameraFramePending; })) {
                return;
            }
            // The fetch fills the buffers this frame held before.
            std::swap(frame, pendingCameraFrame);
            cameraFramePending = false;
        }
        cameraCondition.notify_all();
        // The depth image and the cloud show the same measurement.
        const std::vector<float> distances = MeasureDistances(frame);
        depthImagePublisher.Publish(CreateDepthImageMsg(frame.simTime, distances));
        cameraInfoPublisher.Publish(CreateCameraInfoMsg(frame.simTime));
        if (colorImagePublisher.HasConnections()) {
            colorImagePublisher.Publish(CreateColorImageMsg(frame));
        }
        if (cameraPointsPublisher.HasConnections()) {
            cameraPointsPublisher.Publish(CreateCameraPointCloudMsg(frame, distances));
        }
    }
}

std::vector<float> RGLServerPluginInstance::MeasureDistances(const CameraFrame& frame)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::normal_distribution<float> standard;
    std::vector<float> distances(frame.distances.size());
    for (std::size_t i = 0; i < distances.size(); ++i) {
        const float distance = frame.distances[i];
        const bool own = filterOwnModel && frame.entityIds[i] == ownModelId;
        // NaN, a miss, fails both comparisons.
        if (own || !(distance >= lidarMinMaxRange.value[0] && distance <= lidarMinMaxRange.value[1])) {
            distances[i] = nan;
            continue;
        }
        distances[i] = noise.Active() ? distance + noise.Stddev(distance) * standard(cameraNoiseGenerator) : distance;
    }
    return distances;
}

gz::msgs::Image RGLServerPluginInstance::CreateDepthImageMsg(std::chrono::steady_clock::duration simTime,
                                                             const std::vector<float>& distances)
{
    gz::msgs::Image outMsg;
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    auto frameData = outMsg.mutable_header()->add_data();
    frameData->set_key("frame_id");
    frameData->add_value(frameId);
    outMsg.set_width(camera.width);
    outMsg.set_height(camera.height);
    outMsg.set_pixel_format_type(gz::msgs::PixelFormatType::R_FLOAT32);
    outMsg.set_step(camera.width * sizeof(float));

    // Depth along the optical axis in metres.
    std::string* data = outMsg.mutable_data();
    data->resize(distances.size() * sizeof(float));
    float* depth = reinterpret_cast<float*>(data->data());
    for (std::size_t i = 0; i < distances.size(); ++i) {
        depth[i] = distances[i] * opticalDirections[i].Z();
    }
    return outMsg;
}

gz::msgs::Image RGLServerPluginInstance::CreateColorImageMsg(const CameraFrame& frame)
{
    gz::msgs::Image outMsg;
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(frame.simTime);
    auto frameData = outMsg.mutable_header()->add_data();
    frameData->set_key("frame_id");
    frameData->add_value(frameId);
    outMsg.set_width(camera.width);
    outMsg.set_height(camera.height);
    outMsg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
    outMsg.set_step(camera.width * 3);

    std::string* data = outMsg.mutable_data();
    data->resize(frame.colors.size() * 3);
    for (std::size_t i = 0; i < frame.colors.size(); ++i) {
        const uint32_t color = frame.colors[i];
        (*data)[3 * i + 0] = static_cast<char>((color >> 16) & 0xFF);
        (*data)[3 * i + 1] = static_cast<char>((color >> 8) & 0xFF);
        (*data)[3 * i + 2] = static_cast<char>(color & 0xFF);
    }
    return outMsg;
}

gz::msgs::PointCloudPacked RGLServerPluginInstance::CreateCameraPointCloudMsg(const CameraFrame& frame,
                                                                              const std::vector<float>& distances)
{
    gz::msgs::PointCloudPacked outMsg;
    std::vector<std::pair<std::string, gz::msgs::PointCloudPacked::Field::DataType>> msgFields = {
            {"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}};
    if (publishColor) {
        msgFields.emplace_back("rgb", gz::msgs::PointCloudPacked::Field::FLOAT32);
    }
    if (publishTimestamps) {
        msgFields.emplace_back("timestamp", gz::msgs::PointCloudPacked::Field::FLOAT64);
    }
    gz::msgs::InitPointCloudPacked(outMsg, frameId, false, msgFields);
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(frame.simTime);

    // Organized as the image, NaN where the depth image is.
    const std::size_t step = outMsg.point_step();
    const double timestamp = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(frame.simTime).count());
    outMsg.mutable_data()->resize(distances.size() * step);
    char* out = outMsg.mutable_data()->data();
    for (std::size_t i = 0; i < distances.size(); ++i) {
        char* point = out + i * step;
        const gz::math::Vector3f xyz = distances[i] * opticalDirections[i];
        const float coordinates[3] = {xyz.X(), xyz.Y(), xyz.Z()};
        memcpy(point, coordinates, sizeof(coordinates));
        std::size_t offset = sizeof(coordinates);
        if (publishColor) {
            memcpy(point + offset, &frame.colors[i], sizeof(uint32_t));
            offset += sizeof(uint32_t);
        }
        if (publishTimestamps) {
            memcpy(point + offset, &timestamp, sizeof(timestamp));
        }
    }
    outMsg.set_height(camera.height);
    outMsg.set_width(camera.width);
    outMsg.set_row_step(camera.width * step);
    outMsg.set_is_dense(false);
    return outMsg;
}

gz::msgs::CameraInfo RGLServerPluginInstance::CreateCameraInfoMsg(std::chrono::steady_clock::duration simTime)
{
    gz::msgs::CameraInfo outMsg;
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    auto frame = outMsg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    outMsg.set_width(camera.width);
    outMsg.set_height(camera.height);

    const double f = camera.focalLength;
    outMsg.mutable_distortion()->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
    for (int i = 0; i < 5; ++i) {
        outMsg.mutable_distortion()->add_k(0.0);
    }
    for (double k : {f, 0.0, camera.cx, 0.0, f, camera.cy, 0.0, 0.0, 1.0}) {
        outMsg.mutable_intrinsics()->add_k(k);
    }
    for (double p : {f, 0.0, camera.cx, 0.0, 0.0, f, camera.cy, 0.0, 0.0, 0.0, 1.0, 0.0}) {
        outMsg.mutable_projection()->add_p(p);
    }
    for (double r : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
        outMsg.add_rectification_matrix(r);
    }
    return outMsg;
}

void RGLServerPluginInstance::DestroyLidar()
{
    if (!isLidarInitialized) {
        return;
    }

    if (cameraPublisherThread.joinable()) {
        cameraPublisherThread.request_stop();
        cameraPublisherThread.join();
    }
    if (!CheckRGL(rgl_graph_destroy(rglNodeRaytrace))) {
        gzerr << "Failed to destroy RGL lidar.\n";
    }
    // Reset publishers
    pointCloudPublisher = gz::transport::Node::Publisher();
    laserScanPublisher = gz::transport::Node::Publisher();
    depthImagePublisher = gz::transport::Node::Publisher();
    cameraInfoPublisher = gz::transport::Node::Publisher();
    colorImagePublisher = gz::transport::Node::Publisher();
    cameraPointsPublisher = gz::transport::Node::Publisher();
    pointCloudWorldPublisher = gz::transport::Node::Publisher();
    isLidarInitialized = false;
}

}  // namespace rgl
