cmake_minimum_required(VERSION 3.21)
project(rgl_gazebo_plugin)

find_package(ament_cmake REQUIRED)

# RobotecGPULidar only ships x86_64 prebuilt binaries (see external/CMakeLists.txt).
# On arm64 the downloaded .so is wrong-format and linking fails. There is no upstream
# arm64 release, so skip the build.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(WARNING
        "rgl_gazebo_plugin: RobotecGPULidar has no arm64 prebuilt binary; skipping build on "
        "${CMAKE_SYSTEM_PROCESSOR}.")
    ament_package()
    return()
endif()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install")
endif()

install(FILES
    lidar_patterns/LivoxMid360.mat3x4f
    lidar_patterns/OusterOS0_128.mat3x4f
    DESTINATION share/${PROJECT_NAME}/lidar_patterns
    )
install (DIRECTORY test_world DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)
ament_environment_hooks(hooks/${PROJECT_NAME}.sh.in)

add_subdirectory(external)
add_subdirectory(RGLServerPlugin)
add_subdirectory(RGLVisualize)

ament_package()