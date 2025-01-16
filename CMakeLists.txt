cmake_minimum_required(VERSION 3.21)
project(rgl_gazebo_plugin)

find_package(ament_cmake REQUIRED)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install")
endif()

install(FILES
    lidar_patterns/LivoxMid360.mat3x4f
    DESTINATION share/${PROJECT_NAME}/lidar_patterns
    )
install (DIRECTORY test_world DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)
ament_environment_hooks(hooks/${PROJECT_NAME}.sh.in)

add_subdirectory(external)
add_subdirectory(RGLServerPlugin)
add_subdirectory(RGLVisualize)

ament_package()