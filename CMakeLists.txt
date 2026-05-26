cmake_minimum_required(VERSION 3.22.1)
project(nvblox_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
# Some local ROS Humble installs do not automatically discover Ubuntu's
# multiarch spdlog CMake config while resolving rcl_logging_spdlog.
if(NOT spdlog_DIR AND EXISTS "/usr/lib/x86_64-linux-gnu/cmake/spdlog/spdlogConfig.cmake")
  set(spdlog_DIR "/usr/lib/x86_64-linux-gnu/cmake/spdlog" CACHE PATH "" FORCE)
endif()
find_package(spdlog REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Keep the core nvblox library as a plain CMake/CUDA dependency. If a system or
# workspace nvblox install is available, use it; otherwise build the sibling
# source tree as part of this ROS package.
find_package(nvblox QUIET)
if(NOT nvblox_FOUND)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTORCH_WRAPPER OFF CACHE BOOL "" FORCE)
  set(BUILD_RENDERER OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../nvblox
    ${CMAKE_CURRENT_BINARY_DIR}/nvblox_core)
endif()

if(TARGET nvblox::nvblox_lib)
  set(NVBLOX_CORE_TARGET nvblox::nvblox_lib)
elseif(TARGET nvblox_lib)
  set(NVBLOX_CORE_TARGET nvblox_lib)
else()
  message(FATAL_ERROR "Could not find or build nvblox_lib")
endif()

add_executable(tsdf_only_node src/tsdf_only_node.cpp)
target_link_libraries(tsdf_only_node ${NVBLOX_CORE_TARGET})
ament_target_dependencies(
  tsdf_only_node
  rclcpp
  sensor_msgs
  message_filters
  tf2_msgs)

add_executable(color_tsdf_node src/color_tsdf_node.cpp)
target_link_libraries(color_tsdf_node ${NVBLOX_CORE_TARGET})
ament_target_dependencies(
  color_tsdf_node
  rclcpp
  sensor_msgs
  message_filters
  tf2_msgs)

install(TARGETS tsdf_only_node color_tsdf_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
