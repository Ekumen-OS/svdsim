# Copyright 2025 Ekumen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(svdsim_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(svdsim_proto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Non-ROS dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BOX2D REQUIRED box2d)

# Download EnTT using ExternalProject
include(ExternalProject)

set(ENTT_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/entt-src)
set(ENTT_INCLUDE_DIR ${ENTT_SOURCE_DIR}/src)

externalproject_add(entt_external
  GIT_REPOSITORY https://github.com/skypjack/entt.git
  GIT_TAG b4e58bdd364ad72246c123a0c28538eab3252672
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/entt
  SOURCE_DIR ${ENTT_SOURCE_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  UPDATE_COMMAND ""
)

# Include directories
include_directories(
  include
  ${BOX2D_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ENTT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# Create library with all object code
add_library(${PROJECT_NAME} SHARED
  src/collision/collider_manager.cpp
  src/collision/static_map_handler.cpp
  src/ecs/components/drive_model_component.cpp
  src/ecs/components/imu_sensor_component.cpp
  src/ecs/components/lidar_sensor_component.cpp
  src/ecs/components/remote_control_server_component.cpp
  src/ecs/components/single_track_kinematic_model_component.cpp
  src/ecs/components/suspension_simulation_component.cpp
  src/ecs/components/wheel_encoder_sensor_component.cpp
  src/ecs/services/simulation_clock.cpp
  src/ecs/systems/imu_sensor_system.cpp
  src/ecs/systems/lidar_sensor_system.cpp
  src/ecs/systems/physics_simulation_system.cpp
  src/ecs/systems/remote_control_system.cpp
  src/ecs/systems/odometry_sensor_system.cpp
  src/ecs/systems/groundtruth_sensor_system.cpp
  src/ecs/systems/sparring_control_system.cpp
  src/ecs/systems/collision_count_system.cpp
  src/configuration/grid_csv_reader.cpp
  src/models/imu_sensor_model.cpp
  src/models/lidar_sensor_model.cpp
  src/models/pulse_encoder_model.cpp
  src/models/simple_drive_model.cpp
  src/models/single_track_drift_model.cpp
  src/models/single_track_kinematic_model.cpp
  src/models/sprung_mass_inclination_model.cpp
  src/models/mnc_tire_model.cpp
  src/svdsim_server_node.cpp
  src/discrete_time.cpp
  src/wall_time.cpp
  src/utils/map_loader.cpp
  src/utils/imu_sensor_utils.cpp
  src/utils/pid_controller.cpp
  src/utils/second_order_smoothing_filter.cpp
  src/utils/pure_pursuit_controller.cpp
  src/utils/trajectory_reader.cpp
  src/trajectory_buffer.cpp
)

# Add dependency on EnTT external project
add_dependencies(${PROJECT_NAME} entt_external)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  svdsim_proto
  rclcpp
  rclcpp_components
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  Sophus::Sophus
  ${BOX2D_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

# Register the component
rclcpp_components_register_nodes(${PROJECT_NAME}
  "svdsim_server::MonacoF1SimServerNode"
)

# Create executable node
add_executable(${PROJECT_NAME}_node
  src/svdsim_server_main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  svdsim_proto
  rclcpp
  rclcpp_components
  std_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  include(test/CMakeLists.txt)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
