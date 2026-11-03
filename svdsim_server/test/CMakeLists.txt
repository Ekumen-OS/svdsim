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

# Unit tests
ament_add_gtest(test_${PROJECT_NAME}_basic
  test/test_basic.cpp
)

target_link_libraries(test_${PROJECT_NAME}_basic
  ${PROJECT_NAME}
)

# EnTT library tests
ament_add_gtest(test_${PROJECT_NAME}_entt_library
  test/test_entt_library.cpp
)

add_dependencies(test_${PROJECT_NAME}_entt_library entt_external)

target_include_directories(test_${PROJECT_NAME}_entt_library PRIVATE
  ${ENTT_INCLUDE_DIR}
)

# ActivationQueue tests
ament_add_gtest(test_${PROJECT_NAME}_activation_queue
  test/test_activation_queue.cpp
)

target_link_libraries(test_${PROJECT_NAME}_activation_queue
  ${PROJECT_NAME}
)

# Sophus library tests
ament_add_gtest(test_${PROJECT_NAME}_sophus_library
  test/test_sophus_library.cpp
)

target_link_libraries(test_${PROJECT_NAME}_sophus_library
  Sophus::Sophus
)

# Runge-Kutta integration tests
ament_add_gtest(test_${PROJECT_NAME}_runge_kutta
  test/test_runge_kutta.cpp
)

target_link_libraries(test_${PROJECT_NAME}_runge_kutta
  ${PROJECT_NAME}
)

# Single track kinematic model tests
ament_add_gtest(test_${PROJECT_NAME}_single_track_kinematic_model
  test/test_single_track_kinematic_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_single_track_kinematic_model
  ${PROJECT_NAME}
)

# Box2D linking tests
ament_add_gtest(test_${PROJECT_NAME}_box2d_linking
  test/test_box2d_linking.cpp
)

target_include_directories(test_${PROJECT_NAME}_box2d_linking PRIVATE
  ${BOX2D_INCLUDE_DIRS}
)

target_link_libraries(test_${PROJECT_NAME}_box2d_linking
  ${BOX2D_LIBRARIES}
)

# Collider manager tests
ament_add_gtest(test_${PROJECT_NAME}_collider_manager
  test/test_collider_manager.cpp
)

target_link_libraries(test_${PROJECT_NAME}_collider_manager
  ${PROJECT_NAME}
)

# Static map handler tests
ament_add_gtest(test_${PROJECT_NAME}_static_map_handler
  test/test_static_map_handler.cpp
)

target_link_libraries(test_${PROJECT_NAME}_static_map_handler
  ${PROJECT_NAME}
)

# Trajectory buffer tests
ament_add_gtest(test_${PROJECT_NAME}_trajectory_buffer
  test/test_trajectory_buffer.cpp
)

target_link_libraries(test_${PROJECT_NAME}_trajectory_buffer
  ${PROJECT_NAME}
)

# Map loader tests
ament_add_gtest(test_${PROJECT_NAME}_map_loader
  test/test_map_loader.cpp
)

target_link_libraries(test_${PROJECT_NAME}_map_loader
  ${PROJECT_NAME}
)

# Grid CSV reader tests
ament_add_gtest(test_${PROJECT_NAME}_grid_csv_reader
  test/test_grid_csv_reader.cpp
)

target_link_libraries(test_${PROJECT_NAME}_grid_csv_reader
  ${PROJECT_NAME}
)

# Pulse encoder model tests
ament_add_gtest(test_${PROJECT_NAME}_pulse_encoder_model
  test/test_pulse_encoder_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_pulse_encoder_model
  ${PROJECT_NAME}
)

# Lidar sensor model tests
ament_add_gtest(test_${PROJECT_NAME}_lidar_sensor_model
  test/test_lidar_sensor_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_lidar_sensor_model
  ${PROJECT_NAME}
)

# IMU sensor model tests
ament_add_gtest(test_${PROJECT_NAME}_imu_sensor_model
  test/test_imu_sensor_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_imu_sensor_model
  ${PROJECT_NAME}
)

# IMU sensor utils tests
ament_add_gtest(test_${PROJECT_NAME}_imu_sensor_utils
  test/test_imu_sensor_utils.cpp
)

target_link_libraries(test_${PROJECT_NAME}_imu_sensor_utils
  ${PROJECT_NAME}
)

# Simple drive model tests
ament_add_gtest(test_${PROJECT_NAME}_simple_drive_model
  test/test_simple_drive_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_simple_drive_model
  ${PROJECT_NAME}
)

# Tire model tests
ament_add_gtest(test_${PROJECT_NAME}_tire_model
  test/test_mnc_tire_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_tire_model
  ${PROJECT_NAME}
)

# Sprung mass inclination model tests
ament_add_gtest(test_${PROJECT_NAME}_sprung_mass_inclination_model
  test/test_sprung_mass_inclination_model.cpp
)

target_link_libraries(test_${PROJECT_NAME}_sprung_mass_inclination_model
  ${PROJECT_NAME}
)

# DiscreteTime tests
ament_add_gtest(test_${PROJECT_NAME}_discrete_time
  test/test_discrete_time.cpp
)

target_link_libraries(test_${PROJECT_NAME}_discrete_time
  ${PROJECT_NAME}
)

# WallTime tests
ament_add_gtest(test_${PROJECT_NAME}_wall_time
  test/test_wall_time.cpp
)

target_link_libraries(test_${PROJECT_NAME}_wall_time
  ${PROJECT_NAME}
)

# Simulation clock tests
ament_add_gtest(test_${PROJECT_NAME}_simulation_clock
  test/test_simulation_clock.cpp
)

target_link_libraries(test_${PROJECT_NAME}_simulation_clock
  ${PROJECT_NAME}
)

# PID controller tests
ament_add_gtest(test_${PROJECT_NAME}_pid_controller
  test/test_pid_controller.cpp
)

target_link_libraries(test_${PROJECT_NAME}_pid_controller
  ${PROJECT_NAME}
)

# Second-order smoothing filter tests
ament_add_gtest(test_${PROJECT_NAME}_second_order_smoothing_filter
  test/test_second_order_smoothing_filter.cpp
)

target_link_libraries(test_${PROJECT_NAME}_second_order_smoothing_filter
  ${PROJECT_NAME}
)

# Pure pursuit controller tests
ament_add_gtest(test_${PROJECT_NAME}_pure_pursuit_controller
  test/test_pure_pursuit_controller.cpp
)

target_link_libraries(test_${PROJECT_NAME}_pure_pursuit_controller
  ${PROJECT_NAME}
)

# Trajectory reader tests
ament_add_gtest(test_${PROJECT_NAME}_trajectory_reader
  test/test_trajectory_reader.cpp
)

target_link_libraries(test_${PROJECT_NAME}_trajectory_reader
  ${PROJECT_NAME}
)
