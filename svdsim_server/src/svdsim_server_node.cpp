// Copyright 2025 Ekumen
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

#include "svdsim_server/svdsim_server_node.hpp"

// Standard library
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>

// Non-ROS dependencies
#include <entt/entt.hpp>
#include <zmq.hpp>

// ROS dependencies
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

// Project components
#include <svdsim_server/utils/pure_pursuit_controller.hpp>
#include <svdsim_server/utils/trajectory_reader.hpp>
#include "svdsim_proto/vehicle_control_server.hpp"
#include "svdsim_server/collision/body_handle.hpp"
#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/collision/static_map_handler.hpp"
#include "svdsim_server/configuration/grid_csv_reader.hpp"
#include "svdsim_server/ecs/components/drive_model_component.hpp"
#include "svdsim_server/ecs/components/emulated_steering_servo_sensor_component.hpp"
#include "svdsim_server/ecs/components/groundtruth_sensor_component.hpp"
#include "svdsim_server/ecs/components/imu_sensor_component.hpp"
#include "svdsim_server/ecs/components/lidar_sensor_component.hpp"
#include "svdsim_server/ecs/components/remote_control_server_component.hpp"
#include "svdsim_server/ecs/components/single_track_kinematic_model_component.hpp"
#include "svdsim_server/ecs/components/sparring_controller_component.hpp"
#include "svdsim_server/ecs/components/suspension_simulation_component.hpp"
#include "svdsim_server/ecs/components/true_steering_servo_sensor_component.hpp"
#include "svdsim_server/ecs/components/vehicle_collider_component.hpp"
#include "svdsim_server/ecs/components/wheel_encoder_sensor_component.hpp"
#include "svdsim_server/ecs/systems/sparring_control_system.hpp"
#include "svdsim_server/models/imu_sensor_model.hpp"
#include "svdsim_server/models/lidar_sensor_model.hpp"
#include "svdsim_server/models/mnc_tire_model.hpp"
#include "svdsim_server/models/pulse_encoder_model.hpp"
#include "svdsim_server/models/simple_drive_model.hpp"
#include "svdsim_server/models/single_track_drift_model.hpp"
#include "svdsim_server/models/single_track_kinematic_model.hpp"
#include "svdsim_server/models/single_track_model_interface.hpp"
#include "svdsim_server/models/sprung_mass_inclination_model.hpp"
#include "svdsim_server/utils/map_loader.hpp"

namespace svdsim_server {

SvdsimServerNode::SvdsimServerNode(const rclcpp::NodeOptions& options) : Node("svdsim_server", options) {
  declare_parameters();
  load_parameters();

  // Initialize simulation clock as an entt service
  registry_.ctx().emplace<SimulationClock>(physics_rate_);
  RCLCPP_INFO(this->get_logger(), "Simulation clock initialized at %.1f Hz", physics_rate_);

  // Initialize collision system
  collider_manager_ = std::make_shared<ColliderManager>();
  RCLCPP_INFO(this->get_logger(), "Collider manager initialized");

  // Load static map from file
  try {
    static_map_ = utils::load_map_from_yaml(map_path_);
    RCLCPP_INFO(
        this->get_logger(), "Successfully loaded map: %zux%zu cells at %.3f m/cell resolution", static_map_.width,
        static_map_.height, static_map_.resolution);
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to load map from '" + map_path_ + "': " + e.what());
  }

  static_map_handler_ = std::make_unique<StaticMapHandler>(static_map_, collider_manager_);
  RCLCPP_INFO(this->get_logger(), "Static map handler initialized");

  setup_entities();
  setup_systems();

  start();

  RCLCPP_INFO(this->get_logger(), "SVDSim server node initialized");
  RCLCPP_INFO(this->get_logger(), "Dependencies loaded: Protobuf, EnTT, ZeroMQ");
}

SvdsimServerNode::~SvdsimServerNode() {
  RCLCPP_INFO(this->get_logger(), "Shutting down SVDSim server node");

  // Signal simulation thread to stop
  running_ = false;

  // Wait for simulation thread to finish
  if (simulation_thread_.joinable()) {
    simulation_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Simulation thread joined");
  }
}

void SvdsimServerNode::declare_parameters() {
  // Declare top-level parameters with descriptors
  auto physics_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  physics_rate_desc.description = "Physics simulation update rate [Hz]";
  physics_rate_desc.floating_point_range.resize(1);
  physics_rate_desc.floating_point_range[0].from_value = 1.0;
  physics_rate_desc.floating_point_range[0].to_value = 10000.0;
  this->declare_parameter("physics_rate", 1000.0, physics_rate_desc);

  auto command_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  command_rate_desc.description = "Remote control command processing rate [Hz]";
  command_rate_desc.floating_point_range.resize(1);
  command_rate_desc.floating_point_range[0].from_value = 1.0;
  command_rate_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("command_rate", 20.0, command_rate_desc);

  auto odometry_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  odometry_rate_desc.description = "Wheel encoder odometry update rate [Hz]";
  odometry_rate_desc.floating_point_range.resize(1);
  odometry_rate_desc.floating_point_range[0].from_value = 0.0;
  odometry_rate_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("odometry_rate", 20.0, odometry_rate_desc);

  auto ground_truth_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  ground_truth_rate_desc.description = "Ground truth stream rate [Hz]";
  ground_truth_rate_desc.floating_point_range.resize(1);
  ground_truth_rate_desc.floating_point_range[0].from_value = 0.0;
  ground_truth_rate_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("ground_truth_rate", 100.0, ground_truth_rate_desc);

  auto map_path_desc = rcl_interfaces::msg::ParameterDescriptor{};
  map_path_desc.description = "Path to static map YAML file";
  map_path_desc.read_only = true;
  this->declare_parameter("map_path", "", map_path_desc);

  auto grid_file_desc = rcl_interfaces::msg::ParameterDescriptor{};
  grid_file_desc.description = "Path to vehicle grid CSV file";
  grid_file_desc.read_only = true;
  this->declare_parameter("grid_listing_file", "", grid_file_desc);

  // Declare lidar parameters with descriptors
  auto lidar_fov_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_fov_desc.description = "LiDAR horizontal field of view [rad]";
  lidar_fov_desc.floating_point_range.resize(1);
  lidar_fov_desc.floating_point_range[0].from_value = 0.017;               // ~1 degree
  lidar_fov_desc.floating_point_range[0].to_value = 6.283;                 // 2*pi (360 degrees)
  this->declare_parameter("lidar.horizontal_fov", 4.712, lidar_fov_desc);  // 270 degrees in radians

  auto lidar_res_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_res_desc.description = "LiDAR angular resolution [rad]";
  lidar_res_desc.floating_point_range.resize(1);
  lidar_res_desc.floating_point_range[0].from_value = 0.00017;                      // ~0.01 degrees
  lidar_res_desc.floating_point_range[0].to_value = 0.175;                          // ~10 degrees
  this->declare_parameter("lidar.horizontal_resolution", 0.00873, lidar_res_desc);  // 0.5 degrees in radians

  auto lidar_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_rate_desc.description = "LiDAR scan rate [Hz]";
  lidar_rate_desc.floating_point_range.resize(1);
  lidar_rate_desc.floating_point_range[0].from_value = 0.0;
  lidar_rate_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("lidar.scan_rate", 20.0, lidar_rate_desc);

  auto lidar_max_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_max_dist_desc.description = "LiDAR maximum detection range [m]";
  lidar_max_dist_desc.floating_point_range.resize(1);
  lidar_max_dist_desc.floating_point_range[0].from_value = 0.1;
  lidar_max_dist_desc.floating_point_range[0].to_value = 100.0;
  this->declare_parameter("lidar.max_distance", 10.0, lidar_max_dist_desc);

  auto lidar_min_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_min_dist_desc.description = "LiDAR minimum detection range [m]";
  lidar_min_dist_desc.floating_point_range.resize(1);
  lidar_min_dist_desc.floating_point_range[0].from_value = 0.001;
  lidar_min_dist_desc.floating_point_range[0].to_value = 10.0;
  this->declare_parameter("lidar.min_distance", 0.1, lidar_min_dist_desc);

  auto lidar_noise_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_noise_desc.description = "LiDAR range measurement noise standard deviation [m]";
  lidar_noise_desc.floating_point_range.resize(1);
  lidar_noise_desc.floating_point_range[0].from_value = 0.0;
  lidar_noise_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("lidar.range_noise_stddev", 0.01, lidar_noise_desc);

  auto lidar_skew_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_skew_desc.description = "Enable motion compensation for LiDAR scan skew";
  this->declare_parameter("lidar.enable_skew", true, lidar_skew_desc);

  auto lidar_x_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_x_desc.description = "LiDAR X offset from vehicle center [m]";
  this->declare_parameter("lidar.x_offset", 0.0, lidar_x_desc);

  auto lidar_y_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_y_desc.description = "LiDAR Y offset from vehicle center [m]";
  this->declare_parameter("lidar.y_offset", 0.0, lidar_y_desc);

  auto lidar_yaw_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_yaw_desc.description = "LiDAR yaw offset from vehicle heading [rad]";
  lidar_yaw_desc.floating_point_range.resize(1);
  lidar_yaw_desc.floating_point_range[0].from_value = -3.142;  // -180 degrees
  lidar_yaw_desc.floating_point_range[0].to_value = 3.142;     // +180 degrees
  this->declare_parameter("lidar.yaw_offset", 0.0, lidar_yaw_desc);

  // Declare IMU sensor parameters with descriptors
  auto imu_rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_rate_desc.description = "IMU sensor sample rate [Hz]";
  imu_rate_desc.floating_point_range.resize(1);
  imu_rate_desc.floating_point_range[0].from_value = 0.0;
  imu_rate_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("imu.sample_rate", 100.0, imu_rate_desc);

  auto imu_x_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_x_desc.description = "IMU X offset in body frame [m]";
  this->declare_parameter("imu.x_offset", 0.0, imu_x_desc);

  auto imu_y_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_y_desc.description = "IMU Y offset in body frame [m]";
  this->declare_parameter("imu.y_offset", 0.0, imu_y_desc);

  auto imu_z_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_z_desc.description = "IMU Z offset in body frame [m]";
  this->declare_parameter("imu.z_offset", 0.05, imu_z_desc);

  auto imu_roll_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_roll_desc.description = "IMU roll offset in body frame [rad]";
  this->declare_parameter("imu.roll_offset", 0.0, imu_roll_desc);

  auto imu_pitch_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_pitch_desc.description = "IMU pitch offset in body frame [rad]";
  this->declare_parameter("imu.pitch_offset", 0.0, imu_pitch_desc);

  auto imu_yaw_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_yaw_desc.description = "IMU yaw offset in body frame [rad]";
  this->declare_parameter("imu.yaw_offset", 0.0, imu_yaw_desc);

  auto imu_gravity_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_gravity_desc.description = "Gravity magnitude for IMU simulation [m/s^2]";
  imu_gravity_desc.floating_point_range.resize(1);
  imu_gravity_desc.floating_point_range[0].from_value = 0.0;
  imu_gravity_desc.floating_point_range[0].to_value = 100.0;
  this->declare_parameter("imu.gravity", 9.81, imu_gravity_desc);

  auto accel_noise_desc = rcl_interfaces::msg::ParameterDescriptor{};
  accel_noise_desc.description = "Accelerometer noise density [m/s^2/sqrt(Hz)]";
  this->declare_parameter("imu.accelerometer_noise_density", 1.40e-02, accel_noise_desc);

  auto accel_walk_desc = rcl_interfaces::msg::ParameterDescriptor{};
  accel_walk_desc.description = "Accelerometer random walk [m/s^3/sqrt(Hz)]";
  this->declare_parameter("imu.accelerometer_random_walk", 4.45e-04, accel_walk_desc);

  auto gyro_noise_desc = rcl_interfaces::msg::ParameterDescriptor{};
  gyro_noise_desc.description = "Gyroscope noise density [rad/s/sqrt(Hz)]";
  this->declare_parameter("imu.gyroscope_noise_density", 1.75e-03, gyro_noise_desc);

  auto gyro_walk_desc = rcl_interfaces::msg::ParameterDescriptor{};
  gyro_walk_desc.description = "Gyroscope random walk [rad/s^2/sqrt(Hz)]";
  this->declare_parameter("imu.gyroscope_random_walk", 1.05e-05, gyro_walk_desc);

  auto mag_noise_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mag_noise_desc.description = "Magnetometer noise density [T/sqrt(Hz)]";
  this->declare_parameter("imu.magnetometer_noise_density", 1.35e-06, mag_noise_desc);

  auto accel_tau_desc = rcl_interfaces::msg::ParameterDescriptor{};
  accel_tau_desc.description = "Accelerometer bias autocorrelation time [s]";
  this->declare_parameter("imu.accelerometer_tau", 3600.0, accel_tau_desc);

  auto gyro_tau_desc = rcl_interfaces::msg::ParameterDescriptor{};
  gyro_tau_desc.description = "Gyroscope bias autocorrelation time [s]";
  this->declare_parameter("imu.gyroscope_tau", 3600.0, gyro_tau_desc);

  auto mag_tau_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mag_tau_desc.description = "Magnetometer bias autocorrelation time [s]";
  this->declare_parameter("imu.magnetometer_tau", 300.0, mag_tau_desc);

  auto accel_init_bias_desc = rcl_interfaces::msg::ParameterDescriptor{};
  accel_init_bias_desc.description = "Initial accelerometer bias [m/s^2]";
  this->declare_parameter("imu.initial_accelerometer_bias_x", 0.0, accel_init_bias_desc);
  this->declare_parameter("imu.initial_accelerometer_bias_y", 0.0, accel_init_bias_desc);
  this->declare_parameter("imu.initial_accelerometer_bias_z", 0.0, accel_init_bias_desc);

  auto gyro_init_bias_desc = rcl_interfaces::msg::ParameterDescriptor{};
  gyro_init_bias_desc.description = "Initial gyroscope bias [rad/s]";
  this->declare_parameter("imu.initial_gyroscope_bias_x", 0.0, gyro_init_bias_desc);
  this->declare_parameter("imu.initial_gyroscope_bias_y", 0.0, gyro_init_bias_desc);
  this->declare_parameter("imu.initial_gyroscope_bias_z", 0.0, gyro_init_bias_desc);

  auto mag_init_bias_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mag_init_bias_desc.description = "Initial magnetometer bias [T]";
  this->declare_parameter("imu.initial_magnetometer_bias_x", 0.0, mag_init_bias_desc);
  this->declare_parameter("imu.initial_magnetometer_bias_y", 0.0, mag_init_bias_desc);
  this->declare_parameter("imu.initial_magnetometer_bias_z", 0.0, mag_init_bias_desc);

  // Declare wheel encoder parameters with descriptors
  auto encoder_ppr_desc = rcl_interfaces::msg::ParameterDescriptor{};
  encoder_ppr_desc.description = "Wheel encoder pulses per revolution";
  encoder_ppr_desc.floating_point_range.resize(1);
  encoder_ppr_desc.floating_point_range[0].from_value = 1.0;
  encoder_ppr_desc.floating_point_range[0].to_value = 100000.0;
  this->declare_parameter("wheel_encoders.pulses_per_revolution", 4096.0, encoder_ppr_desc);

  auto encoder_ratio_desc = rcl_interfaces::msg::ParameterDescriptor{};
  encoder_ratio_desc.description = "Wheel encoder gear/conversion ratio";
  encoder_ratio_desc.floating_point_range.resize(1);
  encoder_ratio_desc.floating_point_range[0].from_value = 0.01;
  encoder_ratio_desc.floating_point_range[0].to_value = 100.0;
  this->declare_parameter("wheel_encoders.conversion_ratio", 1.0, encoder_ratio_desc);

  auto use_true_steering_feedback_desc = rcl_interfaces::msg::ParameterDescriptor{};
  use_true_steering_feedback_desc.description =
      "When true, report the true simulated steering angle as feedback; when false, report the last commanded "
      "steering reference instead";
  this->declare_parameter("use_true_steering_feedback_sensor", true, use_true_steering_feedback_desc);

  // Declare kinematic single track model parameters with descriptors
  auto wheelbase_desc = rcl_interfaces::msg::ParameterDescriptor{};
  wheelbase_desc.description = "Vehicle wheelbase [m]";
  wheelbase_desc.floating_point_range.resize(1);
  wheelbase_desc.floating_point_range[0].from_value = 0.1;
  wheelbase_desc.floating_point_range[0].to_value = 5.0;
  this->declare_parameter("vehicle_dynamics.wheelbase", 0.3, wheelbase_desc);

  auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
  track_width_desc.description = "Rear track width [m]";
  track_width_desc.floating_point_range.resize(1);
  track_width_desc.floating_point_range[0].from_value = 0.05;
  track_width_desc.floating_point_range[0].to_value = 2.0;
  this->declare_parameter("vehicle_dynamics.rear_track_width", 0.2, track_width_desc);

  auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
  wheel_radius_desc.description = "Effective wheel radius [m]";
  wheel_radius_desc.floating_point_range.resize(1);
  wheel_radius_desc.floating_point_range[0].from_value = 0.01;
  wheel_radius_desc.floating_point_range[0].to_value = 0.5;
  this->declare_parameter("vehicle_dynamics.effective_wheel_radius", 0.05, wheel_radius_desc);

  auto sprung_mass_desc = rcl_interfaces::msg::ParameterDescriptor{};
  sprung_mass_desc.description = "Sprung mass (chassis mass supported by suspension) [kg]";
  sprung_mass_desc.floating_point_range.resize(1);
  sprung_mass_desc.floating_point_range[0].from_value = 0.1;
  sprung_mass_desc.floating_point_range[0].to_value = 1000.0;
  this->declare_parameter("vehicle_dynamics.sprung_mass", 3.47, sprung_mass_desc);

  auto unsprung_mass_desc = rcl_interfaces::msg::ParameterDescriptor{};
  unsprung_mass_desc.description = "Unsprung mass per tire [kg]";
  unsprung_mass_desc.floating_point_range.resize(1);
  unsprung_mass_desc.floating_point_range[0].from_value = 0.01;
  unsprung_mass_desc.floating_point_range[0].to_value = 100.0;
  this->declare_parameter("vehicle_dynamics.unsprung_mass", 0.109, unsprung_mass_desc);

  auto front_track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
  front_track_width_desc.description = "Front track width [m]";
  front_track_width_desc.floating_point_range.resize(1);
  front_track_width_desc.floating_point_range[0].from_value = 0.05;
  front_track_width_desc.floating_point_range[0].to_value = 2.0;
  this->declare_parameter("vehicle_dynamics.front_track_width", 0.2, front_track_width_desc);

  auto cog_x_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cog_x_desc.description = "Center of mass X position relative to rear axle [m]";
  this->declare_parameter("vehicle_dynamics.center_of_mass_x", 0.15, cog_x_desc);

  auto cog_z_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cog_z_desc.description = "Center of mass Z position above ground [m]";
  cog_z_desc.floating_point_range.resize(1);
  cog_z_desc.floating_point_range[0].from_value = 0.0;
  cog_z_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("vehicle_dynamics.center_of_mass_z", 0.04684, cog_z_desc);

  auto steer_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
  steer_max_desc.description = "Maximum steering angle [rad]";
  steer_max_desc.floating_point_range.resize(1);
  steer_max_desc.floating_point_range[0].from_value = 0.0;
  steer_max_desc.floating_point_range[0].to_value = 1.57;
  this->declare_parameter("vehicle_dynamics.steering_max", 0.6, steer_max_desc);

  auto steer_min_desc = rcl_interfaces::msg::ParameterDescriptor{};
  steer_min_desc.description = "Minimum steering angle [rad]";
  steer_min_desc.floating_point_range.resize(1);
  steer_min_desc.floating_point_range[0].from_value = -1.57;
  steer_min_desc.floating_point_range[0].to_value = 0.0;
  this->declare_parameter("vehicle_dynamics.steering_min", -0.6, steer_min_desc);

  auto steer_vel_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
  steer_vel_max_desc.description = "Maximum steering velocity [rad/s]";
  steer_vel_max_desc.floating_point_range.resize(1);
  steer_vel_max_desc.floating_point_range[0].from_value = 0.0;
  steer_vel_max_desc.floating_point_range[0].to_value = 10.0;
  this->declare_parameter("vehicle_dynamics.steering_velocity_max", 3.2, steer_vel_max_desc);

  auto steer_hysteresis_gap_desc = rcl_interfaces::msg::ParameterDescriptor{};
  steer_hysteresis_gap_desc.description =
      "Steering hysteresis loop width [rad]; the effective steering reference lags the commanded "
      "reference by half this value, and commands within half the gap of the current angle are ignored.";
  steer_hysteresis_gap_desc.floating_point_range.resize(1);
  steer_hysteresis_gap_desc.floating_point_range[0].from_value = 0.0;
  steer_hysteresis_gap_desc.floating_point_range[0].to_value = 1.57;
  this->declare_parameter("vehicle_dynamics.steering_hysteresis_gap", 0.0, steer_hysteresis_gap_desc);

  auto accel_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
  accel_max_desc.description = "Maximum vehicle acceleration [m/s²]";
  accel_max_desc.floating_point_range.resize(1);
  accel_max_desc.floating_point_range[0].from_value = 0.0;
  accel_max_desc.floating_point_range[0].to_value = 20.0;
  this->declare_parameter("vehicle_dynamics.acceleration_max", 7.0, accel_max_desc);

  auto vel_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
  vel_max_desc.description = "Maximum vehicle velocity [m/s]";
  vel_max_desc.floating_point_range.resize(1);
  vel_max_desc.floating_point_range[0].from_value = 0.0;
  vel_max_desc.floating_point_range[0].to_value = 50.0;
  this->declare_parameter("vehicle_dynamics.velocity_max", 5.0, vel_max_desc);

  auto vel_min_desc = rcl_interfaces::msg::ParameterDescriptor{};
  vel_min_desc.description = "Minimum vehicle velocity (reverse) [m/s]";
  vel_min_desc.floating_point_range.resize(1);
  vel_min_desc.floating_point_range[0].from_value = -10.0;
  vel_min_desc.floating_point_range[0].to_value = 0.0;
  this->declare_parameter("vehicle_dynamics.velocity_min", -2.0, vel_min_desc);

  auto vel_switch_desc = rcl_interfaces::msg::ParameterDescriptor{};
  vel_switch_desc.description = "Velocity switch threshold for kinematic model [m/s]";
  vel_switch_desc.floating_point_range.resize(1);
  vel_switch_desc.floating_point_range[0].from_value = 0.0;
  vel_switch_desc.floating_point_range[0].to_value = 20.0;
  this->declare_parameter("vehicle_dynamics.velocity_switch", 3.5, vel_switch_desc);

  auto dynamic_model_desc = rcl_interfaces::msg::ParameterDescriptor{};
  dynamic_model_desc.description = "Vehicle dynamics model type: 'single_track_kinematic' or 'single_track_drift'";
  this->declare_parameter("vehicle_dynamics.dynamic_model", "single_track_kinematic", dynamic_model_desc);

  auto yaw_inertia_desc = rcl_interfaces::msg::ParameterDescriptor{};
  yaw_inertia_desc.description = "Vehicle yaw (Z-axis) moment of inertia [kg*m^2]";
  yaw_inertia_desc.floating_point_range.resize(1);
  yaw_inertia_desc.floating_point_range[0].from_value = 0.001;
  yaw_inertia_desc.floating_point_range[0].to_value = 10.0;
  this->declare_parameter("vehicle_dynamics.yaw_inertia", 0.04712, yaw_inertia_desc);

  auto wheel_inertia_desc = rcl_interfaces::msg::ParameterDescriptor{};
  wheel_inertia_desc.description = "Single wheel rotational moment of inertia [kg*m^2]";
  wheel_inertia_desc.floating_point_range.resize(1);
  wheel_inertia_desc.floating_point_range[0].from_value = 0.00001;
  wheel_inertia_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("vehicle_dynamics.wheel_inertia", 0.00028, wheel_inertia_desc);

  auto engine_torque_split_desc = rcl_interfaces::msg::ParameterDescriptor{};
  engine_torque_split_desc.description = "Fraction of engine torque applied to front axle [0.0-1.0]";
  engine_torque_split_desc.floating_point_range.resize(1);
  engine_torque_split_desc.floating_point_range[0].from_value = 0.0;
  engine_torque_split_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("vehicle_dynamics.engine_torque_split", 0.5, engine_torque_split_desc);

  auto brake_torque_split_desc = rcl_interfaces::msg::ParameterDescriptor{};
  brake_torque_split_desc.description = "Fraction of brake torque applied to front axle [0.0-1.0]";
  brake_torque_split_desc.floating_point_range.resize(1);
  brake_torque_split_desc.floating_point_range[0].from_value = 0.0;
  brake_torque_split_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("vehicle_dynamics.brake_torque_split", 0.5, brake_torque_split_desc);

  // Declare simple linear controller parameters with descriptors
  auto braking_deceleration_desc = rcl_interfaces::msg::ParameterDescriptor{};
  braking_deceleration_desc.description = "Braking deceleration magnitude [m/s^2]";
  this->declare_parameter("vehicle_dynamics.braking_deceleration", 9.0, braking_deceleration_desc);

  auto motor_drive_rise_time_desc = rcl_interfaces::msg::ParameterDescriptor{};
  motor_drive_rise_time_desc.description = "Rise time for velocity reference smoothing filter [s]";
  motor_drive_rise_time_desc.floating_point_range.resize(1);
  motor_drive_rise_time_desc.floating_point_range[0].from_value = 0.001;
  motor_drive_rise_time_desc.floating_point_range[0].to_value = 1.0;
  this->declare_parameter("vehicle_dynamics.motor_drive_rise_time", 0.1, motor_drive_rise_time_desc);

  auto velocity_deadband_desc = rcl_interfaces::msg::ParameterDescriptor{};
  velocity_deadband_desc.description = "Velocity dead band magnitude [m/s]; references below this are treated as zero";
  this->declare_parameter("vehicle_dynamics.velocity_deadband", 0.0, velocity_deadband_desc);

  // Declare tire model parameters with descriptors
  auto tire_model_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_model_desc.description = "Tire model type: 'modified_nicolas_comstock' (Modified Nicolas-Comstock)";
  this->declare_parameter("tires.model", "modified_nicolas_comstock", tire_model_desc);

  auto tire_combined_slip_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_combined_slip_desc.description = "Enable combined slip tire model";
  this->declare_parameter("tires.combined_slip", true, tire_combined_slip_desc);

  auto tire_lat_B_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_lat_B_desc.description = "Lateral Pacejka stiffness factor [1/deg]";
  this->declare_parameter("tires.lateral.B", 0.1, tire_lat_B_desc);

  auto tire_lat_C_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_lat_C_desc.description = "Lateral Pacejka shape factor";
  this->declare_parameter("tires.lateral.C", 1.7, tire_lat_C_desc);

  auto tire_lat_D_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_lat_D_desc.description = "Lateral Pacejka normalized peak factor (dimensionless)";
  this->declare_parameter("tires.lateral.Dn", 1.0, tire_lat_D_desc);

  auto tire_lat_E_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_lat_E_desc.description = "Lateral Pacejka curvature factor";
  this->declare_parameter("tires.lateral.E", -1.6, tire_lat_E_desc);

  auto tire_long_B_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_long_B_desc.description = "Longitudinal Pacejka stiffness factor [1/slip ratio]";
  this->declare_parameter("tires.longitudinal.B", 8.0, tire_long_B_desc);

  auto tire_long_C_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_long_C_desc.description = "Longitudinal Pacejka shape factor";
  this->declare_parameter("tires.longitudinal.C", 2.1, tire_long_C_desc);

  auto tire_long_D_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_long_D_desc.description = "Longitudinal Pacejka normalized peak factor (dimensionless)";
  this->declare_parameter("tires.longitudinal.Dn", 1.0, tire_long_D_desc);

  auto tire_long_E_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tire_long_E_desc.description = "Longitudinal Pacejka curvature factor";
  this->declare_parameter("tires.longitudinal.E", 0.97, tire_long_E_desc);

  // Declare suspension simulation parameters
  auto spring_stiffness_desc = rcl_interfaces::msg::ParameterDescriptor{};
  spring_stiffness_desc.description = "Suspension spring stiffness [N/m]";
  spring_stiffness_desc.floating_point_range.resize(1);
  spring_stiffness_desc.floating_point_range[0].from_value = 1.0;
  spring_stiffness_desc.floating_point_range[0].to_value = 10000.0;
  this->declare_parameter("suspension.spring_stiffness", 500.0, spring_stiffness_desc);

  auto roll_center_height_desc = rcl_interfaces::msg::ParameterDescriptor{};
  roll_center_height_desc.description = "Roll center height above ground [m]";
  roll_center_height_desc.floating_point_range.resize(1);
  roll_center_height_desc.floating_point_range[0].from_value = -0.5;
  roll_center_height_desc.floating_point_range[0].to_value = 0.5;
  this->declare_parameter("suspension.roll_center_height", 0.0325, roll_center_height_desc);

  auto max_inclination_desc = rcl_interfaces::msg::ParameterDescriptor{};
  max_inclination_desc.description = "Maximum body inclination angle [rad]";
  max_inclination_desc.floating_point_range.resize(1);
  max_inclination_desc.floating_point_range[0].from_value = 0.0;
  max_inclination_desc.floating_point_range[0].to_value = 0.785398;  // 45 degrees
  this->declare_parameter("suspension.max_inclination_angle", 0.174533, max_inclination_desc);

  // Declare vehicle footprint parameters with descriptors
  auto footprint_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
  footprint_width_desc.description = "Vehicle width for collision detection [m]";
  footprint_width_desc.floating_point_range.resize(1);
  footprint_width_desc.floating_point_range[0].from_value = 0.05;
  footprint_width_desc.floating_point_range[0].to_value = 3.0;
  this->declare_parameter("vehicle_footprint.width", 0.3, footprint_width_desc);

  auto footprint_front_desc = rcl_interfaces::msg::ParameterDescriptor{};
  footprint_front_desc.description = "Vehicle front distance from center [m]";
  footprint_front_desc.floating_point_range.resize(1);
  footprint_front_desc.floating_point_range[0].from_value = 0.01;
  footprint_front_desc.floating_point_range[0].to_value = 5.0;
  this->declare_parameter("vehicle_footprint.front_distance", 0.3, footprint_front_desc);

  auto footprint_rear_desc = rcl_interfaces::msg::ParameterDescriptor{};
  footprint_rear_desc.description = "Vehicle rear distance from center [m]";
  footprint_rear_desc.floating_point_range.resize(1);
  footprint_rear_desc.floating_point_range[0].from_value = 0.01;
  footprint_rear_desc.floating_point_range[0].to_value = 5.0;
  this->declare_parameter("vehicle_footprint.rear_distance", 0.3, footprint_rear_desc);

  // Declare sparring vehicle parameters with descriptors
  auto sparring_traj_desc = rcl_interfaces::msg::ParameterDescriptor{};
  sparring_traj_desc.description = "Path to sparring vehicle trajectory CSV file";
  this->declare_parameter("sparring.trajectory_file", "", sparring_traj_desc);

  auto sparring_vel_desc = rcl_interfaces::msg::ParameterDescriptor{};
  sparring_vel_desc.description = "Sparring vehicle constant velocity [m/s]";
  sparring_vel_desc.floating_point_range.resize(1);
  sparring_vel_desc.floating_point_range[0].from_value = 0.0;
  sparring_vel_desc.floating_point_range[0].to_value = 10.0;
  this->declare_parameter("sparring.constant_velocity", 2.5, sparring_vel_desc);

  auto sparring_lookahead_desc = rcl_interfaces::msg::ParameterDescriptor{};
  sparring_lookahead_desc.description = "Sparring vehicle lookahead distance [m]";
  sparring_lookahead_desc.floating_point_range.resize(1);
  sparring_lookahead_desc.floating_point_range[0].from_value = 0.1;
  sparring_lookahead_desc.floating_point_range[0].to_value = 5.0;
  this->declare_parameter("sparring.lookahead_distance", 0.8, sparring_lookahead_desc);

  auto sparring_max_steer_desc = rcl_interfaces::msg::ParameterDescriptor{};
  sparring_max_steer_desc.description = "Sparring vehicle maximum steering angle [rad]";
  sparring_max_steer_desc.floating_point_range.resize(1);
  sparring_max_steer_desc.floating_point_range[0].from_value = 0.1;
  sparring_max_steer_desc.floating_point_range[0].to_value = 1.57;
  this->declare_parameter("sparring.max_steering", 0.6, sparring_max_steer_desc);

  auto rtf_report_interval_desc = rcl_interfaces::msg::ParameterDescriptor{};
  rtf_report_interval_desc.description = "Interval for real-time factor reporting [s]";
  rtf_report_interval_desc.floating_point_range.resize(1);
  rtf_report_interval_desc.floating_point_range[0].from_value = 0.1;
  rtf_report_interval_desc.floating_point_range[0].to_value = 60.0;
  this->declare_parameter("rtf_report_interval", 3.0, rtf_report_interval_desc);
}

void SvdsimServerNode::load_parameters() {
  // Load top-level parameters
  physics_rate_ = this->get_parameter("physics_rate").as_double();
  command_rate_ = this->get_parameter("command_rate").as_double();
  odometry_rate_ = this->get_parameter("odometry_rate").as_double();
  ground_truth_rate_ = this->get_parameter("ground_truth_rate").as_double();
  rtf_report_interval_ = this->get_parameter("rtf_report_interval").as_double();
  map_path_ = this->get_parameter("map_path").as_string();
  grid_listing_file_ = this->get_parameter("grid_listing_file").as_string();

  // Validate required parameters
  if (map_path_.empty()) {
    throw std::runtime_error("Required parameter 'map_path' is missing or empty");
  }
  if (grid_listing_file_.empty()) {
    throw std::runtime_error("Required parameter 'grid_listing_file' is missing or empty");
  }

  // Load vehicle grid from CSV file
  std::ifstream grid_file(grid_listing_file_);
  if (!grid_file.is_open()) {
    throw std::runtime_error("Failed to open grid listing file: " + grid_listing_file_);
  }
  try {
    GridCsvReader reader;
    vehicle_grid_entries_ = reader << grid_file;
    RCLCPP_INFO(
        this->get_logger(), "Loaded %zu vehicle(s) from grid listing: %s", vehicle_grid_entries_.size(),
        grid_listing_file_.c_str());
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to parse grid listing file '" + grid_listing_file_ + "': " + e.what());
  }

  // Validate that system rates are integer dividers of physics rate (cross-parameter validation)
  const auto command_period = physics_rate_ / command_rate_;
  if (command_period != std::floor(command_period)) {
    throw std::runtime_error(
        "Parameter 'command_rate' must be an integer divider of 'physics_rate'. " + std::to_string(physics_rate_) +
        " / " + std::to_string(command_rate_) + " = " + std::to_string(command_period) + " (not an integer)");
  }
  const auto odometry_period = physics_rate_ / odometry_rate_;
  if (odometry_period != std::floor(odometry_period)) {
    throw std::runtime_error(
        "Parameter 'odometry_rate' must be an integer divider of 'physics_rate'. " + std::to_string(physics_rate_) +
        " / " + std::to_string(odometry_rate_) + " = " + std::to_string(odometry_period) + " (not an integer)");
  }
  const auto ground_truth_period = physics_rate_ / ground_truth_rate_;
  if (ground_truth_period != std::floor(ground_truth_period)) {
    throw std::runtime_error(
        "Parameter 'ground_truth_rate' must be an integer divider of 'physics_rate'. " + std::to_string(physics_rate_) +
        " / " + std::to_string(ground_truth_rate_) + " = " + std::to_string(ground_truth_period) + " (not an integer)");
  }

  // Load vehicle dynamics parameters
  dynamic_model_ = this->get_parameter("vehicle_dynamics.dynamic_model").as_string();
  wheelbase_ = this->get_parameter("vehicle_dynamics.wheelbase").as_double();
  rear_track_width_ = this->get_parameter("vehicle_dynamics.rear_track_width").as_double();
  front_track_width_ = this->get_parameter("vehicle_dynamics.front_track_width").as_double();
  effective_wheel_radius_ = this->get_parameter("vehicle_dynamics.effective_wheel_radius").as_double();
  sprung_mass_ = this->get_parameter("vehicle_dynamics.sprung_mass").as_double();
  unsprung_mass_ = this->get_parameter("vehicle_dynamics.unsprung_mass").as_double();
  center_of_mass_x_ = this->get_parameter("vehicle_dynamics.center_of_mass_x").as_double();
  center_of_mass_z_ = this->get_parameter("vehicle_dynamics.center_of_mass_z").as_double();
  yaw_inertia_ = this->get_parameter("vehicle_dynamics.yaw_inertia").as_double();
  wheel_inertia_ = this->get_parameter("vehicle_dynamics.wheel_inertia").as_double();
  engine_torque_split_ = this->get_parameter("vehicle_dynamics.engine_torque_split").as_double();
  brake_torque_split_ = this->get_parameter("vehicle_dynamics.brake_torque_split").as_double();
  steering_max_ = this->get_parameter("vehicle_dynamics.steering_max").as_double();
  steering_velocity_max_ = this->get_parameter("vehicle_dynamics.steering_velocity_max").as_double();
  steering_hysteresis_gap_ = this->get_parameter("vehicle_dynamics.steering_hysteresis_gap").as_double();
  acceleration_max_ = this->get_parameter("vehicle_dynamics.acceleration_max").as_double();
  braking_deceleration_ = this->get_parameter("vehicle_dynamics.braking_deceleration").as_double();
  motor_drive_rise_time_ = this->get_parameter("vehicle_dynamics.motor_drive_rise_time").as_double();
  velocity_deadband_ = this->get_parameter("vehicle_dynamics.velocity_deadband").as_double();
  velocity_max_ = this->get_parameter("vehicle_dynamics.velocity_max").as_double();
  velocity_min_ = this->get_parameter("vehicle_dynamics.velocity_min").as_double();
  velocity_switch_ = this->get_parameter("vehicle_dynamics.velocity_switch").as_double();

  // Validate dynamic model selection
  if (dynamic_model_ != "single_track_kinematic" && dynamic_model_ != "single_track_drift") {
    throw std::runtime_error(
        "Invalid parameter 'vehicle_dynamics.dynamic_model': '" + dynamic_model_ +
        "'. Must be 'single_track_kinematic' or 'single_track_drift'");
  }

  // Load tire model parameters
  tire_model_ = this->get_parameter("tires.model").as_string();
  tire_combined_slip_ = this->get_parameter("tires.combined_slip").as_bool();
  tire_lateral_stiffness_factor_B_ = this->get_parameter("tires.lateral.B").as_double();
  tire_lateral_shape_factor_C_ = this->get_parameter("tires.lateral.C").as_double();
  tire_lateral_peak_factor_D_ = this->get_parameter("tires.lateral.Dn").as_double();
  tire_lateral_curvature_factor_E_ = this->get_parameter("tires.lateral.E").as_double();
  tire_longitudinal_stiffness_factor_B_ = this->get_parameter("tires.longitudinal.B").as_double();
  tire_longitudinal_shape_factor_C_ = this->get_parameter("tires.longitudinal.C").as_double();
  tire_longitudinal_peak_factor_D_ = this->get_parameter("tires.longitudinal.Dn").as_double();
  tire_longitudinal_curvature_factor_E_ = this->get_parameter("tires.longitudinal.E").as_double();

  // Validate tire model selection
  if (tire_model_ != "modified_nicolas_comstock") {
    throw std::runtime_error(
        "Invalid parameter 'tires.model': '" + tire_model_ +
        "'. Currently only 'modified_nicolas_comstock' is supported");
  }

  // Load suspension simulation parameters
  suspension_params_.spring_stiffness = this->get_parameter("suspension.spring_stiffness").as_double();
  suspension_params_.roll_center_height = this->get_parameter("suspension.roll_center_height").as_double();
  suspension_params_.max_inclination_angle = this->get_parameter("suspension.max_inclination_angle").as_double();

  // Load vehicle footprint parameters
  footprint_width_ = this->get_parameter("vehicle_footprint.width").as_double();
  footprint_front_distance_ = this->get_parameter("vehicle_footprint.front_distance").as_double();
  footprint_rear_distance_ = this->get_parameter("vehicle_footprint.rear_distance").as_double();

  // Load lidar parameters
  lidar_rate_ = this->get_parameter("lidar.scan_rate").as_double();
  lidar_horizontal_fov_ = this->get_parameter("lidar.horizontal_fov").as_double();
  lidar_horizontal_resolution_ = this->get_parameter("lidar.horizontal_resolution").as_double();
  lidar_max_distance_ = this->get_parameter("lidar.max_distance").as_double();
  lidar_min_distance_ = this->get_parameter("lidar.min_distance").as_double();
  lidar_range_noise_stddev_ = this->get_parameter("lidar.range_noise_stddev").as_double();
  lidar_enable_skew_ = this->get_parameter("lidar.enable_skew").as_bool();
  lidar_x_offset_ = this->get_parameter("lidar.x_offset").as_double();
  lidar_y_offset_ = this->get_parameter("lidar.y_offset").as_double();
  lidar_yaw_offset_ = this->get_parameter("lidar.yaw_offset").as_double();

  // Load IMU sensor parameters
  imu_rate_ = this->get_parameter("imu.sample_rate").as_double();
  imu_x_offset_ = this->get_parameter("imu.x_offset").as_double();
  imu_y_offset_ = this->get_parameter("imu.y_offset").as_double();
  imu_z_offset_ = this->get_parameter("imu.z_offset").as_double();
  imu_roll_offset_ = this->get_parameter("imu.roll_offset").as_double();
  imu_pitch_offset_ = this->get_parameter("imu.pitch_offset").as_double();
  imu_yaw_offset_ = this->get_parameter("imu.yaw_offset").as_double();
  imu_gravity_ = this->get_parameter("imu.gravity").as_double();
  imu_accelerometer_noise_density_ = this->get_parameter("imu.accelerometer_noise_density").as_double();
  imu_accelerometer_random_walk_ = this->get_parameter("imu.accelerometer_random_walk").as_double();
  imu_gyroscope_noise_density_ = this->get_parameter("imu.gyroscope_noise_density").as_double();
  imu_gyroscope_random_walk_ = this->get_parameter("imu.gyroscope_random_walk").as_double();
  imu_magnetometer_noise_density_ = this->get_parameter("imu.magnetometer_noise_density").as_double();
  imu_accelerometer_tau_ = this->get_parameter("imu.accelerometer_tau").as_double();
  imu_gyroscope_tau_ = this->get_parameter("imu.gyroscope_tau").as_double();
  imu_magnetometer_tau_ = this->get_parameter("imu.magnetometer_tau").as_double();
  imu_initial_accelerometer_bias_x_ = this->get_parameter("imu.initial_accelerometer_bias_x").as_double();
  imu_initial_accelerometer_bias_y_ = this->get_parameter("imu.initial_accelerometer_bias_y").as_double();
  imu_initial_accelerometer_bias_z_ = this->get_parameter("imu.initial_accelerometer_bias_z").as_double();
  imu_initial_gyroscope_bias_x_ = this->get_parameter("imu.initial_gyroscope_bias_x").as_double();
  imu_initial_gyroscope_bias_y_ = this->get_parameter("imu.initial_gyroscope_bias_y").as_double();
  imu_initial_gyroscope_bias_z_ = this->get_parameter("imu.initial_gyroscope_bias_z").as_double();
  imu_initial_magnetometer_bias_x_ = this->get_parameter("imu.initial_magnetometer_bias_x").as_double();
  imu_initial_magnetometer_bias_y_ = this->get_parameter("imu.initial_magnetometer_bias_y").as_double();
  imu_initial_magnetometer_bias_z_ = this->get_parameter("imu.initial_magnetometer_bias_z").as_double();

  // Validate that IMU rate is an integer divider of physics rate
  const auto imu_period = physics_rate_ / imu_rate_;
  if (imu_period != std::floor(imu_period)) {
    throw std::runtime_error(
        "Parameter 'imu.sample_rate' must be an integer divider of 'physics_rate'. " + std::to_string(physics_rate_) +
        " / " + std::to_string(imu_rate_) + " = " + std::to_string(imu_period) + " (not an integer)");
  }

  // Validate lidar rate is an integer divider of physics rate (cross-parameter validation)
  const auto lidar_period = physics_rate_ / lidar_rate_;
  if (lidar_period != std::floor(lidar_period)) {
    throw std::runtime_error(
        "Parameter 'lidar.scan_rate' must be an integer divider of 'physics_rate'. " + std::to_string(physics_rate_) +
        " / " + std::to_string(lidar_rate_) + " = " + std::to_string(lidar_period) + " (not an integer)");
  }

  // Validate lidar min/max range relationship (cross-parameter validation)
  if (lidar_max_distance_ <= lidar_min_distance_) {
    throw std::runtime_error(
        "Parameter 'lidar.max_distance' must be greater than 'lidar.min_distance', got: " +
        std::to_string(lidar_max_distance_) + " <= " + std::to_string(lidar_min_distance_));
  }

  // Load wheel encoder parameters
  wheel_encoder_pulses_per_revolution_ = this->get_parameter("wheel_encoders.pulses_per_revolution").as_double();
  wheel_encoder_conversion_ratio_ = this->get_parameter("wheel_encoders.conversion_ratio").as_double();

  use_true_steering_feedback_sensor_ = this->get_parameter("use_true_steering_feedback_sensor").as_bool();

  // Load sparring vehicle parameters
  sparring_trajectory_file_ = this->get_parameter("sparring.trajectory_file").as_string();
  sparring_constant_velocity_ = this->get_parameter("sparring.constant_velocity").as_double();
  sparring_lookahead_distance_ = this->get_parameter("sparring.lookahead_distance").as_double();
  sparring_max_steering_ = this->get_parameter("sparring.max_steering").as_double();

  // Validate vehicle dynamics velocity relationships (cross-parameter validation)
  if (velocity_max_ <= velocity_min_) {
    throw std::runtime_error(
        "Parameter 'velocity_max' must be greater than 'velocity_min', got: " + std::to_string(velocity_max_) +
        " <= " + std::to_string(velocity_min_));
  }

  // Log loaded parameters
  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), "  physics_rate: %.1f Hz", physics_rate_);
  RCLCPP_INFO(this->get_logger(), "  command_rate: %.1f Hz", command_rate_);
  RCLCPP_INFO(this->get_logger(), "  odometry_rate: %.1f Hz", odometry_rate_);
  RCLCPP_INFO(this->get_logger(), "  ground_truth_rate: %.1f Hz", ground_truth_rate_);
  RCLCPP_INFO(this->get_logger(), "  map_path: %s", map_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  grid_listing_file: %s", grid_listing_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "LiDAR sensor parameters:");
  RCLCPP_INFO(this->get_logger(), "  scan_rate: %.1f Hz", lidar_rate_);
  RCLCPP_INFO(this->get_logger(), "  horizontal_fov: %.3f rad", lidar_horizontal_fov_);
  RCLCPP_INFO(this->get_logger(), "  horizontal_resolution: %.5f rad", lidar_horizontal_resolution_);
  RCLCPP_INFO(this->get_logger(), "  range: [%.2f, %.2f] m", lidar_min_distance_, lidar_max_distance_);
  RCLCPP_INFO(this->get_logger(), "  noise_stddev: %.3f m", lidar_range_noise_stddev_);
  RCLCPP_INFO(this->get_logger(), "  motion_compensation: %s", lidar_enable_skew_ ? "enabled" : "disabled");
  RCLCPP_INFO(
      this->get_logger(), "  offset: (%.3f, %.3f, %.3f rad)", lidar_x_offset_, lidar_y_offset_, lidar_yaw_offset_);
  RCLCPP_INFO(this->get_logger(), "Wheel encoder parameters:");
  RCLCPP_INFO(this->get_logger(), "  pulses_per_revolution: %.0f", wheel_encoder_pulses_per_revolution_);
  RCLCPP_INFO(this->get_logger(), "  conversion_ratio: %.3f", wheel_encoder_conversion_ratio_);
  RCLCPP_INFO(
      this->get_logger(), "  use_true_steering_feedback_sensor: %s",
      use_true_steering_feedback_sensor_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "IMU sensor parameters:");
  RCLCPP_INFO(this->get_logger(), "  sample_rate: %.1f Hz", imu_rate_);
  RCLCPP_INFO(this->get_logger(), "  gravity: %.3f m/s^2", imu_gravity_);
  RCLCPP_INFO(this->get_logger(), "  position: (%.3f, %.3f, %.3f) m", imu_x_offset_, imu_y_offset_, imu_z_offset_);
  RCLCPP_INFO(
      this->get_logger(), "  orientation: (%.3f, %.3f, %.3f) rad", imu_roll_offset_, imu_pitch_offset_,
      imu_yaw_offset_);
  RCLCPP_INFO(this->get_logger(), "  accel_noise_density: %.3e m/s^2/sqrt(Hz)", imu_accelerometer_noise_density_);
  RCLCPP_INFO(this->get_logger(), "  accel_random_walk: %.3e m/s^3/sqrt(Hz)", imu_accelerometer_random_walk_);
  RCLCPP_INFO(this->get_logger(), "  gyro_noise_density: %.3e rad/s/sqrt(Hz)", imu_gyroscope_noise_density_);
  RCLCPP_INFO(this->get_logger(), "  gyro_random_walk: %.3e rad/s^2/sqrt(Hz)", imu_gyroscope_random_walk_);
  RCLCPP_INFO(this->get_logger(), "  mag_noise_density: %.3e T/sqrt(Hz)", imu_magnetometer_noise_density_);
  RCLCPP_INFO(this->get_logger(), "  accel_tau: %.3e s", imu_accelerometer_tau_);
  RCLCPP_INFO(this->get_logger(), "  gyro_tau: %.3e s", imu_gyroscope_tau_);
  RCLCPP_INFO(this->get_logger(), "  mag_tau: %.3e s", imu_magnetometer_tau_);
  RCLCPP_INFO(
      this->get_logger(), "  initial_accel_bias: (%.3e, %.3e, %.3e) m/s^2", imu_initial_accelerometer_bias_x_,
      imu_initial_accelerometer_bias_y_, imu_initial_accelerometer_bias_z_);
  RCLCPP_INFO(
      this->get_logger(), "  initial_gyro_bias: (%.3e, %.3e, %.3e) rad/s", imu_initial_gyroscope_bias_x_,
      imu_initial_gyroscope_bias_y_, imu_initial_gyroscope_bias_z_);
  RCLCPP_INFO(
      this->get_logger(), "  initial_mag_bias: (%.3e, %.3e, %.3e) T", imu_initial_magnetometer_bias_x_,
      imu_initial_magnetometer_bias_y_, imu_initial_magnetometer_bias_z_);
  RCLCPP_INFO(this->get_logger(), "Tire model parameters:");
  RCLCPP_INFO(this->get_logger(), "  model: %s", tire_model_.c_str());
  RCLCPP_INFO(this->get_logger(), "  combined_slip: %s", tire_combined_slip_ ? "enabled" : "disabled");
  RCLCPP_INFO(
      this->get_logger(), "  lateral: B=%.3f, C=%.3f, D=%.3f, E=%.3f", tire_lateral_stiffness_factor_B_,
      tire_lateral_shape_factor_C_, tire_lateral_peak_factor_D_, tire_lateral_curvature_factor_E_);
  RCLCPP_INFO(
      this->get_logger(), "  longitudinal: B=%.3f, C=%.3f, D=%.3f, E=%.3f", tire_longitudinal_stiffness_factor_B_,
      tire_longitudinal_shape_factor_C_, tire_longitudinal_peak_factor_D_, tire_longitudinal_curvature_factor_E_);
  RCLCPP_INFO(this->get_logger(), "Vehicle dynamics parameters:");
  RCLCPP_INFO(this->get_logger(), "  dynamic_model: %s", dynamic_model_.c_str());
  RCLCPP_INFO(this->get_logger(), "  wheelbase: %.3f m", wheelbase_);
  RCLCPP_INFO(this->get_logger(), "  rear_track_width: %.3f m", rear_track_width_);
  RCLCPP_INFO(this->get_logger(), "  front_track_width: %.3f m", front_track_width_);
  RCLCPP_INFO(this->get_logger(), "  effective_wheel_radius: %.3f m", effective_wheel_radius_);
  const double total_mass = calculate_total_mass();
  RCLCPP_INFO(
      this->get_logger(), "  mass: %.3f kg (sprung: %.3f kg, unsprung: 4x%.3f kg)", total_mass, sprung_mass_,
      unsprung_mass_);
  RCLCPP_INFO(this->get_logger(), "  center_of_mass: (%.3f, %.3f) m", center_of_mass_x_, center_of_mass_z_);
  RCLCPP_INFO(this->get_logger(), "  yaw_inertia: %.5f kg*m^2", yaw_inertia_);
  RCLCPP_INFO(this->get_logger(), "  wheel_inertia: %.5f kg*m^2", wheel_inertia_);
  RCLCPP_INFO(this->get_logger(), "  torque_split: engine=%.2f, brake=%.2f", engine_torque_split_, brake_torque_split_);
  RCLCPP_INFO(this->get_logger(), "  steering: %.3f rad", steering_max_);
  RCLCPP_INFO(this->get_logger(), "  acceleration: %.1f m/s²", acceleration_max_);
  RCLCPP_INFO(this->get_logger(), "  velocity: [%.1f, %.1f] m/s", velocity_min_, velocity_max_);
  RCLCPP_INFO(this->get_logger(), "Suspension parameters:");
  RCLCPP_INFO(this->get_logger(), "  spring_stiffness: %.1f N/m", suspension_params_.spring_stiffness);
  RCLCPP_INFO(this->get_logger(), "  roll_center_height: %.4f m", suspension_params_.roll_center_height);
  RCLCPP_INFO(this->get_logger(), "  max_inclination_angle: %.4f rad", suspension_params_.max_inclination_angle);
  RCLCPP_INFO(this->get_logger(), "Vehicle footprint parameters:");
  RCLCPP_INFO(this->get_logger(), "  width: %.3f m", footprint_width_);
  RCLCPP_INFO(this->get_logger(), "  front_distance: %.3f m", footprint_front_distance_);
  RCLCPP_INFO(this->get_logger(), "  rear_distance: %.3f m", footprint_rear_distance_);
}

void SvdsimServerNode::setup_entities() {
  RCLCPP_INFO(this->get_logger(), "Setting up vehicle entities...");

  // Create an entity for each vehicle in the grid CSV
  for (const auto& grid_entry : vehicle_grid_entries_) {
    entt::entity entity;

    if (grid_entry.type == VehicleType::SPARRING) {
      // Create sparring (autonomous) vehicle
      entity = create_sparring_vehicle(grid_entry.id, grid_entry.init_x, grid_entry.init_y, grid_entry.init_yaw);
      RCLCPP_INFO(
          this->get_logger(), "  Created SPARRING vehicle: %s at (%.2f, %.2f, %.2f)", grid_entry.id.c_str(),
          grid_entry.init_x, grid_entry.init_y, grid_entry.init_yaw);
    } else if (grid_entry.type == VehicleType::REMOTE) {
      // Create remote-controlled vehicle
      entity = create_user_controlled_vehicle(
          grid_entry.id, grid_entry.init_x, grid_entry.init_y, grid_entry.init_yaw, grid_entry.port);
      RCLCPP_INFO(
          this->get_logger(), "  Created REMOTE vehicle: %s at (%.2f, %.2f, %.2f) on port %d", grid_entry.id.c_str(),
          grid_entry.init_x, grid_entry.init_y, grid_entry.init_yaw, grid_entry.port);
    } else {
      throw std::runtime_error(
          "Unknown vehicle type for grid entry '" + grid_entry.id +
          "': " + std::to_string(static_cast<int>(grid_entry.type)));
    }

    vehicle_entities_.push_back(entity);
  }

  RCLCPP_INFO(this->get_logger(), "Created %zu vehicle entities", vehicle_entities_.size());
}

void SvdsimServerNode::setup_systems() {
  RCLCPP_INFO(this->get_logger(), "Setting up ECS systems...");

  // Get simulation clock for initial timestamps
  const auto& sim_clock = registry_.ctx().get<SimulationClock>();
  const auto initial_time = sim_clock.get_ticks();
  const auto time_step = sim_clock.time_step();

  // Create and initialize physics system (does not inherit from SystemInterface)
  physics_system_ = std::make_shared<PhysicsSimulationSystem>(registry_, dispatcher_, time_step);
  RCLCPP_INFO(this->get_logger(), "  Created physics simulation system");

  // Create ground truth sensor system
  const auto ground_truth_period = static_cast<std::size_t>(physics_rate_ / ground_truth_rate_);
  auto ground_truth_system = std::make_shared<GroundtruthSensorSystem>(registry_, dispatcher_, ground_truth_period);
  systems_.push_back(ground_truth_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, ground_truth_system));
  RCLCPP_INFO(this->get_logger(), "  Created ground truth sensor system (%.1f Hz)", ground_truth_rate_);

  // Create remote control system
  const auto remote_control_period = static_cast<std::size_t>(physics_rate_ / command_rate_);
  auto remote_control_system = std::make_shared<RemoteControlSystem>(registry_, dispatcher_, remote_control_period);
  systems_.push_back(remote_control_system);
  // Priority 101 to ensure it runs last in case of a tie with other systems
  activation_queue_.insert(SystemActivationEvent(initial_time, remote_control_system, 101));
  RCLCPP_INFO(this->get_logger(), "  Created remote control system (%.1f Hz)", command_rate_);

  // Create lidar sensor system
  const auto lidar_period = static_cast<std::size_t>(physics_rate_ / lidar_rate_);
  auto lidar_system = std::make_shared<LidarSensorSystem>(registry_, dispatcher_, lidar_period);
  systems_.push_back(lidar_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, lidar_system));
  RCLCPP_INFO(this->get_logger(), "  Created lidar sensor system (%.1f Hz)", lidar_rate_);

  // Create IMU sensor system
  const auto imu_period = static_cast<std::size_t>(physics_rate_ / imu_rate_);
  auto imu_system = std::make_shared<ImuSensorSystem>(registry_, dispatcher_, imu_period);
  systems_.push_back(imu_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, imu_system));
  RCLCPP_INFO(this->get_logger(), "  Created IMU sensor system (%.1f Hz)", imu_rate_);

  // Create sensor feedback publisher system
  const auto odometry_period = static_cast<std::size_t>(physics_rate_ / odometry_rate_);
  auto sensor_feedback_system = std::make_shared<OdometrySensorSystem>(registry_, dispatcher_, odometry_period);
  systems_.push_back(sensor_feedback_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, sensor_feedback_system));
  RCLCPP_INFO(this->get_logger(), "  Created sensor feedback publisher system (%.1f Hz)", odometry_rate_);

  // Create sparring control system
  const auto sparring_control_period = static_cast<std::size_t>(physics_rate_ / command_rate_);
  auto sparring_system = std::make_shared<SparringControlSystem>(registry_, dispatcher_, sparring_control_period);
  systems_.push_back(sparring_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, sparring_system));
  RCLCPP_INFO(this->get_logger(), "  Created sparring control system (%.1f Hz)", command_rate_);

  // Create wall collision detection system
  const auto wall_collision_period = static_cast<std::size_t>(physics_rate_ / command_rate_);
  collision_count_system =
      std::make_shared<CollisionCountSystem>(registry_, dispatcher_, collider_manager_, wall_collision_period);
  systems_.push_back(collision_count_system);
  activation_queue_.insert(SystemActivationEvent(initial_time, collision_count_system));
  RCLCPP_INFO(this->get_logger(), "  Created wall collision system (%.1f Hz)", command_rate_);

  RCLCPP_INFO(
      this->get_logger(), "Initialized %zu systems (%zu in activation queue)", systems_.size() + 1, systems_.size());
}

double SvdsimServerNode::calculate_total_mass() const {
  // Total mass = sprung mass + 4 tires * unsprung mass per tire
  return sprung_mass_ + 4.0 * unsprung_mass_;
}

std::unique_ptr<TireModelInterface> SvdsimServerNode::create_tire_model() {
  if (tire_model_ == "modified_nicolas_comstock") {
    TireModelParams tire_params;
    tire_params.lateral.B = tire_lateral_stiffness_factor_B_;
    tire_params.lateral.C = tire_lateral_shape_factor_C_;
    tire_params.lateral.Dn = tire_lateral_peak_factor_D_;
    tire_params.lateral.E = tire_lateral_curvature_factor_E_;
    tire_params.longitudinal.B = tire_longitudinal_stiffness_factor_B_;
    tire_params.longitudinal.C = tire_longitudinal_shape_factor_C_;
    tire_params.longitudinal.Dn = tire_longitudinal_peak_factor_D_;
    tire_params.longitudinal.E = tire_longitudinal_curvature_factor_E_;
    tire_params.effective_wheel_radius = effective_wheel_radius_;
    tire_params.combined_slip = tire_combined_slip_;

    return std::make_unique<MNCTireModel>(tire_params);
  } else {
    throw std::runtime_error(
        "Invalid tire_model value: '" + tire_model_ + "'. Currently only 'modified_nicolas_comstock' is supported");
  }
}

std::unique_ptr<SingleTrackModelInterface> SvdsimServerNode::create_single_track_model() {
  if (dynamic_model_ == "single_track_kinematic") {
    // Create kinematic model
    SingleTrackKinematicParams params;
    params.wheelbase = wheelbase_;
    params.rear_track_width = rear_track_width_;
    params.effective_wheel_radius = effective_wheel_radius_;
    // Calculate total mass from sprung + unsprung (4 tires)
    params.mass = calculate_total_mass();

    return std::make_unique<SingleTrackKinematicModel>(params);
  } else if (dynamic_model_ == "single_track_drift") {
    // Create tire model using factory
    auto tire_model = create_tire_model();

    // Create drift model
    SingleTrackDriftParams params;
    params.wheelbase = wheelbase_;
    params.rear_track_width = rear_track_width_;
    params.effective_wheel_radius = effective_wheel_radius_;
    // Calculate total mass from sprung + unsprung (4 tires)
    params.total_mass = calculate_total_mass();
    params.yaw_inertia = yaw_inertia_;
    params.center_of_mass_forward_distance = center_of_mass_x_;
    params.center_of_mass_height = center_of_mass_z_;
    params.roll_center_height = suspension_params_.roll_center_height;
    // Single-track model has virtual wheels representing 2 physical wheels each
    params.wheel_inertia = 2.0 * wheel_inertia_;
    params.engine_torque_split = engine_torque_split_;
    params.brake_torque_split = brake_torque_split_;

    return std::make_unique<SingleTrackDriftModel>(params, std::move(tire_model));
  } else {
    throw std::runtime_error(
        "Invalid dynamic_model value: '" + dynamic_model_ +
        "'. Must be 'single_track_kinematic' or 'single_track_drift'");
  }
}

entt::entity SvdsimServerNode::create_user_controlled_vehicle(
    const std::string& vehicle_name,
    double init_x,
    double init_y,
    double init_yaw,
    int port) {
  // Create a new entity in the registry
  auto entity = registry_.create();

  RCLCPP_DEBUG(
      this->get_logger(), "Creating vehicle '%s' at pose (%.2f, %.2f, %.2f) on port %d", vehicle_name.c_str(), init_x,
      init_y, init_yaw, port);

  // Create initial pose from grid parameters
  Sophus::SE2d initial_pose_2d(Sophus::SO2d(init_yaw), Eigen::Vector2d(init_x, init_y));
  Sophus::SE3d initial_pose_3d(Sophus::SO3d::rotZ(init_yaw), Eigen::Vector3d(init_x, init_y, 0.0));

  // Create and attach RemoteControlServerComponent (for ZeroMQ communication)
  auto control_server = std::make_unique<svdsim_proto::VehicleControlServer>(port);
  registry_.emplace<RemoteControlServerComponent>(entity, std::move(control_server));
  RCLCPP_INFO(
      this->get_logger(), "  Remote control server created for vehicle '%s' on port %d", vehicle_name.c_str(), port);

  // Create and attach DriveModelComponent (drive model parameters)
  SimpleDriveModelParameters drive_model_params;
  drive_model_params.steering_velocity_max = steering_velocity_max_;
  drive_model_params.steering_angle_max = steering_max_;
  drive_model_params.steering_hysteresis_gap = steering_hysteresis_gap_;
  drive_model_params.velocity_max = velocity_max_;
  drive_model_params.velocity_min = velocity_min_;
  drive_model_params.drive_motor_acceleration = acceleration_max_;
  drive_model_params.drive_braking_deceleration = braking_deceleration_;
  drive_model_params.velocity_deadband = velocity_deadband_;
  drive_model_params.motor_drive_rise_time = motor_drive_rise_time_;
  drive_model_params.dt = 1.0 / physics_rate_;
  auto drive_model = std::make_unique<SimpleDriveModel>(drive_model_params);
  registry_.emplace<DriveModelComponent>(entity, std::move(drive_model));

  // Create and attach SingleTrackModelComponent (kinematic or drift model)
  auto vehicle_model = create_single_track_model();
  vehicle_model->reset(initial_pose_2d);
  registry_.emplace<SingleTrackKinematicModelComponent>(entity, std::move(vehicle_model));

  // Use footprint parameters to calculate collider corners
  const float half_width = static_cast<float>(footprint_width_ / 2.0);
  const float front_x = static_cast<float>(footprint_front_distance_);
  const float rear_x = static_cast<float>(-footprint_rear_distance_);

  auto collider_handle = collider_manager_->add_rectangular_body(
      initial_pose_2d, BodyCategory::VEHICLE, std::make_tuple(front_x, half_width),  // front_left (x, y)
      std::make_tuple(rear_x, -half_width),                                          // rear_right (x, y)
      entity);
  registry_.emplace<VehicleColliderComponent>(entity, collider_handle);

  // Create and attach LidarSensorComponent (only if rate > 0)
  if (lidar_rate_ > 0.0) {
    Sophus::SE2d lidar_pose(Sophus::SO2d(lidar_yaw_offset_), Eigen::Vector2d(lidar_x_offset_, lidar_y_offset_));

    // Calculate number of scan points from FOV and resolution
    const int points_per_scan = static_cast<int>(lidar_horizontal_fov_ / lidar_horizontal_resolution_) + 1;

    // Create LidarSensorModel with all parameters
    auto lidar_sensor = std::make_unique<LidarSensorModel>(
        collider_manager_, lidar_pose, lidar_min_distance_, lidar_max_distance_, lidar_horizontal_fov_, points_per_scan,
        lidar_range_noise_stddev_, physics_rate_, lidar_rate_, lidar_enable_skew_);

    // Attach LidarSensorComponent with the sensor instance
    registry_.emplace<LidarSensorComponent>(entity, std::move(lidar_sensor));
    RCLCPP_DEBUG(this->get_logger(), "  LiDAR sensor added to vehicle '%s'", vehicle_name.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "  LiDAR sensor disabled (rate = 0) for vehicle '%s'", vehicle_name.c_str());
  }

  // Create and attach IMUSensorComponent (only if rate > 0)
  if (imu_rate_ > 0.0) {
    Sophus::SO3d imu_orientation = Sophus::SO3d::rotZ(imu_yaw_offset_) * Sophus::SO3d::rotY(imu_pitch_offset_) *
                                   Sophus::SO3d::rotX(imu_roll_offset_);
    Sophus::SE3d imu_pose(imu_orientation, Eigen::Vector3d(imu_x_offset_, imu_y_offset_, imu_z_offset_));

    IMUNoiseParameters imu_noise_params;
    imu_noise_params.accelerometer_noise_density = imu_accelerometer_noise_density_;
    imu_noise_params.accelerometer_random_walk = imu_accelerometer_random_walk_;
    imu_noise_params.gyroscope_noise_density = imu_gyroscope_noise_density_;
    imu_noise_params.gyroscope_random_walk = imu_gyroscope_random_walk_;
    imu_noise_params.magnetometer_noise_density = imu_magnetometer_noise_density_;
    imu_noise_params.accelerometer_bias_autocorrelation_time = imu_accelerometer_tau_;
    imu_noise_params.gyroscope_bias_autocorrelation_time = imu_gyroscope_tau_;
    imu_noise_params.magnetometer_bias_autocorrelation_time = imu_magnetometer_tau_;
    imu_noise_params.initial_accelerometer_bias = Eigen::Vector3d(
        imu_initial_accelerometer_bias_x_, imu_initial_accelerometer_bias_y_, imu_initial_accelerometer_bias_z_);
    imu_noise_params.initial_gyroscope_bias =
        Eigen::Vector3d(imu_initial_gyroscope_bias_x_, imu_initial_gyroscope_bias_y_, imu_initial_gyroscope_bias_z_);
    imu_noise_params.initial_magnetometer_bias = Eigen::Vector3d(
        imu_initial_magnetometer_bias_x_, imu_initial_magnetometer_bias_y_, imu_initial_magnetometer_bias_z_);

    auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose, imu_gravity_, imu_noise_params);
    registry_.emplace<IMUSensorComponent>(entity, std::move(imu_sensor));
    RCLCPP_DEBUG(this->get_logger(), "  IMU sensor added to vehicle '%s'", vehicle_name.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "  IMU sensor disabled (rate = 0) for vehicle '%s'", vehicle_name.c_str());
  }

  // Create and attach WheelEncoderSensorComponent (only if rate > 0)
  if (odometry_rate_ > 0.0) {
    auto rear_left_encoder_model =
        std::make_unique<PulseEncoderModel>(wheel_encoder_conversion_ratio_, wheel_encoder_pulses_per_revolution_);
    auto rear_right_encoder_model =
        std::make_unique<PulseEncoderModel>(wheel_encoder_conversion_ratio_, wheel_encoder_pulses_per_revolution_);
    registry_.emplace<WheelEncoderSensorComponent>(
        entity, std::move(rear_left_encoder_model), std::move(rear_right_encoder_model));
    RCLCPP_DEBUG(this->get_logger(), "  Wheel encoders added to vehicle '%s'", vehicle_name.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "  Wheel encoders disabled (rate = 0) for vehicle '%s'", vehicle_name.c_str());
  }

  // Depeneding on the setting of use_true_steering_feedback_sensor, use a true steering state sensor,
  // or just one that simply echoes the steering command like the real roboracer VESC driver does
  if (use_true_steering_feedback_sensor_) {
    registry_.emplace<TrueSteeringServoSensorComponent>(entity);
  } else {
    registry_.emplace<EmulatedSteeringServoSensorComponent>(entity);
  }

  // Create and attach GroundtruthSensorComponent
  registry_.emplace<GroundtruthSensorComponent>(entity);

  // Create and attach SuspensionSimulationComponent
  SprungMassInclinationParameters suspension_params;
  suspension_params.wheelbase = wheelbase_;
  suspension_params.front_track_width = front_track_width_;
  suspension_params.rear_track_width = rear_track_width_;
  suspension_params.sprung_mass = sprung_mass_;
  suspension_params.center_of_mass_height = center_of_mass_z_;
  suspension_params.center_of_mass_forward_distance = center_of_mass_x_;
  suspension_params.spring_stiffness = suspension_params_.spring_stiffness;
  suspension_params.roll_center_height = suspension_params_.roll_center_height;
  suspension_params.max_inclination_angle = suspension_params_.max_inclination_angle;

  auto suspension_model = std::make_unique<SprungMassInclinationModel>(suspension_params);
  registry_.emplace<SuspensionSimulationComponent>(entity, std::move(suspension_model));
  RCLCPP_DEBUG(this->get_logger(), "  Suspension simulation added to vehicle '%s'", vehicle_name.c_str());

  return entity;
}

entt::entity SvdsimServerNode::create_sparring_vehicle(
    const std::string& vehicle_name,
    double init_x,
    double init_y,
    double init_yaw) {
  // Validate trajectory file before creating entity
  if (sparring_trajectory_file_.empty()) {
    throw std::runtime_error("Sparring trajectory file not specified for vehicle '" + vehicle_name + "'");
  }

  if (!std::filesystem::exists(sparring_trajectory_file_)) {
    throw std::runtime_error(
        "Sparring trajectory file does not exist: '" + sparring_trajectory_file_ + "' for vehicle '" + vehicle_name +
        "'");
  }

  // Create a new entity in the registry
  auto entity = registry_.create();

  RCLCPP_DEBUG(
      this->get_logger(), "Creating sparring vehicle '%s' at pose (%.2f, %.2f, %.2f)", vehicle_name.c_str(), init_x,
      init_y, init_yaw);

  // Create initial pose from grid parameters
  Sophus::SE2d initial_pose_2d(Sophus::SO2d(init_yaw), Eigen::Vector2d(init_x, init_y));

  // Create and attach DriveModelComponent (needed for physics system to process commands)
  SimpleDriveModelParameters drive_model_params;
  drive_model_params.steering_velocity_max = steering_velocity_max_;
  drive_model_params.steering_angle_max = steering_max_;
  drive_model_params.steering_hysteresis_gap = steering_hysteresis_gap_;
  drive_model_params.velocity_max = velocity_max_;
  drive_model_params.velocity_min = velocity_min_;
  drive_model_params.drive_motor_acceleration = acceleration_max_;
  drive_model_params.drive_braking_deceleration = braking_deceleration_;
  drive_model_params.velocity_deadband = 0.0;  // Sparring vehicles do not use a velocity dead band
  drive_model_params.motor_drive_rise_time = motor_drive_rise_time_;
  drive_model_params.dt = 1.0 / physics_rate_;
  auto drive_model = std::make_unique<SimpleDriveModel>(drive_model_params);
  registry_.emplace<DriveModelComponent>(entity, std::move(drive_model));

  // Create and attach SingleTrackModelComponent (always kinematic for sparring vehicles)
  SingleTrackKinematicParams params;
  params.wheelbase = wheelbase_;
  params.rear_track_width = rear_track_width_;
  params.effective_wheel_radius = effective_wheel_radius_;
  params.mass = calculate_total_mass();
  auto vehicle_model = std::make_unique<SingleTrackKinematicModel>(params);
  vehicle_model->reset(initial_pose_2d);
  registry_.emplace<SingleTrackKinematicModelComponent>(entity, std::move(vehicle_model));

  // Use footprint parameters to calculate collider corners
  const float half_width = static_cast<float>(footprint_width_ / 2.0);
  const float front_x = static_cast<float>(footprint_front_distance_);
  const float rear_x = static_cast<float>(-footprint_rear_distance_);

  auto collider_handle = collider_manager_->add_rectangular_body(
      initial_pose_2d, BodyCategory::VEHICLE, std::make_tuple(front_x, half_width),  // front_left (x, y)
      std::make_tuple(rear_x, -half_width),                                          // rear_right (x, y)
      entity);
  registry_.emplace<VehicleColliderComponent>(entity, collider_handle);

  // Load trajectory from file (already validated existence at function start)
  std::vector<svdsim_server::TrajectoryPoint> trajectory;
  try {
    svdsim_server::TrajectoryReader reader;
    trajectory = reader.read_from_file(sparring_trajectory_file_);
    RCLCPP_INFO(
        this->get_logger(), "  Loaded trajectory with %zu points from: %s", trajectory.size(),
        sparring_trajectory_file_.c_str());
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to load trajectory for sparring vehicle '" + vehicle_name + "': " + e.what());
  }

  // Create pure pursuit controller
  svdsim_server::PurePursuitParams pp_params;
  pp_params.wheelbase = wheelbase_;
  pp_params.lookahead_distance = sparring_lookahead_distance_;
  pp_params.max_steering = sparring_max_steering_;

  auto controller = std::make_unique<svdsim_server::PurePursuitController>(trajectory, pp_params);

  // Attach SparringControllerComponent
  registry_.emplace<SparringControllerComponent>(entity, std::move(controller), sparring_constant_velocity_);
  RCLCPP_DEBUG(this->get_logger(), "  Sparring controller added to vehicle '%s'", vehicle_name.c_str());

  // Optionally add sensors (groundtruth, lidar, IMU) if needed
  // For now, we'll add groundtruth for visualization purposes
  registry_.emplace<GroundtruthSensorComponent>(entity);

  RCLCPP_INFO(
      this->get_logger(), "  Sparring vehicle '%s' fully configured with %zu trajectory points", vehicle_name.c_str(),
      trajectory.size());

  return entity;
}

void SvdsimServerNode::start() {
  RCLCPP_INFO(this->get_logger(), "Starting simulation with dedicated thread execution");

  // Start simulation runner thread
  running_ = true;
  simulation_thread_ = std::thread(&SvdsimServerNode::simulation_loop, this);

  RCLCPP_INFO(this->get_logger(), "Simulation thread started");
}

void SvdsimServerNode::simulation_loop() {
  RCLCPP_INFO(this->get_logger(), "Simulation loop started");

  auto& sim_clock = registry_.ctx().get<SimulationClock>();

  auto wall_clock_start = std::chrono::steady_clock::now();
  auto sim_clock_start = sim_clock.get_time();

  while (running_) {
    const auto current_time = sim_clock.get_ticks();

    // Process all systems whose activation time has arrived
    while (!activation_queue_.empty() && activation_queue_.front().timestamp() <= current_time) {
      auto& event = activation_queue_.front();
      auto system_ptr = event.payload();
      const auto activation_time = event.timestamp();
      const auto priority = event.priority();
      activation_queue_.pop();

      if (activation_time < current_time) {
        RCLCPP_ERROR(
            this->get_logger(),
            "System activated late: %s < %s (current time). This indicates that the system is not keeping up with "
            "real-time.",
            std::to_string(activation_time).c_str(), std::to_string(current_time).c_str());
      }

      // Invoke the system and get next activation time
      const auto next_activation = system_ptr->update();

      if (next_activation <= activation_time) {
        RCLCPP_ERROR(
            this->get_logger(),
            "System returned invalid next activation time: %s <= %s (current activation time). This would cause an "
            "infinite loop.",
            std::to_string(next_activation).c_str(), std::to_string(activation_time).c_str());
        running_ = false;
        return;
      }

      // Re-insert system into queue with new activation time
      activation_queue_.insert(SystemActivationEvent(next_activation, std::move(system_ptr), priority));

      RCLCPP_DEBUG(
          this->get_logger(), "Activated system at time %s, next activation at %s",
          std::to_string(activation_time).c_str(), std::to_string(next_activation).c_str());
    }

    // Calculate physics steps until next system activation
    if (activation_queue_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Activation queue is empty, no systems to activate");
      running_ = false;
      return;
    }

    const auto next_activation = activation_queue_.front().timestamp();
    const auto steps_until_next_activation = next_activation - current_time;

    RCLCPP_DEBUG(
        this->get_logger(), "Advancing physics by %i steps to next system activation at time %s",
        steps_until_next_activation, std::to_string(next_activation).c_str());

    // Run physics simulation (clock is advanced inside physics system)
    if (steps_until_next_activation > 0) {
      physics_system_->update(steps_until_next_activation);
    }

    // Real-time factor (RTF) control: maintain simulation speed close to real-time.
    // Notice that this will work only if we never reset the simulation clock.
    auto wall_clock_now = std::chrono::steady_clock::now();
    auto sim_clock_now = sim_clock.get_time();

    const auto elapsed_wall_time = std::chrono::duration<double>(wall_clock_now - wall_clock_start).count();
    const auto elapsed_sim_time = (sim_clock_now - sim_clock_start).to_seconds();

    // If simulation is ahead of wall clock, sleep to maintain RTF ≈ 1.0
    if (elapsed_sim_time > elapsed_wall_time) {
      RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Simulation is ahead of real-time by %.3f seconds, sleeping to maintain real-time factor",
          elapsed_sim_time - elapsed_wall_time);
      const auto sleep_duration = std::chrono::duration<double>(elapsed_sim_time - elapsed_wall_time);
      std::this_thread::sleep_for(sleep_duration);

    } else {
      RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000, "Simulation is behind real-time by %.3f seconds",
          elapsed_wall_time - elapsed_sim_time);
    }

    // Periodically calculate the realtime factor baseline to avoid drift
    if (elapsed_wall_time > rtf_report_interval_) {
      RCLCPP_INFO(
          this->get_logger(), "Average real-time factor over the last %.1f seconds: %.1f", elapsed_wall_time,
          elapsed_sim_time / elapsed_wall_time);
      wall_clock_start = wall_clock_now;
      sim_clock_start = sim_clock_now;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Simulation loop terminated");
}

}  // namespace svdsim_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(svdsim_server::SvdsimServerNode)
