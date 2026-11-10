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

#ifndef SVDSIM_SERVER_SVDSIM_SERVER_NODE_HPP_
#define SVDSIM_SERVER_SVDSIM_SERVER_NODE_HPP_

// Standard library
#include <map>
#include <memory>
#include <string>
#include <vector>

// Non-ROS dependencies
#include <entt/entt.hpp>

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/string.hpp>

// Project includes
#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/collision/static_map_handler.hpp"
#include "svdsim_server/configuration/grid_csv_reader.hpp"
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"
#include "svdsim_server/ecs/systems/collision_count_system.hpp"
#include "svdsim_server/ecs/systems/groundtruth_sensor_system.hpp"
#include "svdsim_server/ecs/systems/imu_sensor_system.hpp"
#include "svdsim_server/ecs/systems/lidar_sensor_system.hpp"
#include "svdsim_server/ecs/systems/odometry_sensor_system.hpp"
#include "svdsim_server/ecs/systems/physics_simulation_system.hpp"
#include "svdsim_server/ecs/systems/remote_control_system.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/event_queue/activation_event.hpp"
#include "svdsim_server/event_queue/activation_queue.hpp"
#include "svdsim_server/models/single_track_model_interface.hpp"
#include "svdsim_server/models/tire_model_interface.hpp"

namespace svdsim_server {

/**
 * @brief SVDSim simulation server node
 *
 * This node provides the simulation environment for autonomous racing development,
 * including physics simulation, vehicle dynamics, and sensor simulation.
 */
class SvdsimServerNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SVDSim server node object
   *
   * @param options Node options for configuration
   */
  explicit SvdsimServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the SVDSim server node object
   *
   * Ensures proper shutdown of the simulation thread.
   */
  ~SvdsimServerNode();

  /**
   * @brief Initialize and start the simulation server
   *
   * Sets up the simulation timer and begins periodic execution.
   * Compatible with component-based node composition.
   */
  void start();

 private:
  /**
   * @brief Declare node parameters
   *
   * Parameters are declared with default values and descriptions.
   */
  void declare_parameters();

  /**
   * @brief Load and validate parameters from ROS parameter server
   *
   * @throws std::runtime_error if required parameters are missing
   */
  void load_parameters();

  /**
   * @brief Set up entities for all vehicles in the simulation
   *
   * Creates an entity for each vehicle listed in the grid CSV file.
   */
  void setup_entities();

  /**
   * @brief Set up ECS systems for simulation
   *
   * Creates and initializes all system instances and populates the activation queue.
   */
  void setup_systems();

  /**
   * @brief Create a tire model based on tire model parameter
   *
   * Factory method that instantiates the appropriate tire model based on
   * the tire_model_ parameter value.
   *
   * @return std::unique_ptr<TireModelInterface> The created tire model
   * @throws std::runtime_error if tire_model_ has an invalid value
   */
  std::unique_ptr<TireModelInterface> create_tire_model();

  /**
   * @brief Calculate total vehicle mass (sprung + unsprung)
   *
   * @return double Total vehicle mass [kg] (sprung mass + 4 tires * unsprung mass per tire)
   */
  double calculate_total_mass() const;

  /**
   * @brief Create a single track model based on dynamic_model parameter
   *
   * Factory method that instantiates either SingleTrackKinematicModel or
   * SingleTrackDriftModel based on the dynamic_model_ parameter value.
   *
   * @return std::unique_ptr<SingleTrackModelInterface> The created model
   * @throws std::runtime_error if dynamic_model_ has an invalid value
   */
  std::unique_ptr<SingleTrackModelInterface> create_single_track_model();

  /**
   * @brief Create a user-controlled vehicle entity
   *
   * @param vehicle_name Name identifier for the vehicle
   * @param init_x Initial X position in world coordinates [m]
   * @param init_y Initial Y position in world coordinates [m]
   * @param init_yaw Initial yaw angle [rad]
   * @param port Network port for vehicle communication
   * @return entt::entity The created entity handle
   */
  entt::entity create_user_controlled_vehicle(
      const std::string& vehicle_name,
      double init_x,
      double init_y,
      double init_yaw,
      int port);

  /**
   * @brief Create a sparring (autonomous) vehicle entity
   *
   * @param vehicle_name Name identifier for the vehicle
   * @param init_x Initial X position in world coordinates [m]
   * @param init_y Initial Y position in world coordinates [m]
   * @param init_yaw Initial yaw angle [rad]
   * @return entt::entity The created entity handle
   */
  entt::entity create_sparring_vehicle(const std::string& vehicle_name, double init_x, double init_y, double init_yaw);

  /**
   * @brief Simulation loop runner
   *
   * Runs in a dedicated thread and executes the simulation with RTF control.
   */
  void simulation_loop();

  // Entity-Component-System
  entt::registry registry_;
  entt::dispatcher dispatcher_;
  std::vector<entt::entity> vehicle_entities_;

  // System management
  using SystemPtr = std::shared_ptr<SystemInterface>;
  using SystemActivationEvent = ActivationEvent<SystemPtr>;
  ActivationQueue<SystemActivationEvent> activation_queue_;
  std::vector<SystemPtr> systems_;
  std::shared_ptr<PhysicsSimulationSystem> physics_system_;
  std::shared_ptr<CollisionCountSystem> collision_count_system;

  // Collision and map management
  std::shared_ptr<ColliderManager> collider_manager_;
  std::unique_ptr<StaticMapHandler> static_map_handler_;
  StaticMap static_map_;

  // Configuration parameters
  double physics_rate_;
  double command_rate_;
  double odometry_rate_;
  double ground_truth_rate_;
  double rtf_report_interval_;
  std::string map_path_;
  std::string grid_listing_file_;
  std::vector<VehicleGridEntry> vehicle_grid_entries_;

  // Vehicle dynamics parameters
  std::string dynamic_model_;       // "single_track_kinematic" or "single_track_drift"
  double wheelbase_;                // [m]
  double rear_track_width_;         // [m]
  double front_track_width_;        // [m]
  double effective_wheel_radius_;   // [m]
  double sprung_mass_;              // [kg]
  double unsprung_mass_;            // Per tire [kg]
  double center_of_mass_x_;         // [m]
  double center_of_mass_z_;         // [m]
  double yaw_inertia_;              // [kg*m^2]
  double wheel_inertia_;            // Single wheel [kg*m^2]
  double engine_torque_split_;      // Fraction of engine torque to front axle [0.0-1.0]
  double brake_torque_split_;       // Fraction of brake torque to front axle [0.0-1.0]
  double steering_max_;             // [rad]
  double steering_velocity_max_;    // [rad/s]
  double steering_hysteresis_gap_;  // [rad]
  double acceleration_max_;         // [m/s^2]
  double braking_deceleration_;     // [m/s^2]
  double motor_drive_rise_time_;    // [s]
  double velocity_deadband_;        // [m/s]
  double velocity_max_;             // [m/s]
  double velocity_min_;             // [m/s]
  double velocity_switch_;          // [m/s]

  // Tire model parameters
  std::string tire_model_;                       // for now only "modified_nicolas_comstock"
  bool tire_combined_slip_;                      // Enable combined slip
  double tire_lateral_stiffness_factor_B_;       // [1/deg]
  double tire_lateral_shape_factor_C_;           // [-]
  double tire_lateral_peak_factor_D_;            // [-]
  double tire_lateral_curvature_factor_E_;       // [-]
  double tire_longitudinal_stiffness_factor_B_;  // [1/slip ratio]
  double tire_longitudinal_shape_factor_C_;      // [-]
  double tire_longitudinal_peak_factor_D_;       // [-]
  double tire_longitudinal_curvature_factor_E_;  // [-]

  // Suspension simulation parameters
  struct SuspensionParams {
    double spring_stiffness;
    double roll_center_height;
    double max_inclination_angle;
  };

  SuspensionParams suspension_params_;

  // Vehicle footprint parameters
  double footprint_width_;
  double footprint_front_distance_;
  double footprint_rear_distance_;

  // Lidar sensor parameters
  double lidar_rate_;                   // [Hz]
  double lidar_horizontal_fov_;         // [rad]
  double lidar_horizontal_resolution_;  // [rad]
  double lidar_max_distance_;           // [m]
  double lidar_min_distance_;           // [m]
  double lidar_range_noise_stddev_;     // [m]
  bool lidar_enable_skew_;              // Enable motion compensation
  double lidar_x_offset_;               // [m]
  double lidar_y_offset_;               // [m]
  double lidar_yaw_offset_;             // [rad]

  // Wheel encoder parameters
  double wheel_encoder_pulses_per_revolution_;
  double wheel_encoder_conversion_ratio_;

  // If true, report the true simulated steering angle as feedback; otherwise
  // report the last commanded steering reference
  bool use_true_steering_feedback_sensor_;

  // IMU sensor parameters
  double imu_rate_;                          // [Hz]
  double imu_x_offset_;                      // [m]
  double imu_y_offset_;                      // [m]
  double imu_z_offset_;                      // [m]
  double imu_roll_offset_;                   // [rad]
  double imu_pitch_offset_;                  // [rad]
  double imu_yaw_offset_;                    // [rad]
  double imu_gravity_;                       // [m/s^2]
  double imu_accelerometer_noise_density_;   // [m/s^2/sqrt(Hz)]
  double imu_accelerometer_random_walk_;     // [m/s^3/sqrt(Hz)]
  double imu_gyroscope_noise_density_;       // [rad/s/sqrt(Hz)]
  double imu_gyroscope_random_walk_;         // [rad/s^2/sqrt(Hz)]
  double imu_magnetometer_noise_density_;    // [T/sqrt(Hz)]
  double imu_accelerometer_tau_;             // Accelerometer bias autocorrelation time [s]
  double imu_gyroscope_tau_;                 // Gyroscope bias autocorrelation time [s]
  double imu_magnetometer_tau_;              // Magnetometer bias autocorrelation time [s]
  double imu_initial_accelerometer_bias_x_;  // [m/s^2]
  double imu_initial_accelerometer_bias_y_;  // [m/s^2]
  double imu_initial_accelerometer_bias_z_;  // [m/s^2]
  double imu_initial_gyroscope_bias_x_;      // [rad/s]
  double imu_initial_gyroscope_bias_y_;      // [rad/s]
  double imu_initial_gyroscope_bias_z_;      // [rad/s]
  double imu_initial_magnetometer_bias_x_;   // [T]
  double imu_initial_magnetometer_bias_y_;   // [T]
  double imu_initial_magnetometer_bias_z_;   // [T]

  // Sparring vehicle parameters
  std::string sparring_trajectory_file_;  // Path to trajectory CSV file
  double sparring_constant_velocity_;     // [m/s]
  double sparring_lookahead_distance_;    // [m]
  double sparring_max_steering_;          // [rad]

  // Simulation thread
  std::thread simulation_thread_;
  std::atomic<bool> running_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_SVDSIM_SERVER_NODE_HPP_
