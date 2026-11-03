// Copyright 2026 Ekumen
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

#ifndef MONACOF1_SIM_BRIDGE__MONACOF1_SIM_BRIDGE_NODE_HPP_
#define MONACOF1_SIM_BRIDGE__MONACOF1_SIM_BRIDGE_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sim_messages.pb.h"
#include "svdsim_msgs/msg/collision_data.hpp"
#include "svdsim_msgs/msg/stamped_encoder_data.hpp"
#include "svdsim_msgs/msg/stamped_steering.hpp"
#include "svdsim_msgs/msg/stamped_velocity.hpp"
#include "svdsim_proto/vehicle_control_client.hpp"

namespace svdsim_bridge {

/**
 * @brief Bridge node between ZMQ/protobuf simulation server and ROS 2
 *
 * This node provides bidirectional communication between the Monaco F1TENTH
 * simulation server (via ZMQ/protobuf) and ROS 2 topics. It runs two threads:
 * - Main thread: Services ROS callbacks
 * - ZMQ thread: Polls and processes incoming protobuf messages
 */
class MonacoF1SimBridgeNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Monaco F1 Sim Bridge Node
   *
   * @param options Node options for configuration
   */
  explicit MonacoF1SimBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - ensures clean shutdown of threads
   */
  ~MonacoF1SimBridgeNode() override;

 private:
  /**
   * @brief Initialize ZMQ client connection
   */
  void initialize_zmq_client();

  /**
   * @brief ZMQ polling thread function
   *
   * Continuously polls for incoming messages with 20ms timeout
   * and processes them until shutdown is requested
   */
  void zmq_spinner_thread();

  /**
   * @brief Callback for incoming protobuf messages from simulator
   *
   * @param message The received SimDataMessage
   */
  void on_proto_message_received(const svdsim_server::SimDataMessage& message);

  /**
   * @brief Convert protobuf timestamp to ROS timestamp
   *
   * @param timestamp The protobuf timestamp to convert
   * @return builtin_interfaces::msg::Time The converted ROS timestamp
   */
  builtin_interfaces::msg::Time proto_timestamp_to_ros(const svdsim_server::Timestamp& timestamp) const;

  /**
   * @brief Publish clock message from protobuf timestamp
   *
   * @param timestamp The protobuf timestamp to convert and publish
   */
  void publish_clock_message(const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish steering status message
   *
   * @param steering_feedback The protobuf steering status to convert and publish
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_steering_feedback(
      const svdsim_server::SteeringFeedback& steering_feedback,
      const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish encoder data message
   *
   * @param encoder_data The protobuf encoder data to convert and publish
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_encoder_data(
      const svdsim_server::WheelEncoderData& encoder_data,
      const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish lidar scan message
   *
   * @param lidar_data The protobuf lidar scan data to convert and publish
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_lidar_scan(const svdsim_server::LidarScanData& lidar_data, const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish IMU sample message
   *
   * @param imu_data The protobuf IMU sample data to convert and publish
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_imu_sample(const svdsim_server::IMUSampleData& imu_data, const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish magnetometer message
   *
   * @param imu_data The protobuf IMU sample data containing magnetometer readings
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_magnetometer(const svdsim_server::IMUSampleData& imu_data, const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish ground truth odometry message
   *
   * @param ground_truth_data The protobuf ground truth data to convert and publish
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_ground_truth(
      const svdsim_server::GroundTruthData& ground_truth_data,
      const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish sparring pose marker
   *
   * @param pose_marker_data The protobuf sparring pose marker data
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_sparring_pose_marker(
      const svdsim_server::SparringPoseMarker& pose_marker_data,
      const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish sparring lookahead marker
   *
   * @param lookahead_marker_data The protobuf sparring lookahead marker data
   * @param timestamp The protobuf timestamp for the message header
   */
  void publish_sparring_lookahead_marker(
      const svdsim_server::SparringLookaheadMarker& lookahead_marker_data,
      const svdsim_server::Timestamp& timestamp);

  /**
   * @brief Publish collision data message
   *
   * @param collision_count_data The protobuf collision count data to convert and publish
   */
  void publish_collision_data(const svdsim_server::CollisionCountData& collision_count_data);

  /**
   * @brief Callback for steering command messages
   *
   * @param msg The steering command message
   */
  void steering_command_callback(const svdsim_msgs::msg::StampedSteering::SharedPtr msg);

  /**
   * @brief Callback for velocity command messages
   *
   * @param msg The velocity command message
   */
  void velocity_command_callback(const svdsim_msgs::msg::StampedVelocity::SharedPtr msg);

  /**
   * @brief Callback for reset pose messages
   *
   * @param msg The pose with covariance stamped message
   */
  void reset_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Connection parameters
  std::string hostname_;
  int port_;
  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  std::string map_frame_id_;
  std::string base_link_frame_id_;

  // ZMQ client
  std::unique_ptr<svdsim_proto::VehicleControlClient> zmq_client_;

  // ROS publishers
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<svdsim_msgs::msg::StampedSteering>::SharedPtr steering_feedback_publisher_;
  rclcpp::Publisher<svdsim_msgs::msg::StampedEncoderData>::SharedPtr encoder_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sparring_markers_publisher_;
  rclcpp::Publisher<svdsim_msgs::msg::CollisionData>::SharedPtr collision_data_publisher_;

  // ROS subscribers
  rclcpp::Subscription<svdsim_msgs::msg::StampedSteering>::SharedPtr steering_command_subscriber_;
  rclcpp::Subscription<svdsim_msgs::msg::StampedVelocity>::SharedPtr velocity_command_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;

  // Sparring marker cache and timer
  std::map<uint32_t, visualization_msgs::msg::Marker> cached_pose_markers_;
  std::map<uint32_t, visualization_msgs::msg::Marker> cached_lookahead_markers_;
  std::mutex marker_cache_mutex_;
  rclcpp::TimerBase::SharedPtr sparring_marker_timer_;

  /**
   * @brief Timer callback to publish cached sparring markers
   */
  void publish_sparring_markers_timer_callback();

  // Threading
  std::thread zmq_thread_;
  std::atomic<bool> running_;
};

}  // namespace svdsim_bridge

#endif  // MONACOF1_SIM_BRIDGE__MONACOF1_SIM_BRIDGE_NODE_HPP_
