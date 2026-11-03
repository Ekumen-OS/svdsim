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

#include "svdsim_bridge/svdsim_bridge_node.hpp"

#include <functional>

namespace svdsim_bridge {

MonacoF1SimBridgeNode::MonacoF1SimBridgeNode(const rclcpp::NodeOptions& options)
    : Node("svdsim_bridge", options), running_(false) {
  // Declare and get parameters
  this->declare_parameter("hostname", "localhost");
  this->declare_parameter("port", 4242);
  this->declare_parameter("lidar_frame_id", "laser");
  this->declare_parameter("imu_frame_id", "imu");
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("base_link_frame_id", "base_link");

  hostname_ = this->get_parameter("hostname").as_string();
  port_ = this->get_parameter("port").as_int();
  lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
  imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  base_link_frame_id_ = this->get_parameter("base_link_frame_id").as_string();

  RCLCPP_INFO(this->get_logger(), "Monaco F1TENTH Simulation Bridge Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Connecting to: %s:%d", hostname_.c_str(), port_);
  RCLCPP_INFO(this->get_logger(), "  Lidar frame_id: %s", lidar_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  IMU frame_id: %s", imu_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Map frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Base link frame_id: %s", base_link_frame_id_.c_str());

  // Initialize ROS publishers
  clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // Publishers with default QoS (Reliable)
  steering_feedback_publisher_ = this->create_publisher<svdsim_msgs::msg::StampedSteering>("steering_feedback", 10);
  encoder_data_publisher_ = this->create_publisher<svdsim_msgs::msg::StampedEncoderData>("encoder_data", 10);
  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 5);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  magnetometer_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetometer", 10);
  ground_truth_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
  sparring_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sparring/markers", 10);

  // Create timer for publishing sparring markers at 10 Hz
  sparring_marker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MonacoF1SimBridgeNode::publish_sparring_markers_timer_callback, this));
  collision_data_publisher_ =
      this->create_publisher<svdsim_msgs::msg::CollisionData>("collision_data", rclcpp::QoS(10).transient_local());

  // Initialize ROS subscribers with default QoS
  steering_command_subscriber_ = this->create_subscription<svdsim_msgs::msg::StampedSteering>(
      "steering_command", 10,
      std::bind(&MonacoF1SimBridgeNode::steering_command_callback, this, std::placeholders::_1));
  velocity_command_subscriber_ = this->create_subscription<svdsim_msgs::msg::StampedVelocity>(
      "velocity_command", 10,
      std::bind(&MonacoF1SimBridgeNode::velocity_command_callback, this, std::placeholders::_1));
  initialpose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, std::bind(&MonacoF1SimBridgeNode::reset_pose_callback, this, std::placeholders::_1));

  // Initialize ZMQ client
  initialize_zmq_client();

  // Start ZMQ spinner thread
  running_ = true;
  zmq_thread_ = std::thread(&MonacoF1SimBridgeNode::zmq_spinner_thread, this);

  RCLCPP_INFO(this->get_logger(), "ZMQ spinner thread started");
}

MonacoF1SimBridgeNode::~MonacoF1SimBridgeNode() {
  RCLCPP_INFO(this->get_logger(), "Shutting down Monaco F1TENTH Simulation Bridge Node");

  // Signal thread to stop
  running_ = false;

  // Wait for thread to finish
  if (zmq_thread_.joinable()) {
    zmq_thread_.join();
  }

  RCLCPP_INFO(this->get_logger(), "Bridge node shutdown complete");
}

void MonacoF1SimBridgeNode::initialize_zmq_client() {
  try {
    zmq_client_ = std::make_unique<svdsim_proto::VehicleControlClient>(hostname_, port_);

    // Set callback for incoming protobuf messages
    zmq_client_->set_callback(
        std::bind(&MonacoF1SimBridgeNode::on_proto_message_received, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ZMQ client initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZMQ client: %s", e.what());
    throw;
  }
}

void MonacoF1SimBridgeNode::zmq_spinner_thread() {
  RCLCPP_DEBUG(this->get_logger(), "ZMQ spinner thread running");

  constexpr int kPollTimeoutMs = 20;

  while (running_) {
    // Poll for incoming messages with 20ms timeout
    try {
      (void)zmq_client_->poll_and_receive(kPollTimeoutMs);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in ZMQ polling: %s", e.what());
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "ZMQ spinner thread exiting");
}

builtin_interfaces::msg::Time MonacoF1SimBridgeNode::proto_timestamp_to_ros(
    const svdsim_server::Timestamp& timestamp) const {
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(timestamp.seconds());
  ros_time.nanosec = static_cast<uint32_t>(timestamp.fraction() * 1e9);
  return ros_time;
}

void MonacoF1SimBridgeNode::publish_clock_message(const svdsim_server::Timestamp& timestamp) {
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = proto_timestamp_to_ros(timestamp);

  clock_publisher_->publish(clock_msg);

  RCLCPP_DEBUG(this->get_logger(), "Published clock: %d.%09u", clock_msg.clock.sec, clock_msg.clock.nanosec);
}

void MonacoF1SimBridgeNode::on_proto_message_received(const svdsim_server::SimDataMessage& message) {
  // Process broadcast timestamp messages
  if (message.has_server_outbound() && message.has_timestamp()) {
    const auto& outbound = message.server_outbound();
    const auto& timestamp = message.timestamp();

    if (outbound.has_broadcast_timestamp()) {
      publish_clock_message(timestamp);
    }
    if (outbound.has_steering_feedback()) {
      publish_steering_feedback(outbound.steering_feedback(), timestamp);
    }
    if (outbound.has_wheel_encoder_data()) {
      publish_encoder_data(outbound.wheel_encoder_data(), timestamp);
    }
    if (outbound.has_lidar_scan_data()) {
      publish_lidar_scan(outbound.lidar_scan_data(), timestamp);
    }
    if (outbound.has_imu_sample_data()) {
      publish_imu_sample(outbound.imu_sample_data(), timestamp);
      publish_magnetometer(outbound.imu_sample_data(), timestamp);
    }
    if (outbound.has_ground_truth_data()) {
      publish_ground_truth(outbound.ground_truth_data(), timestamp);
    }
    if (outbound.has_sparring_pose_marker()) {
      publish_sparring_pose_marker(outbound.sparring_pose_marker(), timestamp);
    }
    if (outbound.has_sparring_lookahead_marker()) {
      publish_sparring_lookahead_marker(outbound.sparring_lookahead_marker(), timestamp);
    }
    if (outbound.has_collision_count_data()) {
      publish_collision_data(outbound.collision_count_data());
    }
  }
}

void MonacoF1SimBridgeNode::publish_steering_feedback(
    const svdsim_server::SteeringFeedback& steering_feedback,
    const svdsim_server::Timestamp& timestamp) {
  svdsim_msgs::msg::StampedSteering msg;

  // Fill header with proto timestamp
  msg.header.stamp = proto_timestamp_to_ros(timestamp);

  // Fill steering value
  msg.steering = steering_feedback.value();

  steering_feedback_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_encoder_data(
    const svdsim_server::WheelEncoderData& encoder_data,
    const svdsim_server::Timestamp& timestamp) {
  svdsim_msgs::msg::StampedEncoderData msg;

  // Fill header with proto timestamp
  msg.header.stamp = proto_timestamp_to_ros(timestamp);

  // Fill encoder values
  msg.rear_left_encoder = encoder_data.rear_left_pulse_count();
  msg.rear_right_encoder = encoder_data.rear_right_pulse_count();

  encoder_data_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_lidar_scan(
    const svdsim_server::LidarScanData& lidar_data,
    const svdsim_server::Timestamp& timestamp) {
  sensor_msgs::msg::LaserScan msg;

  // Fill header with proto timestamp
  msg.header.stamp = proto_timestamp_to_ros(timestamp);
  msg.header.frame_id = lidar_frame_id_;

  // Fill scan parameters
  msg.range_min = static_cast<float>(lidar_data.min_range());
  msg.range_max = static_cast<float>(lidar_data.max_range());

  const int num_points = lidar_data.points_per_scan();
  const double fov = lidar_data.horizontal_fov();

  msg.angle_min = static_cast<float>(-fov / 2.0);
  msg.angle_max = static_cast<float>(fov / 2.0);
  msg.angle_increment = static_cast<float>(fov / (num_points - 1));
  msg.time_increment = static_cast<float>(1.0 / (lidar_data.scan_rate() * num_points));
  msg.scan_time = static_cast<float>(1.0 / lidar_data.scan_rate());

  // Fill range data
  msg.ranges.reserve(lidar_data.ranges_size());
  for (int i = 0; i < lidar_data.ranges_size(); ++i) {
    msg.ranges.push_back(static_cast<float>(lidar_data.ranges(i)));
  }

  scan_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_imu_sample(
    const svdsim_server::IMUSampleData& imu_data,
    const svdsim_server::Timestamp& timestamp) {
  sensor_msgs::msg::Imu msg;

  // Fill header with proto timestamp
  msg.header.stamp = proto_timestamp_to_ros(timestamp);
  msg.header.frame_id = imu_frame_id_;

  // Fill linear acceleration (m/s^2)
  msg.linear_acceleration.x = imu_data.accel_x();
  msg.linear_acceleration.y = imu_data.accel_y();
  msg.linear_acceleration.z = imu_data.accel_z();

  // Fill angular velocity (rad/s)
  msg.angular_velocity.x = imu_data.gyro_x();
  msg.angular_velocity.y = imu_data.gyro_y();
  msg.angular_velocity.z = imu_data.gyro_z();

  // Orientation is not provided by the IMU sensor model (only accel + gyro)
  // Set orientation covariance to -1 to indicate unknown orientation
  msg.orientation_covariance[0] = -1.0;

  // Set covariance matrices to unknown (0 indicates unknown)
  // In a real implementation, these would be populated from IMU noise parameters
  for (int i = 0; i < 9; ++i) {
    msg.angular_velocity_covariance[i] = 0.0;
    msg.linear_acceleration_covariance[i] = 0.0;
  }

  imu_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_magnetometer(
    const svdsim_server::IMUSampleData& imu_data,
    const svdsim_server::Timestamp& timestamp) {
  sensor_msgs::msg::MagneticField msg;

  // Fill header with proto timestamp
  msg.header.stamp = proto_timestamp_to_ros(timestamp);
  msg.header.frame_id = imu_frame_id_;

  // Fill magnetic field vector (Tesla)
  msg.magnetic_field.x = imu_data.mag_x();
  msg.magnetic_field.y = imu_data.mag_y();
  msg.magnetic_field.z = imu_data.mag_z();

  // Set covariance matrix to unknown (0 indicates unknown)
  // In a real implementation, this would be populated from IMU noise parameters
  for (int i = 0; i < 9; ++i) {
    msg.magnetic_field_covariance[i] = 0.0;
  }

  magnetometer_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_ground_truth(
    const svdsim_server::GroundTruthData& ground_truth_data,
    const svdsim_server::Timestamp& timestamp) {
  nav_msgs::msg::Odometry msg;

  // Fill header
  msg.header.stamp = proto_timestamp_to_ros(timestamp);
  msg.header.frame_id = map_frame_id_;
  msg.child_frame_id = base_link_frame_id_;

  // Fill pose (position)
  msg.pose.pose.position.x = ground_truth_data.x();
  msg.pose.pose.position.y = ground_truth_data.y();
  msg.pose.pose.position.z = 0.0;

  // Convert yaw to quaternion: q = (0, 0, sin(yaw/2), cos(yaw/2))
  const double half_yaw = ground_truth_data.yaw() / 2.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(half_yaw);
  msg.pose.pose.orientation.w = std::cos(half_yaw);

  // Fill twist (velocity in vehicle/child frame)
  msg.twist.twist.linear.x = ground_truth_data.vx();
  msg.twist.twist.linear.y = ground_truth_data.vy();
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = ground_truth_data.yaw_rate();

  ground_truth_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::publish_sparring_markers_timer_callback() {
  std::lock_guard<std::mutex> lock(marker_cache_mutex_);

  // Early return if no markers to publish
  if (cached_pose_markers_.empty() && cached_lookahead_markers_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // Add all cached pose markers
  for (const auto& [entity_id, marker] : cached_pose_markers_) {
    marker_array.markers.push_back(marker);
  }

  // Add all cached lookahead markers
  for (const auto& [entity_id, marker] : cached_lookahead_markers_) {
    marker_array.markers.push_back(marker);
  }

  // Publish all markers together
  sparring_markers_publisher_->publish(marker_array);
}

void MonacoF1SimBridgeNode::publish_sparring_pose_marker(
    const svdsim_server::SparringPoseMarker& pose_marker_data,
    const svdsim_server::Timestamp& timestamp) {
  visualization_msgs::msg::Marker marker;

  // Fill header
  marker.header.stamp = proto_timestamp_to_ros(timestamp);
  marker.header.frame_id = map_frame_id_;

  // Set marker properties
  marker.ns = "sparring_pose";
  marker.id = static_cast<int>(pose_marker_data.entity_id());
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set pose
  marker.pose.position.x = pose_marker_data.x();
  marker.pose.position.y = pose_marker_data.y();
  marker.pose.position.z = 0.0;

  // Convert yaw to quaternion
  const double half_yaw = pose_marker_data.yaw() / 2.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(half_yaw);
  marker.pose.orientation.w = std::cos(half_yaw);

  // Set scale (arrow size)
  marker.scale.x = 0.60;  // Arrow length
  marker.scale.y = 0.20;  // Arrow width
  marker.scale.z = 0.20;  // Arrow height

  // Set color (red)
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // Cache marker for timer-based publishing
  std::lock_guard<std::mutex> lock(marker_cache_mutex_);
  cached_pose_markers_[pose_marker_data.entity_id()] = marker;
}

void MonacoF1SimBridgeNode::publish_sparring_lookahead_marker(
    const svdsim_server::SparringLookaheadMarker& lookahead_marker_data,
    const svdsim_server::Timestamp& timestamp) {
  visualization_msgs::msg::Marker marker;

  // Fill header
  marker.header.stamp = proto_timestamp_to_ros(timestamp);
  marker.header.frame_id = map_frame_id_;

  // Set marker properties
  marker.ns = "sparring_lookahead";
  marker.id = static_cast<int>(lookahead_marker_data.entity_id());
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set pose
  marker.pose.position.x = lookahead_marker_data.target_x();
  marker.pose.position.y = lookahead_marker_data.target_y();
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set scale (sphere size)
  marker.scale.x = 0.30;
  marker.scale.y = 0.30;
  marker.scale.z = 0.30;

  // Set color (green)
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // Cache marker for timer-based publishing
  std::lock_guard<std::mutex> lock(marker_cache_mutex_);
  cached_lookahead_markers_[lookahead_marker_data.entity_id()] = marker;
}

void MonacoF1SimBridgeNode::publish_collision_data(const svdsim_server::CollisionCountData& collision_count_data) {
  svdsim_msgs::msg::CollisionData msg;
  msg.obstacle_collision_count = collision_count_data.obstacle_collision_count();
  msg.vehicle_collision_count = collision_count_data.vehicle_collision_count();
  msg.in_collision_with_obstacle = collision_count_data.in_collision_with_obstacle();
  msg.in_collision_with_vehicle = collision_count_data.in_collision_with_vehicle();
  collision_data_publisher_->publish(msg);
}

void MonacoF1SimBridgeNode::steering_command_callback(const svdsim_msgs::msg::StampedSteering::SharedPtr msg) {
  // Create proto message
  svdsim_server::SimDataMessage proto_msg;

  auto* steering_cmd = proto_msg.mutable_server_inbound()->mutable_steering_command();
  steering_cmd->set_value(msg->steering);

  // Send through ZMQ client
  if (!zmq_client_->publish(proto_msg)) {
    RCLCPP_WARN(this->get_logger(), "Failed to publish steering command: %f rad", msg->steering);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Published steering command: %f rad", msg->steering);
  }
}

void MonacoF1SimBridgeNode::velocity_command_callback(const svdsim_msgs::msg::StampedVelocity::SharedPtr msg) {
  // Create proto message
  svdsim_server::SimDataMessage proto_msg;

  auto* velocity_cmd = proto_msg.mutable_server_inbound()->mutable_velocity_command();
  velocity_cmd->set_value(msg->velocity);

  // Send through ZMQ client
  if (!zmq_client_->publish(proto_msg)) {
    RCLCPP_WARN(this->get_logger(), "Failed to publish velocity command: %f m/s", msg->velocity);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Published velocity command: %f m/s", msg->velocity);
  }
}

void MonacoF1SimBridgeNode::reset_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // Validate frame_id
  if (msg->header.frame_id != map_frame_id_) {
    RCLCPP_ERROR(
        this->get_logger(), "Reset pose frame_id mismatch: expected '%s' but got '%s'. Ignoring message.",
        map_frame_id_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  // Extract pose data
  const auto& position = msg->pose.pose.position;
  const auto& orientation = msg->pose.pose.orientation;

  // Convert quaternion to yaw angle
  // Using formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

  // Create proto message
  svdsim_server::SimDataMessage proto_msg;

  auto* reset_state = proto_msg.mutable_server_inbound()->mutable_reset_vehicle_state();
  reset_state->set_x(position.x);
  reset_state->set_y(position.y);
  reset_state->set_yaw(yaw);

  // Send through ZMQ client
  if (!zmq_client_->publish(proto_msg)) {
    RCLCPP_WARN(
        this->get_logger(), "Failed to publish reset vehicle state: x=%f, y=%f, yaw=%f", position.x, position.y, yaw);
  } else {
    RCLCPP_INFO(
        this->get_logger(), "Published reset vehicle state: x=%f, y=%f, yaw=%f rad", position.x, position.y, yaw);
  }
}

}  // namespace svdsim_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(svdsim_bridge::MonacoF1SimBridgeNode)
