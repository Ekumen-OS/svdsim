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

// Non-ROS dependencies
#include <gtest/gtest.h>

#include <memory>

// Project headers
#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/models/lidar_sensor_model.hpp"

namespace svdsim_server {

class LidarSensorModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a mock collider manager
    collider_manager_ = std::make_shared<ColliderManager>();

    // Typical lidar parameters
    lidar_pose_ = Sophus::SE2d(Sophus::SO2d(0.1), Eigen::Vector2d(0.5, 0.0));
    min_range_ = 0.1;
    max_range_ = 10.0;
    horizontal_fov_ = 3.14159;
    points_per_scan_ = 360;
    range_noise_stddev_ = 0.01;
    physics_rate_ = 1000.0;
    scan_rate_ = 10.0;
    enable_lidar_skew_ = true;

    lidar_sensor_ = std::make_unique<LidarSensorModel>(
        collider_manager_, lidar_pose_, min_range_, max_range_, horizontal_fov_, points_per_scan_, range_noise_stddev_,
        physics_rate_, scan_rate_, enable_lidar_skew_);
  }

  std::shared_ptr<ColliderManager> collider_manager_;
  Sophus::SE2d lidar_pose_;
  double min_range_;
  double max_range_;
  double horizontal_fov_;
  int points_per_scan_;
  double range_noise_stddev_;
  double physics_rate_;
  double scan_rate_;
  bool enable_lidar_skew_;
  std::unique_ptr<LidarSensorModel> lidar_sensor_;
};

// Test getter for lidar_pose
TEST_F(LidarSensorModelTest, GetLidarPose) {
  const auto& pose = lidar_sensor_->lidar_pose();
  EXPECT_DOUBLE_EQ(pose.translation().x(), lidar_pose_.translation().x());
  EXPECT_DOUBLE_EQ(pose.translation().y(), lidar_pose_.translation().y());
  EXPECT_DOUBLE_EQ(pose.so2().log(), lidar_pose_.so2().log());
}

// Test getter for min_range
TEST_F(LidarSensorModelTest, GetMinRange) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->min_range(), min_range_);
}

// Test getter for max_range
TEST_F(LidarSensorModelTest, GetMaxRange) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->max_range(), max_range_);
}

// Test getter for horizontal_fov
TEST_F(LidarSensorModelTest, GetHorizontalFov) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->horizontal_fov(), horizontal_fov_);
}

// Test getter for points_per_scan
TEST_F(LidarSensorModelTest, GetPointsPerScan) {
  EXPECT_EQ(lidar_sensor_->points_per_scan(), points_per_scan_);
}

// Test getter for range_noise_stddev
TEST_F(LidarSensorModelTest, GetRangeNoiseStddev) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->range_noise_stddev(), range_noise_stddev_);
}

// Test getter for physics_rate
TEST_F(LidarSensorModelTest, GetPhysicsRate) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->physics_rate(), physics_rate_);
}

// Test getter for scan_rate
TEST_F(LidarSensorModelTest, GetScanRate) {
  EXPECT_DOUBLE_EQ(lidar_sensor_->scan_rate(), scan_rate_);
}

// Test getter for enable_lidar_skew
TEST_F(LidarSensorModelTest, GetEnableLidarSkew) {
  EXPECT_EQ(lidar_sensor_->enable_lidar_skew(), enable_lidar_skew_);
}

// Test with lidar skew disabled
TEST_F(LidarSensorModelTest, GetEnableLidarSkewFalse) {
  auto lidar_no_skew = std::make_unique<LidarSensorModel>(
      collider_manager_, lidar_pose_, min_range_, max_range_, horizontal_fov_, points_per_scan_, range_noise_stddev_,
      physics_rate_, scan_rate_, false);
  EXPECT_FALSE(lidar_no_skew->enable_lidar_skew());
}

}  // namespace svdsim_server
