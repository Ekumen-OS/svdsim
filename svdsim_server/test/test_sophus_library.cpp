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
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>

// Test that SO2 (2D rotation) can be created
TEST(SophusLibraryTest, SO2Creation) {
  // Arrange & Act
  const Sophus::SO2d rotation(0.0);

  // Assert
  EXPECT_NEAR(rotation.log(), 0.0, 1e-10);
}

// Test that SE2 (2D rigid transformation) can be created
TEST(SophusLibraryTest, SE2Creation) {
  // Arrange & Act
  const Sophus::SE2d transform(Sophus::SO2d(0.0), Eigen::Vector2d(1.0, 2.0));

  // Assert
  const auto translation = transform.translation();
  EXPECT_NEAR(translation.x(), 1.0, 1e-10);
  EXPECT_NEAR(translation.y(), 2.0, 1e-10);
}

// Test that SO3 (3D rotation) can be created
TEST(SophusLibraryTest, SO3Creation) {
  // Arrange & Act
  const Sophus::SO3d rotation = Sophus::SO3d::rotX(0.0);

  // Assert
  const auto identity = Sophus::SO3d();
  EXPECT_TRUE(rotation.matrix().isApprox(identity.matrix(), 1e-10));
}

// Test that SE3 (3D rigid transformation) can be created
TEST(SophusLibraryTest, SE3Creation) {
  // Arrange & Act
  const Sophus::SE3d transform(Sophus::SO3d(), Eigen::Vector3d(1.0, 2.0, 3.0));

  // Assert
  const auto translation = transform.translation();
  EXPECT_NEAR(translation.x(), 1.0, 1e-10);
  EXPECT_NEAR(translation.y(), 2.0, 1e-10);
  EXPECT_NEAR(translation.z(), 3.0, 1e-10);
}

// Test SE2 transformation composition
TEST(SophusLibraryTest, SE2Composition) {
  // Arrange
  const Sophus::SE2d transform1(Sophus::SO2d(0.0), Eigen::Vector2d(1.0, 0.0));
  const Sophus::SE2d transform2(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 1.0));

  // Act
  const Sophus::SE2d composed = transform1 * transform2;

  // Assert
  const auto translation = composed.translation();
  EXPECT_NEAR(translation.x(), 1.0, 1e-10);
  EXPECT_NEAR(translation.y(), 1.0, 1e-10);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
