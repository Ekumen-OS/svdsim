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

// Basic test to verify that the test infrastructure is working
TEST(SvdsimServerBasicTest, TruthIsTrue) {
  // Arrange & Act & Assert
  ASSERT_TRUE(true);
}

// Test to verify that basic arithmetic works
TEST(SvdsimServerBasicTest, BasicArithmetic) {
  // Arrange
  const int a = 2;
  const int b = 3;

  // Act
  const int result = a + b;

  // Assert
  EXPECT_EQ(result, 5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
