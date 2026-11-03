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

#include <entt/entt.hpp>

// Test that EnTT registry can be created
TEST(EnttLibraryTest, RegistryCreation) {
  // Arrange & Act
  EXPECT_NO_THROW({ entt::registry registry; });
}

// Test that entities can be created
TEST(EnttLibraryTest, EntityCreation) {
  // Arrange
  entt::registry registry;

  // Act
  const auto entity = registry.create();

  // Assert
  EXPECT_TRUE(registry.valid(entity));
}

// Test that components can be added to entities
TEST(EnttLibraryTest, ComponentManagement) {
  // Arrange
  struct TestComponent {
    int value;
  };

  entt::registry registry;
  const auto entity = registry.create();

  // Act
  registry.emplace<TestComponent>(entity, 42);

  // Assert
  EXPECT_TRUE(registry.all_of<TestComponent>(entity));
  const auto& component = registry.get<TestComponent>(entity);
  EXPECT_EQ(component.value, 42);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
